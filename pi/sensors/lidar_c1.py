# -*- coding: utf-8 -*-
"""
lidar_c1.py — درايفر RPLIDAR C1 (خيط خلفي + آخر لفّة كاملة + إعادة اتصال)
==========================================================================
البروتوكول: أوامر RPLIDAR القياسية (A5 xx). SCAN = `A5 20` ثم واصف ردّ
`A5 5A 05 00 00 40 81`، ثم عُقد من 5 بايت:

    b0 = الجودة<<2 | !S<<1 | S       (S = بداية لفّة جديدة)
    b1 = (الزاوية_q6 & 0x7F)<<1 | 1   (بت التحقق C = 1 دائماً)
    b2 = الزاوية_q6 >> 7              ⇒ الزاوية = q6 / 64 °  (مع عقارب الساعة)
    b3,b4 = المسافة_q2                ⇒ المسافة = q2 / 4 مم  (0 = لا قراءة)

- المحرك **لا يقف إلا بأمر STOP** (`A5 25`) — يُرسل عند كل خروج وقبل كل SCAN.
- المنفذ من VID/PID (CP210x 10C4:EA60) أو `/dev/serial/by-id` — لا افتراض ttyUSB0.
- إعادة الاتصال تلقائية بعد فصل USB (تهدئة `LIDAR_REOPEN_S`)، والخطأ **حالة
  معلنة** (`error`) لا استثناء يعبر إلى حلقة القيادة (§6.3).
- التحويل لإطار الروبوت (x أمام · y يسار · الزاوية يسار موجب) في
  `to_robot_frame` — دالة خالصة تُختبر بلا عتاد.
"""
from __future__ import annotations

import glob
import math
import os
import threading
import time

try:
    import serial                                   # pyserial
    from serial.tools import list_ports
    _SERIAL_OK = True
except ImportError:                                 # ويندوز/محاكاة بلا pyserial
    serial = None
    list_ports = None
    _SERIAL_OK = False

from pi.config import (
    LIDAR_USB_VID, LIDAR_USB_PID, LIDAR_BAUD, LIDAR_PORT, LIDAR_REOPEN_S,
    LIDAR_ANGLE_SIGN, LIDAR_YAW_OFFSET_DEG, LIDAR_X_M, LIDAR_Y_M,
    LIDAR_MIN_RANGE_M, LIDAR_MAX_RANGE_M, LIDAR_STALE_S,
)

CMD_STOP = b"\xA5\x25"
CMD_SCAN = b"\xA5\x20"
SCAN_DESCRIPTOR = b"\xA5\x5A\x05\x00\x00\x40\x81"
MIN_POINTS_PER_REV = 50           # لفّة أقل من هذا = بداية متقطعة تُهمل


def find_port() -> str | None:
    """المنفذ: متغيّر البيئة ⇒ VID/PID ⇒ /dev/serial/by-id (CP210x/Silicon Labs)."""
    if LIDAR_PORT:
        return LIDAR_PORT
    if list_ports is not None:
        for p in list_ports.comports():
            if p.vid == LIDAR_USB_VID and p.pid == LIDAR_USB_PID:
                return p.device
    for pat in ("/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*Silicon_Labs*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return os.path.realpath(hits[0])
    return None


class NodeParser:
    """
    محلّل عُقد SCAN مع إعادة تزامن: عقدة تفشل بتي التحقق ⇒ يُسقط بايتاً
    واحداً ويعيد المحاولة (بايت ضائع على USB لا يفسد بقية اللفّة).
    """

    def __init__(self):
        self._buf = bytearray()
        self.resyncs = 0

    def feed(self, data: bytes):
        """يُعيد قائمة (بداية_لفّة، الجودة، الزاوية°، المسافة م)."""
        self._buf.extend(data)
        out = []
        b = self._buf
        i = 0
        n = len(b)
        while n - i >= 5:
            b0, b1 = b[i], b[i + 1]
            s = b0 & 1
            ns = (b0 >> 1) & 1
            if s == ns or not (b1 & 1):
                i += 1
                self.resyncs += 1
                continue
            q6 = (b[i + 2] << 7) | (b1 >> 1)
            q2 = b[i + 3] | (b[i + 4] << 8)
            out.append((bool(s), b0 >> 2, q6 / 64.0, q2 / 4000.0))
            i += 5
        del b[:i]
        return out


def to_robot_frame(raw_points, sign: int = LIDAR_ANGLE_SIGN,
                   yaw_offset_deg: float = LIDAR_YAW_OFFSET_DEG,
                   x0: float = LIDAR_X_M, y0: float = LIDAR_Y_M,
                   min_r: float = LIDAR_MIN_RANGE_M,
                   max_r: float = LIDAR_MAX_RANGE_M):
    """
    (زاوية الليدار°، المسافة م) ⇒ قائمة dict بإطار الروبوت:
    `a` زاوية من **مركز الروبوت** (−180..180، يسار موجب) · `r` بُعدها ·
    `x` أمام · `y` يسار · `la` زاوية الليدار في إطار الروبوت (للقناع).
    ما دون `min_r` أو فوق `max_r` أو صفر يُهمل.
    """
    out = []
    for ang, d in raw_points:
        if not d or d < min_r or d > max_r:
            continue
        la = (sign * ang + yaw_offset_deg + 180.0) % 360.0 - 180.0
        t = math.radians(la)
        x = x0 + d * math.cos(t)
        y = y0 + d * math.sin(t)
        out.append({"a": math.degrees(math.atan2(y, x)), "r": math.hypot(x, y),
                    "x": x, "y": y, "la": la, "d": d})
    return out


class LidarC1:
    """خيط خلفي يحتفظ بآخر لفّة كاملة ووقتها. `start()` ثم `latest()`."""

    def __init__(self, port: str | None = None, baud: int = LIDAR_BAUD):
        self.port_override = port
        self.baud = baud
        self.port = None
        self.error = None                  # آخر خطأ (نصّ) أو None
        self.connected = False
        self.revs = 0                      # لفّات كاملة منذ البدء
        self.reconnects = 0
        self.parser = NodeParser()
        self._scan = None                  # (ts, [(زاوية، مسافة)...])
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._ser = None

    # ── الواجهة ──────────────────────────────────────────────────
    def start(self) -> "LidarC1":
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="lidar_c1",
                                            daemon=True)
            self._thread.start()
        return self

    def latest(self):
        """(الوقت، النقاط الخام) لآخر لفّة كاملة، أو None."""
        with self._lock:
            return self._scan

    def age_s(self) -> float:
        s = self.latest()
        return float("inf") if s is None else time.time() - s[0]

    def fresh(self, max_age_s: float = LIDAR_STALE_S) -> bool:
        return self.age_s() <= max_age_s

    def wait_scan(self, after_ts: float = 0.0, timeout: float = 2.0):
        """ينتظر لفّة أحدث من `after_ts` (أداة للسكربتات)."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            s = self.latest()
            if s is not None and s[0] > after_ts:
                return s
            time.sleep(0.01)
        return None

    def stop(self) -> None:
        """يوقف الخيط **ويرسل STOP** (المحرك لا يقف بغيره)."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._close(send_stop=True)

    def state(self) -> dict:
        return {"port": self.port, "connected": self.connected,
                "error": self.error, "revs": self.revs,
                "age_s": round(self.age_s(), 3) if self.latest() else None,
                "reconnects": self.reconnects, "resyncs": self.parser.resyncs}

    # ── الداخل ───────────────────────────────────────────────────
    def _close(self, send_stop: bool) -> None:
        ser, self._ser = self._ser, None
        self.connected = False
        if ser is None:
            return
        try:
            if send_stop:
                ser.write(CMD_STOP)
                ser.flush()
                time.sleep(0.05)
        except Exception:                  # noqa: BLE001
            pass
        try:
            ser.close()
        except Exception:                  # noqa: BLE001
            pass

    def _open(self) -> None:
        if not _SERIAL_OK:
            raise RuntimeError("pyserial غير مثبّتة")
        port = self.port_override or find_port()
        if not port:
            raise RuntimeError("لم يُعثر على الليدار (CP210x 10C4:EA60) — "
                               "هل هو موصول؟ أو حدّد RMS_LIDAR_PORT")
        self.port = port
        ser = serial.Serial(port, self.baud, timeout=0.2)
        self._ser = ser
        ser.write(CMD_STOP)
        ser.flush()
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.write(CMD_SCAN)
        ser.flush()
        # الواصف: يُبحث عنه ضمن أول البايتات (قد تسبقه بقايا)
        buf = bytearray()
        t_end = time.time() + 2.0
        while time.time() < t_end:
            buf.extend(ser.read(64))
            k = buf.find(SCAN_DESCRIPTOR)
            if k >= 0:
                self.parser = NodeParser()
                rest = bytes(buf[k + len(SCAN_DESCRIPTOR):])
                self.connected, self.error = True, None
                return rest
        raise RuntimeError("لا واصف SCAN من الليدار خلال 2ث")

    def _run(self) -> None:
        first = True
        while not self._stop.is_set():
            try:
                rest = self._open()
                if not first:
                    self.reconnects += 1
                first = False
                cur = []
                pending = rest
                while not self._stop.is_set():
                    data = pending or self._ser.read(1024)
                    pending = None
                    if not data:
                        continue
                    for start, _q, ang, dist in self.parser.feed(data):
                        if start and cur:
                            if len(cur) >= MIN_POINTS_PER_REV:
                                with self._lock:
                                    self._scan = (time.time(), cur)
                                self.revs += 1
                            cur = []
                        cur.append((ang, dist))
            except Exception as e:         # noqa: BLE001 — USB فُصل/لا جهاز
                self.error = str(e)
                self._close(send_stop=False)
                self._stop.wait(LIDAR_REOPEN_S)
        self._close(send_stop=True)


class SimLidar:
    """
    ليدار وهمي لمشهد ثابت: `scene_fn()` تُعيد نقاطاً **بإطار الليدار الخام**
    (زاوية مع عقارب الساعة، مسافة). للتشغيل بلا عتاد (`--sim`) فقط.
    """

    def __init__(self, scene_fn):
        self.scene_fn = scene_fn
        self.error = None
        self.connected = True
        self.revs = 0
        self.port = "sim"

    def start(self):
        return self

    def latest(self):
        self.revs += 1
        return (time.time(), self.scene_fn())

    def age_s(self) -> float:
        return 0.0

    def fresh(self, max_age_s: float = LIDAR_STALE_S) -> bool:
        return True

    def wait_scan(self, after_ts: float = 0.0, timeout: float = 2.0):
        time.sleep(0.1)
        return self.latest()

    def stop(self) -> None:
        pass

    def state(self) -> dict:
        return {"port": "sim", "connected": True, "error": None,
                "revs": self.revs, "age_s": 0.0, "reconnects": 0, "resyncs": 0}


def sim_room_scene(half_len_m: float = 1.5, half_wid_m: float = 1.0):
    """غرفة مستطيلة حول الليدار — نقطة لكل 0.72° (≈500/لفّة) بإطار الشركة."""
    def scene():
        pts = []
        for i in range(500):
            cw = i * 0.72
            t = math.radians(-cw)          # إطار الشركة مع عقارب الساعة
            c, s = math.cos(t), math.sin(t)
            ds = []
            if abs(c) > 1e-6:
                ds.append(half_len_m / abs(c))
            if abs(s) > 1e-6:
                ds.append(half_wid_m / abs(s))
            pts.append((cw, min(ds)))
        return pts
    return scene
