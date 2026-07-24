# -*- coding: utf-8 -*-
"""
bridge.py — جسر الروفر: واجهة موحّدة بوضعَي `sim` / `real`
==========================================================
- `RoverBridge`      : محاكاة قيادة خارجية بإحداثيات lat/lng (تخدم واجهة M1).
- `WaveRoverBridge`  : **جسر Wave Rover الحقيقي** ببروتوكول Waveshare المكتشف
                       تجريبياً، مع بديل محاكاة كامل ليعمل على ويندوز بلا عتاد.

بروتوكول Waveshare (JSON سطري + \\n على /dev/serial0 @115200):
    إرسال:  {"T":1,"L":<-1..1>,"R":<-1..1>}   حركة (تُضرب في MOTOR_INVERT)
            {"T":126}                          طلب IMU كامل
            {"T":130}                          طلب حالة مختصرة
    استقبال: {"T":1002, r,p,y, ax..az, gx..gz, mx..mz, temp}   ردّ 126
            {"T":1001, L,R, r,p,y, temp, v}                    ردّ 130

⚠ ملاحظات مثبتة على العتاد:
  - الفيرموير **يردّد الأمر المُرسل صدىً** قبل الرد الفعلي → تجاهل أي سطر
    يحمل نفس T المُرسل، وانتظر 1002/1001.
  - `y` (yaw) **للعرض فقط لا للملاحة** — الملاحة من `gz` (انظر CLAUDE.md).
  - `T=131`, `T=4`, `T=71` بلا رد — لا تعتمد عليها.
  - **الأمر المرتد لا يعني التنفيذ** — تحقق من الحركة عبر gz/التسارع.
"""
from __future__ import annotations

import json
import logging
import math
import random
import time

from pi.config import (
    DRIVE_SPEED_MPS, TURN_RATE_DPS, ROVER_SAFETY_TIMEOUT_S,
    SIM_HOME_LAT, SIM_HOME_LNG,
    ROVER_PORT, ROVER_BAUD, MOTOR_INVERT, GYRO_SCALE, TURN_POWER,
    DRIVE_POWER_DEFAULT, ROVER_TURN_TIMEOUT_S, GYRO_BIAS_CALIB_S,
    MAX_MOTOR_POWER, GYRO_BIAS_MAX_STD, MAX_TURN_SEGMENT_DEG, TURN_SEGMENT_PAUSE_S,
)

logger = logging.getLogger(__name__)


# ═══ إحصاء متين لانحياز الجايرو (وسيط + استبعاد شواذ) ════════════
def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def _std(vals, center):
    if len(vals) < 2:
        return 0.0
    return math.sqrt(sum((v - center) ** 2 for v in vals) / len(vals))


def robust_bias(samples, max_std: float = GYRO_BIAS_MAX_STD) -> dict:
    """
    انحياز الجايرو بـ**الوسيط لا المتوسط**، مع استبعاد الشواذ خارج 3σ ثم
    إعادة الحساب. السبب المقاس: عينتان شاذتان من 320 رفعتا (max−min) إلى
    31.69، بينما الوسيط بعد التنقية أعطى انحيازاً 0.011 بانحراف 0.31.
    تُرفض المعايرة فقط إذا تجاوز الانحراف **بعد التنقية** max_std.
    """
    if not samples:
        return {"ok": False, "bias": 0.0, "std": 0.0, "n": 0,
                "rejected": 0, "reason": "لا عينات"}
    med0 = _median(samples)
    sd0 = _std(samples, med0)
    kept = [v for v in samples if abs(v - med0) <= 3.0 * sd0] if sd0 > 0 else list(samples)
    if not kept:
        kept = list(samples)
    bias = _median(kept)
    sd = _std(kept, bias)
    ok = sd <= max_std
    return {"ok": ok, "bias": bias, "std": sd, "n": len(kept),
            "rejected": len(samples) - len(kept), "raw_std": sd0,
            "reason": None if ok else f"تشتت {sd:.2f} > {max_std}"}
from pi.rover import battery as batt

# استيراد العتاد محميّ — غيابه (ويندوز) لا يكسر شيئاً
try:
    import serial
    _SERIAL_OK = True
except Exception:                     # noqa: BLE001
    _SERIAL_OK = False

_M_PER_DEG = 111320.0


# ═══════════════════════════════════════════════════════════════
#  1) جسر المحاكاة الخارجية (lat/lng) — يخدم واجهة M1
# ═══════════════════════════════════════════════════════════════
class RoverBridge:
    def __init__(self, mode: str = "sim"):
        self.mode = mode
        self.lat = SIM_HOME_LAT
        self.lng = SIM_HOME_LNG
        self.heading = 0.0
        self.speed = 0.0
        self._dir = "S"
        self._power = 0
        self._last_cmd_ts = 0.0
        self._from_gps = False

    def set_home_from_gps(self, lat: float, lng: float) -> None:
        if not self._from_gps and lat and lng:
            self.lat, self.lng = lat, lng
            self._from_gps = True

    def command(self, direction: str, power: int = 70) -> None:
        d = (direction or "S").upper()
        if d not in ("F", "B", "L", "R", "S"):
            return
        self._dir = d
        try:
            self._power = max(0, min(100, int(power)))
        except (TypeError, ValueError):
            self._power = 70
        self._last_cmd_ts = time.time()

    def update(self, dt: float) -> None:
        if self.mode != "sim" or dt <= 0 or dt > 2.0:
            return
        if time.time() - self._last_cmd_ts > ROVER_SAFETY_TIMEOUT_S:
            self._dir = "S"
        p = self._power / 70.0
        if self._dir == "F":
            self.speed = DRIVE_SPEED_MPS * p
            self._advance(self.speed * dt)
        elif self._dir == "B":
            self.speed = -DRIVE_SPEED_MPS * p
            self._advance(self.speed * dt)
        elif self._dir == "L":
            self.heading = (self.heading - TURN_RATE_DPS * p * dt) % 360.0
            self.speed = 0.0
        elif self._dir == "R":
            self.heading = (self.heading + TURN_RATE_DPS * p * dt) % 360.0
            self.speed = 0.0
        else:
            self.speed = 0.0

    def _advance(self, dist_m: float) -> None:
        hd = math.radians(self.heading)
        self.lat += (dist_m * math.cos(hd)) / _M_PER_DEG
        self.lng += (dist_m * math.sin(hd)) / (_M_PER_DEG * math.cos(math.radians(self.lat)))

    def state(self) -> dict:
        return {"mode": self.mode, "lat": round(self.lat, 6), "lng": round(self.lng, 6),
                "heading": round(self.heading, 1), "speed": round(self.speed, 2),
                "dir": self._dir, "power": self._power}


# ═══════════════════════════════════════════════════════════════
#  2) جسر Wave Rover الحقيقي (+ محاكاة كاملة)
# ═══════════════════════════════════════════════════════════════
class WaveRoverBridge:
    """
    وضعان: `real` (سيريال حقيقي) و`sim` (محاكاة كاملة تعمل على ويندوز).
    التبديل بعلم واحد؛ عند طلب `real` وغياب pyserial/المنفذ يسقط تلقائياً
    إلى `sim` مع تسجيل السبب (لا فشل صامت).
    """

    def __init__(self, mode: str = "sim", port: str = ROVER_PORT, baud: int = ROVER_BAUD):
        self.requested_mode = mode
        self.mode = "sim"
        self.error = None
        self.port, self.baud = port, baud
        self._ser = None
        self.gyro_bias = 0.0
        self.bias_calibrated = False
        self.heading = 0.0              # درجة — من تكامل الجايرو (لا البوصلة)
        self._last_cmd_ts = 0.0
        self._moving = False
        self._cmd_lr = (0.0, 0.0)
        self.rth_requested = False
        self.battery_alarm = False
        self.last_status = {}
        self.clamp_count = 0            # مرات قصّ القوة (تُبثّ في سجل الواجهة)
        self.last_clamp_msg = None
        self.events = []                # أحداث الجسر (قصّ/تجزئة/انحياز) → الواجهة
        self.bias_info = {}
        # محاكاة
        self._sim_v = 12.40
        self._sim_turn_rate = 0.0
        self._sim_bias = -0.28
        self._last_sim_ts = time.time()

        if mode == "real":
            if not _SERIAL_OK:
                self.error = "pyserial غير مثبّت — وضع المحاكاة"
            else:
                try:
                    self._ser = serial.Serial(port, baud, timeout=0.3)
                    self.mode = "real"
                except Exception as e:      # noqa: BLE001
                    self.error = f"تعذّر فتح {port}: {e} — وضع المحاكاة"

    def _event(self, kind: str, msg: str) -> None:
        """يسجّل حدثاً يُصرَّف إلى سجل أحداث الواجهة (البند 3)."""
        self.events.append({"kind": kind, "msg": msg})
        if len(self.events) > 100:
            self.events = self.events[-100:]

    def drain_events(self) -> list:
        evs, self.events = self.events, []
        return evs

    # ── الإرسال/الاستقبال ────────────────────────────────────────
    def _send(self, obj: dict) -> None:
        if self.mode != "real":
            return
        self._ser.write((json.dumps(obj) + "\n").encode("ascii"))

    def _read_until(self, expect_t: int, request_t: int, timeout: float = 1.0):
        """
        يقرأ أسطراً حتى يجد T == expect_t. **يتجاهل صدى الأمر المُرسل**
        (أي سطر T == request_t) وأي سطر غير صالح.
        """
        if self.mode != "real":
            return None
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = self._ser.readline().decode("ascii", errors="replace").strip()
            except Exception:               # noqa: BLE001
                break
            if not line:
                continue
            try:
                d = json.loads(line)
            except ValueError:
                continue                    # سطر غير JSON — تجاهل
            # ⚠ الفيرموير يرسل أحياناً JSON صالحاً لكنه **ليس كائناً** (رقم
            # مجرّد مثل 0) → json.loads يعطي int، ومناداة .get عليه تنفجر
            # بـ'int' object has no attribute 'get' وتُسقط المهمة كلها.
            if not isinstance(d, dict):
                continue
            t = d.get("T")
            if t == request_t:
                continue                    # صدى الأمر — تجاهل
            if t == expect_t:
                return d
        return None

    # ── الحركة (⚠ MOTOR_INVERT) ─────────────────────────────────
    def motors(self, l: float, r: float) -> dict:
        """
        **المسار الوحيد لإرسال أي أمر حركة** (تقدّم/رجوع/لفّ/أوامر الواجهة).
        يطبّق MOTOR_INVERT ثم **حاجزاً صارماً** عند ±MAX_MOTOR_POWER.

        ⚠ الحاجز إلزامي: فيرموير Wave Rover يلتفّ عددياً فوق 0.5 (يطرح 0.5)،
        فإرسال 0.8 يُنتج قوة فعّالة 0.3 — زحف صامت يفسد حساب المسافة في
        deadreckoning بلا أي إنذار. القصّ يحفظ الإشارة ويُسجَّل تحذيراً.
        """
        li = max(-1.0, min(1.0, float(l)))
        ri = max(-1.0, min(1.0, float(r)))
        lw, rw = li * MOTOR_INVERT, ri * MOTOR_INVERT      # قيم السلك

        # الحاجز الصارم — **بعد** MOTOR_INVERT، مع الحفاظ على الإشارة
        lc = max(-MAX_MOTOR_POWER, min(MAX_MOTOR_POWER, lw))
        rc = max(-MAX_MOTOR_POWER, min(MAX_MOTOR_POWER, rw))
        if lc != lw or rc != rw:
            self.clamp_count += 1
            self.last_clamp_msg = (
                f"⚠ قُصّت القوة إلى ±{MAX_MOTOR_POWER} "
                f"(طُلب L={lw:.2f} R={rw:.2f} → L={lc:.2f} R={rc:.2f}) — "
                f"التفاف فيرموير Wave Rover فوق 0.5")
            logger.warning(self.last_clamp_msg)
            self._event("power_clamp", self.last_clamp_msg)

        self._cmd_lr = (lc, rc)
        self._moving = not (lc == 0.0 and rc == 0.0)
        self._last_cmd_ts = time.time()
        self._send({"T": 1, "L": round(lc, 3), "R": round(rc, 3)})
        if self.mode == "sim":
            # معدل الدوران من القوة **الفعّالة بعد القصّ**، مُعاداً لإطار النية
            eff_l, eff_r = lc * MOTOR_INVERT, rc * MOTOR_INVERT
            self._sim_turn_rate = (eff_l - eff_r) * TURN_RATE_DPS
        return {"L": lc, "R": rc, "clamped": (lc != lw or rc != rw)}

    # اسم قديم — كل المسارات تمرّ عبر motors()
    def _drive(self, l: float, r: float) -> None:
        self.motors(l, r)

    def forward(self, power: float = DRIVE_POWER_DEFAULT) -> None:
        self._drive(power, power)

    def backward(self, power: float = DRIVE_POWER_DEFAULT) -> None:
        self._drive(-power, -power)

    def turn(self, direction: str, power: float = TURN_POWER) -> None:
        """دوران بالمكان: 'R' يمين، 'L' يسار."""
        if direction.upper() == "R":
            self._drive(power, -power)
        else:
            self._drive(-power, power)

    def stop(self) -> None:
        self._drive(0.0, 0.0)
        self._moving = False
        if self.mode == "sim":
            self._sim_turn_rate = 0.0

    # ── القراءات ────────────────────────────────────────────────
    def read_imu(self) -> dict:
        """T=126 → T:1002. في المحاكاة يولّد gz متسقاً مع أمر الدوران."""
        if self.mode == "real":
            self._send({"T": 126})
            d = self._read_until(1002, 126)
            return d or {}
        now = time.time()
        self._last_sim_ts = now
        gz = self._sim_turn_rate + self._sim_bias + random.uniform(-0.4, 0.4)
        return {"T": 1002, "r": 0.0, "p": 0.0, "y": self.heading,
                "ax": 0.0, "ay": 0.0, "az": 9.8,
                "gx": 0.0, "gy": 0.0, "gz": round(gz, 3),
                "mx": 0.0, "my": 0.0, "mz": 0.0, "temp": 31.0}

    def read_status(self) -> dict:
        """T=130 → T:1001 (يتضمّن جهد البطارية v)."""
        if self.mode == "real":
            self._send({"T": 130})
            d = self._read_until(1001, 130) or {}
        else:
            if self._moving:                # استهلاك وهمي بسيط
                self._sim_v = max(9.5, self._sim_v - 0.0004)
            d = {"T": 1001, "L": self._cmd_lr[0], "R": self._cmd_lr[1],
                 "r": 0.0, "p": 0.0, "y": self.heading, "temp": 31.0,
                 "v": round(self._sim_v, 2)}
        if d:
            self.last_status = d
        return d

    def voltage(self):
        return self.last_status.get("v")

    # ── أدوات المحاكاة (لاختبار العتبات في الواجهة) ─────────────
    def sim_set_voltage(self, v: float) -> None:
        self._sim_v = float(v)
        self.battery_alarm = False
        self.rth_requested = False

    def sim_set_moving(self, moving: bool) -> None:
        """يجعل الاستهلاك الوهمي يعكس كون المهمة جارية."""
        self._moving = bool(moving)

    # ── معايرة انحياز الجايرو (تلقائياً عند بدء كل مهمة) ─────────
    def calibrate_gyro_bias(self, seconds: float = GYRO_BIAS_CALIB_S) -> float:
        """
        يقيس انحياز gz والروبوت **ساكن**، بالوسيط مع استبعاد الشواذ.
        ⚠ لا يُثبَّت في الكود — يتغيّر بين التجارب، فيُقاس عند بدء كل مهمة.
        """
        self.stop()
        samples, deadline = [], time.time() + seconds
        while time.time() < deadline:
            d = self.read_imu()
            if "gz" in d:
                samples.append(float(d["gz"]))
            time.sleep(0.05)
        info = robust_bias(samples)
        self.bias_info = info
        if info["ok"]:
            self.gyro_bias = info["bias"]
            self.bias_calibrated = True
            if info["rejected"]:
                self._event("gyro_bias",
                            f"انحياز {info['bias']:.3f} (σ={info['std']:.2f})، "
                            f"استُبعدت {info['rejected']} عينة شاذّة")
        else:
            self.bias_calibrated = False
            self._event("gyro_bias_rejected",
                        f"⚠ رُفضت معايرة الانحياز — {info['reason']}")
        return self.gyro_bias

    # ── الدوران بزاوية عبر تكامل الجايرو (مهلة أمان) ────────────
    def _turn_segment(self, degrees: float, timeout: float, power: float) -> dict:
        """مرحلة لفّ واحدة بتكامل الجايرو، بمهلة أمان خاصة بها."""
        turned = 0.0
        timed_out = False
        direction = "R" if degrees >= 0 else "L"
        try:
            self.turn(direction, power)
            last = start = time.time()
            while abs(turned) < abs(degrees):
                if time.time() - start > timeout:
                    timed_out = True
                    break
                d = self.read_imu()
                now = time.time()
                dt = now - last
                last = now
                gz = float(d.get("gz", 0.0)) if isinstance(d, dict) else 0.0
                turned += (gz - self.gyro_bias) * dt * GYRO_SCALE
                # ⚠ **جدّد أمر اللفّ** كل دورة: بلا تجديد يمرّ 1.5ث فيعتبره
                # حارس الـheartbeat انقطاعاً ويوقف المحركات في منتصف اللفّة
                # (كانت اللفّة تتوقف عند ~24° لهذا السبب).
                self.turn(direction, power)
                time.sleep(0.02)
        finally:
            self.stop()                      # ⚠ إيقاف مضمون لكل مرحلة
        return {"turned": turned, "timed_out": timed_out}

    def turn_by_angle(self, degrees: float,
                      timeout: float = ROVER_TURN_TIMEOUT_S,
                      power: float = TURN_POWER) -> dict:
        """
        heading += (gz - bias) * dt * GYRO_SCALE.
        ⚠ **اللفّات الطويلة تُجزَّأ** إلى مراحل ≤ MAX_TURN_SEGMENT_DEG مع توقف
        بينها — لفّة 360° متواصلة تفشل على العتاد (تُنهك البطارية فتتوقف عند
        ~195°). المهلة (8ث) **لكل مرحلة** لا للفّة كاملة.
        """
        if not self.bias_calibrated:
            self.calibrate_gyro_bias()
        sign = 1.0 if degrees >= 0 else -1.0
        remaining = abs(float(degrees))
        segments = []
        while remaining > 0:
            seg = min(MAX_TURN_SEGMENT_DEG, remaining)
            segments.append(sign * seg)
            remaining -= seg
        if len(segments) > 1:
            self._event("turn_segmented",
                        f"لفّ {degrees:.0f}° على {len(segments)} مراحل")

        turned_total = 0.0
        timed_out = False
        try:
            for i, seg in enumerate(segments):
                r = self._turn_segment(seg, timeout, power)
                turned_total += r["turned"]
                if r["timed_out"]:
                    timed_out = True
                    break
                if i < len(segments) - 1:
                    time.sleep(TURN_SEGMENT_PAUSE_S)   # استرداد البطارية والمحركات
        finally:
            self.stop()                      # ⚠ إيقاف مضمون
        self.heading = (self.heading + turned_total) % 360.0
        return {"requested_deg": degrees, "turned_deg": round(turned_total, 1),
                "timed_out": timed_out, "heading": round(self.heading, 1),
                "segments": len(segments)}

    # ── السلامة: heartbeat + البطارية ───────────────────────────
    def check_heartbeat(self) -> bool:
        """بلا أوامر > 1.5ث والمحركات تعمل → إيقاف فوري. True إن أوقف."""
        if self._moving and (time.time() - self._last_cmd_ts) > ROVER_SAFETY_TIMEOUT_S:
            self.stop()
            return True
        return False

    def check_battery(self) -> dict:
        """يقرأ الجهد ويطبّق عتبات أ-3 الإلزامية."""
        self.read_status()
        info = batt.classify(self.voltage())
        if info["action"] == batt.ACTION_STOP:
            self.stop()
            self.battery_alarm = True
        elif info["action"] == batt.ACTION_RTH:
            self.rth_requested = True
        return info

    def state(self) -> dict:
        return {
            "mode": self.mode, "requested_mode": self.requested_mode,
            "error": self.error, "heading": round(self.heading, 1),
            "gyro_bias": round(self.gyro_bias, 4),
            "bias_calibrated": self.bias_calibrated,
            "moving": self._moving, "cmd": {"L": self._cmd_lr[0], "R": self._cmd_lr[1]},
            "battery": batt.classify(self.voltage()),
            "rth_requested": self.rth_requested, "battery_alarm": self.battery_alarm,
            "max_motor_power": MAX_MOTOR_POWER,
            "clamp_count": self.clamp_count, "last_clamp_msg": self.last_clamp_msg,
        }

    def close(self) -> None:
        try:
            self.stop()
        except Exception:                    # noqa: BLE001
            pass
        try:
            if self._ser:
                self._ser.close()
        except Exception:                    # noqa: BLE001
            pass
