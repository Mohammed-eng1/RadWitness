#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_setup.py — ماذا على الروبوت فعلاً؟  (لا يحرّك المحركات إطلاقاً)
================================================================================
أعلام الحضور في config (IR_PRESENT, ULTRASONIC_PRESENT, INA219, مصدر الاتجاه…)
**تُعلَن يدوياً ولا تُستنتج** (CLAUDE.md §6.1). هذا السكربت لا يغيّر أي علم —
بل يجمع الحقيقة من العتاد في دقيقة واحدة، ليُضبط كل علم على **قياس** لا تخمين.

ما يفحصه بالترتيب:
  1. config.txt          : أسطر UART4/UART0/I2C الفعلية
  2. المنافذ             : /dev/ttyAMA4 (اللوحة) · /dev/serial0 (GPS) · مجموعة dialout
  3. لوحة UGV01          : ردود T:130 (والجهد v) · T:126 (gz، القفزات، وحدة التسارع)
  4. نواقل I2C           : INA219 (0x40–0x45) · MPU (0x68/0x69 + WHO_AM_I) · BNO055
  5. الكاميرا            : /dev/video* وإطار واحد فعلي (إن وُجدت opencv)
  6. GPS                 : أسطر NMEA على serial0 وحالة الـfix
  7. الجيجر              : نبضات على BCM17 (RISING، بلا شدّ — §3) خلال --geiger-seconds

وبـ **--gyro-sign** مرحلة تفاعلية: إشارة gz لوحة UGV01 **باليد بلا محركات**
(§2 — قياسها بالمحركات حاصل ضرب خطأين). تدوّر الروبوت بيدك ربع لفّة مع عقارب
الساعة ثم تعيده، فيُطبع `UGV01_GYRO_Z_SIGN` المقاس — وهو ما يفتح مصدر الاتجاه
`ugv01_gyro` (يعلن ok=False حتى يُكتب).

⚠ **أوقف السيرفر قبل التشغيل**: يحجز منفذ اللوحة وGPS ومنفذ الجيجر، فتظهر هنا
«مشغول» لا «غائب».

الحكم: GO إن ردّت اللوحة على المنفذ المثبت؛ الباقي **معلومات** (⚠ لا ❌) لأن
غياب قطعة قد يكون قراراً (IR/الألترا سونيك أُزيلا عمداً). الملخّص يُحفظ في
logs/setup_<التاريخ>.json — أرسله كما هو.

التشغيل:
    python3 -m pi.tests.ugv01.check_setup
    python3 -m pi.tests.ugv01.check_setup --gyro-sign
    python3 -m pi.tests.ugv01.check_setup --geiger-seconds 30 --i2c-scan
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import statistics
import sys
import threading
import time

try:
    import serial  # pyserial — الاعتماد الوحيد الإلزامي
except ImportError:
    sys.exit("خطأ: pyserial غير مثبّتة.  pip3 install pyserial")

try:
    from smbus2 import SMBus   # اختياري: بدونه يُتخطّى فحص I2C بسبب معلن
except ImportError:
    SMBus = None


# ═══ ثوابت مستقلة (لا استيراد من pi.* — انظر README) ═══════════════════
BAUD = 115200
BOARD_PORT = "/dev/ttyAMA4"          # UART4: TX GPIO8 الدبوس 24 · RX GPIO9 الدبوس 21
GPS_PORT = "/dev/serial0"
GPS_BAUD = 9600
GEIGER_GPIO = 17                     # BCM17 (دبوس 11)
CAMERA_INDEX = 0
CONFIG_TXT = ("/boot/firmware/config.txt", "/boot/config.txt")
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

INIT_CMDS = (
    {"T": 900, "main": 3, "module": 0},   # إلزامي — ثوابت UGV01
    {"T": 143, "cmd": 0},                 # إطفاء الصدى
    {"T": 605, "cmd": 0},                 # إطفاء التشخيص
    {"T": 131, "cmd": 0},                 # إطفاء البثّ المستمر
)
STOP_CMD = {"T": 1, "L": 0, "R": 0}

# عناوين I2C المعروفة في هذا المشروع
INA219_ADDRS = (0x40, 0x41, 0x42, 0x43, 0x44, 0x45)
MPU_ADDRS = (0x68, 0x69)
MPU_WHOAMI = {0x68: "MPU-6050", 0x70: "MPU-6500", 0x71: "MPU-9250", 0x73: "MPU-9255"}
BNO055_ADDRS = (0x28, 0x29)

SPIKE_JUMP_DPS = 8.0                 # قفزة gz منفردة (المقاس ±15.9) — نصفها
GRAVITY_UNITS = (((490.0, 1470.0), 100.0, "×100 (سم/ث²)"),
                 ((4.9, 14.7), 1.0, "م/ث²"),
                 ((0.5, 1.5), 1.0 / 9.80665, "g"))
SIGN_MIN_DEG = 45.0
I2C_PASSES = 5                       # كل عنوان يُسأل 5 مرات: ناقل سليم يجيب 5/5 أو 0/5                  # دون هذا في المرحلة اليدوية: حركة غير كافية


# ═══ السجل والنتيجة ═══════════════════════════════════════════════════
ICON = {"ok": "✅", "warn": "⚠ ", "fail": "❌", "skip": "—"}


class Report:
    def __init__(self):
        self.rows = []               # (القسم، البند، الحالة، التفصيل)
        self.data = {}

    def add(self, section, item, status, detail):
        self.rows.append((section, item, status, detail))
        print("   %s %-26s %s" % (ICON[status], item, detail))


# ═══ وصلة اللوحة (قراءة بطابع زمني عند أول بايت — كما في test_sync) ═══
class Link:
    def __init__(self, ser):
        self.ser = ser
        self._buf = b""

    def send(self, obj):
        self.ser.write((json.dumps(obj, separators=(",", ":")) + "\n").encode("ascii"))
        self.ser.flush()

    def lines(self, seconds):
        t_end = time.monotonic() + seconds
        while True:
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                self._buf += chunk
                while b"\n" in self._buf:
                    raw, self._buf = self._buf.split(b"\n", 1)
                    text = raw.decode("ascii", errors="replace").strip()
                    if not text:
                        continue
                    obj = None
                    if text.startswith("{"):
                        try:
                            parsed = json.loads(text)
                            obj = parsed if isinstance(parsed, dict) else None
                        except ValueError:
                            pass
                    yield text, obj, time.monotonic()
            if time.monotonic() >= t_end:
                return

    def ask(self, cmd, want_t, seconds=0.4):
        self._buf = b""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self.send(cmd)
        for _, obj, t in self.lines(seconds):
            if obj is not None and obj.get("T") == want_t:
                return obj, t
        return None, None


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def median3(values):
    """(السلسلة بعد وسيط ثلاثي، عدد القفزات فوق SPIKE_JUMP_DPS)."""
    if len(values) < 3:
        return list(values), 0
    out, rej = [values[0]], 0
    for k in range(1, len(values) - 1):
        m = sorted(values[k - 1:k + 2])[1]
        if abs(values[k] - m) > SPIKE_JUMP_DPS:
            rej += 1
        out.append(m)
    out.append(values[-1])
    return out, rej


# ═══ 1) config.txt ═══════════════════════════════════════════════════
def check_config_txt(rep):
    print("\n[1] config.txt")
    path = next((p for p in CONFIG_TXT if os.path.exists(p)), None)
    if path is None:
        rep.add("config", "config.txt", "warn", "غير موجود في %s" % " / ".join(CONFIG_TXT))
        return
    try:
        lines = [ln.split("#", 1)[0].strip() for ln in open(path, encoding="utf-8",
                                                              errors="replace")]
    except Exception as e:
        rep.add("config", "config.txt", "warn", "تعذّرت قراءته: %s" % e)
        return
    lines = [ln for ln in lines if ln]
    rep.data["config_txt"] = {"path": path, "lines": lines}
    want = (("dtoverlay=uart4", "UART4 للوحة (الدبوسان 24 و21)", True),
            ("enable_uart=1", "UART", False),
            ("dtoverlay=disable-bt", "إطلاق UART0 من البلوتوث", False),
            ("dtparam=uart0=on", "UART0 للـGPS", False),
            ("dtparam=i2c_arm=on", "I2C-1 (INA219؛ الدبوسان 3 و5)", False))
    for key, why, critical in want:
        found = any(ln.replace(" ", "") == key for ln in lines)
        rep.add("config", key, "ok" if found else ("fail" if critical else "warn"),
                ("موجود — " if found else "غائب — ") + why)
    i2c_extra = [ln for ln in lines if ln.startswith("dtoverlay=i2c")]
    rep.add("config", "نواقل I2C إضافية", "ok" if i2c_extra else "warn",
            ", ".join(i2c_extra) if i2c_extra else
            "لا dtoverlay=i2c* ⇒ لا /dev/i2c-4 (موضع MPU السابق)")


# ═══ 2) المنافذ ══════════════════════════════════════════════════════
def check_ports(rep):
    print("\n[2] المنافذ")
    rep.add("ports", BOARD_PORT, "ok" if os.path.exists(BOARD_PORT) else "fail",
            "موجود" if os.path.exists(BOARD_PORT) else
            "غائب ⇒ dtoverlay=uart4 ثم إعادة إقلاع")
    if os.path.exists(GPS_PORT):
        rep.add("ports", GPS_PORT, "ok", "→ %s" % os.path.realpath(GPS_PORT))
    else:
        rep.add("ports", GPS_PORT, "warn", "غائب (GPS لن يُقرأ)")
    try:
        import grp
        in_dialout = grp.getgrnam("dialout").gr_gid in os.getgroups()
    except Exception:
        in_dialout = None
    rep.add("ports", "مجموعة dialout", "ok" if in_dialout else "warn",
            {True: "المستخدم فيها", False: "ليس فيها ⇒ sudo usermod -aG dialout $USER",
             None: "تعذّر التحقق"}[in_dialout])


# ═══ 3) لوحة UGV01 ═══════════════════════════════════════════════════
def open_board(boot_wait):
    try:
        ser = serial.Serial(BOARD_PORT, BAUD, timeout=0.05, exclusive=True)
    except TypeError:
        ser = serial.Serial(BOARD_PORT, BAUD, timeout=0.05)
    link = Link(ser)
    for _ in link.lines(boot_wait):
        pass
    for cmd in INIT_CMDS:
        link.send(cmd)
        time.sleep(0.2)
    link.send(STOP_CMD)           # لا حركة: إيقاف صريح احتياطاً لا غير
    time.sleep(0.1)
    ser.reset_input_buffer()
    return link


def check_board(rep, link):
    print("\n[3] لوحة UGV01 (%s)" % BOARD_PORT)
    got, volts = 0, []
    for _ in range(5):
        obj, _ = link.ask({"T": 130}, 1001)
        if obj:
            got += 1
            v = num(obj.get("v"))
            if v is not None:
                volts.append(v)
    rep.data["board_status_replies"] = got
    rep.add("board", "T:130 → T:1001", "ok" if got == 5 else ("warn" if got else "fail"),
            "%d/5 ردود" % got + (" · v=%.2f فولت" % volts[-1] if volts else ""))
    if volts:
        rep.data["board_v"] = volts[-1]
        rep.add("board", "حقل v", "warn",
                "%.2f فولت — ⚠ لا يرى الحمل (مقاس مرتين) فلا يصلح حماية" % volts[-1])

    gz, grav = [], []
    for _ in range(40):
        obj, _ = link.ask({"T": 126}, 1002)
        if not obj:
            continue
        g = num(obj.get("gz"))
        if g is not None:
            gz.append(g)
        a = [num(obj.get(k)) for k in ("ax", "ay", "az")]
        if None not in a:
            grav.append(math.sqrt(sum(x * x for x in a)))
    if not gz:
        rep.add("board", "T:126 → gz", "fail", "لا قراءة gz")
        return
    filt, spikes = median3(gz)
    rep.data["gz"] = {"n": len(gz), "median": statistics.median(gz),
                      "sd_raw": statistics.pstdev(gz), "sd_median3": statistics.pstdev(filt),
                      "spikes": spikes}
    rep.add("board", "T:126 → gz", "ok" if statistics.pstdev(gz) > 0 else "fail",
            "%d عيّنة · وسيط %+.3f · σ خام %.3f / بعد الوسيط %.3f · قفزات %d"
            % (len(gz), statistics.median(gz), statistics.pstdev(gz),
               statistics.pstdev(filt), spikes))
    if grav:
        raw = statistics.mean(grav)
        unit = next(((d, lab) for (lo, hi), d, lab in GRAVITY_UNITS if lo <= raw <= hi), None)
        rep.data["gravity_raw"] = raw
        rep.add("board", "التسارع", "ok" if unit else "warn",
                ("%.2f خام ⇒ %.3f م/ث² (%s)" % (raw, raw / unit[0], unit[1])) if unit
                else "%.2f خام — وحدة غير معروفة" % raw)


# ═══ 4) I2C ══════════════════════════════════════════════════════════
def check_i2c(rep, full_scan):
    print("\n[4] نواقل I2C")
    buses = sorted(int(p.rsplit("-", 1)[1]) for p in glob.glob("/dev/i2c-*")
                   if p.rsplit("-", 1)[1].isdigit())
    rep.data["i2c_buses"] = buses
    if not buses:
        rep.add("i2c", "/dev/i2c-*", "warn", "لا نواقل ⇒ dtparam=i2c_arm=on")
        return
    rep.add("i2c", "/dev/i2c-*", "ok", "النواقل: %s" % ", ".join(map(str, buses)))
    if SMBus is None:
        rep.add("i2c", "smbus2", "skip", "غير مثبّتة ⇒ pip install smbus2 (تُخطّى الفحوص)")
        return
    found = {}
    for b in buses:
        try:
            bus = SMBus(b)
        except Exception as e:
            rep.add("i2c", "i2c-%d" % b, "warn", "تعذّر فتحه: %s" % e)
            continue
        try:
            addrs = range(0x03, 0x78) if full_scan else \
                INA219_ADDRS + MPU_ADDRS + BNO055_ADDRS
            # 🔴 تكرار لا قراءة واحدة: مقاس 2026-09-26 — تشغيلان متتاليان على
            #    i2c-1 أعطيا عناوين مختلفة (0x40/0x44/0x69/0x28 تظهر وتختفي،
            #    وINA219 يقرأ 11.91V ثم Errno 5). عنوان يجيب أحياناً = **ناقل
            #    متقطّع** (§0: قِس الخط)، لا جهاز. والقراءة الواحدة لا تميّزه.
            score = {}
            for _ in range(I2C_PASSES):
                for a in addrs:
                    try:
                        bus.read_byte(a)
                        score[a] = score.get(a, 0) + 1
                    except Exception:
                        pass
            hits = sorted(a for a, n in score.items() if n == I2C_PASSES)
            flaky = sorted(a for a, n in score.items() if 0 < n < I2C_PASSES)
            found[b] = hits
            rep.data.setdefault("i2c_score", {})[str(b)] = {"0x%02x" % a: n
                                                            for a, n in score.items()}
            if flaky:
                rep.data.setdefault("i2c_flaky", {})[str(b)] = ["0x%02x" % a for a in flaky]
                rep.add("i2c", "i2c-%d متقطّع" % b, "fail",
                        "عناوين تجيب أحياناً: %s ⇒ الناقل غير سليم — قِس الخط"
                        " (SDA/SCL/GND، مقاومات الشدّ، طول السلك) قبل أي برمجية"
                        % ", ".join("0x%02x=%d/%d" % (a, score[a], I2C_PASSES)
                                    for a in flaky))
            for a in hits:
                if a in INA219_ADDRS:
                    try:
                        d = bus.read_i2c_block_data(a, 0x02, 2)
                        raw = (d[0] << 8) | d[1]
                        v = (raw >> 3) * 0.004
                        ovf = bool(raw & 1)
                        rep.data.setdefault("ina219", []).append(
                            {"bus": b, "addr": a, "v": v, "ovf": ovf})
                        rep.add("i2c", "INA219? i2c-%d @0x%02x" % (b, a),
                                "ok" if (5.0 <= v <= 14.0 and not ovf) else "warn",
                                "جهد الناقل %.2f فولت%s" % (v, " · OVF" if ovf else "")
                                + ("" if (b, a) == (1, 0x42) else
                                   " — ⚠ config يتوقّع i2c-1 @0x42"))
                    except Exception as e:
                        rep.add("i2c", "i2c-%d @0x%02x" % (b, a), "warn",
                                "ردّ ولم تُقرأ الجهد: %s" % e)
                elif a in MPU_ADDRS:
                    try:
                        who = bus.read_byte_data(a, 0x75)
                    except Exception:
                        who = None
                    rep.data.setdefault("mpu", []).append({"bus": b, "addr": a, "whoami": who})
                    rep.add("i2c", "MPU i2c-%d @0x%02x" % (b, a),
                            "ok" if who in MPU_WHOAMI else "warn",
                            "WHO_AM_I=%s ⇒ %s" % ("0x%02x" % who if who is not None else "?",
                                                  MPU_WHOAMI.get(who, "غير معروف")))
                elif a in BNO055_ADDRS:
                    rep.add("i2c", "i2c-%d @0x%02x" % (b, a), "warn",
                            "عنوان BNO055 — الوحدة القديمة **تالفة** (§0)")
            if full_scan:
                rep.add("i2c", "مسح i2c-%d" % b, "ok",
                        " ".join("0x%02x" % a for a in hits) or "لا أجهزة")
        finally:
            bus.close()
    rep.data["i2c_found"] = {str(k): v for k, v in found.items()}
    if not any(a in INA219_ADDRS for v in found.values() for a in v):
        rep.add("i2c", "INA219", "warn", "غير موجود ⇒ لا حماية جهد فعلية (الحاجز الزمني وحده)")
    if not any(a in MPU_ADDRS for v in found.values() for a in v):
        rep.add("i2c", "MPU", "warn", "غير موجود ⇒ مصدر الاتجاه ugv01_gyro (جايرو اللوحة)")


# ═══ 5) الكاميرا ═════════════════════════════════════════════════════
def check_camera(rep):
    print("\n[5] الكاميرا")
    devs = sorted(glob.glob("/dev/video*"))
    rep.data["video_devices"] = devs
    if not devs:
        rep.add("camera", "/dev/video*", "warn", "لا أجهزة فيديو")
        return
    rep.add("camera", "/dev/video*", "ok", ", ".join(devs))
    try:
        import cv2
    except Exception:
        rep.add("camera", "opencv", "skip", "غير مثبّتة — لا اختبار إطار")
        return
    box = {}

    def grab():
        try:
            cap = cv2.VideoCapture(CAMERA_INDEX)
            ok, frame = cap.read()
            box["ok"] = bool(ok) and frame is not None
            box["shape"] = None if frame is None else frame.shape
            cap.release()
        except Exception as e:
            box["error"] = str(e)

    th = threading.Thread(target=grab, daemon=True)
    th.start()
    th.join(5.0)                        # جهاز معطوب قد يعلّق read() — لا ننتظره للأبد
    if th.is_alive():
        rep.add("camera", "إطار /dev/video%d" % CAMERA_INDEX, "warn", "علقت اللقطة > 5ث")
    elif box.get("ok"):
        h, w = box["shape"][:2]
        rep.data["camera_frame"] = [w, h]
        rep.add("camera", "إطار /dev/video%d" % CAMERA_INDEX, "ok", "%d×%d" % (w, h))
    else:
        rep.add("camera", "إطار /dev/video%d" % CAMERA_INDEX, "warn",
                "لا إطار (%s)" % box.get("error", "مشغولة؟ السيرفر يعمل؟"))


# ═══ 6) GPS ══════════════════════════════════════════════════════════
def check_gps(rep, seconds):
    print("\n[6] GPS (%s @%d)" % (GPS_PORT, GPS_BAUD))
    if not os.path.exists(GPS_PORT):
        rep.add("gps", "NMEA", "skip", "لا منفذ")
        return
    try:
        ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.2)
    except Exception as e:
        rep.add("gps", "NMEA", "warn", "تعذّر الفتح: %s" % e)
        return
    n, gga, fix, sats = 0, 0, 0, None
    t_end = time.monotonic() + seconds
    try:
        while time.monotonic() < t_end:
            ln = ser.readline().decode("ascii", errors="replace").strip()
            if not ln.startswith("$"):
                continue
            n += 1
            f = ln.split(",")
            if f[0][3:6] == "GGA" and len(f) > 7:
                gga += 1
                if f[6].isdigit() and int(f[6]) > 0:
                    fix += 1
                sats = f[7] or sats
    finally:
        ser.close()
    rep.data["gps"] = {"nmea": n, "gga": gga, "fix": fix, "sats": sats}
    if n == 0:
        rep.add("gps", "NMEA", "warn", "صمت %.0fث ⇒ سلك GPS-TX (الدبوس 10) أو السرعة" % seconds)
    else:
        rep.add("gps", "NMEA", "ok" if fix else "warn",
                "%d سطراً · GGA %d · fix %d · أقمار %s%s"
                % (n, gga, fix, sats, "" if fix else " (بلا fix — طبيعي داخل مبنى)"))


# ═══ 7) الجيجر ═══════════════════════════════════════════════════════
def check_geiger(rep, seconds):
    print("\n[7] الجيجر (BCM%d، %.0fث)" % (GEIGER_GPIO, seconds))
    try:
        import lgpio
    except Exception:
        rep.add("geiger", "نبضات", "skip", "lgpio غير مثبّتة")
        return
    h = cb = None
    try:
        for chip in (0, 4):                 # باي 4 = 0، باي 5 = 4
            try:
                h = lgpio.gpiochip_open(chip)
                break
            except Exception:
                h = None
        if h is None:
            rep.add("geiger", "نبضات", "warn", "تعذّر فتح gpiochip")
            return
        # §3: عائم بلا شدّ — أي pull يقتل الإشارة
        lgpio.gpio_claim_alert(h, GEIGER_GPIO, lgpio.RISING_EDGE, lgpio.SET_PULL_NONE)
        cb = lgpio.callback(h, GEIGER_GPIO, lgpio.RISING_EDGE)
        time.sleep(seconds)
        n = int(cb.tally())
        cpm = n * 60.0 / seconds
        rep.data["geiger"] = {"pulses": n, "seconds": seconds, "cpm": cpm}
        rep.add("geiger", "نبضات", "ok" if n else "warn",
                "%d نبضة ⇒ ~%.0f CPM" % (n, cpm) if n else
                "صفر في %.0fث ⇒ غير موصول؟ (الخلفية ~15–30 CPM) — أعد بـ--geiger-seconds 60"
                % seconds)
    except Exception as e:
        rep.add("geiger", "نبضات", "warn", "%s (السيرفر يحجز المنفذ؟)" % e)
    finally:
        try:
            if cb is not None:
                cb.cancel()
            if h is not None:
                lgpio.gpio_free(h, GEIGER_GPIO)
                lgpio.gpiochip_close(h)
        except Exception:
            pass


# ═══ المرحلة اليدوية: إشارة gz ═══════════════════════════════════════
def integrate_hand(link, seconds, bias):
    """يكامل (وسيط ثلاثي(gz) − الانحياز) خلال مدة، ويطبع كل ثانية."""
    vals, times = [], []
    t0 = time.monotonic()
    last_print = 0.0
    while time.monotonic() - t0 < seconds:
        obj, t = link.ask({"T": 126}, 1002, 0.3)
        g = num(obj.get("gz")) if obj else None
        if g is None:
            continue
        vals.append(g)
        times.append(t)
        el = t - t0
        if el - last_print >= 1.0:
            last_print = el
            filt, _ = median3(vals)
            ang = sum((filt[k] - bias) * (times[k] - times[k - 1])
                      for k in range(1, len(filt)))
            print("      %3.0fث   تراكمي %+7.1f°" % (el, ang))
    filt, spikes = median3(vals)
    ang = sum((filt[k] - bias) * (times[k] - times[k - 1]) for k in range(1, len(filt)))
    return ang, len(vals), spikes


def gyro_sign(rep, link, seconds):
    print("\n[8] إشارة gz باليد — **بلا محركات** (CLAUDE.md §2)")
    print("    الاصطلاح: الموجب = دوران مع عقارب الساعة منظوراً **من الأعلى** (يمين).")
    print("    ضع الروبوت على الأرض ساكناً. المحركات لا تُشغَّل في أي لحظة.")
    try:
        input("    اضغط Enter لقياس الانحياز (3ث، لا تلمسه)… ")
    except EOFError:
        rep.add("sign", "gz", "skip", "أُلغي")
        return
    bias_s = []
    t_end = time.monotonic() + 3.0
    while time.monotonic() < t_end:
        obj, _ = link.ask({"T": 126}, 1002, 0.3)
        g = num(obj.get("gz")) if obj else None
        if g is not None:
            bias_s.append(g)
    if len(bias_s) < 5:
        rep.add("sign", "gz", "fail", "قراءات انحياز غير كافية")
        return
    med = statistics.median(bias_s)
    sd = statistics.pstdev(bias_s)
    kept = [v for v in bias_s if abs(v - med) <= 3 * sd] or bias_s
    bias = statistics.median(kept)
    print("    الانحياز (وسيط): %+.4f°/ث من %d عيّنة" % (bias, len(bias_s)))

    trials = []
    for label, hint in (("cw", "**مع** عقارب الساعة (يمين)"),
                        ("ccw", "**عكس** عقارب الساعة (يسار) — أعِده مكانه")):
        print("\n    ▶ دوّر الروبوت **باليد** ربع لفّة (~90°) %s خلال ~3ث ثم اتركه ساكناً." % hint)
        try:
            input("      اضغط Enter ثم ابدأ الدوران… ")
        except EOFError:
            rep.add("sign", "gz", "skip", "أُلغي")
            return
        ang, n, spikes = integrate_hand(link, seconds, bias)
        trials.append((label, ang, n, spikes))
        print("      ⇒ التكامل الخام: %+.1f° (%d عيّنة · قفزات مرفوضة %d)" % (ang, n, spikes))

    (_, cw, _, _), (_, ccw, _, _) = trials
    rep.data["gyro_sign"] = {"bias": bias, "cw_raw_deg": cw, "ccw_raw_deg": ccw}
    if abs(cw) < SIGN_MIN_DEG or abs(ccw) < SIGN_MIN_DEG:
        rep.add("sign", "UGV01_GYRO_Z_SIGN", "warn",
                "حركة غير كافية (يمين %+.0f° · يسار %+.0f°، المطلوب ≥%.0f) — أعد"
                % (cw, ccw, SIGN_MIN_DEG))
        return
    if (cw > 0) == (ccw > 0):
        rep.add("sign", "UGV01_GYRO_Z_SIGN", "fail",
                "الاتجاهان بنفس الإشارة (%+.0f° · %+.0f°) ⇒ القياس غير متّسق — أعد"
                % (cw, ccw))
        return
    sign = 1 if cw > 0 else -1
    rep.data["gyro_sign"]["sign"] = sign
    rep.add("sign", "UGV01_GYRO_Z_SIGN", "ok",
            "= %+d   (يمين خاماً %+.1f° · يسار %+.1f°) — أرسل هذا السطر" % (sign, cw, ccw))
    print("    ⚠ المقادير هنا لا تُعاير شيئاً: ربع لفّة باليد تقديرية، والغرض الإشارة وحدها.")


# ═══ التشغيل ═════════════════════════════════════════════════════════
def main():
    global BOARD_PORT, GPS_PORT          # سكربت مستقل: --port/--gps-port تتجاوزهما
    ap = argparse.ArgumentParser(description="UGV01: جرد العتاد الفعلي — بلا حركة")
    ap.add_argument("--port", default=BOARD_PORT, help="منفذ اللوحة (افتراضياً المثبت)")
    ap.add_argument("--gps-port", default=GPS_PORT)
    ap.add_argument("--boot-wait", type=float, default=1.0,
                    help="انتظار بعد فتح منفذ اللوحة (ث) — UART لا يعيد إقلاعها")
    ap.add_argument("--gps-seconds", type=float, default=4.0)
    ap.add_argument("--geiger-seconds", type=float, default=10.0)
    ap.add_argument("--i2c-scan", action="store_true", help="مسح كامل 0x03–0x77 لكل ناقل")
    ap.add_argument("--gyro-sign", action="store_true",
                    help="مرحلة تفاعلية: إشارة gz باليد بلا محركات")
    ap.add_argument("--sign-seconds", type=float, default=6.0,
                    help="مدة كل دوران يدوي (ث)")
    a = ap.parse_args()
    BOARD_PORT, GPS_PORT = a.port, a.gps_port   # التجاوز يسري على كل الفحوص

    print("═" * 66)
    print("  UGV01 · جرد العتاد (check_setup) — لا يحرّك المحركات")
    print("  ⚠ أوقف السيرفر أولاً: يحجز المنافذ فتبدو «مشغولة» هنا")
    print("═" * 66)

    rep = Report()
    link = None
    try:
        check_config_txt(rep)
        check_ports(rep)
        if os.path.exists(BOARD_PORT):
            try:
                link = open_board(a.boot_wait)
            except Exception as e:
                rep.add("board", "فتح %s" % BOARD_PORT, "fail",
                        "%s (السيرفر يعمل؟ مجموعة dialout؟)" % e)
            if link:
                check_board(rep, link)
        check_i2c(rep, a.i2c_scan)
        check_camera(rep)
        check_gps(rep, a.gps_seconds)
        check_geiger(rep, a.geiger_seconds)
        if a.gyro_sign:
            if link:
                gyro_sign(rep, link, a.sign_seconds)
            else:
                rep.add("sign", "UGV01_GYRO_Z_SIGN", "skip", "لا وصلة باللوحة")
    except KeyboardInterrupt:
        print("\nأُوقف بـCtrl+C.")
    finally:
        if link:
            try:
                link.send(STOP_CMD)
                link.ser.close()
            except Exception:
                pass

    # ── الخلاصة ─────────────────────────────────────────────────────
    board_ok = any(s == "board" and it == "T:130 → T:1001" and st != "fail"
                   for s, it, st, _ in rep.rows)
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, "setup_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    rep.data["rows"] = [{"section": s, "item": i, "status": st, "detail": d}
                        for s, i, st, d in rep.rows]
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(rep.data, fh, ensure_ascii=False, indent=1, default=str)

    print()
    print("═" * 66)
    counts = {k: sum(1 for r in rep.rows if r[2] == k) for k in ICON}
    print("  ✅ %d · ⚠ %d · ❌ %d · — %d" % (counts["ok"], counts["warn"],
                                           counts["fail"], counts["skip"]))
    for s, i, st, d in rep.rows:
        if st in ("warn", "fail"):
            print("  %s %s: %s" % (ICON[st], i, d))
    print("  السجل: " + path)
    print("═" * 66)
    print("GO" if board_ok else "NO-GO")
    return 0 if board_ok else 1


if __name__ == "__main__":
    sys.exit(main())
