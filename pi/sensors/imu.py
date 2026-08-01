# -*- coding: utf-8 -*-
"""
imu.py — قارئ BNO055 (سائقان: smbus2 الافتراضي، Adafruit احتياطياً)
====================================================================
يوفّر:
  - `gyro_z_dps()` : معدل الدوران حول z بالدرجة/ث — **أساس الاتجاه الآن**
                     بعد موت جايرو الروفر (انظر pi/sensors/heading.py).
  - `euler_yaw()`  : yaw المدموج داخلياً (نسبي في وضع IMUPLUS).
  - `read()/state()`: للعرض في الواجهة (heading + حالة المعايرة الرباعية).

⚠ **الناقل 4 لا 1** (مثبت على العتاد): `i2cdetect -y 1` يُظهر أجهزة أخرى
   (0x0c, 0x3c, 0x42, 0x6b) ولا أثر لـBNO055، بينما القراءة تنجح على
   /dev/i2c-4 عند 0x29 وCHIP_ID=0xA0. مكتبة Adafruit عبر
   `busio.I2C(board.SCL, board.SDA)` **مقيَّدة بالناقل 1** (GPIO2/3) فلا تراه
   مهما فعلنا بالعناوين — لهذا السائق الافتراضي smbus2 (يقبل رقم الناقل).

⚠ الوضع الافتراضي **IMUPLUS (بلا مغنيتومتر)** — نفس سبب رفض البوصلة في
   CLAUDE.md §1: الهيكل معدني وحوله محركات بمغانط، فالمجال المقروء مجال
   الروبوت ويدور معه. في هذا الوضع `mag_cal` يبقى 0 **وهذا متوقَّع لا خطأ**.

⚠ الوحدات **مسؤولية كل سائق داخلياً** فلا يقع ضرب مزدوج في 57.3:
   - smbus2 : يضبط UNIT_SEL=0x00 → الجايرو درجة/ث (1/16 لكل LSB) مباشرة.
   - Adafruit: يُرجع راديان/ث → يحوّله السائق بحسب BNO055_GYRO_IN_RAD.
   كلاهما يخرج **درجة/ث** إلى الطبقة الأعلى.

⚠ الوصول من خيطين (حلقة بثّ السيرفر + حلقة اللفّ/السير) على نفس الناقل
   يتشابك، فكل تعامل مع الحسّاس داخل قُفل واحد.

⚠ **«صفر مضبوط» ليس دليل موت**: في وضع CONFIG تقرأ كل سجلات البيانات 0x00،
   والشريحة تعود إلى CONFIG وحدها بعد أي إعادة تشغيل ذاتية (هبوط جهد لحظي
   عند إقلاع المحركات). فقبل إعلان الوفاة **تُسأل الشريحة**: `health()` تقرأ
   CHIP_ID/OPR_MODE/SYS_STAT/SYS_ERR، و`recover()` تعيد التهيئة إن كانت حاضرة.
"""
from __future__ import annotations

import math
import struct
import threading
import time

from pi.config import (
    BNO055_ADDR, BNO055_I2C_BUS, BNO055_NO_MAG_MODE, BNO055_GYRO_IN_RAD,
)

try:
    from smbus2 import SMBus
    _SMBUS_OK = True
except Exception:                     # noqa: BLE001
    _SMBUS_OK = False


# ⚠ استيراد Adafruit **كسول**: مكتبة Blinka تفحص المنصة عند الاستيراد (قراءة
# device-tree ومطابقة اللوحة) فتكلّف ثوانٍ في **كل إقلاع** — وهي هنا مسار
# احتياطي لا يعمل أصلاً على عتادنا (busio مقيَّد بالناقل 1 والحسّاس على 4).
# دفع ثمنها عند الاستيراد يعني تأخير بدء السيرفر مقابل لا شيء.
_ADAFRUIT = None                      # None = لم تُجرَّب بعد، False = غير متاحة


def _load_adafruit():
    """يحمّل مكتبات Adafruit عند **أول حاجة فعلية** فقط. False عند غيابها."""
    global _ADAFRUIT
    if _ADAFRUIT is None:
        try:
            import board
            import busio
            import adafruit_bno055
            _ADAFRUIT = (board, busio, adafruit_bno055)
        except Exception:             # noqa: BLE001
            _ADAFRUIT = False
    return _ADAFRUIT


# ═══ سائق 1: smbus2 مباشرة بالسجلات (الافتراضي — يقبل أي ناقل) ════
class _SMBusDriver:
    """
    وصول مباشر بسجلات BNO055 — مأخوذ من سكربت أثبت عمله على العتاد
    (i2c-4 @ 0x29). لا يعتمد على board/busio فلا يقيّده الناقل 1.
    """
    # سجلات BNO055 (من ورقة البيانات)
    CHIP_ID_REG = 0x00
    GYR_X_LSB = 0x14
    EUL_H_LSB = 0x1A
    CALIB_STAT = 0x35
    SYS_STATUS = 0x39           # 0=خامل 1=خطأ 5=دمج يعمل 6=دمج متوقف
    SYS_ERR = 0x3A              # 0 = بلا خطأ
    UNIT_SEL = 0x3B
    OPR_MODE = 0x3D
    SYS_TRIGGER = 0x3F
    CHIP_ID_VAL = 0xA0
    MODE_CONFIG = 0x00
    MODE_IMUPLUS = 0x08
    MODE_NDOF = 0x0C
    LSB_PER_DEG = 16.0          # UNIT_SEL=0x00: زاوية ومعدل بـ1/16 درجة

    name = "smbus2"

    def __init__(self, bus_num: int, addr: int):
        self.bus_num = int(bus_num)
        self.addr = int(addr)
        self._bus = SMBus(self.bus_num)
        chip = self._bus.read_byte_data(self.addr, self.CHIP_ID_REG)
        if chip != self.CHIP_ID_VAL:
            try:
                self._bus.close()
            except Exception:         # noqa: BLE001
                pass
            raise OSError(f"CHIP_ID={hex(chip)} ≠ 0xA0 على i2c-{self.bus_num} "
                          f"@ {hex(self.addr)} — ليس BNO055")
        self._configure()

    def _configure(self) -> None:
        """
        تسلسل التهيئة من ورقة البيانات: CONFIG ثم الوحدات ثم وضع التشغيل.
        مفصول عن `__init__` لأن **`recover()` يعيده حرفياً** بعد أن تُعيد
        الشريحة تشغيل نفسها — لا نسختين تفترقان مع الوقت.
        """
        self._bus.write_byte_data(self.addr, self.OPR_MODE, self.MODE_CONFIG)
        time.sleep(0.03)
        self._bus.write_byte_data(self.addr, self.SYS_TRIGGER, 0x00)
        time.sleep(0.03)
        # UNIT_SEL=0x00 → درجات و درجة/ث (لا راديان) — أساس LSB_PER_DEG أعلاه
        self._bus.write_byte_data(self.addr, self.UNIT_SEL, 0x00)
        time.sleep(0.03)

    def set_mode(self, no_mag: bool) -> tuple:
        """يضبط وضع التشغيل. يُعيد (اسم الوضع، هل يُستخدم المغنيتومتر)."""
        mode = self.MODE_IMUPLUS if no_mag else self.MODE_NDOF
        self._bus.write_byte_data(self.addr, self.OPR_MODE, mode)
        time.sleep(0.05)
        return (("IMUPLUS (بلا مغنيتومتر)", False) if no_mag
                else ("NDOF", True))

    # ── تشخيص الشريحة نفسها (لا بيانات) ─────────────────────────
    def health(self) -> dict:
        """
        لقطة **حالة الشريحة** تفرّق بين ثلاث حالات يخلطها «الجايرو يقرأ صفراً»:
          • CHIP_ID ≠ 0xA0        → الشريحة غائبة عن الناقل (تغذية/أسلاك).
          • OPR_MODE == 0 (CONFIG) → حاضرة لكنها أعادت تشغيل نفسها؛ في CONFIG
                                     **كل سجلات البيانات تقرأ 0x00** فيبدو
                                     الجايرو ميتاً وهو حيّ → قابلة للإحياء.
          • وضع تشغيل سليم وصفر    → عطل حقيقي؛ SYS_ERR يسمّيه.
        """
        return {
            "chip_id": self._bus.read_byte_data(self.addr, self.CHIP_ID_REG),
            "opr_mode": self._bus.read_byte_data(self.addr, self.OPR_MODE) & 0x0F,
            "sys_stat": self._bus.read_byte_data(self.addr, self.SYS_STATUS),
            "sys_err": self._bus.read_byte_data(self.addr, self.SYS_ERR),
        }

    def reinit(self, no_mag: bool) -> tuple:
        """يعيد تسلسل التهيئة كاملاً (بعد إعادة تشغيل ذاتية للشريحة)."""
        self._configure()
        return self.set_mode(no_mag)

    @staticmethod
    def _s16(lo: int, hi: int) -> int:
        return struct.unpack("<h", bytes([lo, hi]))[0]

    def gyro_z_dps(self) -> float:
        """درجة/ث مباشرة (UNIT_SEL=0x00) — بلا أي تحويل راديان."""
        g = self._bus.read_i2c_block_data(self.addr, self.GYR_X_LSB, 6)
        return self._s16(g[4], g[5]) / self.LSB_PER_DEG

    def euler_yaw(self) -> float:
        e = self._bus.read_i2c_block_data(self.addr, self.EUL_H_LSB, 2)
        return self._s16(e[0], e[1]) / self.LSB_PER_DEG

    def calibration(self) -> tuple:
        """(sys, gyro, accel, mag) — كلٌّ 0..3."""
        c = self._bus.read_byte_data(self.addr, self.CALIB_STAT)
        return ((c >> 6) & 3, (c >> 2) & 3, (c >> 4) & 3, c & 3)

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:             # noqa: BLE001
            pass


# ═══ سائق 2: Adafruit (احتياطي — الناقل 1 حصراً) ═════════════════
class _AdafruitDriver:
    name = "adafruit"

    def __init__(self, addr: int):
        libs = _load_adafruit()
        if not libs:
            raise OSError("مكتبات Adafruit غير مثبّتة")
        board, busio, self._ada = libs
        self.addr = int(addr)
        i2c = busio.I2C(board.SCL, board.SDA)
        self._s = self._ada.BNO055_I2C(i2c, address=self.addr)
        _ = self._s.calibration_status        # قراءة تحقق
        self.bus_num = 1                      # busio مقيَّد بالناقل 1

    def set_mode(self, no_mag: bool) -> tuple:
        if not no_mag:
            return ("NDOF", True)
        self._s.mode = self._ada.IMUPLUS_MODE
        return ("IMUPLUS (بلا مغنيتومتر)", False)

    def health(self) -> dict:
        """المكتبة لا تكشف SYS_STAT/SYS_ERR — نُعيد ما تكشفه فقط، بلا تخمين."""
        return {"chip_id": None, "opr_mode": int(self._s.mode),
                "sys_stat": None, "sys_err": None}

    def reinit(self, no_mag: bool) -> tuple:
        return self.set_mode(no_mag)

    def gyro_z_dps(self):
        g = self._s.gyro
        if not g or g[2] is None:
            return None
        z = float(g[2])
        # ⚠ التحويل هنا **وحده** — مكتبة Adafruit تُرجع راديان/ث
        return math.degrees(z) if BNO055_GYRO_IN_RAD else z

    def euler_yaw(self):
        e = self._s.euler
        if not e or e[0] is None:
            return None
        return float(e[0])

    def calibration(self) -> tuple:
        return self._s.calibration_status

    def close(self) -> None:
        pass


class IMUReader:
    """
    يجرّب smbus2 على `BNO055_I2C_BUS` أولاً (المسار المثبت على العتاد)، ثم
    Adafruit على الناقل 1 احتياطياً.

    ⚠ **لا طمس لسبب الفشل**: كل محاولة تُسجَّل بعنوانها وناقلها، وتُجمع كلها
       في `self.error`. (قبل هذا كان `self.error` يُكتب فوقه في كل محاولة،
       فيُبلَّغ عن فشل العنوان الأخير 0x28 بينما العنوان المُعدّ 0x29 —
       رسالة تقود التشخيص في الاتجاه الخاطئ.)
    """

    def __init__(self, addr: int = BNO055_ADDR, bus_num: int = BNO055_I2C_BUS,
                 no_mag_mode: bool = BNO055_NO_MAG_MODE):
        self.ok = False
        self.error = None
        self.addr = None
        self.bus_num = None
        self.driver = None
        self.mode_name = "غير مهيّأ"
        self.mag_used = True          # NDOF الافتراضي يدمج المغنيتومتر
        self._drv = None
        self._lock = threading.Lock()
        self._heading = 0.0
        self._gyro_z_dps = 0.0
        self._cal = (0, 0, 0, 0)
        self._no_mag = bool(no_mag_mode)
        self.recoveries = 0           # مرات إحياء ناجحة (تُعرض في الواجهة)
        self.last_recovery = None     # تفاصيل آخر محاولة إحياء

        attempts = []                 # (وصف المحاولة، سبب الفشل)

        # ── 1) smbus2 على الناقل المُعدّ (المسار المثبت) ─────────
        if _SMBUS_OK:
            for a in (addr, 0x28 if addr != 0x28 else 0x29):
                try:
                    self._drv = _SMBusDriver(bus_num, a)
                    self.addr, self.bus_num = a, bus_num
                    self.ok = True
                    break
                except Exception as e:        # noqa: BLE001
                    attempts.append((f"smbus2 i2c-{bus_num} @ {hex(a)}", str(e)))
        else:
            attempts.append(("smbus2", "غير مثبّت (pip install smbus2)"))

        # ── 2) Adafruit على الناقل 1 (احتياطي — يُحمَّل الآن فقط) ──
        if not self.ok:
            if _load_adafruit():
                for a in (addr, 0x28 if addr != 0x28 else 0x29):
                    try:
                        self._drv = _AdafruitDriver(a)
                        self.addr, self.bus_num = a, 1
                        self.ok = True
                        break
                    except Exception as e:    # noqa: BLE001
                        attempts.append((f"adafruit i2c-1 @ {hex(a)}", str(e)))
            else:
                attempts.append(("adafruit", "المكتبات غير مثبّتة"))

        if self.ok:
            self.driver = self._drv.name
            self._set_mode(no_mag_mode)
        else:
            self.error = " · ".join(f"{what}: {why}" for what, why in attempts)

    # ── الوضع: IMUPLUS (جايرو + تسارع، بلا مغنيتومتر) ────────────
    def _set_mode(self, no_mag: bool) -> None:
        """
        يُقصي المغنيتومتر كلياً. الفشل هنا **لا يُسكت**: يبقى الوضع NDOF
        وتُعلن `mag_used=True` فيرفض مصدر الاتجاه المدموج العمل (قاعدة §1).
        """
        if not no_mag:
            self.mode_name, self.mag_used = "NDOF", True
            return
        try:
            with self._lock:
                self.mode_name, self.mag_used = self._drv.set_mode(True)
        except Exception as e:            # noqa: BLE001
            self.mode_name = "NDOF (تعذّر التبديل)"
            self.mag_used = True
            self.error = f"تعذّر ضبط وضع IMUPLUS: {e}"

    # ── تشخيص وإحياء (يُستدعيان من مصدر الاتجاه عند اشتباه الموت) ─
    def health(self) -> dict:
        """حالة الشريحة كما تقرأها هي — بلا تفسير ولا تخمين."""
        if not self.ok or self._drv is None:
            return {"ok": False, "reason": self.error or "الحسّاس غير مفتوح"}
        try:
            with self._lock:
                h = self._drv.health()
            h["ok"] = True
            return h
        except Exception as e:            # noqa: BLE001
            return {"ok": False, "reason": f"تعذّرت قراءة حالة الشريحة: {e}"}

    def recover(self) -> dict:
        """
        محاولة إحياء بعد «صفر مضبوط متتابع». **تسأل الشريحة أولاً** ثم تقرر:

        | ما تقوله الشريحة | التشخيص | الإجراء |
        |---|---|---|
        | لا تردّ على الناقل | مفقودة (تغذية/أسلاك) | لا إحياء — عطل حقيقي |
        | CHIP_ID ≠ 0xA0 | ليست BNO055 / ناقل خاطئ | لا إحياء |
        | OPR_MODE = 0 (CONFIG) | أعادت تشغيل نفسها | **إعادة تهيئة** |
        | وضع سليم + صفر | عطل داخلي | لا إحياء — يُبلَّغ بـSYS_ERR |

        ⚠ لا تُعلن النجاح إلا بعد **قراءة تحقّق** بعد إعادة التهيئة: إعلان
           الإحياء بمجرد نجاح الكتابة يعيدنا إلى نفس الفشل الصامت الذي نطارده.
        يُعيد: {"recovered": bool, "detail": نص عربي مقروء, "health": {...}}
        """
        h = self.health()
        if not h.get("ok"):
            self.last_recovery = {"recovered": False, "detail": h.get("reason")}
            return {"recovered": False, "detail": h.get("reason"), "health": h}

        chip, mode = h.get("chip_id"), h.get("opr_mode")
        if chip is not None and chip != _SMBusDriver.CHIP_ID_VAL:
            detail = (f"الشريحة لا تردّ بهويتها (CHIP_ID={hex(chip)} ≠ 0xA0) — "
                      f"مفقودة عن i2c-{self.bus_num} @ {hex(self.addr or 0)}: "
                      f"افحص التغذية والأسلاك (i2cdetect -y {self.bus_num})")
            self.last_recovery = {"recovered": False, "detail": detail}
            return {"recovered": False, "detail": detail, "health": h}

        if mode not in (None, _SMBusDriver.MODE_CONFIG):
            detail = (f"الشريحة في وضع تشغيل سليم (OPR_MODE={hex(mode)}) ومع ذلك "
                      f"الجايرو صفر مضبوط — SYS_STAT={h.get('sys_stat')} "
                      f"SYS_ERR={h.get('sys_err')}: عطل داخلي لا تصلحه إعادة التهيئة")
            self.last_recovery = {"recovered": False, "detail": detail}
            return {"recovered": False, "detail": detail, "health": h}

        # حاضرة وفي CONFIG (أو وضع مجهول) → أعِد التهيئة ثم **تحقّق**
        try:
            with self._lock:
                self.mode_name, self.mag_used = self._drv.reinit(self._no_mag)
                after = self._drv.health()
                z = self._drv.gyro_z_dps()
        except Exception as e:                # noqa: BLE001
            detail = f"فشلت إعادة التهيئة: {e}"
            self.last_recovery = {"recovered": False, "detail": detail}
            return {"recovered": False, "detail": detail, "health": h}

        back = after.get("opr_mode")
        if back in (None, _SMBusDriver.MODE_CONFIG):
            detail = (f"أُعيدت التهيئة لكن الشريحة بقيت في CONFIG "
                      f"(OPR_MODE={back}) — لا تقبل وضع التشغيل")
            self.last_recovery = {"recovered": False, "detail": detail}
            return {"recovered": False, "detail": detail, "health": after}

        self.recoveries += 1
        detail = (f"عادت الشريحة إلى CONFIG (إعادة تشغيل ذاتية — الأرجح هبوط "
                  f"جهد عند إقلاع المحركات) وأُعيدت تهيئتها إلى {self.mode_name}؛ "
                  f"قراءة تحقّق gz={z}")
        self.last_recovery = {"recovered": True, "detail": detail}
        return {"recovered": True, "detail": detail, "health": after}

    # ── قراءة سريعة للاتجاه (تُستدعى من حلقات اللفّ ~50Hz) ────────
    def gyro_z_dps(self):
        """معدل الدوران حول z بالدرجة/ث، أو None عند فشل قراءة عابر."""
        if not self.ok:
            return None
        try:
            with self._lock:
                z = self._drv.gyro_z_dps()
        except Exception:                 # noqa: BLE001 — I2C عابر
            return None
        if z is None:
            return None
        self._gyro_z_dps = float(z)
        return self._gyro_z_dps

    def euler_yaw(self):
        """yaw المدموج داخلياً (نسبي في IMUPLUS)، أو None عند فشل عابر."""
        if not self.ok:
            return None
        try:
            with self._lock:
                y = self._drv.euler_yaw()
        except Exception:                 # noqa: BLE001
            return None
        if y is None:
            return None
        self._heading = float(y)
        return self._heading

    # ── قراءة دورية للعرض (كل ثانية من حلقة السيرفر) ─────────────
    def read(self) -> None:
        if not self.ok:
            return
        try:
            with self._lock:
                y = self._drv.euler_yaw()
                cal = self._drv.calibration()
                z = self._drv.gyro_z_dps()
            if y is not None:
                self._heading = float(y)
            if cal:
                self._cal = cal
            if z is not None:
                self._gyro_z_dps = float(z)
        except Exception:                 # noqa: BLE001 — I2C عابر
            pass

    def state(self) -> dict:
        sys_c, gyro_c, accel_c, mag_c = self._cal
        return {
            "ok": self.ok,
            "error": self.error,
            "addr": self.addr,
            "bus": self.bus_num,
            "driver": self.driver,
            "mode": self.mode_name,
            # None عند غياب الحسّاس: لا نُبلّغ «يستخدم المغنيتومتر» عن حسّاس
            # غير موجود (الواجهة تفرّق بين «معطّل» و«مجهول»).
            "mag_used": self.mag_used if self.ok else None,
            "heading": round(self._heading, 1),
            "gyro_z_dps": round(self._gyro_z_dps, 2),
            "sys_cal": sys_c,
            "gyro_cal": gyro_c,
            "accel_cal": accel_c,
            "mag_cal": mag_c,
            # ⚠ في IMUPLUS يبقى mag_cal=0 بالتصميم — فالتحذير يخصّ الأوضاع
            # التي تستخدم المغنيتومتر فعلاً، وإلا كان إنذاراً كاذباً دائماً.
            "mag_warn": bool(self.mag_used and mag_c < 2),
            "gyro_ready": gyro_c >= 2,    # الجايرو هو ما يهمّ الاتجاه الآن
            # إحياء الشريحة بعد عودتها إلى CONFIG — **يجب أن يكون مرئياً**:
            # عدّاد يتصاعد يعني تغذية 3.3V تنهار مع كل إقلاع محركات.
            "recoveries": self.recoveries,
            "last_recovery": self.last_recovery,
        }

    def close(self) -> None:
        if self._drv is not None:
            self._drv.close()


# ── قارئ مشترك: منع فتح I2C مرتين على نفس الحسّاس ────────────────
# حلقة بثّ السيرفر ومصدر الاتجاه يحتاجان نفس الحسّاس؛ نسختان تعنيان تبديل
# وضعٍ متسابقاً وحملاً مضاعفاً على الناقل.
_shared: IMUReader | None = None
_shared_lock = threading.Lock()


def get_imu() -> IMUReader:
    global _shared
    with _shared_lock:
        if _shared is None:
            _shared = IMUReader()
        return _shared
