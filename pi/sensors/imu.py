# -*- coding: utf-8 -*-
"""
imu.py — قارئ BNO055 (منقول من pi/tests/test_bno055.py المثبت)
==============================================================
I2C على العنوان المثبت 0x29 (اكتشاف احتياطي 0x28). يوفّر:
  - `gyro_z_dps()` : معدل الدوران حول z بالدرجة/ث — **أساس الاتجاه الآن**
                     بعد موت جايرو الروفر (انظر pi/sensors/heading.py).
  - `euler_yaw()`  : yaw المدموج داخلياً (نسبي في وضع IMUPLUS).
  - `read()/state()`: للعرض في الواجهة (heading + حالة المعايرة الرباعية).

⚠ الوضع الافتراضي **IMUPLUS (بلا مغنيتومتر)** — نفس سبب رفض البوصلة في
   CLAUDE.md §1: الهيكل معدني وحوله محركات بمغانط، فالمجال المقروء مجال
   الروبوت ويدور معه. في هذا الوضع `mag_cal` يبقى 0 **وهذا متوقَّع لا خطأ**.

⚠ الوحدات: مكتبة Adafruit تُرجع الجايرو بـ**راديان/ث** (ثابتها الداخلي
   0.00109083 = (1/16 درجة/ث لكل LSB) × π/180) → نحوّل هنا إلى درجة/ث.

⚠ الوصول من خيطين (حلقة بثّ السيرفر + حلقة اللفّ/السير) على نفس ناقل I2C
   يتشابك، فكل تعامل مع الحسّاس داخل قُفل واحد.
"""
from __future__ import annotations

import math
import threading

from pi.config import BNO055_ADDR, BNO055_NO_MAG_MODE, BNO055_GYRO_IN_RAD

try:
    import board
    import busio
    import adafruit_bno055
    _IMU_LIBS_OK = True
except Exception:                     # noqa: BLE001
    _IMU_LIBS_OK = False


class IMUReader:
    def __init__(self, addr: int = BNO055_ADDR, no_mag_mode: bool = BNO055_NO_MAG_MODE):
        self.ok = False
        self.error = None
        self.addr = None
        self.mode_name = "غير مهيّأ"
        self.mag_used = True          # NDOF الافتراضي يدمج المغنيتومتر
        self._sensor = None
        self._lock = threading.Lock()
        self._heading = 0.0
        self._gyro_z_dps = 0.0
        self._cal = (0, 0, 0, 0)

        if not _IMU_LIBS_OK:
            self.error = "مكتبات BNO055 غير مثبّتة"
            return
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
        except Exception as e:            # noqa: BLE001
            self.error = str(e)
            return
        for a in (addr, 0x28):            # المثبت 0x29 أولاً ثم الافتراضي
            try:
                s = adafruit_bno055.BNO055_I2C(i2c, address=a)
                _ = s.calibration_status  # قراءة تحقق
                self._sensor = s
                self.addr = a
                self.ok = True
                break
            except Exception as e:        # noqa: BLE001
                self.error = str(e)
        if self.ok and no_mag_mode:
            self._set_no_mag_mode()

    # ── الوضع: IMUPLUS (جايرو + تسارع، بلا مغنيتومتر) ────────────
    def _set_no_mag_mode(self) -> None:
        """
        يُقصي المغنيتومتر كلياً. الفشل هنا **لا يُسكت**: يبقى الوضع NDOF
        وتُعلن `mag_used=True` فيرفض مصدر الاتجاه المدموج العمل (قاعدة §1).
        """
        try:
            with self._lock:
                self._sensor.mode = adafruit_bno055.IMUPLUS_MODE
            self.mode_name = "IMUPLUS (بلا مغنيتومتر)"
            self.mag_used = False
        except Exception as e:            # noqa: BLE001
            self.mode_name = "NDOF (تعذّر التبديل)"
            self.mag_used = True
            self.error = f"تعذّر ضبط وضع IMUPLUS: {e}"

    # ── قراءة سريعة للاتجاه (تُستدعى من حلقات اللفّ ~50Hz) ────────
    def gyro_z_dps(self):
        """معدل الدوران حول z بالدرجة/ث، أو None عند فشل قراءة عابر."""
        if not self.ok:
            return None
        try:
            with self._lock:
                g = self._sensor.gyro
        except Exception:                 # noqa: BLE001 — I2C عابر
            return None
        if not g or g[2] is None:
            return None
        z = float(g[2])
        if BNO055_GYRO_IN_RAD:
            z = math.degrees(z)
        self._gyro_z_dps = z
        return z

    def euler_yaw(self):
        """yaw المدموج داخلياً (نسبي في IMUPLUS)، أو None عند فشل عابر."""
        if not self.ok:
            return None
        try:
            with self._lock:
                euler = self._sensor.euler
        except Exception:                 # noqa: BLE001
            return None
        if not euler or euler[0] is None:
            return None
        self._heading = float(euler[0])
        return self._heading

    # ── قراءة دورية للعرض (كل ثانية من حلقة السيرفر) ─────────────
    def read(self) -> None:
        if not self.ok:
            return
        try:
            with self._lock:
                euler = self._sensor.euler
                cal = self._sensor.calibration_status
                gyro = self._sensor.gyro
            if euler and euler[0] is not None:
                self._heading = float(euler[0])
            self._cal = cal
            if gyro and gyro[2] is not None:
                z = float(gyro[2])
                self._gyro_z_dps = math.degrees(z) if BNO055_GYRO_IN_RAD else z
        except Exception:                 # noqa: BLE001 — I2C عابر
            pass

    def state(self) -> dict:
        sys_c, gyro_c, accel_c, mag_c = self._cal
        return {
            "ok": self.ok,
            "error": self.error,
            "addr": self.addr,
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
        }


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
