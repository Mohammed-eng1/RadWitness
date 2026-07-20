# -*- coding: utf-8 -*-
"""
imu.py — قارئ BNO055 (منقول من pi/tests/test_bno055.py المثبت)
==============================================================
I2C على العنوان المثبت 0x29 (اكتشاف احتياطي 0x28). heading مطلق + حالة
المعايرة الرباعية. read() سريع (~ms) يُستدعى كل ثانية من حلقة السيرفر.
"""
from pi.config import BNO055_ADDR

try:
    import board
    import busio
    import adafruit_bno055
    _IMU_LIBS_OK = True
except Exception:                     # noqa: BLE001
    _IMU_LIBS_OK = False


class IMUReader:
    def __init__(self, addr: int = BNO055_ADDR):
        self.ok = False
        self.error = None
        self.addr = None
        self._sensor = None
        self._heading = 0.0
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
                return
            except Exception as e:        # noqa: BLE001
                self.error = str(e)

    def read(self) -> None:
        if not self.ok:
            return
        try:
            euler = self._sensor.euler
            if euler and euler[0] is not None:
                self._heading = euler[0]
            self._cal = self._sensor.calibration_status
        except Exception:                 # noqa: BLE001 — I2C عابر
            pass

    def state(self) -> dict:
        sys_c, gyro_c, accel_c, mag_c = self._cal
        return {
            "ok": self.ok,
            "heading": round(self._heading, 1),
            "sys_cal": sys_c,
            "gyro_cal": gyro_c,
            "accel_cal": accel_c,
            "mag_cal": mag_c,
            "mag_warn": mag_c < 2,        # تحذير: الاتجاه غير موثوق حتى mag≥2
        }
