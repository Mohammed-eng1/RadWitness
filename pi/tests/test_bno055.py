#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_bno055.py — اختبار منفرد لحساس التوجيه BNO055 على الراسبري
===============================================================
يقرأ heading المطلق (وضع NDOF: دمج مغناطيسي/جيرو/تسارع) وحالة المعايرة
الرباعية (sys/gyro/accel/mag) عبر مكتبة Adafruit على I2C.

⚠ العتاد: BNO055 على I2C — SDA=GPIO2 (دبوس 3)، SCL=GPIO3 (دبوس 5)،
   التغذية 3.3V (دبوس 17)، GND مشترك. العنوان 0x28 (اكتشاف تلقائي 0x29).
   افحص الوجود أولاً:  i2cdetect -y 1

⚠ المعايرة: لوّح بالحساس على شكل ∞ حتى يبلغ mag ≥ 2 — قبلها الـheading
   غير موثوق (نُظهر التحذير). heading المطلق أساس المسح الدوراني 360°.

التشغيل على الراسبري:
    python3 pi/tests/test_bno055.py       # Ctrl-C للإيقاف
"""
import sys
import time

try:
    import board
    import busio
    import adafruit_bno055
except ImportError:
    sys.exit("خطأ: مكتبات Adafruit BNO055 غير مثبّتة. ثبّتها عبر setup_pi.sh (تعمل على الراسبري).")


def connect():
    """يهيّئ I2C ويجرّب العنوانين 0x28 ثم 0x29 (اكتشاف تلقائي)."""
    i2c = busio.I2C(board.SCL, board.SDA)
    for addr in (0x28, 0x29):
        try:
            sensor = adafruit_bno055.BNO055_I2C(i2c, address=addr)
            _ = sensor.calibration_status      # قراءة تحقق
            print(f"وُجد BNO055 على العنوان 0x{addr:02X}")
            return sensor
        except (ValueError, OSError):
            continue
    sys.exit("خطأ: لم يُعثر على BNO055 على 0x28/0x29. افحص التوصيل: i2cdetect -y 1")


def main() -> None:
    sensor = connect()
    print("يقرأ الاتجاه والمعايرة. لوّح على شكل ∞ حتى mag ≥ 2. Ctrl-C للإيقاف.\n")
    try:
        while True:
            time.sleep(0.5)
            euler = sensor.euler                 # (heading, roll, pitch) بالدرجات
            sys_c, gyro_c, accel_c, mag_c = sensor.calibration_status
            heading = euler[0] if euler and euler[0] is not None else float("nan")
            warn = "  ⚠ mag<2: الاتجاه غير موثوق — عايِر" if mag_c < 2 else ""
            print(f"heading={heading:6.1f}°  |  معايرة sys={sys_c} gyro={gyro_c} "
                  f"accel={accel_c} mag={mag_c}{warn}")
    except KeyboardInterrupt:
        print("\nتوقّف.")


if __name__ == "__main__":
    main()
