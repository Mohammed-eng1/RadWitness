#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_gps.py — اختبار منفرد لوحدة GPS NEO-M8N على الراسبري
=========================================================
يقرأ جُمل NMEA من UART العتاد ويحللها عبر pynmea2، ويعرض حالة القفل،
الإحداثيات، عدد الأقمار، وHDOP. قراءة فقط (لا نكتب للوحدة).

⚠ العتاد: GPS TX → GPIO15 RXD (الدبوس الفيزيائي 10) على UART الراسبري،
   9600 باود. منطق الوحدة 3.3V متوافق. تغذية الوحدة 5V (دبوس 4).
   يتطلب تفعيل UART وتحرير المنفذ من كونسول النظام (setup_pi.sh يفعله).

⚠ تعارض معروف: نفس المنفذ (GPIO14/15) يستخدمه UART لوحة Wave Rover عند
   وصولها — عندها تُخصَّص UART إضافية عبر dtoverlay (يوثَّق في wiring.md).

التشغيل على الراسبري:
    python3 pi/tests/test_gps.py                 # المنفذ الافتراضي /dev/serial0
    python3 pi/tests/test_gps.py /dev/ttyAMA0    # لتحديد منفذ آخر
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("خطأ: مكتبة pyserial غير مثبّتة. ثبّتها عبر setup_pi.sh.")
try:
    import pynmea2
except ImportError:
    sys.exit("خطأ: مكتبة pynmea2 غير مثبّتة. ثبّتها عبر setup_pi.sh.")

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"
BAUD = 9600


def main() -> None:
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
    except serial.SerialException as e:
        sys.exit(f"خطأ: تعذّر فتح {PORT} ({e}). تأكد من تفعيل UART وصحة المنفذ.")

    print(f"يقرأ GPS من {PORT} @ {BAUD}. القفل الأول للأقمار قد يستغرق دقائق في العراء. Ctrl-C للإيقاف.\n")
    fix = False
    lat = lng = 0.0
    sats = 0
    hdop = 0.0
    try:
        while True:
            raw = ser.readline().decode("ascii", errors="replace").strip()
            if not raw.startswith("$"):
                continue
            try:
                msg = pynmea2.parse(raw)
            except pynmea2.ParseError:
                continue

            # GGA: جودة القفل + عدد الأقمار + HDOP + الإحداثيات
            if isinstance(msg, pynmea2.types.talker.GGA):
                sats = int(msg.num_sats) if msg.num_sats else 0
                hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 0.0
                fix = msg.gps_qual not in (None, 0, "0")
                if fix and msg.latitude and msg.longitude:
                    lat, lng = msg.latitude, msg.longitude
            # RMC: تأكيد صلاحية الموقع (A = صالح)
            elif isinstance(msg, pynmea2.types.talker.RMC):
                if msg.status == "A" and msg.latitude and msg.longitude:
                    fix = True
                    lat, lng = msg.latitude, msg.longitude
            else:
                continue

            if fix:
                print(f"قفل ✅  lat={lat:.6f}  lng={lng:.6f}  أقمار={sats}  HDOP={hdop:.1f}")
            else:
                print(f"لا قفل ⏳  أقمار مرئية={sats}  (انتظر…)")
    except KeyboardInterrupt:
        print("\nتوقّف.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
