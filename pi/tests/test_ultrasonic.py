#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_ultrasonic.py — اختبار مكتبي لحساس المسافة فوق-الصوتي HC-SR04
==================================================================
يرسل نبضة TRIG (10µs) ويقيس زمن بقاء ECHO مرتفعاً عبر تنبيهات lgpio
(طابع زمني بالنانوثانية من النواة — أدق من الاستقصاء في بايثون)، ثم
يحوّله إلى مسافة بالسنتيمتر.

⚠ ملاحظة معمارية: في تصميم v2 الحساس فوق-الصوتي ليس على الراسبري — هو
   ضمن Wave Rover (ESP32) ويصل عبر بروتوكول Waveshare (أو محاكاةً في sim).
   هذا السكربت **اختبار مكتبي للعتاد** فقط للتحقق من الحساس بين يديك.

⚠ العتاد (حرج): HC-SR04 يعمل بـ5V وطرف ECHO يُخرج 5V — يتلف GPIO الراسبري
   (3.3V فقط) إن وُصل مباشرة. **إلزامي مقسّم جهد على ECHO** (1kΩ + 2kΩ):
       ECHO ─[1kΩ]─┬─ GPIO24 (دبوس 18)
                   └─[2kΩ]─ GND
   TRIG على GPIO23 (دبوس 16) مباشر، VCC=5V، GND مشترك.

التشغيل على الراسبري:
    python3 pi/tests/test_ultrasonic.py            # المنافذ الافتراضية 23/24
    python3 pi/tests/test_ultrasonic.py 23 24      # TRIG ECHO مخصّصان
"""
import sys
import time

try:
    import lgpio
except ImportError:
    sys.exit("خطأ: مكتبة lgpio غير مثبّتة. ثبّتها عبر setup_pi.sh (apt: python3-lgpio).")

TRIG_GPIO = int(sys.argv[1]) if len(sys.argv) > 1 else 23   # BCM23 (دبوس 16) → TRIG
ECHO_GPIO = int(sys.argv[2]) if len(sys.argv) > 2 else 24   # BCM24 (دبوس 18) ← ECHO (عبر مقسّم)

SPEED_CM_PER_US = 0.0343    # سرعة الصوت في الهواء (~343 م/ث)
TIMEOUT_S       = 0.06      # مهلة انتظار الصدى (~4م ذهاباً وإياباً)
MAX_PLAUSIBLE_CM = 450.0    # فوقها = خارج مدى HC-SR04 (نتجاهلها)


def open_chip():
    """يفتح gpiochip0 (باي 4) أو gpiochip4 (باي 5)."""
    last_err = None
    for chip in (0, 4):
        try:
            return lgpio.gpiochip_open(chip), chip
        except Exception as e:                 # noqa: BLE001
            last_err = e
    sys.exit(f"خطأ: تعذّر فتح gpiochip ({last_err}). تأكد أن المستخدم ضمن مجموعة gpio.")


def main() -> None:
    h, chip = open_chip()
    lgpio.gpio_claim_output(h, TRIG_GPIO, 0)   # TRIG منخفض ابتداءً
    lgpio.gpio_claim_alert(h, ECHO_GPIO, lgpio.BOTH_EDGES)

    # حالة القياس تُحدَّث من خيط التنبيهات (طوابع زمنية بالنانوثانية)
    st = {"rise": 0, "width": None}

    def cb_echo(chip_, gpio_, level, tick):
        if level == 1:                         # حافة صاعدة: بدء الصدى
            st["rise"] = tick
        elif level == 0 and st["rise"]:        # حافة هابطة: نهاية الصدى
            st["width"] = tick - st["rise"]

    cb = lgpio.callback(h, ECHO_GPIO, lgpio.BOTH_EDGES, cb_echo)

    def measure_once():
        """قياس مفرد: يُعيد المسافة (سم) أو None عند لا صدى/قراءة غير منطقية."""
        st["rise"] = 0
        st["width"] = None
        lgpio.gpio_write(h, TRIG_GPIO, 1)          # نبضة تحفيز 10µs
        time.sleep(0.00001)
        lgpio.gpio_write(h, TRIG_GPIO, 0)
        t0 = time.time()
        while st["width"] is None and (time.time() - t0) < TIMEOUT_S:
            time.sleep(0.0005)
        if st["width"] is None:
            return None
        dist_cm = (st["width"] / 1000.0) * SPEED_CM_PER_US / 2.0   # ns→µs→سم
        if dist_cm <= 0 or dist_cm > MAX_PLAUSIBLE_CM:
            return None                            # قراءة شاذّة تُرفض
        return dist_cm

    print(f"HC-SR04: TRIG=GPIO{TRIG_GPIO}، ECHO=GPIO{ECHO_GPIO} (gpiochip{chip}). "
          f"وسيط 5 قياسات لكبح الضجيج. Ctrl-C للإيقاف.\n")
    try:
        while True:
            # 5 قياسات متتابعة (فاصل 60ms — يحتاجه HC-SR04 لتفادي تداخل الصدى)،
            # ثم الوسيط (median) — يلغي القفزات الشاذّة تماماً.
            samples = []
            for _ in range(5):
                d = measure_once()
                if d is not None:
                    samples.append(d)
                time.sleep(0.06)
            if len(samples) >= 3:
                samples.sort()
                median = samples[len(samples) // 2]
                print(f"المسافة = {median:6.1f} سم   (من {len(samples)}/5 قياسات صالحة)")
            else:
                print(f"قياس ضعيف ⏳ ({len(samples)}/5 صالحة) — تحقّق المقسّم/التوصيل أو السطح عاكس رديء")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nتوقّف.")
    finally:
        cb.cancel()
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
