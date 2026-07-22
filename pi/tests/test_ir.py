#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_ir.py — اختبار مكتبي لحساس عائق بالأشعة تحت الحمراء (IR obstacle)
=====================================================================
حساس رقمي (وحدة FC-51 أو ما يشابهها بمقارن LM393): خرج OUT رقمي — عادةً
LOW عند وجود عائق أمامه، HIGH عند الخلوّ (بعض الوحدات معكوسة). فيه مُعايِر
(بوتنشيومتر) لضبط المدى، ومؤشر LED يضيء عند الكشف.

⚠ معماريّاً: في v2 حساسات العوائق ضمن Wave Rover — هذا اختبار مكتبي للعتاد
   بين يديك على الراسبري مباشرة.

⚠ العتاد (حرج): **غذِّ الوحدة بـ3.3V (وليس 5V)** حتى يبقى خرج OUT عند 3.3V
   آمناً على دبوس GPIO الراسبري (يتحمل 3.3V فقط). GND مشترك.
   OUT → GPIO25 (الدبوس الفيزيائي 22).

التشغيل على الراسبري:
    python3 pi/tests/test_ir.py            # المنفذ الافتراضي GPIO25
    python3 pi/tests/test_ir.py 6          # لتحديد BCM آخر (مثل يمين/يسار)
"""
import sys
import time

try:
    import lgpio
except ImportError:
    sys.exit("خطأ: مكتبة lgpio غير مثبّتة. ثبّتها عبر setup_pi.sh (apt: python3-lgpio).")

IR_GPIO = int(sys.argv[1]) if len(sys.argv) > 1 else 25   # BCM25 (دبوس 22)
OBSTACLE_LEVEL = 0        # LOW = عائق (اقلبه إلى 1 لو وحدتك معكوسة)


def open_chip():
    """gpiochip0 (باي 4) أو gpiochip4 (باي 5)."""
    last_err = None
    for chip in (0, 4):
        try:
            return lgpio.gpiochip_open(chip), chip
        except Exception as e:            # noqa: BLE001
            last_err = e
    sys.exit(f"خطأ: تعذّر فتح gpiochip ({last_err}). تأكد أن المستخدم ضمن مجموعة gpio.")


def main() -> None:
    h, chip = open_chip()
    # دخل رقمي بلا مقاومة (خرج المقارن مدفوع). لو تذبذبت القراءة جرّب SET_PULL_UP.
    lgpio.gpio_claim_input(h, IR_GPIO)

    print(f"حساس IR على GPIO{IR_GPIO} (gpiochip{chip}). لوّح يدك أمامه. Ctrl-C للإيقاف.\n")
    last = None
    try:
        while True:
            v = lgpio.gpio_read(h, IR_GPIO)
            obstacle = (v == OBSTACLE_LEVEL)
            if v != last:
                print(("🚧 عائق مكتشَف!" if obstacle else "✅ المسار خالٍ")
                      + f"   (GPIO{IR_GPIO} = {v})")
                last = v
            time.sleep(0.12)
    except KeyboardInterrupt:
        print("\nتوقّف.")
    finally:
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
