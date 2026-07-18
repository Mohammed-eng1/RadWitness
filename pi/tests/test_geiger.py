#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_geiger.py — اختبار منفرد لعداد جيجر CAJOE (J305) على الراسبري
==================================================================
يعدّ النبضات عبر pigpio (مقاطعة على الحافة الصاعدة، دخل عائم بلا مقاومة
رفع/سحب — أي مقاومة تقتل نبضة VIN الضعيفة عالية الممانعة، مؤكد على
العتاد في المشروع السابق)، ثم يحسب CPM على نافذة 30 ثانية منزلقة +
متوسط أسّي سريع، ويحوّلها إلى µSv/h بمعايرة مخبرية مثبتة ضد Cs-137.

⚠ العتاد: لوحة الجيجر تُغذّى 3.3V حصراً (وليس 5V) فتبقى ذروة النبضة
   آمنة على دبوس GPIO الراسبري (يتحمل 3.3V فقط). النبضة على GPIO17
   (BCM17 = الدبوس الفيزيائي 11). GND مشترك إلزامي.

المعايرة (ثوابت مثبتة مخبرياً — لا تُغيَّر):
    K = 111 CPM/(µSv/h)   مقاسة ضد مصدر Cs-137 مرجعي (662 keV)، 2026-07
    τ = 200 µs            زمن ميت مقاس من نفس جلسة المعايرة
    المصدر: legacy/RadiationRover/firmware/esp32_main/{config.h, geiger.h}

التشغيل على الراسبري:
    sudo systemctl start pigpiod      # لازم لعمل pigpio
    python3 pi/tests/test_geiger.py   # Ctrl-C للإيقاف
"""
import sys
import time
from collections import deque

try:
    import pigpio
except ImportError:
    sys.exit("خطأ: مكتبة pigpio غير مثبّتة. ثبّتها عبر setup_pi.sh (تعمل على الراسبري).")

# ── الثوابت ────────────────────────────────────────────────────────
GEIGER_GPIO           = 17        # BCM17 (دبوس 11) — جيجر 1 (خلف SINGLE_TUBE_MODE)
CPM_PER_USVH          = 111.0     # K — معاير ضد Cs-137 (662 keV)، 2026-07 — ثابت مثبت
DEADTIME_TAU_S        = 200e-6    # τ = 200µs — تصحيح الزمن الميت — ثابت مثبت
WINDOW_S              = 30        # نافذة CPM المنزلقة (ثوانٍ)
EMA_ALPHA             = 0.2       # معامل المتوسط الأسّي السريع (~5s)
HIGH_RATE_WARNING_CPM = 30000.0   # فوقها: تحذير دقة — مقاطعات لينكس تفقد نبضات عند
                                  #        المعدلات العالية (وثّقناه بشفافية، خلاف ISR
                                  #        الـESP32 المدمج في IRAM)


def correct_dead_time(cpm_meas: float) -> float:
    """تصحيح الزمن الميت: CPM_true = CPM_meas / (1 − CPM_meas·τ/60)."""
    d = 1.0 - cpm_meas * DEADTIME_TAU_S / 60.0
    if d < 0.1:
        d = 0.1                    # حماية من القسمة على صفر عند التشبع الشديد
    return cpm_meas / d


def main() -> None:
    pi = pigpio.pi()
    if not pi.connected:
        sys.exit("خطأ: تعذّر الاتصال بـpigpiod. شغّله: sudo systemctl start pigpiod")

    # دخل عائم تماماً — لا PUD (أي مقاومة رفع/سحب تقتل نبضة VIN)
    pi.set_mode(GEIGER_GPIO, pigpio.INPUT)
    pi.set_pull_up_down(GEIGER_GPIO, pigpio.PUD_OFF)

    total = {"count": 0}           # العدّاد التراكمي (يزيده رد النداء في خيط pigpio)

    def on_pulse(gpio, level, tick):
        total["count"] += 1

    cb = pi.callback(GEIGER_GPIO, pigpio.RISING_EDGE, on_pulse)

    print(f"يعدّ نبضات الجيجر على GPIO{GEIGER_GPIO} (BCM) — نافذة {WINDOW_S}s. Ctrl-C للإيقاف.\n")
    per_sec = deque(maxlen=WINDOW_S)   # عدّات كل ثانية (نافذة منزلقة)
    ema = 0.0
    last_total = 0
    try:
        while True:
            time.sleep(1.0)
            now_total = total["count"]
            counts_last_sec = now_total - last_total
            last_total = now_total
            per_sec.append(counts_last_sec)

            # CPM من النافذة المنزلقة (تكبر تدريجياً حتى تمتلئ)
            window_counts = sum(per_sec)
            window_min = len(per_sec) / 60.0
            cpm_raw = window_counts / window_min if window_min > 0 else 0.0

            # المتوسط الأسّي السريع على المعدل اللحظي (counts/sec → CPM)
            ema += EMA_ALPHA * (counts_last_sec * 60.0 - ema)

            cpm = correct_dead_time(cpm_raw)     # المصحَّح يغذّي الجرعة
            usvh = cpm / CPM_PER_USVH
            warn = "  ⚠ HIGH_RATE_WARNING" if cpm_raw > HIGH_RATE_WARNING_CPM else ""

            print(f"CPM_خام={cpm_raw:8.1f} | CPM={cpm:8.1f} (مصحَّح) | "
                  f"µSv/h={usvh:7.3f} | سريع≈{ema:7.0f} | الإجمالي={now_total}{warn}")
    except KeyboardInterrupt:
        print("\nتوقّف.")
    finally:
        cb.cancel()
        pi.stop()


if __name__ == "__main__":
    main()
