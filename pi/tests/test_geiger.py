#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_geiger.py — اختبار منفرد لعداد جيجر CAJOE (J305) على الراسبري
==================================================================
يعدّ النبضات عبر lgpio (تنبيه على الحافة الصاعدة، دخل عائم بلا مقاومة
رفع/سحب — أي مقاومة تقتل نبضة VIN الضعيفة عالية الممانعة، مؤكد على
العتاد في المشروع السابق)، ثم يحسب CPM على نافذة 30 ثانية منزلقة +
متوسط أسّي سريع، ويحوّلها إلى µSv/h بمعايرة مخبرية مثبتة ضد Cs-137.

لماذا lgpio وليس pigpio؟ الـBRIEF ذكر pigpio، لكنه محذوف من مستودعات
Debian trixie (راسبري باي أوس الحالي). lgpio هو البديل الرسمي الحديث:
يوفّر نفس تنبيه الحافة، ولا يحتاج daemon (يصل إلى gpiochip مباشرة).

⚠ العتاد: لوحة الجيجر تُغذّى 3.3V حصراً (وليس 5V) فتبقى ذروة النبضة
   آمنة على دبوس GPIO الراسبري (يتحمل 3.3V فقط). النبضة على GPIO17
   (BCM17 = الدبوس الفيزيائي 11). GND مشترك إلزامي.

المعايرة (ثوابت مثبتة مخبرياً — لا تُغيَّر):
    K = 111 CPM/(µSv/h)   مقاسة ضد مصدر Cs-137 مرجعي (662 keV)، 2026-07
    τ = 200 µs            زمن ميت مقاس من نفس جلسة المعايرة
    المصدر: legacy/RadiationRover/firmware/esp32_main/{config.h, geiger.h}

التشغيل على الراسبري (بلا daemon — المستخدم pi ضمن مجموعة gpio):
    python3 pi/tests/test_geiger.py   # Ctrl-C للإيقاف
"""
import sys
import time
from collections import deque

try:
    import lgpio
except ImportError:
    sys.exit("خطأ: مكتبة lgpio غير مثبّتة. ثبّتها عبر setup_pi.sh (apt: python3-lgpio).")

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


def open_geiger():
    """يفتح خط الجيجر ويطالب بتنبيه الحافة الصاعدة. يجرّب gpiochip0 (باي 4)
    ثم gpiochip4 (باي 5). يُعيد (handle, chip). دخل عائم — بلا pull."""
    last_err = None
    for chip in (0, 4):
        try:
            h = lgpio.gpiochip_open(chip)
        except Exception as e:               # noqa: BLE001 — الشريحة غير موجودة
            last_err = e
            continue
        try:
            lgpio.gpio_claim_alert(h, GEIGER_GPIO, lgpio.RISING_EDGE, lgpio.SET_PULL_NONE)
            return h, chip
        except Exception as e:               # noqa: BLE001 — الخط مشغول/غير صالح
            last_err = e
            lgpio.gpiochip_close(h)
    sys.exit(f"خطأ: تعذّر فتح خط الجيجر GPIO{GEIGER_GPIO} ({last_err}). "
             f"تأكد من التوصيل ومن أن المستخدم ضمن مجموعة gpio.")


def main() -> None:
    handle, chip = open_geiger()
    # callback بلا دالة = يعدّ تلقائياً في خيط القراءة (نقرأ العدّ عبر tally)
    cb = lgpio.callback(handle, GEIGER_GPIO, lgpio.RISING_EDGE)

    print(f"يعدّ نبضات الجيجر على GPIO{GEIGER_GPIO} (gpiochip{chip}) — نافذة {WINDOW_S}s. "
          f"Ctrl-C للإيقاف.\n")
    per_sec = deque(maxlen=WINDOW_S)   # عدّات كل ثانية (نافذة منزلقة)
    ema = 0.0
    last_total = 0
    try:
        while True:
            time.sleep(1.0)
            now_total = cb.tally()               # إجمالي الحواف منذ البدء
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
        lgpio.gpiochip_close(handle)


if __name__ == "__main__":
    main()
