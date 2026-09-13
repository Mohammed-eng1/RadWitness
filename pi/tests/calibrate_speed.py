# -*- coding: utf-8 -*-
"""
calibrate_speed.py — جدول الأرض: قوة → سرعة (م/ث)
===================================================
**أداة خارجية** (سكربت معايرة على العتاد — البند 8).

يسدّ فجوة: معايرة السرعة كانت **في الواجهة وحدها**
(`POST /api/calibration/speed`) فلا سبيل إليها من الطرفية، بينما
`DRIVE_SPEED_MPS` و`FREENOVE_SPEED_PER_POWER` موسومان «غير معايرين»
على هذه المنصّة (§2.0.2).

الطريقة: الروبوت يسير مدة معلومة عند قوة معلومة، وأنت تقيس المسافة
**بشريط قياس** ⇒ م/ث. يُحفظ في ملف المعايرة النشط.

🔴 **لماذا شريط قياس ولا شيء آخر**: لا إنكودرات على هذه المنصّة، والموضع
   تقدير مفتوح الحلقة — مقاس على العتاد أن شوطاً قطع **0.20م بينما
   النموذج يقول 1.8م** (خطأ 9 أضعاف بلا إنذار). فالمرجع الوحيد الصادق
   هنا مسطرة على الأرض.

⚠ **ببطارية مشحونة**: السرعة تتأثر بالجهد، والجدول المأخوذ ببطارية
  منخفضة أنتج مرة انحيازاً جانبياً فُسّر خطأً انعكاسَ ثوابت.
⚠ **مساحة خالية**: الروبوت يسير فعلاً بلا أي تفادٍ — هذا سكربت مباشر لا
  يمرّ بطبقة السلامة.
⚠ **أوقف السيرفر أولاً**: يمسك I2C ومصدر الاتجاه، ومنازعته تُفسد القياس.

الاستعمال:
    python3 -m pi.tests.calibrate_speed                      # 0.20,0.30,0.40
    python3 -m pi.tests.calibrate_speed --powers 0.3 --seconds 3
    python3 -m pi.tests.calibrate_speed --profile "ارضية_المختبر"
"""
from __future__ import annotations

import argparse
import sys
import time

from pi.config import MAX_MOTOR_POWER, MIN_MOTOR_POWER
from pi.nav.calibration import (CalibrationProfile, CalibrationStore,
                                compute_speed_mps)


def _ask_float(prompt: str):
    """يعيد `None` عند التخطّي — 🔴 لا صفراً (الصفر قراءة، والتخطّي غياب)."""
    while True:
        try:
            raw = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print("\n⛔ أُلغي."); raise SystemExit(1)
        if not raw:
            return None
        try:
            return float(raw.replace("،", "."))
        except ValueError:
            print("  ⚠ رقم فقط (أو Enter للتخطّي)")


def main() -> int:
    ap = argparse.ArgumentParser(description="جدول الأرض: قوة → سرعة م/ث")
    ap.add_argument("--powers", nargs="+", type=float,
                    default=[0.20, 0.30, 0.40],
                    help="مستويات القوة (افتراضي 0.20 0.30 0.40)")
    ap.add_argument("--seconds", type=float, default=3.0,
                    help="مدة كل شوط بالثواني (افتراضي 3.0)")
    ap.add_argument("--profile", default=None,
                    help="اسم ملف المعايرة (افتراضي: جديد بتاريخ اليوم)")
    ap.add_argument("--allow-sim", action="store_true",
                    help="⚠ متابعة رغم فشل فتح العتاد — الأرقام بلا معنى")
    a = ap.parse_args()

    from pi.rover.bridge import RoverControlBridge
    bridge = RoverControlBridge(mode="real")
    print(f"المنصّة: {bridge._tp.name}  ·  الوضع: {bridge.mode}")
    if bridge.mode != "real":
        print(f"⛔ لم يُفتح العتاد: {bridge.error}")
        if not a.allow_sim:
            # 🔴 لا سقوط صامت: جدول من المحاكاة يُكتب في ملف معايرة ثم
            #    تُحسب عليه المسافات في مهمة حقيقية (البند 6.1).
            return 2
        print("⚠ متابعة في المحاكاة — **الجدول الناتج بلا معنى**.")

    store = CalibrationStore()
    name = a.profile or f"freenove_{time.strftime('%Y-%m-%d')}"
    try:
        prof = store.load(name)
        print(f"ملف موجود: {name} (سرعات مسجّلة: {sorted(prof.speeds)})")
    except Exception:                            # noqa: BLE001
        prof = CalibrationProfile(name=name, note="جدول أرض Freenove")
        print(f"ملف جديد: {name}")

    v = bridge.voltage()
    print(f"جهد حزمة الراسبري: {v if v is not None else 'غير معروف'}"
          f"  ({bridge.voltage_source})")
    # 🔴 كانت تقول «حزمة المحركات بلا رقيب على هذه المنصّة» — **بطلت**
    #    بتوصيل ADS7830 ومعايرته (§2.0.1). ورسالةٌ تنفي وجود رقيب موجود
    #    تدفع المشغّل لتخمين الشحن بينما الرقم أمامه — وهي أسوأ من الصمت:
    #    المعايرة على حزمة شبه فارغة تُنتج جدولاً بطيئاً يُطبَّق على حزمة
    #    ممتلئة فتُحسب المسافات ناقصة بلا إنذار.
    mp = bridge.motor_pack_state()
    if mp.get("v") is None:
        print(f"⚠ جهد حزمة **المحركات** غير معروف ({mp.get('source')}) — "
              f"تأكّد يدوياً أنها مشحونة: python3 -m pi.tests.battery")
    else:
        print(f"جهد حزمة المحركات: {mp['v']:.2f}V "
              f"({mp['cell_v']}V/خلية · {mp['level']})")
        if mp.get("level") != "good":
            print("  🔴 **لا تعاير على حزمة ضعيفة**: الجدول سيخرج بطيئاً "
                  "ويُطبَّق لاحقاً على حزمة ممتلئة ⇒ مسافات محسوبة ناقصة.")
    print("\n⚠⚠ الروبوت سيسير فعلاً بلا أي تفادٍ. مساحة خالية أمامه، "
          "وأصبعك على الطاقة.")

    measured = 0
    try:
        for p in a.powers:
            pc = max(MIN_MOTOR_POWER, min(MAX_MOTOR_POWER, abs(p)))
            if pc != abs(p):
                print(f"\n⚠ القوة {p} قُصّت إلى {pc:.2f} "
                      f"(المدى {MIN_MOTOR_POWER}–{MAX_MOTOR_POWER})")
            try:
                input(f"\n  ▶ شوط عند قوة {pc:.2f} لمدة {a.seconds:.1f}ث — "
                      f"علّم نقطة البداية ثم Enter…")
            except (EOFError, KeyboardInterrupt):
                print("\n⛔ أُلغي."); break
            t0 = time.time()
            bridge.forward(pc)
            while time.time() - t0 < a.seconds:
                time.sleep(0.02)
            bridge.stop()
            real_s = time.time() - t0
            print(f"    توقّف بعد {real_s:.2f}ث فعلياً")
            d = _ask_float("    المسافة المقاسة بالشريط (متر، Enter لتخطّي): ")
            if d is None:
                print("    ⏭ تُخطّي — لا قيمة تُسجَّل (التخطّي ليس صفراً)")
                continue
            if d <= 0:
                print("    🔴 مسافة صفر أو سالبة: الروبوت لم يتحرّك فعلياً.")
                print("       افحص خريطة المحركات (probe_motor_map) والتغذية.")
                continue
            speed = compute_speed_mps(d, real_s)
            key = str(int(round(pc * 100)))
            prof.speeds[key] = round(speed, 3)
            measured += 1
            print(f"    ✅ {d:.2f}م ÷ {real_s:.2f}ث = **{speed:.3f} م/ث** "
                  f"(المفتاح {key})")
    finally:
        try:
            bridge.stop(); bridge.close()
        except Exception:                        # noqa: BLE001
            pass

    if not measured:
        print("\n⛔ لا قياس مسجَّل — الملف لم يُحفظ.")
        return 1

    prof.battery_v = v or prof.battery_v
    prof.date = time.strftime("%Y-%m-%d %H:%M")
    store.save(prof)
    print(f"\n✅ حُفظ «{name}» — {measured} قياساً.")
    print(f"   السرعات: {dict(sorted(prof.speeds.items(), key=lambda kv: int(kv[0])))}")

    # ── الاشتقاق المفيد لـconfig ───────────────────────────────
    pts = sorted((int(k) / 100.0, v2) for k, v2 in prof.speeds.items())
    if len(pts) >= 2:
        ratios = [sp / pw for pw, sp in pts if pw > 0]
        avg = sum(ratios) / len(ratios)
        print(f"\n  ── للـconfig ──")
        print(f"  FREENOVE_SPEED_PER_POWER = {avg:.2f}   "
              f"(م/ث لكل وحدة قوة · المدى {min(ratios):.2f}–{max(ratios):.2f})")
        # 🔴 **الدلالة لا السرعة الخام**: `RoverBridge` يحسب
        #    `speed = DRIVE_SPEED_MPS × (power/70)`، فالثابت السرعة عند
        #    القوة الاسمية **70** لا عند آخر قوة مقاسة. وطباعة السرعة
        #    الخام هنا كانت تدفع إلى قيمة أقلّ 43% من الصحيحة.
        nominal = pts[-1][1] / (pts[-1][0] / 0.70)
        print(f"  DRIVE_SPEED_MPS = {nominal:.2f}   "
              f"(السرعة عند القوة الاسمية 0.70 — مشتقّة من "
              f"{pts[-1][1]:.2f} م/ث عند {pts[-1][0]:.2f})")
        print(f"     ⚠ الثابت يخدم **محاكاة الخريطة** وحدها؛ مسافة المهمة "
              f"الحقيقية من ملف المعايرة المحفوظ.")
        spread = (max(ratios) - min(ratios)) / avg if avg else 0.0
        if spread > 0.25:
            print(f"  ⚠ التشتّت {spread*100:.0f}% — العلاقة **ليست خطية** في "
                  f"هذا المدى. لا تستقرئ خارج النقاط المقاسة.")
        print(f"  ⚠ وبعد تعديل السقف أو هذين، أعد تشغيل:")
        print(f"      python3 -m pi.tests.check_config_hygiene")
    else:
        print("\n  ⚠ نقطة واحدة لا تكفي لاشتقاق معامل — قِس مستويين على الأقل.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
