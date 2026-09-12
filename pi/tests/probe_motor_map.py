# -*- coding: utf-8 -*-
"""
probe_motor_map.py — مسبار المعالم الفيزيائية: اشتقاق خريطة المحركات
=====================================================================
**أداة خارجية** (سكربت تشخيص — البند 8).

يشتقّ `MOTOR_SWAP_LR` و`MOTOR_INVERT` بالمشاهدة، **مستقلاً عن المنصّة**:
يمرّ بناقل الجسر (`transport.py`) لا بمنفذ خام، فيصلح لمنصّة بلا UART
(Freenove يُقاد I2C مباشرة).

🔴 **لماذا مسبار لا شهادة عين** (سيرة ثابت انقلب مرتين في يومين):
  - **حقل واحد لكل نبضة** (L وحده ثم R وحده) — نبضة مركّبة تخلط الأثرين.
  - **الجهة تُسمّى بمعلم فيزيائي** (أنبوب الجيجر / الجهة المقابلة) لا
    يمين/يسار: «يمين» تنعكس بموقع الواقف، والنظر من أمام الروبوت يقلبها.
  - **الروبوت مرفوع والعجلات حرة**: اختلال جانبين تحت الحمل ببطارية
    منخفضة أنتج مرة انحرافاً فُسّر خطأً انعكاسَ ثوابت (جدول الأرض انحاز
    يساراً حتى في الأوامر المتماثلة، واختفى الانحياز والعجلات مرفوعة).
  - **لا 180°**: تنتهي مواجهة نفس الجهة يميناً ويساراً فلا تكشف الانعكاس.

⚠ **لا يقيس إشارة الجايرو ولا يلمسها**: `MPU6050_GYRO_Z_SIGN` تُقاس
  **باليد بلا محركات** فهي مستقلة عن قطبية المحركات ولا تتغيّر معها. قياسها
  بالمحركات يعطي **حاصل ضرب الخطأين** فتبدو سليمة وهي معكوسة — وهو الخطأ
  الذي عاش شهراً (البند 2). استعمل `calibrate_mpu6050` لتلك.

الاستعمال (على العتاد، الروبوت **مرفوع**):
    python3 -m pi.tests.probe_motor_map
    python3 -m pi.tests.probe_motor_map --kind freenove --power 0.3
"""
from __future__ import annotations

import argparse
import sys
import time

from pi.config import (ROVER_KIND, MAX_MOTOR_POWER, MIN_MOTOR_POWER,
                       FREENOVE_WHEEL_CHANNELS, FREENOVE_WHEEL_NAMES)

PULSE_S = 2.0
CMD_GAP_S = 0.1


def _ask(question: str, options: tuple) -> str:
    """سؤال بحرف واحد — يعيد السؤال حتى يأتي حرف من القائمة."""
    while True:
        print(question)
        try:
            v = input(f"[{'/'.join(options)}] ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n⛔ أُلغي.")
            raise SystemExit(1)
        if v in options:
            return v
        print(f"  حرف من: {' · '.join(options)}")


def _pulse(bridge, lp: float, rp: float, seconds: float) -> None:
    """
    نبضة على **إطار الأسلاك مباشرة** — تتجاوز `motors()` عمداً.

    🔴 السبب: `motors()` يطبّق `MOTOR_SWAP_LR` ثم `MOTOR_INVERT`، وهما ما
       نحاول اشتقاقه. المرور بهما يجعل المسبار يقيس نفسه فيؤكّد أي قيمة
       موجودة مهما كانت خاطئة. القصّ عند `MAX_MOTOR_POWER` مطبَّق يدوياً
       هنا لأننا تجاوزنا الحاجز الذي يطبّقه `motors()`.
    """
    lc = max(-MAX_MOTOR_POWER, min(MAX_MOTOR_POWER, lp))
    rc = max(-MAX_MOTOR_POWER, min(MAX_MOTOR_POWER, rp))
    t0 = time.time()
    while time.time() - t0 < seconds:
        bridge._tp.send_motors(lc, rc)
        time.sleep(CMD_GAP_S)
    bridge._tp.send_motors(0.0, 0.0)
    bridge._tp.send_motors(0.0, 0.0)      # تأكيد الإيقاف


def probe_wheels(bridge, power: float, seconds: float) -> int:
    """
    🔴 **الفحص الأول دائماً**: عجلة واحدة لكل نبضة.

    مسبار المعالم ينبض **جانباً كاملاً**، فإن تعاكست عجلتا الجانب شلّ
    الجانبُ نفسَه وبدت النتيجة «عشوائية يصعب تحديد أيّها تحرّك» — وهو ما
    حدث فعلاً على العتاد (2026-09-12) بسبب خطأ ترتيب قنوات السفلية
    اليسرى. **جانب يتنازع لا تُشتقّ منه خريطة**، فهذا الطور يسبقه.
    """
    print(f"\n🔍 فحص العجلات واحدةً واحدة — قوة {power:.2f}")
    print("⚠ الروبوت **مرفوع** والعجلات حرة. راقب **العجلة المسمّاة وحدها**.")
    bad = []
    for side in ("left", "right"):
        for i, (ch_a, ch_b) in enumerate(FREENOVE_WHEEL_CHANNELS[side]):
            name = FREENOVE_WHEEL_NAMES[side][i]
            try:
                input(f"\n  ▶ {name}  (قناتا {ch_a}/{ch_b}) — Enter…")
            except (EOFError, KeyboardInterrupt):
                print("\n⛔ أُلغي."); return 1
            t0 = time.time()
            while time.time() - t0 < seconds:
                bridge._tp.drive_one_wheel(side, i, power)
                time.sleep(CMD_GAP_S)
            bridge._tp.drive_one_wheel(side, i, 0.0)
            ans = _ask("     ماذا حدث؟  o = **هذه العجلة وحدها** دارت · "
                       "x = عجلة أخرى دارت · m = أكثر من واحدة · "
                       "n = لا شيء", ("o", "x", "m", "n"))
            if ans != "o":
                bad.append((name, ans))
    print("\n  ── النتيجة ──")
    if not bad:
        print("  ✅ كل عجلة تستجيب لقناتيها وحدها — الخريطة سليمة.")
        print("     تابِع إلى طور المعالم (بلا --wheels).")
        return 0
    ar = {"x": "دارت عجلة أخرى ⇒ **تبديل قنوات**",
          "m": "دارت أكثر من واحدة ⇒ قنوات متداخلة/تغذية",
          "n": "لا دوران ⇒ قناة غير مقودة أو محرّك ميت"}
    for name, ans in bad:
        print(f"  🔴 {name}: {ar[ans]}")
    print("\n  🔴 لا تُكمل إلى طور المعالم: خريطة القنوات خاطئة، وأي")
    print("     اشتقاق فوقها يقيس العطل لا الاتجاه. صحّح")
    print("     FREENOVE_WHEEL_CHANNELS في pi/config.py أولاً.")
    print("  ⚠ وإن دارت عجلتا جانب **عكس بعضهما** فالسبب ترتيب الزوج:")
    print("     الزوج (a,b) يعني «موجب يقود b»، وقلبه يقلب العجلة وحدها.")
    return 3


def main() -> int:
    ap = argparse.ArgumentParser(
        description="مسبار المعالم: اشتقاق MOTOR_SWAP_LR و MOTOR_INVERT")
    ap.add_argument("--kind", default=ROVER_KIND,
                    help="اسم المنصّة (freenove)")
    ap.add_argument("--power", type=float, default=0.3,
                    help=f"قوة النبضة ({MIN_MOTOR_POWER}–{MAX_MOTOR_POWER})")
    ap.add_argument("--seconds", type=float, default=PULSE_S)
    ap.add_argument("--wheels", action="store_true",
                    help="🔍 فحص كل عجلة وحدها أولاً — يكشف تعاكس عجلتَي "
                         "الجانب الواحد الذي يُفسد طور المعالم")
    ap.add_argument("--allow-sim", action="store_true",
                    help="متابعة رغم فشل فتح العتاد (لتجربة نص المسبار)")
    a = ap.parse_args()

    from pi.rover.bridge import RoverControlBridge
    bridge = RoverControlBridge(mode="real", kind=a.kind)
    print(f"الهيكل: {bridge._tp.name}  ·  الوضع: {bridge.mode}")
    if bridge.mode != "real":
        print(f"⛔ لم يُفتح العتاد: {bridge.error}")
        if not a.allow_sim:
            # 🔴 لا سقوط صامت إلى المحاكاة: مسبار في المحاكاة يُخرج وصفة
            #    مُختلَقة تُكتب في config وتقود محركات حقيقية (البند 6.1).
            return 2
        print("⚠ متابعة في المحاكاة — **الوصفة الناتجة بلا معنى**.")

    p = max(MIN_MOTOR_POWER, min(MAX_MOTOR_POWER, abs(a.power)))

    if a.wheels:
        try:
            return probe_wheels(bridge, p, a.seconds)
        finally:
            try:
                bridge.stop(); bridge.close()
            except Exception:                 # noqa: BLE001
                pass

    print(f"\n🧭 مسبار المعالم — قوة {p:.2f} لمدة {a.seconds:.1f}ث لكل نبضة.")
    print("⚠ الروبوت **مرفوع** والعجلات حرة (اختلال الجانبين تحت الحمل "
          "أفسد قياساً سابقاً).")
    print("  المعلم: **أنبوب الجيجر** على أحد جانبي الروبوت — "
          "لا نستعمل يمين/يسار أبداً.")

    results = {}
    try:
        for field, lp, rp in (("L", p, 0.0), ("R", 0.0, p)):
            try:
                input(f"\n  ▶ نبضة الحقل {field} وحده (+{p:.2f}) — Enter…")
            except (EOFError, KeyboardInterrupt):
                print("\n⛔ أُلغي."); return 1
            _pulse(bridge, lp, rp, a.seconds)
            side = _ask("  أي جهة دارت عجلاتها؟  g = جهة أنبوب الجيجر · "
                        "o = الجهة المقابلة · n = لا شيء دار", ("g", "o", "n"))
            direction = "n"
            if side != "n":
                direction = _ask("  ودارت نحو:  f = مقدمة الروبوت "
                                 "(جهة الكاميرا) · b = مؤخرته", ("f", "b"))
            results[field] = (side, direction)
    finally:
        # الإيقاف مضمون مهما حدث — بما فيه Ctrl-C وسط نبضة
        try:
            bridge.stop(); bridge.close()
        except Exception:                     # noqa: BLE001
            pass

    ar_side = {"g": "جهة الجيجر", "o": "الجهة المقابلة", "n": "لا دوران"}
    ar_dir = {"f": "أمام", "b": "خلف", "n": "—"}
    print("\n  ── المرصود ──")
    for field in ("L", "R"):
        sd, dr = results[field]
        print(f"    الحقل {field}=+{p:.2f} ⇒ {ar_side[sd]} · {ar_dir[dr]}")

    ls, ld = results["L"]
    rs, rd = results["R"]
    print("\n  ── الاشتقاق ──")
    if "n" in (ls, rs):
        print("  🔴 حقل بلا استجابة — جانب ميت أو قناة غير مقودة.")
        print("     على Freenove: راجع خريطة القنوات FREENOVE_WHEEL_CHANNELS "
              "وتغذية لوح المحركات. لا يصلحه أي ثابت اتجاه.")
        return 3
    if ls == rs:
        print("  🔴 الحقلان يحرّكان الجهة نفسها — قناتا الخرج على جانب واحد.")
        print("     خطأ خريطة قنوات لا خطأ قطبية. لا يصلحه أي ثابت اتجاه.")
        return 3
    if ld != rd:
        print("  🔴 قطبية الجانبين مختلفة — انعكاس محرّك جانب واحد (أسلاك).")
        print("     العرَض المميّز: أمر التقدّم يجعله **يدور بالمكان** بدل أن "
              "يتقدّم. 🔴 لا يصلحه أي ثابت في config — أصلح الأسلاك.")
        return 3

    # الجيجر على **يسار** الروبوت (تثبيت هذا المشروع)
    left_is_L = (ls == "g")
    swap = not left_is_L               # True ⇒ الحقل L يقود الجانب الأيمن
    invert = -1 if ld == "b" else +1   # ‎+p يجب أن يدفع العجلة للأمام

    print(f"  ✅ الوصفة في pi/config.py:")
    print(f"     MOTOR_SWAP_LR = {swap}")
    print(f"     MOTOR_INVERT  = {invert}")
    print(f"\n  ⚠ ثم **تحقّق** بلفّة 90° (لا 180°: تنتهي مواجهة نفس الجهة في")
    print(f"     الاتجاهين فلا تكشف الانعكاس)، وشاهدها **من خلف الروبوت**.")
    print(f"  ⚠ ولا تلمس MPU6050_GYRO_Z_SIGN — تُقاس باليد بلا محركات:")
    print(f"     python3 -m pi.tests.calibrate_mpu6050")
    return 0


if __name__ == "__main__":
    sys.exit(main())
