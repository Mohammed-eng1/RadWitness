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
import signal
import sys
import time

from pi.config import (ROVER_KIND, MAX_MOTOR_POWER, MIN_MOTOR_POWER,
                       FREENOVE_WHEEL_CHANNELS, FREENOVE_WHEEL_NAMES)

#: أسماء العجلات **للطباعة بالإنجليزية** — `FREENOVE_WHEEL_NAMES` العربية
#: تبقى للواجهة والسجل، وهذه لشاشة المسبار التي يقرأها المشغّل أثناء العمل.
WHEEL_EN = {
    ("left", 0):  "LEFT FRONT wheel",
    ("left", 1):  "LEFT BACK wheel",
    ("right", 0): "RIGHT FRONT wheel",
    ("right", 1): "RIGHT BACK wheel",
}

PULSE_S = 2.0
CMD_GAP_S = 0.1


def _safe_shutdown(bridge) -> None:
    """
    🔴 **إيقاف لا يُقاطَع** (عطل مقاس 2026-09-12).

    Ctrl+C يرفع `KeyboardInterrupt` وهو من **`BaseException`** لا
    `Exception` — فـ`except Exception` حول التنظيف لا يمسّه. وضغطة ثانية
    تصل **وسط `stop()`** فتُجهضه، فتبقى قناة PWM مكتوبة و**العجلة تدور
    بلا توقف**. ولا تُصلحها إعادة إقلاع الراسبري: PCA9685 شريحة مستقلة
    لها سجلاتها وتغذيتها، فتحتفظ بآخر قيمة كُتبت فيها.

    العلاج: **تُصمّ المقاطعات أولاً** ثم يُنفَّذ الإيقاف، ويُلتقط
    `BaseException` لا `Exception`.
    """
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, signal.SIG_IGN)
        except Exception:                     # noqa: BLE001
            pass
    for attempt in (1, 2):                    # محاولتان — والثانية أبسط
        try:
            if attempt == 1:
                bridge.stop()
            bridge.close()
            return
        except BaseException:                 # noqa: BLE001 — مقصود
            continue


def _ask(question: str, options: tuple) -> str:
    """سؤال بحرف واحد — يعيد السؤال حتى يأتي حرف من القائمة."""
    while True:
        print(question)
        try:
            v = input(f"[{'/'.join(options)}] ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n⛔ Cancelled.")
            raise SystemExit(1)
        if v in options:
            return v
        print(f"  Please type one of: {' / '.join(options)}")


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
    try:
        while time.time() - t0 < seconds:
            bridge._tp.send_motors(lc, rc)
            time.sleep(CMD_GAP_S)
    finally:
        # 🔴 الإيقاف قبل خروج أي استثناء (Ctrl+C وسط النبضة)
        for _ in range(2):
            try:
                bridge._tp.send_motors(0.0, 0.0)
            except BaseException:             # noqa: BLE001
                pass


def probe_wheels(bridge, power: float, seconds: float) -> int:
    """
    🔴 **الفحص الأول دائماً**: عجلة واحدة لكل نبضة.

    مسبار المعالم ينبض **جانباً كاملاً**، فإن تعاكست عجلتا الجانب شلّ
    الجانبُ نفسَه وبدت النتيجة «عشوائية يصعب تحديد أيّها تحرّك» — وهو ما
    حدث فعلاً على العتاد (2026-09-12) بسبب خطأ ترتيب قنوات السفلية
    اليسرى. **جانب يتنازع لا تُشتقّ منه خريطة**، فهذا الطور يسبقه.
    """
    print(f"\n🔍 WHEEL CHECK — one wheel at a time (power {power:.2f})")
    print("⚠ LIFT THE ROBOT UP. Wheels must spin freely.")
    print("   Watch ONLY the wheel I name before each pulse.")
    bad = []
    for side in ("left", "right"):
        for i, (ch_a, ch_b) in enumerate(FREENOVE_WHEEL_CHANNELS[side]):
            name = FREENOVE_WHEEL_NAMES[side][i]
            try:
                input(f"\n  ▶ {WHEEL_EN[(side, i)]}  "
                      f"(channels {ch_a}/{ch_b}) — press Enter…")
            except (EOFError, KeyboardInterrupt):
                print("\n⛔ Cancelled."); return 1
            t0 = time.time()
            try:
                while time.time() - t0 < seconds:
                    bridge._tp.drive_one_wheel(side, i, power)
                    time.sleep(CMD_GAP_S)
            finally:
                # 🔴 إطفاء هذه العجلة **قبل** أن يخرج أي استثناء —
                #    بما فيه Ctrl+C وسط النبضة.
                try:
                    bridge._tp.drive_one_wheel(side, i, 0.0)
                except BaseException:         # noqa: BLE001
                    pass
            ans = _ask("     What happened?\n"
                       "       o = ONLY this wheel turned  (good)\n"
                       "       x = a DIFFERENT wheel turned\n"
                       "       m = MORE than one wheel turned\n"
                       "       n = NOTHING turned",
                       ("o", "x", "m", "n"))
            if ans != "o":
                bad.append((name, ans))
    print("\n  ── RESULT ──")
    if not bad:
        print("  ✅ Every wheel responds to its own two channels. Map is OK.")
        print("     Next step: run the probe WITHOUT --wheels.")
        return 0
    why = {"x": "a different wheel moved  ⇒ channels are swapped",
           "m": "more than one wheel moved ⇒ overlapping channels / power",
           "n": "nothing moved ⇒ channel not driven, or dead motor"}
    for name, ans in bad:
        print(f"  🔴 {name}: {why[ans]}")
    print("\n  🔴 STOP. Do not run the landmark probe yet.")
    print("     The channel map is wrong, and anything measured on top of")
    print("     a wrong map measures the fault, not the direction.")
    print("     Fix FREENOVE_WHEEL_CHANNELS in pi/config.py first.")
    print("  ⚠ If two wheels on the SAME side spin in OPPOSITE directions,")
    print("     the pair order is wrong. A pair (a, b) means \"positive")
    print("     power drives b\". Swapping it flips that one wheel only.")
    return 3


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Motor map probe: find MOTOR_SWAP_LR and MOTOR_INVERT")
    ap.add_argument("--kind", default=ROVER_KIND,
                    help="platform name (freenove)")
    ap.add_argument("--power", type=float, default=0.3,
                    help=f"pulse power ({MIN_MOTOR_POWER}-{MAX_MOTOR_POWER})")
    ap.add_argument("--seconds", type=float, default=PULSE_S)
    ap.add_argument("--wheels", action="store_true",
                    help="check each wheel alone FIRST - catches two wheels "
                         "on the same side spinning opposite ways")
    ap.add_argument("--allow-sim", action="store_true",
                    help="continue even if hardware did not open (text test only)")
    a = ap.parse_args()

    from pi.rover.bridge import RoverControlBridge
    bridge = RoverControlBridge(mode="real", kind=a.kind)
    print(f"Platform: {bridge._tp.name}  |  Mode: {bridge.mode}")
    if bridge.mode != "real":
        print(f"⛔ Hardware did not open: {bridge.error}")
        if not a.allow_sim:
            # 🔴 لا سقوط صامت إلى المحاكاة: مسبار في المحاكاة يُخرج وصفة
            #    مُختلَقة تُكتب في config وتقود محركات حقيقية (البند 6.1).
            return 2
        print("⚠ Running in SIMULATION - the result is meaningless.")

    p = max(MIN_MOTOR_POWER, min(MAX_MOTOR_POWER, abs(a.power)))

    if a.wheels:
        try:
            return probe_wheels(bridge, p, a.seconds)
        finally:
            _safe_shutdown(bridge)

    print(f"\n🧭 LANDMARK PROBE — power {p:.2f}, {a.seconds:.1f}s per pulse.")
    print("⚠ LIFT THE ROBOT UP, wheels free.")
    print("   (Measuring on the ground once gave a wrong answer: a weak")
    print("    battery made one side drag.)")
    print("  Landmark: the GEIGER TUBE on one side of the robot.")
    print("  🔴 We never say left/right — those flip depending on where")
    print("     YOU stand. We only say \"geiger side\" or \"other side\".")

    results = {}
    try:
        for field, lp, rp in (("L", p, 0.0), ("R", 0.0, p)):
            try:
                input(f"\n  ▶ Pulse field {field} only (+{p:.2f}) — "
                      f"press Enter…")
            except (EOFError, KeyboardInterrupt):
                print("\n⛔ Cancelled."); return 1
            _pulse(bridge, lp, rp, a.seconds)
            side = _ask("  Which side's wheels turned?\n"
                        "       g = the GEIGER TUBE side\n"
                        "       o = the OTHER side\n"
                        "       n = nothing turned", ("g", "o", "n"))
            direction = "n"
            if side != "n":
                direction = _ask("  Which way did they roll?\n"
                                 "       f = toward the FRONT (camera side)\n"
                                 "       b = toward the BACK", ("f", "b"))
            results[field] = (side, direction)
    finally:
        # 🔴 الإيقاف مضمون مهما حدث — بما فيه **Ctrl+C متكرر وسط نبضة**
        _safe_shutdown(bridge)

    en_side = {"g": "geiger side", "o": "other side", "n": "nothing"}
    en_dir = {"f": "forward", "b": "backward", "n": "—"}
    print("\n  ── WHAT YOU SAW ──")
    for field in ("L", "R"):
        sd, dr = results[field]
        print(f"    field {field}=+{p:.2f}  ⇒  {en_side[sd]}, {en_dir[dr]}")

    ls, ld = results["L"]
    rs, rd = results["R"]
    print("\n  ── RESULT ──")
    if "n" in (ls, rs):
        print("  🔴 One field did nothing — dead side or undriven channel.")
        print("     Check FREENOVE_WHEEL_CHANNELS in pi/config.py and the")
        print("     power going to the Freenove motor board.")
        print("     No direction constant can fix this.")
        return 3
    if ls == rs:
        print("  🔴 Both fields moved the SAME side.")
        print("     This is a channel-map error, not a polarity error.")
        print("     No direction constant can fix this.")
        return 3
    if ld != rd:
        print("  🔴 The two sides have OPPOSITE polarity — one side's motor")
        print("     is wired backwards.")
        print("     Telltale sign: a \"go forward\" command makes the robot")
        print("     SPIN IN PLACE instead of moving forward.")
        print("  🔴 No config value can fix this — fix the wiring.")
        return 3

    # الجيجر على **يسار** الروبوت (تثبيت هذا المشروع)
    left_is_L = (ls == "g")
    swap = not left_is_L               # True ⇒ الحقل L يقود الجانب الأيمن
    invert = -1 if ld == "b" else +1   # ‎+p يجب أن يدفع العجلة للأمام

    print(f"  ✅ DONE. Put these two lines in pi/config.py:")
    print(f"     MOTOR_SWAP_LR = {swap}")
    print(f"     MOTOR_INVERT  = {invert}")
    print(f"\n  Next: check it with a 90 degree turn on the ground:")
    print(f"     python3 -m pi.tests.check_directions --power 0.25")
    print(f"  ⚠ Use 90 degrees, NOT 180. After a 180 turn the robot faces")
    print(f"     the same way whether it turned left or right, so a 180")
    print(f"     turn cannot tell you if the direction is flipped.")
    print(f"  ⚠ Stand BEHIND the robot when you judge left/right.")
    print(f"\n  🔴 If the turn goes the WRONG way, do NOT change")
    print(f"     MPU6050_GYRO_Z_SIGN. That value was measured by turning")
    print(f"     the robot BY HAND with motors off, so it does not depend")
    print(f"     on motor wiring. Changing it hides the real bug.")
    print(f"     Run this probe again instead.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
