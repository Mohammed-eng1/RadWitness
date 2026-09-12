#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_directions.py — تثبيت اتجاهات العتاد الثلاثة (قبل أي معايرة)
==================================================================
يحدّد بالتجربة قيمتَي `MOTOR_INVERT` و`MPU6050_GYRO_Z_SIGN`، ويكشف
**انعكاس محرك واحد** (خطأ أسلاك) الذي لا يظهر في أي اختبار زاوية.

⚠ لماذا سكربت مستقل؟ لأن **لفّة 180° لا تفرّق بين اليمين واليسار** — الروبوت
   ينتهي مواجهاً نفس الجهة في الحالتين. فمعايرة المعامل بلفّات 180° قد تنجح
   تماماً بينما المحركات **والجايرو معكوسان معاً**، إذ يُلغي الخطآن بعضهما في
   حارس الإشارة. الأعراض تظهر لاحقاً كسير للخلف وخريطة معكوسة.

ترتيب المراحل **مقصود**: الجايرو أولاً **بلا محركات** (تدوير باليد)، فلا
يستطيع خطأ المحركات أن يلوّث قياس إشارته. ثم تُقاس المحركات وقد صار لدينا
مرجع اتجاه موثوق.

  أ) إشارة الجايرو  — تدوير **باليد**، بلا محركات إطلاقاً (آمن تماماً).
  ب) اتجاه التقدّم  — نبضة قصيرة للأمام بقوة منخفضة.
  ج) اتجاه الدوران  — نبضة دوران قصيرة، وتقاطعها مع (ب) يكشف خطأ الأسلاك.

    python3 -m pi.tests.check_directions
    RMS_ROVER_MODE=real python3 -m pi.tests.check_directions --power 0.3

⚠ السلامة: نبضات قصيرة بقوة منخفضة، الأمر يُجدَّد كل دورة (حارس heartbeat
   1.5ث)، وإيقاف مضمون في `finally` وعند Ctrl-C. ارفع الروبوت على حامل أو
   أبقِ مساحة ≥1م أمامه وخلفه.
"""
from __future__ import annotations

import argparse
import sys
import time

from pi.config import (
    ROVER_MODE, MOTOR_INVERT, MOTOR_SWAP_LR, MPU6050_GYRO_Z_SIGN,
    BNO055_READ_PERIOD_S,
)
from pi.rover.bridge import WaveRoverBridge

PULSE_S = 1.2          # مدة النبضة — تكفي للرؤية ولا تقطع مسافة خطرة
PULSE_POWER = 0.25     # قوة منخفضة (داخل النطاق الآمن، وأبطأ من قوة المهمة)
HAND_TURN_S = 6.0      # مهلة تدوير اليد


# ⚠ النص العربي يُطبع بـ`print` **قبل** `input()` ولا يُمرَّر محثّاً له:
# خلط RTL/LTR داخل محثّ input() يربك readline في حساب موضع المؤشّر فيحقن
# بايتات في مخزن الدخل → `UnicodeDecodeError: byte 0xd8` يُسقط الجلسة **بعد**
# أن يكون المستخدم قد أدّى قياساً يدوياً لا يمكن استرجاعه. ولنفس السبب
# يُلتقط خطأ الترميز ويُعاد السؤال بدل أن ينهار السكربت.
def _input(ascii_prompt: str = "> ") -> str:
    """قراءة سطر بمحثّ ASCII فقط. يرفع KeyboardInterrupt عند EOF/Ctrl-C."""
    try:
        return input(ascii_prompt)
    except UnicodeDecodeError:
        return "\x00"                 # إشارة «إدخال تالف» — يعالجها المنادي
    except (EOFError, KeyboardInterrupt):
        raise KeyboardInterrupt


def _pause(msg: str) -> None:
    """توقّف حتى Enter — الرسالة العربية بـprint لا داخل input()."""
    print(msg)
    _input("[Enter] ")


def _ask(prompt: str, options: dict):
    """يسأل ويقبل أحد المفاتيح؛ يعيد السؤال عند إدخال غير معروف أو تالف."""
    keys = "/".join(options)
    while True:
        print(prompt)
        raw = _input(f"[{keys}] ").strip().lower()
        if raw == "\x00":
            print("  ⚠ تعذّرت قراءة الإدخال (ترميز) — أعد الكتابة.")
            continue
        if raw in options:
            return options[raw]
        print(f"  ⚠ أدخل أحد: {keys}")


def _pulse(rover, l: float, r: float, seconds: float = PULSE_S) -> None:
    """
    نبضة حركة قصيرة. ⚠ الأمر **يُجدَّد** كل 100ms: بلا تجديد يمرّ 1.5ث
    فيعتبره حارس heartbeat انقطاعاً ويوقف المحركات في منتصف النبضة.
    """
    deadline = time.time() + seconds
    try:
        while time.time() < deadline:
            rover.motors(l, r)
            time.sleep(0.1)
    finally:
        rover.stop()


# ═══ أ) إشارة الجايرو — باليد، بلا محركات ════════════════════════
def stage_gyro_sign(rover) -> dict:
    """
    يقيس إشارة gz أثناء تدوير الروبوت **باليد** يميناً (عقارب الساعة منظوراً
    من فوق). فصل المحركات هنا **جوهري**: لو دُوّر بالمحركات وكانت معكوسة
    لقِسنا حاصل ضرب الخطأين وخرجنا بإشارة تبدو سليمة.
    """
    src = rover.heading_source
    imu = getattr(src, "imu", None)
    print("\n=== أ) إشارة الجايرو (بلا محركات — آمن) ===")
    if imu is None:
        print("  ⚠ المصدر الحالي لا يعتمد BNO055 — تُتخطّى.")
        return {"ok": False, "reason": "لا BNO055"}

    print("  ضع الروبوت على الطاولة. عند البدء **لُفّه بيدك نحو اليمين**")
    print("  (عقارب الساعة منظوراً من فوق) ~90° خلال المهلة.")
    _pause("  اضغط Enter ثم ابدأ التدوير فوراً…")

    total_raw = 0.0
    peak = 0.0
    samples = 0
    t0 = time.time()
    last = t0
    while time.time() - t0 < HAND_TURN_S:
        v = imu.gyro_z_dps()          # ⚠ **خام** بلا إشارة config
        now = time.time()
        dt = now - last
        last = now
        if v is not None:
            total_raw += v * dt
            peak = max(peak, abs(v))
            samples += 1
        time.sleep(BNO055_READ_PERIOD_S)

    print(f"  تكامل خام: {total_raw:+.1f}°  (ذروة {peak:.1f}°/ث، {samples} عينة)")
    if peak < 5.0:
        print("  ⚠ لم أرَ دوراناً يُذكر — أعد المحاولة ولُفّه بوضوح.")
        return {"ok": False, "reason": "لا دوران مقروء", "total_raw": total_raw}

    # اصطلاح المشروع: **موجب = يميناً**. الإشارة الصحيحة تجعل لفّة اليد
    # اليمنى موجبة بعد ضربها في الثابت.
    needed = 1 if total_raw > 0 else -1
    ok = (needed == MPU6050_GYRO_Z_SIGN)
    print(f"  → الإشارة الصحيحة: MPU6050_GYRO_Z_SIGN = **{needed:+d}**  "
          f"(الحالية {MPU6050_GYRO_Z_SIGN:+d})" + ("  ✅" if ok else "  ⚠ تحتاج تغيير"))
    return {"ok": True, "needed_sign": needed, "current_sign": MPU6050_GYRO_Z_SIGN,
            "matches": ok, "total_raw_deg": round(total_raw, 1),
            "peak_dps": round(peak, 1)}


# ═══ ب) اتجاه التقدّم ════════════════════════════════════════════
def stage_forward(rover, power: float) -> dict:
    """
    `forward()` تُرسل motors(+p, +p) فتصير على السلك (MOTOR_INVERT·p).
    إن تحرّك الروبوت للخلف فالثابت مقلوب — وهو **نفس المسار الذي تستخدمه
    المهمة كاملة**، لا سكربت المعايرة وحده.
    """
    print("\n=== ب) اتجاه التقدّم ===")
    print(f"  ⚠ سيتحرك {PULSE_S:.1f}ث بقوة {power}. أخلِ ≥1م أمامه وخلفه.")
    _pause("  اضغط Enter للنبضة…")

    _pulse(rover, power, power)       # نيّة: للأمام

    moved = _ask("  إلى أين تحرّك فعلياً؟ (f=للأمام, b=للخلف, n=لم يتحرك)",
                 {"f": "forward", "b": "backward", "n": "none"})
    if moved == "none":
        print("  ⚠ لم يتحرك — بطارية ضعيفة أو أسلاك مفصولة. لا استنتاج.")
        return {"ok": False, "reason": "لا حركة"}
    needed = MOTOR_INVERT if moved == "forward" else -MOTOR_INVERT
    ok = (needed == MOTOR_INVERT)
    print(f"  → MOTOR_INVERT = **{needed:+d}**  (الحالي {MOTOR_INVERT:+d})"
          + ("  ✅" if ok else "  ⚠ تحتاج تغيير"))
    return {"ok": True, "observed": moved, "needed_invert": needed,
            "current_invert": MOTOR_INVERT, "matches": ok}


# ═══ ج) اتجاه الدوران (تقاطع يكشف خطأ الأسلاك) ═══════════════════
def stage_turn(rover, power: float) -> dict:
    """
    `turn("R")` تُرسل motors(+p, −p). الاستنتاج هنا **يجب أن يطابق** استنتاج
    المرحلة (ب) لأن كليهما يشتقّ من نفس الثابت. اختلافهما يعني أن الانعكاس
    **ليس عاماً** بل في محرك واحد (أسلاك مقلوبة على جانب) — وهي حالة لا
    يصلحها أي ثابت في config.
    """
    print("\n=== ج) اتجاه الدوران ===")
    print(f"  ⚠ سيدور بالمكان {PULSE_S:.1f}ث بقوة {power}. أخلِ مساحة حوله.")
    print("  🔴 **قف خلف الروبوت** (وجهك باتجاه وجهته) — النظر من أمامه يعكس")
    print("     اليمين واليسار في عينك: شهادة أمامية خاطئة بنت نموذج عتاد")
    print("     كاملاً وأضاعت يوماً (2026-08-09). وعند أي شك، الحكم لقياس")
    print("     الجايرو باليد: calibrate_mpu6050 --only sign")
    _pause("  اضغط Enter للنبضة…")

    _pulse(rover, power, -power)      # نيّة: يميناً

    got = _ask("  إلى أين دار فعلياً؟ (r=يمين, l=يسار, n=لم يدر/اهتزّ فقط)",
               {"r": "right", "l": "left", "n": "none"})
    if got == "none":
        print("  ⚠ لم يدر — قوة غير كافية أو عجلة معلّقة. لا استنتاج.")
        return {"ok": False, "reason": "لا دوران"}
    needed = MOTOR_INVERT if got == "right" else -MOTOR_INVERT
    print(f"  → من الدوران: MOTOR_INVERT = **{needed:+d}**")
    return {"ok": True, "observed": got, "needed_invert": needed}


# ═══ الخلاصة ═════════════════════════════════════════════════════
def _verdict(g: dict, f: dict, t: dict) -> None:
    print("\n" + "═" * 62)
    print("الخلاصة:")
    lines = []

    # تقاطع (ب) و(ج): انعكاس عام أم محرك واحد؟
    if f.get("ok") and t.get("ok"):
        if f["needed_invert"] != t["needed_invert"]:
            # 🔴 مقاس 2026-08-07: تقدّم معكوس ودوران صحيح = تخطيط **مرآتي**
            #    في الفيرموير (تبديل القناتين + قلب القطبية معاً) — يصلحه
            #    زوج الثابتين، لا الأسلاك. (كان النص القديم يحيل للأسلاك
            #    حصراً — صحيح فقط حين يدور بالمكان عند أمر التقدّم.)
            print("  ⛔ **تناقض بين التقدّم والدوران** — انعكاس غير عام.")
            print("     بصمة تخطيط **مرآتي** في الفيرموير. العلاج في config:")
            print(f"       MOTOR_SWAP_LR = {not MOTOR_SWAP_LR}"
                  f"   # قلب الحالي ({MOTOR_SWAP_LR})")
            print(f"       MOTOR_INVERT  = {f['needed_invert']:+d}"
                  f"   # من قياس التقدّم (مستقل عن التبديل)")
            print("     ثم أعد هذا السكربت للتحقق.")
            print("     ⚠ أما إن كان أمر التقدّم يجعله **يدور بالمكان**: محرك")
            print("     واحد بقطبية مقلوبة — أسلاك، لا يصلحها config.")
            print("═" * 62)
            return
        if not f["matches"]:
            lines.append(f"  MOTOR_INVERT = {f['needed_invert']:+d}"
                         f"        # كان {f['current_invert']:+d}")
    elif f.get("ok") and not f["matches"]:
        lines.append(f"  MOTOR_INVERT = {f['needed_invert']:+d}"
                     f"        # كان {f['current_invert']:+d}")

    if g.get("ok") and not g["matches"]:
        lines.append(f"  MPU6050_GYRO_Z_SIGN = {g['needed_sign']:+d}"
                     f"  # كان {g['current_sign']:+d}")

    if lines:
        print("  عدّل في pi/config.py:")
        for ln in lines:
            print(ln)
        print("\n  ⚠ **أعد معايرة المعامل بعد التعديل** "
              "(pi.tests.calibrate_heading --stage 2)")
        print("     وبزاوية **90° لا 180°** — الـ180° لا تكشف انعكاس الجهة.")
    elif not any(x.get("ok") for x in (g, f, t)):
        # ⚠ كانت تطبع «كلها سليمة» هنا — قائمة تعديلات فارغة لأن **كل**
        #   المراحل بلا استنتاج ليست شهادة سلامة (مقاس 2026-08-06: روبوت
        #   لا يتحرك إطلاقاً خرج بخلاصة ✅).
        print("  ⛔ **لا استنتاج** — كل المراحل فشلت أو تُخطّيت.")
        print("     المحركات لم تتحرك أصلاً: افحص وصلة UART (المسبار:")
        print("     pi.tests.test_rover_link) والفيرموير قبل أي حكم اتجاهات.")
    else:
        skipped = [n for n, x in (("الجايرو", g), ("التقدّم", f),
                                  ("الدوران", t)) if not x.get("ok")]
        print("  ✅ الاتجاهات **المفحوصة** سليمة — لا تعديل مطلوب."
              + (f"  (بلا استنتاج: {'، '.join(skipped)})" if skipped else ""))
    print("═" * 62)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="تثبيت اتجاهات العتاد")
    ap.add_argument("--power", type=float, default=PULSE_POWER,
                    help=f"قوة النبضة (افتراضي {PULSE_POWER})")
    ap.add_argument("--allow-sim", action="store_true",
                    help="⚠ للاختبار البرمجي فقط (المحاكاة لا تُثبت اتجاهاً)")
    a = ap.parse_args(argv)

    rover = WaveRoverBridge(mode=ROVER_MODE)
    print("═" * 62)
    print(f"فحص الاتجاهات · جسر: {rover.mode} · مصدر: {rover.heading_source.name}")
    print(f"  الحالي: MOTOR_INVERT={MOTOR_INVERT:+d} · "
          f"MOTOR_SWAP_LR={MOTOR_SWAP_LR} · "
          f"MPU6050_GYRO_Z_SIGN={MPU6050_GYRO_Z_SIGN:+d}")
    if rover.error:
        print(f"  ⚠ {rover.error}")
    print("═" * 62)

    if rover.mode != "real" and not a.allow_sim:
        print("⛔ الجسر في وضع sim — المحاكاة تُولّد الاتجاه من نفس الثوابت")
        print("   فلا تُثبت شيئاً عن العتاد. شغّل:")
        print("   RMS_ROVER_MODE=real python3 -m pi.tests.check_directions")
        return 2

    g = f = t = {"ok": False}
    try:
        g = stage_gyro_sign(rover)
        f = stage_forward(rover, a.power)
        t = stage_turn(rover, a.power)
    except KeyboardInterrupt:
        print("\n⛔ أُوقف بالمستخدم — المحركات متوقفة.")
    finally:
        rover.stop()
        rover.close()

    _verdict(g, f, t)
    return 0


if __name__ == "__main__":
    sys.exit(main())
