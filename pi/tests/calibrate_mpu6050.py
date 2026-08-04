#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_mpu6050.py — القياسات الثلاثة التي بلا معنى بدونها أي رقم انحراف
==========================================================================
كل ثوابت الاتجاه في المشروع كانت معايرة لـ**BNO055** (معدل ~45 Hz، أرضية
ضجيج σ=0.0702°/ث). واستُبدلت الشريحة بـMPU-6050 (39.7 Hz، أرضية غير معروفة)،
فصارت تلك الثوابت **غير معايرة** ولا يجوز البناء عليها.

⚠ ورقم المواصفة «3.6° انحراف/ساعة بعد الطرح» **لا يتّسق حسابياً**:
   0.06°/ث × 3600 = 216°/ساعة لا 3.6. فإما أن المقصود انحرافاً متبقياً
   0.001°/ث (متفائل جداً لهذه الشريحة) أو أن الرقم سهو. المرحلة 2 تحسمه.

المراحل:

  1) **أرضية الضجيج ساكناً** ⇒ `HEADING_DEADBAND_DPS`
     عتبة السكون يجب أن تعلو ضجيج الحسّاس وإلا لاحقه المتحكّم؛ ولو علت
     كثيراً لمرّ انعطاف بطيء **غير مرئي** (0.30 الحالية مشتقّة من BNO055
     الأهدأ — قد تكون منخفضة جداً هنا).

  2) 🔴 **الانحراف الحقيقي عبر زمن مهمة** ⇒ `HEADING_DRIFT_PER_S_DEG`
     يُقاس الانحياز ثم يُطرح، ويُترك التكامل يعمل **10 دقائق والروبوت ساكن
     تماماً**. الزاوية المتراكمة ÷ الزمن = الانحراف المتبقي الفعلي. هذا
     الرقم وحده يقرّر إن كان `pos_sigma_m` صادقاً أم متفائلاً.

  3) **إشارة محور Z** ⇒ `MPU6050_GYRO_Z_SIGN`
     ⚠ **باليد بلا محركات** (CLAUDE.md §2): قياسها بالمحركات يعطي حاصل ضرب
     خطأين فتبدو سليمة وهي معكوسة. ⚠ وتتغيّر مع `MOTOR_INVERT` **معاً**.

    python3 -m pi.tests.calibrate_mpu6050                 # الثلاث مراحل
    python3 -m pi.tests.calibrate_mpu6050 --drift-min 10  # إطالة مرحلة 2
    python3 -m pi.tests.calibrate_mpu6050 --only sign     # مرحلة واحدة
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

from pi.config import (
    MPU6050_ADDR, MPU6050_I2C_BUS, HEADING_DEADBAND_DPS,
    HEADING_DRIFT_PER_S_DEG, MPU6050_GYRO_Z_SIGN, MPU6050_GYRO_SCALE,
    GYRO_BIAS_CALIB_S,
)
from pi.sensors.mpu6050 import MPU6050Reader

SAMPLE_S = 0.025            # ~40 Hz، معدل الشريحة المقاس


def measure_bias(mpu: MPU6050Reader, seconds: float) -> dict:
    """انحياز المحاور الثلاثة والروبوت **ساكن تماماً**."""
    xs, ys, zs = [], [], []
    t_end = time.time() + seconds
    while time.time() < t_end:
        g = mpu.gyro_dps()
        if g is not None:
            xs.append(g[0]); ys.append(g[1]); zs.append(g[2])
        print(f"\r    {t_end - time.time():4.1f}ث · {len(zs)} عيّنة", end="",
              flush=True)
        time.sleep(SAMPLE_S)
    print()
    if len(zs) < 20:
        return {"ok": False, "reason": f"عيّنات غير كافية ({len(zs)})"}
    return {"ok": True, "n": len(zs),
            "bias": (statistics.fmean(xs), statistics.fmean(ys),
                     statistics.fmean(zs)),
            "std_z": statistics.pstdev(zs),
            "rate_hz": len(zs) / seconds}


def phase_noise(mpu: MPU6050Reader, seconds: float) -> None:
    print(f"\n[1/3] أرضية الضجيج ({seconds:.0f}ث، **ساكن تماماً، لا تلمسه**)")
    b = measure_bias(mpu, seconds)
    if not b["ok"]:
        print(f"    ⛔ {b['reason']}")
        return
    bz, sz = b["bias"][2], b["std_z"]
    print(f"    الانحياز: X={b['bias'][0]:+.3f} Y={b['bias'][1]:+.3f} "
          f"Z={bz:+.3f} °/ث")
    print(f"    σ(Z) = {sz:.4f} °/ث · معدل القراءة {b['rate_hz']:.1f} Hz")
    rec = round(max(3.0 * sz, 0.05), 2)
    print(f"\n    ✅ HEADING_DEADBAND_DPS الموصى بها = **{rec}**  (3σ)")
    print(f"       الحالية {HEADING_DEADBAND_DPS} — "
          + ("مناسبة" if abs(rec - HEADING_DEADBAND_DPS) < 0.1 else
             "🔴 **تحتاج تحديثاً** (كانت مشتقّة من BNO055)"))
    if b["rate_hz"] < 30:
        print(f"    ⚠ معدل القراءة {b['rate_hz']:.1f} Hz أقل من المتوقَّع "
              f"(39.7) — راجع سرعة الناقل وحمل المعالج.")


def phase_drift(mpu: MPU6050Reader, minutes: float, bias_s: float) -> None:
    print(f"\n[2/3] 🔴 الانحراف الحقيقي — {minutes:.0f} دقائق سكون")
    print("    ⚠ لا تلمس الروبوت ولا الطاولة إطلاقاً خلال القياس.")
    print(f"    أولاً: قياس الانحياز ({bias_s:.0f}ث)…")
    b = measure_bias(mpu, bias_s)
    if not b["ok"]:
        print(f"    ⛔ {b['reason']}")
        return
    bz = b["bias"][2]
    print(f"    الانحياز Z = {bz:+.4f} °/ث — يُطرح من كل قراءة الآن.")

    seconds = minutes * 60.0
    heading, last = 0.0, time.time()
    t_end = last + seconds
    peak = 0.0
    while time.time() < t_end:
        g = mpu.gyro_dps()
        now = time.time()
        dt, last = now - last, now
        if g is not None and 0 < dt < 1.0:
            heading += (g[2] - bz) * dt * MPU6050_GYRO_SCALE
            peak = max(peak, abs(heading))
        el = now - (t_end - seconds)
        print(f"\r    {el / 60:5.2f}/{minutes:.0f}د · الانحراف المتراكم "
              f"{heading:+7.2f}° · الذروة {peak:.2f}°", end="", flush=True)
        time.sleep(SAMPLE_S)
    print()

    per_s = abs(heading) / seconds
    per_h = per_s * 3600.0
    print(f"\n    الانحراف بعد {minutes:.0f}د = **{heading:+.2f}°** "
          f"⇒ {per_s:.5f} °/ث ({per_h:.1f} °/ساعة)")
    print(f"    ✅ HEADING_DRIFT_PER_S_DEG الموصى بها = **{per_s:.4f}**")
    print(f"       الحالية {HEADING_DRIFT_PER_S_DEG} (غير معايرة)")
    # الأثر العملي على المهمة — الرقم الذي يهمّ فعلاً
    from pi.config import MISSION_TIME_LIMIT_S, WALL_ALIGN_TOL_DEG
    at_mission = per_s * MISSION_TIME_LIMIT_S
    print(f"\n    ⇒ على مهمة {MISSION_TIME_LIMIT_S:.0f}ث: انحراف "
          f"**{at_mission:.1f}°**")
    if at_mission > WALL_ALIGN_TOL_DEG:
        print(f"    🔴 يتجاوز تسامح المحاذاة {WALL_ALIGN_TOL_DEG:.0f}° ⇒ "
              f"**تصحيح الجدران سينهار قبل نهاية المهمة** (الحلقة المفرغة). "
              f"قصّر المهمة أو زد تكرار التصحيح أو أضف مرجعاً مطلقاً.")
    else:
        print(f"    ✅ دون تسامح المحاذاة {WALL_ALIGN_TOL_DEG:.0f}° — "
              f"تصحيح الجدران يبقى متاحاً طوال المهمة.")


def phase_sign(mpu: MPU6050Reader, seconds: float) -> None:
    print(f"\n[3/3] إشارة محور Z — **باليد بلا محركات**")
    print("    ⚠ قياسها بالمحركات يعطي حاصل ضرب خطأين فتبدو سليمة وهي معكوسة.")
    print(f"    أدر الروبوت **يميناً ~90°** باليد خلال {seconds:.0f} ثانية.")
    input("    اضغط Enter ثم أدره… ")
    raw, last = 0.0, time.time()
    t_end = last + seconds
    peak = 0.0
    while time.time() < t_end:
        g = mpu.gyro_dps()
        now = time.time()
        dt, last = now - last, now
        if g is not None and 0 < dt < 1.0:
            raw += g[2] * dt                     # **خام** بلا إشارة ولا انحياز
            peak = max(peak, abs(g[2]))
        print(f"\r    {t_end - time.time():4.1f}ث · التكامل الخام "
              f"{raw:+7.1f}° · الذروة {peak:5.1f}°/ث", end="", flush=True)
        time.sleep(SAMPLE_S)
    print()
    if peak < 20.0:
        print("    ⛔ لم يُرصد دوران يُذكر — أعد المحاولة وأدره بوضوح.")
        return
    sign = -1 if raw < 0 else +1
    print(f"\n    التكامل الخام لدوران **يميناً** = {raw:+.1f}°")
    print(f"    ✅ MPU6050_GYRO_Z_SIGN الموصى بها = **{sign:+d}**")
    print(f"       الحالية {MPU6050_GYRO_Z_SIGN:+d} — "
          + ("مطابقة ✓" if sign == MPU6050_GYRO_Z_SIGN else
             "🔴 **معكوسة!** غيّرها، و⚠ راجع MOTOR_INVERT معها (يتغيّران معاً)"))
    # معامل التحويل من زاوية معلومة
    est = abs(raw)
    print(f"\n    ولو كانت اللفّة 90° بالضبط: MPU6050_GYRO_SCALE ≈ "
          f"{90.0 / est:.4f} (الحالية {MPU6050_GYRO_SCALE})")
    print("    ⚠ دقّة العين ±5° على 90° = ±5.5% — لا تعتمدها إلا بلفّات متعددة.")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", choices=("noise", "drift", "sign"), default=None)
    ap.add_argument("--noise-s", type=float, default=20.0)
    ap.add_argument("--drift-min", type=float, default=10.0)
    ap.add_argument("--bias-s", type=float, default=GYRO_BIAS_CALIB_S)
    ap.add_argument("--sign-s", type=float, default=12.0)
    a = ap.parse_args()

    print(f"\n=== معايرة MPU-6050 (i2c-{MPU6050_I2C_BUS} @ {hex(MPU6050_ADDR)}) ===")
    mpu = MPU6050Reader()
    if not mpu.ok:
        print(f"\n  ⛔ {mpu.error}")
        print(f"     افحص: i2cdetect -y {MPU6050_I2C_BUS}  (توقّع 0x68)")
        print("     وتأكد من سطر dtoverlay=i2c4,pins_6_7 في config.txt")
        return 1
    print(f"  ✅ استجاب · WHO_AM_I={hex(mpu.who_am_i)}")
    acc = mpu.accel_mps2()
    if acc:
        mag = math.sqrt(sum(v * v for v in acc)) / 9.80665
        print(f"  التسارع ساكناً = {mag:.3f} g "
              + ("✓" if 0.9 < mag < 1.1 else "⚠ بعيد عن 1g — راجع التثبيت"))

    if a.only in (None, "noise"):
        phase_noise(mpu, a.noise_s)
    if a.only in (None, "drift"):
        phase_drift(mpu, a.drift_min, a.bias_s)
    if a.only in (None, "sign"):
        phase_sign(mpu, a.sign_s)

    print("\n" + "═" * 58)
    print("ضع القيم الموصى بها في pi/config.py واذكر تاريخ القياس،")
    print("ثم احذف وسم «غير معاير» عنها. ولا تنسخ رقماً بين حسّاسين.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nأُلغي.")
        sys.exit(130)
