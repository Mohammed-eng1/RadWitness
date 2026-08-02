#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_motion_check.py — معايرة التحقق من الحركة على العتاد (البند 0)
====================================================================
يحوّل الثابت **التقديري** الوحيد في `pi/nav/motion_check.py` إلى رقم مقاس،
ويثبت أن الكشف يعمل فعلاً على الأرض لا في الاختبار المنطقي وحده.

⚠ **يُشغَّل والروبوت على الأرض** ووجهه إلى جدار مستوٍ على 1–2.5م، وأمامه
   مسار خالٍ ~1م. (خلاف `test_ultrasonic_load` الذي يُشغَّل والعجلات معلّقة.)

ثلاث مراحل:

  1) **أرضية الضجيج ساكناً**: المحركات مطفأة تماماً — σ هنا هو ما يجب أن
     تبقى العتبة فوقه.
  2) **محركات تدور والروبوت محجوز** (امسكه باليد أو ألصقه بجدار): أصدق
     محاكاة لحالة «عالق» — المحركات تسحب تياراً وتهتزّ والروبوت لا يتحرّك.
     ⚠ هذه هي المرحلة الحاسمة: لو تجاوز σ فيها العتبة لأعلن النظام حركةً
       وهو عالق — وهو الفشل الذي بُني البند كله لمنعه.
  3) **شوط حقيقي**: يتقدّم مسافة معلومة ويقارن **المقاس بالألترا سونيك**
     بـ**المحسوب من السرعة المعايرة**. الفارق بينهما هو خطأ الحلقة المفتوحة
     الذي كان يمرّ صامتاً (سُجّل شوط 0.20م والنموذج يقول 1.8م).

يطبع في النهاية العتبة الموصى بها لـ`MOTION_ACCEL_STD_MPS2` = منتصف المسافة
هندسياً بين أعلى σ ساكن/عالق وأدنى σ متحرّك — بلا لمس أي ملف.

    python3 -m pi.tests.test_motion_check
    RMS_ROVER_MODE=real python3 -m pi.tests.test_motion_check --distance 0.5
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

from pi.config import (
    ROVER_MODE, DRIVE_POWER_DEFAULT, MOTION_ACCEL_STD_MPS2, MOTION_SETTLE_S,
    MOTION_REF_MAX_CM, MAX_MOTOR_POWER,
)
from pi.nav.motion_check import AccelWitness, verify_motion, tolerance_for

RENEW_S = 0.1            # تجديد الأمر (حارس heartbeat 1.5ث)
SAMPLE_S = 0.05          # فاصل أخذ العيّنات


def _collect(imu, seconds: float, rover=None, power: float = 0.0) -> AccelWitness:
    """
    يجمع عيّنات تسارع لمدة معطاة. `rover` غير None ⇒ تُشغَّل المحركات ويُجدَّد
    الأمر دورياً (heartbeat). ⚠ الإيقاف مضمون في `finally`.
    """
    w = AccelWitness(min_samples=1)
    t_end = time.time() + seconds
    last_cmd = 0.0
    try:
        while time.time() < t_end:
            if rover is not None and (time.time() - last_cmd) > RENEW_S:
                rover.motors(power, power)
                last_cmd = time.time()
            w.add(imu.accel_mps2())
            time.sleep(SAMPLE_S)
    finally:
        if rover is not None:
            rover.stop()
    return w


def _front_cm(us) -> float | None:
    """المسافة الأمامية بعد استقرار المرشّح (نفس ما يفعله المنفّذ)."""
    time.sleep(MOTION_SETTLE_S)
    return us.distance_cm if us is not None else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=6.0, help="مدة كل مرحلة")
    ap.add_argument("--distance", type=float, default=0.5, help="مسافة الشوط (م)")
    ap.add_argument("--power", type=float, default=DRIVE_POWER_DEFAULT)
    ap.add_argument("--skip-drive", action="store_true",
                    help="المرحلتان 1 و2 فقط (بلا شوط حقيقي)")
    a = ap.parse_args()
    power = min(abs(a.power), MAX_MOTOR_POWER)   # ⚠ الحاجز الصارم لا يُتجاوز

    from pi.sensors.imu import get_imu
    from pi.sensors.ultrasonic import UltrasonicReader
    from pi.rover.bridge import WaveRoverBridge

    imu = get_imu()
    print(f"\n=== معايرة التحقق من الحركة — وضع الجسر: {ROVER_MODE} ===")
    print(f"BNO055: ok={imu.ok} · {imu.error or ''}")
    if not imu.ok:
        print("⛔ لا مقياس تسارع — المرحلتان 1 و2 بلا معنى. افحص i2c-4 @ 0x29.")
        return 1
    if imu.accel_mps2() is None:
        print("⛔ قراءة التسارع تُعيد None — راجع سجلات LIA/ACC في pi/sensors/imu.py")
        return 1

    us = UltrasonicReader()
    print(f"ألترا سونيك: ok={us.ok} · خلفية={us.backend} · {us.error or ''}")

    # ── 1) أرضية الضجيج ساكناً ───────────────────────────────────
    print(f"\n[1/3] ساكن تماماً، المحركات مطفأة ({a.seconds:.0f}ث)… لا تلمس الروبوت")
    still = _collect(imu, a.seconds)
    print(f"      عيّنات={still.n} · σ={still.std:.4f} م/ث² · "
          f"مدى={still.state()['span_mps2']}")

    rover = WaveRoverBridge(mode=ROVER_MODE)
    if rover.mode != "real":
        print("\n⚠ الجسر ليس real — المرحلتان 2 و3 تحتاجان محركات فعلية.")
        print("   شغّل:  RMS_ROVER_MODE=real python3 -m pi.tests.test_motion_check")
        return 1

    # ── 2) محركات تدور والروبوت محجوز (حالة «عالق») ──────────────
    print(f"\n[2/3] ⚠ **امسك الروبوت أو ألصقه بجدار** — محركات تدور بلا حركة "
          f"({a.seconds:.0f}ث)")
    input("      اضغط Enter حين تكون ممسكاً به… ")
    stuck = _collect(imu, a.seconds, rover=rover, power=power)
    print(f"      عيّنات={stuck.n} · σ={stuck.std:.4f} م/ث² · "
          f"مدى={stuck.state()['span_mps2']}")

    moving = None
    if not a.skip_drive:
        # ── 3) شوط حقيقي: المقاس مقابل المحسوب ───────────────────
        print(f"\n[3/3] شوط حقيقي {a.distance:.2f}م — أفلت الروبوت وأخلِ مساره")
        input("      اضغط Enter للانطلاق… ")
        d0 = _front_cm(us)
        print(f"      المسافة الأمامية قبل: {d0}سم")
        if d0 is None or d0 > MOTION_REF_MAX_CM:
            print(f"      ⚠ لا سطح مرجعي داخل {MOTION_REF_MAX_CM:.0f}سم — "
                  f"المقارنة الكمّية ستسقط إلى شاهد التسارع وحده.")
        moving = AccelWitness(min_samples=1)
        # مسافة زمنية من السرعة المعايرة (هي بالضبط ما نريد التحقق منه)
        speed = 1.5 * power        # التقريب الموثّق داخل 0.1–0.5 فقط
        t_end = time.time() + a.distance / max(0.05, speed)
        last_cmd = 0.0
        try:
            while time.time() < t_end:
                if (time.time() - last_cmd) > RENEW_S:
                    rover.motors(power, power)
                    last_cmd = time.time()
                moving.add(imu.accel_mps2())
                time.sleep(SAMPLE_S)
        finally:
            rover.stop()
        d1 = _front_cm(us)
        print(f"      المسافة الأمامية بعد: {d1}سم · "
              f"عيّنات تسارع={moving.n} · σ={moving.std:.4f} م/ث²")

        v = verify_motion(a.distance, d0, d1, moving)
        print(f"\n  ── الحكم: {v['verdict']} ({v['method']}) ──")
        print(f"  {v['reason']}")
        if v["measured_m"] is not None:
            err = v["measured_m"] - a.distance
            print(f"  مأمور {a.distance:.2f}م · مقاس {v['measured_m']:.2f}م · "
                  f"فارق {err:+.2f}م (تسامح ±{tolerance_for(a.distance):.2f}م)")
            if abs(err) > tolerance_for(a.distance):
                print("  🔴 هذا بالضبط الخطأ الذي كان يمرّ صامتاً قبل البند 0 — "
                      "النموذج كان سيُعلّم الخلية كأن الشوط تمّ كاملاً.")

    # ── التوصية ──────────────────────────────────────────────────
    floor = max(still.std, stuck.std)
    print("\n" + "═" * 58)
    print(f"σ ساكن            = {still.std:.4f} م/ث²")
    print(f"σ عالق (محركات)   = {stuck.std:.4f} م/ث²   ← الأهمّ")
    if moving is not None:
        print(f"σ متحرّك           = {moving.std:.4f} م/ث²")
    print(f"العتبة الحالية     = {MOTION_ACCEL_STD_MPS2:.4f} م/ث² (تقديرية)")

    if moving is None:
        print("\nبلا مرحلة 3 لا توصية: العتبة تحتاج طرفَي المقارنة معاً.")
        return 0
    if moving.std <= floor:
        print("\n🔴 σ المتحرّك ليس أعلى من σ العالق — الشاهد الثنائي **لا يفصل**")
        print("   على هذه الأرضية. أبقِ العتبة منخفضة (يمتنع عن النفي) واعتمد")
        print("   على الألترا سونيك، ولا تسمح للتسارع بإيقاف المهمة وحده.")
        return 1
    rec = math.sqrt(max(floor, 1e-4) * moving.std)   # وسط هندسي بين الطرفين
    print(f"\n✅ العتبة الموصى بها: MOTION_ACCEL_STD_MPS2 = {rec:.3f}")
    print(f"   (وسط هندسي بين {floor:.4f} و{moving.std:.4f} — "
          f"هامش ×{moving.std / rec:.1f} على كل طرف)")
    print("   ضعها في pi/config.py واذكر تاريخ القياس والأرضية.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nأُلغي.")
        sys.exit(130)
