#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_imu_health.py — صحة MPU-6050: هل الحسّاس حيّ، وهل تُسقطه المحركات؟
=========================================================================
أُعيدت كتابته لـMPU-6050 بعد تلف BNO055 (كان يفحص 0x29 على شريحة لم تعد
موجودة — سكربت بائت اكتُشف على العتاد 2026-08-06).

**البصمة المكافئة لعودة BNO055 إلى CONFIG**: إعادة التشغيل الذاتية لـ
MPU-6050 (هبوط 3.3V لحظي عند اندفاع تيار المحركات) تعيده إلى وضع
**السكون** (بت SLEEP في PWR_MGMT_1) — فيبقى حيّاً على الناقل ويردّ
بهويته، ويقرأ الجايرو **صفراً مضبوطاً إلى الأبد**. نفس مظهر «حسّاس ميت»
وعلاجه سطر واحد (إيقاظ) لا استبدال.

المراحل:
  1. حالة الشريحة: WHO_AM_I · بت السكون · الحرارة · جاذبية التسارع.
  2. مراقبة gz والمحركات **مطفأة** — أرضية مرجعية (σ يجب ألا تكون صفراً:
     ضجيج القياس نفسه دليل الحياة، والصفر المضبوط المتتابع دليل السكون).
  3. (--motors) نبضات **دوران بالمكان** (أثقل مناورة كهربائياً — §6.2) مع
     مراقبة gz وبت السكون بعد كل نبضة: سقوط متزامن مع الإقلاع = تغذية.
  4. عند اكتشاف السكون: محاولة إيقاظ واحدة + قراءة تحقّق.

التشغيل على الراسبري (لا يحرّك شيئاً افتراضياً):
    python3 -m pi.tests.check_imu_health
    python3 -m pi.tests.check_imu_health --motors --pulses 6

⚠ `--motors` يدير الروبوت بالمكان — أفرغ حوله مساحة وأمسك مفتاح الطوارئ.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

from pi.config import TURN_POWER, MAX_MOTOR_POWER
from pi.sensors.mpu6050 import (get_mpu, REG_WHO_AM_I,
                                WHO_AM_I_NAMES, chip_name)

REG_PWR_MGMT_1 = 0x6B          # بت 6 = SLEEP (يُضبط تلقائياً بعد إعادة تشغيل)


def sleep_bit(m):
    """يقرأ بت السكون مباشرة من الشريحة. None عند تعذّر القراءة."""
    try:
        return bool(m._bus.read_byte_data(m.addr, REG_PWR_MGMT_1) & 0x40)
    except Exception:              # noqa: BLE001
        return None


def wake(m) -> bool:
    """محاولة إيقاظ واحدة ثم **قراءة تحقّق** — لا نصدّق نجاح الكتابة وحده."""
    try:
        m._bus.write_byte_data(m.addr, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        return sleep_bit(m) is False
    except Exception:              # noqa: BLE001
        return False


def show_health(m) -> bool:
    print("\n── حالة الشريحة الآن ──")
    if not m.ok:
        print(f"  ⛔ لم يُفتح: {m.error}")
        print("     افحص:  i2cdetect -y 4   (يجب أن يظهر 68)")
        return False
    try:
        who = m._bus.read_byte_data(m.addr, REG_WHO_AM_I)
    except Exception as e:         # noqa: BLE001
        print(f"  ⛔ الناقل لا يردّ: {e}")
        return False
    # ⚠ العائلة أربع شرائح بنفس السجلات — الاسم يُطبع لا يُفترض
    print(f"  WHO_AM_I = {hex(who)} → **{chip_name(who)}**  "
          + ("✅" if who in WHO_AM_I_NAMES else "⛔ خارج عائلة MPU المدعومة"))
    sb = sleep_bit(m)
    print(f"  بت السكون = {sb}  "
          + ("✅ مستيقظ" if sb is False else
             "🔴 **نائم — هذه بصمة إعادة تشغيل ذاتية** (كل قراءات الجايرو صفر)"
             if sb else "؟ تعذّرت القراءة"))
    # ⚠ قراءة واحدة قد تعود أصفاراً عابرة (شوهد 2026-08-06: جاذبية 0.00
    #   والجايرو حيّ في نفس اللحظة) — ثلاث محاولات قبل أي حكم.
    g = 0.0
    for _ in range(3):
        ax, ay, az = m.accel_mps2() or (0, 0, 0)
        g = (ax * ax + ay * ay + az * az) ** 0.5
        if g > 1.0:
            break
        time.sleep(0.05)
    print(f"  الجاذبية ساكناً = {g:.2f} م/ث²  "
          + ("✅" if 9.0 <= g <= 10.6 else "⚠ خارج [9.0, 10.6] — اهتزاز أو عطل"))
    t = m.temperature_c()
    if t is not None:
        print(f"  الحرارة = {t:.1f}°C")
    return sb is not True


def watch(m, seconds: float, label: str) -> dict:
    """يراقب gz: المعدل وσ وأطول سلسلة **صفر مضبوط** وأخطاء الناقل."""
    vals, streak, worst, errors = [], 0, 0, 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            z = m.gyro_z_dps()
        except Exception:          # noqa: BLE001
            errors += 1
            z = None
        if z is None:
            errors += 1
        elif z == 0.0:             # الصفر **المضبوط** — لا القريب من الصفر
            streak += 1
            worst = max(worst, streak)
            vals.append(z)
        else:
            streak = 0
            vals.append(z)
        time.sleep(0.01)
    rate = len(vals) / max(seconds, 1e-9)
    sig = statistics.stdev(vals) if len(vals) > 2 else 0.0
    print(f"  {label}: {len(vals)} عيّنة (~{rate:.0f}Hz) · σ={sig:.4f}°/ث · "
          f"أطول سلسلة صفر مضبوط={worst} · أخطاء ناقل={errors}")
    if sig == 0.0 and vals:
        print("  🔴 σ=0 بالضبط — الشريحة نائمة أو معلّقة (الضجيج الحي لا يكون صفراً)")
    return {"rate": rate, "sigma": sig, "worst_zero_streak": worst,
            "errors": errors}


def main() -> int:
    ap = argparse.ArgumentParser(description="صحة MPU-6050 (+ اختبار هبوط التغذية مع المحركات)")
    ap.add_argument("--seconds", type=float, default=8.0,
                    help="مدة المراقبة الساكنة")
    ap.add_argument("--motors", action="store_true",
                    help="⚠ نبضات دوران بالمكان مع المراقبة (يحرّك الروبوت)")
    ap.add_argument("--pulses", type=int, default=6)
    ap.add_argument("--power", type=float, default=TURN_POWER)
    a = ap.parse_args()

    m = get_mpu()
    alive = show_health(m)
    if not m.ok:
        return 1
    if not alive and not wake(m):
        print("  ⛔ فشل الإيقاظ — افحص التغذية 3.3V واللحامات ثم أعد التشغيل")
        return 1

    print(f"\n── أرضية مرجعية ({a.seconds:.0f}ث، محركات مطفأة، لا تلمس الروبوت) ──")
    base = watch(m, a.seconds, "ساكن")
    verdict_ok = base["sigma"] > 0.0 and base["errors"] == 0

    if a.motors:
        p = min(abs(a.power), MAX_MOTOR_POWER)
        print(f"\n── نبضات المحركات (دوران بالمكان × {a.pulses} بقوة {p}) ──")
        print("  ⚠ الروبوت سيدور — أفرغ حوله مساحة الآن (5 ثوانٍ)…")
        time.sleep(5.0)
        from pi.rover.bridge import WaveRoverBridge
        rover = WaveRoverBridge(mode="real")
        if rover.mode != "real":
            print(f"  ⛔ لا وصلة روفر ({rover.error}) — شغّل سويتش الهيكل")
            return 1
        drops = []
        try:
            for i in range(1, a.pulses + 1):
                # نبضة 0.8ث مع تجديد الأمر (حارس heartbeat 1.5ث)
                t0 = time.time()
                errs_during = 0
                while time.time() - t0 < 0.8:
                    rover.motors(p, -p)
                    try:
                        m.gyro_z_dps()
                    except Exception:  # noqa: BLE001
                        errs_during += 1
                    time.sleep(0.02)
                rover.stop()
                time.sleep(0.3)
                sb = sleep_bit(m)
                mark = "✅" if (sb is False and errs_during == 0) else "🔴"
                print(f"  نبضة {i}: أخطاء أثناءها={errs_during} · "
                      f"نائم بعدها={sb}  {mark}")
                if sb or errs_during:
                    drops.append(i)
                    if sb and wake(m):
                        print("     ↻ أُوقظ ونجحت قراءة التحقّق")
        finally:
            rover.stop()           # ⚠ إيقاف مضمون
        print("\n── الحكم ──")
        if drops:
            print(f"  🔴 سقوط متزامن مع المحركات في النبضات {drops}: "
                  f"التغذية تهبط مع الاندفاع ⇒ **إصلاح عتادي** — مكثّف "
                  f"470µF+ عند الحسّاس أو فصل تغذيته عن خط المحركات")
            return 1
        print("  ✅ لا سقوط ولا أخطاء عبر كل النبضات — التغذية صامدة")
    elif verdict_ok:
        print("\n✅ الحسّاس حيّ وسليم ساكناً. أعد مع --motors (والسويتش شغّال) "
              "لاختبار الصمود مع اندفاع المحركات.")
    return 0 if verdict_ok else 1


if __name__ == "__main__":
    sys.exit(main())
