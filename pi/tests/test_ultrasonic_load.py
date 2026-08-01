#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_ultrasonic_load.py — هل ينقطع الألترا سونيك تحت حمل المحركات؟
==================================================================
⚠ **لا تشغّله والروبوت على الأرض** — ارفعه على حامل/صندوق حتى تدور العجلات
   في الهواء، ووجّه الحسّاس إلى جدار ثابت على 0.5–2م.

المقارنة الوحيدة التي تفصل السببين:

  أ) المحركات **مطفأة**  → معدل نجاح القراءة
  ب) المحركات **تعمل**   → معدل نجاح القراءة

الأساس المقاس على العتاد 2026-08-01: على الطاولة بلا محركات كانت النتيجة
**5/5 صالحة دائماً** وتشتت 1.1سم؛ وأثناء المهمة انقطعت القراءة 11 مرة
(`no_reading`). هذا السكربت يثبت أن الفارق سببه المحركات لا شيء آخر.

تفسير النتيجة:
  • نجاح مرتفع في (أ) ومنخفض في (ب) ⇒ **المحركات هي السبب**. والتمييز بين
    فرعيه بالجهد: هبوط 5V تحت السحب (الأرجح مع 4×4) أو ضجيج على خط ECHO.
    العلاج مختلف: مكثّف تنعيم/تغذية منفصلة للحسّاس مقابل تمرير الأسلاك
    بعيداً عن أسلاك المحركات وتقصير خط ECHO.
  • نجاح منخفض في الحالتين ⇒ ليست المحركات — راجع التوصيل ومقسّم الجهد.
  • نجاح مرتفع في الحالتين ⇒ السبب في مكان آخر (تزاحم المعالج أثناء المهمة،
    أو اهتزاز التثبيت أثناء السير الفعلي على الأرض).

    python3 -m pi.tests.test_ultrasonic_load
    RMS_ROVER_MODE=real python3 -m pi.tests.test_ultrasonic_load --seconds 12
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

from pi.config import ROVER_MODE

PULSE_POWER = 0.30      # قوة كافية لسحب تيار واقعي بلا خطر
RENEW_S = 0.1           # تجديد الأمر (حارس heartbeat 1.5ث)


def _sample(us, seconds: float, rover=None, power: float = 0.0) -> dict:
    """
    يجمع قراءات خام لمدة معطاة. `rover` غير None ⇒ تُشغَّل المحركات ويُجدَّد
    الأمر دورياً. الإيقاف مضمون في `finally`.
    """
    vals, fails = [], 0
    t_end = time.time() + seconds
    last_cmd = 0.0
    try:
        while time.time() < t_end:
            if rover is not None and (time.time() - last_cmd) > RENEW_S:
                rover.motors(power, power)
                last_cmd = time.time()
            v = us.raw_cm          # الخام لا المُرشَّح: المرشّح يخفي الانقطاع
            if v is None:
                fails += 1
            else:
                vals.append(v)
            time.sleep(0.05)
    finally:
        if rover is not None:
            rover.stop()
    n = len(vals) + fails
    return {
        "n": n, "ok": len(vals), "fail": fails,
        "success_pct": round(100.0 * len(vals) / n, 1) if n else 0.0,
        "mean": round(statistics.fmean(vals), 1) if vals else None,
        "stdev": round(statistics.pstdev(vals), 2) if len(vals) > 1 else 0.0,
        "min": round(min(vals), 1) if vals else None,
        "max": round(max(vals), 1) if vals else None,
    }


def _show(title: str, r: dict) -> None:
    print(f"\n  {title}")
    print(f"    نجاح {r['success_pct']}%  ({r['ok']} من {r['n']}، "
          f"فشل {r['fail']})")
    if r["mean"] is not None:
        print(f"    المسافة {r['mean']}سم · تشتت {r['stdev']} · "
              f"المدى {r['min']}–{r['max']}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="الألترا سونيك تحت حمل المحركات")
    ap.add_argument("--seconds", type=float, default=10.0, help="مدة كل مرحلة")
    ap.add_argument("--power", type=float, default=PULSE_POWER)
    a = ap.parse_args(argv)

    from pi.sensors.ultrasonic import UltrasonicReader
    us = UltrasonicReader()
    if not us.ok:
        print(f"⛔ الألترا سونيك غير متاح: {us.error}")
        return 2

    from pi.rover.bridge import WaveRoverBridge
    rover = WaveRoverBridge(mode=ROVER_MODE)
    print("═" * 62)
    print(f"الألترا سونيك تحت الحمل · جسر: {rover.mode} · قوة {a.power}")
    if rover.mode != "real":
        print("  ⚠ الجسر في وضع sim — المرحلة (ب) لن تسحب تياراً فعلياً.")
    print("═" * 62)
    print("⚠ **ارفع الروبوت** حتى تدور العجلات في الهواء، ووجّه الحسّاس")
    print("   إلى جدار ثابت على 0.5–2م ولا تحرّك شيئاً بينهما.")

    off = on = None
    try:
        input("\n  [Enter] لبدء المرحلة (أ) — المحركات مطفأة… ")
        rover.stop()
        time.sleep(0.5)
        off = _sample(us, a.seconds)
        _show("(أ) محركات مطفأة:", off)

        input("\n  [Enter] لبدء المرحلة (ب) — المحركات تعمل ⚠… ")
        on = _sample(us, a.seconds, rover=rover, power=a.power)
        _show("(ب) محركات تعمل:", on)
    except KeyboardInterrupt:
        print("\n⛔ أُوقف بالمستخدم — المحركات متوقفة.")
    finally:
        rover.stop()
        rover.close()
        us.close()

    print("\n" + "═" * 62)
    if not (off and on and off["n"] and on["n"]):
        print("لم تكتمل المرحلتان — لا حكم.")
        print("═" * 62)
        return 1

    drop = off["success_pct"] - on["success_pct"]
    print(f"الفارق في معدل النجاح: {off['success_pct']}% → {on['success_pct']}%"
          f"  ({drop:+.1f} نقطة)")
    if drop >= 15.0:
        print("\n⛔ **المحركات هي السبب** — الانقطاع يتبع تشغيلها.")
        print("   ميّز بين فرعيه: قِس 5V عند أطراف الحسّاس نفسه أثناء دوران")
        print("   المحركات. لو هبط دون ~4.75V فالسبب **هبوط الجهد**:")
        print("     • مكثّف 100–470µF عند أطراف تغذية الحسّاس، أو")
        print("     • تغذية 5V منفصلة للحسّاس عن خط المحركات.")
        print("   ولو ظل الجهد ثابتاً فالسبب **ضجيج على خط ECHO**:")
        print("     • أبعِد سلكَي الحسّاس عن أسلاك المحركات وقصّرهما، و")
        print("     • ضع مكثّف 100nF بين ECHO والأرضي عند دبوس الراسبري.")
    elif off["success_pct"] < 85.0:
        print("\n⚠ النجاح منخفض **حتى بلا محركات** — المشكلة في التوصيل نفسه")
        print("   لا في الحمل. راجع مقسّم الجهد على ECHO (1kΩ/2kΩ) والتلامس.")
    else:
        print("\n✅ لا فرق يُذكر — المحركات ليست السبب.")
        print("   المشتبه التالي: **تزاحم المعالج أثناء المهمة** (خيط القياس")
        print("   يُزاحمه خيط المحركات وقراءات I2C وحلقة السيرفر)، أو اهتزاز")
        print("   التثبيت أثناء السير الفعلي على الأرض لا في الهواء.")
    print("═" * 62)
    return 0


if __name__ == "__main__":
    sys.exit(main())
