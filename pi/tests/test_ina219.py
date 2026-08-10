#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_ina219.py — التحقق من قراءة جهد البطارية (UPS Module 3S)
==============================================================
يثبت أن الحماية الجهدية **عادت فعلاً** لا في الشيفرة فقط:

  1) العنوان والقراءة: هل يستجيب INA219 على 0x41، وهل الجهد معقول لحزمة 3S؟
  2) الثبات: تشتت القراءة ساكناً — قراءة تقفز فولتاً كل ثانية لا يُبنى عليها قرار.
  3) بت التجاوز (OVF): كم قراءة رُفضت؟ رفض متكرر = إعداد أو توصيل خاطئ.
  4) 🔴 **الهبوط تحت حمل المحركات** — الأهم عملياً: أربعة محركات تسحب تياراً
     فيهبط الجهد لحظياً. لو تجاوز الهبوط الفارق إلى عتبة العودة (10.2V) لأطلق
     الروبوت عودة إجبارية كاذبة كلما تحرّك. هذا الرقم يقرّر إن كانت العتبات
     صالحة كما هي أم تحتاج تنعيماً.

⚠ المرحلة 4 **ترفع العجلات عن الأرض** (حامل/صندوق) — تدور بلا أن يتحرّك.

    python3 -m pi.tests.test_ina219
    RMS_ROVER_MODE=real python3 -m pi.tests.test_ina219 --load --seconds 10
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

from pi.config import (
    ROVER_MODE, BATT_EXCELLENT_V, BATT_GOOD_V, BATT_LOW_V, BATT_CRITICAL_V,
    INA219_ADDR, INA219_I2C_BUS, BATTERY_MONITOR_ENABLED, MAX_MOTOR_POWER,
    INA219_CURRENT_SIGN,
)
from pi.rover import battery as batt
from pi.sensors.ina219 import INA219Reader

RENEW_S = 0.1            # تجديد أمر المحركات (حارس heartbeat 1.5ث)

_ok = []


def check(name, cond, detail=""):
    _ok.append(bool(cond))
    print(("  ✅ " if cond else "  ❌ ") + name + (f"   [{detail}]" if detail else ""))
    return bool(cond)


def sample(ina: INA219Reader, seconds: float, rover=None, power: float = 0.0) -> dict:
    """يجمع قراءات جهد لمدة معطاة. `rover` غير None ⇒ تُشغَّل المحركات."""
    vals, rejects, ovf = [], 0, 0
    t_end = time.time() + seconds
    last_cmd = 0.0
    try:
        while time.time() < t_end:
            if rover is not None and (time.time() - last_cmd) > RENEW_S:
                rover.motors(power, power)
                last_cmd = time.time()
            r = ina.read()
            if r["v"] is None:
                rejects += 1
                if r.get("ovf"):
                    ovf += 1
            else:
                vals.append(r["v"])
            time.sleep(0.05)
    finally:
        if rover is not None:
            rover.stop()                      # ⚠ إيقاف مضمون
    return {"n": len(vals) + rejects, "ok": len(vals), "rejects": rejects,
            "ovf": ovf, "vals": vals,
            "mean": statistics.fmean(vals) if vals else None,
            "min": min(vals) if vals else None,
            "max": max(vals) if vals else None,
            "stdev": statistics.pstdev(vals) if len(vals) > 1 else 0.0}


def sample_amps(ina: INA219Reader, seconds: float, rover=None,
                power: float = 0.0) -> float | None:
    """وسيط التيار **الخام** (قبل إشارة اللوحة) — أساس قياس الإشارة."""
    vals = []
    t_end = time.time() + seconds
    last_cmd = 0.0
    try:
        while time.time() < t_end:
            if rover is not None and (time.time() - last_cmd) > RENEW_S:
                rover.motors(power, power)
                last_cmd = time.time()
            a = ina.read().get("amps_raw")
            if a is not None:
                vals.append(a)
            time.sleep(0.05)
    finally:
        if rover is not None:
            rover.stop()
    return statistics.median(vals) if vals else None


def measure_sign(ina: INA219Reader, seconds: float, power: float) -> int:
    """
    🔴 يقيس `INA219_CURRENT_SIGN` **بالمحركات** لا بالشاحن.

    المنطق الذي يجعله حاسماً: تشغيل المحركات **يزيد السحب من الحزمة**
    يقيناً — لا احتمال آخر ولا حاجة إلى شاحن ولا إلى ثقة بحالته. فإن صار
    التيار الخام **أكثر موجبيةً** تحت الحمل ⇒ الموجب **تفريغ** (الإشارة
    ‎-1)، وإن صار أكثر سلبيةً ⇒ الموجب **شحن** (‎+1).

    ⚠ ولا يُقاس بمقارنة «موصول/مفصول»: حالة الشاحن شهادة عين، وشهادة
      العين هي التي قلبت ثوابت المحركات مرتين في يومين (§2).
    """
    print(f"\n  ⚠ **ارفع العجلات عن الأرض** — قوة {power:.2f} لمدة "
          f"{seconds:.0f}ث")
    input("      اضغط Enter حين تكون العجلات في الهواء… ")
    from pi.rover.bridge import WaveRoverBridge
    rover = WaveRoverBridge(mode="real")
    idle = sample_amps(ina, 3.0)
    load = sample_amps(ina, seconds, rover=rover, power=power)
    rover.stop()
    if idle is None or load is None:
        print("  ⛔ تعذّرت قراءة التيار — لا معايرة")
        return 0
    delta = load - idle
    print(f"      خام ساكناً = {idle:+.3f}A · تحت الحمل = {load:+.3f}A · "
          f"الفرق = {delta:+.3f}A")
    if abs(delta) < 0.10:
        print("  ⛔ الفرق أصغر من أن يحسم (<0.10A). ارفع القوة أو تأكّد أن "
              "المحركات دارت فعلاً — لا تخمّن الإشارة.")
        return 0
    sign = -1 if delta > 0 else +1
    print(f"\n  ✅ **INA219_CURRENT_SIGN = {sign:+d}**  "
          + ("(الموجب = تفريغ)" if sign < 0 else "(الموجب = شحن)"))
    print(f"     الحمل زاد السحب فصار التيار الخام أكثر "
          + ("موجبيةً" if delta > 0 else "سلبيةً")
          + " — ولا مصدر آخر لهذا الاتجاه.")
    print(f"     ضعها في pi/config.py:  INA219_CURRENT_SIGN = {sign:+d}")
    if INA219_CURRENT_SIGN and INA219_CURRENT_SIGN != sign:
        print(f"  🔴 القيمة الحالية ({INA219_CURRENT_SIGN:+d}) **معكوسة** — "
              f"النظام يقرأ التفريغ شحناً وطبقة الإطفاء معطّلة عملياً.")
    elif not INA219_CURRENT_SIGN:
        print("  ⚠ القيمة الحالية 0 (غير معايرة): النظام لا يدّعي شحناً "
              "وطبقة الإطفاء مسلَّحة — آمن لكنه ناقص.")
    return sign


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--load", action="store_true",
                    help="⚠ يشغّل المحركات — ارفع العجلات عن الأرض")
    ap.add_argument("--power", type=float, default=0.30)
    ap.add_argument("--sign", action="store_true",
                    help="⚠ يشغّل المحركات — يقيس اتجاه إشارة التيار")
    a = ap.parse_args()

    print("\n=== قراءة جهد البطارية (INA219 على UPS Module 3S) ===")
    print(f"  العنوان {hex(INA219_ADDR)} على i2c-{INA219_I2C_BUS} · "
          f"المراقبة مفعّلة={BATTERY_MONITOR_ENABLED}")

    ina = INA219Reader()
    if not ina.ok:
        print(f"\n  ⛔ {ina.error}")
        print(f"     افحص: i2cdetect -y {INA219_I2C_BUS}   (توقّع "
              f"{hex(INA219_ADDR)} في الجدول)")
        print("     إن ظهر على عنوان آخر، عدّل INA219_ADDR في pi/config.py.")
        return 1
    print("  ✅ استجاب وكُتب سجل الإعدادات 0x399F")

    # ── (1) قراءة واحدة ─────────────────────────────────────────
    r = ina.read()
    if r["v"] is None:
        print(f"\n  ⛔ أول قراءة مرفوضة: {r['reason']}")
        return 1
    b = batt.classify(r["v"])
    print(f"\n[1/4] القراءة: **{b['v']}V** · {b['cell_v']}V/خلية · "
          f"~{b['percent']}% · {b['text']}")
    print(f"      raw={hex(r['raw'])} · CNVR={r['cnvr']} · OVF={r['ovf']}")
    check("الجهد داخل مدى حزمة 3S المعقول", 9.0 <= r["v"] <= 12.8,
          f"{r['v']:.2f}V")
    check("جهد الخلية داخل مدى ليثيوم سليم (3.0–4.25V)",
          3.0 <= b["cell_v"] <= 4.25, f"{b['cell_v']}V/خلية")

    # ── (2) الثبات ساكناً ───────────────────────────────────────
    print(f"\n[2/4] الثبات ({a.seconds:.0f}ث، ساكن)…")
    st = sample(ina, a.seconds)
    print(f"      متوسط={st['mean']:.3f}V · مدى={st['min']:.3f}–{st['max']:.3f} · "
          f"σ={st['stdev']:.4f}V · مرفوضة={st['rejects']}/{st['n']}")
    check("القراءة ثابتة (σ < 0.05V) — يُبنى عليها قرار",
          st["stdev"] < 0.05, f"σ={st['stdev']:.4f}V")

    # ── (3) بت التجاوز ──────────────────────────────────────────
    print(f"\n[3/4] بت التجاوز: {st['ovf']} من {st['n']} قراءة")
    check("لا تجاوز متكرر (OVF نادر أو معدوم)",
          st["ovf"] <= max(1, st["n"] // 100), f"{st['ovf']} OVF")
    check("نسبة الرفض الكلية منخفضة",
          st["rejects"] <= max(1, st["n"] // 50),
          f"{st['rejects']}/{st['n']}")

    # ── (4) 🔴 الهبوط تحت الحمل ─────────────────────────────────
    if not a.load:
        print("\n[4/4] ⏭ تخطّي اختبار الحمل (أضف --load بعد رفع العجلات)")
        print("      ⚠ بدونه لا نعرف إن كانت عتبة 10.2V ستُطلق عودة كاذبة "
              "كلما تحرّك الروبوت.")
    elif ROVER_MODE != "real":
        print("\n[4/4] ⛔ يحتاج RMS_ROVER_MODE=real")
        _ok.append(False)
    else:
        print(f"\n[4/4] ⚠ **ارفع العجلات عن الأرض** — محركات بقوة "
              f"{min(a.power, MAX_MOTOR_POWER):.2f}")
        input("      اضغط Enter حين تكون العجلات في الهواء… ")
        from pi.rover.bridge import WaveRoverBridge
        rover = WaveRoverBridge(mode="real")
        idle = sample(ina, 3.0)
        ld = sample(ina, a.seconds, rover=rover,
                    power=min(a.power, MAX_MOTOR_POWER))
        rover.stop()
        sag = (idle["mean"] - ld["min"]) if (idle["mean"] and ld["min"]) else None
        print(f"      ساكناً={idle['mean']:.3f}V · تحت الحمل: "
              f"متوسط={ld['mean']:.3f}V أدنى={ld['min']:.3f}V")
        print(f"      🔴 **الهبوط الأقصى = {sag:.3f}V**")
        margin = (idle["mean"] or 0) - BATT_LOW_V
        print(f"      الفارق إلى عتبة العودة ({BATT_LOW_V}V) = {margin:.2f}V")
        if sag is not None and sag >= margin:
            print("      🔴 الهبوط **يبتلع الفارق**: أول شوط محركات سيُطلق "
                  "عودة إجبارية كاذبة. الحلول: تنعيم القراءة (متوسط متحرك) "
                  "أو اشتراط تكرار القراءة المنخفضة قبل الإجراء.")
        check("الهبوط تحت الحمل لا يبتلع الفارق إلى عتبة العودة",
              sag is not None and sag < margin,
              f"هبوط {sag:.2f}V مقابل فارق {margin:.2f}V")
        check("لا رفض قراءات تحت الحمل (ضجيج المحركات لا يفسد I2C)",
              ld["rejects"] <= max(1, ld["n"] // 50),
              f"{ld['rejects']}/{ld['n']}")

    # ── (5) 🔴 إشارة التيار — تُقاس بالمحركات لا بالشاحن ────────
    if a.sign:
        print("\n[5] 🔴 قياس إشارة التيار")
        if ROVER_MODE != "real":
            print("      ⛔ يحتاج RMS_ROVER_MODE=real")
            _ok.append(False)
        else:
            s = measure_sign(ina, a.seconds, min(a.power, MAX_MOTOR_POWER))
            check("إشارة التيار حُسمت (لا تخمين)", s != 0,
                  f"INA219_CURRENT_SIGN = {s:+d}" if s else "لم تُحسم")
    elif not INA219_CURRENT_SIGN:
        print("\n[5] ⚠ إشارة التيار **غير معايرة** — النظام لا يدّعي شحناً "
              "(آمن) لكنه لا يعرفه أيضاً.")
        print("      قِسها: RMS_ROVER_MODE=real python3 -m pi.tests.test_ina219 "
              "--sign   (ارفع العجلات)")

    print(f"\n  العتبات الفاعلة: ممتاز>{BATT_EXCELLENT_V} · جيد>{BATT_GOOD_V} · "
          f"عودة<{BATT_LOW_V} · إيقاف<{BATT_CRITICAL_V}")
    print(f"\n=== النتيجة: {sum(_ok)}/{len(_ok)} نجح ===")
    return 0 if all(_ok) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nأُلغي.")
        sys.exit(130)
