#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_heading.py — معايرة الاتجاه على العتاد مع MPU-6050 (البند 1)
====================================================================
⚠ **كل معايرة اتجاه سابقة باطلة**: أُخذت بجايرو الروفر الداخلي (مات مع
   Wave Rover) أو بـBNO055 (تلفت §0). المصدر الحيّ وحدة **MPU على
   `/dev/i2c-4` @0x68** بمعدل قراءة آخر — لا تنسخ رقماً من جدول قديم،
   قِس كل شيء من جديد هنا.

ثلاث مراحل، كل واحدة تُشغَّل وحدها أو الثلاث بالتسلسل:

  1) **الانحياز والضجيج** (بلا حركة): يقيس bias وσ ومعدل القراءة الفعلي،
     ويقترح `HEADING_DEADBAND_DPS = 3σ`. يفحص الوحدات أيضاً.
  2) **GYRO_SCALE**: لفّات بزاوية معلومة، تقيس الفعلي بعينك وتُدخله.
     ⚠ للـ**مقدار** استعمل 180° فأكثر (خطأ العين النسبي أصغر). والـ90°
     تخصّ فحص **الجهة** وحدها — انظر `check_directions`.
  3) **KP و MAX_CORR**: أشواط سير مستقيم بمرشّحي كسب مختلفة، ويُقاس لكل
     شوط: خطأ الاتجاه (متوسط/أقصى/نهائي)، تذبذب، نسبة إشباع التصحيح.

    python3 -m pi.tests.calibrate_heading                    # الثلاث مراحل
    python3 -m pi.tests.calibrate_heading --stage 1
    python3 -m pi.tests.calibrate_heading --stage 2 --angle 180 --trials 3
    python3 -m pi.tests.calibrate_heading --stage 3 --duration 4

⚠ السلامة:
  - المحركات تُوقَف في `finally` وعند Ctrl-C في كل مسار.
  - لا شوط سير إلا بمسار خالٍ ≥3م؛ الألترا سونيك (إن توفّر) يُجهض الشوط
    عند STOP_CM.
  - **مراقبة الجهد معطّلة الآن** → ابدأ ببطارية مشحونة، والحماية زمنية
    (MISSION_TIME_LIMIT_S في المهمة؛ هذا السكربت أشواطه ثوانٍ).
  - كل قوة تمرّ عبر `bridge.motors()` فتُحكم بـMAX_MOTOR_POWER.

النتائج تُحفظ في `docs/results/heading_calibration_<ts>.json` وتُطبع كأسطر
config جاهزة للّصق — **لا يُعدّل config تلقائياً** (قيمة مقاسة تُراجَع بعين
بشرية قبل أن تصبح ثابتاً).
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time

from pi.config import (
    ROVER_MODE, HEADING_SOURCE, GYRO_SCALE,
    HEADING_DEADBAND_DPS, HEADING_KP, HEADING_KD, HEADING_MAX_CORR,
    STRAIGHT_BASE_POWER, MOTOR_TRIM_L, MOTOR_TRIM_R, MAX_MOTOR_POWER,
    HEADING_HOLD_LOOP_S, MPU6050_READ_PERIOD_S, GYRO_BIAS_CALIB_S,
    STOP_CM, BATTERY_MONITOR_ENABLED, MISSION_TIME_LIMIT_S,
    MPU6050_ADDR, MPU6050_I2C_BUS,
)
from pi.nav.heading_hold import HeadingController, signed_error, available_headroom, config_sanity
from pi.rover.bridge import WaveRoverBridge
from pi.sensors.heading import robust_bias

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
RESULTS_DIR = os.path.join(_REPO_ROOT, "docs", "results")

# فوق هذه النسبة من الدورات يكون التصحيح ملتصقاً بالسقف، فالشوط يقيس السقف
# لا الكسب ولا يصلح للترشيح (انظر التعليق في stage3_gains).
SATURATION_LIMIT_PCT = 50.0


# ═══ أدوات ═══════════════════════════════════════════════════════
def _pctl(values, p: float) -> float:
    """المئين p من قائمة (استيفاء خطي بسيط — لا نستدعي numpy على الراسبري)."""
    if not values:
        return 0.0
    s = sorted(values)
    k = (len(s) - 1) * (p / 100.0)
    lo, hi = int(k), min(int(k) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


# ⚠ النص العربي يُطبع بـ`print` **قبل** `input()` ولا يُمرَّر محثّاً له:
# خلط RTL/LTR داخل محثّ input() يربك readline في حساب موضع المؤشّر فيحقن
# بايتات في مخزن الدخل → `UnicodeDecodeError: byte 0xd8` يُسقط السكربت **بعد**
# أن يكون المستخدم قد لفّ الروبوت وقاس بالمنقلة — قياس يدوي يضيع بلا رجعة.
def _ask_float(prompt: str, interactive: bool = True):
    """يقرأ رقماً من المستخدم؛ Enter فارغ = تخطّي (None)."""
    if not interactive:
        return None
    print(prompt)
    try:
        # ⚠ المحثّ **ASCII حصراً**: «[رقم]» العربية هنا أعادت نفس العلّة
        # (UnicodeDecodeError يبتلع قياساً يدوياً) — لا عربية في محثّ input.
        raw = input("[num] ").strip().replace("،", ".")
    except UnicodeDecodeError:
        print("  ⚠ تعذّرت قراءة الإدخال (ترميز) — تُخطّى.")
        return None
    except (EOFError, KeyboardInterrupt):
        return None
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        print("  ⚠ ليس رقماً — تُخطّى.")
        return None


def _pause(msg: str, interactive: bool = True) -> None:
    if not interactive:
        return
    print(msg)
    try:
        input("[Enter] ")
    except UnicodeDecodeError:
        return                        # إدخال تالف = تابع (لا تُسقط الجلسة)
    except (EOFError, KeyboardInterrupt):
        raise KeyboardInterrupt


def _front_cm(ultrasonic):
    return None if ultrasonic is None else ultrasonic.distance_cm


def _open_ultrasonic():
    """حاجز أمان للأشواط — غيابه لا يمنع المعايرة لكنه يُعلن بوضوح."""
    try:
        from pi.sensors.proximity import UltrasonicReader
        u = UltrasonicReader()
        if u.ok:
            return u, None
        return None, u.error
    except Exception as e:                    # noqa: BLE001
        return None, str(e)


# ═══ المرحلة 1: الانحياز والضجيج ومعدل القراءة ═══════════════════
def stage1_bias(rover, seconds: float, interactive: bool = True) -> dict:
    src = rover.heading_source
    print(f"\n=== 1) الانحياز والضجيج — المصدر «{src.name}» ===")
    print(f"⚠ لا تلمس الروبوت ولا الطاولة {seconds:.0f} ثوانٍ (السكون شرط الصحة).")
    _pause("اضغط Enter للبدء… ", interactive)
    rover.stop()

    samples, deadline = [], time.time() + seconds
    t0 = time.time()
    while time.time() < deadline:
        v = src._read_rate_dps() if src.kind == "rate" else src._read_angle_deg()
        if v is not None:
            samples.append((time.time(), float(v)))
        time.sleep(MPU6050_READ_PERIOD_S)
    elapsed = time.time() - t0
    if len(samples) < 10:
        return {"ok": False, "reason": f"عينات غير كافية ({len(samples)}) — تحقّق من الحسّاس",
                "samples": len(samples)}

    if src.kind == "angle":
        # مصدر **زاوية مطلقة**: «انحياز» الزاوية نفسها بلا معنى — المقاس هو
        # **الانجراف**: فرق الزاوية الملفوف ÷ الزمن بين قراءتين.
        raw = []
        for (t_prev, a_prev), (t_now, a_now) in zip(samples, samples[1:]):
            dt = t_now - t_prev
            if dt > 0:
                raw.append(((a_now - a_prev + 180.0) % 360.0 - 180.0) / dt)
    else:
        raw = [v for _, v in samples]
    if len(raw) < 10:
        return {"ok": False, "reason": "عينات غير كافية بعد التحويل",
                "samples": len(raw)}

    hz = len(samples) / elapsed if elapsed > 0 else 0.0

    # ⚠ حارس الحسّاس الميت — **قبل** أي اقتراح معامل. هذه المرحلة تقرأ
    # `_read_rate_dps()` مباشرة فتتجاوز حارس الصفر المضبوط في `update()`،
    # وσ=0 يبدو «معايرة مثالية». النتيجة المشاهَدة: جايرو ميت أعطى انحياز
    # 0.0000 وσ 0.0000 فأُعلن «سليم»، ثم تحرّكت المحركات في المرحلة 2 حتى
    # أطلق حارس update() الإجهاض بعد ~1.3ث (وقد دار الروبوت ~95° عمياناً).
    if all(v == 0.0 for v in raw):
        print(f"\n  ⛔ كل العينات ({len(raw)}) **صفر مضبوط** — الحسّاس لا يرسل.")
        print(f"     المصدر الفعلي «{src.name}» بينما المطلوب {HEADING_SOURCE}.")
        if src.name != "mpu6050":
            print("     ⚠ سقط المصنع إلى مصدر بديل — ومصادر Wave Rover/BNO055 "
                  "لقطع لم تعد على العتاد (§0)، فقراءاتها بلا معنى.")
        print(f"     افحص: i2cdetect -y {MPU6050_I2C_BUS}  "
              f"(يجب أن يظهر {hex(MPU6050_ADDR)[2:]})، ثم "
              f"python3 -m pi.tests.check_imu_health")
        print("     ⚠ لا تلصق أي رقم من هذا التشغيل في config — كله باطل.")
        return {"ok": False, "reason": "كل العينات صفر مضبوط — حسّاس ميت",
                "source": src.name, "samples": len(raw), "rate_hz": round(hz, 1),
                "dead_sensor": True}
    info = robust_bias(raw, max_std=src.max_bias_std)
    resid = [v - info["bias"] for v in raw]
    sigma = statistics.pstdev(resid) if len(resid) > 1 else 0.0
    peak = max(abs(v) for v in resid)
    suggested_deadband = round(3.0 * sigma + 0.05, 2)

    out = {"ok": info["ok"], "source": src.name, "kind": src.kind,
           "samples": len(raw), "rate_hz": round(hz, 1),
           "period_ms": round(1000.0 / hz, 1) if hz else None,
           "bias_dps": round(info["bias"], 4), "sigma_dps": round(sigma, 4),
           "peak_resid_dps": round(peak, 3), "rejected": info["rejected"],
           "min": round(min(raw), 3), "max": round(max(raw), 3),
           "suggested_deadband_dps": suggested_deadband,
           "current_deadband_dps": HEADING_DEADBAND_DPS,
           "bias_reason": info.get("reason")}

    label = "الانجراف" if src.kind == "angle" else "الانحياز"
    print(f"  عينات {len(samples)} في {elapsed:.1f}ث → **{hz:.1f} قراءة/ث** "
          f"({out['period_ms']}ms للقراءة)")
    print(f"  {label} = {info['bias']:+.4f}°/ث "
          f"({info['bias'] * 60.0:+.2f}°/دقيقة)   σ = {sigma:.4f}   "
          f"أقصى شذوذ = {peak:.3f}")
    print(f"  المدى الخام: {min(raw):+.3f} … {max(raw):+.3f}")
    if src.kind == "rate":
        print(f"  → عتبة السكون المقترحة (3σ): **{suggested_deadband}** "
              f"(الحالية {HEADING_DEADBAND_DPS})")
        # الانحياز المقاس **يُطبَّق فوراً** على المصدر: المراحل التالية تعتمده،
        # ولا معنى لقياسه ثم إهماله (يُقاس مجدداً عند بدء كل مهمة).
        src.bias = info["bias"]
        src.bias_calibrated = bool(info["ok"])
        src.bias_info = info
    else:
        out["suggested_deadband_dps"] = None
        print("  (مصدر زاوية مطلقة: لا عتبة سكون ولا انحياز — المهم الانجراف "
              "أعلاه: أقل من ~1°/دقيقة مقبول لمهمة دقائق.)")
    if not info["ok"]:
        print(f"  ⚠ تشتت كبير: {info['reason']} — أعد القياس على سطح ثابت.")

    # فحص الوحدات: الحسّاس الساكن يجب أن يعطي أعشار الدرجة/ث لا راديان/ث
    if src.kind == "rate" and abs(info["bias"]) > 20.0:
        print("  ⚠ انحياز ضخم (>20°/ث) والروبوت ساكن — احتمال اهتزاز أو "
              "خطأ وحدات. سائق MPU يقسّم الخام على 131 فيُخرج °/ث: "
              "افحص بـ check_imu_health قبل اعتماد أي رقم.")
        out["unit_warning"] = True
    st = src.state()
    print(f"  حالة الحسّاس: {st}")
    out["source_state"] = st
    return out


# ═══ المرحلة 2: معامل التحويل (GYRO_SCALE) ═══════════════════════
def stage2_scale(rover, angle: float, trials: int, interactive: bool = True) -> dict:
    src = rover.heading_source
    cur_scale = src.scale
    print(f"\n=== 2) معامل التحويل — {trials} لفّات × {angle:.0f}° ===")
    print("طريقة القياس (لفّة كبيرة أدق: خطأ العين النسبي أصغر):")
    if abs(angle) < 170.0:
        print("  ⚠ زاوية صغيرة: أدقّ معايرة للـ**مقدار** تكون بـ180° فأكثر.")
        print("     (الـ90° تلزم لفحص **الجهة** لا المقدار — لا تخلط بينهما:")
        print("      الـ180 تنتهي مواجهةً نفس الجهة يميناً ويساراً فتعمى عن")
        print("      الانعكاس، والـ90 خطؤها النسبي أكبر فتُضعف قياس المقدار.)")
    print("  • علّم اتجاه مقدّمة الروبوت على الأرض بشريط لاصق قبل اللفّ.")
    print("  • بعد التوقف علّم الاتجاه الجديد وقِس الزاوية بينهما بمنقلة.")
    print("  • أدخِل الزاوية **بإشارة موجبة إن كان الدوران بنفس الاتجاه المطلوب**.")
    if not rover.bias_calibrated:
        print("  (يقيس الانحياز أولاً…)")
        rover.calibrate_gyro_bias()

    imu = getattr(src, "imu", None)
    results = []
    for i in range(1, trials + 1):
        _pause(f"\n  [{i}/{trials}] مساحة دوران خالية؟ اضغط Enter للفّ "
               f"{angle:.0f}°… ", interactive)
        yaw_before = imu.euler_yaw() if imu is not None else None
        src.reset(0.0)
        t0 = time.time()
        res = rover.turn_by_angle(angle)
        dur = time.time() - t0
        yaw_after = imu.euler_yaw() if imu is not None else None
        fusion_delta = None
        if yaw_before is not None and yaw_after is not None:
            fusion_delta = (yaw_after - yaw_before + 180.0) % 360.0 - 180.0
            # اللفّات > 180° تلتفّ في الزاوية المطلقة — لا تُقارن بلا حذر
            if abs(angle) > 170.0:
                fusion_delta = None

        print(f"      قدّر المصدر: {res['turned_deg']:+.1f}°  "
              f"(مراحل {res['segments']}، {dur:.1f}ث"
              + (f"، مهلة!" if res["timed_out"] else "")
              + (f"، إجهاض: {res['aborted']}" if res.get("aborted") else "") + ")")
        if res.get("aborted") == "sign_mismatch":
            # 🔴 **لا تقلب إشارة الجايرو هنا** (القاعدة §2): الإشارة تُقاس
            #    باليد بلا محركات فهي مستقلة عن قطبية المحركات. قلبها
            #    «لتصحيح» لفّة معكوسة يعيد فخّ الخطأين المتلازمين — يُلغيان
            #    بعضهما فيصمت هذا الحارس نفسه، وعاش الخطأ شهراً بسببه.
            print("      ⛔ الروبوت دار **عكس** المطلوب.")
            print("         🔴 لا تلمس MPU6050_GYRO_Z_SIGN — الخطأ في خريطة")
            print("            المحركات. اشتقّها من جديد بمسبار المعالم:")
            print("              python3 -m pi.tests.probe_motor_map")
            print("         (وإشارة الجايرو تُقاس باليد بلا محركات:")
            print("              python3 -m pi.tests.calibrate_mpu6050 --only sign)")
            return {"ok": False, "reason": "sign_mismatch", "trials": results}
        if fusion_delta is not None:
            print(f"      وللمقارنة، yaw المدموج داخلياً: {fusion_delta:+.1f}°")

        actual = _ask_float("      الزاوية الفعلية المقاسة (درجة، Enter لتخطّي): ",
                            interactive)
        row = {"trial": i, "requested_deg": angle,
               "estimated_deg": res["turned_deg"], "actual_deg": actual,
               "fusion_delta_deg": (None if fusion_delta is None
                                    else round(fusion_delta, 1)),
               "duration_s": round(dur, 2), "segments": res["segments"],
               "timed_out": res["timed_out"], "scale_used": cur_scale}
        if actual and res["turned_deg"]:
            # المعامل الجديد = الحالي × (الفعلي ÷ المقدَّر)
            row["scale_needed"] = round(cur_scale * actual / res["turned_deg"], 4)
            if fusion_delta:
                row["fusion_ratio"] = round(actual / fusion_delta, 4)
            print(f"      → معامل هذه اللفّة: {row['scale_needed']}")
        results.append(row)

    scales = [r["scale_needed"] for r in results if r.get("scale_needed")]
    out = {"ok": bool(scales), "angle": angle, "trials": results,
           "current_scale": cur_scale}
    if scales:
        out["suggested_scale"] = round(statistics.median(scales), 4)
        out["spread"] = round(max(scales) - min(scales), 4)
        print(f"\n  → المعامل المقترح (وسيط {len(scales)} لفّات): "
              f"**{out['suggested_scale']}**  (تشتت {out['spread']})")
        if out["spread"] > 0.10:
            print("  ⚠ تشتت > 0.10 بين اللفّات: انزلاق عجلات أو قياس عين غير "
                  "دقيق — زد الزاوية أو أعد التجارب قبل اعتماد الرقم.")
        s = out["suggested_scale"]
        if s > 5 or s < 0.2:
            print(f"  ⚠ معامل شاذ ({s}) — الأرجح خطأ **وحدات** لا معايرة: "
                  f"≈57 يعني القراءة راديان/ث والمفترض درجة/ث (أو العكس ≈0.017). "
                  f"سائق MPU يقسّم على 131 ⇒ °/ث؛ افحص القسمة لا المعامل.")
        ratios = [r["fusion_ratio"] for r in results if r.get("fusion_ratio")]
        if ratios:
            out["fusion_ratio_median"] = round(statistics.median(ratios), 4)
            print(f"  للمقارنة: yaw المدموج يحتاج معامل "
                  f"{out['fusion_ratio_median']} — الأقرب إلى 1.0 هو المصدر "
                  f"الأصدق قبل المعايرة (HEADING_SOURCE).")
    else:
        print("\n  (لم تُدخل زوايا فعلية — لا اقتراح معامل.)")
    return out


# ═══ المرحلة 3: KP و MAX_CORR (تثبيت الاتجاه) ════════════════════
def _straight_run(rover, kp: float, kd: float, duration: float,
                  base: float, ultrasonic=None) -> dict:
    """
    شوط سير مستقيم واحد بتثبيت الاتجاه. الهدف heading = 0 (اتجاه البداية).
    ⚠ الأمر يُجدَّد كل دورة (حارس heartbeat 1.5ث يوقف المحركات بلا تجديد).
    """
    src = rover.heading_source
    ctl = HeadingController(kp=kp, kd=kd)
    src.set_phase("drive")
    src.reset(0.0)
    series = []
    aborted = None
    loops = 0
    t0 = time.time()
    last = t0
    try:
        while time.time() - t0 < duration:
            d = src.update()
            now = time.time()
            dt = now - last
            last = now
            if not src.ok:
                aborted = f"مصدر الاتجاه توقّف: {src.error}"
                break
            front = _front_cm(ultrasonic)
            if front is not None and front <= STOP_CM:
                aborted = f"عائق أمامي {front:.0f}سم ≤ {STOP_CM:.0f}"
                break
            err = signed_error(src.total_deg, 0.0)
            w = ctl.wheels(err, dt, base_power=base)
            rover.motors(w["left"], w["right"])
            series.append({"t": round(now - t0, 3), "err": round(err, 2),
                           "corr": w["correction"], "dps": d["dps"]})
            loops += 1
            time.sleep(HEADING_HOLD_LOOP_S)
    finally:
        rover.stop()                        # ⚠ إيقاف مضمون
    elapsed = time.time() - t0
    corrs = [abs(s["corr"]) for s in series]
    summ = ctl.summary()
    summ.update({
        "duration_s": round(elapsed, 2), "loops": loops,
        "loop_hz": round(loops / elapsed, 1) if elapsed > 0 else 0.0,
        "final_err_deg": round(series[-1]["err"], 2) if series else None,
        "corr_p95": round(_pctl(corrs, 95), 4),
        "corr_max": round(max(corrs), 4) if corrs else 0.0,
        "base_power": base, "aborted": aborted,
        "series": series[-400:],            # آخر 400 نقطة كدليل في الملف
    })
    return summ


def stage3_gains(rover, kps, kd: float, duration: float, base: float,
                 interactive: bool = True) -> dict:
    print(f"\n=== 3) KP و MAX_CORR — {len(kps)} أشواط × {duration:.0f}ث ===")
    san = config_sanity()
    head = available_headroom(base)
    print(f"  أساس القوة {base} · وزنية {MOTOR_TRIM_L:+}/{MOTOR_TRIM_R:+} · "
          f"الفراغ المتاح للتصحيح **{head:.3f}** (حدّ القوة {MAX_MOTOR_POWER})")
    if not san["ok"]:
        print(f"  ⚠ {san['reason']}")
    print("  ضع الروبوت في ممر خالٍ ≥3م، وعلّم موضع مقدّمته على الأرض؛ بعد كل "
          "شوط قِس **الانحراف الجانبي** عن الخط المستقيم بالسنتيمتر.")
    if not rover.bias_calibrated:
        rover.calibrate_gyro_bias()

    ultrasonic, us_err = _open_ultrasonic()
    print(f"  حاجز الألترا سونيك: " +
          (f"مفعّل (توقف عند {STOP_CM:.0f}سم)" if ultrasonic else f"غير متاح ({us_err})"))

    runs = []
    try:
        for kp in kps:
            _pause(f"\n  [KP={kp}] أعِد الروبوت لبداية الممر ثم Enter… ", interactive)
            print(f"  يسير {duration:.0f}ث بـKP={kp} KD={kd}…")
            r = _straight_run(rover, kp, kd, duration, base, ultrasonic)
            print(f"      خطأ نهائي {r['final_err_deg']}° · متوسط "
                  f"{r['mean_abs_error_deg']}° · أقصى {r['max_abs_error_deg']}° · "
                  f"تذبذب {r['sign_changes']} · إشباع {r['saturated_pct']}% · "
                  f"{r['loop_hz']} دورة/ث"
                  + (f" · ⚠ {r['aborted']}" if r["aborted"] else ""))
            r["lateral_dev_cm"] = _ask_float(
                "      الانحراف الجانبي المقاس (سم، Enter لتخطّي): ", interactive)
            r["distance_m"] = _ask_float(
                "      المسافة المقطوعة (م، Enter لتخطّي): ", interactive)
            runs.append(r)
    finally:
        rover.stop()
        if ultrasonic is not None:
            try:
                ultrasonic.close()
            except Exception:                # noqa: BLE001
                pass

    valid = [r for r in runs if not r["aborted"]]
    out = {"ok": bool(valid), "runs": runs, "headroom": round(head, 4),
           "base_power": base, "kd": kd}
    if not valid:
        print("\n  ⚠ كل الأشواط أُجهضت — لا اقتراح كسب.")
        return out

    # ⚠ **الإشباع يُبطل الشوط كقياس لـKP** (درس مقاس 2026-07-30): فوق
    #    SATURATION_LIMIT_PCT يكون التصحيح ملتصقاً بالسقف فلا يقيس الشوط الكسب
    #    بل يقيس السقف — KP=0.035 وKP=0.10 يعطيان السلوك نفسه. والأسوأ أن
    #    الالتصاق **يُصفّر sign_changes** (الإشارة لا تتبدّل وهي عند الحدّ)
    #    فيبدو الشوط «بلا تذبذب» وهو تحكّم قافز. الصيغة القديمة كافأت ذلك
    #    فرشّحت شوطاً بإشباع 72.7% على شوط انحرافه الأرضي أقل بأربعة أضعاف.
    for r in valid:
        osc = r["sign_changes"] / max(1e-6, r["duration_s"])      # تبديل/ث
        r["score"] = round(r["mean_abs_error_deg"] + 2.0 * osc, 3)
        r["saturated_out"] = r["saturated_pct"] > SATURATION_LIMIT_PCT
        # الانحراف الجانبي لكل متر — تطبيع ضروري: شوط انحرف 33سم في 1.6م
        # أسوأ من شوط انحرف 33سم في 3م.
        d = r.get("distance_m")
        r["dev_per_m"] = (round(r["lateral_dev_cm"] / d, 1)
                          if r.get("lateral_dev_cm") is not None and d else None)

    linear = [r for r in valid if not r["saturated_out"]]
    dropped = [r for r in valid if r["saturated_out"]]
    if dropped:
        print(f"\n  ⚠ استُبعدت من الترشيح (إشباع > {SATURATION_LIMIT_PCT}%): "
              + "، ".join(f"KP={r['kp']} ({r['saturated_pct']}%)" for r in dropped))
        print("     التصحيح ملتصق بالسقف فيها فلا تقيس الكسب، و«تذبذب 0» فيها")
        print("     أثر الالتصاق لا دليل استقرار.")
    pool = linear or valid          # لو أُشبعت كلها نرتّب على ما لدينا مع تحذير
    if not linear:
        print("  ⚠ **كل** الأشواط مُشبَعة — الفراغ المتاح ضيق. أنزِل "
              "STRAIGHT_BASE_POWER وأعد، فالترشيح أدناه غير موثوق.")

    # ⚠ **الانحراف الجانبي المقاس يعلو على كل مؤشّر داخلي**: الجايرو يقيس
    #    الاتجاه لا الموضع، فقد يحفظ الاتجاه بينما انزلق الروبوت جانبياً.
    #    لا نرتّب به إلا إذا قِيس في **كل** أشواط المجموعة (وإلا قارنّا
    #    مقيساً بغير مقيس — وهو ما رشّح شوطاً لم يُقَس انحرافه أصلاً).
    ground = [r for r in pool if r["dev_per_m"] is not None]
    if ground and len(ground) == len(pool):
        best = min(pool, key=lambda r: r["dev_per_m"])
        out["ranked_by"] = "الانحراف الجانبي المقاس (سم/م)"
    else:
        best = min(pool, key=lambda r: r["score"])
        out["ranked_by"] = "مؤشّر داخلي (خطأ + تذبذب)"
        if ground:
            print(f"\n  ⚠ الانحراف الجانبي قِيس في {len(ground)} من {len(pool)} "
                  f"أشواط فقط — الترتيب بالمؤشّر الداخلي. قِسه في **كلها** "
                  f"ليُرجَّح القياس الأرضي.")
    out["suggested_kp"] = best["kp"]
    out["best_run"] = {k: best[k] for k in
                       ("kp", "mean_abs_error_deg", "max_abs_error_deg",
                        "final_err_deg", "sign_changes", "saturated_pct",
                        "corr_p95", "corr_max", "loop_hz", "score")}
    # السقف: يغطّي 95% من التصحيح المطلوب فعلاً بهامش 1.5، مقصوصاً على الفراغ
    need = max(0.02, best["corr_p95"] * 1.5)
    out["suggested_max_corr"] = round(min(head, need), 3)
    print(f"\n  جدول الأشواط (الترتيب: {out['ranked_by']}):")
    for r in valid:
        print(f"    KP={r['kp']:<7} متوسط {r['mean_abs_error_deg']:>6}° "
              f"أقصى {r['max_abs_error_deg']:>6}° تذبذب {r['sign_changes']:>3} "
              f"إشباع {r['saturated_pct']:>5}% نقاط {r['score']}"
              + ("  ⛔مُشبَع" if r["saturated_out"] else "")
              + (f" · {r['dev_per_m']}سم/م" if r["dev_per_m"] is not None else "")
              + (f" · جانبي {r['lateral_dev_cm']}سم"
                 if r.get("lateral_dev_cm") is not None else ""))
    print(f"\n  → KP المقترح: **{out['suggested_kp']}** "
          f"(الحالي {HEADING_KP})")
    print(f"  → MAX_CORR المقترح: **{out['suggested_max_corr']}** "
          f"(الحالي {HEADING_MAX_CORR}، الفراغ المتاح {head:.3f})")
    if best["saturated_pct"] > 20:
        print("  ⚠ إشباع > 20% من الدورات: السقف يقيّد المتحكم — إما ترفع "
              "السقف (وتنزل بأساس القوة ليتّسع الفراغ) أو تخفّض KP.")
    if best["corr_p95"] >= head * 0.95:
        print("  ⚠ التصحيح يلامس الفراغ المتاح كله — أنزِل STRAIGHT_BASE_POWER "
              "قليلاً ليتّسع مجال التصحيح (الأساس يأكل من حدّ القوة 0.5).")
    return out


# ═══ التشغيل ═════════════════════════════════════════════════════
def _save(payload: dict) -> str:
    os.makedirs(RESULTS_DIR, exist_ok=True)
    path = os.path.join(RESULTS_DIR,
                        f"heading_calibration_{time.strftime('%Y%m%d_%H%M%S')}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return path


def _print_config_lines(payload: dict) -> None:
    print("\n" + "═" * 62)
    print("أسطر config الجاهزة (راجعها بعينك ثم الصقها في pi/config.py):")
    s1 = payload.get("stage1") or {}
    s2 = payload.get("stage2") or {}
    s3 = payload.get("stage3") or {}
    src = payload.get("source", "")
    # ⚠ لا يُقترح رقم من مرحلة فاشلة: قيمة مقيسة بحسّاس ميت تبدو ممتازة
    # (σ=0 → عتبة 0.05) وهي باطلة تماماً — طباعتها تغري بلصقها في config.
    if not (s1.get("ok") or s2.get("ok") or s3.get("ok")):
        print("  (لا اقتراح — لم تنجح أي مرحلة. صحّح الحسّاس أولاً ثم أعد.)")
        print("═" * 62)
        return
    if s1.get("ok") and s1.get("suggested_deadband_dps") is not None:
        print(f"  HEADING_DEADBAND_DPS = {s1['suggested_deadband_dps']}   "
              f"# 3σ مقاس ({s1['sigma_dps']}) على {src}")
    if s2.get("suggested_scale"):
        # 🔴 المصدر الوحيد mpu6050 (BNO055 تالفة §0) — الثابت الحيّ واحد
        print(f"  GYRO_SCALE = {s2['suggested_scale']}   "
              f"# وسيط {len(s2.get('trials', []))} لفّات × {s2.get('angle')}°")
    if s3.get("suggested_kp"):
        print(f"  HEADING_KP = {s3['suggested_kp']}       "
              f"# أدنى خطأ بلا تذبذب عند {s3['best_run']['loop_hz']} دورة/ث")
        print(f"  HEADING_MAX_CORR = {s3['suggested_max_corr']}   "
              f"# مئين 95 للتصحيح ×1.5، داخل الفراغ {s3['headroom']}")
    print("⚠ سجّل في التعليق: التاريخ، الأرضية، وحالة البطارية — القيم تتغيّر بها.")
    print("═" * 62)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="معايرة الاتجاه مع MPU-6050")
    ap.add_argument("--stage", type=int, choices=(1, 2, 3), action="append",
                    help="مرحلة محدّدة (تكرارها يجمع مراحل). الافتراضي: الثلاث")
    ap.add_argument("--seconds", type=float, default=GYRO_BIAS_CALIB_S,
                    help="مدة قياس الانحياز (المرحلة 1)")
    ap.add_argument("--angle", type=float, default=180.0,
                    help="زاوية اللفّة (المرحلة 2) — الكبيرة أدق")
    ap.add_argument("--trials", type=int, default=3, help="عدد اللفّات (المرحلة 2)")
    ap.add_argument("--duration", type=float, default=4.0,
                    help="مدة شوط السير بالثواني (المرحلة 3)")
    ap.add_argument("--base", type=float, default=STRAIGHT_BASE_POWER,
                    help="قوة أساس السير (المرحلة 3)")
    ap.add_argument("--kp", type=str, default="0.008,0.015,0.025,0.035",
                    help="مرشّحو KP مفصولين بفواصل (المرحلة 3)")
    ap.add_argument("--kd", type=float, default=HEADING_KD)
    ap.add_argument("--allow-sim", action="store_true",
                    help="⚠ للاختبار البرمجي فقط: يسمح بالتشغيل على جسر المحاكاة")
    ap.add_argument("--non-interactive", action="store_true",
                    help="بلا أسئلة (يشغّل المسارات فقط — لا يقترح معاملات)")
    a = ap.parse_args(argv)
    stages = sorted(set(a.stage or (1, 2, 3)))
    interactive = not a.non_interactive

    rover = WaveRoverBridge(mode=ROVER_MODE)
    src = rover.heading_source
    print("═" * 62)
    print(f"معايرة الاتجاه · جسر: {rover.mode} "
          f"(RMS_ROVER_MODE={ROVER_MODE}) · "
          f"مصدر: {src.name} (مطلوب {HEADING_SOURCE})")
    if rover.mode != "real":
        # 🔴 اسم متغيّر البيئة يُطبع كاملاً: `ROVER_MODE=real` (بلا البادئة)
        #    لا يفعل شيئاً، والسكربت يرفض بعدها بسطرين فيبدو الرفض بلا سبب.
        print("  ⚠ الجسر **محاكاة** — المراحل 2/3 تحتاج جسراً حقيقياً:")
        print("     RMS_ROVER_MODE=real python3 -m pi.tests.calibrate_heading")
    if rover.error:
        print(f"  ⚠ {rover.error}")
    if getattr(src, "fallback_reason", None):
        print(f"  {src.fallback_reason}")
    # ⚠ BNO055 قطعة ميتة — تُعرض للتوثيق لا للتعديل (§0)
    print(f"  المعامل الحيّ: GYRO_SCALE={GYRO_SCALE} (⚠ غير معاير على "
          f"Freenove) · KP={HEADING_KP} · KD={HEADING_KD} · "
          f"MAX_CORR={HEADING_MAX_CORR}")

    if not BATTERY_MONITOR_ENABLED:
        print(f"  ⚠ مراقبة الجهد معطّلة — ابدأ ببطارية مشحونة (الحماية زمنية: "
              f"{MISSION_TIME_LIMIT_S:.0f}ث في المهمة).")
    print("═" * 62)

    if not src.ok:
        print(f"⛔ مصدر الاتجاه غير سليم: {src.error}")
        print(f"   تحقّق: i2cdetect -y {MPU6050_I2C_BUS} (يجب أن يظهر "
              f"{hex(MPU6050_ADDR)[2:]})، وتغذية 3.3V، والأسلاك.")
        print("   والتشخيص الكامل: python3 -m pi.tests.check_imu_health")
        return 2
    motion = [s for s in stages if s in (2, 3)]
    if motion and rover.mode != "real" and not a.allow_sim:
        print("⛔ المراحل 2/3 تحرّك المحركات، والجسر في وضع sim → ستُنتج أرقاماً "
              "وهمية. شغّل: RMS_ROVER_MODE=real python3 -m pi.tests.calibrate_heading")
        return 2

    payload = {"ts": time.strftime("%Y-%m-%d %H:%M:%S"),
               "source": src.name, "requested_source": HEADING_SOURCE,
               "rover_mode": rover.mode, "stages": stages,
               "config_before": {
                   "GYRO_SCALE": GYRO_SCALE,
                   "HEADING_DEADBAND_DPS": HEADING_DEADBAND_DPS,
                   "HEADING_KP": HEADING_KP, "HEADING_KD": HEADING_KD,
                   "HEADING_MAX_CORR": HEADING_MAX_CORR,
                   "STRAIGHT_BASE_POWER": STRAIGHT_BASE_POWER,
                   "MOTOR_TRIM_L": MOTOR_TRIM_L, "MOTOR_TRIM_R": MOTOR_TRIM_R,
               }}
    try:
        if 1 in stages:
            payload["stage1"] = stage1_bias(rover, a.seconds, interactive)
        if 2 in stages:
            payload["stage2"] = stage2_scale(rover, a.angle, a.trials, interactive)
        if 3 in stages:
            kps = [float(x) for x in a.kp.split(",") if x.strip()]
            payload["stage3"] = stage3_gains(rover, kps, a.kd, a.duration,
                                             a.base, interactive)
    except KeyboardInterrupt:
        print("\n⛔ أُوقف بالمستخدم — المحركات متوقفة.")
        payload["interrupted"] = True
    finally:
        rover.stop()
        rover.close()

    path = _save(payload)
    _print_config_lines(payload)
    print(f"\nالنتائج الخام محفوظة في: {os.path.relpath(path, _REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
