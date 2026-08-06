#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_wall_follow.py — أداة خارجية: 🔴 القياس الحاسم — سير 3م بمحاذاة جدار
==========================================================================
البند (هـ) من خطة اختبار العتاد: نفس المسار مرتين، **بلا تصحيح** ثم
**بتصحيح الاتجاه من الجدار الجانبي** — والفرق بين الانحرافين الجانبيين
هو قيمة الألترا سونيك الجانبي كله.

    python3 -m pi.tests.test_wall_follow --side right              # بلا تصحيح
    python3 -m pi.tests.test_wall_follow --side right --correct    # بالتصحيح

**التجهيز الفيزيائي** (متطابق بين الجولتين — وإلا فالمقارنة باطلة):
  1. جدار مستقيم بطول ≥ 3.5م بلا فتحات ولا أثاث بارز.
  2. ضع الروبوت موازياً للجدار على بعد 30–60سم (النطاق الصالح للتصحيح
     WALL_FOLLOW_MIN/MAX = 15–120سم)، وعلى نفس علامة البداية في الجولتين.
  3. الطريق الأمامي خالٍ ≥ 4م. ابقَ بجانب مفتاح البطارية طوال التشغيل —
     طبقة IR **غير مفعّلة** في هذا السكربت (الأمامي وحده يحرس).
  4. علّم على الأرض موضع بداية الروبوت — لقياس الانحراف بشريط أيضاً
     كشاهد مستقل عن الحسّاس نفسه.

**ماذا يفعل**: يقيس المسافة الجانبية العمودية، يسير `--distance` على
أشواط `--segment` بتثبيت الاتجاه (نفس متحكّم الإنتاج `DriveExecutor`)،
وبعد كل شوط يقرأ الجانب ويغذّي `WallHeadingCorrector`. مع `--correct`
يُطبَّق التصحيح المقترح على الشوط التالي (بآلية §1.1.2: خطأ صغير يُغلق
أثناء السير لا بلفّة)؛ بدونه يُسجَّل الاقتراح **دون تطبيق** — فتتحقق من
إشارته يدوياً قبل أن تأتمنه على المحركات.

**القيم المتوقَّعة** (3م):
    بلا تصحيح : انحراف جانبي يتراكم مع انحراف الجايرو — سنتيمترات إلى
                عشرات، ويكبر مع طول المسافة.
    بالتصحيح  : |Δ| ≤ ~5سم ويستقر (لا يتراكم).
    🔴 إن كبُر الانحراف **بالتصحيح** عن «بلا تصحيح» ⇒ إشارة معكوسة
       (تغذية راجعة موجبة) — أوقف فوراً وراجع جهة `--side` قبل أي شيء.

⚠ يعمل دون تفعيل `SIDE_ULTRASONIC_ENABLED` (يبني المصفوفة بـ enabled=True).
⚠ حارس انجراف: يُجهض السير إذا تغيّرت المسافة الجانبية عن البداية أكثر
   من `--abort-drift-cm` (افتراضي 30سم) أو اقترب الجدار الأمامي.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

from pi.config import (
    WALL_FOLLOW_MIN_CM, WALL_FOLLOW_MAX_CM, STRAIGHT_BASE_POWER,
    GYRO_BIAS_CALIB_S, PERIMETER_WALL_SIDE, MAX_MOTOR_POWER,
)
from pi.nav.calibration import CalibrationStore, CalibrationProfile
from pi.nav.executor import DriveExecutor
from pi.nav.motion_check import NO_MOTION
from pi.nav.reactive import ReactiveSafety
from pi.nav.wall_heading import WallHeadingCorrector
from pi.rover.bridge import WaveRoverBridge
from pi.sensors.ultrasonic_array import UltrasonicArray

# ثوابت السكربت (أداة قياس — لا منطق مهمة)
SIDE_SETTLE_S = 1.2            # انتظار بعد التوقف قبل قراءة الجانب (خمود دورة تناوب)
SIDE_SAMPLE_N = 5              # وسيط عدة قراءات جانبية لا قراءة واحدة
FRONT_MARGIN_CM = 45.0         # هامش أمامي فوق طول الشوط قبل كل انطلاقة


def fallback_profile() -> CalibrationProfile:
    """ملف سرعة من العلاقة المقاسة (السرعة ≈ 1.5 × القوة داخل 0.1–0.5)."""
    return CalibrationProfile(
        name="مؤقت-1.5x",
        speeds={str(int(p * 100)): round(1.5 * p, 3)
                for p in (0.1, 0.2, 0.3, 0.4, 0.5)},
        turn_rate_dps=90.0, note="علاقة 1.5× المقاسة — لا ملف معايرة أرضية")


def side_median_cm(arr: UltrasonicArray, side: str):
    """وسيط عدة قراءات **عمودية** للجانب — None إن غاب الجدار."""
    vals = []
    t0 = time.time()
    while len(vals) < SIDE_SAMPLE_N and time.time() - t0 < 3.0:
        v = arr.perpendicular_cm(side)
        if v is not None:
            vals.append(v)
        time.sleep(0.2)                # ≥ دورة تناوب — قراءة جديدة فعلاً
    return round(statistics.median(vals), 2) if len(vals) >= 3 else None


def main() -> int:
    ap = argparse.ArgumentParser(description="سير 3م بمحاذاة جدار — بتصحيح وبدونه")
    ap.add_argument("--side", choices=("right", "left"),
                    default=PERIMETER_WALL_SIDE, help="جهة الجدار")
    ap.add_argument("--correct", action="store_true",
                    help="تطبيق تصحيح الاتجاه (بدونه: تسجيل الاقتراح فقط)")
    ap.add_argument("--distance", type=float, default=3.0)
    ap.add_argument("--segment", type=float, default=0.5)
    ap.add_argument("--power", type=float, default=STRAIGHT_BASE_POWER)
    ap.add_argument("--profile", default=None, help="اسم ملف معايرة أرضية")
    ap.add_argument("--abort-drift-cm", type=float, default=30.0)
    ap.add_argument("--bias-s", type=float, default=GYRO_BIAS_CALIB_S)
    a = ap.parse_args()
    if not (0.0 < a.power <= MAX_MOTOR_POWER):
        sys.exit(f"خطأ: القوة {a.power} خارج (0, {MAX_MOTOR_POWER}] — "
                 f"فوق 0.5 التفاف فيرموير صامت (CLAUDE.md §2.1)")

    # ── الجسر ومصدر الاتجاه ─────────────────────────────────────
    # ⚠ فتح المنفذ التسلسلي لا يثبت أن الروفر مُشغَّل: الكتابة على UART تنجح
    #    وإن كان ESP32 مطفأً، فتتكامل الأودومترية على روبوت ساكن. الحارس
    #    الفعلي هو حكم التحقق من الحركة بعد أول شوط (أدناه).
    print("⚠ تأكّد أن مفتاح هيكل Wave Rover مُشغَّل — بدونه تُرسل الأوامر "
          "إلى الفراغ بلا أي خطأ.")
    rover = WaveRoverBridge(mode="real")
    if rover.mode != "real":
        sys.exit(f"خطأ: الجسر لم يفتح المنفذ الحقيقي ({rover.error}) — "
                 f"هذا قياس عتاد لا محاكاة.")
    print(f"معايرة انحياز الجايرو {a.bias_s:.0f}ث — الروبوت ساكن تماماً…")
    rover.calibrate_gyro_bias(seconds=a.bias_s)
    hs = rover.heading_source
    if not getattr(hs, "ok", False):
        sys.exit(f"خطأ: مصدر الاتجاه غير صالح ({getattr(hs, 'error', '؟')}) — "
                 f"لا سير بلا اتجاه (CLAUDE.md §1.2)")

    # ── المصفوفة (تجاوز صريح للعلم — القياس هو ما يقرّر تفعيله) ──
    arr = UltrasonicArray(enabled=True)
    if not arr.ok or a.side not in arr.channels:
        sys.exit(f"خطأ: قناة {a.side} غير جاهزة — {arr.error}")
    arr.moving_hint = False

    # ── ملف السرعة ──────────────────────────────────────────────
    store = CalibrationStore()
    profile = None
    if a.profile:
        profile = store.load(a.profile)
    elif store.any_exists():
        profile = store.load(store.list_names()[0])
    if profile is None:
        profile = fallback_profile()
        print(f"⚠ لا ملف معايرة أرضية — أستعمل {profile.name} "
              f"(سرعة {profile.speed_for_power(int(a.power*100)):.2f} م/ث "
              f"عند {a.power:.2f})")

    def sensors() -> dict:
        # الأمامي من المصفوفة نفسها؛ IR غير موصول في هذا السكربت عمداً —
        # قيمة 1 = «خالٍ» لسلّم السرعة، والحارس البشري بجانب المفتاح.
        return {"ultrasonic_cm": arr.distance_cm("front"),
                "ir_left": 1, "ir_right": 1}

    ex = DriveExecutor(rover, ReactiveSafety(), sensors, profile)
    wh = WallHeadingCorrector(side=a.side)

    n_seg = max(1, round(a.distance / a.segment))
    mode = "بالتصحيح ✅" if a.correct else "بلا تصحيح (تسجيل فقط)"
    print(f"\nسير {a.distance}م على {n_seg} شوط × {a.segment}م · "
          f"جدار {a.side} · {mode}\n")

    d0 = side_median_cm(arr, a.side)
    if d0 is None or not (WALL_FOLLOW_MIN_CM <= d0 <= WALL_FOLLOW_MAX_CM):
        sys.exit(f"خطأ: المسافة الجانبية الابتدائية {d0} خارج النطاق الصالح "
                 f"[{WALL_FOLLOW_MIN_CM}, {WALL_FOLLOW_MAX_CM}]سم — "
                 f"ضع الروبوت 30–60سم من الجدار موازياً له.")
    print(f"  البداية: جانب⊥ = {d0}سم · heading = {hs.heading:.1f}°")

    rows = []
    odom = 0.0
    heading_err = 0.0
    applied = 0
    aborted = None
    try:
        for i in range(1, n_seg + 1):
            front = arr.distance_cm("front")
            need = a.segment * 100.0 + FRONT_MARGIN_CM
            if front is not None and front < need:
                aborted = f"جدار أمامي {front:.0f}سم < المطلوب {need:.0f}سم"
                break
            arr.moving_hint = True
            r = ex.forward_cell(a.segment, heading_error_deg=heading_err)
            arr.moving_hint = False
            heading_err = 0.0
            if not r["ok"]:
                aborted = f"شوط {i} أُجهض: {r.get('aborted') or r.get('reason')}"
                break
            # 🔴 حكم التحقق من الحركة — CLAUDE.md §2.2: الأودومترية تقدير
            #    مفتوح الحلقة، وقد ادّعت جولة كاملة 3.11م على روبوت **ساكن**
            #    (مقاس 2026-08-06: الروفر مطفأ والسكربت أكمل بلا إنذار).
            mv = r.get("motion") or {}
            if mv.get("verdict") == NO_MOTION:
                aborted = (f"لا حركة متحقَّقة في الشوط {i} — الروبوت لم "
                           "يتحرك فعلاً (روفر مطفأ؟ كابل UART؟ عجلة عالقة؟). "
                           f"شاهد التحقق: {mv.get('reason', '؟')}")
                break
            odom += r["covered_m"]
            time.sleep(SIDE_SETTLE_S)
            d = side_median_cm(arr, a.side)
            dec = wh.feed(d, odom)
            hh = r.get("heading_hold") or {}
            row = {"seg": i, "odom_m": round(odom, 2), "side_cm": d,
                   "drift_cm": (None if d is None or d0 is None
                                else round(d - d0, 1)),
                   "tilt_deg": dec.get("tilt_deg"),
                   "suggest_deg": dec.get("correction_deg", 0.0),
                   "apply": bool(dec.get("apply")), "why": dec["reason"],
                   "hold_err_deg": hh.get("final_error_deg"),
                   "sat_pct": hh.get("saturated_pct")}
            rows.append(row)
            print(f"  شوط {i}/{n_seg}: أودو={row['odom_m']}م · "
                  f"جانب⊥={d}سم (Δ={row['drift_cm']}سم) · "
                  f"اقتراح={row['suggest_deg']:+.2f}° [{row['why']}]"
                  + (" ← طُبّق" if (a.correct and row["apply"]) else ""))
            if a.correct and dec.get("apply"):
                heading_err = dec["correction_deg"]
                applied += 1
                wh.reset()             # الأدلة استُهلكت — كما في مسار المهمة
            if d is not None and abs(d - d0) > a.abort_drift_cm:
                aborted = (f"انجراف جانبي {d - d0:+.0f}سم تجاوز الحدّ "
                           f"±{a.abort_drift_cm:.0f}سم")
                break
    finally:
        rover.stop()                   # ⚠ إيقاف مضمون مهما حدث
        arr.close()

    # ── الخلاصة ─────────────────────────────────────────────────
    print("\n=== الخلاصة ===")
    valid = [r for r in rows if r["drift_cm"] is not None]
    if aborted:
        print(f"  ⛔ أُجهض: {aborted}")
    if not valid:
        print("  ❌ لا قراءات جانبية صالحة — لا حكم.")
        return 1
    final = valid[-1]
    max_abs = max(abs(r["drift_cm"]) for r in valid)
    print(f"  الوضع: {mode} · قُطع {final['odom_m']}م من {a.distance}م "
          f"(وصلة الروفر: {'سليمة' if rover.link_ok else '⛔ منقطعة'})")
    print(f"  الانحراف الجانبي النهائي: {final['drift_cm']:+.1f}سم · "
          f"الأقصى: {max_abs:.1f}سم")
    print(f"  تصحيحات مطبَّقة: {applied} · "
          f"مرفوضات المصحّح: {wh.rejects or 'لا شيء'}")
    if not a.correct:
        pos = [r["suggest_deg"] for r in valid if r["apply"]]
        if pos:
            print(f"  اقتراحات كانت ستُطبَّق: {[f'{v:+.2f}°' for v in pos]}")
            print("  🔴 تحقّق يدوياً قبل جولة --correct: الروبوت انحرف "
                  + ("نحو الجدار" if final["drift_cm"] < 0 else "بعيداً عنه")
                  + f" — فهل إشارة الاقتراح تعيده؟ (جدار {a.side}: "
                  "ابتعاد ⇒ تصحيح " + ("موجب/يمين)" if a.side == "right"
                                       else "سالب/يسار)"))
    print("\n  قِس أيضاً بشريط المتر إزاحة الروبوت عن علامة البداية — "
          "شاهد مستقل عن الحسّاس نفسه.")
    print("  المقارنة المطلوبة: |Δنهائي| بالتصحيح ≪ بلا تصحيح، "
          "وإلا فالإشارة معكوسة أو σ الحسّاس أكبر من المدّعى.")
    return 0 if not aborted else 1


if __name__ == "__main__":
    sys.exit(main())
