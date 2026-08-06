#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_us_array.py — أداة خارجية: إحصاء الألترا سونيك منفرداً وبالتناوب
======================================================================
سكربت القياس للبندين (ب) و(ج) من خطة اختبار العتاد:

**الوضع المنفرد** (بند ب — كل حسّاس وحده + مقارنة بشريط قياس):
    python3 -m pi.tests.test_us_array --only front --seconds 20 --truth 50
    python3 -m pi.tests.test_us_array --only right --seconds 20 --truth 40
    python3 -m pi.tests.test_us_array --only left  --seconds 20 --truth 40
يقيس القناة المطلوبة وحدها (نبضة كل 60ms) ويُخرج: عدد النبضات، نسبة
الصالح، الوسيط، المتوسط، **σ**، والمسافة العمودية للجانبيين (تصحيح ميل
التركيب `US_SIDE_TILT_DEG`)، ومقارنة بالحقيقة الأرضية إن أُعطيت.

**وضع التناوب** (بند ج — الثلاثة معاً عبر مجدوِل الإنتاج نفسه):
    python3 -m pi.tests.test_us_array --seconds 30 \\
        --truth-front 60 --truth-right 40 --truth-left 45
يشغّل `UltrasonicArray` الحقيقي (خيط التناوب أمامي→أيمن→أيسر بفاصل
`US_ROUND_ROBIN_GAP_S`) — أي أن ما يُقاس هنا هو **نفس المسار** الذي
ستستعمله المهمة، لا إعادة تنفيذ.

⚠ `--truth` هي المسافة **العمودية** بشريط القياس من وجه الحسّاس إلى
   الجدار: للجانبيين تُقارن بالقيمة العمودية (خام × cos الميل)، وللأمامي
   بالخام مباشرة.
⚠ يعمل **دون** تفعيل `SIDE_ULTRASONIC_ENABLED` — يمرّر `enabled=True`
   للمصفوفة صراحةً، فالعلم في config يبقى False حتى تُحسم القياسات.

معايير الحكم (تُطبع مع النتيجة):
    σ ≤ 1.0 سم  ✅ ممتاز (هدف البند ج)
    σ ≤ 2.0 سم  ⚠ مقبول — راجع ثبات التثبيت والسطح المستهدف
    σ > 2.0 سم  ❌ راجع: اهتزاز التثبيت · سطح ماصّ/مائل · تداخل صدى
    صالح < 80%  ❌ توصيل/مقسّم جهد/زاوية سطح
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

from pi.config import US_ROUND_ROBIN_GAP_S, US_SIDE_TILT_DEG
from pi.sensors.ultrasonic_array import (
    UltrasonicArray, UltrasonicChannel, CHANNEL_PINS, CHANNEL_TILT_DEG,
    ROBIN_ORDER, FRONT, perpendicular_cm,
)

try:
    import lgpio
except ImportError:
    sys.exit("خطأ: مكتبة lgpio غير مثبّتة — هذا السكربت يعمل على الراسبري فقط "
             "(apt: python3-lgpio).")

# عتبات الحكم — محلية للسكربت (أداة قياس لا منطق مهمة)
SIGMA_GOOD_CM = 1.0
SIGMA_WARN_CM = 2.0
VALID_MIN_PCT = 80.0
SINGLE_GAP_S = 0.06            # نفس فاصل التناوب — HC-SR04 يحتاجه لخمود الصدى


def channel_stats(name: str, samples: list, truth_cm=None) -> dict:
    """إحصاء قناة: الخام كما قاسه العتاد، والعمودي للجانبيين."""
    valid = [s for s in samples if s is not None]
    tilt = CHANNEL_TILT_DEG.get(name, 0.0)
    out = {"name": name, "n": len(samples), "n_valid": len(valid),
           "valid_pct": round(100.0 * len(valid) / max(len(samples), 1), 1),
           "tilt_deg": tilt}
    if len(valid) >= 3:
        med = statistics.median(valid)
        out.update({
            "median_cm": round(med, 2),
            "mean_cm": round(statistics.fmean(valid), 2),
            "sigma_cm": round(statistics.stdev(valid), 3),
            "min_cm": round(min(valid), 1), "max_cm": round(max(valid), 1),
            "perp_median_cm": round(perpendicular_cm(med, tilt), 2),
        })
        if truth_cm is not None:
            compare = out["perp_median_cm"] if tilt else out["median_cm"]
            out["truth_cm"] = float(truth_cm)
            out["bias_cm"] = round(compare - float(truth_cm), 2)
    return out


def print_stats(st: dict) -> bool:
    """يطبع سطر الحكم؛ يُعيد False عند فشل قاطع (قناة ميتة/σ خارج الحد)."""
    name = st["name"]
    if st["n_valid"] < 3:
        print(f"  ❌ {name}: قناة ميتة — {st['n_valid']}/{st['n']} قراءة صالحة "
              f"فقط. راجع TRIG/ECHO والمقسّم والتغذية.")
        return False
    sig = st["sigma_cm"]
    verdict = ("✅" if sig <= SIGMA_GOOD_CM else
               "⚠" if sig <= SIGMA_WARN_CM else "❌")
    line = (f"  {verdict} {name}: σ={sig}سم · وسيط={st['median_cm']}سم"
            + (f" · عمودي={st['perp_median_cm']}سم (ميل {st['tilt_deg']}°)"
               if st["tilt_deg"] else "")
            + f" · مدى [{st['min_cm']}, {st['max_cm']}]"
            + f" · صالح {st['valid_pct']}% ({st['n_valid']}/{st['n']})")
    if "bias_cm" in st:
        line += f" · انحياز عن الشريط {st['bias_cm']:+.1f}سم"
    print(line)
    ok = sig <= SIGMA_WARN_CM and st["valid_pct"] >= VALID_MIN_PCT
    if st["valid_pct"] < VALID_MIN_PCT:
        print(f"     ❌ نسبة الصالح {st['valid_pct']}% دون {VALID_MIN_PCT}% — "
              f"توصيل أو مقسّم أو سطح رديء الانعكاس.")
    return ok


def run_single(name: str, seconds: float, truth_cm) -> int:
    """قناة واحدة، نبضة كل SINGLE_GAP_S — بلا أي قناة أخرى تعمل."""
    trig, echo = CHANNEL_PINS[name]
    h = None
    for chip in (0, 4):
        try:
            h = lgpio.gpiochip_open(chip)
            break
        except Exception:              # noqa: BLE001
            continue
    if h is None:
        sys.exit("خطأ: تعذّر فتح gpiochip — المستخدم ضمن مجموعة gpio؟")
    ch = UltrasonicChannel(name, trig, echo)
    if not ch.claim(h):
        sys.exit(f"خطأ: تعذّر حجز {name} (TRIG={trig}/ECHO={echo}): {ch.error}")
    print(f"قياس {name} منفرداً {seconds:.0f}ث (TRIG=BCM{trig} · ECHO=BCM{echo})"
          + (f" · شريط القياس={truth_cm}سم" if truth_cm is not None else "")
          + " — لا تحرّك الروبوت.\n")
    samples = []
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            ch.measure()
            samples.append(ch.raw_cm)
            time.sleep(SINGLE_GAP_S)
    finally:
        stuck = ch.stuck
        ch.close()
        lgpio.gpiochip_close(h)
    ok = print_stats(channel_stats(name, samples, truth_cm))
    if stuck:
        print(f"  ⚠ انحشار ECHO: {stuck} دورة وُجد فيها مرتفعاً قبل التحفيز")
    return 0 if ok else 1


def run_round_robin(seconds: float, truths: dict) -> int:
    """الثلاثة عبر مجدوِل الإنتاج — نبضة واحدة في كل لحظة، بالدور."""
    arr = UltrasonicArray(enabled=True)
    if not arr.ok:
        sys.exit(f"خطأ: المصفوفة لم تعمل — {arr.error}")
    missing = [n for n in ROBIN_ORDER if n not in arr.channels]
    if missing:
        print(f"⚠ قنوات لم تُحجز: {'، '.join(missing)} — {arr.error}")
    print(f"تناوب الحساسات الثلاثة {seconds:.0f}ث "
          f"(فاصل {US_ROUND_ROBIN_GAP_S*1000:.0f}ms · دورة ≈"
          f"{len(arr.channels)*US_ROUND_ROBIN_GAP_S*1000:.0f}ms) "
          f"— لا تحرّك الروبوت.\n")
    samples = {n: [] for n in arr.channels}
    seen = {n: 0 for n in arr.channels}
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            for n, ch in arr.channels.items():
                if ch.measurements > seen[n]:
                    # قد تفوتنا نبضة عند استقصاء بطيء — نسجّل الأحدث فقط،
                    # والعدّ الكلي يُؤخذ من عدّاد القناة نفسها
                    seen[n] = ch.measurements
                    samples[n].append(ch.raw_cm)
            time.sleep(0.01)
    finally:
        stuck = {n: ch.stuck for n, ch in arr.channels.items()}
        arr.close()
    all_ok = True
    for n in ROBIN_ORDER:
        if n in samples:
            all_ok &= print_stats(channel_stats(n, samples[n], truths.get(n)))
            if stuck.get(n):
                print(f"     ⚠ انحشار ECHO في {n}: {stuck[n]} دورة "
                      f"(الوحدة كانت عالقة والحارس تخطّاها بدل تعميتها)")
    rate = arr.cycles / max(seconds, 1e-9)
    print(f"\n  دورات تناوب كاملة: {arr.cycles} (≈{rate:.1f}/ث · المتوقَّع "
          f"{1.0/(3*US_ROUND_ROBIN_GAP_S+0.05):.0f}–"
          f"{1.0/(3*US_ROUND_ROBIN_GAP_S):.0f}/ث) · "
          f"قراءات أولوية أمامية: {arr.priority_reads}")
    # ⚠ فحص التداخل: σ التناوب يجب ألا ينفجر مقارنة بالوضع المنفرد.
    #    إن كان σ هنا أضعاف σ المنفرد لنفس القناة ⇒ صدى متداخل رغم التناوب
    #    (فاصل قصير أو انعكاسات غرفة) — جرّب رفع US_ROUND_ROBIN_GAP_S.
    print("  ⚠ قارن σ كل قناة هنا بقياسها المنفرد: تضاعفه = تداخل صدى "
          "(ارفع US_ROUND_ROBIN_GAP_S وأعد).")
    return 0 if all_ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="إحصاء الألترا سونيك منفرداً وبالتناوب")
    ap.add_argument("--only", choices=ROBIN_ORDER, default=None,
                    help="قناة واحدة فقط (بند ب) — الافتراضي: الثلاثة بالتناوب")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--truth", type=float, default=None,
                    help="مسافة شريط القياس العمودية (سم) — للوضع المنفرد")
    ap.add_argument("--truth-front", type=float, default=None)
    ap.add_argument("--truth-right", type=float, default=None)
    ap.add_argument("--truth-left", type=float, default=None)
    a = ap.parse_args()
    if a.only:
        return run_single(a.only, a.seconds, a.truth)
    truths = {"front": a.truth_front, "right": a.truth_right,
              "left": a.truth_left}
    return run_round_robin(a.seconds, {k: v for k, v in truths.items()
                                       if v is not None})


if __name__ == "__main__":
    sys.exit(main())
