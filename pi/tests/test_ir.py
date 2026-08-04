#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_ir.py — فحص حسّاسات IR الخمسة — **تفاعلي إلزاماً**
========================================================
🔴 **لماذا لا تكفي القراءة اللحظية**: المنافذ تُقرأ بشدّ مرتفع داخلي، فالمنفذ
   **غير الموصول يقرأ `1` = خالٍ** تماماً كحسّاس سليم لا يرى شيئاً. أي فحص
   يكتفي بلحظة واحدة **لا يميّز سلكاً مقطوعاً عن حسّاس يعمل**، ويُخرج «كل
   شيء سليم» عن روبوت أعمى.

   الطريقة الوحيدة القاطعة: **التلويح الفعلي** أمام كل حسّاس ومراقبة تغيّر
   القراءة بين القيمتين. هذا ما يفعله هذا السكربت.

المنطق: **0 = عائق** · التغذية **3.3V حصراً** (5V تتلف دبوس GPIO فوراً).
المدى يُضبط بالمقاومة الزرقاء على **25–30 سم** لا الافتراضي ~10سم: عند
0.6 م/ث الفرق **0.33 ثانية** إضافية لزمن الاستجابة.

    python3 -m pi.tests.test_ir              # يفحص الحاضر في IR_PRESENT
    python3 -m pi.tests.test_ir --all        # الخمسة (لتأكيد تركيب جديد)
    python3 -m pi.tests.test_ir --watch      # عرض حيّ (لضبط المقاومة)
"""
from __future__ import annotations

import argparse
import sys
import time

from pi.config import IR_PRESENT, IR_OBSTACLE_LEVEL, IR_PULL_UP
from pi.sensors.proximity import IRReader, IR_PINS

AR = {"front_left": "أمام-يسار", "front_right": "أمام-يمين",
      "front_mid": "أمام-وسط", "side_left": "جانبي-أيسر",
      "side_right": "جانبي-أيمن"}

_ok = []


def check(name, cond, detail=""):
    _ok.append(bool(cond))
    print(("  ✅ " if cond else "  ❌ ") + name + (f"   [{detail}]" if detail else ""))
    return bool(cond)


def watch(ir: IRReader) -> int:
    """عرض حيّ — لضبط المقاومة الزرقاء على 25-30 سم."""
    print("\nعرض حيّ (Ctrl-C للإيقاف) — 0 = عائق · 1 = خالٍ · — = مجهول\n")
    try:
        while True:
            v = ir.read_all()
            row = " | ".join(
                f"{AR[n]}: " + ("—" if v[n] is None else
                                ("🔴0" if v[n] == IR_OBSTACLE_LEVEL else " 1"))
                for n, _ in IR_PINS)
            print("\r  " + row, end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nتوقّف.")
    return 0


def probe(ir: IRReader, name: str, seconds: float) -> bool:
    """
    مرحلة تفاعلية لحسّاس واحد: يُطلب التلويح ونراقب **تغيّر** القراءة.
    النجاح = شوهدت القيمتان معاً (خالٍ **و** عائق) خلال المهلة.
    """
    pin = dict(IR_PINS)[name]
    print(f"\n  ── {AR[name]} (BCM{pin}) ──")
    print(f"     لوّح بيدك أمامه على ~10-20 سم خلال {seconds:.0f} ثانية…")
    seen = set()
    t_end = time.time() + seconds
    while time.time() < t_end:
        v = ir.read_all().get(name)
        if v is not None:
            seen.add(int(v))
        mark = "—" if v is None else ("🔴 عائق" if v == IR_OBSTACLE_LEVEL else "خالٍ")
        print(f"\r     {t_end - time.time():4.1f}ث · القراءة: {mark}   "
              f"شوهد: {sorted(seen)}   ", end="", flush=True)
        if len(seen) >= 2:
            break
        time.sleep(0.05)
    print()
    if not seen:
        return check(f"{AR[name]}: يستجيب", False,
                     "لا قراءة إطلاقاً — المنفذ غير محجوز")
    if len(seen) < 2:
        return check(
            f"{AR[name]}: يستجيب للتلويح", False,
            ("ثابت على «خالٍ» — **سلك مقطوع أو حسّاس تالف**: الشدّ المرتفع "
             "يعطي 1 دائماً على منفذ غير موصول"
             if 1 in seen else
             "عالق على «عائق» — تغذية مفقودة أو مقاومة مدى مضبوطة خطأً "
             "أو خرج مقصور"))
    return check(f"{AR[name]}: يستجيب للتلويح ✓", True, "شوهدت القيمتان 0 و1")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--all", action="store_true",
                    help="افحص الخمسة حتى المعلَن غائباً (بعد تركيب جديد)")
    ap.add_argument("--watch", action="store_true", help="عرض حيّ متواصل")
    ap.add_argument("--seconds", type=float, default=12.0,
                    help="مهلة التلويح لكل حسّاس")
    a = ap.parse_args()

    present = {n: True for n, _ in IR_PINS} if a.all else dict(IR_PRESENT)
    print("\n=== فحص حسّاسات IR ===")
    print(f"  المنطق: {IR_OBSTACLE_LEVEL} = عائق · شدّ مرتفع={IR_PULL_UP} · "
          f"تغذية 3.3V")
    print("  الحاضر: " + (", ".join(AR[n] for n, _ in IR_PINS if present.get(n))
                          or "لا شيء"))
    skipped = [AR[n] for n, _ in IR_PINS if not present.get(n)]
    if skipped:
        print("  ⏸ معلَن غائباً (لا يُحجز منفذه): " + "، ".join(skipped))
        print("     بعد تركيبه: شغّل بـ--all ثم ارفع علمه في IR_PRESENT.")

    ir = IRReader(present=present)
    if not ir.ok:
        print(f"\n  ⛔ تعذّر فتح المنافذ: {ir.error}")
        print("     تحقّق أن المستخدم ضمن مجموعة gpio وأن سيرفر الويب متوقف.")
        return 1
    print(f"  محجوز فعلاً: {len(ir.claimed)} منفذ")

    if a.watch:
        return watch(ir)

    # 🔴 تُعرض **للسياق فقط** ولا يُبنى عليها حكم (انظر ترويسة الملف)
    print("\n  القراءة اللحظية (لا تُثبت شيئاً):")
    snap = ir.read_all()
    for n, _ in IR_PINS:
        v = snap.get(n)
        print(f"    {AR[n]:12} = " + ("—" if v is None else str(v)))

    for n, _ in IR_PINS:
        if present.get(n):
            probe(ir, n, a.seconds)

    ir.close()
    print(f"\n=== النتيجة: {sum(_ok)}/{len(_ok)} نجح ===")
    if not all(_ok):
        print("  ⚠ حسّاس لم يستجب = سلك مقطوع أو تغذية مفقودة أو مقاومة مدى "
              "مضبوطة خطأً.\n     صحّح ثم أعد الفحص — ولا تشغّل مسحاً ذاتياً قبله.")
    return 0 if all(_ok) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nأُلغي.")
        sys.exit(130)
