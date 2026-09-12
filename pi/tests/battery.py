#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
battery.py — **أداة خارجية**: نسبة وجهد الحزمتين بأمر واحد.

🔴 **حزمتان لا واحدة** (CLAUDE.md §2.0.1)، ولا تُخلطان أبداً:
  • **الراسبري**: 3S عبر UPS، الجهد من INA219 @0x42 على i2c-1.
  • **المحركات**: 2S ≈ 7.4V، الجهد من ADS7830 @0x48 قناة 2.
خلطُ عتباتهما مقاس أثره: حزمة 2S **ممتلئة** (8.4V) تُصنَّف بعتبات 3S
⇒ `shutdown` — إطفاء نظام تشغيل على بطارية مشحونة.

⚠ والنسبة **تقديرية والقرار على الجهد**: منحنى الليثيوم مسطّح في وسطه
   فالخطّية تضلّل، ولهذا استيفاء منحنى لا معادلة. ونسبة حزمة المحركات
   مشتقّة من **نفس منحنى الخلية** (المنحنى ÷3 لكل خلية ×2) — الخلية
   واحدة والعدد يختلف.

⚠ ودقّة ADS7830 **8 بت** (~40mV/عدّة) فالنسبة تقفز بخطوات محسوسة.

القراءة **غير مؤذية** (لا تلمس محركاً)، لكن إن كان السيرفر شغّالاً فالأفضل
قراءتها منه: `curl -s localhost:8000/api/status`.

    python3 -m pi.tests.battery
    python3 -m pi.tests.battery --watch          # كل ثانيتين حتى Ctrl+C
"""
from __future__ import annotations

import argparse
import sys
import time

from pi.config import (BATT_CURVE, MOTOR_BATT_CELLS, MOTOR_BATT_GOOD_V,
                       MOTOR_BATT_LOW_V, MOTOR_BATT_CRIT_V)
from pi.rover import battery as batt

PACK_CELLS_3S = 3


def motor_percent(v: float) -> int:
    """
    نسبة حزمة 2S من **نفس منحنى الخلية** المستعمل للـ3S.

    ⚠ لا يُنسخ منحنى جديد ولا تُكتب أرقام 2S يدوياً: الكيمياء واحدة
       والعدد يختلف، فالمنحنى يُقسَّم على 3 ليصير لكل خلية ثم يُضرب في
       عدد خلايا هذه الحزمة. ونسخُ جدول ثانٍ يعني جدولين ينحرفان.
    """
    per_cell = [(pv / PACK_CELLS_3S, pct) for pv, pct in BATT_CURVE]
    scaled = [(cv * MOTOR_BATT_CELLS, pct) for cv, pct in per_cell]
    if v >= scaled[0][0]:
        return 100
    if v <= scaled[-1][0]:
        return 0
    for (v1, p1), (v2, p2) in zip(scaled, scaled[1:]):
        if v2 <= v <= v1:
            return int(round(p2 + (v - v2) * (p1 - p2) / (v1 - v2)))
    return 0


def bar(pct) -> str:
    """شريط نصّي — و«?» عرضاً صريحاً للمجهول لا صفراً كاذباً."""
    if pct is None:
        return "[" + "?" * 20 + "]"
    n = max(0, min(20, round(pct / 5)))
    return "[" + "#" * n + "-" * (20 - n) + "]"


def show(rover) -> int:
    worst = 0

    # ── ① حزمة الراسبري (3S / INA219) ───────────────────────────
    b = rover.battery_state()
    v, pct = b.get("v"), b.get("percent")
    print("Raspberry Pi pack (3S, UPS / INA219 @0x42)")
    if v is None:
        print(f"  {bar(None)}   voltage UNKNOWN   source={b.get('source')}")
        print(f"    -> {b.get('source_reason') or 'no reading'}")
        worst = max(worst, 1)
    else:
        amps = b.get("amps")
        chg = ("charging" if b.get("charging") else
               "unknown" if b.get("charging_unknown") else "discharging")
        print(f"  {bar(pct)}  {pct:3d}%   {v:5.2f} V   "
              f"({b.get('cell_v')} V/cell)   {chg}"
              + (f"   {amps:+.3f} A" if amps is not None else ""))
        print(f"    {b.get('level')}  ->  action: {b.get('action')}"
              f"   [{b.get('source')}]")
        if b.get("action") in ("rth", "stop", "shutdown"):
            worst = max(worst, 2)

    # ── ② حزمة المحركات (2S / ADS7830) ──────────────────────────
    m = rover.motor_pack_state()
    mv = m.get("v")
    print(f"\nMotor pack ({MOTOR_BATT_CELLS}S, ADS7830 @0x48 ch2)")
    if mv is None:
        print(f"  {bar(None)}   voltage UNKNOWN   source={m.get('source')}")
        print(f"    -> {m.get('source_reason') or m.get('text')}")
        worst = max(worst, 1)
    else:
        mp = motor_percent(mv)
        print(f"  {bar(mp)}  {mp:3d}%   {mv:5.2f} V   "
              f"({m.get('cell_v')} V/cell)   raw byte {m.get('raw')}")
        print(f"    {m.get('level')}  ->  action: {m.get('action')}"
              f"   [thresholds: good>={MOTOR_BATT_GOOD_V} "
              f"low>={MOTOR_BATT_LOW_V} crit>={MOTOR_BATT_CRIT_V}]")
        if m.get("action") in ("rth", "stop"):
            worst = max(worst, 2)
    return worst


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Battery percentage for BOTH packs (Pi 3S + motor 2S)")
    ap.add_argument("--watch", action="store_true",
                    help="repeat every --every seconds until Ctrl+C")
    ap.add_argument("--every", type=float, default=2.0)
    a = ap.parse_args()

    from pi.rover.bridge import RoverControlBridge
    rover = RoverControlBridge(mode="real")
    # ⚠ الوضع يُعلَن: قراءة من جسر محاكى **رقم مُختلَق**، وإخفاؤها هنا
    #    يجعل أداة البطارية نفسها مصدر طمأنينة كاذبة.
    if rover.mode != "real":
        print(f"⚠ bridge is NOT real ({rover.error}) — "
              f"any number below is SIMULATED, not measured.\n")
    try:
        if not a.watch:
            return show(rover)
        while True:
            print("\033[2J\033[H", end="")     # مسح الشاشة
            print(time.strftime("%H:%M:%S"))
            show(rover)
            time.sleep(max(0.5, a.every))
    except KeyboardInterrupt:
        print("\nstopped.")
        return 0
    finally:
        # ⚠ لا يمسّ المحركات: أداة قراءة فقط، والإغلاق يحرّر الناقل.
        try:
            rover.close()
        except BaseException:                  # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
