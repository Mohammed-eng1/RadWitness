#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_lidar_mask.py — قناع أجزاء الروبوت التي يراها الليدار
===============================================================
🔴 ضع الروبوت في **مكان مفتوح** (لا شيء أقرب من ~0.5م حوله) ولا تقف بجانبه.

50 لفّة: كل زاوية (1°، بإطار الروبوت) فيها نقاط **ثابتة** أقرب من 0.35م
(ظهرت في ≥60٪ من اللفّات) = جزء من الروبوت (الكاميرا، الهوائي…) ⇒ تُحجب
مع ±2°. يُحفظ في pi/data/lidar_mask.json وتُطبع القطاعات.

⚠ قناع يغطي الأمام (±30°) = تحذير: عائق هناك لن يُرى. التشغيل لا يُمنع.

    python3 -m pi.tests.ugv01.calibrate_lidar_mask [--revs 50]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

from pi.config import (
    LIDAR_MASK_PATH, LIDAR_MASK_NEAR_M, LIDAR_MASK_PAD_DEG, LIDAR_MASK_REVS,
    LIDAR_MASK_MIN_FRACTION, LIDAR_ANGLE_SIGN, LIDAR_YAW_OFFSET_DEG,
    LIDAR_X_M, LIDAR_Y_M,
)
from pi.nav.lidar_avoid import mask_front_overlap
from pi.sensors.lidar_c1 import LidarC1, to_robot_frame


def build_mask(scans, near_m=LIDAR_MASK_NEAR_M, min_frac=LIDAR_MASK_MIN_FRACTION,
               pad=LIDAR_MASK_PAD_DEG) -> tuple[list, dict]:
    """scans: قوائم نقاط بإطار الروبوت. ⇒ (زوايا محجوبة، تكرار كل زاوية قريبة)."""
    hits = {}
    for pts in scans:
        seen = {int(round(p["la"])) for p in pts if p["d"] < near_m}
        for k in seen:
            k = (k + 180) % 360 - 180
            hits[k] = hits.get(k, 0) + 1
    n = max(1, len(scans))
    core = [k for k, c in hits.items() if c / n >= min_frac]
    blocked = set()
    for k in core:
        for dk in range(-pad, pad + 1):
            blocked.add((k + dk + 180) % 360 - 180)
    return sorted(blocked), {k: round(c / n, 2) for k, c in sorted(hits.items())}


def sectors(blocked: list) -> list:
    """زوايا متتالية ⇒ قطاعات [من، إلى] (مع التفاف −180/179)."""
    if not blocked:
        return []
    out, start, prev = [], blocked[0], blocked[0]
    for d in blocked[1:]:
        if d != prev + 1:
            out.append([start, prev])
            start = d
        prev = d
    out.append([start, prev])
    if len(out) > 1 and out[0][0] == -180 and out[-1][1] == 179:
        out[0][0] = out.pop()[0]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--revs", type=int, default=LIDAR_MASK_REVS)
    ap.add_argument("--port", default=None)
    ap.add_argument("--out", default=LIDAR_MASK_PATH)
    a = ap.parse_args()
    print(f"🔴 مكان مفتوح، لا تقف بجانب الروبوت. جمع {a.revs} لفّة…")
    lidar = LidarC1(port=a.port).start()
    scans, last = [], 0.0
    try:
        t_end = time.time() + max(10.0, a.revs * 0.3)
        while len(scans) < a.revs and time.time() < t_end:
            s = lidar.wait_scan(last, timeout=1.0)
            if s is None:
                print(f"… لا لفّة ({lidar.error})")
                continue
            last = s[0]
            scans.append(to_robot_frame(s[1]))
    finally:
        lidar.stop()
    if len(scans) < a.revs:
        print(f"⚠ جُمعت {len(scans)} لفّة فقط من {a.revs} — يكمل بها")
    if not scans:
        print("⚠ لا لفّات إطلاقاً — لم يُحفظ قناع (التشغيل يكمل بلا قناع)")
        return 1
    blocked, freq = build_mask(scans)
    secs = sectors(blocked)
    mask = {"blocked_deg": blocked, "sectors": secs, "revs": len(scans),
            "near_m": LIDAR_MASK_NEAR_M, "min_fraction": LIDAR_MASK_MIN_FRACTION,
            "pad_deg": LIDAR_MASK_PAD_DEG, "near_freq": freq,
            "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "config": {"sign": LIDAR_ANGLE_SIGN, "yaw_offset_deg": LIDAR_YAW_OFFSET_DEG,
                       "x_m": LIDAR_X_M, "y_m": LIDAR_Y_M}}
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(mask, f, ensure_ascii=False, indent=1)
    print(f"حُفظ: {a.out} · {len(blocked)} درجة محجوبة")
    if not secs:
        print("لا قطاعات محجوبة (لا أجزاء روبوت قريبة من مستوى المسح)")
    for lo, hi in secs:
        mid = (lo + hi) / 2.0
        side = "يسار" if mid > 0 else "يمين"
        print(f"  قطاع {lo:+4d}° .. {hi:+4d}°  ({side})")
    front = mask_front_overlap(set(blocked))
    if front:
        print(f"⚠ القناع يغطي الأمام: {front} — عائق هناك لن يُرى! "
              f"تأكد أن المكان كان مفتوحاً وأعد المعايرة (التشغيل يكمل)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
