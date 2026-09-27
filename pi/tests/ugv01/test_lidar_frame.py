#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_lidar_frame.py — تحقّق إطار الليدار (LIDAR_ANGLE_SIGN) بجسم تضعه بيدك
==========================================================================
ضع جسماً (علبة/يدك) قريباً من الروبوت على جهة واحدة، والمكان حوله فارغ:
يطبع مرتين في الثانية «أقرب جسم: <الجهة> + الزاوية + المسافة».

  جسم على **يسار** الروبوت ⇒ يجب «يسار» وزاوية **موجبة** (~+90°).
  إن طبع «يمين» فاقلب LIDAR_ANGLE_SIGN في pi/config.py.
  جسم **أمامه** ⇒ ~0°؛ إن ظهر منحرفاً ثابتاً فذاك LIDAR_YAW_OFFSET_DEG.

القناع ونقاط داخل مستطيل الروبوت تُستبعد (--raw يعرض كل شيء).

    python3 -m pi.tests.ugv01.test_lidar_frame [--seconds 30] [--raw]
"""
from __future__ import annotations

import argparse
import sys
import time

from pi.config import LIDAR_ANGLE_SIGN, LIDAR_YAW_OFFSET_DEG
from pi.nav.lidar_avoid import SECTOR_AR, sector_of, filter_points
from pi.nav.lidar_drive import load_mask
from pi.sensors.lidar_c1 import LidarC1, to_robot_frame


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--raw", action="store_true", help="بلا قناع ولا استبعاد جسم الروبوت")
    ap.add_argument("--port", default=None)
    a = ap.parse_args()
    blocked, warns = (set(), []) if a.raw else load_mask()
    for w in warns:
        print(w)
    print(f"LIDAR_ANGLE_SIGN={LIDAR_ANGLE_SIGN} · YAW_OFFSET={LIDAR_YAW_OFFSET_DEG}° "
          f"· يسار = زاوية موجبة · Ctrl+C للخروج")
    lidar = LidarC1(port=a.port).start()
    try:
        t_end = time.time() + a.seconds
        last_ts = 0.0
        while time.time() < t_end:
            s = lidar.wait_scan(last_ts, timeout=1.0)
            if s is None:
                print(f"… لا لفّة ({lidar.error})")
                continue
            last_ts = s[0]
            pts = to_robot_frame(s[1])
            if not a.raw:
                pts, _ = filter_points(pts, blocked)
            if not pts:
                print("لا نقاط")
                continue
            key = "r" if a.raw else "c"
            p = min(pts, key=lambda q: q[key])
            side = SECTOR_AR[sector_of(p["a"])]
            print(f"أقرب جسم: {side:<10} زاوية {p['a']:+6.1f}° · "
                  f"{'من المركز' if a.raw else 'من حافة الروبوت'} {p[key]:.2f}م "
                  f"(x={p['x']:+.2f} y={p['y']:+.2f})")
            time.sleep(0.4)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()                                   # STOP للمحرك
    return 0


if __name__ == "__main__":
    sys.exit(main())
