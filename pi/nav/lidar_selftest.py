# -*- coding: utf-8 -*-
"""
lidar_selftest.py — سيناريوهات قرار الليدار بلا عتاد (ثوانٍ)
==========================================================
    python3 -m pi.nav.lidar_selftest

المشاهد تُبنى **بإطار الشركة الخام** (زاوية مع عقارب الساعة) بتتبّع أشعة ضد
صناديق بإطار الروبوت، ثم تمرّ بـ`to_robot_frame` الحقيقي — فالاختبار يغطي
التحويل والتصفية والقرار معاً لا القرار وحده.
"""
from __future__ import annotations

import math
import sys
import time

from pi.config import (
    LIDAR_ANGLE_SIGN, LIDAR_YAW_OFFSET_DEG, LIDAR_X_M, LIDAR_Y_M,
    LIDAR_MAX_SPEED, ESCAPE_MAX_ATTEMPTS, ROBOT_LENGTH_M, LIDAR_BLOCK_CONFIRM_SCANS,
)
from pi.nav.lidar_avoid import LidarAvoider, mask_front_overlap, filter_points
from pi.sensors.lidar_c1 import NodeParser, to_robot_frame

_results = []


def check(name, cond, detail=""):
    ok = bool(cond)
    _results.append(ok)
    print(("  ✅ " if ok else "  ❌ ") + name + (f"   [{detail}]" if detail else ""))
    return ok


def _ray_box(ox, oy, dx, dy, box):
    """تقاطع شعاع مع صندوق محاذٍ (xmin,xmax,ymin,ymax) ⇒ المسافة أو None."""
    xmin, xmax, ymin, ymax = box
    tmin, tmax = 0.0, float("inf")
    for o, d, lo, hi in ((ox, dx, xmin, xmax), (oy, dy, ymin, ymax)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin if tmin > 1e-9 else None


def scene_raw(boxes, room=3.0, step_deg=0.72):
    """نقاط خام (زاوية الشركة°، مسافة م) لصناديق + غرفة مربعة نصف ضلعها `room`."""
    walls = [(room, room + 0.1, -room, room), (-room - 0.1, -room, -room, room),
             (-room, room, room, room + 0.1), (-room, room, -room - 0.1, -room)]
    pts = []
    n = int(round(360.0 / step_deg))
    for i in range(n):
        raw = i * step_deg
        la = LIDAR_ANGLE_SIGN * raw + LIDAR_YAW_OFFSET_DEG
        t = math.radians(la)
        dx, dy = math.cos(t), math.sin(t)
        best = None
        for b in list(boxes) + walls:
            d = _ray_box(LIDAR_X_M, LIDAR_Y_M, dx, dy, b)
            if d is not None and (best is None or d < best):
                best = d
        pts.append((raw, best or 0.0))
    return pts


def decide(boxes, avoider=None, extra_raw=(), room=3.0):
    av = avoider or LidarAvoider(set(), LIDAR_MAX_SPEED)
    pts = to_robot_frame(scene_raw(boxes, room) + list(extra_raw))
    # لفّتان متتاليتان لنفس المشهد: «مسدود» يُؤكَّد بـLIDAR_BLOCK_CONFIRM_SCANS
    for _ in range(LIDAR_BLOCK_CONFIRM_SCANS - 1):
        av.decide(pts, time.time())
    return av.decide(pts, time.time()), av


def main() -> int:
    print("\n=== قرار الليدار — سيناريوهات بلا عتاد ===\n")
    front_edge = ROBOT_LENGTH_M / 2.0

    print("أ) الإطار والمحلّل:")
    fr = to_robot_frame([(90.0, 1.0), (270.0, 1.0), (0.0, 1.0), (0.01, 0.03)])
    check("زاوية الشركة 90° (مع عقارب الساعة) = يمين الروبوت (−90°)",
          abs(fr[0]["a"] - (-90.0 * (-LIDAR_ANGLE_SIGN))) < 1.0 and fr[0]["y"] < 0,
          f"a={fr[0]['a']:.1f} y={fr[0]['y']:.2f}")
    check("270° = يسار (+90°) · 0° = أمام", fr[1]["y"] > 0 and fr[2]["x"] > 0.9)
    check("ما دون 5سم يُهمل", len(fr) == 3)
    # عقدة SCAN: بداية لفّة، جودة 15، زاوية 90°، مسافة 1234 مم — وبايت دخيل قبلها
    q6, q2 = int(90.0 * 64), 1234 * 4
    node = bytes([(15 << 2) | 0b01, ((q6 & 0x7F) << 1) | 1, q6 >> 7,
                  q2 & 0xFF, q2 >> 8])
    np_ = NodeParser()
    got = np_.feed(b"\x00" + node[:3]) + np_.feed(node[3:])
    check("محلّل العُقد + إعادة تزامن بعد بايت دخيل",
          len(got) == 1 and got[0][0] and abs(got[0][2] - 90.0) < 0.02
          and abs(got[0][3] - 1.234) < 1e-6 and np_.resyncs >= 1, str(got))

    print("\nب) القرارات:")
    d, _ = decide([])
    check("غرفة مفتوحة ⇒ سير بأقصى سرعة", d["action"] == "go"
          and abs(d["l"] - LIDAR_MAX_SPEED) < 1e-9, f"{d['action']} {d.get('l')}")

    wall = [(front_edge + 0.20, front_edge + 0.30, -2.5, 2.5)]
    d, av = decide(wall)
    check("جدار أمام (20سم) والخلف فاضٍ ⇒ رجوع", d["action"] == "backup", d["reason"])
    av.note_backed_up()
    d2 = av.decide(to_robot_frame(scene_raw(wall)), time.time())   # اللفّة التالية مباشرة
    check("بعده ⇒ لفّ بالمكان بزاوية كبيرة", d2["action"] == "spin"
          and abs(d2["deg"]) >= 45, d2["reason"])

    wall_far = [(front_edge + 0.55, front_edge + 0.65, -2.5, 2.5)]
    d, _ = decide(wall_far)
    check("جدار أمام (55سم، منطقة التفادي) ⇒ إبطاء + لفّ نحو فتحة جانبية",
          d["action"] == "spin" and abs(d["deg"]) >= 45, d["reason"])

    box_left = [(front_edge + 0.35, front_edge + 0.65, 0.0, 0.45)]
    d, _ = decide(box_left)
    right = (d["action"] == "arc" and d["l"] > d["r"]) or \
            (d["action"] == "spin" and d["deg"] < 0)
    check("صندوق أمام-يسار ⇒ انعطاف يميناً", right,
          f"{d['action']} {d.get('deg', '')} L={d.get('l')} R={d.get('r')} — {d['reason']}")

    box_right = [(front_edge + 0.35, front_edge + 0.65, -0.45, 0.0)]
    d, _ = decide(box_right)
    left = (d["action"] == "arc" and d["r"] > d["l"]) or \
           (d["action"] == "spin" and d["deg"] > 0)
    check("صندوق أمام-يمين ⇒ انعطاف يساراً", left,
          f"{d['action']} {d.get('deg', '')} L={d.get('l')} R={d.get('r')} — {d['reason']}")

    # نقطة ملاصقة **عابرة** (كابل يُصاب متقطعاً — مقاس 2026-09-27) لا تستهلك
    # محاولات التحرر: توقف فوري، ولا محاولة ما لم يتكرّر الانسداد
    av = LidarAvoider(set(), LIDAR_MAX_SPEED)
    blip = to_robot_frame(scene_raw([]) + [(0.0, 0.10), (0.5, 0.10)])
    clear = to_robot_frame(scene_raw([]))
    seq = []
    for pts_ in (blip, clear, blip, clear, blip, clear):
        seq.append(av.decide(pts_, time.time())["action"])
    check("انسداد عابر لفّة واحدة ⇒ توقف فوري بلا استهلاك محاولات",
          seq == ["stop", "go"] * 3 and av.attempts == 0, " → ".join(seq))

    nudge = [(front_edge + 0.50, front_edge + 0.60, -0.20, 0.05)]
    d, _ = decide(nudge)
    check("عائق يلامس طرف الممرّ (زاوية صغيرة) ⇒ قوس يساراً لا لفّ بالمكان",
          d["action"] == "arc" and d["r"] > d["l"], d["reason"])

    # مقاس 2026-09-27: فتحة زاويتها ~0 قلبت «سير/قوس» كل نصف ثانية ⇒ ارتجاف.
    #   داخل منطقة التفادي: قوس دائماً وبأدنى زاوية، ولا «سير» نحو العائق.
    from pi.config import LIDAR_ARC_MIN_DEG
    acts = set()
    for yy in (-0.14, -0.12, -0.10, -0.08):
        d, _ = decide([(front_edge + 0.50, front_edge + 0.60, -0.40, yy)])
        acts.add(d["action"])
        ok_side = d["action"] != "arc" or d["r"] - d["l"] > 0.02
    check(f"عائق على طرف الممرّ ⇒ قوس ثابت (≥{LIDAR_ARC_MIN_DEG:.0f}°) لا تذبذب مع «سير»",
          "go" not in acts and ok_side, f"الأفعال={sorted(acts)}")

    # محصور: صندوق ضيق حول الروبوت (10سم من كل حافة)
    av = LidarAvoider(set(), LIDAR_MAX_SPEED)
    acts = []
    for _ in range(ESCAPE_MAX_ATTEMPTS + 3):
        d, av = decide([], av, room=front_edge + 0.10)
        acts.append(d["action"])
        if d["action"] == "backup":
            av.note_backed_up()
        if d["action"] == "trapped":
            break
    check(f"محصور ⇒ «محصور» بعد {ESCAPE_MAX_ATTEMPTS} محاولات بلا حركة خطرة",
          acts[-1] == "trapped" and "go" not in acts and "arc" not in acts,
          " → ".join(acts))

    lone = [(0.0, front_edge + 0.10)]                  # نقطة واحدة في الممرّ
    d, _ = decide([], extra_raw=lone)
    check("نقطة منفردة في الممرّ ⇒ تُهمل ويسير", d["action"] == "go"
          and d["stats"]["isolated"] >= 1, f"منفردة={d['stats']['isolated']}")
    pair = [(0.0, front_edge + 0.10), (0.7, front_edge + 0.11)]
    d, _ = decide([], extra_raw=pair)
    check("نقطتان متجاورتان (≤2° و5سم) ⇒ عائق حقيقي", d["action"] != "go"
          or d["l"] < LIDAR_MAX_SPEED, f"{d['action']} — {d['reason']}")

    print("\nج) القناع وجسم الروبوت:")
    blocked = set(range(-100, -79))                   # قطاع يمين محجوب
    pts = to_robot_frame([(90.0, 0.2), (90.5, 0.2), (0.0, 1.0), (0.5, 1.0)])
    kept, st = filter_points(pts, blocked)
    check("النقاط المحجوبة تُحذف", st["masked"] == 2 and len(kept) == 2, str(st))
    check("قناع يغطي الأمام ⇒ يُكتشف (تحذير)", mask_front_overlap({-3, 0, 90}) == [-3, 0]
          and mask_front_overlap(blocked) == [])
    inside = to_robot_frame([(0.0, 0.08), (0.8, 0.08)])
    kept, st = filter_points(inside, set())
    check("نقاط داخل مستطيل الروبوت = عوائق ملاصقة (خلوص 0) وتُعدّ",
          st["inside"] == 2 and len(kept) == 2 and all(p["c"] == 0.0 for p in kept), str(st))
    # 🔴 المقاس 2026-09-27: كوب قريب داخل المستطيل المفترض يحجب جداراً خلفه —
    #    كان يُحذف فيطبع «الممرّ خالٍ» والكوب أمامه
    fe = front_edge
    d, _ = decide([(fe + 0.55, fe + 0.65, -1.0, 1.0), (0.08, 0.14, -0.04, 0.04)])
    check("كوب ملاصق أمام الليدار يحجب جداراً ⇒ لا «سير»", d["action"] != "go"
          and d["front_m"] == 0.0, f"{d['action']} — {d['reason']}")

    print("\nد) السلامة في الحلقة:")
    from pi.nav.lidar_drive import LidarDriver

    class FakeBridge:
        mode = "sim"
        error = None
        def __init__(self):
            self.cmds = []
        def motors(self, l, r):
            self.cmds.append((l, r))
        def wheel_speeds_mps(self):
            return None

    class FakeLidar:
        error = None
        def __init__(self):
            self.ok = True
            self.pts = scene_raw([])
        def latest(self):
            return (time.time(), self.pts)
        def fresh(self, _age=0.3):
            return self.ok
        def age_s(self):
            return 0.0 if self.ok else 9.9
        def state(self):
            return {}

    fb, fl = FakeBridge(), FakeLidar()
    drv = LidarDriver(fb, fl, LIDAR_MAX_SPEED, echo=lambda *_: None)
    drv._tick()
    moving = fb.cmds[-1]
    fl.ok = False
    drv._tick()
    stopped = fb.cmds[-1]
    fl.ok = True
    drv._tick()
    check("ليدار متجمّد أثناء المشي ⇒ T:1 صفر، وعودته ⇒ يكمل",
          moving[0] > 0 and stopped == (0.0, 0.0) and fb.cmds[-1][0] > 0,
          f"{moving} → {stopped} → {fb.cmds[-1]}")
    fb.cmds.clear()
    for _ in range(6):
        drv._tick()
    ls = [c[0] for c in fb.cmds]
    check("رفع السرعة متدرّج (≤0.03 م/ث لكل دورة) والسقف يُبلغ",
          all(b - a <= 0.03 + 1e-9 for a, b in zip(ls, ls[1:]))
          and abs(ls[-1] - LIDAR_MAX_SPEED) < 1e-9, " → ".join(f"{v:.2f}" for v in ls))
    fl.ok = False
    drv._tick()
    check("والتوقف فوري (صفر مباشرة من السقف)", fb.cmds[-1] == (0.0, 0.0))
    fl.ok = True
    drv._shutdown()
    check("الخروج يرسل صفراً", fb.cmds[-1] == (0.0, 0.0))

    n_ok = sum(_results)
    print(f"\n=== النتيجة: {n_ok}/{len(_results)} نجح ===")
    return 0 if n_ok == len(_results) else 1


if __name__ == "__main__":
    sys.exit(main())
