# -*- coding: utf-8 -*-
"""
selftest.py — اختبارات sim لمعايير قبول الدفعة الأولى (بلا عتاد)
================================================================
يشغّل كل بنود «معايير القبول» في وضع sim ويطبع PASS/FAIL. يخرج بكود ≠0
عند أي فشل. التشغيل:  python -m pi.nav.selftest
"""
from __future__ import annotations

import json
import sys
import tempfile

from pi.config import REACTIVE_SAFETY_ENABLED, REACTIVE_LOOP_S, SPEED_NO_READING
from pi.nav.reactive import speed_for_distance, median, JumpFilter
from pi.nav.room import Room, OccupancyGrid
from pi.nav.calibration import (
    CalibrationStore, CalibrationProfile, compute_speed_mps, compute_turn_rate_dps,
)
from pi.nav.deadreckoning import DeadReckoning
from pi.nav.scanner import Scanner
from pi.nav.sim_world import SimWorld
from pi.nav.reactive import ReactiveSafety, EscapeSequence

_results = []


def check(name, cond, detail=""):
    ok = bool(cond)
    _results.append(ok)
    print(("  ✅ " if ok else "  ❌ ") + name + (f"   [{detail}]" if detail else ""))


def main() -> int:
    print("\n=== اختبارات sim — الدفعة الأولى للملاحة ===\n")

    # (1)(2) المعايرة: المنع بلا ملف + الحفظ والاستعادة بعد «إعادة تشغيل»
    print("أ) المعايرة:")
    with tempfile.TemporaryDirectory() as d:
        store = CalibrationStore(directory=d)
        check("بلا ملف معايرة → البدء ممنوع", store.any_exists() is False)
        speed75 = compute_speed_mps(1.35, 3.0)        # 1.35م/3ث = 0.45 م/ث
        turn = compute_turn_rate_dps(360.0, 5.14)     # ≈70 د/ث
        store.save(CalibrationProfile(
            name="سيراميك", speeds={"50": 0.30, "75": speed75, "100": 0.60},
            turn_rate_dps=turn, note="اختبار"))
        store2 = CalibrationStore(directory=d)         # محاكاة إعادة التشغيل
        check("بعد المعايرة الملف موجود", store2.any_exists() is True)
        profile = store2.load("سيراميك")
        check("الملف يُعاد تحميله بقيمه",
              abs(profile.speed_for_power(75) - 0.45) < 1e-9 and profile.name == "سيراميك",
              f"speed@75={profile.speed_for_power(75):.3f} م/ث")

    # (3) مسح كامل 4×3 → 100%
    print("\nب) المسح الشبكي:")
    room = Room(4.0, 3.0, "back_left")
    grid = OccupancyGrid(room)
    summ = Scanner(grid, SimWorld(room)).run_sim()
    check("غرفة 4×3 → تغطية 100%", summ["coverage_pct"] == 100.0, summ["coverage_text"])

    # (4) دائرة عدم اليقين تكبر
    print("\nج) عدم اليقين وتصحيح الجدران:")
    dr = DeadReckoning(room, profile, 0.25, 0.25, 0.0)
    u0 = dr.uncertainty
    dr.move("F", 75, 2.0)
    dr.turn(90)
    check("الشك يكبر مع الحركة والدوران", dr.uncertainty > u0,
          f"{u0:.3f}→{dr.uncertainty:.3f} م")

    # (5) الصندوق الوهمي: لا تصحيح
    dr_box = DeadReckoning(room, profile, 1.5, 0.25, 0.0)   # يواجه الأمام، المتوقع 3.75م
    r_box = dr_box.try_front_wall_correction(0.3)           # «صندوق» على 0.3م
    check("الصندوق الوهمي (وسط الغرفة) → لا تصحيح، يُعامل عائقاً",
          not r_box["corrected"] and r_box["reason"] == "implausible_treat_as_obstacle",
          r_box["reason"])
    dr_mis = DeadReckoning(room, profile, 1.5, 3.5, 45.0)   # محاذاة خاطئة
    check("محاذاة خارج ±15° → لا تصحيح",
          not dr_mis.try_front_wall_correction(0.5)["corrected"])

    # (6) جدار حقيقي: تصحيح + تصغير الدائرة
    dr_w = DeadReckoning(room, profile, 1.5, 3.5, 0.0)      # يواجه الأمام، المتوقع 0.5م
    dr_w.uncertainty = 0.6
    r_wall = dr_w.try_front_wall_correction(0.55)           # فرق 0.05 ضمن الحد
    check("جدار حقيقي محاذٍ → تصحيح مطبَّق ومسجَّل",
          r_wall["corrected"] and len(dr_w.corrections) == 1, f"delta={r_wall['delta']}")
    check("الدائرة تصغر عند التصحيح", dr_w.uncertainty < 0.6, f"→{dr_w.uncertainty:.3f} م")

    # (7) الممر المسدود: إعادة تخطيط تكمل التغطية
    print("\nد) إعادة التخطيط حول العوائق:")
    room7 = Room(2.0, 1.5, "back_left")                     # 4×3 خلايا
    grid7 = OccupancyGrid(room7)
    sc7 = Scanner(grid7, SimWorld(room7, obstacles={(1, 0), (1, 1)}))
    s7 = sc7.run_sim()
    c7 = grid7.counts()
    check("ممر مسدود → التغطية تكتمل عبر طريق بديل",
          s7["coverage_pct"] == 100.0 and c7["blocked"] == 2, s7["coverage_text"])

    # (8) هدف محاصر → unreachable بلا حلقة لا نهائية
    room8 = Room(2.5, 2.5, "back_left")                     # 5×5
    grid8 = OccupancyGrid(room8)
    sc8 = Scanner(grid8, SimWorld(room8, obstacles={(1, 2), (2, 1), (2, 3), (3, 2)}))
    s8 = sc8.run_sim(max_steps=5000)
    check("هدف محاصر (2,2) → unreachable + المسح يكتمل",
          grid8.get(2, 2).unreachable and s8["coverage_pct"] == 100.0,
          f"unreachable={grid8.counts()['unreachable']}")

    # (9) إيقاف واستئناف من نفس الشبكة
    print("\nهـ) الاستئناف والشذوذ:")
    room9 = Room(4.0, 3.0, "back_left")
    grid9 = OccupancyGrid(room9)
    world9 = SimWorld(room9)
    Scanner(grid9, world9).run_sim(max_steps=10)            # جزئي
    partial = grid9.counts()["visited"]
    grid9b = OccupancyGrid.from_dict(json.loads(json.dumps(grid9.to_dict())))
    s9 = Scanner(grid9b, world9).run_sim()                  # استئناف
    check("إيقاف ثم استئناف يكمل من نفس الشبكة",
          0 < partial < 48 and s9["coverage_pct"] == 100.0, f"جزئي={partial}/48 ثم 100%")

    # (10) الشذوذ: مصدر → كشف + تكثيف الجيران
    room10 = Room(3.0, 3.0, "back_left")
    grid10 = OccupancyGrid(room10)
    s10 = Scanner(grid10, SimWorld(room10, source_xy=(2.5, 2.5),
                                   source_cpm_1m=8000, bg_cpm=22)).run_sim()
    check("مصدر إشعاعي → شذوذ يُكتشف وتُرفع أولوية الجيران", s10["anomalies"] >= 1,
          f"شذوذات={s10['anomalies']}")

    # (11) السلامة التفاعلية: العلم معطّل + منطق القرار سليم
    print("\nو) السلامة التفاعلية (معطّلة، منطق فقط):")
    check("REACTIVE_SAFETY_ENABLED = False (كما نصّ البريف)", REACTIVE_SAFETY_ENABLED is False)
    rs = ReactiveSafety(enabled=False)
    check("عائق أمامي < 30سم (STOP_CM) → توقف", rs.decide(20.0, 1, 1)["action"] == "stop")
    check("IR يسار عائق (0) → انعطاف يمين", rs.decide(100.0, 0, 1)["action"] == "turn_right")
    check("المسار خالٍ → مواصلة", rs.decide(100.0, 1, 1)["action"] == "go")
    esc = EscapeSequence(max_attempts=3)
    maneuvers = [esc.next_maneuver()["maneuver"] for _ in range(12)]
    check("تسلسل التحرر ينتهي بالاستسلام (3 محاولات)", "give_up" in maneuvers)

    # ═══ ترقية السلامة: البنود 1-5 ═══════════════════════════════
    print("\nز) ترقية التفادي (البنود 1-5):")

    # (2) السرعة المتدرّجة — التنازل عبر درجات السلّم
    ladder = [(200, 0.50), (100, 0.40), (60, 0.30), (35, 0.20), (20, 0.0)]
    got = [speed_for_distance(cm)["speed"] for cm, _ in ladder]
    check("السرعة تتنازل مع الاقتراب (سلّم متدرّج)",
          got == [s for _, s in ladder], " → ".join(str(s) for s in got))
    check("فشل القراءة → احترس ولا تقف",
          speed_for_distance(None)["speed"] == SPEED_NO_READING
          and speed_for_distance(None)["rung"] == "no_reading")

    # (5) الوسيط + تصفية القفزات
    check("وسيط 3 قراءات يلغي الشاذّة", median([10, 500, 12]) == 12,
          "median([10,500,12])=12")
    jf = JumpFilter(max_jump_cm=100.0)
    seq = [jf.feed(100), jf.feed(300), jf.feed(300)]
    check("القفزة تُتجاهل حتى تتكرر مرتين", seq == [100, 100, 300], str(seq))
    check("زمن الحلقة 0.08s (بدل 0.15)", REACTIVE_LOOP_S == 0.08)

    # (4) جدول اختيار الجهة (IR في الركنين، 0=عائق)
    rs2 = ReactiveSafety(enabled=False)
    check("IR أيسر فقط → لفّ يميناً", rs2.decide(200, 0, 1)["action"] == "turn_right")
    check("IR أيمن فقط → لفّ يساراً", rs2.decide(200, 1, 0)["action"] == "turn_left")
    check("IR الجهتان → رجوع + لفّ 90°",
          rs2.decide(200, 0, 0)["action"] == "backup_turn"
          and rs2.decide(200, 0, 0)["turn_deg"] == 90.0)
    d_us = rs2.decide(20, 1, 1)
    check("ألترا سونيك فقط → توقف ثم مسح دوراني",
          d_us["action"] == "stop" and d_us["next"] == "smart_avoid")
    check("أولوية IR على الألترا سونيك (بلا مسح)",
          rs2.decide(10, 0, 1)["priority"] == "ir")

    # (3) المسح الدوراني: اتجه نحو الأبعد
    class FakeRover:
        def __init__(self):
            self.heading = 0.0
            self.stops = 0
        def stop(self):        self.stops += 1
        def backward(self, p=None): pass
        def forward(self, p=None):  pass
        def turn(self, d, p=None):  pass
        def turn_by_angle(self, deg, **kw):
            self.heading = (self.heading + deg) % 360.0
            return {"turned_deg": deg}

    def sampler_for(rover, right_cm, left_cm):
        def s():
            h = round(rover.heading) % 360
            return right_cm if h == 30 else (left_cm if h == 330 else 100)
        return s

    rv = FakeRover()
    res = ReactiveSafety(enabled=False).smart_avoid(rv, sampler_for(rv, 200, 50))
    check("مسح دوراني → اليمين أبعد → لفّ يميناً",
          res["decision"] == "turn_right" and res["right_cm"] == 200 and res["left_cm"] == 50,
          f"يمين={res['right_cm']} يسار={res['left_cm']}")
    rv2 = FakeRover()
    res2 = ReactiveSafety(enabled=False).smart_avoid(rv2, sampler_for(rv2, 45, 220))
    check("المسح يختار اليسار حين يكون الأبعد", res2["decision"] == "turn_left")
    rv3 = FakeRover()
    res3 = ReactiveSafety(enabled=False).smart_avoid(rv3, sampler_for(rv3, 20, 25))
    check("الجهتان مسدودتان → لفّ 180° + إعادة تخطيط",
          res3["decision"] == "replan_180", res3["reason"][:40])

    # سقوط تلقائي للمنطق البسيط عند تعطيل المسح الذكي
    rv4 = FakeRover()
    rs_simple = ReactiveSafety(enabled=False, smart_avoid=False)
    res4 = rs_simple.smart_avoid(rv4, sampler_for(rv4, 200, 50))
    check("SMART_AVOID_ENABLED=False → منطق بسيط بلا أخطاء",
          res4["decision"] == "simple_stop" and rs_simple.decide(20, 1, 1)["next"] == "simple")
    check("إيقاف مضمون بعد كل مناورة (finally)", rv.stops > 0 and rv4.stops > 0)

    # الخلاصة
    passed = sum(_results)
    total = len(_results)
    print(f"\n=== النتيجة: {passed}/{total} نجح ===")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
