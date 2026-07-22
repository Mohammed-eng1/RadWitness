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

from pi.config import REACTIVE_SAFETY_ENABLED
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
    check("عائق أمامي < 25سم → توقف", rs.decide(20.0, 1, 1)["action"] == "stop")
    check("IR يسار عائق (0) → انعطاف يمين", rs.decide(100.0, 0, 1)["action"] == "turn_right")
    check("المسار خالٍ → مواصلة", rs.decide(100.0, 1, 1)["action"] == "go")
    esc = EscapeSequence(max_attempts=3)
    maneuvers = [esc.next_maneuver()["maneuver"] for _ in range(12)]
    check("تسلسل التحرر ينتهي بالاستسلام (3 محاولات)", "give_up" in maneuvers)

    # الخلاصة
    passed = sum(_results)
    total = len(_results)
    print(f"\n=== النتيجة: {passed}/{total} نجح ===")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
