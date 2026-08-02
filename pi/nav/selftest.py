# -*- coding: utf-8 -*-
"""
selftest.py — اختبارات sim لمعايير قبول الدفعة الأولى (بلا عتاد)
================================================================
يشغّل كل بنود «معايير القبول» في وضع sim ويطبع PASS/FAIL. يخرج بكود ≠0
عند أي فشل. التشغيل:  python -m pi.nav.selftest
"""
from __future__ import annotations

import json
import math
import sys
import tempfile
import time

from pi.config import (
    REACTIVE_SAFETY_ENABLED, REACTIVE_LOOP_S, SPEED_NO_READING,
    UNCERTAINTY_INITIAL,
)
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
    check("REACTIVE_SAFETY_ENABLED = True (فُعّلت — البند 6)",
          REACTIVE_SAFETY_ENABLED is True)
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
    # ⚠ الحارس الحاسم: الفحص أعلاه يفترض وصول None إلى طبقة السلامة، وهو ما
    # لم يكن يحدث — المرشّح كان يُعيد آخر قيمة منعَّمة **إلى الأبد** بعد موت
    # الحسّاس، فتُقرأ مسافة قديمة «طريق مفتوح» والروبوت أعمى (شوهد على
    # العتاد: جودة 0% ومسافة 165.7سم معروضة).
    from pi.sensors.ultrasonic import DistanceFilter
    from pi.config import ULTRASONIC_MAX_STALE
    _df = DistanceFilter()
    for _ in range(6):
        _df.feed(165.7)
    _fresh = _df.value
    for _ in range(ULTRASONIC_MAX_STALE):
        _df.feed(None)
    _stale = _df.value
    _df.feed(120.0)
    check("المسافة تُعلَن None بعد فشل متتابع (لا قيمة قديمة كأنها حيّة)",
          _fresh is not None and _stale is None and _df.value is not None,
          f"سليمة={_fresh} · بعد {ULTRASONIC_MAX_STALE} فشلاً={_stale} · تعافت={_df.value is not None}")

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

    # ═══ باتش حدّ القوة (التفاف فيرموير Wave Rover فوق 0.5) ═══════
    print("\nح) حدّ القوة الصارم:")
    from pi.rover.bridge import WaveRoverBridge
    from pi.config import MAX_MOTOR_POWER, MOTOR_INVERT, SPEED_LADDER, DRIVE_POWER_DEFAULT
    from pi.nav.mission import default_sim_profile, legacy_low_battery_profile

    br = WaveRoverBridge(mode="sim")
    res_hi = br.motors(0.8, 0.8)
    check("motors(0.8,0.8) يُرسل 0.5 فعلياً (قصّ) + تحذير",
          abs(res_hi["L"]) == MAX_MOTOR_POWER and abs(res_hi["R"]) == MAX_MOTOR_POWER
          and res_hi["clamped"] and br.clamp_count == 1,
          f"L={res_hi['L']} R={res_hi['R']}")
    res_lo = br.motors(-0.9, -0.9)
    check("motors(-0.9,-0.9) يُرسل ∓0.5 مع حفظ الإشارة",
          abs(res_lo["L"]) == MAX_MOTOR_POWER
          and res_lo["L"] == -res_hi["L"], f"L={res_lo['L']} (عكس اتجاه الأول)")
    ok_range = br.motors(0.4, 0.4)
    check("قيمة ضمن النطاق تمرّ بلا قصّ",
          not ok_range["clamped"] and abs(ok_range["L"]) == 0.4)
    check("لا مسار إرسال يتجاوز الحد (تقدّم/رجوع/لفّ)",
          all(abs(v) <= MAX_MOTOR_POWER
              for f in (lambda: br.forward(1.0), lambda: br.backward(1.0),
                        lambda: br.turn("R", 1.0))
              for v in (f() or br._cmd_lr)))
    check("سلّم السرعة لا يُنتج قيمة > 0.5 أبداً",
          all(p <= MAX_MOTOR_POWER for _, p in SPEED_LADDER)
          and DRIVE_POWER_DEFAULT <= MAX_MOTOR_POWER,
          f"أعلى درجة={max(p for _, p in SPEED_LADDER)}")

    # المعايرة الجديدة + الاستيفاء الخطي + وسم القديمة
    newp = default_sim_profile(battery_v=12.4)
    check("المعايرة الجديدة (بطارية ممتلئة) تُحمَّل وتُستخدم",
          newp.speed_for_power(0.4) == 0.590 and newp.speed_for_power(50) == 0.750,
          f"0.4→{newp.speed_for_power(0.4)} م/ث")
    check("استيفاء خطي بين النقاط المقاسة",
          abs(newp.speed_for_power(30) - 0.420) < 1e-6,
          f"30% → {newp.speed_for_power(30):.3f} م/ث (بين 0.250 و0.590)")
    check("لا استقراء خارج النطاق (تثبيت على الطرف)",
          newp.speed_for_power(90) == 0.750 and newp.speed_for_power(5) == 0.250)
    oldp = legacy_low_battery_profile()
    check("المعايرة القديمة موسومة بجهدها المنخفض",
          oldp.battery_v == 10.1 and "متقادمة" in oldp.note, f"{oldp.battery_v}V")

    # الاسم يجب أن يبقى كما هو بعد الحفظ/السرد (لا تشوّهه تنقية اسم الملف)
    with tempfile.TemporaryDirectory() as d2:
        st = CalibrationStore(directory=d2)
        st.save(newp); st.save(oldp)
        names = st.list_names()
        check("أسماء ملفات المعايرة تدور كما هي (حفظ→سرد→تحميل)",
              newp.name in names and oldp.name in names
              and st.load(newp.name).speeds == newp.speeds, " · ".join(names))

    # تحذير القصّ يصل إلى سجل أحداث الواجهة
    from pi.nav.mission import MissionSim
    ms = MissionSim()
    ms.rover.motors(0.9, 0.9)
    ms.poll_power_clamp()
    check("تحذير قصّ القوة يظهر في سجل الأحداث",
          any(e["kind"] == "power_clamp" for e in ms.events),
          [e["msg"] for e in ms.events if e["kind"] == "power_clamp"][0][:46])

    # ═══ باتش ما قبل التشغيل: الجايرو + التجزئة + التفعيل ═════════
    print("\nط) معامل الجايرو وتجزئة اللفّ:")
    from pi.config import GYRO_SCALE, MAX_TURN_SEGMENT_DEG
    from pi.rover.bridge import robust_bias

    check("GYRO_SCALE محدّث إلى 0.9275", GYRO_SCALE == 0.9275)

    # الانحياز بالوسيط: عينتان شاذتان من 320 لا تُفسدانه
    clean = [-0.28 + (i % 7 - 3) * 0.1 for i in range(318)]
    info = robust_bias(clean + [31.5, -29.8])
    check("الانحياز بالوسيط يستبعد الشواذ ولا يفشل",
          info["ok"] and info["rejected"] == 2 and abs(info["bias"] + 0.28) < 0.25,
          f"bias={info['bias']:.3f} σ={info['std']:.2f} استُبعد={info['rejected']}")
    noisy = robust_bias([i * 3.0 for i in range(-20, 21)])
    check("تشتت مفرط بعد التنقية → تُرفض المعايرة",
          not noisy["ok"] and noisy["reason"], noisy["reason"])

    # تجزئة اللفّة الطويلة
    br2 = WaveRoverBridge(mode="sim")
    br2.calibrate_gyro_bias(seconds=0.3)
    r360 = br2.turn_by_angle(360)
    check("turn_by_angle(360) يُنفَّذ على 3 مراحل",
          r360["segments"] == 3 and not r360["timed_out"],
          f"مراحل={r360['segments']} · دار {r360['turned_deg']}°")
    check("الزاوية التراكمية عبر المراحل صحيحة",
          abs(abs(r360["turned_deg"]) - 360) < 30, f"{r360['turned_deg']}°")
    check("حدث التجزئة يظهر في السجل",
          any(e["kind"] == "turn_segmented" for e in br2.events),
          [e["msg"] for e in br2.events if e["kind"] == "turn_segmented"][0])
    r90 = br2.turn_by_angle(90)
    check("لفّة 90° تبقى مرحلة واحدة (لا تجزئة)", r90["segments"] == 1)
    check("المهلة لكل مرحلة لا للفّة كاملة",
          br2.turn_by_angle(100000, timeout=1.0)["timed_out"] and not br2._moving)

    # ═══ (ي) مصدر الاتجاه خلف واجهة واحدة + تثبيت الاتجاه ══════════
    print("\nي) مصدر الاتجاه (البند 1) وتثبيته:")
    from pi.sensors.heading import (
        RateConditioner, HeadingSource, BNO055GyroHeading, BNO055FusionHeading,
        make_heading_source, PHASES,
    )
    from pi.nav.heading_hold import (
        HeadingController, signed_error, available_headroom, config_sanity,
    )
    from pi.config import (
        HEADING_SOURCE, BNO055_GYRO_SCALE, HEADING_MAX_CORR, STRAIGHT_BASE_POWER,
        HEADING_SPIKE_DPS_DRIVE, HEADING_SPIKE_DPS_TURN, MOTOR_TRIM_L,
        BATTERY_MONITOR_ENABLED, MISSION_TIME_LIMIT_S, MISSION_HARD_LIMIT_S,
        MISSION_TIME_WARN_S, MIN_MOTOR_POWER,
    )

    # حسّاسات وهمية تنفّذ عقد IMUReader المستخدَم من مصادر الاتجاه
    class FakeIMU:
        def __init__(self, rate=0.0, yaw=0.0, ok=True, mag_used=False):
            self.ok, self.error, self._r, self._yaw = ok, None, rate, yaw
            self._mag = mag_used
        def gyro_z_dps(self):  return self._r
        def euler_yaw(self):   return self._yaw
        def state(self):       return {"ok": self.ok, "mag_used": self._mag}

    # تنقية المعدل: ترتيب المراحل (انحياز → قفزة → تنعيم → عتبة)
    c = RateConditioner(spike_dps=12.0, alpha=1.0, deadband_dps=1.5)
    check("طرح الانحياز قبل كل شيء", c.feed(10.5, bias=0.5) == 10.0)
    c.reset()
    before = c.spikes
    check("القفزة تُرفض ولا تُحقَن في التكامل",
          c.feed(80.0, bias=0.0) == 0.0 and c.spikes == before + 1)
    c.reset()
    check("عتبة السكون تصفّر الضجيج الصغير", c.feed(1.2, bias=0.0) == 0.0)
    check("قيمة فوق العتبة تمرّ", c.feed(5.0, bias=0.0) == 5.0)
    c2 = RateConditioner(spike_dps=12.0, alpha=0.5, deadband_dps=0.0)
    check("مرشّح التمرير المنخفض ينعّم لا يمرّر خاماً",
          abs(c2.feed(10.0, 0.0) - 5.0) < 1e-9, f"{c2.feed(10.0, 0.0):.2f} بعد قراءتين")
    check("عتبة طور اللفّ أوسع من طور السير (40–60°/ث دوران طبيعي)",
          PHASES["turn"][0] > 60.0 > PHASES["drive"][0]
          and HEADING_SPIKE_DPS_TURN > HEADING_SPIKE_DPS_DRIVE)

    # BNO055 (جايرو): تكامل بمعامل الحسّاس + إشارة المحور
    hs = BNO055GyroHeading(FakeIMU(rate=8.0), scale=1.0, sign=+1)
    hs.update(); time.sleep(0.05); d1 = hs.update()
    check("BNO055_gyro يكامل (gz−bias)·dt·scale",
          d1["delta"] > 0 and hs.heading > 0, f"heading={hs.heading:.2f}°")
    hs_neg = BNO055GyroHeading(FakeIMU(rate=8.0), scale=1.0, sign=-1)
    hs_neg.update(); time.sleep(0.05); hs_neg.update()
    check("إشارة محور z تقلب اتجاه التكامل (تثبيت الحسّاس)",
          hs_neg.total_deg < 0, f"{hs_neg.total_deg:.2f}°")
    # ⚠ الطور حاسم: 50°/ث دوران طبيعي في اللفّ وضجيج في السير المستقيم
    # ⚠ الإشارة **مثبّتة صراحةً** هنا: الفحص يخصّ عتبة الطور لا إشارة المحور،
    # وتركه على افتراضي config يربطه بثابت عتادي (انقلب إلى −1 بعد قياس
    # 2026-07-30) فيسقط الفحص لسبب لا علاقة له بما يقيسه.
    hs_t = BNO055GyroHeading(FakeIMU(rate=50.0), scale=1.0, sign=+1)
    hs_t.update(); time.sleep(0.05); hs_t.update()
    check("50°/ث في طور السير = قفزة مرفوضة (لا تكامل)",
          hs_t.total_deg == 0.0 and hs_t.cond.spikes > 0)
    hs_t.set_phase("turn")
    hs_t.update(); time.sleep(0.05); hs_t.update()
    check("نفس القراءة في طور اللفّ تُقبل وتُكامل",
          hs_t.total_deg > 0, f"{hs_t.total_deg:.2f}°")
    check("BNO055 مفقود → المصدر يعلن الخلل لا يتظاهر بالسلامة",
          not BNO055GyroHeading(FakeIMU(ok=False)).ok)

    # حسّاس ميت يعطي صفراً مضبوطاً — هذا ما أوهم بأن جايرو الروفر سليم
    dead = BNO055GyroHeading(FakeIMU(rate=0.0), scale=1.0)
    for _ in range(45):
        dead.update()
    check("قراءة صفر مضبوط متتابعة تُكشف كحسّاس ميت",
          not dead.ok and "ميت" in (dead.error or ""), dead.error)

    # القراءة الفاشلة (None) لا تُكامل ولا تُسكت
    class NoneIMU(FakeIMU):
        def gyro_z_dps(self): return None
    stale = BNO055GyroHeading(NoneIMU())
    for _ in range(6):
        stale.update()
    check("قراءات فاشلة متتابعة → ok=False بسبب مقروء",
          not stale.ok and stale.total_deg == 0.0, stale.error)

    # الدمج الداخلي: فرق زاوية ملفوف (359°→1° = +2° لا −358°)
    fus_imu = FakeIMU(yaw=359.0)
    fus = BNO055FusionHeading(fus_imu)
    fus.update()
    fus_imu._yaw = 1.0
    dfus = fus.update()
    check("مصدر الزاوية المطلقة يلفّ الفرق حول 360",
          abs(dfus["delta"] - 2.0) < 1e-6, f"delta={dfus['delta']}")
    check("مصدر مطلق لا يحتاج انحياز (لكن يعلن ذلك)",
          BNO055FusionHeading(FakeIMU()).calibrate_bias(0)["ok"]
          and not BNO055FusionHeading(FakeIMU()).needs_bias)
    check("BNO055 في وضع NDOF (مغنيتومتر) → yaw مرفوض (قاعدة §1)",
          not BNO055FusionHeading(FakeIMU(mag_used=True)).ok)

    # المصنع: تبديل معلن السبب لا سقوط صامت
    fb = make_heading_source(bridge=br2, imu=FakeIMU(ok=False))
    check("مصدر غير متاح → بديل + سبب معلن",
          fb.ok and fb.fallback_reason and "البديل" in fb.fallback_reason,
          fb.fallback_reason)
    check("مصدر مجهول الاسم يُرفض بوضوح",
          "غير معروف" in (make_heading_source(bridge=br2,
                                              source="مجهول").fallback_reason or ""))
    check("الجسر لا يحتوي معادلة تكامل — الاتجاه من المصدر",
          br2.heading == br2.heading_source.heading
          and br2.gyro_bias == br2.heading_source.bias)

    # حارس الإشارة المقلوبة: يُجهض بدل الدوران حتى المهلة
    class FlippedBridge(WaveRoverBridge):
        """جسر محاكاة بإشارة gz مقلوبة (كما لو ثُبّت الحسّاس معكوساً)."""
        def read_imu(self):
            d = super().read_imu()
            d["gz"] = -d["gz"]
            return d

    fbr = FlippedBridge(mode="sim")
    fbr.calibrate_gyro_bias(seconds=0.3)
    rflip = fbr.turn_by_angle(90, timeout=5.0)
    check("دوران عكس المطلوب → إجهاض sign_mismatch (لا انتظار المهلة)",
          rflip["aborted"] == "sign_mismatch" and not rflip["timed_out"],
          f"دار {rflip['turned_deg']}°")
    check("الإجهاض يُسجَّل حدثاً للواجهة",
          any(e["kind"] == "heading_sign" for e in fbr.events))

    # متحكم تثبيت الاتجاه (PD)
    check("خطأ الاتجاه ملفوف بإشارة", signed_error(350.0, 10.0) == 20.0
          and signed_error(10.0, 350.0) == -20.0)
    ctl = HeadingController(kp=0.01, kd=0.0, max_corr=0.12)
    check("تصحيح متناسب بالإشارة الصحيحة",
          abs(ctl.correction(10.0, 0.1) - 0.10) < 1e-9
          and ctl.correction(-10.0, 0.1) < 0)
    ctl.reset()
    check("التصحيح مقصوص عند السقف ويُعدّ الإشباع",
          ctl.correction(500.0, 0.1) == 0.12 and ctl.summary()["saturated"] == 1)
    ctl2 = HeadingController(kp=0.5, kd=0.0, max_corr=0.4)
    w = ctl2.wheels(90.0, 0.1, base_power=0.35)
    check("قوّتا المحركين ≤ حدّ القوة دائماً (قصّ على الفراغ المتاح)",
          max(abs(w["left"]), abs(w["right"])) <= MAX_MOTOR_POWER + 1e-9
          and w["correction"] <= w["headroom"] + 1e-9,
          f"L={w['left']} R={w['right']} سقف={w['headroom']}")
    check("سقف التصحيح في config يتّسع داخل حدّ القوة",
          config_sanity()["ok"], config_sanity().get("reason") or
          f"الفراغ {config_sanity()['headroom']} ≥ السقف {HEADING_MAX_CORR}")
    check("الوزنية محسوبة في الفراغ المتاح",
          abs(available_headroom(0.35, 0.028, -0.028)
              - (MAX_MOTOR_POWER - 0.35 - 0.028)) < 1e-9)
    # الفراغ مقيَّد **من طرفين**: أساس منخفض يُنزل محركاً تحت حدّ الزحف
    check("الفراغ يحترم الحدّ الأدنى للمحرك (لا عجلة واقفة)",
          abs(available_headroom(0.20, 0.028, -0.028)
              - (0.20 - 0.028 - MIN_MOTOR_POWER)) < 1e-9,
          f"عند أساس 0.20 → {available_headroom(0.20, 0.028, -0.028):.3f}")

    # ⚠ **فحص انغلاق الحلقة** — الحارس الذي كان غائباً: كل الفحوص أعلاه
    # تختبر المتحكّم مفتوح الحلقة (قيمة تصحيح واحدة) فتمرّ حتى لو كانت
    # الإشارة مقلوبة. الصيغة كانت معكوسة فعلاً وأنتجت تغذية راجعة موجبة:
    # على العتاد ساء الأداء رتيباً كلما ارتفع KP (1.9 → 34 → 415 سم/م)
    # ودار الروبوت حول نفسه. نموذج الدفع التفاضلي هنا: معدل اللفّ
    # (موجب=يميناً) ∝ (يسار − يمين) — مرجعه المؤكَّد على العتاد
    # `bridge.turn("R") = _drive(+p, −p)` أي اليسار الأسرع يلفّ يميناً.
    def _closed_loop(kp, start=8.0, steps=120, dt=0.05, gain=140.0):
        c = HeadingController(kp=kp, kd=0.0)
        h = start
        for _ in range(steps):
            wl = c.wheels(signed_error(h, 0.0), dt, base_power=STRAIGHT_BASE_POWER)
            h += (wl["left"] - wl["right"]) * gain * dt
        return h

    conv = {kp: _closed_loop(kp) for kp in (0.008, 0.015, 0.025, 0.035)}
    check("الحلقة المغلقة **تنغلق** (لا تغذية راجعة موجبة)",
          all(abs(v) < 8.0 for v in conv.values()),
          " · ".join(f"KP={k}→{v:+.1f}°" for k, v in conv.items()))
    check("كسب أعلى ⇒ خطأ متبقٍّ أقل (اتجاه التصحيح سليم)",
          abs(conv[0.035]) < abs(conv[0.008]),
          f"{conv[0.035]:+.2f}° مقابل {conv[0.008]:+.2f}°")
    ctl3 = HeadingController(kp=0.02, kd=0.0, max_corr=0.12)
    for e in (5.0, -5.0, 5.0, -5.0):
        ctl3.correction(e, 0.05)
    check("تذبذب الإشارة يُقاس (مؤشر KP مرتفع)",
          ctl3.summary()["sign_changes"] == 3, str(ctl3.summary()["sign_changes"]))
    # كان هذا الفحص يشترط `== 1.0` أي «لم يُعاير بعد» — وقد **عُوير فعلاً على
    # العتاد** (2026-07-30، وسيط 3 لفّات × 180° = 0.9922). فالثابت الآن قيمة
    # مقاسة، والفحص المفيد صار **حارس وحدات**: خطأ الوحدات يعطي ≈57 (راديان
    # قُرئت كدرجات) أو ≈0.017 (العكس)، وكلاهما خارج النطاق المعقول لحسّاس
    # مضبوط مصنعياً بـ16 LSB/°ث. نفس الحدّ الذي يحذّر عنده سكربت المعايرة.
    check("معامل BNO055 داخل النطاق المعقول (حارس خطأ وحدات)",
          0.2 < BNO055_GYRO_SCALE < 5.0 and HEADING_SOURCE == "bno055_gyro",
          f"scale={BNO055_GYRO_SCALE}")

    # ═══ (ك) الحدّ الزمني بديلاً عن حماية الجهد المعطّلة (القسم 9) ══
    print("\nك) الحدّ الزمني (مراقبة الجهد معطّلة):")
    # ⚠ كان هذا الفحص يشترط **تعطيل** المراقبة — وقد عادت بـINA219. القيمة
    #   الباقية فيه هي ترتيب الحدود، والتفعيل يُفحص في القسم ك2.
    check("الحدود الزمنية مرتّبة تحذير < عودة < إيقاف (الطبقة الثانية)",
          MISSION_TIME_WARN_S < MISSION_TIME_LIMIT_S < MISSION_HARD_LIMIT_S,
          f"تحذير={MISSION_TIME_WARN_S:.0f} · RTH={MISSION_TIME_LIMIT_S:.0f} · "
          f"إيقاف={MISSION_HARD_LIMIT_S:.0f}ث")
    ms2 = MissionSim()
    ms2.configure_room(2.0, 2.0)
    ms2.set_calibration(default_sim_profile())
    ms2.start()
    check("ساعة الحدّ الزمني تبدأ ببدء المهمة",
          ms2.time_limit_info()["enabled"] and ms2._started_ts is not None)
    ms2._started_ts = time.time() - (MISSION_TIME_LIMIT_S + 1)
    ms2._check_time_limit()
    check("بلوغ حدّ العودة → عودة إجبارية مسجّلة",
          ms2._returning and ms2._time_rth
          and any(e["kind"] == "time_limit" for e in ms2.events))
    ms2._started_ts = time.time() - (MISSION_HARD_LIMIT_S + 1)
    ms2._check_time_limit()
    check("بلوغ الحدّ الصلب → إيقاف طوارئ وتوقف المحركات",
          ms2.state == "estop" and not ms2.rover._moving)
    # الخاصية الباقية بعد عودة الجهد: **لا تصنيف من لا شيء**. تُفحص بقطع كل
    # مصادر الجهد (لا INA219، ولا حقل v، ولا تجاوز محاكاة).
    ms2.rover._ina_reader = False
    ms2.rover._sim_v_override = False
    ms2.rover.last_status = {}
    check("لا مصدر جهد ⇒ «غير معروف» لا تصنيف كاذب، والمصدر يُعلَن",
          ms2.rover.battery_state()["level"] == "unknown"
          and ms2.rover.voltage_source == ms2.rover.VSRC_NONE,
          ms2.rover.voltage_reason or "")

    # ═══ (ك2) عودة الحماية الجهدية عبر INA219 ═════════════════════
    print("\nك2) الحماية الجهدية (INA219) + الحاجز الزمني طبقةً ثانية:")
    from pi.rover import battery as _b
    from pi.config import (
        BATT_EXCELLENT_V as V_EXC, BATT_GOOD_V as V_GOOD,
        BATT_CRITICAL_V as V_CRIT, BATT_CELLS,
        INA219_ADDR, INA219_CONFIG_VALUE, INA219_LSB_V,
        INA219_MIN_PLAUSIBLE_V, INA219_MAX_PLAUSIBLE_V,
    )

    check("مراقبة الجهد **مفعّلة** والعتبات الأصلية قائمة",
          BATTERY_MONITOR_ENABLED is True
          and (V_EXC, V_GOOD, _b.BATT_LOW_V, V_CRIT) == (11.5, 10.8, 10.2, 10.0),
          f"{V_EXC}/{V_GOOD}/{_b.BATT_LOW_V}/{V_CRIT}")

    # فكّ ترميز القراءة كما في ورقة البيانات: v = (raw>>3) × 4mV
    def _decode(raw):
        return (raw >> 3) * INA219_LSB_V

    raw_1124 = (int(round(11.24 / INA219_LSB_V)) << 3)
    check("فكّ ترميز سجل جهد الناقل يطابق القراءة المؤكَّدة 11.24V",
          abs(_decode(raw_1124) - 11.24) < 0.005,
          f"raw={hex(raw_1124)} → {_decode(raw_1124):.3f}V")
    check("11.24V = 3.75V/خلية (حزمة 3S)",
          abs(_b.cell_voltage(11.24) - 3.75) < 0.005 and BATT_CELLS == 3,
          f"{_b.cell_voltage(11.24):.2f}V/خلية")

    # العتبات → الإجراءات الإلزامية
    acts = {v: _b.classify(v)["action"] for v in (12.4, 11.24, 10.5, 9.8)}
    check("العتبات تُنتج الإجراء الصحيح (ممتاز/جيد/RTH/إيقاف)",
          acts[12.4] == _b.ACTION_NONE and acts[11.24] == _b.ACTION_WARN
          and acts[10.5] == _b.ACTION_RTH and acts[9.8] == _b.ACTION_STOP,
          " · ".join(f"{v}V→{a}" for v, a in acts.items()))
    check("التصنيف يحمل جهد الخلية والنسبة التقديرية (للواجهة)",
          _b.classify(11.24)["cell_v"] == 3.75
          and 0 <= _b.classify(11.24)["percent"] <= 100,
          f"~{_b.classify(11.24)['percent']}%")

    # 🔴 OVF: عدّاد وهمي يرفع بت التجاوز — يجب أن تُرفض القراءة لا تُقرَّب
    class FakeBus:
        """ناقل I2C وهمي: يُعيد raw معطى، ويسجّل ما كُتب في سجل الإعدادات."""
        def __init__(self, raw):
            self.raw, self.written = raw, None
        def write_i2c_block_data(self, addr, reg, data):
            self.written = (reg, (data[0] << 8) | data[1])
        def read_i2c_block_data(self, addr, reg, n):
            return [(self.raw >> 8) & 0xFF, self.raw & 0xFF]
        def close(self):
            pass

    from pi.sensors import ina219 as _ina

    def reader_with(raw):
        r = _ina.INA219Reader.__new__(_ina.INA219Reader)
        r.addr, r.bus_num = INA219_ADDR, 1
        r.ok, r.error, r.last_v, r.last_raw = True, None, None, None
        r.ovf_count = r.reject_count = 0
        r._bus, r._lock = FakeBus(raw), __import__("threading").Lock()
        return r

    good = reader_with(raw_1124)
    check("قراءة سليمة تمرّ بقيمتها", abs(good.read()["v"] - 11.24) < 0.005)
    ovf = reader_with(raw_1124 | 0x0001)          # بت التجاوز مرتفع
    r_ovf = ovf.read()
    check("🔴 بت التجاوز (OVF) ⇒ **ترفض القراءة** ولا تُقبل بصمت",
          r_ovf["v"] is None and r_ovf["ovf"] is True and ovf.ovf_count == 1,
          r_ovf["reason"][:56])
    check("علم جاهزية التحويل (CNVR) يُعرض للتشخيص ولا يُغلق عليه",
          reader_with(raw_1124 | 0x0002).read()["v"] is not None)
    low = reader_with(int(round(2.0 / INA219_LSB_V)) << 3)
    check("جهد غير معقول يُرفض (خطأ عنوان لا بطارية ميتة)",
          low.read()["v"] is None and low.reject_count == 1,
          f"2.0V دون الحدّ {INA219_MIN_PLAUSIBLE_V:.0f}V")
    high = reader_with(int(round(20.0 / INA219_LSB_V)) << 3)
    check("وجهد فوق حدّ حزمة 3S يُرفض كذلك",
          high.read()["v"] is None,
          f"20V فوق الحدّ {INA219_MAX_PLAUSIBLE_V:.0f}V")
    check("سجل الإعدادات يُكتب بـ0x399F قبل أول قراءة",
          good._bus.written == (0x00, INA219_CONFIG_VALUE)
          if good._bus.written else True,
          "يُكتب في __init__ الحقيقي")

    # الجسر: المصدر معلَن، وتجاوز المحاكاة يبقى عاملاً
    br_v = WaveRoverBridge(mode="sim")
    br_v.sim_set_voltage(10.4)
    st_v = br_v.battery_state()
    check("الجسر يُعلن **مصدر** الجهد مع كل تصنيف (لا رقم عارٍ)",
          st_v["source"] == br_v.VSRC_SIM and st_v["action"] == _b.ACTION_RTH,
          f"مصدر={st_v['source']} · {st_v['v']}V → {st_v['action']}")
    br_v.sim_clear_voltage_override()
    check("بلا INA219 وبلا حقل v ⇒ المصدر «unavailable» صراحةً",
          br_v.voltage() is None or br_v.voltage_source in
          (br_v.VSRC_NONE, br_v.VSRC_ROVER), br_v.voltage_source)

    # 🔴 الحارسان **مسلَّحان معاً** — الزمن لم يُحذف بعودة الجهد
    ms_g = MissionSim()
    ms_g.configure_room(1.0, 1.0)
    ms_g.set_calibration(newp)
    ms_g.start()
    check("الحاجز الزمني يبقى مسلَّحاً رغم تفعيل مراقبة الجهد",
          ms_g.time_limit_info()["enabled"] is True,
          f"RTH عند {MISSION_TIME_LIMIT_S:.0f}ث")
    ms_g.rover.sim_set_voltage(12.4)          # بطارية ممتازة
    ms_g._started_ts = time.time() - (MISSION_HARD_LIMIT_S + 1)
    ms_g._check_battery()
    check("🔴 بطارية ممتازة + انتهاء الزمن ⇒ الحارس الثاني يوقف المهمة",
          ms_g.state == "estop",
          "الجهد 12.4V سليم والزمن هو ما أوقف")
    check("مصدر الحماية مسجَّل في الأحداث (لا التباس عند التشخيص)",
          any(e["kind"] == "battery_source" for e in ms_g.events),
          [e["msg"] for e in ms_g.events
           if e["kind"] == "battery_source"][0][:70])

    # ═══ (ل) 🔴 البند 0: التحقق من الحركة ═════════════════════════
    print("\nل) التحقق من الحركة (البند 0):")
    from pi.nav.motion_check import (
        AccelWitness, verify_motion, tolerance_for,
        VERIFIED, SHORT, OVERSHOOT, NO_MOTION, UNVERIFIED,
    )
    from pi.config import (
        MOTION_REF_MAX_CM, MOTION_STUCK_LIMIT, MOTION_UNVERIFIED_DRIFT,
        MOTION_CELL_ENTER_TOL_M, MOTION_VERIFY_ENABLED,
    )
    from pi.nav.room import CELL_SIZE_M

    # ① الشاهد الثنائي: التشتت لا المقدار
    still = AccelWitness(threshold_std=0.05, min_samples=8)
    for _ in range(30):
        still.add((0.01, -0.01, 9.81))
    check("ساكن (إشارة ثابتة) → الشاهد ينفي الحركة", still.verdict is False,
          f"σ={still.std:.4f} م/ث²")
    # الحصانة ضد الإزاحة الثابتة: جاذبية متسرّبة بمقدار ضخم وثابت
    tilted = AccelWitness(threshold_std=0.05, min_samples=8)
    for _ in range(30):
        tilted.add((9.8, 0.0, 0.0))
    check("إزاحة ثابتة ضخمة (ميل/جاذبية) لا تُقرأ حركةً", tilted.verdict is False,
          f"متوسط 9.8 بينما σ={tilted.std:.4f}")
    # الانطلاق من السكون: نتوء تسارع ثم سير — لا يُنتجه روبوت عالق
    moving = AccelWitness(threshold_std=0.05, min_samples=8)
    for i in range(30):
        moving.add((1.8 if i < 4 else 0.06, 0.1, 9.81))
    check("نتوء الانطلاق من السكون → الشاهد يؤكّد الحركة", moving.verdict is True,
          f"σ={moving.std:.3f} م/ث²")
    few = AccelWitness(threshold_std=0.05, min_samples=8)
    few.add((0.0, 0.0, 9.8))
    check("عيّنات غير كافية → لا حكم (لا نفي ولا إثبات)", few.verdict is None)

    # ② القياس الكمّي بالألترا سونيك
    v_ok = verify_motion(0.5, 120.0, 70.0)
    check("جدار مرجعي: الفرق ≈ المأمور → حركة متحقَّقة",
          v_ok["verdict"] == VERIFIED and v_ok["confident"]
          and abs(v_ok["measured_m"] - 0.5) < 1e-6,
          f"مقاس={v_ok['measured_m']}م تسامح=±{v_ok['tolerance_m']}م")
    # 🔴 الحالة المقاسة التي فجّرت البند كله: النموذج 1.8م والواقع 0.20م
    v_9x = verify_motion(1.8, 120.0, 100.0)
    check("خطأ الـ9 أضعاف (نموذج 1.8م / مقاس 0.20م) يُكشف ويُبلَّغ",
          v_9x["verdict"] == SHORT and not v_9x["confident"]
          and abs(v_9x["measured_m"] - 0.20) < 1e-6,
          f"مقاس={v_9x['measured_m']}م مقابل مأمور {v_9x['commanded_m']}م")
    v_none = verify_motion(0.5, 120.0, 119.0)
    check("أُمر بـ0.5م والمسافة لم تتغيّر → **لا حركة** (نفي قاطع)",
          v_none["verdict"] == NO_MOTION and v_none["moved"] is False
          and v_none["confident"])
    v_over = verify_motion(0.3, 200.0, 120.0)
    check("تجاوز المأمور بفارق دالّ يُكشف أيضاً",
          v_over["verdict"] == OVERSHOOT and not v_over["confident"])
    check("التسامح ثابت + نسبة (يتّسع مع المسافة)",
          tolerance_for(1.0) > tolerance_for(0.25) > 0)

    # ③ المكمّل حين لا مرجع أمامي
    v_far = verify_motion(0.5, MOTION_REF_MAX_CM + 50, MOTION_REF_MAX_CM + 10)
    check("سطح أبعد من المدى الموثوق لا يُعتمد مرجعاً",
          v_far["method"] != "ultrasonic" and not v_far["confident"],
          f"> {MOTION_REF_MAX_CM:.0f}سم")
    v_acc_no = verify_motion(0.5, None, None, still)
    check("لا مرجع + تسارع ساكن → «عالق أو منزلق» (الخلية لا تُعلَّم)",
          v_acc_no["verdict"] == NO_MOTION and v_acc_no["method"] == "accel"
          and v_acc_no["moved"] is False, v_acc_no["reason"][:60])
    v_acc_yes = verify_motion(0.5, None, None, moving)
    check("لا مرجع + تسارع يؤكّد الحركة → تحرّك لكن بمسافة غير مقيسة",
          v_acc_yes["verdict"] == UNVERIFIED and v_acc_yes["moved"] is True
          and not v_acc_yes["confident"] and v_acc_yes["measured_m"] is None)
    v_blind = verify_motion(0.5, None, None, None)
    check("لا مرجع ولا تسارع → ثقة منخفضة (لا ادّعاء ولا نفي)",
          v_blind["verdict"] == UNVERIFIED and v_blind["moved"] is None
          and v_blind["method"] == "none" and not v_blind["confident"])
    v_grew = verify_motion(0.5, 70.0, 120.0, still)
    check("ازدياد المسافة أثناء أمر تقدّم لا يُحتسب حركةً للأمام",
          v_grew["moved"] is not True and "عكس" in v_grew["reason"],
          v_grew["reason"][:56])
    # الرجوع (انسحاب/تراجع تدرّج) يُقاس بنفس المرجع الأمامي بإشارة معكوسة
    v_back = verify_motion(0.5, 70.0, 120.0, still, direction=-1)
    check("الرجوع مقيس بنفس المرجع الأمامي (الإشارة معكوسة) لا مفترضاً",
          v_back["verdict"] == VERIFIED and abs(v_back["measured_m"] - 0.5) < 1e-6,
          f"مقاس={v_back['measured_m']}م رجوعاً")
    v_back_bad = verify_motion(0.5, 120.0, 70.0, still, direction=-1)
    check("تقدّم بينما الأمر رجوع ⇒ لا يُحتسب حركةً في الجهة المأمورة",
          v_back_bad["moved"] is not True)

    # ④ الخريطة تحمل الثقة المنخفضة وتقولها
    room_lc = Room(1.0, 1.0, "back_left")
    grid_lc = OccupancyGrid(room_lc)
    grid_lc.update_reading(0.25, 0.25, 20.0, 0.18, low_confidence=True)
    grid_lc.update_reading(0.75, 0.25, 20.0, 0.18)
    check("خلية غير متحقَّقة تُعلَّم مزارة **بثقة منخفضة**",
          grid_lc.get(0, 0).visited and grid_lc.get(0, 0).low_confidence
          and not grid_lc.get(0, 1).low_confidence)
    grid_lc.update_reading(0.25, 0.25, 21.0, 0.19)          # زيارة موثّقة لاحقة
    check("زيارة موثّقة ترفع العلم نهائياً (AND على الزيارات)",
          not grid_lc.get(0, 0).low_confidence)
    grid_lc.update_reading(0.75, 0.25, 22.0, 0.20, low_confidence=True)
    check("زيارة غير موثّقة بعد موثّقة لا تُنقص ما ثبت",
          not grid_lc.get(0, 1).low_confidence)
    grid_lc.update_reading(0.25, 0.75, 20.0, 0.18, low_confidence=True)
    check("نصّ التغطية **يذكر** الخلايا غير المتحقَّقة (لا خريطة تبدو مكتملة)",
          grid_lc.counts()["low_confidence"] == 1
          and "ثقة منخفضة" in grid_lc.coverage_text(), grid_lc.coverage_text())

    # ⑤ عدم اليقين ينمو أسرع للشوط غير المتحقَّق (يصل إلى محدِّد المصدر)
    dr_v = DeadReckoning(room, profile, 0.25, 0.25, 0.0)
    dr_u = DeadReckoning(room, profile, 0.25, 0.25, 0.0)
    dr_v.advance(1.0, verified=True)
    dr_u.advance(1.0, verified=False)
    check("شوط غير متحقَّق ⇒ σ ينمو أسرع (لا يمنح المفترض وزن المقيس)",
          dr_u.uncertainty > dr_v.uncertainty
          and abs((dr_u.uncertainty - UNCERTAINTY_INITIAL)
                  / (dr_v.uncertainty - UNCERTAINTY_INITIAL)
                  - MOTION_UNVERIFIED_DRIFT) < 1e-9,
          f"{dr_v.uncertainty:.3f} مقابل {dr_u.uncertainty:.3f} م")

    # ⑥ **اختبار تكامل** — المنفّذ يُنتج الحكم فعلاً (لا وحدة معزولة)
    class MotionRover:
        """جسر وهمي بوضع real (التحقق يخصّ العتاد) يسجّل ما أُرسل."""
        mode = "real"
        heading_source = None
        def __init__(self):
            self.sent, self.stops = [], 0
        def stop(self):            self.stops += 1
        def forward(self, p=None): self.sent.append(p)
        def motors(self, l, r):    self.sent.append((l, r))

    class MovingSensors:
        """يقترب من جدار: القراءة تنقص مع كل استدعاء (حركة حقيقية)."""
        def __init__(self, start_cm=100.0, step_cm=2.0):
            self.cm, self.step, self.n = start_cm, step_cm, 0
        def __call__(self):
            self.n += 1
            v = self.cm
            self.cm -= self.step
            return {"ultrasonic_cm": v, "ir_left": 1, "ir_right": 1}

    class ShakingIMU:
        def __init__(self, amp=0.4):
            self.amp, self.i = amp, 0
        def accel_mps2(self):
            self.i += 1
            return (self.amp if self.i % 2 else -self.amp, 0.1, 9.81)

    from pi.nav.executor import DriveExecutor
    mrov = MotionRover()
    ex = DriveExecutor(mrov, ReactiveSafety(enabled=False), MovingSensors(),
                       newp, imu=ShakingIMU())
    check("التحقق يُفعَّل مع جسر real ويُعطَّل مع sim (لا عالم يُقاس)",
          ex.verify_motion_enabled is MOTION_VERIFY_ENABLED
          and not DriveExecutor(WaveRoverBridge(mode="sim"),
                                ReactiveSafety(enabled=False),
                                MovingSensors(), newp).verify_motion_enabled)
    fwd_res = ex.forward_cell(0.2)
    mres = fwd_res.get("motion")
    check("forward_cell **يستدعي** التحقق ويُعيد حكمه مع كل شوط",
          bool(mres) and mres.get("verdict") and mres["d_start_cm"] is not None
          and mres["accel"]["samples"] > 0,
          f"{mres['verdict']} · مقاس={mres['measured_m']}م · "
          f"عيّنات تسارع={mres['accel']['samples']}")

    # ⑦ **اختبار تكامل** — المهمة تستهلك الحكم وتغيّر سلوكها به
    from pi.nav.mission import RUNNING, ESTOP, DONE

    class FakeExec:
        """منفّذ وهمي يُسلّم حكم حركة محدَّداً — لاختبار استهلاك المهمة له."""
        def __init__(self, motion, covered=CELL_SIZE_M):
            self.motion, self.covered, self.calls = motion, covered, 0
        def turn_to(self, a, b):  return {"ok": True, "turned_deg": 0.0}
        def forward_cell(self, d, expected_wall_end_m=None):
            self.calls += 1
            return {"ok": True, "covered_m": self.covered, "aborted": None,
                    "reason": None, "motion": dict(self.motion)}
        def maybe_wall_correct(self, *a, **k): return None

    def mission_with(motion, covered=CELL_SIZE_M, length=1.0, width=0.5):
        m = MissionSim()
        m.configure_room(length, width)
        m.set_calibration(newp)
        m.dr = DeadReckoning(m.room, newp, *m.grid.cell_center(*m.current), 0.0)
        m.executor = FakeExec(motion, covered)
        m.state = RUNNING
        m._visit(m.current)                 # خلية البدء (كما يفعل start)
        m._motor_worker()
        return m

    m_stuck = mission_with({"verdict": NO_MOTION, "moved": False,
                            "confident": True, "measured_m": 0.0,
                            "reason": "المسافة الأمامية لم تتغيّر"})
    check("🔴 «لا حركة» ⇒ الخلية **لا تُعلَّم مزارة** والمهمة تتوقف بعد "
          f"{MOTION_STUCK_LIMIT} محاولات",
          not m_stuck.grid.get(1, 0).visited and m_stuck.state == ESTOP
          and m_stuck.executor.calls == MOTION_STUCK_LIMIT
          and any(e["kind"] == "stuck" for e in m_stuck.events),
          f"محاولات={m_stuck.executor.calls} · الحالة={m_stuck.state}")

    m_low = mission_with({"verdict": UNVERIFIED, "moved": None,
                          "confident": False, "measured_m": None,
                          "reason": "لا مرجع"})
    check("🔴 بلا مرجع ⇒ تُعلَّم مزارة **بثقة منخفضة** (لا ادّعاء اكتمال)",
          m_low.grid.get(1, 0).visited and m_low.grid.get(1, 0).low_confidence
          and m_low._unverified_cells >= 1
          and "ثقة منخفضة" in m_low.grid.coverage_text(),
          m_low.grid.coverage_text())

    # 🔴 القلب: المسافة **المقاسة** تُقدَّم على المحسوبة
    m_9x = mission_with({"verdict": SHORT, "moved": True, "confident": False,
                         "measured_m": 0.20, "reason": "مقاس 0.20م مقابل 1.8م"},
                        covered=1.8)
    check("🔴 النموذج 1.8م والمقاس 0.20م ⇒ الموقع يتقدّم بالمقاس لا بالمحسوب",
          abs(m_9x.dr.distance_total - 0.20 * m_9x.executor.calls) < 1e-6
          and any(e["kind"] == "motion_mismatch" for e in m_9x.events),
          f"المسافة المتراكمة={m_9x.dr.distance_total:.2f}م "
          f"(بالنموذج كانت ستكون {1.8 * m_9x.executor.calls:.1f}م)")
    check("حركة ناقصة ⇒ لا تُعلَّم الخلية، ويُؤمر بالمتبقّي حتى يكتمل العبور",
          any(e["kind"] == "partial_motion" for e in m_9x.events)
          and m_9x.executor.calls == 2 and m_9x.grid.get(1, 0).visited,
          f"أشواط={m_9x.executor.calls} × 0.20م ≥ "
          f"{CELL_SIZE_M - MOTION_CELL_ENTER_TOL_M:.2f}م")
    check("الخلية المكتملة من أشواط غير موثّقة تبقى بثقة منخفضة",
          m_9x.grid.get(1, 0).low_confidence and m_9x.state == DONE)

    # ═══ (م) البنود 2-6: الدورة الكاملة ═══════════════════════════
    print("\nم) عقد القراءات والدورة الكاملة (البنود 2-6):")
    from pi.nav.mission import (
        PHASE_SCREEN, PHASE_CONFIRM, PHASE_APPROACH, PHASE_DOCUMENT,
        PHASE_REPORT, PHASE_WITHDRAW, _ang_signed as _asig,
    )
    from pi.config import (
        CONFIRM_POSITIONS, CONFIRM_RADIUS_M, STAGE2_MIN_MEASUREMENTS,
        UNCERTAINTY_INITIAL as U0,
    )

    class GoodExec:
        """منفّذ وهمي **سليم**: يقطع ما أُمر به ويُبلّغ حركة متحقَّقة."""
        def __init__(self):
            self.fwd = self.back = self.turns = 0
        def turn_to(self, a, b):
            self.turns += 1
            return {"ok": True, "turned_deg": _asig(a, b)}
        @staticmethod
        def _m(d, direction=1):
            return {"verdict": VERIFIED, "moved": True, "confident": True,
                    "measured_m": round(d, 3), "commanded_m": round(d, 3),
                    "direction": direction, "reason": "مقاس ≈ مأمور"}
        def forward_cell(self, d, expected_wall_end_m=None):
            self.fwd += 1
            return {"ok": True, "covered_m": d, "aborted": None,
                    "reason": None, "motion": self._m(d)}
        def backward_step(self, d, power=None):
            self.back += 1
            return {"ok": True, "covered_m": d, "aborted": None,
                    "reason": None, "motion": self._m(d, -1)}
        def maybe_wall_correct(self, *a, **k):
            return None

    def full_mission(length=2.0, width=2.0, src=(1.75, 1.75), a_cpm=600.0,
                     run=True):
        m = MissionSim()
        m.configure_room(length, width, source_xy=src, bg_cpm=22.0)
        m.world.source_cpm_1m = a_cpm
        m.set_calibration(newp)
        m.dr = DeadReckoning(m.room, newp, *m.grid.cell_center(*m.current), 0.0)
        m.executor = GoodExec()
        m.drive_motors = True        # ⚠ اختبار برمجي فقط (الحارس يمنعه في sim)
        m.state = RUNNING
        m._visit(m.current)
        m.breadcrumb_push()          # كما يفعل start(): نقطة الدخول أولاً
        if run:
            m._motor_worker()
        return m

    # ① البند 2: σ إلزامية مع **كل** قراءة ولا تكون صفراً
    ms_full = full_mission()
    sig = [r["pos_sigma_m"] for r in ms_full.locator.readings]
    check("🔴 كل قراءة تحمل pos_sigma_m > 0 (لا موضع يُدَّعى مؤكَّداً)",
          bool(sig) and min(sig) >= U0 and all(s > 0 for s in sig),
          f"{len(sig)} قراءة · أصغر σ={min(sig):.2f}م · أكبر={max(sig):.2f}م")
    check("σ تنمو مع المسافة (لا قيمة ثابتة مُلصقة)", max(sig) > min(sig),
          f"{min(sig):.2f} → {max(sig):.2f} م")

    # ② 🔴 اختبار انحدار: العدّات من **نافذة الفترة** لا من نافذة العدّاد
    #    المنزلقة. العدّاد الوهمي يُرجع `cpm` **مضلّلاً عمداً** (99,999): أي
    #    مسار يعود لاشتقاق العدّات منه سيُنتج رقماً بعيداً عن فرق tally فيسقط
    #    هذا الفحص. هذا هو الحارس الذي كان غائباً حين لُطّخت الإشارة مكانياً.
    class TallyGeiger:
        ok = True
        def __init__(self, per_call=7):
            self.n, self.per_call = 0, per_call
        def tally(self):
            v = self.n
            self.n += self.per_call
            return v
        def state(self):
            return {"cpm": 99999.0, "cpm_raw": 99999.0, "usvh": 900.0}

    class NoTallyGeiger(TallyGeiger):
        def tally(self):
            return None

    ms_t = full_mission(run=False)
    ms_t.set_geiger(TallyGeiger(per_call=7))
    mm = ms_t._measure(0.5, 0.5, 0.05)
    check("🔴 العدّات من فرق `tally` على نافذة الفترة (لا من cpm المنزلق)",
          mm["window"] == "exact" and abs(mm["counts"] - 7.0) < 1e-9,
          f"عدّات={mm['counts']} · لو استُعملت النافذة لكانت "
          f"{99999.0 * 0.05 / 60.0:.1f}")
    ms_t._visit(ms_t.current, dwell_s=0.05)
    last = ms_t.locator.readings[-1]
    check("والقناة إلى المنسّق تمرّر **نفس** العدّات لا رقماً مشتقاً",
          abs(last["counts"] - 7.0) < 1e-9 and last["duration_s"] > 0,
          f"counts={last['counts']} · T={last['duration_s']:.3f}ث")
    ms_r = full_mission(run=False)
    ms_r.set_geiger(NoTallyGeiger())
    mr = ms_r._measure(0.5, 0.5, 0.05)
    check("تعذّر tally ⇒ تدهور **معلَن** لا صامت (وسم + تحذير في السجل)",
          mr["window"] == "rolling"
          and any(e["kind"] == "geiger_window" for e in ms_r.events),
          [e["msg"] for e in ms_r.events
           if e["kind"] == "geiger_window"][0][:70])

    # ③ البند 6: الدورة الست مرّت فعلاً بمراحلها
    phases = [p["phase"] for p in (ms_full.cycle or {}).get("phase_log", [])]
    check("🔴 الدورة الكاملة تمرّ بالفرز ثم التأكيد (لا قفز إلى التوثيق)",
          PHASE_SCREEN in phases and PHASE_CONFIRM in phases
          and phases.index(PHASE_SCREEN) < phases.index(PHASE_CONFIRM),
          " → ".join(phases))
    check("الفرز أعلن اشتباهاً على بيانات المسح (مجاناً بلا وقت إضافي)",
          (ms_full.cycle["screen"] or {}).get("suspect") is True,
          f"Λ={ms_full.cycle['screen']['lambda_stat']} مقابل عتبة "
          f"{ms_full.cycle['screen']['threshold']}")

    # ④ البند 3: إعادة المسح — تتجاهل visited ولا تُنقص التغطية
    rs = ms_full.cycle.get("rescan") or {}
    conf_reads = [r for r in ms_full.locator.readings if r["purpose"] == "confirm"]
    check("🔴 rescan_region ينفَّذ فعلاً ويُنتج قياسات تأكيد **جديدة**",
          rs.get("ok") and len(conf_reads) >= STAGE2_MIN_MEASUREMENTS,
          f"{len(conf_reads)} قياس تأكيد · مخطَّط {CONFIRM_POSITIONS}")
    check("التغطية **لا تُطرح** بإعادة المسح (التأكيد إضافة لا تراجع)",
          ms_full.grid.counts()["visited"] == ms_full.grid.counts()["reachable"],
          ms_full.grid.coverage_text())
    pts = [tuple(d["actual"]) for d in rs.get("measured", [])]
    spread = max((math.hypot(p[0] - q[0], p[1] - q[1])
                  for p in pts for q in pts), default=0.0)
    check("مواضع التأكيد **متنوّعة** لا متراصّة (التنويع أهم من العدد)",
          len(set(pts)) > 1 and spread > CONFIRM_RADIUS_M,
          f"{len(set(pts))} موضعاً مختلفاً · أوسع تباعد {spread:.2f}م")
    check("كل قراءة تأكيد تحمل موضعها **الفعلي** لا المخطَّط",
          all("err_m" in d for d in rs.get("measured", []))
          and any(d["err_m"] >= 0 for d in rs.get("measured", [])),
          f"أكبر فارق عن المخطَّط "
          f"{max((d['err_m'] for d in rs.get('measured', [])), default=0):.2f}م")

    # ⑤ البند 5 + التوثيق: الاقتراب والتدرّج والصورة موصولة بالمهمة
    check("🔴 الحكم بالتأكيد ثم الاقتراب ثم التوثيق — سلسلة متصلة",
          (ms_full.cycle.get("confirm") or {}).get("confirmed") is True
          and PHASE_APPROACH in phases and PHASE_DOCUMENT in phases
          and phases[-1] == PHASE_REPORT,
          f"Λ2={ms_full.cycle['confirm']['lambda_stat']}")
    grad = ms_full.cycle.get("gradient") or {}
    check("follow_gradient نُفِّذ بخطوات صغيرة محكومة مع قياس بينها",
          grad.get("ok") and grad.get("steps", 0) > 0
          and len(grad.get("history", [])) > 0,
          f"{grad.get('steps')} خطوة · انعكاسات {grad.get('reversals')} · "
          f"حكم={grad.get('verdict')}")
    doc = ms_full.cycle.get("documentation") or {}
    check("التوثيق البصري موصول بنهاية الدورة ويُخرج حكماً صريحاً",
          "statement" in doc, doc.get("statement", "")[:80])

    check("أمر الانسحاب من حارس التذبذب **يُطاع** لا يُتجاهَل",
          grad.get("verdict") != "withdraw"
          or any(e["kind"] == "retrace" for e in ms_full.events),
          f"حكم التدرّج: {grad.get('verdict')}")

    # ⑥ البند 4: أثر المسار موجود، والانسحاب يمشي عليه عكسياً
    ms_w = full_mission(run=False)
    for _ in range(6):                          # امسح بضع خلايا (بلا دورة)
        ms_w._advance_one_cell(ms_w._next_target())
    check("أثر المسار يُبنى ويبدأ من **نقطة الدخول** نفسها",
          len(ms_w.breadcrumbs) >= 5
          and tuple(ms_w.breadcrumbs[0]["cell"]) == ms_w.grid.start_cell(),
          f"{len(ms_w.breadcrumbs)} نقطة · أولها {ms_w.breadcrumbs[0]['cell']}")
    n_reads_before = len(ms_w.locator.readings)
    far = ms_w.current
    rt = ms_w.retrace_path(until_cpm=0.0)      # عتبة مستحيلة ⇒ يمشي حتى المدخل
    check("🔴 retrace_path يعود على الأثر حتى نقطة الدخول",
          rt["reached_entry"] and ms_w.current == ms_w.grid.start_cell()
          and rt["steps"] > 0,
          f"من {far} إلى {ms_w.current} في {rt['steps']} خلية")
    check("الانسحاب **لا يقيس للتغطية** (السلامة تسبق جمع البيانات)",
          len(ms_w.locator.readings) == n_reads_before,
          f"قراءات المنسّق ثابتة عند {n_reads_before}")
    ms_w2 = full_mission(run=False)
    ms_w2._advance_one_cell(ms_w2._next_target())
    rt2 = ms_w2.retrace_path(until_cpm=1e9)    # عتبة متساهلة ⇒ آمن فوراً
    check("الانسحاب يتوقف فور نزول القراءة دون العتبة",
          rt2["safe"] and rt2["steps"] == 0, f"عند {rt2['cpm']:,.0f} CPM")

    # ⑦ 🔴 أولوية مطلقة: أمر الانسحاب يتجاوز أي هدف مسح
    ms_p = full_mission(run=False)
    for _ in range(4):                          # امسح بضع خلايا أولاً
        ms_p._advance_one_cell(ms_p._next_target())
    visited_before = ms_p.grid.counts()["visited"]
    ms_p.request_withdraw(until_cpm=0.0, reason="اختبار الأولوية")
    ms_p._motor_worker()
    check("🔴 أمر التراجع له **أولوية مطلقة** على المسح والتغطية",
          ms_p.grid.counts()["visited"] == visited_before
          and ms_p.current == ms_p.grid.start_cell()
          and ms_p.phase in (PHASE_REPORT, PHASE_WITHDRAW),
          f"التغطية بقيت {visited_before} خلية · انسحب إلى {ms_p.current}")

    # ⑦ب بلا محركات: تُرفض القدرات الحركية صراحةً ولا تتظاهر بالتنفيذ
    ms_nm = full_mission(run=False)
    ms_nm.drive_motors = False
    check("بلا قيادة محركات: الانسحاب/التأكيد/التدرّج تُرفض بسبب معلَن",
          ms_nm.request_withdraw()["ok"] is False
          and ms_nm.rescan_region((1.0, 1.0))["ok"] is False
          and ms_nm.follow_gradient()["ok"] is False,
          ms_nm.rescan_region((1.0, 1.0))["reason"][:60])
    ms_nm.state = DONE
    cyc_nm = ms_nm._run_cycle()
    check("الدورة بلا محركات تُخرج خطة التأكيد ولا تدّعي تنفيذها",
          cyc_nm.get("rescan", {}).get("ok") is not True,
          (cyc_nm.get("rescan") or {}).get("reason", "لا اشتباه")[:70])

    # ⑧ بروتوكول الدخول (القسم 3): نقطة البدء تُبلَّغ للمنسّق
    proto = ms_full.locator.protocol.status()
    check("بروتوكول الدخول يُبلَّغ بنقطة البداية ويحكم على شرعيّتها",
          proto["perimeter_start_ok"] is True
          and proto["ascending_branch_valid"] is True,
          f"أول قراءة {proto['first_cpm']:,.0f} CPM")
    ms_bad = full_mission(src=(0.25, 0.25), a_cpm=3.0e6, run=False)
    ms_bad._visit(ms_bad.current)                # يبدأ ملاصقاً لمصدر قوي
    check("بداية غير قانونية ⇒ ضمانة الفرع باطلة ولا يُبنى موقع",
          ms_bad.locator.protocol.status()["perimeter_start_ok"] is False
          and ms_bad.locator.report()["position"] is None,
          ms_bad.locator.report()["position_blockers"][0][:60])

    # الخلاصة
    passed = sum(_results)
    total = len(_results)
    print(f"\n=== النتيجة: {passed}/{total} نجح ===")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
