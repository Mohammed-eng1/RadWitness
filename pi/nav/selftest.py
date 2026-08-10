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
    # ⚠ العدد يُشتقّ من الثابت لا يُثبَّت: رُفع الحدّ 120→180 (2026-08-10)
    #    لخفض كلفة لفّات الـ180 الشائعة، ورقم مثبت هنا يكسر الاختبار بلا
    #    علاقة بما يقيسه (أن اللفّة الطويلة **تُجزَّأ** أصلاً).
    _exp_seg = math.ceil(360.0 / MAX_TURN_SEGMENT_DEG)
    check(f"turn_by_angle(360) يُنفَّذ على {_exp_seg} مراحل (حدّ "
          f"{MAX_TURN_SEGMENT_DEG:.0f}°)",
          r360["segments"] == _exp_seg and not r360["timed_out"],
          f"مراحل={r360['segments']} · دار {r360['turned_deg']}°")
    check("ولفّة 180° صارت مرحلة واحدة (نصف كلفة التوقف والتصحيح)",
          math.ceil(180.0 / MAX_TURN_SEGMENT_DEG) == 1,
          f"حدّ المرحلة {MAX_TURN_SEGMENT_DEG:.0f}°")
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
        make_heading_source, PHASES, ZERO_RUN_DEAD,
    )
    from pi.config import HEADING_RECOVERY_ATTEMPTS
    from pi.nav.heading_hold import (
        HeadingController, signed_error, available_headroom, config_sanity,
    )
    from pi.config import (
        HEADING_SOURCE, BNO055_GYRO_SCALE, MPU6050_GYRO_SCALE,
        HEADING_MAX_CORR, STRAIGHT_BASE_POWER,
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
    # 🔴 انقلب الشرط 2026-08-06: كان يشترط عتبة السير **دون** 60°/ث —
    #    والمقاس أن الاضطراب الميكانيكي أثناء السير (عجلة تعلق = فرق قوة
    #    كامل) يدور 40–60°/ث **حقيقيةً لا ضجيجاً**، فعمي المتحكّم عن
    #    الدوران وكاد الروبوت يصدم الجدار. العتبة فوق الاضطراب بهامش ×2.
    check("عتبة السير تغطي الاضطراب الممكن (40–60°/ث) بهامش، ودون عتبة اللفّ",
          PHASES["drive"][0] >= 2.0 * 60.0
          and HEADING_SPIKE_DPS_TURN > HEADING_SPIKE_DPS_DRIVE)
    # ⚠ حارس مقاس: ذروة الدوران ببطارية مشحونة عند TURN_POWER=0.40 بلغت
    #    **198°/ث** بينما العتبة كانت 200 — أي 99% منها. فوق العتبة يُعيد
    #    المرشّح صفراً لا القراءة، فيتجمّد التكامل في أسرع لحظة من اللفّة.
    #    العتبة يجب أن تسبق أسرع دوران ممكن بهامش، لا أن تلامسه.
    check("عتبة اللفّ تسبق أسرع دوران مقاس (221°/ث) بهامش ≥2×",
          HEADING_SPIKE_DPS_TURN >= 2.0 * 221.0,
          f"{HEADING_SPIKE_DPS_TURN:.0f}°/ث مقابل 221°/ث مقاسة")

    # BNO055 (جايرو): تكامل بمعامل الحسّاس + إشارة المحور
    hs = BNO055GyroHeading(FakeIMU(rate=8.0), scale=1.0, sign=+1)
    hs.update(); time.sleep(0.05); d1 = hs.update()
    check("BNO055_gyro يكامل (gz−bias)·dt·scale",
          d1["delta"] > 0 and hs.heading > 0, f"heading={hs.heading:.2f}°")
    hs_neg = BNO055GyroHeading(FakeIMU(rate=8.0), scale=1.0, sign=-1)
    hs_neg.update(); time.sleep(0.05); hs_neg.update()
    check("إشارة محور z تقلب اتجاه التكامل (تثبيت الحسّاس)",
          hs_neg.total_deg < 0, f"{hs_neg.total_deg:.2f}°")
    # ⚠ الإشارة **مثبّتة صراحةً** هنا: الفحص يخصّ عتبة الطور لا إشارة المحور،
    # وتركه على افتراضي config يربطه بثابت عتادي (انقلب إلى −1 بعد قياس
    # 2026-07-30) فيسقط الفحص لسبب لا علاقة له بما يقيسه.
    # 🔴 انقلب هذا الفحص 2026-08-06: كان يثبت أن 50°/ث في السير «قفزة
    #    مرفوضة» — وهو الافتراض الذي كاد يصدم الروبوت بالجدار: اضطراب
    #    ميكانيكي حقيقي (~40-60°/ث) كان يُرمى ضجيجاً فيتجمّد التكامل
    #    والمتحكّم أعمى عن دوران يراه كل من في الغرفة. الاضطراب دوران
    #    **ممكن** فالعتبة فوقه (قاعدة §1: تُقاس على الأسرع الممكن).
    hs_t = BNO055GyroHeading(FakeIMU(rate=50.0), scale=1.0, sign=+1)
    hs_t.update(); time.sleep(0.05); hs_t.update()
    check("اضطراب ميكانيكي 50°/ث أثناء السير **يُكامَل** (لا يُرمى ضجيجاً)",
          hs_t.total_deg > 0 and hs_t.cond.spikes == 0,
          f"{hs_t.total_deg:.2f}°")
    hs_x = BNO055GyroHeading(FakeIMU(rate=300.0), scale=1.0, sign=+1)
    hs_x.update(); time.sleep(0.05); hs_x.update()
    check("300°/ث في طور السير = خرافية مرفوضة (فوق عتبة 120)",
          hs_x.total_deg == 0.0 and hs_x.cond.spikes > 0)
    hs_x.set_phase("turn")
    hs_x.update(); time.sleep(0.05); hs_x.update()
    check("نفس القراءة في طور اللفّ تُقبل وتُكامل",
          hs_x.total_deg > 0, f"{hs_x.total_deg:.2f}°")
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
    # ⚠ كان يشترط `HEADING_SOURCE == "bno055_gyro"` — والشريحة **تلفت**
    #   (5.40V على حدّ 3.6V) واستُبدلت بـMPU-6050. الحارس الباقي هو نفسه:
    #   خطأ الوحدات يعطي ≈57 (راديان قُرئت درجات) أو ≈0.017 (العكس).
    check("معامل MPU-6050 داخل النطاق المعقول (حارس خطأ وحدات)",
          0.2 < MPU6050_GYRO_SCALE < 5.0 and HEADING_SOURCE == "mpu6050",
          f"scale={MPU6050_GYRO_SCALE} · المصدر={HEADING_SOURCE}")

    # ── MPU-6050: نفس عقد المصدر تماماً ──────────────────────────
    from pi.sensors.heading import MPU6050GyroHeading
    hm = MPU6050GyroHeading(FakeIMU(rate=8.0), scale=1.0, sign=+1)
    hm.update(); time.sleep(0.05); hm.update()
    check("mpu6050 يكامل (gz−bias)·dt·scale خلف نفس الواجهة",
          hm.heading > 0 and hm.ok, f"heading={hm.heading:.2f}°")
    hm_neg = MPU6050GyroHeading(FakeIMU(rate=8.0), scale=1.0, sign=-1)
    hm_neg.update(); time.sleep(0.05); hm_neg.update()
    check("إشارة محور z تقلب التكامل (تثبيت الحسّاس — تُقاس باليد)",
          hm_neg.total_deg < 0, f"{hm_neg.total_deg:.2f}°")
    check("MPU مفقود → المصدر يعلن الخلل لا يتظاهر بالسلامة",
          not MPU6050GyroHeading(FakeIMU(ok=False)).ok)
    check("MPU-6050 بلا مغنيتومتر ولا مرجع مطلق — ويُصرَّح بذلك",
          MPU6050GyroHeading(FakeIMU()).state()["mag_used"] is False
          and MPU6050GyroHeading(FakeIMU()).state()["calibrated"] is False)

    # 🔴 لا مصدر ميت يُقبل بديلاً على العتاد الحقيقي
    class RealishBridge(WaveRoverBridge):
        """جسر يدّعي وضع real لاختبار سياسة المصادر (بلا منفذ فعلي)."""
        def __init__(self):
            super().__init__(mode="sim")
            self.mode = "real"

    rb = RealishBridge()
    for dead_src, label in (("bno055_gyro", "BNO055 التالفة"),
                            ("rover_gyro", "جايرو الروفر الميت")):
        s_dead = make_heading_source(bridge=rb, source=dead_src)
        check(f"🔴 {label} **لا يُقبل** مصدراً على العتاد (لا سقوط صامت)",
              s_dead.ok is False and "لا مصدر اتجاه صالح"
              in (s_dead.fallback_reason or ""),
              (s_dead.error or "")[:58])
    check("وفي المحاكاة يبقى البديل الوهمي مشروعاً ومعلَناً",
          make_heading_source(bridge=br2, source="bno055_gyro").ok is True)

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

    # ── 🔴 إشارة التيار: «مجهول» لا يُقرأ «يشحن» ─────────────────
    # شاهد مقاس 2026-08-10: البطارية **مفصولة عن الشاحن** والقراءة
    # `+0.54A ⚡ يشحن · 6.5W` — و6.5W عند 11.97V حِمل راسبري 4 بالضبط.
    # وخطره ليس في العرض: `charging` **يُعفي من الإطفاء المنظَّم**، فإشارة
    # معكوسة تعني أن تفريغاً عادياً يُقرأ شحناً ⇒ الإطفاء لا يُطلق أبداً
    # ⇒ تنقطع التغذية فجأةً عند قطع الحماية أثناء الكتابة على البطاقة.
    class TwoRegBus(FakeBus):
        """ناقل يميّز سجل الجهد عن سجل الشنت (الأول لا يكفي لاختبار الإشارة)."""
        def __init__(self, raw_v, raw_shunt):
            super().__init__(raw_v)
            self.raw_shunt = raw_shunt & 0xFFFF
        def read_i2c_block_data(self, addr, reg, n):
            w = self.raw_shunt if reg == 0x01 else self.raw
            return [(w >> 8) & 0xFF, w & 0xFF]

    def reader_amps(raw_shunt):
        r = reader_with(raw_1124)
        r._bus = TwoRegBus(raw_1124, raw_shunt)
        return r

    from pi.config import INA219_SHUNT_OHM as _SH, INA219_CURRENT_LSB_V as _LSB
    _shunt_pos = int(round(0.54 * _SH / _LSB))    # نفس القراءة المقاسة حيّاً
    r0 = reader_amps(_shunt_pos).read()
    check("🔴 قارئ INA219 **لا يحكم بالشحن إطلاقاً** (الشنت على مسار الحِمل)",
          "charging" not in r0 and r0["load_a"] is not None,
          f"تيار حِمل {r0['load_a']:.2f}A — بلا اتجاه")
    check("والقدرة من المطلق (لا تحتاج إشارة أصلاً)",
          r0["watts"] is not None and r0["watts"] > 0, f"{r0['watts']}W")

    # 🔴 كاشف الشحن من **ميل الجهد** — البديل الوحيد على هذا العتاد
    from pi.config import BATT_CHARGE_QUIET_S as _QUIET
    _Q = _QUIET + 10.0                    # «ساكن منذ وقت طويل»
    _cd = _b.ChargeDetector()
    _t = 1000.0
    check("نافذة غير مكتملة ⇒ **مجهول** لا «لا يشحن»",
          _cd.feed(_t, 11.9, _Q) is False and _cd.state()["unknown"] is True,
          _cd.reason)
    # شحن بالمعدّل المقاس فعلياً (+90 mV/دقيقة) لمدة تتجاوز النافذة
    for i in range(1, 121):
        _cd.feed(_t + i * 0.5, 11.90 + (i * 0.5) * (0.090 / 60.0), _Q)
    check("🔴 شحن حقيقي (+90 mV/دقيقة كما قِيس) يُكشف بلا إشارة تيار",
          _cd.charging is True and _cd.state()["unknown"] is False,
          _cd.reason)
    _cd2 = _b.ChargeDetector()
    for i in range(1, 121):                    # تفريغ ساكن بطيء
        _cd2.feed(_t + i * 0.5, 11.90 - (i * 0.5) * (0.010 / 60.0), _Q)
    check("وتفريغ ساكن بطيء لا يُقرأ شحناً", not _cd2.charging, _cd2.reason)
    _cd3 = _b.ChargeDetector()                 # ضجيج بلا اتجاه (σ≈14mV مقاسة)
    _noise = [0.014, -0.011, 0.008, -0.014, 0.012, -0.006, 0.013, -0.012]
    for i in range(1, 121):
        _cd3.feed(_t + i * 0.5, 11.90 + _noise[i % len(_noise)], _Q)
    check("والضجيج المقاس (σ≈14mV) لا يصنع شحناً كاذباً",
          not _cd3.charging, _cd3.reason)

    # 🔴 الشَرَك الحقيقي: **ارتداد الجهد بعد رفع الحمل** يرتفع كالشحن
    _cd4 = _b.ChargeDetector()
    for i in range(1, 121):                    # نافذة شحن مكتملة أولاً
        _cd4.feed(_t + i * 0.5, 11.90 + (i * 0.5) * (0.090 / 60.0), _Q)
    _was = _cd4.charging
    _cd4.feed(_t + 61.0, 11.60, 0.0)           # أمر حركة الآن (سكون = 0)
    check("🔴 أي حركة **تمسح النافذة** (لا تُخاط عبرها)",
          _was and not _cd4.charging and _cd4.state()["samples"] == 0,
          "ارتداد ما بعد الحمل يرتفع كالشحن تماماً")
    # وارتداد سريع خلال فترة السكون لا يُقبل أصلاً
    _cd5 = _b.ChargeDetector()
    for i in range(1, 60):
        _cd5.feed(_t + i * 0.5, 11.60 + i * 0.004, 2.0)   # سكون 2ث ≪ الحدّ
    check("وعيّنات ما قبل انقضاء السكون تُهمَل كلها",
          _cd5.state()["samples"] == 0 and not _cd5.charging)

    # 🔴 الفحص الحاسم: **منحنى ارتداد أُسّي حقيقي** بعد رفع الحمل — أخطر
    #    شبيه بالشحن على الإطلاق، ويقع بالضبط حين يُركن الروبوت بعد مهمة
    #    منهِكة (أي عند أدنى جهد، حيث يهمّ إعفاء الإطفاء فعلاً).
    def _rebound_charging(tau_s, quiet_s, drop_v=0.30):
        """
        V(t) = V∞ − ΔV·exp(−t/τ) — استرخاء الحزمة بعد رفع الحمل.
        `quiet_s` = حدّ السكون المطبَّق (يُحقن ليُقارَن القديم بالحالي).
        """
        _sv = _b.BATT_CHARGE_QUIET_S
        try:
            _b.BATT_CHARGE_QUIET_S = quiet_s
            d = _b.ChargeDetector()
            for i in range(0, 361):           # 180ث من العيّنات كل 0.5ث
                t_since_stop = quiet_s + i * 0.5
                v = 9.90 - drop_v * math.exp(-t_since_stop / tau_s)
                d.feed(_t + i * 0.5, v, t_since_stop)
                if d.charging:
                    return True, d.reason
            return False, d.reason
        finally:
            _b.BATT_CHARGE_QUIET_S = _sv
    _rb8, _why8 = _rebound_charging(90.0, 8.0)
    check("🔴 وحدّ سكون 8ث **كان يقرأ الارتداد شحناً** (لهذا صار 180)",
          _rb8 is True, f"τ=90ث ⇒ {_why8}")
    _rb180, _why180 = _rebound_charging(90.0, _QUIET)
    check("🔴 وبالحدّ الفعلي يسقط الارتداد بالشرطين معاً (لا إعفاء كاذب)",
          _rb180 is False, f"τ=90ث بعد {_QUIET:.0f}ث ⇒ {_why180}")
    for _tau in (20.0, 40.0, 60.0):
        _rb, _w = _rebound_charging(_tau, _QUIET)
        check(f"وكذلك عند τ={_tau:.0f}ث (مدى ثوابت الزمن المعقولة)",
              _rb is False, _w)
    # ومع ذلك الشحن الحقيقي بعد السكون نفسه يُكشف — الحدّ لم يُعطّل الميزة
    _cd6 = _b.ChargeDetector()
    for i in range(1, 241):
        _cd6.feed(_t + i * 0.5, 11.90 + (i * 0.5) * (0.090 / 60.0),
                  _QUIET + i * 0.5)
    check("والشحن الحقيقي يُكشف بعد انقضاء السكون (الحدّ لم يقتل الميزة)",
          _cd6.charging is True, _cd6.reason)

    # (أثر ادّعاء الشحن على طبقة الإطفاء يُختبر بعد `shutdown_mission` أدناه)

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

    # ── منحنى النسبة + طبقة الإطفاء عند 9.6V ─────────────────────
    from pi.config import (BATT_SHUTDOWN_V, BATT_SHUTDOWN_CONSECUTIVE,
                           INA219_CHARGING_A)
    check("النسبة من **منحنى** لا معادلة خطية (الطرفان حادّان والوسط مسطّح)",
          _b.percent(12.60) == 100 and _b.percent(9.00) == 0
          and abs(_b.percent(11.30) - 45) <= 1,
          f"11.30V → {_b.percent(11.30)}% · 11.00V → {_b.percent(11.00)}%")
    check("هبوط الحمل يُعوَّض بمقاومة الحزمة (النسبة تحت الحمل ليست أدنى كذباً)",
          _b.percent(10.8, amps=2.0) > _b.percent(10.8),
          f"{_b.percent(10.8)}% → {_b.percent(10.8, amps=2.0)}% عند 2A")
    check("العتبات الأصلية **لم تُخفَّض**، و9.6V طبقة **تحت** إيقاف المحركات",
          BATT_SHUTDOWN_V < V_CRIT < _b.BATT_LOW_V < V_GOOD < V_EXC,
          f"إطفاء {BATT_SHUTDOWN_V} < إيقاف {V_CRIT} < عودة {_b.BATT_LOW_V}")
    check("جهد دون 9.6V يُصنَّف إطفاءً منظَّماً",
          _b.classify(9.5)["action"] == _b.ACTION_SHUTDOWN
          and _b.classify(9.8)["action"] == _b.ACTION_STOP,
          f"9.5V→{_b.classify(9.5)['action']} · 9.8V→{_b.classify(9.8)['action']}")

    def shutdown_mission():
        m = MissionSim()
        m.configure_room(1.0, 1.0)
        m.set_calibration(newp)
        m.fired = []
        m._do_shutdown = lambda: m.fired.append(True)   # لا إطفاء فعلي
        return m

    def feed(m, info, times):
        for _ in range(times):
            m._check_shutdown(dict(info))

    base = {"action": _b.ACTION_SHUTDOWN, "v": 9.4, "cell_v": 3.13,
            "charging": False, "source": "ina219"}
    ms_sd = shutdown_mission()
    feed(ms_sd, base, BATT_SHUTDOWN_CONSECUTIVE - 1)
    part = len(ms_sd.fired)
    feed(ms_sd, base, 1)
    check(f"🔴 الإطفاء يحتاج {BATT_SHUTDOWN_CONSECUTIVE} قراءات متتالية "
          f"(هبوط المحركات اللحظي لا يُطفئ الروبوت)",
          part == 0 and len(ms_sd.fired) == 1,
          f"بعد {BATT_SHUTDOWN_CONSECUTIVE - 1} قراءة: {part} · بعد الخامسة: "
          f"{len(ms_sd.fired)}")
    ms_r = shutdown_mission()
    feed(ms_r, base, 3)
    ms_r._check_shutdown({"action": _b.ACTION_NONE, "v": 11.0})   # قراءة سليمة
    feed(ms_r, base, 4)
    check("أي قراءة سليمة تُصفّر العدّاد (لا تراكم عبر انقطاعات)",
          not ms_r.fired, f"3 ثم تصفير ثم 4 = {len(ms_r.fired)} إطفاء")
    ms_c = shutdown_mission()
    feed(ms_c, dict(base, charging=True), BATT_SHUTDOWN_CONSECUTIVE + 2)
    check("قيد الشحن ⇒ **لا إطفاء** (الجهد أثناء الشحن مضلّل)",
          not ms_c.fired, "الشحن مؤكَّد بميل الجهد")
    ms_x = shutdown_mission()
    feed(ms_x, dict(base, source="sim_override"), BATT_SHUTDOWN_CONSECUTIVE + 2)
    check("🔴 مصدر غير INA219 ⇒ لا إطفاء (لا نُطفئ جهازاً على رقم غير مقيس)",
          not ms_x.fired
          and any("ليس INA219" in e["msg"] for e in ms_x.events))

    # 🔴 وأثر إشارة التيار على هذه الطبقة بالذات — وهو الخطر الحقيقي:
    #    `charging` **يُعفي من الإطفاء**، فإشارة معكوسة تعني أن تفريغاً
    #    عادياً يُقرأ شحناً ⇒ الطبقة الرابعة تموت صامتة.
    ms_unk = shutdown_mission()
    feed(ms_unk, dict(base, charging=False, charging_unknown=True),
         BATT_SHUTDOWN_CONSECUTIVE)
    check("🔴 شحن مجهول (نافذة الميل لم تكتمل) **لا يُعطّل** الإطفاء",
          bool(ms_unk.fired),
          "ادّعاء شحن بلا دليل كان سيقتل الطبقة الرابعة بلا أثر")

    # 🔴 والحالة الواقعية: الروبوت يتحرّك ⇒ النافذة ممسوحة دائماً ⇒ الإطفاء
    #    مسلَّح طوال المهمة. وهذا هو السلوك المطلوب حرفياً.
    br_mv = WaveRoverBridge(mode="sim")
    br_mv._moving = True
    for i in range(140):
        br_mv._feed_charge_detector(11.90 + i * 0.001)   # صعود حادّ متعمَّد
    check("🔴 وأثناء الحركة لا يُدّعى شحن مهما صعد الجهد (النافذة ممسوحة)",
          br_mv.battery_charging is False
          and br_mv.charge_detector.state()["samples"] == 0,
          "صعود 140mV أثناء الحركة لم يُقرأ شحناً")
    ms_chg = shutdown_mission()
    feed(ms_chg, dict(base, charging=True, charging_unknown=False),
         BATT_SHUTDOWN_CONSECUTIVE + 2)
    check("وشحن **مؤكَّد** (بإشارة معايرة) يبقى يُعفي كما صُمّم",
          not ms_chg.fired,
          "الجهد أثناء الشحن مضلّل — الإعفاء صحيح حين يُعرف لا حين يُفترض")

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

    # ── 🔋 رفع العودة الإجبارية بخيار المشغّل — **وأرضيته لا تُعبَر** ──
    # الخيار الأمني الذي لا يُختبر حدُّه ليس خياراً بل ثغرة: كل قيمته في
    # أنه يرفع طبقةً **واحدة** ويترك الباقي مسلَّحاً. فنختبر ما لا يمسّه
    # أكثر ممّا يمسّه.
    from pi.config import (BATT_RTH_OVERRIDE_FLOOR_V, BATT_CRITICAL_V,
                           BATT_GOOD_V as _BGV)

    def _batt_mission(v, override):
        m = MissionSim()
        m.configure_room(1.0, 1.0)
        m.set_calibration(newp)
        m.rover.sim_set_voltage(12.4)         # ابدأ سليماً (الحاجب ⑦)
        m.start()
        m.set_batt_rth_override(override)
        m.rover.sim_set_voltage(v)
        m._check_battery()
        return m

    _v_band = round((BATT_RTH_OVERRIDE_FLOOR_V + _BGV) / 2.0, 2)   # وسط النطاق
    m_on = _batt_mission(_v_band, True)
    check("🔋 التجاوز مرفوع ⇒ لا عودة إجبارية داخل النطاق (المهمة تمضي)",
          not m_on._returning and not m_on._rth_triggered
          and m_on.state == "running",
          f"{_v_band}V داخل [{BATT_RTH_OVERRIDE_FLOOR_V}, {_BGV}) — الحالة {m_on.state}")
    check("والكتم **معلَن** في السجل لا صامت (وفيه أرضيته وثمنها)",
          any(e["kind"] == "battery" and "مكتومة" in e["msg"]
              and "إيقاف في المكان" in e["msg"] for e in m_on.events))
    m_off = _batt_mission(_v_band, False)
    check("وبلا التجاوز تنطلق العودة كما كانت (السلوك الافتراضي لم يتغيّر)",
          m_off._returning and m_off._rth_triggered,
          f"{_v_band}V ⇒ عودة إجبارية")

    # 🔴 الأرضية: تحتها **لا شيء قابل للتجاوز**
    m_floor = _batt_mission(BATT_CRITICAL_V - 0.2, True)
    check("🔴 تحت الأرضية: التجاوز مرفوع ومع ذلك **إيقاف فوري**",
          m_floor.state == "estop",
          f"{BATT_CRITICAL_V - 0.2}V < {BATT_RTH_OVERRIDE_FLOOR_V}V ⇒ "
          f"{m_floor.state}")
    check("والأرضية هي عتبة الإيقاف نفسها (لا فجوة بين الطبقتين)",
          BATT_RTH_OVERRIDE_FLOOR_V == BATT_CRITICAL_V,
          f"{BATT_RTH_OVERRIDE_FLOOR_V}V")

    # 🔴 الحاجز الزمني طبقة **مستقلة** — التجاوز لا يمسّها
    m_time = MissionSim()
    m_time.configure_room(1.0, 1.0)
    m_time.set_calibration(newp)
    m_time.rover.sim_set_voltage(12.4)
    m_time.start()
    m_time.set_batt_rth_override(True)
    m_time._started_ts = time.time() - (MISSION_TIME_LIMIT_S + 1)
    m_time._check_battery()
    check("🔴 التجاوز لا يرفع الحاجز الزمني (طبقة مستقلة عمداً)",
          m_time._returning and m_time._time_rth,
          "عودة زمنية نُفّذت رغم رفع عودة الجهد")

    # 🔴 ولا يرفع الإطفاء المنظَّم (الطبقة الرابعة تحت الإيقاف)
    m_sd = shutdown_mission()
    m_sd.set_batt_rth_override(True)
    feed(m_sd, dict(base), BATT_SHUTDOWN_CONSECUTIVE + 1)
    check("🔴 ولا يرفع الإطفاء المنظَّم لنظام التشغيل",
          m_sd.fired, "الإطفاء نُفّذ رغم رفع عودة الجهد")

    # الحاجب ⑦ يسقط بسقوط علّته — لكن **دون الأرضية يبقى حاجباً**
    m_rd = MissionSim()
    m_rd.configure_room(1.0, 1.0)
    m_rd.set_calibration(newp)
    m_rd.rover.sim_set_voltage(_v_band)
    _blk_before = m_rd.mission_readiness()["blockers"]
    m_rd.set_batt_rth_override(True)
    _rd_after = m_rd.mission_readiness()
    check("حاجب «نطاق العودة» يسقط برفع العودة (علّته كانت دورة بدء-وعودة)",
          any("نطاق العودة" in b for b in _blk_before)
          and not any("نطاق العودة" in b for b in _rd_after["blockers"])
          and any("مرفوعة بخيارك" in w for w in _rd_after["warnings"]),
          f"حواجب {len(_blk_before)} → {len(_rd_after['blockers'])}")
    m_rd.rover.sim_set_voltage(BATT_CRITICAL_V - 0.2)
    check("🔴 لكنه يبقى حاجباً دون الأرضية مهما كان الخيار",
          any("نطاق العودة" in b
              for b in m_rd.mission_readiness()["blockers"]),
          "بدء مهمة على جهد الإيقاف الفوري ليس اختياراً")

    # 🔴 التجاوز يبقى بين المهمات ⇒ يُعلَن عند بدء **كل** مهمة
    m_next = MissionSim()
    m_next.configure_room(1.0, 1.0)
    m_next.set_calibration(newp)
    m_next.rover.sim_set_voltage(12.4)
    m_next.set_batt_rth_override(True)
    m_next.events.clear()                     # كأنها جولة جديدة
    m_next.start()
    check("وخيار مرفوع من جولة سابقة يُعاد إعلانه عند بدء كل مهمة",
          any(e["kind"] == "battery" and "مرفوعة" in e["msg"]
              for e in m_next.events),
          "خيار أمان منسيّ أخطر من خيار لم يُتَح")

    # الحالة تُبثّ (وإلا لم يمكن للواجهة أن تعكس الحقيقة ولا أن تُنذر)
    _st_ov = m_on.state_dict()
    check("الحالة تُبثّ مع أرضيتها (الواجهة تعرض الأرضية لا رقماً مكتوباً فيها)",
          _st_ov["batt_rth_override"] is True
          and _st_ov["batt_rth_floor_v"] == BATT_RTH_OVERRIDE_FLOOR_V)

    # ═══ (ك3) الحساسات: خمسة IR · الغائب مجهول · جاهزية المهمة ════
    print("\nك3) أعلام الحساسات وجاهزية المهمة:")
    from pi.config import (IR_PRESENT, IR_PULL_UP, IR_FRONT_MID_GPIO,
                           IR_SIDE_LEFT_GPIO, IR_SIDE_RIGHT_GPIO,
                           IR_SIDE_LOGIC_ENABLED, ULTRASONIC_MIN_PERIOD_S)
    from pi.sensors.proximity import IR_PINS
    from pi.sensors.ultrasonic import GroundEchoDetector

    check("منافذ IR الخمسة معرَّفة بمواضعها",
          len(IR_PINS) == 5 and dict(IR_PINS)["front_mid"] == IR_FRONT_MID_GPIO
          and dict(IR_PINS)["side_left"] == IR_SIDE_LEFT_GPIO
          and dict(IR_PINS)["side_right"] == IR_SIDE_RIGHT_GPIO,
          " · ".join(f"{n}=BCM{p}" for n, p in IR_PINS))
    check("الشدّ المرتفع مفعّل، ومنطق الجانبيين **مؤجَّل** حتى تأكيد المواضع",
          IR_PULL_UP is True and IR_SIDE_LOGIC_ENABLED is False)

    # 🔴 الغائب **مجهول** لا خالٍ
    rs_u = ReactiveSafety(enabled=False)
    d_unk = rs_u.decide(200.0, None, None)
    check("🔴 IR غائب (None) لا يُقرأ «خالياً» بل يُذكر مجهولاً",
          d_unk["action"] == "go" and set(d_unk["unknown"]) >= {"front_left",
                                                               "front_right"}
          and "مجهول" in d_unk["reason"],
          d_unk["reason"][:60])
    check("ولا يُقرأ «عائقاً» أيضاً (لا معلومة عنده — لا يخترع تفادياً)",
          rs_u.decide(200.0, None, 1)["action"] == "go")
    d_mid = rs_u.decide(200.0, 1, 1, 0)
    check("IR الأوسط الجديد يسدّ العمى المركزي بين الركنين",
          d_mid["action"] == "backup_turn" and d_mid["rung"] == "ir_mid",
          d_mid["reason"])
    check("ومنطق الركنين الأصلي لم يتغيّر",
          rs_u.decide(200, 0, 1)["action"] == "turn_right"
          and rs_u.decide(200, 1, 0)["action"] == "turn_left"
          and rs_u.decide(200, 0, 0)["action"] == "backup_turn")

    # كاشف الصدى الأرضي — إعلان لا معالجة
    ge = GroundEchoDetector(min_samples=20)
    for i in range(40):
        ge.add(21.0 + (i % 3) * 0.4 if i % 6 else 182.0)
    check("🔴 عنقودان ضيّقان والروبوت ساكن ⇒ **اشتباه صدى أرضي معلَن**",
          ge.suspect and "تثبيت" in (ge.detail or ""),
          (ge.detail or "")[:70])
    ge_ok = GroundEchoDetector(min_samples=20)
    for i in range(40):
        ge_ok.add(120.0 + (i % 5) * 0.3)
    check("قراءة سليمة (تشتت ~1سم) لا تُطلق إنذاراً", not ge_ok.suspect)
    ge_mv = GroundEchoDetector(min_samples=20)
    for i in range(40):
        ge_mv.add(21.0 if i % 6 else 182.0, moving=True)
    check("والعيّنات أثناء الحركة تُهمَل (تغيّر المسافة حينها حقيقي)",
          not ge_mv.suspect)
    check("حدّ معدل النداء ~16/ث (الصدى السابق يلوّث التالي)",
          abs(ULTRASONIC_MIN_PERIOD_S - 1 / 16) < 1e-6,
          f"{1 / ULTRASONIC_MIN_PERIOD_S:.0f} مرة/ث")

    # 🔴 جاهزية المهمة: الرفض **بسبب معلَن**
    ms_rd = MissionSim()
    ms_rd.configure_room(1.0, 1.0)
    ms_rd.set_calibration(newp)
    rd0 = ms_rd.mission_readiness()
    check("الجاهزية تُبثّ دائماً ومعها الموانع (لا رفض صامت)",
          isinstance(rd0.get("blockers"), list) and "heading_ok" in rd0,
          f"جاهز={rd0['ready']} · اتجاه={rd0['heading_source']}")

    class BlindMission(MissionSim):
        """مهمة بلا استشعار أمامي وبلا مصدر اتجاه — أسوأ حالة."""
        def sensors(self):
            return {"ultrasonic_cm": None, "ir_left": None, "ir_right": None,
                    "ir_mid": None, "cpm": 0.0, "source": "unavailable",
                    "quality": 0}

    ms_bl = BlindMission()
    ms_bl.configure_room(1.0, 1.0)
    ms_bl.set_calibration(newp)
    ms_bl.rover.heading_source.ok = False
    ms_bl.rover.heading_source.error = "MPU-6050 غير متاح"
    rd = ms_bl.mission_readiness()
    check("🔴 بلا اتجاه وبلا استشعار أمامي ⇒ **غير جاهز** بسببين صريحين",
          not rd["ready"] and len(rd["blockers"]) >= 2
          and any("اتجاه" in b for b in rd["blockers"])
          and any("استشعار أمامي" in b for b in rd["blockers"]),
          " · ".join(b[:34] for b in rd["blockers"]))
    ms_bl.drive_motors = True
    res_bl = ms_bl.start()
    check("و«بدء مسح» يرفض **برسالة مقروءة** لا بصمت",
          res_bl["ok"] is False and "تعذّر بدء المسح" in res_bl["error"]
          and any(e["kind"] == "not_ready" for e in ms_bl.events),
          res_bl["error"][:80])
    check("ولا يبدأ المسح رغم النقص (الرفض صحيح)",
          ms_bl.state == "idle")

    # ═══ (ن) 🔴 تصحيح الاتجاه من الجدار الجانبي ═══════════════════
    print("\nن) تصحيح الاتجاه من الجدار الجانبي (المرجع المفقود):")
    from pi.nav.wall_heading import (
        WallHeadingCorrector, tilt_from_delta, RIGHT as W_R, LEFT as W_L,
        OK as W_OK, R_RANGE, R_TRAVEL, R_TILT, R_INCONSISTENT,
    )
    from pi.sensors.ultrasonic_array import perpendicular_cm
    from pi.config import (
        WALL_HEADING_CONSISTENT_N, WALL_HEADING_MAX_TILT_DEG,
        WALL_HEADING_MAX_CORR_DEG, WALL_FOLLOW_MAX_CM,
        WALL_HEADING_SIGMA_SHRINK, SIDE_ULTRASONIC_ENABLED, US_SIDE_TILT_DEG,
        WALL_ALIGN_TOL_DEG as W_ALIGN_TOL,
    )

    check("✅ العلم مفعّل بعد اكتمال بوابة القياس (2026-08-07: σ 0.26–0.55سم "
          "+ جهة مثبتة + جولتا سير حقيقيتان)",
          SIDE_ULTRASONIC_ENABLED is True)
    check("ميل التركيب يُحوَّل إلى مسافة عمودية (لا تُستعمل القراءة خاماً)",
          abs(perpendicular_cm(100.0, 0.0) - 100.0) < 1e-9
          and perpendicular_cm(100.0, US_SIDE_TILT_DEG) < 100.0,
          f"100سم بميل {US_SIDE_TILT_DEG}° → "
          f"{perpendicular_cm(100.0, US_SIDE_TILT_DEG):.1f}سم عمودية")

    # 🔴 الإشارة: خطؤها يُنتج تغذية راجعة موجبة لا تصحيحاً ضعيفاً
    t_r = tilt_from_delta(+10.0, 0.5, W_R)     # جدار يمين · ابتعدنا
    t_l = tilt_from_delta(+10.0, 0.5, W_L)     # جدار يسار · ابتعدنا
    check("ابتعاد عن جدار **يمين** ⇒ تصحيح موجب (لفّ يميناً نحوه)",
          t_r > 0, f"{t_r:+.2f}°")
    check("ونفس الابتعاد عن جدار **يسار** ⇒ تصحيح سالب (الإشارة تنعكس)",
          t_l < 0 and abs(t_l + t_r) < 1e-9, f"{t_l:+.2f}°")
    check("الميل من atan2(Δd, المسافة) — 10سم على 0.5م ≈ 11.3°",
          abs(abs(t_r) - math.degrees(math.atan2(0.1, 0.5))) < 1e-9)

    # مسار سليم: ميل ثابت ⇒ تصحيح بعد N قراءات متسقة
    wc = WallHeadingCorrector(side=W_R)
    dec = None
    for i in range(WALL_HEADING_CONSISTENT_N + 2):
        dec = wc.feed(40.0 + i * 6.0, i * 0.30)   # يبتعد 6سم كل 30سم
    check(f"ميل ثابت ⇒ تصحيح بعد {WALL_HEADING_CONSISTENT_N} قراءات متسقة",
          dec["apply"] and dec["reason"] == W_OK and dec["correction_deg"] > 0,
          f"تصحيح {dec['correction_deg']:+.2f}° من {dec['n_consistent']} قراءات")

    # 🔴 التطبيق: heading يتغيّر **فعلاً** وσ تصغر
    dr_w = DeadReckoning(room, profile, 1.0, 1.0, 90.0)
    dr_w.heading_sigma_deg = 12.0
    res_w = wc.apply_to(dr_w, dec)
    check("🔴 يصحّح **الاتجاه** فعلياً لا الموضع (ما لا يفعله الجدار الأمامي)",
          res_w["applied"] and abs(dr_w.heading - 90.0) > 0.1
          and dr_w.x == 1.0 and dr_w.y == 1.0,
          f"{res_w['heading_before']}° → {res_w['heading_after']}°")
    check("🔴 وσ الاتجاه **تصغر** — وهذا ما يكسر الحلقة المفرغة",
          dr_w.heading_sigma_deg < 12.0
          and abs(dr_w.heading_sigma_deg - 12.0 * WALL_HEADING_SIGMA_SHRINK) < 1e-6,
          f"{res_w['sigma_before']}° → {res_w['sigma_after']}°")
    check("ولا تُصفَّر (القياس نفسه فيه ضجيج — لا ثقة بلا مصدر)",
          dr_w.heading_sigma_deg > 0)
    # وبوابة المحاذاة تُعاد فتحها فعلاً
    dr_g2 = DeadReckoning(room, profile, 1.0, 1.0, 0.0)
    dr_g2.heading_sigma_deg = W_ALIGN_TOL + 6.0
    was_lost = dr_g2.alignment_gate_lost
    WallHeadingCorrector(side=W_R).apply_to(
        dr_g2, {"apply": True, "correction_deg": 1.0, "n_consistent": 3})
    check("🔴 بوابة محاذاة منهارة **تُعاد فتحها** بعد التصحيح",
          was_lost and not dr_g2.alignment_gate_lost,
          f"σ {W_ALIGN_TOL + 6.0:.0f}° → {dr_g2.heading_sigma_deg:.1f}°")
    check("والسجل يحمل كل حقول التصحيح (الشفافية)",
          set(res_w) >= {"side", "angle_deg", "heading_before", "heading_after",
                         "sigma_before", "sigma_after"})

    # ── شروط الأمان: كل رفض بسبب معلَن ──────────────────────────
    wr = WallHeadingCorrector(side=W_R)
    check("قراءة خارج النطاق ⇒ لا تصحيح (فتحة أو عائق لا جدار)",
          wr.feed(WALL_FOLLOW_MAX_CM + 50, 0.0)["reason"] == R_RANGE)
    wr2 = WallHeadingCorrector(side=W_R)
    wr2.feed(40.0, 0.0)
    check("مسافة غير كافية بين القراءتين ⇒ لا تصحيح (تضخيم ضجيج)",
          wr2.feed(41.0, 0.02)["reason"] == R_TRAVEL)
    wr3 = WallHeadingCorrector(side=W_R)
    wr3.feed(40.0, 0.0)
    big = wr3.feed(40.0 + math.tan(math.radians(WALL_HEADING_MAX_TILT_DEG + 15))
                   * 50.0, 0.5)
    check("ميل مفرط ⇒ لا تصحيح (انعطاف جدار أو عائق لا ميل روبوت)",
          big["reason"] == R_TILT, f"{big.get('tilt_deg')}°")
    wr4 = WallHeadingCorrector(side=W_R)
    wr4.feed(40.0, 0.0)
    d1 = wr4.feed(46.0, 0.3)
    check("قراءة واحدة لا تكفي — الاتساق شرط",
          d1["reason"] == R_INCONSISTENT and not d1["apply"])
    # ميل متأرجح الإشارة = ضجيج لا ميل
    wr5 = WallHeadingCorrector(side=W_R)
    osc, seq = None, (40.0, 46.0, 40.0, 46.0, 40.0, 46.0)
    for i, v in enumerate(seq):
        osc = wr5.feed(v, i * 0.3)
    check("ميل متأرجح الإشارة ⇒ لا تصحيح (ضجيج لا انحراف)",
          not osc["apply"], f"سبب={osc['reason']}")
    # سقف التصحيح
    wr6 = WallHeadingCorrector(side=W_R)
    hard = None
    for i in range(WALL_HEADING_CONSISTENT_N + 1):
        hard = wr6.feed(30.0 + i * 3.9, i * 0.16)
    check("التصحيح مقصوص عند السقف (لا قفزات اتجاه)",
          not hard["apply"] or abs(hard["correction_deg"]) <= WALL_HEADING_MAX_CORR_DEG,
          f"{hard.get('correction_deg')}° ≤ {WALL_HEADING_MAX_CORR_DEG}°")
    check("وكل الرفوض معدودة بأسبابها (لا رفض صامت)",
          sum(wr.rejects.values()) + sum(wr3.rejects.values()) > 0,
          " · ".join(f"{k}={v}" for k, v in wr3.rejects.items()))

    # ── تتبّع المحيط: أربعة أضلاع وأبعاد مقاسة ───────────────────
    from pi.nav.perimeter import (PerimeterTracker, FOLLOW, TURN, DONE,
                                  ABORT, SEEK)
    from pi.config import (PERIMETER_TARGET_CM, PERIMETER_MAX_SIDE_M,
                           PERIMETER_OPENING_JUMP_CM)

    def run_perimeter(w=3.0, l=4.0, step=0.1, gap_at=None, no_wall_side=None):
        """غرفة محاكاة: يمشي الأضلاع بالترتيب ويرى الجدار على يمينه."""
        pt = PerimeterTracker(side="right")
        plan = [w, l, w, l]
        odom, cmds = 0.0, []
        for si, seg in enumerate(plan):
            walked = 0.0
            while walked < seg - 1e-9:
                walked += step
                odom += step
                front = (seg - walked) * 100.0
                side = PERIMETER_TARGET_CM
                if no_wall_side == si + 1:
                    side = None                    # ضلع بلا جدار
                elif gap_at and si + 1 == gap_at[0] and \
                        abs(walked - gap_at[1]) < step / 2:
                    side = PERIMETER_TARGET_CM + PERIMETER_OPENING_JUMP_CM + 20
                r = pt.step(side, front, odom)
                cmds.append(r["command"])
                if r["command"] in (DONE, ABORT):
                    return pt, cmds
                if r["command"] == TURN:
                    break              # الضلع انتهى — انتقل للتالي
        return pt, cmds

    pt_ok, cmds_ok = run_perimeter()
    check("دورة محيط في غرفة محاكاة → **أربعة أضلاع** ودورة مكتملة",
          pt_ok.finished and len(pt_ok.sides) == 4 and pt_ok.turns == 4,
          f"أضلاع={len(pt_ok.sides)} · لفّات={pt_ok.turns}")
    dims = pt_ok.measured_dimensions()
    check("والأبعاد المقاسة تطابق غرفة المحاكاة",
          dims and abs(dims[0] - 3.0) < 0.25 and abs(dims[1] - 4.0) < 0.25,
          f"مقاس {dims}")
    cmp_ok = pt_ok.compare_to(3.0, 4.0)
    check("🔴 الفحص الذاتي: المقاس مقابل المُدخل ⇒ الملاحة موثوقة",
          cmp_ok["ok"], cmp_ok["reason"][:64])
    cmp_bad = pt_ok.compare_to(6.0, 8.0)
    check("واختلاف كبير ⇒ **تحذير قبل بدء المسح** لا اكتشاف لاحق",
          not cmp_bad["ok"] and "انزلاق" in cmp_bad["reason"],
          cmp_bad["reason"][:60])
    check("اكتمال الدورة **بعدّ اللفّات** لا بالموضع (أصدق مع موضع تقديري)",
          cmds_ok.count(TURN) == 3 and cmds_ok[-1] == DONE,
          f"لفّات={cmds_ok.count(TURN)} ثم {cmds_ok[-1]}")
    check("والاتباع تناسبي على الخطأ الجانبي",
          FOLLOW in cmds_ok and abs(
              PerimeterTracker(side="right").step(
                  PERIMETER_TARGET_CM + 20, 500.0, 1.0)["steer"]) > 0)

    pt_nw, _ = run_perimeter(no_wall_side=2)
    seg2 = next((s for s in pt_nw.sides if s["index"] == 2), None)
    check("ضلع بلا جدار ⇒ **يُكمل** ويُعلَّم «بلا مرجع» (لا فشل)",
          pt_nw.finished and seg2 is not None
          and seg2["has_reference"] is False
          and all(s["has_reference"] for s in pt_nw.sides if s["index"] != 2),
          f"الضلع 2 بلا مرجع · طوله {seg2['length_m']}م" if seg2 else "—")
    check("و«لا جدار» يُخرج أمر SEEK لا إجهاضاً",
          PerimeterTracker(side="right").step(None, 500.0, 0.5)["command"] == SEEK)

    pt_gap, _ = run_perimeter(gap_at=(1, 1.5))
    check("فتحة/باب ⇒ **تُسجَّل ولا تُدخَل** (المحيط أولاً)",
          len(pt_gap.openings) >= 1 and pt_gap.finished,
          f"{len(pt_gap.openings)} فتحة عند "
          f"{pt_gap.openings[0]['at_odom_m']}م")

    pt_lost = PerimeterTracker(side="right")
    r_lost = None
    for i in range(1, 200):
        r_lost = pt_lost.step(PERIMETER_TARGET_CM, 900.0, i * 0.2)
        if r_lost["command"] == ABORT:
            break
    check("ضلع أطول من الحدّ ⇒ **إجهاض بسبب معلَن** (فقدان جدار)",
          r_lost["command"] == ABORT and "الجدار" in r_lost["reason"],
          r_lost["reason"][:58])
    check("الجهة ثابتة طوال الدورة (قاعدة تمنع الالتباس)",
          all(e for e in [pt_ok.side == "right"]))

    # ── تحكيم IR / الألترا سونيك الجانبي: تكامل لا تكرار ─────────
    rs_side = ReactiveSafety(enabled=False)
    conf = rs_side.side_arbitration(ir_side=0, us_side_cm=95.0, side="right")
    check("🔴 تعارض IR/ألترا سونيك ⇒ **IR له الأولوية** والتعارض مسجَّل",
          conf["blocked"] and conf["conflict"] and conf["priority"] == "ir"
          and "مائل" in conf["reason"], conf["reason"][:66])
    agree = rs_side.side_arbitration(ir_side=1, us_side_cm=95.0, side="right")
    check("لا تعارض حين يتفقان (IR خالٍ والألترا سونيك بعيد)",
          not agree["blocked"] and not agree["conflict"])
    near = rs_side.side_arbitration(ir_side=1, us_side_cm=12.0, side="left")
    check("وقرب الألترا سونيك بلا IR **ليس تعارضاً** (مدى IR أقصر أصلاً)",
          not near["conflict"] and near["priority"] == "ultrasonic")
    unk = rs_side.side_arbitration(ir_side=None, us_side_cm=None, side="left")
    check("والغائب **مجهول لا خالٍ** في الطرفين",
          not unk["blocked"] and set(unk["unknown"]) == {"ir", "ultrasonic"}
          and unk["priority"] == "unknown")

    # ═══ (ل) 🔴 البند 0: التحقق من الحركة ═════════════════════════
    print("\nل) التحقق من الحركة (البند 0):")
    from pi.nav.motion_check import (
        AccelWitness, verify_motion, tolerance_for,
        VERIFIED, SHORT, OVERSHOOT, NO_MOTION, UNVERIFIED,
        reference_ok as _mc_ref_ok,
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
    # 🔴 مقاس 2026-08-06: جولة سير حقيقية (شوهدت بالعين) أُجهضت «لا حركة»
    #    كاذبةً — صدى الأمامي الغالب من جسم مائل عن محور السير فلا تتغيّر
    #    مسافته مع التقدّم. «لا حركة» قاتلة فلا تُعلَن إلا **باتفاق الشاهدين**.
    v_conf = verify_motion(0.33, 120.0, 119.0, moving)
    check("أمامي ثابت والتسارع يرى حركة → تعارض = ثقة منخفضة لا إجهاض",
          v_conf["verdict"] == UNVERIFIED and v_conf["method"] == "conflict"
          and not v_conf["confident"], v_conf["reason"][:60])
    v_agree = verify_motion(0.33, 120.0, 119.0, still)
    check("أمامي ثابت والتسارع ساكن → «لا حركة» تبقى قاطعة (روفر مطفأ)",
          v_agree["verdict"] == NO_MOTION and v_agree["confident"])
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
    # ⚠ الفرق يُقاس على **حدّ المسافة** وحده: `advance` صار يضيف أيضاً مساهمة
    #   خطأ الاتجاه (المسافة × الزاوية) وهي مشتركة بين الحالتين، فالنسبة
    #   الخام لم تعد تساوي المعامل بينما الخاصية المقصودة قائمة.
    from pi.config import DRIFT_PER_METER as _DPM
    check("شوط غير متحقَّق ⇒ σ ينمو أسرع (لا يمنح المفترض وزن المقيس)",
          dr_u.uncertainty > dr_v.uncertainty
          # ⚠ التسامح 1e-4 لا 1e-9: نموّ شكّ الاتجاه يُحسب من **ساعة الحائط**،
          #   فاختلاف ميكروثوانٍ بين الكائنين يُنتج فرقاً حقيقياً دون 0.1مم.
          and abs((dr_u.uncertainty - dr_v.uncertainty)
                  - _DPM * (MOTION_UNVERIFIED_DRIFT - 1.0)) < 1e-4,
          f"{dr_v.uncertainty:.3f} مقابل {dr_u.uncertainty:.3f} م")

    # ⑤ب 🔴 انحراف الاتجاه بعد MPU-6050: مكوّن **زمني** واقتران ضربي
    from pi.config import (HEADING_DRIFT_PER_S_DEG, HEADING_SIGMA_PER_TURN_DEG,
                           HEADING_SIGMA_INITIAL_DEG, WALL_ALIGN_TOL_DEG)
    dr_t = DeadReckoning(room, profile, 0.25, 0.25, 0.0)
    dr_t._sigma_ts -= 100.0                      # محاكاة 100ث سكون
    u_before = dr_t.uncertainty
    dr_t.sync_time()
    check("🔴 شكّ الاتجاه ينمو بالزمن **والروبوت واقف** (الجايرو ينحرف بلا حركة)",
          dr_t.heading_sigma_deg > HEADING_SIGMA_INITIAL_DEG
          and abs(dr_t.heading_sigma_deg - HEADING_SIGMA_INITIAL_DEG
                  - 100.0 * HEADING_DRIFT_PER_S_DEG) < 1e-6,
          f"{HEADING_SIGMA_INITIAL_DEG}° → {dr_t.heading_sigma_deg:.2f}° بعد 100ث")
    check("والسكون وحده **لا يزيح** الموضع (الاقتران ضربي لا جمعي)",
          dr_t.uncertainty == u_before,
          "σ الموضع لم تتغيّر بالسكون")
    dr_t.advance(1.0)
    check("وعند أول حركة تظهر مساهمته: المسافة × الزاوية",
          dr_t.uncertainty > u_before + _DPM,
          f"σ الموضع {u_before:.3f} → {dr_t.uncertainty:.3f} م بعد متر واحد")
    dr_q = DeadReckoning(room, profile, 0.25, 0.25, 0.0)
    dr_q.turn(90.0)
    check("واللفّ يزيد شكّ الاتجاه أيضاً",
          dr_q.heading_sigma_deg >= HEADING_SIGMA_INITIAL_DEG
          + HEADING_SIGMA_PER_TURN_DEG - 1e-6,
          f"{dr_q.heading_sigma_deg:.2f}° بعد لفّة 90°")

    # 🔴 الحلقة المفرغة: تصحيح الجدار **لا يصحّح الاتجاه**، وبوابته تنهار
    dr_g = DeadReckoning(room, profile, 1.5, 3.5, 0.0)
    dr_g.uncertainty = 0.6
    sig_before = dr_g.heading_sigma_deg
    rc = dr_g.try_front_wall_correction(0.55)
    check("تصحيح الجدار يصغّر شكّ **الموضع** ولا يمسّ شكّ **الاتجاه**",
          rc["corrected"] and dr_g.uncertainty < 0.6
          and dr_g.heading_sigma_deg == sig_before,
          f"الموضع ↓ · الاتجاه ثابت عند {sig_before:.2f}°")
    dr_g.heading_sigma_deg = WALL_ALIGN_TOL_DEG + 1.0
    check("🔴 تجاوز شكّ الاتجاه تسامح المحاذاة ⇒ **إعلان انهيار البوابة**",
          dr_g.alignment_gate_lost and not DeadReckoning(
              room, profile, 0, 0, 0).alignment_gate_lost,
          f"{dr_g.heading_sigma_deg:.1f}° > {WALL_ALIGN_TOL_DEG:.0f}° "
          f"⇒ لا تصحيح ممكن ⇒ الشك يكبر أكثر")
    ms_gate = MissionSim()
    ms_gate.configure_room(1.0, 1.0)
    ms_gate.dr = DeadReckoning(ms_gate.room, newp, 0.25, 0.25, 0.0)
    ms_gate.dr.heading_sigma_deg = WALL_ALIGN_TOL_DEG + 5.0
    ms_gate._check_alignment_gate()
    ms_gate._check_alignment_gate()              # لا يتكرر
    gate_ev = [e for e in ms_gate.events if e["kind"] == "alignment_gate"]
    check("والمهمة تُعلنه **مرة واحدة** لا في كل خلية",
          len(gate_ev) == 1, gate_ev[0]["msg"][:70])

    # ⑤ج توسيع **تسامح القبول** لا نصف قطر الخطة (البند ب٣)
    from pi.config import (CONFIRM_TOL_SIGMA_K, CONFIRM_REGION_TOL_M,
                           CONFIRM_RADIUS_M)
    from pi.ai.source_locator import SourceLocator
    loc_tol = SourceLocator(3.0, 4.0, background_cpm=20.0)
    check("بلا قراءات: التسامح هو الثابت الأساسي",
          abs(loc_tol.region_tolerance() - CONFIRM_REGION_TOL_M) < 1e-9)
    loc_tol.add_reading(1.0, 1.0, 0.0, 5.0, 0.0, 3.0, pos_uncertainty_m=0.8)
    check("σ كبيرة ⇒ **التسامح يتّسع** (ولا يُمسّ نصف قطر الخطة)",
          abs(loc_tol.region_tolerance() - CONFIRM_TOL_SIGMA_K * 0.8) < 1e-9
          and len(loc_tol.confirmation_plan((1.5, 2.0), CONFIRM_RADIUS_M)) > 0,
          f"σ=0.8م → تسامح {loc_tol.region_tolerance():.2f}م "
          f"(الخطة تبقى على {CONFIRM_RADIUS_M}م)")

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

    # 🔴 المرجع يُقرأ **خاماً**: عتبة ثقة الأمامي (30سم) تخصّ المسافة
    #    المطلقة لقرارات العوائق، والتحقق يستعمل **فرق قراءتين** فينجو من
    #    أي انحياز ثابت — وحدّ صلاحيته الخاص MOTION_REF_MAX_CM (250سم).
    #    عطل مقاس 2026-08-11 تكرّر في **كل** جولة: في غرفة 2×1م تتجاوز
    #    المسافة 30سم دائماً ⇒ المقصوصة None ⇒ «7 من 8 خلايا بثقة منخفضة
    #    (حركة غير متحقَّقة)» ⇒ الخريطة كلها مفتوحة الحلقة، وأثره يمتدّ إلى
    #    محدِّد المصدر الذي يبني تقديره على مواضع القراءات (§2.2).
    class CappedSensors(MovingSensors):
        """يحاكي العتاد الحقيقي: المقصوصة None فوق 30سم والخام موجودة."""
        def __call__(self):
            d = super().__call__()
            raw = d["ultrasonic_cm"]
            d["ultrasonic_cm"] = raw if raw <= 30.0 else None
            d["ultrasonic_raw_cm"] = raw
            return d

    ex_cap = DriveExecutor(MotionRover(), ReactiveSafety(enabled=False),
                           CappedSensors(start_cm=150.0), newp, imu=ShakingIMU())
    m_cap = ex_cap.forward_cell(0.2).get("motion") or {}
    check("🔴 مرجع الحركة يُقرأ خاماً فوق عتبة ثقة العوائق (لا يضيع المرجع)",
          m_cap.get("d_start_cm") is not None
          and m_cap.get("d_end_cm") is not None
          and m_cap.get("measured_m") is not None,
          f"{m_cap.get('d_start_cm')}→{m_cap.get('d_end_cm')}سم ⇒ "
          f"{m_cap.get('verdict')}")
    check("ولا يُعتدّ بقراءة فوق حدّ صلاحية التحقق نفسه (250سم)",
          not _mc_ref_ok(MOTION_REF_MAX_CM + 1.0)
          and _mc_ref_ok(MOTION_REF_MAX_CM - 1.0),
          f"حدّان مختلفان لغرضين: 30سم للعوائق · {MOTION_REF_MAX_CM:.0f}سم للتحقق")

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

    # ⑦ج 🔴 الفجوة الوظيفية: البحث عن مصدر **ثانٍ** بعد أي حكم
    from pi.ai.two_stage import CONFIRMED as _CONF, SCREENING as _SCR
    ms_two = full_mission()
    loc2 = ms_two.locator
    check("بعد الدورة أُغلق الحكم وعاد الكاشف إلى **الفرز** (لا يعلق للأبد)",
          loc2.detector.state == _SCR and len(loc2.findings) == 1
          and loc2.findings[0]["verdict"] == _CONF,
          f"أحكام مغلقة={len(loc2.findings)} · الحالة={loc2.detector.state}")
    sub = (ms_two.cycle.get("closed") or {}).get("subtracted") or {}
    check("🔴 ومساهمة المؤكَّد **طُرحت** — بلاها يُعاد اكتشاف نفسه بلا نهاية",
          sub.get("ok") and sub.get("removed_counts", 0) > 0,
          f"طُرح {sub.get('removed_counts')} عدّة · بقي "
          f"{sub.get('remaining_counts')}")
    rs2 = (ms_two.cycle.get("closed") or {}).get("rescreen") or {}
    check("وإعادة الفرز على البواقي **لا تُعيد نفس المصدر**",
          rs2.get("suspect") is not True
          or (rs2.get("position") != loc2.findings[0]["position"]),
          f"اشتباه جديد={rs2.get('suspect')} · Λ={rs2.get('lambda_stat')}")
    check("والأحكام المغلقة تظهر في التقرير (لا تُمحى بإعادة الفرز)",
          loc2.report()["n_findings"] == 1)
    # الرفض لا يُطرح منه شيء — الاشتباه كان ضوضاء لا مساهمة
    loc3 = SourceLocator(3.0, 4.0, background_cpm=20.0)
    loc3.detector.state = "rejected"
    n_before = loc3.detector.screen_grid.n_measurements
    r3c = loc3.close_finding()
    check("حكم **الرفض** يُغلق بلا طرح (الضوضاء لا مساهمة لها تُخصم)",
          r3c["ok"] and r3c["subtracted"] is None
          and loc3.detector.state == _SCR
          and loc3.detector.screen_grid.n_measurements == n_before)
    check("ولا يُغلق حكم غير موجود", loc3.close_finding()["ok"] is False)

    # ⑦د 🔴 اختبار تكامل: المهمة **تستدعي** تصحيح الاتجاه الجانبي فعلاً
    #      (قاعدة CLAUDE.md §8 — لا وحدة مبنيّة معزولة)
    class FakeArray:
        """
        مصفوفة وهمية: جدار يمين يبتعد تدريجياً ⇒ ميل ثابت.
        ⚠ الخطوة 2.5سم/استدعاء لا 7: المهمة تستدعي القناة مرتين لكل خلية
          (تصحيح + تحكيم جانبي) ⇒ Δ=5سم/0.5م = ميل ~5.7° — واقعي وتحت سقف
          WALL_HEADING_MAX_TILT_DEG المشدود (12° منذ 2026-08-07؛ الخطوة
          القديمة 7 أنتجت 15.6° فرُفضت كلها R_TILT وانكسر الاختبار).
        """
        ok = True
        def __init__(self):
            self.n = 0
        def perpendicular_cm(self, side):
            if side != "right":
                return None
            self.n += 1
            return 40.0 + self.n * 2.5
        def state(self):
            return {"enabled": True, "ok": True, "channels": {}}

    ms_wh = full_mission(run=False)
    ms_wh.set_side_ultrasonic(FakeArray())
    ms_wh.dr.heading_sigma_deg = WALL_ALIGN_TOL_DEG + 8.0   # بوابة منهارة
    check("حقن المصفوفة يُنشئ مصحّح الاتجاه", ms_wh.wall_heading is not None)
    for _ in range(6):
        t = ms_wh._next_target()
        if t is None:
            break
        ms_wh._advance_one_cell(t)
    check("🔴 المهمة **تستدعي** تصحيح الاتجاه بعد كل خلية (لا وحدة معزولة)",
          ms_wh._wall_heading_corrections > 0
          and any(e["kind"] == "wall_heading" for e in ms_wh.events),
          next((e["msg"][:64] for e in ms_wh.events
                if e["kind"] == "wall_heading"), "—"))
    check("🔴 وσ الاتجاه صغرت فعلياً ⇒ **بوابة المحاذاة أُعيد فتحها**",
          not ms_wh.dr.alignment_gate_lost,
          f"σ = {ms_wh.dr.heading_sigma_deg:.2f}° "
          f"(كانت {WALL_ALIGN_TOL_DEG + 8.0:.0f}°)")
    check("وحالة الجانبي تُبثّ في الواجهة",
          ms_wh.state_dict()["side_us"]["corrections"] > 0)
    # وبلا حقن: صفر تغيير في السلوك القائم
    ms_off = full_mission(run=False)
    for _ in range(3):
        t = ms_off._next_target()
        if t is None:
            break
        ms_off._advance_one_cell(t)
    check("وبلا **حقن** مصفوفة لا يتغيّر أي سلوك قائم (العلم وحده لا يكفي)",
          ms_off.wall_heading is None
          and ms_off._wall_heading_corrections == 0
          and ms_off.state_dict()["side_us"]["array"] is None)

    # ⑦هـ دورة المحيط موصولة بالمهمة وترفض بسبب معلَن عند النقص
    ms_pm = full_mission(run=False)
    r_pm = ms_pm.run_perimeter_cycle()
    check("دورة المحيط تُرفض **بسبب معلَن** بلا ألترا سونيك جانبي",
          r_pm["ok"] is False and "جانبي" in r_pm["reason"], r_pm["reason"][:60])

    class WallArray(FakeArray):
        """جدار يمين ثابت على 40سم — لدورة محيط قابلة للتنفيذ."""
        def perpendicular_cm(self, side):
            return 40.0 if side == "right" else None

    ms_pm2 = full_mission(run=False)
    ms_pm2.set_side_ultrasonic(WallArray())
    r_pm2 = ms_pm2.run_perimeter_cycle(max_steps=60)
    check("🔴 ومع المصفوفة **تُنفَّذ فعلاً** وتُخرج حصيلة (لا وحدة معزولة)",
          ms_pm2.perimeter is not None
          and isinstance(r_pm2.get("summary"), dict)
          and any(e["kind"] == "perimeter" for e in ms_pm2.events),
          f"أضلاع={len(r_pm2['summary']['sides'])} · "
          f"لفّات={r_pm2['summary']['turns']}")
    check("والفحص الذاتي يقارن المقاس بالمُدخل ويُسجَّل",
          ("self_check" in r_pm2) and any(
              e["kind"] == "perimeter_check" for e in ms_pm2.events),
          (r_pm2.get("self_check") or {}).get("reason", "—")[:58])

    # ⑦و 🔴 البندان الأخيران في التدقيق: مبنيّان وصارا **مستدعَيين**
    from pi.ai.dynamic_range import recheck_needed as _rn
    hot_r = _rn(counts=60, duration_s=3.0, background_cpm=18.0)
    cold_r = _rn(counts=1, duration_s=3.0, background_cpm=18.0)
    check("`recheck_needed` يميّز الارتفاع المشبوه عن الخلفية",
          hot_r["recheck"] and not cold_r["recheck"],
          hot_r["reason"][:52])
    # وفي المهمة: قراءة ساخنة ⇒ إعادة قياس أطول فعلياً
    ms_rc = full_mission(run=False, src=(0.3, 0.3), a_cpm=4000.0)
    n0 = len(ms_rc.locator.readings)
    ms_rc._visit(ms_rc.current)
    check("🔴 والمهمة **تستدعيه**: ارتفاع مشبوه ⇒ إعادة قياس أطول",
          any(e["kind"] == "recheck" for e in ms_rc.events)
          and len(ms_rc.locator.readings) > n0 + 1,
          next((e["msg"][:60] for e in ms_rc.events
                if e["kind"] == "recheck"), "—"))
    ms_cold = full_mission(run=False, src=None, a_cpm=1.0)
    n1 = len(ms_cold.locator.readings)
    ms_cold._visit(ms_cold.current)
    check("وقراءة عند الخلفية **لا** تُعاد (لا إهدار زمن المهمة)",
          len(ms_cold.locator.readings) == n1 + 1
          and not any(e["kind"] == "recheck" for e in ms_cold.events))
    # set_vision_stats: العدّاد يصل التقرير
    ms_vs = full_mission(run=False)
    st_vs = ms_vs._push_vision_stats()
    check("🔴 عدّاد الرؤية يصل التقرير (كان `vision_stats` فارغاً دائماً)",
          bool(st_vs) and ms_vs.locator.report()["vision_stats"].get("provider"),
          f"مزوّد={st_vs.get('provider')} · استدعاءات={st_vs.get('calls')}")

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

    # ═══ (ل) عطل مصدر الاتجاه: إحياء، لا محركات، ولا حلقة لا نهائية ══
    # عطل مقاس على العتاد (2026-08-01): مصدر الاتجاه أعلن الموت، فصار كل لفّ
    # يفشل في صفر ثانية وتعيده حلقة المحركات فوراً — عشرات المحاولات في عُشر
    # ثانية، كلٌّ تنبض المحركات لحظة (ارتجاف في المكان) وتكتب سطراً مكرّراً.
    print("\nل) عطل مصدر الاتجاه — إحياء وحواجز:")

    class ResettableIMU(FakeIMU):
        """
        يحاكي BNO055 عاد إلى وضع CONFIG: حاضر على الناقل ويردّ **صفراً
        مضبوطاً** حتى تُعاد تهيئته، وبعدها يقرأ طبيعياً.
        """
        def __init__(self, recoverable=True):
            super().__init__(rate=0.0)
            self.recoverable = recoverable
            self.reinits = 0

        def recover(self):
            if not self.recoverable:
                return {"recovered": False,
                        "detail": "CHIP_ID=0xff — الشريحة مفقودة عن الناقل"}
            self.reinits += 1
            self._r = 30.0                 # عادت الحياة بعد إعادة التهيئة
            return {"recovered": True, "detail": "عادت إلى CONFIG وأُعيدت تهيئتها"}

    live = ResettableIMU(recoverable=True)
    hs_rec = BNO055GyroHeading(live, scale=1.0, sign=+1)
    for _ in range(ZERO_RUN_DEAD + 2):
        hs_rec.update()
    check("صفر مضبوط + شريحة قابلة للإحياء → تُحيا ولا يُعلَن العطل",
          hs_rec.ok and hs_rec.recoveries == 1 and live.reinits == 1,
          f"إحياءات={hs_rec.recoveries} · ok={hs_rec.ok}")

    gone = ResettableIMU(recoverable=False)
    hs_dead = BNO055GyroHeading(gone, scale=1.0, sign=+1)
    for _ in range(ZERO_RUN_DEAD * 8):
        hs_dead.update()
    check("شريحة غير قابلة للإحياء → عطل معلن **بحالتها المقروءة**",
          not hs_dead.ok and "CHIP_ID" in (hs_dead.error or ""), hs_dead.error)
    check("محاولات الإحياء محدودة (لا إغراق للناقل)",
          hs_dead.recovery_attempts <= HEADING_RECOVERY_ATTEMPTS,
          f"{hs_dead.recovery_attempts} ≤ {HEADING_RECOVERY_ATTEMPTS}")

    # الحاجز الحاسم: **لا أمر حركة** ومصدر الاتجاه معطّل
    class SpyBridge(WaveRoverBridge):
        """
        يعدّ أوامر المحركات **المُشغِّلة فقط** (قوة ≠ 0). أوامر الإيقاف
        (0,0) لا تُعدّ: هي عكس ما نحرسه — الإيقاف المضمون مطلوب دائماً.
        """
        def motors(self, l, r):
            if l or r:
                self.motor_calls = getattr(self, "motor_calls", 0) + 1
            return super().motors(l, r)

    spy = SpyBridge(mode="sim", heading_source=hs_dead)
    spy.motor_calls = 0
    t0 = time.time()
    rdead = spy.turn_by_angle(90, timeout=5.0)
    check("لفّ ومصدر الاتجاه معطّل → إجهاض بلا أي أمر حركة",
          rdead["aborted"] == "heading_source_fault" and spy.motor_calls == 0,
          f"أوامر محركات={spy.motor_calls} · إجهاض={rdead['aborted']}")
    check("لا معايرة انحياز 5ث على حسّاس معطّل (الإجهاض فوري)",
          (time.time() - t0) < 1.0, f"{time.time() - t0:.2f}ث")

    # الأحداث المتطابقة تُضغط بعدّاد بدل أن تدفن ما قبلها
    for _ in range(5):
        spy._event("heading_fault", "نفس الرسالة")
    faults = [e for e in spy.events if e["kind"] == "heading_fault"
              and e.get("base") == "نفس الرسالة"]
    check("الأحداث المتطابقة تُضغط بعدّاد لا تُكدَّس",
          len(faults) == 1 and faults[0]["n"] == 5, faults[0]["msg"])

    # خيط المحركات: عطل الاتجاه يوقف المهمة بدل الدوران على الفراغ
    ms3 = MissionSim()
    ms3.rover = SpyBridge(mode="sim", heading_source=hs_dead)
    ms3.rover.motor_calls = 0
    ms3.configure_room(2.0, 2.0)
    ms3.set_calibration(default_sim_profile())
    ms3.set_drive_motors(True, allow_sim=True)
    # ⚠ حارسان في **لحظتين مختلفتين**، وكلاهما لازم:
    #   (أ) بوابة الجاهزية ترفض **البدء** بمصدر اتجاه ميت — تسبق الخيط أصلاً.
    #   (ب) وحارس الخيط يُجهض حين يموت المصدر **وسط المهمة** (الجاهزية مرّت).
    res3 = ms3.start()
    check("(أ) مصدر اتجاه ميت ⇒ **رفض البدء** بسبب مقروء قبل تشغيل الخيط",
          res3["ok"] is False and ms3._worker is None
          and any("اتجاه" in b for b in res3["readiness"]["blockers"]),
          res3["error"][:70])
    # (ب) نتجاوز البوابة عمداً لنختبر الحارس داخل الخيط (موت وسط المهمة)
    from pi.nav.executor import DriveExecutor as _DE
    ms3.dr = DeadReckoning(ms3.room, ms3.profile,
                           *ms3.grid.cell_center(*ms3.current), 0.0)
    ms3.executor = _DE(ms3.rover, ms3.reactive, ms3.sensors, ms3.profile)
    ms3.state = "running"
    ms3._visit(ms3.current)          # كما يفعل start: خلية البدء أولاً
    r3 = ms3._advance_one_cell(ms3._next_target())
    aborts = [e for e in ms3.events if e["kind"] == "mission_abort"]
    check("(ب) وموته وسط المهمة **قاتل للخطوة** لا خطوة تُعاد",
          r3.get("fatal") is True and ms3.state == "estop" and aborts,
          aborts[0]["msg"][:60] if aborts else f"الحالة={ms3.state}")
    check("لم تُشغَّل المحركات ولا مرة أثناء الإجهاض",
          ms3.rover.motor_calls == 0, f"{ms3.rover.motor_calls} أمر")
    turn_fails = [e for e in ms3.events if e["kind"] == "turn_failed"]
    check("سطر فشل واحد لا عشرات (السجل يحفظ السبب الأول)",
          len(turn_fails) <= 1, f"{len(turn_fails)} سطر")

    # ═══ (م) دقّة نهاية اللفّة: تهدئة + قياس القصور + تصحيح ═════════
    # مقاس على العتاد (2026-08-01): `rate_dps = 137.4` **لحظة قطع الطاقة**،
    # فيواصل الروبوت الدوران بالقصور الذاتي بينما حلقة اللفّ انتهت — التجاوز
    # لا يُقاس أصلاً فيظنّ النظام أنه على 90° وهو على ~100°.
    print("\nم) دقّة نهاية اللفّة (التجاوز والقصور الذاتي):")
    import math as _math
    from pi.config import (
        TURN_SLOWDOWN_DEG, TURN_MIN_POWER, TURN_TOLERANCE_DEG,
        TURN_CORRECTION_PASSES, TURN_POWER,
    )

    # منحنى القوة: منطق خالص، يُختبر مباشرةً
    _tp = WaveRoverBridge._turn_power
    check("قوة كاملة بعيداً عن الهدف",
          _tp(TURN_SLOWDOWN_DEG + 10, 45.0, TURN_POWER) == TURN_POWER)
    check("القوة تنزل إلى الأدنى عند الهدف",
          abs(_tp(0.0, 45.0, TURN_POWER) - TURN_MIN_POWER) < 1e-9,
          f"{_tp(0.0, 45.0, TURN_POWER)}")
    check("التهدئة تنازلية لا قفزة",
          _tp(30, 45.0, TURN_POWER) > _tp(10, 45.0, TURN_POWER)
          > _tp(2, 45.0, TURN_POWER))
    # ⚠ الحارس الحاسم: لفّة تصحيح صغيرة تبدأ **داخل** قوس التهدئة، فلو خُفّضت
    #   قوّتها قبل أن يدور الروبوت لما كسرت السكون ولما تحرّك إطلاقاً.
    check("قبل بدء الدوران القوة كاملة (كسر السكون في لفّة التصحيح)",
          _tp(5.0, 0.0, TURN_POWER) == TURN_POWER)

    class InertialBridge(WaveRoverBridge):
        """
        جسر محاكاة **بقصور ذاتي**: الدوران لا ينقطع مع الطاقة بل يخبو أسّياً.
        بدونه تبقى المحاكاة أنظف من العتاد في النقطة التي انكسر فيها بالضبط.
        """
        TAU = 0.10

        def stop(self):
            rate = self._sim_turn_rate
            super().stop()
            if rate:
                self._coast_rate, self._coast_ts = rate, time.time()

        def read_imu(self):
            d = super().read_imu()
            if not self._moving and getattr(self, "_coast_rate", 0.0):
                r = self._coast_rate * _math.exp(
                    -(time.time() - self._coast_ts) / self.TAU)
                if abs(r) < 1.0:
                    r = self._coast_rate = 0.0
                d["gz"] = round(r + self._sim_bias, 3)
            return d

    inr = InertialBridge(mode="sim")
    inr.calibrate_gyro_bias(seconds=0.3)
    check("لا استباق مفترض قبل أي قياس (τ يبدأ صفراً)", inr._coast_tau == 0.0)

    first = inr.turn_by_angle(90, timeout=8.0)
    check("القصور الذاتي بعد قطع الطاقة **يُقاس** لا يُهمَل",
          abs(first["coast_deg"]) > 0.5, f"قصور={first['coast_deg']}°")
    check("اللفّة الأولى: التجاوز رقم معلوم لا 90.0 مضبوطة تخفيه",
          abs(first["overshoot_deg"]) > 0.0,
          f"تجاوز={first['overshoot_deg']}° · دار {first['turned_deg']}°")
    check("τ يُشتقّ من القياس لا من config",
          inr._coast_tau > 0.0, f"τ={inr._coast_tau:.3f}ث")

    # ⚠ جوهر الإصلاح: اللفّات التالية تستفيد مما قيس في الأولى
    for _ in range(3):
        last = inr.turn_by_angle(90, timeout=8.0)
    check("بعد التعلّم: اللفّة تصيب الهدف ضمن التسامح",
          abs(last["residual_deg"]) <= TURN_TOLERANCE_DEG,
          f"متبقٍّ={last['residual_deg']}° مقابل {first['residual_deg']}° أولاً")
    check("الاستباق ألغى الحاجة إلى دورات التصحيح",
          last["corrections"] == 0, f"{last['corrections']} تصحيح")
    check("التصحيح محدود العدد دائماً (لا مطاردة بلا نهاية)",
          first["corrections"] <= TURN_CORRECTION_PASSES
          and last["corrections"] <= TURN_CORRECTION_PASSES)

    # منصّة بلا قصور ذاتي: الاستباق يبقى صفراً فلا تقصُر اللفّة أبداً
    clean = WaveRoverBridge(mode="sim")
    clean.calibrate_gyro_bias(seconds=0.3)
    rcl = clean.turn_by_angle(90, timeout=8.0)
    check("بلا قصور ذاتي: إصابة ضمن التسامح بلا تصحيح ولا نقص",
          rcl["corrections"] == 0
          and abs(rcl["residual_deg"]) <= TURN_TOLERANCE_DEG,
          f"متبقٍّ={rcl['residual_deg']}° · تصحيحات={rcl['corrections']}")
    check("τ يبقى ~صفر على منصّة تقف فوراً (لا استباق كاذب)",
          clean._coast_tau < 0.02, f"τ={clean._coast_tau:.4f}ث")

    # ── الخطأ الصغير: يُغلق أثناء السير لا بلفّة بالمكان ───────────
    # ⚠ العطل الذي أنتجه قياسُ التجاوز نفسه: صار النظام يرى خطأه (+4.8°)
    #    فيطلب لفّة −5°، وقصورها 6.2° فينتهي عند −3.4° فيطلب +3°… رجفة عند
    #    كل خلية بلا تقارب. الأداة كانت خطأ لا القياس.
    from pi.nav.executor import DriveExecutor
    from pi.config import TURN_MIN_ACHIEVABLE_DEG

    ex_br = WaveRoverBridge(mode="sim")
    ex_br.calibrate_gyro_bias(seconds=0.3)
    open_road = lambda: {"ultrasonic_cm": 300.0, "ir_left": 1, "ir_right": 1}  # noqa: E731
    ex = DriveExecutor(ex_br, ReactiveSafety(), open_road, default_sim_profile())

    t_small = ex.turn_to(94.8, 90.0)
    check("خطأ أصغر من أصغر لفّة ممكنة → لا لفّة بالمكان إطلاقاً",
          t_small.get("skipped") and t_small["turned_deg"] == 0.0,
          f"متبقٍّ={t_small.get('residual_deg'):.1f}° < {TURN_MIN_ACHIEVABLE_DEG}°")
    check("الخطأ المتبقّي يُمرَّر لا يُبتلع (وإلا ثبّتنا الاتجاه الخاطئ)",
          abs(t_small["residual_deg"] - (-4.8)) < 1e-6,
          f"{t_small['residual_deg']}°")
    check("لفّة 90° تبقى لفّة بالمكان كالمعتاد",
          not ex.turn_to(0.0, 90.0).get("skipped"))

    fw = ex.forward_cell(0.5, heading_error_deg=6.0)
    hh = fw.get("heading_hold") or {}
    check("الشوط يثبّت **الهدف** لا الاتجاه الحالي",
          hh.get("target_offset_deg") == 6.0 and hh["max_abs_error_deg"] >= 5.0,
          f"إزاحة={hh.get('target_offset_deg')}° أقصى خطأ={hh['max_abs_error_deg']}°")
    # ⚠ الحارس الحاسم: شوط بخطأ حقيقي = طور **توجيه** لا سير. بعتبة السير
    #    (12°/ث) يرفض المرشّح دوران الروبوت الذي أمر به المتحكّم نفسه
    #    (41–62°/ث عند الإشباع) فيتجمّد التكامل ويبقى مشبعاً بلا انغلاق —
    #    قِيس قبل الإصلاح: 6.0° بقيت 5.1° بإشباع **100%** طوال الشوط.
    check("شوط بخطأ زاوي حقيقي يعمل في طور «التوجيه»",
          hh.get("phase") == "steer", f"الطور={hh.get('phase')}")
    check("تثبيت الاتجاه **يغلق** الخطأ فعلاً أثناء السير",
          abs(hh["final_error_deg"]) < 1.5 and hh["saturated_pct"] < 50.0,
          f"6.0° → {hh['final_error_deg']}° · إشباع {hh['saturated_pct']}%")
    check("الخطأ النهائي **مقاس** ويصل إلى المهمة",
          hh.get("final_error_deg") is not None)
    hh0 = (ex.forward_cell(0.5, heading_error_deg=0.0) or {})["heading_hold"]
    check("السير المستقيم يبقى في طوره (تنعيم أثقل — والعتبة موحّدة مع التوجيه)",
          hh0.get("phase") == "drive", f"الطور={hh0.get('phase')}")

    # ── «مهلة اللفّ» تُسمّى: هل تعثّر بعد 70° أم لم يدر أصلاً؟ ──────
    class StuckBridge(WaveRoverBridge):
        """روبوت تصل إليه الأوامر ولا يدور (بطارية منهكة / عجلة عالقة)."""
        def read_imu(self):
            d = super().read_imu()
            d["gz"] = self._sim_bias        # لا دوران مهما أُمر
            return d

    stuck = StuckBridge(mode="sim")
    stuck.calibrate_gyro_bias(seconds=0.3)
    rst = stuck.turn_by_angle(90, timeout=1.0)
    check("لفّ بلا دوران يُسمّى بوضوح لا «مهلة» مبهمة",
          rst["timed_out"]
          and any(e["kind"] == "turn_no_rotation" for e in stuck.events),
          next((e["msg"][:70] for e in stuck.events
                if e["kind"] == "turn_no_rotation"), "لا حدث"))

    # ── إعادة تعريف الغرفة توقف خيط المحركات الجاري ────────────────
    # ⚠ بدونه يواصل الخيط القديم قيادة الروبوت على شبكة استُبدلت تحته، ويكتب
    #   أحداثه في سجل صُفّر للتوّ — فتظهر وكأنها **سبقت** بدء المهمة الجديدة
    #   (شوهد على العتاد: turn_failed عند 1.9ث و mission_start عند 4.0ث).
    ms4 = MissionSim()
    ms4.configure_room(2.0, 2.0)
    ms4.set_calibration(default_sim_profile())
    ms4.set_drive_motors(True, allow_sim=True)
    ms4.start()
    ms4.configure_room(3.0, 2.0)              # إعادة تعريف والمهمة جارية
    check("إعادة تعريف الغرفة توقف المهمة والخيط الجاري",
          ms4.state == "idle"
          and (ms4._worker is None or not ms4._worker.is_alive())
          and not ms4.rover._moving,
          f"الحالة={ms4.state} · محركات={ms4.rover._moving}")

    # ═══ (ن) وصلة الروفر + الحماية البديلة عن حسّاس الجهد ══════════
    print("\nن) وصلة الروفر والحماية بلا حسّاس جهد:")
    from pi.config import TURN_RATE_FADE_WARN, TURN_NO_ROTATION_LIMIT

    # ⚠ الحارس الحاسم: منفذ ميت كان يجعل `finally: self.stop()` **نفسه** يرمي
    #    استثناءً — فتضيع «الإيقاف المضمون» في اللحظة الوحيدة التي وُجدت لها،
    #    ويطبع بايثون شلال استثناءات متداخلة يدفن السبب الأول (شوهد على العتاد).
    class DeadPort:
        closed = False
        def write(self, _b):  raise OSError(9, "Bad file descriptor")   # noqa: E704
        def close(self):      self.closed = True                        # noqa: E704

    dead = WaveRoverBridge(mode="sim")
    dead.mode, dead._ser = "real", DeadPort()      # منفذ يرفض كل كتابة
    try:
        dead.motors(0.4, -0.4)
        dead.stop()
        raised = False
    except Exception:                              # noqa: BLE001
        raised = True
    check("منفذ ميت لا يرمي استثناءً — الإيقاف المضمون يبقى مضموناً",
          not raised and not dead.link_ok, f"link_ok={dead.link_ok}")
    check("انقطاع الوصلة يُعلَن حدثاً مقروءاً مرة واحدة",
          sum(1 for e in dead.events if e["kind"] == "rover_link_fault") == 1,
          next((e["msg"][:60] for e in dead.events
                if e["kind"] == "rover_link_fault"), "لا حدث"))
    rlink = dead.turn_by_angle(90, timeout=1.0)
    check("اللفّ على وصلة ميتة يُجهض بسبب صريح لا بمهلة",
          rlink["aborted"] == "rover_link_fault", str(rlink["aborted"]))

    # ذروة معدل الدوران = مقياس شحن ضمني (لا فولتميتر على هذا العتاد)
    class FadingBridge(WaveRoverBridge):
        """بطارية تنهك: معدل الدوران المتاح ينزل لفّة بعد لفّة."""
        gain = 1.0
        def read_imu(self):
            d = super().read_imu()
            d["gz"] = round(d["gz"] * self.gain, 3)
            return d

    fade = FadingBridge(mode="sim")
    fade.calibrate_gyro_bias(seconds=0.3)
    fade.turn_by_angle(90, timeout=8.0)
    base_peak = fade.turn_peak_baseline
    fade.gain = 0.4                                # البطارية تنهك
    fade.turn_by_angle(90, timeout=8.0)
    check("ذروة الدوران تُقاس وتُتخذ مرجعاً (بديل حسّاس الجهد)",
          base_peak and base_peak > 0, f"مرجع={base_peak:.0f}°/ث")
    check("هبوط ذروة الدوران يُعلَن تحذير إنهاك بطارية",
          any(e["kind"] == "turn_fade" for e in fade.events),
          next((e["msg"][:64] for e in fade.events
                if e["kind"] == "turn_fade"), f"عتبة={TURN_RATE_FADE_WARN}"))

    # العجز الكامل عن اللفّ يُنهي المهمة (RTH يحتاج لفّاً أيضاً)
    ms5 = MissionSim()
    ms5.rover = StuckBridge(mode="sim")
    ms5.configure_room(2.0, 2.0)
    ms5.set_calibration(default_sim_profile())
    ms5.set_drive_motors(True, allow_sim=True)
    ms5.start()
    ms5._worker.join(timeout=30.0)
    check(f"{TURN_NO_ROTATION_LIMIT} لفّتان بلا دوران → إنهاء المهمة بسبب صريح",
          ms5.state == "estop"
          and any("اشحن البطارية" in e["msg"] for e in ms5.events),
          next((e["msg"][:70] for e in ms5.events
                if e["kind"] == "mission_abort"), f"الحالة={ms5.state}"))

    # ═══ (س) بوابة الجاهزية تفحص تماسك ثوابت الاتجاه ═══════════════
    # ⚠ `config_sanity` كانت تعمل في selftest وسكربت المعايرة فقط — فتعديل
    #   config على الراسبري مباشرةً كان يمرّ بلا حارس، وخطؤه يظهر على العتاد
    #   إشباعَ محرك عند الحاجز: سلوك غير خطي لا يُشخَّص من السلوك وحده.
    print("\nس) بوابة الجاهزية وثوابت الاتجاه:")
    import pi.nav.heading_hold as _hh

    ms_cfg = MissionSim()
    ms_cfg.configure_room(2.0, 2.0)
    ms_cfg.set_calibration(default_sim_profile())
    rd_base = ms_cfg.mission_readiness()
    check("ثوابت اتجاه متماسكة → لا حاجب إعدادات في الجاهزية",
          not any("ثوابت الاتجاه" in b for b in rd_base["blockers"]))
    _saved_corr = _hh.HEADING_MAX_CORR
    try:
        _hh.HEADING_MAX_CORR = 1.0        # إعداد مستحيل: يفوق الفراغ المتاح
        rd_bad = ms_cfg.mission_readiness()
        check("إعداد اتجاه غير متماسك **يحجب البدء** بسبب مقروء",
              any("ثوابت الاتجاه" in b for b in rd_bad["blockers"]),
              next((b[:70] for b in rd_bad["blockers"]
                    if "ثوابت الاتجاه" in b), "لا حاجب"))
    finally:
        _hh.HEADING_MAX_CORR = _saved_corr

    # ── ①ب: جسر حقيقي بمصدر اتجاه محاكى ⇒ البدء محجوب ──────────────
    # ⚠ مقاس 2026-08-08 على العتاد: سيرفر أقلع بلا RMS_ROVER_MODE=real ثم
    #   بُدّل الوضع من الواجهة — heading_source بقي «sim» على جسر real،
    #   والمصدر الوهمي ok=True فيمرّ من فحص الصلاحية العادي.
    ms_hd = MissionSim()
    ms_hd.configure_room(2.0, 2.0)
    ms_hd.set_calibration(default_sim_profile())
    ms_hd.rover.mode = "real"
    rd_sim = ms_hd.mission_readiness()
    check("🔴 جسر حقيقي بمصدر اتجاه محاكى → البدء محجوب بسبب مقروء",
          any("محاكى" in b for b in rd_sim["blockers"]),
          next((b[:64] for b in rd_sim["blockers"] if "محاكى" in b), "لا حاجب"))
    ms_hd.rover.mode = "sim"
    check("وعلى جسر sim يبقى المصدر المحاكى مشروعاً (لا حجب زائفاً)",
          not any("محاكى" in b for b in ms_hd.mission_readiness()["blockers"]))

    # ── 🔴 تذبذب التدرّج: تجاوز ذروة (وصول) ≠ انقلاب زمن ميت (خطر) ──
    # عطل مقاس 2026-08-10: مصدر تدريبي 1.4 µSv/h في غرفة 2×1م أعطى انعكاسين
    # والمعدّل 1,176–6,816 CPM ⇒ أُعلن «انقلاب» وانسحب الروبوت — بينما كل
    # القراءات **دون عتبة الموثوقية** فلا تشبّع أصلاً. السبب هندسي بحت:
    # خطوة 0.25م أكبر من بُعد المصدر فيقفز فوق القمة ذهاباً وإياباً.
    from pi.ai.approach_document import GradientApproach as _GA

    def _osc(peak_cpm, bg=18.0):
        """يفتعل انعكاسين بقراءات حول ذروة معطاة (عدّات 5ث)."""
        ga = _GA(background_cpm=bg, step_m=0.25)
        # صعود/هبوط مرتين: كل «هبوط» يُنتج انعكاساً، وانعكاسان يبلغان الحدّ
        seq = [peak_cpm * 0.2, peak_cpm, peak_cpm * 0.2,
               peak_cpm, peak_cpm * 0.2]
        out = None
        for i, c in enumerate(seq):
            out = ga.step(0.5, 1.0 + 0.25 * i, counts_L=c * 5.0 / 60.0,
                          duration_s=5.0)
        return ga, out

    ga_lo, out_lo = _osc(6800.0)       # دون حدّ الموثوقية 10,000
    check("🔴 تذبذب بقراءات موثوقة = **ذروة محاصَرة** ⇒ قف ووثّق لا تنسحب",
          out_lo["command"] == "stop" and ga_lo.reversals >= 2
          and "الذروة مُحاصَرة" in out_lo["reason"],
          f"{out_lo['command']} · {ga_lo.reversals} انعكاس")
    ga_hi, out_hi = _osc(120000.0)     # فوق الموثوقية بكثير = انقلاب حقيقي
    check("وتذبذب بقراءات غير موثوقة يبقى **انسحاباً** (انقلاب زمن ميت)",
          out_hi["command"] == "withdraw",
          f"{out_hi['command']} · {out_hi['reason'][:48]}")

    # ── 🔴 قراءة دون المدى الفيزيائي = مجهول لا «عائق» ──────────────
    # عطل مقاس 2026-08-10: الأمامي أعطى 0.8سم ثابتة والطريق خالٍ وIR الثلاثة
    # خضراء ⇒ «عائق أمامي 1سم» شلّ المهمة من الخلية الأولى وعلّم خمس خلايا
    # «غير قابلة للوصول» زوراً. وأدنى مدى HC-SR04 ≈2سم: ما دونه رنين المرسِل.
    from pi.sensors.ultrasonic import (_plausible as _plaus,
                                       MIN_PLAUSIBLE_CM as _MINP,
                                       MAX_PLAUSIBLE_CM as _MAXP)
    check("🔴 قراءة دون المدى الفيزيائي تُرفض (رنين المرسِل لا جسم)",
          not _plaus(0.8) and not _plaus(1.9) and not _plaus(0.0),
          f"الحدّ الأدنى {_MINP}سم")
    check("والقراءات داخل المدى تُقبل كما هي",
          _plaus(_MINP) and _plaus(30.0) and _plaus(_MAXP))
    check("وما فوق المدى يُرفض أيضاً (كما كان)", not _plaus(_MAXP + 1))

    # ── 🔴 «صفر مضبوط» على MPU: إيقاظ من السكون لا إعلان وفاة ────────
    # عطل مقاس 2026-08-10: المهمة أُجهضت بـ«40 قراءة صفر مضبوط — الحسّاس لا
    # يرسل شيئاً (ميت؟)» و`i2cdetect` يُظهر 0x68 حاضراً. المسار كان مفقوداً
    # كلياً لـMPU بينما يملكه مصدرا BNO055 **التالفة**.
    from pi.sensors.heading import MPU6050GyroHeading as _MPUH

    class _SleepyMPU:
        """MPU نائمة: ترد بهويتها وتعيد أصفاراً مضبوطة حتى تُوقَظ."""
        ok = True
        error = None
        def __init__(self): self.awake = False
        def gyro_z_dps(self): return 12.0 if self.awake else 0.0
        def state(self): return {"awake": self.awake}
        def recover(self):
            self.awake = True
            return {"recovered": True, "detail": "أُوقظت من **السكون**"}

    sleepy = _SleepyMPU()
    hs_slp = _MPUH(sleepy, scale=1.0, sign=1)
    for _ in range(60):
        hs_slp.update()
    check("🔴 صفر مضبوط على MPU **يوقظها** تلقائياً ولا يُعلَن عطلاً",
          hs_slp.ok and sleepy.awake and hs_slp.recoveries >= 1,
          f"إحياءات={hs_slp.recoveries} · مستيقظة={sleepy.awake} · "
          f"ok={hs_slp.ok}")
    check("وتفصيل الإحياء يسمّي السبب (سكون لا موت)",
          "السكون" in ((hs_slp.last_recovery or {}).get("detail") or ""),
          (hs_slp.last_recovery or {}).get("detail", ""))
    hs_slp.update()
    check("ويستأنف التكامل بعدها (لا عطل أبدي)", hs_slp.ok)
    # 🔴 نوم وسط لفّة: الزاوية المفقودة تُجهض اللفّة ولا تقتل المهمة
    class SleepyBridge(WaveRoverBridge):
        """جسر ينام حسّاسه بعد أول قراءات اللفّة (اندفاع تيار المحركات)."""
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self._n = 0
        def read_imu(self):
            d = super().read_imu()
            self._n += 1
            if 12 <= self._n <= 60:       # نافذة نوم وسط اللفّة
                d["gz"] = 0.0
            return d

    slp_br = SleepyBridge(mode="sim")
    slp_br.calibrate_gyro_bias(seconds=0.3)
    slp_br.heading_source._try_recover = lambda: {"recovered": True,
                                                  "detail": "أُوقظت (اختبار)"}
    r_slp = slp_br.turn_by_angle(90, timeout=6.0)
    check("🔴 نوم الحسّاس **وسط لفّة** يُجهضها (زاوية مفقودة لا تُبتلع)",
          r_slp.get("aborted") == "heading_slept_midturn"
          and any(e["kind"] == "heading_slept_midturn" for e in slp_br.events),
          f"aborted={r_slp.get('aborted')}")
    check("وليس من القاتلات — المهمة تعيد الخطوة (عطل تغذية عابر)",
          r_slp.get("aborted") not in ("heading_source_fault", "sign_mismatch",
                                       "rover_link_fault"))

    dead = _SleepyMPU()
    dead.recover = lambda: {"recovered": False, "detail": "الناقل لا يردّ"}
    hs_dead = _MPUH(dead, scale=1.0, sign=1)
    for _ in range(60):
        hs_dead.update()
    check("وحسّاس لا يستجيب للإحياء يبقى عطلاً بسببه المقروء (لا تظاهر)",
          not hs_dead.ok and not hs_dead.attempt_recovery()["recovered"])

    # ── 🔴 كاسر الانحشار: اللفّ على أرضية عالية الاحتكاك (سجاد/فرش) ──
    # عرَض مقاس 2026-08-10: «يرتجف ويلتفّ قليلاً ثم يقف». سببان مركّبان:
    # القوة 0.40 لا تكسر احتكاك الوبر، **والتهدئة تنزل بها إلى 0.25** قرب
    # الهدف فتتوقف الحركة داخل قوس التهدئة.
    from pi.config import (TURN_STALL_DPS as _TSD, TURN_MIN_POWER as _TMP,
                           TURN_STALL_FLOOR_FRAC as _TFF)

    class CarpetBridge(WaveRoverBridge):
        """منصّة لا تدور إلا فوق عتبة قوة (احتكاك سجاد)."""
        need = 0.47                       # فوق TURN_POWER الافتراضية 0.40
        def motors(self, l, r):
            out = super().motors(l, r)
            if abs(out["L"]) < self.need:
                self._sim_turn_rate = 0.0   # مأمور ولا يدور
            return out

    carpet = CarpetBridge(mode="sim")
    carpet.calibrate_gyro_bias(seconds=0.3)
    r_carp = carpet.turn_by_angle(90, timeout=8.0)
    check("🔴 أرضية عالية الاحتكاك: القوة تُرفع بالقياس حتى يدور فعلاً",
          r_carp.get("stall_boosts", 0) > 0 and abs(r_carp["turned_deg"]) > 45,
          f"{r_carp.get('stall_boosts')} زيادة · قوة {r_carp.get('power_used')} "
          f"· دار {r_carp['turned_deg']}°")
    check("والرفع معلَن حدثاً مقروءاً مرة واحدة (لا تعويض صامت)",
          sum(1 for e in carpet.events if e["kind"] == "turn_stall_boost") == 1,
          next((e["msg"][:64] for e in carpet.events
                if e["kind"] == "turn_stall_boost"), "لا حدث"))
    check("ولا يتجاوز الحاجز الصلب 0.5 مهما تكرّر الانحشار",
          r_carp.get("power_used", 0) <= MAX_MOTOR_POWER + 1e-9,
          f"{r_carp.get('power_used')} ≤ {MAX_MOTOR_POWER}")
    # أرضية التهدئة ترتفع مع القوة — وإلا وقف داخل قوس التهدئة
    from pi.rover.bridge import WaveRoverBridge as _WB
    _p_boosted = 0.50
    _floor = max(_TMP, _p_boosted * _TFF)
    check("وأرضية التهدئة ترتفع مع القوة (لا عودة إلى قوة الانحشار)",
          _WB._turn_power(1.0, 45.0, _p_boosted, _floor) >= _floor > _TMP,
          f"أرضية {_floor:.2f} > الثابتة {_TMP}")
    # المنصّة العادية لا تتأثر: صفر زيادة
    plain = WaveRoverBridge(mode="sim")
    plain.calibrate_gyro_bias(seconds=0.3)
    r_plain = plain.turn_by_angle(90, timeout=8.0)
    check("وعلى أرضية عادية: صفر زيادة (لا تدخّل بلا داعٍ)",
          r_plain.get("stall_boosts", 0) == 0
          and abs(r_plain.get("power_used", 0) - TURN_POWER) < 1e-9,
          f"زيادات={r_plain.get('stall_boosts')} · قوة={r_plain.get('power_used')}")

    # ── 🔴 الكاميرا تحيا بعد close (مغادرة النمط اليدوي قبل كل مهمة) ──
    # عطل مقاس 2026-08-10 وهو **السبب المباشر لصفر صورة**: `close()` كان
    # يحرّر المقبض بلا تصفيره، و`_ensure_open` يعود `isOpened()=False` أبداً
    # ⇒ الكاميرا ميتة لبقية عمر العملية. و`ModeManager` ينادي `close()` عند
    # مغادرة «القيادة اليدوية» — أي قبل كل مهمة ذاتية بالضبط.
    from pi.sensors.camera import CameraReader as _CamR

    class _FakeCap:
        def __init__(self): self._open = True
        def isOpened(self): return self._open
        def set(self, *a): return True
        def read(self): return (self._open, b"frame")
        def release(self): self._open = False

    cam = _CamR()
    cam._cap = _FakeCap()
    cam._opened = True
    check("الكاميرا مفتوحة قبل الإغلاق", cam.state()["opened"] is True)
    cam.close()
    check("🔴 `close()` يُصفّر المقبض (لا مقبض ميت يمنع إعادة الفتح للأبد)",
          cam._cap is None and cam._opened is False)
    check("وحالة الكاميرا تُقرأ من المقبض لا من راية بائتة (لا صحة كاذبة)",
          cam.state()["opened"] is False)
    cam2 = _CamR()
    cam2._cap = _FakeCap()
    cam2._cap.release()               # مقبض ميت (جهاز اختفى/حُرّر)
    check("ومقبض ميت يُطرح ليُعاد الفتح (لا `isOpened()=False` أبدية)",
          cam2._ensure_open() in (True, False) and cam2._cap is not True)

    # 🔴 لقطة تحجب بلا نهاية لا تجمّد المهمة (مقاس 2026-08-10: تجمّد دقائق
    #    عند التوثيق — `cv2` ينادي V4L2 بلا أي مهلة).
    cam3 = _CamR()
    cam3.available = True
    cam3._snapshot_blocking = lambda: time.sleep(30) or b"never"
    _t0 = time.time()
    _res = cam3.snapshot_jpeg(timeout_s=0.4)
    _dt = time.time() - _t0
    check("🔴 لقطة عالقة تُقطع بمهلتها ولا تجمّد خيط المهمة",
          _res is None and _dt < 3.0 and cam3.timeouts == 1,
          f"عادت خلال {_dt:.1f}ث · مهلات={cam3.timeouts}")
    check("والسبب معلَن في الحالة (لا فشل صامت)",
          "لا يستجيب" in (cam3.state().get("error") or "")
          and cam3.state()["stuck"] is True,
          (cam3.state().get("error") or "")[:50])
    check("ولا تُكدَّس خيوط عالقة (النداء التالي يرفض فوراً)",
          cam3.snapshot_jpeg(timeout_s=0.4) is None and cam3.timeouts == 1)
    cam4 = _CamR()
    cam4.available = True
    cam4._snapshot_blocking = lambda: b"JPEG-OK"
    check("واللقطة السليمة تمرّ عبر المهلة بلا تغيير",
          cam4.snapshot_jpeg(timeout_s=2.0) == b"JPEG-OK"
          and cam4.timeouts == 0)

    # 🔴 القفل **الحقيقي** لا محاكاته: الاختبارات أعلاه تستبدل
    #    `_snapshot_blocking` نفسه، فلا تمرّ بالقفل أصلاً — والانحدار الذي
    #    نحرسه هنا يقع **داخله**: العامل المهجور يبقى ممسكاً بالقفل بحكم
    #    التصميم، و`close()` يُنادى من **حلقة asyncio** (تبديل النمط عبر
    #    `/api/mode` وإغلاق السيرفر). فانتظار بلا مهلة ينقل التجمّد من خيط
    #    المهمة إلى الواجهة كلها: لا تيليمتري ولا زرّ إيقاف — والمحركات
    #    ماضية. (انحدار أدخلَته مهلةُ اللقطة نفسها.)
    import pi.sensors.camera as _cammod
    _cv2_saved = _cammod._CV2_OK
    _cammod._CV2_OK = True                # المسار الحقيقي بلا opencv مثبّتة
    try:
        class _HangCap(_FakeCap):
            def read(self):
                time.sleep(30)            # V4L2 يحجب بلا نهاية
                return (True, b"frame")

        cam5 = _CamR()
        cam5.available = True
        cam5._cap = _HangCap()
        cam5._opened = True
        cam5.snapshot_jpeg(timeout_s=0.4)  # عامل يعلق **ممسكاً بالقفل**
        check("عامل اللقطة العالق يملك القفل فعلاً (لا محاكاة للمسار)",
              cam5._lock.locked() is True and cam5.timeouts == 1)
        _t1 = time.time()
        cam5.close()
        _dt1 = time.time() - _t1
        check("🔴 `close()` لا ينتظر قفلاً محتجزاً — الواجهة لا تتجمّد",
              _dt1 < 3.0 and cam5._cap is None and cam5.abandoned == 1,
              f"عاد خلال {_dt1:.1f}ث · مهجورة={cam5.abandoned}")
        check("والهجر معلَن في الحالة (لا تسريب صامت لمقبض)",
              "هُجر" in (cam5.state().get("error") or "")
              and cam5.state()["abandoned"] == 1)
        # الجيل يرتفع عند الهجر: العامل إن استفاق يحرّر **مقبضه هو** فقط
        check("ورفع الجيل يمنع عاملاً مهجوراً من تحرير مقبض فُتح بعده",
              cam5._gen == 1)
    finally:
        _cammod._CV2_OK = _cv2_saved

    # ── 🔴 العودة الإجبارية تُنتج تقريراً وتوثيقاً (لا تخرج بلا مخرَج) ──
    # عطل مقاس 2026-08-10: `_finish` يستدعي الدورة في فرع «انتهى المسح»
    # وحده، والعودة الإجبارية (زمن 480ث أو جهد 10.2–10.8V) شبه حتمية —
    # فأكثر المهمات واقعيةً كانت تخرج بلا تقرير ولا سبب.
    ms_rt = full_mission(run=False)
    ms_rt._returning = True
    ms_rt.state = RUNNING
    ms_rt._finish()
    check("🔴 مهمة انتهت بعودة إجبارية **تمرّ بالدورة** (تقرير لا صمت)",
          ms_rt.cycle is not None
          and any(e["kind"] == "cycle_after_return" for e in ms_rt.events),
          f"مراحل={[p['phase'] for p in (ms_rt.cycle or {}).get('phase_log', [])]}")
    check("ولا تُنفَّذ فيها أي مرحلة حركية (الحالة IDLE تمنع القيادة)",
          ms_rt._can_drive() and not ms_rt._can_move(),
          f"can_drive={ms_rt._can_drive()} · can_move={ms_rt._can_move()}")

    # ── 🔴 التوثيق يُنفَّذ من المسافة الآمنة ولو مُنع الاقتراب ──────
    # ثغرة مقاسة 2026-08-09: تتبّع التدرّج بلغ عتبة التوقف وأعلن «جاهز
    # للتوثيق البصري»، فرفضه حاجب نصّه نفسه يقول «وثّق من بعيد» — أي أن
    # كل مصدر قويّ بما يكفي ليكون مهماً كان غير قابل للتوثيق.
    from pi.ai.approach_document import (approach_blockers as _apb,
                                         documentation_blockers as _dcb)
    _hot = {"hazard_level": "normal", "max_usvh": 3000.0,
            "position_reliable": True, "found": True,
            "protocol": {"perimeter_start_ok": True}}
    check("🔴 جرعة فوق عتبة التوقف: الاقتراب ممنوع **والتوثيق مسموح**",
          any("عتبة التوقف" in b for b in _apb(_hot)) and not _dcb(_hot),
          f"موانع اقتراب={len(_apb(_hot))} · موانع توثيق={len(_dcb(_hot))}")
    _blind = dict(_hot, position_reliable=False)
    check("وبلا موقع موثوق يُمنع التوثيق (لا وجهة للكاميرا)",
          any("وجهة" in b for b in _dcb(_blind)))
    from pi.ai import dynamic_range as _drm
    _evac = dict(_hot, hazard_level=_drm.HAZARD_EVACUATE)
    check("وعند الإخلاء يُمنع التوثيق (الفرار أولوية مطلقة)",
          any("الانسحاب أولاً" in b for b in _dcb(_evac)))

    # ── 🔴 حدّ التسلسل: **لا بايتات في أي حمولة تُبثّ** ───────────────
    # عطل مقاس 2026-08-10 وهو سبب «الموقع علق ولا طلعت صورة» فعلياً:
    # `run_documentation` كان يضع نسخة ثانية من صور JPEG في
    # `capture.images`، و`_doc_state` يجرّد المفتاح الأعلى وحده. فمرّت
    # البايتات إلى حمولة الحالة ⇒ `/api/sim/status` ردّ **500**
    # (`UnicodeDecodeError: byte 0xff` — أول بايت في JPEG) **وكل WebSocket
    # مات عند أول بثّ بعد التوثيق**، فأعادت الواجهة الاتصال بلا توقف.
    # المهمة كانت قد أتمّت التصوير والتقرير بنجاح — الذي مات هو العرض.
    # ⚠ الحارس يسلك المسار **الحقيقي** (`run_documentation` بكاميرا تُعيد
    #   JPEG فعلياً) لا قاموساً مُلفَّقاً: التسريب وقع في التركيب لا في
    #   `_doc_state` وحده، فقاموس مُلفَّق كان سيمرّ ويُخفي العطل نفسه.
    from pi.ai.approach_document import run_documentation as _run_doc
    import pi.nav.mission as _mn

    class _JpegCam:
        def snapshot_jpeg(self, *a, **k):
            return b"\xff\xd8\xff\xe0JFIF-fake-payload"   # بايت 0xff الحقيقي

    _doc_rep = dict(_hot, position=(1.5, 1.5), uncertainty_m=0.2,
                    confidence=0.9, headline="اختبار")
    ms_ser = MissionSim()
    ms_ser.configure_room(2.0, 2.0)
    ms_ser.documentation = _run_doc(_doc_rep, robot_xy=(0.5, 0.5),
                                    robot_heading_deg=0.0,
                                    camera=_JpegCam(), turn_fn=lambda d: True)
    check("التوثيق التُقطت صوره فعلاً (شرط أن يكون الحارس ذا معنى)",
          len(ms_ser.documentation.get("images") or []) >= 1,
          f"{len(ms_ser.documentation.get('images') or [])} صورة خام")

    def _has_bytes(v, d=0):
        if d > 9:
            return False
        if isinstance(v, (bytes, bytearray)):
            return True
        if isinstance(v, dict):
            return any(_has_bytes(x, d + 1) for x in v.values())
        if isinstance(v, (list, tuple)):
            return any(_has_bytes(x, d + 1) for x in v)
        return False

    # 🔴 والحارس يمرّ بـ**تجميع الدورة الحقيقي** (`_run_cycle`) لا بإسناد
    #    `.documentation` وحده: `self.cycle` يُبثّ خاماً، وخمسة مسارات خروج
    #    كانت تضع فيه `_document_source()` بصوره. حارس يُسند الحقل مباشرةً
    #    يترك `cycle = None` فيمرّ أخضر على شجرة معطوبة — وهو ما حدث فعلاً.
    _doc_real = ms_ser.documentation

    def _fake_document_source(_self=ms_ser, _d=_doc_real):
        """يقف مقام «الكاميرا التقطت» — الجزء الوحيد غير القابل للمحاكاة."""
        _self.documentation = _d
        return _d

    ms_ser._document_source = _fake_document_source
    ms_ser._feed_locator(1.0, 1.0, {"counts": 5, "duration_s": 3.0})
    ms_ser._run_cycle()                       # مسار التجميع الحقيقي
    check("الدورة جُمّعت فعلاً (وإلا كان الحارس يفحص cycle=None)",
          isinstance(ms_ser.cycle, dict) and "documentation" in ms_ser.cycle,
          f"مفاتيح الدورة: {sorted(ms_ser.cycle)[:4]}")
    check("🔴 والتوثيق يدخل الدورة **مجرَّداً** (n_images لا بايتات)",
          (ms_ser.cycle["documentation"] or {}).get("n_images", 0) >= 1
          and "images" not in (ms_ser.cycle["documentation"] or {}),
          f"n_images={(ms_ser.cycle['documentation'] or {}).get('n_images')}")

    _st = ms_ser.state_dict(include_full_grid=True)
    _rp = ms_ser.report()
    _ser_ok, _ser_err = True, ""
    try:
        json.dumps(_st, ensure_ascii=False)
        json.dumps(_rp, ensure_ascii=False)
    except Exception as e:                    # noqa: BLE001
        _ser_ok, _ser_err = False, str(e)[:70]
    check("🔴 حمولة الحالة والتقرير تُسلسَلان JSON بلا استثناء (لا 500 بعد التوثيق)",
          _ser_ok, _ser_err or "state_dict + report نظيفان")
    check("ولا بايتة واحدة تنجو **بأي عمق** (لا في capture.images)",
          not _has_bytes(_st) and not _has_bytes(_rp))
    check("والصور تُعدّ ولا تُبثّ — مسارها /api/doc/image/{i}",
          (_st.get("documentation") or {}).get("n_images", 0) >= 1
          and "images" not in (_st.get("documentation") or {}),
          f"n_images={(_st.get('documentation') or {}).get('n_images')}")
    check("والبايتات الخام تبقى في الذاكرة ليقدّمها المسار (لم تُتلف)",
          isinstance((ms_ser.documentation.get("images") or [None])[0], bytes))
    # وحارس العمق نفسه مُختبَر مستقلاً (بنية متداخلة كما في capture)
    _nested = {"a": [{"b": {"c": b"\xff\xd8"}}]}
    check("`json_safe` يستبدل البايتات بواصف حجم بأي عمق",
          _mn.json_safe(_nested)["a"][0]["b"]["c"] == {"_bytes": 2})
    # 🔴 الطبقة الثانية: حقل **مستقبلي** يُضاف إلى الدورة بلا انتباه.
    #    `cycle` قاموس حرّ تُضاف إليه حقول مع كل مرحلة — فالحارس عند حدّ
    #    البثّ هو ما يمنع تكرار العطل نفسه بمفتاح لم يُخترع بعد.
    ms_ser.cycle["حقل_مستقبلي"] = {"لقطة": b"\xff\xd8\xff"}
    try:
        json.dumps(ms_ser.state_dict(), ensure_ascii=False)
        json.dumps(ms_ser.report(), ensure_ascii=False)
        _fut_ok = True
    except Exception:                         # noqa: BLE001
        _fut_ok = False
    check("🔴 وحقل جديد يحمل بايتات في الدورة يُقتل عند حدّ البثّ",
          _fut_ok and not _has_bytes(ms_ser.state_dict()),
          "الحارس لا يعتمد على معرفة أسماء الحقول")

    # ── إزاحة أنبوب الجيجر: القراءة تُنسب للأنبوب لا لمركز الروبوت ──
    # ⚠ الأنبوب على يسار الهيكل (2026-08-09) — نسبته للمركز = انحياز موضع
    #   ثابت بحجم الإزاحة في كل تقدير، وخطأ الموقع المستهدف 18سم.
    import pi.nav.mission as _mn
    ms_go = MissionSim()
    ms_go.configure_room(2.0, 2.0)
    _sv_off = _mn.GEIGER_OFFSET_LEFT_M
    try:
        _mn.GEIGER_OFFSET_LEFT_M = 0.5
        ms_go.heading = 0.0
        ms_go._feed_locator(1.0, 1.0, {"counts": 5, "duration_s": 3.0})
        rr_off = ms_go.locator.readings[-1]
        check("قراءة الجيجر تُنسب لموضع **الأنبوب** (إزاحة يسارية مُدارة بالاتجاه)",
              abs(rr_off["x"] - 0.5) < 1e-9 and abs(rr_off["y"] - 1.0) < 1e-9,
              f"مركز (1,1) + يسار 0.5 عند 0° ⇒ ({rr_off['x']}, {rr_off['y']})")
        ms_go.heading = 90.0
        ms_go._feed_locator(1.0, 1.0, {"counts": 5, "duration_s": 3.0})
        rr_off2 = ms_go.locator.readings[-1]
        check("والإزاحة تدور مع اتجاه الروبوت (90° ⇒ اليسار صار +y)",
              abs(rr_off2["x"] - 1.0) < 1e-9 and abs(rr_off2["y"] - 1.5) < 1e-9,
              f"({rr_off2['x']}, {rr_off2['y']})")
    finally:
        _mn.GEIGER_OFFSET_LEFT_M = _sv_off

    # ── 🎯 المصدر التدريبي: عدّ مصنّع فوق مسار القياس الواحد ────────
    # بروفة الاختبار (طلب المشغّل 2026-08-09): قيادة حقيقية وعدّ بواسون من
    # التربيع العكسي — معلَن بحدث ووسم في كل بثّ، ولا يمرّ كقياس حقيقي.
    ms_tr = MissionSim()
    ms_tr.configure_room(3.0, 3.0, bg_cpm=18.0)
    ms_tr.set_training_source(1.0, 1.0, usvh_1m=100.0)
    m_tr = ms_tr._measure(1.0, 2.0, 3.0)      # على بعد 1م من المصدر
    check("🎯 المصدر التدريبي يصنّع عدّات التربيع العكسي (وسم training)",
          m_tr["window"] == "training" and 380 < m_tr["counts"] < 750,
          f"{m_tr['counts']:.0f} عدّة (متوقّع ~556 عند 1م بشدة 100µSv/h)")
    check("وتفعيله معلَن حدثاً ويُوسم في البثّ",
          any(e["kind"] == "training_source" for e in ms_tr.events)
          and ms_tr.state_dict()["training_source"] is not None)
    ms_tr.set_training_source()
    check("ومسحه يعيد العدّ للمسار الأصلي فوراً",
          ms_tr.training_source is None
          and ms_tr._measure(1.0, 2.0, 0.1)["window"] != "training")

    # ── عتبة ثقة الأمامي (30سم — الأمامي وحده، الجانبان مثبتان) ─────
    class FarUS:
        ok = True
        distance_cm = 200.0
        quality = 90
    ms_ft = MissionSim()
    ms_ft.configure_room(2.0, 2.0)
    ms_ft.set_proximity(FarUS(), None)
    s_ft = ms_ft.sensors()
    check("🔴 أمامي فوق عتبة الثقة ⇒ مجهول للقرار والخام موسوم للعرض",
          s_ft["ultrasonic_cm"] is None and s_ft["ultrasonic_raw_cm"] == 200.0,
          f"خام={s_ft['ultrasonic_raw_cm']} · قرار={s_ft['ultrasonic_cm']}")
    FarUS.distance_cm = 25.0
    check("ودون العتبة يُعتمد كما هو",
          ms_ft.sensors()["ultrasonic_cm"] == 25.0)

    # ═══ (ع) حارس «المبنيّ غير المستدعى» — قاعدة CLAUDE.md §8 آلياً ═══
    # النمط تكرر ست مرات: وحدة مبنيّة ومختبَرة وحدةً ولا يستدعيها أحد في
    # مسار المهمة، واختبار الوحدة المعزول يمرّ وإن لم يستدعِها أحد. الحارس
    # يفرض البديلين المشروعين: استدعاء إنتاجي (أي ملف غير اختباري في pi/،
    # ومنه السيرفر) أو وسم «أداة خارجية» في السطر الأول من docstring.
    # ⚠ فحص نصّي تقريبي: ذكر الاسم في تعليق يُحتسب استدعاءً — فهو يمسك
    #   الاسم الذي لا يذكره أحد إطلاقاً، وهذه كانت بصمة الحالات الست كلها.
    print("\nع) حارس التوصيل:")
    import ast as _ast
    import pathlib as _pl
    import re as _re

    _pi_dir = _pl.Path(__file__).resolve().parents[1]

    def _is_test_path(rp: str) -> bool:
        return "/tests/" in rp or rp.endswith("selftest.py")

    _prod = {}
    for _p in _pi_dir.rglob("*.py"):
        _rp = _p.relative_to(_pi_dir.parent).as_posix()
        if not _is_test_path(_rp):
            _prod[_rp] = _p.read_text(encoding="utf-8")

    # ⚠ ثلاث واجهات مهمة جاهزة ومختبَرة تنتظر ربط الواجهة (المسار الآخر).
    #   وجودها هنا **مؤقت معلَن** لا إعفاء دائم: إن وُصلت أو وُسمت فأزل
    #   سطرها — الفحص الثاني يصرخ على الإعفاء البائت عمداً.
    # كانت هنا قائمة «بانتظار ربط الواجهة» لثلاث واجهات مهمة بلا مستدعٍ —
    # فوصّلها المسار الآخر كلها والحارس كشف إعفاءها البائت واحدة واحدة:
    # request_withdraw (أمر WDRAW في pi/comms/control.py) ·
    # set_side_ultrasonic (server.init_side_ultrasonic عند الإقلاع) ·
    # run_perimeter_cycle (السيرفر). أي إعفاء جديد يُضاف هنا **بسبب وتاريخ**.
    _pending_web: set = set()

    def _public_funcs(tree):
        out = []
        for node in _ast.iter_child_nodes(tree):
            if isinstance(node, (_ast.FunctionDef, _ast.AsyncFunctionDef)):
                out.append(node)
            elif isinstance(node, _ast.ClassDef):
                out.extend(n for n in _ast.iter_child_nodes(node)
                           if isinstance(n, (_ast.FunctionDef,
                                             _ast.AsyncFunctionDef)))
        return [f for f in out if not f.name.startswith("_")]

    _viol, _stale = [], []
    for _d in ("pi/ai", "pi/nav"):
        for _f in sorted((_pi_dir.parent / _d).glob("*.py")):
            _rp = _f.relative_to(_pi_dir.parent).as_posix()
            if _f.name in ("__init__.py", "selftest.py", "sim_world.py"):
                continue
            for _fn in _public_funcs(_ast.parse(_prod[_rp])):
                _key = f"{_rp}::{_fn.name}"
                _tagged = "أداة خارجية" in (_ast.get_docstring(_fn) or "")
                _name = _re.compile(r"\b" + _re.escape(_fn.name) + r"\b")
                _defs = _re.compile(r"def\s+" + _re.escape(_fn.name) + r"\b")
                _refs = sum(len(_name.findall(t)) - len(_defs.findall(t))
                            for t in _prod.values())
                if _key in _pending_web:
                    if _refs > 0 or _tagged:
                        _stale.append(_key)
                    continue
                if _refs <= 0 and not _tagged:
                    _viol.append(_key)
    check("كل دالة عامة في pi/ai وpi/nav مستدعاة إنتاجياً أو موسومة «أداة خارجية»",
          not _viol,
          " · ".join(_viol[:4]) if _viol else f"فُحصت {len(_prod)} وحدة إنتاجية")
    check("قائمة «بانتظار ربط الواجهة» ما زالت دقيقة (لا إعفاء بائتاً)",
          not _stale,
          " · ".join(_stale) if _stale else f"{len(_pending_web)} واجهات معلَّقة")

    # الخلاصة
    passed = sum(_results)
    total = len(_results)
    print(f"\n=== النتيجة: {passed}/{total} نجح ===")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
