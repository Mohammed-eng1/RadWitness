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
import time

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
        make_heading_source, PHASES, ZERO_RUN_DEAD,
    )
    from pi.config import HEADING_RECOVERY_ATTEMPTS
    from pi.nav.heading_hold import (
        HeadingController, signed_error, available_headroom, config_sanity,
    )
    from pi.config import (
        HEADING_SOURCE, BNO055_GYRO_SCALE, HEADING_MAX_CORR, STRAIGHT_BASE_POWER,
        HEADING_SPIKE_DPS_DRIVE, HEADING_SPIKE_DPS_TURN, MOTOR_TRIM_L,
        BATTERY_MONITOR_ENABLED, MISSION_TIME_LIMIT_S, MISSION_HARD_LIMIT_S,
        MIN_MOTOR_POWER,
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
    check("المراقبة معطّلة والحدود مرتّبة تحذير<عودة<إيقاف",
          not BATTERY_MONITOR_ENABLED
          and MISSION_TIME_LIMIT_S < MISSION_HARD_LIMIT_S,
          f"RTH={MISSION_TIME_LIMIT_S:.0f}ث · إيقاف={MISSION_HARD_LIMIT_S:.0f}ث")
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
    check("الجهد غير مقروء → لا يُعرض تصنيف كاذب",
          ms2.rover.check_battery()["level"] == "unknown"
          and ms2.state_dict()["battery"]["level"] == "unknown")

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
    ms3.start()
    ms3._worker.join(timeout=10.0)
    aborts = [e for e in ms3.events if e["kind"] == "mission_abort"]
    check("عطل الاتجاه يُنهي المهمة بسبب صريح (لا حلقة لا نهائية)",
          ms3.state == "estop" and aborts and not ms3._worker.is_alive(),
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
    check("السير المستقيم يبقى على عتبته الضيّقة المعايرة (لا توسيع مجاني)",
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

    # ═══ (س) الاتجاه البصري — منطق خالص بصور مُزاحة اصطناعياً ══════
    # الجايرو مصدر **نسبي** يتراكم خطؤه؛ الكاميرا مرجع **مطلق** لأن المشهد لا
    # يتحرك. كل ما يلي numpy فقط — بلا كاميرا وبلا cv2 (يعمل على ويندوز).
    print("\nس) الاتجاه البصري (مرجع مطلق يكسر تراكم الجايرو):")
    import numpy as _np
    from pi.sensors.visual_heading import (
        column_signature, signature_contrast, match_shift, shift_to_deg,
        compare, VisualHeading, _grid_key,
    )
    from pi.config import (
        VISUAL_MIN_CONTRAST, VISUAL_MAX_CORRECTION_DEG, CAMERA_HFOV_DEG,
    )

    _rng = _np.random.default_rng(7)
    W, H = 320, 240
    scene = _np.repeat(_rng.integers(0, 255, size=(1, W)).astype(_np.float32),
                       H, axis=0)                    # مشهد عمودي غنيّ بالمعالم

    def shifted(px):
        """المشهد نفسه مُزاحاً أفقياً px بكسل (محاكاة دوران الروبوت)."""
        return _np.roll(scene, px, axis=1)

    sig0 = column_signature(scene)
    check("توقيع الأعمدة بطول عرض الصورة", len(sig0) == W, f"{len(sig0)}")
    check("مشهد غنيّ بالمعالم يتجاوز عتبة التباين",
          signature_contrast(sig0) >= VISUAL_MIN_CONTRAST,
          f"تباين={signature_contrast(sig0):.1f}")

    m = match_shift(sig0, column_signature(shifted(17)))
    check("المطابقة تستخرج الإزاحة بدقّة دون-البكسل",
          m["ok"] and abs(m["shift_px"] - 17.0) < 1.0,
          f"طُلب 17 → {m['shift_px']:.2f} بكسل (ارتباط {m['peak']:.2f})")
    mneg = match_shift(sig0, column_signature(shifted(-11)))
    check("الإشارة صحيحة في الاتجاهين",
          mneg["ok"] and abs(mneg["shift_px"] + 11.0) < 1.0,
          f"{mneg['shift_px']:.2f}")

    # ⚠ الإضاءة تتغيّر باستمرار في غرفة حقيقية — التطبيع يجعلها لا تُذكر
    bright = column_signature(shifted(9) * 1.4 + 25.0)
    mb = match_shift(sig0, bright)
    check("تغيّر الإضاءة لا يزيح المطابقة (تطبيع)",
          mb["ok"] and abs(mb["shift_px"] - 9.0) < 1.0,
          f"سطوع ×1.4 +25 → {mb['shift_px']:.2f}")

    # ── الحرّاس: الحالات التي **يجب** أن تُرفض ─────────────────────
    blank = _np.full((H, W), 128.0, dtype=_np.float32)
    c_blank = compare(column_signature(blank), column_signature(blank), 60.0)
    check("جدار أملس بلا معالم → «لا أعرف» لا رقم واثق",
          not c_blank["ok"] and "معالم" in c_blank["reason"], c_blank["reason"])

    # ⚠ انحدار مقاس على العتاد: مشهد **سليم تماماً** كان يُرفض «نمطاً متكرّراً»
    #   لأن جوار استبعاد الذروة كان ثابتاً (±3 بكسل) بينما الارتباط الذاتي
    #   لمشهد طبيعي منعَّم عريض — فعند ±4 بكسل يبقى ~0.98. (سُجّل: تباين 36.3
    #   وارتباط 1.00 وإزاحة 0.00 رُفض بحدّة 0.02 — أي رُفض أفضل مشهد ممكن.)
    #   الفصّ الرئيسي يُحدَّد بالنزول من الذروة، لا بجوار ثابت.
    natural = _np.repeat(
        _np.convolve(_rng.normal(128, 40, W + 20), _np.ones(21) / 21,
                     mode="valid")[:W][None, :].astype(_np.float32), H, axis=0)
    nat_sig = column_signature(natural)
    c_same = compare(nat_sig, column_signature(natural.copy()), 60.0)
    check("مشهد طبيعي مقابل نفسه يُقبل (لا يُرفض كنمط متكرّر)",
          c_same["ok"] and abs(c_same["deg"]) < 0.5,
          f"حدّة={c_same.get('margin')} · إزاحة={c_same.get('shift_px')} بكسل")
    m_nat = match_shift(nat_sig, column_signature(_np.roll(natural, 6, axis=1)))
    check("الفصّ الرئيسي أعرض من ±3 بكسل (سبب الرفض الكاذب)",
          (m_nat["lobe"][1] - m_nat["lobe"][0] + 1) > 7,
          f"عرض الفصّ={m_nat['lobe'][1] - m_nat['lobe'][0] + 1} خطوة")

    # نمط دوري (بلاط/ستائر/أرفف): ذرى متساوية عند إزاحات مختلفة
    per = _np.tile(_np.array([0, 0, 0, 0, 255, 255, 255, 255], dtype=_np.float32),
                   W // 8)
    per_img = _np.repeat(per[None, :], H, axis=0)
    c_per = compare(column_signature(per_img),
                    column_signature(_np.roll(per_img, 24, axis=1)), 60.0)
    check("نمط متكرّر → يُرفض بحدّة الذروة لا يُصدَّق أعلاه بفارق ضئيل",
          not c_per["ok"], c_per.get("reason", "قُبل!"))

    # ⚠ نفس درس τ: رقم يُضرب ولم يُقَس ⇒ رفض صريح لا اختراع
    raised = False
    try:
        shift_to_deg(10.0, W, hfov_deg=0.0)
    except ValueError:
        raised = True
    check("HFOV غير مقاس → رفض صريح لا رقم مخترع", raised)
    check("HFOV مقاس → تحويل خطي صحيح",
          abs(shift_to_deg(W / 4.0, W, hfov_deg=60.0, sign=+1) - 15.0) < 1e-6,
          f"{shift_to_deg(W / 4.0, W, 60.0, +1):.2f}°")

    # ── المراسي: مقارنة بمرجع مطلق لا تكامل ───────────────────────
    check("مفتاح المرساة يثبّت على الاتجاه الشبكي",
          (_grid_key(88.6), _grid_key(271.0), _grid_key(359.5)) == (90, 270, 0))

    class FakeCam:
        """كاميرا وهمية: المشهد يُزاح بمقدار ما «دار» الروبوت."""
        def __init__(self):
            self.px = 0
        def frame_array(self):
            return shifted(self.px)

    cam = FakeCam()
    vh = VisualHeading(cam, hfov_deg=60.0, sign=+1)
    check("الاتجاه البصري جاهز بعد قياس HFOV", vh.ready and not vh.error)
    check("مرساة تُثبَّت أول مرة فقط",
          vh.set_anchor(90.0) and not vh.set_anchor(90.0) and vh.anchors == {90: vh.anchors[90]})
    cam.px = 32                      # المشهد انزاح ⇒ الروبوت دار
    r = vh.residual_deg(90.0)
    check("الانحراف عن المرساة يُقاس بالدرجات (لا تكامل)",
          r["ok"] and abs(r["deg"] - 6.0) < 0.6,
          f"32 بكسل من 320 عند HFOV=60 → {r.get('deg')}° (المتوقَّع 6.0)")
    check("لا مرساة للاتجاه ⇒ «لا أعرف» لا صفر",
          not vh.residual_deg(0.0)["ok"])
    # 80 بكسل من 320 عند HFOV=60 ⇒ 15° — داخل مدى البحث لكن فوق سقف التصحيح
    cam.px = 80                      # انحراف كبير = مشهد تغيّر لا دوران
    big = vh.residual_deg(90.0)
    check("انحراف بصري فوق السقف يُرفض (مطابقة خاطئة أخطر من غيابها)",
          not big["ok"] and str(VISUAL_MAX_CORRECTION_DEG) in big["reason"],
          big["reason"][:56])
    check("العلم مطفأ افتراضياً حتى يُقاس HFOV على العتاد",
          CAMERA_HFOV_DEG == 0.0 and not VisualHeading(cam).ready)

    # الخلاصة
    passed = sum(_results)
    total = len(_results)
    print(f"\n=== النتيجة: {passed}/{total} نجح ===")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
