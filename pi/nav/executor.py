# -*- coding: utf-8 -*-
"""
executor.py — المرحلة 2: تحويل خطوة الشبكة إلى حركة حقيقية
==========================================================
الحلقة الوسطى في تدرّج الطبقات (CLAUDE.md):

    ملاحة (mission/planner)  → تقرر الخلية التالية
            ↓
    **executor (هنا)**        → لفّ نحو الاتجاه + تقدّم خلية واحدة
            ↓
    سلامة تفاعلية (reactive)  → **أولوية مطلقة**: تُجهض الحركة فوراً

كل تقدّم **تحت إشراف مستمر**: تُقرأ الحساسات كل REACTIVE_LOOP_S، والسرعة
تُشتق من سلّم السلامة (لا ثابتة)، والمسافة تُتكامل من السرعة **المعايرة**
الفعلية لا من افتراض. أي قرار غير "go" يوقف المحرك ويُجهض الخطوة بسببها.

⚠ لا يفترض هذا الملف عتاداً: يعمل مع جسر `sim` أو `real` بنفس الشيفرة،
   فيمكن اختبار المنطق كاملاً على ويندوز قبل لمس المحركات.
"""
from __future__ import annotations

import time

from pi.config import (
    REACTIVE_LOOP_S, ROVER_TURN_TIMEOUT_S, CELL_DWELL_S,
    WALL_ALIGN_TOL_DEG, MAX_MOTOR_POWER, SPEED_LADDER,
    KNOWN_WALL_TOL_M, HARD_STOP_CM, IR_RANGE_CM,
    HEADING_HOLD_IN_MISSION, HEADING_HOLD_MAX_BASE, SPEED_RAMP_UP_PER_S,
    MOTION_VERIFY_ENABLED, MOTION_SETTLE_S, MOTION_BACKWARD_MAX_M,
)
from pi.nav.room import CELL_SIZE_M
from pi.nav.heading_hold import HeadingController, signed_error
from pi.nav.motion_check import AccelWitness, verify_motion

# أقصى زمن لعبور خلية واحدة قبل اعتبار الخطوة فاشلة (حماية من الانحشار)
MAX_CELL_TRAVEL_S = 12.0


def _ang_signed(frm: float, to: float) -> float:
    return (to - frm + 180.0) % 360.0 - 180.0


class DriveExecutor:
    """
    ينفّذ خطوة واحدة (لفّ ثم تقدّم خلية) على الجسر، تحت إشراف السلامة.
    - `rover`    : WaveRoverBridge (sim أو real)
    - `reactive` : ReactiveSafety (سلّم السرعة + قرار التفادي)
    - `sensors`  : دالة تُعيد {"ultrasonic_cm", "ir_left", "ir_right"}
    - `profile`  : CalibrationProfile (لتحويل القوة → م/ث المعايرة)
    """

    def __init__(self, rover, reactive, sensors, profile, imu=None):
        self.rover = rover
        self.reactive = reactive
        self.sensors = sensors
        self.profile = profile
        self.log = []
        # **نفس المتحكّم لا نسخة ثانية** — ثوابته معايرة على العتاد.
        self.heading_ctl = HeadingController()
        # مقياس التسارع للشاهد الثنائي (البند 0). يُؤخذ من مصدر الاتجاه —
        # **نفس نسخة IMUReader** لا ثانية: فتحُ I2C مرتين يتسابق على الناقل.
        self.imu = imu if imu is not None else getattr(
            getattr(rover, "heading_source", None), "imu", None)
        # ⚠ التحقق يخصّ **العتاد**: في وضع sim لا عالم فيزيائي يُقاس، والقراءة
        #    الأمامية مشتقّة من نفس الموقع المحسوب الذي نريد التحقق منه — فلا
        #    تُثبت شيئاً وتُنتج «لا حركة» كاذبة توقف المهمة.
        self.verify_motion_enabled = (MOTION_VERIFY_ENABLED
                                      and getattr(rover, "mode", "sim") == "real")

    # ── اللفّ نحو اتجاه مطلوب ────────────────────────────────────
    def turn_to(self, current_heading: float, target_heading: float) -> dict:
        delta = _ang_signed(current_heading, target_heading)
        if abs(delta) < 2.0:
            return {"ok": True, "turned_deg": 0.0, "skipped": True}
        res = self.rover.turn_by_angle(delta, timeout=ROVER_TURN_TIMEOUT_S)
        # `aborted` يعني عطلاً في مصدر الاتجاه أو إشارة محور مقلوبة — فشل
        # صريح لا يجوز اعتباره لفّة ناجحة (وإلا تقدّم الروبوت باتجاه خاطئ).
        return {"ok": not res["timed_out"] and not res.get("aborted"),
                "turned_deg": res["turned_deg"], "aborted": res.get("aborted"),
                "segments": res.get("segments", 1), "timed_out": res["timed_out"]}

    # ── قراءتا التحقق من الحركة (البند 0) ────────────────────────
    def _settled_front_cm(self):
        """
        المسافة الأمامية **بعد استقرار المرشّح**. مرشّح الألترا سونيك وسيط 5
        ثم EMA، وكل قياس ~60ms ⇒ نافذته لا تتبدّل قبل ~0.3ث. القراءة الفورية
        تخلط ما قبل الحركة بما بعدها فتُنتج فرقاً مصغَّراً — أي تحقّقاً يمرّ
        على أخطاء حقيقية. `None` تُمرَّر كما هي (لا سطح مرجعي).
        """
        time.sleep(MOTION_SETTLE_S)
        try:
            return self.sensors().get("ultrasonic_cm")
        except Exception:                          # noqa: BLE001
            return None

    def _read_accel(self):
        """(ax, ay, az) م/ث² أو None — غياب الحسّاس **ليس خطأً** هنا."""
        if self.imu is None:
            return None
        try:
            return self.imu.accel_mps2()
        except Exception:                          # noqa: BLE001 — I2C عابر
            return None

    # ── التقدّم خلية واحدة تحت إشراف السلامة ─────────────────────
    def forward_cell(self, distance_m: float = CELL_SIZE_M,
                     expected_wall_end_m=None) -> dict:
        """
        يتقدّم `distance_m` بسرعة يقررها سلّم السلامة لحظياً.

        `expected_wall_end_m`: المسافة المتوقَّعة **من الخريطة** بين نهاية
        الخطوة والجدار المواجه. تُستخدم لتمييز **الجدار المعروف** عن العائق
        المفاجئ: بدونها لا يمكن دخول أي خلية طرفية (مركزها 25سم من الجدار
        بينما عتبة التوقف 30سم) فتُعلَّم خطأً كأنها محجوبة.
        عند مطابقة المقاس للمتوقَّع نزحف بدل الإجهاض — مع أرضية صلبة
        (HARD_STOP_CM) توقف مهما كان السبب.
        """
        covered = 0.0
        started = time.time()
        aborted = None
        last_decision = None
        crawl_power = SPEED_LADDER[-1][1]

        # ── 🔴 مرجع التحقق من الحركة: المسافة الأمامية **قبل** التحرّك ──
        # تُؤخذ بعد استقرار المرشّح: اللفّة السابقة تُمرّر الحسّاس على الغرفة
        # كلها، فقراءة فورية بعدها خليط من أسطح لم نعد نواجهها.
        witness = AccelWitness() if self.verify_motion_enabled else None
        d_start = self._settled_front_cm() if self.verify_motion_enabled else None

        # ── تثبيت الاتجاه: الهدف هو الاتجاه **لحظة بدء العبور** ──────
        # يُلتقط من الحسّاس لا من إطار الغرفة: مرجع مصدر الاتجاه يُصفَّر
        # مستقلاً عن `mission.heading` فقد يكون الإطاران مزاحين، والتقاطه
        # هنا يجعل الشوط **خطاً مستقيماً** أياً كان الإطار.
        src = getattr(self.rover, "heading_source", None)
        hold = None
        if HEADING_HOLD_IN_MISSION and src is not None and getattr(src, "ok", False):
            src.set_phase("drive")   # عتبة السير (12°/ث) لا عتبة اللفّ (200)
            src.update()             # يثبّت مرجع الزمن قبل أول تصحيح
            hold = src.heading
            self.heading_ctl.reset()
        hold_lost = None
        base_capped = None       # (المطلوب من السلّم، المطبَّق) عند تقييد السقف
        applied = 0.0            # آخر قوة **مطبَّقة** — أساس تدرّج التسارع
        ramped = 0               # عدد الدورات التي قُيّد فيها الرفع
        last_ts = time.time()
        try:
            while covered < distance_m:
                if (time.time() - started) > MAX_CELL_TRAVEL_S:
                    aborted = "timeout"
                    break
                s = self.sensors()
                front_cm = s.get("ultrasonic_cm")
                if witness is not None:
                    witness.add(self._read_accel())
                d = self.reactive.decide(front_cm, s.get("ir_left", 1),
                                         s.get("ir_right", 1), s.get("ir_mid"))
                last_decision = d
                if d["action"] != "go":
                    # هل يفسّره جدار معروف من الخريطة؟
                    known_wall = False
                    if expected_wall_end_m is not None:
                        remaining = distance_m - covered
                        expected_now = expected_wall_end_m + remaining
                        if front_cm is None:
                            known_wall = expected_now <= IR_RANGE_CM / 100.0
                        else:
                            known_wall = (abs(front_cm / 100.0 - expected_now)
                                          <= KNOWN_WALL_TOL_M)
                    hard = (front_cm is not None and front_cm <= HARD_STOP_CM)
                    if known_wall and not hard:
                        power = crawl_power        # جدار متوقَّع → ازحف وأكمل
                    else:
                        aborted = d["action"]      # عائق مفاجئ → أجهض فوراً
                        break
                else:
                    power = min(d["speed"], MAX_MOTOR_POWER)

                # ⚠ سقف تثبيت الاتجاه إلزامي: عند 0.50 (أعلى درجة السلّم)
                #    الفراغ صفر فلا توجيه ممكن — انظر HEADING_HOLD_MAX_BASE.
                # ⚠ ويُسجَّل عند تقييده: السلّم يقرّر 0.50 والواجهة تعرضه، فلو
                #    طُبّق 0.40 صامتاً لعُرض رقم لم يحدث.
                if hold is not None and power > HEADING_HOLD_MAX_BASE:
                    base_capped = (power, HEADING_HOLD_MAX_BASE)
                    power = HEADING_HOLD_MAX_BASE

                # ── تنعيم التسارع: **الرفع فقط** ────────────────────
                # الخفض يمرّ فورياً (كبح لا يُؤجَّل — شرط سلامة). الرفع
                # يتدرّج لأن القفزة تُدير العجلات أسرع من قدرة الاحتكاك
                # فتنزلق، والانزلاق يفسد المسافة والاتجاه بلا إنكودرات
                # تكشفه. يُطبَّق **قبل** تكامل المسافة أدناه.
                step = SPEED_RAMP_UP_PER_S * REACTIVE_LOOP_S
                if power > applied + step:
                    power = applied + step
                    ramped += 1
                applied = power

                # ── القيادة: بتثبيت الاتجاه إن توفّر، وإلا قوة متساوية ──
                if hold is not None and src.ok:
                    src.update()
                    now = time.time()
                    dt = now - last_ts
                    last_ts = now
                    w = self.heading_ctl.wheels(signed_error(src.heading, hold),
                                                dt, base_power=power)
                    self.rover.motors(w["left"], w["right"])
                else:
                    if hold is not None and hold_lost is None:
                        # لا فشل صامت: يُبلَّغ مرة واحدة ونكمل بحلقة مفتوحة
                        # (عطل اتجاه لا يُسقط المهمة، لكنه لا يُخفى).
                        hold_lost = getattr(src, "error", "مصدر الاتجاه توقّف")
                    self.rover.forward(power)
                time.sleep(REACTIVE_LOOP_S)
                # المسافة تُتكامل من السرعة **المعايرة** للقوة المطبَّقة فعلاً.
                # ⚠ التصحيح لا يغيّرها: متوسط (يسار، يمين) = الأساس بالضبط
                #    لأن الوزنيتين متعاكستان و±corr متعاكسان.
                covered += self.profile.speed_for_power(power) * REACTIVE_LOOP_S
        finally:
            self.rover.stop()                      # ⚠ إيقاف مضمون
        out = {"ok": aborted is None, "covered_m": round(covered, 3),
               "aborted": aborted,
               "reason": (last_decision or {}).get("reason"),
               "elapsed_s": round(time.time() - started, 2)}
        # ── 🔴 الحكم: هل تحرّك فعلاً بالقدر المأمور؟ ─────────────────
        # ⚠ يُحسب على `covered` (ما أُمر به فعلاً قبل أي إجهاض) لا على
        #    `distance_m`: خطوة أُجهضت بعد 0.1م ليست «حركة ناقصة» بل أمر أقصر.
        if self.verify_motion_enabled:
            d_end = self._settled_front_cm()
            out["motion"] = verify_motion(max(covered, 0.0), d_start, d_end,
                                          witness)
        if hold is not None:
            s = self.heading_ctl.summary()
            out["heading_hold"] = {
                "mean_abs_error_deg": s["mean_abs_error_deg"],
                "max_abs_error_deg": s["max_abs_error_deg"],
                "saturated_pct": s["saturated_pct"],
                "samples": s["samples"], "lost": hold_lost,
                "base_capped": base_capped,
            }
        elif HEADING_HOLD_IN_MISSION:
            out["heading_hold"] = {"lost": "مصدر الاتجاه غير متاح عند بدء العبور"}
        self.log.append(out)
        return out

    # ── رجوع قصير محكوم (انسحاب / تراجع التدرّج) ─────────────────
    def backward_step(self, distance_m: float, power: float = None) -> dict:
        """
        شوط رجوع قصير — للانسحاب على أثر المسار ولتراجع تتبّع التدرّج.

        ⚠ **لا حسّاس خلفي في العتاد**: هذا الاتجاه بلا إشراف سلامة، فلا يُسمح
           به إلا على أرض عبرها الروبوت للتوّ وبمسافة مقصوصة عند
           `MOTION_BACKWARD_MAX_M`. هذا قيد عتاد لا تفضيل — والبديل (لفّة
           180° لتوجيه الحسّاس) يستهلك وقتاً وبطارية في أسوأ لحظة، ويفقد
           اتجاه الأثر الذي نتراجع عليه.
        ⚠ التحقق من الحركة يعمل هنا بإشارة معكوسة: المسافة الأمامية **تزداد**
           بالرجوع، فالانسحاب مقيس لا مفترض.
        """
        dist = min(abs(float(distance_m)), MOTION_BACKWARD_MAX_M)
        p = min(abs(power if power is not None else SPEED_LADDER[-1][1]),
                MAX_MOTOR_POWER)
        witness = AccelWitness() if self.verify_motion_enabled else None
        d_start = self._settled_front_cm() if self.verify_motion_enabled else None
        covered, started = 0.0, time.time()
        try:
            while covered < dist and (time.time() - started) <= MAX_CELL_TRAVEL_S:
                if witness is not None:
                    witness.add(self._read_accel())
                self.rover.backward(p)
                time.sleep(REACTIVE_LOOP_S)
                covered += self.profile.speed_for_power(p) * REACTIVE_LOOP_S
        finally:
            self.rover.stop()                      # ⚠ إيقاف مضمون
        out = {"ok": True, "covered_m": round(covered, 3), "aborted": None,
               "reason": "رجوع قصير (بلا إشراف خلفي)", "direction": -1,
               "elapsed_s": round(time.time() - started, 2)}
        if self.verify_motion_enabled:
            out["motion"] = verify_motion(max(covered, 0.0), d_start,
                                          self._settled_front_cm(), witness,
                                          direction=-1)
        self.log.append(out)
        return out

    # ── خطوة كاملة: لفّ + تقدّم + توقّف للقياس ───────────────────
    def step(self, current_heading: float, target_heading: float,
             distance_m: float = CELL_SIZE_M, dwell_s: float = 0.0,
             expected_wall_end_m=None) -> dict:
        t = self.turn_to(current_heading, target_heading)
        if not t["ok"]:
            self.rover.stop()
            return {"ok": False, "phase": "turn", "turn": t,
                    "reason": "انتهت مهلة اللفّ"}
        f = self.forward_cell(distance_m, expected_wall_end_m)
        if dwell_s > 0:
            time.sleep(dwell_s)                    # توقّف لتجميع عدّ إحصائي كافٍ
        return {"ok": f["ok"], "phase": "forward" if not f["ok"] else "done",
                "turn": t, "forward": f, "reason": f.get("reason")}

    # ── تصحيح الجدار عند الاقتراب (يُستدعى بعد الوصول) ───────────
    def maybe_wall_correct(self, dr, heading: float, front_cm, range_m: float = 0.8):
        """يحاول تصحيح الموقع بجدار مواجَه — الشرطان داخل DeadReckoning."""
        if front_cm is None or dr is None:
            return None
        measured_m = front_cm / 100.0
        if measured_m > range_m:
            return None
        # الشرط الأول (المحاذاة) يتحقق داخلياً أيضاً، لكن نتفاداه مبكراً
        if min(abs(_ang_signed(heading, a)) for a in (0.0, 90.0, 180.0, 270.0)) > WALL_ALIGN_TOL_DEG:
            return None
        return dr.try_front_wall_correction(measured_m)
