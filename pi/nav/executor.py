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
    TURN_MIN_ACHIEVABLE_DEG, HEADING_STEER_PHASE_DEG,
)
from pi.nav.room import CELL_SIZE_M
from pi.nav.heading_hold import HeadingController, signed_error

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

    def __init__(self, rover, reactive, sensors, profile):
        self.rover = rover
        self.reactive = reactive
        self.sensors = sensors
        self.profile = profile
        self.log = []
        # **نفس المتحكّم لا نسخة ثانية** — ثوابته معايرة على العتاد.
        self.heading_ctl = HeadingController()

    # ── اللفّ نحو اتجاه مطلوب ────────────────────────────────────
    def turn_to(self, current_heading: float, target_heading: float) -> dict:
        delta = _ang_signed(current_heading, target_heading)
        # ⚠⚠ **لا تُطلب لفّة أصغر ممّا تستطيعه المنصّة** ⚠⚠
        # القصور الذاتي بعد قطع الطاقة 5.5–8.0° مهما صغرت اللفّة (قِيس: طلب
        # 2° أنتج 6.4°)، فطلب 3° لا يقرّب من الهدف بل يتجاوزه إلى الجهة
        # الأخرى — ويتكرّر عند كل خلية كرجفة بلا تقارب. الخطأ الصغير يعود
        # إلى `forward_cell` ليغلقه تثبيت الاتجاه **أثناء السير**: تصحيح
        # سلس، بلا قصور ذاتي، وبمتحكّم معاير أصلاً.
        if abs(delta) < TURN_MIN_ACHIEVABLE_DEG:
            return {"ok": True, "turned_deg": 0.0, "skipped": True,
                    "residual_deg": delta}
        res = self.rover.turn_by_angle(delta, timeout=ROVER_TURN_TIMEOUT_S)
        # `aborted` يعني عطلاً في مصدر الاتجاه أو إشارة محور مقلوبة — فشل
        # صريح لا يجوز اعتباره لفّة ناجحة (وإلا تقدّم الروبوت باتجاه خاطئ).
        return {"ok": not res["timed_out"] and not res.get("aborted"),
                "turned_deg": res["turned_deg"], "aborted": res.get("aborted"),
                "segments": res.get("segments", 1), "timed_out": res["timed_out"]}

    # ── التقدّم خلية واحدة تحت إشراف السلامة ─────────────────────
    def forward_cell(self, distance_m: float = CELL_SIZE_M,
                     expected_wall_end_m=None,
                     heading_error_deg: float = 0.0) -> dict:
        """
        يتقدّم `distance_m` بسرعة يقررها سلّم السلامة لحظياً.

        `heading_error_deg`: الخطأ الزاوي المتبقّي بعد اللفّ (موجب = يجب اللفّ
        يميناً للوصول إلى اتجاه الشبكة). **يُغلق أثناء السير** بدل لفّة بالمكان
        ثانية — لفّة أصغر من القصور الذاتي مستحيلة (TURN_MIN_ACHIEVABLE_DEG).

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

        # ── تثبيت الاتجاه: الهدف = الاتجاه الحالي + الخطأ المتبقّي ─────
        # يُلتقط من الحسّاس لا من إطار الغرفة: مرجع مصدر الاتجاه يُصفَّر
        # مستقلاً عن `mission.heading` فقد يكون الإطاران مزاحين. ولهذا يُجمع
        # **الفرق فقط** (`heading_error_deg`) لا الاتجاه المطلق — فيصير الهدف
        # صحيحاً في إطار الحسّاس أياً كان الإزاحة بين الإطارين.
        # ⚠ بلا هذا الجمع كان الشوط يثبّت الاتجاه **الخاطئ**: يمشي مستقيماً
        #   تماماً لكن على خط منحرف عن الشبكة، ويتراكم الانزياح خلية بعد خلية.
        src = getattr(self.rover, "heading_source", None)
        hold = None
        if HEADING_HOLD_IN_MISSION and src is not None and getattr(src, "ok", False):
            # ⚠ الطور يُختار **مرة واحدة في البداية** لا وسط الشوط: تبديله
            #   يصفّر المرشّح فيُحدث قفزة في أسوأ لحظة. شوطٌ يبدأ بخطأ حقيقي
            #   هو شوط **توجيه**: المتحكّم سيدير الروبوت عمداً بمعدل ترفضه
            #   عتبة السير الضيّقة (12°/ث) فيعمى الحسّاس عن دوران الروبوت نفسه.
            phase = ("steer" if abs(heading_error_deg) >= HEADING_STEER_PHASE_DEG
                     else "drive")
            src.set_phase(phase)
            src.update()             # يثبّت مرجع الزمن قبل أول تصحيح
            hold = (src.heading + float(heading_error_deg)) % 360.0
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
                d = self.reactive.decide(front_cm,
                                         s.get("ir_left", 1), s.get("ir_right", 1))
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
        if hold is not None:
            s = self.heading_ctl.summary()
            out["heading_hold"] = {
                "mean_abs_error_deg": s["mean_abs_error_deg"],
                "max_abs_error_deg": s["max_abs_error_deg"],
                "saturated_pct": s["saturated_pct"],
                "samples": s["samples"], "lost": hold_lost,
                "base_capped": base_capped,
                # الخطأ المتبقّي **مقاساً** عند نهاية الشوط — به تحدّث المهمة
                # اتجاهها بدل افتراض أن التثبيت أغلقه تماماً.
                "final_error_deg": s.get("final_error_deg"),
                "target_offset_deg": round(float(heading_error_deg), 2),
                "phase": phase,
                # إشباع مرتفع مع خطأ لا ينغلق = سلطة التصحيح لا تكفي
                # (ارفع HEADING_MAX_CORR أو أنزل قوة الأساس).
                "spikes": getattr(src.cond, "spikes", None),
            }
        elif HEADING_HOLD_IN_MISSION:
            out["heading_hold"] = {"lost": "مصدر الاتجاه غير متاح عند بدء العبور"}
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
