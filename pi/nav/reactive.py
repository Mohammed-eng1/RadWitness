# -*- coding: utf-8 -*-
"""
reactive.py — طبقة السلامة والتفادي التفاعلية (البنود 1-5 مطبَّقة)
==================================================================
الطبقة الأدنى والأعلى أولوية: حلقة سريعة (`REACTIVE_LOOP_S` = 0.08s) مستقلة
عن أي تخطيط أو رؤية أو LLM أو واجهة. تعمل بلا إنترنت.

⚠ **معطّلة خلف `REACTIVE_SAFETY_ENABLED = False`** (البند 6 يرفعها للاختبار
   على العتاد). منطق القرار هنا **خالص وقابل للاختبار في sim** بأي قيم.

────────────────────────────────────────────────────────────────────
البند 1 — تقسيم أدوار الحساسات (**تكامل لا تكرار**):

| الحساس | المدى/التغطية | الدور |
|---|---|---|
| ألترا سونيك (TRIG 23 / ECHO 24) | 2-400 سم، شعاع ضيق ~15° | **التخطيط المبكر**: يحدد السرعة ويقرر الانحراف قبل الوصول |
| IR أمامي أيسر (BCM25) | 2-30 سم، زاوية أوسع | **خط الدفاع الأخير** + تغطية المنطقة العمياء |
| IR أمامي أيمن (BCM16) | نفسه | نفسه (**0 = عائق**) |

⚠ **الألترا سونيك يفشل** مع:
  - **الأسطح المائلة** — الموجة ترتدّ بعيداً فلا يعود صدى (يبدو الطريق خالياً).
  - **المواد الماصّة** (قماش، إسفنج، ستائر) — تبتلع الموجة.
  - **الأجسام المنخفضة أو خارج الشعاع الضيق** (~15°) — لا يراها أصلاً.
ولهذا وجود IR **إلزامي وليس احتياطياً**: هو ما يمسك ما يعميه الألترا سونيك.
────────────────────────────────────────────────────────────────────
"""
from __future__ import annotations

import time

from pi.config import (
    REACTIVE_SAFETY_ENABLED, REACTIVE_LOOP_S, ESCAPE_MAX_ATTEMPTS,
    ROVER_SAFETY_TIMEOUT_S, STOP_CM, SPEED_LADDER, SPEED_NO_READING,
    DISTANCE_SAMPLES, MAX_JUMP_CM, SMART_AVOID_ENABLED, SCAN_ANGLE_DEG,
    BACK_TIME_S, BLOCKED_BOTH_CM, AVOID_TURN_DEG, IR_OBSTACLE_LEVEL,
)


# ═══ البند 5: تصفية القراءة ══════════════════════════════════════
def median(samples) -> float | None:
    """وسيط القيم الصالحة (لا متوسط — الوسيط يلغي القيمة الشاذّة المفردة)."""
    vals = sorted(v for v in samples if v is not None)
    if not vals:
        return None
    return vals[len(vals) // 2]


class JumpFilter:
    """
    يتجاهل أي قراءة تختلف عن سابقتها بأكثر من MAX_JUMP_CM **إلا إذا تكررت
    مرتين** — يمنع «الأشباح» (صدى شارد) دون أن يعمي النظام عن تغيّر حقيقي.
    """

    def __init__(self, max_jump_cm: float = MAX_JUMP_CM):
        self.max_jump = max_jump_cm
        self.last = None
        self._pending = 0

    def feed(self, value):
        if value is None:
            return self.last
        if self.last is None:
            self.last = value
            return value
        if abs(value - self.last) > self.max_jump:
            self._pending += 1
            if self._pending >= 2:          # تكررت → قفزة حقيقية، اقبلها
                self.last = value
                self._pending = 0
                return value
            return self.last                # شبح — أبقِ السابقة
        self._pending = 0
        self.last = value
        return value


# ═══ البند 2: السرعة المتدرّجة ═══════════════════════════════════
def speed_for_distance(front_cm) -> dict:
    """يحوّل المسافة الأمامية إلى سرعة + درجة السلّم وسببها (للبثّ والسجل)."""
    if front_cm is None:
        return {"speed": SPEED_NO_READING, "rung": "no_reading",
                "reason": "لا قراءة ألترا سونيك — احترس ولا تقف"}
    if front_cm < STOP_CM:
        return {"speed": 0.0, "rung": "stop",
                "reason": f"عائق أمامي {front_cm:.0f}سم < {STOP_CM:.0f}سم"}
    for threshold, spd in SPEED_LADDER:     # مرتبة تنازلياً
        if front_cm >= threshold:
            return {"speed": spd, "rung": f"≥{threshold:.0f}سم",
                    "reason": f"مسافة {front_cm:.0f}سم"}
    slowest = SPEED_LADDER[-1][1]
    return {"speed": slowest, "rung": "زحف",
            "reason": f"مسافة {front_cm:.0f}سم"}


# ═══ تسلسل التحرر (يُستخدم عند الانحشار) ═════════════════════════
class EscapeSequence:
    STEPS = ("backup", "turn", "forward_try")

    def __init__(self, max_attempts: int = ESCAPE_MAX_ATTEMPTS):
        self.max_attempts = max_attempts
        self.attempt = 0
        self._step = 0

    def next_maneuver(self) -> dict:
        if self.attempt >= self.max_attempts:
            return {"maneuver": "give_up"}
        m = self.STEPS[self._step]
        self._step += 1
        if self._step >= len(self.STEPS):
            self._step = 0
            self.attempt += 1
        return {"maneuver": m, "attempt": self.attempt + 1}

    def reset(self) -> None:
        self.attempt = 0
        self._step = 0


# ═══ الطبقة التفاعلية ════════════════════════════════════════════
class ReactiveSafety:
    def __init__(self, enabled: bool = REACTIVE_SAFETY_ENABLED,
                 stop_cm: float = STOP_CM,
                 smart_avoid: bool = SMART_AVOID_ENABLED):
        self.enabled = enabled
        self.stop_cm = stop_cm
        self.smart_avoid_enabled = smart_avoid
        self.escape = EscapeSequence()
        self.jump = JumpFilter()
        self._last_cmd_ts = time.time()
        self.events = []                    # سجل القرارات (يُبثّ في البند 6)

    # ── قراءة مسافة مصفّاة (وسيط 3 + تصفية القفزات) ─────────────
    def read_distance(self, sampler, filtered: bool = True) -> float | None:
        """
        sampler(): دالة تُعيد مسافة خام بالسنتيمتر أو None.
        `filtered=True` يطبّق تصفية القفزات — وهي تفترض **استمرارية زمنية**
        (قراءات متتابعة والروبوت يسير في اتجاه واحد). أثناء **المسح الدوراني**
        نوجّه الجسم عمداً إلى جهات مختلفة، فالفروق الكبيرة حقيقية لا أشباح →
        نستخدم الوسيط فقط (`filtered=False`) وإلا رفض الفلتر قراءةً صحيحة.
        """
        raw = [sampler() for _ in range(DISTANCE_SAMPLES)]
        med = median(raw)
        return self.jump.feed(med) if filtered else med

    # ── البندان 2+4: القرار اللحظي ──────────────────────────────
    def decide(self, front_cm, ir_left: int, ir_right: int) -> dict:
        """
        أولوية مطلقة لـIR (خط الدفاع الأخير — الوقت لا يسمح بمسح دوراني)،
        ثم الألترا سونيك (توقف + مسح دوراني)، وإلا سِر بسرعة السلّم.
        """
        left_obs = (ir_left == IR_OBSTACLE_LEVEL)
        right_obs = (ir_right == IR_OBSTACLE_LEVEL)

        # (البند 4) IR في الركنين الأماميين — استجابة فورية بلا مسح
        if left_obs and right_obs:
            return {"action": "backup_turn", "speed": 0.0, "priority": "ir",
                    "rung": "ir_both", "turn_deg": 90.0,
                    "reason": "عائق عريض على IR الجهتين → رجوع + لفّ 90°"}
        if left_obs:
            return {"action": "turn_right", "speed": 0.0, "priority": "ir",
                    "rung": "ir_left", "reason": "IR أمام-يسار → لفّ يميناً"}
        if right_obs:
            return {"action": "turn_left", "speed": 0.0, "priority": "ir",
                    "rung": "ir_right", "reason": "IR أمام-يمين → لفّ يساراً"}

        # (البند 2) الألترا سونيك: سرعة متدرّجة أو توقف
        lad = speed_for_distance(front_cm)
        if lad["rung"] == "stop":
            return {"action": "stop", "speed": 0.0, "priority": "ultrasonic",
                    "rung": "stop", "reason": lad["reason"],
                    # (البند 3) الخطوة التالية بعد التوقف
                    "next": "smart_avoid" if self.smart_avoid_enabled else "simple"}
        return {"action": "go", "speed": lad["speed"], "priority": "clear",
                "rung": lad["rung"], "reason": lad["reason"]}

    # ── البند 3: المسح الدوراني (الحساس ثابت والجسم يدور) ───────
    def smart_avoid(self, rover, sampler) -> dict:
        """
        لا لفّ أعمى: توقف → رجوع قصير → انظر 30° يميناً ويساراً → اتجه نحو
        **الأبعد**. لو الجهتان مسدودتان → لفّ 180° وإعادة تخطيط عبر planner.
        التكلفة ~2ث لكل عائق، والمكسب منع الانحشار في زوايا الغرف الضيقة.
        """
        if not self.smart_avoid_enabled:
            rover.stop()
            return {"decision": "simple_stop", "smart": False,
                    "reason": "المسح الذكي معطّل — منطق بسيط"}
        try:
            rover.stop()
            rover.backward()
            time.sleep(BACK_TIME_S)
            rover.stop()

            rover.turn_by_angle(SCAN_ANGLE_DEG)            # انظر يميناً
            right_cm = self.read_distance(sampler, filtered=False)
            rover.turn_by_angle(-SCAN_ANGLE_DEG)           # عُد للوضع الأصلي

            rover.turn_by_angle(-SCAN_ANGLE_DEG)           # انظر يساراً
            left_cm = self.read_distance(sampler, filtered=False)
            rover.turn_by_angle(SCAN_ANGLE_DEG)            # عُد للوضع الأصلي

            r = right_cm if right_cm is not None else -1.0
            l = left_cm if left_cm is not None else -1.0

            if r < BLOCKED_BOTH_CM and l < BLOCKED_BOTH_CM:
                rover.turn_by_angle(180.0)
                out = {"decision": "replan_180", "smart": True,
                       "right_cm": right_cm, "left_cm": left_cm,
                       "reason": f"الجهتان مسدودتان (يمين {r:.0f} / يسار {l:.0f}سم)"
                                 f" → لفّ 180° وإعادة تخطيط"}
            elif r >= l:
                rover.turn_by_angle(AVOID_TURN_DEG)
                out = {"decision": "turn_right", "smart": True,
                       "right_cm": right_cm, "left_cm": left_cm,
                       "reason": f"اليمين أبعد ({r:.0f} مقابل {l:.0f}سم) → لفّ يميناً"}
            else:
                rover.turn_by_angle(-AVOID_TURN_DEG)
                out = {"decision": "turn_left", "smart": True,
                       "right_cm": right_cm, "left_cm": left_cm,
                       "reason": f"اليسار أبعد ({l:.0f} مقابل {r:.0f}سم) → لفّ يساراً"}
            self.events.append(out)
            return out
        finally:
            rover.stop()                     # ⚠ إيقاف مضمون مهما حدث

    # ── السلامة العامة ──────────────────────────────────────────
    def feed_heartbeat(self) -> None:
        self._last_cmd_ts = time.time()

    def heartbeat_expired(self) -> bool:
        return (time.time() - self._last_cmd_ts) > ROVER_SAFETY_TIMEOUT_S

    def run_hardware_loop(self, sensors, rover):   # pragma: no cover — عتاد
        """
        حلقة العتاد كل REACTIVE_LOOP_S — **لا تعمل إلا إذا رُفع العلم** (البند 6).
        لها أولوية مطلقة على أي أمر من الملاحة/الرؤية/الكابتن.
        """
        if not self.enabled:
            return
        try:
            while self.enabled:
                front = self.read_distance(sensors.front_cm)
                d = self.decide(front, sensors.ir_left(), sensors.ir_right())
                act = d["action"]
                if act == "go":
                    rover.forward(d["speed"])
                elif act == "turn_right":
                    rover.turn("R")
                elif act == "turn_left":
                    rover.turn("L")
                elif act == "backup_turn":
                    rover.backward(); time.sleep(BACK_TIME_S)
                    rover.turn_by_angle(d.get("turn_deg", 90.0))
                elif act == "stop":
                    rover.stop()
                    if d.get("next") == "smart_avoid":
                        self.smart_avoid(rover, sensors.front_cm)
                if self.heartbeat_expired():
                    rover.stop()
                time.sleep(REACTIVE_LOOP_S)
        finally:
            rover.stop()                     # ⚠ أي استثناء → إيقاف
