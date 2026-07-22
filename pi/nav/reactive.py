# -*- coding: utf-8 -*-
"""
reactive.py — البند 3: السلامة التفاعلية (مكتوبة الآن، **معطّلة**)
==================================================================
الطبقة الأدنى والأعلى أولوية: حلقة سريعة (≥10Hz) مستقلة عن أي تخطيط أو
رؤية أو LLM أو حتى واجهة. تعمل بلا إنترنت. لا يتحكم بها أي طبقة أعلى.

⚠ **معطّلة خلف `REACTIVE_SAFETY_ENABLED = False`** (config): تلمس عتاداً
   حقيقياً (ألترا سونيك + IR + محركات) وستُختبر في دفعة عتاد منفصلة. هنا:
   - منطق القرار `decide()` **خالص وقابل للاختبار في sim** بأي قيم حساسات.
   - حلقة العتاد `run_hardware_loop()` لا تعمل إلا إذا رُفع العلم.

المنطق (البند 3):
   - ألترا سونيك أمامي < ULTRASONIC_STOP_CM → توقف فوري.
   - IR يمين/يسار = عائق (0=عائق، منطق معكوس مثبت) → انعطاف مضاد + تعليم blocked.
   - تسلسل تحرر: رجوع، لفّ، محاولة بديلة — ESCAPE_MAX_ATTEMPTS محاولات ثم
     تعليم الخلية unreachable والانتقال للتالية.
   - مهلة heartbeat 1.5ث (تُطبَّق في جسر الروفر: ROVER_SAFETY_TIMEOUT_S).
"""
from __future__ import annotations

import time

from pi.config import (
    REACTIVE_SAFETY_ENABLED, ULTRASONIC_STOP_CM, REACTIVE_LOOP_HZ,
    ESCAPE_MAX_ATTEMPTS, ROVER_SAFETY_TIMEOUT_S,
)

# منطق IR المثبت على العتاد (من pi/tests/test_ir.py): 0 = عائق
IR_OBSTACLE_LEVEL = 0


class EscapeSequence:
    """تسلسل تحرر بعد التوقف: رجوع → لفّ → محاولة، حتى الحد ثم الاستسلام."""
    STEPS = ("backup", "turn", "forward_try")

    def __init__(self, max_attempts: int = ESCAPE_MAX_ATTEMPTS):
        self.max_attempts = max_attempts
        self.attempt = 0
        self._step = 0

    def next_maneuver(self) -> dict:
        if self.attempt >= self.max_attempts:
            return {"maneuver": "give_up"}       # علّم الخلية unreachable وتابع
        m = self.STEPS[self._step]
        self._step += 1
        if self._step >= len(self.STEPS):
            self._step = 0
            self.attempt += 1
        return {"maneuver": m, "attempt": self.attempt + 1}

    def reset(self) -> None:
        self.attempt = 0
        self._step = 0


class ReactiveSafety:
    def __init__(self, enabled: bool = REACTIVE_SAFETY_ENABLED,
                 stop_cm: float = ULTRASONIC_STOP_CM):
        self.enabled = enabled
        self.stop_cm = stop_cm
        self.escape = EscapeSequence()
        self._last_cmd_ts = time.time()

    def decide(self, front_cm, ir_left: int, ir_right: int) -> dict:
        """
        قرار خالص من قراءات الحساسات (قابل للاختبار في sim بأي قيم).
        الأولوية: توقف أمامي > انعطاف عن IR > مواصلة.
        """
        if front_cm is not None and front_cm < self.stop_cm:
            return {"action": "stop", "reason": "front_obstacle", "front_cm": front_cm}
        left_obstacle = (ir_left == IR_OBSTACLE_LEVEL)
        right_obstacle = (ir_right == IR_OBSTACLE_LEVEL)
        if left_obstacle and not right_obstacle:
            return {"action": "turn_right", "reason": "ir_left"}
        if right_obstacle and not left_obstacle:
            return {"action": "turn_left", "reason": "ir_right"}
        if left_obstacle and right_obstacle:
            return {"action": "stop", "reason": "ir_both"}
        return {"action": "go", "reason": "clear"}

    def heartbeat_expired(self) -> bool:
        """مهلة انقطاع الأوامر (تُطبَّق فعلياً في جسر الروفر)."""
        return (time.time() - self._last_cmd_ts) > ROVER_SAFETY_TIMEOUT_S

    def feed_heartbeat(self) -> None:
        self._last_cmd_ts = time.time()

    def run_hardware_loop(self, sensors, rover):   # pragma: no cover — عتاد
        """
        حلقة العتاد ≥10Hz — **لا تعمل إلا إذا REACTIVE_SAFETY_ENABLED=True**.
        تُختبر في دفعة العتاد المنفصلة، ليست ضمن اختبارات sim لهذه الدفعة.
        """
        if not self.enabled:
            return
        period = 1.0 / REACTIVE_LOOP_HZ
        while self.enabled:
            action = self.decide(sensors.front_cm(), sensors.ir_left(), sensors.ir_right())
            if action["action"] == "stop":
                rover.command("S", 0)
            time.sleep(period)
