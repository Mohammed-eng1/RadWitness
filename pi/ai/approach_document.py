# -*- coding: utf-8 -*-
"""
approach_document.py — وحدة بديلة: الاقتراب من المصدر وتوثيقه **غير متاحين**
==========================================================================
تحفظ الأسماء التي تستوردها المهمة. الموانع تُعيد دائماً «غير متاح» فلا
اقتراب ولا تصوير، و`GradientApproach` يأمر بالتوقف من أول خطوة.
"""
from __future__ import annotations

import math

MOVE, STOP, WITHDRAW = "move", "stop", "withdraw"

UNAVAILABLE_REASON = "الاقتراب من المصدر وتوثيقه غير متاحين في هذه النسخة"


def approach_blockers(report: dict) -> list:
    return [UNAVAILABLE_REASON]


def documentation_blockers(report: dict) -> list:
    return [UNAVAILABLE_REASON]


def source_bearing_deg(robot_xy, source_xy) -> float:
    """
    زاوية نقطة من الروبوت في إطار الغرفة (0° = +y، يزداد يميناً — اصطلاح
    heading نفسه). هندسة بين نقطتين فقط، لا تقدير لأي مصدر.
    """
    dx = float(source_xy[0]) - float(robot_xy[0])
    dy = float(source_xy[1]) - float(robot_xy[1])
    return math.degrees(math.atan2(dx, dy)) % 360.0


class GradientApproach:
    """بديل: يأمر بالتوقف من أول خطوة ولا يحسب أي تدرّج."""

    def __init__(self, background_cpm: float = 0.0, step_m: float = 0.0,
                 max_steps: int = 1, **_ignored):
        self.max_steps = 1
        self.direction = 1
        self.n_steps = 0
        self.reversals = 0
        self.stopped = True

    def step(self, x: float, y: float, counts_L: float, counts_R: float = 0.0,
             **_ignored) -> dict:
        self.n_steps += 1
        return {"command": STOP, "cpm": None, "step_m": 0.0,
                "direction": None, "reason": UNAVAILABLE_REASON}


def run_documentation(locator_report: dict, robot_xy, robot_heading_deg: float,
                      camera=None, turn_fn=None, **_ignored) -> dict:
    return {"documented": False, "blockers": [UNAVAILABLE_REASON], "images": [],
            "statement": "🔴 لم يُنفَّذ التوثيق البصري: " + UNAVAILABLE_REASON}
