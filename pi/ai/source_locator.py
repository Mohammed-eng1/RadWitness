# -*- coding: utf-8 -*-
"""
source_locator.py — وحدة بديلة: تحديد موقع المصدر **غير متاح** في هذه النسخة
============================================================================
تحفظ الواجهة التي تستدعيها المهمة (نفس الأسماء والتواقيع) كي يعمل المسح
والملاحة والخريطة وطبقة السلامة كاملةً. **لا حساب إطلاقاً**: لا تُخزَّن
القراءات، ولا اشتباه ولا تأكيد ولا موقع ولا أمر انسحاب — كل نداء يُعيد
«غير متاح» صراحةً.
"""
from __future__ import annotations

SURVEY, CONFIRM = "survey", "confirm"

# علم الطبقة: selftest يقرؤه فيتخطّى فحوص طبقة المصدر ويعدّها «متخطّى»
AVAILABLE = False

UNAVAILABLE_REASON = "تحديد موقع المصدر غير متاح في هذه النسخة"


class _NoDetector:
    """لا كاشف ⇒ لا منطقة مشتبهة أبداً."""
    suspect = None


class _NoProtocol:
    """لا بروتوكول مهمة ⇒ الحالة «غير متاح» لا «سليم»."""

    def status(self) -> dict:
        return {"available": False, "perimeter_start_ok": None,
                "reason": UNAVAILABLE_REASON}


class SourceLocator:
    """بديل بلا حساب: يقبل القراءات ولا يخزّنها ولا يستنتج منها شيئاً."""

    def __init__(self, width_m: float, length_m: float,
                 background_cpm: float = 0.0, **_ignored):
        self.width_m = float(width_m)
        self.length_m = float(length_m)
        self.background_cpm = float(background_cpm)
        self.readings: list = []          # يبقى فارغاً — لا قراءة تُخزَّن
        self.detector = _NoDetector()
        self.protocol = _NoProtocol()

    def set_background(self, mu_cpm: float, sigma_cpm: float = 0.0) -> None:
        self.background_cpm = float(mu_cpm)

    def set_vision_stats(self, stats: dict) -> None:
        return None

    def add_reading(self, *_args, **_kwargs) -> dict:
        return {"available": False, "reason": UNAVAILABLE_REASON}

    def screen(self) -> dict:
        return {"available": False, "suspect": False, "verdict": "unavailable",
                "lambda_stat": None, "threshold": None, "position": None,
                "reason": UNAVAILABLE_REASON}

    def hazard(self) -> dict:
        return {"available": False, "hazard_level": "unavailable",
                "withdraw": False, "alarm": None, "reason": UNAVAILABLE_REASON}

    def confirmation_plan(self, center=None, radius: float = None) -> list:
        return []

    def confirm_region(self, center=None, radius: float = None) -> dict:
        return {"available": False, "ready": False, "confirmed": False,
                "lambda_stat": None, "position": None, "uncertainty_m": None,
                "n_in_region": 0, "reason": UNAVAILABLE_REASON}

    def close_finding(self) -> dict:
        return {"ok": False, "available": False, "reason": UNAVAILABLE_REASON}

    def report(self) -> dict:
        return {"available": False, "found": False,
                "detection_state": "unavailable",
                "headline": UNAVAILABLE_REASON, "reason": UNAVAILABLE_REASON,
                "position": None, "position_reliable": False,
                "uncertainty_m": None, "confidence": 0.0,
                "hazard_level": "unavailable", "withdraw": False, "alarm": None,
                "n_readings": 0, "heatmap": None,
                "confirmed_findings": [], "primary_finding": None,
                "background_cpm": self.background_cpm}
