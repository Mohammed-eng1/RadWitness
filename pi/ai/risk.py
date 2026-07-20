# -*- coding: utf-8 -*-
"""
risk.py — التصنيف الخماسي لمستوى الخطر على µSv/h (M1: التصنيف فقط)
==================================================================
عتبات BRIEF الثابتة. كشف الشذوذ (Welford μ+3σ) والتراجع التلقائي عند
Critical طبقةُ سلامة تأتي في M2 — هنا نصنّف فقط لتغذية حقل risk في البث.
"""
from pi.config import RISK_LOW_USVH, RISK_MED_USVH, RISK_HIGH_USVH, RISK_CRIT_USVH

# (العتبة الدنيا، الاسم، المستوى، لون الواجهة) — مرتبة تنازلياً
_RISK_TABLE = [
    (RISK_CRIT_USVH, "Critical", 4, "#dc2626"),   # أحمر
    (RISK_HIGH_USVH, "High",     3, "#ea580c"),   # برتقالي محروق
    (RISK_MED_USVH,  "Medium",   2, "#d97706"),   # كهرماني
    (RISK_LOW_USVH,  "Low",      1, "#eab308"),   # أصفر
    (0.0,            "Safe",     0, "#22c55e"),   # أخضر
]


def classify(usvh: float) -> dict:
    """يُصنّف الجرعة (µSv/h) إلى مستوى خطر واسمه ولونه."""
    for threshold, name, level, color in _RISK_TABLE:
        if usvh >= threshold:
            return {"risk": name, "lvl": level, "color": color}
    return {"risk": "Safe", "lvl": 0, "color": "#22c55e"}
