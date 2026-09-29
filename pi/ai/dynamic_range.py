# -*- coding: utf-8 -*-
"""
dynamic_range.py — وحدة بديلة: معالجة المدى الديناميكي **غير متاحة**
======================================================================
🔴 تحذير: الجرعة المعروضة في هذه النسخة **بلا تصحيح زمن ميت**.
المعدّل الخام يُحوَّل بمعامل المعايرة K وحده (`CPM_PER_USVH`) لأن تصنيف
الخطر في طبقة السلامة يحتاج رقماً. عند المعدّلات العالية يفقد أنبوب GM
نبضات، فتقلّ القراءة الخام عن الحقيقية — **الجرعة المعروضة أقل من الفعلية**
قرب مصدر قوي، وتصنيف الخطر المبني عليها أقل من الواجب.
"""
from __future__ import annotations

from pi.config import CPM_PER_USVH, RISK_CRIT_USVH

UNAVAILABLE_REASON = "معالجة المدى الديناميكي غير متاحة في هذه النسخة"


def dead_time_correct(cpm_meas: float, tau_s: float = None) -> dict:
    """
    🔴 **لا تصحيح**: تُعيد المعدّل الخام كما هو (`cpm_true = cpm_meas`،
    `corrected = False`) والجرعة = الخام ÷ K. أقل من الفعلية عند المعدّلات
    العالية.
    """
    m = max(float(cpm_meas), 0.0)
    return {"cpm_meas": m, "cpm_true": m, "corrected": False,
            "usvh": m / CPM_PER_USVH,
            "reason": "بلا تصحيح زمن ميت — " + UNAVAILABLE_REASON}


def recheck_needed(counts: float, duration_s: float,
                   background_cpm: float = 0.0, **_ignored) -> dict:
    return {"recheck": False, "reason": UNAVAILABLE_REASON}


def approach_stop_cpm(max_dose_usvh: float = RISK_CRIT_USVH,
                      **_ignored) -> dict:
    """حدّ التوقف = عتبة Critical في تصنيف الخطر، محوَّلة بـK وحده."""
    stop = float(max_dose_usvh) * CPM_PER_USVH
    return {"stop_cpm": stop, "stop_usvh": float(max_dose_usvh),
            "available": False, "reason": UNAVAILABLE_REASON}
