# -*- coding: utf-8 -*-
"""
battery.py — تصنيف جهد البطارية وحمايتها (البند أ-3)
====================================================
بطارية 3S: اسمي 11.1V، ممتلئة 12.6V. الجهد يُقرأ من الحقل `v` في رسالة
`T=130` باستمرار. سُجّل أثناء الاختبار هبوط إلى 8.98V — هذه الحماية تمنع تكراره.

| الجهد | الحالة | الإجراء التلقائي |
|---|---|---|
| > 11.5V      | ممتاز  | تشغيل طبيعي |
| 10.8 – 11.5V | جيد    | تنبيه خفيف |
| 10.2 – 10.8V | منخفض  | **RTH إجباري** |
| < 10.0V      | حرج    | **إيقاف المحركات فوراً** + إنذار |
"""
from pi.config import (
    BATT_EXCELLENT_V, BATT_GOOD_V, BATT_LOW_V, BATT_CRITICAL_V,
    BATT_FULL_V, BATT_EMPTY_V, BATT_CELLS, CALIB_VOLTAGE_DELTA_WARN,
    BATT_CURVE, BATT_R_INTERNAL_OHM, BATT_SHUTDOWN_V,
)

# (الإجراء، الاسم، اللون)
ACTION_NONE, ACTION_WARN, ACTION_RTH, ACTION_STOP = "none", "warn", "rth", "stop"
# 🔴 إطفاء نظام التشغيل — **تحت** إيقاف المحركات لا بدلاً منه
ACTION_SHUTDOWN = "shutdown"


def percent(v: float, amps: float = None) -> int:
    """
    نسبة الشحن **باستيفاء منحنى** لا بمعادلة خطية.

    ⚠ منحنى تفريغ الليثيوم مسطّح في وسطه وحادّ عند طرفيه، فالخطّية تبالغ في
    المنتصف وتضلّل عند الأطراف. والجهد **ينخفض تحت الحمل** (0.3–0.5V مقاسة
    عند تشغيل المحركات) فيُعوَّض بمقاومة الحزمة: V_مفتوح ≈ V + |I|×R.
    ⚠ تبقى **تقديرية** ويُتَّخذ القرار على **الجهد** لا عليها.
    """
    vv = float(v)
    if amps:
        vv += abs(float(amps)) * BATT_R_INTERNAL_OHM
    if vv >= BATT_CURVE[0][0]:
        return 100
    if vv <= BATT_CURVE[-1][0]:
        return 0
    for (v1, p1), (v2, p2) in zip(BATT_CURVE, BATT_CURVE[1:]):
        if v2 <= vv <= v1:
            return int(round(p2 + (vv - v2) * (p1 - p2) / (v1 - v2)))
    return 0


def cell_voltage(v: float) -> float:
    """جهد الخلية الواحدة — المؤشّر الأصدق على صحة حزمة ليثيوم 3S."""
    return v / max(1, BATT_CELLS)


def classify(v, amps: float = None) -> dict:
    """يُصنّف الجهد ويحدّد الإجراء التلقائي الإلزامي."""
    if v is None:
        return {"level": "unknown", "action": ACTION_NONE, "color": "#8a93a6",
                "percent": 0, "v": None, "cell_v": None, "amps": None,
                "charging": False, "cells": BATT_CELLS,
                "text": "جهد غير معروف"}
    common = {"percent": percent(v, amps), "v": round(v, 2),
              "cell_v": round(cell_voltage(v), 2), "cells": BATT_CELLS,
              "amps": (round(amps, 3) if amps is not None else None)}
    # 🔴 الطبقة الأخيرة: تحت هذا الحدّ يقترب قطع الحماية (~8.4V) الذي يقطع
    #    التغذية فجأةً — وانقطاعها أثناء الكتابة على البطاقة يفسدها.
    #    ⚠ الإجراء نفسه مشروط بـ5 قراءات متتالية وبعدم الشحن — في mission.
    if v < BATT_SHUTDOWN_V:
        return {"level": "shutdown", "action": ACTION_SHUTDOWN,
                "color": "#7f1d1d",
                "text": "🔴 حرج جداً — إطفاء منظَّم", **common}
    if v < BATT_CRITICAL_V:
        return {"level": "critical", "action": ACTION_STOP, "color": "#dc2626",
                "text": "حرج — إيقاف فوري", **common}
    if v < BATT_GOOD_V:
        # كل ما دون 10.8V (وفوق الحرج) = منخفض → RTH إجباري
        return {"level": "low", "action": ACTION_RTH, "color": "#ea580c",
                "text": "منخفض — عودة إجبارية", **common}
    if v < BATT_EXCELLENT_V:
        return {"level": "good", "action": ACTION_WARN, "color": "#eab308",
                "text": "جيد", **common}
    return {"level": "excellent", "action": ACTION_NONE, "color": "#22c55e",
            "text": "ممتاز", **common}


def calibration_voltage_warning(calib_v, current_v):
    """
    تحذير صلاحية المعايرة: أرقام السرعة تتأثر بالجهد. فارق > 1V بين لحظة
    المعايرة ولحظة التشغيل → المعايرة قد تكون غير دقيقة (البند أ-5).
    """
    if not calib_v or current_v is None:
        return None
    delta = abs(current_v - calib_v)
    if delta > CALIB_VOLTAGE_DELTA_WARN:
        return (f"⚠ المعايرة قد تكون غير دقيقة — فارق الجهد {delta:.1f}V "
                f"(معايرة عند {calib_v:.1f}V، التشغيل عند {current_v:.1f}V)")
    return None
