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
    BATT_FULL_V, BATT_EMPTY_V, CALIB_VOLTAGE_DELTA_WARN,
)

# (الإجراء، الاسم، اللون)
ACTION_NONE, ACTION_WARN, ACTION_RTH, ACTION_STOP = "none", "warn", "rth", "stop"


def percent(v: float) -> int:
    """تقدير نسبة الشحن خطياً بين الفارغة والممتلئة (تقديري لا دقيق)."""
    if BATT_FULL_V <= BATT_EMPTY_V:
        return 0
    pct = (v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0
    return int(max(0.0, min(100.0, pct)))


def classify(v) -> dict:
    """يُصنّف الجهد ويحدّد الإجراء التلقائي الإلزامي."""
    if v is None:
        return {"level": "unknown", "action": ACTION_NONE, "color": "#8a93a6",
                "percent": 0, "v": None, "text": "جهد غير معروف"}
    if v < BATT_CRITICAL_V:
        return {"level": "critical", "action": ACTION_STOP, "color": "#dc2626",
                "percent": percent(v), "v": round(v, 2),
                "text": "حرج — إيقاف فوري"}
    if v < BATT_GOOD_V:
        # كل ما دون 10.8V (وفوق الحرج) = منخفض → RTH إجباري
        return {"level": "low", "action": ACTION_RTH, "color": "#ea580c",
                "percent": percent(v), "v": round(v, 2),
                "text": "منخفض — عودة إجبارية"}
    if v < BATT_EXCELLENT_V:
        return {"level": "good", "action": ACTION_WARN, "color": "#eab308",
                "percent": percent(v), "v": round(v, 2), "text": "جيد"}
    return {"level": "excellent", "action": ACTION_NONE, "color": "#22c55e",
            "percent": percent(v), "v": round(v, 2), "text": "ممتاز"}


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
