# -*- coding: utf-8 -*-
"""
platform_detect.py — كشف المنصة تلقائياً (عتاد راسبري أم بيئة تطوير)
====================================================================
مكتبات العتاد (lgpio, adafruit...) لا تُثبَّت على ويندوز/ماك. نكشف وجودها
دون كسر التشغيل: عند غيابها يُجبَر وضع المحاكاة وتظهر لافتة واضحة في الواجهة.
لا استيراد عتاد على مستوى وحدات المنطق — الاستيراد هنا محمي وخفيف.
"""
import os

# lgpio مؤشر كافٍ لوجود عتاد الراسبري (كل تعامل GPIO يمرّ عبره)
HARDWARE_AVAILABLE = False
try:
    import lgpio  # noqa: F401
    HARDWARE_AVAILABLE = True
except Exception:                     # noqa: BLE001 — ويندوز/تطوير: طبيعي
    HARDWARE_AVAILABLE = False

# يمكن فرض المحاكاة يدوياً عبر متغيّر بيئة حتى على الراسبري
_FORCED = os.environ.get("RMS_SIMULATION", "").strip().lower() in ("1", "true", "yes")
SIMULATION_MODE = _FORCED or not HARDWARE_AVAILABLE


def banner() -> dict:
    """معلومات اللافتة للواجهة."""
    return {
        "simulation_mode": SIMULATION_MODE,
        "hardware_available": HARDWARE_AVAILABLE,
        "message": ("وضع المحاكاة — لا يوجد عتاد متصل"
                    if SIMULATION_MODE else "عتاد متصل"),
    }
