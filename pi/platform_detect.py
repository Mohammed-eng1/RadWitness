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


def i2c_bus_check(bus_num: int = None) -> dict:
    """
    🔴 فحص وجود ناقل I2C المطلوب لمصدر الاتجاه — **عند الإقلاع**.

    السبب: وجود `/dev/i2c-4` معلَّق كلياً على سطر واحد في
    `/boot/firmware/config.txt`:

        dtoverlay=i2c4,pins_6_7,baudrate=100000

    وضياعه (إعادة تثبيت، تحديث يستبدل الملف، تعديل يدوي) **يُخفي حسّاس
    الاتجاه ويُسقط المصدر صامتاً** — وقد حدث فعلاً مرّتين. الفحص هنا يحوّل
    الفشل الصامت إلى رسالة صريحة عند الإقلاع.

    ⚠ على غير لينكس يُعاد `available=None` (غير منطبق) لا `False` — «غير
       منطبق» ليس «مفقود».
    """
    from pi.config import MPU6050_I2C_BUS
    n = MPU6050_I2C_BUS if bus_num is None else int(bus_num)
    path = f"/dev/i2c-{n}"
    if not HARDWARE_AVAILABLE:
        return {"bus": n, "path": path, "available": None,
                "message": "غير منطبق (بلا عتاد)"}
    ok = os.path.exists(path)
    return {
        "bus": n, "path": path, "available": ok,
        "message": (f"✅ {path} موجود" if ok else
                    f"🔴 **{path} مفقود** — مصدر الاتجاه (MPU-6050) لن يُقرأ. "
                    f"تحقّق من سطر `dtoverlay=i2c4,pins_6_7,baudrate=100000` "
                    f"في /boot/firmware/config.txt ثم أعد الإقلاع."),
    }


def banner() -> dict:
    """معلومات اللافتة للواجهة."""
    i2c = i2c_bus_check()
    msg = ("وضع المحاكاة — لا يوجد عتاد متصل" if SIMULATION_MODE
           else "عتاد متصل")
    if i2c["available"] is False:
        msg += " · " + i2c["message"]
    return {
        "simulation_mode": SIMULATION_MODE,
        "hardware_available": HARDWARE_AVAILABLE,
        "i2c_heading_bus": i2c,
        "message": msg,
    }
