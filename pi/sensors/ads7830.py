# -*- coding: utf-8 -*-
"""
ads7830.py — جهد **حزمة المحركات** (ADS7830 على لوح Freenove)
===============================================================
اكتُشف على العتاد 2026-09-12: `i2cdetect -y 1` أظهر **0x48** إلى جانب
PCA9685 — وهو مُحوّل Freenove التماثلي/الرقمي. قناته **2** موصولة بمقسّم
جهد على حزمة المحركات.

🔴 **لماذا يهمّ**: حزمة المحركات (2S ≈ 7.4V) كانت **بلا أي رقيب** —
Freenove لا يبثّ جهدها، وINA219 يقيس حزمة **الراسبري** لا المحركات
(§2.0.1). فكان حارس إنهاك اللفّ الرقيبَ الوحيد عليها. هذا يعيدها إلى
**طبقتين**.

⚠⚠ **ولا تُخلط الحزمتان أبداً**: عتبات 3S (10.0–12.6V) مطبَّقة على قراءة
2S تُطلق إيقافاً طارئاً كاذباً **دائماً** (ممتلئة 8.4V دون
`BATT_CRITICAL_V = 10.0`). لذلك هذا المصدر **منفصل تماماً** عن
`voltage()` وله عتباته وتصنيفه، ويُعلَن باسمه في الحالة (§6: المصدر
يُسجَّل دائماً، وإلا استحال تفسير أي إيقاف).

⚠ **الدقّة 8 بت**: 5.2V ÷ 255 ≈ 20mV لكل عدّة، ×معامل المقسّم ≈ 40mV على
الحزمة — عشرة أضعاف خشونة INA219. صالح حارساً، **لا** أساساً لإطفاء
منظَّم (§6 يشترط لذلك مصدراً حقيقياً دقيقاً).

⚠ ومعامل التحويل يتبع **إصدار لوح Freenove** — يُعلَن في config ولا
  يُستنتج (البند 6.1)، وقيمته الحالية موسومة «غير معايرة» حتى تُقارن
  بالملتيميتر.
"""
from __future__ import annotations

import logging

from pi.config import (ADS7830_I2C_BUS, ADS7830_ADDR, ADS7830_BATT_CHANNEL,
                       ADS7830_VREF_V, ADS7830_DIVIDER, MOTOR_BATT_MIN_V,
                       MOTOR_BATT_MAX_V)

logger = logging.getLogger(__name__)

_CMD_BASE = 0x84          # single-ended, تشغيل داخلي (ورقة البيانات)


class ADS7830:
    """قارئ جهد حزمة المحركات. `read()` **لا يرفع استثناءً**."""

    def __init__(self, bus: int = ADS7830_I2C_BUS, address: int = ADS7830_ADDR):
        self.bus_num, self.address = int(bus), int(address)
        self.ok, self.error = False, None
        self._bus = None
        try:
            import smbus2                    # كسول (البند 7)
            self._bus = smbus2.SMBus(self.bus_num)
            self._bus.read_byte(self.address)   # تحقّق وجود فعلي
            self.ok = True
        except Exception as e:               # noqa: BLE001
            self.error = (f"تعذّر فتح ADS7830 على i2c-{self.bus_num} "
                          f"@0x{self.address:02x}: {e}")

    def _read_raw(self, channel: int):
        """
        بايت مستقرّ من القناة. ⚠ **قراءتان متطابقتان** لا واحدة: المُحوّل
        يُخرج بايتاً انتقالياً أثناء التبديل بين القنوات، فقراءة واحدة قد
        تلتقط بقية القناة السابقة — خطأ **صامت** يبدو هبوط جهد.
        """
        cmd = _CMD_BASE | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
        self._bus.write_byte(self.address, cmd)
        prev = None
        for _ in range(4):                   # محاولات محدودة — لا حلقة مفتوحة
            v = self._bus.read_byte(self.address)
            if v == prev:
                return v
            prev = v
        return prev

    def read(self) -> dict:
        """
        `{"v": جهد أو None, "raw": البايت, "reason": سبب الغياب}`.
        🔴 `None` = **مجهول لا صفر** (البند 6.1): صفر يُقرأ «بطارية فارغة».
        """
        if not self.ok:
            return {"v": None, "raw": None, "reason": self.error}
        try:
            raw = self._read_raw(ADS7830_BATT_CHANNEL)
        except Exception as e:               # noqa: BLE001
            self.ok, self.error = False, str(e)
            return {"v": None, "raw": None, "reason": self.error}
        if raw is None:
            return {"v": None, "raw": None, "reason": "قراءة غير مستقرّة"}
        v = raw / 255.0 * ADS7830_VREF_V * ADS7830_DIVIDER
        # 🔴 خارج النطاق المعقول = **خطأ قياس لا حالة بطارية** (نفس قاعدة
        #    INA219 في §6): قبوله يُطلق إنذاراً كاذباً أو يُخفي خطراً حقيقياً.
        if not (MOTOR_BATT_MIN_V <= v <= MOTOR_BATT_MAX_V):
            return {"v": None, "raw": raw,
                    "reason": (f"{v:.2f}V خارج نطاق 2S المعقول "
                               f"({MOTOR_BATT_MIN_V}–{MOTOR_BATT_MAX_V}V) — "
                               f"خطأ قياس أو معامل مقسّم خاطئ")}
        return {"v": round(v, 2), "raw": raw, "reason": None}

    def state(self) -> dict:
        return {"ok": self.ok, "error": self.error,
                "addr": f"0x{self.address:02x}", "bus": self.bus_num,
                "channel": ADS7830_BATT_CHANNEL}

    def close(self) -> None:
        try:
            if self._bus is not None:
                self._bus.close()
        except Exception:                    # noqa: BLE001
            pass


_reader = None


def get_ads7830():
    """قارئ مشترك (يُهيَّأ مرة) — **أداة داخلية** يستدعيها الجسر."""
    global _reader
    if _reader is None:
        _reader = ADS7830()
        if not _reader.ok:
            logger.warning(_reader.error)
    return _reader
