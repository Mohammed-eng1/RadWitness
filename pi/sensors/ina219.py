# -*- coding: utf-8 -*-
"""
ina219.py — قراءة جهد البطارية من INA219 على لوحة UPS Module 3S
================================================================
🔴 **عودة الحماية الجهدية**. كانت مراقبة الجهد معطّلة لأن الحقل `v` في رسالة
الروفر `T=130` لم يعد يُقرأ، فحلّ محلّها **حاجز زمني خام** لا يعرف حالة الشحن
الابتدائية. الآن الجهد يُقرأ من INA219 على لوحة الـUPS مباشرةً عبر I2C
الراسبري — **قناة مستقلة تماماً** عن I2C الروبوت المعطّل، فلا يُسقطها عطله.

طريقة القراءة (من ورقة بيانات INA219):

    سجل الإعدادات 0x00 ← 0x399F   (مدى 32V، دقة 12-بت، قياس مستمر)
    سجل جهد الناقل  0x02 → بايتان
    raw = (d[0] << 8) | d[1]
    الجهد = (raw >> 3) × 0.004 V     ← البتّات 15:3 هي القيمة

    البتّان الأدنيان **علَمان لا بيانات**:
        بت 1 = CNVR  جاهزية التحويل
        بت 0 = OVF   **تجاوز حسابي** ⇒ القيمة بلا معنى

⚠ **OVF يُرفض ولا يُقبل بصمت**: قراءة جهد خاطئة أخطر من غيابها — الغياب
   يُسقطنا على الحارس الزمني، أما رقم خاطئ فيُتَّخذ عليه قرار إيقاف أو
   استمرار. والصمت هنا هو بالضبط نمط الفشل الذي عولج في بقية المشروع.

⚠ ولا نغلق على CNVR: في وضع القياس المستمر يبقى مضبوطاً غالباً، والإغلاق
   عليه يرفض قراءات صحيحة. يُعرض للتشخيص فقط.
"""
from __future__ import annotations

import threading

from pi.config import (
    INA219_ADDR, INA219_I2C_BUS, INA219_CONFIG_VALUE, INA219_LSB_V,
    INA219_MIN_PLAUSIBLE_V, INA219_MAX_PLAUSIBLE_V,
    INA219_SHUNT_OHM, INA219_CURRENT_LSB_V, INA219_CURRENT_SIGN,
)

try:
    from smbus2 import SMBus
    _SMBUS_OK = True
except Exception:                      # noqa: BLE001 — ويندوز/بلا عتاد
    _SMBUS_OK = False

REG_CONFIG = 0x00
REG_SHUNT_VOLTAGE = 0x01
REG_BUS_VOLTAGE = 0x02


class INA219Reader:
    """
    قارئ جهد الناقل. **لا يرمي استثناءً للأعلى**: يُعيد جهداً أو `None`
    ومعه سبب مقروء — الطبقة الأعلى تقرّر ماذا تفعل بالغياب.
    """

    def __init__(self, addr: int = INA219_ADDR, bus_num: int = INA219_I2C_BUS):
        self.addr = int(addr)
        self.bus_num = int(bus_num)
        self.ok = False
        self.error = None
        self.last_v = None
        self.last_a = None
        self.last_raw = None
        self.ovf_count = 0            # كم قراءة رُفضت بالتجاوز
        self.reject_count = 0         # كم رُفضت لأي سبب (تشخيص التوصيل)
        self._bus = None
        self._lock = threading.Lock()

        if not _SMBUS_OK:
            self.error = "smbus2 غير مثبّتة (pip install smbus2)"
            return
        try:
            self._bus = SMBus(self.bus_num)
            # ⚠ الإعداد يُكتب **قبل** أول قراءة: بلا قياس مستمر يبقى السجل
            #    على قيمة قديمة أو صفر فتُقرأ بطارية «ميتة» وهي سليمة.
            self._bus.write_i2c_block_data(
                self.addr, REG_CONFIG,
                [(INA219_CONFIG_VALUE >> 8) & 0xFF, INA219_CONFIG_VALUE & 0xFF])
            self.ok = True
        except Exception as e:         # noqa: BLE001
            self.error = (f"تعذّر فتح INA219 على i2c-{self.bus_num} "
                          f"@ {hex(self.addr)}: {e} — تحقّق بـ"
                          f"`i2cdetect -y {self.bus_num}`")
            self._close_bus()

    def _close_bus(self) -> None:
        try:
            if self._bus is not None:
                self._bus.close()
        except Exception:              # noqa: BLE001
            pass
        self._bus = None

    # ── القراءة ──────────────────────────────────────────────────
    def read(self) -> dict:
        """
        يُعيد `{"v", "ok", "ovf", "cnvr", "raw", "reason"}`.
        `v = None` تعني **لا قراءة صالحة** — ولا يُقدَّم رقم بديل ولا آخر
        قيمة معروفة: تمرير قيمة قديمة كأنها حيّة هو نفس فخّ الألترا سونيك
        الذي عولج (جودة 0% ومسافة معروضة).
        """
        if not self.ok:
            return {"v": None, "ok": False, "ovf": None, "cnvr": None,
                    "raw": None, "reason": self.error or "غير مهيّأ"}
        try:
            with self._lock:
                d = self._bus.read_i2c_block_data(self.addr, REG_BUS_VOLTAGE, 2)
        except Exception as e:         # noqa: BLE001 — I2C عابر
            self.reject_count += 1
            return {"v": None, "ok": False, "ovf": None, "cnvr": None,
                    "raw": None, "reason": f"فشل قراءة I2C: {e}"}

        raw = ((d[0] << 8) | d[1]) & 0xFFFF
        self.last_raw = raw
        ovf = bool(raw & 0x0001)
        cnvr = bool(raw & 0x0002)
        if ovf:
            # 🔴 تجاوز حسابي: الرقم بلا معنى — يُرفض ولا يُقرَّب ولا يُقصّ
            self.ovf_count += 1
            self.reject_count += 1
            return {"v": None, "ok": False, "ovf": True, "cnvr": cnvr,
                    "raw": raw,
                    "reason": "🔴 بت التجاوز (OVF) مرتفع — القراءة مرفوضة"}

        v = (raw >> 3) * INA219_LSB_V
        # ⚠ حدّ معقولية: الراسبري نفسه يُغذَّى من هذه الحزمة عبر لوحة الـUPS،
        #   فقراءة دون الحدّ الأدنى **والشيفرة تعمل** خطأ قياس لا حالة بطارية
        #   (وقبولها يُطلق إيقافاً طارئاً كاذباً). وفوق الحدّ الأعلى يستحيل
        #   لحزمة 3S (12.6V ممتلئة) — عنوان خاطئ أو سجل خاطئ.
        if not (INA219_MIN_PLAUSIBLE_V <= v <= INA219_MAX_PLAUSIBLE_V):
            self.reject_count += 1
            return {"v": None, "ok": False, "ovf": False, "cnvr": cnvr,
                    "raw": raw,
                    "reason": (f"جهد غير معقول {v:.2f}V خارج "
                               f"{INA219_MIN_PLAUSIBLE_V:.0f}–"
                               f"{INA219_MAX_PLAUSIBLE_V:.0f}V — "
                               f"خطأ عنوان/سجل لا حالة بطارية")}
        self.last_v = v
        raw_a = self._read_amps()
        # الاتجاه الفيزيائي = الخام × إشارة اللوحة المقاسة (0 ⇒ مجهول)
        amps = (None if raw_a is None or not INA219_CURRENT_SIGN
                else raw_a * INA219_CURRENT_SIGN)
        self.last_a = amps if amps is not None else raw_a
        # 🔴 **لا يُحكَم بالشحن من هنا إطلاقاً** (مقاس 2026-08-11): الشنت على
        #    مسار الحِمل، فالتيار سحب الراسبري ولا يصير سالباً مهما كانت
        #    الحالة — وُصل الشاحن فقفز الجهد 180mV والتيار لم يتغيّر.
        #    الحكم من **ميل الجهد** في `battery.ChargeDetector`.
        return {"v": v, "ok": True, "ovf": False, "cnvr": cnvr, "raw": raw,
                "amps": amps,               # موقّع فقط إن عُوير (وهنا لا)
                "amps_raw": raw_a,          # الخام = **تيار الحِمل** (مقدار)
                "load_a": (abs(raw_a) if raw_a is not None else None),
                "watts": (round(v * abs(raw_a), 2) if raw_a is not None
                          else None),
                "reason": None}

    def _read_amps(self):
        """
        التيار **الخام** من سجل الشنت: خطوة 10µV ÷ مقاومة الشنت.

        🔴 مُوقَّع لكن **اتجاهه من اللوحة لا من المواصفة**: إشارة سجل الشنت
        تتبع تركيب VIN+/VIN− فيزيائياً، وتنقلب مع مراجعة اللوحة. الاتجاه
        الحقيقي يُضرب في `INA219_CURRENT_SIGN` **المقاس**، لا يُفترض هنا.
        (كان هذا السطر يقول «موجب = شحن» افتراضاً — والقياس ناقضه.)
        """
        try:
            with self._lock:
                d = self._bus.read_i2c_block_data(self.addr, REG_SHUNT_VOLTAGE, 2)
        except Exception:              # noqa: BLE001 — I2C عابر
            return None
        raw = (d[0] << 8) | d[1]
        if raw > 32767:                # مكمّل اثنين 16-بت
            raw -= 65536
        return (raw * INA219_CURRENT_LSB_V) / max(INA219_SHUNT_OHM, 1e-9)

    def voltage(self):
        """الجهد بالفولت أو `None` — الواجهة المختصرة للطبقات الأعلى."""
        return self.read()["v"]

    def state(self) -> dict:
        return {"ok": self.ok, "error": self.error, "addr": self.addr,
                "bus": self.bus_num, "shunt_ohm": INA219_SHUNT_OHM,
                "last_v": (round(self.last_v, 2)
                           if self.last_v is not None else None),
                "last_a": (round(self.last_a, 3)
                           if self.last_a is not None else None),
                "ovf_count": self.ovf_count, "reject_count": self.reject_count}

    def close(self) -> None:
        self._close_bus()
        self.ok = False


# ── قارئ مشترك: لا نفتح الناقل مرتين على نفس الشريحة ─────────────
_shared: INA219Reader | None = None
_shared_lock = threading.Lock()


def get_ina219() -> INA219Reader:
    global _shared
    with _shared_lock:
        if _shared is None:
            _shared = INA219Reader()
        return _shared
