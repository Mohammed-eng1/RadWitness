# -*- coding: utf-8 -*-
"""
mpu6050.py — قارئ MPU-6050 (GY-521) — **مصدر الاتجاه بعد تلف BNO055**
======================================================================
BNO055 تعرّضت لـ5.40V وحدّها المطلق 3.6V، فانهارت مشغّلات خرجها وصارت تشدّ
SDA/SCL للأرضي وتشلّ الناقل بالكامل. البديل المركَّب: MPU-6050 على نفس
الناقل `/dev/i2c-4` عند العنوان `0x68`.

⚠ **ما لم يتغيّر — والالتباس هنا مكلف**: المشروع **لم يكن يستعمل مرجعاً
   مطلقاً أصلاً**. BNO055 كانت تعمل في وضع IMUPLUS (بلا مغنيتومتر) والاتجاه
   من **تكامل معدّل الجايرو** (CLAUDE.md §1 — البوصلة مرفوضة لأن الهيكل
   معدني وحوله محركات بمغانط). فالانحراف بلا حد كان قائماً قبل التبديل،
   و«صفّر الاتجاه» زرٌّ يدوي يضبط صفر الغرفة.

⚠ **ما تغيّر فعلاً هو جودة الحسّاس** — وهذا ما يستوجب إعادة معايرة لا إعادة
   تصميم:
     • معدل القراءة  45.3 Hz (BNO055) → **39.7 Hz** (مقاس)
     • أرضية الضجيج  σ=0.0702°/ث مقاسة → **غير معروفة** (تُقاس)
     • حالة المعايرة الذاتية `gyro_cal` 0..3 → **لا يوجد مؤشّر**
     • معامل التحويل مضبوط مصنعياً → **يُقاس** (÷131 لمدى ±250°/ث)

⚠ **طرف AD0 يجب ربطه بالأرضي**: يعمل معلّقاً لكن طرفاً عائماً في روبوت
   يهتزّ قد يقفز بالعنوان إلى 0x69 عشوائياً.

⚠ **الانحياز يُقاس عند كل إقلاع** — يتغيّر مع الحرارة ولا تُحفظ قيم
   (نفس قاعدة CLAUDE.md §1). المقاس مرجعياً: Z = −0.06 °/ث.

🔎 **تشخيص يستحق الحفظ**: إن ظهر الناقل ممتلئاً بكل العناوين 0x03–0x77 فهذه
   **ليست أجهزة** — بل خط SDA مشدود للأرضي بجهاز تالف. لا تعامله اكتشافاً.
   (الفرق: `pinctrl get 6,7` يعطي `lo lo` بدل `hi hi`.)
"""
from __future__ import annotations

import struct
import threading
import time

from pi.config import (
    MPU6050_ADDR, MPU6050_I2C_BUS, MPU6050_GYRO_LSB, MPU6050_ACCEL_LSB,
)

try:
    from smbus2 import SMBus
    _SMBUS_OK = True
except Exception:                     # noqa: BLE001 — ويندوز/بلا عتاد
    _SMBUS_OK = False

G_MPS2 = 9.80665                      # تحويل g → م/ث²

# سجلات MPU-6050 (من ورقة البيانات)
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_X = 0x3B
REG_TEMP = 0x41
REG_GYRO_X = 0x43
REG_WHO_AM_I = 0x75
WHO_AM_I_VAL = 0x68


class MPU6050Reader:
    """
    قارئ مباشر بالسجلات. **لا يرمي للأعلى**: يُعيد قيمة أو `None` ومعه سبب.

    القراءة بكتلة واحدة (6 بايت) لا ببايتين لكل محور: معاملة I2C واحدة أسرع
    و**متسقة زمنياً** (المحاور الثلاثة من نفس اللحظة، لا ثلاث لحظات متفرقة).
    """

    def __init__(self, addr: int = MPU6050_ADDR, bus_num: int = MPU6050_I2C_BUS):
        self.ok = False
        self.error = None
        self.addr = int(addr)
        self.bus_num = int(bus_num)
        self.who_am_i = None
        self._bus = None
        self._lock = threading.Lock()
        self._gyro = (0.0, 0.0, 0.0)
        self._accel = (0.0, 0.0, 0.0)
        self._temp_c = 0.0

        if not _SMBUS_OK:
            self.error = "smbus2 غير مثبّتة (pip install smbus2)"
            return
        try:
            self._bus = SMBus(self.bus_num)
            who = self._bus.read_byte_data(self.addr, REG_WHO_AM_I)
            self.who_am_i = who
            if who != WHO_AM_I_VAL:
                raise OSError(f"WHO_AM_I={hex(who)} ≠ {hex(WHO_AM_I_VAL)} — "
                              f"ليست MPU-6050")
            # ⚠ الإيقاظ إلزامي: الشريحة تُقلع في وضع السكون وتُرجع أصفاراً
            #    مضبوطة — وهي بالضبط بصمة «الحسّاس الميت» التي يكشفها حارس
            #    الصفر في مصدر الاتجاه، فتبدو المشكلة اتجاهاً وهي إيقاظ.
            self._bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0)
            self.ok = True
        except Exception as e:        # noqa: BLE001
            self.error = (f"تعذّر فتح MPU-6050 على i2c-{self.bus_num} "
                          f"@ {hex(self.addr)}: {e} — تحقّق بـ"
                          f"`i2cdetect -y {self.bus_num}` (توقّع 0x68)")
            self._close_bus()

    def recover(self) -> dict:
        """
        🔴 **اسأل الشريحة قبل إعلان الوفاة** (CLAUDE.md §1.1، بنسخة MPU).

        عطل مقاس (2026-08-10): المهمة أُجهضت بـ«40 قراءة صفر مضبوط — الحسّاس
        لا يرسل شيئاً (ميت؟)» بينما `i2cdetect -y 4` يُظهر 0x68 حاضراً. وهذا
        بالضبط مكافئ عودة BNO055 إلى CONFIG: هبوط جهد 3.3V لحظي عند اندفاع
        تيار المحركات يُعيد تشغيل الشريحة ذاتياً، فتُقلع في وضع **السكون**
        (بت SLEEP في PWR_MGMT_1) وتردّ بهويتها ويقرأ الجايرو صفراً مضبوطاً
        إلى الأبد. نفس بصمة الحسّاس الميت تماماً — وعلاجهما مختلف كلياً.

        ⚠ الإحياء **لا يُعلَن إلا بقراءة تحقّق**: نُوقظ ثم نقرأ WHO_AM_I
           وبت السكون فعلياً — نجاح الكتابة وحده ليس دليلاً (وهو الدرس
           نفسه المحفور في مسار BNO055).
        """
        if self._bus is None:
            return {"recovered": False, "detail": "لا ناقل مفتوح"}
        with self._lock:
            try:
                who = self._bus.read_byte_data(self.addr, REG_WHO_AM_I)
                pwr = self._bus.read_byte_data(self.addr, REG_PWR_MGMT_1)
                asleep = bool(pwr & 0x40)
                if who != WHO_AM_I_VAL:
                    return {"recovered": False,
                            "detail": f"WHO_AM_I={hex(who)} ≠ "
                                      f"{hex(WHO_AM_I_VAL)} — ليست MPU-6050"}
                # 🔴 **الإيقاظ يُعاد لا يُجرَّب مرة**، ومهلته أطول من 50ms.
                #    عطل مقاس 2026-08-11: المهمة أُجهضت بـ«الإيقاظ كُتب ولم
                #    يثبت (PWR_MGMT_1=0x40) — تغذية 3.3V غير مستقرة»، ثم
                #    أثبت الفحص اليدوي عكس ذلك تماماً: `i2cset 0x6b 0x00`
                #    نجح **وثبت بعد خمس ثوانٍ**، وستّ نبضات محركات مرّت بصفر
                #    أخطاء وσ=0.33°/ث. أي أن التغذية سليمة والمحاولة الواحدة
                #    القصيرة هي التي فشلت — وحكمُها **قاتل للمهمة**، فثمن
                #    تسرّعها إجهاض جولة كاملة.
                # ⚠ والمحاولة الثانية تبدأ بإعادة تعيين كاملة (0x80): أوصت
                #   بها ورقة البيانات، ولا تُفقدنا شيئاً — التهيئة كلها على
                #   القيم الافتراضية (لا سجل إعدادات آخر يُكتب في هذا الملف).
                pwr2, why = pwr, ""
                for attempt in range(3):
                    if attempt:
                        # إعادة تعيين الجهاز ثم إيقاظه من جديد
                        self._bus.write_byte_data(self.addr,
                                                  REG_PWR_MGMT_1, 0x80)
                        time.sleep(0.12)
                    self._bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0)
                    time.sleep(0.12 + 0.08 * attempt)
                    pwr2 = self._bus.read_byte_data(self.addr, REG_PWR_MGMT_1)
                    if not (pwr2 & 0x40):
                        why = ("" if attempt == 0
                               else f" (بعد {attempt + 1} محاولات وإعادة تعيين)")
                        break
                else:
                    return {"recovered": False,
                            "detail": f"الإيقاظ لم يثبت بعد 3 محاولات وإعادة "
                                      f"تعيين (PWR_MGMT_1={hex(pwr2)}) — "
                                      f"تغذية 3.3V غير مستقرة"}
                # 🔴 الدليل القاطع على الحياة: **الجاذبية**. بت السكون مرفوع
                #    عن الشريحة لا عن البيانات، ومقياس التسارع الحيّ يقرأ ~1g
                #    دائماً — فأصفار مضبوطة فيه تعني «لا بيانات» مهما قال
                #    السجل. (وهذا ما تفحصه أداة `check_imu_health` يدوياً.)
                live = None
                try:
                    d = self._bus.read_i2c_block_data(self.addr, REG_ACCEL_X, 6)
                    live = any(d[i] or d[i + 1] for i in (0, 2, 4))
                except Exception:         # noqa: BLE001 — I2C عابر
                    live = None           # تعذّرت القراءة: لا نفي ولا إثبات
                if live is False:
                    return {"recovered": False,
                            "detail": "استيقظت لكن مقياس التسارع يقرأ أصفاراً "
                                      "مضبوطة — لا جاذبية ⇒ لا بيانات فعلاً"}
                self.ok = True
                self.error = None
                return {"recovered": True,
                        "detail": ("أُوقظت من **السكون** (إعادة تشغيل ذاتية — "
                                   "هبوط جهد عند اندفاع المحركات)" if asleep
                                   else "كانت مستيقظة؛ أُعيدت التهيئة") + why}
            except Exception as e:    # noqa: BLE001
                return {"recovered": False,
                        "detail": f"الناقل لا يردّ: {e}"}

    def _close_bus(self) -> None:
        try:
            if self._bus is not None:
                self._bus.close()
        except Exception:             # noqa: BLE001
            pass
        self._bus = None

    @staticmethod
    def _s16be(hi: int, lo: int) -> int:
        return struct.unpack(">h", bytes([hi, lo]))[0]

    def _block(self, reg: int, n: int = 6):
        with self._lock:
            return self._bus.read_i2c_block_data(self.addr, reg, n)

    # ── القراءات ─────────────────────────────────────────────────
    def gyro_dps(self):
        """(gx, gy, gz) بالدرجة/ث أو None عند فشل قراءة عابر."""
        if not self.ok:
            return None
        try:
            d = self._block(REG_GYRO_X, 6)
        except Exception:             # noqa: BLE001 — I2C عابر
            return None
        v = tuple(self._s16be(d[i], d[i + 1]) / MPU6050_GYRO_LSB
                  for i in (0, 2, 4))
        self._gyro = v
        return v

    def gyro_z_dps(self):
        """معدل الدوران حول z — **أساس الاتجاه** (نفس عقد BNO055 السابق)."""
        g = self.gyro_dps()
        return None if g is None else g[2]

    def accel_mps2(self):
        """
        (ax, ay, az) م/ث² — لشاهد الحركة الثنائي (البند 0).
        ⚠ لا تُشتقّ منه مسافة أبداً: التكامل المزدوج ينجرف تربيعياً.
        """
        if not self.ok:
            return None
        try:
            d = self._block(REG_ACCEL_X, 6)
        except Exception:             # noqa: BLE001
            return None
        v = tuple(self._s16be(d[i], d[i + 1]) / MPU6050_ACCEL_LSB * G_MPS2
                  for i in (0, 2, 4))
        self._accel = v
        return v

    def temperature_c(self):
        if not self.ok:
            return None
        try:
            d = self._block(REG_TEMP, 2)
        except Exception:             # noqa: BLE001
            return None
        self._temp_c = self._s16be(d[0], d[1]) / 340.0 + 36.53
        return self._temp_c

    def read(self) -> None:
        """قراءة دورية للعرض (تُستدعى من حلقة السيرفر)."""
        if not self.ok:
            return
        self.gyro_dps()
        self.accel_mps2()
        self.temperature_c()

    def state(self) -> dict:
        gx, gy, gz = self._gyro
        return {
            "ok": self.ok, "error": self.error, "addr": self.addr,
            "bus": self.bus_num, "driver": "smbus2", "chip": "MPU-6050",
            "who_am_i": (hex(self.who_am_i) if self.who_am_i is not None else None),
            "mode": "جيرو+تسارع خام (6 محاور، بلا دمج داخلي)",
            # 🔴 لا مغنيتومتر ولا مرجع مطلق — والمشروع لم يكن يستعملهما أصلاً
            "mag_used": False,
            "gyro_z_dps": round(gz, 2),
            "gyro_dps": [round(v, 2) for v in self._gyro],
            "accel_mps2": [round(v, 2) for v in self._accel],
            "temp_c": round(self._temp_c, 1),
            # ⚠ لا مؤشّر معايرة ذاتية في MPU-6050 (خلاف BNO055) — الجاهزية
            #    تعني «استجاب» لا «معاير»، والمعايرة مسؤولية الطبقة الأعلى.
            "gyro_ready": self.ok,
            "self_calibration": False,
            "mag_warn": False,
        }

    def close(self) -> None:
        self._close_bus()
        self.ok = False


# ── قارئ مشترك: منع فتح I2C مرتين على نفس الشريحة ────────────────
_shared: MPU6050Reader | None = None
_shared_lock = threading.Lock()


def get_mpu() -> MPU6050Reader:
    global _shared
    with _shared_lock:
        if _shared is None:
            _shared = MPU6050Reader()
        return _shared
