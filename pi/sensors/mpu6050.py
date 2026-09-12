# -*- coding: utf-8 -*-
"""
mpu6050.py — قارئ عائلة MPU (6050 / 6500 / 9250) — **مصدر الاتجاه**
======================================================================
BNO055 تعرّضت لـ5.40V وحدّها المطلق 3.6V، فانهارت مشغّلات خرجها وصارت تشدّ
SDA/SCL للأرضي وتشلّ الناقل بالكامل. البديل: وحدة من عائلة MPU على نفس
الناقل `/dev/i2c-4` عند العنوان `0x68`.

🔴 **الوحدة المركَّبة حالياً: MPU-6500** (‏WHO_AM_I = `0x70`، مقاس
   2026-08-12). واسم الملف تاريخي — المنطق واحد للعائلة كلها.

⚠ **العائلة تُقبل بمعرّفاتها لا بمعرّف واحد**: خريطة السجلات ومعاملات
   التحويل متطابقة (÷131 جايرو · ÷16384 تسارع)، ويتغيّر `WHO_AM_I` وحده.
   وقصرُ القبول على `0x68` كان يرفض وحدة **سليمة** ويُعلنها «ليست
   MPU-6050» — عطل تشخيصي كامل من ثابت واحد.

🔎 **وكيف يُميَّز المقلَّد من السليم** (الفحص الذي حسم استبدال الوحدة
   السابقة): اكتب `0x07` في سجل `0x19` (SMPLRT_DIV) ثم اقرأه. السليمة
   تُعيد `0x07`؛ والمقلَّدة تُعيد `0x00` — تردّ بهوية صحيحة ولا تحتفظ
   بالكتابة. وهذه بالضبط بصمة «الإيقاظ كُتب ولم يثبت» التي عذّبتنا:
   الشريحة كانت **مقلَّدة** لا التغذية منهارة.

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
#: 🔴 **سجل يُنسى فيَقتل مقياس التسارع صامتاً**: كل بت فيه يُطفئ محوراً
#  (‏DIS_XA..DIS_ZG). قيمته الافتراضية 0x00 نظرياً، لكن MPU-6500/9250 قد
#  تُقلع بمحاور معطّلة بعد إعادة تشغيل ذاتية أو على وحدة مضروبة الإعدادات
#  ⇒ الجايرو يعمل ومقياس التسارع يقرأ **أصفاراً مضبوطة**. وهي بالضبط بصمة
#  «الحسّاس ميت» — وقياس 2026-09-12 على العتاد أظهرها: تكامل جايرو 91.7°
#  سليم مع `التسارع ساكناً = 0.000 g`. فيُكتب صراحةً **ويُقرأ رجعياً**.
REG_PWR_MGMT_2 = 0x6C
REG_ACCEL_X = 0x3B
REG_TEMP = 0x41
REG_GYRO_X = 0x43
REG_WHO_AM_I = 0x75

#: إعادة محاولة قراءة الناقل — عثرة واحدة ليست عطل حسّاس (انظر `_block`).
I2C_READ_RETRIES = 2          # محاولتان إضافيتان بعد الأولى
I2C_RETRY_DELAY_S = 0.004     # 4ms — أطول من معاملة كاملة عند 100kHz

#: 🔴 **عائلة كاملة لا شريحة واحدة**: نفس خريطة السجلات ونفس معاملات
#  التحويل (÷131 جايرو · ÷16384 تسارع) — يتغيّر المعرّف وحده. قصرُ القبول
#  على 0x68 كان يرفض وحدة **سليمة** ويُعلنها «ليست MPU-6050».
#  ⚠ مقاس 2026-08-12: الوحدة المركّبة **MPU-6500** (0x70).
WHO_AM_I_NAMES = {
    0x68: "MPU-6050",
    0x70: "MPU-6500",
    0x71: "MPU-9250",
    0x73: "MPU-9255",
}
WHO_AM_I_VAL = 0x68        # يبقى للتوافق مع أدوات التشخيص القديمة


def chip_name(who) -> str:
    """اسم الشريحة من معرّفها — أو المعرّف الخام إن كان مجهولاً."""
    return WHO_AM_I_NAMES.get(who, f"غير معروف ({hex(who)})"
                              if who is not None else "غير مقروء")


class MPU6050Reader:
    """
    قارئ مباشر بالسجلات. **لا يرمي للأعلى**: يُعيد قيمة أو `None` ومعه سبب.

    القراءة بكتلة واحدة (6 بايت) لا ببايتين لكل محور: معاملة I2C واحدة أسرع
    و**متسقة زمنياً** (المحاور الثلاثة من نفس اللحظة، لا ثلاث لحظات متفرقة).
    """

    #: قيم افتراضية على مستوى الصنف — لا على النسخة وحدها: قوالب الاختبار
    #  تتجاوز `__init__` بـ`__new__`، فحقلٌ يُعرَّف في `__init__` وحده يُسقط
    #  `state()` بـ`AttributeError` بدل أن يقول «غير مفحوص».
    accel_axes_ok = None          # None غير مفحوص · True كل المحاور · False مطفأة
    pwr_mgmt_2 = None

    def __init__(self, addr: int = MPU6050_ADDR, bus_num: int = MPU6050_I2C_BUS):
        self.ok = False
        self.error = None
        self.addr = int(addr)
        self.bus_num = int(bus_num)
        self.who_am_i = None
        self.chip = "غير مقروءة"      # اسم الشريحة المكتشفة (للسجل والواجهة)
        # عدّادات صحّة الناقل — تصاعدها يكشف تدهوراً كهربائياً لا برمجياً
        self.i2c_retries = 0          # قراءات نجحت **بعد** إعادة محاولة
        self.i2c_errors = 0           # فشلت رغم كل المحاولات
        #: None = لم يُفحص · True = كل المحاور مفعّلة · False = بعضها مطفأ
        self.accel_axes_ok = None
        self.pwr_mgmt_2 = None
        self.last_i2c_error = None
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
            if who not in WHO_AM_I_NAMES:
                raise OSError(
                    f"WHO_AM_I={hex(who)} خارج عائلة MPU المدعومة "
                    f"({'، '.join(f'{hex(k)}={v}' for k, v in WHO_AM_I_NAMES.items())})")
            self.chip = chip_name(who)
            # ⚠ الإيقاظ إلزامي: الشريحة تُقلع في وضع السكون وتُرجع أصفاراً
            #    مضبوطة — وهي بالضبط بصمة «الحسّاس الميت» التي يكشفها حارس
            #    الصفر في مصدر الاتجاه، فتبدو المشكلة اتجاهاً وهي إيقاظ.
            self._bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0)
            # 🔴 وتفعيل **كل المحاور** صراحةً — لا اتّكال على الافتراضي.
            #    نجاح الكتابة ليس دليلاً (درس الشريحة المقلَّدة §0):
            #    يُقرأ رجعياً، والفشل **حالة معلَنة** لا استثناء يُسقط الإقلاع
            #    — الجايرو وحده يكفي للاتجاه، والتسارع شاهد حركة مكمّل.
            self.accel_axes_ok = self._enable_all_axes()
            self.ok = True
        except Exception as e:        # noqa: BLE001
            self.error = (f"تعذّر فتح وحدة MPU على i2c-{self.bus_num} "
                          f"@ {hex(self.addr)}: {e} — تحقّق بـ"
                          f"`i2cdetect -y {self.bus_num}` (توقّع "
                          f"{hex(self.addr)})")
            self._close_bus()

    def _enable_all_axes(self) -> bool:
        """
        يكتب `PWR_MGMT_2 = 0x00` (كل المحاور عاملة) **ويقرأه رجعياً**.

        ⚠ القراءة الرجعية ليست ترفاً: الوحدة المقلَّدة تقبل الكتابة ولا
           تحتفظ بها (§0)، ونجاح `write` وحده كان سيُخفي الحالتين معاً —
           محوراً مطفأً، وشريحةً لا تكتب أصلاً.
        """
        try:
            self._bus.write_byte_data(self.addr, REG_PWR_MGMT_2, 0x00)
            time.sleep(0.01)
            back = self._bus.read_byte_data(self.addr, REG_PWR_MGMT_2)
            self.pwr_mgmt_2 = back
            return back == 0x00
        except Exception:             # noqa: BLE001 — I2C عابر
            self.pwr_mgmt_2 = None
            return False

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
                if who not in WHO_AM_I_NAMES:
                    return {"recovered": False,
                            "detail": f"WHO_AM_I={hex(who)} خارج عائلة MPU "
                                      f"المدعومة — شريحة أخرى أو ناقل مضطرب"}
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
                # ⚠ وإعادة التعيين (0x80) تمسح `PWR_MGMT_2` إلى افتراضه —
                #    فيُعاد تفعيل المحاور قبل الحكم على الجاذبية، وإلا حكمنا
                #    «لا بيانات» على سجلٍ نحن من أعاده.
                self.accel_axes_ok = self._enable_all_axes()
                live = None
                try:
                    d = self._bus.read_i2c_block_data(self.addr, REG_ACCEL_X, 6)
                    live = any(d[i] or d[i + 1] for i in (0, 2, 4))
                except Exception:         # noqa: BLE001 — I2C عابر
                    live = None           # تعذّرت القراءة: لا نفي ولا إثبات
                if live is False:
                    return {"recovered": False,
                            "detail": ("استيقظت لكن مقياس التسارع يقرأ أصفاراً "
                                       "مضبوطة — لا جاذبية ⇒ لا بيانات فعلاً"
                                       + ("" if self.accel_axes_ok else
                                          f" (و PWR_MGMT_2="
                                          f"{self.pwr_mgmt_2 if self.pwr_mgmt_2 is None else hex(self.pwr_mgmt_2)}"
                                          f" لم يثبت على 0x00 ⇒ محاور مطفأة)"))}
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
        """
        قراءة كتلة **بإعادة محاولة**: عثرة ناقل واحدة ليست عطل حسّاس.

        عطل مقاس 2026-08-12: المهمة أُجهضت بـ«5 قراءات فاشلة متتابعة —
        الناقل لا يردّ [Errno 121]»، بينما سكربت مباشر على نفس الناقل
        واللحظة كان يقرأ **جاذبية سليمة 0.98g** وجايرو يتذبذب حول انحيازه
        المقاس. أي أن الشريحة والتوصيل سليمان، والفاشل معاملة عابرة.

        و`Errno 121` (Remote I/O) يعني أن الشريحة لم تُقرّ بمعاملة واحدة —
        وهذا يحدث على ناقل عتادي مع ضجيج المحركات أو حِمل معالج عالٍ
        (بثّ الكاميرا يفكّ ترميز الإطارات ويعيده). محاولة ثانية بعد
        أربعة ملّي ثانية تعبرها.

        🔴 وحدّ «5 قراءات فاشلة» **قاتل للمهمة**، فثمن عدم إعادة المحاولة
           إجهاض جولة كاملة على عثرة ناقل. والحدّ يبقى كما هو: خمس
           إخفاقات **بعد** إعادة المحاولة تعني عطلاً حقيقياً.

        ⚠ وإعادة المحاولة **لا تُخفي ناقلاً متدهوراً**: `i2c_retries`
          و`i2c_errors` في `state()` — تصاعدهما يعني مشكلة كهربائية
          (أسلاك طويلة · شدّ مرتفع ضعيف · ضجيج محركات) تستحق إصلاحاً.
        """
        last = None
        with self._lock:
            for attempt in range(1 + I2C_READ_RETRIES):
                try:
                    d = self._bus.read_i2c_block_data(self.addr, reg, n)
                    if attempt:
                        self.i2c_retries += 1
                    return d
                except OSError as e:      # ENXIO/EREMOTEIO — عثرة ناقل
                    last = e
                    if attempt < I2C_READ_RETRIES:
                        time.sleep(I2C_RETRY_DELAY_S)
        self.i2c_errors += 1
        self.last_i2c_error = f"{type(last).__name__}: {last}"
        raise last

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
            "bus": self.bus_num, "driver": "smbus2",
            # 🔴 الاسم **مكتشَف** لا مكتوب: العائلة أربع شرائح بنفس السجلات
            "chip": self.chip,
            "who_am_i": (hex(self.who_am_i) if self.who_am_i is not None else None),
            # صحّة الناقل: `i2c_retries` يتصاعد ⇒ عثرات تُعبر بإعادة المحاولة
            # (أسلاك/ضجيج محركات)، و`i2c_errors` ⇒ فشل رغمها.
            "i2c_retries": self.i2c_retries, "i2c_errors": self.i2c_errors,
            "last_i2c_error": self.last_i2c_error,
            "mode": "جيرو+تسارع خام (6 محاور، بلا دمج داخلي)",
            # 🔴 لا مغنيتومتر ولا مرجع مطلق — والمشروع لم يكن يستعملهما أصلاً
            "mag_used": False,
            "gyro_z_dps": round(gz, 2),
            "gyro_dps": [round(v, 2) for v in self._gyro],
            "accel_mps2": [round(v, 2) for v in self._accel],
            "temp_c": round(self._temp_c, 1),
            # ⚠ لا مؤشّر معايرة ذاتية في MPU-6050 (خلاف BNO055) — الجاهزية
            #    تعني «استجاب» لا «معاير»، والمعايرة مسؤولية الطبقة الأعلى.
            # 🔴 صحّة مقياس التسارع **معلَنة**: محور مطفأ يُنتج أصفاراً
            #    مضبوطة، وهي بصمة «لم يتحرّك» بثقة في شاهد الحركة.
            "accel_axes_ok": self.accel_axes_ok,
            "pwr_mgmt_2": (hex(self.pwr_mgmt_2)
                           if self.pwr_mgmt_2 is not None else None),
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
