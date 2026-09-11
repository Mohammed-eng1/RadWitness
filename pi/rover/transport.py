# -*- coding: utf-8 -*-
"""
transport.py — ناقل أوامر المحركات **خلف واجهة واحدة**
========================================================
سابقة معمارية مقصودة: مصدر الاتجاه وُضع خلف واجهة واحدة
(`pi/sensors/heading.py`) فلم يعد الجسر يعرف أي حسّاس يقف خلفه. هذه الوحدة
تفعل الشيء نفسه **للناقل**: الجسر لا يعرف أي هيكل يقف تحته.

🔴 **هذا الفرع منصّة Freenove وحدها**: Wave Rover روبوت مختلف تماماً — لا
يشترك في المتحكّم ولا نظام الطاقة ولا المحركات — فلم يُنقل ناقله ولا قيوده
ولا «احتياطاً». تاريخه في `main` لمن أراده.

الناقلات:
    FreenoveTransport : PCA9685 على I2C مباشرة من الراسبري

🔴 **المحاكاة ليست ناقلاً**: في وضع `sim` لا يُستدعى الناقل إطلاقاً — سلوك
المحاكاة (الجهد الوهمي، `gz` المتسق مع أمر الدوران، الاستهلاك) يبقى في
الجسر كما كان بالضبط. خلط المحاكاة بالنقل كان سيغيّر سلوكاً مختبَراً في
تغيير هدفه **ألّا يغيّر شيئاً**.
"""
from __future__ import annotations

import logging

from pi.config import (
    FREENOVE_I2C_BUS, FREENOVE_PCA9685_ADDR, FREENOVE_PWM_FREQ_HZ,
    FREENOVE_WHEEL_CHANNELS, FREENOVE_BRAKE_ON_STOP, FREENOVE_DUTY_MAX,
)

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════
#  الواجهة
# ═══════════════════════════════════════════════════════════════
class Transport:
    """
    واجهة الناقل. المتعاقَد عليه:

    - `send_motors(l, r)` **لا يرفع استثناءً أبداً** (البند 6.3): استثناء
      في مسار الحركة ينتشر إلى `_turn_segment` ثم يفجّر
      `finally: self.stop()` نفسه لأنه يسلك نفس المسار — فتضيع «الإيقاف
      المضمون» في اللحظة الوحيدة التي وُجدت لأجلها. الخطأ **حالة معلنة**
      (`link_ok`) لا استثناء منتشر.
    - `read_imu` / `read_status` تعيدان `None` إن كان الناقل لا يوفّرهما.
      🔴 `None` = **مجهول لا صفر** (البند 6.1).
    """

    kind = "none"
    name = "بلا ناقل"
    #: سبب قصّ القوة — يختلف بالهيكل، ويُعرض في تحذير القصّ
    clamp_reason = ""
    #: هل خريطة المحركات (نفي/تبديل) مشتقّة بمسبار المعالم على هذا الهيكل؟
    motor_map_calibrated = False

    def __init__(self) -> None:
        self.mode = "sim"           # "real" إن نجح الفتح
        self.error = None           # سبب السقوط إلى المحاكاة (لا فشل صامت)
        self.link_ok = True
        self.link_error = None
        self.emit = lambda kind, msg: None   # يضبطه الجسر

    def open(self) -> None: ...
    def send_motors(self, l: float, r: float) -> None: ...
    def read_imu(self): return None
    def read_status(self): return None
    def state(self) -> dict: return {}
    def close(self) -> None: ...


# ═══════════════════════════════════════════════════════════════
#  Freenove 4WD — PCA9685 على I2C
# ═══════════════════════════════════════════════════════════════
class FreenoveTransport(Transport):
    """
    تحكّم **مباشر** من الراسبري: لا وسيط ولا بروتوكول ولا ردود.

    ⇒ يسقط كل ما بُني حول ESP32: انتظار الإقلاع · كاشف الأصفار · صدى
      الأوامر · الأوامر المحظورة · إعادة فتح المنفذ.
    ⇒ ولا IMU ولا جهد من الهيكل: `read_imu`/`read_status` تعيدان `None`
      (**مجهول لا صفر** — البند 6.1). والاتجاه من MPU على i2c-4 كما هو،
      والجهد من INA219 على i2c-1 كما هو.

    🔴 **خريطة المحركات غير مشتقّة بعد** على هذا الهيكل: النفي والتبديل
       يبقيان محايدَين (+1/False) ويُعلَن ذلك بصوت عالٍ حتى يُشتقّا بمسبار
       المعالم (م2 في وثيقة النقل). القيم الموروثة من Wave Rover **لا
       تُستعمل**: كانت تعويضاً عن فيرموير مرآتي لم يعد موجوداً.

    ⚠ `duty = 0` في تخطيط Freenove **كبح فعّال** (4095 على القناتين) لا
      انسياب. أثره على القصور الذاتي للفّة **يُقاس ولا يُفترض** (م5) —
      ولهذا يبقى `TURN_COAST_TAU_S = 0` يتعلّم ذاتياً.
    """

    kind = "freenove"
    name = "Freenove 4WD (PCA9685/I2C)"
    clamp_reason = "حدّ أمان (PCA9685 دقّته 12 بت — لا التفاف عددي)"
    motor_map_calibrated = False     # 🔴 تُشتقّ بمسبار المعالم — م2

    def __init__(self, bus: int = FREENOVE_I2C_BUS,
                 address: int = FREENOVE_PCA9685_ADDR,
                 freq_hz: float = FREENOVE_PWM_FREQ_HZ):
        super().__init__()
        self.bus_num, self.address, self.freq_hz = bus, address, freq_hz
        self._pwm = None
        self._last_duty = (0, 0)

    def open(self) -> None:
        try:
            from pi.rover.pca9685 import PCA9685
            self._pwm = PCA9685(bus=self.bus_num, address=self.address,
                                freq_hz=self.freq_hz)
            self.mode = "real"
        except Exception as e:               # noqa: BLE001
            self.error = (f"تعذّر فتح PCA9685 على i2c-{self.bus_num} "
                          f"@0x{self.address:02x}: {e} — وضع المحاكاة")

    def _wheel(self, ch_a: int, ch_b: int, duty: int) -> None:
        """
        عجلة واحدة على قناتين (جسر H). موجب ⇒ B يقود وA صفر، والعكس بالعكس.

        ⚠ عند `duty == 0` يكتب تخطيط Freenove **4095 على القناتين معاً** =
          كبح قصير (short brake) لا فصل طاقة. سلوك الهيكل الأصلي، ومُبقى
          كما هو ليكون القياس في م5 على السلوك الحقيقي — ويُعطَّل من
          `FREENOVE_BRAKE_ON_STOP` عند اختبار البديل.
        """
        if duty > 0:
            self._pwm.set_duty(ch_a, 0)
            self._pwm.set_duty(ch_b, duty)
        elif duty < 0:
            self._pwm.set_duty(ch_b, 0)
            self._pwm.set_duty(ch_a, -duty)
        elif FREENOVE_BRAKE_ON_STOP:
            self._pwm.set_duty(ch_a, FREENOVE_DUTY_MAX)
            self._pwm.set_duty(ch_b, FREENOVE_DUTY_MAX)
        else:
            self._pwm.set_duty(ch_a, 0)
            self._pwm.set_duty(ch_b, 0)

    def send_motors(self, l: float, r: float) -> None:
        """⚠⚠ **لا يرفع استثناءً أبداً** — انظر عقد `Transport`."""
        if self.mode != "real" or self._pwm is None:
            return
        dl = int(round(max(-1.0, min(1.0, l)) * FREENOVE_DUTY_MAX))
        dr = int(round(max(-1.0, min(1.0, r)) * FREENOVE_DUTY_MAX))
        self._last_duty = (dl, dr)
        for side, duty in (("left", dl), ("right", dr)):
            for ch_a, ch_b in FREENOVE_WHEEL_CHANNELS[side]:
                self._wheel(ch_a, ch_b, duty)
        # صحّة الناقل **حالة معلنة** لا استثناء (البند 6.3)
        if self._pwm.ok:
            if not self.link_ok:
                self.link_ok, self.link_error = True, None
                self.emit("rover_link", "✅ عاد ناقل I2C للمحركات")
        elif self.link_ok:
            self.link_ok = False
            self.link_error = self._pwm.error
            self.emit("rover_link_fault",
                      f"⛔ فشلت الكتابة على PCA9685 (i2c-{self.bus_num} "
                      f"@0x{self.address:02x}): {self._pwm.error} — لا أمر "
                      f"حركة يصل. ⚠ آخر duty قد يبقى منفَّذاً: افصل الطاقة "
                      f"يدوياً إن تحرّك.")

    # 🔴 الهيكل لا يوفّرهما — `None` مجهول لا صفر (البند 6.1)
    def read_imu(self):    return None
    def read_status(self): return None

    def state(self) -> dict:
        return {"i2c_bus": self.bus_num,
                "pca9685_addr": f"0x{self.address:02x}",
                "pwm_freq_hz": self.freq_hz,
                "duty": {"L": self._last_duty[0], "R": self._last_duty[1]},
                "write_errors": (self._pwm.write_errors
                                 if self._pwm is not None else None)}

    def close(self) -> None:
        try:
            if self._pwm is not None:
                self._pwm.close()
        except Exception:                    # noqa: BLE001
            pass


# ═══════════════════════════════════════════════════════════════
#  المصنع
# ═══════════════════════════════════════════════════════════════
_KINDS = {
    "freenove": FreenoveTransport,
}


def make_transport(kind: str, mode: str = "sim", **kw) -> Transport:
    """
    يبني الناقل ويفتحه عند طلب `real`. عند تعذّر الفتح يبقى `mode="sim"`
    **مع سبب معلن في `error`** — لا فشل صامت (البند 7).
    """
    cls = _KINDS.get((kind or "").strip().lower())
    if cls is None:
        tp = Transport()
        tp.error = (f"نوع هيكل غير معروف: {kind!r} — "
                    f"المتاح: {', '.join(sorted(_KINDS))}. وضع المحاكاة")
        return tp
    tp = cls(**kw)
    if mode == "real":
        tp.open()
    return tp
