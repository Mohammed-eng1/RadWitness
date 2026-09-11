# -*- coding: utf-8 -*-
"""
transport.py — ناقل أوامر المحركات **خلف واجهة واحدة**
========================================================
سابقة معمارية مقصودة: مصدر الاتجاه وُضع خلف واجهة واحدة
(`pi/sensors/heading.py`) فلم يعد الجسر يعرف أي حسّاس يقف خلفه. هذه الوحدة
تفعل الشيء نفسه **للناقل**: الجسر لا يعرف أي هيكل يقف تحته.

السبب المباشر: هيكل Wave Rover تعطّل ويُستبدل بـFreenove 4WD
(`docs/BRIEF_FREENOVE_PORT.md`). والاقتران بالهيكل كان **سطراً واحداً** —
`_send({"T":1,...})` داخل `motors()` — لأن قاعدة «كل أمر حركة يمرّ بـ
`bridge.motors()`» حصرته هناك. هذه الوحدة تقتطع ذلك السطر فقط.

الناقلات:
    WaveRoverTransport : UART JSON سطري → ESP32 (Waveshare)
    FreenoveTransport  : PCA9685 على I2C مباشرة من الراسبري

🔴 **المحاكاة ليست ناقلاً**: في وضع `sim` لا يُستدعى الناقل إطلاقاً — سلوك
المحاكاة (الجهد الوهمي، `gz` المتسق مع أمر الدوران، الاستهلاك) يبقى في
الجسر كما كان بالضبط. خلط المحاكاة بالنقل كان سيغيّر سلوكاً مختبَراً في
تغيير هدفه **ألّا يغيّر شيئاً**.
"""
from __future__ import annotations

import json
import logging
import time

from pi.config import (
    ROVER_PORT, ROVER_BAUD, ROVER_LINK_REOPEN_S,
    ESP32_BOOT_WAIT_S, ESP32_STUCK_ZERO_BYTES, ESP32_STUCK_RETRY_S,
    ESP32_STUCK_RETRIES,
    FREENOVE_I2C_BUS, FREENOVE_PCA9685_ADDR, FREENOVE_PWM_FREQ_HZ,
    FREENOVE_WHEEL_CHANNELS, FREENOVE_BRAKE_ON_STOP, FREENOVE_DUTY_MAX,
)

logger = logging.getLogger(__name__)

try:
    import serial
    _SERIAL_OK = True
except Exception:                     # noqa: BLE001
    _SERIAL_OK = False


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
#  Wave Rover — UART JSON إلى ESP32
# ═══════════════════════════════════════════════════════════════
class WaveRoverTransport(Transport):
    """
    بروتوكول Waveshare (JSON سطري + \\n @115200):
        {"T":1,"L":..,"R":..} حركة · {"T":126} IMU · {"T":130} حالة
        الردود: T=1002 (IMU كامل) · T=1001 (حالة مختصرة فيها الجهد v)

    ⚠ الملاحظات المثبتة على العتاد منقولة كما هي من الجسر — الفيرموير كان
      يردّد الأمر صدىً (أُطفئ بـboot.mission، ومنطق التجاهل باقٍ دفاعياً)،
      و`y` للعرض لا للملاحة، و`T=131/4/71` بلا رد.
    """

    kind = "waverover"
    name = "Wave Rover (UART/ESP32)"
    clamp_reason = "التفاف فيرموير Wave Rover فوق 0.5"
    motor_map_calibrated = True      # ✅ مشتقّة بمسبار المعالم 2026-08-09

    def __init__(self, port: str = ROVER_PORT, baud: int = ROVER_BAUD):
        super().__init__()
        self.port, self.baud = port, baud
        self._ser = None
        self._last_reopen_ts = 0.0
        # كاشف «ESP32 عالق في الإقلاع» — حالة **مستقلة عن link_ok عمداً**:
        # المنفذ سليم والكتابة تنجح لكن الطرف الآخر يبثّ أصفاراً. عرَض مختلف
        # عن «لا رد» وعلاجه مختلف (زرّ Reset)، وخلطهما يضيّعهما معاً.
        self.esp32_stuck = False
        self._zero_run = 0
        self._stuck_attempts = 0
        self._stuck_cooldown_until = 0.0
        self._port_ready_ts = 0.0

    def open(self) -> None:
        if not _SERIAL_OK:
            self.error = "pyserial غير مثبّت — وضع المحاكاة"
            return
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.3)
            self.mode = "real"
            # ⚠ لا أمر قبل اكتمال إقلاع ESP32 (~3ث): الإرسال أثناءه قد يعلّقه
            #   في وضع الإقلاع (يبثّ أصفاراً، علاجه Reset يدوي). الانتظار
            #   كسول — عند أول أمر فعلي، وغالباً انقضت المدة في تهيئة الباقي.
            self._port_ready_ts = time.time() + ESP32_BOOT_WAIT_S
        except Exception as e:            # noqa: BLE001
            self.error = f"تعذّر فتح {self.port}: {e} — وضع المحاكاة"

    # ── إقلاع ESP32 وكاشف العلق ─────────────────────────────────
    def _wait_esp32_boot(self) -> None:
        if not self._port_ready_ts:
            return
        wait = self._port_ready_ts - time.time()
        self._port_ready_ts = 0.0
        if wait > 0:
            self.emit("esp32_boot_wait",
                      f"انتظار اكتمال إقلاع ESP32 ({wait:.1f}ث) قبل أول أمر")
            time.sleep(wait)

    def _mark_esp32_alive(self) -> None:
        """أي JSON صالح من الفيرموير يصفّر الكاشف ويرفع إعلان العلق إن وُجد."""
        self._zero_run = 0
        self._stuck_attempts = 0
        if self.esp32_stuck:
            self.esp32_stuck = False
            self.emit("esp32_recovered",
                      "✅ عاد ESP32 يستجيب (يبدو أن زرّ Reset ضُغط) — "
                      "الوصلة سليمة")

    def _note_zero_bytes(self, n: int) -> None:
        """
        كاشف العلق: العرَض المقاس **تدفق أصفار** مستمر (لا صمت ولا بيانات).
        ⚠ إعادة الفحص **موزَّعة على الاستدعاءات** لا حلقة نوم واحدة: القراءة
           الدورية هي المحاولة التالية أصلاً، وحلقة نوم داخل مسار القراءة
           كانت ستجمّد حلقة السيرفر كلها.
        """
        if self.esp32_stuck:
            return                  # مُعلَن — بانتظار Reset يدوي، لا تكرار
        now = time.time()
        if now < self._stuck_cooldown_until:
            return                  # مهلة بين المحاولات — أصفارها لا تُحسب
        self._zero_run += n
        if self._zero_run < ESP32_STUCK_ZERO_BYTES:
            return
        self._zero_run = 0
        self._stuck_attempts += 1
        if self._stuck_attempts < ESP32_STUCK_RETRIES:
            self._stuck_cooldown_until = now + ESP32_STUCK_RETRY_S
            self.emit("esp32_stuck_suspect",
                      f"⚠ ESP32 يبثّ أصفاراً (عالق في الإقلاع؟) — إعادة "
                      f"الفحص بعد {ESP32_STUCK_RETRY_S:.0f}ث "
                      f"(محاولة {self._stuck_attempts}/{ESP32_STUCK_RETRIES})")
            try:
                self._ser.reset_input_buffer()
            except Exception:        # noqa: BLE001
                pass
            return
        self.esp32_stuck = True
        self.emit("esp32_stuck",
                  "🔴 ESP32 عالق في الإقلاع — اضغط زر Reset على لوحة "
                  "الروبوت. (الوقاية: شغّل الروبوت أولاً وانتظر 5 ثوانٍ "
                  "قبل الراسبري/الأوامر. ومؤشر شبكة UGV لم يعد صالحاً "
                  "بعد تعطيل راديو WiFi في الفيرموير)")

    # ── الإرسال/الاستقبال ────────────────────────────────────────
    def _reopen(self) -> bool:
        """محاولة إحياء واحدة **بتهدئة**: الحلقة ترسل كل 20ms، وفتح منفذ
        فاشل في كل دورة يحوّل العطل إلى شلل."""
        now = time.time()
        if not _SERIAL_OK or now - self._last_reopen_ts < ROVER_LINK_REOPEN_S:
            return False
        self._last_reopen_ts = now
        try:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:            # noqa: BLE001
                    pass
            self._ser = serial.Serial(self.port, self.baud, timeout=0.3)
            return True
        except Exception:                    # noqa: BLE001
            return False

    def _send(self, obj: dict) -> None:
        """⚠⚠ **لا يرفع استثناءً أبداً** — انظر عقد `Transport`."""
        if self.mode != "real" or self._ser is None:
            return
        self._wait_esp32_boot()
        payload = (json.dumps(obj) + "\n").encode("ascii")
        try:
            self._ser.write(payload)
            if not self.link_ok:
                self.link_ok, self.link_error = True, None
                self.emit("rover_link", "✅ عاد اتصال الروفر")
            return
        except Exception as e:               # noqa: BLE001
            why = str(e)
        if self._reopen():
            try:
                self._ser.write(payload)
                self.link_ok, self.link_error = True, None
                self.emit("rover_link", "✅ أُعيد فتح منفذ الروفر")
                return
            except Exception as e:           # noqa: BLE001
                why = str(e)
        if self.link_ok:
            self.emit("rover_link_fault",
                      f"⛔ انقطع اتصال الروفر ({self.port}): {why} — "
                      f"لا أمر حركة يصل. ⚠ آخر أمر قد يبقى منفَّذاً في "
                      f"الفيرموير حتى مهلته: افصل الطاقة يدوياً إن تحرّك.")
        self.link_ok = False
        self.link_error = why

    def _read_until(self, expect_t: int, request_t: int, timeout: float = 1.0):
        """يقرأ حتى يجد T == expect_t، متجاهلاً صدى الأمر وأي سطر غير صالح."""
        if self.mode != "real":
            return None
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                raw = self._ser.readline()
            except Exception:               # noqa: BLE001
                break
            if not raw:
                continue
            # عدّ الأصفار من البايتات **الخام** قبل فكّ الترميز
            zeros = raw.count(b"\x00")
            if zeros:
                self._note_zero_bytes(zeros)
            line = raw.decode("ascii", errors="replace").strip("\x00 \t\r\n")
            if not line:
                continue
            try:
                d = json.loads(line)
            except ValueError:
                continue                    # سطر غير JSON — تجاهل
            # ⚠ الفيرموير يرسل أحياناً JSON صالحاً ليس كائناً (رقم مجرّد) →
            #   مناداة .get عليه تنفجر وتُسقط المهمة كلها.
            if not isinstance(d, dict):
                continue
            self._mark_esp32_alive()
            t = d.get("T")
            if t == request_t:
                continue                    # صدى الأمر — تجاهل
            if t == expect_t:
                return d
        return None

    # ── الواجهة ─────────────────────────────────────────────────
    def send_motors(self, l: float, r: float) -> None:
        self._send({"T": 1, "L": round(l, 3), "R": round(r, 3)})

    def read_imu(self):
        self._send({"T": 126})
        return self._read_until(1002, 126) or {}

    def read_status(self):
        self._send({"T": 130})
        return self._read_until(1001, 130) or {}

    def state(self) -> dict:
        return {"port": self.port,
                # علق إقلاع ESP32 — حالة مستقلة عن link_ok (العلاج: Reset)
                "esp32_stuck": self.esp32_stuck}

    def close(self) -> None:
        try:
            if self._ser:
                self._ser.close()
        except Exception:                    # noqa: BLE001
            pass


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
    "waverover": WaveRoverTransport,
    "freenove":  FreenoveTransport,
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
