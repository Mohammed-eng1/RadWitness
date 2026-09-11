# -*- coding: utf-8 -*-
"""
bridge.py — جسر الروفر: واجهة موحّدة بوضعَي `sim` / `real`
==========================================================
- `RoverBridge`        : محاكاة قيادة خارجية بإحداثيات lat/lng (تخدم واجهة M1).
- `RoverControlBridge` : **جسر الروبوت الحقيقي** — منطق اللفّ والقصور الذاتي
                         والإنهاك والبطارية، مع بديل محاكاة كامل ليعمل على
                         ويندوز بلا عتاد. (`WaveRoverBridge` اسم قديم مُبقى.)

🔴 **الجسر لا يعرف أي منصّة تحته**: بروتوكول المنصّة خلف واجهة واحدة
`pi/rover/transport.py` — سابقة مقصودة تكرّر ما فُعل بمصدر الاتجاه في
`pi/sensors/heading.py`. وهذا الفرع **منصّة Freenove وحدها**
(`docs/BRIEF_FREENOVE_PORT.md`).

⚠ ما بقي في هذا الملف **مستقل عن الهيكل بالكامل**: تجزئة اللفّات، تهدئة
  الاقتراب، قياس القصور الذاتي وتعلّم `τ`، حارس إنهاك البطارية، تصفية
  الإشارة، كشف الشحن، تصنيف الجهد. لا يتغيّر منه حرف بتبديل الهيكل.

⚠ `read_imu`/`read_status` تعيدان `None` على Freenove: المنصّة لا توفّر
  IMU ولا جهداً — الاتجاه من MPU على i2c-4 والجهد من INA219 على i2c-1.
  🔴 `None` = **مجهول لا صفر** (البند 6.1).
"""
from __future__ import annotations

import logging
import math
import random
import time

from pi.config import (
    DRIVE_SPEED_MPS, TURN_RATE_DPS, ROVER_SAFETY_TIMEOUT_S,
    SIM_HOME_LAT, SIM_HOME_LNG,
    MOTOR_INVERT, MOTOR_SWAP_LR, TURN_POWER,
    DRIVE_POWER_DEFAULT, ROVER_TURN_TIMEOUT_S, GYRO_BIAS_CALIB_S,
    MAX_MOTOR_POWER, MAX_TURN_SEGMENT_DEG, TURN_SEGMENT_PAUSE_S,
    TURN_STALL_DPS, TURN_STALL_AFTER_S, TURN_STALL_BOOST,
    TURN_STALL_FLOOR_FRAC,
    TURN_SIGN_CHECK_DEG, BATTERY_MONITOR_ENABLED, ROVER_KIND,
    TURN_SLOWDOWN_DEG, TURN_MIN_POWER, TURN_SETTLE_S, TURN_SETTLE_RATE_DPS,
    TURN_TOLERANCE_DEG, TURN_CORRECTION_PASSES, TURN_CORRECTION_TIMEOUT_S,
    TURN_COAST_TAU_S, TURN_COAST_TAU_ALPHA, TURN_COAST_TAU_MAX_S,
    TURN_MAX_LEAD_DEG, TURN_MIN_ACHIEVABLE_DEG,
    TURN_RATE_FADE_WARN,
)
# مصدر الاتجاه صار **خلف واجهة واحدة** (البند 1): الجسر لا يعرف أي حسّاس
# يقف خلفه، ولا يحتوي معادلة تكامل. `robust_bias` مُعاد تصديره للتوافق.
from pi.sensors.heading import make_heading_source, robust_bias  # noqa: F401
# وناقل أوامر المحركات خلف واجهة واحدة كذلك — الجسر لا يعرف أي هيكل تحته.
from pi.rover.transport import make_transport

logger = logging.getLogger(__name__)

from pi.rover import battery as batt

_M_PER_DEG = 111320.0


# ═══════════════════════════════════════════════════════════════
#  1) جسر المحاكاة الخارجية (lat/lng) — يخدم واجهة M1
# ═══════════════════════════════════════════════════════════════
class RoverBridge:
    def __init__(self, mode: str = "sim"):
        self.mode = mode
        self.lat = SIM_HOME_LAT
        self.lng = SIM_HOME_LNG
        self.heading = 0.0
        self.speed = 0.0
        self._dir = "S"
        self._power = 0
        self._last_cmd_ts = 0.0
        self._from_gps = False

    def set_home_from_gps(self, lat: float, lng: float) -> None:
        if not self._from_gps and lat and lng:
            self.lat, self.lng = lat, lng
            self._from_gps = True

    def command(self, direction: str, power: int = 70) -> None:
        d = (direction or "S").upper()
        if d not in ("F", "B", "L", "R", "S"):
            return
        self._dir = d
        try:
            self._power = max(0, min(100, int(power)))
        except (TypeError, ValueError):
            self._power = 70
        self._last_cmd_ts = time.time()

    def update(self, dt: float) -> None:
        if self.mode != "sim" or dt <= 0 or dt > 2.0:
            return
        if time.time() - self._last_cmd_ts > ROVER_SAFETY_TIMEOUT_S:
            self._dir = "S"
        p = self._power / 70.0
        if self._dir == "F":
            self.speed = DRIVE_SPEED_MPS * p
            self._advance(self.speed * dt)
        elif self._dir == "B":
            self.speed = -DRIVE_SPEED_MPS * p
            self._advance(self.speed * dt)
        elif self._dir == "L":
            self.heading = (self.heading - TURN_RATE_DPS * p * dt) % 360.0
            self.speed = 0.0
        elif self._dir == "R":
            self.heading = (self.heading + TURN_RATE_DPS * p * dt) % 360.0
            self.speed = 0.0
        else:
            self.speed = 0.0

    def _advance(self, dist_m: float) -> None:
        hd = math.radians(self.heading)
        self.lat += (dist_m * math.cos(hd)) / _M_PER_DEG
        self.lng += (dist_m * math.sin(hd)) / (_M_PER_DEG * math.cos(math.radians(self.lat)))

    def state(self) -> dict:
        return {"mode": self.mode, "lat": round(self.lat, 6), "lng": round(self.lng, 6),
                "heading": round(self.heading, 1), "speed": round(self.speed, 2),
                "dir": self._dir, "power": self._power}


# ═══════════════════════════════════════════════════════════════
#  2) جسر الروبوت الحقيقي (+ محاكاة كاملة)
# ═══════════════════════════════════════════════════════════════
class RoverControlBridge:
    """
    وضعان: `real` (عتاد حقيقي) و`sim` (محاكاة كاملة تعمل على ويندوز).
    التبديل بعلم واحد؛ عند تعذّر فتح الناقل يسقط تلقائياً إلى `sim` مع
    تسجيل السبب (لا فشل صامت — البند 7).

    الهيكل يُختار بـ`kind` (افتراضه `ROVER_KIND` من config).
    """

    def __init__(self, mode: str = "sim", heading_source=None, kind: str = None):
        self.requested_mode = mode
        self.mode = "sim"
        self.error = None
        self._last_cmd_ts = 0.0
        self._moving = False
        self._cmd_lr = (0.0, 0.0)
        self.rth_requested = False
        self.battery_alarm = False
        self.last_status = {}
        # ثابت القصور الذاتي للفّ — **يُتعلَّم من القياس** بعد كل لفّة، فلا
        # يُثبَّت رقم يتغيّر بالأرضية والحمل وشحن البطارية.
        self._coast_tau = TURN_COAST_TAU_S
        self.clamp_count = 0            # مرات قصّ القوة (تُبثّ في سجل الواجهة)
        self.last_clamp_msg = None
        self.events = []                # أحداث الجسر (قصّ/تجزئة/انحياز) → الواجهة
        # الجهد: المصدر يُعلَن دائماً (البند 5)
        self._ina_reader = None         # None=لم يُجرَّب · False=غير متاح
        self.voltage_source = self.VSRC_NONE
        self.voltage_reason = None
        self.battery_amps = None
        self.battery_charging = False
        # الشحن يُكشف من **ميل الجهد** — الشنت على مسار الحِمل (config §INA219)
        self.battery_charging_unknown = True
        self.battery_amps_raw = None
        self.charge_detector = batt.ChargeDetector()
        self._sim_v_override = False
        # صحّة الوصلة وكاشف علق الإقلاع **يملكهما الناقل** — والجسر يقرأهما
        # خاصيّتين. تبقى «حالة معلنة لا استثناء منتشر» (البند 6.3).
        # ذروة معدل الدوران لأول لفّة — مرجع كشف إنهاك البطارية سلوكياً
        # ⚠ يبقى **طبقة ثانية** بعد عودة INA219: يكشف الإنهاك سلوكياً بلا
        #   فولتميتر (الدوران بالمكان أول ما يسقط)، والحارسان لا يتعارضان.
        self.turn_peak_baseline = None
        self.last_turn_peak = None
        # محاكاة
        self._sim_v = 12.40
        self._sim_turn_rate = 0.0
        self._sim_bias = -0.28
        self._last_sim_ts = time.time()

        # ── الناقل: الهيكل خلف واجهة واحدة ─────────────────────
        self.kind = (kind or ROVER_KIND)
        self._tp = make_transport(self.kind, mode)
        self._tp.emit = self._event
        self.mode = self._tp.mode
        self.error = self._tp.error
        if self.error:
            logger.warning(self.error)
            self._event("rover_transport", self.error)
        else:
            self._event("rover_transport",
                        f"ناقل الروبوت: {self._tp.name} ({self.mode})")
        # 🔴 خريطة المحركات غير مشتقّة على هيكل جديد ⇒ **تُعلَن بصوت عالٍ**
        #    (البند 6.1: المجهول يُعلَن ولا يُفترض سليماً). القيم الموروثة
        #    من Wave Rover كانت تعويضاً عن فيرموير مرآتي لم يعد موجوداً.
        if self.mode == "real" and not self._tp.motor_map_calibrated:
            self._event("motor_map_uncalibrated",
                        f"🔴 خريطة المحركات غير مشتقّة على {self._tp.name} — "
                        f"MOTOR_INVERT/MOTOR_SWAP_LR موروثان من هيكل آخر. "
                        f"اشتقّهما بمسبار المعالم قبل أي مهمة: "
                        f"python3 -m pi.tests.probe_motor_map")

        # ⚠ **بعد** تثبيت self.mode: المصنع يحتاج معرفة الوضع الفعلي ليختار
        # مصدراً صالحاً (لا BNO055 وهمي في المحاكاة ولا العكس).
        self.heading_source = heading_source or make_heading_source(bridge=self)
        if getattr(self.heading_source, "fallback_reason", None):
            logger.warning(self.heading_source.fallback_reason)
            self._event("heading_source", self.heading_source.fallback_reason)
        else:
            self._event("heading_source", f"مصدر الاتجاه: {self.heading_source.name}")

    # ── الاتجاه والانحياز: مملوكان لمصدر الاتجاه لا للجسر ────────
    @property
    def heading(self) -> float:
        return self.heading_source.heading

    @heading.setter
    def heading(self, value: float) -> None:
        self.heading_source.reset(value)

    @property
    def gyro_bias(self) -> float:
        return self.heading_source.bias

    @property
    def bias_calibrated(self) -> bool:
        return self.heading_source.bias_calibrated

    @property
    def bias_info(self) -> dict:
        return self.heading_source.bias_info

    def _event(self, kind: str, msg: str) -> None:
        """
        يسجّل حدثاً يُصرَّف إلى سجل أحداث الواجهة (البند 3).
        ⚠ **يُضغط التكرار**: عطل واحد مستمر كان يكتب عشرات الأسطر المتطابقة في
           أجزاء الثانية فيدفن كل ما قبله في نافذة الأحداث الأربعين — أي أن
           الفيضان نفسه كان يُخفي السبب الأول الذي نبحث عنه.
        """
        if self.events and self.events[-1]["kind"] == kind \
                and self.events[-1].get("base") == msg:
            e = self.events[-1]
            e["n"] = e.get("n", 1) + 1
            e["msg"] = f"{msg}  (تكرر ×{e['n']})"
            return
        self.events.append({"kind": kind, "msg": msg, "base": msg, "n": 1})
        if len(self.events) > 100:
            self.events = self.events[-100:]

    def drain_events(self) -> list:
        evs, self.events = self.events, []
        return evs

    # ── صحّة الوصلة: يملكها الناقل، والجسر يقرأها ───────────────
    # ⚠ **قابلة للكتابة** عمداً: اختبارات `pi/comms/selftest.py` تحاكي انقطاع
    #   الوصلة بضبطها، والحارس في `_turn_segment` يقرأها.
    @property
    def link_ok(self) -> bool:
        return self._tp.link_ok

    @link_ok.setter
    def link_ok(self, value: bool) -> None:
        self._tp.link_ok = bool(value)

    @property
    def link_error(self):
        return self._tp.link_error

    # ── الحركة (⚠ MOTOR_INVERT) ─────────────────────────────────
    def motors(self, l: float, r: float) -> dict:
        """
        **المسار الوحيد لإرسال أي أمر حركة** (تقدّم/رجوع/لفّ/أوامر الواجهة).
        يطبّق MOTOR_INVERT ثم **حاجزاً صارماً** عند ±MAX_MOTOR_POWER.

        ⚠ الحاجز إلزامي على Wave Rover: الفيرموير يضرب المدخل في 512 على
        PWM بدقة 8 بت (256 عدّة)، فما فوق 0.5 يلتفّ (يُطرح 0.5): إرسال 0.8
        يُنتج قوة فعّالة 0.3 — زحف صامت يفسد حساب المسافة في deadreckoning
        بلا أي إنذار. القصّ يحفظ الإشارة ويُسجَّل تحذيراً.
        🔴 والحدّ ليس تقييداً هناك: 0.5×512 = 256 = duty كامل 100% — أي كامل
        قدرة الروبوت أصلاً، فلا شيء فوق 0.5 يُخسر (CLAUDE.md §2.1).
        ⚠ وعلى Freenove **السبب يبطل** (PCA9685 دقّته 12 بت، لا التفاف) لكن
        الحاجز يبقى **حدّ أمان**: كل الثوابت المعايرة مشتقّة على هذا السقف،
        ورفعه بلا إعادة قياس سلّم القوة كاملاً يكسرها بصمت (م3).
        """
        li = max(-1.0, min(1.0, float(l)))
        ri = max(-1.0, min(1.0, float(r)))
        # 🔴 تعويض تخطيط الفيرموير المرآتي (مقاس 2026-08-07 على Wave Rover):
        #    الفيرموير المفلوش بدّل القناتين وعكس القطبية معاً — التقدّم كان
        #    يرجع للخلف والدوران يميناً يبقى يميناً. التعويض: تبديل ثم نفي.
        # ⚠ على هيكل آخر هذان الثابتان **غير مشتقّين** ويُعلَن ذلك عند
        #    الإقلاع — يُشتقّان بمسبار المعالم لا بشهادة عين (البند 2).
        sw_l, sw_r = (ri, li) if MOTOR_SWAP_LR else (li, ri)
        lw, rw = sw_l * MOTOR_INVERT, sw_r * MOTOR_INVERT  # قيم السلك

        # الحاجز الصارم — **بعد** MOTOR_INVERT، مع الحفاظ على الإشارة
        lc = max(-MAX_MOTOR_POWER, min(MAX_MOTOR_POWER, lw))
        rc = max(-MAX_MOTOR_POWER, min(MAX_MOTOR_POWER, rw))
        if lc != lw or rc != rw:
            self.clamp_count += 1
            self.last_clamp_msg = (
                f"⚠ قُصّت القوة إلى ±{MAX_MOTOR_POWER} "
                f"(طُلب L={lw:.2f} R={rw:.2f} → L={lc:.2f} R={rc:.2f}) — "
                f"{self._tp.clamp_reason}")
            logger.warning(self.last_clamp_msg)
            self._event("power_clamp", self.last_clamp_msg)

        self._cmd_lr = (lc, rc)
        self._moving = not (lc == 0.0 and rc == 0.0)
        self._last_cmd_ts = time.time()
        # 🔴 **الاقتران الوحيد بالهيكل** — خلف واجهة الناقل (transport.py)
        if self.mode == "real":
            self._tp.send_motors(lc, rc)
        if self.mode == "sim":
            # معدل الدوران من القوة **الفعّالة بعد القصّ**، مُعاداً لإطار
            # النية بعكس التحويل كاملاً (النفي ثم التبديل)
            un_l, un_r = (rc, lc) if MOTOR_SWAP_LR else (lc, rc)
            eff_l, eff_r = un_l * MOTOR_INVERT, un_r * MOTOR_INVERT
            self._sim_turn_rate = (eff_l - eff_r) * TURN_RATE_DPS
        return {"L": lc, "R": rc, "clamped": (lc != lw or rc != rw)}

    # اسم قديم — كل المسارات تمرّ عبر motors()
    def _drive(self, l: float, r: float) -> None:
        self.motors(l, r)

    def forward(self, power: float = DRIVE_POWER_DEFAULT) -> None:
        self._drive(power, power)

    def backward(self, power: float = DRIVE_POWER_DEFAULT) -> None:
        self._drive(-power, -power)

    def turn(self, direction: str, power: float = TURN_POWER) -> None:
        """دوران بالمكان: 'R' يمين، 'L' يسار."""
        if direction.upper() == "R":
            self._drive(power, -power)
        else:
            self._drive(-power, power)

    def stop(self) -> None:
        self._drive(0.0, 0.0)
        self._moving = False
        if self.mode == "sim":
            self._sim_turn_rate = 0.0

    # ── القراءات ────────────────────────────────────────────────
    def read_imu(self) -> dict:
        """
        قراءة IMU **من الهيكل** إن وفّرها. في المحاكاة يولّد gz متسقاً مع
        أمر الدوران.

        🔴 على هيكل بلا IMU (Freenove) يعيد `None` = **مجهول لا صفر**
           (البند 6.1): مصدر الاتجاه يرى `None` فيرفض القراءة بدل أن يبني
           تكاملاً على صفر مُختلَق. والاتجاه أصلاً من MPU على i2c-4.
        """
        if self.mode == "real":
            return self._tp.read_imu()
        now = time.time()
        self._last_sim_ts = now
        gz = self._sim_turn_rate + self._sim_bias + random.uniform(-0.4, 0.4)
        return {"T": 1002, "r": 0.0, "p": 0.0, "y": self.heading,
                "ax": 0.0, "ay": 0.0, "az": 9.8,
                "gx": 0.0, "gy": 0.0, "gz": round(gz, 3),
                "mx": 0.0, "my": 0.0, "mz": 0.0, "temp": 31.0}

    def read_status(self) -> dict:
        """
        حالة مختصرة **من الهيكل** إن وفّرها (وفيها جهد البطارية على
        Wave Rover). على هيكل لا يوفّرها تعيد `None` ويبقى `last_status`
        فارغاً — فيسقط `voltage()` إلى INA219 وحده ويُعلَن المصدر.
        """
        if self.mode == "real":
            d = self._tp.read_status() or {}
        else:
            if self._moving:                # استهلاك وهمي بسيط
                self._sim_v = max(9.5, self._sim_v - 0.0004)
            d = {"T": 1001, "L": self._cmd_lr[0], "R": self._cmd_lr[1],
                 "r": 0.0, "p": 0.0, "y": self.heading, "temp": 31.0,
                 "v": round(self._sim_v, 2)}
        if d:
            self.last_status = d
        return d

    # ── الجهد: INA219 أولاً، ثم رسالة الروفر، ثم لا شيء ──────────
    # ⚠ المصدر **يُعلَن دائماً** في `voltage_source`: خلط مصدرَي حماية يجعل
    #    تشخيص أي حادثة مستحيلاً («هل توقّف لأن الجهد هبط أم لأن الزمن نفد؟»).
    VSRC_INA, VSRC_ROVER, VSRC_SIM, VSRC_NONE = (
        "ina219", "rover_serial", "sim_override", "unavailable")

    def voltage(self):
        """أحدث جهد صالح أو `None`. يضبط `voltage_source` و`voltage_reason`."""
        # تجاوز المحاكاة له الأسبقية **صراحةً**: أداة اختبار العتبات في
        # الواجهة يجب أن تبقى عاملة على العتاد أيضاً، وإلا تعطّلت بصمت.
        if self._sim_v_override:
            self.voltage_source, self.voltage_reason = self.VSRC_SIM, None
            self.battery_amps, self.battery_charging = None, False
            self.battery_charging_unknown, self.battery_amps_raw = False, None
            return self._sim_v
        ina = self._ina()
        if ina is not None:
            r = ina.read()
            if r["v"] is not None:
                self.voltage_source, self.voltage_reason = self.VSRC_INA, None
                self.battery_amps = r.get("amps")
                self.battery_amps_raw = r.get("amps_raw")
                # 🔴 الشحن من **ميل الجهد** لا من التيار: الشنت على مسار
                #    الحِمل فلا يحمل اتجاهاً (مقاس — config §INA219).
                self._feed_charge_detector(r["v"])
                return r["v"]
            self.voltage_reason = r["reason"]
        self.battery_amps, self.battery_charging = None, False
        # بلا جهد لا ميل ⇒ الشحن **مجهول** لا منفيّ، والنافذة تُمسح
        self.battery_amps_raw = None
        self.charge_detector.reset("لا قراءة جهد — الشحن مجهول")
        self.battery_charging_unknown = True
        v = self.last_status.get("v")
        if v is not None:
            self.voltage_source = self.VSRC_ROVER
            return v
        self.voltage_source = self.VSRC_NONE
        if self.voltage_reason is None:
            self.voltage_reason = "لا INA219 ولا حقل v في رسالة الروفر"
        return None

    def _feed_charge_detector(self, v) -> None:
        """
        يغذّي كاشف الشحن بالجهد **ومدّة السكون منذ آخر أمر حركة**.

        ⚠ السكون ليس تفصيلاً: ارتداد الجهد بعد رفع الحمل (0.2–0.3V خلال
        ثوانٍ) يرتفع تماماً كالشحن، ولولا هذا الشرط لادّعى النظام شحناً
        بعد كل توقّف — وادّعاء الشحن **يُعفي من الإطفاء المنظَّم**.
        """
        quiet = (0.0 if self._moving
                 else max(0.0, time.time() - self._last_cmd_ts))
        self.battery_charging = self.charge_detector.feed(
            time.time(), v, quiet)
        self.battery_charging_unknown = bool(
            self.charge_detector.state()["unknown"])

    def _ina(self):
        """قارئ INA219 المشترك — يُهيَّأ كسولاً ولا يُسقط الجسر إن غاب."""
        if self._ina_reader is False:
            return None
        if self._ina_reader is None:
            try:
                from pi.sensors.ina219 import get_ina219
                r = get_ina219()
                self._ina_reader = r if r.ok else False
                if not r.ok:
                    self.voltage_reason = r.error
                    self._event("battery_source",
                                f"⚠ INA219 غير متاح: {r.error}")
                    return None
            except Exception as e:              # noqa: BLE001
                self._ina_reader = False
                self.voltage_reason = str(e)
                return None
        return self._ina_reader or None

    def battery_state(self) -> dict:
        """تصنيف الجهد **مع مصدره وسببه** — لا رقم عارٍ (البند 5)."""
        v = self.voltage()
        info = batt.classify(v, self.battery_amps)
        info["source"] = self.voltage_source
        info["source_reason"] = self.voltage_reason
        info["charging"] = self.battery_charging
        info["charging_unknown"] = self.battery_charging_unknown
        info["amps_raw"] = self.battery_amps_raw
        # الدليل نفسه لا الحكم وحده — «يشحن» بلا ميل معروض ادّعاء بلا سند
        info["charge"] = self.charge_detector.state()
        ina = self._ina_reader if self._ina_reader not in (None, False) else None
        info["ina219"] = ina.state() if ina is not None else None
        return info

    # ── أدوات المحاكاة (لاختبار العتبات في الواجهة) ─────────────
    def sim_set_voltage(self, v: float) -> None:
        self._sim_v = float(v)
        self._sim_v_override = True      # يتقدّم على INA219 عمداً (أداة اختبار)
        self.battery_alarm = False
        self.rth_requested = False

    def sim_clear_voltage_override(self) -> None:
        """يُعيد مصدر الجهد إلى العتاد الحقيقي بعد اختبار العتبات."""
        self._sim_v_override = False

    def sim_set_moving(self, moving: bool) -> None:
        """
        يجعل الاستهلاك الوهمي يعكس كون المهمة جارية — **في وضع sim حصراً**.

        ⚠ في وضع `real` لا يُلمس `_moving`: هو علم **أوامر المحركات الفعلية**
        الذي يبني عليه حارس heartbeat قراره، ومصدره الوحيد المشروع هو
        `motors()`. ضبطه من خارج مسار الإرسال يجعل الحارس يرى «يتحرك بلا
        أوامر» أثناء كل توقف مشروع (توقف القياس عند الخلية، المهلة بين مراحل
        اللفّ، معايرة الانحياز 4-5ث) فيطلق إنذارات كاذبة.
        الضرر ليس الإزعاج: الإنذار الكاذب المتكرر **يُخفي الانقطاع الحقيقي**
        وسط الضجيج، وهذا حارس سلامة. (شوهد على العتاد: 8 إنذارات في مهمة
        واحدة، وفي المحاكاة **بلا محركات إطلاقاً**.)
        """
        if self.mode != "sim":
            return
        self._moving = bool(moving)

    # ── معايرة انحياز مصدر الاتجاه (تلقائياً عند بدء كل مهمة) ────
    def calibrate_gyro_bias(self, seconds: float = GYRO_BIAS_CALIB_S) -> float:
        """
        يقيس الانحياز والروبوت **ساكن** (وسيط + استبعاد شواذ) — التنفيذ في
        مصدر الاتجاه، والجسر يوقف المحركات ويسجّل النتيجة فقط.
        ⚠ لا يُثبَّت في الكود — يتغيّر بين التجارب، فيُقاس عند بدء كل مهمة.
        """
        self.stop()
        info = self.heading_source.calibrate_bias(seconds)
        if info.get("ok"):
            if info.get("rejected"):
                self._event("gyro_bias",
                            f"انحياز {info['bias']:.3f} (σ={info['std']:.2f})، "
                            f"استُبعدت {info['rejected']} عينة شاذّة")
        else:
            self._event("gyro_bias_rejected",
                        f"⚠ رُفضت معايرة الانحياز ({self.heading_source.name}) — "
                        f"{info.get('reason')}")
        return self.gyro_bias

    # ── الدوران بزاوية عبر مصدر الاتجاه (مهلة أمان) ─────────────
    @staticmethod
    def _turn_power(remaining_deg: float, turned_deg: float, base: float,
                    floor: float = TURN_MIN_POWER) -> float:
        """
        قوة اللفّ لحظياً: كاملة حتى يقترب الهدف، ثم تنزل خطياً إلى `floor`
        داخل آخر `TURN_SLOWDOWN_DEG`. الطاقة الحركية تتناسب مع مربّع المعدل،
        فخفض المعدل قبل القطع يقلّص التجاوز أكثر من نسبياً.

        ⚠ التهدئة **لا تبدأ قبل أن يدور فعلاً** (`turned` > 0.5°): كسر السكون
           يحتاج القوة الكاملة، ولفّة تصحيح صغيرة تبدأ داخل قوس التهدئة أصلاً
           فلو خُفّضت قوّتها من اللحظة الأولى لما تحرّك الروبوت إطلاقاً.
        ⚠ `floor` يرتفع مع كاسر الانحشار: على أرضية عالية الاحتكاك تُعيد
           الأرضية الثابتة (0.25) الروبوتَ إلى القوة التي انحشر عندها.
        """
        floor = max(0.0, min(float(floor), base))
        if abs(turned_deg) < 0.5 or remaining_deg >= TURN_SLOWDOWN_DEG:
            return base
        frac = max(0.0, remaining_deg) / TURN_SLOWDOWN_DEG
        p = floor + (base - floor) * frac
        return max(floor, min(base, p))

    def _measure_coast(self) -> float:
        """
        يقيس الدوران **بعد قطع الطاقة** حتى يستقرّ الروبوت أو تنتهي النافذة.

        ⚠ بدونه يبقى التجاوز غير مرئي تماماً: حلقة اللفّ تنتهي لحظة بلوغ
           الهدف فيتجمّد `turned` عند 90 بينما القصور الذاتي يواصل الدوران —
           فيظنّ النظام أنه على 90° وهو على 100°، ويسير مستقيماً في اتجاه
           خاطئ (المشي مستقيم والوجهة غلط). القياس لا يمنع التجاوز، لكنه
           يجعله **رقماً معلوماً** فيصير قابلاً للتصحيح بدل أن يتراكم صامتاً.

        ⚠ يُستدعى **قبل** العودة إلى طور «السير»: عتبة السير (12°/ث) ترفض
           قراءات القصور الذاتي (عشرات الدرجات/ث) فتضيع القياس كله.
        """
        if not self.heading_source.ok:
            return 0.0
        extra = 0.0
        deadline = time.time() + TURN_SETTLE_S
        while time.time() < deadline:
            d = self.heading_source.update()
            extra += d["delta"]
            if abs(d["dps"]) < TURN_SETTLE_RATE_DPS:
                break                        # استقرّ — لا تنتظر بقية النافذة
            time.sleep(0.02)
        return extra

    def _check_turn_fade(self, peak: float) -> None:
        """
        **مقياس شحن ضمني بلا فولتميتر** (حسّاس الجهد معطّل على هذا العتاد).

        الدوران بالمكان أثقل مناورة على المنصّة — أربعة محركات تصارع احتكاكاً
        جانبياً — فهو **أول ما يسقط** مع ضعف البطارية بينما يبقى السير ممكناً.
        فذروة معدل الدوران تقيس القدرة المتاحة، وهي تُقاس في كل لفّة أصلاً بلا
        وقت إضافي ولا عتاد إضافي. مرجعُها أول لفّة في هذه الجلسة.

        ⚠ تحذير لا إيقاف: انخفاض الذروة قد يكون أرضية مختلفة أو حملاً زائداً،
           وإيقاف كاذب يوقف مسحاً سليماً. الإيقاف يخصّ **العجز الكامل عن
           اللفّ** وحده (`turn_no_rotation`) وهو لا يحتمل تأويلاً.
        """
        if peak <= 0.0:
            return
        self.last_turn_peak = peak
        if self.turn_peak_baseline is None:
            self.turn_peak_baseline = peak
            return
        if peak > self.turn_peak_baseline:        # أرضية أفضل/شحن أعلى
            self.turn_peak_baseline = peak
            return
        ratio = peak / self.turn_peak_baseline
        if ratio < TURN_RATE_FADE_WARN:
            self._event("turn_fade",
                        f"⚠ ذروة الدوران {peak:.0f}°/ث مقابل "
                        f"{self.turn_peak_baseline:.0f}°/ث في أول لفّة "
                        f"({ratio * 100:.0f}%) — علامة إنهاك بطارية. "
                        f"الدوران أول ما يسقط، والسير يبقى ممكناً فترة بعده.")

    def _learn_coast_tau(self, rate_at_cut: float, coast_deg: float) -> None:
        """
        يتعلّم ثابت القصور الذاتي من القياس: `τ = القصور ÷ المعدل لحظة القطع`.

        ⚠ **يُقاس ولا يُثبَّت**: يتغيّر بالأرضية (سيراميك/سجاد)، وبالحمل،
           وبشحن البطارية — تماماً كانحياز الجايرو. تقدير config قيمة ابتدائية
           تُستبدل بأول قياس صالح لا رقم نهائي.
        ⚠ الشرط على **المقسوم عليه وحده** (المعدّل لحظة القطع): قسمة على معدّل
           صغير تضخّم الضجيج فتُفسد التقدير. أما قصورٌ يقارب الصفر مع معدّل
           كبير فهو **قياس صحيح ومفيد** يقول «هذه المنصّة تقف فوراً» — وكان
           استبعاده يمنع τ المبالَغ فيه من النزول أبداً، فتقصُر كل لفّة.
        """
        if abs(rate_at_cut) < TURN_SETTLE_RATE_DPS:
            return
        tau = min(abs(coast_deg) / abs(rate_at_cut), TURN_COAST_TAU_MAX_S)
        a = TURN_COAST_TAU_ALPHA
        self._coast_tau = (1.0 - a) * self._coast_tau + a * tau

    def _turn_segment(self, degrees: float, timeout: float, power: float) -> dict:
        """
        مرحلة لفّ واحدة، بمهلة أمان خاصة بها. **لا معادلة تكامل هنا** — تُقرأ
        الزاوية من مصدر الاتجاه (البند 1) فيصير تبديل الحسّاس بلا لمس الجسر.

        حارس الإشارة: لو دار الروبوت **عكس** المطلوب بأكثر من
        TURN_SIGN_CHECK_DEG فمحور z مقلوب (تثبيت الحسّاس) أو الأسلاك معكوسة —
        نُجهض بسبب صريح بدل الدوران حتى المهلة (وقد يكون دورانه بلا نهاية).
        """
        turned = 0.0
        coast = 0.0
        rate_at_cut = 0.0
        peak_rate = 0.0
        link_fault = False
        timed_out = False
        sign_mismatch = False
        want = 1.0 if degrees >= 0 else -1.0
        direction = "R" if degrees >= 0 else "L"

        # ⚠⚠ **لا تُشغَّل المحركات ومصدر الاتجاه معطّل** ⚠⚠
        # اللفّ بلا زاوية مقروءة لفّ أعمى، والأسوأ أن حلقة المهمة كانت تعيد
        # المحاولة فوراً: كل محاولة تُشغّل المحركات لحظة ثم توقفها عند أول
        # `update()` فاشل، فيرتجف الروبوت في مكانه بمعدل مئات النبضات في
        # الثانية (شوهد على العتاد مع 40 حدثاً في 0.2ث). الفحص هنا — قبل أي
        # أمر حركة — يجعل الإجهاض **بلا حركة إطلاقاً**.
        if not self.heading_source.ok:
            rec = self.heading_source.attempt_recovery()
            if rec.get("recovered"):
                self._event("heading_recovered",
                            f"✅ أُحيي مصدر الاتجاه: {rec.get('detail')}")
            else:
                self._event("heading_fault",
                            f"⚠ لفّ مرفوض — مصدر الاتجاه معطّل: "
                            f"{self.heading_source.error}")
                return {"turned": 0.0, "timed_out": False,
                        "sign_mismatch": False, "source_ok": False}
        try:
            # ⚠ طور «اللفّ»: يوسّع عتبة القفزة (40–60°/ث دوران طبيعي لا ضجيج)
            #    ويخفّف التنعيم. بعتبة طور السير كانت كل قراءة تُرفض والزاوية
            #    المتكاملة تبقى صفراً فيلفّ الروبوت حتى المهلة.
            self.heading_source.set_phase("turn")
            spikes0 = self.heading_source.cond.spikes
            # 🔴 كاسر الانحشار: القوة الفعّالة تُرفع **بالقياس** حين تُؤمر
            #    المحركات ولا يدور الروبوت (أرضية عالية الاحتكاك: سجاد/فرش).
            power_now = float(power)
            stall_t0 = None
            boosts = 0
            # 🔴 الإحياء وسط اللفّة يُفقد زاويةً صامتاً — انظر الحارس أدناه
            rec0 = self.heading_source.recoveries
            slept = False
            self.turn(direction, power_now)
            start = time.time()
            self.heading_source.update()      # يثبّت مرجع الزمن/الزاوية
            while abs(turned) < abs(degrees):
                if time.time() - start > timeout:
                    timed_out = True
                    break
                d = self.heading_source.update()
                turned += d["delta"]
                rate_at_cut = d["dps"]
                peak_rate = max(peak_rate, abs(rate_at_cut))
                # ⚠ وصلة ميتة = لا أمر حركة يصل: التوقف فوراً بسبب صريح بدل
                #    الدوران حتى المهلة على روبوت لا يسمع.
                if not self.link_ok:
                    link_fault = True
                    break
                if turned * want < -TURN_SIGN_CHECK_DEG:
                    sign_mismatch = True
                    self._event("heading_sign",
                                f"⚠ دار {turned:.0f}° عكس المطلوب ({degrees:.0f}°) — "
                                f"إشارة محور z مقلوبة؟ راجع BNO055_GYRO_Z_SIGN")
                    break
                if not self.heading_source.ok:
                    self._event("heading_fault",
                                f"⚠ مصدر الاتجاه توقّف: {self.heading_source.error}")
                    break
                # 🔴 **نام الحسّاس وأُحيي وسط اللفّة ⇒ الزاوية غير موثوقة**
                # مقاس على العتاد (2026-08-10، check_imu_health --motors):
                # اندفاع تيار المحركات أنام MPU في النبضة الثانية من ستّ.
                # وأثناء النوم يقرأ الجايرو صفراً مضبوطاً — فدوران تلك
                # المللي‑ثانية **يُفقد من التكامل صامتاً**، واللفّة تظنّ
                # نفسها ناقصة فتواصل وتتجاوز هدفها فعلياً. الإحياء التلقائي
                # (وهو صحيح) لا يستطيع استرجاع ما فات، فالحكم الوحيد الأمين:
                # أجهض هذه اللفّة **وأعدها** — الزاوية بعدها مقيسة من جديد.
                # ⚠ ليست قاتلة للمهمة: العطل عابر بطبيعته (نبضة تيار)،
                #    والمهمة تعيد الخطوة بتهدئتها المعتادة.
                if self.heading_source.recoveries > rec0:
                    slept = True
                    self._event(
                        "heading_slept_midturn",
                        f"⚠ نام مصدر الاتجاه وأُحيي **أثناء لفّة** "
                        f"{degrees:+.0f}° — زاوية مفقودة، تُعاد اللفّة. "
                        f"(هبوط تغذية عند اندفاع المحركات ⇒ إصلاح عتادي: "
                        f"مكثّف عند الحسّاس أو فصل تغذيته عن خط المحركات)")
                    break
                # ── الاستباق: اقطع الطاقة **قبل** الهدف بزاوية القصور ────
                # التصحيح بعد الوقوع لا يقارب: لفّة تصحيح صغيرة تحتاج القوة
                # الكاملة لكسر السكون فتُنتج قصوراً بحجم الخطأ نفسه فتتأرجح.
                # هنا يقع التصحيح قبل القطع، والثابت الزمني مقاس لا مفترض.
                lead = min(abs(rate_at_cut) * self._coast_tau, TURN_MAX_LEAD_DEG)
                if abs(turned) + lead >= abs(degrees):
                    break
                # ── كاسر الانحشار (سجاد/فرش) ────────────────────────
                # ⚠ **بعد** فحوص الإشارة والمصدر والاستباق عمداً: لا نرفع
                #    القوة على روبوت يدور عكسياً أو على حسّاس معطّل.
                if abs(rate_at_cut) < TURN_STALL_DPS:
                    now_s = time.time()
                    if stall_t0 is None:
                        stall_t0 = now_s
                    elif (now_s - stall_t0 >= TURN_STALL_AFTER_S
                          and power_now < MAX_MOTOR_POWER):
                        power_now = min(MAX_MOTOR_POWER,
                                        power_now + TURN_STALL_BOOST)
                        boosts += 1
                        stall_t0 = now_s
                        if boosts == 1:
                            self._event(
                                "turn_stall_boost",
                                f"⚠ لا دوران عند قوة {power:.2f} — أرضية "
                                f"عالية الاحتكاك (سجاد/فرش؟). تُرفع القوة "
                                f"تدريجياً حتى {MAX_MOTOR_POWER:.2f}.")
                else:
                    stall_t0 = None
                # ⚠ أرضية التهدئة ترتفع مع القوة: الأرضية الثابتة (0.25)
                #    تُعيد الروبوت إلى القوة التي انحشر عندها أصلاً فيقف
                #    داخل قوس التهدئة — وهو بالضبط عرَض «يلتفّ قليلاً ثم يقف».
                floor_now = (TURN_MIN_POWER if not boosts else
                             max(TURN_MIN_POWER,
                                 power_now * TURN_STALL_FLOOR_FRAC))
                # ⚠ **جدّد أمر اللفّ** كل دورة: بلا تجديد يمرّ 1.5ث فيعتبره
                # حارس الـheartbeat انقطاعاً ويوقف المحركات في منتصف اللفّة
                # (كانت اللفّة تتوقف عند ~24° لهذا السبب).
                # القوة تتهدّأ قرب الهدف — لا قطع مفاجئ من 137°/ث إلى صفر.
                self.turn(direction,
                          self._turn_power(abs(degrees) - abs(turned), turned,
                                           power_now, floor_now))
                time.sleep(0.02)
        finally:
            self.stop()                      # ⚠ إيقاف مضمون لكل مرحلة
            # ⚠ الترتيب ملزم: القياس **ثم** العودة إلى طور السير (عتبة السير
            #    ترفض معدّلات القصور الذاتي فتبتلع القياس).
            coast = self._measure_coast()
            turned += coast
            self._learn_coast_tau(rate_at_cut, coast)
            self.heading_source.set_phase("drive")
        # ⚠ «مهلة اللفّ» وحدها تشخيص فقير: لفّة أنجزت 70° ثم تعثّرت ≠ لفّة
        #    **لم تدر أصلاً**. الثانية تعني أن الأوامر تُرسل والروبوت لا
        #    يستجيب — وأشيع أسبابها بترتيب الاحتمال:
        #      • بطارية منهكة: الدوران بالمكان أثقل مناورة (أربعة محركات +
        #        احتكاك جانبي) فهو أول ما يسقط، بينما يبقى السير ممكناً.
        #      • عائق مادي يمنع الدوران، أو عجلة عالقة.
        #      • الأوامر لا تصل الفيرموير (منفذ/أسلاك).
        #    ⚠ مراقبة الجهد معطّلة، فلا شيء يكشف الأول تلقائياً — لذلك تُسمّى
        #      الاحتمالات في الرسالة بدل تركها «مهلة».
        # ⚠⚠ **القراءات المرفوضة كقفزة لا تُبتلع صامتةً** ⚠⚠
        # فوق عتبة الطور يُعيد المرشّح صفراً لا القراءة، فيتجمّد التكامل في
        # أسرع لحظة من اللفّة: الروبوت يدور والجايرو لا يعدّ. والعرَض المميّز
        # خبيث — لفّة تتجاوز هدفها فعلياً بينما البرمجية تظنّها ناقصة، أي نفس
        # بصمة «معامل جايرو خاطئ» وهي ليست كذلك.
        # مقاس على العتاد (2026-08-01): ذروة 198°/ث مقابل عتبة كانت 200.
        spikes = self.heading_source.cond.spikes - spikes0
        if spikes:
            self._event("turn_spikes",
                        f"⚠ رُفضت {spikes} قراءة كقفزة أثناء لفّة {degrees:+.0f}° "
                        f"(ذروة {peak_rate:.0f}°/ث مقابل عتبة "
                        f"{self.heading_source.cond.spike_dps:.0f}°/ث) — "
                        f"التكامل يفقد جزءاً من الدوران. ارفع "
                        f"HEADING_SPIKE_DPS_TURN.")
        no_rotation = (timed_out and not link_fault
                       and abs(turned) < TURN_MIN_ACHIEVABLE_DEG)
        if no_rotation:
            self._event("turn_no_rotation",
                        f"⚠ اللفّ لم يبدأ أصلاً: أُمرت المحركات {timeout:.0f}ث "
                        f"ودار الروبوت {turned:+.1f}° فقط (من {degrees:+.0f}°). "
                        f"الأرجح **بطارية منهكة** — الدوران بالمكان أثقل مناورة "
                        f"وأول ما يسقط بينما يبقى السير ممكناً. تحقّق أيضاً من "
                        f"عائق يمنع الدوران أو عجلة عالقة.")
        return {"turned": turned, "timed_out": timed_out,
                "sign_mismatch": sign_mismatch, "coast": coast,
                "rate_at_cut": rate_at_cut, "coast_tau": self._coast_tau,
                "peak_rate": peak_rate, "link_fault": link_fault,
                "no_rotation": no_rotation, "spikes": spikes,
                "stall_boosts": boosts, "power_used": power_now,
                "slept_midturn": slept,
                "source_ok": self.heading_source.ok}

    def turn_by_angle(self, degrees: float,
                      timeout: float = ROVER_TURN_TIMEOUT_S,
                      power: float = TURN_POWER) -> dict:
        """
        heading += (gz - bias) * dt * GYRO_SCALE.
        ⚠ **اللفّات الطويلة تُجزَّأ** إلى مراحل ≤ MAX_TURN_SEGMENT_DEG مع توقف
        بينها — لفّة 360° متواصلة تفشل على العتاد (تُنهك البطارية فتتوقف عند
        ~195°). المهلة (8ث) **لكل مرحلة** لا للفّة كاملة.
        """
        # ⚠ الترتيب مقصود: **صحّة المصدر قبل المعايرة**. معايرة الانحياز تدور
        #   GYRO_BIAS_CALIB_S كاملة (4-5ث) وهي تقرأ أصفاراً من حسّاس معطّل، ثم
        #   تُرفض حتماً — فكل لفّة تدفع خمس ثوانٍ ثمناً لنتيجة معروفة سلفاً.
        if not self.heading_source.ok:
            rec = self.heading_source.attempt_recovery()
            if not rec.get("recovered"):
                self._event("heading_fault",
                            f"⚠ لفّ مرفوض — مصدر الاتجاه معطّل: "
                            f"{self.heading_source.error}")
                return {"requested_deg": degrees, "turned_deg": 0.0,
                        "timed_out": False, "aborted": "heading_source_fault",
                        "heading": round(self.heading, 1), "segments": 0}
        if not self.bias_calibrated:
            self.calibrate_gyro_bias()
        sign = 1.0 if degrees >= 0 else -1.0
        remaining = abs(float(degrees))
        segments = []
        while remaining > 0:
            seg = min(MAX_TURN_SEGMENT_DEG, remaining)
            segments.append(sign * seg)
            remaining -= seg
        if len(segments) > 1:
            self._event("turn_segmented",
                        f"لفّ {degrees:.0f}° على {len(segments)} مراحل")

        turned_total = 0.0
        timed_out = False
        aborted = None
        coast_total = 0.0
        overshoot = 0.0
        corrections = 0
        peak = 0.0
        spikes_total = 0
        no_rotation = False
        # كاسر الانحشار: يُجمع عبر المراحل والتصحيحات ليصل المهمة والواجهة
        stall_boosts = 0
        power_used = float(power)
        try:
            for i, seg in enumerate(segments):
                r = self._turn_segment(seg, timeout, power)
                turned_total += r["turned"]
                peak = max(peak, r.get("peak_rate", 0.0))
                spikes_total += r.get("spikes", 0)
                stall_boosts += r.get("stall_boosts", 0)
                power_used = max(power_used, r.get("power_used", power))
                no_rotation = no_rotation or r.get("no_rotation", False)
                if r.get("link_fault"):
                    aborted = "rover_link_fault"
                    break
                # ⚠ قصور **اللفّة الأصلية وحدها**. جمعه مع قصور لفّات التصحيح
                #    يُلغيه: التصحيح يدور عكس الاتجاه فيأتي قصوره بالإشارة
                #    المضادة، فيظهر المجموع ~0 ويبدو كأن لا قصور أصلاً — وهو
                #    بالضبط الرقم الذي أُضيف ليُرى.
                coast_total += r.get("coast", 0.0)
                if r["timed_out"]:
                    timed_out = True
                    break
                if r.get("sign_mismatch"):
                    aborted = "sign_mismatch"
                    break
                # ⚠ **ليس في قائمة القاتلات**: عطل تغذية عابر تُعاد الخطوة
                #    بعده — لا سبب لقتل مهمة كاملة بنبضة تيار.
                if r.get("slept_midturn"):
                    aborted = "heading_slept_midturn"
                    break
                if not r.get("source_ok", True):
                    aborted = "heading_source_fault"
                    break
                if i < len(segments) - 1:
                    time.sleep(TURN_SEGMENT_PAUSE_S)   # استرداد البطارية والمحركات

            # ── تصحيح الخطأ المتبقّي بعد الاستقرار ────────────────
            # صار ممكناً **لأن التجاوز صار مقاساً**: قبل نافذة الاستقرار كان
            # `turned` يتجمّد عند الهدف فيبدو الخطأ صفراً دائماً ولا شيء
            # يُصحَّح. الآن الفرق حقيقي، ولفّة قصيرة تغلقه.
            overshoot = turned_total - degrees
            while (aborted is None and not timed_out
                   and abs(turned_total - degrees) > TURN_TOLERANCE_DEG
                   and corrections < TURN_CORRECTION_PASSES):
                corrections += 1
                err = degrees - turned_total
                c = self._turn_segment(err, TURN_CORRECTION_TIMEOUT_S, power)
                turned_total += c["turned"]
                stall_boosts += c.get("stall_boosts", 0)
                power_used = max(power_used, c.get("power_used", power))
                if c.get("sign_mismatch") or not c.get("source_ok", True):
                    break
                # لم يتحرّك: الزاوية أصغر من أن تكسر السكون عند هذه القوة —
                # التكرار لن يغيّر شيئاً، فنقبل ونُبلّغ بدل حرق المهلة.
                if abs(c["turned"]) < 0.5:
                    self._event("turn_stalled",
                                f"⚠ تعذّر تصحيح {err:+.1f}° — الزاوية أصغر من "
                                f"أن تكسر السكون عند قوة {power}")
                    break
                # ⚠ حارس عدم التقارب: محاولة لم تُقرّبنا من الهدف تعني أن
                #    قصور لفّة التصحيح نفسه بحجم الخطأ — التكرار يتأرجح حول
                #    الهدف بلا اقتراب ويستهلك البطارية. نقف ونُبلّغ بالرقم.
                if abs(degrees - turned_total) >= abs(err):
                    self._event("turn_stalled",
                                f"⚠ توقّف التصحيح: الخطأ {err:+.1f}° → "
                                f"{degrees - turned_total:+.1f}° (لا تقارب) — "
                                f"قصور لفّة التصحيح بحجم الخطأ نفسه")
                    break
            residual = turned_total - degrees
            if corrections or abs(overshoot) > TURN_TOLERANCE_DEG:
                self._event("turn_accuracy",
                            f"لفّ {degrees:.0f}°: تجاوز {overshoot:+.1f}° "
                            f"(منه {coast_total:+.1f}° قصور ذاتي بعد قطع الطاقة)"
                            + (f" → {corrections} تصحيح → المتبقّي "
                               f"{residual:+.1f}°" if corrections else ""))
            self._check_turn_fade(peak)
        finally:
            self.stop()                      # ⚠ إيقاف مضمون
        # ⚠ لا نجمع turned_total على self.heading: مصدر الاتجاه حدّثه أصلاً في
        #    كل update() — الجمع مرة ثانية يضاعف كل لفّة.
        return {"requested_deg": degrees, "turned_deg": round(turned_total, 1),
                "peak_rate_dps": round(peak, 1), "no_rotation": no_rotation,
                "spikes": spikes_total,
                # كاسر الانحشار: تصاعده عبر المهمة = الأرضية تزداد مقاومة
                # (سجاد سميك) أو البطارية تنهك — كلاهما يستحق أن يُرى.
                "stall_boosts": stall_boosts, "power_used": round(power_used, 3),
                "timed_out": timed_out, "aborted": aborted,
                "heading": round(self.heading, 1),
                "segments": len(segments),
                # أرقام الدقة **مرئية**: تجاوزٌ يتصاعد مع المهمة يعني تهدئة
                # غير كافية (أنزل TURN_MIN_POWER أو وسّع TURN_SLOWDOWN_DEG).
                "coast_deg": round(coast_total, 1),
                "overshoot_deg": round(overshoot, 1),
                "corrections": corrections,
                "residual_deg": round(turned_total - degrees, 1),
                "coast_tau_s": round(self._coast_tau, 4)}

    # ── السلامة: heartbeat + البطارية ───────────────────────────
    def check_heartbeat(self) -> bool:
        """بلا أوامر > 1.5ث والمحركات تعمل → إيقاف فوري. True إن أوقف."""
        if self._moving and (time.time() - self._last_cmd_ts) > ROVER_SAFETY_TIMEOUT_S:
            self.stop()
            return True
        return False

    def check_battery(self) -> dict:
        """
        يقرأ الجهد ويطبّق عتبات أ-3 الإلزامية.
        ⚠ مع تعطيل المراقبة (BATTERY_MONITOR_ENABLED=False) لا يُقرأ الجهد ولا
        يُتَّخذ إجراء — الحماية البديلة **حدّ زمني** على مدة المهمة (mission.py).
        """
        if not BATTERY_MONITOR_ENABLED:
            return batt.classify(None)
        self.read_status()
        info = self.battery_state()
        if info["action"] == batt.ACTION_STOP:
            self.stop()
            self.battery_alarm = True
        elif info["action"] == batt.ACTION_RTH:
            self.rth_requested = True
        return info

    def state(self) -> dict:
        return {
            "mode": self.mode, "requested_mode": self.requested_mode,
            "error": self.error, "heading": round(self.heading, 1),
            "gyro_bias": round(self.gyro_bias, 4),
            "bias_calibrated": self.bias_calibrated,
            "moving": self._moving, "cmd": {"L": self._cmd_lr[0], "R": self._cmd_lr[1]},
            "link_ok": self.link_ok, "link_error": self.link_error,
            # المنصّة وحالة ناقلها — 🔴 وخريطة المحركات: «مشتقّة أم لا»
            # تُعرض صراحةً (البند 6.1: المجهول يُعلَن).
            "kind": self.kind, "transport": self._tp.name,
            "motor_map_calibrated": self._tp.motor_map_calibrated,
            "transport_state": self._tp.state(),
            "battery": (self.battery_state() if BATTERY_MONITOR_ENABLED
                        else batt.classify(None)),
            "rth_requested": self.rth_requested, "battery_alarm": self.battery_alarm,
            "max_motor_power": MAX_MOTOR_POWER,
            "clamp_count": self.clamp_count, "last_clamp_msg": self.last_clamp_msg,
            # ثابت القصور الذاتي المتعلَّم: صفر = لم تُنفَّذ لفّة بعد (لا استباق).
            "coast_tau_s": round(self._coast_tau, 4),
            "heading_source": self.heading_source.state(),
        }

    def close(self) -> None:
        try:
            self.stop()
        except Exception:                    # noqa: BLE001
            pass
        try:
            self._tp.close()
        except Exception:                    # noqa: BLE001
            pass


# ── الاسم القديم — مُبقى للتوافق ────────────────────────────────
# 🔴 لا يُعاد تسميته في المستدعين (15 ملفاً): التسمية ليست الهدف، والهدف
#    أن الجسر لم يعد يعرف هيكله. الاسم القديم أثرٌ تاريخي كـ`mpu6050.py`
#    الذي صار يقرأ MPU-6500.
WaveRoverBridge = RoverControlBridge
