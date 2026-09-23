# -*- coding: utf-8 -*-
"""
bridge.py — جسر الروفر: واجهة موحّدة بوضعَي `sim` / `real`
==========================================================
- `RoverBridge`      : محاكاة قيادة خارجية بإحداثيات lat/lng (تخدم واجهة M1).
- `WaveRoverBridge`  : **جسر Wave Rover الحقيقي** ببروتوكول Waveshare المكتشف
                       تجريبياً، مع بديل محاكاة كامل ليعمل على ويندوز بلا عتاد.

بروتوكول Waveshare (JSON سطري + \\n على ROVER_PORT @115200 — uart3
`/dev/ttyAMA3` منذ 2026-08-08 بعد موت TXD0، انظر config):
    إرسال:  {"T":1,"L":<-1..1>,"R":<-1..1>}   حركة (تُضرب في MOTOR_INVERT)
            {"T":126}                          طلب IMU كامل
            {"T":130}                          طلب حالة مختصرة
    استقبال: {"T":1002, r,p,y, ax..az, gx..gz, mx..mz, temp}   ردّ 126
            {"T":1001, L,R, r,p,y, temp, v}                    ردّ 130

⚠ ملاحظات مثبتة على العتاد:
  - الفيرموير كان **يردّد الأمر المُرسل صدىً** قبل الرد الفعلي. أُطفئ الصدى
    نهائياً (2026-08-07) بخطوة {"T":143,"cmd":0} في boot.mission على ESP32،
    ومنطق تجاهله في `_read_until` **باقٍ دفاعياً**: لوحة بديلة أو مسح
    boot.mission يعيدان الصدى (افتراضيه في الفيرموير مفعّل).
  - `y` (yaw) **للعرض فقط لا للملاحة** — الملاحة من `gz` (انظر CLAUDE.md).
  - `T=131`, `T=4`, `T=71` بلا رد — لا تعتمد عليها.
  - **الأمر المرتد لا يعني التنفيذ** — تحقق من الحركة عبر gz/التسارع.
"""
from __future__ import annotations

import json
import logging
import math
import random
import threading
import time

from pi.config import (
    DRIVE_SPEED_MPS, TURN_RATE_DPS, ROVER_SAFETY_TIMEOUT_S,
    SIM_HOME_LAT, SIM_HOME_LNG,
    ROVER_PORT, ROVER_BAUD, MOTOR_INVERT, MOTOR_SWAP_LR, TURN_POWER,
    DRIVE_POWER_DEFAULT, ROVER_TURN_TIMEOUT_S, GYRO_BIAS_CALIB_S,
    MAX_MOTOR_POWER, MAX_TURN_SEGMENT_DEG, TURN_SEGMENT_PAUSE_S,
    TURN_STALL_DPS, TURN_STALL_AFTER_S, TURN_STALL_BOOST,
    TURN_STALL_FLOOR_FRAC,
    TURN_SIGN_CHECK_DEG, BATTERY_MONITOR_ENABLED,
    TURN_SLOWDOWN_DEG, TURN_MIN_POWER, TURN_SETTLE_S, TURN_SETTLE_RATE_DPS,
    TURN_TOLERANCE_DEG, TURN_CORRECTION_PASSES, TURN_CORRECTION_TIMEOUT_S,
    TURN_COAST_TAU_S, TURN_COAST_TAU_ALPHA, TURN_COAST_TAU_MAX_S,
    TURN_MAX_LEAD_DEG, TURN_MIN_ACHIEVABLE_DEG, ROVER_LINK_REOPEN_S,
    TURN_RATE_FADE_WARN,
    ESP32_BOOT_WAIT_S, ESP32_STUCK_ZERO_BYTES, ESP32_STUCK_RETRY_S,
    ESP32_STUCK_RETRIES,
    IS_UGV01, UGV01_INIT_CMDS, UGV01_INIT_GAP_S,
)
# مصدر الاتجاه صار **خلف واجهة واحدة** (البند 1): الجسر لا يعرف أي حسّاس
# يقف خلفه، ولا يحتوي معادلة تكامل. `robust_bias` مُعاد تصديره للتوافق.
from pi.sensors.heading import make_heading_source, robust_bias  # noqa: F401

logger = logging.getLogger(__name__)

from pi.rover import battery as batt

# استيراد العتاد محميّ — غيابه (ويندوز) لا يكسر شيئاً
try:
    import serial
    _SERIAL_OK = True
except Exception:                     # noqa: BLE001
    _SERIAL_OK = False

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
#  2) جسر Wave Rover الحقيقي (+ محاكاة كاملة)
# ═══════════════════════════════════════════════════════════════
class WaveRoverBridge:
    """
    وضعان: `real` (سيريال حقيقي) و`sim` (محاكاة كاملة تعمل على ويندوز).
    التبديل بعلم واحد؛ عند طلب `real` وغياب pyserial/المنفذ يسقط تلقائياً
    إلى `sim` مع تسجيل السبب (لا فشل صامت).
    """

    def __init__(self, mode: str = "sim", port: str = ROVER_PORT, baud: int = ROVER_BAUD,
                 heading_source=None):
        self.requested_mode = mode
        self.mode = "sim"
        self.error = None
        self.port, self.baud = port, baud
        self._ser = None
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
        # صحّة وصلة السيريال — **حالة معلنة لا استثناء منتشر** (انظر `_send`)
        self.link_ok = True
        self.link_error = None
        self._last_reopen_ts = 0.0
        # كاشف «ESP32 عالق في الإقلاع» (تدفق أصفار — انظر config §ESP32)
        # ⚠ حالة مستقلة عن link_ok عمداً: المنفذ سليم والكتابة تنجح، لكن
        #   الطرف الآخر يبثّ أصفاراً — عرَض مختلف عن «لا رد» وعلاجه مختلف.
        self.esp32_stuck = False
        self._zero_run = 0
        self._stuck_attempts = 0
        self._stuck_cooldown_until = 0.0
        self._port_ready_ts = 0.0
        # 🔴 يحجز كل مُرسِل حتى تنتهي نافذة الإقلاع **والتهيئة** معاً — انظر
        #    `_wait_esp32_boot`.
        self._boot_lock = threading.Lock()
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

        if mode == "real":
            if not _SERIAL_OK:
                self.error = "pyserial غير مثبّت — وضع المحاكاة"
            else:
                try:
                    self._ser = serial.Serial(port, baud, timeout=0.3)
                    self.mode = "real"
                    # ⚠ لا أمر قبل اكتمال إقلاع ESP32 (~3ث): الإرسال أثناءه
                    #   قد يعلّقه في وضع الإقلاع (يبثّ أصفاراً، علاجه Reset
                    #   يدوي). الانتظار كسول في `_wait_esp32_boot` — عند أول
                    #   أمر فعلي لا هنا، وغالباً تكون المدة انقضت أصلاً في
                    #   تهيئة بقية الأنظمة.
                    self._port_ready_ts = time.time() + ESP32_BOOT_WAIT_S
                except Exception as e:      # noqa: BLE001
                    self.error = f"تعذّر فتح {port}: {e} — وضع المحاكاة"

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

    # ── إقلاع ESP32 وكاشف العلق ─────────────────────────────────
    def _wait_esp32_boot(self) -> None:
        """
        ينتظر اكتمال إقلاع ESP32 قبل **أول** أمر بعد فتح المنفذ.

        عطل مشخَّص (2026-08-07): الإرسال على TX أثناء الإقلاع (~3ث مقاسة) قد
        يعبث بأطراف وضع الإقلاع (GPIO0 ونحوها) فيعلق ESP32 يبثّ أصفاراً
        متدفقة ولا يخرج منه إلا زرّ Reset. الانتظار مرة واحدة، ويُقتطع منه
        ما انقضى منذ فتح المنفذ (تهيئة بقية الأنظمة تستهلك المدة غالباً).
        """
        if not self._port_ready_ts:
            return                                  # المسار السريع بعد الإقلاع
        # 🔴 قفل لا علم: عطل مقاس (2026-09-23، لوحة وهمية): كان العلم يُصفَّر
        #    **قبل** النوم، فخيط ثانٍ يجده صفراً ويكتب فوراً أثناء نافذة
        #    الإقلاع. النتيجة المقاسة: «إيقاف» من خيط ثانٍ خرج أولاً، ثم
        #    «تقدّم» الخيط الأول بعد النوم — **فبقي الروبوت يتحرّك بعد أمر
        #    الإيقاف**. وعلى UGV01 يسبق الأمرُ المتسرّب T:900 أيضاً. الآن
        #    يُصفَّر العلم **بعد** التهيئة، وكل مُرسِل آخر ينتظر القفل.
        with self._boot_lock:
            if not self._port_ready_ts:
                return                              # أنهاه خيط آخر ونحن ننتظر
            wait = self._port_ready_ts - time.time()
            if wait > 0:
                self._event("esp32_boot_wait",
                            f"انتظار اكتمال إقلاع ESP32 ({wait:.1f}ث) قبل أول أمر")
                time.sleep(wait)
            self._ugv01_init()
            self._port_ready_ts = 0.0

    def _ugv01_init(self) -> None:
        """
        تهيئة لوحة UGV01 قبل أول أمر فعلي (وبعد كل إعادة فتح للمنفذ).

        🔴 `{"T":900,"main":3}` إلزامي: بدونه تستعمل اللوحة ثوابت روبوت آخر
        **بصمت** (قطر عجلة وعرض مسار) فتكذب كل سرعة مقاسة وكل أمر T:1.
        والبقية تُطفئ الصدى والتشخيص والبثّ المستمر كي يبقى التحليل السطري
        نظيفاً. نفس التهيئة المختبرة على العتاد في pi/tests/ugv01.
        ⚠ كتابة مباشرة لا `_send`: هذا المسار داخل `_send` نفسه، ولا يرفع
        استثناءً أبداً (§6.3) — فشله يُعلَن حدثاً ويبقى `link_ok` لـ`_send`.
        """
        if not IS_UGV01 or self.mode != "real" or self._ser is None:
            return
        try:
            for cmd in UGV01_INIT_CMDS:
                self._ser.write((json.dumps(cmd) + "\n").encode("ascii"))
                time.sleep(UGV01_INIT_GAP_S)
            self._ser.reset_input_buffer()      # ردود التهيئة لا تخصّ أحداً
            self._event("ugv01_init",
                        "تهيئة UGV01: T:900 main=3 · إطفاء الصدى/التشخيص/البثّ")
        except Exception as e:                  # noqa: BLE001
            self._event("ugv01_init_fault", f"⚠ تعذّرت تهيئة UGV01: {e}")

    def _mark_esp32_alive(self) -> None:
        """أي JSON صالح من الفيرموير يصفّر الكاشف ويرفع إعلان العلق إن وُجد."""
        self._zero_run = 0
        self._stuck_attempts = 0
        if self.esp32_stuck:
            self.esp32_stuck = False
            self._event("esp32_recovered",
                        "✅ عاد ESP32 يستجيب (يبدو أن زرّ Reset ضُغط) — "
                        "الوصلة سليمة")

    def _note_zero_bytes(self, n: int) -> None:
        """
        كاشف «ESP32 عالق في الإقلاع» — العرَض المقاس: **تدفق أصفار** مستمر
        على UART (لا صمت ولا بيانات)، والعلاج زرّ Reset على اللوحة.

        ⚠ **ليس** «لا رد»: الصمت عرَض وصلة/heartbeat وعلاجه مختلف — خلط
           التشخيصين يضيّعهما معاً، لذلك حالة مستقلة (`esp32_stuck`) لا
           `link_ok`.
        ⚠ إعادة الفحص **موزَّعة على الاستدعاءات** لا حلقة نوم واحدة: القراءة
           الدورية (poll_battery كل 200ms) هي المحاولة التالية أصلاً، وحلقة
           نوم 3×2ث داخل مسار القراءة كانت ستجمّد حلقة السيرفر كلها.
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
            self._event("esp32_stuck_suspect",
                        f"⚠ ESP32 يبثّ أصفاراً (عالق في الإقلاع؟) — إعادة "
                        f"الفحص بعد {ESP32_STUCK_RETRY_S:.0f}ث "
                        f"(محاولة {self._stuck_attempts}/{ESP32_STUCK_RETRIES})")
            # تفريغ المتراكم حتى لا تُحسب أصفار قديمة على المحاولة التالية
            try:
                self._ser.reset_input_buffer()
            except Exception:        # noqa: BLE001
                pass
            return
        self.esp32_stuck = True
        # ⚠ مؤشر «شبكة UGV تظهر عند الإقلاع السليم» بطل منذ تعطيل راديو
        #   WiFi في الفيرموير (DISABLE_WIFI_RADIO — الهوائي محترق): الشبكة
        #   لا تظهر أبداً الآن، وظهورها يعني فيرموير قديماً على اللوحة.
        self._event("esp32_stuck",
                    "🔴 ESP32 عالق في الإقلاع — اضغط زر Reset على لوحة "
                    "الروبوت. (الوقاية: شغّل الروبوت أولاً وانتظر 5 ثوانٍ "
                    "قبل الراسبري/الأوامر. ومؤشر شبكة UGV لم يعد صالحاً "
                    "بعد تعطيل راديو WiFi في الفيرموير)")

    # ── الإرسال/الاستقبال ────────────────────────────────────────
    def _reopen(self) -> bool:
        """
        محاولة إحياء واحدة للمنفذ، **بتهدئة**: الحلقة تُرسل كل 20ms، وفتح
        منفذ فاشل في كل دورة يحوّل العطل إلى شلل.
        """
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
            self._ugv01_init()        # لوحة أُعيد إقلاعها تنسى T:900
            return True
        except Exception:                    # noqa: BLE001
            return False

    def _send(self, obj: dict) -> None:
        """
        ⚠⚠ **لا يرفع استثناءً أبداً** ⚠⚠
        عطل مقاس (2026-08-01): عند موت المنفذ (أُغلق تحت الخيط، أو فُصل
        الكيبل) كان `write` يرمي SerialException فينتشر من `motors()` إلى
        `_turn_segment` — **ثم يرمي `finally: self.stop()` نفسه** لأنه يسلك
        نفس المسار بالضبط. فتضيع «الإيقاف المضمون» في اللحظة الوحيدة التي
        وُجدت لأجلها، ويطبع بايثون شلال استثناءات متداخلة يدفن السبب الأول.
        الخطأ يُلتقط هنا ويصير **حالة معلنة** (`link_ok`) تقرأها طبقة أعلى.
        """
        if self.mode != "real" or self._ser is None:
            return
        self._wait_esp32_boot()
        payload = (json.dumps(obj) + "\n").encode("ascii")
        try:
            self._ser.write(payload)
            if not self.link_ok:
                self.link_ok, self.link_error = True, None
                self._event("rover_link", "✅ عاد اتصال الروفر")
            return
        except Exception as e:               # noqa: BLE001
            why = str(e)
        # إحياء واحد ثم إعادة المحاولة — المنفذ قد يكون أُغلق ويُعاد فتحه
        if self._reopen():
            try:
                self._ser.write(payload)
                self.link_ok, self.link_error = True, None
                self._event("rover_link", "✅ أُعيد فتح منفذ الروفر")
                return
            except Exception as e:           # noqa: BLE001
                why = str(e)
        if self.link_ok:
            self._event("rover_link_fault",
                        f"⛔ انقطع اتصال الروفر ({self.port}): {why} — "
                        f"لا أمر حركة يصل. ⚠ آخر أمر قد يبقى منفَّذاً في "
                        f"الفيرموير حتى مهلته: افصل الطاقة يدوياً إن تحرّك.")
        self.link_ok = False
        self.link_error = why

    def _read_until(self, expect_t: int, request_t: int, timeout: float = 1.0):
        """
        يقرأ أسطراً حتى يجد T == expect_t. **يتجاهل صدى الأمر المُرسل**
        (أي سطر T == request_t) وأي سطر غير صالح.
        """
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
            # كاشف العلق: عدّ الأصفار من البايتات **الخام** قبل فكّ الترميز —
            # ESP32 العالق في الإقلاع يبثّ b'\x00...' متدفقة لا JSON ولا صمتاً.
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
            # ⚠ الفيرموير يرسل أحياناً JSON صالحاً لكنه **ليس كائناً** (رقم
            # مجرّد مثل 0) → json.loads يعطي int، ومناداة .get عليه تنفجر
            # بـ'int' object has no attribute 'get' وتُسقط المهمة كلها.
            if not isinstance(d, dict):
                continue
            self._mark_esp32_alive()
            t = d.get("T")
            if t == request_t:
                continue                    # صدى الأمر — تجاهل
            if t == expect_t:
                return d
        return None

    # ── الحركة (⚠ MOTOR_INVERT) ─────────────────────────────────
    def motors(self, l: float, r: float) -> dict:
        """
        **المسار الوحيد لإرسال أي أمر حركة** (تقدّم/رجوع/لفّ/أوامر الواجهة).
        يطبّق MOTOR_INVERT ثم **حاجزاً صارماً** عند ±MAX_MOTOR_POWER.

        ⚠ الحاجز إلزامي: الفيرموير يضرب المدخل في 512 على PWM بدقة 8 بت
        (256 عدّة)، فما فوق 0.5 يلتفّ (يُطرح 0.5): إرسال 0.8 يُنتج قوة فعّالة
        0.3 — زحف صامت يفسد حساب المسافة في deadreckoning بلا أي إنذار.
        القصّ يحفظ الإشارة ويُسجَّل تحذيراً.
        🔴 والحدّ ليس تقييداً: 0.5×512 = 256 = duty كامل 100% — أي كامل قدرة
        الروبوت أصلاً، فلا شيء فوق 0.5 يُخسر (CLAUDE.md §2.1).
        """
        li = max(-1.0, min(1.0, float(l)))
        ri = max(-1.0, min(1.0, float(r)))
        # 🔴 تعويض تخطيط الفيرموير المرآتي (مقاس 2026-08-07): الفيرموير
        #    المفلوش بدّل القناتين وعكس القطبية معاً — التقدّم كان يرجع
        #    للخلف والدوران يميناً يبقى يميناً. التعويض: تبديل ثم نفي.
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
                + ("سقف سرعة UGV01 العملي 0.5 م/ث" if IS_UGV01
                   else "التفاف فيرموير Wave Rover فوق 0.5"))
            logger.warning(self.last_clamp_msg)
            self._event("power_clamp", self.last_clamp_msg)

        self._cmd_lr = (lc, rc)
        self._moving = not (lc == 0.0 and rc == 0.0)
        self._last_cmd_ts = time.time()
        self._send({"T": 1, "L": round(lc, 3), "R": round(rc, 3)})
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
        """T=126 → T:1002. في المحاكاة يولّد gz متسقاً مع أمر الدوران."""
        if self.mode == "real":
            self._send({"T": 126})
            d = self._read_until(1002, 126)
            return d or {}
        now = time.time()
        self._last_sim_ts = now
        gz = self._sim_turn_rate + self._sim_bias + random.uniform(-0.4, 0.4)
        return {"T": 1002, "r": 0.0, "p": 0.0, "y": self.heading,
                "ax": 0.0, "ay": 0.0, "az": 9.8,
                "gx": 0.0, "gy": 0.0, "gz": round(gz, 3),
                "mx": 0.0, "my": 0.0, "mz": 0.0, "temp": 31.0}

    def read_status(self) -> dict:
        """T=130 → T:1001 (يتضمّن جهد البطارية v)."""
        if self.mode == "real":
            self._send({"T": 130})
            d = self._read_until(1001, 130) or {}
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
            # علق إقلاع ESP32 — حالة مستقلة عن link_ok (العلاج: زرّ Reset)
            "esp32_stuck": self.esp32_stuck,
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
            if self._ser:
                self._ser.close()
        except Exception:                    # noqa: BLE001
            pass
