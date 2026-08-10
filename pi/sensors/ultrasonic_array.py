# -*- coding: utf-8 -*-
"""
ultrasonic_array.py — ثلاثة ألترا سونيك **بالتناوب** (أمامي + جانبيان)
=======================================================================
🔴 **التناوب قيد فيزيائي لا تفضيل**: تشغيل الحساسات معاً يجعل كل واحد
   يلتقط صدى الآخر، فتصير القراءات خاطئة **بلا أي إنذار** — وهو أخطر
   أنواع الخطأ لأن الرقم يبدو سليماً. الدورة: **أمامي → أيمن → أيسر**
   بفاصل `US_ROUND_ROBIN_GAP_S` (≥60ms) ⇒ دورة ≈180ms (~5/ث).

⚠ **الحسّاس الأمامي له الأولوية**: طبقة السلامة قد تحتاج قراءة أمامية
   عاجلة، فلا تنتظر دورها في التناوب (`read_front_now`).

⚠ **القراءة تُعلَن قديمة** فوق `US_STALE_AFTER_S` بدل تقديمها كأنها حيّة:
   قيمة عمرها ثانية على روبوت يسير 0.6 م/ث تعني خطأ 60سم. وهذا نفس درس
   `DistanceFilter.value` الذي أُصلح سابقاً (كان يُعيد آخر قيمة إلى الأبد
   بعد موت الحسّاس فتقرأ طبقة السلامة «طريق مفتوح» وهي عمياء).

⚠ **الميل الأمامي 10–15°**: الحسّاس الجانبي مائل ليرى الجدار قبل الوصول،
   فالمسافة المقاسة **ليست العمودية** — العمودية = المقاسة × cos(الميل).
   `perpendicular_cm()` تتولّى التحويل، ولا يُحسب ميل الجدار من الخام.

🔴 **معطّل خلف `SIDE_ULTRASONIC_ENABLED`**: عند False لا يُحجز أي منفذ
   جانبي ولا يتغيّر أي سلوك قائم — الحسّاس الأمامي يبقى على مساره المثبت
   في `UltrasonicReader` بلا لمس (قاعدة: لا تلمس ما يعمل).
"""
from __future__ import annotations

import math
import threading
import time

from pi.config import (
    ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO,
    US_RIGHT_TRIG_GPIO, US_RIGHT_ECHO_GPIO,
    US_LEFT_TRIG_GPIO, US_LEFT_ECHO_GPIO,
    US_ROUND_ROBIN_GAP_S, US_STALE_AFTER_S, US_FRONT_PRIORITY,
    US_SIDE_TILT_DEG, SIDE_ULTRASONIC_ENABLED,
)
from pi.sensors.ultrasonic import (
    DistanceFilter, QualityTracker, GroundEchoDetector,
    SPEED_CM_PER_US, ECHO_TIMEOUT_S, MAX_PLAUSIBLE_CM, MIN_PLAUSIBLE_CM,
)

try:
    import lgpio
    _LGPIO_OK = True
except Exception:                     # noqa: BLE001 — ويندوز/تطوير
    _LGPIO_OK = False

FRONT, RIGHT, LEFT = "front", "right", "left"
# ترتيب التناوب — الأمامي أولاً لأنه حسّاس السلامة
ROBIN_ORDER = (FRONT, RIGHT, LEFT)

CHANNEL_PINS = {
    FRONT: (ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO),
    RIGHT: (US_RIGHT_TRIG_GPIO, US_RIGHT_ECHO_GPIO),
    LEFT:  (US_LEFT_TRIG_GPIO, US_LEFT_ECHO_GPIO),
}
# الجانبيان مائلان للأمام؛ الأمامي عمودي على مسار السير
CHANNEL_TILT_DEG = {FRONT: 0.0, RIGHT: US_SIDE_TILT_DEG, LEFT: US_SIDE_TILT_DEG}


def perpendicular_cm(measured_cm, tilt_deg: float):
    """
    المسافة **العمودية** على الجدار من قراءة حسّاس مائل.

    ⚠ خطأ سهل الوقوع: استعمال القراءة الخام كأنها المسافة العمودية يجعل
       كل حساب لميل الجدار **منحازاً بمقدار ثابت** — وهو انحياز لا يظهر
       في أي اختبار لأن القراءتين تحملانه معاً، فيبدو الميل صفراً دائماً.
    """
    if measured_cm is None:
        return None
    return float(measured_cm) * math.cos(math.radians(float(tilt_deg)))


class UltrasonicChannel:
    """
    قناة واحدة: حجز المنفذين + قياس **عند الطلب** (بلا خيط خاص بها).

    ⚠ بلا خيط عمداً: الخيوط المتوازية تُلغي التناوب من أصله. المجدوِل
       الواحد في `UltrasonicArray` هو من يقرّر متى تُطلق كل قناة.
    """

    def __init__(self, name: str, trig: int, echo: int, chip_handle=None):
        self.name = name
        self.trig, self.echo = int(trig), int(echo)
        self.ok = False
        self.error = None
        self.filter = DistanceFilter()
        self.quality = QualityTracker()
        self.ground_echo = GroundEchoDetector()
        self.raw_cm = None
        self.last_value_cm = None
        self.last_ts = 0.0            # لحظة آخر قراءة **صالحة**
        self.measurements = 0
        self.stuck = 0                # مرات وُجد ECHO مرتفعاً قبل التحفيز
        self._h = chip_handle
        self._cb = None
        self._st = {"rise": 0, "width": None}

    def claim(self, handle) -> bool:
        if not _LGPIO_OK:
            self.error = "lgpio غير مثبّت — محاكاة"
            return False
        try:
            lgpio.gpio_claim_output(handle, self.trig, 0)
            lgpio.gpio_claim_alert(handle, self.echo, lgpio.BOTH_EDGES)
            self._cb = lgpio.callback(handle, self.echo, lgpio.BOTH_EDGES,
                                      self._on_edge)
            self._h = handle
            self.ok = True
            return True
        except Exception as e:        # noqa: BLE001
            self.error = f"{self.name}: {e}"
            return False

    def _on_edge(self, chip, gpio, level, tick):
        if level == 1:
            self._st["rise"] = tick
        elif level == 0 and self._st["rise"]:
            self._st["width"] = tick - self._st["rise"]

    def measure(self, moving: bool = False):
        """نبضة واحدة + انتظار الصدى. يُحدّث المرشّح والطابع الزمني."""
        if not self.ok:
            return None
        # 🔴 حارس الانحشار (مقاس على العتاد 2026-08-06): HC-SR04 الذي ضاعت
        #    نبضته يُبقي ECHO مرتفعاً حتى ~200ms، وتحفيزه في هذه الحالة
        #    يعمّيه **نهائياً** — قناة اليمين ماتت 0/124 في التناوب بينما
        #    تعمل منفردةً بهذه البصمة بالضبط. ننتظر هبوطه بدل التحفيز فوقه.
        try:
            t_g = time.time()
            while lgpio.gpio_read(self._h, self.echo) == 1:
                if time.time() - t_g > 0.03:       # لم يهبط — دورة ضائعة معلَنة
                    self.stuck += 1
                    self.measurements += 1
                    self.quality.add(False)
                    return None
                time.sleep(0.002)
        except Exception:             # noqa: BLE001
            pass                      # تعذُّر القراءة لا يمنع محاولة القياس
        self._st["rise"], self._st["width"] = 0, None
        try:
            lgpio.gpio_write(self._h, self.trig, 1)
            time.sleep(0.00001)                     # نبضة تحفيز 10µs
            lgpio.gpio_write(self._h, self.trig, 0)
        except Exception:             # noqa: BLE001
            return None
        t0 = time.time()
        while self._st["width"] is None and (time.time() - t0) < ECHO_TIMEOUT_S:
            time.sleep(0.0005)
        raw = None
        if self._st["width"] is not None:
            cm = (self._st["width"] / 1000.0) * SPEED_CM_PER_US / 2.0
            # ⚠ دون MIN_PLAUSIBLE_CM = رنين المرسِل لا جسم (§ ultrasonic.py):
            #    قراءة 0.8سم شلّت مهمة كاملة بشبح عائق (2026-08-10).
            raw = (None if (cm < MIN_PLAUSIBLE_CM or cm > MAX_PLAUSIBLE_CM)
                   else cm)
        self.raw_cm = raw
        self.measurements += 1
        self.quality.add(raw is not None)
        self.filter.feed(raw)
        self.ground_echo.add(raw, moving=moving)
        v = self.filter.value
        if v is not None:
            self.last_value_cm = v
            self.last_ts = time.time()
        return v

    # ── القراءة مع **عمرها** ─────────────────────────────────────
    def value(self, max_age_s: float = US_STALE_AFTER_S):
        """المسافة الخام (سم) أو `None` إن غابت **أو قدُمت**."""
        if self.last_value_cm is None:
            return None
        if (time.time() - self.last_ts) > max_age_s:
            return None
        return round(self.last_value_cm, 1)

    def perpendicular(self, max_age_s: float = US_STALE_AFTER_S):
        """المسافة العمودية على الجدار (تصحيح ميل التركيب)."""
        return perpendicular_cm(self.value(max_age_s),
                                CHANNEL_TILT_DEG.get(self.name, 0.0))

    @property
    def age_s(self) -> float:
        return float("inf") if not self.last_ts else time.time() - self.last_ts

    @property
    def stale(self) -> bool:
        return self.age_s > US_STALE_AFTER_S

    def state(self) -> dict:
        return {"name": self.name, "ok": self.ok, "error": self.error,
                "trig": self.trig, "echo": self.echo,
                "distance_cm": self.value(), "raw_cm": self.raw_cm,
                "perpendicular_cm": (round(self.perpendicular(), 1)
                                     if self.perpendicular() is not None else None),
                "tilt_deg": CHANNEL_TILT_DEG.get(self.name, 0.0),
                "age_s": round(min(self.age_s, 999.0), 2), "stale": self.stale,
                "quality": self.quality.percent, "n": self.measurements,
                "stuck": self.stuck,
                "ground_echo": self.ground_echo.state()}

    def close(self):
        try:
            if self._cb:
                self._cb.cancel()
        except Exception:             # noqa: BLE001
            pass


class UltrasonicArray:
    """
    مجدوِل التناوب: خيط **واحد** يُطلق القنوات بالدور، ولا نبضتان معاً أبداً.

    ⚠ عند `SIDE_ULTRASONIC_ENABLED=False` لا يُحجز أي منفذ ولا يعمل أي خيط —
       صفر تغيير على السلوك القائم.
    """

    def __init__(self, enabled: bool = None, gap_s: float = US_ROUND_ROBIN_GAP_S):
        self.enabled = SIDE_ULTRASONIC_ENABLED if enabled is None else bool(enabled)
        self.gap_s = float(gap_s)
        self.ok = False
        self.error = None
        self.channels: dict[str, UltrasonicChannel] = {}
        self.moving_hint = False
        self.cycles = 0
        self.priority_reads = 0       # كم مرة سبق الأمامي دوره
        self._h = None
        self._stop = False
        self._lock = threading.Lock()  # يضمن **نبضة واحدة في كل لحظة**
        self._thread = None

        if not self.enabled:
            self.error = ("مصفوفة الألترا سونيك معطّلة "
                          "(SIDE_ULTRASONIC_ENABLED=False)")
            return
        if not _LGPIO_OK:
            self.error = "lgpio غير مثبّت — محاكاة"
            return
        for chip in (0, 4):           # باي 4 = gpiochip0 · باي 5 = gpiochip4
            try:
                self._h = lgpio.gpiochip_open(chip)
                break
            except Exception as e:    # noqa: BLE001
                self.error = str(e)
        if self._h is None:
            return
        for name in ROBIN_ORDER:
            trig, echo = CHANNEL_PINS[name]
            ch = UltrasonicChannel(name, trig, echo)
            if ch.claim(self._h):
                self.channels[name] = ch
            else:
                self.error = (self.error or "") + f" · {ch.error}"
        if self.channels:
            self.ok = True
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    # ── حلقة التناوب ─────────────────────────────────────────────
    def _run(self):
        while not self._stop:
            for name in ROBIN_ORDER:
                if self._stop:
                    return
                ch = self.channels.get(name)
                if ch is None:
                    continue
                with self._lock:      # 🔴 لا نبضتان معاً
                    ch.measure(moving=self.moving_hint)
                time.sleep(self.gap_s)
            self.cycles += 1

    def read_front_now(self):
        """
        قراءة أمامية **عاجلة** تسبق الدور — لطبقة السلامة.
        ⚠ تأخذ نفس القفل فلا تتداخل مع نبضة جارية: الأولوية **لا تعني**
           كسر قاعدة التناوب، بل تقديم الدور فقط.
        """
        ch = self.channels.get(FRONT)
        if ch is None or not US_FRONT_PRIORITY:
            return None
        with self._lock:
            self.priority_reads += 1
            return ch.measure(moving=self.moving_hint)

    # ── الواجهة للطبقات الأعلى ───────────────────────────────────
    def distance_cm(self, name: str):
        ch = self.channels.get(name)
        return None if ch is None else ch.value()

    def perpendicular_cm(self, name: str):
        ch = self.channels.get(name)
        return None if ch is None else ch.perpendicular()

    def state(self) -> dict:
        return {"enabled": self.enabled, "ok": self.ok, "error": self.error,
                "gap_s": self.gap_s, "cycles": self.cycles,
                "priority_reads": self.priority_reads,
                "stale_after_s": US_STALE_AFTER_S,
                "channels": {n: c.state() for n, c in self.channels.items()}}

    def close(self):
        self._stop = True
        for c in self.channels.values():
            c.close()
        try:
            if self._h is not None:
                lgpio.gpiochip_close(self._h)
        except Exception:             # noqa: BLE001
            pass
        self.ok = False
