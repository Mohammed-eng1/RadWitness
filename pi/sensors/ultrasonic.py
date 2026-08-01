# -*- coding: utf-8 -*-
"""
ultrasonic.py — قارئ مسافة منعَّم (البند أ)
===========================================
تحسينات على القياس المباشر داخل الحلقة (حجب + ضوضاء):

1. **خيط مستقل**: القياس يجري باستمرار في الخلفية ويحدّث قيمة مشتركة؛
   الحلقة الرئيسية تقرأ آخر قيمة **بلا انتظار** (القياس الواحد قد يستغرق
   60ms، ووسيط 5 ≈ 300ms — أبطأ بكثير من حلقة السلامة 80ms).
2. **مرشّح مركّب**: وسيط 5 قراءات (يقتل الشواذ) ثم EMA خفيف α≈0.4 (ينعّم).
3. **تصفية القفزات** محفوظة (>MAX_JUMP_CM تُتجاهل ما لم تتكرر)، لكنها
   **تُعطَّل أثناء المسح الدوراني** — هناك الفروق الكبيرة حقيقية لا أشباح.
4. **`quality`**: نسبة القراءات الناجحة في آخر ثانية — مؤشر تشخيصي؛ انخفاضه
   المستمر يعني مشكلة توصيل أو سطحاً ماصّاً للموجة.
5. خلفية **gpiozero.DistanceSensor** خلف علم `USE_GPIOZERO_DISTANCE` للمقارنة
   والتراجع، مع بقاء التطبيق المباشر (المثبت على العتاد) افتراضاً.

المنافذ من السكربت المثبت: TRIG=BCM23، ECHO=BCM24 (عبر مقسّم جهد 1kΩ/2kΩ).
"""
from __future__ import annotations

import threading
import time
from collections import deque

from pi.config import (
    ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO,
    ULTRASONIC_MEDIAN_N, ULTRASONIC_EMA_ALPHA, ULTRASONIC_QUALITY_WINDOW_S,
    USE_GPIOZERO_DISTANCE, MAX_JUMP_CM, ULTRASONIC_MAX_STALE,
)

try:
    import lgpio
    _LGPIO_OK = True
except Exception:                     # noqa: BLE001 — ويندوز/تطوير
    _LGPIO_OK = False

SPEED_CM_PER_US = 0.0343              # سرعة الصوت (~343 م/ث)
ECHO_TIMEOUT_S = 0.06                 # مهلة الصدى (~4م ذهاباً وإياباً)
MAX_PLAUSIBLE_CM = 450.0
_MEASURE_GAP_S = 0.06                 # فاصل يحتاجه HC-SR04 بين القياسات


def median(vals):
    s = sorted(v for v in vals if v is not None)
    return s[len(s) // 2] if s else None


class DistanceFilter:
    """
    المرشّح المركّب — **منطق خالص قابل للاختبار بلا عتاد**:
        وسيط N  →  EMA(α)  →  تصفية القفزات (اختيارية)
    """

    def __init__(self, median_n: int = ULTRASONIC_MEDIAN_N,
                 alpha: float = ULTRASONIC_EMA_ALPHA,
                 max_jump_cm: float = MAX_JUMP_CM,
                 max_stale: int = ULTRASONIC_MAX_STALE):
        self.median_n = median_n
        self.alpha = alpha
        self.max_jump = max_jump_cm
        self.max_stale = int(max_stale)
        self.jump_filter_enabled = True
        self._raw = deque(maxlen=median_n)
        self._ema = None
        self._last_accepted = None
        self._pending_jump = 0
        self.stale_reads = 0        # قراءات فاشلة متتابعة

    def feed(self, raw_cm):
        """يضيف قراءة خاماً ويُعيد القيمة المنعَّمة (أو None قبل توفّر بيانات)."""
        # ⚠ عدّاد التقادم: القيمة المنعَّمة لا تُقدَّم إلى ما لا نهاية بعد موت
        # الحسّاس — انظر `value` وتعليق ULTRASONIC_MAX_STALE في config.
        if raw_cm is None:
            self.stale_reads += 1
        else:
            self.stale_reads = 0
        self._raw.append(raw_cm)
        med = median(self._raw)
        if med is None:
            return self._ema

        # تصفية القفزات — تفترض استمرارية زمنية، فتُعطَّل أثناء المسح الدوراني
        if self.jump_filter_enabled and self._last_accepted is not None:
            if abs(med - self._last_accepted) > self.max_jump:
                self._pending_jump += 1
                if self._pending_jump < 2:          # شبح — تجاهل هذه المرة
                    return self._ema
            else:
                self._pending_jump = 0
        self._last_accepted = med

        # EMA خفيف فوق الوسيط
        self._ema = med if self._ema is None else (
            self._ema + self.alpha * (med - self._ema))
        return self._ema

    def reset(self):
        self._raw.clear()
        self._ema = None
        self._last_accepted = None
        self._pending_jump = 0

    @property
    def value(self):
        """
        ⚠ **None بعد max_stale فشلاً متتابعاً** — لا تُقدَّم قيمة قديمة كأنها
        حيّة. كانت تُعاد `self._ema` إلى الأبد بعد موت الحسّاس، فتقرأ طبقة
        السلامة «طريق مفتوح» وهي عمياء تماماً (شوهد على العتاد: جودة 0%
        ومسافة 165.7سم معروضة). None ينزل بالسرعة إلى SPEED_NO_READING.
        """
        if self.stale_reads >= self.max_stale:
            return None
        return self._ema


class QualityTracker:
    """نسبة القراءات الناجحة خلال آخر نافذة زمنية (تشخيص التوصيل/السطح)."""

    def __init__(self, window_s: float = ULTRASONIC_QUALITY_WINDOW_S):
        self.window_s = window_s
        self._events = deque()          # (ts, ok)

    def add(self, ok: bool):
        now = time.time()
        self._events.append((now, bool(ok)))
        while self._events and now - self._events[0][0] > self.window_s:
            self._events.popleft()

    @property
    def percent(self) -> int:
        if not self._events:
            return 0
        ok = sum(1 for _, o in self._events if o)
        return int(round(100.0 * ok / len(self._events)))


class UltrasonicReader:
    """قياس مستمر في الخلفية؛ `distance_cm` آخر قيمة منعَّمة (أو None)."""

    def __init__(self, trig: int = ULTRASONIC_TRIG_GPIO,
                 echo: int = ULTRASONIC_ECHO_GPIO,
                 use_gpiozero: bool = USE_GPIOZERO_DISTANCE):
        self.ok = False
        self.error = None
        self.backend = None
        self.filter = DistanceFilter()
        self.quality_tracker = QualityTracker()
        self.raw_cm = None
        self._trig, self._echo = trig, echo
        self._h = None
        self._cb = None
        self._gz = None
        self._st = {"rise": 0, "width": None}
        self._stop = False

        if use_gpiozero and self._init_gpiozero():
            self.backend = "gpiozero"
            self.ok = True
        elif self._init_lgpio():
            self.backend = "lgpio"
            self.ok = True
        if not self.ok:
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ── الخلفيات ────────────────────────────────────────────────
    def _init_gpiozero(self) -> bool:
        try:
            from gpiozero import DistanceSensor
            self._gz = DistanceSensor(echo=self._echo, trigger=self._trig,
                                      max_distance=MAX_PLAUSIBLE_CM / 100.0)
            return True
        except Exception as e:            # noqa: BLE001
            self.error = f"gpiozero غير متاح ({e}) — سقوط إلى lgpio"
            return False

    def _init_lgpio(self) -> bool:
        if not _LGPIO_OK:
            self.error = "lgpio غير مثبّت — محاكاة"
            return False
        for chip in (0, 4):               # باي 4 = gpiochip0، باي 5 = gpiochip4
            try:
                h = lgpio.gpiochip_open(chip)
            except Exception as e:        # noqa: BLE001
                self.error = str(e)
                continue
            try:
                lgpio.gpio_claim_output(h, self._trig, 0)
                lgpio.gpio_claim_alert(h, self._echo, lgpio.BOTH_EDGES)
                self._cb = lgpio.callback(h, self._echo, lgpio.BOTH_EDGES, self._on_edge)
                self._h = h
                return True
            except Exception as e:        # noqa: BLE001
                self.error = str(e)
                try:
                    lgpio.gpiochip_close(h)
                except Exception:         # noqa: BLE001
                    pass
        return False

    # ── القياس ──────────────────────────────────────────────────
    def _on_edge(self, chip, gpio, level, tick):
        if level == 1:
            self._st["rise"] = tick
        elif level == 0 and self._st["rise"]:
            self._st["width"] = tick - self._st["rise"]

    def _measure_once(self):
        if self.backend == "gpiozero":
            try:
                cm = self._gz.distance * 100.0
                return None if (cm <= 0 or cm > MAX_PLAUSIBLE_CM) else cm
            except Exception:             # noqa: BLE001
                return None
        self._st["rise"] = 0
        self._st["width"] = None
        lgpio.gpio_write(self._h, self._trig, 1)
        time.sleep(0.00001)               # نبضة تحفيز 10µs
        lgpio.gpio_write(self._h, self._trig, 0)
        t0 = time.time()
        while self._st["width"] is None and (time.time() - t0) < ECHO_TIMEOUT_S:
            time.sleep(0.0005)
        if self._st["width"] is None:
            return None
        cm = (self._st["width"] / 1000.0) * SPEED_CM_PER_US / 2.0   # ns→µs→سم
        return None if (cm <= 0 or cm > MAX_PLAUSIBLE_CM) else cm

    def _run(self):
        while not self._stop:
            raw = self._measure_once()
            self.raw_cm = raw
            self.quality_tracker.add(raw is not None)
            self.filter.feed(raw)
            time.sleep(_MEASURE_GAP_S)

    # ── الواجهة العامة ──────────────────────────────────────────
    @property
    def distance_cm(self):
        v = self.filter.value
        return None if v is None else round(v, 1)

    @property
    def quality(self) -> int:
        return self.quality_tracker.percent

    def set_jump_filter(self, enabled: bool) -> None:
        """يُعطَّل أثناء المسح الدوراني (الفروق بين الجهات حقيقية لا أشباح)."""
        self.filter.jump_filter_enabled = bool(enabled)

    def state(self) -> dict:
        return {"ok": self.ok, "error": self.error, "backend": self.backend,
                "distance_cm": self.distance_cm, "raw_cm": (
                    round(self.raw_cm, 1) if self.raw_cm is not None else None),
                "quality": self.quality}

    def close(self):
        self._stop = True
        try:
            if self._gz:
                self._gz.close()
            if self._cb:
                self._cb.cancel()
            if self._h is not None:
                lgpio.gpiochip_close(self._h)
        except Exception:                 # noqa: BLE001
            pass
