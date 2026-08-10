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
    ULTRASONIC_MIN_PERIOD_S, ULTRASONIC_PRESENT,
)

try:
    import lgpio
    _LGPIO_OK = True
except Exception:                     # noqa: BLE001 — ويندوز/تطوير
    _LGPIO_OK = False

SPEED_CM_PER_US = 0.0343              # سرعة الصوت (~343 م/ث)
ECHO_TIMEOUT_S = 0.06                 # مهلة الصدى (~4م ذهاباً وإياباً)
MAX_PLAUSIBLE_CM = 450.0
# 🔴 **أدنى مدى فيزيائي لـHC-SR04 ≈ 2سم** — وما دونه ليس جسماً قريباً بل
#    رنين المرسِل نفسه يُقرأ صدىً (أو تسرّب كهربائي على خط ECHO). عطل مقاس
#    (2026-08-10): الأمامي أعطى **0.8سم ثابتة** والطريق خالٍ وIR الثلاثة
#    خضراء، فرأت طبقة السلامة «عائقاً على 1سم» ورفضت كل حركة — فأُجهضت
#    المهمة من الخلية الأولى وعُلّمت خمس خلايا «غير قابلة للوصول» زوراً.
#    ⚠ القراءة دون هذا الحدّ تصير **مجهولة** (None) لا «عائقاً»: المجهول
#    يُبقي سلّم السلامة على درجة «احترس» ويترك المدى القريب لـIR (2–30سم)
#    وهو الحسّاس المؤهَّل له أصلاً — أما اعتبارها عائقاً فيشلّ الروبوت
#    بشبح لا وجود له.
MIN_PLAUSIBLE_CM = 2.5
_MEASURE_GAP_S = 0.06                 # فاصل يحتاجه HC-SR04 بين القياسات


def _plausible(cm) -> bool:
    """هل القراءة داخل المدى الفيزيائي للحسّاس؟ (خارجه = مجهول لا قيمة)."""
    return cm is not None and MIN_PLAUSIBLE_CM <= float(cm) <= MAX_PLAUSIBLE_CM


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


class GroundEchoDetector:
    """
    🔴 كاشف **تشخيصي** لصدى الأرض — يُعلن ولا يعالج.

    العلّة (مقاسة 2026-08-01): تثبيت منخفض يجعل الحافة السفلى لمخروط الشعاع
    (±7.5°) تصطدم بالأرض، فتُقرأ **21سم ثابتة والروبوت ساكن ولا شيء أمامه**.
    التوقيع المميّز **عنقودان ضيّقان** (21سم و182سم بنسبة 84%/15%) — لا تشتّت
    عشوائي كالضجيج الكهربائي. والأثر: سلّم السلامة يتشنّج بين «توقف» و«طريق
    مفتوح»، فينزلق الروبوت وتُعلَّم خلايا سليمة **محجوبة** خطأً.

    ⚠⚠ **لا يُعالَج برمجياً ولا يُضاف مرشّح يتجاهل ما دون 30سم**: ذلك يُعمي
    طبقة السلامة عن العوائق القريبة الحقيقية — وهي أخطر ما تحرسه. الحلّ
    **تثبيت لا كود**، ووظيفة هذا الكاشف أن يقول ذلك بدل تركه يُخمَّن.

    المرجع: حسّاس سليم على هدف ثابت يعطي تشتتاً ~1.1سم.
    """

    def __init__(self, window: int = 60, min_samples: int = 30,
                 gap_cm: float = 40.0, cluster_cm: float = 8.0,
                 min_share: float = 0.15):
        self.window = int(window)
        self.min_samples = int(min_samples)
        self.gap_cm = float(gap_cm)
        self.cluster_cm = float(cluster_cm)
        self.min_share = float(min_share)
        self._vals = deque(maxlen=self.window)
        self.suspect = False
        self.detail = None

    def add(self, cm, moving: bool = False) -> None:
        """⚠ تُهمَل العيّنات أثناء الحركة: تغيّر المسافة حينها **حقيقي**."""
        if moving:
            self._vals.clear()
            self.suspect, self.detail = False, None
            return
        if cm is not None:
            self._vals.append(float(cm))
        self._evaluate()

    def _evaluate(self) -> None:
        if len(self._vals) < self.min_samples:
            return
        v = sorted(self._vals)
        # أوسع فجوة بين قيمتين متتاليتين تفصل العنقودين
        gi, gap = 0, 0.0
        for i in range(1, len(v)):
            d = v[i] - v[i - 1]
            if d > gap:
                gap, gi = d, i
        if gap < self.gap_cm:
            self.suspect, self.detail = False, None
            return
        lo, hi = v[:gi], v[gi:]
        share = min(len(lo), len(hi)) / len(v)
        lo_w = (max(lo) - min(lo)) if lo else 0.0
        hi_w = (max(hi) - min(hi)) if hi else 0.0
        # عنقودان **ضيّقان** متباعدان، وأصغرهما ذو حصة معتبرة
        if share >= self.min_share and lo_w <= self.cluster_cm and hi_w <= self.cluster_cm:
            self.suspect = True
            self.detail = (
                f"⚠ **اشتباه صدى أرضي — راجع ارتفاع تثبيت الألترا سونيك**: "
                f"عنقودان ضيّقان والروبوت ساكن — "
                f"{sum(lo) / len(lo):.0f}سم ({100 * len(lo) / len(v):.0f}%) و"
                f"{sum(hi) / len(hi):.0f}سم ({100 * len(hi) / len(v):.0f}%). "
                f"حسّاس سليم يعطي تشتتاً ~1.1سم على هدف ثابت. "
                f"🔴 الحل **تثبيت لا كود** — لا تضف مرشّحاً يتجاهل ما دون 30سم.")
        else:
            self.suspect, self.detail = False, None

    def state(self) -> dict:
        return {"suspect": self.suspect, "detail": self.detail,
                "samples": len(self._vals)}


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
        self.ground_echo = GroundEchoDetector()
        self.moving_hint = False      # تُضبط من المهمة أثناء الحركة
        self.raw_cm = None
        self._trig, self._echo = trig, echo
        self._h = None
        self._cb = None
        self._gz = None
        self._st = {"rise": 0, "width": None}
        self._stop = False

        # 🔴 معلَن غائباً ⇒ **لا يُحجز المنفذ ولا يُقرأ**: منفذ طافٍ يعطي
        #    ضجيجاً أسوأ من غياب معلَن، والقيمة تبقى `None` = مجهولة.
        if not ULTRASONIC_PRESENT:
            self.error = "الألترا سونيك معلَن غائباً (ULTRASONIC_PRESENT=False)"
            return
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
                return None if not _plausible(cm) else cm
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
        return None if not _plausible(cm) else cm

    def _run(self):
        while not self._stop:
            t0 = time.time()
            raw = self._measure_once()
            self.raw_cm = raw
            self.quality_tracker.add(raw is not None)
            self.filter.feed(raw)
            self.ground_echo.add(raw, moving=self.moving_hint)
            # ⚠ **لا تنادِه أسرع من ~16 مرة/ث**: الصدى السابق يلوّث التالي،
            #    فيُنتج قراءات شبحية تبدو عوائق. الفاصل يُحسب من بدء القياس
            #    لا من نهايته حتى تبقى الدورة ثابتة مهما طال انتظار الصدى.
            time.sleep(max(0.0, ULTRASONIC_MIN_PERIOD_S - (time.time() - t0)))

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
                "present": ULTRASONIC_PRESENT,
                "distance_cm": self.distance_cm, "raw_cm": (
                    round(self.raw_cm, 1) if self.raw_cm is not None else None),
                "quality": self.quality,
                "ground_echo": self.ground_echo.state()}

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
