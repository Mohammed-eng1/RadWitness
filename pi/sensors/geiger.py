# -*- coding: utf-8 -*-
"""
geiger.py — قارئ عداد جيجر (منقول حرفياً من pi/tests/test_geiger.py المثبت)
==========================================================================
lgpio، BCM17، RISING_EDGE، SET_PULL_NONE (عائم). عدّ عبر tally في خيط lgpio،
نافذة 30ث منزلقة + EMA سريع، تصحيح زمن ميت، تحويل لـµSv/h بمعايرة Cs-137.
القيم من config.py — لا تُغيَّر.
"""
import time
from collections import deque

from pi.config import (
    GEIGER_GPIO, CPM_PER_USVH, DEADTIME_TAU_S,
    GEIGER_WINDOW_S, GEIGER_EMA_ALPHA, HIGH_RATE_WARNING_CPM, GEIGER_PRESENT,
    GEIGER_BG_FLOOR_CPM, GEIGER_SILENCE_WARN_S, GEIGER_SILENCE_FAULT_S,
)
import math

try:
    import lgpio
    _LGPIO_OK = True
except Exception:                     # noqa: BLE001 — غير متوفر خارج الراسبري
    _LGPIO_OK = False


def _correct_dead_time(cpm_meas: float) -> float:
    """CPM_true = CPM_meas / (1 − CPM_meas·τ/60) — مثبت مخبرياً."""
    d = 1.0 - cpm_meas * DEADTIME_TAU_S / 60.0
    if d < 0.1:
        d = 0.1
    return cpm_meas / d


class GeigerReader:
    """يفتح خط الجيجر ويعدّ. sample() تُستدعى كل ثانية لدحرجة النافذة."""

    def __init__(self):
        self.ok = False
        self.error = None
        self.chip = None
        self._h = None
        self._cb = None
        self._per_sec = deque(maxlen=GEIGER_WINDOW_S)
        self._last_total = 0
        self._ema = 0.0
        self._cpm_raw = 0.0
        self._cpm = 0.0
        self._usvh = 0.0
        self._total = 0
        # 🔴 حارس الصمت: ثوانٍ مرّت **بلا أي عدّة منذ الإقلاع**
        self._silent_s = 0.0
        self._seen_any = False

        # 🔴 معلَن غائباً ⇒ لا يُحجز المنفذ ولا يُقرأ (منفذ طافٍ يعدّ ضجيجاً
        #    فيبدو إشعاعاً — وهو أخطر فشل ممكن في عدّاد إشعاع).
        if not GEIGER_PRESENT:
            self.error = "الجيجر معلَن غائباً (GEIGER_PRESENT=False)"
            return
        if not _LGPIO_OK:
            self.error = "lgpio غير مثبّت"
            return
        for chip in (0, 4):               # باي 4 = gpiochip0، باي 5 = gpiochip4
            try:
                h = lgpio.gpiochip_open(chip)
            except Exception as e:        # noqa: BLE001
                self.error = str(e)
                continue
            try:
                # ★ السطر المثبت على العتاد — عائم بلا مقاومة
                lgpio.gpio_claim_alert(h, GEIGER_GPIO, lgpio.RISING_EDGE, lgpio.SET_PULL_NONE)
                self._cb = lgpio.callback(h, GEIGER_GPIO, lgpio.RISING_EDGE)
                self._h = h
                self.chip = chip
                self.ok = True
                return
            except Exception as e:        # noqa: BLE001
                self.error = str(e)
                lgpio.gpiochip_close(h)

    def sample(self) -> None:
        """يُستدعى كل ثانية: يدحرج النافذة ويحدّث cpm_raw/cpm/usvh."""
        if not self.ok:
            return
        total = self._cb.tally()
        counts = total - self._last_total
        self._last_total = total
        self._total = total
        self._per_sec.append(counts)
        # 🔴 حارس «صفر عدّات»: أنبوب حيّ يرى الخلفية دائماً — والصمت الطويل
        #    عطل لا سلامة. يُقاس **منذ الإقلاع حتى أول عدّة** لا كنافذة
        #    منزلقة: نبضة واحدة تثبت الحياة، وبعدها يخصّ التقييمَ المستوى
        #    لا الوجود.
        if counts > 0:
            self._seen_any = True
        if not self._seen_any:
            self._silent_s += 1.0

        window_counts = sum(self._per_sec)
        window_min = len(self._per_sec) / 60.0
        self._cpm_raw = window_counts / window_min if window_min > 0 else 0.0
        self._ema += GEIGER_EMA_ALPHA * (counts * 60.0 - self._ema)
        self._cpm = _correct_dead_time(self._cpm_raw)     # المصحَّح يغذّي الجرعة
        self._usvh = self._cpm / CPM_PER_USVH

    def tally(self):
        """
        العدّ التراكمي **الخام واللحظي** (لا نافذة منزلقة)، أو None بلا عتاد.

        🔴 يلزم لقياس عدّات **فترة محدَّدة** بطرح قراءتين. وهذا ليس تحسيناً
        تجميلياً: `cpm` نافذته 30ث منزلقة، فاشتقاق «عدّات هذه الخلية» منها
        يخلط عدّ الخلية بعدّ العشر خلايا السابقة — أي **يلطّخ الإشارة مكانياً**
        وهي المعلومة الوحيدة التي تبني عليها طبقة تحديد المصدر تقديرها.
        """
        if not self.ok or self._cb is None:
            return None
        try:
            return int(self._cb.tally())
        except Exception:                 # noqa: BLE001
            return None

    @property
    def silence_p(self) -> float:
        """احتمال بواسون لهذا الصمت لو كان العدّاد سليماً: exp(−B·T/60)."""
        return math.exp(-GEIGER_BG_FLOOR_CPM * self._silent_s / 60.0)

    @property
    def counting(self):
        """
        🔴 **هل يعدّ العدّاد أصلاً؟** `True` رأى عدّة · `False` صمتٌ يتجاوز
        اليقين العملي · `None` **لم يُحسم بعد** (لم يمرّ زمن كافٍ).

        وهذا ليس تفصيلاً تجميلياً: بدونه يُصنَّف `usvh = 0.0` **Safe أخضر**،
        أي أن الجهاز يُعلن المكان آمناً **لأنه لا يقيس** — انقلابٌ كامل
        لمعنى القياس، ومخالفة صريحة للقاعدة 6.1 (الغائب مجهول لا شهادة).
        """
        if not self.ok:
            return False                  # لم يُحجز الخط أصلاً
        if self._seen_any:
            return True
        if self._silent_s >= GEIGER_SILENCE_FAULT_S:
            return False
        return None                       # لم يُحسم — لا تبرئة ولا اتهام

    def state(self) -> dict:
        counting = self.counting
        silent = round(self._silent_s, 1)
        note = None
        if counting is False and self.ok:
            note = (f"⛔ **صفر عدّات منذ {silent:.0f}ث** — أنبوب حيّ يرى "
                    f"الخلفية دائماً (احتمال هذا الصمت سليماً "
                    f"{self.silence_p:.1e}). افحص: جهد الأنبوب · دبوس "
                    f"BCM{GEIGER_GPIO} · السطر عائم بلا مقاومة رفع. "
                    f"🔴 الجرعة صفر هنا تعني **لا قياس** لا «آمن».")
        elif counting is None and self._silent_s >= GEIGER_SILENCE_WARN_S:
            note = (f"⚠ لا عدّات منذ {silent:.0f}ث (احتمال "
                    f"{self.silence_p:.1e}) — يُحسم عند "
                    f"{GEIGER_SILENCE_FAULT_S:.0f}ث")
        return {
            "ok": self.ok,
            "error": self.error,          # يُعرض في الواجهة عند فشل حجز الخط
            "samples": len(self._per_sec),  # ثوانٍ مُجمّعة (النافذة تمتلئ عند 30)
            "cpm_raw": round(self._cpm_raw, 1),
            "cpm": round(self._cpm, 1),
            "usvh": round(self._usvh, 3),
            "ema": round(self._ema, 0),
            "total": self._total,
            "high_rate": self._cpm_raw > HIGH_RATE_WARNING_CPM,
            # 🔴 «يعدّ أم لا» جزء من الحالة — والصفر بلا هذا الحقل كذبة
            "counting": counting,
            "silent_s": silent,
            "silence_p": self.silence_p if not self._seen_any else None,
            "counting_note": note,
        }

    def close(self) -> None:
        try:
            if self._cb:
                self._cb.cancel()
            if self._h is not None:
                lgpio.gpiochip_close(self._h)
        except Exception:                 # noqa: BLE001
            pass
