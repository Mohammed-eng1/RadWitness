# -*- coding: utf-8 -*-
"""
proximity.py — حساسات القرب الحقيقية (ألترا سونيك + IR الركنين)
================================================================
منقولة **حرفياً** من السكربتات المثبتة على العتاد:
    pi/tests/test_ultrasonic.py  →  TRIG=BCM23، ECHO=BCM24 (عبر مقسّم جهد)
    pi/tests/test_ir.py          →  IR أمام-يسار BCM25، أمام-يمين BCM16، **0 = عائق**

⚠ الألترا سونيك يُقاس في **خيط خلفي مستمر** لأن القياس الواحد قد يستغرق حتى
   60ms (مهلة الصدى)، ووسيط 3 قراءات = ~100ms — أبطأ من حلقة السلامة (80ms).
   فالحلقة تقرأ آخر قيمة مُصفّاة فوراً بلا حجب.

استيراد lgpio محميّ: غيابه (ويندوز) → وضع محاكاة بلا كسر.
"""
from __future__ import annotations

import threading
import time

from pi.config import (
    ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO,
    IR_FRONT_LEFT_GPIO, IR_FRONT_RIGHT_GPIO, IR_OBSTACLE_LEVEL,
    DISTANCE_SAMPLES,
)

try:
    import lgpio
    _LGPIO_OK = True
except Exception:                     # noqa: BLE001 — ويندوز/تطوير
    _LGPIO_OK = False

SPEED_CM_PER_US = 0.0343              # سرعة الصوت (~343 م/ث)
ECHO_TIMEOUT_S = 0.06                 # مهلة الصدى (~4م ذهاباً وإياباً)
MAX_PLAUSIBLE_CM = 450.0
_MEASURE_GAP_S = 0.06                 # فاصل بين القياسات (يحتاجه HC-SR04)


def _open_chip():
    """gpiochip0 (باي 4) أو gpiochip4 (باي 5)."""
    last = None
    for chip in (0, 4):
        try:
            return lgpio.gpiochip_open(chip), None
        except Exception as e:        # noqa: BLE001
            last = e
    return None, str(last)


class UltrasonicReader:
    """قياس مستمر في خيط خلفي؛ `distance_cm` آخر قيمة مصفّاة (أو None)."""

    def __init__(self, trig: int = ULTRASONIC_TRIG_GPIO, echo: int = ULTRASONIC_ECHO_GPIO):
        self.ok = False
        self.error = None
        self.distance_cm = None
        self._trig, self._echo = trig, echo
        self._h = None
        self._cb = None
        self._st = {"rise": 0, "width": None}
        self._stop = False

        if not _LGPIO_OK:
            self.error = "lgpio غير مثبّت — محاكاة"
            return
        h, err = _open_chip()
        if h is None:
            self.error = err
            return
        try:
            lgpio.gpio_claim_output(h, self._trig, 0)
            lgpio.gpio_claim_alert(h, self._echo, lgpio.BOTH_EDGES)
            self._cb = lgpio.callback(h, self._echo, lgpio.BOTH_EDGES, self._on_edge)
            self._h = h
            self.ok = True
        except Exception as e:        # noqa: BLE001
            self.error = str(e)
            try:
                lgpio.gpiochip_close(h)
            except Exception:         # noqa: BLE001
                pass
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _on_edge(self, chip, gpio, level, tick):
        if level == 1:
            self._st["rise"] = tick
        elif level == 0 and self._st["rise"]:
            self._st["width"] = tick - self._st["rise"]

    def _measure_once(self):
        self._st["rise"] = 0
        self._st["width"] = None
        lgpio.gpio_write(self._h, self._trig, 1)
        time.sleep(0.00001)                       # نبضة تحفيز 10µs
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
            samples = []
            for _ in range(DISTANCE_SAMPLES):
                d = self._measure_once()
                if d is not None:
                    samples.append(d)
                time.sleep(_MEASURE_GAP_S)
            if samples:                            # الوسيط يلغي القيمة الشاذّة
                samples.sort()
                self.distance_cm = round(samples[len(samples) // 2], 1)
            else:
                self.distance_cm = None

    def state(self) -> dict:
        return {"ok": self.ok, "error": self.error, "distance_cm": self.distance_cm}

    def close(self):
        self._stop = True
        try:
            if self._cb:
                self._cb.cancel()
            if self._h is not None:
                lgpio.gpiochip_close(self._h)
        except Exception:                          # noqa: BLE001
            pass


class IRReader:
    """قراءة فورية لحسّاسي IR في الركنين الأماميين (**0 = عائق**)."""

    def __init__(self, left: int = IR_FRONT_LEFT_GPIO, right: int = IR_FRONT_RIGHT_GPIO):
        self.ok = False
        self.error = None
        self._left, self._right = left, right
        self._h = None
        if not _LGPIO_OK:
            self.error = "lgpio غير مثبّت — محاكاة"
            return
        h, err = _open_chip()
        if h is None:
            self.error = err
            return
        try:
            # دخل بلا مقاومة (خرج المقارن مدفوع — مثبت على العتاد)
            lgpio.gpio_claim_input(h, self._left)
            lgpio.gpio_claim_input(h, self._right)
            self._h = h
            self.ok = True
        except Exception as e:        # noqa: BLE001
            self.error = str(e)
            try:
                lgpio.gpiochip_close(h)
            except Exception:         # noqa: BLE001
                pass

    def read(self):
        """(يسار، يمين) — 0 = عائق، 1 = خالٍ. عند غياب العتاد يُبلّغ خالياً."""
        if not self.ok:
            return 1, 1
        try:
            return (lgpio.gpio_read(self._h, self._left),
                    lgpio.gpio_read(self._h, self._right))
        except Exception:             # noqa: BLE001
            return 1, 1

    def state(self) -> dict:
        l, r = self.read()
        return {"ok": self.ok, "error": self.error, "left": l, "right": r}

    def close(self):
        try:
            if self._h is not None:
                lgpio.gpiochip_close(self._h)
        except Exception:             # noqa: BLE001
            pass
