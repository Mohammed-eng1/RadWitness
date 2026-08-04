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

# الألترا سونيك انتقل إلى وحدته المحسّنة (خيط + وسيط+EMA + quality)
from pi.sensors.ultrasonic import UltrasonicReader  # noqa: F401 — إعادة تصدير
from pi.config import (
    IR_FRONT_LEFT_GPIO, IR_FRONT_RIGHT_GPIO, IR_FRONT_MID_GPIO,
    IR_SIDE_LEFT_GPIO, IR_SIDE_RIGHT_GPIO, IR_OBSTACLE_LEVEL,
    IR_PRESENT, IR_PULL_UP,
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


# ترتيب ثابت للمواضع الخمسة (الاسم → ثابت المنفذ)
IR_PINS = (
    ("front_left",  IR_FRONT_LEFT_GPIO),
    ("front_right", IR_FRONT_RIGHT_GPIO),
    ("front_mid",   IR_FRONT_MID_GPIO),
    ("side_left",   IR_SIDE_LEFT_GPIO),
    ("side_right",  IR_SIDE_RIGHT_GPIO),
)


class IRReader:
    """
    قراءة فورية لحسّاسات IR الخمسة (**0 = عائق**).

    🔴 **الغائب يُعلَن مجهولاً (`None`) لا خالياً (`1`)**. الفرق ليس لفظياً:
    مع الشدّ المرتفع يقرأ المنفذ غير الموصول `1` — أي «الطريق خالٍ» — فلا
    يُميَّز سلك مقطوع عن حسّاس سليم بقراءة لحظية. وافتراض السلامة عند غياب
    الحسّاس هو بالضبط ما يقود إلى اصطدام. ولهذا:
      • ما أُعلن غائباً في `IR_PRESENT` **لا يُحجز منفذه ولا يُقرأ**.
      • وما فشل حجزه يُعلَن `None` كذلك.
    """

    def __init__(self, present: dict = None, pull_up: bool = IR_PULL_UP):
        self.ok = False
        self.error = None
        self.pull_up = bool(pull_up)
        self.present = dict(IR_PRESENT if present is None else present)
        self.claimed = {}             # الاسم → المنفذ (المحجوز فعلاً)
        self.skipped = [n for n, _ in IR_PINS if not self.present.get(n)]
        self._h = None
        if not _LGPIO_OK:
            self.error = "lgpio غير مثبّت — محاكاة"
            return
        if not any(self.present.get(n) for n, _ in IR_PINS):
            self.error = "كل حسّاسات IR معلَنة غائبة في IR_PRESENT"
            return
        h, err = _open_chip()
        if h is None:
            self.error = err
            return
        failed = []
        for name, pin in IR_PINS:
            if not self.present.get(name):
                continue              # ⚠ لا نحجز منفذاً معلَناً غائباً
            try:
                if self.pull_up:
                    lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_UP)
                else:
                    lgpio.gpio_claim_input(h, pin)
                self.claimed[name] = pin
            except Exception as e:    # noqa: BLE001
                failed.append(f"{name}(BCM{pin}): {e}")
        if self.claimed:
            self._h = h
            self.ok = True
        else:
            try:
                lgpio.gpiochip_close(h)
            except Exception:         # noqa: BLE001
                pass
        if failed:
            self.error = " · ".join(failed)

    def read_all(self) -> dict:
        """{الاسم: 0/1/None} — `None` = **مجهول** (غائب أو تعذّرت قراءته)."""
        out = {}
        for name, _ in IR_PINS:
            pin = self.claimed.get(name)
            if pin is None or not self.ok:
                out[name] = None
                continue
            try:
                out[name] = lgpio.gpio_read(self._h, pin)
            except Exception:         # noqa: BLE001
                out[name] = None

        return out

    def read(self):
        """
        (يسار، يمين) — توافق مع النداءات القائمة.
        ⚠ يُعيد `None` للغائب بدل `1`: الطبقات الأعلى تفرّق بين «خالٍ» و«مجهول».
        """
        v = self.read_all()
        return v["front_left"], v["front_right"]

    def state(self) -> dict:
        v = self.read_all()
        return {"ok": self.ok, "error": self.error, "pull_up": self.pull_up,
                "present": self.present, "skipped": self.skipped,
                "pins": {n: p for n, p in IR_PINS},
                "values": v, "left": v["front_left"], "right": v["front_right"],
                "n_present": len(self.claimed)}

    def close(self):
        try:
            if self._h is not None:
                lgpio.gpiochip_close(self._h)
        except Exception:             # noqa: BLE001
            pass
