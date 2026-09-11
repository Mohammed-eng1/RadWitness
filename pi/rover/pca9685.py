# -*- coding: utf-8 -*-
"""
pca9685.py — سائق PCA9685 مُضمَّن (هيكل Freenove 4WD)
=====================================================
سائق صغير مكتفٍ بذاته لمولّد PWM من 16 قناة عبر I2C.

🔴 **لماذا مُضمَّن ولا يُستورد من `reference/`**: مشروع `radbot` يستورد
`motor`/`servo` من مجلّد `reference/` وهو **مستبعد من git** (read-only).
نقل ذلك النمط إلى هنا يكسر البند 7 («المشروع يعمل كاملاً على ويندوز بلا
عتاد») ويجعل البناء غير قابل لإعادة الإنتاج: نسخة تعمل على راسبري واحد
لأن فيه مجلّداً غير مُتتبَّع. السائق كله ~30 سطراً فعلياً، فالتبعية أغلى
من الكود.

⚠ الاستيراد الثقيل كسول (البند 7): `smbus2` يُحمَّل عند أول فتح ناقل لا
عند استيراد الوحدة، فلا يكلّف ويندوز شيئاً.

المرجع: ورقة بيانات NXP PCA9685 — المذبذب الداخلي 25MHz، والعدّاد 12 بت
(4096 عدّة لكل دورة).
"""
from __future__ import annotations

import logging
import math

logger = logging.getLogger(__name__)

# ── السجلات (من ورقة البيانات) ──────────────────────────────────
MODE1        = 0x00
PRESCALE     = 0xFE
LED0_ON_L    = 0x06          # كل قناة 4 بايتات متتالية: ON_L/ON_H/OFF_L/OFF_H
ALLLED_ON_L  = 0xFA

MODE1_RESTART = 0x80
MODE1_SLEEP   = 0x10
MODE1_AI      = 0x20         # الزيادة التلقائية للعنوان (كتابة 4 بايتات دفعة)

OSC_HZ   = 25_000_000.0
COUNTS   = 4096              # 12 بت — ⚠ لا التفاف عددي هنا خلافاً لـWave Rover
DUTY_MAX = COUNTS - 1        # 4095


class PCA9685:
    """
    سائق PWM. يرفع استثناءً عند فشل الفتح (المتصل يقرّر السقوط للمحاكاة)،
    لكن `set_duty` **لا يرفع أبداً** — انظر البند 6.3: استثناء في مسار
    الحركة يقتل `finally: stop()` نفسه فتضيع «الإيقاز المضمون».
    """

    def __init__(self, bus: int = 1, address: int = 0x40, freq_hz: float = 50.0):
        import smbus2                      # كسول عمداً (البند 7)
        self.address = int(address)
        self.bus_num = int(bus)
        self._bus = smbus2.SMBus(self.bus_num)
        self.ok = True
        self.error = None
        self.write_errors = 0
        self._write(MODE1, MODE1_AI)       # تصفير + زيادة تلقائية
        self.set_freq(freq_hz)

    # ── أوّليات الناقل ──────────────────────────────────────────
    def _write(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(self.address, reg, value & 0xFF)

    def _read(self, reg: int) -> int:
        return self._bus.read_byte_data(self.address, reg)

    def set_freq(self, freq_hz: float) -> None:
        """
        يضبط تردّد PWM. يستلزم إدخال الشريحة في السُّبات أثناء كتابة
        PRESCALE (قيد في ورقة البيانات) ثم إعادة تشغيلها.

        ⚠ تردّد Freenove الافتراضي 50Hz منخفض لمحركات DC (أنين مسموع
        وخشونة عند القوى الصغيرة — وهي بالضبط منطقة TURN_MIN_POWER).
        قابل للرفع من config بعد استقرار النقل، لا أثناءه.
        """
        prescale = int(math.floor(OSC_HZ / COUNTS / float(freq_hz) - 1.0 + 0.5))
        prescale = max(3, min(255, prescale))      # حدّ الشريحة
        old = self._read(MODE1)
        self._write(MODE1, (old & ~MODE1_RESTART) | MODE1_SLEEP)
        self._write(PRESCALE, prescale)
        self._write(MODE1, old)
        # إعادة التشغيل تحتاج ≥500µs بعد رفع السُّبات (ورقة البيانات)
        import time
        time.sleep(0.005)
        self._write(MODE1, old | MODE1_RESTART | MODE1_AI)
        self.freq_hz = float(freq_hz)

    # ── القنوات ────────────────────────────────────────────────
    def set_duty(self, channel: int, duty: int) -> bool:
        """
        يضبط نسبة عمل قناة (0..4095). يعيد True عند النجاح.

        ⚠⚠ **لا يرفع استثناءً** (البند 6.3): عثرة I2C أثناء الحركة كانت
        ستنتشر من `motors()` إلى `_turn_segment` ثم تُفجّر
        `finally: self.stop()` نفسه لأنه يسلك نفس المسار — فيضيع الإيقاف
        المضمون في اللحظة الوحيدة التي وُجد لأجلها. الخطأ يصير **حالة
        معلنة** (`ok`) تقرأها الطبقة الأعلى.
        """
        d = max(0, min(DUTY_MAX, int(duty)))
        base = LED0_ON_L + 4 * int(channel)
        try:
            # ON=0 دائماً (نبضة تبدأ من رأس الدورة)، وOFF=duty
            self._bus.write_i2c_block_data(
                self.address, base, [0x00, 0x00, d & 0xFF, (d >> 8) & 0x0F])
            if not self.ok:
                self.ok, self.error = True, None
            return True
        except Exception as e:               # noqa: BLE001
            self.write_errors += 1
            self.ok, self.error = False, str(e)
            return False

    def all_off(self) -> None:
        """يطفئ كل القنوات — مسار الإغلاق الآمن."""
        try:
            self._bus.write_i2c_block_data(
                self.address, ALLLED_ON_L, [0x00, 0x00, 0x00, 0x10])
        except Exception:                    # noqa: BLE001
            pass

    def close(self) -> None:
        self.all_off()
        try:
            self._bus.close()
        except Exception:                    # noqa: BLE001
            pass
