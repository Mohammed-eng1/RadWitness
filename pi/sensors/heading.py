# -*- coding: utf-8 -*-
"""
heading.py — **مصدر الاتجاه خلف واجهة واحدة** (البند 1)
=======================================================
قبل هذا الملف كان تكامل الجايرو مكتوباً داخل `WaveRoverBridge` مباشرة، فتبديل
الحسّاس يعني تعديل جسر الروفر. الآن كل مصدر اتجاه ينفّذ نفس الواجهة:

    src = make_heading_source()          # يقرأ pi.config.HEADING_SOURCE
    src.calibrate_bias(5.0)              # الروبوت **ساكن**
    d = src.update()                     # {"ok","dps","delta","heading",...}
    src.heading                          # درجة 0..360 (نسبي لمرجع الغرفة)
    src.total_deg                        # تراكمي غير ملفوف (للفّات والمقارنات)

المصادر:
| الاسم | الحسّاس | المعادلة |
|---|---|---|
| `bno055_gyro`   | BNO055 gz (I2C) | تكامل: `h += (gz−bias)·dt·scale` |
| `bno055_fusion` | BNO055 yaw (IMUPLUS) | فرق الزاوية المطلقة النسبية |
| `rover_gyro`    | T=126 gz (سيريال) | تكامل — **الحسّاس مات، للتوثيق** |
| `sim`           | جسر المحاكاة | تكامل من أمر الدوران الوهمي |

⚠ القاعدة الحاكمة باقية (CLAUDE.md §1): الاتجاه من **الجايرو** لا من البوصلة.
   `bno055_fusion` مسموح **فقط** لأن وضع IMUPLUS يُقصي المغنيتومتر كلياً؛ لو
   عاد الحسّاس إلى NDOF فالـyaw يورّث تأرجح المجال المعدني ويصير غير صالح.

⚠ لا فشل صامت: مصدر لا يقرأ (حسّاس مفقود/ميت) يُعلن `ok=False` مع سبب مقروء،
   ويُصعِّد التبديل التلقائي إلى مصدر بديل مع تسجيل السبب — لا يُرجِع أصفاراً
   تبدو «اتجاهاً ثابتاً» (وهو ما جعل جايرو الروفر الميت يوهم بأن كل شيء سليم).

⚠ **العطل ليس أبدياً**: كان `ok=False` طريقاً بلا رجعة — لحظة عابرة تقتل
   الملاحة حتى إعادة تشغيل العملية. وسبب «الصفر المضبوط» الأشيع في BNO055 هو
   عودة الشريحة إلى وضع CONFIG بعد إعادة تشغيل ذاتية (هبوط جهد عند إقلاع
   المحركات)، وهي حالة تُصلَح في ~150ms. فقبل إعلان الوفاة يُستدعى
   `attempt_recovery()` (محاولات محدودة بفاصل زمني)، والعطل يُعلَن **بحالة
   الشريحة المقروءة** لا بتخمين.
"""
from __future__ import annotations

import math
import time

from pi.config import (
    HEADING_SOURCE, GYRO_SCALE, BNO055_GYRO_SCALE, BNO055_GYRO_Z_SIGN,
    GYRO_BIAS_CALIB_S, GYRO_BIAS_MAX_STD, GYRO_BIAS_MAX_STD_BNO,
    HEADING_SPIKE_DPS, HEADING_LPF_ALPHA, HEADING_DEADBAND_DPS,
    HEADING_SPIKE_DPS_DRIVE, HEADING_SPIKE_DPS_TURN, HEADING_SPIKE_DPS_STEER,
    HEADING_LPF_ALPHA_DRIVE, HEADING_LPF_ALPHA_TURN, HEADING_LPF_ALPHA_STEER,
    BNO055_READ_PERIOD_S,
    HEADING_RECOVERY_ATTEMPTS, HEADING_RECOVERY_COOLDOWN_S,
)

# ── أطوار الحركة: عتبة القفزة والتنعيم يختلفان بينها (انظر config) ──
# `drive` (سير مستقيم/سكون): إشارة صغيرة → عتبة ضيقة وتنعيم قوي.
# `steer` (سير مع تصحيح خطأ زاوي): المتحكّم يدير الروبوت **عمداً** بمعدل
#          41–62°/ث عند الإشباع — بعتبة السير يرفض المرشّح دوران الروبوت
#          نفسه فيتجمّد التكامل ويبقى المتحكّم مشبعاً بلا انغلاق.
# `turn`  (دوران بالمكان)  : 40–60°/ث طبيعية → عتبة واسعة وتنعيم خفيف.
PHASES = {
    "drive": (HEADING_SPIKE_DPS_DRIVE, HEADING_LPF_ALPHA_DRIVE),
    "steer": (HEADING_SPIKE_DPS_STEER, HEADING_LPF_ALPHA_STEER),
    "turn":  (HEADING_SPIKE_DPS_TURN,  HEADING_LPF_ALPHA_TURN),
}

# أقصى فترة بين قراءتين تُقبل للتكامل: أكبر منها يعني توقّف الحلقة (تحميل
# معالج/حجب I2C) فالتكامل عليها يبتلع دوراناً لم نره → تُتجاهل بلا صمت.
MAX_INTEGRATION_DT_S = 0.5
MAX_STALE_READS = 5          # قراءات فاشلة متتابعة → المصدر غير سليم
ZERO_RUN_DEAD = 40           # قراءة صفر **مضبوط** متتابعة = حسّاس ميت لا سكون


# ═══ إحصاء متين لانحياز الجايرو (وسيط + استبعاد شواذ) ════════════
def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def _std(vals, center):
    if len(vals) < 2:
        return 0.0
    return math.sqrt(sum((v - center) ** 2 for v in vals) / len(vals))


def robust_bias(samples, max_std: float = GYRO_BIAS_MAX_STD) -> dict:
    """
    انحياز الجايرو بـ**الوسيط لا المتوسط**، مع استبعاد الشواذ خارج 3σ ثم
    إعادة الحساب. السبب المقاس: عينتان شاذتان من 320 رفعتا (max−min) إلى
    31.69، بينما الوسيط بعد التنقية أعطى انحيازاً 0.011 بانحراف 0.31.
    تُرفض المعايرة فقط إذا تجاوز الانحراف **بعد التنقية** max_std.
    """
    if not samples:
        return {"ok": False, "bias": 0.0, "std": 0.0, "n": 0,
                "rejected": 0, "reason": "لا عينات"}
    # ⚠ حارس الحسّاس الميت: صفر **مضبوط** في كل العينات ليس «سكوناً مثالياً»
    # بل حسّاس لا يرسل. بدونه ينجح الميت بأفضل درجة ممكنة (σ=0 ≤ أي حدّ)
    # ويمرّ إلى مرحلة تحرّك المحركات — وهو ما حدث فعلاً مع جايرو الروفر
    # الميت: انحياز 0.0000 وσ 0.0000 في 152 عينة قُبلت كمعايرة سليمة.
    if all(float(v) == 0.0 for v in samples):
        return {"ok": False, "bias": 0.0, "std": 0.0, "n": len(samples),
                "rejected": 0, "raw_std": 0.0,
                "reason": f"كل العينات ({len(samples)}) صفر مضبوط — "
                          f"الحسّاس لا يرسل شيئاً (ميت أو ناقل خاطئ؟)"}
    med0 = _median(samples)
    sd0 = _std(samples, med0)
    kept = [v for v in samples if abs(v - med0) <= 3.0 * sd0] if sd0 > 0 else list(samples)
    if not kept:
        kept = list(samples)
    bias = _median(kept)
    sd = _std(kept, bias)
    ok = sd <= max_std
    return {"ok": ok, "bias": bias, "std": sd, "n": len(kept),
            "rejected": len(samples) - len(kept), "raw_std": sd0,
            "reason": None if ok else f"تشتت {sd:.2f} > {max_std}"}


# ═══ تنقية معدل الدوران (منقولة من سكربت العتاد docs/ff.py) ══════
class RateConditioner:
    """
    ترتيب المراحل **مقصود ومجرَّب على العتاد**:
      1) طرح الانحياز.
      2) استبعاد القفزة (> spike_dps): ضجيج كهربائي من المحركات لا دوران.
      3) مرشّح تمرير منخفض (EMA) يمتصّ اهتزاز الهيكل.
      4) عتبة السكون **على القيمة المنعَّمة** — لو طُبّقت قبل التنعيم لمرّ
         الضجيج المنعَّم بعدها وتراكم في التكامل كانجراف وهمي.
    منطق خالص بلا عتاد → قابل للاختبار على ويندوز في selftest.
    """

    def __init__(self, spike_dps: float = HEADING_SPIKE_DPS,
                 alpha: float = HEADING_LPF_ALPHA,
                 deadband_dps: float = HEADING_DEADBAND_DPS):
        self.spike_dps = float(spike_dps)
        self.alpha = float(alpha)
        self.deadband_dps = float(deadband_dps)
        self.filtered = 0.0
        self.spikes = 0

    def reset(self) -> None:
        self.filtered = 0.0

    def feed(self, raw_dps: float, bias: float = 0.0) -> float:
        val = float(raw_dps) - float(bias)
        if abs(val) > self.spike_dps:
            self.spikes += 1
            # نُبقي آخر قيمة معقولة بدل حقن القفزة؛ وإن كانت هي نفسها كبيرة
            # فالأسلم صفر (لا نخترع دوراناً لم نتحقق منه).
            return self.filtered if abs(self.filtered) < self.spike_dps / 2.0 else 0.0
        self.filtered = self.alpha * val + (1.0 - self.alpha) * self.filtered
        if abs(self.filtered) < self.deadband_dps:
            return 0.0
        return self.filtered


# ═══ الواجهة الواحدة ═════════════════════════════════════════════
class HeadingSource:
    """
    الأساس المشترك. المشتقات تنفّذ إحدى الطريقتين فقط:
      - `_read_rate_dps()`  → مصدر تكاملي (جايرو).
      - `_read_angle_deg()` → مصدر زاوية مطلقة (دمج داخلي).
    """
    name = "base"
    kind = "rate"                     # "rate" (تكامل) أو "angle" (مطلق)
    needs_bias = True

    def __init__(self, scale: float = 1.0, max_bias_std: float = GYRO_BIAS_MAX_STD):
        self.scale = float(scale)
        self.max_bias_std = float(max_bias_std)
        self.ok = True
        self.error = None
        self.fallback_reason = None    # يعبّئه المصنع عند التبديل الاضطراري
        self.heading = 0.0            # 0..360 نسبي لمرجع الغرفة
        self.total_deg = 0.0          # تراكمي غير ملفوف (موجب = يميناً)
        self.bias = 0.0
        self.bias_calibrated = False
        self.bias_info = {}
        self.cond = RateConditioner()
        self.phase = "drive"
        self.last_rate_dps = 0.0
        self.samples = 0
        self.stale_reads = 0
        self._zero_run = 0
        self._last_ts = None
        self._last_angle = None
        self.skipped_dt = 0           # قراءات أُسقطت لفجوة زمنية كبيرة
        self.recoveries = 0           # مرات إحياء ناجحة للحسّاس
        self.recovery_attempts = 0    # محاولات الإحياء (تُستنفد فيُعلَن العطل)
        self.last_recovery = None
        self._last_recovery_ts = 0.0

    # ── تُنفَّذ في المشتقات ──────────────────────────────────────
    def _read_rate_dps(self):
        raise NotImplementedError

    def _read_angle_deg(self):
        raise NotImplementedError

    def _try_recover(self):
        """
        إحياء الحسّاس خلف هذا المصدر. المشتقات التي لا تملك حسّاساً قابلاً
        للإحياء تتركها كما هي → `None` = «لا سبيل للإحياء».
        """
        return None

    # ── الإحياء المحدود ─────────────────────────────────────────
    def rearm(self) -> None:
        """
        يعيد تسليح المصدر بعد إحياء **مؤكَّد** للحسّاس. لا يُستدعى إلا من
        `attempt_recovery` بعد قراءة تحقّق ناجحة — تسليحه بلا تحقّق يعيدنا
        إلى التظاهر بالسلامة.
        """
        self.ok = True
        self.error = None
        self.stale_reads = 0
        self._zero_run = 0
        self.cond.reset()
        self._last_ts = None          # لا تكامل على الفجوة الزمنية للإحياء
        self._last_angle = None

    def reset_recovery_budget(self) -> None:
        """
        يعيد رصيد محاولات الإحياء كاملاً — **بطلب صريح من المستخدم فقط**
        (زرّ «أعِد المحاولة» بعد إصلاح التغذية مثلاً). تصفيره تلقائياً يحوّل
        الحدّ إلى زينة ويعيد إغراق الناقل.
        """
        self.recovery_attempts = 0
        self._last_recovery_ts = 0.0

    def attempt_recovery(self) -> dict:
        """
        محاولة إحياء **محدودة العدد وبفاصل زمني**: الإغراق هنا يشلّ الناقل
        ويطيل كل دورة تحكّم. النتيجة: {"recovered", "detail"}.
        """
        now = time.time()
        if now - self._last_recovery_ts < HEADING_RECOVERY_COOLDOWN_S:
            return {"recovered": False, "detail": "محاولة إحياء قريبة جداً — انتظار"}
        if self.recovery_attempts >= HEADING_RECOVERY_ATTEMPTS:
            return {"recovered": False,
                    "detail": f"استُنفدت محاولات الإحياء ({HEADING_RECOVERY_ATTEMPTS})"}
        self._last_recovery_ts = now
        self.recovery_attempts += 1
        res = self._try_recover()
        if res is None:
            return {"recovered": False, "detail": "لا سبيل لإحياء هذا المصدر"}
        self.last_recovery = res
        if res.get("recovered"):
            self.recoveries += 1
            self.recovery_attempts = 0        # نجح → أعِد الرصيد كاملاً
            self.rearm()
        return res

    def _fault(self, reason: str) -> None:
        """
        يُعلن العطل **بعد** استنفاد الإحياء، ويضمّ تفصيل آخر محاولة إلى السبب
        (حالة الشريحة المقروءة) — رسالة تقود التشخيص لا تعمّمه.
        """
        detail = (self.last_recovery or {}).get("detail")
        self.ok = False
        self.error = f"{self.name}: {reason}" + (f" — {detail}" if detail else "")

    # ── واجهة الاستخدام ────────────────────────────────────────
    def set_phase(self, phase: str) -> str:
        """
        يبدّل عتبات التنقية بحسب الطور (`drive` / `turn`) ويصفّر حالة المرشّح
        — بقاء قيمة الطور السابق فيه يسرّب دوراناً قديماً إلى الطور الجديد.
        يُستدعى من الجسر عند بدء/انتهاء اللفّ. مصدر الزاوية المطلقة لا يتأثر.
        """
        spike, alpha = PHASES.get(phase, PHASES["drive"])
        self.cond.spike_dps = spike
        self.cond.alpha = alpha
        self.cond.reset()
        self.phase = phase if phase in PHASES else "drive"
        return self.phase

    def reset(self, value: float = 0.0) -> None:
        """تصفير الاتجاه على مرجع الغرفة (زر «صفّر الاتجاه»)."""
        self.heading = float(value) % 360.0
        self.total_deg = 0.0
        self.cond.reset()
        self._last_ts = None
        self._last_angle = None

    def calibrate_bias(self, seconds: float = GYRO_BIAS_CALIB_S) -> dict:
        """
        يقيس الانحياز والروبوت **ساكن** (وسيط + استبعاد شواذ).
        ⚠ لا يُثبَّت في الكود — يتغيّر بين التجارب فيُقاس عند بدء كل مهمة.
        مصدر الزاوية المطلقة لا يحتاج انحيازاً (الدمج الداخلي يتولّاه).
        """
        if not self.needs_bias:
            self.bias_calibrated = True
            self.bias_info = {"ok": True, "bias": 0.0, "std": 0.0, "n": 0,
                              "rejected": 0, "reason": "مصدر مطلق — لا يحتاج انحيازاً"}
            return self.bias_info
        samples = []
        deadline = time.time() + float(seconds)
        while time.time() < deadline:
            v = self._read_rate_dps()
            if v is not None:
                samples.append(float(v))
            time.sleep(BNO055_READ_PERIOD_S)
        info = robust_bias(samples, max_std=self.max_bias_std)
        self.bias_info = info
        if info["ok"]:
            self.bias = info["bias"]
            self.bias_calibrated = True
        else:
            self.bias_calibrated = False
        self.cond.reset()
        self._last_ts = None
        return info

    def update(self) -> dict:
        """قراءة واحدة → تحديث الاتجاه. تُستدعى في حلقات اللفّ والسير."""
        now = time.time()
        if self.kind == "angle":
            raw = self._read_angle_deg()
        else:
            raw = self._read_rate_dps()
        if raw is None:
            self.stale_reads += 1
            if self.stale_reads >= MAX_STALE_READS and self.ok:
                # ⚠ جرّب الإحياء قبل الإعلان: قد يكون انقطاعاً عابراً على الناقل
                if not self.attempt_recovery().get("recovered"):
                    self._fault(f"{self.stale_reads} قراءات فاشلة متتابعة — "
                                f"تحقّق من التوصيل")
            self._last_ts = now
            return {"ok": self.ok, "dps": 0.0, "delta": 0.0,
                    "heading": self.heading, "dt": 0.0, "error": self.error}
        self.stale_reads = 0
        self.samples += 1

        # صفر **مضبوط** متتابع: إمّا حسّاس ميت (كجايرو الروفر) وإمّا شريحة
        # BNO055 عادت إلى وضع CONFIG حيث تقرأ كل سجلات البيانات 0x00 وهي حيّة.
        # الفرق لا يُخمَّن — يُسأل عنه الحسّاس في `attempt_recovery`.
        if self.kind == "rate":
            if float(raw) == 0.0:
                self._zero_run += 1
                if self._zero_run >= ZERO_RUN_DEAD and self.ok:
                    if not self.attempt_recovery().get("recovered"):
                        self._fault(f"{self._zero_run} قراءة صفر مضبوط متتابعة — "
                                    f"الحسّاس لا يرسل شيئاً (ميت؟)")
            else:
                self._zero_run = 0

        last = self._last_ts
        self._last_ts = now
        dt = (now - last) if last is not None else 0.0
        delta = 0.0
        rate = 0.0

        if self.kind == "angle":
            prev = self._last_angle
            self._last_angle = float(raw)
            if prev is not None:
                # فرق ملفوف حول 360 (يمنع قفزة 359°→1° من أن تُقرأ −358)
                delta = ((float(raw) - prev + 180.0) % 360.0 - 180.0) * self.scale
                rate = (delta / dt) if dt > 0 else 0.0
        else:
            rate = self.cond.feed(float(raw), self.bias)
            if last is None or dt <= 0:
                delta = 0.0
            elif dt > MAX_INTEGRATION_DT_S:
                self.skipped_dt += 1        # فجوة كبيرة → لا تُكامل فراغاً
                delta = 0.0
            else:
                delta = rate * dt * self.scale

        self.last_rate_dps = rate
        self.total_deg += delta
        self.heading = (self.heading + delta) % 360.0
        return {"ok": self.ok, "dps": round(rate, 3), "delta": delta,
                "heading": self.heading, "dt": dt, "error": self.error}

    def state(self) -> dict:
        return {
            "source": self.name, "kind": self.kind, "ok": self.ok,
            "error": self.error, "heading": round(self.heading, 1),
            "total_deg": round(self.total_deg, 1), "scale": self.scale,
            "bias": round(self.bias, 4), "bias_calibrated": self.bias_calibrated,
            "bias_std": round(self.bias_info.get("std", 0.0), 3),
            "rate_dps": round(self.last_rate_dps, 2),
            "samples": self.samples, "spikes": self.cond.spikes,
            "stale_reads": self.stale_reads, "skipped_dt": self.skipped_dt,
            "deadband_dps": self.cond.deadband_dps, "phase": self.phase,
            # الإحياء مرئي: عدّاد يتصاعد = الحسّاس يُعاد تشغيله فعلياً أثناء
            # المهمة (تغذية غير مستقرة)، وليس مجرّد ضجيج قراءة.
            "recoveries": self.recoveries,
            "recovery_attempts": self.recovery_attempts,
            "last_recovery": (self.last_recovery or {}).get("detail"),
        }


# ═══ 1) BNO055 — تكامل الجايرو (الافتراضي) ═══════════════════════
class BNO055GyroHeading(HeadingSource):
    """
    `heading += (gz − bias) · dt · BNO055_GYRO_SCALE`
    نفس معادلة CLAUDE.md، بحسّاس حيّ بدل الميت. المعامل **لهذا الحسّاس**
    (لا تنسخ 0.9275 المقاس بجايرو الروفر — حسّاسان مختلفان).
    """
    name = "bno055_gyro"
    kind = "rate"

    def __init__(self, imu, scale: float = BNO055_GYRO_SCALE,
                 sign: int = BNO055_GYRO_Z_SIGN):
        super().__init__(scale=scale, max_bias_std=GYRO_BIAS_MAX_STD_BNO)
        self.imu = imu
        self.sign = 1 if sign >= 0 else -1
        if imu is None or not getattr(imu, "ok", False):
            self.ok = False
            self.error = f"BNO055 غير متاح: {getattr(imu, 'error', 'لا قارئ')}"

    def _read_rate_dps(self):
        v = self.imu.gyro_z_dps()
        return None if v is None else self.sign * v

    def _try_recover(self):
        """
        يفوّض الإحياء إلى قارئ BNO055 (يسأل الشريحة ثم يعيد التهيئة).
        قارئ لا يدعمه (حسّاس وهمي في الاختبارات) → None = لا إحياء.
        """
        rec = getattr(self.imu, "recover", None)
        return rec() if callable(rec) else None

    def state(self) -> dict:
        st = super().state()
        st["sign"] = self.sign
        st["mag_used"] = self.imu.state().get("mag_used") if self.imu else None
        return st


# ═══ 2) BNO055 — الدمج الداخلي بلا مغنيتومتر (IMUPLUS) ═══════════
class BNO055FusionHeading(HeadingSource):
    """
    yaw من دمج BNO055 الداخلي (100Hz) في وضع **IMUPLUS** — بلا مغنيتومتر.
    ميزته: التكامل يحدث داخل الحسّاس فلا يتأثر بتأخّر بايثون ولا بفجوات
    الحلقة. عيبه: زاوية **نسبية** تنجرف ببطء، ولا يمكن التحقق من وضعه إلا
    بالسؤال — لذلك يرفض العمل إن كان الوضع يستخدم المغنيتومتر (قاعدة §1).
    """
    name = "bno055_fusion"
    kind = "angle"
    needs_bias = False

    def __init__(self, imu):
        super().__init__(scale=1.0)
        self.imu = imu
        if imu is None or not getattr(imu, "ok", False):
            self.ok = False
            self.error = f"BNO055 غير متاح: {getattr(imu, 'error', 'لا قارئ')}"
        elif imu.state().get("mag_used"):
            self.ok = False
            self.error = ("BNO055 في وضع يدمج المغنيتومتر (NDOF) — yaw غير صالح "
                          "داخل هيكل معدني. فعّل BNO055_NO_MAG_MODE")

    def _read_angle_deg(self):
        return self.imu.euler_yaw()

    def _try_recover(self):
        rec = getattr(self.imu, "recover", None)
        return rec() if callable(rec) else None


# ═══ 3) جايرو الروفر عبر T=126 — **معطّل عتادياً** ════════════════
class RoverGyroHeading(HeadingSource):
    """
    المصدر الأصلي: gz من رسالة `T=126`. **IMU الروفر الداخلي مات** (I2C
    الداخلي معطّل) فيُبقى للتوثيق والتراجع لو استُبدل الهيكل. حارس الصفر
    المضبوط في `update()` يكشف موته بوضوح بدل التكامل على أصفار.
    """
    name = "rover_gyro"
    kind = "rate"

    def __init__(self, bridge, scale: float = GYRO_SCALE):
        super().__init__(scale=scale, max_bias_std=GYRO_BIAS_MAX_STD)
        self.bridge = bridge

    def _read_rate_dps(self):
        d = self.bridge.read_imu()
        if not isinstance(d, dict) or "gz" not in d:
            return None
        try:
            return float(d["gz"])
        except (TypeError, ValueError):
            return None


# ═══ 4) مصدر المحاكاة (ويندوز/بلا عتاد) ══════════════════════════
class SimHeading(RoverGyroHeading):
    """gz الوهمي من جسر المحاكاة — يجعل كل منطق اللفّ قابلاً للاختبار بلا عتاد."""
    name = "sim"


# ═══ المصنع: يختار المصدر ويعلن أي تبديل بسببه ════════════════════
def make_heading_source(bridge=None, imu=None, source: str = None) -> HeadingSource:
    """
    يبني المصدر المطلوب من `HEADING_SOURCE`. عند تعذّره **يسقط إلى بديل**
    ويكتب السبب في `fallback_reason` (لا فشل صامت ولا أصفار صامتة).

    - `bridge`: WaveRoverBridge (لمصادر السيريال والمحاكاة).
    - `imu`   : IMUReader؛ إن كان None يُطلب القارئ المشترك عند الحاجة فقط
                (حتى لا يُفتح I2C على ويندوز بلا سبب).
    """
    req = (source or HEADING_SOURCE or "").strip().lower()
    if req == "sim":
        return SimHeading(bridge)

    if req.startswith("bno055"):
        if imu is None:
            imu = _shared_imu()
        cls = BNO055FusionHeading if req == "bno055_fusion" else BNO055GyroHeading
        src = cls(imu)
        if src.ok:
            src.fallback_reason = None
            return src
        reason = src.error
        # BNO055 مفقود → لا نتظاهر بالسلامة: نسقط إلى المحاكاة/الروفر بسبب معلن
        alt = SimHeading(bridge) if getattr(bridge, "mode", "sim") != "real" \
            else RoverGyroHeading(bridge)
        alt.fallback_reason = f"⚠ {reason} → البديل: {alt.name}"
        return alt

    if req == "rover_gyro":
        src = RoverGyroHeading(bridge)
        src.fallback_reason = ("⚠ rover_gyro: جايرو الروفر الداخلي معطّل عتادياً — "
                               "المعايرات المأخوذة عبره باطلة")
        return src

    src = SimHeading(bridge)
    src.fallback_reason = f"⚠ مصدر اتجاه غير معروف «{req}» → مصدر محاكاة"
    return src


def _shared_imu():
    """القارئ المشترك — يُستورد متأخراً حتى لا يُفتح I2C إلا عند الطلب."""
    try:
        from pi.sensors.imu import get_imu
        return get_imu()
    except Exception as e:                    # noqa: BLE001
        class _Missing:
            ok = False
            error = str(e)

            def state(self):
                return {"ok": False, "error": self.error}
        return _Missing()
