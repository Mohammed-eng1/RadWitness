# -*- coding: utf-8 -*-
"""
heading_hold.py — متحكم تثبيت الاتجاه (PD) للسير المستقيم
=========================================================
منطق **خالص** بلا عتاد ولا زمن: يأخذ خطأ الاتجاه بالدرجات و`dt` ويُعيد
التصحيح وقوّتي المحركين. لذلك يُختبر كاملاً على ويندوز، ويُستخدم من:
  - `pi/tests/calibrate_heading.py` (المرحلة 3: تثبيت KP/MAX_CORR).
  - المشي المستقيم في `DriveExecutor` لاحقاً — نفس المتحكم لا نسخة ثانية.

    correction = KP·error + KD·d(error)/dt        (مقصوص عند سقف مشتق)
    left  = base + trim_L + correction
    right = base + trim_R − correction

⚠ **إشارة التصحيح** (صُحّحت 2026-07-30 بعد انفلات مقاس على العتاد): كانت
   الصيغة معكوسة (`left − corr` و`right + corr`) فصارت الحلقة **تغذية راجعة
   موجبة**: خطأ موجب = «يجب اللفّ يميناً» فينتج تصحيحاً يجعل اليمين أسرع،
   والعجلة الأسرع تدفع نحو الجهة الأبطأ ⇒ يلفّ **يساراً** فيكبر الخطأ.
   المرجع المؤكَّد على العتاد: `bridge.turn("R")` = `_drive(+p, −p)` أي
   **اليسار أسرع ⇒ لفّ يميناً**، فالتصحيح الموجب يجب أن يزيد اليسار.
   العرَض المقاس: كلما ارتفع KP ساء الأداء رتيباً (1.9 → 34 → 415 سم/م)،
   وشوط KP=0.035 دار حول نفسه ~180° وقطع 0.20م فقط في 4ث.

⚠ سقف التصحيح **يُشتق من حدّ القوة** لا يُختار: لو تجاوز
   `base + |trim| + correction` الحاجز 0.5 لشبع محرك عند الحاجز بينما نزل
   الآخر → كسب غير متماثل ولا خطي في أحرج لحظة. المتحكم يقصّ التصحيح على
   الفراغ المتاح فعلاً (`headroom`) ويعدّ مرات الإشباع للتشخيص.
   (سكربت العتاد docs/ff.py استخدم سقف 0.25 مع أساس 0.35 = 0.628 > 0.5،
    فالزائد كان يُقصّ في `motors()` صامتاً ولم يُطبَّق أصلاً.)
"""
from __future__ import annotations

from pi.config import (
    HEADING_KP, HEADING_KD, HEADING_MAX_CORR, MAX_MOTOR_POWER, MIN_MOTOR_POWER,
    MOTOR_TRIM_L, MOTOR_TRIM_R, STRAIGHT_BASE_POWER,
)


def signed_error(current_deg: float, target_deg: float) -> float:
    """خطأ الاتجاه بإشارة، ملفوف إلى [−180, +180] (موجب = يجب اللفّ يميناً)."""
    return (float(target_deg) - float(current_deg) + 180.0) % 360.0 - 180.0


def available_headroom(base_power: float,
                       trim_l: float = MOTOR_TRIM_L,
                       trim_r: float = MOTOR_TRIM_R,
                       max_power: float = MAX_MOTOR_POWER,
                       min_power: float = MIN_MOTOR_POWER) -> float:
    """
    أقصى تصحيح يمكن تطبيقه **متماثلاً** عند قوة أساس معطاة. القيد **من طرفين**
    لأن التصحيح يرفع محركاً ويخفض الآخر بنفس المقدار:

        سقف علوي = MAX_MOTOR_POWER − base − |trim|   (لا يتجاوز حاجز 0.5)
        سقف سفلي = base − |trim| − MIN_MOTOR_POWER   (لا ينزل محرك تحت 0.1)
        headroom = min(الاثنين)

    ⚠ الطرف السفلي كان **مفقوداً** (صُحّح 2026-07-30): بأساس 0.30 وسقف 0.17
       ينزل أحد المحركين إلى 0.102 — عند حافة المدى الموثّق (0.1–0.5) أو
       تحته، فيزحف أو يقف بينما الآخر عند 0.498 ⇒ **دوران بالمكان لا سير
       مصحَّح**. (لا يقلّ عن صفر: أساس ملامس لأحد الحدّين لا يترك فراغاً.)
    """
    upper = float(max_power) - float(base_power) \
        - max(abs(float(trim_l)), abs(float(trim_r)))
    lower = float(base_power) - max(abs(float(trim_l)), abs(float(trim_r))) \
        - float(min_power)
    return max(0.0, min(upper, lower))


class HeadingController:
    """متحكم PD لتثبيت الاتجاه. `reset()` قبل كل شوط سير."""

    def __init__(self, kp: float = HEADING_KP, kd: float = HEADING_KD,
                 max_corr: float = HEADING_MAX_CORR,
                 trim_l: float = MOTOR_TRIM_L, trim_r: float = MOTOR_TRIM_R):
        self.kp = float(kp)
        self.kd = float(kd)
        self.max_corr = float(max_corr)
        self.trim_l = float(trim_l)
        self.trim_r = float(trim_r)
        self.reset()

    def reset(self) -> None:
        self._last_error = 0.0
        self._has_last = False
        self.saturated = 0          # مرات بلوغ السقف (سقف صغير جداً؟)
        self.headroom_clipped = 0   # مرات تقييد السقف بالفراغ المتاح
        self.samples = 0
        self.abs_error_sum = 0.0
        self.max_abs_error = 0.0
        self.sign_changes = 0       # مؤشر تذبذب: كثرتها = KP مرتفع
        self._last_sign = 0
        self.last_correction = 0.0

    # ── التصحيح الخام (بلا معرفة بالقوة) ────────────────────────
    def correction(self, error_deg: float, dt: float,
                   limit: float = None) -> float:
        e = float(error_deg)
        d = 0.0
        if self._has_last and dt and dt > 0:
            d = (e - self._last_error) / float(dt)
        self._last_error = e
        self._has_last = True

        raw = self.kp * e + self.kd * d
        cap = self.max_corr if limit is None else min(self.max_corr, float(limit))
        corr = max(-cap, min(cap, raw))
        if corr != raw:
            self.saturated += 1

        # إحصاءات الضبط (تُقرأ في سكربت المعايرة والسجل)
        self.samples += 1
        self.abs_error_sum += abs(e)
        self.max_abs_error = max(self.max_abs_error, abs(e))
        sign = (1 if corr > 0 else (-1 if corr < 0 else 0))
        if sign and self._last_sign and sign != self._last_sign:
            self.sign_changes += 1
        if sign:
            self._last_sign = sign
        self.last_correction = corr
        return corr

    # ── قوّتا المحركين مع التصحيح ────────────────────────────────
    def wheels(self, error_deg: float, dt: float,
               base_power: float = STRAIGHT_BASE_POWER) -> dict:
        """
        يُعيد {"left","right","correction","headroom"} — كلها ≤ MAX_MOTOR_POWER
        بحكم قصّ التصحيح على الفراغ المتاح (لا اعتماد على قصّ الجسر).
        """
        head = available_headroom(base_power, self.trim_l, self.trim_r)
        if head < self.max_corr:
            self.headroom_clipped += 1
        corr = self.correction(error_deg, dt, limit=head)
        # ⚠ التصحيح الموجب (خطأ موجب = يجب اللفّ يميناً) يزيد **اليسار**:
        # العجلة الأسرع تدفع نحو الجهة الأبطأ. عكسُها يقلب الحلقة إلى تغذية
        # راجعة موجبة (انظر رأس الملف) — يحرسها فحص الانغلاق في selftest.
        left = base_power + self.trim_l + corr
        right = base_power + self.trim_r - corr
        return {"left": round(left, 4), "right": round(right, 4),
                "correction": round(corr, 4), "headroom": round(head, 4)}

    # ── ملخّص شوط واحد (يقيس السكربت عليه جودة KP) ───────────────
    def summary(self) -> dict:
        n = max(1, self.samples)
        return {"samples": self.samples,
                "mean_abs_error_deg": round(self.abs_error_sum / n, 2),
                "max_abs_error_deg": round(self.max_abs_error, 2),
                # آخر خطأ مقروء — به يُحدَّث اتجاه المهمة بعد الشوط **قياساً**
                # لا افتراضاً بأن التثبيت أغلق الخطأ تماماً.
                "final_error_deg": (round(self._last_error, 2)
                                    if self._has_last else None),
                "sign_changes": self.sign_changes,
                "saturated": self.saturated,
                "saturated_pct": round(100.0 * self.saturated / n, 1),
                "headroom_clipped": self.headroom_clipped,
                "kp": self.kp, "kd": self.kd, "max_corr": self.max_corr}


def config_sanity() -> dict:
    """
    فحص تماسك ثوابت config: هل السقف يتّسع داخل حدّ القوة عند أساس السير؟
    يُستدعى في `mission_readiness` (حاجب بدء على العتاد) وفي selftest وسكربت
    المعايرة — خطأ هنا يظهر كسلوك غير خطي في العتاد ويصعب تشخيصه لاحقاً.
    """
    head = available_headroom(STRAIGHT_BASE_POWER)
    ok = HEADING_MAX_CORR <= head + 1e-9
    return {
        "ok": ok, "headroom": round(head, 4), "max_corr": HEADING_MAX_CORR,
        "base": STRAIGHT_BASE_POWER, "limit": MAX_MOTOR_POWER,
        "reason": None if ok else (
            f"HEADING_MAX_CORR={HEADING_MAX_CORR} > الفراغ المتاح {head:.3f} عند "
            f"أساس {STRAIGHT_BASE_POWER} — سيشبع محرك عند الحاجز {MAX_MOTOR_POWER}"),
    }
