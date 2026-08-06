# -*- coding: utf-8 -*-
"""
wall_heading.py — 🔴 تصحيح **الاتجاه** من الجدار الجانبي (منطق خالص)
======================================================================
**المرجع الاتجاهي المفقود.** كل ما في المشروع قبله كان يصحّح **الموضع**:
`try_front_wall_correction` يضبط x أو y من مسافة أمامية ولا يمسّ `heading`
إطلاقاً — بل **يفترضه صحيحاً** (بوابته الأولى محاذاة ±`WALL_ALIGN_TOL_DEG`).

⇒ **الحلقة المفرغة الموثّقة**: انحراف > 15° ⇒ يُرفض التصحيح ⇒ لا تصحيح
موضع ⇒ الانحراف يكبر ⇒ يُرفض أكثر. الأداة الوحيدة تتعطّل بالضبط حين تشتدّ
الحاجة إليها. ومع MPU-6050 بلا مرجع مطلق، الانحراف يتراكم بلا حدّ.

**المبدأ**: قراءتان متتاليتان للمسافة الجانبية أثناء السير المستقيم:

    المسافة ثابتة  → موازٍ للجدار  → الاتجاه سليم
    المسافة تزداد  → مائل مبتعداً  → يجب اللفّ نحو الجدار
    المسافة تنقص   → مائل مقترباً  → يجب اللفّ بعيداً عنه

    الميل = atan2(Δd, المسافة المقطوعة بين القراءتين)

⚠ **الإشارة تعتمد جهة الجدار**: جدار على اليمين وابتعاد ⇒ الروبوت انحرف
   **يساراً** ⇒ التصحيح موجب (يميناً). وعلى اليسار العكس تماماً. خطأ
   الإشارة هنا لا يُنتج تصحيحاً ضعيفاً بل **تغذية راجعة موجبة** تُبعد
   الروبوت عن الجدار بتسارع — ولهذا يُختبر الاتجاهان معاً.

🔴 **شروط أمان قبل أي تصحيح** (على غرار شروط الجدار الأمامي):
   المسافة داخل نطاق معقول · مسافة كافية بين القراءتين · الميل دون حدّ
   أقصى (فوقه عائق أو انعطاف جدار لا ميل) · و**عدة قراءات متسقة** —
   لا تصحيح من قراءة واحدة.

⚠ الوحدة **لا تلمس عتاداً ولا حالة**: تستقبل أرقاماً وتُخرج قراراً، فتُختبر
   كاملةً على ويندوز.
"""
from __future__ import annotations

import math
import time

from pi.config import (
    WALL_FOLLOW_MIN_CM, WALL_FOLLOW_MAX_CM, WALL_HEADING_MIN_TRAVEL_M,
    WALL_HEADING_MAX_TILT_DEG, WALL_HEADING_CONSISTENT_N,
    WALL_HEADING_MAX_CORR_DEG, WALL_HEADING_SIGMA_SHRINK,
    WALL_HEADING_SIGMA_FLOOR_DEG,
)

RIGHT, LEFT = "right", "left"

# أسباب الرفض — كلها معلَنة، ولا رفض صامت
OK = "ok"
R_RANGE = "out_of_range"
R_TRAVEL = "insufficient_travel"
R_TILT = "tilt_too_large"
R_INCONSISTENT = "not_consistent_yet"
R_NO_PREV = "no_previous_reading"


def tilt_from_delta(delta_cm: float, travel_m: float, side: str) -> float:
    """
    زاوية ميل الروبوت عن الجدار بالدرجات (موجب = يجب اللفّ يميناً لتصحيحه).

    `delta_cm` = المسافة الآن − المسافة السابقة (موجب = ابتعدنا عن الجدار).

    ⚠ الإشارة: جدار **يمين** وابتعاد ⇒ انحرفنا يساراً ⇒ التصحيح **يميناً**
       (موجب). جدار **يسار** وابتعاد ⇒ انحرفنا يميناً ⇒ التصحيح يساراً.
    """
    if travel_m <= 0:
        return 0.0
    ang = math.degrees(math.atan2(delta_cm / 100.0, float(travel_m)))
    return ang if side == RIGHT else -ang


class WallHeadingCorrector:
    """
    يتتبّع المسافة الجانبية ويُصدر تصحيح اتجاه حين تكفي الأدلة.

    الاستعمال: `feed(distance_cm, odom_m)` بعد كل قراءة جانبية أثناء السير،
    وتنفيذ `correction_deg` حين يعود `apply=True`.
    """

    def __init__(self, side: str = RIGHT,
                 consistent_n: int = WALL_HEADING_CONSISTENT_N,
                 max_corr_deg: float = WALL_HEADING_MAX_CORR_DEG):
        self.side = side if side in (RIGHT, LEFT) else RIGHT
        self.consistent_n = max(1, int(consistent_n))
        self.max_corr_deg = float(max_corr_deg)
        self._prev = None             # (distance_cm, odom_m)
        self._tilts: list[float] = []  # ميول متتالية متسقة الإشارة
        self.corrections: list[dict] = []
        self.rejects: dict[str, int] = {}

    def reset(self) -> None:
        """يُصفَّر بعد كل تصحيح وبعد أي لفّة (الجدار تغيّر أو الإطار تبدّل)."""
        self._prev = None
        self._tilts.clear()

    def _reject(self, reason: str, **extra) -> dict:
        self.rejects[reason] = self.rejects.get(reason, 0) + 1
        return {"apply": False, "reason": reason, "correction_deg": 0.0,
                "n_consistent": len(self._tilts), **extra}

    def feed(self, distance_cm, odom_m: float) -> dict:
        """
        قراءة جانبية جديدة عند مسافة أودومترية `odom_m`.
        يُعيد قراراً: `apply` + `correction_deg` + السبب دائماً.
        """
        if distance_cm is None:
            self._prev = None         # انقطاع المرجع يكسر التسلسل
            return self._reject(R_NO_PREV)
        d = float(distance_cm)
        if not (WALL_FOLLOW_MIN_CM <= d <= WALL_FOLLOW_MAX_CM):
            # ⚠ خارج النطاق: قد يكون فتحة أو عائقاً أو لا جدار — لا يُبنى
            #    عليه ميل، ويُكسر التسلسل حتى لا يُقاس Δ عبر الفجوة.
            self._prev = None
            self._tilts.clear()
            return self._reject(R_RANGE, distance_cm=d)
        if self._prev is None:
            self._prev = (d, float(odom_m))
            return self._reject(R_NO_PREV, distance_cm=d)

        prev_d, prev_odom = self._prev
        travel = float(odom_m) - prev_odom
        if travel < WALL_HEADING_MIN_TRAVEL_M:
            # ⚠ مسافة قصيرة ⇒ الزاوية تقسمة على رقم صغير = تضخيم للضجيج.
            #    لا نُحدّث `_prev` حتى نتراكم مسافة كافية.
            return self._reject(R_TRAVEL, travel_m=round(travel, 3))

        tilt = tilt_from_delta(d - prev_d, travel, self.side)
        self._prev = (d, float(odom_m))
        if abs(tilt) > WALL_HEADING_MAX_TILT_DEG:
            # عائق أو انعطاف جدار — لا ميل روبوت
            self._tilts.clear()
            return self._reject(R_TILT, tilt_deg=round(tilt, 2))

        # اتساق الإشارة: ميل حقيقي يستمر، والضجيج يتأرجح
        if self._tilts and (tilt >= 0) != (self._tilts[-1] >= 0):
            self._tilts.clear()
        self._tilts.append(tilt)
        if len(self._tilts) < self.consistent_n:
            return self._reject(R_INCONSISTENT, tilt_deg=round(tilt, 2))

        # 🔴 التصحيح = متوسط الميول المتسقة، مقصوصاً عند السقف
        avg = sum(self._tilts) / len(self._tilts)
        corr = max(-self.max_corr_deg, min(self.max_corr_deg, avg))
        return {"apply": True, "reason": OK, "correction_deg": round(corr, 3),
                "tilt_deg": round(avg, 3), "n_consistent": len(self._tilts),
                "side": self.side, "distance_cm": d,
                "clamped": abs(avg) > self.max_corr_deg}

    # ── تطبيق التصحيح على الحساب الميت ───────────────────────────
    def apply_to(self, dr, decision: dict) -> dict:
        """
        🔴 يصحّح `heading` **فعلياً** ويُصغّر `heading_sigma_deg` — وهذا ما
        يكسر الحلقة المفرغة ويعيد فتح بوابة محاذاة تصحيح الجدار الأمامي.

        ⚠ σ **تُصغَّر ولا تُصفَّر**: القياس نفسه فيه ضجيج (ميل التركيب،
           خشونة الجدار، ضجيج الحسّاس)، وتصفيرها يخلق ثقة اتجاهية لا مصدر
           لها — وهي أخطر من الشك المعلَن.
        """
        if not decision.get("apply") or dr is None:
            return {"applied": False}
        before_h = dr.heading
        before_s = dr.heading_sigma_deg
        dr.heading = (dr.heading + decision["correction_deg"]) % 360.0
        dr.heading_sigma_deg = max(
            WALL_HEADING_SIGMA_FLOOR_DEG,
            dr.heading_sigma_deg * WALL_HEADING_SIGMA_SHRINK)
        rec = {"time": round(time.time(), 2), "side": self.side,
               "delta_cm": decision.get("distance_cm"),
               "angle_deg": decision["correction_deg"],
               "heading_before": round(before_h, 2),
               "heading_after": round(dr.heading, 2),
               "sigma_before": round(before_s, 2),
               "sigma_after": round(dr.heading_sigma_deg, 2),
               "n_consistent": decision.get("n_consistent")}
        self.corrections.append(rec)
        self.reset()                  # الأدلة استُهلكت — لا تُطبَّق مرتين
        return {"applied": True, **rec}

    def state(self) -> dict:
        return {"side": self.side, "n_corrections": len(self.corrections),
                "pending_consistent": len(self._tilts),
                "rejects": dict(self.rejects),
                "last": self.corrections[-1] if self.corrections else None}
