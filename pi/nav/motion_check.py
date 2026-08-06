# -*- coding: utf-8 -*-
"""
motion_check.py — البند 0: هل تحرّك الروبوت فعلاً؟ (منطق خالص، بلا عتاد)
=========================================================================
المشكلة التي يحلّها: `deadreckoning` يشتقّ المسافة من **زمن × سرعة معايرة**
وهي حلقة مفتوحة تماماً — Wave Rover بلا إنكودرات، فلا شيء يكشف انزلاق عجلة
ولا انحشاراً. قياس مسجَّل: شوط قطع **0.20م** والنموذج يقول **1.8م** — خطأ
9 أضعاف بلا أي إنذار. وأثره يتعدّى الخريطة: طبقة تحديد المصدر تبني تقديرها
على **مواضع** القراءات، فالمواضع الخيالية تُنتج خريطة احتمالية بلا معنى.

ثلاثة مستويات من الدليل، بهذا الترتيب:

    1) **الألترا سونيك** (كمّي، الأقوى): المسافة الأمامية قبل العبور وبعده.
       إن وُجد سطح مرجعي في المدى فالفرق ≈ المسافة المأمورة.
    2) **مقياس التسارع** (ثنائي، مكمّل): يعمل حين لا سطح مرجعي أمام الروبوت.
       يجيب عن «هل تحرّك أصلاً؟» فقط — ⚠ **لا يُحسب منه مسافة إطلاقاً**:
       التكامل المزدوج ينجرف تربيعياً وانحياز صغير يصير أمتاراً في ثوانٍ.
    3) **لا مرجع أصلاً**: لا نفي ولا إثبات ⇒ تُعلَّم الخلية **بثقة منخفضة**
       ولون مختلف في الخريطة. الصدق في البيانات أهم من خريطة تبدو مكتملة.

⚠ الوحدة **لا تقرأ حسّاساً ولا تلمس عتاداً** — تستقبل أرقاماً وتُصدر حكماً،
   فتُختبر كاملةً على ويندوز (`python -m pi.nav.selftest`).
"""
from __future__ import annotations

import math

from pi.config import (
    MOTION_REF_MAX_CM, MOTION_TOL_M, MOTION_TOL_FRAC, MOTION_NO_MOTION_M,
    MOTION_ACCEL_STD_MPS2, MOTION_ACCEL_MIN_SAMPLES,
)

# ── الأحكام ──────────────────────────────────────────────────────
VERIFIED = "verified"        # قِيست الإزاحة وطابقت المأمور ضمن التسامح
SHORT = "short"              # تحرّك لكن **أقل** من المأمور بفارق دالّ
OVERSHOOT = "overshoot"      # تحرّك **أكثر** من المأمور بفارق دالّ
NO_MOTION = "no_motion"      # لم يتحرّك — الخلية لا تُعلَّم
UNVERIFIED = "unverified"    # لا مرجع: لا نفي ولا إثبات ⇒ ثقة منخفضة


def tolerance_for(commanded_m: float) -> float:
    """
    تسامح المطابقة = ثابت + نسبة من المسافة. الثابت يغطي ضجيج قراءتين
    (±3سم لكل واحدة) وميل السطح المرجعي؛ والنسبة تغطي خطأ السرعة المعايرة
    كلما طال الشوط. الهدف كشف **الخطأ الفادح** لا معايرة سنتيمترية.
    """
    return MOTION_TOL_M + MOTION_TOL_FRAC * abs(float(commanded_m))


class AccelWitness:
    """
    شاهد ثنائي على الحركة من مقياس تسارع BNO055.

    المعيار **الانحراف المعياري** لمقدار التسارع الأفقي √(ax²+ay²) خلال
    العبور — لا المتوسط ولا الذروة:
      • الانحراف مناعته تامة ضد أي **إزاحة ثابتة** (ميل الهيكل، أو جاذبية
        متسرّبة حين نسقط إلى التسارع الخام بدل الخطّي).
      • السكون التامّ يُعطي إشارة شبه ثابتة مهما كان مقدارها.
    ولهذا يُقاس الأفقي وحده: محور z هو محور الـyaw (مثبت في CLAUDE.md §2)
    فالجاذبية تقع عليه، واستبعاده يزيل أكبر ثابت في الإشارة.

    **ما الذي يُنتج التشتت فعلاً؟** الانطلاق من السكون أولاً: كل شوط يبدأ
    بتسارع من صفر إلى ~0.6 م/ث خلال ~0.3ث (تنعيم التسارع في config) —
    نتوء لا يمكن لروبوت عالق أن يُنتجه. ثم اهتزاز أربعة محركات على الأرضية.
    ⚠ والحالة العمياء نظرياً هي **سرعة ثابتة تماماً بلا اهتزاز**: التسارع
       الخطّي عندها صفر ثابت. لهذا لا يُستدعى هذا الشاهد إلا حين يغيب المرجع
       الأمامي، وعتبته منخفضة عمداً فيميل إلى الامتناع عن النفي.
    ⚠ ومقدار المتّجه لا يرى **انعكاس الإشارة** (0.3− و0.3+ مقدارهما واحد):
       مقصود — نسأل «هل هناك تسارع؟» لا «إلى أين؟»، والاتجاه شأن الجايرو.

    ⚠ العتبة **تقديرية حتى تُقاس على العتاد** (`pi.tests.test_motion_check`)،
       واتجاه خطئها مقصود: عتبة منخفضة تجعل الشاهد **يمتنع عن النفي** فتُعلَّم
       الخلية بثقة منخفضة — وهو تدهور آمن. عتبة مرتفعة تعلن «عالق» كاذباً
       فتوقف المهمة، وهذا الأسوأ.
    """

    def __init__(self, threshold_std: float = MOTION_ACCEL_STD_MPS2,
                 min_samples: int = MOTION_ACCEL_MIN_SAMPLES):
        self.threshold_std = float(threshold_std)
        self.min_samples = int(min_samples)
        self.n = 0
        self._mean = 0.0
        self._m2 = 0.0          # Welford — تشتت بمرور واحد بلا تخزين
        self.lo = None
        self.hi = None

    def add(self, vec) -> None:
        """يضيف (ax, ay, az) م/ث². `None` (فشل قراءة) تُتجاهَل بلا ضجيج."""
        if vec is None or len(vec) < 2 or vec[0] is None or vec[1] is None:
            return
        mag = math.hypot(float(vec[0]), float(vec[1]))
        self.n += 1
        d = mag - self._mean
        self._mean += d / self.n
        self._m2 += d * (mag - self._mean)
        self.lo = mag if self.lo is None else min(self.lo, mag)
        self.hi = mag if self.hi is None else max(self.hi, mag)

    @property
    def std(self) -> float:
        return math.sqrt(self._m2 / (self.n - 1)) if self.n > 1 else 0.0

    @property
    def verdict(self):
        """True تحرّك · False لم يتحرّك · None لا حكم (عيّنات غير كافية)."""
        if self.n < self.min_samples:
            return None
        return self.std >= self.threshold_std

    def state(self) -> dict:
        return {"samples": self.n, "std_mps2": round(self.std, 4),
                "span_mps2": round((self.hi - self.lo), 3)
                if self.lo is not None else None,
                "threshold_std": self.threshold_std, "moved": self.verdict}


def reference_ok(d_cm) -> bool:
    """هل هذه القراءة صالحة سطحاً مرجعياً؟ (موجودة وداخل المدى الموثوق)"""
    return d_cm is not None and 0.0 < float(d_cm) <= MOTION_REF_MAX_CM


def verify_motion(commanded_m: float, d_start_cm, d_end_cm,
                  accel: AccelWitness | None = None, direction: int = +1) -> dict:
    """
    يحكم على شوط واحد. المدخلات أرقام لا حساسات:
      `commanded_m` المسافة المأمورة · `d_start_cm`/`d_end_cm` المسافة
      الأمامية قبل الشوط وبعده (سم أو None) · `accel` شاهد التسارع ·
      `direction` +1 تقدّم و−1 رجوع.

    ⚠ الإزاحة تُقاس **باتجاه السير**: المسافة الأمامية تنقص عند التقدّم نحو
       السطح وتزداد عند الرجوع عنه، فالإزاحة = (البداية − النهاية) × الاتجاه.
       ولهذا يعمل التحقق في الاتجاهين بنفس المرجع الأمامي — وهو ما يجعل
       الانسحاب على أثر المسار مقيساً هو الآخر لا مفترضاً.
       تغيّرها في الجهة **المعاكسة** للأمر إشارة تشخيصية (سطح مرجعي تبدّل،
       أو حركة عكسية) ولا تُحتسب حركةً — يُسلَّم الحكم عندها للتسارع.
    """
    commanded = abs(float(commanded_m))
    tol = tolerance_for(commanded)
    sgn = -1.0 if direction < 0 else 1.0
    a_state = accel.state() if accel is not None else None
    a_verdict = accel.verdict if accel is not None else None
    out = {"commanded_m": round(commanded, 3), "tolerance_m": round(tol, 3),
           "d_start_cm": d_start_cm, "d_end_cm": d_end_cm,
           "direction": int(direction), "measured_m": None, "accel": a_state}

    if reference_ok(d_start_cm) and reference_ok(d_end_cm):
        delta = sgn * (float(d_start_cm) - float(d_end_cm)) / 100.0
        if delta <= -MOTION_NO_MOTION_M:
            # تحرّك في الجهة المعاكسة للأمر
            out.update(_by_accel(a_verdict, "reference_grew"))
            out["reason"] = (
                f"⚠ الإزاحة {abs(delta):.2f}م في **عكس** الجهة المأمورة "
                f"({'تقدّم' if sgn > 0 else 'رجوع'}) — سطح مرجعي تبدّل أو "
                f"حركة عكسية")
            return out
        if abs(delta) < MOTION_NO_MOTION_M:
            # 🔴 «لا حركة» حكم قاتل (لا تُعلَّم الخلية، و3 تكرارات توقف
            #    المهمة) — فلا يُعلَن من الأمامي وحده إذا عارضه شاهد التسارع.
            #    مقاس 2026-08-06: جولة سير حقيقية (شوهدت بالعين) أُجهضت «لا
            #    حركة» كاذبةً لأن صدى الأمامي الغالب جاء من جسم **مائل عن
            #    محور السير** فلم تتغيّر مسافته مع التقدّم (Δ=4سم على أمر
            #    33سم). التعارض ⇒ ثقة منخفضة لا إجهاض. وحالة الروفر المطفأ
            #    تبقى مكشوفة: سكون تامّ في الشاهدين **معاً**.
            if a_verdict is True:
                out.update({"verdict": UNVERIFIED, "method": "conflict",
                            "moved": True, "confident": False,
                            "reason": (f"⚠ تعارض الشاهدين: الأمامي ثابت "
                                       f"({abs(delta) * 100:.0f}سم على أمر "
                                       f"{commanded:.2f}م) والتسارع يرى حركة "
                                       f"— مرجع أمامي مشبوه (صدى مائل عن "
                                       f"محور السير؟) ⇒ ثقة منخفضة")})
                return out
            out.update({"verdict": NO_MOTION, "method": "ultrasonic",
                        "moved": False, "confident": True, "measured_m": 0.0,
                        "reason": f"المسافة الأمامية لم تتغيّر ({abs(delta) * 100:.0f}سم) "
                                  f"رغم أمر بـ{commanded:.2f}م — لا حركة"})
            return out
        out["measured_m"] = round(delta, 3)
        if abs(delta - commanded) <= tol:
            out.update({"verdict": VERIFIED, "method": "ultrasonic",
                        "moved": True, "confident": True,
                        "reason": f"مقاس {delta:.2f}م ≈ مأمور {commanded:.2f}م "
                                  f"(±{tol:.2f})"})
        else:
            short = delta < commanded
            out.update({"verdict": SHORT if short else OVERSHOOT,
                        "method": "ultrasonic", "moved": True,
                        "confident": False,
                        "reason": (f"⚠ مقاس {delta:.2f}م مقابل مأمور "
                                   f"{commanded:.2f}م — "
                                   f"{'أقصر' if short else 'أطول'} من التسامح "
                                   f"(±{tol:.2f}م)")})
        return out

    # لا سطح مرجعي صالح → الشاهد الثنائي
    why = ("لا قراءة ألترا سونيك" if d_start_cm is None or d_end_cm is None
           else f"لا سطح مرجعي داخل {MOTION_REF_MAX_CM:.0f}سم")
    out.update(_by_accel(a_verdict, "no_reference"))
    out["reason"] = why + " — " + {
        NO_MOTION: "ومقياس التسارع لم يرَ حركة ⇒ عالق أو منزلق",
        UNVERIFIED: ("والتسارع يؤكّد الحركة، لكن المسافة غير مقيسة ⇒ ثقة منخفضة"
                     if a_verdict else "ولا حكم من التسارع ⇒ ثقة منخفضة"),
    }[out["verdict"]]
    return out


def _by_accel(a_verdict, cause: str) -> dict:
    """الحكم حين يتعذّر القياس الكمّي: النفي من التسارع أو ثقة منخفضة."""
    if a_verdict is False:
        return {"verdict": NO_MOTION, "method": "accel", "moved": False,
                "confident": True, "cause": cause}
    return {"verdict": UNVERIFIED,
            "method": "accel" if a_verdict else "none",
            "moved": True if a_verdict else None,
            "confident": False, "cause": cause}
