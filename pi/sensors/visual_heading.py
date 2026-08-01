# -*- coding: utf-8 -*-
"""
visual_heading.py — الاتجاه من الكاميرا: **مرجع مطلق يوقف تراكم الانحراف**
==========================================================================
الجايرو مصدر **نسبي**: كل لفّة تضيف خطأها الصغير، ولا شيء يعيده إلى الصفر —
فينحرف الروبوت عن خطوط الشبكة خلية بعد خلية. الكاميرا تكسر هذا التراكم لأن
**المشهد لا يتحرك**: إن رأى الروبوت نفس المنظر الذي رآه عند الاتجاه 90° أول
مرة، فهو على 90° بالضبط مهما تراكم في الجايرو.

────────────────────────────────────────────────────────────────────
الفكرة في ثلاثة أسطر:

1. **توقيع الأعمدة**: شريط أفقي من الصورة → متوسط كل عمود → إشارة 1-بُعد.
   (دوران الروبوت يميناً = انزلاق المشهد يساراً في الصورة، بلا تشوّه يُذكر.)
2. **مطابقة بالإزاحة**: ارتباط متقاطع مطبَّع بين توقيعين → إزاحة بالبكسل،
   ثم درجات عبر مجال الرؤية الأفقي:  `درجة = بكسل × (HFOV ÷ العرض)`.
3. **مراسي الاتجاه**: يُحفظ توقيع كل اتجاه شبكي (0/90/180/270) أول مرة يقفه
   الروبوت، فتصير مقارنة لاحقة به **قياس خطأ مطلق** لا تكاملاً.

────────────────────────────────────────────────────────────────────
⚠ قواعد ملزمة (CLAUDE.md §5): هذه الطبقة **استشارية لا آمرة**.
  - لا تقود المحركات ولا توقف المهمة؛ تُخرج «خطأ اتجاه مقترحاً» فقط، ويغلقه
    متحكّم تثبيت الاتجاه أثناء السير (لا لفّة بالمكان — §1.1.2).
  - كل مخرجاتها **مقصوصة** بـ`VISUAL_MAX_CORRECTION_DEG`: مطابقة خاطئة
    واثقة أخطر من غياب المطابقة.
  - انقطاع الكاميرا أو ضعف المشهد ⇒ `ok=False` والملاحة تكمل بالجايرو وحده.

⚠ `CAMERA_HFOV_DEG` **يُقاس ولا يُخمَّن** (نفس درس `τ` و`BNO055_GYRO_Z_SIGN`):
   الرقم يُضرب في الإزاحة، فقيمة مفترضة خاطئة تُنتج تصحيحاً خاطئاً **واثقاً**.
   صفر = «لم يُقَس» والوحدة ترفض إخراج أي درجة. القياس:
       python3 -m pi.tests.check_visual_heading --calibrate

⚠ منطق المطابقة **خالص** (numpy فقط، بلا cv2 وبلا كاميرا) فيُختبر كاملاً على
   ويندوز بصور مُزاحة اصطناعياً — انظر `pi.nav.selftest` القسم (س).
"""
from __future__ import annotations

import numpy as np

from pi.config import (
    CAMERA_HFOV_DEG, VISUAL_YAW_SIGN, VISUAL_BAND, VISUAL_SMOOTH_PX,
    VISUAL_MIN_CONTRAST, VISUAL_MIN_CORR, VISUAL_MIN_MARGIN,
    VISUAL_MAX_CORRECTION_DEG, VISUAL_MAX_SHIFT_FRAC,
)

# تسامح الاستواء عند تتبّع حدود الفصّ الرئيسي (ضجيج الارتباط)
_LOBE_FLAT_TOL = 1e-3


# ═══ 1) منطق خالص: التوقيع والمطابقة ═════════════════════════════
def to_gray(frame) -> np.ndarray:
    """صورة ملوّنة أو رمادية → مصفوفة رمادية float32."""
    a = np.asarray(frame)
    if a.ndim == 3:
        a = a[..., :3].mean(axis=2)
    return a.astype(np.float32)


def column_signature(frame, band=VISUAL_BAND,
                     smooth_px: int = VISUAL_SMOOTH_PX) -> np.ndarray:
    """
    شريط أفقي من الصورة → متوسط كل عمود → إشارة 1-بُعد بطول عرض الصورة.

    ⚠ **الشريط لا الصورة كاملة**: الأرضية القريبة تنزلق بسرعة مع تقدّم
       الروبوت (تزيّح المنظر)، والسقف كثيراً ما يكون سطحاً أملس بلا معالم.
       الشريط الأوسط يرى الجدران والأثاث البعيد — حيث الدوران يُنتج إزاحة
       شبه نقية والانتقال يُنتج أقلّ ما يمكن.
    ⚠ التنعيم يقتل ضجيج المستشعر الذي يخلق ذروة ارتباط كاذبة حادّة.
    """
    g = to_gray(frame)
    h = g.shape[0]
    r0 = max(0, int(h * float(band[0])))
    r1 = min(h, max(r0 + 1, int(h * float(band[1]))))
    sig = g[r0:r1].mean(axis=0)
    k = int(smooth_px)
    if k > 1 and len(sig) > k:
        # ⚠ **تبطين بالحافة لا بالأصفار**: `np.convolve(..., "same")` يبطّن
        #    بأصفار، فيهبط الطرفان هبوطاً حاداً ويولّد تبايناً **مصطنعاً**.
        #    الأثر خبيث: جدار أملس تماماً (توقيع مسطّح، التباين الحقيقي صفر)
        #    كان يعطي σ≈4.5 فيتجاوز عتبة `VISUAL_MIN_CONTRAST` — أي أن حارس
        #    «المشهد بلا معالم» يسقط في الحالة الوحيدة التي وُجد لأجلها.
        #    (كشفه فحص الجدار الأملس في selftest.)
        pad = k // 2
        padded = np.pad(sig, (pad, k - 1 - pad), mode="edge")
        sig = np.convolve(padded, np.ones(k, dtype=np.float32) / k, mode="valid")
    return sig.astype(np.float32)


def signature_contrast(sig) -> float:
    """
    تباين التوقيع (انحراف معياري). جدار أملس مضاء بانتظام يعطي توقيعاً شبه
    مسطّح، فأي إزاحة تطابقه بنفس الجودة تقريباً ⇒ **ارتباط عالٍ بلا معلومة**.
    هذا الفحص يسبق كل شيء: لا معنى للثقة في مطابقة مشهد بلا معالم.
    """
    return float(np.std(np.asarray(sig, dtype=np.float32)))


def peak_margin(scores, k: int) -> dict:
    """
    فرق الذروة عن أفضل **ذروة منافسة**: أعلى قيمة خارج فصّها الرئيسي.

    ⚠ عطل مقاس على العتاد (2026-08-01): كان الجوار المستبعَد **ثابتاً** (±3
       بكسل)، وهذا خطأ في القياس لا في المشهد — الارتباط الذاتي لمشهد طبيعي
       منعَّم **عريض**: عند ±4 بكسل يبقى ~0.98. فكان كل مشهد سليم يُرفض
       كأنه «نمط متكرّر» (سُجّل: تباين 36.3 وارتباط 1.00 وإزاحة 0.00 بكسل
       رُفض بحدّة 0.02 — أي رُفض أفضلُ مشهد ممكن).

    الصحيح أن الفصّ الرئيسي يُحدَّد **بالنزول من الذروة حتى ينقلب الميل**،
    وما بعده وحده منافس حقيقي. النمط الدوري يُنتج فصّاً ثانياً مرتفعاً بعيداً
    عن الأول — وهو ما نريد رفضه فعلاً.
    """
    s = np.asarray(scores, dtype=np.float32)
    n = len(s)
    i = k
    while i > 0 and s[i - 1] <= s[i] + _LOBE_FLAT_TOL:
        i -= 1
    j = k
    while j < n - 1 and s[j + 1] <= s[j] + _LOBE_FLAT_TOL:
        j += 1
    mask = np.ones(n, dtype=bool)
    mask[i:j + 1] = False
    if not bool(mask.any()):
        # الفصّ يغطي مدى البحث كله ⇒ لا منافس أصلاً: أوضح حالة ممكنة.
        return {"margin": 1.0, "lobe": (int(i), int(j)), "rival_at": None}
    rival = int(np.argmax(np.where(mask, s, -2.0)))
    return {"margin": float(s[k] - s[rival]), "lobe": (int(i), int(j)),
            "rival_at": rival, "rival": float(s[rival])}


def _unit(v: np.ndarray) -> np.ndarray:
    v = v - v.mean()
    n = float(np.sqrt(float((v * v).sum())))
    return (v / n) if n > 1e-9 else np.zeros_like(v)


def match_shift(a, b, max_shift: int = None, min_overlap: int = 32) -> dict:
    """
    إزاحة أفقية بين توقيعين بالارتباط المتقاطع **المطبَّع**.

    الاصطلاح: `shift_px` موجب إذا انتقل المشهد في `b` نحو **اليمين** مقارنة
    بـ`a` (أي `b[i] ≈ a[i − shift]`).

    التطبيع (طرح المتوسط ثم القسمة على المعيار) يجعل النتيجة **محصّنة ضد
    تغيّر الإضاءة**: مصباح يُشعل أو ظل يمرّ يغيّر السطوع لا الشكل.

    يُعيد أيضاً `margin` = فرق الذروة عن أفضل قيمة **خارج جوارها**. وهذا
    الحارس ضروري في الأماكن المغلقة تحديداً: البلاط والستائر والأرفف أنماط
    **دورية**، تعطي ذرىً متساوية عند إزاحات متعددة — ارتباط 0.9 لا يعني
    شيئاً إن كان هناك 0.89 آخر عند إزاحة مختلفة تماماً.
    """
    a = np.asarray(a, dtype=np.float32)
    b = np.asarray(b, dtype=np.float32)
    n = int(min(len(a), len(b)))
    if n < min_overlap + 2:
        return {"ok": False, "reason": f"التوقيع أقصر من الحدّ ({n})"}
    a, b = a[:n], b[:n]
    if max_shift is None:
        max_shift = int(n * VISUAL_MAX_SHIFT_FRAC)
    max_shift = int(max(1, min(max_shift, n - min_overlap)))

    scores = np.empty(2 * max_shift + 1, dtype=np.float32)
    for idx, s in enumerate(range(-max_shift, max_shift + 1)):
        if s >= 0:
            seg_b, seg_a = b[s:], a[:n - s]
        else:
            seg_b, seg_a = b[:n + s], a[-s:]
        scores[idx] = float(np.dot(_unit(seg_a), _unit(seg_b)))

    k = int(np.argmax(scores))
    peak = float(scores[k])
    shift = float(k - max_shift)

    # تنقيح دون-البكسل بقطع مكافئ حول الذروة (دقة ~0.1 بكسل)
    if 0 < k < len(scores) - 1:
        y0, y1, y2 = float(scores[k - 1]), float(scores[k]), float(scores[k + 1])
        den = y0 - 2.0 * y1 + y2
        if abs(den) > 1e-9:
            shift += float(np.clip(0.5 * (y0 - y2) / den, -1.0, 1.0))

    pm = peak_margin(scores, k)
    return {"ok": True, "shift_px": shift, "peak": peak,
            "margin": pm["margin"], "lobe": pm["lobe"],
            "rival_shift_px": (None if pm["rival_at"] is None
                               else pm["rival_at"] - max_shift),
            "at_limit": abs(k - max_shift) >= max_shift}


def shift_to_deg(shift_px: float, width_px: int,
                 hfov_deg: float = CAMERA_HFOV_DEG,
                 sign: int = VISUAL_YAW_SIGN) -> float:
    """
    بكسل → درجة عبر مجال الرؤية الأفقي. ⚠ `hfov_deg = 0` تعني **لم يُقَس**
    فترفع ValueError بدل إنتاج رقم مخترع (انظر رأس الملف).
    """
    if not hfov_deg or float(hfov_deg) <= 0.0:
        raise ValueError("CAMERA_HFOV_DEG غير مقاس — شغّل "
                         "python3 -m pi.tests.check_visual_heading --calibrate")
    return float(sign) * float(shift_px) * (float(hfov_deg) / float(width_px))


def compare(sig_a, sig_b, hfov_deg: float = CAMERA_HFOV_DEG,
            sign: int = VISUAL_YAW_SIGN) -> dict:
    """
    توقيعان → زاوية الدوران بينهما، **مع كل حرّاس الجودة**. النتيجة
    `{"ok", "deg", "peak", "margin", "contrast", "reason"}`.
    `ok=False` يعني «لا أعرف» — وهي إجابة مشروعة تكمل الملاحة بالجايرو.
    """
    ca, cb = signature_contrast(sig_a), signature_contrast(sig_b)
    contrast = min(ca, cb)
    if contrast < VISUAL_MIN_CONTRAST:
        return {"ok": False, "contrast": round(contrast, 2),
                "reason": f"المشهد بلا معالم كافية (تباين {contrast:.1f} < "
                          f"{VISUAL_MIN_CONTRAST}) — جدار أملس أو إضاءة مسطّحة"}
    m = match_shift(sig_a, sig_b)
    if not m.get("ok"):
        return {"ok": False, "contrast": round(contrast, 2),
                "reason": m.get("reason")}
    if m["peak"] < VISUAL_MIN_CORR:
        return {"ok": False, "contrast": round(contrast, 2), "peak": m["peak"],
                "reason": f"تطابق ضعيف ({m['peak']:.2f} < {VISUAL_MIN_CORR}) — "
                          f"تغيّر المشهد كثيراً"}
    if m["margin"] < VISUAL_MIN_MARGIN:
        return {"ok": False, "contrast": round(contrast, 2), "peak": m["peak"],
                "margin": round(m["margin"], 3),
                "reason": f"ذروة غير مميّزة (حدّة {m['margin']:.2f}) — ذروة "
                          f"منافسة عند إزاحة {m.get('rival_shift_px')} بكسل: "
                          f"نمط متكرّر (بلاط/ستائر/أرفف)؟"}
    if m.get("at_limit"):
        return {"ok": False, "contrast": round(contrast, 2), "peak": m["peak"],
                "reason": "الإزاحة عند حدّ البحث — الدوران أكبر من مجال الرؤية"}
    try:
        deg = shift_to_deg(m["shift_px"], len(sig_a), hfov_deg, sign)
    except ValueError as e:
        return {"ok": False, "reason": str(e)}
    return {"ok": True, "deg": round(deg, 2), "shift_px": round(m["shift_px"], 2),
            "peak": round(m["peak"], 3), "margin": round(m["margin"], 3),
            "contrast": round(contrast, 2)}


# ═══ 2) مراسي الاتجاه (تحتاج كاميرا) ═════════════════════════════
def _grid_key(heading_deg: float) -> int:
    """أقرب اتجاه شبكي (0/90/180/270) — مفتاح المرساة."""
    return int(round((float(heading_deg) % 360.0) / 90.0)) % 4 * 90


class VisualHeading:
    """
    مرجع اتجاه بصري مبني على **مراسي** الاتجاهات الشبكية الأربعة.

    الاستعمال في المهمة:
        vh = VisualHeading(camera)
        res = vh.residual_deg(target_heading)   # قبل الشوط
        if res["ok"]: ...استعمل res["deg"] خطأً مقترحاً...
        vh.set_anchor(target_heading)           # بعد الشوط (يُثبّت أول مرة)

    ⚠ لا يملك حالة زمنية ولا تكاملاً: كل قياس مستقل عن سابقه، فخطأ مطابقة
       واحد لا يلوّث ما بعده — عكس الجايرو تماماً، وهذا سبب وجوده.
    """

    def __init__(self, camera=None, hfov_deg: float = CAMERA_HFOV_DEG,
                 sign: int = VISUAL_YAW_SIGN):
        self.camera = camera
        self.hfov_deg = float(hfov_deg or 0.0)
        self.sign = int(sign)
        self.anchors: dict[int, np.ndarray] = {}
        self.last = None            # آخر توقيع (لقياس الدلتا بين لقطتين)
        self.grabs = 0
        self.failures = 0
        self.corrections = 0
        self.last_result = None
        self.error = None
        if self.hfov_deg <= 0.0:
            self.error = ("CAMERA_HFOV_DEG غير مقاس — الاتجاه البصري معطّل. "
                          "شغّل: python3 -m pi.tests.check_visual_heading --calibrate")

    # ── التقاط ──────────────────────────────────────────────────
    @property
    def ready(self) -> bool:
        return self.hfov_deg > 0.0 and self.camera is not None

    def grab(self):
        """لقطة → توقيع أعمدة، أو None عند تعذّر الكاميرا (بلا استثناء)."""
        if self.camera is None:
            return None
        try:
            frame = self.camera.frame_array()
        except Exception as e:                 # noqa: BLE001
            self.failures += 1
            self.error = f"تعذّرت قراءة الكاميرا: {e}"
            return None
        if frame is None:
            self.failures += 1
            return None
        self.grabs += 1
        sig = column_signature(frame)
        self.last = sig
        return sig

    # ── المراسي ─────────────────────────────────────────────────
    def set_anchor(self, heading_deg: float, force: bool = False) -> bool:
        """
        يثبّت مرساة الاتجاه الشبكي إن لم تكن مثبّتة.
        ⚠ **أول مرة فقط** افتراضياً: تحديثها في كل زيارة يجعلها تنجرف مع
           الجايرو الذي وُجدت لتصحيحه — فتصير مرآةً لا مرجعاً.
        """
        if not self.ready:
            return False
        key = _grid_key(heading_deg)
        if key in self.anchors and not force:
            return False
        sig = self.grab()
        if sig is None or signature_contrast(sig) < VISUAL_MIN_CONTRAST:
            return False
        self.anchors[key] = sig
        return True

    def residual_deg(self, heading_deg: float) -> dict:
        """
        كم انحرف الروبوت عن الاتجاه الذي ثُبّتت عنده المرساة؟

        موجب = الروبوت مستدير **يميناً** عن المرساة، فالتصحيح المطلوب سالب.
        النتيجة مقصوصة بـ`VISUAL_MAX_CORRECTION_DEG`: انحراف بصري ضخم يعني
        مطابقة خاطئة أو مشهداً تغيّر (باب فُتح، شخص مرّ) لا انحرافاً حقيقياً
        بهذا الحجم — والتصحيح على أساسه أسوأ من عدمه.
        """
        if not self.ready:
            return {"ok": False, "reason": self.error or "لا كاميرا"}
        key = _grid_key(heading_deg)
        anchor = self.anchors.get(key)
        if anchor is None:
            return {"ok": False, "reason": f"لا مرساة للاتجاه {key}°"}
        sig = self.grab()
        if sig is None:
            return {"ok": False, "reason": self.error or "تعذّرت اللقطة"}
        res = compare(anchor, sig, self.hfov_deg, self.sign)
        res["anchor"] = key
        if res.get("ok"):
            if abs(res["deg"]) > VISUAL_MAX_CORRECTION_DEG:
                res = {"ok": False, "anchor": key, "deg": res["deg"],
                       "reason": f"انحراف بصري {res['deg']:+.1f}° فوق الحدّ "
                                 f"({VISUAL_MAX_CORRECTION_DEG}°) — الأرجح "
                                 f"تغيّر المشهد لا دوران الروبوت"}
            else:
                self.corrections += 1
        self.last_result = res
        return res

    def state(self) -> dict:
        return {
            "ready": self.ready, "error": self.error,
            "hfov_deg": self.hfov_deg, "sign": self.sign,
            "anchors": sorted(self.anchors.keys()),
            "grabs": self.grabs, "failures": self.failures,
            "corrections": self.corrections,
            "last": self.last_result,
        }
