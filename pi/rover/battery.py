# -*- coding: utf-8 -*-
"""
battery.py — تصنيف جهد البطارية وحمايتها (البند أ-3)
====================================================
بطارية 3S: اسمي 11.1V، ممتلئة 12.6V. الجهد يُقرأ من الحقل `v` في رسالة
`T=130` باستمرار. سُجّل أثناء الاختبار هبوط إلى 8.98V — هذه الحماية تمنع تكراره.

| الجهد | الحالة | الإجراء التلقائي |
|---|---|---|
| > 11.5V      | ممتاز  | تشغيل طبيعي |
| 10.8 – 11.5V | جيد    | تنبيه خفيف |
| 10.2 – 10.8V | منخفض  | **RTH إجباري** |
| < 10.0V      | حرج    | **إيقاف المحركات فوراً** + إنذار |
"""
from pi.config import (
    BATT_EXCELLENT_V, BATT_GOOD_V, BATT_LOW_V, BATT_CRITICAL_V,
    BATT_FULL_V, BATT_EMPTY_V, BATT_CELLS, CALIB_VOLTAGE_DELTA_WARN,
    BATT_CURVE, BATT_R_INTERNAL_OHM, BATT_SHUTDOWN_V,
    BATT_CHARGE_WINDOW_S, BATT_CHARGE_MIN_SPAN_S, BATT_CHARGE_MIN_SAMPLES,
    BATT_CHARGE_SLOPE_MV_MIN, BATT_CHARGE_MIN_RISE_MV, BATT_CHARGE_QUIET_S,
)

# (الإجراء، الاسم، اللون)
ACTION_NONE, ACTION_WARN, ACTION_RTH, ACTION_STOP = "none", "warn", "rth", "stop"
# 🔴 إطفاء نظام التشغيل — **تحت** إيقاف المحركات لا بدلاً منه
ACTION_SHUTDOWN = "shutdown"


def percent(v: float, amps: float = None) -> int:
    """
    نسبة الشحن **باستيفاء منحنى** لا بمعادلة خطية.

    ⚠ منحنى تفريغ الليثيوم مسطّح في وسطه وحادّ عند طرفيه، فالخطّية تبالغ في
    المنتصف وتضلّل عند الأطراف. والجهد **ينخفض تحت الحمل** (0.3–0.5V مقاسة
    عند تشغيل المحركات) فيُعوَّض بمقاومة الحزمة: V_مفتوح ≈ V + |I|×R.
    ⚠ تبقى **تقديرية** ويُتَّخذ القرار على **الجهد** لا عليها.
    """
    vv = float(v)
    if amps:
        vv += abs(float(amps)) * BATT_R_INTERNAL_OHM
    if vv >= BATT_CURVE[0][0]:
        return 100
    if vv <= BATT_CURVE[-1][0]:
        return 0
    for (v1, p1), (v2, p2) in zip(BATT_CURVE, BATT_CURVE[1:]):
        if v2 <= vv <= v1:
            return int(round(p2 + (vv - v2) * (p1 - p2) / (v1 - v2)))
    return 0


class ChargeDetector:
    """
    🔴 يكشف الشحن من **ميل الجهد** — لا من إشارة التيار.

    السبب مقاس لا مفترض (2026-08-11): شنت INA219 على هذه اللوحة يقع على
    **مسار الحِمل**. وُصل الشاحن في منتصف مراقبة 120ث فقفز الجهد
    11.92→12.10V و**التيار لم يتغيّر** (0.505–0.621 قبله · 0.495–0.615
    بعده). فهو يقرأ سحب الراسبري لا تيار الحزمة، ولا يصير سالباً أبداً ⇒
    اتجاه الشحن **غير مستخرَج منه أصلاً**.

    والميل مرجع مستقل: الشحن قِيس **+90 mV/دقيقة** مستمرة، والتفريغ الساكن
    عند 4.0V/خلية بضعة ملّي-فولت سالبة. فصل واسع.

    🔴 **يفشل نحو الأمان بالبناء**: `charging` يُعفي من الإطفاء المنظَّم،
    وحِمل المحركات يُنزل الجهد — فأقصى ما يفعله الضجيج أن يمنع ادّعاء
    الشحن، فتبقى الطبقة الرابعة مسلَّحة. العكس (ادّعاء شحن كاذب) هو الخطر،
    ولذلك كل شروطه مجتمعة لا أيّها.

    ⚠ وأخطر شَرَك: **ارتداد الجهد بعد رفع الحمل** يرتفع تماماً كالشحن
    (الحزمة تسترخي فتستعيد 0.2–0.3V). لهذا تُهمَل كل عيّنة قبل مرور
    `BATT_CHARGE_QUIET_S` على آخر أمر حركة، و**أي حركة تمسح النافذة
    كاملة** — لا تُرمَّم بعيّنات ما قبل الحركة.

    أداة داخلية: يُغذّيها `WaveRoverBridge.voltage()` وتُقرأ في
    `battery_state()`.
    """

    def __init__(self):
        self._pts = []             # [(t, v)] داخل النافذة، بعد السكون فقط
        self.slope_mv_min = None   # الميل المحسوب أو None (لا يكفي بعد)
        self.charging = False
        self.reason = "لا عيّنات بعد"

    def reset(self, why: str = "حركة — النافذة مُسحت") -> None:
        self._pts.clear()
        self.slope_mv_min = None
        self.charging = False
        self.reason = why

    def feed(self, t: float, v: float, quiet_for_s: float) -> bool:
        """يضيف عيّنة ويُعيد `charging`. `quiet_for_s` = منذ آخر أمر حركة."""
        if v is None:
            return self.charging
        if quiet_for_s < BATT_CHARGE_QUIET_S:
            # ⚠ المسح لا الإهمال: عيّنات ما قبل الحركة + ارتداد ما بعدها
            #   يصنعان ميلاً صاعداً كاذباً لو خُيّطت النافذة عبر الحركة.
            self.reset()
            return False
        self._pts.append((float(t), float(v)))
        cutoff = t - BATT_CHARGE_WINDOW_S
        while self._pts and self._pts[0][0] < cutoff:
            self._pts.pop(0)
        n = len(self._pts)
        span = self._pts[-1][0] - self._pts[0][0] if n > 1 else 0.0
        if n < BATT_CHARGE_MIN_SAMPLES or span < BATT_CHARGE_MIN_SPAN_S:
            self.slope_mv_min = None
            self.charging = False
            self.reason = (f"نافذة غير مكتملة ({n} عيّنة · {span:.0f}ث) — "
                           f"الشحن **مجهول** لا منفيّ")
            return False
        # انحدار خطي (أقل المربعات) — لا فرق طرفين: قفزة واحدة تخدعه
        mt = sum(p[0] for p in self._pts) / n
        mv = sum(p[1] for p in self._pts) / n
        num = sum((p[0] - mt) * (p[1] - mv) for p in self._pts)
        den = sum((p[0] - mt) ** 2 for p in self._pts)
        slope = (num / den) if den > 1e-9 else 0.0        # V/ث
        self.slope_mv_min = round(slope * 60000.0, 1)
        rise_mv = (self._pts[-1][1] - self._pts[0][1]) * 1000.0
        # 🔴 الشرطان **معاً**: الميل وحده يمرّره ضجيج منحاز، والارتفاع
        #    الصافي وحده تمرّره قفزة واحدة ثم استواء.
        self.charging = (self.slope_mv_min >= BATT_CHARGE_SLOPE_MV_MIN
                         and rise_mv >= BATT_CHARGE_MIN_RISE_MV)
        self.reason = (f"ميل {self.slope_mv_min:+.0f} mV/دقيقة · ارتفاع "
                       f"{rise_mv:+.0f} mV خلال {span:.0f}ث")
        return self.charging

    def state(self) -> dict:
        return {"charging": self.charging, "slope_mv_min": self.slope_mv_min,
                "samples": len(self._pts), "reason": self.reason,
                # «مجهول» ≠ «لا يشحن»: النافذة لم تكتمل بعد (§6.1)
                "unknown": self.slope_mv_min is None}


def cell_voltage(v: float) -> float:
    """جهد الخلية الواحدة — المؤشّر الأصدق على صحة حزمة ليثيوم 3S."""
    return v / max(1, BATT_CELLS)


def classify(v, amps: float = None) -> dict:
    """يُصنّف الجهد ويحدّد الإجراء التلقائي الإلزامي."""
    if v is None:
        return {"level": "unknown", "action": ACTION_NONE, "color": "#8a93a6",
                "percent": 0, "v": None, "cell_v": None, "amps": None,
                "charging": False, "cells": BATT_CELLS,
                "text": "جهد غير معروف"}
    common = {"percent": percent(v, amps), "v": round(v, 2),
              "cell_v": round(cell_voltage(v), 2), "cells": BATT_CELLS,
              "amps": (round(amps, 3) if amps is not None else None)}
    # 🔴 الطبقة الأخيرة: تحت هذا الحدّ يقترب قطع الحماية (~8.4V) الذي يقطع
    #    التغذية فجأةً — وانقطاعها أثناء الكتابة على البطاقة يفسدها.
    #    ⚠ الإجراء نفسه مشروط بـ5 قراءات متتالية وبعدم الشحن — في mission.
    if v < BATT_SHUTDOWN_V:
        return {"level": "shutdown", "action": ACTION_SHUTDOWN,
                "color": "#7f1d1d",
                "text": "🔴 حرج جداً — إطفاء منظَّم", **common}
    if v < BATT_CRITICAL_V:
        return {"level": "critical", "action": ACTION_STOP, "color": "#dc2626",
                "text": "حرج — إيقاف فوري", **common}
    if v < BATT_GOOD_V:
        # كل ما دون 10.8V (وفوق الحرج) = منخفض → RTH إجباري
        return {"level": "low", "action": ACTION_RTH, "color": "#ea580c",
                "text": "منخفض — عودة إجبارية", **common}
    if v < BATT_EXCELLENT_V:
        return {"level": "good", "action": ACTION_WARN, "color": "#eab308",
                "text": "جيد", **common}
    return {"level": "excellent", "action": ACTION_NONE, "color": "#22c55e",
            "text": "ممتاز", **common}


def calibration_voltage_warning(calib_v, current_v):
    """
    تحذير صلاحية المعايرة: أرقام السرعة تتأثر بالجهد. فارق > 1V بين لحظة
    المعايرة ولحظة التشغيل → المعايرة قد تكون غير دقيقة (البند أ-5).
    """
    if not calib_v or current_v is None:
        return None
    delta = abs(current_v - calib_v)
    if delta > CALIB_VOLTAGE_DELTA_WARN:
        return (f"⚠ المعايرة قد تكون غير دقيقة — فارق الجهد {delta:.1f}V "
                f"(معايرة عند {calib_v:.1f}V، التشغيل عند {current_v:.1f}V)")
    return None
