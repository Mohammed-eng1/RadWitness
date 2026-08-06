# -*- coding: utf-8 -*-
"""
deadreckoning.py — الدفعة1/أ: تتبّع الموقع بلا GPS داخل الغرفة
==============================================================
الموقع (x,y) بالمتر داخل الغرفة، يُحدَّث من heading (نسبي لمرجع الغرفة) +
المسافة المقطوعة (السرعة المعايرة × الزمن). لا يدّعي دقة سنتيمترية —
الهدف ±20-30 سم، ويُعبَّر عن الشك بـ«دائرة عدم اليقين».

نموذج نمو عدم اليقين (تقديري حتى يُقاس تجريبياً — المعاملات في config):
    uncertainty += DRIFT_PER_METER × المسافة المقطوعة (م)
    uncertainty += DRIFT_PER_TURN  × (درجات الدوران ÷ 90)
    عند تصحيح جدار ناجح: uncertainty = max(RESET_FLOOR, uncertainty × SHRINK)

تصحيح الجدران — **شرطا أمان إلزاميان معاً** (البند أ-4)، وإلا لا تصحيح:
    1) محاذاة: heading النسبي قرب محور جدار (0/90/180/270 ±WALL_ALIGN_TOL_DEG).
    2) معقولية: |المقاس − المتوقع من الشبكة| ≤ WALL_CORRECTION_MAX_DELTA_M.
    الفشل في أيٍّ منهما → المرجّح أنه عائق لا جدار → لا يُصحَّح (يُعامل عائقاً).
"""
from __future__ import annotations

import math
import time

from pi.config import (
    DRIFT_PER_METER, DRIFT_PER_TURN, UNCERTAINTY_INITIAL,
    UNCERTAINTY_RESET_FLOOR, WALL_CORRECTION_SHRINK,
    WALL_ALIGN_TOL_DEG, WALL_CORRECTION_MAX_DELTA_M,
    MOTION_UNVERIFIED_DRIFT, HEADING_SIGMA_INITIAL_DEG,
    HEADING_DRIFT_PER_S_DEG, HEADING_SIGMA_PER_TURN_DEG,
)

# محاور الجدران الأربعة: (زاوية الغرفة، المحور، جهة الجدار)
#   0°→أمام (+y عند y=length) · 90°→يمين (+x عند x=width) ·
#   180°→خلف (−y عند y=0)     · 270°→يسار (−x عند x=0)
_WALL_AXES = (0.0, 90.0, 180.0, 270.0)


def _ang_diff(a: float, b: float) -> float:
    """أصغر فرق زاويّ مطلق بين زاويتين [0,180]."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


class DeadReckoning:
    def __init__(self, room, profile, x0: float, y0: float, heading0: float = 0.0):
        self.room = room
        self.profile = profile              # CalibrationProfile (للسرعة/الدوران)
        self.x = float(x0)
        self.y = float(y0)
        self.heading = heading0 % 360.0     # نسبي لمرجع الغرفة
        self.uncertainty = UNCERTAINTY_INITIAL
        self.distance_total = 0.0
        self.corrections: list = []         # سجل التصحيحات للشفافية
        # ── 🔴 عدم يقين **الاتجاه** (بعد استبدال BNO055 بـMPU-6050) ──
        # ينمو مع **الزمن** لا المسافة، ولا شيء في العتاد الحالي يصحّحه.
        self.heading_sigma_deg = HEADING_SIGMA_INITIAL_DEG
        self._sigma_ts = time.time()
        self._gate_lost_announced = False

    # ── الحركة ────────────────────────────────────────────────────
    def move(self, direction: str, power: int, dt: float) -> None:
        """يحرّك الموقع مسافة (سرعة معايرة × dt) على heading الحالي."""
        speed = self.profile.speed_for_power(power)
        dist = speed * dt
        if direction == "B":
            dist = -dist
        hd = math.radians(self.heading)
        self.x += dist * math.sin(hd)       # x نحو اليمين (heading 90°)
        self.y += dist * math.cos(hd)       # y نحو الأمام (heading 0°)
        self.distance_total += abs(dist)
        self.uncertainty += DRIFT_PER_METER * abs(dist)

    # ── عدم يقين الاتجاه: ينمو بالزمن ولا شيء يصحّحه ──────────────
    def sync_time(self) -> float:
        """
        ينمّي شكّ الاتجاه بما مضى من **زمن حائط** منذ آخر مزامنة.

        يُستدعى عند كل حركة وعند كل قياس. حسابه من الساعة يجعله يشمل الوقوف
        والسير واللفّ بلا احتساب مزدوج — والوقوف هو أطول أطوار المهمة
        (3ث لكل خلية، 10ث × 8 في التأكيد، حتى 60ث في التجميع التكيّفي).
        """
        now = time.time()
        dt = max(0.0, now - self._sigma_ts)
        self._sigma_ts = now
        self.heading_sigma_deg += HEADING_DRIFT_PER_S_DEG * dt
        return dt

    @property
    def alignment_gate_lost(self) -> bool:
        """
        🔴 **الحلقة المفرغة**: تصحيح الجدار مشروط بمحاذاة الاتجاه ضمن
        `WALL_ALIGN_TOL_DEG`. فحين يتجاوز شكّ الاتجاه هذا الحدّ يصير التصحيح
        مرفوضاً بنيوياً ⇒ لا تصحيح موضع ⇒ الشك يكبر ⇒ يُرفض أكثر…

        ⚠ وتصحيح الجدار **لا يصحّح الاتجاه أصلاً** (يضبط x أو y فقط)، فلا
           شيء في العتاد الحالي يعكس هذا النمو. الخاصية تُعلنه بدل إخفائه.
        """
        return self.heading_sigma_deg > WALL_ALIGN_TOL_DEG

    def turn(self, degrees: float) -> None:
        """يلفّ بالمكان (+يمين، −يسار) ويُنمّي الشك حسب مقدار الدوران."""
        self.sync_time()
        self.heading = (self.heading + degrees) % 360.0
        self.uncertainty += DRIFT_PER_TURN * (abs(degrees) / 90.0)
        self.heading_sigma_deg += HEADING_SIGMA_PER_TURN_DEG * (abs(degrees) / 90.0)

    def advance(self, dist_m: float, verified: bool = True) -> None:
        """
        يقدّم الموقع بمسافة **مقطوعة فعلياً** (من التنفيذ الحقيقي على المحركات)
        بدل اشتقاقها من زمن×سرعة مفترضة، وينمّي الشك بنفس النموذج.

        `verified=False` (البند 0): الشوط لم يُقَس بمرجع — لا ألترا سونيك ولا
        حكم من التسارع — فالمسافة اشتُقّت من نموذج مفتوح الحلقة. الشك ينمو
        عندها **أسرع** (`MOTION_UNVERIFIED_DRIFT`): تمرير نفس σ لموضع مقيس
        وآخر مفترض يمنح المفترضَ وزن المقيس في شبكة تحديد المصدر.
        """
        self.sync_time()
        hd = math.radians(self.heading)
        self.x += dist_m * math.sin(hd)
        self.y += dist_m * math.cos(hd)
        self.distance_total += abs(dist_m)
        drift = DRIFT_PER_METER * (1.0 if verified else MOTION_UNVERIFIED_DRIFT)
        # 🔴 مساهمة **خطأ الاتجاه** في خطأ الموضع — اقتران ضربي لا جمعي:
        #    الاتجاه المنحرف لا يزيح الروبوت وهو واقف، بل عند أول حركة بعده
        #    بمقدار (المسافة × الخطأ الزاوي). ولهذا يُضرب هنا لا يُجمع في
        #    حدّ زمني مسطّح على σ الموضع.
        lateral = abs(dist_m) * math.radians(self.heading_sigma_deg)
        self.uncertainty += drift * abs(dist_m) + lateral

    def set_pose(self, x: float, y: float, heading: float) -> None:
        """أداة خارجية (سكربتات/تشخيص) — المهمة تصحّح عبر مسارات الجدران لا بفرض وضعة."""
        self.x, self.y, self.heading = float(x), float(y), heading % 360.0

    # ── تصحيح الجدار الأمامي (الشرطان الإلزاميان) ─────────────────
    def try_front_wall_correction(self, measured_dist_m: float, sensor: str = "ultrasonic") -> dict:
        """
        يحاول تصحيح الإحداثي المحوري من مسافة أمامية مقاسة (ألترا سونيك).
        يُعيد dict فيه corrected (bool) وreason. يطبّق فقط إن تحقق الشرطان.
        """
        # الشرط 1: محاذاة الاتجاه لأقرب محور جدار
        axis_angle = min(_WALL_AXES, key=lambda a: _ang_diff(self.heading, a))
        align_err = _ang_diff(self.heading, axis_angle)
        if align_err > WALL_ALIGN_TOL_DEG:
            return {"corrected": False, "reason": "not_aligned",
                    "align_err": round(align_err, 1)}

        # المسافة المتوقعة للجدار المواجَه حسب تقدير الموقع الحالي
        if axis_angle == 0.0:        # يواجه الأمام (y=length)
            axis, expected, coord_from_measured = "y", self.room.length_m - self.y, \
                lambda m: self.room.length_m - m
        elif axis_angle == 180.0:    # يواجه الخلف (y=0)
            axis, expected, coord_from_measured = "y", self.y, lambda m: m
        elif axis_angle == 90.0:     # يواجه اليمين (x=width)
            axis, expected, coord_from_measured = "x", self.room.width_m - self.x, \
                lambda m: self.room.width_m - m
        else:                        # 270° يواجه اليسار (x=0)
            axis, expected, coord_from_measured = "x", self.x, lambda m: m

        # الشرط 2: معقولية المسافة (الفرق ضمن الحد → جدار؛ أكبر → عائق)
        delta = measured_dist_m - expected
        if abs(delta) > WALL_CORRECTION_MAX_DELTA_M:
            return {"corrected": False, "reason": "implausible_treat_as_obstacle",
                    "axis": axis, "delta": round(delta, 3)}

        # تحقق الشرطان → طبّق التصحيح
        new_val = coord_from_measured(measured_dist_m)
        old_val = self.y if axis == "y" else self.x
        if axis == "y":
            self.y = new_val
        else:
            self.x = new_val
        self.uncertainty = max(UNCERTAINTY_RESET_FLOOR,
                               self.uncertainty * WALL_CORRECTION_SHRINK)
        # ⚠ **لا يُصغَّر شكّ الاتجاه هنا عمداً**: هذا التصحيح يضبط إحداثياً
        #    واحداً (x أو y) من مسافة أمامية، ولا يحمل أي معلومة عن الزاوية —
        #    بل يفترضها صحيحة أصلاً (شرط المحاذاة أعلاه). ادّعاء تصغيره يخلق
        #    ثقة اتجاهية لا مصدر لها، وهي أخطر من الشك المعلَن.
        record = {"time": time.time(), "axis": axis,
                  "old_value": round(old_val, 3), "new_value": round(new_val, 3),
                  "delta": round(delta, 3), "sensor": sensor}
        self.corrections.append(record)
        return {"corrected": True, "reason": "wall", **record}

    # ── الحالة (للواجهة/السجل) ────────────────────────────────────
    def state(self) -> dict:
        return {
            "x": round(self.x, 3), "y": round(self.y, 3),
            "heading": round(self.heading, 1),
            "uncertainty_m": round(self.uncertainty, 3),
            "heading_sigma_deg": round(self.heading_sigma_deg, 2),
            "alignment_gate_lost": self.alignment_gate_lost,
            "align_tol_deg": WALL_ALIGN_TOL_DEG,
            "distance_total_m": round(self.distance_total, 2),
            "corrections": len(self.corrections),
        }
