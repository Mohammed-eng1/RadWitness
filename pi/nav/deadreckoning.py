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

    def turn(self, degrees: float) -> None:
        """يلفّ بالمكان (+يمين، −يسار) ويُنمّي الشك حسب مقدار الدوران."""
        self.heading = (self.heading + degrees) % 360.0
        self.uncertainty += DRIFT_PER_TURN * (abs(degrees) / 90.0)

    def set_pose(self, x: float, y: float, heading: float) -> None:
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
            "distance_total_m": round(self.distance_total, 2),
            "corrections": len(self.corrections),
        }
