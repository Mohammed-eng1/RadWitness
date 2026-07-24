# -*- coding: utf-8 -*-
"""
calibration.py — الدفعة1/أ: ملفات معايرة السرعة والدوران (أرضيات مختلفة)
=======================================================================
**ممنوع سرعة مفترضة في الكود.** المستخدم يعاير فعلياً: يسيّر الروبوت مدة
معلومة عند مستوى قوة، يقيس المسافة الحقيقية بشريط قياس ويدخلها → نحسب م/ث.
كذلك معايرة الدوران (لُف 360° وتصحيح درجة/ث). تُحفظ في ملفات لكل أرضية.

- `profiles/calibration/<اسم>.json`: سرعة كل مستوى قوة + معدل الدوران + التاريخ + ملاحظة.
- إن لم يوجد أي ملف → `CalibrationStore.any_exists()` تُرجع False فتمنع الواجهة بدء المسح.
"""
from __future__ import annotations

import json
import os
import re
import time
from dataclasses import dataclass, field, asdict

from pi.config import CALIBRATION_DIR


def compute_speed_mps(distance_m: float, duration_s: float) -> float:
    """السرعة من إجراء المعايرة: المسافة المقاسة ÷ المدة."""
    if duration_s <= 0:
        raise ValueError("المدة يجب أن تكون موجبة")
    return distance_m / duration_s


def compute_turn_rate_dps(degrees_turned: float, duration_s: float) -> float:
    """معدل الدوران: الدرجات الفعلية ÷ المدة (المستخدم يلاحظ الزيادة/النقص عن 360)."""
    if duration_s <= 0:
        raise ValueError("المدة يجب أن تكون موجبة")
    return degrees_turned / duration_s


@dataclass
class CalibrationProfile:
    """معايرة أرضية واحدة. speeds: {"50": م/ث, "75": ..} لكل مستوى قوة."""
    name: str
    speeds: dict = field(default_factory=dict)   # مفاتيح نصية لمستوى القوة
    turn_rate_dps: float = 0.0
    date: str = ""
    note: str = ""
    # جهد البطارية وقت المعايرة — مؤشر صلاحيتها (البند أ-5): السرعة تتأثر
    # بالجهد، وفارق > 1V عن لحظة التشغيل يستدعي تحذيراً في الواجهة.
    battery_v: float = 0.0

    def speed_for_power(self, power) -> float:
        """
        السرعة المعايرة لمستوى قوة، بـ**استيفاء خطي بين النقاط المقاسة**.
        - يقبل القوة كنسبة مئوية (40) أو كسر (0.4) — يوحّدهما إلى النسبة.
        - خارج نطاق القياس **لا يستقرئ**، بل يثبّت على أقرب طرف مقاس
          (الاستقراء خطر: العلاقة خطية داخل 0.1–0.5 فقط).
        """
        if not self.speeds:
            raise ValueError("لا سرعات في هذا الملف — عايِر أولاً")
        p = float(power)
        if 0.0 < p <= 1.0:                       # كسر (0.4) → نسبة (40)
            p *= 100.0
        pts = sorted((float(k), float(v)) for k, v in self.speeds.items())
        if p <= pts[0][0]:
            return pts[0][1]                     # تثبيت لا استقراء
        if p >= pts[-1][0]:
            return pts[-1][1]                    # تثبيت لا استقراء
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            if x0 <= p <= x1:
                if x1 == x0:
                    return y0
                return y0 + (y1 - y0) * (p - x0) / (x1 - x0)
        return pts[-1][1]

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "CalibrationProfile":
        return cls(name=d["name"], speeds=d.get("speeds", {}),
                   turn_rate_dps=d.get("turn_rate_dps", 0.0),
                   date=d.get("date", ""), note=d.get("note", ""),
                   battery_v=d.get("battery_v", 0.0))


_SAFE_NAME = re.compile(r"[^\w؀-ۿ \-]")   # يسمح عربي/إنجليزي/أرقام/مسافة/شرطة


def _safe_filename(name: str) -> str:
    cleaned = _SAFE_NAME.sub("", name).strip()
    if not cleaned:
        raise ValueError("اسم ملف معايرة غير صالح")
    return cleaned


class CalibrationStore:
    """إدارة ملفات المعايرة في مجلد واحد (حفظ/تحميل/سرد/حذف/إعادة تسمية)."""

    def __init__(self, directory: str = CALIBRATION_DIR):
        self.directory = directory
        os.makedirs(self.directory, exist_ok=True)

    def _path(self, name: str) -> str:
        return os.path.join(self.directory, _safe_filename(name) + ".json")

    def list_names(self) -> list:
        if not os.path.isdir(self.directory):
            return []
        return sorted(f[:-5] for f in os.listdir(self.directory) if f.endswith(".json"))

    def any_exists(self) -> bool:
        """هل يوجد أي ملف معايرة؟ (الواجهة تمنع المسح إن كانت False)."""
        return len(self.list_names()) > 0

    def save(self, profile: CalibrationProfile) -> None:
        if not profile.date:
            profile.date = time.strftime("%Y-%m-%d %H:%M")
        with open(self._path(profile.name), "w", encoding="utf-8") as f:
            json.dump(profile.to_dict(), f, ensure_ascii=False, indent=2)

    def load(self, name: str) -> CalibrationProfile:
        with open(self._path(name), encoding="utf-8") as f:
            return CalibrationProfile.from_dict(json.load(f))

    def delete(self, name: str) -> None:
        path = self._path(name)
        if os.path.exists(path):
            os.remove(path)

    def rename(self, old: str, new: str) -> None:
        prof = self.load(old)
        prof.name = new
        self.save(prof)
        self.delete(old)
