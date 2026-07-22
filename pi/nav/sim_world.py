# -*- coding: utf-8 -*-
"""
sim_world.py — عالم محاكاة لاختبار منطق الملاحة برمجياً (بلا عتاد)
=================================================================
يوفّر «الحقيقة الأرضية» لاختبار الماسح/المخطّط/السلامة في sim:
- حقل إشعاعي حقيقي: خلفية + مصدر بقانون التربيع العكسي (يؤكّده العدّاد فقط).
- عوائق حقيقية على الشبكة (يكتشفها الروبوت عند محاولة الدخول).
- مسافة أمامية وهمية (شعاع حتى أقرب جدار/عائق) — لاختبار السلامة التفاعلية.
"""
from __future__ import annotations

import math

from pi.config import CPM_PER_USVH
from pi.nav.room import CELL_SIZE_M


class SimWorld:
    def __init__(self, room, source_xy=None, source_cpm_1m=30000.0,
                 bg_cpm=22.0, obstacles=None):
        self.room = room
        self.source_xy = source_xy            # (x,y) بالمتر أو None
        self.source_cpm_1m = source_cpm_1m
        self.bg_cpm = bg_cpm
        self.obstacles = set(obstacles or [])  # مجموعة (row,col) عوائق حقيقية

    def reading_at(self, x: float, y: float):
        """(cpm, usvh) عند نقطة — خلفية + مصدر بالتربيع العكسي."""
        cpm = self.bg_cpm
        if self.source_xy is not None:
            d = math.hypot(x - self.source_xy[0], y - self.source_xy[1])
            d = max(0.3, d)
            cpm += self.source_cpm_1m / (d * d)
        return cpm, cpm / CPM_PER_USVH

    def is_obstacle(self, row: int, col: int) -> bool:
        return (row, col) in self.obstacles

    def front_distance_cm(self, x: float, y: float, heading_deg: float) -> float:
        """مسافة أقرب جدار/عائق أمام الروبوت (لاختبار السلامة التفاعلية)."""
        hd = math.radians(heading_deg)
        dx, dy = math.sin(hd), math.cos(hd)      # heading 0=+y، 90=+x
        step = 0.02
        dist = 0.0
        while dist < 10.0:
            dist += step
            px, py = x + dx * dist, y + dy * dist
            if px < 0 or py < 0 or px > self.room.width_m or py > self.room.length_m:
                return dist * 100.0              # جدار الغرفة
            col = min(int(px / CELL_SIZE_M), 10_000)
            row = min(int(py / CELL_SIZE_M), 10_000)
            if (row, col) in self.obstacles:
                return dist * 100.0              # عائق
        return 1000.0
