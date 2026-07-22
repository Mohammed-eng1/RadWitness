# -*- coding: utf-8 -*-
"""
scanner.py — الدفعة1/ب: المسح الشبكي بالتمشيط (Boustrophedon)
=============================================================
مسار ثعباني من ركن الانطلاق: ممر ذهاباً، إزاحة صف، إياباً. عند كل خلية
يُسجَّل عد الجيجر (يحتفظ بالأقصى). الأولوية توجّه الترتيب ولا تُلغي التغطية.
عند شذوذ (>μ+3σ عبر Welford): وسم الخلية ورفع أولوية الجيران الثمانية.
يتكامل مع planner لإعادة التخطيط حول العوائق، والشبكة تُحفظ للاستئناف.

قابل للتشغيل في sim عبر SimWorld (بلا عتاد): يكتشف العوائق عند محاولة الدخول.
"""
from __future__ import annotations

import math

from pi.config import (
    CELL_DWELL_S, NAV_ANOMALY_SIGMA, NAV_ANOMALY_MIN_SAMPLES,
    MAX_REPLANS_PER_TARGET,
)
from pi.nav.planner import find_path

ANOMALY_NEIGHBOR_PRIORITY = 5     # أولوية الجيران الثمانية عند الشذوذ (تكثيف المسح)


class Welford:
    """إحصاء تدفّقي للمتوسط والانحراف المعياري (Welford) — لكشف الشذوذ."""

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self._m2 = 0.0

    def add(self, x: float) -> None:
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        self._m2 += d * (x - self.mean)

    def std(self) -> float:
        return math.sqrt(self._m2 / self.n) if self.n > 1 else 0.0

    def is_anomaly(self, x: float, sigma: float = NAV_ANOMALY_SIGMA) -> bool:
        if self.n < NAV_ANOMALY_MIN_SAMPLES:
            return False
        return x > self.mean + sigma * self.std()


class Scanner:
    """
    ماسح شبكي قابل للتشغيل في sim. يحتاج grid و world (SimWorld). في الدمج
    الحقيقي (دفعة لاحقة) يُستبدل world بقراءات الجيجر الحقيقية وجسر الروفر.
    """

    def __init__(self, grid, world, dwell_s: float = CELL_DWELL_S, start_pos=None):
        self.grid = grid
        self.world = world
        self.dwell_s = dwell_s
        self.stats = Welford()
        self.base_order = self._boustrophedon_order()
        self.current = start_pos if start_pos is not None else grid.start_cell()
        self.replans = {}
        self.anomalies = []
        self.log = []
        self.mission_time_s = 0.0

    # ── توليد مسار التمشيط من ركن الانطلاق ────────────────────────
    def _boustrophedon_order(self):
        sr, sc = self.grid.start_cell()
        rows_order = (range(self.grid.rows) if sr == 0
                      else range(self.grid.rows - 1, -1, -1))
        left_to_right = (sc == 0)
        order = []
        for r in rows_order:
            cols = (range(self.grid.cols) if left_to_right
                    else range(self.grid.cols - 1, -1, -1))
            order.extend((r, c) for c in cols)
            left_to_right = not left_to_right
        return order

    def _remaining(self):
        return [rc for rc in self.base_order
                if self.grid.passable(*rc) and not self.grid.get(*rc).visited]

    def _next_target(self):
        """الأولوية أولاً (الأعلى، ثم ترتيب التمشيط)، ثم الخلية التالية عادةً."""
        rem = self._remaining()
        if not rem:
            return None
        prioritized = [rc for rc in rem if self.grid.get(*rc).priority > 0]
        if prioritized:
            return max(prioritized, key=lambda rc: self.grid.get(*rc).priority)
        return rem[0]

    # ── زيارة خلية: تسجيل + كشف شذوذ ──────────────────────────────
    def _visit(self, cell) -> None:
        r, c = cell
        if self.grid.get(r, c).visited:
            return
        x, y = self.grid.cell_center(r, c)
        cpm, usvh = self.world.reading_at(x, y)
        # كشف الشذوذ **قبل** إضافة القراءة للإحصاء (كي لا يرفع المصدرُ الأساس)
        anomaly = self.stats.is_anomaly(cpm)
        self.stats.add(cpm)
        self.grid.update_reading(x, y, cpm, usvh)   # يعلّم visited ويحفظ الأقصى
        self.mission_time_s += self.dwell_s
        if anomaly:
            self.anomalies.append({"cell": cell, "cpm": round(cpm, 1),
                                   "usvh": round(usvh, 3)})
            for nb in self.grid.neighbors8(r, c):
                self.grid.set_priority(*nb, ANOMALY_NEIGHBOR_PRIORITY)
            self.log.append(("anomaly", cell, round(cpm, 1)))

    # ── تنفيذ المسح كاملاً في sim ─────────────────────────────────
    def run_sim(self, save_cb=None, max_steps: int = 100_000) -> dict:
        self._visit(self.current)               # سجّل خلية الانطلاق
        steps = 0
        while steps < max_steps:
            steps += 1
            target = self._next_target()
            if target is None:
                break                            # اكتملت التغطية الممكنة
            path = find_path(self.grid, self.current, target)
            if path is None:
                self.grid.mark_unreachable(*target)
                self.log.append(("unreachable_no_path", target))
                continue
            # اتبع المسار خلية بخلية، مكتشفاً العوائق عند محاولة الدخول
            hit_obstacle = False
            for nxt in path[1:]:
                if self.world.is_obstacle(*nxt) and not self.grid.get(*nxt).blocked:
                    self.grid.mark_blocked(*nxt)
                    self.replans[target] = self.replans.get(target, 0) + 1
                    self.log.append(("obstacle_detected", nxt))
                    hit_obstacle = True
                    break
                self.current = nxt
                self._visit(self.current)        # سجّل كل خلية نمرّ بها
            if hit_obstacle:
                if self.replans[target] > MAX_REPLANS_PER_TARGET:
                    self.grid.mark_unreachable(*target)
                    self.log.append(("unreachable_max_replans", target))
                continue                         # أعد التخطيط
            if save_cb:                          # حفظ دوري للاستئناف
                save_cb(self.grid)
        return self.summary(steps)

    def summary(self, steps: int = 0) -> dict:
        c = self.grid.counts()
        return {
            "coverage_pct": round(self.grid.coverage_pct(), 1),
            "coverage_text": self.grid.coverage_text(),
            "counts": c,
            "anomalies": len(self.anomalies),
            "steps": steps,
            "mission_time_s": round(self.mission_time_s, 1),
            "replans": dict(self.replans),
        }
