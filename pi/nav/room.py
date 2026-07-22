# -*- coding: utf-8 -*-
"""
room.py — البند 1: تعريف نطاق البحث + شبكة الإشغال (Occupancy Grid)
==================================================================
طبقة بيانات خالصة (بلا عتاد) هي **مصدر الحقيقة** للتغطية والخريطة الحرارية
والاستئناف بعد الانقطاع. لا GPS داخل المباني — كل الإحداثيات نسبية للغرفة.

نظام الإحداثيات (نسبي للغرفة، لا شمال حقيقي):
    - اتجاه الغرفة 0° = الجدار الأمامي (يُضبط بزر «صفّر الاتجاه»).
    - المحور x: 0 (يسار) → width_m (يمين).
    - المحور y: 0 (خلف)  → length_m (أمام، نحو الجدار الأمامي).
    - خلية الشبكة 50×50 سم؛ الصف row ~ y، العمود col ~ x.

المكوّنات:
    Cell           خلية واحدة (visited/max_usvh/max_cpm/blocked/priority/timestamp).
    Room           أبعاد الغرفة + ركن الانطلاق + التباعد + مرجع الاتجاه.
    OccupancyGrid  مصفوفة الخلايا + الفهرسة + التحديث + التغطية + الحفظ/التحميل.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, asdict

CELL_SIZE_M = 0.5                       # حجم الخلية: 50×50 سم (ثابت)

# أركان الانطلاق الأربعة (نسبة إلى اتجاه الغرفة 0° = الجدار الأمامي)
CORNERS = ("back_left", "back_right", "front_left", "front_right")


@dataclass
class Cell:
    """خلية إشغال واحدة — الحقول كما نصّ البند 1 + unreachable (الدفعة 1)."""
    visited: bool = False
    max_usvh: float = 0.0
    max_cpm: float = 0.0
    blocked: bool = False
    priority: int = 0
    timestamp: float = 0.0             # لحظة آخر تحديث (time.time())
    unreachable: bool = False          # محاصرة بعوائق — تُستثنى من مقام التغطية


@dataclass
class Room:
    """
    أبعاد الغرفة وإعداداتها. مرجع الاتجاه (heading_ref) يبقى None حتى يضغط
    المستخدم «صفّر الاتجاه» وهو موجّه للجدار الأمامي.
    """
    length_m: float                    # الطول (محور y)
    width_m: float                     # العرض (محور x)
    start_corner: str = "back_left"
    scan_spacing_m: float = 0.5        # إزاحة صف المسح (البند 4)
    heading_ref: float | None = None   # heading المطلق الذي يمثّل 0° للغرفة

    def __post_init__(self):
        if self.length_m <= 0 or self.width_m <= 0:
            raise ValueError("أبعاد الغرفة يجب أن تكون موجبة")
        if self.start_corner not in CORNERS:
            raise ValueError(f"ركن غير صالح: {self.start_corner} (المتاح: {CORNERS})")
        if self.scan_spacing_m <= 0:
            raise ValueError("تباعد المسح يجب أن يكون موجباً")

    # ── مرجع الاتجاه (زر «صفّر الاتجاه») ───────────────────────────
    def set_heading_reference(self, current_abs_heading: float) -> None:
        """يجعل heading الحالي (من BNO055) هو 0° المرجعي للغرفة."""
        self.heading_ref = current_abs_heading % 360.0

    def to_room_heading(self, abs_heading: float):
        """يحوّل heading مطلقاً إلى زاوية نسبية للغرفة [0,360). None قبل التصفير."""
        if self.heading_ref is None:
            return None
        return (abs_heading - self.heading_ref) % 360.0

    @property
    def heading_zeroed(self) -> bool:
        return self.heading_ref is not None

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Room":
        return cls(
            length_m=d["length_m"], width_m=d["width_m"],
            start_corner=d.get("start_corner", "back_left"),
            scan_spacing_m=d.get("scan_spacing_m", 0.5),
            heading_ref=d.get("heading_ref"),
        )


class OccupancyGrid:
    """شبكة إشغال 50×50 سم فوق الغرفة — مصدر الحقيقة للتغطية والخريطة."""

    def __init__(self, room: Room):
        self.room = room
        self.cols = max(1, math.ceil(room.width_m / CELL_SIZE_M))   # x
        self.rows = max(1, math.ceil(room.length_m / CELL_SIZE_M))  # y
        self.cells: list[list[Cell]] = [
            [Cell() for _ in range(self.cols)] for _ in range(self.rows)
        ]

    # ── الفهرسة بين المتر والخلية ─────────────────────────────────
    def cell_index(self, x_m: float, y_m: float):
        """(row, col) للخلية التي تقع فيها النقطة (x,y) بالمتر، أو None خارج الغرفة."""
        if x_m < 0 or y_m < 0 or x_m > self.room.width_m or y_m > self.room.length_m:
            return None
        col = min(self.cols - 1, int(x_m / CELL_SIZE_M))
        row = min(self.rows - 1, int(y_m / CELL_SIZE_M))
        return row, col

    def cell_center(self, row: int, col: int):
        """مركز الخلية (x,y) بالمتر."""
        return ((col + 0.5) * CELL_SIZE_M, (row + 0.5) * CELL_SIZE_M)

    def get(self, row: int, col: int) -> Cell:
        return self.cells[row][col]

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    # ── ركن الانطلاق → خلية البداية ───────────────────────────────
    def start_cell(self):
        """(row, col) لخلية ركن الانطلاق."""
        top = self.rows - 1       # أمام (y كبير)
        right = self.cols - 1     # يمين (x كبير)
        return {
            "back_left":   (0, 0),
            "back_right":  (0, right),
            "front_left":  (top, 0),
            "front_right": (top, right),
        }[self.room.start_corner]

    # ── التحديث أثناء المسح ───────────────────────────────────────
    def update_reading(self, x_m: float, y_m: float, cpm: float, usvh: float,
                       mark_visited: bool = True) -> bool:
        """يسجّل قراءة جيجر في خلية الموقع (يحتفظ بالأقصى). يُعيد نجاح التحديث."""
        idx = self.cell_index(x_m, y_m)
        if idx is None:
            return False
        c = self.cells[idx[0]][idx[1]]
        c.max_cpm = max(c.max_cpm, cpm)
        c.max_usvh = max(c.max_usvh, usvh)
        if mark_visited:
            c.visited = True
        c.timestamp = time.time()
        return True

    def mark_blocked(self, row: int, col: int) -> None:
        if self.in_bounds(row, col):
            c = self.cells[row][col]
            c.blocked = True
            c.timestamp = time.time()

    def mark_unreachable(self, row: int, col: int) -> None:
        if self.in_bounds(row, col):
            c = self.cells[row][col]
            c.unreachable = True
            c.timestamp = time.time()

    def passable(self, row: int, col: int) -> bool:
        """خلية يمكن دخولها (داخل الحدود وغير محجوبة/غير قابلة للوصول)."""
        return self.in_bounds(row, col) and not (
            self.cells[row][col].blocked or self.cells[row][col].unreachable
        )

    def neighbors4(self, row: int, col: int):
        """الجيران الأربعة (شمال/جنوب/شرق/غرب) داخل الحدود."""
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            if self.in_bounds(row + dr, col + dc):
                yield row + dr, col + dc

    def neighbors8(self, row: int, col: int):
        """الجيران الثمانية داخل الحدود (لتكثيف المسح حول الشذوذ)."""
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if (dr or dc) and self.in_bounds(row + dr, col + dc):
                    yield row + dr, col + dc

    def set_priority(self, row: int, col: int, priority: int) -> None:
        if self.in_bounds(row, col):
            c = self.cells[row][col]
            c.priority = max(c.priority, priority)
            c.timestamp = time.time()

    # ── التغطية ───────────────────────────────────────────────────
    def counts(self) -> dict:
        total = self.rows * self.cols
        visited = blocked = unreachable = 0
        for r in range(self.rows):
            for c in range(self.cols):
                cell = self.cells[r][c]
                if cell.blocked:
                    blocked += 1
                elif cell.unreachable:
                    unreachable += 1
                elif cell.visited:
                    visited += 1
        reachable = total - blocked - unreachable   # المقام يستثني المحجوب وغير القابل للوصول
        return {"total": total, "visited": visited, "blocked": blocked,
                "unreachable": unreachable, "reachable": reachable}

    def coverage_pct(self) -> float:
        """نسبة الخلايا المزارة من القابلة للوصول (المحجوبة/غير القابلة تُستبعَد)."""
        c = self.counts()
        return 100.0 * c["visited"] / c["reachable"] if c["reachable"] > 0 else 100.0

    def coverage_text(self) -> str:
        """نص شفّاف يعرض الرقمين معاً (كما نصّ البند ب)."""
        c = self.counts()
        return (f"التغطية {self.coverage_pct():.0f}% "
                f"({c['visited']} من {c['reachable']} قابلة للوصول، "
                f"{c['blocked']} محجوبة، {c['unreachable']} غير قابلة)")

    # ── الحفظ/التحميل (للاستئناف بعد الانقطاع وللخريطة) ────────────
    def to_dict(self) -> dict:
        return {
            "room": self.room.to_dict(),
            "cols": self.cols, "rows": self.rows,
            "cells": [[asdict(c) for c in row] for row in self.cells],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "OccupancyGrid":
        grid = cls(Room.from_dict(d["room"]))
        for r, row in enumerate(d["cells"]):
            for c, cd in enumerate(row):
                grid.cells[r][c] = Cell(**cd)
        return grid

    def heatmap_cells(self) -> list:
        """قائمة مبسّطة للواجهة: مركز كل خلية + حالتها (للخريطة الحرارية)."""
        out = []
        for r in range(self.rows):
            for c in range(self.cols):
                cell = self.cells[r][c]
                x, y = self.cell_center(r, c)
                out.append({
                    "row": r, "col": c, "x": round(x, 2), "y": round(y, 2),
                    "visited": cell.visited, "blocked": cell.blocked,
                    "unreachable": cell.unreachable, "priority": cell.priority,
                    "max_usvh": round(cell.max_usvh, 3), "max_cpm": round(cell.max_cpm, 1),
                })
        return out
