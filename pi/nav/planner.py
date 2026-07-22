# -*- coding: utf-8 -*-
"""
planner.py — الدفعة1/ج: إعادة التخطيط حول العوائق
==================================================
التفادي التفاعلي وحده لا يكفي: عائق كبير قد يسدّ ممراً، فنحتاج **طريقاً
بديلاً** للخلايا خلفه. بحث A* على الشبكة (حركة بأربعة اتجاهات) يتجنّب
الخلايا `blocked` و`unreachable`.

دورة العمل (يديرها الماسح): عائق يُكتشف → علّم الخلية blocked → أعد الحساب
من الموقع الحالي للهدف → لو وُجد مسار اتبعه، وإلا علّم الهدف unreachable.
حد `MAX_REPLANS_PER_TARGET` يمنع الحلقات اللانهائية.
"""
from __future__ import annotations

import heapq


def _heuristic(a, b) -> int:
    """مسافة مانهاتن (تتوافق مع الحركة بأربعة اتجاهات)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def find_path(grid, start, goal):
    """
    A* من start إلى goal (كلاهما (row,col)). يتجنّب blocked/unreachable.
    يسمح بأن تكون خلية البداية غير سالكة (الروبوت واقف فيها). يُعيد قائمة
    الخلايا من start إلى goal شاملةً الطرفين، أو None إن لا مسار.
    """
    if start == goal:
        return [start]
    # الهدف يجب أن يكون قابلاً للدخول
    if not grid.passable(*goal):
        return None

    open_heap = [(_heuristic(start, goal), 0, start)]
    came_from = {start: None}
    g_score = {start: 0}
    while open_heap:
        _, cost, current = heapq.heappop(open_heap)
        if current == goal:
            # إعادة بناء المسار
            path = [current]
            while came_from[path[-1]] is not None:
                path.append(came_from[path[-1]])
            path.reverse()
            return path
        for nb in grid.neighbors4(*current):
            if not grid.passable(*nb):
                continue
            tentative = cost + 1
            if tentative < g_score.get(nb, 1_000_000):
                g_score[nb] = tentative
                came_from[nb] = current
                heapq.heappush(open_heap, (tentative + _heuristic(nb, goal), tentative, nb))
    return None


def reachable_cells(grid, start):
    """مجموعة الخلايا القابلة للوصول من start (لتشخيص التغطية الممكنة)."""
    seen = {start}
    stack = [start]
    while stack:
        cur = stack.pop()
        for nb in grid.neighbors4(*cur):
            if nb not in seen and grid.passable(*nb):
                seen.add(nb)
                stack.append(nb)
    return seen
