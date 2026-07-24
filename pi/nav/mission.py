# -*- coding: utf-8 -*-
"""
mission.py — محرّك محاكاة المهمة الخطوي (يقود وحدات pi/nav حيّاً)
=================================================================
خلاف Scanner.run_sim (يكمل دفعة واحدة)، هذا المحرّك يتقدّم **خطوة خلية
واحدة لكل tick** ليُعرض حيّاً في الواجهة: قابل للإيقاف/الاستئناف، يستقبل
عوائق وهمية أثناء التنفيذ فيعيد التخطيط ويُظهر المسار البديل فوراً.

منطق خالص بلا عتاد — يعمل على ويندوز. يستخدم:
    Room/OccupancyGrid, boustrophedon_order, find_path (A*),
    DeadReckoning (موقع + دائرة عدم يقين + تصحيح جدار), Welford (شذوذ),
    SimWorld (حقل إشعاعي + عوائق), risk.classify.
"""
from __future__ import annotations

import csv
import io
import math
import time
from pathlib import Path

from pi.config import (
    CELL_DWELL_S, MAX_REPLANS_PER_TARGET, DRIFT_PER_METER,
    DRIVE_POWER_DEFAULT, MEASURED_SPEEDS, MEASURED_SPEEDS_LOW_BATT, LOW_BATT_CALIB_V,
)
from pi.nav.room import Room, OccupancyGrid, CELL_SIZE_M
from pi.nav.scanner import boustrophedon_order, Welford, ANOMALY_NEIGHBOR_PRIORITY
from pi.nav.planner import find_path
from pi.nav.deadreckoning import DeadReckoning
from pi.nav.calibration import CalibrationProfile
from pi.nav.sim_world import SimWorld
from pi.ai.risk import classify
from pi.rover.bridge import WaveRoverBridge
from pi.rover import battery as batt

BASE_TICK_S = 0.25              # زمن خطوة الخلية عند مضاعف ×1
DEFAULT_POWER = DRIVE_POWER_DEFAULT   # 0.40 — ضمن الحد الآمن (لا التفاف فيرموير)
WALL_CORRECTION_RANGE_M = 0.8   # لا يُحاول تصحيح إلا قرب جدار (لإظهار نمو/تصغّر الشك)
MEASURE_NOISE_M = 0.03          # ضجيج قياس الألترا سونيك الوهمي

# حالات المهمة
IDLE, RUNNING, PAUSED, DONE, RETURNING, ESTOP = (
    "idle", "running", "paused", "done", "returning", "estop")


def default_sim_profile(battery_v: float = 0.0) -> CalibrationProfile:
    """
    ملف المعايرة الافتراضي: **القيم المقاسة ببطارية ممتلئة على السيراميك**
    (0.2→0.250، 0.4→0.590، 0.5→0.750 م/ث). يسجّل جهد البطارية وقت المعايرة.
    """
    return CalibrationProfile(
        name="سيراميك - بطارية ممتلئة", speeds=dict(MEASURED_SPEEDS),
        turn_rate_dps=70.0, date=time.strftime("%Y-%m-%d %H:%M"),
        note="مقاسة على العتاد ببطارية ممتلئة (السرعة ≈ 1.5 × القوة، صالحة 0.1–0.5)",
        battery_v=float(battery_v or 0.0))


def legacy_low_battery_profile() -> CalibrationProfile:
    """
    المعايرة القديمة **متقادمة** — أُخذت ببطارية منخفضة (~10.1V) وفارقها ~25%
    عند نفس القوة. تُحفظ موسومة بجهدها المنخفض ليحذّر منها فارقُ الجهد.
    """
    return CalibrationProfile(
        name="سيراميك - بطارية منخفضة متقادمة",
        speeds=dict(MEASURED_SPEEDS_LOW_BATT), turn_rate_dps=70.0,
        date="2026-07-22", note="⚠ متقادمة — قيست ببطارية منخفضة، فارق ~25%",
        battery_v=LOW_BATT_CALIB_V)


def _heading_between(a, b) -> float:
    """اتجاه الغرفة من الخلية a إلى المجاورة b (0=+y أمام، 90=+x يمين)."""
    dr, dc = b[0] - a[0], b[1] - a[1]
    if dc > 0:
        return 90.0
    if dc < 0:
        return 270.0
    if dr < 0:
        return 180.0
    return 0.0


def _ang_signed(frm: float, to: float) -> float:
    return (to - frm + 180.0) % 360.0 - 180.0


class MissionSim:
    def __init__(self):
        self.profile = None
        # جسر الروفر (محاكاة على ويندوز؛ يُبدَّل إلى real على الراسبري بعلم واحد)
        self.rover = WaveRoverBridge(mode="sim")
        self._reset_full()

    # ── تهيئة ────────────────────────────────────────────────────
    def _reset_full(self):
        self.room = None
        self.grid = None
        self.world = None
        self.state = IDLE
        self.current = (0, 0)
        self.heading = 0.0
        self.dr = None
        self.stats = Welford()
        self.planned_path = []
        self.trail = []
        self.replans = {}
        self.anomalies = []
        self.events = []
        self.records = []
        self.dirty = set()
        self.speed_mult = 1
        self.mission_time_s = 0.0
        self.last_reading = {"cpm": 0.0, "usvh": 0.0, "risk": "Safe", "color": "#22c55e"}
        self._returning = False
        self._batt_level = None
        self._rth_triggered = False
        self._clamp_seen = 0

    def configure_room(self, length_m, width_m, start_corner="back_left",
                       scan_spacing_m=0.5, source_xy=None, bg_cpm=22.0):
        self.room = Room(length_m=float(length_m), width_m=float(width_m),
                         start_corner=start_corner, scan_spacing_m=float(scan_spacing_m))
        self.grid = OccupancyGrid(self.room)
        self.world = SimWorld(self.room, source_xy=source_xy, bg_cpm=bg_cpm)
        self.state = IDLE
        self.current = self.grid.start_cell()
        self.heading = 0.0
        self.stats = Welford()
        self.planned_path = []
        self.trail = [self.current]
        self.replans = {}
        self.anomalies = []
        self.events = []
        self.records = []
        self.dirty = set()
        self.mission_time_s = 0.0
        self._returning = False
        cx, cy = self.grid.cell_center(*self.current)
        self._log("room_configured",
                  f"غرفة {length_m}×{width_m}م، ركن {start_corner}")

    def set_calibration(self, profile: CalibrationProfile):
        self.profile = profile

    def set_source(self, x, y):
        if self.world is not None:
            self.world.source_xy = (float(x), float(y))

    def set_speed(self, mult: int):
        self.speed_mult = max(1, int(mult))

    def tick_period(self) -> float:
        return BASE_TICK_S / self.speed_mult

    # ── أوامر المهمة ─────────────────────────────────────────────
    def start(self) -> dict:
        if self.grid is None:
            return {"ok": False, "error": "عرّف الغرفة أولاً"}
        if self.profile is None:
            return {"ok": False, "error": "لا يوجد ملف معايرة — عايِر أولاً"}
        self.current = self.grid.start_cell()
        cx, cy = self.grid.cell_center(*self.current)
        self.dr = DeadReckoning(self.room, self.profile, cx, cy, 0.0)
        self.heading = 0.0
        self.trail = [self.current]
        self._returning = False
        self.state = RUNNING
        self._visit(self.current)
        self._log("mission_start", "بدء المسح الذاتي")
        return {"ok": True}

    def pause(self):
        if self.state == RUNNING:
            self.state = PAUSED
            self._log("pause", "إيقاف مؤقت")

    def resume(self):
        if self.state in (PAUSED, ESTOP):
            self.state = RUNNING
            self._log("resume", "استئناف")

    def estop(self):
        self.state = ESTOP
        self.planned_path = []
        self._log("estop", "إيقاف طوارئ")

    def return_home(self):
        if self.grid is not None and self.state in (RUNNING, PAUSED, DONE):
            self._returning = True
            self.state = RUNNING
            self._log("return_home", "عودة لنقطة الانطلاق")

    def toggle_obstacle(self, row, col) -> dict:
        """أداة اختبار: وضع/إزالة عائق وهمي (الروبوت يعيد التخطيط فوراً)."""
        if self.grid is None or not self.grid.in_bounds(row, col):
            return {"ok": False}
        if (row, col) == self.current:
            return {"ok": False, "error": "لا يمكن وضع عائق تحت الروبوت"}
        cell = self.grid.get(row, col)
        if cell.blocked:
            self.grid.set_blocked(row, col, False)
            self.world.obstacles.discard((row, col))
            self._log("obstacle_removed", f"أُزيل عائق ({row},{col})")
        else:
            self.grid.set_blocked(row, col, True)
            self.world.obstacles.add((row, col))
            self._log("obstacle_placed", f"عائق وهمي ({row},{col})")
        self.dirty.add((row, col))
        return {"ok": True}

    # ── الخطوة الواحدة ───────────────────────────────────────────
    def poll_power_clamp(self):
        """ينقل تحذيرات قصّ القوة من الجسر إلى سجل أحداث الواجهة."""
        if self.rover.clamp_count > self._clamp_seen:
            self._clamp_seen = self.rover.clamp_count
            self._log("power_clamp", self.rover.last_clamp_msg or "قُصّت القوة")

    def poll_battery(self):
        """
        يُستدعى دورياً من حلقة السيرفر **مهما كانت حالة المهمة** — الجهد يجب
        أن يظهر في الواجهة دائماً (أ-3)، لا أثناء المسح فقط.
        """
        return self._check_battery()

    def _check_battery(self):
        """يطبّق عتبات الجهد الإلزامية (أ-3) على مسار المهمة."""
        self.rover.sim_set_moving(self.state == RUNNING)
        info = self.rover.check_battery()
        if info["action"] == batt.ACTION_STOP:
            if self.state == RUNNING:
                self._log("battery", f"⚠ جهد حرج {info['v']}V — إيقاف المحركات فوراً")
                self.estop()
        elif info["action"] == batt.ACTION_RTH:
            if self.state == RUNNING and not self._rth_triggered:
                self._rth_triggered = True
                self._log("battery", f"⚠ جهد منخفض {info['v']}V — عودة إجبارية")
                self.return_home()
        elif info["level"] == "good" and self._batt_level != "good":
            self._log("battery", f"تنبيه: جهد {info['v']}V (جيد)")
        self._batt_level = info["level"]
        return info

    def sensors(self) -> dict:
        """قراءات القرب الوهمية للواجهة: ألترا سونيك أمامي + IR الركنين."""
        if self.grid is None or self.dr is None:
            return {"ultrasonic_cm": None, "ir_left": 1, "ir_right": 1,
                    "cpm": self.last_reading["cpm"]}
        front_cm = self.world.front_distance_cm(self.dr.x, self.dr.y, self.heading)
        # اتجاه الأمام واليسار بالخلايا (dx=عمود، dy=صف)
        th = math.radians(self.heading)
        fx, fy = round(math.sin(th)), round(math.cos(th))
        lx, ly = round(-math.cos(th)), round(math.sin(th))
        r, c = self.current

        def occupied(rr, cc):
            # خارج الغرفة = جدار = عائق (0)، أو خلية عائق حقيقية
            if not self.grid.in_bounds(rr, cc):
                return 0
            return 0 if self.world.is_obstacle(rr, cc) else 1

        ir_left = occupied(r + fy + ly, c + fx + lx)
        ir_right = occupied(r + fy - ly, c + fx - lx)
        return {"ultrasonic_cm": round(front_cm, 1),
                "ir_left": ir_left, "ir_right": ir_right,
                "cpm": self.last_reading["cpm"]}

    def tick(self):
        self._check_battery()
        if self.state != RUNNING or self.grid is None:
            return
        target = self._current_start() if self._returning else self._next_target()
        if target is None:
            self._finish()
            return
        path = find_path(self.grid, self.current, target)
        if path is None:
            if self._returning:
                self._finish()
            else:
                self.grid.mark_unreachable(*target)
                self.dirty.add(target)
                self._log("unreachable", f"هدف غير قابل للوصول ({target[0]},{target[1]})")
            return
        self.planned_path = path
        if len(path) < 2:
            if self._returning:
                self._finish()
            return
        nxt = path[1]
        # اكتشاف عائق عند محاولة الدخول
        if self.world.is_obstacle(*nxt) and not self.grid.get(*nxt).blocked:
            self.grid.mark_blocked(*nxt)
            self.dirty.add(nxt)
            self.replans[target] = self.replans.get(target, 0) + 1
            self._log("obstacle_detected", f"عائق مكتشَف ({nxt[0]},{nxt[1]}) → إعادة تخطيط")
            if self.replans[target] > MAX_REPLANS_PER_TARGET:
                self.grid.mark_unreachable(*target)
                self.dirty.add(target)
                self._log("unreachable", f"هدف محاصر ({target[0]},{target[1]}) بعد {MAX_REPLANS_PER_TARGET} محاولات")
            return
        # حرّك خطوة خلية واحدة
        new_heading = _heading_between(self.current, nxt)
        if abs(_ang_signed(self.heading, new_heading)) > 1.0:
            self.dr.turn(_ang_signed(self.heading, new_heading))   # ينمّي شك الدوران
            self.heading = new_heading
        speed = self.profile.speed_for_power(DEFAULT_POWER)
        self.dr.move("F", DEFAULT_POWER, CELL_SIZE_M / max(0.05, speed))  # ينمّي شك المسافة
        self.current = nxt
        self.trail.append(self.current)
        self.mission_time_s += CELL_DWELL_S
        self._visit(self.current)
        self._try_wall_correction()
        if self._returning and self.current == self.grid.start_cell():
            self._finish()

    def _current_start(self):
        s = self.grid.start_cell()
        return None if self.current == s else s

    # ── مساعدات داخلية ───────────────────────────────────────────
    def _base_order(self):
        return boustrophedon_order(self.grid)

    def _next_target(self):
        rem = [rc for rc in self._base_order()
               if self.grid.passable(*rc) and not self.grid.get(*rc).visited]
        if not rem:
            return None
        prioritized = [rc for rc in rem if self.grid.get(*rc).priority > 0]
        if prioritized:
            return max(prioritized, key=lambda rc: self.grid.get(*rc).priority)
        return rem[0]

    def _visit(self, cell):
        r, c = cell
        gc = self.grid.get(r, c)
        already = gc.visited
        x, y = self.grid.cell_center(r, c)
        cpm, usvh = self.world.reading_at(x, y)
        anomaly = (not already) and self.stats.is_anomaly(cpm)
        if not already:
            self.stats.add(cpm)
        self.grid.update_reading(x, y, cpm, usvh)
        self.dirty.add(cell)
        risk = classify(usvh)
        self.last_reading = {"cpm": round(cpm, 1), "usvh": round(usvh, 3),
                             "risk": risk["risk"], "color": risk["color"]}
        self.records.append({
            "time": round(self.mission_time_s, 1), "row": r, "col": c,
            "x": round(x, 2), "y": round(y, 2), "cpm": round(cpm, 1),
            "usvh": round(usvh, 3), "risk": risk["risk"],
            "heading": round(self.heading, 0),
            "uncertainty_m": round(self.dr.uncertainty, 2) if self.dr else 0.0,
            "anomaly": 1 if anomaly else 0,
        })
        if anomaly:
            self.anomalies.append({"cell": cell, "cpm": round(cpm, 1), "usvh": round(usvh, 3)})
            for nb in self.grid.neighbors8(r, c):
                self.grid.set_priority(*nb, ANOMALY_NEIGHBOR_PRIORITY)
                self.dirty.add(nb)
            self._log("anomaly", f"شذوذ ({r},{c}) عند {cpm:.0f} CPM → تكثيف الجيران")

    def _try_wall_correction(self):
        """قرب جدار مواجَه: قياس ألترا سونيك وهمي → تصحيح إن تحقق الشرطان."""
        if self.dr is None:
            return
        front_cm = self.world.front_distance_cm(self.dr.x, self.dr.y, self.heading)
        measured_m = front_cm / 100.0
        if measured_m > WALL_CORRECTION_RANGE_M:
            return                                   # بعيد عن الجدار — دع الشك ينمو
        measured_m += MEASURE_NOISE_M                # ضجيج واقعي بسيط
        res = self.dr.try_front_wall_correction(measured_m)
        if res.get("corrected"):
            self._log("wall_correction",
                      f"تصحيح جدار (محور {res['axis']}, Δ={res['delta']}م) → تصغير الشك")

    def _finish(self):
        if self._returning:
            self.state = IDLE
            self._returning = False
            self._log("home", "وصل نقطة الانطلاق")
        else:
            self.state = DONE
            self._log("mission_end",
                      f"انتهى المسح — {self.grid.coverage_text()}")
        self.planned_path = []

    def _log(self, kind, msg):
        self.events.append({"t": round(time.time(), 2), "kind": kind, "msg": msg})
        if len(self.events) > 300:
            self.events = self.events[-300:]

    # ── الحالة والبثّ ────────────────────────────────────────────
    def drain_dirty(self):
        cells = [self._cell_dict(r, c) for (r, c) in self.dirty]
        self.dirty.clear()
        return cells

    def _cell_dict(self, r, c):
        cell = self.grid.get(r, c)
        return {"row": r, "col": c, "visited": cell.visited, "blocked": cell.blocked,
                "unreachable": cell.unreachable, "priority": cell.priority,
                "max_usvh": round(cell.max_usvh, 3), "max_cpm": round(cell.max_cpm, 1)}

    def grid_meta(self):
        if self.grid is None:
            return None
        return {
            "rows": self.grid.rows, "cols": self.grid.cols,
            "cell_size_m": CELL_SIZE_M,
            "length_m": self.room.length_m, "width_m": self.room.width_m,
            "start_cell": list(self.grid.start_cell()),
            "cells": [self._cell_dict(r, c)
                      for r in range(self.grid.rows) for c in range(self.grid.cols)],
        }

    def state_dict(self, include_full_grid=False):
        if self.grid is None:
            return {"t": "sim", "state": "no_room"}
        counts = self.grid.counts()
        robot = None
        if self.dr is not None:
            robot = {"row": self.current[0], "col": self.current[1],
                     "x": round(self.dr.x, 3), "y": round(self.dr.y, 3),
                     "heading": round(self.heading, 0),
                     "uncertainty_m": round(self.dr.uncertainty, 3)}
        else:
            cx, cy = self.grid.cell_center(*self.current)
            robot = {"row": self.current[0], "col": self.current[1],
                     "x": round(cx, 3), "y": round(cy, 3),
                     "heading": round(self.heading, 0), "uncertainty_m": 0.0}
        msg = {
            "t": "sim", "state": self.state,
            "robot": robot,
            "planned_path": [list(p) for p in self.planned_path],
            "trail": [list(p) for p in self.trail[-400:]],
            "coverage_pct": round(self.grid.coverage_pct(), 1),
            "coverage_text": self.grid.coverage_text(),
            "counts": counts,
            "reading": self.last_reading,
            "mission_time_s": round(self.mission_time_s, 1),
            "corrections": len(self.dr.corrections) if self.dr else 0,
            "anomalies": len(self.anomalies),
            "speed_mult": self.speed_mult,
            "events": self.events[-40:],
            "anomaly_cells": [list(a["cell"]) for a in self.anomalies],
            "battery": batt.classify(self.rover.voltage()),
            "sensors": self.sensors(),
            "rover": {"mode": self.rover.mode, "error": self.rover.error,
                      "gyro_bias": round(self.rover.gyro_bias, 4),
                      "bias_calibrated": self.rover.bias_calibrated},
            "calib_warning": batt.calibration_voltage_warning(
                self.profile.battery_v if self.profile else 0.0, self.rover.voltage()),
        }
        if include_full_grid:
            msg["grid"] = self.grid_meta()
            msg["cells_full"] = True
        else:
            changed = self.drain_dirty()
            if changed:
                msg["cells"] = changed
        return msg

    # ── تصدير ────────────────────────────────────────────────────
    def csv_bytes(self) -> bytes:
        buf = io.StringIO()
        cols = ["time", "row", "col", "x", "y", "cpm", "usvh", "risk",
                "heading", "uncertainty_m", "anomaly"]
        w = csv.DictWriter(buf, fieldnames=cols)
        w.writeheader()
        for row in self.records:
            w.writerow(row)
        return buf.getvalue().encode("utf-8-sig")   # BOM لدعم العربية في Excel

    def report(self) -> dict:
        max_rec = max(self.records, key=lambda r: r["usvh"], default=None)
        return {
            "room": {"length_m": self.room.length_m, "width_m": self.room.width_m}
            if self.room else None,
            "calibration": self.profile.name if self.profile else None,
            "state": self.state,
            "coverage": self.grid.counts() if self.grid else None,
            "coverage_text": self.grid.coverage_text() if self.grid else None,
            "duration_s": round(self.mission_time_s, 1),
            "max_reading": max_rec,
            "anomalies": self.anomalies,
            "corrections": len(self.dr.corrections) if self.dr else 0,
            "events": self.events,
        }
