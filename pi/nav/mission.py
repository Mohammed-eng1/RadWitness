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
import threading
import time
from pathlib import Path

from pi.config import (
    CELL_DWELL_S, MAX_REPLANS_PER_TARGET, DRIFT_PER_METER,
    DRIVE_POWER_DEFAULT, MEASURED_SPEEDS, MEASURED_SPEEDS_LOW_BATT, LOW_BATT_CALIB_V,
    ROVER_MODE, IR_RANGE_CM, WALL_ALIGN_TOL_DEG,
)
from pi.nav.room import Room, OccupancyGrid, CELL_SIZE_M
from pi.nav.scanner import boustrophedon_order, Welford, ANOMALY_NEIGHBOR_PRIORITY
from pi.nav.planner import find_path
from pi.nav.deadreckoning import DeadReckoning
from pi.nav.calibration import CalibrationProfile
from pi.nav.sim_world import SimWorld
from pi.nav.reactive import ReactiveSafety
from pi.nav.executor import DriveExecutor
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
        self.rover = WaveRoverBridge(mode=ROVER_MODE)
        self.reactive = ReactiveSafety()   # أولوية مطلقة (البند 6 مفعّل)
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
        self._last_rung = None
        self.last_reactive = None
        self.drive_motors = False
        self.executor = None
        self._worker = None
        self.ultrasonic = None
        self.ir = None
        self.geiger = None

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
        if self.drive_motors:
            # ⚠ لا تبدأ قيادة محركات بحسّاس قرب معطوب — يُجهض كل خطوة ويلوّث الخريطة
            pf = self.preflight_check()
            if not pf["ok"]:
                for p in pf["problems"]:
                    self._log("preflight", "⚠ " + p)
                return {"ok": False, "error": " · ".join(pf["problems"]),
                        "preflight": pf}
        self.current = self.grid.start_cell()
        cx, cy = self.grid.cell_center(*self.current)
        self.dr = DeadReckoning(self.room, self.profile, cx, cy, 0.0)
        self.heading = 0.0
        self.trail = [self.current]
        self._returning = False
        self.state = RUNNING
        self._visit(self.current)
        self._log("mission_start",
                  "بدء المسح الذاتي" + (" — قيادة محركات ⚠" if self.drive_motors else " (منطقي)"))
        if self.drive_motors:
            self.executor = DriveExecutor(self.rover, self.reactive,
                                          self.sensors, self.profile)
            self._worker = threading.Thread(target=self._motor_worker, daemon=True)
            self._worker.start()
        return {"ok": True}

    def pause(self):
        if self.state == RUNNING:
            self.state = PAUSED
            self.rover.stop()
            self._log("pause", "إيقاف مؤقت")

    def resume(self):
        if self.state in (PAUSED, ESTOP):
            self.state = RUNNING
            self._log("resume", "استئناف")

    def estop(self):
        self.state = ESTOP
        self.rover.stop()          # ⚠ إيقاف محركات فوري
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
        """
        يصرّف أحداث الجسر (قصّ القوة، تجزئة اللفّ، الانحياز) إلى سجل أحداث
        الواجهة، ويطبّق مهلة heartbeat (فقدان الاتصال > 1.5ث → إيقاف).
        """
        for e in self.rover.drain_events():
            self._log(e["kind"], e["msg"])
        if self.rover.check_heartbeat():
            self._log("heartbeat", "⚠ انقطاع أوامر > 1.5ث — أوقفت المحركات")

    def poll_reactive(self):
        """
        (البند 3) يبثّ قرار طبقة السلامة: السرعة **وسببها** ودرجة السلّم.
        يُسجَّل عند تغيّر الدرجة فقط (لا يُغرق السجل).
        """
        s = self.sensors()
        dec = self.reactive.decide(s["ultrasonic_cm"], s["ir_left"], s["ir_right"])
        self.last_reactive = dec
        if dec["rung"] != self._last_rung:
            self._last_rung = dec["rung"]
            if self.state == RUNNING:
                self._log("speed",
                          f"السرعة {dec['speed']} — {dec['reason']} (درجة: {dec['rung']})")
        return dec

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

    # ── مصادر حقيقية (تُحقن من السيرفر على الراسبري) ──────────────
    def set_proximity(self, ultrasonic=None, ir=None):
        self.ultrasonic, self.ir = ultrasonic, ir

    def set_geiger(self, geiger=None):
        self.geiger = geiger

    def set_drive_motors(self, enabled: bool) -> dict:
        """
        يبدّل بين المسح **المنطقي** (محاكاة على الشبكة) و**تشغيل المحركات**.
        لا يُسمح بتشغيل المحركات بلا ملف معايرة (المسافة تُشتق من السرعة).
        """
        enabled = bool(enabled)
        if enabled and self.profile is None:
            return {"ok": False, "error": "لا يوجد ملف معايرة — عايِر أولاً"}
        if enabled and self.state == RUNNING:
            return {"ok": False, "error": "أوقف المهمة قبل تبديل وضع القيادة"}
        self.drive_motors = enabled
        self._log("drive_mode",
                  "قيادة المحركات مفعّلة ⚠" if enabled else "مسح منطقي (بلا محركات)")
        return {"ok": True, "drive_motors": self.drive_motors}

    def sensors(self) -> dict:
        """
        قراءات القرب. تُفضّل **الحساسات الحقيقية** متى توفّرت (على الراسبري)،
        وإلا تسقط إلى نموذج SimWorld (على ويندوز/بلا عتاد).
        """
        real_us = getattr(self, "ultrasonic", None)
        real_ir = getattr(self, "ir", None)
        if (real_us is not None and real_us.ok) or (real_ir is not None and real_ir.ok):
            us = real_us.distance_cm if (real_us and real_us.ok) else None
            l, r = real_ir.read() if (real_ir and real_ir.ok) else (1, 1)
            return {"ultrasonic_cm": us, "ir_left": l, "ir_right": r,
                    "cpm": self.last_reading["cpm"], "source": "real"}
        if self.grid is None or self.dr is None:
            return {"ultrasonic_cm": None, "ir_left": 1, "ir_right": 1,
                    "cpm": self.last_reading["cpm"]}
        front_cm = self.world.front_distance_cm(self.dr.x, self.dr.y, self.heading)
        # نموذج IR واقعي: الحسّاسان مداهما 2-30سم، فلا يُطلقان إلا إذا كان
        # هناك شيء **قريب فعلاً أمامنا**. (النموذج القديم كان يقرأ الخلية
        # القطرية فيعتبر جدار الغرفة عائقاً دائماً كلما سار الروبوت بمحاذاته.)
        ir_left = ir_right = 1
        if front_cm is not None and front_cm <= IR_RANGE_CM:
            th = math.radians(self.heading)
            fx, fy = round(math.sin(th)), round(math.cos(th))
            lx, ly = round(-math.cos(th)), round(math.sin(th))
            r, c = self.current

            def obstacle_at(rr, cc):
                return self.grid.in_bounds(rr, cc) and self.world.is_obstacle(rr, cc)

            left_hit = obstacle_at(r + fy + ly, c + fx + lx)
            right_hit = obstacle_at(r + fy - ly, c + fx - lx)
            if left_hit or right_hit:
                ir_left = 0 if left_hit else 1
                ir_right = 0 if right_hit else 1
            else:
                ir_left = ir_right = 0      # عائق/جدار أمامي مركزي
        return {"ultrasonic_cm": round(front_cm, 1),
                "ir_left": ir_left, "ir_right": ir_right,
                "cpm": self.last_reading["cpm"]}

    # ══ المرحلة 2: تنفيذ حقيقي على المحركات (خيط منفصل) ══════════
    def _motor_worker(self):
        """
        ينفّذ المسح على المحركات فعلياً. يعمل في خيط مستقل لأن كل خطوة
        تستغرق ثوانٍ (لفّ + تقدّم + توقّف قياس)، بينما تبقى حلقة البثّ حيّة.
        ⚠ ينتهي دائماً بـstop() مهما حدث.
        """
        try:
            while self.state in (RUNNING, PAUSED):
                if self.state == PAUSED:
                    self.rover.stop()
                    time.sleep(0.2)
                    continue
                target = self._current_start() if self._returning else self._next_target()
                if target is None:
                    self._finish()
                    break
                path = find_path(self.grid, self.current, target)
                if path is None or len(path) < 2:
                    if self._returning:
                        self._finish()
                        break
                    self.grid.mark_unreachable(*target)
                    self.dirty.add(target)
                    self._log("unreachable", f"هدف غير قابل للوصول ({target[0]},{target[1]})")
                    continue
                self.planned_path = path
                nxt = path[1]
                target_heading = _heading_between(self.current, nxt)

                # ⚠ اللفّ أولاً ثم **تحديث الاتجاه فوراً** قبل التقدّم: قراءة
                # الحساسات تعتمد self.heading، فلو بقي قديماً لقاس الروبوت
                # المسافة في الاتجاه الخاطئ وأجهض الخطوة بعائق وهمي.
                t = self.executor.turn_to(self.heading, target_heading)
                turned = t.get("turned_deg", 0.0)
                if self.dr and turned:
                    self.dr.turn(turned)
                    self.heading = self.dr.heading
                if not t["ok"]:
                    self._log("turn_timeout", f"انتهت مهلة اللفّ نحو {target_heading:.0f}°")
                    continue

                # المسافة المتوقَّعة للجدار من **مركز الخلية الهدف** (من الخريطة)
                tx, ty = self.grid.cell_center(*nxt)
                exp_wall = self._wall_distance_from(tx, ty, self.heading)
                fwd = self.executor.forward_cell(CELL_SIZE_M, expected_wall_end_m=exp_wall)
                covered = fwd.get("covered_m", 0.0)
                if self.dr and covered:
                    self.dr.advance(covered)
                res = {"ok": fwd["ok"], "turn": t, "forward": fwd,
                       "reason": fwd.get("reason")}

                if not res["ok"]:
                    # أُجهضت الخطوة. إن كان السبب IR بينما الألترا سونيك يرى
                    # الطريق خالياً، فالأرجح حسّاس معطوب لا عائق — نقولها صراحة
                    # بدل اتهام خلية سليمة (وتلويث الخريطة).
                    s_now = self.sensors()
                    us_now = s_now.get("ultrasonic_cm")
                    ir_fired = (s_now.get("ir_left") == 0 or s_now.get("ir_right") == 0)
                    if ir_fired and (us_now is None or us_now > IR_RANGE_CM * 2):
                        self._log("sensor_fault",
                                  f"⚠ IR يقول «عائق» والألترا سونيك يرى {us_now}سم — "
                                  f"تحقّق من توصيل IR (BCM25/BCM16) ومقاومة المدى")
                    if covered < 0.05:
                        self._log("no_motion",
                                  f"لم يتحرك الروبوت ({covered:.2f}م) — أُجهضت الخطوة فوراً")
                    self.grid.mark_blocked(*nxt)
                    self.dirty.add(nxt)
                    self._log("obstacle_detected",
                              f"عائق ({nxt[0]},{nxt[1]}) — {res.get('reason') or fwd.get('aborted')}")
                    self.replans[target] = self.replans.get(target, 0) + 1
                    if self.replans[target] > MAX_REPLANS_PER_TARGET:
                        self.grid.mark_unreachable(*target)
                        self.dirty.add(target)
                        self._log("unreachable", f"هدف محاصر ({target[0]},{target[1]})")
                    continue

                self.current = nxt
                self.trail.append(self.current)
                self.mission_time_s += CELL_DWELL_S
                time.sleep(CELL_DWELL_S)        # توقّف لتجميع عدّ إحصائي كافٍ
                self._visit(self.current)
                s = self.sensors()
                self.executor.maybe_wall_correct(self.dr, self.heading,
                                                 s.get("ultrasonic_cm"))
                self._check_battery()
                if self._returning and self.current == self.grid.start_cell():
                    self._finish()
                    break
        except Exception as e:                  # noqa: BLE001
            self._log("error", f"⚠ خطأ في التنفيذ: {e}")
            self.estop()
        finally:
            self.rover.stop()                   # ⚠ إيقاف مضمون

    def tick(self):
        self._check_battery()
        if self.drive_motors:
            return                              # الخيط العامل يقود بدلاً منه
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

    def _wall_distance_from(self, x: float, y: float, heading: float):
        """مسافة جدار الغرفة أمام نقطة باتجاه معيّن (من الخريطة لا الحساس)."""
        # الاتجاه الفعلي بعد تكامل الجايرو نادراً ما يكون 90.0 بالضبط (قد يكون
        # 88.6)، فنثبّته على أقرب محور ضمن التسامح بدل رفضه.
        h = heading % 360.0
        axis = min((0.0, 90.0, 180.0, 270.0),
                   key=lambda a: abs((h - a + 180.0) % 360.0 - 180.0))
        if abs((h - axis + 180.0) % 360.0 - 180.0) > WALL_ALIGN_TOL_DEG:
            return None
        if axis == 0.0:
            return max(0.0, self.room.length_m - y)
        if axis == 180.0:
            return max(0.0, y)
        if axis == 90.0:
            return max(0.0, self.room.width_m - x)
        return max(0.0, x)

    def preflight_check(self, samples: int = 8, gap: float = 0.15) -> dict:
        """
        فحص ما قبل القيادة بالمحركات — يكشف حسّاس قرب **غير موصول أو مضبوطاً
        خطأً** قبل أن يُفسد المهمة كلها.

        المنطق: مدى IR هو 2-30سم فقط. فإن ظلّ يقول «عائق» في كل العيّنات
        بينما الألترا سونيك يرى مسافة بعيدة (أو لا يقرأ)، فالأرجح أن دخله
        **عائم (غير موصول)** أو مقاومة المدى مضبوطة خطأً. عندها نرفض البدء
        برسالة صريحة — أفضل بكثير من إجهاض كل خطوة وتلويث الخريطة بخلايا
        محجوبة كاذبة (فشل صامت).
        """
        left_hits = right_hits = far_us = 0
        last = {}
        for _ in range(max(1, samples)):
            s = self.sensors()
            last = s
            if s.get("ir_left") == 0:
                left_hits += 1
            if s.get("ir_right") == 0:
                right_hits += 1
            us = s.get("ultrasonic_cm")
            if us is None or us > IR_RANGE_CM * 2:
                far_us += 1
            time.sleep(gap)

        problems = []
        if far_us == samples:
            if left_hits == samples:
                problems.append("حسّاس IR أمام-يسار يقرأ «عائق» دائماً والمسار أمامه خالٍ "
                                "— تحقّق من توصيله (BCM25) ومن مقاومة المدى")
            if right_hits == samples:
                problems.append("حسّاس IR أمام-يمين يقرأ «عائق» دائماً والمسار أمامه خالٍ "
                                "— تحقّق من توصيله (BCM16) ومن مقاومة المدى")
        return {"ok": not problems, "problems": problems, "sensors": last,
                "ir_left_hits": left_hits, "ir_right_hits": right_hits,
                "samples": samples}

    def _read_radiation(self, x: float, y: float):
        """
        قراءة الإشعاع: **العدّاد الحقيقي** متى توفّر (مسح حقيقي)، وإلا نموذج
        SimWorld. الإشعاع لا يُحاكى أبداً حين يكون العتاد متصلاً.
        """
        g = getattr(self, "geiger", None)
        if g is not None and getattr(g, "ok", False):
            st = g.state()
            return st["cpm"], st["usvh"]
        return self.world.reading_at(x, y)

    def _visit(self, cell):
        r, c = cell
        gc = self.grid.get(r, c)
        already = gc.visited
        x, y = self.grid.cell_center(r, c)
        cpm, usvh = self._read_radiation(x, y)
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
            "reactive": self.last_reactive,
            "reactive_enabled": self.reactive.enabled,
            "drive_motors": self.drive_motors,
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
