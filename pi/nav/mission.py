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
    ROVER_MODE, IR_RANGE_CM, WALL_ALIGN_TOL_DEG, ROVER_TURN_TIMEOUT_S,
    GEIGER_WINDOW_S,
    BATTERY_MONITOR_ENABLED, MISSION_TIME_WARN_S, MISSION_TIME_LIMIT_S,
    MISSION_HARD_LIMIT_S, MOTION_TRUST_MEASURED, MOTION_STUCK_LIMIT,
    MOTION_CELL_ENTER_TOL_M, UNCERTAINTY_INITIAL, DWELL_MAX_S,
    CONFIRM_RADIUS_M, CONFIRM_DWELL_S, CONFIRM_POSITIONS,
    APPROACH_STEP_M, APPROACH_MAX_STEPS, APPROACH_DWELL_S,
    BREADCRUMB_MAX, RETRACE_MAX_CELLS, RETRACE_SAFE_CPM_FACTOR,
    RESCAN_MAX_CELLS_PER_POINT, GRADIENT_TRANSIT_STOP_M,
)
from pi.nav.room import Room, OccupancyGrid, CELL_SIZE_M
from pi.ai.source_locator import SourceLocator, SURVEY, CONFIRM
from pi.ai.approach_document import (
    run_documentation, approach_blockers, GradientApproach, source_bearing_deg,
    MOVE, STOP, WITHDRAW,
)
from pi.ai.dynamic_range import dead_time_correct
from pi.nav.scanner import boustrophedon_order, Welford, ANOMALY_NEIGHBOR_PRIORITY
from pi.nav.planner import find_path
from pi.nav.deadreckoning import DeadReckoning
from pi.nav.calibration import CalibrationProfile
from pi.nav.sim_world import SimWorld
from pi.nav.reactive import ReactiveSafety
from pi.nav.executor import DriveExecutor
from pi.nav.motion_check import SHORT, OVERSHOOT, NO_MOTION
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

# ── مراحل الدورة الكاملة (البند 6): مسح → فرز → تأكيد → اقتراب →
#    توقف → توثيق. كل انتقال بحكم **المنسّق** لا برغبة الملاحة.
PHASE_SURVEY, PHASE_SCREEN, PHASE_CONFIRM = "survey", "screen", "confirm"
PHASE_APPROACH, PHASE_STOP, PHASE_DOCUMENT = "approach", "stop", "document"
PHASE_WITHDRAW, PHASE_REPORT = "withdraw", "report"
PHASE_AR = {
    PHASE_SURVEY: "١ مسح", PHASE_SCREEN: "٢ فرز", PHASE_CONFIRM: "٣ تأكيد",
    PHASE_APPROACH: "٤ اقتراب", PHASE_STOP: "٥ توقف",
    PHASE_DOCUMENT: "٦ توثيق", PHASE_WITHDRAW: "انسحاب", PHASE_REPORT: "تقرير",
}


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
        self._batt_source = None       # مصدر الحماية الفاعل (يُسجَّل عند تغيّره)
        self._rth_triggered = False
        # الحدّ الزمني (بديل حماية الجهد المعطّلة — القسم 9)
        self._started_ts = None
        self._time_warned = False
        self._time_rth = False
        self._clamp_seen = 0
        self._base_cap_logged = False
        self._last_rung = None
        # ── البند 0: تتبّع الحركة المتحقَّق منها ──────────────────
        self._stuck = 0               # انعدامات حركة متتالية (→ إيقاف)
        self._progress_cell = None    # الخلية التي نتقدّم نحوها جزئياً
        self._cell_progress = 0.0     # ما قُطع منها فعلاً (متر)
        self._unverified_cells = 0    # خلايا عُلّمت بثقة منخفضة
        self._last_motion = None      # آخر حكم تحقّق (للبثّ في الواجهة)
        # ── البنود 3-6: الأثر والمراحل ───────────────────────────
        self.breadcrumbs = []         # أثر المسار (البند 4)
        self._retracing = False       # داخل انسحاب (لا تُضاف نقاط أثر)
        self._withdraw_req = None     # طلب انسحاب معلّق (أولوية مطلقة)
        self.phase = PHASE_SURVEY
        self.cycle = None             # حصيلة الدورة الكاملة (للبثّ والتقرير)
        self._rolling_warned = False  # حُذّر من نافذة العدّاد المنزلقة مرة
        self.last_reactive = None
        self.drive_motors = False
        self.executor = None
        self._worker = None
        self.ultrasonic = None
        self.ir = None
        self.geiger = None
        self.locator = None      # يُنشأ عند configure_room
        self.camera = None       # تُحقن من السيرفر (اختيارية)
        self.documentation = None  # نتيجة التوثيق البصري بعد المسح

    def configure_room(self, length_m, width_m, start_corner="back_left",
                       scan_spacing_m=0.5, source_xy=None, bg_cpm=22.0):
        self.room = Room(length_m=float(length_m), width_m=float(width_m),
                         start_corner=start_corner, scan_spacing_m=float(scan_spacing_m))
        self.grid = OccupancyGrid(self.room)
        self.world = SimWorld(self.room, source_xy=source_xy, bg_cpm=bg_cpm)
        # ── محدِّد موقع المصدر: يُنشأ مع الغرفة ويُغذّى من كل قراءة مسح ──
        # ⚠ محاور الشبكة البايزية: x = العرض، y = الطول — نفس اصطلاح
        #   `grid.cell_center` تماماً، وإلا انعكست الخريطة الحرارية صامتةً.
        self.locator = SourceLocator(width_m=float(width_m),
                                     length_m=float(length_m),
                                     background_cpm=float(bg_cpm))
        self.locator.set_background(float(bg_cpm))
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
        self._reset_motion_state()
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
        self._reset_motion_state()
        self.state = RUNNING
        self._started_ts = time.time()      # ساعة الحدّ الزمني (القسم 9)
        self._time_warned = False
        self._time_rth = False
        self._visit(self.current)
        # 🔴 **نقطة الدخول أول نقطة في الأثر وأهمّها**: بدونها ينتهي الانسحاب
        #    عند الخلية التالية لها ويُعلن «بلغت المدخل» وهو لم يبلغه.
        self.breadcrumb_push()
        self._log("mission_start",
                  "بدء المسح الذاتي" + (" — قيادة محركات ⚠" if self.drive_motors else " (منطقي)"))
        # الحارسان يُعلَنان عند البدء — لا يُترك المشغّل يخمّن ما يحميه
        v0 = self.rover.voltage() if BATTERY_MONITOR_ENABLED else None
        self._log_battery_source(
            self.rover.voltage_source if BATTERY_MONITOR_ENABLED else "disabled",
            self.rover.voltage_reason if BATTERY_MONITOR_ENABLED else None)
        if v0 is not None:
            b0 = batt.classify(v0)
            self._log("battery",
                      f"جهد البدء {b0['v']}V ({b0['cell_v']}V/خلية · "
                      f"~{b0['percent']}%) — {b0['text']}")
        self._log("time_limit",
                  f"الحاجز الزمني مسلَّح كطبقة ثانية: عودة إجبارية عند "
                  f"{MISSION_TIME_LIMIT_S:.0f}ث، إيقاف عند "
                  f"{MISSION_HARD_LIMIT_S:.0f}ث"
                  + ("" if v0 is not None else
                     " — ⚠ **وهو الحارس الوحيد الآن** (لا قراءة جهد)"))
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

    # ── الحدّ الزمني: بديل مؤقت عن حماية الجهد (القسم 9) ──────────
    def time_limit_info(self) -> dict:
        """قراءة **بلا أثر جانبي** للبثّ في الواجهة (الإنفاذ في _check_time_limit)."""
        elapsed = 0.0 if self._started_ts is None else time.time() - self._started_ts
        return {"enabled": True, "running": self.state == RUNNING,
                "elapsed_s": round(elapsed, 1),
                "warn_s": MISSION_TIME_WARN_S, "limit_s": MISSION_TIME_LIMIT_S,
                "hard_s": MISSION_HARD_LIMIT_S,
                "remaining_s": round(max(0.0, MISSION_HARD_LIMIT_S - elapsed), 1),
                "rth_triggered": self._time_rth}

    def _check_time_limit(self) -> dict:
        """
        الحاجز الزمني: تحذير → عودة إجبارية (RTH) → إيقاف محركات صلب.

        🔴 **طبقة ثانية باقية بعد عودة قراءة الجهد** لا بديل مؤقت: أي حارس
        واحد نقطة فشل واحدة. لو تعذّرت قراءة INA219 (ناقل مشغول، عنوان
        تغيّر، OVF متكرر) يبقى الزمن حارساً يعمل — والحارسان مسلَّحان معاً
        وأيّهما بلغ حدّه أولاً يُنفَّذ.
        ⚠ الزمن حاجز خام: لا يعرف حالة الشحن الابتدائية، فابدأ ببطارية مشحونة.
        الساعة تعمل **من بدء المهمة** بزمن الحائط (لا `mission_time_s` الذي
        يتقدّم بخطوات منطقية ويتجمّد أثناء الإيقاف المؤقت).
        """
        if self._started_ts is None:
            return {"enabled": False}
        info = self.time_limit_info()
        elapsed = info["elapsed_s"]
        if self.state != RUNNING:
            return info
        if elapsed >= MISSION_HARD_LIMIT_S:
            self._log("time_limit",
                      f"⛔ بلغت المهمة الحدّ الصلب {MISSION_HARD_LIMIT_S:.0f}ث — "
                      f"إيقاف المحركات فوراً (حماية بديلة عن الجهد)")
            self.estop()
            info["action"] = "estop"
        elif elapsed >= MISSION_TIME_LIMIT_S and not self._time_rth:
            self._time_rth = True
            self._log("time_limit",
                      f"⚠ مضى {elapsed:.0f}ث ≥ {MISSION_TIME_LIMIT_S:.0f}ث — "
                      f"عودة إجبارية (مراقبة الجهد معطّلة)")
            self.return_home()
            info["action"] = "rth"
        elif elapsed >= MISSION_TIME_WARN_S and not self._time_warned:
            self._time_warned = True
            self._log("time_limit",
                      f"تنبيه: مضى {elapsed:.0f}ث من أصل {MISSION_TIME_LIMIT_S:.0f}ث "
                      f"قبل العودة الإجبارية")
            info["action"] = "warn"
        return info

    def _check_battery(self):
        """
        يطبّق عتبات الجهد الإلزامية (أ-3) — و**الحاجز الزمني معها دائماً**
        كطبقة ثانية: أي حارس واحد نقطة فشل واحدة.
        """
        self.rover.sim_set_moving(self.state == RUNNING)
        # 🔴 الطبقة الثانية تعمل **مهما كانت حالة الجهد** (البند 4)
        self._check_time_limit()
        if not BATTERY_MONITOR_ENABLED:
            self._log_battery_source("disabled", "مراقبة الجهد معطّلة في config")
            return self.rover.check_battery()
        info = self.rover.check_battery()
        # البند 5: مصدر الحماية يُسجَّل عند كل تغيّر — خلط المصدرين يجعل
        # تشخيص أي حادثة مستحيلاً («هبط الجهد أم نفد الزمن؟»).
        self._log_battery_source(info.get("source"), info.get("source_reason"))
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
            self._log("battery",
                      f"تنبيه: جهد {info['v']}V ({info.get('cell_v')}V/خلية) — جيد")
        self._batt_level = info["level"]
        return info

    def _log_battery_source(self, source, reason=None) -> None:
        """
        (البند 5) يسجّل **مصدر الحماية الفاعل** عند تغيّره فقط:
        `ina219` قراءة جهد حقيقية · `rover_serial` حقل v من الروفر ·
        `sim_override` تجاوز اختبار · `unavailable` لا جهد ⇒ الزمن هو الحارس.
        سطر واحد عند التبديل يكفي لتفسير أي إيقاف لاحق.
        """
        if source == self._batt_source:
            return
        self._batt_source = source
        names = {
            "ina219": "✅ الجهد من INA219 (حماية جهدية فاعلة + الحاجز الزمني)",
            "rover_serial": "⚠ الجهد من رسالة الروفر — INA219 غير متاح",
            "sim_override": "⚠ جهد محاكاة (تجاوز اختبار) — ليس قراءة عتاد",
            "unavailable": "🔴 **لا قراءة جهد** — الحارس الفاعل هو الحدّ الزمني وحده",
            "disabled": "⚠ مراقبة الجهد معطّلة — الحدّ الزمني وحده",
        }
        msg = names.get(source, f"مصدر جهد غير معروف: {source}")
        self._log("battery_source", msg + (f" · {reason}" if reason else ""))

    # ── مصادر حقيقية (تُحقن من السيرفر على الراسبري) ──────────────
    def set_proximity(self, ultrasonic=None, ir=None):
        self.ultrasonic, self.ir = ultrasonic, ir

    def set_geiger(self, geiger=None):
        self.geiger = geiger

    def set_drive_motors(self, enabled: bool, allow_sim: bool = False) -> dict:
        """
        يبدّل بين المسح **المنطقي** (محاكاة على الشبكة) و**تشغيل المحركات**.
        لا يُسمح بتشغيل المحركات بلا ملف معايرة (المسافة تُشتق من السرعة).

        ⚠ ولا يُسمح به والجسر في وضع `sim`: عندها يقود المنفّذ جسراً وهمياً
        فتتقدّم الخريطة وتُعلن تغطية 100% **بينما لا يتحرك أي محرك** — وهو
        فشل صامت يوهم بمسح لم يحدث. (`allow_sim=True` للاختبار البرمجي فقط.)
        """
        enabled = bool(enabled)
        if enabled and self.profile is None:
            return {"ok": False, "error": "لا يوجد ملف معايرة — عايِر أولاً"}
        if enabled and not allow_sim and self.rover.mode != "real":
            return {"ok": False,
                    "error": "جسر الروفر في وضع sim — بدّل إلى «وضع real» أولاً، "
                             "وإلا ستتقدّم الخريطة بلا حركة فعلية للروبوت",
                    "rover_mode": self.rover.mode}
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
            q = getattr(real_us, "quality", None) if (real_us and real_us.ok) else None
            return {"ultrasonic_cm": us, "ir_left": l, "ir_right": r,
                    "cpm": self.last_reading["cpm"], "source": "real",
                    "quality": q}
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
                # 🔴 **أولوية مطلقة**: أمر الانسحاب يتجاوز أي هدف مسح أو تغطية
                if self._withdraw_req is not None:
                    self._serve_withdraw()
                    continue
                target = self._current_start() if self._returning else self._next_target()
                if target is None:
                    self._finish()
                    break
                r = self._advance_one_cell(target)
                if r.get("fatal"):
                    break
                if r.get("no_path"):
                    if self._returning:
                        self._finish()
                        break
                    self.grid.mark_unreachable(*target)
                    self.dirty.add(target)
                    self._log("unreachable",
                              f"هدف غير قابل للوصول ({target[0]},{target[1]})")
                    continue
                if r.get("blocked_cell"):
                    self.replans[target] = self.replans.get(target, 0) + 1
                    if self.replans[target] > MAX_REPLANS_PER_TARGET:
                        self.grid.mark_unreachable(*target)
                        self.dirty.add(target)
                        self._log("unreachable",
                                  f"هدف محاصر ({target[0]},{target[1]})")
                    continue
                if (r.get("entered") and self._returning
                        and self.current == self.grid.start_cell()):
                    self._finish()
                    break
        except Exception as e:                  # noqa: BLE001
            self._log("error", f"⚠ خطأ في التنفيذ: {e}")
            self.estop()
        finally:
            self.rover.stop()                   # ⚠ إيقاف مضمون

    # ══ الأوّلية الحركية: خطوة خلية واحدة (يستعملها الجميع) ══════
    def _advance_one_cell(self, target, measure: bool = True) -> dict:
        """
        خطوة خلية واحدة نحو `target`: تخطيط → لفّ → عبور → **تحقّق من الحركة**
        → دخول وقياس. هذه **الأوّلية الوحيدة** للحركة الشبكية: يستعملها المسح
        وإعادة المسح والانسحاب والاقتراب معاً، فلا يوجد مسار حركة ثانٍ يفلت
        من التحقق أو من السلامة.

        `measure=False` أثناء الانسحاب: السلامة تسبق جمع البيانات، ولا نتوقف
        3ث في كل خلية ونحن نخرج من حقل إشعاعي.
        """
        path = find_path(self.grid, self.current, target)
        if path is None or len(path) < 2:
            return {"ok": False, "no_path": True, "entered": False}
        self.planned_path = path
        nxt = path[1]
        target_heading = _heading_between(self.current, nxt)

        # ⚠ اللفّ أولاً ثم **تحديث الاتجاه فوراً** قبل التقدّم: قراءة
        # الحساسات تعتمد self.heading، فلو بقي قديماً لقاس الروبوت
        # المسافة في الاتجاه الخاطئ وأجهض الخطوة بعائق وهمي.
        if not self._face(target_heading):
            return {"ok": False, "entered": False, "turn_failed": True}

        # المسافة المتوقَّعة للجدار من **مركز الخلية الهدف** (من الخريطة)
        tx, ty = self.grid.cell_center(*nxt)
        exp_wall = self._wall_distance_from(tx, ty, self.heading)
        # ⚠ **المتبقّي** لا الخلية كاملة: محاولة سابقة قطعت جزءاً من
        #    المسافة (تحقّق قاسها)، فالأمر بخلية كاملة يتجاوز الهدف.
        done = self._progress_toward(nxt)
        fwd = self.executor.forward_cell(CELL_SIZE_M - done,
                                         expected_wall_end_m=exp_wall)
        covered = fwd.get("covered_m", 0.0)
        mv = fwd.get("motion") or {}
        if mv:
            self._last_motion = dict(mv, model_m=round(covered, 3))
        # 🔴 المسافة المقاسة تُقدَّم على المحسوبة (البند 0)
        step_m = self._motion_step_m(covered, mv)
        if self.dr and step_m:
            self.dr.advance(step_m, verified=mv.get("confident", True))
        self._log_heading_hold(fwd)

        if not fwd["ok"]:
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
            if covered < 0.05 or mv.get("moved") is False:
                self._log("no_motion",
                          f"لم يتحرك الروبوت ({step_m:.2f}م) — أُجهضت الخطوة فوراً"
                          + (f" · {mv['reason']}" if mv.get("reason") else ""))
            # الإجهاض بعائق **ليس انحشاراً**: التوقف أمام عائق هو
            # السلوك الصحيح فلا يُحتسب في عدّاد العلوق. ويُصفَّر التقدّم
            # الجزئي لأن العبور **مهجور** لا مؤجَّل (الخلية تُحجب الآن)،
            # وبقاؤه يقصّر شوطاً لاحقاً نحوها لو رُفع الحجب.
            self._set_progress(None, 0.0)
            self.grid.mark_blocked(*nxt)
            self.dirty.add(nxt)
            self._log("obstacle_detected",
                      f"عائق ({nxt[0]},{nxt[1]}) — "
                      f"{fwd.get('reason') or fwd.get('aborted')}")
            return {"ok": False, "entered": False, "blocked_cell": nxt}

        # ══ 🔴 البند 0: هل تحرّك الروبوت فعلاً؟ ═══════════════
        # الأمر نُفّذ بلا إجهاض — وهذا **لا يعني** أن الروبوت تحرّك.
        # بلا هذا الفحص تُعلَّم الخلية من الموقع المحسوب وحده، وهو ما
        # أنتج ادّعاء 1.8م لشوط قطع 0.20م.
        if mv.get("verdict") in (SHORT, OVERSHOOT):
            self._log("motion_mismatch",
                      f"⚠ {mv.get('reason', 'فرق بين المقاس والمحسوب')} — "
                      f"اعتُمدت المسافة المقاسة لا المحسوبة")
        if mv.get("moved") is False:
            self._stuck += 1
            self._log("no_motion",
                      f"⛔ {mv.get('reason', 'لا حركة')} — الخلية "
                      f"({nxt[0]},{nxt[1]}) **لا تُعلَّم مزارة** "
                      f"(محاولة {self._stuck}/{MOTION_STUCK_LIMIT})")
            if self._stuck >= MOTION_STUCK_LIMIT:
                self._log("stuck",
                          f"⛔ لا حركة في {self._stuck} محاولات متتالية — "
                          f"الروبوت عالق أو منزلق. إيقاف المهمة بدل "
                          f"استنزاف البطارية بلا تقدّم.")
                self.estop()
                return {"ok": False, "entered": False, "fatal": True}
            return {"ok": False, "entered": False, "no_motion": True}
        self._stuck = 0

        # حركة ناقصة: تقدّم لكنه لم يبلغ الخلية ⇒ لا تُعلَّم، ويُؤمر
        # بالمتبقّي في الدورة التالية (لا بخلية كاملة تتجاوزها).
        prog = self._progress_toward(nxt) + max(step_m, 0.0)
        if prog < CELL_SIZE_M - MOTION_CELL_ENTER_TOL_M:
            self._set_progress(nxt, prog)
            self._log("partial_motion",
                      f"تقدّم {prog:.2f}م من {CELL_SIZE_M:.2f}م نحو "
                      f"({nxt[0]},{nxt[1]}) — يُستكمل المتبقّي")
            return {"ok": True, "entered": False, "partial_m": round(prog, 2)}
        self._set_progress(None, 0.0)

        self.current = nxt
        self.trail.append(self.current)
        self.breadcrumb_push()              # أثر الدخول (للانسحاب الآمن)
        low_conf = not mv.get("confident", True)
        if measure:
            # القاعدة الثالثة: بلا مرجع لا تُدَّعى ثقة كاملة
            self._visit(self.current, low_confidence=low_conf)
        else:
            # انسحاب: لا وقت للقياس، والخلية مزارة أصلاً على الأثر
            self.dirty.add(self.current)
        s = self.sensors()
        self.executor.maybe_wall_correct(self.dr, self.heading,
                                         s.get("ultrasonic_cm"))
        self._check_battery()
        return {"ok": True, "entered": True, "cell": nxt,
                "low_confidence": low_conf}

    def _face(self, target_heading: float) -> bool:
        """يلفّ نحو اتجاه مطلوب ويحدّث الاتجاه فوراً. False عند فشل اللفّة."""
        t = self.executor.turn_to(self.heading, target_heading)
        turned = t.get("turned_deg", 0.0)
        if self.dr and turned:
            self.dr.turn(turned)
            self.heading = self.dr.heading
        if not t["ok"]:
            why = ("مهلة اللفّ" if t.get("timed_out")
                   else f"إجهاض ({t.get('aborted')})")
            self._log("turn_failed",
                      f"فشل اللفّ نحو {target_heading:.0f}° — {why}")
            return False
        return True

    def _log_heading_hold(self, fwd: dict) -> None:
        """تثبيت الاتجاه: عطله يُبلَّغ **دائماً**، وجودته عند تدهورها فقط."""
        hh = fwd.get("heading_hold") or {}
        if hh.get("base_capped") and not self._base_cap_logged:
            self._base_cap_logged = True     # مرة واحدة لا كل خلية
            want, got = hh["base_capped"]
            self._log("speed_cap",
                      f"سلّم السلامة يطلب {want} والمطبَّق {got} — "
                      f"تثبيت الاتجاه يحتاج فراغاً للتصحيح "
                      f"(0.50 تترك فراغاً صفراً). السرعة ≈{got * 1.5:.2f} م/ث")
        if hh.get("lost"):
            self._log("heading_hold",
                      f"⚠ تعذّر تثبيت الاتجاه — {hh['lost']} (تقدّم بحلقة مفتوحة)")
        elif hh.get("max_abs_error_deg", 0) > 5.0:
            self._log("heading_hold",
                      f"⚠ خطأ اتجاه {hh['max_abs_error_deg']:.1f}° "
                      f"(متوسط {hh['mean_abs_error_deg']:.1f}°، "
                      f"إشباع {hh['saturated_pct']}%)")

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
        self.breadcrumb_push()
        self._visit(self.current)       # ← يحتسب زمن القياس بنفسه
        self._try_wall_correction()
        if self._returning and self.current == self.grid.start_cell():
            self._finish()

    def _current_start(self):
        s = self.grid.start_cell()
        return None if self.current == s else s

    # ── البند 0: المسافة المعتمدة والتقدّم الجزئي ────────────────
    def _reset_motion_state(self) -> None:
        self._stuck = 0
        self._progress_cell = None
        self._cell_progress = 0.0
        self._unverified_cells = 0
        self._last_motion = None
        self.breadcrumbs = []
        self._retracing = False
        self._withdraw_req = None
        self.phase = PHASE_SURVEY
        self.cycle = None
        self._rolling_warned = False

    def _motion_step_m(self, covered: float, motion: dict) -> float:
        """
        المسافة التي يتقدّم بها `deadreckoning`.

        🔴 **المقاسة تسبق المحسوبة**: `covered` تكامل نظري (سرعة معايرة ×
        زمن) لا يعرف انزلاقاً ولا انحشاراً — وهو مصدر خطأ الـ9 أضعاف. متى
        قِيست الإزاحة بالألترا سونيك اعتُمدت هي، ويبقى `covered` احتياطاً
        حين لا مرجع (وهو حينها معلَن بثقة منخفضة لا مسكوتاً عنه).
        """
        measured = motion.get("measured_m") if motion else None
        if measured is None or not MOTION_TRUST_MEASURED:
            return max(float(covered), 0.0)
        return max(float(measured), 0.0)

    def _progress_toward(self, cell) -> float:
        """ما قُطع فعلاً نحو هذه الخلية في محاولات سابقة (0 لخلية جديدة)."""
        return self._cell_progress if self._progress_cell == cell else 0.0

    def _set_progress(self, cell, meters: float) -> None:
        self._progress_cell = cell
        self._cell_progress = max(0.0, float(meters)) if cell else 0.0

    # ══════════════════════════════════════════════════════════════
    # البند 4: أثر المسار والانسحاب الآمن
    # ══════════════════════════════════════════════════════════════
    def breadcrumb_push(self) -> None:
        """
        نقطة على أثر المسار (خلية + موضع + اتجاه).

        🔴 لماذا نحتفظ بالأثر أصلاً: حين يصير الموقع **غير قابل للتحديد**
        (عدّاد مشلول، بداية غير قانونية) يأمر النظام بالانسحاب وهو **لا يعرف
        أين المصدر** — فأي اتجاه قد يكون نحوه. الاتجاه الآمن الوحيد هو الطريق
        الذي دخل منه: قِيس فعلاً وكان آمناً حين عُبر.
        """
        if self.dr is None or self._retracing:
            return
        pt = {"cell": tuple(self.current), "x": round(self.dr.x, 3),
              "y": round(self.dr.y, 3), "heading": round(self.heading, 1),
              "t": round(self.mission_time_s, 1)}
        if self.breadcrumbs and self.breadcrumbs[-1]["cell"] == pt["cell"]:
            return                       # لا تكرار لنفس الخلية
        self.breadcrumbs.append(pt)
        if len(self.breadcrumbs) > BREADCRUMB_MAX:
            # ⚠ **لا نحذف الأقدم**: أقدم النقاط هي طريق **الخروج** وأهمّها.
            #    نُخفّف دقّة النصف الأقدم بدل بتره — والتخطيط (A*) يملأ ما
            #    بين النقاط المتباعدة فيبقى المسار سالكاً.
            self.breadcrumbs = (self.breadcrumbs[:BREADCRUMB_MAX // 2:2]
                                + self.breadcrumbs[BREADCRUMB_MAX // 2:])

    def request_withdraw(self, until_cpm=None, reason: str = "") -> dict:
        """
        يطلب انسحاباً. 🔴 **أولوية مطلقة**: يتجاوز أي هدف مسح أو تغطية،
        ويُخدَم في رأس حلقة التنفيذ قبل اختيار أي هدف.

        ⚠ يُرفض صراحةً بلا قيادة محركات بدل أن يُقبل ويُهمَل بصمت: أمر سلامة
           «مقبول» لا يُنفَّذ أسوأ من أمر مرفوض بوضوح.
        """
        if not self._can_drive():
            msg = "طلب انسحاب بلا قيادة محركات — لا حركة تُنفَّذ"
            self._log("withdraw_request", f"⚠ {msg}")
            return {"ok": False, "error": msg}
        self._withdraw_req = {"until_cpm": until_cpm, "reason": reason or "أمر انسحاب"}
        self._log("withdraw_request", f"🔴 طلب انسحاب — {self._withdraw_req['reason']}")
        return {"ok": True, **self._withdraw_req}

    def _serve_withdraw(self) -> None:
        """يُنفّذ الطلب المعلّق ثم ينهي المهمة (لا استئناف مسح بعد انسحاب)."""
        req = self._withdraw_req or {}
        self._withdraw_req = None
        res = self.retrace_path(until_cpm=req.get("until_cpm"),
                                reason=req.get("reason", ""))
        self.cycle = dict(self.cycle or {}, withdraw=res)
        if self.state == RUNNING:
            self.state = DONE
            self._set_phase(PHASE_REPORT)

    def _safe_cpm_threshold(self) -> float:
        bg = self.locator.background_cpm if self.locator else 0.0
        return max(RETRACE_SAFE_CPM_FACTOR * float(bg), 1.0)

    def retrace_path(self, until_cpm=None, reason: str = "") -> dict:
        """
        الانسحاب **على أثر الدخول عكسياً** حتى تنخفض القراءة دون العتبة.

        ⚠ لا قياس تغطية أثناء الانسحاب (`measure=False`): السلامة تسبق جمع
           البيانات، والتوقف 3ث في كل خلية داخل حقل إشعاعي خطأ لا دقّة.
        ⚠ الرجوع هنا **بالتقدّم على الأثر** لا بالقهقرى: الحساسات تنظر لجهة
           السير، فالدوران ومتابعة الأثر أأمن من رجوع أعمى طويل.
        """
        thr = float(until_cpm) if until_cpm is not None else self._safe_cpm_threshold()
        self._set_phase(PHASE_WITHDRAW)
        self._log("retrace",
                  f"🔴 انسحاب على أثر المسار حتى < {thr:,.0f} CPM"
                  + (f" — {reason}" if reason else ""))
        self._retracing = True
        steps, reached_entry = 0, False
        try:
            while steps < RETRACE_MAX_CELLS:
                if self.state in (ESTOP, IDLE):
                    break
                x, y = ((self.dr.x, self.dr.y) if self.dr
                        else self.grid.cell_center(*self.current))
                # قياس قصير: نريد «هل صرنا آمنين؟» لا تقديراً دقيقاً
                m = self._measure(x, y, min(CELL_DWELL_S, 2.0))
                self.last_reading = {"cpm": round(m["cpm"], 1),
                                     "usvh": round(m["usvh"], 3),
                                     **{k: v for k, v in classify(m["usvh"]).items()
                                        if k in ("risk", "color")}}
                if m["cpm"] <= thr:
                    self._log("retrace_done",
                              f"✅ القراءة {m['cpm']:,.0f} CPM دون العتبة "
                              f"{thr:,.0f} — توقف الانسحاب هنا")
                    return {"ok": True, "safe": True, "steps": steps,
                            "cpm": round(m["cpm"], 1), "threshold_cpm": thr,
                            "reached_entry": reached_entry}
                # آخر نقطة أثر تختلف عن الخلية الحالية
                while self.breadcrumbs and self.breadcrumbs[-1]["cell"] == self.current:
                    self.breadcrumbs.pop()
                if not self.breadcrumbs:
                    reached_entry = True
                    self._log("retrace_done",
                              f"⚠ بلغ نقطة الدخول والقراءة ما زالت "
                              f"{m['cpm']:,.0f} CPM — لا أثر أبعد منه")
                    return {"ok": True, "safe": False, "steps": steps,
                            "cpm": round(m["cpm"], 1), "threshold_cpm": thr,
                            "reached_entry": True}
                tgt = self.breadcrumbs[-1]["cell"]
                r = self._advance_one_cell(tgt, measure=False)
                steps += 1
                if r.get("fatal"):
                    return {"ok": False, "safe": False, "steps": steps,
                            "reason": "توقف الانسحاب: الروبوت عالق"}
                if r.get("no_path"):
                    self.breadcrumbs.pop()      # نقطة غير قابلة للوصول — تخطَّها
        finally:
            self._retracing = False
        return {"ok": False, "safe": False, "steps": steps,
                "reason": f"بلغ سقف {RETRACE_MAX_CELLS} خلية انسحاب"}

    # ══════════════════════════════════════════════════════════════
    # البند 3: إعادة مسح منطقة (التأكيد على مرحلتين)
    # ══════════════════════════════════════════════════════════════
    def _drive_to_cell(self, target, max_cells: int = 40,
                       measure: bool = False) -> dict:
        """يكرّر الأوّلية الحركية حتى بلوغ الخلية أو تعذّره."""
        for _ in range(max(1, int(max_cells))):
            if self.current == tuple(target):
                return {"ok": True, "arrived": True}
            if self.state in (ESTOP, IDLE) or self._withdraw_req is not None:
                return {"ok": False, "arrived": False, "reason": "أُلغي"}
            r = self._advance_one_cell(target, measure=measure)
            if r.get("fatal"):
                return {"ok": False, "arrived": False, "reason": "عالق"}
            if r.get("no_path"):
                return {"ok": False, "arrived": False, "reason": "لا مسار"}
        return {"ok": False, "arrived": self.current == tuple(target),
                "reason": "بلغ سقف الخلايا"}

    def rescan_region(self, center_xy, radius_m: float = None,
                      positions: int = None, dwell_s: float = None) -> dict:
        """
        إعادة مسح **منطقة** حول موقع مشتبه بقياسات جديدة مستقلة (البند 3).

        🔴 **تتجاهل علم `visited` عمداً**: `_next_target` يرفض الخلايا المزارة،
        والتأكيد يحتاج بالضبط **إعادة زيارة** خلايا مُعلَّمة. هذه الدالة تقود
        الروبوت إلى مواضع محدَّدة بلا المرور بمخطّط التغطية أصلاً.

        🔴 **والخلايا المُعاد مسحها تبقى محسوبة في التغطية ولا تُطرح منها** —
        التأكيد إضافة لا تراجع. (لا سطر هنا يُنقص `visited` ولا يُعيد ضبطها.)

        **منطقة لا نقطة**: بلا إنكودرات تنحرف العودة عشرات السنتيمترات، فبدل
        اشتراط بلوغ نقطة نقيس عند **عدة مواضع متفرقة الزوايا** حولها ونقبل
        القراءة أينما وقعت داخل المنطقة — والتنويع **أهم من العدد** لأنه
        يُثلّث المصدر. وكل قياس بزمن أطول من المسح العادي: ننفق الوقت حيث يفيد.
        """
        if self.grid is None or self.locator is None:
            return {"ok": False, "reason": "لا غرفة/منسّق"}
        if not self._can_drive():
            return {"ok": False, "reason": "إعادة المسح تحتاج قيادة محركات "
                                           "فعلية (قياسات جديدة لا محاكاة)"}
        r = float(radius_m if radius_m is not None else CONFIRM_RADIUS_M)
        dwell = float(dwell_s if dwell_s is not None else CONFIRM_DWELL_S)
        n = int(positions if positions is not None else CONFIRM_POSITIONS)
        # الخطة من المنسّق نفسه (حلقة زوايا متفرقة) — مصدر واحد للحقيقة
        plan = self.locator.confirmation_plan(tuple(center_xy), r)[:max(1, n)]
        self._log("rescan",
                  f"إعادة مسح منطقة حول ({center_xy[0]:.2f},{center_xy[1]:.2f}) "
                  f"نصف قطر {r:.2f}م — {len(plan)} موضعاً × {dwell:.0f}ث")
        done, skipped = [], []
        for (px, py) in plan:
            if self.state in (ESTOP, IDLE) or self._withdraw_req is not None:
                break
            cell = self.grid.cell_index(px, py)
            if cell is None or not self.grid.passable(*cell):
                skipped.append({"xy": [px, py], "why": "خارج الغرفة أو محجوبة"})
                continue
            nav = self._drive_to_cell(cell, RESCAN_MAX_CELLS_PER_POINT)
            # ⚠ نقيس عند الموضع **الفعلي** لا المخطَّط: هذا جوهر «منطقة لا
            #    نقطة» — نقبل حيث وصلنا ونُبلّغ موضعه الحقيقي مع σ الخاصة به،
            #    بدل تسجيل قراءة على إحداثيات لم يقف عندها الروبوت.
            ax, ay = ((self.dr.x, self.dr.y) if self.dr
                      else self.grid.cell_center(*self.current))
            m = self._measure(ax, ay, dwell)
            self._feed_locator(ax, ay, m, purpose=CONFIRM)
            # القراءة الجديدة تُحدّث الخريطة أيضاً (إضافة لا تراجع)
            self.grid.update_reading(ax, ay, m["cpm"], m["usvh"])
            self.dirty.add(tuple(self.current))
            self.last_reading = {"cpm": round(m["cpm"], 1),
                                 "usvh": round(m["usvh"], 3),
                                 **{k: v for k, v in classify(m["usvh"]).items()
                                    if k in ("risk", "color")}}
            done.append({"planned": [px, py], "actual": [round(ax, 3), round(ay, 3)],
                         "cpm": round(m["cpm"], 1), "arrived": nav.get("arrived"),
                         "err_m": round(math.hypot(ax - px, ay - py), 2)})
        self._log("rescan_done",
                  f"إعادة المسح: {len(done)} قياس تأكيد جديد"
                  + (f" · تُخطّي {len(skipped)}" if skipped else ""))
        return {"ok": bool(done), "measured": done, "skipped": skipped,
                "center": list(center_xy), "radius_m": r, "dwell_s": dwell}

    # ══════════════════════════════════════════════════════════════
    # البند 5: تتبّع التدرّج في المتر الأخير
    # ══════════════════════════════════════════════════════════════
    def _move_meters(self, dist_m: float, direction: int = +1) -> dict:
        """
        شوط حرّ بالمتر خارج شبكة الخلايا — للاقتراب والتراجع القصير.
        يمرّ بنفس التحقق من الحركة، ويحدّث الموقع بالمسافة **المقاسة**.
        """
        d = abs(float(dist_m))
        if direction >= 0:
            wall = (self._wall_distance_from(self.dr.x, self.dr.y, self.heading)
                    if self.dr else None)
            exp = max(0.0, wall - d) if wall is not None else None
            fwd = self.executor.forward_cell(d, expected_wall_end_m=exp)
        else:
            fwd = self.executor.backward_step(d)
        mv = fwd.get("motion") or {}
        if mv:
            self._last_motion = dict(mv, model_m=fwd.get("covered_m", 0.0))
        step = self._motion_step_m(fwd.get("covered_m", 0.0), mv)
        if self.dr and step:
            self.dr.advance(step if direction >= 0 else -step,
                            verified=mv.get("confident", True))
        # الخلية الحالية تُشتقّ من الموقع (الحركة الحرّة تعبر الحدود)
        idx = (self.grid.cell_index(self.dr.x, self.dr.y) if self.dr else None)
        if idx is not None and idx != self.current:
            self.current = idx
            self.trail.append(idx)
            self.dirty.add(idx)
            self.breadcrumb_push()
        # ⚠ حارس البطارية/الزمن يجب أن يبقى يعمل في **أخطر** الأطوار: الحركة
        #    الحرّة تجري خارج حلقة الخلايا، فلولا هذا السطر لتوقّف الحارس أثناء
        #    الاقتراب من المصدر تحديداً.
        self._check_battery()
        return {"ok": fwd.get("ok", False), "moved_m": round(step, 3),
                "direction": direction, "motion": mv}

    def follow_gradient(self, target_xy=None, max_steps: int = None) -> dict:
        """
        المتر الأخير: **توقّف عن الملاحة بالإحداثيات واتبع الإشارة** (البند 5).
        قِس → تحرّك قليلاً → قِس ثانيةً؛ العدّ يزيد = تقترب.

        ⚠ **حارس التذبذب مبني في الطبقة الأخرى** (`GradientApproach`) ولا
           يُكرَّر هنا — لكنه **يُطاع**: أمر `withdraw` يُنفَّذ فوراً بالانسحاب
           على الأثر، وأمر التراجع ينفَّذ خطوةً للخلف بلا نقاش.
        """
        if not self._can_drive():
            return {"ok": False, "reason": "تتبّع التدرّج يحتاج قيادة محركات"}
        bg = self.locator.background_cpm if self.locator else 0.0
        ga = GradientApproach(background_cpm=bg, step_m=APPROACH_STEP_M,
                              max_steps=max_steps or APPROACH_MAX_STEPS)
        # آخر استعمال للإحداثيات: التوجّه نحو الهدف مرة واحدة، ثم الإشارة تقود
        if target_xy is not None and self.dr is not None:
            self._face(source_bearing_deg((self.dr.x, self.dr.y), target_xy))
        self._log("gradient",
                  f"تتبّع التدرّج — خطوة {APPROACH_STEP_M:.2f}م وقياس "
                  f"{APPROACH_DWELL_S:.0f}ث بينها")
        history, verdict = [], None
        for _ in range(ga.max_steps):
            if self.state in (ESTOP, IDLE) or self._withdraw_req is not None:
                verdict = "أُلغي"
                break
            x, y = ((self.dr.x, self.dr.y) if self.dr
                    else self.grid.cell_center(*self.current))
            m = self._measure(x, y, APPROACH_DWELL_S)
            # `approaching` **معلوم يقيناً** في هذا الطور — وهو مورد حارس
            # الانخفاض-أثناء-الاقتراب الطبيعي (لا تخمين).
            self._feed_locator(x, y, m, purpose=SURVEY,
                               approaching=(ga.direction > 0))
            self.last_reading = {"cpm": round(m["cpm"], 1),
                                 "usvh": round(m["usvh"], 3),
                                 **{k: v for k, v in classify(m["usvh"]).items()
                                    if k in ("risk", "color")}}
            cmd = ga.step(x, y, m["counts"], duration_s=m["duration_s"],
                          heading_deg=self.heading)
            history.append({"xy": [round(x, 2), round(y, 2)],
                            "cpm": cmd["cpm"], "command": cmd["command"]})
            if cmd["command"] == STOP:
                verdict = "stop"
                self._log("gradient_stop", cmd["reason"][:150])
                break
            if cmd["command"] == WITHDRAW:
                verdict = "withdraw"
                self._log("gradient_withdraw", cmd["reason"][:150])
                self.retrace_path(reason="أمر تراجع من حارس التذبذب")
                break
            if cmd["command"] == MOVE:
                self._move_meters(cmd["step_m"], direction=cmd["direction"])
        else:
            verdict = "max_steps"
        return {"ok": True, "verdict": verdict, "steps": ga.n_steps,
                "reversals": ga.reversals, "history": history,
                "stopped": ga.stopped}

    # ══════════════════════════════════════════════════════════════
    # البند 6: تنسيق المراحل الست
    # ══════════════════════════════════════════════════════════════
    def _can_drive(self) -> bool:
        return bool(self.drive_motors and self.executor is not None)

    def _set_phase(self, phase: str) -> None:
        if phase != self.phase:
            self.phase = phase
            self._log("phase", f"المرحلة: {PHASE_AR.get(phase, phase)}")

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

    # ══ عقد القراءات: قياس على نافذة زمنية **محدَّدة** (القسم 1) ══
    def _measure(self, x: float, y: float, duration_s: float) -> dict:
        """
        قياس واحد عند الموضع الحالي — يتوقّف ويعدّ **فعلاً**.

        🔴 مع عدّاد حقيقي تُطرح قراءتا `tally` قبل/بعد فتكون العدّات عدّات
        **هذه الفترة وهذا الموضع**. اشتقاقها من `cpm` (نافذة 30ث منزلقة) يخلط
        عدّ الخلية بعدّ ~عشر خلايا سابقة، أي **يلطّخ الإشارة مكانياً** — وهي
        المعلومة الوحيدة التي يبني عليها محدِّد المصدر تقديره.
        وحين يتعذّر `tally` نسقط إلى النافذة المنزلقة **ونُعلن ذلك** في
        `window` بدل تمريرها كأنها قياس فترة.

        بلا عتاد (ويندوز/محاكاة): نموذج SimWorld فوراً بلا انتظار — لا معلومة
        في الانتظار حين تكون القيمة محسوبة.
        """
        dur = max(0.0, float(duration_s))
        g = getattr(self, "geiger", None)
        if g is not None and getattr(g, "ok", False):
            t0 = g.tally() if hasattr(g, "tally") else None
            if t0 is not None:
                started = time.time()
                time.sleep(dur)
                t1 = g.tally()
                actual = max(time.time() - started, 1e-6)
                if t1 is not None and t1 >= t0:
                    counts = float(t1 - t0)
                    cpm_raw = counts * 60.0 / actual
                    corr = dead_time_correct(cpm_raw)
                    return {"counts": counts, "duration_s": actual,
                            "cpm": corr["cpm_true"], "usvh": corr["usvh"],
                            "cpm_raw": cpm_raw, "window": "exact"}
            # ⚠ **تدهور معلَن لا صامت**: هذا المسار يشتقّ العدّات من النافذة
            #    المنزلقة (30ث) — أي العلّة نفسها التي عولجت بـ`tally`. يبقى
            #    كاحتياط لأن قراءة بلا عدّات أسوأ من قراءة ملطَّخة موسومة،
            #    لكنه يُصرَّح به مرة واحدة في السجل ويُوسم في كل قراءة.
            if not self._rolling_warned:
                self._rolling_warned = True
                self._log("geiger_window",
                          f"⚠ تعذّر `tally` — العدّات مشتقّة من نافذة "
                          f"{GEIGER_WINDOW_S:.0f}ث المنزلقة، فالإشارة **ملطَّخة "
                          f"مكانياً** عبر الخلايا السابقة. تحقّق من العدّاد.")
            st = g.state()
            return {"counts": st["cpm_raw"] * dur / 60.0, "duration_s": dur,
                    "cpm": st["cpm"], "usvh": st["usvh"],
                    "cpm_raw": st["cpm_raw"], "window": "rolling"}
        cpm, usvh = self.world.reading_at(x, y)
        return {"counts": cpm * dur / 60.0, "duration_s": dur,
                "cpm": cpm, "usvh": usvh, "cpm_raw": cpm, "window": "sim"}

    def _pos_sigma(self) -> float:
        """
        🔴 عدم يقين الموضع — **إلزامي مع كل قراءة، ولا يكون صفراً أبداً**.

        الصفر يعني «موضع مؤكَّد» والروبوت بلا إنكودرات. أثره مقاس: عند انحراف
        0.8م يُبلَّغ عدم يقين 0.21م بينما الخطأ الفعلي 0.63م — أضيق 2.9×،
        ومع الترجيح الصحيح صار خطأ التقدير 0.14م بدل 0.46م.
        الأرضية `UNCERTAINTY_INITIAL` لأن أدقّ حالاتنا (تصحيح جدار مباشر)
        لا تنزل تحتها أصلاً — فادّعاء ما دونها اختراع.
        """
        u = self.dr.uncertainty if self.dr is not None else 0.0
        return max(float(u), UNCERTAINTY_INITIAL)

    def _feed_locator(self, x, y, m: dict, purpose: str = SURVEY,
                      approaching=None) -> dict:
        """
        القناة **الوحيدة** إلى محدِّد المصدر — فلا مسار يفلت من `pos_sigma_m`.
        الاستثناء يُسجَّل ولا يُسقط المهمة (طبقة أعلى لا تتحكّم بالأدنى).
        """
        if self.locator is None:
            return {}
        try:
            return self.locator.add_reading(
                float(x), float(y), self.heading,
                counts_L=m["counts"], duration_s=m["duration_s"],
                pos_uncertainty_m=self._pos_sigma(),
                purpose=purpose, approaching=approaching) or {}
        except Exception as e:                 # noqa: BLE001
            self._log("locator_error", f"⚠ تعذّرت تغذية المنسّق: {e}"[:140])
            return {}

    def _inconclusive_extension_s(self, info: dict) -> float:
        """
        كم يُطال التجميع. **فقط للقراءة الغامضة** — أي انحراف نزولي دالّ لم
        يبلغ حدّ الحسم، وهي وحدها التي تحتمل تفسيرين: منطقة نظيفة أم شلل زمن
        ميت. ⚠ الإطالة لكل قراءة قليلة العدّات كارثة زمنية: في غرفة نظيفة
        كل قراءة دون `DWELL_MIN_COUNTS`، فتصير كل خلية 60ث.
        المصدر هو وسم المنسّق نفسه على آخر قراءة (`inconclusive`) — لا نُعيد
        اشتقاق المعيار هنا فيصير مصدرا حقيقة متناقضان.
        """
        if not info or self.locator is None or not self.locator.readings:
            return 0.0
        if not self.locator.readings[-1].get("inconclusive"):
            return 0.0
        extra = info.get("extend_dwell_s") or 0.0
        return min(float(extra), DWELL_MAX_S)

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
        # يُشترط أن يكون **كل** العيّنات متفقة (حسّاس عالق فعلاً) وأن يكون
        # الألترا سونيك يرى بعيداً في كلها — وإلا فقد يكون عائقاً حقيقياً.
        if far_us == samples and samples >= 3:
            if left_hits == samples:
                problems.append("حسّاس IR أمام-يسار يقرأ «عائق» دائماً والمسار أمامه خالٍ "
                                "— تحقّق من توصيله (BCM25) ومن مقاومة المدى")
            if right_hits == samples:
                problems.append("حسّاس IR أمام-يمين يقرأ «عائق» دائماً والمسار أمامه خالٍ "
                                "— تحقّق من توصيله (BCM16) ومن مقاومة المدى")
        return {"ok": not problems, "problems": problems, "sensors": last,
                "ir_left_hits": left_hits, "ir_right_hits": right_hits,
                "samples": samples}

    def _visit(self, cell, low_confidence: bool = False, dwell_s=None,
               purpose: str = SURVEY):
        """
        **يقيس** الخلية (يتوقّف ويعدّ) ثم يسجّلها ويغذّي محدِّد المصدر.

        `low_confidence`: لم يُتحقَّق من حركة الوصول إليها (البند 0) — تُحتسب
        في التغطية لكن بلون مختلف وذكرٍ صريح في النص والتقرير.
        """
        r, c = cell
        gc = self.grid.get(r, c)
        already = gc.visited
        x, y = self.grid.cell_center(r, c)
        # الموضع المقاس أصدق من مركز الخلية حين يتوفّر تتبّع فعلي
        px, py = (self.dr.x, self.dr.y) if self.dr is not None else (x, y)
        m = self._measure(px, py, CELL_DWELL_S if dwell_s is None else dwell_s)
        self.mission_time_s += m["duration_s"]
        cpm, usvh = m["cpm"], m["usvh"]
        anomaly = (not already) and self.stats.is_anomaly(cpm)
        if not already:
            self.stats.add(cpm)
        self.grid.update_reading(x, y, cpm, usvh, low_confidence=low_confidence)
        if low_confidence and not already:
            self._unverified_cells += 1
        self.dirty.add(cell)
        risk = classify(usvh)
        # ── تغذية محدِّد المصدر — **مجاناً بلا وقت إضافي** ────────
        # كل قراءة مسح تدخل الشبكة البايزية أصلاً؛ لا توقف إضافي ولا مسار
        # منفصل. وعدم يقين الموقع **إلزامي** مع كل قراءة (القسم 1).
        info = self._feed_locator(px, py, m, purpose=purpose)
        # التجميع التكيّفي: يُطال الزمن **فقط** للقراءة الغامضة (نظيفة أم شلل؟)
        extra = self._inconclusive_extension_s(info)
        if extra > 0:
            self._log("extend_dwell",
                      f"قراءة غامضة عند ({px:.2f},{py:.2f}) — إطالة التجميع "
                      f"{extra:.0f}ث للحسم بين «نظيفة» و«شلل زمن ميت»")
            m2 = self._measure(px, py, extra)
            self.mission_time_s += m2["duration_s"]
            self._feed_locator(px, py, m2, purpose=purpose)
            cpm, usvh = m2["cpm"], m2["usvh"]     # الأطول أدقّ إحصائياً
            self.grid.update_reading(x, y, cpm, usvh,
                                     low_confidence=low_confidence)
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
            # البند 0: هل تحقّقنا من حركة الوصول إلى هذا الموضع؟
            "motion_verified": 0 if low_confidence else 1,
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
            self._run_cycle()                # ← الدورة الكاملة لا التوثيق وحده
        self.planned_path = []

    # ── الدورة الكاملة: مسح → فرز → تأكيد → اقتراب → توقف → توثيق ──
    def _run_cycle(self) -> dict:
        """
        تنسيق المراحل الست (البند 6). كل انتقال **بحكم المنسّق** لا برغبة
        الملاحة — تدرّج الطبقات في CLAUDE.md §5: الملاحة تنفّذ والمنسّق يحكم.

        ⚠ المراحل الحركية (تأكيد/اقتراب) تحتاج **قيادة محركات فعلية**: التأكيد
           معناه قياسات **جديدة مستقلة**، ومحاكاتها على نفس نموذج البيانات
           السابق ليست تأكيداً بل ترديداً. في المسح المنطقي نُخرج الحكم
           والخطة ونقول صراحةً إن الحركة لم تُنفَّذ.
        """
        out = {"phase_log": []}
        self.cycle = out

        def mark(ph, note=""):
            self._set_phase(ph)
            out["phase_log"].append({"phase": ph, "note": note})

        if self.locator is None:
            return out
        try:
            # ── ٢ فرز: عتبة متسامحة على بيانات المسح (بلا وقت إضافي) ──
            mark(PHASE_SCREEN)
            s = self.locator.screen()
            out["screen"] = {k: s.get(k) for k in
                             ("suspect", "lambda_stat", "threshold", "position",
                              "verdict", "reason")}
            hz = self.locator.hazard()
            out["hazard_level"] = hz["hazard_level"]
            if hz.get("withdraw"):
                # 🔴 السلامة تسبق كل شيء: لا تأكيد ولا اقتراب
                mark(PHASE_WITHDRAW, hz.get("alarm"))
                out["withdraw"] = self.retrace_path(
                    reason=hz.get("alarm") or "تقييم خطر يأمر بالانسحاب") \
                    if self._can_drive() else {"ok": False,
                                               "reason": "بلا محركات"}
                out["documentation"] = self._document_source()
                mark(PHASE_REPORT)
                return out

            if not s.get("suspect"):
                # لا اشتباه ⇒ لا تأكيد ولا اقتراب. التوثيق يُحجب بسببه صراحةً.
                out["documentation"] = self._document_source()
                mark(PHASE_REPORT, "لا اشتباه — لا تأكيد ولا اقتراب")
                return out

            # ── ٣ تأكيد: قياسات جديدة مستقلة على **منطقة** ──────────
            center = self.locator.detector.suspect
            mark(PHASE_CONFIRM, f"منطقة مشتبهة عند {center}")
            if self._can_drive():
                out["rescan"] = self.rescan_region(center)
            else:
                out["rescan"] = {"ok": False, "plan": self.locator
                                 .confirmation_plan(center, CONFIRM_RADIUS_M),
                                 "reason": "المسح المنطقي لا يُنتج قياسات جديدة "
                                           "— خطة التأكيد جاهزة للتنفيذ بالمحركات"}
            v = self.locator.confirm_region(center)
            out["confirm"] = {k: v.get(k) for k in
                              ("ready", "confirmed", "lambda_stat", "position",
                               "uncertainty_m", "n_in_region", "reason")}
            if not v.get("confirmed"):
                out["documentation"] = self._document_source()
                mark(PHASE_REPORT, v.get("reason", "لم يُؤكَّد"))
                return out

            # ── ٤ اقتراب: إحداثيات حتى المتر الأخير ثم التدرّج ───────
            rep = self.locator.report()
            blockers = approach_blockers(rep)
            out["approach_blockers"] = blockers
            if blockers:
                # الموانع تحجب الاقتراب **والصورة** معاً
                out["documentation"] = self._document_source()
                mark(PHASE_REPORT, " · ".join(blockers)[:120])
                return out
            mark(PHASE_APPROACH)
            pos = rep.get("position")
            if self._can_drive() and pos:
                out["transit"] = self._transit_toward(pos)
                out["gradient"] = self.follow_gradient(target_xy=pos)
            else:
                out["approach"] = {"ok": False,
                                   "reason": "الاقتراب يحتاج قيادة محركات"}

            # ── ٥ توقف (قرار المنسّق داخل follow_gradient) ثم ٦ توثيق ──
            mark(PHASE_STOP)
            mark(PHASE_DOCUMENT)
            out["documentation"] = self._document_source()
            mark(PHASE_REPORT)
        except Exception as e:                # noqa: BLE001 — لا تُسقط المهمة
            self._log("cycle_error", f"⚠ خطأ في دورة المراحل: {e}"[:160])
            out["error"] = str(e)[:200]
        finally:
            try:
                self.rover.stop()             # ⚠ إيقاف مضمون بعد أي مرحلة
            except Exception:                 # noqa: BLE001
                pass
        return out

    def _transit_toward(self, pos) -> dict:
        """
        الانتقال بالإحداثيات حتى `GRADIENT_TRANSIT_STOP_M` من الموقع المقدَّر.
        دقّة ±50سم كافية هنا — يكفي بلوغ المنطقة، والمتر الأخير للتدرّج.
        """
        cell = self.grid.cell_index(*pos)
        if cell is None:
            return {"ok": False, "reason": "الموقع المقدَّر خارج الغرفة"}
        for _ in range(RETRACE_MAX_CELLS):
            if self.state in (ESTOP, IDLE) or self._withdraw_req is not None:
                return {"ok": False, "reason": "أُلغي"}
            x, y = ((self.dr.x, self.dr.y) if self.dr
                    else self.grid.cell_center(*self.current))
            if math.hypot(x - pos[0], y - pos[1]) <= GRADIENT_TRANSIT_STOP_M:
                return {"ok": True, "arrived": True,
                        "distance_m": round(math.hypot(x - pos[0], y - pos[1]), 2)}
            if self.current == cell:
                return {"ok": True, "arrived": True, "distance_m": None}
            r = self._advance_one_cell(cell, measure=False)
            if r.get("fatal") or r.get("no_path"):
                return {"ok": False, "reason": r.get("reason", "تعذّر الوصول")}
        return {"ok": False, "reason": "بلغ سقف خلايا الانتقال"}

    # ── التوثيق البصري بعد المسح (يكمل حلقة المشروع) ─────────────
    def _document_source(self) -> dict:
        """
        بعد انتهاء المسح: **اذهب إلى الموقع المقدَّر وصوّره**.

        بلا هذا يبقى نصف هدف المشروع المعلن («تحديد + صورة + تقرير») مبنياً
        وغير موصول: المسح يغذّي المنسّق وتُعرض الخريطة الحرارية ثم تنتهي
        المهمة بلا اقتراب ولا صورة.

        🔴 **الموانع تُفحص أولاً** (`approach_blockers`): منطقة خطرة بكاملها،
        أو موقع غير قابل للتحديد، أو بداية غير قانونية، أو لا دليل أصلاً ⇒
        **لا اقتراب ولا تصوير**، ويُسجَّل السبب صراحةً في تقرير المهمة.
        وهذا ليس تحفّظاً زائداً: الاقتراب من موقع لا نثق به سيرٌ عشوائي داخل
        حقل إشعاعي، والصورة عندها توثّق **مكاناً خاطئاً** وتمنح التقرير ثقة
        لا يملكها.
        """
        self.documentation = None
        if self.locator is None:
            return {"documented": False, "statement": "لا منسّق"}
        try:
            rep = self.locator.report()
            blockers = approach_blockers(rep)
            if blockers:
                self.documentation = {"documented": False, "blockers": blockers,
                                      "statement": "🔴 لم يُنفَّذ التوثيق البصري: "
                                                   + " · ".join(blockers)}
                self._log("documentation_skipped", self.documentation["statement"])
                return self.documentation
            # الكاميرا والدوران **اختياريان**: غيابهما يُبلَّغ ولا يُسقط المهمة
            x = self.dr.x if self.dr else 0.0
            y = self.dr.y if self.dr else 0.0
            self.documentation = run_documentation(
                rep, robot_xy=(x, y), robot_heading_deg=self.heading,
                camera=self.camera, turn_fn=self._doc_turn_fn())
            self._log("documentation",
                      self.documentation.get("statement", "")[:160])
        except Exception as e:                # noqa: BLE001 — لا يُسقط المهمة
            self.documentation = {"documented": False, "blockers": [],
                                  "statement": f"⚠ تعذّر التوثيق البصري: {e}"}
            self._log("documentation_error", str(e)[:120])
        finally:
            # 🔴 المحركات تُوقَف صراحةً بعد التصوير مهما كانت النتيجة
            try:
                self.rover.stop()
            except Exception:                 # noqa: BLE001
                pass
        return self.documentation

    def _doc_turn_fn(self):
        """
        دالة الدوران للتصوير — أو `None` حين لا يُسمح بالحركة.

        ⚠ لا نُدير الروبوت إلا إذا كانت المهمة **تقود المحركات فعلاً**
        (`drive_motors`): في وضع «المسح المنطقي» لا حركة أصلاً، وتمرير دالة
        دوران هناك يجعل التقرير يدّعي «زوايا مختلفة» بلا دوران — نفس العلّة
        المقاسة على العتاد.
        """
        # ⚠ ولا بعد إيقاف الطوارئ: التوثيق قد يُستدعى بعد توقّف قسري، وتدوير
        #    الروبوت حينها يخالف أمر الإيقاف نفسه.
        if not self.drive_motors or self.state in (ESTOP, IDLE):
            return None

        def _turn(deg):
            r = self.rover.turn_by_angle(deg, timeout=ROVER_TURN_TIMEOUT_S)
            ok = not (r.get("timed_out") or r.get("aborted"))
            if ok:
                self.heading = (self.heading + deg) % 360.0
            return ok
        return _turn

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
                "low_confidence": cell.low_confidence,
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

    def _source_state(self) -> dict:
        """
        حالة محدِّد المصدر للبثّ. **الخريطة الحرارية تُبثّ كاملةً**: شبكة غرفة
        3×4م بدقة 0.25م = 192 خلية — عبء تافه أمام قيمتها البصرية (أقوى عنصر
        في العرض: تراها تتركّز مع تقدّم المسح).
        ⚠ أي استثناء هنا يُعيد حالة فارغة ولا يُسقط بثّ المهمة كله.
        """
        if self.locator is None:
            return {"active": False, "reason": "لم تُعرَّف الغرفة بعد"}
        try:
            rep = self.locator.report()
            rep["active"] = True
            rep["truth"] = (list(self.world.source_xy)
                            if self.world and self.world.source_xy else None)
            return rep
        except Exception as e:                 # noqa: BLE001
            return {"active": False, "reason": f"خطأ في المحدِّد: {e}"}

    def _doc_state(self) -> dict | None:
        """نتيجة التوثيق البصري للبثّ — بلا بايتات الصور (تُطلب بمسارها)."""
        d = self.documentation
        if not d:
            return None
        return {k: v for k, v in d.items() if k != "images"} | {
            "n_images": len(d.get("images") or [])}

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
            # البند 0: حالة التحقق من الحركة — تُعرض دائماً، فخريطة كاملة
            # نصفها غير متحقَّق منه يجب أن تقول ذلك لا أن تبدو مكتملة.
            "motion": {"unverified_cells": self._unverified_cells,
                       "stuck_streak": self._stuck,
                       "partial_m": round(self._cell_progress, 2),
                       "last": self._last_motion},
            # المراحل الست + أثر المسار (البنود 4 و6)
            "phase": self.phase,
            "phase_ar": PHASE_AR.get(self.phase, self.phase),
            "cycle": self.cycle,
            "breadcrumbs": [list(b["cell"]) for b in self.breadcrumbs[-200:]],
            "withdraw_pending": bool(self._withdraw_req),
            "source": self._source_state(),
            "documentation": self._doc_state(),
            "battery": (self.rover.battery_state() if BATTERY_MONITOR_ENABLED
                        else batt.classify(None)),
            # الحارسان **معاً**: الجهد أدقّ، والزمن طبقة ثانية لا تُحذف
            "time_limit": self.time_limit_info(),
            "sensors": self.sensors(),
            "reactive": self.last_reactive,
            "reactive_enabled": self.reactive.enabled,
            "drive_motors": self.drive_motors,
            "rover": {"mode": self.rover.mode, "error": self.rover.error,
                      "gyro_bias": round(self.rover.gyro_bias, 4),
                      "bias_calibrated": self.rover.bias_calibrated,
                      # مصدر الاتجاه الفعلي + سلامته (البند 1) — يجب أن يكون
                      # مرئياً: مصدر ساقط إلى بديل يفسّر أي انحراف لاحق
                      "heading_source": self.rover.heading_source.state()},
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
                "heading", "uncertainty_m", "anomaly", "motion_verified"]
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
            "unverified_cells": self._unverified_cells,
            "phase": self.phase,
            "cycle": self.cycle,
            "documentation": self._doc_state(),
            "source": (self.locator.report() if self.locator else None),
            "anomalies": self.anomalies,
            "corrections": len(self.dr.corrections) if self.dr else 0,
            "events": self.events,
        }
