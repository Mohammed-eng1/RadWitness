# -*- coding: utf-8 -*-
"""
lidar_drive.py — قيادة ذاتية بتفادي العوائق من الليدار (نسخة اختبار، بلا SLAM)
=============================================================================
    python3 -m pi.nav.lidar_drive --max-speed 0.15            # على الأرض
    python3 -m pi.nav.lidar_drive --dry-run                   # قرارات فقط (مرفوع)
    python3 -m pi.nav.lidar_drive --sim --dry-run             # بلا أي عتاد

🔴 **التشغيل لا يُمنع أبداً** (قرار المشغّل): قناع غائب، جهة أمام غير مؤكَّدة،
   جايرو معطّل، لوحة لا تردّ ⇒ تحذير مطبوع ويكمل.

الأمان:
- أمر حركة كل ~100ms ومهلة الفيرموير 500ms أثناء القيادة (تُعاد 3000 عند الخروج).
- أثناء المشي: لا لفّة جديدة خلال 0.3ث · خطأ · Ctrl+C ⇒ {"T":1,"L":0,"R":0}.
  عودة الليدار ⇒ يكمل تلقائياً.
- اللفّ بالمكان يُغلق **بالجايرو** (`turn_by_angle`)، والتوقيت احتياط معلَن فقط.
- كل الأوامر عبر `bridge.motors()` (خريطة المحركات + الحاجز).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

from pi.config import (
    LIDAR_MASK_PATH, LIDAR_MAX_SPEED, LIDAR_STALE_S, LIDAR_DRIVE_LOOP_S,
    LIDAR_FW_HEARTBEAT_MS, LIDAR_FW_HEARTBEAT_RESTORE_MS, LIDAR_BACKUP_M,
    LIDAR_BACKUP_SPEED, LIDAR_BACKUP_ABORT_M, LIDAR_LOG_EVERY_S,
    LIDAR_ANGLE_SIGN, LIDAR_YAW_OFFSET_DEG, LIDAR_X_M, LIDAR_Y_M,
    ROBOT_LENGTH_M, ROBOT_WIDTH_M, TURN_POWER, UGV01_TURN_RATE_DPS,
    UGV01_FORWARD_VERIFIED, IS_UGV01,
)
from pi.nav.lidar_avoid import (
    LidarAvoider, mask_set, mask_front_overlap, filter_points, corridor_clearance,
)
from pi.sensors.lidar_c1 import LidarC1, SimLidar, sim_room_scene, to_robot_frame

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "logs")


def load_mask(path: str = LIDAR_MASK_PATH) -> tuple[set, list]:
    """(الزوايا المحجوبة، تحذيرات). غياب القناع أو تغطيته الأمام = تحذير فقط."""
    warns = []
    try:
        with open(path, encoding="utf-8") as f:
            mask = json.load(f)
    except FileNotFoundError:
        return set(), [f"⚠ لا قناع ({path}) — أجزاء الروبوت قد تُرى عوائق. "
                       f"شغّل: python3 -m pi.tests.ugv01.calibrate_lidar_mask"]
    except Exception as e:                      # noqa: BLE001
        return set(), [f"⚠ تعذّرت قراءة القناع ({e}) — يُكمل بلا قناع"]
    blocked = mask_set(mask)
    front = mask_front_overlap(blocked)
    if front:
        warns.append(f"⚠ القناع يغطي زوايا أمامية {front[:6]}{'…' if len(front) > 6 else ''}"
                     f" — عائق هناك لن يُرى! أعد المعايرة في مكان مفتوح")
    cfg = mask.get("config", {})
    if cfg and (cfg.get("sign") != LIDAR_ANGLE_SIGN
                or cfg.get("yaw_offset_deg") != LIDAR_YAW_OFFSET_DEG):
        warns.append("⚠ القناع حُفظ بإعداد زاوية مختلف (LIDAR_ANGLE_SIGN/"
                     "YAW_OFFSET) — أعد المعايرة")
    return blocked, warns


class LidarDriver:
    """الحلقة. تُستعمل من السطر (`main`) ومن الواجهة (`start_thread`/`request_stop`)."""

    def __init__(self, bridge, lidar, max_speed: float = LIDAR_MAX_SPEED,
                 dry_run: bool = False, log_dir: str = LOG_DIR, echo=print):
        self.bridge = bridge
        self.lidar = lidar
        self.dry_run = dry_run or bridge is None
        self.echo = echo
        self.blocked, self.warnings = load_mask()
        self.avoider = LidarAvoider(self.blocked, max_speed)
        self.max_speed = self.avoider.max_speed
        self._stop = threading.Event()
        self._thread = None
        self.running = False
        self.last = None                 # آخر قرار (للواجهة)
        self.outcome = None
        self.moving = False
        self._stale_announced = False
        self._last_log = (None, 0.0)
        self._last_echo = (None, 0.0)    # (الفعل، الوقت) — سطر للشاشة لكل تغيّر فعل أو كل ثانية
        self._inside_warned = False
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(
            log_dir, time.strftime("lidar_drive_%Y%m%d_%H%M%S.jsonl"))
        self._log_f = open(self.log_path, "a", encoding="utf-8")

    # ── واجهة التشغيل ─────────────────────────────────────────────
    def warn(self, msg: str) -> None:
        self.warnings.append(msg)
        self.echo(msg)
        self._log({"event": "warning", "msg": msg})

    def request_stop(self) -> None:
        self._stop.set()

    def start_thread(self) -> None:
        self._thread = threading.Thread(target=self.run, name="lidar_drive",
                                        daemon=True)
        self._thread.start()

    def join(self, timeout: float = 5.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def state(self) -> dict:
        d = self.last or {}
        return {"running": self.running, "dry_run": self.dry_run,
                "max_speed": self.max_speed, "outcome": self.outcome,
                "action": d.get("action"), "reason": d.get("reason"),
                "front_m": d.get("front_m"), "attempts": d.get("attempts"),
                "lidar": self.lidar.state(), "warnings": self.warnings[-8:],
                "log": self.log_path}

    # ── الحركة (dry-run لا يرسل شيئاً) ────────────────────────────
    def _motors(self, l: float, r: float) -> None:
        self.moving = bool(l or r)
        if not self.dry_run:
            self.bridge.motors(l, r)

    def _halt(self) -> None:
        self.moving = False
        if not self.dry_run:
            self.bridge.motors(0.0, 0.0)

    def _fw(self, obj: dict) -> None:
        if not self.dry_run and getattr(self.bridge, "mode", "") == "real":
            self.bridge._send(obj)

    # ── الدورة ────────────────────────────────────────────────────
    def run(self) -> str:
        self.running = True
        self.outcome = None
        try:
            self._prepare()
            while not self._stop.is_set():
                t0 = time.time()
                if self._tick() == "trapped":
                    if self.dry_run:
                        # بلا حركة لا يتغيّر المشهد فالمحاولات تُستهلك في ثوانٍ:
                        # يُعلن ويكمل (والقيادة الفعلية تتوقف كما طُلب)
                        self.echo("(dry-run: يكمل بعد «محصور» — المحاولات صُفّرت)")
                        self.avoider.reset()
                        self._stop.wait(2.0)
                        continue
                    self.outcome = "trapped"
                    break
                dt = LIDAR_DRIVE_LOOP_S - (time.time() - t0)
                if dt > 0:
                    self._stop.wait(dt)
            if self.outcome is None:
                self.outcome = "stopped"
        except KeyboardInterrupt:
            self.outcome = "ctrl_c"
        except Exception as e:                   # noqa: BLE001
            self.outcome = f"error: {e}"
            self.echo(f"⛔ خطأ: {e}")
        finally:
            self._shutdown()
        return self.outcome

    def _prepare(self) -> None:
        for w in list(self.warnings):
            self.echo(w)
        self._log({"event": "start", "dry_run": self.dry_run,
                   "max_speed": self.max_speed, "mask_deg": len(self.blocked),
                   "config": {"sign": LIDAR_ANGLE_SIGN,
                              "yaw_offset_deg": LIDAR_YAW_OFFSET_DEG,
                              "lidar_xy": [LIDAR_X_M, LIDAR_Y_M],
                              "robot_lw": [ROBOT_LENGTH_M, ROBOT_WIDTH_M]}})
        if IS_UGV01 and not UGV01_FORWARD_VERIFIED and not self.dry_run:
            self.warn("⚠ جهة «أمام» لم تُؤكَّد بعد تصحيح الخريطة — راقب أول "
                      "ثانية: إن رجع للخلف اضغط Ctrl+C")
        if not self.dry_run:
            if getattr(self.bridge, "mode", "sim") != "real":
                why = getattr(self.bridge, "error", None) or "وضع محاكاة"
                self.warn(f"⚠ لوحة الروبوت ليست متصلة ({why}) — الأوامر إلى المحاكاة")
            self._halt()                          # يُكمل إقلاع/تهيئة اللوحة
            self._fw({"T": 136, "cmd": LIDAR_FW_HEARTBEAT_MS})
            self.echo(f"مهلة الفيرموير {LIDAR_FW_HEARTBEAT_MS}ms · قياس انحياز "
                      f"الجايرو (الروبوت ساكن)…")
            self.bridge.calibrate_gyro_bias()
            src = self.bridge.heading_source
            if not src.ok:
                self.warn(f"⚠ الجايرو غير صالح ({src.error}) — اللفّ بالتوقيت (تقريبي)")
        if self.lidar.wait_scan(timeout=3.0) is None:
            self.warn(f"⚠ لا لفّة من الليدار بعد 3ث ({self.lidar.error}) — "
                      f"ينتظر ويكمل حين تصل")

    def _shutdown(self) -> None:
        try:
            self._halt()
            self._fw({"T": 136, "cmd": LIDAR_FW_HEARTBEAT_RESTORE_MS})
        except Exception:                        # noqa: BLE001
            pass
        self.running = False
        self._log({"event": "end", "outcome": self.outcome})
        try:
            self._log_f.close()
        except Exception:                        # noqa: BLE001
            pass
        self.echo(f"انتهى: {self.outcome} · السجل {self.log_path}")

    def _points(self):
        s = self.lidar.latest()
        return None if s is None else to_robot_frame(s[1])

    def _tick(self):
        if not self.lidar.fresh(LIDAR_STALE_S):
            # 🔴 أثناء المشي فقط يهمّ، لكن الصفر وهو واقف لا يضرّ ويُبقي النبضة
            self._halt()
            if not self._stale_announced:
                self._stale_announced = True
                age = self.lidar.age_s()
                age_txt = "لا لفّة بعد" if age == float("inf") else f"{age:.1f}ث"
                self.warn(f"⚠ الليدار بلا لفّة جديدة ({age_txt} · "
                          f"{self.lidar.error}) ⇒ توقف — يكمل حين يعود")
            return None
        if self._stale_announced:
            self._stale_announced = False
            self.echo("✅ عاد الليدار — يكمل")
        d = self.avoider.decide(self._points(), time.time())
        self.last = d
        self._record(d)
        act = d["action"]
        if act in ("go", "arc"):
            self._motors(d["l"], d["r"])
        elif act == "spin":
            self._spin(d["deg"])
        elif act == "backup":
            self._backup()
            self.avoider.note_backed_up()
        elif act in ("stop", "trapped"):
            self._halt()
            if act == "trapped":
                self.echo("⛔ محصور — توقف")
        return act

    def _spin(self, deg: float) -> None:
        """لفّ بالمكان بالجايرو (موجب = يسار). `turn_by_angle` موجب = يمين."""
        self._halt()
        if self.dry_run:
            self._stop.wait(min(1.0, abs(deg) / UGV01_TURN_RATE_DPS))
            return
        res = self.bridge.turn_by_angle(-deg)
        self._log({"event": "spin", "deg_left": round(deg, 1),
                   "turned": res.get("turned_deg"), "aborted": res.get("aborted"),
                   "timed_out": res.get("timed_out")})
        if res.get("aborted") == "heading_source_fault":
            self._timed_spin(deg)

    def _timed_spin(self, deg: float) -> None:
        """⚠ احتياط معلَن: الجايرو معطّل ⇒ لفّ بالزمن من المعدل المقاس (88°/ث)."""
        self.warn(f"⚠ لفّ {deg:+.0f}° بالتوقيت (الجايرو معطّل) — تقريبي")
        t_end = time.time() + abs(deg) / UGV01_TURN_RATE_DPS
        while time.time() < t_end and not self._stop.is_set():
            self.bridge.turn("L" if deg > 0 else "R", TURN_POWER)
            time.sleep(LIDAR_DRIVE_LOOP_S)
        self._halt()

    def _backup(self) -> None:
        """رجوع بطيء على خلف فاضٍ؛ يقف إن اقترب شيء خلفه أو تجمّد الليدار."""
        if self.dry_run:
            self._stop.wait(0.3)
            return
        covered, last = 0.0, time.time()
        t_end = last + 3.0 * LIDAR_BACKUP_M / max(LIDAR_BACKUP_SPEED, 1e-3)
        while covered < LIDAR_BACKUP_M and time.time() < t_end \
                and not self._stop.is_set():
            if not self.lidar.fresh(LIDAR_STALE_S):
                break
            pts, _ = filter_points(self._points() or [], self.blocked)
            rear = corridor_clearance(pts, forward=False)
            if rear is not None and rear < LIDAR_BACKUP_ABORT_M:
                self._log({"event": "backup_abort", "rear_m": round(rear, 3)})
                break
            self._motors(-LIDAR_BACKUP_SPEED, -LIDAR_BACKUP_SPEED)
            time.sleep(LIDAR_DRIVE_LOOP_S)
            now = time.time()
            ws = self.bridge.wheel_speeds_mps()
            v = -(ws[0] + ws[1]) / 2.0 if ws else LIDAR_BACKUP_SPEED
            covered += max(0.0, v) * (now - last)
            last = now
        self._halt()
        self._log({"event": "backup", "covered_m": round(covered, 3)})

    # ── السجل ─────────────────────────────────────────────────────
    def _log(self, obj: dict) -> None:
        try:
            self._log_f.write(json.dumps(dict(obj, t=round(time.time(), 3)),
                                         ensure_ascii=False) + "\n")
            self._log_f.flush()
        except Exception:                        # noqa: BLE001
            pass

    def _record(self, d: dict) -> None:
        if d["stats"].get("inside") and not self._inside_warned:
            self._inside_warned = True
            self.warn(f"⚠ {d['stats']['inside']} نقطة داخل مستطيل الروبوت "
                      f"(زاوية°، بُعد م: {d.get('inside_pts')}) (ROBOT_LENGTH_M×"
                      f"ROBOT_WIDTH_M حول LIDAR_X_M/Y_M) — تُعامل **عوائق ملاصقة**. "
                      f"إن كانت جزءاً من الروبوت: أعد calibrate_lidar_mask أو صحّح "
                      f"الأبعاد/موضع الليدار في config")
        key = (d["action"], d.get("reason"))
        now = time.time()
        if key == self._last_log[0] and now - self._last_log[1] < LIDAR_LOG_EVERY_S:
            return
        self._last_log = (key, now)
        rnd = lambda v: None if v is None else round(v, 3)   # noqa: E731
        self._log({"event": "decision", "action": d["action"],
                   "l": rnd(d.get("l")), "r": rnd(d.get("r")),
                   "deg": rnd(d.get("deg")), "front_m": rnd(d.get("front_m")),
                   "rear_m": rnd(d.get("rear_m")),
                   "sectors": {k: rnd(v) for k, v in d["sectors"].items()},
                   "stats": d["stats"], "attempts": d.get("attempts"),
                   "inside_pts": d.get("inside_pts"),
                   "lidar_age_s": rnd(self.lidar.age_s()), "reason": d.get("reason")})
        # الشاشة: تغيّر **الفعل** أو مرور ثانية — لا كل تذبذب 1سم في السبب
        if d["action"] != self._last_echo[0] or now - self._last_echo[1] >= LIDAR_LOG_EVERY_S:
            self._last_echo = (d["action"], now)
            extra = (f" L={d['l']:.2f} R={d['r']:.2f}" if "l" in d else
                     f" {d['deg']:+.0f}°" if "deg" in d else "")
            self.echo(f"[{time.strftime('%H:%M:%S')}] {d['action']}{extra} — {d.get('reason')}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="قيادة ذاتية بالليدار (تفادي عوائق)")
    ap.add_argument("--max-speed", type=float, default=LIDAR_MAX_SPEED,
                    help=f"م/ث (السقف {LIDAR_MAX_SPEED})")
    ap.add_argument("--dry-run", action="store_true",
                    help="قرارات فقط بلا أوامر حركة (الروبوت مرفوع)")
    ap.add_argument("--sim", action="store_true", help="ليدار وهمي (غرفة) بلا عتاد")
    ap.add_argument("--port", default=None, help="منفذ الليدار (وإلا VID/PID)")
    a = ap.parse_args(argv)
    if a.max_speed > LIDAR_MAX_SPEED:
        print(f"⚠ --max-speed {a.max_speed} فوق السقف ⇒ {LIDAR_MAX_SPEED}")

    lidar = (SimLidar(sim_room_scene()) if a.sim else LidarC1(port=a.port)).start()
    bridge = None
    if not a.dry_run:
        from pi.rover.bridge import WaveRoverBridge
        bridge = WaveRoverBridge(mode="sim" if a.sim else "real")
    drv = LidarDriver(bridge, lidar, a.max_speed, dry_run=a.dry_run)
    print(f"القيادة بالليدار · سقف {drv.max_speed} م/ث · "
          f"{'dry-run (بلا حركة)' if drv.dry_run else 'حركة فعلية'} · Ctrl+C للإيقاف")
    try:
        outcome = drv.run()
    finally:
        lidar.stop()                              # 🔴 STOP للمحرك
        if bridge is not None:
            bridge.close()
    return 0 if outcome in ("stopped", "ctrl_c") else 1


if __name__ == "__main__":
    sys.exit(main())
