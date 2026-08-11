# -*- coding: utf-8 -*-
"""
recorder.py — 🎥 تسجيل المهمة كاملةً على الراسبري (بطلب صريح)
=============================================================
جلسة واحدة لكل مهمة داخل مجلد يحمل ختم الوقت:

    captures/missions/mission_YYYYmmdd_HHMMSS/
        frames/frame_00001.jpg …     إطارات الكاميرا الدورية
        events.jsonl                 سجل الأحداث كاملاً (سطر لكل حدث)
        readings.csv                 نفس تصدير `/api/mission/csv`
        report.json                  التقرير النهائي
        photos/doc_1.jpg …           صور التوثيق الثلاث كما هي
        summary.txt                  خلاصة مقروءة + أمر ffmpeg لتجميع فيديو

🔴 **معطّل افتراضياً**: الكتابة المستمرة تستهلك عمر بطاقة SD. لا تسجيل بلا
   ضغطة المشغّل، ويزول العلم بإعادة تشغيل السيرفر.

⚠ **لا يزاحم الصور التوثيقية**: `CameraReader.snapshot_jpeg` يرفض لقطة
  ثانية أثناء لقطة جارية (حارس تكديس الخيوط)، فمزاحمة مرحلة التوثيق كانت
  ستُفقد صور التقرير — وهي المخرَج الذي لا يُعوَّض. المسجّل يتوقف عن
  الالتقاط في تلك المرحلة تماماً، ويسجّل عدد الإطارات المتخطّاة.

⚠ **لا يرمي استثناءً أبداً**: خيط مستقل، وكل عملية قرص داخل `try`. تسجيل
  فاشل يجب أن يفقد التسجيل وحده لا المهمة.

⚠ ولا يحمل أي منطق ملاحة: يقرأ حالة المهمة ولا يكتب فيها إطلاقاً.
"""
from __future__ import annotations

import json
import os
import shutil
import threading
import time

from pi.config import (
    MISSION_RECORD_DIR, MISSION_RECORD_FPS, MISSION_RECORD_MAX_MB,
    MISSION_RECORD_JPEG_MIN_B,
)

# أطوار المهمة التي لا نلتقط فيها — التوثيق يملك الكاميرا وحده
_CAMERA_BUSY_PHASES = frozenset({"document"})
# حالات المهمة التي تعني «جلسة جارية»
_ACTIVE_STATES = frozenset({"running", "paused"})


class MissionRecorder:
    """
    مسجّل جلسة المهمة. يُنشأ مرة في السيرفر ويعيش بخيط واحد.

    دورة الحياة **يقودها هو** بمراقبة حالة المهمة — لا تعديل في `mission.py`:
    حالة المهمة تصير `running` والتسجيل مفعّل ⇒ تُفتح جلسة؛ وتخرج من
    الحالات النشطة ⇒ تُغلق وتُكتب الملفات.
    """

    def __init__(self, mission, camera, root: str = MISSION_RECORD_DIR):
        self.mission = mission
        self.camera = camera
        self.root = root
        self.enabled = False          # ⚠ لا تسجيل إلا بطلب صريح
        self.session = None           # مسار الجلسة الجارية أو None
        self.frames = 0
        self.skipped = 0              # إطارات تُخطّيت (توثيق أو التقاط فاشل)
        self.bytes_written = 0
        self.error = None
        self.last_saved = None        # آخر جلسة مكتملة (يعرضها المشغّل)
        self._t0 = None
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

    # ── التحكم ───────────────────────────────────────────────────
    def set_enabled(self, on: bool) -> dict:
        """
        يرفع/يخفض علم التسجيل. الإطفاء **يُغلق جلسة جارية فوراً** ويحفظها —
        لا نترك جلسة نصف مكتوبة على القرص بلا ملفاتها.
        """
        on = bool(on)
        self.enabled = on
        if not on and self.session is not None:
            self._close_session("أُوقف التسجيل بطلب المشغّل")
        if on and self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        return self.state()

    def shutdown(self) -> None:
        """إغلاق نظيف عند إيقاف السيرفر — الجلسة الجارية تُحفظ لا تُهدر."""
        self._stop.set()
        if self.session is not None:
            self._close_session("أُغلق السيرفر")

    def state(self) -> dict:
        return {"enabled": self.enabled,
                "recording": self.session is not None,
                "session": (os.path.basename(self.session)
                            if self.session else None),
                "frames": self.frames, "skipped": self.skipped,
                "mb": round(self.bytes_written / 1e6, 1),
                "max_mb": MISSION_RECORD_MAX_MB,
                "last_saved": self.last_saved, "error": self.error}

    # ── الحلقة ───────────────────────────────────────────────────
    def _loop(self) -> None:
        period = 1.0 / max(0.2, float(MISSION_RECORD_FPS))
        while not self._stop.is_set():
            try:
                self._tick()
            except Exception as e:            # noqa: BLE001 — لا يسقط الخيط
                self.error = str(e)[:120]
            self._stop.wait(period)

    def _tick(self) -> None:
        st = getattr(self.mission, "state", "idle")
        active = st in _ACTIVE_STATES
        if self.session is None:
            if self.enabled and active:
                self._open_session()
            return
        if not active:
            self._close_session(f"انتهت المهمة (الحالة {st})")
            return
        # ⚠ التوثيق يملك الكاميرا وحده — نتخطّى ونعدّ
        if getattr(self.mission, "phase", "") in _CAMERA_BUSY_PHASES:
            self.skipped += 1
            return
        if self.bytes_written >= MISSION_RECORD_MAX_MB * 1e6:
            return                                # بلغ السقف — أُعلن عند بلوغه
        self._grab_frame()

    def _grab_frame(self) -> None:
        cam = self.camera
        if cam is None:
            self.skipped += 1
            return
        try:
            data = cam.snapshot_jpeg()
        except Exception:                         # noqa: BLE001
            data = None
        # ⚠ إطار فارغ/قزم ليس صورة: `snapshot_jpeg` يُعيد None حين تكون
        #   لقطة أخرى جارية — وهو تخطٍّ مشروع لا خطأ.
        if not data or len(data) < MISSION_RECORD_JPEG_MIN_B:
            self.skipped += 1
            return
        self.frames += 1
        path = os.path.join(self.session, "frames",
                            f"frame_{self.frames:05d}.jpg")
        try:
            with open(path, "wb") as f:
                f.write(data)
            self.bytes_written += len(data)
            if self.bytes_written >= MISSION_RECORD_MAX_MB * 1e6:
                self._log_to_mission(
                    f"🎥 بلغ التسجيل سقف {MISSION_RECORD_MAX_MB}MB — "
                    f"توقّف الالتقاط والملفات ستُحفظ عند انتهاء المهمة")
        except Exception as e:                    # noqa: BLE001
            self.frames -= 1
            self.skipped += 1
            self.error = f"تعذّرت كتابة الإطار: {e}"[:120]

    # ── الجلسة ───────────────────────────────────────────────────
    def _open_session(self) -> None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.root, f"mission_{stamp}")
        try:
            os.makedirs(os.path.join(path, "frames"), exist_ok=True)
            os.makedirs(os.path.join(path, "photos"), exist_ok=True)
        except Exception as e:                    # noqa: BLE001
            self.error = f"تعذّر إنشاء مجلد التسجيل: {e}"[:120]
            return
        with self._lock:
            self.session = path
            self.frames = self.skipped = self.bytes_written = 0
            self.error = None
            self._t0 = time.time()
        self._log_to_mission(f"🎥 بدأ تسجيل المهمة → {os.path.basename(path)}")

    def _close_session(self, why: str) -> None:
        path = self.session
        if path is None:
            return
        with self._lock:
            self.session = None
        elapsed = time.time() - (self._t0 or time.time())
        # 🔴 كل كتابة على حدة: فشل ملف لا يمنع البقية. تسجيل ناقص خير من لا شيء.
        self._write_events(path)
        self._write_csv(path)
        self._write_report(path)
        n_photos = self._copy_photos(path)
        self._write_summary(path, why, elapsed, n_photos)
        self.last_saved = os.path.basename(path)
        self._log_to_mission(
            f"🎥 حُفظ تسجيل المهمة: {self.frames} إطاراً · "
            f"{n_photos} صورة توثيق · {self.bytes_written / 1e6:.1f}MB → "
            f"{os.path.basename(path)}")

    # ── الملفات (كلٌّ محروس على حدة) ─────────────────────────────
    def _write_events(self, path: str) -> None:
        try:
            events = list(getattr(self.mission, "events", []) or [])
            with open(os.path.join(path, "events.jsonl"), "w",
                      encoding="utf-8") as f:
                for e in events:
                    f.write(json.dumps(e, ensure_ascii=False) + "\n")
        except Exception as e:                    # noqa: BLE001
            self.error = f"events: {e}"[:120]

    def _write_csv(self, path: str) -> None:
        try:
            data = self.mission.csv_bytes()
            with open(os.path.join(path, "readings.csv"), "wb") as f:
                f.write(data)
        except Exception as e:                    # noqa: BLE001
            self.error = f"csv: {e}"[:120]

    def _write_report(self, path: str) -> None:
        try:
            rep = self.mission.report()
            with open(os.path.join(path, "report.json"), "w",
                      encoding="utf-8") as f:
                json.dump(rep, f, ensure_ascii=False, indent=2)
        except Exception as e:                    # noqa: BLE001
            self.error = f"report: {e}"[:120]

    def _copy_photos(self, path: str) -> int:
        """صور التوثيق من الذاكرة إلى القرص — الجلسة تكتمل بلا الرجوع للسيرفر."""
        n = 0
        try:
            imgs = (getattr(self.mission, "documentation", None) or {}).get("images") or []
            for i, data in enumerate(imgs):
                if not data:
                    continue
                with open(os.path.join(path, "photos", f"doc_{i + 1}.jpg"),
                          "wb") as f:
                    f.write(data)
                n += 1
        except Exception as e:                    # noqa: BLE001
            self.error = f"photos: {e}"[:120]
        return n

    def _write_summary(self, path: str, why: str, elapsed: float,
                       n_photos: int) -> None:
        try:
            src = {}
            try:
                src = (self.mission.report() or {}).get("source") or {}
            except Exception:                     # noqa: BLE001
                pass
            lines = [
                "تسجيل مهمة RMS Rover v2",
                "=" * 40,
                f"المجلد        : {os.path.basename(path)}",
                f"سبب الإغلاق   : {why}",
                f"المدة         : {elapsed / 60.0:.1f} دقيقة",
                f"الإطارات      : {self.frames} (متخطّاة {self.skipped})",
                f"الحجم         : {self.bytes_written / 1e6:.1f} MB",
                f"صور التوثيق   : {n_photos}",
                f"الخلاصة       : {src.get('headline', '—')}",
                f"التغطية       : {getattr(self.mission, 'coverage_text', lambda: '—')()}"
                if callable(getattr(self.mission, "coverage_text", None))
                else "",
                "",
                "الملفات:",
                "  frames/       إطارات الكاميرا الدورية",
                "  events.jsonl  سجل الأحداث كاملاً",
                "  readings.csv  القراءات",
                "  report.json   التقرير النهائي",
                "  photos/       صور التوثيق",
                "",
                "لتجميع فيديو من الإطارات (يحتاج ffmpeg):",
                f"  ffmpeg -framerate {MISSION_RECORD_FPS:.0f} -pattern_type glob \\",
                f"         -i '{os.path.basename(path)}/frames/*.jpg' \\",
                f"         -c:v libx264 -pix_fmt yuv420p {os.path.basename(path)}.mp4",
            ]
            with open(os.path.join(path, "summary.txt"), "w",
                      encoding="utf-8") as f:
                f.write("\n".join(x for x in lines if x != "") + "\n")
        except Exception as e:                    # noqa: BLE001
            self.error = f"summary: {e}"[:120]

    def _log_to_mission(self, msg: str) -> None:
        """يكتب في سجل المهمة — التسجيل حدث تشغيلي يستحق أثراً مرئياً."""
        try:
            self.mission._log("record", msg)
        except Exception:                         # noqa: BLE001
            pass


def list_sessions(root: str = MISSION_RECORD_DIR, limit: int = 30) -> list:
    """أداة خارجية (واجهة): الجلسات المحفوظة، الأحدث أولاً."""
    out = []
    try:
        names = sorted(os.listdir(root), reverse=True)[:limit]
    except Exception:                             # noqa: BLE001
        return out
    for name in names:
        p = os.path.join(root, name)
        if not os.path.isdir(p):
            continue
        try:
            frames = len(os.listdir(os.path.join(p, "frames")))
        except Exception:                         # noqa: BLE001
            frames = 0
        size = 0
        for base, _dirs, files in os.walk(p):
            for fn in files:
                try:
                    size += os.path.getsize(os.path.join(base, fn))
                except Exception:                 # noqa: BLE001
                    pass
        out.append({"name": name, "frames": frames,
                    "mb": round(size / 1e6, 1),
                    "has_report": os.path.exists(os.path.join(p, "report.json"))})
    return out
