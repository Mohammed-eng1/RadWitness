# -*- coding: utf-8 -*-
"""
modes.py — الأنماط الثلاثة الحصرية (طبقة الواجهة)
==================================================
| النمط | السلوك |
|---|---|
| `sim`    | محاكاة `sim_world` — بلا عتاد، للتطوير والعرض |
| `manual` | قيادة يدوية مباشرة + بث كاميرا + قراءات حية |
| `auto`   | الدورة الكاملة: مسح → فرز → تأكيد → اقتراب → توثيق |

⚠ **هذه طبقة واجهة لا منطق ملاحة**: لا تعرف شيئاً عن المراحل ولا الشبكة.
كل ما تفعله أن تترجم «النمط» إلى نداءات على واجهة المهمة **القائمة**
(`pause`/`resume`/`set_drive_motors`) وإلى تفعيل بوابة الأوامر. منطق
الدورة يبقى في `mission.py` بالكامل.

────────────────────────────────────────────────────────────────────
## 🔴 الحصرية: نمط واحد نشط

نمطان نشطان معاً يعنيان آمرين على منفذ سيريال واحد. الانتقال يمرّ دائماً
بـ`_leave()` قبل `_enter()`، و`_leave(manual)` **يوقف المحركات**.

## 🔴 التبديل أثناء مهمة: إيقاف مؤقت لا إلغاء

مسح نصف مكتمل يمثّل دقائق من الزمن وخلايا مقيسة؛ إلغاؤه لأن المستخدم
أراد نظرة على الكاميرا خسارة بلا مقابل. فالتبديل:
1. يتطلب **تأكيداً صريحاً** (`confirm=True`) — وإلا يُرفض ويُعيد
   `needs_confirm` فتسأل الواجهة.
2. ينادي `mission.pause()` — الحالة والخلايا المقيسة تبقى.
3. والعودة إلى `auto` تنادي `mission.resume()` **إن كنّا نحن من أوقفها**.

⚠ الشرط الأخير مقصود: لو أوقف المستخدم المهمة بنفسه ثم بدّل الأنماط
ذهاباً وإياباً، فاستئنافها تلقائياً يجعل الروبوت يتحرك بلا أمر — وهو
بالضبط ما لا يجوز أن يحدث في نظام يقود محركات.
"""
from __future__ import annotations

MODE_SIM = "sim"
MODE_MANUAL = "manual"
MODE_AUTO = "auto"
MODES = (MODE_SIM, MODE_MANUAL, MODE_AUTO)

MODE_AR = {MODE_SIM: "محاكاة", MODE_MANUAL: "قيادة يدوية", MODE_AUTO: "قيادة ذاتية"}


class ModeManager:
    """يدير النمط النشط وانتقالاته. لا يملك عتاداً — ينادي واجهات قائمة."""

    def __init__(self, mission, manual, camera=None):
        self.mission = mission
        self.manual = manual
        self.camera = camera
        self.mode = MODE_SIM
        self.camera_streaming = False
        self._paused_by_switch = False
        self.last_reason = ""

    # ── الاستعلام ───────────────────────────────────────────────
    def is_mission_active(self) -> bool:
        """هل هناك مسح جارٍ فعلاً (حالة أو خيط محركات حيّ)؟"""
        if self.mission.state in ("running", "returning"):
            return True
        w = getattr(self.mission, "_worker", None)
        return w is not None and w.is_alive()

    # ── الانتقال ────────────────────────────────────────────────
    def switch(self, mode: str, confirm: bool = False) -> dict:
        """
        ينتقل إلى نمط. يُعيد `needs_confirm` بدل التنفيذ إن كانت مهمة جارية.
        """
        mode = str(mode or "").strip().lower()
        if mode not in MODES:
            return {"ok": False, "error": f"نمط غير معروف: {mode!r}",
                    "mode": self.mode}
        if mode == self.mode:
            return {"ok": True, "mode": self.mode, "unchanged": True,
                    **self.state()}

        # 🔴 مهمة جارية ⇒ تأكيد أولاً، ثم **إيقاف مؤقت لا إلغاء**
        if self.is_mission_active() and not confirm:
            return {
                "ok": False, "needs_confirm": True, "mode": self.mode,
                "target": mode,
                "error": (f"مهمة جارية — التبديل إلى «{MODE_AR[mode]}» سيوقفها "
                          f"**مؤقتاً** (لا إلغاء) وتُستأنف عند العودة. أتؤكّد؟"),
            }

        prev = self.mode
        self._leave(prev)
        self.mode = mode
        self._enter(mode, prev)
        self._log("mode", f"النمط: {MODE_AR[mode]}" +
                  (" ⚠ المحركات مفعّلة — الروبوت سيتحرك فعلياً"
                   if self.motors_live() else ""))
        return {"ok": True, "mode": self.mode, "from": prev, **self.state()}

    def _leave(self, mode: str) -> None:
        if mode == MODE_MANUAL:
            # 🔴 الخروج يوقف المحركات دائماً — لا عجلة تدور بعد تبديل تبويب
            self.manual.set_enabled(False)
            self._stop_camera()
        if mode == MODE_AUTO and self.is_mission_active():
            self.mission.pause()
            self._paused_by_switch = True
            self.last_reason = "أُوقفت المهمة مؤقتاً بسبب تبديل النمط"

    def _enter(self, mode: str, prev: str) -> None:
        if mode == MODE_MANUAL:
            self.manual.set_enabled(True)
            self._start_camera()
        elif mode == MODE_AUTO:
            # ⚠ لا استئناف إلا إذا **نحن** من أوقفها (انظر شرح الرأس)
            if self._paused_by_switch and self.mission.state == "paused":
                self.mission.resume()
                self.last_reason = "استُؤنفت المهمة بعد العودة إلى القيادة الذاتية"
            self._paused_by_switch = False
        elif mode == MODE_SIM:
            # المحاكاة لا تقود محركات مطلقاً
            try:
                self.mission.set_drive_motors(False)
            except Exception:                    # noqa: BLE001
                pass

    # ── الكاميرا: بثّ **عند الطلب فقط** ─────────────────────────
    def _start_camera(self) -> None:
        """يبدأ البث مع دخول النمط (توفيراً للمعالج والنطاق)."""
        self.camera_streaming = True

    def _stop_camera(self) -> None:
        """
        يخفض راية البثّ عند مغادرة النمط — **ولا يحرّر الجهاز**.

        🔴 كان يُغلق الكاميرا هنا، والسبب وجيه وقتها: البثّ كان مقصوراً على
        نمط القيادة اليدوية، فمغادرته تعني ألّا مشاهد. لكن البثّ صار مسموحاً
        في كل الأنماط (يتوقف مؤقتاً في مرحلة التوثيق وحدها)، فإغلاق الجهاز
        عند كل تبديل نمط صار **يقطع بثّاً جارياً** بلا داعٍ.

        ⚠ وأسوأ من الانقطاع: `close()` يُنادى من حلقة asyncio، ولو صادف
          لقطةً جارية تملك القفل لهُجر المقبض (انظر `camera.close`) — تسريب
          مقبض عند كل تبديل نمط. الجهاز يُحرَّر عند إغلاق السيرفر.
        """
        self.camera_streaming = False

    # ── الحالة المعروضة ─────────────────────────────────────────
    def motors_live(self) -> bool:
        """
        هل المحركات **قادرة على الحركة فعلياً** الآن؟ (لافتة التحذير)

        🔴 الجسر في وضع `real` هو الشرط الحاسم لا النمط: في `sim` تتقدّم
        الخريطة بلا حركة، وادّعاء «المحركات مفعّلة» هناك يفقد التحذير معناه
        حين يصير حقيقياً.
        """
        if getattr(self.mission.rover, "mode", "sim") != "real":
            return False
        if self.mode == MODE_MANUAL:
            return bool(self.manual.enabled)
        if self.mode == MODE_AUTO:
            return bool(getattr(self.mission, "drive_motors", False))
        return False

    def state(self) -> dict:
        return {
            "mode": self.mode,
            "mode_ar": MODE_AR[self.mode],
            "modes": [{"id": m, "ar": MODE_AR[m]} for m in MODES],
            "camera_streaming": self.camera_streaming,
            "motors_live": self.motors_live(),
            "mission_active": self.is_mission_active(),
            "paused_by_switch": self._paused_by_switch,
            "rover_mode": getattr(self.mission.rover, "mode", "sim"),
            "reason": self.last_reason,
        }

    def _log(self, kind: str, msg: str) -> None:
        try:
            self.mission._log(kind, msg)
        except Exception:                        # noqa: BLE001
            pass
