# -*- coding: utf-8 -*-
"""
control.py — مسار الأوامر اليدوية والراديو (بوابة واحدة)
=========================================================
🔴 **بوابة واحدة لا مسارين**: القيادة اليدوية من الواجهة وأوامر الراديو
تمرّان من هنا معاً. الغرض ليس تنظيماً بل سلامة — كل مسار ثانٍ يقود
المحركات هو حارس سلامة يجب صيانته مرتين، وواحد منهما سيُنسى.

⚠ **ما كان قائماً قبل هذا الملف**: `ws/sensors` في السيرفر كان يقود
`RoverBridge(mode="sim")` — **نسخة جسر ثانية غير جسر المهمة** — بلا أي
فحص حساسات ولا heartbeat ولا حدّ قوة. هذه الوحدة تُلغي ذلك المسار.

────────────────────────────────────────────────────────────────────
## طبقة السلامة تبقى **فوق** الأمر لا تحته

المستخدم البعيد لا يرى ما أمام الروبوت. فالأمر يمرّ ببوابة تستعمل
**كائنات المهمة نفسها** — `mission.sensors()` و`mission.reactive` — لا
نسخاً موازية منها:

| الأمر | البوابة |
|---|---|
| `ESTOP` | **لا يُرفض أبداً** — يفشل إلى الأمان |
| `STOP` · `STATUS` | لا حركة، لا بوابة |
| `FWD` | 🔴 تُرفض عند عائق أمامي، وتُقصّ سرعتها بسلّم المسافة |
| `BACK` · `LEFT` · `RIGHT` | مسموحة — هي **طريق الخروج** من الانحشار |
| `RTH` | عبر `mission.return_home()` (مسار المهمة القائم) |
| `WDRAW` | عبر `mission.request_withdraw()` — **مهمة نشطة فقط** (انظر أدناه) |

⚠ **لماذا يُشترط لـ`WDRAW` مهمة نشطة؟** طلب الانسحاب علمٌ معلّق تخدمه
حلقة التنفيذ. قبوله والمهمة idle يتركه كامناً حتى **المهمة التالية**
فتنسحب فور بدئها بلا أمر أحد — رفض واضح الآن خير من مفاجأة لاحقاً.

⚠ **لماذا `BACK` بلا بوابة؟** لا حسّاس خلفيّ على هذا العتاد. رفضها يسدّ
المخرج الوحيد من عائق أمامي؛ والسماح بها يجعل الخلف **مسؤولية المشغّل**
صراحةً. القيد معلَن في الردّ (`rear_unknown`) لا مسكوت عنه.

## مهلة الأوامر (heartbeat)
انقطاع الأوامر ⇒ إيقاف المحركات: `MANUAL_HEARTBEAT_S` (0.8ث) للواجهة
و`LORA_COMMAND_TIMEOUT_S` (2.0ث) للراديو — الراديو أبطأ وأكثر فقداً.
الحارس يُنفَّذ في `poll()` التي يناديها السيرفر من حلقة البثّ.
"""
from __future__ import annotations

import time

from pi.comms.protocol import (
    CMD_FWD, CMD_BACK, CMD_LEFT, CMD_RIGHT, CMD_STOP, CMD_ESTOP,
    CMD_STATUS, CMD_RTH, CMD_WITHDRAW, RADIO_COMMANDS, MOTION_COMMANDS,
    ACK_OK, ACK_BADCMD, ACK_SAFE, ACK_BUSY, ACK_FAULT,
)
from pi.config import (
    MANUAL_POWER_MIN, MANUAL_POWER_MAX, MANUAL_POWER_DEFAULT,
    MANUAL_HEARTBEAT_S, LORA_COMMAND_TIMEOUT_S,
)

SOURCE_MANUAL = "manual"
SOURCE_RADIO = "radio"

#: قرارات الطبقة التفاعلية التي تعني «عائق أمامي» ⇒ تمنع التقدّم وحده.
_BLOCKING_ACTIONS = frozenset({"stop", "backup_turn", "turn_left", "turn_right"})


class ManualControl:
    """
    بوابة الأوامر. لا تملك حساسات ولا جسراً — **تستعير كائنات المهمة**،
    فتبقى طبقة السلامة واحدة ومصدر الحقيقة واحداً.
    """

    def __init__(self, mission, clock=time.time):
        self.mission = mission
        self._clock = clock
        self.enabled = False          # يُرفع عند دخول نمط القيادة اليدوية
        self.last_source = None
        self.last_cmd = None
        self.last_reason = ""
        self.last_ts = None
        self._engaged = False         # محركات تعمل الآن بأمر يدوي/راديو
        self.stops_by_timeout = 0
        self.blocked_count = 0
        self.rejected_count = 0
        # 🔴 تجاوز حساسات العوائق — **قيادة يدوية فقط وبطلب صريح**: المشغّل
        #    يقود بالكاميرا/بعينه ويتحمّل المسؤولية. heartbeat وحدّ القوة
        #    وESTOP لا يتأثرون. يُصفَّر تلقائياً عند إغلاق القيادة اليدوية.
        self.ignore_sensors = False
        self.overridden_count = 0

    # ── التفعيل (حصرية الأنماط) ─────────────────────────────────
    def set_enabled(self, on: bool, source: str = SOURCE_MANUAL) -> dict:
        """
        يفتح/يغلق القيادة اليدوية. الإغلاق **يوقف المحركات دائماً** — لا
        نترك عجلة تدور لأن المستخدم بدّل تبويباً.
        """
        on = bool(on)
        if not on and self._engaged:
            self._hard_stop("أُغلقت القيادة اليدوية")
        if not on and self.ignore_sensors:
            # ⚠ التجاوز لا يعيش خارج جلسة اليدوية — لا نورّثه لوضع آخر
            self.ignore_sensors = False
            self._log("manual_mode", "أُعيد تفعيل حساسات العوائق (تصفير تلقائي)")
        self.enabled = on
        self._log("manual_mode",
                  "قيادة يدوية مفعّلة ⚠ المحركات تستجيب" if on
                  else "قيادة يدوية مغلقة")
        return self.state()

    # ── البوابة ─────────────────────────────────────────────────
    def command(self, cmd: str, power=None, source: str = SOURCE_MANUAL,
                now: float = None) -> dict:
        """
        ينفّذ أمراً واحداً من القائمة المغلقة عبر بوابة السلامة.

        يُعيد دائماً قاموساً فيه `ok` و`ack` و`reason` — **لا استثناء
        يعبر مسار الحركة** (قاعدة §6.3: الاستثناء يقتل الإيقاف المضمون).
        """
        now = self._clock() if now is None else now
        cmd = str(cmd or "").strip().upper()
        self.last_source = source

        if cmd not in RADIO_COMMANDS:
            self.rejected_count += 1
            return self._reject(cmd, ACK_BADCMD,
                                f"أمر خارج القائمة المغلقة: {cmd!r}", source)

        # ── ESTOP: لا بوابة ولا شرط — يفشل إلى الأمان ─────────────
        if cmd == CMD_ESTOP:
            self.mission.estop()
            self._engaged = False
            self.last_cmd, self.last_ts = cmd, now
            self._log("estop", f"⛔ إيقاف طوارئ ({_ar_source(source)})")
            return self._ok(cmd, ACK_OK, "إيقاف طوارئ — أُوقفت المحركات", source)

        if cmd == CMD_STOP:
            self._safe_stop()
            self._engaged = False
            self.last_cmd, self.last_ts = cmd, now
            return self._ok(cmd, ACK_OK, "توقف", source)

        if cmd == CMD_STATUS:
            self.last_cmd, self.last_ts = cmd, now
            return self._ok(cmd, ACK_OK, "استعلام حالة", source,
                            extra={"telemetry": self.telemetry()})

        if cmd == CMD_RTH:
            self.mission.return_home()
            self.last_cmd, self.last_ts = cmd, now
            self._log("return_home", f"عودة لنقطة الانطلاق ({_ar_source(source)})")
            return self._ok(cmd, ACK_OK, "عودة لنقطة الانطلاق", source)

        # ── WDRAW: انسحاب على أثر الدخول — أولوية مطلقة في المهمة ──
        if cmd == CMD_WITHDRAW:
            self.last_cmd, self.last_ts = cmd, now
            # مهمة نشطة فقط: العلم المعلّق على idle يفاجئ المهمة التالية
            if self.mission.state not in ("running", "paused", "returning"):
                self.rejected_count += 1
                return self._reject(
                    cmd, ACK_BUSY,
                    f"لا مهمة نشطة (الحالة: {self.mission.state}) — "
                    "الانسحاب يخصّ مسحاً ذاتياً جارياً", source)
            try:
                res = self.mission.request_withdraw(
                    reason=f"أمر انسحاب ({_ar_source(source)})")
            except Exception as e:              # noqa: BLE001 — §6.3
                return self._reject(cmd, ACK_FAULT,
                                    f"تعذّر طلب الانسحاب: {e}", source)
            if res.get("ok"):
                return self._ok(cmd, ACK_OK,
                                "طلب انسحاب — أولوية مطلقة على أي هدف مسح",
                                source)
            # المهمة رفضته (مثلاً: بلا قيادة محركات) — السبب يصل كما هو
            self.rejected_count += 1
            return self._reject(cmd, ACK_FAULT,
                                res.get("error", "رفضت المهمة طلب الانسحاب"),
                                source)

        # ── من هنا: أوامر حركة فقط ────────────────────────────────
        # 🔴 حصرية الأنماط: لا قيادة يدوية فوق مسح ذاتي جارٍ. لو سُمح
        #    لتنازع خيطُ المحركات والأمرُ اليدوي على نفس المنفذ.
        if not self.enabled:
            self.rejected_count += 1
            return self._reject(cmd, ACK_BUSY,
                                "القيادة اليدوية غير مفعّلة — بدّل إلى نمط "
                                "«قيادة يدوية» أولاً", source)
        # حارس ثانٍ مستقل عن الواجهة: خيط محركات المسح الذاتي حيّ ⇒ رفض.
        # ⚠ لا يكفي أن يكون `enabled` صحيحاً — لو تسرّب أمر يدوي بينما
        #   الخيط يقود لتنازعا على نفس منفذ السيريال وتداخلت الأوامر.
        worker = getattr(self.mission, "_worker", None)
        if worker is not None and worker.is_alive():
            self.rejected_count += 1
            return self._reject(cmd, ACK_BUSY,
                                "مهمة ذاتية جارية — أوقفها قبل القيادة اليدوية",
                                source)

        # عطل وصلة الروفر: لا معنى لأمر حركة على منفذ ميت
        rover = self.mission.rover
        if getattr(rover, "link_ok", True) is False:
            self.rejected_count += 1
            return self._reject(cmd, ACK_FAULT,
                                f"وصلة الروفر معطّلة: {getattr(rover, 'link_error', '')}",
                                source)

        pwr = self._clamp_power(power)
        gate = self._safety_gate(cmd)
        if not gate["allow"] and self.ignore_sensors:
            # 🔴 تجاوز مقصود بطلب المشغّل: البوابة **تُحتسب وتُسجَّل** لكنها
            #    لا تمنع — والسبب الأصلي يبقى في الردّ مسبوقاً بالتحذير حتى
            #    تعرف الواجهة ماذا كان سيُمنع. heartbeat وحدّ القوة باقيان.
            self.overridden_count += 1
            self._log("safety_override",
                      f"⚠ تجاوز الحساسات: نُفّذ {cmd} رغم «{gate['reason']}»")
            gate = dict(gate, allow=True, overridden=True,
                        reason="⚠ تجاوز مفعّل — " + str(gate.get("reason", "")))
        if not gate["allow"]:
            self.blocked_count += 1
            self._safe_stop()
            self._engaged = False
            self.last_cmd, self.last_ts = cmd, now
            self._log("safety_block",
                      f"⛔ رُفض {cmd} ({_ar_source(source)}) — {gate['reason']}")
            return self._reject(cmd, ACK_SAFE, gate["reason"], source,
                                extra={"safety": gate})

        # قصّ السرعة بسلّم المسافة (التقدّم وحده يتأثر — والتجاوز يعطّله أيضاً)
        if (gate.get("speed_cap") is not None and cmd == CMD_FWD
                and not gate.get("overridden")):
            pwr = min(pwr, float(gate["speed_cap"]))
            pwr = max(pwr, MANUAL_POWER_MIN)

        try:
            if cmd == CMD_FWD:
                rover.forward(pwr)
            elif cmd == CMD_BACK:
                rover.backward(pwr)
            elif cmd == CMD_LEFT:
                rover.turn("L", pwr)
            elif cmd == CMD_RIGHT:
                rover.turn("R", pwr)
        except Exception as e:              # noqa: BLE001 — §6.3
            self._safe_stop()
            self._engaged = False
            return self._reject(cmd, ACK_FAULT, f"تعذّر إرسال الأمر: {e}", source)

        self._engaged = True
        self.last_cmd, self.last_ts = cmd, now
        return self._ok(cmd, ACK_OK, gate["reason"], source,
                        extra={"power": round(pwr, 3), "safety": gate})

    # ── حارس المهلة (يناديه السيرفر من حلقة البثّ) ───────────────
    def poll(self, now: float = None) -> dict | None:
        """
        يوقف المحركات إذا انقطعت الأوامر. يُعيد وصفاً عند التنفيذ وإلا None.

        ⚠ يعمل **حتى لو أُغلقت القيادة اليدوية** ما دام `_engaged` — الحالة
        الخطرة هي بالضبط أن يختفي الآمر ويبقى الأمر سارياً.
        """
        if not self._engaged or self.last_ts is None:
            return None
        now = self._clock() if now is None else now
        limit = (LORA_COMMAND_TIMEOUT_S if self.last_source == SOURCE_RADIO
                 else MANUAL_HEARTBEAT_S)
        if (now - self.last_ts) <= limit:
            return None
        self.stops_by_timeout += 1
        self._hard_stop(f"انقطاع الأوامر > {limit:.1f}ث ({_ar_source(self.last_source)})")
        return {"stopped": True, "reason": self.last_reason, "limit_s": limit}

    # ── طبقة السلامة ────────────────────────────────────────────
    def _safety_gate(self, cmd: str) -> dict:
        """
        يستشير **كائنات المهمة نفسها**: قراءاتها وطبقتها التفاعلية.

        🔴 `None` = مجهول لا «خالٍ»: بلا استشعار أمامي لا يُدَّعى أن الطريق
        سالك — تُقصّ السرعة إلى درجة «لا قراءة» ويُذكر المجهول في الردّ
        فيظهر في الواجهة. (لا نرفض التقدّم كلياً: المشغّل يقود بالكاميرا،
        والرفض التام يجعل قناة الطوارئ عديمة الفائدة وقت الحاجة إليها.)
        """
        if cmd != CMD_FWD:
            # الرجوع واللفّ مخرج الانحشار — لا تُسدّ
            return {"allow": True, "reason": f"{cmd} مسموح (مخرج الانحشار)",
                    "rear_unknown": cmd == CMD_BACK,
                    "note": ("⚠ لا حسّاس خلفيّ — الخلف مسؤولية المشغّل"
                             if cmd == CMD_BACK else "")}
        try:
            s = self.mission.sensors()
            dec = self.mission.reactive.decide(
                s.get("ultrasonic_cm"), s.get("ir_left"), s.get("ir_right"),
                s.get("ir_mid"))
        except Exception as e:                  # noqa: BLE001
            # تعذّرت قراءة الحساسات ⇒ **مجهول**، وأخطر ما يكون مع التقدّم
            return {"allow": False,
                    "reason": f"تعذّرت قراءة الحساسات ({e}) — لا تقدّم على مجهول"}

        self.mission.last_reactive = dec        # تبقى الواجهة ترى آخر قرار
        if dec.get("action") in _BLOCKING_ACTIONS:
            return {"allow": False, "reason": dec.get("reason", "عائق أمامي"),
                    "action": dec.get("action"), "priority": dec.get("priority"),
                    "unknown": dec.get("unknown", [])}
        return {"allow": True, "reason": dec.get("reason", ""),
                "speed_cap": dec.get("speed"), "rung": dec.get("rung"),
                "unknown": dec.get("unknown", [])}

    # ── مساعدات ─────────────────────────────────────────────────
    @staticmethod
    def _clamp_power(power) -> float:
        """
        🔴 القصّ إلى [MIN, MAX] — والسقف هو `MAX_MOTOR_POWER` نفسه: أي قيمة
        فوقه يلتفّ عليها فيرموير Wave Rover **صامتاً** (0.6→0.1).
        """
        try:
            p = float(MANUAL_POWER_DEFAULT if power is None else power)
        except (TypeError, ValueError):
            p = MANUAL_POWER_DEFAULT
        return max(MANUAL_POWER_MIN, min(MANUAL_POWER_MAX, p))

    def _safe_stop(self) -> None:
        """إيقاف لا يرفع استثناءً أبداً (§6.3)."""
        try:
            self.mission.rover.stop()
        except Exception:                       # noqa: BLE001
            pass

    def _hard_stop(self, reason: str) -> None:
        self._safe_stop()
        self._engaged = False
        self.last_reason = reason
        self._log("manual_timeout", f"⛔ أُوقفت المحركات — {reason}")

    def _log(self, kind: str, msg: str) -> None:
        try:
            self.mission._log(kind, msg)
        except Exception:                       # noqa: BLE001
            pass

    def _ok(self, cmd, ack, reason, source, extra=None) -> dict:
        self.last_reason = reason
        out = {"ok": True, "ack": ack, "cmd": cmd, "reason": reason,
               "source": source}
        if extra:
            out.update(extra)
        return out

    def _reject(self, cmd, ack, reason, source, extra=None) -> dict:
        self.last_reason = reason
        out = {"ok": False, "ack": ack, "cmd": cmd, "reason": reason,
               "source": source}
        if extra:
            out.update(extra)
        return out

    # ── تيليمتري مختصر (يُبثّ على الراديو ويُعرض في الواجهة) ─────
    def telemetry(self) -> dict:
        m = self.mission
        try:
            cpm = float(m.last_reading.get("cpm", 0.0))
        except Exception:                       # noqa: BLE001
            cpm = 0.0
        try:
            volts = m.rover.voltage()
        except Exception:                       # noqa: BLE001
            volts = None
        return {"cpm": cpm, "volts": volts, "state": m.state}

    def state(self) -> dict:
        return {"enabled": self.enabled, "engaged": self._engaged,
                "last_cmd": self.last_cmd, "last_source": self.last_source,
                "last_reason": self.last_reason,
                "stops_by_timeout": self.stops_by_timeout,
                "blocked": self.blocked_count, "rejected": self.rejected_count,
                "ignore_sensors": self.ignore_sensors,
                "overridden": self.overridden_count,
                "power_min": MANUAL_POWER_MIN, "power_max": MANUAL_POWER_MAX,
                "heartbeat_s": MANUAL_HEARTBEAT_S,
                "radio_timeout_s": LORA_COMMAND_TIMEOUT_S}


def _ar_source(source: str) -> str:
    return {SOURCE_RADIO: "راديو", SOURCE_MANUAL: "يدوي"}.get(source, str(source))
