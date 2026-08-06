# -*- coding: utf-8 -*-
"""
lora.py — وصلة راديو HC-14 (قناة الطوارئ)
==========================================
    المتصفح ──USB──> شاشة CYD ──LoRa──> HC-14 على الراسبري ──> الروبوت

مستقلة تماماً عن الإنترنت وTailscale: إن سقطت الشبكة يبقى هذا المسار.

⚠ **قناة محدودة لا امتداد للواجهة**: عرضها كيلوبتات — أوامر قصيرة
وتيليمتري مختصر فقط. لا كاميرا ولا خرائط ولا WebSocket.

## الطبقات
1. `protocol.py` — تأطير وchecksum وقائمة مغلقة ومنع إعادة إرسال (خالص).
2. **هنا** — السيريال وحده: فتح المنفذ، قراءة الأسطر، بثّ التيليمتري.
3. `control.py` — بوابة السلامة. هذه الوحدة **لا تلمس محركاً مباشرة**؛
   كل أمر يمرّ عبر `ManualControl.command(..., source="radio")`.

## 🔴 لا فشل صامت
غياب pyserial · منفذ مفقود · إذن مرفوض ⇒ `ok=False` مع `error` مقروء
يظهر في الواجهة (`/api/lora/status`)، ولا يُسقط السيرفر ولا يُخفي السبب.

## التشخيص على العتاد
`python3 pi/tests/test_lora.py` يفهم نفس التأطير — يصلح لفحص الوصلة
قبل تشغيل هذه الطبقة (وله وضع `at` لفحص وحدة واحدة).
"""
from __future__ import annotations

import threading
import time

from pi.comms.control import ManualControl, SOURCE_RADIO
from pi.comms.protocol import (
    ProtocolError, SequenceGuard, decode_command, encode_ack, encode_telemetry,
    parse_frame, ACK_BADCRC, ACK_BADSEQ, CMD_STATUS,
)
from pi.config import (
    LORA_ENABLED, LORA_PORT, LORA_BAUD, LORA_TELEMETRY_PERIOD_S,
)

# ⚠ استيراد العتاد خلف try/except مع بديل معلن (يعمل على ويندوز بلا عتاد)
try:
    import serial                                    # type: ignore
    _SERIAL_ERR = None
except ImportError as _e:                            # pragma: no cover
    serial = None
    _SERIAL_ERR = f"مكتبة pyserial غير مثبّتة ({_e}) — ثبّتها عبر setup_pi.sh"


class LoRaLink:
    """
    وصلة الراديو. آمنة الإنشاء دائماً — لا ترمي، وتُعلن حالتها في `state()`.
    """

    def __init__(self, mission, control: ManualControl,
                 port: str = LORA_PORT, baud: int = LORA_BAUD,
                 enabled: bool = LORA_ENABLED, clock=time.time):
        self.mission = mission
        self.control = control
        self.port = port
        self.baud = baud
        self.enabled = bool(enabled)
        self._clock = clock
        self.ok = False
        self.error = None
        self._ser = None
        self._thread = None
        self._stop = threading.Event()
        self.seq_guard = SequenceGuard()
        self._tx_seq = 0
        self._last_tx = 0.0
        self.frames_rx = 0
        self.frames_bad = 0
        self.commands_ok = 0
        self.last_rx_ts = None
        self.last_frame = ""
        # ضغط تكرار الرفض: قناة راديو مشوّشة تبثّ آلاف الأطر التالفة،
        # وتسجيل كلٍّ منها يدفن **أول سبب** وهو ما يُبحث عنه أصلاً.
        self._last_bad_reason = ""
        self._bad_streak = 0

    # ── دورة الحياة ─────────────────────────────────────────────
    def start(self) -> dict:
        """يفتح المنفذ ويبدأ خيط القراءة. لا يرمي — يُعلن السبب ويعود."""
        if not self.enabled:
            self.error = "الراديو معطّل (LORA_ENABLED=False)"
            return self.state()
        if serial is None:
            self.error = _SERIAL_ERR
            return self.state()
        if self._thread is not None and self._thread.is_alive():
            return self.state()
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.3)
            self.ok, self.error = True, None
        except Exception as e:                       # noqa: BLE001
            self.ok = False
            self.error = f"تعذّر فتح {self.port}: {e}"
            self._log("lora", f"⚠ {self.error}")
            return self.state()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="lora-rx")
        self._thread.start()
        self._log("lora", f"✅ وصلة الراديو مفتوحة على {self.port} @ {self.baud}")
        return self.state()

    def stop(self) -> None:
        """يوقف الخيط ويغلق المنفذ. ⚠ يوقف المحركات إن كان الراديو آمرها."""
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=2.0)
        self._thread = None
        if self.control.last_source == SOURCE_RADIO:
            self.control.command("STOP", source=SOURCE_RADIO)
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:                            # noqa: BLE001
            pass
        self._ser = None
        self.ok = False

    # ── الحلقة ──────────────────────────────────────────────────
    def _loop(self) -> None:                         # pragma: no cover — عتاد
        """
        تقرأ الأسطر وتبثّ التيليمتري. **لا تسقط بأي استثناء** — أي عطل
        يُعلَن في `error` وتتباطأ الحلقة بدل أن تموت أو تلتهم المعالج.
        """
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
                if raw:
                    self.handle_line(raw.decode("ascii", errors="replace"))
                self._maybe_send_telemetry()
            except Exception as e:                   # noqa: BLE001
                self.ok = False
                self.error = f"عطل في وصلة الراديو: {e}"
                self._log("lora", f"⚠ {self.error}")
                # ⚠ تهدئة إلزامية: بلا هذا تدور الحلقة آلاف المرات في
                #    الثانية على منفذ ميت فتلتهم المعالج (§6.4).
                self._stop.wait(1.0)

    # ── معالجة إطار واحد (عامّة كي تُختبر بلا عتاد) ──────────────
    def handle_line(self, line: str, now: float = None) -> dict:
        """
        يفكّ إطاراً واحداً وينفّذه عبر بوابة السلامة، ويردّ بإقرار.

        عامة عمداً: كل منطق الاستقبال يُختبر على ويندوز بسطر نصّي بلا
        سيريال — وهو المكان الذي **يجب** أن يُختبر فيه (قناة مكشوفة).
        """
        now = self._clock() if now is None else now
        line = (line or "").strip()
        if not line:
            return {"ok": False, "ignored": True}
        self.frames_rx += 1
        self.last_rx_ts = now
        self.last_frame = line[:64]

        # ١) التأطير و الـchecksum
        try:
            payload = parse_frame(line)
        except ProtocolError as e:
            return self._bad(ACK_BADCRC, str(e), seq=0)

        # ٢) فكّ الأمر (القائمة المغلقة تُفرض هنا)
        try:
            cmd = decode_command(payload)
        except ProtocolError as e:
            return self._bad(ACK_BADCRC, str(e), seq=0)

        # ٣) 🔴 منع إعادة الإرسال
        if not self.seq_guard.check(cmd.seq, now):
            return self._bad(ACK_BADSEQ, self.seq_guard.last_reason, seq=cmd.seq)
        if self.seq_guard.last_reason:
            # إعادة مزامنة أو أول إطار — تُعلَن ولا تمرّ صامتة
            self._log("lora_seq", self.seq_guard.last_reason)

        # ٤) البوابة (طبقة السلامة فوق أمر الراديو)
        res = self.control.command(cmd.cmd, power=(cmd.p1 or None),
                                   source=SOURCE_RADIO, now=now)
        self._bad_streak = 0
        self._last_bad_reason = ""
        if res.get("ok"):
            self.commands_ok += 1
        self._send(encode_ack(cmd.seq, res.get("ack", "OK")))
        if cmd.cmd == CMD_STATUS:
            self._send_telemetry(now)
        return res

    def _bad(self, ack: str, reason: str, seq: int = 0) -> dict:
        """يسجّل رفضاً **مضغوطاً** ويردّ به على وحدة التحكم."""
        self.frames_bad += 1
        if reason == self._last_bad_reason:
            self._bad_streak += 1
            if self._bad_streak in (10, 100, 1000):
                self._log("lora_reject",
                          f"⚠ تكرر الرفض ×{self._bad_streak}: {reason}")
        else:
            self._last_bad_reason = reason
            self._bad_streak = 1
            self._log("lora_reject", f"⚠ إطار راديو مرفوض — {reason}")
        self._send(encode_ack(seq, ack))
        return {"ok": False, "ack": ack, "reason": reason}

    # ── الإرسال ─────────────────────────────────────────────────
    def _send(self, frame: str) -> None:
        """إرسال لا يرمي أبداً (§6.3 — الاستثناء يقتل الإيقاف المضمون)."""
        if self._ser is None:
            return
        try:
            self._ser.write(frame.encode("ascii", errors="replace"))
        except Exception as e:                       # noqa: BLE001
            self.ok = False
            self.error = f"تعذّر الإرسال: {e}"

    def _maybe_send_telemetry(self, now: float = None) -> None:
        now = self._clock() if now is None else now
        if (now - self._last_tx) >= LORA_TELEMETRY_PERIOD_S:
            self._send_telemetry(now)

    def _send_telemetry(self, now: float = None) -> str:
        now = self._clock() if now is None else now
        self._last_tx = now
        self._tx_seq += 1
        t = self.control.telemetry()
        frame = encode_telemetry(self._tx_seq, t["cpm"], t["volts"],
                                 _short_state(t["state"]))
        self._send(frame)
        return frame

    def _log(self, kind: str, msg: str) -> None:
        try:
            self.mission._log(kind, msg)
        except Exception:                            # noqa: BLE001
            pass

    def state(self) -> dict:
        """حالة الوصلة — تُعرض في الواجهة وتفرّق بين أسباب التعطّل."""
        return {
            "enabled": self.enabled, "ok": self.ok, "error": self.error,
            "port": self.port, "baud": self.baud,
            "frames_rx": self.frames_rx, "frames_bad": self.frames_bad,
            "commands_ok": self.commands_ok,
            "last_frame": self.last_frame,
            "last_rx_ts": self.last_rx_ts,
            "seq": self.seq_guard.state(),
        }


def _short_state(state: str) -> str:
    """حالة المهمة برمز قصير (القناة ضيقة والشاشة صغيرة)."""
    return {"idle": "IDLE", "running": "RUN", "paused": "PAUSE",
            "done": "DONE", "returning": "RTH", "estop": "ESTOP"
            }.get(str(state), str(state)[:5].upper())
