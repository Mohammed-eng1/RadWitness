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
from collections import deque

from pi.comms.control import ManualControl, SOURCE_RADIO
from pi.comms.protocol import (
    ProtocolError, SequenceGuard, decode_command, encode_ack, encode_telemetry,
    parse_frame, ACK_BADCRC, ACK_BADCMD, ACK_BADSEQ, ACK_OK,
    CMD_STATUS, CMD_STOP, CMD_ESTOP,
)
from pi.config import (
    LORA_ENABLED, LORA_PORT, LORA_BAUD, LORA_TELEMETRY_PERIOD_S,
)

# ⚠ هذه الثوابت **مكانها `pi/config.py`** (قاعدة: لا أرقام مبعثرة) — وهي
#   هنا مؤقتاً لأن ذلك الملف خارج نطاق هذه الجلسة (شجرة مشتركة). تُنقل
#   بطلب المستخدم. القيم مشتقة من قياس هذا الأسبوع لا من تخمين:
#   الطرفان يبثّان كل 2ث ⇒ نجا ثلث الأطر ⇒ إشغال القناة ~0.67ث لكل إطار
#   عند S3، أي سعة عملية ~1.5 إطار/ثانية. وكنا نُحمّلها ~4.5 إطار/ثانية.
LORA_SERIAL_TIMEOUT_S = 0.05    # نافذة القراءة: كانت 0.3 فتؤخّر الإقرار
LORA_ACK_MIN_GAP_S = 1.5        # لا إقرار OK مكرَّر أسرع من ذلك
LORA_TELEM_HOLDOFF_S = 1.0      # اصمت بعد أمر وارد: نصف ازدواج
LORA_TELEM_MAX_SILENCE_S = 3.0  # 🔴 لكن لا تصمت أكثر — الشاشة تُعلن الوصلة
                                #    ميتة بعد 8ث بلا إطار وارد أياً كان نوعه

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
        # 🔴 كشف الصدى المحلي (مقاس 2026-08-06): المحوّل/الوحدة قد يعيد
        #    كل ما نرسله. بلا هذا الكشف كان **الرد على الصدى يولّد صدى
        #    الردّ**: تيليمتري ← صدى ← إقرار رفض ← صدى الإقرار ← إقرار…
        #    2473 إطاراً في دقائق، كلها مرفوضة، والقناة الهوائية مشوَّشة.
        self._tx_recent = deque(maxlen=32)   # (إطار مُرسَل، وقت إرساله)
        self.echoes = 0
        # ── ضبط الهواء: القناة نصف مزدوجة وسعتها ~1.5 إطار/ث ──────
        self._last_rx_cmd_ts = 0.0    # آخر أمر وارد — نصمت بعده لحظة
        self._last_ack_code = None    # لا نكرّر إقرار OK نفسه بلا داعٍ
        self._last_ack_ts = 0.0
        self.acks_suppressed = 0      # يُعرض: كم إطاراً وفّرناه من الهواء
        self.telem_held = 0

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
            # ⚠ نافذة القراءة قصيرة عمداً: `read(64)` يحجب حتى تمتلئ 64 بايتاً
            #    أو تنتهي المهلة، وإطارنا 20 بايتاً فلا يملؤها أبداً ⇒ كل
            #    قراءة كانت تحرق بقية الـ0.3ث قبل أن يُبنى الإقرار.
            self._ser = serial.Serial(self.port, self.baud,
                                      timeout=LORA_SERIAL_TIMEOUT_S)
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

        ⚠ **تجميع يدوي حتى `\\n` — لا `readline` بمهلة**: LoRa بطيء المعدل
        الهوائي (وضع S3) والوحدة تسلّم الإطار على دفعات؛ `readline` بمهلة
        0.3ث يقطع الإطار نصفين عند أي فجوة، فيفشل كل نصف في الـchecksum
        ويبدو ضجيجاً — بينما الإطار سليم. (فيرموير الشاشة محصّن بنفس
        الأسلوب في `pump()` — هذه مرآته.)
        """
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(64)           # المهلة 0.3ث تحكم الدورة
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self.handle_line(line.decode("ascii", errors="replace"))
                    if len(buf) > 512:
                        # ضجيج متدفق بلا نهاية سطر — أفرغه ولا تكدّسه
                        buf = b""
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

        # ٠) 🔴 صدى محلي: نفس ما أرسلناه للتوّ يعود إلينا (المحوّل/الوحدة).
        #    يُسقَط **قبل أي معالجة وبلا أي ردّ** — الرد على الصدى يولّد
        #    صدى الردّ فحلقة لا تنتهي (قِيست: 2473 إطاراً في دقائق).
        for sent, ts in self._tx_recent:
            if line == sent and (now - ts) <= 3.0:
                self.echoes += 1
                if self.echoes in (1, 100, 10000):
                    self._log("lora_echo",
                              f"⚠ صدى محلي ×{self.echoes}: العتاد يعيد ما "
                              "نرسله — ليس استقبالاً لاسلكياً")
                return {"ok": False, "echo": True}

        self.frames_rx += 1
        self.last_rx_ts = now
        self.last_frame = line[:64]

        # ١) التأطير و الـchecksum — 🔴 **بلا إقرار**: إطار لا نثق حتى
        #    بتسلسله لا يستحق ردّاً، والرد على الضجيج في قناة فيها صدى
        #    وقودُ الحلقة أعلاه.
        try:
            payload = parse_frame(line)
        except ProtocolError as e:
            return self._bad(ACK_BADCRC, str(e), seq=0, send_ack=False)

        # ١.٥) 🔴 ليست حمولة أمر أصلاً (تيليمتري/إقرار من طرف آخر — أو
        #    صدى فات نافذة الكشف): تُسقَط صامتة معدودة. **لا إقرار أبداً**:
        #    الإقرار على `T`/`A` هو بالضبط ما يجعل الصدى حلقة أبدية.
        if not payload.startswith("C,"):
            return self._bad(ACK_BADCRC,
                             f"حمولة ليست أمراً ({payload[:12]!r}) — أُسقطت بلا ردّ",
                             seq=0, send_ack=False)

        # ٢) فكّ الأمر (القائمة المغلقة تُفرض هنا) — الإطار سليم التأطير
        #    وبادئته `C,` ⇒ مرسله وحدة تحكم فعلية تستحق ردّاً بالرفض.
        try:
            cmd = decode_command(payload)
        except ProtocolError as e:
            return self._bad(ACK_BADCMD, str(e), seq=0)

        # ٣) 🔴 منع إعادة الإرسال
        if not self.seq_guard.check(cmd.seq, now):
            return self._bad(ACK_BADSEQ, self.seq_guard.last_reason, seq=cmd.seq)
        if self.seq_guard.last_reason:
            # إعادة مزامنة أو أول إطار — تُعلَن ولا تمرّ صامتة
            self._log("lora_seq", self.seq_guard.last_reason)

        # ٤) البوابة (طبقة السلامة فوق أمر الراديو)
        self._last_rx_cmd_ts = now          # ابدأ نافذة صمت التيليمتري
        res = self.control.command(cmd.cmd, power=(cmd.p1 or None),
                                   source=SOURCE_RADIO, now=now)
        self._bad_streak = 0
        self._last_bad_reason = ""
        if res.get("ok"):
            self.commands_ok += 1
        self._send_ack(cmd, res.get("ack", "OK"), now)
        if cmd.cmd == CMD_STATUS:
            self._send_telemetry(now)
        return res

    def _send_ack(self, cmd, code: str, now: float) -> None:
        """
        🔴 إقرار **لكل ما يهمّ**، وصمت عن التكرار العقيم.

        الزرّ المضغوط يعيد أمره مرات في الثانية، وكل إقرار OK مطابق يسرق
        من الهواء ما تحتاجه ضغطة الطوارئ التالية. فلا يُكتم إلا إقرار
        **OK مكرَّر لأمر غير طارئ** داخل نافذة قصيرة. ويبقى مضموناً:
          - ESTOP و STOP: يُقرّان دائماً (المشغّل ينتظر تأكيد التوقف).
          - أي رمز غير OK (SAFE · BUSY · FAULT · BADSEQ): رفضٌ لا يُبتلع.
          - أي **تغيّر** في الرمز: أول SAFE بعد سلسلة OK يصل فوراً.
        """
        always = (cmd.cmd in (CMD_ESTOP, CMD_STOP) or code != ACK_OK
                  or code != self._last_ack_code)
        if not always and (now - self._last_ack_ts) < LORA_ACK_MIN_GAP_S:
            self.acks_suppressed += 1
            return
        self._last_ack_code, self._last_ack_ts = code, now
        self._send(encode_ack(cmd.seq, code))

    def _bad(self, ack: str, reason: str, seq: int = 0,
             send_ack: bool = True) -> dict:
        """
        يسجّل رفضاً **مضغوطاً**، ويردّ بإقرار **لأطر الأوامر وحدها**.

        🔴 `send_ack=False` لكل ما ليس أمر `C,` سليم التأطير: الرد على
        ضجيج/تيليمتري/إقرارات في قناة فيها صدى يولّد حلقة ذاتية التغذية
        (قِيست). الإقرار حقّ وحدة تحكم حقيقية تنتظر نتيجة أمرها — فقط.
        """
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
        if send_ack:
            self._send(encode_ack(seq, ack))
        return {"ok": False, "ack": ack, "reason": reason}

    # ── الإرسال ─────────────────────────────────────────────────
    def _send(self, frame: str) -> None:
        """إرسال لا يرمي أبداً (§6.3 — الاستثناء يقتل الإيقاف المضمون)."""
        # يُسجَّل قبل الكتابة كي يُكشف صداه حتى لو عاد في نفس الدورة
        self._tx_recent.append((frame.strip(), self._clock()))
        if self._ser is None:
            return
        try:
            self._ser.write(frame.encode("ascii", errors="replace"))
        except Exception as e:                       # noqa: BLE001
            self.ok = False
            self.error = f"تعذّر الإرسال: {e}"

    def _maybe_send_telemetry(self, now: float = None) -> None:
        """
        🔴 التيليمتري يتنحّى للأوامر — بأرضية صلبة تمنع كذبة «لا وصلة».

        القناة نصف مزدوجة: بثّ الروبوت يُصمّه عن ضغطة المشغّل. فبعد كل أمر
        وارد نصمت لحظة كي يمرّ الأمر التالي وإقراره بلا تصادم.

        ⚠ والأرضية إلزامية لا تحسين: الشاشة تُعلن الوصلة ميتة بعد 8ث بلا
        **أي** إطار وارد، وكتم الإقرارات المكرَّرة يجري بالتوازي — فلولا
        هذا السقف لأنتج ضغطٌ مستمرّ لعشر ثوانٍ شاشةَ «NO LINK» والروبوت
        يسير تحت أمر المشغّل نفسه. (تعارضٌ لا يظهر إلا بجمع التحسينين.)
        """
        now = self._clock() if now is None else now
        if (now - self._last_tx) < LORA_TELEMETRY_PERIOD_S:
            return
        busy = (now - self._last_rx_cmd_ts) < LORA_TELEM_HOLDOFF_S
        if busy and (now - self._last_tx) < LORA_TELEM_MAX_SILENCE_S:
            self.telem_held += 1
            return
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
            "echoes": self.echoes,
            # سبب آخر رفض — رقم «مرفوضة» العاري بلا سببه نصف معلومة
            "last_reject": self._last_bad_reason,
            "reject_streak": self._bad_streak,
            # ضبط الهواء: كم إطاراً وُفّر (تشخيص ازدحام القناة)
            "acks_suppressed": self.acks_suppressed,
            "telem_held": self.telem_held,
            "last_frame": self.last_frame,
            "last_rx_ts": self.last_rx_ts,
            "seq": self.seq_guard.state(),
        }


def _short_state(state: str) -> str:
    """حالة المهمة برمز قصير (القناة ضيقة والشاشة صغيرة)."""
    return {"idle": "IDLE", "running": "RUN", "paused": "PAUSE",
            "done": "DONE", "returning": "RTH", "estop": "ESTOP"
            }.get(str(state), str(state)[:5].upper())
