# -*- coding: utf-8 -*-
"""
protocol.py — بروتوكول الراديو (منطق خالص بلا عتاد)
====================================================
كل ما في هذا الملف قابل للاختبار على ويندوز: لا سيريال ولا GPIO ولا زمن
حقيقي إلا ما يُحقن صراحةً (`now`). هذا متعمّد — منطق **قناة مكشوفة** يجب
أن يُختبر بكل حالاته قبل أن يقود محركات.

────────────────────────────────────────────────────────────────────
## التأطير — نفس صيغة `pi/tests/test_lora.py` المجرَّبة على العتاد

    $<payload>*<HH>\\n

`<HH>` بايتان hex = XOR لكل بايتات payload (طراز NMEA). لم تُخترع صيغة
جديدة عمداً: التأطير القائم مُختبر على وصلة حقيقية، وأداة التشخيص
`test_lora.py` تفهمه أصلاً — فتبقى صالحة لتشخيص هذه الطبقة.

⚠ **لا JSON**: مسرف على قناة عرضها كيلوبتات.

## الرسائل الثلاث

| البادئة | الاتجاه | الحمولة |
|---|---|---|
| `C` | وحدة التحكم → الروبوت | `C,<seq>,<cmd>,<p1>,<p2>` |
| `T` | الروبوت → وحدة التحكم | `T,<seq>,<cpm>,<mv>,<state>` |
| `A` | الروبوت → وحدة التحكم | `A,<seq>,<code>` (إقرار/رفض) |

⚠ الجهد بالـ**ميلي فولت صحيحة** لا بالفولت العشرية: يتجنّب تحليل الأعداد
العشرية على ESP32 واختلاف الفاصلة العشرية بين اللغات.

## 🔴 قائمة الأوامر **مغلقة**

`FWD · BACK · LEFT · RIGHT · STOP · ESTOP · STATUS · RTH · WDRAW` — لا
أوامر مركّبة ولا تنفيذ نصوص حرّة. أي شيء خارجها يُرفض **ويُسجَّل بسببه**.
(`WDRAW` = طلب انسحاب: أولوية مطلقة في المهمة، يسير على أثر الدخول عكسياً.)

## 🔴 منع إعادة الإرسال

القناة مكشوفة: من يسمع إطار `FWD` يستطيع بثّه ثانيةً. الحماية تسلسل
متزايد داخل نافذة أمامية — المكرَّر (فجوة صفر) والقديم يُرفضان. وحدود
هذه الحماية موصوفة عند `SequenceGuard` بلا تجميل.
"""
from __future__ import annotations

from dataclasses import dataclass

from pi.config import (
    LORA_MAX_PAYLOAD, LORA_SEQ_MODULO, LORA_SEQ_WINDOW, LORA_SEQ_RESYNC_S,
)

# ═══ التأطير ══════════════════════════════════════════════════════
FRAME_START = "$"
FRAME_SEP = "*"


class ProtocolError(ValueError):
    """إطار/حمولة غير صالحة — الرسالة عربية لأنها تُعرض وتُسجَّل كما هي."""


def xor_checksum(payload: str) -> int:
    """XOR لكل بايتات الحمولة (ASCII) — طراز NMEA."""
    c = 0
    for b in payload.encode("ascii", errors="replace"):
        c ^= b
    return c


def build_frame(payload: str) -> str:
    """يبني `$<payload>*<HH>\\n` مع فرض حدّ الطول."""
    if len(payload) > LORA_MAX_PAYLOAD:
        raise ProtocolError(
            f"الحمولة {len(payload)} بايت تتجاوز الحدّ {LORA_MAX_PAYLOAD}")
    return f"{FRAME_START}{payload}{FRAME_SEP}{xor_checksum(payload):02X}\n"


def parse_frame(line: str) -> str:
    """
    يفكّ إطاراً ويتحقق من الـchecksum. يُعيد الحمولة أو يرفع `ProtocolError`.

    ⚠ **الرفع لا الإرجاع الصامت لـNone**: سبب الرفض هو ما يُبحث عنه عند
    تشخيص وصلة راديو رديئة (ضجيج؟ باود خاطئ؟ سلك؟)، وابتلاعه يترك
    المستخدم أمام «لا يعمل» بلا دليل.
    """
    line = (line or "").strip()
    if not line:
        raise ProtocolError("إطار فارغ")
    if not line.startswith(FRAME_START):
        raise ProtocolError(f"إطار بلا بادئة '{FRAME_START}': {line[:24]!r}")
    if FRAME_SEP not in line:
        raise ProtocolError(f"إطار بلا فاصل checksum '{FRAME_SEP}': {line[:24]!r}")
    body, _, cksum = line[1:].rpartition(FRAME_SEP)
    if len(body) > LORA_MAX_PAYLOAD:
        raise ProtocolError(f"الحمولة {len(body)} بايت تتجاوز الحدّ {LORA_MAX_PAYLOAD}")
    try:
        got = int(cksum, 16)
    except ValueError:
        raise ProtocolError(f"checksum ليس hex: {cksum!r}") from None
    want = xor_checksum(body)
    if got != want:
        raise ProtocolError(f"checksum لا يطابق: وصل {got:02X} والمحسوب {want:02X}")
    return body


# ═══ 🔴 قائمة الأوامر المغلقة ═════════════════════════════════════
CMD_FWD = "FWD"
CMD_BACK = "BACK"
CMD_LEFT = "LEFT"
CMD_RIGHT = "RIGHT"
CMD_STOP = "STOP"
CMD_ESTOP = "ESTOP"
CMD_STATUS = "STATUS"
CMD_RTH = "RTH"
CMD_WITHDRAW = "WDRAW"   # انسحاب على أثر الدخول — أمر سلامة لا حركة مباشرة

#: القائمة المغلقة — أي أمر خارجها يُرفض ويُسجَّل.
RADIO_COMMANDS = frozenset({
    CMD_FWD, CMD_BACK, CMD_LEFT, CMD_RIGHT,
    CMD_STOP, CMD_ESTOP, CMD_STATUS, CMD_RTH, CMD_WITHDRAW,
})

#: الأوامر التي **تحرّك المحركات** — وحدها تمرّ ببوابة السلامة.
MOTION_COMMANDS = frozenset({CMD_FWD, CMD_BACK, CMD_LEFT, CMD_RIGHT})

#: أوامر لا تُرفض أبداً (تفشل إلى الأمان: إيقاف أو استعلام).
SAFE_ALWAYS = frozenset({CMD_STOP, CMD_ESTOP, CMD_STATUS})

# ── رموز الإقرار (قصيرة — تُبثّ على قناة ضيقة وتُعرض على الشاشة) ──
ACK_OK = "OK"            # نُفّذ
ACK_BADCMD = "BADCMD"    # خارج القائمة المغلقة
ACK_BADSEQ = "BADSEQ"    # مكرَّر أو خارج النافذة
ACK_BADCRC = "BADCRC"    # checksum/تأطير
ACK_SAFE = "SAFE"        # رفضته طبقة السلامة (عائق أمامي)
ACK_BUSY = "BUSY"        # مهمة ذاتية جارية — لا قيادة يدوية فوقها
ACK_FAULT = "FAULT"      # عطل عتاد (وصلة روفر/مصدر اتجاه)


@dataclass(frozen=True)
class Command:
    """أمر راديو مفكوك ومُتحقَّق منه."""
    seq: int
    cmd: str
    p1: float = 0.0
    p2: float = 0.0


def encode_command(seq: int, cmd: str, p1: float = 0.0, p2: float = 0.0) -> str:
    """يبني إطار أمر جاهزاً للإرسال (يرفض ما هو خارج القائمة المغلقة)."""
    cmd = str(cmd).strip().upper()
    if cmd not in RADIO_COMMANDS:
        raise ProtocolError(f"أمر خارج القائمة المغلقة: {cmd!r}")
    seq = int(seq) % LORA_SEQ_MODULO
    return build_frame(f"C,{seq},{cmd},{_num(p1)},{_num(p2)}")


def _num(v) -> str:
    """يختصر الأعداد: 0.30 → 0.3 و1.0 → 1 (كل بايت محسوب على LoRa)."""
    f = float(v)
    if f == int(f):
        return str(int(f))
    return f"{f:.2f}".rstrip("0").rstrip(".")


def decode_command(payload: str) -> Command:
    """
    يفكّ حمولة أمر `C,<seq>,<cmd>,<p1>,<p2>`.

    🔴 **الأمر خارج القائمة المغلقة يُرفض هنا** — قبل أن يصل إلى أي شيء
    يقود محركاً. ولا يوجد أي مسار يمرّر نصاً حرّاً إلى منفّذ.
    """
    parts = (payload or "").split(",")
    if len(parts) < 3 or parts[0] != "C":
        raise ProtocolError(f"حمولة أمر غير صالحة: {payload[:32]!r}")
    try:
        seq = int(parts[1])
    except ValueError:
        raise ProtocolError(f"تسلسل ليس عدداً: {parts[1]!r}") from None
    if not 0 <= seq < LORA_SEQ_MODULO:
        raise ProtocolError(f"تسلسل خارج المدى 0..{LORA_SEQ_MODULO - 1}: {seq}")
    cmd = parts[2].strip().upper()
    if cmd not in RADIO_COMMANDS:
        raise ProtocolError(f"أمر خارج القائمة المغلقة: {cmd!r}")
    try:
        p1 = float(parts[3]) if len(parts) > 3 and parts[3] != "" else 0.0
        p2 = float(parts[4]) if len(parts) > 4 and parts[4] != "" else 0.0
    except ValueError:
        raise ProtocolError(f"وسيط ليس عدداً في: {payload[:32]!r}") from None
    return Command(seq=seq, cmd=cmd, p1=p1, p2=p2)


def encode_telemetry(seq: int, cpm: float, volts, state: str) -> str:
    """
    تيليمتري مختصر: `T,<seq>,<cpm>,<mv>,<state>`.

    ⚠ الجهد **مجهولاً** يُبثّ `-1` لا `0`: صفر فولت قراءة كارثية وصفر
    «لا أعرف» يُعرض بنفس الشكل — والفرق بينهما هو كل شيء (قاعدة: الغائب
    مجهول لا سالم).
    """
    mv = -1 if volts is None else int(round(float(volts) * 1000.0))
    state = str(state or "?")[:8]
    return build_frame(f"T,{int(seq) % LORA_SEQ_MODULO},{int(round(cpm))},{mv},{state}")


@dataclass(frozen=True)
class Telemetry:
    seq: int
    cpm: float
    volts: float | None      # None = مجهول (وصل -1)
    state: str


def decode_telemetry(payload: str) -> Telemetry:
    """يفكّ حمولة تيليمتري (تستعملها وحدة التحكم والاختبارات)."""
    parts = (payload or "").split(",")
    if len(parts) < 5 or parts[0] != "T":
        raise ProtocolError(f"حمولة تيليمتري غير صالحة: {payload[:32]!r}")
    try:
        seq, cpm, mv = int(parts[1]), float(parts[2]), int(parts[3])
    except ValueError:
        raise ProtocolError(f"حقل عددي تالف في: {payload[:32]!r}") from None
    return Telemetry(seq=seq, cpm=cpm,
                     volts=None if mv < 0 else mv / 1000.0,
                     state=parts[4])


def encode_ack(seq: int, code: str) -> str:
    """إقرار/رفض — يجعل الرفض **مرئياً على وحدة التحكم** لا صامتاً."""
    return build_frame(f"A,{int(seq) % LORA_SEQ_MODULO},{code}")


# ═══ 🔴 منع إعادة الإرسال ═════════════════════════════════════════
class SequenceGuard:
    """
    يقبل التسلسل المتزايد داخل نافذة أمامية، ويرفض المكرَّر والقديم.

    الفجوة تُحسب دورياً: `(seq - last) % MODULO`.
      - `0`            ⇒ **مكرَّر** (إعادة إرسال) → رفض.
      - `1..WINDOW`    ⇒ تقدّم مقبول (يسمح بفقدان حزم).
      - غير ذلك        ⇒ قديم أو قفزة مريبة → رفض.

    ⚠ **حدود هذه الحماية — تُقال كما هي**:
    - ليست تعميةً ولا استيثاقاً. من يسمع القناة يقرأ كل شيء، ومن يبثّ
      **إطاراً جديداً بتسلسل صحيح** يُقبل. الحماية ضد **الإعادة** فقط.
    - `resync_after_s`: بعد صمت أطول من هذه المدة يُقبل أول إطار أياً كان
      تسلسله. بلا هذا يتجمّد الرابط للأبد إذا أُعيد تشغيل وحدة التحكم
      (تسلسلها يعود إلى 1 والمستقبِل عند 500 ⇒ رفض دائم) — وقناة الطوارئ
      التي تموت بعد إعادة تشغيل عديمة الفائدة. المقايضة معلنة: إطار مُعاد
      قد يُقبل مرة واحدة بعد صمت طويل، وحدود ضرره أن طبقة السلامة تبقى
      فوقه ومهلة الأوامر توقف المحركات خلال ثانيتين.
    - إعادة المزامنة **تُعلَن** في `last_reason` ويُحصيها `resyncs`.
    """

    def __init__(self, modulo: int = LORA_SEQ_MODULO,
                 window: int = LORA_SEQ_WINDOW,
                 resync_after_s: float = LORA_SEQ_RESYNC_S):
        self.modulo = int(modulo)
        self.window = int(window)
        self.resync_after_s = float(resync_after_s)
        self.last_seq = None
        self.last_accept_ts = None
        self.accepted = 0
        self.rejected = 0
        self.resyncs = 0
        self.last_reason = ""

    def check(self, seq: int, now: float) -> bool:
        """
        هل يُقبل هذا التسلسل؟ `now` يُحقن (لا `time.time()` داخلياً) كي
        يبقى المنطق قابلاً للاختبار بلا انتظار حقيقي.
        """
        seq = int(seq) % self.modulo
        if self.last_seq is None:
            self._accept(seq, now, "أول إطار — بدء التسلسل")
            return True

        silent = (self.last_accept_ts is not None
                  and (now - self.last_accept_ts) > self.resync_after_s)
        if silent:
            self.resyncs += 1
            self._accept(seq, now,
                         f"⚠ إعادة مزامنة بعد صمت > {self.resync_after_s:.0f}ث "
                         f"(تسلسل {seq} بعد {self.last_seq})")
            return True

        gap = (seq - self.last_seq) % self.modulo
        if gap == 0:
            self.rejected += 1
            self.last_reason = f"تسلسل مكرَّر ({seq}) — إعادة إرسال مرفوضة"
            return False
        if gap > self.window:
            self.rejected += 1
            self.last_reason = (f"تسلسل خارج النافذة: {seq} بعد {self.last_seq} "
                                f"(فجوة {gap} > {self.window})")
            return False
        self._accept(seq, now, "")
        return True

    def _accept(self, seq: int, now: float, reason: str) -> None:
        self.last_seq = seq
        self.last_accept_ts = now
        self.accepted += 1
        self.last_reason = reason

    def state(self) -> dict:
        return {"last_seq": self.last_seq, "accepted": self.accepted,
                "rejected": self.rejected, "resyncs": self.resyncs,
                "reason": self.last_reason}
