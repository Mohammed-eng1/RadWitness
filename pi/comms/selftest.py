# -*- coding: utf-8 -*-
"""
selftest.py — اختبارات طبقة الاتصال (منطق خالص، بلا عتاد)
==========================================================
    python -m pi.comms.selftest

يغطّي: التأطير والـchecksum · القائمة المغلقة · منع إعادة الإرسال ·
بوابة السلامة · مهلة الأوامر · تكامل الراديو مع المهمة.

🔴 **اختبارات تكامل لا وحدة معزولة** (قاعدة CLAUDE.md §8): الأقسام
«و» و«ز» تُثبت أن الأمر الآتي من سطر راديو نصّي يصل فعلاً إلى المحركات
عبر بوابة السلامة — لا أن الدوال تعمل كلٌّ على حدة. النمط الذي تكرر ست
مرات في هذا المشروع هو **وحدة مبنيّة لا يستدعيها أحد**.
"""
import sys
import time

from pi.comms.control import ManualControl, SOURCE_RADIO, SOURCE_MANUAL
from pi.comms.lora import LoRaLink
from pi.comms.protocol import (
    ProtocolError, SequenceGuard, build_frame, parse_frame, xor_checksum,
    encode_command, decode_command, encode_telemetry, decode_telemetry,
    encode_ack, RADIO_COMMANDS, ACK_SAFE, ACK_BADCMD, ACK_BUSY, ACK_BADSEQ,
    ACK_BADCRC, ACK_OK, ACK_FAULT,
)
from pi.config import (
    MANUAL_POWER_MAX, MANUAL_POWER_MIN, MANUAL_HEARTBEAT_S,
    LORA_COMMAND_TIMEOUT_S, LORA_SEQ_RESYNC_S, MAX_MOTOR_POWER,
)

_passed = 0
_failed = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global _passed, _failed
    if ok:
        _passed += 1
    else:
        _failed += 1
    mark = "  ✅ " if ok else "  ❌ "
    print(mark + name + (f"   [{detail}]" if detail else ""))


def section(title: str) -> None:
    print(f"\n{title}")


# ═══ مزدوجات اختبار (تحاكي المهمة بأقل سطح ممكن) ═════════════════
class FakeRover:
    """جسر وهمي يسجّل ما أُرسل إليه — لا سيريال ولا محركات."""

    def __init__(self):
        self.calls = []
        self.link_ok = True
        self.link_error = None
        self._v = 11.32

    def forward(self, power=0.4):
        self.calls.append(("forward", round(power, 3)))

    def backward(self, power=0.4):
        self.calls.append(("backward", round(power, 3)))

    def turn(self, direction, power=0.4):
        self.calls.append((f"turn_{direction}", round(power, 3)))

    def stop(self):
        self.calls.append(("stop", 0))

    def voltage(self):
        return self._v

    @property
    def last(self):
        return self.calls[-1] if self.calls else None


class FakeReactive:
    """طبقة تفاعلية مضبوطة يدوياً — القرار يُملى في الاختبار."""

    def __init__(self):
        self.decision = {"action": "go", "speed": 0.4, "priority": "clear",
                         "rung": "≥100سم", "reason": "الطريق سالك", "unknown": []}

    def decide(self, front_cm, ir_left, ir_right, ir_mid=None):
        return dict(self.decision)


class FakeMission:
    """أقل سطح مهمة تحتاجه البوابة — يكشف أي اعتماد خفيّ إضافي."""

    def __init__(self):
        self.rover = FakeRover()
        self.reactive = FakeReactive()
        self.state = "idle"
        self.events = []
        self.last_reactive = None
        self.last_reading = {"cpm": 42.0}
        self._worker = None
        self._sensors = {"ultrasonic_cm": 150.0, "ir_left": 1, "ir_right": 1,
                         "ir_mid": 1}
        self.estopped = 0
        self.returned_home = 0
        self.withdraw_reqs = []
        self.withdraw_ok = True

    def sensors(self):
        return dict(self._sensors)

    def estop(self):
        self.estopped += 1
        self.state = "estop"
        self.rover.stop()

    def return_home(self):
        self.returned_home += 1

    def request_withdraw(self, until_cpm=None, reason=""):
        self.withdraw_reqs.append(reason)
        if self.withdraw_ok:
            return {"ok": True, "until_cpm": until_cpm, "reason": reason}
        return {"ok": False,
                "error": "طلب انسحاب بلا قيادة محركات — لا حركة تُنفَّذ"}

    def _log(self, kind, msg):
        self.events.append((kind, msg))

    def log_kinds(self):
        return [k for k, _ in self.events]


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt
        return self.t


def main() -> None:
    # ═══ أ) التأطير والـchecksum ══════════════════════════════════
    section("أ) التأطير و checksum:")
    f = build_frame("PING,42")
    check("الإطار يطابق الصيغة المجرَّبة على العتاد $…*HH\\n",
          f.startswith("$") and f.endswith("\n") and "*" in f, repr(f))
    check("الذهاب والإياب يحفظ الحمولة", parse_frame(f) == "PING,42")
    check("checksum يطابق حساب NMEA اليدوي",
          xor_checksum("PING,42") == _manual_xor("PING,42"))

    bad = f.replace("PING", "PONG")
    check("🔴 بايت مقلوب في الحمولة يُرفض", _raises(parse_frame, bad),
          "checksum لم يعد يطابق")
    check("🔴 checksum خاطئ صراحةً يُرفض", _raises(parse_frame, "$PING,42*00"))
    check("🔴 checksum ليس hex يُرفض", _raises(parse_frame, "$PING,42*ZZ"))
    check("إطار بلا بادئة $ يُرفض", _raises(parse_frame, "PING,42*3A"))
    check("إطار بلا فاصل * يُرفض", _raises(parse_frame, "$PING,42"))
    check("إطار فارغ يُرفض", _raises(parse_frame, "   "))
    check("سبب الرفض **مقروء** لا None صامتة",
          "checksum" in _err(parse_frame, "$PING,42*00"),
          _err(parse_frame, "$PING,42*00"))
    check("حمولة تتجاوز الحدّ تُرفض عند البناء",
          _raises(build_frame, "X" * 200))

    # ═══ ب) 🔴 القائمة المغلقة ════════════════════════════════════
    section("ب) القائمة المغلقة:")
    check("القائمة هي التسعة المتفق عليها بالضبط (الثمانية + WDRAW)",
          RADIO_COMMANDS == {"FWD", "BACK", "LEFT", "RIGHT", "STOP",
                             "ESTOP", "STATUS", "RTH", "WDRAW"},
          str(sorted(RADIO_COMMANDS)))
    check("WDRAW يُفكّ كأي أمر من القائمة",
          decode_command("C,3,WDRAW,0,0").cmd == "WDRAW")
    c = decode_command(parse_frame(encode_command(7, "FWD", 0.3)))
    check("أمر سليم يُفكّ بحقوله", c.seq == 7 and c.cmd == "FWD" and c.p1 == 0.3,
          str(c))
    for evil in ("REBOOT", "rm -rf /", "FWD;BACK", "EXEC", ""):
        check(f"🔴 أمر خارج القائمة يُرفض: {evil!r}",
              _raises(decode_command, f"C,1,{evil},0,0"))
    check("🔴 لا أمر مركّب: الفاصلة تفصل حقولاً لا أوامر",
          _raises(decode_command, "C,1,FWD;BACK,0,0"))
    check("أمر بحروف صغيرة يُقبل ويُوحَّد",
          decode_command("C,1,fwd,0,0").cmd == "FWD")
    check("تسلسل ليس عدداً يُرفض", _raises(decode_command, "C,x,FWD,0,0"))
    check("وسيط ليس عدداً يُرفض", _raises(decode_command, "C,1,FWD,abc,0"))
    check("حمولة ليست أمراً (بادئة خاطئة) تُرفض",
          _raises(decode_command, "T,1,FWD,0,0"))
    check("البناء يرفض أمراً خارج القائمة قبل الإرسال",
          _raises(encode_command, 1, "REBOOT"))

    # ═══ ج) 🔴 منع إعادة الإرسال ══════════════════════════════════
    section("ج) منع إعادة الإرسال (التسلسل):")
    g = SequenceGuard()
    check("أول إطار يُقبل ويؤسّس التسلسل", g.check(5, 100.0))
    check("التالي المتزايد يُقبل", g.check(6, 100.1))
    check("🔴 **المكرَّر يُرفض** (إعادة إرسال)", not g.check(6, 100.2),
          g.last_reason)
    check("🔴 القديم يُرفض", not g.check(3, 100.3), g.last_reason)
    check("فجوة داخل النافذة تُقبل (فقدان حزم)", g.check(20, 100.4))
    check("🔴 فجوة خارج النافذة تُرفض", not g.check(500, 100.5), g.last_reason)
    check("عدّاد الرفض يُحصي", g.rejected == 3, f"rejected={g.rejected}")

    g2 = SequenceGuard()
    g2.check(995, 200.0)
    check("الالتفاف الدوري يُقبل (995 → 2)", g2.check(2, 200.1),
          "الفجوة تُحسب دورياً")

    g3 = SequenceGuard()
    g3.check(500, 300.0)
    check("🔴 قبل انقضاء مهلة الصمت لا إعادة مزامنة",
          not g3.check(1, 300.0 + LORA_SEQ_RESYNC_S - 0.1), g3.last_reason)
    check("إعادة المزامنة بعد صمت طويل تُقبل (وحدة تحكم أُعيد تشغيلها)",
          g3.check(1, 300.0 + LORA_SEQ_RESYNC_S + 0.1))
    check("وإعادة المزامنة **تُعلَن** لا تمرّ صامتة",
          g3.resyncs == 1 and "مزامنة" in g3.last_reason, g3.last_reason)

    # ═══ د) التيليمتري ════════════════════════════════════════════
    section("د) التيليمتري:")
    t = decode_telemetry(parse_frame(encode_telemetry(3, 120.4, 11.32, "RUN")))
    check("ذهاب وإياب يحفظ العدّ والجهد والحالة",
          t.seq == 3 and t.cpm == 120 and abs(t.volts - 11.32) < 1e-6
          and t.state == "RUN", str(t))
    tn = decode_telemetry(parse_frame(encode_telemetry(4, 0, None, "IDLE")))
    check("🔴 جهد مجهول يُبثّ -1 ويُفكّ None (لا صفر يوهم بانهيار)",
          tn.volts is None, str(tn))
    check("إطار التيليمتري داخل حدّ الطول",
          len(encode_telemetry(999, 99999, 12.6, "ESTOP")) <= 60,
          f"{len(encode_telemetry(999, 99999, 12.6, 'ESTOP'))} بايت")

    # ═══ هـ) بوابة السلامة ════════════════════════════════════════
    section("هـ) بوابة السلامة (القيادة اليدوية والراديو):")
    m = FakeMission()
    clk = FakeClock()
    mc = ManualControl(m, clock=clk)

    r = mc.command("FWD", 0.3)
    check("أمر حركة قبل تفعيل النمط يُرفض (حصرية الأنماط)",
          not r["ok"] and r["ack"] == ACK_BUSY, r["reason"])

    mc.set_enabled(True)
    r = mc.command("FWD", 0.3)
    check("بعد التفعيل والطريق سالك: يتقدّم",
          r["ok"] and m.rover.last[0] == "forward", str(m.rover.last))

    # عائق أمامي
    m.reactive.decision = {"action": "stop", "speed": 0.0, "priority": "ultrasonic",
                           "rung": "stop", "reason": "عائق أمامي 20سم < 30سم",
                           "unknown": []}
    n_before = len(m.rover.calls)
    r = mc.command("FWD", 0.5)
    check("🔴 **طبقة السلامة توقف الروبوت رغم أمر بالتقدم**",
          not r["ok"] and r["ack"] == ACK_SAFE, r["reason"])
    check("والرفض **يُوقف المحركات فعلاً** لا يتجاهل الأمر فقط",
          m.rover.calls[n_before:][-1][0] == "stop",
          str(m.rover.calls[n_before:]))
    check("والسبب مسجَّل في سجل الأحداث (لا فشل صامت)",
          "safety_block" in m.log_kinds(), m.events[-1][1])

    r = mc.command("BACK", 0.3)
    check("والرجوع يبقى مسموحاً — هو مخرج الانحشار",
          r["ok"] and m.rover.last[0] == "backward", str(m.rover.last))
    check("والخلف يُعلَن **مجهولاً** لا سالماً (لا حسّاس خلفيّ)",
          r.get("safety", {}).get("rear_unknown") is True,
          r.get("safety", {}).get("note", ""))
    r = mc.command("LEFT", 0.3)
    check("واللفّ يبقى مسموحاً مع عائق أمامي",
          r["ok"] and m.rover.last[0] == "turn_L", str(m.rover.last))

    # IR
    m.reactive.decision = {"action": "turn_right", "speed": 0.0, "priority": "ir",
                           "rung": "ir_left", "reason": "IR أمام-يسار",
                           "unknown": []}
    r = mc.command("FWD")
    check("🔴 عائق IR يمنع التقدّم أيضاً (لا الألترا سونيك وحده)",
          not r["ok"] and r["ack"] == ACK_SAFE, r["reason"])

    # ── تجاوز الحساسات (يدوي فقط، بطلب صريح — 2026-08-09) ──────────
    m.reactive.decision = {"action": "stop", "speed": 0.0, "priority": "ultrasonic",
                           "rung": "stop", "reason": "عائق أمامي 20سم",
                           "unknown": []}
    mc.ignore_sensors = True
    r = mc.command("FWD", 0.3)
    check("⚠ التجاوز مفعّل: التقدّم يُنفَّذ رغم العائق والسبب يبقى في الردّ",
          r["ok"] and m.rover.last[0] == "forward"
          and r.get("safety", {}).get("overridden") is True
          and "تجاوز" in r["reason"], r["reason"][:60])
    check("والتجاوز مسجَّل في السجل بعدّاده (لا تجاوز صامتاً)",
          "safety_override" in m.log_kinds() and mc.overridden_count > 0)
    mc.set_enabled(False)
    check("🔴 إغلاق اليدوية **يصفّر التجاوز** تلقائياً (لا يُورَّث لوضع آخر)",
          mc.ignore_sensors is False)
    mc.set_enabled(True)

    # قصّ السرعة بالسلّم
    m.reactive.decision = {"action": "go", "speed": 0.2, "priority": "clear",
                           "rung": "≥60سم", "reason": "مسافة 70سم", "unknown": []}
    r = mc.command("FWD", 0.5)
    check("سلّم المسافة **يقصّ** القوة المطلوبة عند اقتراب العائق",
          m.rover.last == ("forward", 0.2), str(m.rover.last))

    # المجهول
    m.reactive.decision = {"action": "go", "speed": 0.15, "priority": "clear",
                           "rung": "no_reading", "reason": "لا قراءة ألترا سونيك",
                           "unknown": ["front_left", "front_right"]}
    r = mc.command("FWD", 0.5)
    check("🔴 حسّاس غائب يُذكر **مجهولاً** في الردّ لا يُبتلع",
          r["safety"]["unknown"] == ["front_left", "front_right"],
          str(r["safety"]["unknown"]))

    # فشل قراءة الحساسات
    def boom():
        raise RuntimeError("ناقل I2C صامت")

    m.sensors = boom
    r = mc.command("FWD", 0.3)
    check("🔴 تعذّر قراءة الحساسات ⇒ **لا تقدّم على مجهول**",
          not r["ok"] and r["ack"] == ACK_SAFE, r["reason"])
    m.sensors = lambda: dict(m._sensors)
    m.reactive.decision = {"action": "go", "speed": 0.4, "priority": "clear",
                           "rung": "≥100سم", "reason": "سالك", "unknown": []}

    # ── WDRAW: انسحاب عبر البوابة نفسها ──────────────────────────
    r = mc.command("WDRAW")
    check("🔴 WDRAW بلا مهمة نشطة يُرفض (علم معلّق يفاجئ المهمة التالية)",
          not r["ok"] and r["ack"] == ACK_BUSY and not m.withdraw_reqs,
          r["reason"])
    m.state = "running"
    r = mc.command("WDRAW", source=SOURCE_RADIO)
    check("🔴 WDRAW أثناء مهمة يصل إلى `mission.request_withdraw` فعلاً",
          r["ok"] and len(m.withdraw_reqs) == 1, str(m.withdraw_reqs))
    check("ومصدر الأمر (راديو) داخل سبب الانسحاب المسجَّل",
          "راديو" in m.withdraw_reqs[0], m.withdraw_reqs[0])
    mc.set_enabled(False)
    r = mc.command("WDRAW")
    check("والقيادة اليدوية **ليست شرطاً** — أمر سلامة يمرّ والنمط مغلق",
          r["ok"] and len(m.withdraw_reqs) == 2, r.get("reason", ""))
    mc.set_enabled(True)
    m.withdraw_ok = False
    r = mc.command("WDRAW")
    check("ورفض المهمة (بلا قيادة محركات) يصل بسببه لا يُبتلع",
          not r["ok"] and r["ack"] == ACK_FAULT and "قيادة محركات" in r["reason"],
          r["reason"])
    m.withdraw_ok = True
    m.state = "idle"

    # ═══ و) حدّ القوة و ESTOP ═════════════════════════════════════
    section("و) حدّ القوة والإيقاف:")
    # ⚠ الطريق سالك تماماً أولاً: وإلا قصّ **سلّم المسافة** القوةَ قبل أن
    #   نرى أثر حدّ الفيرموير، فيبدو الحدّ مكسوراً وهو سليم (وقع فعلاً).
    m.reactive.decision = {"action": "go", "speed": MANUAL_POWER_MAX,
                           "priority": "clear", "rung": "≥100سم",
                           "reason": "سالك", "unknown": []}
    mc.command("FWD", 0.9)
    check("🔴 قوة فوق الحدّ تُقصّ إلى MAX_MOTOR_POWER (التفاف الفيرموير)",
          m.rover.last == ("forward", MANUAL_POWER_MAX),
          f"طُلب 0.9 → أُرسل {m.rover.last[1]}")
    m.reactive.decision = {"action": "go", "speed": 0.4, "priority": "clear",
                           "rung": "≥80سم", "reason": "مسافة 90سم", "unknown": []}
    mc.command("FWD", 0.9)
    check("🔴 وسلّم المسافة يقصّ **تحت** حدّ الفيرموير عند اقتراب العائق",
          m.rover.last == ("forward", 0.4),
          f"حدّ الفيرموير {MANUAL_POWER_MAX} لكن السلّم 0.4 → أُرسل {m.rover.last[1]}")
    check("والسقف هو MAX_MOTOR_POWER نفسه لا رقماً مستقلاً",
          MANUAL_POWER_MAX == MAX_MOTOR_POWER, f"{MANUAL_POWER_MAX}")
    mc.command("FWD", 0.01)
    check("قوة تحت الأرضية تُرفع إلى الحدّ الأدنى",
          m.rover.last == ("forward", MANUAL_POWER_MIN), str(m.rover.last))
    mc.command("FWD", "غير رقم")
    check("قوة غير رقمية تسقط إلى الافتراضي بلا انهيار",
          m.rover.last[0] == "forward", str(m.rover.last))

    m.reactive.decision = {"action": "stop", "speed": 0.0, "priority": "ir",
                           "rung": "ir_both", "reason": "عائق", "unknown": []}
    est = mc.command("ESTOP")
    check("🔴 **ESTOP لا يُرفض أبداً** حتى وطبقة السلامة رافضة",
          est["ok"] and m.estopped == 1, str(est["ack"]))
    mc2 = ManualControl(FakeMission(), clock=FakeClock())
    e2 = mc2.command("ESTOP")
    check("🔴 و ESTOP يعمل **بلا تفعيل النمط** (طوارئ لا شرط لها)",
          e2["ok"] and mc2.mission.estopped == 1)
    check("و STOP كذلك بلا شرط", mc2.command("STOP")["ok"])
    check("و STATUS يردّ تيليمتري بلا حركة",
          mc2.command("STATUS")["telemetry"]["cpm"] == 42.0)
    check("و RTH يمرّ عبر مسار المهمة القائم",
          mc2.command("RTH")["ok"] and mc2.mission.returned_home == 1)
    check("أمر خارج القائمة يُرفض في البوابة أيضاً",
          mc2.command("REBOOT")["ack"] == ACK_BADCMD)

    # وصلة روفر ميتة
    m.reactive.decision = {"action": "go", "speed": 0.4, "priority": "clear",
                           "rung": "≥100سم", "reason": "سالك", "unknown": []}
    m.rover.link_ok = False
    m.rover.link_error = "[Errno 9] Bad file descriptor"
    r = mc.command("FWD")
    check("🔴 وصلة روفر ميتة ⇒ رفض بسبب صريح لا محاولة عمياء",
          not r["ok"] and r["ack"] == "FAULT", r["reason"])
    m.rover.link_ok = True

    # مهمة ذاتية جارية
    class LiveWorker:
        @staticmethod
        def is_alive():
            return True

    m._worker = LiveWorker()
    r = mc.command("FWD")
    check("🔴 خيط مسح ذاتي حيّ ⇒ رفض القيادة اليدوية (تنازع على المنفذ)",
          not r["ok"] and r["ack"] == ACK_BUSY, r["reason"])
    m._worker = None

    # ═══ ز) مهلة الأوامر (heartbeat) ══════════════════════════════
    section("ز) مهلة الأوامر:")
    m2 = FakeMission()
    clk2 = FakeClock()
    mc3 = ManualControl(m2, clock=clk2)
    mc3.set_enabled(True)
    mc3.command("FWD", 0.3)
    check("لا إيقاف قبل انقضاء المهلة",
          mc3.poll(clk2.advance(MANUAL_HEARTBEAT_S - 0.1)) is None)
    res = mc3.poll(clk2.advance(0.2))
    check(f"🔴 انقطاع أوامر الواجهة > {MANUAL_HEARTBEAT_S}ث ⇒ إيقاف المحركات",
          res is not None and m2.rover.last[0] == "stop", str(res))
    check("والسبب مسجَّل", "manual_timeout" in m2.log_kinds(), m2.events[-1][1])
    check("ولا يتكرر الإيقاف بعد التنفيذ (لا فيضان سجل)",
          mc3.poll(clk2.advance(10.0)) is None)

    m3 = FakeMission()
    clk3 = FakeClock()
    mc4 = ManualControl(m3, clock=clk3)
    mc4.set_enabled(True)
    mc4.command("FWD", 0.3, source=SOURCE_RADIO)
    check(f"مهلة الراديو أطول ({LORA_COMMAND_TIMEOUT_S}ث) — أبطأ وأكثر فقداً",
          mc4.poll(clk3.advance(MANUAL_HEARTBEAT_S + 0.2)) is None)
    check(f"🔴 **انقطاع أوامر الراديو > {LORA_COMMAND_TIMEOUT_S}ث ⇒ إيقاف**",
          mc4.poll(clk3.advance(LORA_COMMAND_TIMEOUT_S)) is not None,
          str(m3.rover.last))
    check("ومهلة الراديو أقصر من مهلة الروفر العتادية (تسبقها)",
          LORA_COMMAND_TIMEOUT_S < 1.5 + LORA_COMMAND_TIMEOUT_S)

    m4 = FakeMission()
    mc5 = ManualControl(m4, clock=FakeClock())
    mc5.set_enabled(True)
    mc5.command("FWD", 0.3)
    mc5.set_enabled(False)
    check("🔴 إغلاق نمط القيادة يوقف المحركات (لا عجلة تدور بعد التبديل)",
          m4.rover.last[0] == "stop", str(m4.rover.last))

    # ═══ ح) تكامل: من سطر راديو نصّي إلى المحركات ═════════════════
    section("ح) تكامل الراديو (سطر نصّي → بوابة → محركات):")
    m5 = FakeMission()
    clk5 = FakeClock()
    mc6 = ManualControl(m5, clock=clk5)
    mc6.set_enabled(True)
    link = LoRaLink(m5, mc6, enabled=False, clock=clk5)

    res = link.handle_line(encode_command(1, "FWD", 0.3))
    check("🔴 **سطر راديو سليم يصل إلى المحركات فعلاً**",
          res["ok"] and m5.rover.last == ("forward", 0.3),
          str(m5.rover.last))
    check("وعدّاد الأوامر الناجحة يتقدّم", link.commands_ok == 1)

    res = link.handle_line(encode_command(1, "FWD", 0.3))
    check("🔴 **إعادة إرسال نفس الإطار تُرفض** (نفس التسلسل)",
          not res["ok"] and res["ack"] == ACK_BADSEQ, res["reason"])

    frame = encode_command(2, "FWD", 0.3)
    res = link.handle_line(frame[:-3] + "00\n")
    check("🔴 **checksum خاطئ يُرفض**",
          not res["ok"] and res["ack"] == ACK_BADCRC, res["reason"])

    res = link.handle_line("$C,3,REBOOT,0,0*" + f"{xor_checksum('C,3,REBOOT,0,0'):02X}")
    check("🔴 **أمر خارج القائمة المغلقة يُرفض ويُسجَّل**",
          not res["ok"] and "REBOOT" in res["reason"], res["reason"])
    check("والرفض ظاهر في سجل المهمة", "lora_reject" in m5.log_kinds(),
          m5.events[-1][1])

    n = len(m5.events)
    for _ in range(50):
        link.handle_line("$C,9,REBOOT,0,0*00")
    check("🔴 فيضان أطر تالفة **يُضغط** ولا يدفن أول سبب",
          len(m5.events) - n <= 3, f"{len(m5.events) - n} حدثاً لـ50 إطاراً")

    m5.reactive.decision = {"action": "stop", "speed": 0.0, "priority": "ir",
                            "rung": "ir_both", "reason": "عائق أمامي على IR",
                            "unknown": []}
    res = link.handle_line(encode_command(10, "FWD", 0.4))
    check("🔴 **طبقة السلامة تسبق أمر الراديو** — عائق ⇒ رفض وإيقاف",
          not res["ok"] and res["ack"] == ACK_SAFE and m5.rover.last[0] == "stop",
          res["reason"])

    res = link.handle_line(encode_command(11, "ESTOP"))
    check("🔴 **ESTOP عبر الراديو يعمل** رغم رفض السلامة للتقدّم",
          res["ok"] and m5.estopped == 1)

    tel = link._send_telemetry(clk5.t)
    check("التيليمتري يُبنى إطاراً صالحاً", parse_frame(tel).startswith("T,"),
          tel.strip())

    # ── 🔴 كسر حلقة الصدى (عطل مقاس 2026-08-06) ──────────────────
    # العتاد أعاد كل ما يُرسل، وكان الرد بإقرار على كل إطار مرفوض يولّد
    # صدى الإقرار فإقراراً جديداً: 2473 إطاراً في دقائق كلها مرفوضة
    # والقناة الهوائية مشوَّشة. القاعدة الآن: الإقرار لأطر `C,` سليمة
    # التأطير حصراً، والصدى يُسقَط قبل أي معالجة.
    rx0, tx0 = link.frames_rx, len(link._tx_recent)
    res = link.handle_line(tel, now=clk5.t)
    check("🔴 صدى تيليمترينا يُكشف ويُسقَط **قبل أي معالجة**",
          res.get("echo") is True and link.echoes >= 1, str(res))
    check("ولا يُحسب استقبالاً (لا تضليل في العدّادات)",
          link.frames_rx == rx0, f"rx={link.frames_rx}")
    check("🔴 **ولا يولّد أي إرسال** — الحلقة مكسورة من جذرها",
          len(link._tx_recent) == tx0, f"tx={len(link._tx_recent) - tx0:+}")
    check("والصدى معلَن حدثاً مقروءاً في السجل",
          "lora_echo" in m5.log_kinds())

    # صدى فات نافذة الكشف (3ث) ⇒ يمرّ كإطار وارد، لكنه ليس `C,` ⇒
    # يُسقَط معدوداً **بلا إقرار** — فلا وقود للحلقة من أي طريق.
    clk5.advance(4.0)
    tx0 = len(link._tx_recent)
    res = link.handle_line(tel, now=clk5.t)
    check("🔴 حمولة ليست أمراً (T/A) تُرفض **بلا أي ردّ** ولو فاتت نافذة الصدى",
          not res["ok"] and not res.get("echo")
          and len(link._tx_recent) == tx0, str(res.get("reason"))[:60])

    # إطار تالف الـchecksum ⇒ يُرفض ويُسجَّل، وبلا إقرار أيضاً
    tx0 = len(link._tx_recent)
    link.handle_line("$PING,42*00", now=clk5.t)
    check("🔴 checksum خاطئ ⇒ رفض مسجَّل **بلا إقرار** (لا ردّ على ضجيج)",
          len(link._tx_recent) == tx0)

    # وأمر `C,` سليم التأطير مرفوض الفكّ ⇒ **يستحق** إقرار BADCMD:
    # مرسله وحدة تحكم حقيقية، وصداه إن عاد `A,` فيُسقَط صامتاً — لا حلقة.
    tx0 = len(link._tx_recent)
    res = link.handle_line("$C,3,REBOOT,0,0*"
                           + f"{xor_checksum('C,3,REBOOT,0,0'):02X}", now=clk5.t)
    check("أمر سليم التأطير خارج القائمة ⇒ إقرار BADCMD يُرسل (حقّ وحدة التحكم)",
          len(link._tx_recent) == tx0 + 1 and res["ack"] == ACK_BADCMD,
          res["reason"][:50])
    ack_frame = link._tx_recent[-1][0]
    clk5.advance(4.0)                     # حتى صدى الإقرار المتأخر
    tx0 = len(link._tx_recent)
    link.handle_line(ack_frame, now=clk5.t)
    check("🔴 وصدى ذلك الإقرار (حتى المتأخر) لا يولّد إقراراً جديداً — **لا حلقة**",
          len(link._tx_recent) == tx0)

    link2 = LoRaLink(FakeMission(), mc6, enabled=False)
    st = link2.start()
    check("🔴 الراديو معطّل ⇒ سبب مقروء لا فشل صامت",
          not st["ok"] and "معطّل" in st["error"], st["error"])
    check("وحالة الوصلة تفرّق بين الأسباب في الواجهة",
          set(("enabled", "ok", "error", "port", "seq")) <= set(st.keys()))

    # ═══ ط) الحالة المبثوثة ═══════════════════════════════════════
    section("ط) الحالة المعروضة:")
    s = mc6.state()
    for k in ("enabled", "engaged", "last_cmd", "last_source", "blocked",
              "rejected", "power_min", "power_max", "heartbeat_s"):
        check(f"حالة البوابة تعرض `{k}`", k in s)
    check("وتميّز المصدر (يدوي/راديو) للسجل",
          mc6.state()["last_source"] == SOURCE_RADIO, str(s["last_source"]))

    # ═══ ي) 🔴 تكامل مع MissionSim الحقيقي (لا مزدوجات) ═══════════
    # القاعدة التي كُسرت ست مرات: وحدة مبنيّة لا يستدعيها أحد. والمزدوجات
    # وحدها لا تكشفها — لو تغيّر توقيع `sensors()` أو `reactive.decide()`
    # لبقيت الاختبارات أعلاه خضراء والنظام الحقيقي معطّلاً. هنا الكائن
    # الحقيقي بلا أي بديل.
    section("ي) تكامل مع MissionSim الحقيقي:")
    from pi.nav.mission import MissionSim

    real = MissionSim()
    real.configure_room(length_m=4.0, width_m=3.0)
    rmc = ManualControl(real)
    rmc.set_enabled(True)

    r = rmc.command("FWD", 0.3)
    check("🔴 البوابة تعمل على توقيع `mission.sensors()` الحقيقي",
          isinstance(r, dict) and "ack" in r, str(r.get("reason"))[:70])
    check("و`reactive.decide()` الحقيقية تُستدعى فعلاً (قرار محفوظ)",
          real.last_reactive is not None,
          str((real.last_reactive or {}).get("rung")))
    check("والقوة المُرسلة داخل حدّ الفيرموير على الجسر الحقيقي",
          max(abs(v) for v in real.rover._cmd_lr) <= MAX_MOTOR_POWER,
          f"_cmd_lr={real.rover._cmd_lr}")

    # WDRAW على المهمة الحقيقية: في المحاكاة `drive_motors=False`، فالرفض
    # المتوقَّع يأتي **من المهمة نفسها** بسببها المقروء — أي أن النداء وصل
    # فعلاً إلى `request_withdraw` الحقيقية لا إلى مزدوجة (قاعدة التوصيل).
    real.state = "running"
    r = rmc.command("WDRAW")
    check("🔴 WDRAW يبلغ `request_withdraw` الحقيقية (رفضها يعود بسببها)",
          not r["ok"] and r["ack"] == ACK_FAULT and "قيادة محركات" in r["reason"],
          r["reason"])
    check("وطلب الانسحاب مسجَّل في أحداث المهمة الحقيقية",
          any(e["kind"] == "withdraw_request" for e in real.events),
          str([e["kind"] for e in real.events[-3:]]))
    real.state = "idle"

    rmc.command("ESTOP")
    check("🔴 ESTOP يغيّر حالة المهمة الحقيقية إلى estop",
          real.state == "estop", real.state)
    check("والحدث مسجَّل في سجل المهمة الحقيقي",
          any(e.get("kind") == "estop" for e in real.events),
          str(real.events[-1].get("msg", ""))[:60])

    real2 = MissionSim()
    real2.configure_room(length_m=4.0, width_m=3.0)
    rmc2 = ManualControl(real2)
    link3 = LoRaLink(real2, rmc2, enabled=False)
    rmc2.set_enabled(True)
    res = link3.handle_line(encode_command(1, "FWD", 0.3))
    check("🔴 **سطر راديو → MissionSim حقيقية → جسر حقيقي**",
          isinstance(res, dict) and "ack" in res, str(res.get("reason"))[:70])
    res = link3.handle_line(encode_command(2, "ESTOP"))
    check("و ESTOP عبر الراديو يوقف المهمة الحقيقية",
          res["ok"] and real2.state == "estop", real2.state)

    # السيرفر يستدعي البوابة فعلاً (لا وحدة معلّقة)
    import inspect
    import pi.web.server as srv
    src = inspect.getsource(srv)
    check("🔴 السيرفر يُنشئ البوابة ووصلة الراديو",
          "ManualControl(mission)" in src and "LoRaLink(mission, manual)" in src)
    check("🔴 وحلقة السيرفر تنادي `manual.poll` (حارس المهلة يعمل فعلاً)",
          "manual.poll(" in src)
    check("🔴 ولا مسار يقود جسراً ثانياً حول طبقة السلامة",
          "rover.command(cmd.get" not in src,
          "أُزيل مسار ws/sensors القديم")
    check("وحالة الراديو مكشوفة للواجهة", "/api/lora/status" in src)
    check("وأمر القيادة اليدوية يمرّ بالبوابة",
          "manual.command(" in src)
    check("وإغلاق السيرفر يوقف الراديو والمحركات",
          "lora.stop()" in src and "manual.set_enabled(False)" in src)

    # ═══ ك) 🔴 تطابق فيرموير الشاشة مع بايثون ═════════════════════
    # البروتوكول مكتوب مرتين (بايثون هنا، C++ على CYD). أي انحراف —
    # بايت في checksum أو صياغة رقم — يُنتج «أطراً تالفة» على قناة راديو
    # يصعب تنقيحها، والعرَض يبدو عطلاً كهربائياً لا برمجياً. فيُترجَم
    # اختبار الفيرموير ويُقارن آلياً بدل الاعتماد على المراجعة البصرية.
    # (وقع فعلاً: C++ كان يكتب `0.30` وبايثون `0.3`.)
    section("ك) تطابق فيرموير الشاشة (C++) مع بايثون:")

    # 🔴 حارس ASCII: تعليق عربي واحد في C++ كسر ترجمة Arduino فعلاً
    # (BiDi يخلط ترتيب المحارف فيقع `//` في غير موضعه — والخطأ يشير إلى
    # سطر بريء). بايثون تتحمّل العربية، ومترجم Arduino لا. هذا الفحص
    # يمسك التسرّب **قبل** أي مترجم وعلى كل منصة.
    from pathlib import Path as _P
    _fw_base = _P(__file__).resolve().parents[2] / "firmware"
    _fw_root = _fw_base / "controller_display"
    for _f in sorted(_fw_base.rglob("*")):
        if _f.suffix not in (".ino", ".h", ".cpp"):
            continue
        _raw = _f.read_bytes()
        _bad = [i for i, b in enumerate(_raw) if b > 127]
        check(f"🔴 ASCII صرف (لا عربية/BiDi): {_f.name}",
              not _bad,
              f"أول بايت غير ASCII عند الإزاحة {_bad[0]}" if _bad else
              f"{len(_raw)} بايت كلها ASCII")

    for _sketch in ("controller_display", "lora_probe"):
        _ino = _arduino_compile(_fw_base / _sketch)
        if _ino is None:
            print(f"  ⏭  ترجمة {_sketch} تُخطّى: لا arduino-cli أو لا نواة esp32")
            print("     يدوياً: arduino-cli compile --fqbn esp32:esp32:esp32 "
                  f"firmware/{_sketch}")
        else:
            ok_c, err_c = _ino
            check(f"🔴 {_sketch} يُترجَم بلا خطأ (arduino-cli)", ok_c,
                  err_c[:120] if not ok_c else "compile OK")

    firm = _run_firmware_vectors()
    if firm is None:
        print("  ⏭  تُخطّى: لا مترجم g++ (تُشغَّل على جهاز فيه مترجم)")
        print("     يدوياً: cd firmware/controller_display && "
              "g++ -std=c++17 -I test_stub -o test_protocol test_protocol.cpp")
    else:
        cmds = [ln.split("\t", 1)[1] for ln in firm if ln.startswith("CMD\t")]
        expect = [encode_command(s, c, p) for s, c, p in (
            (1, "FWD", 0.30), (7, "FWD", 0.0), (42, "ESTOP", 0.0),
            (999, "STOP", 0.5), (123, "RTH", 0.25), (0, "STATUS", 0.0),
            (500, "LEFT", 0.4), (12, "BACK", 0.1), (1001, "RIGHT", 1.0),
            (77, "WDRAW", 0.0))]
        for got, want in zip(cmds, expect):
            check(f"إطار متطابق بايتاً ببايت: {want.strip()}",
                  got == want.rstrip("\n"), f"الفيرموير: {got}")

        allow = {ln.split("\t")[1]: ln.split("\t")[2]
                 for ln in firm if ln.startswith("ALLOW\t")}
        check("🔴 والقائمة المغلقة نفسها على الطرفين",
              all(v == "0" for v in allow.values()), str(allow))

        xr = {ln.split("\t")[1]: ln.split("\t")[2]
              for ln in firm if ln.startswith("XOR\t")}
        for payload, hexv in xr.items():
            check(f"checksum متطابق: {payload}",
                  f"{xor_checksum(payload):02X}" == hexv,
                  f"C++={hexv} بايثون={xor_checksum(payload):02X}")

        par = [ln.split("\t") for ln in firm if ln.startswith("PARSE\t")]
        for _, frame_s, res in par:
            try:
                py = parse_frame(frame_s)
            except ProtocolError:
                py = "REJECT"
            check(f"قرار الفكّ متطابق: {frame_s!r}", py == res,
                  f"C++={res} بايثون={py}")

        tel = [ln.split("\t") for ln in firm if ln.startswith("TEL\t")]
        for row in tel:
            payload, valid, seq, cpm, mv, state = row[1], row[2], row[3], row[4], row[5], row[6]
            t = decode_telemetry(payload)
            same = (valid == "1" and t.seq == int(seq) and t.cpm == float(cpm)
                    and (t.volts is None if int(mv) < 0
                         else abs(t.volts - int(mv)/1000.0) < 1e-9)
                    and t.state == state)
            check(f"تيليمتري مفكوك متطابق: {payload}", same,
                  f"C++ mv={mv} بايثون volts={t.volts}")

    print(f"\n=== النتيجة: {_passed}/{_passed + _failed} نجح ===")
    if _failed:
        sys.exit(1)


def _arduino_compile(fw_root):
    """
    يترجم الفيرموير كاملاً بـarduino-cli إن توفّرت الأداة **ونواة esp32**.

    يُعيد None عند غياب أيّهما (تخطٍّ معلَن لا نجاح صامت) — أما إذا توفّرا
    وفشلت الترجمة فهذا **فشل حقيقي**: على الجهاز المجهّز للرفع هذا بالضبط
    ما نريد أن يصرخ قبل محاولة الرفع لا أثناءها.
    """
    import shutil
    import subprocess

    cli = shutil.which("arduino-cli")
    if cli is None:
        return None
    try:
        cores = subprocess.run([cli, "core", "list"], capture_output=True,
                               text=True, timeout=30).stdout
        if "esp32:esp32" not in cores:
            return None
        r = subprocess.run(
            [cli, "compile", "--fqbn", "esp32:esp32:esp32", str(fw_root)],
            capture_output=True, text=True, timeout=600)
        return (r.returncode == 0, (r.stderr or r.stdout))
    except Exception as e:                          # noqa: BLE001
        return (False, f"تعذّر تشغيل arduino-cli: {e}")


def _run_firmware_vectors():
    """
    يترجم `test_protocol.cpp` ويُشغّله، ويُعيد أسطره — أو None بلا مترجم.

    ⚠ **لا يُفشل الاختبار عند غياب g++**: الراسبري قد لا يحمل مترجماً،
    وإفشال كل الطاقم لأجل ذلك يدفع إلى تعطيل الاختبار كله. الغياب
    يُعلَن سطراً صريحاً (تخطٍّ معروف لا نجاح صامت).
    """
    import shutil
    import subprocess
    import tempfile
    from pathlib import Path

    gpp = shutil.which("g++")
    root = Path(__file__).resolve().parents[2] / "firmware" / "controller_display"
    src = root / "test_protocol.cpp"
    if gpp is None or not src.exists():
        return None
    try:
        with tempfile.TemporaryDirectory() as td:
            exe = str(Path(td) / "tp.exe")
            r = subprocess.run(
                [gpp, "-std=c++17", "-I", str(root / "test_stub"),
                 "-o", exe, str(src)],
                capture_output=True, text=True, timeout=120)
            if r.returncode != 0:
                print("  ⚠ تعذّرت ترجمة اختبار الفيرموير:\n" + r.stderr[:400])
                return None
            out = subprocess.run([exe], capture_output=True, text=True,
                                 timeout=60, cwd=str(root))
            return [ln for ln in out.stdout.splitlines() if ln.strip()]
    except Exception as e:                          # noqa: BLE001
        print(f"  ⚠ تعذّر تشغيل اختبار الفيرموير: {e}")
        return None


def _manual_xor(s: str) -> int:
    c = 0
    for ch in s.encode("ascii"):
        c ^= ch
    return c


def _raises(fn, *a) -> bool:
    try:
        fn(*a)
        return False
    except ProtocolError:
        return True
    except Exception:                              # noqa: BLE001
        return False


def _err(fn, *a) -> str:
    try:
        fn(*a)
        return ""
    except Exception as e:                         # noqa: BLE001
        return str(e)


if __name__ == "__main__":
    main()
