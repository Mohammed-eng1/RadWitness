# -*- coding: utf-8 -*-
"""
selftest.py — اختبارات طبقة الواجهة عبر HTTP الحقيقي
=====================================================
    python -m pi.web.selftest

🔴 **عبر السيرفر الحقيقي لا بنداء الدوال مباشرة**: يبني `TestClient` على
`app` نفسه، فيمرّ كل اختبار بالمسار الكامل (توجيه → نموذج → بوابة →
مهمة). النمط الذي كسر هذا المشروع ست مرات هو وحدة تعمل معزولة ولا
يستدعيها أحد — واختبار يستدعي الدالة مباشرةً يعيد إنتاجه بالضبط.

يغطّي معايير القبول: الأنماط الثلاثة · التبديل أثناء مهمة (إيقاف مؤقت
لا إلغاء) · إيقاف الطوارئ في الأنماط الثلاثة · بثّ الكاميرا مع النمط ·
رفض أمر خارج القائمة · طبقة السلامة فوق أمر الراديو.
"""
import sys

from fastapi.testclient import TestClient

_passed = 0
_failed = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global _passed, _failed
    if ok:
        _passed += 1
    else:
        _failed += 1
    print(("  ✅ " if ok else "  ❌ ") + name + (f"   [{detail}]" if detail else ""))


def section(title: str) -> None:
    print(f"\n{title}")


def main() -> None:
    import pi.web.server as srv
    from pi.comms.protocol import encode_command

    c = TestClient(srv.app)
    mission = srv.mission
    modes = srv.modes
    manual = srv.manual

    # ═══ أ) الصفحات ═══════════════════════════════════════════════
    section("أ) الصفحات:")
    r = c.get("/")
    check("`/` تُقدّم لوحة التحكم الموحّدة", r.status_code == 200
          and "لوحة التحكم" in r.text, f"HTTP {r.status_code}")
    check("🔴 **بلا أي CDN** (نعمل بلا إنترنت)",
          "//cdn" not in r.text and "https://" not in r.text.split("<style>")[0],
          "لا مورد خارجي")
    check("عربية RTL", 'dir="rtl"' in r.text and 'lang="ar"' in r.text)
    check("الواجهة التفصيلية السابقة مُبقاة على `/sim`",
          c.get("/sim").status_code == 200)
    check("وواجهة الحساسات على `/sensors`", c.get("/sensors").status_code == 200)

    # ═══ ب) الأنماط الثلاثة ═══════════════════════════════════════
    section("ب) الأنماط الثلاثة:")
    st = c.get("/api/mode").json()
    check("ثلاثة أنماط بالضبط",
          [m["id"] for m in st["modes"]] == ["sim", "manual", "auto"],
          str([m["ar"] for m in st["modes"]]))
    check("الافتراضي محاكاة", st["mode"] == "sim", st["mode_ar"])

    r = c.post("/api/mode", json={"mode": "manual"}).json()
    check("التبديل إلى القيادة اليدوية ينجح", r["ok"] and r["mode"] == "manual")
    check("🔴 والحصرية: البوابة تُفتح مع النمط", manual.enabled is True)
    check("والبثّ يبدأ **بدخول النمط**", modes.camera_streaming is True)

    r = c.post("/api/mode", json={"mode": "auto"}).json()
    check("التبديل إلى القيادة الذاتية ينجح", r["ok"] and r["mode"] == "auto")
    check("🔴 ومغادرة اليدوي تُغلق البوابة (لا محرّك يستجيب)",
          manual.enabled is False)
    check("🔴 والبثّ **يتوقف بمغادرة النمط**", modes.camera_streaming is False)

    r = c.post("/api/mode", json={"mode": "زائف"})
    check("نمط غير معروف يُرفض بسبب مقروء",
          r.status_code == 400 and "غير معروف" in r.json()["error"],
          r.json().get("error", ""))

    c.post("/api/mode", json={"mode": "sim"})
    check("العودة إلى المحاكاة", c.get("/api/mode").json()["mode"] == "sim")

    # ═══ ج) 🔴 التبديل أثناء مهمة: إيقاف مؤقت لا إلغاء ════════════
    section("ج) التبديل أثناء مهمة جارية:")
    # ⚠ `start()` يشترط ملف معايرة (المسافة تُشتق من السرعة المعايرة).
    #   يُضبط في الذاكرة لا عبر `/api/calibration/new_sim` كي لا يكتب
    #   الاختبارُ ملفات معايرة في مجلد المستخدم.
    from pi.nav.mission import default_sim_profile
    mission.set_calibration(default_sim_profile(battery_v=12.0))
    c.post("/api/room", json={"length_m": 4, "width_m": 3})
    c.post("/api/mode", json={"mode": "auto"})
    started = c.post("/api/mission/start", json={}).json()
    check("المهمة جارية", mission.state == "running",
          mission.state + " · " + str(started.get("error", ""))[:40])

    r = c.post("/api/mode", json={"mode": "manual"}).json()
    check("🔴 **التبديل يتطلب تأكيداً** ولا ينفَّذ فوراً",
          r.get("needs_confirm") is True and not r.get("ok"), r.get("error", "")[:60])
    check("والمهمة **لم تُمسّ** قبل التأكيد", mission.state == "running")

    r = c.post("/api/mode", json={"mode": "manual", "confirm": True}).json()
    check("وبالتأكيد ينتقل النمط", r["ok"] and r["mode"] == "manual")
    check("🔴 **إيقاف مؤقت لا إلغاء** — الحالة paused",
          mission.state == "paused", mission.state)
    check("والخلايا المقيسة باقية (لا إلغاء)",
          mission.grid is not None and mission.room is not None)

    r = c.post("/api/mode", json={"mode": "auto", "confirm": True}).json()
    check("🔴 **والعودة تستأنف**", mission.state == "running", mission.state)

    # لا استئناف لما أوقفه المستخدم بنفسه
    c.post("/api/mission/pause", json={})
    c.post("/api/mode", json={"mode": "sim", "confirm": True})
    c.post("/api/mode", json={"mode": "auto", "confirm": True})
    check("🔴 ولا تُستأنف مهمة أوقفها المستخدم بنفسه (لا حركة بلا أمر)",
          mission.state == "paused", mission.state)

    c.post("/api/mission/estop", json={})

    # ═══ د) إيقاف الطوارئ في الأنماط الثلاثة ══════════════════════
    section("د) إيقاف الطوارئ:")
    for mode in ("sim", "manual", "auto"):
        c.post("/api/mode", json={"mode": mode, "confirm": True})
        mission.state = "running"
        r = c.post("/api/manual/command", json={"cmd": "ESTOP"}).json()
        check(f"🔴 **إيقاف الطوارئ يعمل في نمط «{mode}»**",
              r["ok"] and mission.state == "estop", mission.state)
    mission.state = "idle"

    # ═══ هـ) القيادة اليدوية عبر HTTP ═════════════════════════════
    section("هـ) القيادة اليدوية:")
    c.post("/api/mode", json={"mode": "manual", "confirm": True})
    r = c.post("/api/manual/command", json={"cmd": "REBOOT"}).json()
    check("🔴 أمر خارج القائمة المغلقة يُرفض عبر HTTP أيضاً",
          not r["ok"] and r["ack"] == "BADCMD", r["reason"])

    r = c.post("/api/manual/command", json={"cmd": "FWD", "power": 0.9}).json()
    check("أمر تقدّم يمرّ بالبوابة ويعود بقراره",
          "ack" in r and "reason" in r, str(r.get("reason"))[:60])
    check("والقوة المُرسلة داخل حدّ الفيرموير",
          max(abs(v) for v in mission.rover._cmd_lr) <= 0.5,
          f"_cmd_lr={mission.rover._cmd_lr}")

    # 🔴 التجاوز يرفع **القصّ** كما يرفع المنع ──────────────────────
    # عطل مقاس 2026-08-11: منفذ مفتوح تماماً والتقدّم عالق على 0.25 رغم
    # تفعيل «تجاوز حساسات العوائق». السبب أن أشيع حالة قصّ **ليست منعاً**:
    # الأمامي فوق عتبة الثقة (30سم) ⇒ `ultrasonic_cm` مجهولة ⇒ درجة
    # `no_reading` = 0.25 دائماً في الفضاء المفتوح. فالبوابة تسمح ولا
    # تُطلق فرع التجاوز، والقصّ يبقى مطبَّقاً — بينما التعليق في الكود كان
    # يعِد بعكس ذلك. (الحارس هنا لا في comms/selftest تفادياً لتصادم
    # تحرير متزامن جارٍ على ذلك الملف.)
    from pi.config import SPEED_NO_READING as _SNR
    _sv_sensors = mission.sensors
    mission.sensors = lambda: {"ultrasonic_cm": None,
                               "ultrasonic_raw_cm": 180.0,
                               "ir_left": 1, "ir_right": 1, "ir_mid": 1}
    try:
        manual.ignore_sensors = False
        r_cap = c.post("/api/manual/command",
                       json={"cmd": "FWD", "power": 0.5}).json()
        manual.ignore_sensors = True
        r_free = c.post("/api/manual/command",
                        json={"cmd": "FWD", "power": 0.5}).json()
    finally:
        mission.sensors = _sv_sensors
        manual.ignore_sensors = False
    check("بلا تجاوز: التقدّم يُقصّ إلى درجة «لا قراءة» (السلامة سليمة)",
          r_cap.get("ok") and abs(r_cap.get("power", 0) - _SNR) < 1e-9,
          f"قوة مطلوبة 0.50 → منفَّذة {r_cap.get('power')}")
    check("🔴 ومع التجاوز يُرفع **القصّ** لا المنع وحده (الوعد يُنفَّذ)",
          r_free.get("ok") and abs(r_free.get("power", 0) - 0.5) < 1e-9
          and (r_free.get("safety") or {}).get("overridden") is True,
          f"قوة منفَّذة {r_free.get('power')} · تجاوز="
          f"{(r_free.get('safety') or {}).get('overridden')}")
    check("والرفع **معلَن** في السبب (لا سلامة تُرفع بصمت)",
          "تجاوز مفعّل" in (r_free.get("reason") or ""),
          (r_free.get("reason") or "")[:56])

    c.post("/api/mode", json={"mode": "sim", "confirm": True})
    r = c.post("/api/manual/command", json={"cmd": "FWD"}).json()
    check("🔴 وأمر حركة خارج نمط القيادة اليدوية يُرفض (حصرية)",
          not r["ok"] and r["ack"] == "BUSY", r["reason"])

    st = c.get("/api/manual/status").json()
    check("حالة البوابة والراديو مكشوفة للواجهة",
          "manual" in st and "lora" in st)

    # ── الانسحاب عبر نفس البوابة ─────────────────────────────────
    r = c.post("/api/manual/command", json={"cmd": "WDRAW"}).json()
    check("🔴 WDRAW بلا مهمة نشطة يُرفض بسبب مقروء (لا علم كامناً)",
          not r["ok"] and "لا مهمة نشطة" in r["reason"], r["reason"])
    mission.state = "running"
    r = c.post("/api/manual/command", json={"cmd": "WDRAW"}).json()
    check("🔴 WDRAW أثناء مهمة يمرّ عبر HTTP إلى `request_withdraw`",
          "ack" in r and ("ok" in r), f"ack={r.get('ack')} — {str(r.get('reason'))[:60]}")
    # في المحاكاة drive_motors=False ⇒ ترفضه المهمة بسببها — وهذا إثبات
    # الوصول (قاعدة التوصيل): السبب صادر من mission لا من البوابة.
    check("وسبب المهمة الحقيقية يصل إلى الواجهة كما هو",
          "قيادة محركات" in str(r.get("reason", "")), str(r.get("reason"))[:70])
    mission.state = "idle"
    full = c.get("/api/sim/status").json()
    check("حالة الانسحاب (pending/active) داخل البثّ للشريط العلوي",
          "withdraw" in full and "pending" in full["withdraw"]
          and "active" in full["withdraw"], str(full.get("withdraw")))
    html = c.get("/").text
    check("زر الانسحاب موجود في الواجهة ويمرّ بالبوابة",
          'id="withdraw"' in html and '"WDRAW"' in html.replace("'", '"'))

    # ═══ و) الكاميرا: بثّ عند الطلب فقط ═══════════════════════════
    section("و) بثّ الكاميرا:")
    r = c.get("/stream.mjpg")
    check("🔴 **البثّ مرفوض خارج نمط القيادة اليدوية** بسبب مقروء",
          r.status_code == 409 and "القيادة اليدوية" in r.text,
          f"HTTP {r.status_code}")
    opts = c.get("/api/camera/options").json()
    check("الدقّات المتاحة معروضة", opts["options"] == ["640x480", "320x240"],
          str(opts["options"]))
    check("وغياب الكاميرا يُعلَن بسببه لا بانهيار",
          "camera" in opts and ("error" in opts),
          str(opts.get("error"))[:50])
    # ⚠ **لا يُطلب `/stream.mjpg` من الاختبار**: MJPEG تيّار لا نهائي، وعلى
    #   جهاز فيه كاميرا فعلاً يفتح الطلبُ الجهازَ ويعلّق الاختبار (وقع على
    #   ويندوز بـopencv مثبّتة). ولا يصحّ أن يستولي اختبارٌ على كاميرا
    #   المستخدم أصلاً. فيُختبر المولّد مباشرةً بجهاز وهمي — وهو موضع
    #   المنطق الحقيقي، والبوابة (409) مُختبرة أعلاه عبر HTTP.
    from pi.web.stream import mjpeg_frames

    class DeadCam:
        @staticmethod
        def snapshot_jpeg():
            return None

    n = sum(1 for _ in mjpeg_frames(DeadCam(), "320x240", is_active=lambda: True))
    check("🔴 كاميرا غائبة ⇒ البثّ ينتهي بلا إطارات (لا انهيار ولا تعليق)",
          n == 0, f"{n} إطاراً")

    class FakeCam:
        def __init__(self):
            self.n = 0

        def snapshot_jpeg(self):
            self.n += 1
            return b"\xff\xd8\xff\xdb" + b"x" * 64      # ليست JPEG صالحة عمداً

    fc = FakeCam()
    live = {"on": True}

    def gen():
        for i, chunk in enumerate(mjpeg_frames(fc, "320x240",
                                               is_active=lambda: live["on"])):
            yield chunk
            if i >= 1:
                live["on"] = False                     # حاكِ مغادرة النمط

    frames = list(gen())
    # الرايةُ تُطفأ بعد الإطار الثاني (i>=1) ⇒ الدورة التالية تفحص فتتوقف
    check("🔴 **مغادرة النمط تُنهي البثّ فوراً** (`is_active`) لا تنتظر المتصفح",
          len(frames) == 2, f"{len(frames)} إطارين ثم توقّف فوري")
    check("والإطار بصيغة multipart الصحيحة",
          frames and frames[0].startswith(b"--frame\r\nContent-Type: image/jpeg"))
    check("وإطار تالف يُمرَّر كما هو بدل إسقاط البثّ",
          frames and b"\xff\xd8\xff\xdb" in frames[0], "تصغير فاشل ⇒ الأصل")

    c.post("/api/mode", json={"mode": "sim", "confirm": True})
    check("🔴 ومغادرة النمط تُطفئ راية البثّ",
          modes.camera_streaming is False)

    # ═══ ز) 🔴 السلامة فوق أمر الراديو (عبر السيرفر الحقيقي) ══════
    section("ز) طبقة السلامة فوق الراديو:")
    srv.manual.set_enabled(True)

    class Blocked:
        @staticmethod
        def decide(front, l, r, mid=None):
            return {"action": "stop", "speed": 0.0, "priority": "ir",
                    "rung": "ir_both", "reason": "عائق أمامي مقاس", "unknown": []}

    real_reactive = mission.reactive
    mission.reactive = Blocked()
    res = srv.lora.handle_line(encode_command(1, "FWD", 0.4))
    check("🔴 **الحساسات توقف الروبوت رغم أمر راديو بالتقدم**",
          not res["ok"] and res["ack"] == "SAFE", res["reason"])
    check("والمحركات أُوقفت فعلاً", mission.rover._cmd_lr == (0.0, 0.0),
          str(mission.rover._cmd_lr))
    res = srv.lora.handle_line(encode_command(2, "ESTOP"))
    check("🔴 و ESTOP عبر الراديو يعمل رغم ذلك",
          res["ok"] and mission.state == "estop", mission.state)
    mission.reactive = real_reactive
    srv.manual.set_enabled(False)
    mission.state = "idle"

    st = c.get("/api/lora/status").json()
    check("حالة الراديو تفرّق بين أسباب التعطّل",
          "error" in st and "enabled" in st and "seq" in st,
          str(st.get("error")))

    # ═══ ح) الحالة المبثوثة تحمل كل ما تعرضه الواجهة ══════════════
    section("ح) الحالة المبثوثة:")
    full = c.get("/api/sim/status").json()
    for k in ("ui_mode", "manual", "lora", "battery", "reading", "events",
              "grid", "robot", "readiness"):
        check(f"الحالة تحمل `{k}`", k in full)
    check("والنمط الحالي داخلها (الواجهة لا تخمّن)",
          full["ui_mode"]["mode"] in ("sim", "manual", "auto"),
          full["ui_mode"]["mode_ar"])
    check("ولافتة «المحركات مفعّلة» محسوبة لا مفترضة",
          isinstance(full["ui_mode"]["motors_live"], bool),
          str(full["ui_mode"]["motors_live"]))
    check("🔴 ولا تدّعي «مفعّلة» والجسر في sim (التحذير يفقد معناه)",
          full["ui_mode"]["motors_live"] is False,
          f"rover={full['ui_mode']['rover_mode']}")

    # ═══ ط) توصيل الألترا سونيك الجانبي (§1.2 — كان بلا مسار حقن) ══
    section("ط) توصيل الألترا سونيك الجانبي:")
    boot = c.get("/api/boot").json()
    check("خلاصة الإقلاع تعلن حالة المصفوفة الجانبية بسببها",
          "side_ultrasonic" in boot["health"]
          and boot["health"]["side_ultrasonic"]["error"],
          str(boot["health"]["side_ultrasonic"])[:70])

    r = srv.init_side_ultrasonic(mission, enabled=False)
    check("🔴 العلم مطفأ ⇒ لا بناء ولا حجز منفذ، والسبب معلَن (§6.1)",
          not r["ok"] and r.get("skipped") and "SIDE_ULTRASONIC" in r["reason"],
          r["reason"])

    prev_arr = getattr(mission, "us_array", None)
    r = srv.init_side_ultrasonic(mission, enabled=True)
    check("🔴 العلم مضاء ⇒ تُبنى المصفوفة **وتُحقن** عبر set_side_ultrasonic",
          getattr(mission, "us_array", None) is r.get("array")
          and r.get("array") is not None, str(r.get("error"))[:60])
    check("وعلى ويندوز بلا lgpio: معطّلة بسبب مقروء لا انهيار",
          r["ok"] is False and r.get("error"),
          str(r.get("error"))[:60])
    mission.set_side_ultrasonic(prev_arr)   # إعادة الحال — لا أثر جانبي

    # ═══ ي) قياس أبعاد الغرفة تلقائياً (run_perimeter_cycle) ═══════
    section("ي) قياس أبعاد الغرفة:")
    import time as _t
    r = c.post("/api/room/measure").json()
    check("🔴 زر القياس يبدأ الدورة في الخلفية (لا معالج HTTP معلَّق)",
          r.get("ok") and r.get("started"), str(r))
    for _ in range(40):                     # في المحاكاة يُرفض فوراً
        st = c.get("/api/room/measure/status").json()
        if not st["running"] and st["result"] is not None:
            break
        _t.sleep(0.05)
    check("والنتيجة تصل عبر نقطة الحالة",
          st["result"] is not None, str(st)[:60])
    # في المحاكاة drive_motors=False ⇒ الرفض يأتي من المهمة **بسببها** —
    # إثبات أن `run_perimeter_cycle` الحقيقية هي المستدعاة (قاعدة التوصيل).
    check("🔴 والرفض المبكر بسبب `run_perimeter_cycle` الحقيقية المقروء",
          st["result"].get("ok") is False
          and "قيادة محركات" in str(st["result"].get("reason")),
          str(st["result"].get("reason")))
    html_sim = c.get("/sim").text
    check("وزر «قِس الأبعاد تلقائياً» موجود في إعداد الغرفة",
          'id="btnMeasure"' in html_sim and "room/measure" in html_sim)

    # ═══ ي٢) استشارة الذكاء الاصطناعي (قاعدة §8) ═══════════════════
    section("ي٢) استشارة الذكاء الاصطناعي:")
    import inspect
    html_ai = c.get("/").text
    check("بطاقة الاستشارة موجودة بمزوّدين ومفتاح متصفحي",
          'id="cardAI"' in html_ai and 'id="aiProvider"' in html_ai
          and "localStorage" in html_ai)
    check("🔴 Web Serial: التفريق بين السببين باقٍ (معيار قبول) بلا حجب الزر",
          "insecure" in html_ai and "unsupported" in html_ai
          and 'unsafely-treat-insecure-origin-as-secure' in html_ai
          and '$("#serialBtn").disabled = false;' in html_ai,
          "الزر يبقى متاحاً والسبب يُشرح")
    check("🔴 §8: النداء من المتصفح مباشرة (ترويسة النداء المتصفحي حاضرة)",
          "anthropic-dangerous-direct-browser-access" in html_ai
          and "api.anthropic.com" in html_ai)
    check("والتحليل معلَن «اقتراح فقط» (طبقة 5 — لا تحكم بالروبوت)",
          "لا يتحكم بالروبوت" in html_ai)
    # 🔴 الحارس البنيوي: لا سطر واحد في السيرفر يلمس مفاتيح/مزوّدي AI —
    #    وجود أيٍّ منها هناك يعني أن المفتاح صار يمرّ بالراسبري (كسر §8)
    srv_src = inspect.getsource(srv)
    check("🔴 §8 بنيوياً: صفر أثر لمزوّدي AI أو مفاتيحهم في كود السيرفر",
          all(t not in srv_src for t in
              ("anthropic", "api_key", "generativelanguage", "openai", "gemini")),
          "المفتاح لا يلمس الراسبري")

    # ═══ ك) تشغيلية اللوحة الرئيسية (دفعة ملاحظات 2026-08-06) ══════
    section("ك) تشغيلية اللوحة الرئيسية:")
    html = c.get("/").text
    check("إعداد الغرفة في اللوحة الرئيسية (طول/عرض/تباعد/ركن/معايرة)",
          'id="cardRoom"' in html and 'id="rmCorners"' in html
          and 'id="rmCalib"' in html and 'id="rmMeasure"' in html)
    check("إرشاد «لا غرفة» بدل خريطة سوداء", 'id="mapGuide"' in html)
    check("لافتة الجسر البارزة في القيادة اليدوية + نتيجة كل أمر",
          'id="bridgeBanner"' in html and 'id="cmdResult"' in html)
    check("لوحة المصدر (حالة كشف · Λ · ثقة) + تراكب حراري",
          'id="srcPanel"' in html and "drawHeatmap" in html)
    check("لوحة التقرير بالتصدير CSV والطباعة",
          'id="cardReport"' in html and "/api/mission/csv" in html)

    r = c.post("/api/manual/command", json={"cmd": "STOP"}).json()
    check("🔴 ردّ الأمر اليدوي يحمل وضع الجسر والقيم المرسلة (لا قبول صامتاً على sim)",
          "rover" in r and "mode" in r["rover"] and "cmd_lr" in r["rover"],
          str(r.get("rover"))[:60])

    e = c.get("/api/mission/estimate").json()
    check("الزمن التقديري مع غرفة قائمة: خلايا وثوانٍ وحكم الحاجز",
          e.get("ok") and e.get("survey_s", 0) > 0
          and "fits_time_limit" in e and "battery" in e,
          f"{e.get('cells_free')} خلية ≈ {e.get('survey_s')}ث")

    r = c.get("/api/doc/image/0")
    check("صور التوثيق: 404 بسبب مقروء قبل التوثيق (كانت حبيسة الذاكرة)",
          r.status_code == 404 and "توثيق" in r.text, r.text[:50])

    st = c.get("/api/lora/status").json()
    check("سبب آخر رفض راديو مكشوف (لا رقم عارياً)",
          "last_reject" in st and "echoes" in st)

    # ── 🔴 فشل التسلسل يُقال ولا يُبتلع ────────────────────────────
    # عطل مقاس 2026-08-10: بايتات JPEG في الحمولة جعلت `json.dumps` يرمي
    # **قبل** أي إرسال، و`except: pass` في الحلقة يبتلعه — فبقيت الحلقة
    # حيّة وكل بثّة تفشل بصمت إلى الأبد. الصمت هو ما أطال التشخيص لا العطل.
    section("ف) حدّ التسلسل: الفشل معلَن لا صامت")
    _n_ev = len(mission.events)
    _bad = srv.dumps_or_report({"صورة": b"\xff\xd8\xff"}, "اختبار")
    check("حمولة فيها بايتات ⇒ None (لا إرسال نصف مكتوب ولا استثناء يتسرّب)",
          _bad is None)
    check("والسبب مكتوب في سجل المهمة (يُقرأ من /api/mission/report)",
          any(e["kind"] == "broadcast_error" for e in mission.events[_n_ev:]),
          [e["msg"] for e in mission.events[_n_ev:]
           if e["kind"] == "broadcast_error"][0][:60])
    check("والحمولة السليمة تمرّ كما هي (لا كلفة على المسار الطبيعي)",
          srv.dumps_or_report({"a": 1}, "اختبار") == '{"a": 1}')
    # والحالة الحقيقية تُسلسَل — وجود الحارس لا يعني أن الحمولة سليمة
    check("🔴 حالة المهمة الحقيقية تُسلسَل فعلاً (لا اتّكال على الحارس)",
          srv.dumps_or_report(srv._full_state(include_full_grid=True),
                              "حالة كاملة") is not None)
    check("و/api/sim/status يردّ 200 لا 500",
          c.get("/api/sim/status").status_code == 200)

    # ── 🎥 تسجيل المهمة على الراسبري ──────────────────────────────
    section("ق) تسجيل المهمة:")
    import tempfile as _tf, os as _os2, json as _js2
    from pi.web.recorder import MissionRecorder, list_sessions

    class _RecCam:
        """كاميرا وهمية: تُعيد JPEG صالحاً وتعدّ كم مرة طُلبت."""
        def __init__(self): self.calls = 0
        def snapshot_jpeg(self, *a, **k):
            self.calls += 1
            return b"\xff\xd8" + b"J" * 2048

    _root = _tf.mkdtemp(prefix="rec_test_")
    _cam = _RecCam()
    rec = MissionRecorder(mission, _cam, root=_root)
    check("🔴 التسجيل **معطّل افتراضياً** (لا كتابة على SD بلا طلب)",
          rec.enabled is False and rec.state()["recording"] is False)

    _sv_state, _sv_phase = mission.state, mission.phase
    try:
        rec.set_enabled(True)
        mission.state = "running"
        mission.phase = "survey"
        rec._tick()                       # تفتح الجلسة
        rec._tick()                       # وتلتقط إطاراً
        check("تُفتح جلسة مع بدء المهمة ويُلتقط إطار",
              rec.session is not None and rec.frames >= 1,
              f"{rec.frames} إطار · {_os2.path.basename(rec.session or '')}")
        # 🔴 وأهم خاصية: لا يزاحم الكاميرا في مرحلة التوثيق
        _before = _cam.calls
        mission.phase = "document"
        for _ in range(5):
            rec._tick()
        check("🔴 **لا يلمس الكاميرا في مرحلة التوثيق** (صور التقرير لا تُفقد)",
              _cam.calls == _before and rec.skipped >= 5,
              f"طلبات الكاميرا ثابتة عند {_before} · تُخطّي {rec.skipped}")
        mission.phase = "survey"
        rec._tick()
        check("ويستأنف بعد انتهاء التوثيق", _cam.calls > _before)
        # انتهاء المهمة ⇒ تُغلق الجلسة وتُكتب الملفات
        _sess = rec.session
        mission.state = "done"
        rec._tick()
        check("انتهاء المهمة يُغلق الجلسة ويكتب الملفات",
              rec.session is None and rec.last_saved is not None)
        _files = ["events.jsonl", "readings.csv", "report.json", "summary.txt"]
        _missing = [f for f in _files
                    if not _os2.path.exists(_os2.path.join(_sess, f))]
        check("🔴 الجلسة تحوي السجل والقراءات والتقرير والخلاصة",
              not _missing, "الناقص: " + (", ".join(_missing) or "لا شيء"))
        check("والتقرير المحفوظ JSON صالح يُقرأ بلا السيرفر",
              isinstance(_js2.load(open(_os2.path.join(_sess, "report.json"),
                                        encoding="utf-8")), dict))
        check("والجلسة تظهر في القائمة للمشغّل (يعرف أين ذهب التسجيل)",
              any(s["name"] == _os2.path.basename(_sess)
                  for s in list_sessions(root=_root)))
        # الإطفاء وسط جلسة يحفظها لا يهدرها
        rec.set_enabled(True)
        mission.state = "running"
        rec._tick(); rec._tick()
        _sess2 = rec.session
        rec.set_enabled(False)
        check("🔴 وإطفاء التسجيل وسط جلسة **يحفظها** لا يتركها ناقصة",
              rec.session is None
              and _os2.path.exists(_os2.path.join(_sess2, "summary.txt")))
    finally:
        rec.shutdown()
        mission.state, mission.phase = _sv_state, _sv_phase
        try:
            __import__("shutil").rmtree(_root, ignore_errors=True)
        except Exception:                          # noqa: BLE001
            pass

    st_rec = c.get("/api/sim/status").json()
    check("وحالة التسجيل تُبثّ للواجهة (لا تسجيل صامت)",
          "record" in st_rec and "enabled" in st_rec["record"])
    r_recs = c.get("/api/mission/recordings").json()
    check("ومسار الجلسات مكشوف عبر نقطة صريحة",
          r_recs["ok"] and "sessions" in r_recs)

    boot = c.get("/api/boot").json()
    check("البطارية بمصدرها في خلاصة الإقلاع",
          "battery" in boot["health"] and "source" in boot["health"]["battery"],
          str(boot["health"]["battery"])[:60])

    print(f"\n=== النتيجة: {_passed}/{_passed + _failed} نجح ===")
    if _failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
