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

    c.post("/api/mode", json={"mode": "sim", "confirm": True})
    r = c.post("/api/manual/command", json={"cmd": "FWD"}).json()
    check("🔴 وأمر حركة خارج نمط القيادة اليدوية يُرفض (حصرية)",
          not r["ok"] and r["ack"] == "BUSY", r["reason"])

    st = c.get("/api/manual/status").json()
    check("حالة البوابة والراديو مكشوفة للواجهة",
          "manual" in st and "lora" in st)

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

    print(f"\n=== النتيجة: {_passed}/{_passed + _failed} نجح ===")
    if _failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
