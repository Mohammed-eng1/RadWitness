# -*- coding: utf-8 -*-
"""
server.py — سيرفر RMS Rover v2 (FastAPI + WebSocket)
=====================================================
الصفحة الرئيسية `/` = **واجهة محاكاة المسح الداخلي** (الدفعة 2): تقود وحدات
pi/nav حيّاً عبر محرّك mission.py — تعمل بالكامل على ويندوز بلا عتاد.
`/sensors` = واجهة الحساسات الحية (M1، على الراسبري).

كشف المنصة تلقائي: بلا مكتبات عتاد → وضع محاكاة + لافتة واضحة (لا فشل صامت).

التشغيل:
    python -m pi.web.server        # أو: uvicorn pi.web.server:app --port 8000
ثم: http://localhost:8000 (ويندوز) أو http://therover:8000 (الراسبري/Tailscale)
"""
import asyncio
import json
import sys
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from pydantic import BaseModel

from pi.config import (
    WEB_HOST, WEB_PORT, BROADCAST_S, CAPTURES_DIR, BATTERY_MONITOR_ENABLED,
    LORA_ENABLED, LORA_PORT, MISSION_RECORD_DIR,
)
from pi.comms.control import ManualControl, SOURCE_MANUAL, SOURCE_RADIO
from pi.comms.lora import LoRaLink
from pi.web.modes import ModeManager, MODE_MANUAL
from pi.web.recorder import MissionRecorder, list_sessions
from pi.web.stream import mjpeg_frames, resolution_options
from pi.platform_detect import banner as platform_banner
from pi.ai.risk import classify
from pi.nav.mission import MissionSim, default_profile, legacy_low_battery_profile
from pi.nav.calibration import CalibrationStore, compute_speed_mps

# الحساسات الحقيقية (استيرادها آمن على ويندوز — كل مكتبات العتاد محمية داخلها)
from pi.sensors.geiger import GeigerReader
from pi.sensors.gps import GPSReader
from pi.sensors.imu import get_imu
from pi.sensors.camera import CameraReader
from pi.sensors.proximity import UltrasonicReader, IRReader
from pi.rover.bridge import RoverBridge, WaveRoverBridge

class RoverTestReq(BaseModel):
    """جسم اختبار العتاد — نموذج صريح ليعمل على كل إصدارات FastAPI."""
    action: str = "stop"
    seconds: float = 1.0
    degrees: float = 90.0


_STATIC = Path(__file__).parent / "static"


# ═══ إقلاع مُقاس: «يطول ولا يشتغل» يجب أن يصير سطراً معروفاً ══════
# ⚠ كل الأنظمة الفرعية تُبنى **عند استيراد الوحدة**، أي قبل أن تطبع uvicorn
#    حرفاً واحداً. فأي تعثّر (منفذ سيريال مشغول، ناقل I2C صامت، مكتبة ثقيلة)
#    يظهر للمستخدم كشاشة سوداء بلا أي دليل على موضعه. هذه اللافتة تحوّل
#    «يطول» إلى رقم بجانب اسم النظام الذي أخذ الوقت.
_BOOT = []


def _say(msg: str) -> None:
    """
    طباعة **لا تُسقط الإقلاع بترميز الطرفية**.

    ⚠ عطل حقيقي على ويندوز: الطرفية الافتراضية cp1256 لا تُرمّز `⚠` ولا
    `✅`، فكان `print` يرمي `UnicodeEncodeError` **أثناء استيراد الوحدة**
    — أي يموت السيرفر قبل أن يطبع حرفاً، والرسالة الوحيدة الظاهرة هي
    انهيار في سطر طباعة لافتة الإقلاع نفسها. وهذا يكسر وعد المشروع
    «يعمل كاملاً على ويندوز بلا عتاد».
    البديل: تجريد ما يعجز الترميز عنه بدل إسقاط كل شيء.
    """
    try:
        print(msg, flush=True)
    except UnicodeEncodeError:
        enc = (getattr(sys.stdout, "encoding", None) or "ascii")
        print(msg.encode(enc, errors="replace").decode(enc, errors="replace"),
              flush=True)


def _boot(label: str, factory):
    """يبني نظاماً فرعياً ويطبع زمنه فوراً (flush) — لا يُسقط الإقلاع بفشله."""
    t0 = time.time()
    _say(f"[إقلاع] {label} …")
    try:
        obj = factory()
        dt = time.time() - t0
        _BOOT.append({"name": label, "seconds": round(dt, 2), "ok": True})
        _say(f"[إقلاع] {label}: تم في {dt:.2f}ث")
        return obj
    except Exception as e:                     # noqa: BLE001
        dt = time.time() - t0
        _BOOT.append({"name": label, "seconds": round(dt, 2),
                      "ok": False, "error": str(e)})
        _say(f"[إقلاع] {label}: ⚠ فشل بعد {dt:.2f}ث — {e}")
        raise


_T_BOOT = time.time()

# ── محرّك المحاكاة (الدفعة 2) ─────────────────────────────────────
# ⚠ MissionSim أولاً: جسره يفتح السيريال ويطلب **القارئ المشترك** للـBNO055،
#    فيبقى `imu` أدناه نفس النسخة لا نسخة ثانية تتنازع الناقل.
mission = _boot("جسر الروفر + مصدر الاتجاه (MissionSim)", MissionSim)
calib_store = CalibrationStore()
_active_calib = {"name": None}
_sim_clients: set[WebSocket] = set()

# ── الحساسات الحقيقية (M1 — تعمل على الراسبري، خاملة على ويندوز) ──
geiger = _boot("عدّاد جيجر (lgpio BCM17)", GeigerReader)
gps = _boot("GPS", GPSReader)
imu = _boot("وحدة القصور الذاتي (القارئ المشترك)", get_imu)
# 🔴 **اسم الشريحة المكتشَف يُطبع** لا المفترض: العائلة أربع شرائح بنفس
#    السجلات (0x68 MPU-6050 · 0x70 MPU-6500 · 0x71 MPU-9250 · 0x73 MPU-9255)
#    وقد تُستبدل الوحدة دون علم أحد. طباعة المعرّف تُنهي دقائق من الحيرة
#    (وقعت 2026-08-12: وحدة MPU-6500 سليمة رُفضت لأن الكود يقبل 0x68 وحده).
# ⚠ الوحدة تُقرأ من **قارئ MPU نفسه** لا من غلاف BNO055 (`get_imu`):
#    غلافه لا يحمل `chip`/`who_am_i` فكان السطر يطبع `None` مضلّلاً.
try:
    from pi.sensors.mpu6050 import get_mpu as _get_mpu
    _m = _get_mpu()
    _st = _m.state() if _m is not None else {}
    if _st.get("who_am_i"):
        _say(f"[إقلاع] وحدة القصور الذاتي: **{_st.get('chip')}** "
             f"(WHO_AM_I={_st.get('who_am_i')}) على i2c-{_st.get('bus')} "
             f"@ {hex(_st['addr'])}")
    else:
        _say(f"[إقلاع] ⚠ وحدة القصور الذاتي لم تُقرأ: "
             f"{_st.get('error') or 'لا قارئ'}")
except Exception as _e:                 # noqa: BLE001 — لا يُسقط الإقلاع
    _say(f"[إقلاع] ⚠ تعذّر فحص وحدة القصور الذاتي: {_e}")
camera = _boot("الكاميرا (فتحها كسول)", CameraReader)
rover = RoverBridge(mode="sim")
# حساسات القرب الحقيقية + مصدر الإشعاع → محرّك المهمة (المرحلة 2)
ultrasonic = _boot("ألترا سونيك", UltrasonicReader)
ir_sensors = _boot("حسّاسا IR", IRReader)
mission.set_proximity(ultrasonic, ir_sensors)
mission.set_geiger(geiger)
_sensor_clients: set[WebSocket] = set()
_last_sensor_loop = time.time()

# ── بوابة الأوامر اليدوية + وصلة الراديو ─────────────────────────
# 🔴 **بوابة واحدة**: القيادة من الواجهة وأوامر الراديو تمرّان من هنا معاً
#    فوق كائنات السلامة التي تملكها المهمة (`sensors()` و`reactive`).
manual = ManualControl(mission)
lora = _boot(f"وصلة الراديو HC-14 ({LORA_PORT})",
             lambda: LoRaLink(mission, manual))
if LORA_ENABLED:
    lora.start()          # يُعلن سببه إن فشل — لا يُسقط الإقلاع
# الأنماط الثلاثة الحصرية (طبقة واجهة — منطق الملاحة يبقى في mission)
modes = ModeManager(mission, manual, camera)
# 🎥 مسجّل المهمة — **معطّل حتى يطلبه المشغّل**. يقود دورة حياته بنفسه
#    بمراقبة حالة المهمة، فلا سطر تسجيل واحد في `mission.py`.
recorder = MissionRecorder(mission, camera)


def init_side_ultrasonic(mission_obj, enabled: bool = None) -> dict:
    """
    يبني مصفوفة الألترا سونيك (أمامي + جانبان بالتناوب) ويحقنها عبر
    `mission.set_side_ultrasonic` — التوصيل الذي كان مفقوداً (§1.2).

    - العلم مطفأ ⇒ **لا يُحجز منفذ ولا يُبنى شيء** (قاعدة §6.1)، والسبب
      يُعلَن لا يُسكت عنه.
    - العلم مضاء والبناء فشل (لا lgpio / منفذ محجوز) ⇒ الحقن يجري لكن
      المهمة نفسها تبقيه معطّلاً (`array.ok=False`) والسبب في الخلاصة.
    """
    from pi.config import SIDE_ULTRASONIC_ENABLED
    enabled = SIDE_ULTRASONIC_ENABLED if enabled is None else bool(enabled)
    if not enabled:
        return {"ok": False, "skipped": True,
                "reason": "SIDE_ULTRASONIC_ENABLED=False — لا يُحجز منفذ ولا يُقرأ"}
    # استيراد كسول: لا يلمس lgpio إلا حين يُطلب فعلاً
    from pi.sensors.ultrasonic_array import UltrasonicArray
    arr = UltrasonicArray(enabled=True)
    res = mission_obj.set_side_ultrasonic(arr)
    return {"ok": bool(res.get("ok")), "error": arr.error, "array": arr}


_side_us = _boot("مصفوفة الألترا سونيك الجانبية",
                 lambda: init_side_ultrasonic(mission))
if not _side_us.get("ok"):
    _say(f"[إقلاع] ⚠ الألترا سونيك الجانبي غير فاعل: "
         f"{_side_us.get('reason') or _side_us.get('error')}")


def _boot_summary() -> dict:
    """خلاصة الإقلاع: زمن كل نظام وحالته — تُطبع وتُعرض عبر /api/boot."""
    return {
        "total_s": round(time.time() - _T_BOOT, 2),
        "steps": list(_BOOT),
        "health": {
            "geiger": {"ok": geiger.ok, "error": geiger.error},
            "gps": {"ok": gps.ok, "error": gps.error},
            "imu": {"ok": imu.ok, "error": imu.error,
                    "bus": imu.bus_num, "addr": imu.addr,
                    "driver": imu.driver, "mode": imu.mode_name},
            "ultrasonic": {"ok": ultrasonic.ok, "error": ultrasonic.error},
            "ir": {"ok": ir_sensors.ok, "error": ir_sensors.error},
            "rover": {"mode": mission.rover.mode, "error": mission.rover.error,
                      "heading_source": mission.rover.heading_source.name},
            "lora": {"ok": lora.ok, "error": lora.error,
                     "enabled": lora.enabled, "port": lora.port},
            "side_ultrasonic": {
                "ok": bool(_side_us.get("ok")),
                "error": _side_us.get("reason") or _side_us.get("error")},
            # البطارية بمصدرها وسبب غيابه — «جهد مجهول» بلا سبب نصف معلومة
            "battery": {k: v for k, v in mission.rover.battery_state().items()
                        if k in ("v", "source", "source_reason", "text")},
        },
    }


_say(f"[إقلاع] اكتمل تجهيز الأنظمة في "
     f"{time.time() - _T_BOOT:.2f}ث — يبدأ uvicorn الآن")
for _k, _v in _boot_summary()["health"].items():
    if _v.get("error"):
        _say(f"[إقلاع] ⚠ {_k}: {_v['error']}")


# ═══ حلقات الخلفية ═══════════════════════════════════════════════
async def _sim_loop() -> None:
    """يقدّم المحاكاة خطوة كل tick_period ويبثّ الحالة كل ~200ms."""
    last_tick = time.time()
    last_bcast = time.time()
    while True:
        try:
            now = time.time()
            if mission.state == "running" and (now - last_tick) >= mission.tick_period():
                mission.tick()
                last_tick = now
            # 🔴 حارس مهلة الأوامر — **كل دورة لا كل بثّة**: انقطاع أوامر
            #    القيادة اليدوية/الراديو يجب أن يوقف المحركات خلال 0.8/2.0ث،
            #    وتأخيره إلى دورة البثّ (0.2ث) يضيف زمناً بلا داعٍ.
            manual.poll(now)
            if (now - last_bcast) >= 0.2:
                last_bcast = now
                mission.poll_battery()     # الجهد يُعرض دائماً لا أثناء المسح فقط
                mission.poll_ground_echo()  # اشتباه صدى الأرض (إعلان لا معالجة)
                mission.poll_power_clamp() # أحداث الجسر + heartbeat → السجل
                mission.poll_reactive()    # بثّ السرعة وسببها (البند 3)
                if _sim_clients:
                    msg = dumps_or_report(_full_state(include_full_grid=False),
                                          "حالة المهمة")
                    for ws in (list(_sim_clients) if msg else ()):
                        try:
                            await ws.send_text(msg)
                        except Exception:      # noqa: BLE001
                            _sim_clients.discard(ws)
        except Exception:                      # noqa: BLE001 — لا نُسقط الحلقة
            pass
        await asyncio.sleep(0.01)


_bcast_fail_ts = 0.0
_bcast_fails = 0


def dumps_or_report(payload, label: str):
    """
    🔴 تسلسل حمولة البثّ — والفشل **يُقال** لا يُبتلع. أداة خارجية للحلقات.

    عطل مقاس 2026-08-10: بايتات JPEG في الحمولة جعلت `json.dumps` يرمي
    **قبل** أي إرسال، فلا عميل يُسقَط ولا سطر يُكتب — و`except: pass` في
    الحلقة يبتلعه. النتيجة: الحلقة حيّة وكل بثّة تفشل بصمت إلى الأبد، فتبدو
    الواجهة متجمّدة بلا سبب في السجل. **الصمت هو ما جعل تشخيصه يستغرق
    يومين، لا العطل نفسه.**

    ⚠ يُطبع على stdout عمداً: القناة الوحيدة السالمة حين يكون البثّ هو
      المكسور — كتابته في سجل المهمة وحده تعني كتابته حيث لا يصل.
    """
    global _bcast_fail_ts, _bcast_fails
    try:
        return json.dumps(payload)
    except Exception as e:                     # noqa: BLE001
        _bcast_fails += 1
        now = time.time()
        if now - _bcast_fail_ts >= 10.0:       # لا نُغرق الطرفية
            _bcast_fail_ts = now
            print(f"[بثّ] 🔴 تعذّر تسلسل حمولة «{label}» "
                  f"({_bcast_fails} مرة): {type(e).__name__}: {e} — "
                  f"الواجهة لن تتحدّث حتى يُصلَح هذا", flush=True)
        try:
            mission._log("broadcast_error",
                         f"🔴 حمولة «{label}» غير قابلة للتسلسل: "
                         f"{type(e).__name__} — الواجهة متجمّدة والسبب هنا")
        except Exception:                      # noqa: BLE001
            pass
        return None


def _full_state(include_full_grid: bool = False) -> dict:
    """
    حالة المهمة + طبقة الواجهة (النمط · البوابة · الراديو) في رسالة واحدة.

    ⚠ الدمج هنا لا في `mission.state_dict()`: النمط والراديو مفهوما
    **واجهة** لا ملاحة، وحشرهما في محرّك المهمة يخلط الطبقات.
    """
    return {**mission.state_dict(include_full_grid=include_full_grid),
            "ui_mode": modes.state(),
            "manual": manual.state(),
            "lora": lora.state(),
            "record": recorder.state(),
            # حالة الانسحاب للشريط العلوي: «معلّق» قبل أن تخدمه الحلقة،
            # و«جارٍ» حين يصير الطور withdraw. (قراءة فقط — كما تقرأ
            # البوابة `mission._worker` — والمنطق كله يبقى في mission)
            "withdraw": {
                "pending": getattr(mission, "_withdraw_req", None) is not None,
                "active": getattr(mission, "phase", "") == "withdraw",
            }}


def _sensor_telemetry() -> dict:
    g = geiger.state(); p = gps.state(); m = imu.state(); r = rover.state()
    risk = classify(g["usvh"])
    return {
        "t": "telemetry", "ts": int(time.time()),
        "cpm_raw": g["cpm_raw"], "cpm": g["cpm"], "usvh": g["usvh"],
        "high_rate": g["high_rate"], "total": g["total"],
        "geiger_err": g.get("error"), "geiger_samples": g.get("samples", 0),
        "risk": risk["risk"], "lvl": risk["lvl"], "risk_color": risk["color"],
        "fix": p["fix"], "lat": p["lat"], "lng": p["lng"], "sats": p["sats"], "hdop": p["hdop"],
        "heading": m["heading"], "mag_cal": m["mag_cal"], "sys_cal": m["sys_cal"],
        "gyro_cal": m["gyro_cal"], "accel_cal": m["accel_cal"], "mag_warn": m["mag_warn"],
        # وضع BNO055 ومعدل الجايرو — الاتجاه صار من هذا الحسّاس (البند 1)
        "imu_mode": m["mode"], "mag_used": m["mag_used"],
        "gyro_z_dps": m["gyro_z_dps"], "gyro_ready": m["gyro_ready"],
        "heading_source": mission.rover.heading_source.state(),
        "rover": r,
        "health": {"geiger": g["ok"], "gps": p["ok"] and p["fix"], "gps_link": p["ok"],
                   "imu": m["ok"], "camera": camera.state()["available"]},
        "sim": rover.mode == "sim",
    }


async def _sensor_loop() -> None:
    global _last_sensor_loop
    while True:
        try:
            now = time.time(); dt = now - _last_sensor_loop; _last_sensor_loop = now
            geiger.sample(); imu.read()
            p = gps.state()
            if p["fix"]:
                rover.set_home_from_gps(p["lat"], p["lng"])
            rover.update(dt)
            if _sensor_clients:
                msg = dumps_or_report(_sensor_telemetry(), "تيليمتري الحساسات")
                for ws in (list(_sensor_clients) if msg else ()):
                    try:
                        await ws.send_text(msg)
                    except Exception:          # noqa: BLE001
                        _sensor_clients.discard(ws)
        except Exception:                      # noqa: BLE001
            pass
        await asyncio.sleep(BROADCAST_S)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    t1 = asyncio.create_task(_sim_loop())
    t2 = asyncio.create_task(_sensor_loop())
    yield
    t1.cancel(); t2.cancel()
    recorder.shutdown()        # 🎥 جلسة جارية تُحفظ لا تُهدر عند الإغلاق
    try:
        camera.close()         # تحرير الجهاز هنا لا عند كل تبديل نمط
    except Exception:          # noqa: BLE001
        pass
    lora.stop()                # ⚠ يوقف المحركات إن كان الراديو آمرها
    if _lidar["driver"] is not None:
        _lidar["driver"].request_stop(); _lidar["driver"].join(3.0)
    if _lidar["sensor"] is not None:
        _lidar["sensor"].stop()    # 🔴 STOP لمحرك الليدار (لا يقف بغيره)
    manual.set_enabled(False)  # ولا تُترك عجلة تدور عند إغلاق السيرفر
    geiger.close(); gps.close(); camera.close()
    ultrasonic.close(); ir_sensors.close()
    if _side_us.get("array") is not None:
        _side_us["array"].close()   # يحرّر منافذ الجانبيين إن حُجزت


# ⚠ حقن الكاميرا في المهمة: التوثيق البصري بعد المسح يحتاجها، وبلا هذا
#   السطر يعمل كل شيء **عدا التقاط الصور** بلا أي رسالة خطأ.
mission.camera = camera

app = FastAPI(title="RMS Rover v2", lifespan=lifespan)


def _page(name: str) -> HTMLResponse:
    return HTMLResponse((_STATIC / name).read_text(encoding="utf-8"))


def _dir_to_cmd(d: str) -> str:
    """يحوّل اتجاه الواجهة القديم (F/B/L/R/S) إلى القائمة المغلقة."""
    return {"F": "FWD", "B": "BACK", "L": "LEFT", "R": "RIGHT",
            "S": "STOP"}.get(str(d).upper()[:1], "STOP")


# ═══ الصفحات ═════════════════════════════════════════════════════
@app.get("/")
async def index():
    """لوحة التحكم الموحّدة — الأنماط الثلاثة بمبدّل واحد."""
    return _page("control.html")


@app.get("/sim")
async def sim_page():
    """
    الواجهة التفصيلية السابقة — **مُبقاة عمداً** لا مهجورة: فيها ضبط
    الغرفة والمعايرة واختبارات العتاد التي لا مكان لها في لوحة تشغيل
    مبسّطة. اللوحة الجديدة للتشغيل، وهذه للإعداد والتشخيص.
    """
    return _page("sim.html")


@app.get("/sensors")
async def sensors_page():
    return _page("index.html")


# ═══ REST — المحاكاة ═════════════════════════════════════════════
@app.get("/api/platform")
async def api_platform():
    return platform_banner()


@app.get("/api/boot")
async def api_boot():
    """زمن إقلاع كل نظام فرعي وحالته — لتشخيص «السيرفر يطول ولا يشتغل»."""
    return _boot_summary()


@app.get("/api/imu/health")
def api_imu_health():
    """
    حالة شريحة BNO055 كما تقرأها **هي** (CHIP_ID/OPR_MODE/SYS_STAT/SYS_ERR).
    تفرّق بين «الحسّاس مفقود» و«عاد إلى CONFIG فيقرأ أصفاراً وهو حيّ».
    """
    return {"reader": imu.state(), "chip": imu.health(),
            "heading_source": mission.rover.heading_source.state()}


@app.post("/api/imu/recover")
def api_imu_recover():
    """إحياء يدوي للحسّاس + إعادة تسليح مصدر الاتجاه (زر «أعِد المحاولة»)."""
    src = mission.rover.heading_source
    src.reset_recovery_budget()        # طلب صريح من المستخدم → رصيد جديد
    res = src.attempt_recovery()
    mission._log("heading_recover",
                 ("✅ أُحيي مصدر الاتجاه: " if res.get("recovered")
                  else "⚠ تعذّر الإحياء: ") + str(res.get("detail")))
    return {"ok": bool(res.get("recovered")), **res,
            "heading_source": src.state()}


@app.post("/api/room")
async def api_room(req: Request):
    d = await req.json()
    try:
        src = None
        if d.get("source_x") is not None and d.get("source_y") is not None:
            src = (float(d["source_x"]), float(d["source_y"]))
        mission.configure_room(
            length_m=d["length_m"], width_m=d["width_m"],
            start_corner=d.get("start_corner", "back_left"),
            scan_spacing_m=d.get("scan_spacing_m", 0.5), source_xy=src)
        return {"ok": True, "grid": mission.grid_meta()}
    except (KeyError, ValueError) as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=400)


# ═══ قياس أبعاد الغرفة تلقائياً (دورة المحيط — §1.2) ═════════════
# 🔴 الدورة **تقود المحركات وقد تستغرق دقائق** (سقفها PERIMETER_MAX_CYCLE_S)
#    فلا تُشغَّل داخل معالج HTTP — خيط خلفي واحد ونقطة حالة تُستطلع.
_perimeter_run = {"thread": None, "result": None, "started_ts": None}


def _perimeter_worker() -> None:
    try:
        _perimeter_run["result"] = mission.run_perimeter_cycle()
    except Exception as e:                     # noqa: BLE001 — §6.3
        _perimeter_run["result"] = {"ok": False, "reason": f"عطل غير متوقّع: {e}"}


@app.post("/api/room/measure")
async def api_room_measure():
    """
    يبدأ دورة قياس المحيط في الخلفية. الرفض المبكر (بلا محركات/مصفوفة)
    يعود فوراً بسببه من `run_perimeter_cycle` نفسها — لا ازدواج شروط هنا.
    """
    t = _perimeter_run["thread"]
    if t is not None and t.is_alive():
        return JSONResponse({"ok": False, "running": True,
                             "error": "دورة قياس جارية — انتظر نتيجتها"},
                            status_code=409)
    _perimeter_run["result"] = None
    _perimeter_run["started_ts"] = time.time()
    th = threading.Thread(target=_perimeter_worker, daemon=True,
                          name="perimeter-measure")
    _perimeter_run["thread"] = th
    th.start()
    return {"ok": True, "started": True}


@app.get("/api/room/measure/status")
async def api_room_measure_status():
    """حالة القياس: جارٍ · نتيجة (بمقارنة المُدخل إن وُجدت غرفة) · لم يبدأ."""
    t = _perimeter_run["thread"]
    return {"running": t is not None and t.is_alive(),
            "result": _perimeter_run["result"],
            "started_ts": _perimeter_run["started_ts"]}


@app.get("/api/calibration")
async def api_calib_list():
    return {"names": calib_store.list_names(), "active": _active_calib["name"]}


@app.post("/api/calibration/new_sim")
async def api_calib_new_sim():
    """
    ينشئ ملف المعايرة الحالي (**بطارية ممتلئة**) ويجعله النشط، ويحفظ بجانبه
    الملف القديم **موسوماً بجهده المنخفض** ليظهر الفارق في القائمة.
    """
    legacy = legacy_low_battery_profile()
    if legacy.name not in calib_store.list_names():
        calib_store.save(legacy)
    # المنصّة الفعلية تختار الملف: على UGV01 قيم Wave Rover تكذب كل مسافة
    prof = default_profile(battery_v=mission.rover.voltage() or 0.0)
    calib_store.save(prof)
    mission.set_calibration(prof)
    _active_calib["name"] = prof.name
    return {"ok": True, "name": prof.name, "battery_v": prof.battery_v,
            "legacy": legacy.name, "legacy_battery_v": legacy.battery_v}


@app.post("/api/calibration/speed")
async def api_calib_speed(req: Request):
    """
    معايرة سرعة حقيقية: الروبوت سار `duration_s` عند `power`، والمستخدم قاس
    `distance_m` بشريط قياس → م/ث تُحفظ في الملف النشط مع جهد البطارية.
    """
    d = await req.json()
    if mission.profile is None:
        return JSONResponse({"ok": False, "error": "اختر ملف معايرة أولاً"}, status_code=400)
    try:
        power = float(d["power"])
        speed = compute_speed_mps(float(d["distance_m"]), float(d["duration_s"]))
    except (KeyError, ValueError, ZeroDivisionError) as e:
        return JSONResponse({"ok": False, "error": f"مدخلات غير صالحة: {e}"}, status_code=400)
    key = str(int(power * 100)) if power <= 1 else str(int(power))
    mission.profile.speeds[key] = round(speed, 3)
    mission.profile.battery_v = mission.rover.voltage() or mission.profile.battery_v
    mission.profile.date = time.strftime("%Y-%m-%d %H:%M")
    calib_store.save(mission.profile)
    return {"ok": True, "power_key": key, "speed_mps": round(speed, 3),
            "battery_v": mission.profile.battery_v}


@app.post("/api/rover/calibrate_gyro")
def api_calib_gyro():
    """قياس انحياز الجايرو والروبوت ساكن (لا يُثبَّت في الكود — أ-2)."""
    bias = mission.rover.calibrate_gyro_bias()
    mission._log("gyro_bias", f"معايرة انحياز الجايرو: {bias:.3f}")
    return {"ok": True, "gyro_bias": round(bias, 4)}


@app.post("/api/rover/mode")
async def api_rover_mode(req: Request):
    """
    تبديل جسر الروفر بين `sim` و`real` وقت التشغيل (العلم الواحد).
    عند تعذّر فتح المنفذ يسقط إلى sim ويُعيد سبب الفشل — لا فشل صامت.
    """
    d = await req.json()
    mode = str(d.get("mode", "sim")).lower()
    if mode not in ("sim", "real"):
        return JSONResponse({"ok": False, "error": "الوضع يجب أن يكون sim أو real"},
                            status_code=400)
    # ⚠⚠ **لا تبديل والمهمة جارية** ⚠⚠
    # `close()` يقفل منفذ السيريال بينما خيط المحركات يكتب عليه، فيموت بـ
    # `write failed: [Errno 9] Bad file descriptor` وسط الحركة (شوهد على
    # العتاد 2026-08-01). والأسوأ أن `DriveExecutor` يحتفظ بمرجع **الجسر
    # القديم** فيظل يكتب على منفذ مغلق حتى لو نجح التبديل.
    if mission.state in ("running", "paused") or (
            mission._worker is not None and mission._worker.is_alive()):
        return JSONResponse(
            {"ok": False, "rover_mode": mission.rover.mode,
             "error": "المهمة جارية — أوقفها (إيقاف طوارئ) قبل تبديل وضع "
                      "الروفر. التبديل يقفل منفذ السيريال تحت خيط المحركات."},
            status_code=409)
    try:
        mission.rover.close()
    except Exception:                          # noqa: BLE001
        pass
    mission.rover = WaveRoverBridge(mode=mode)
    mission.executor = None        # المنفّذ يحمل مرجع الجسر القديم — أبطِله
    if mission.rover.mode != "real":
        mission.drive_motors = False   # لا تُبقِ قيادة محركات على جسر sim
    mission._log("rover_mode",
                 f"وضع الروفر: {mission.rover.mode}"
                 + (f" ⚠ {mission.rover.error}" if mission.rover.error else ""))
    return {"ok": True, "mode": mission.rover.mode, "error": mission.rover.error}


@app.post("/api/rover/test")
def api_rover_test(body: RoverTestReq):
    """
    اختبارات حركة **يدوية قصيرة** (بنود القائمة 8-9) — للتحقق من اتجاه
    الحركة ودقة اللفّ على الأرض. كل اختبار قصير ويُنهى بـstop().
    ⚠ يحرّك المحركات فعلياً في وضع real — أبقِ يدك على إيقاف الطوارئ.

    الجسم عبر نموذج Pydantic صريح (لا `dict` — سلوكه يختلف بين إصدارات
    FastAPI فيصل الوسيط مشوّهاً). و`def` لا `async def` كي تعمل النداءات
    الحاجبة (sleep/turn) في threadpool بلا تجميد حلقة البثّ.
    """
    action = body.action
    rv = mission.rover
    try:
        if action == "stop":
            rv.stop()
            return {"ok": True, "action": "stop"}
        if action == "forward":
            rv.forward()                        # DRIVE_POWER_DEFAULT (0.40)
            sent = rv._cmd_lr                   # القيم المُرسلة فعلاً (قبل الإيقاف)
            time.sleep(body.seconds)
            rv.stop()
            return {"ok": True, "action": "forward", "cmd": list(sent)}
        if action == "turn90":
            res = rv.turn_by_angle(body.degrees)
            return {"ok": True, "action": "turn90", **res}
        if action == "calibrate_gyro":
            bias = rv.calibrate_gyro_bias()
            return {"ok": True, "gyro_bias": round(bias, 4), "info": rv.bias_info}
        return JSONResponse({"ok": False, "error": "أمر غير معروف"}, status_code=400)
    except Exception as e:                      # noqa: BLE001
        rv.stop()                               # ⚠ أي استثناء → إيقاف
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


@app.get("/api/rover/status")
def api_rover_status():
    """
    ردّ `T=130` **خاماً** (تشخيص وصلة السيريال والفيرموير).

    ⚠ الحقل `v` في ردّ الروفر **صفر دائماً** (ناقل I2C الداخلي في اللوحة
       معطّل فيزيائياً) — الجهد الفعلي من INA219 على الراسبري (0x42، ناقل 1)
       عبر `battery_state()` والمصدر يُعلَن دائماً. وتبقى فوقه طبقتان لا
       تحتاجان فولتميتر: الحدّ الزمني وذروة معدل الدوران.
    """
    st = mission.rover.read_status()
    rv = mission.rover
    return {
        "raw": st, "mode": rv.mode,
        "link_ok": rv.link_ok, "link_error": rv.link_error,
        # علق إقلاع ESP32 (تدفق أصفار) — العلاج زرّ Reset لا إعادة محاولة
        "esp32_stuck": rv.esp32_stuck,
        "voltage_source": rv.voltage_source,
        "battery_monitor_enabled": BATTERY_MONITOR_ENABLED,
        # الحماية البديلة الفعلية
        "turn_peak_baseline_dps": rv.turn_peak_baseline,
        "last_turn_peak_dps": rv.last_turn_peak,
        "time_limit": mission.time_limit_info(),
    }


@app.get("/api/sensors/check")
def api_sensors_check():
    """فحص حسّاسات القرب (يكشف حسّاساً غير موصول قبل تشغيل المحركات)."""
    return mission.preflight_check()


@app.post("/api/mission/drive_mode")
async def api_drive_mode(req: Request):
    """تبديل بين المسح المنطقي وقيادة المحركات فعلياً (المرحلة 2)."""
    d = await req.json()
    return mission.set_drive_motors(bool(d.get("motors", False)),
                                    allow_sim=bool(d.get("allow_sim", False)))


@app.post("/api/mission/record")
async def api_mission_record(req: Request):
    """
    🎥 تفعيل/إيقاف تسجيل المهمة كاملةً على الراسبري.

    ⚠ مُسجَّل قبل مسار `/api/mission/{action}` العام عمداً — وإلا ابتلعه.
    """
    d = await req.json()
    return recorder.set_enabled(bool(d.get("enabled", False)))


@app.get("/api/mission/recordings")
async def api_mission_recordings():
    """الجلسات المحفوظة على القرص — ليعرف المشغّل أين ذهب التسجيل."""
    return {"ok": True, "root": MISSION_RECORD_DIR,
            "sessions": list_sessions()}


@app.post("/api/mission/batt_rth_override")
async def api_batt_rth_override(req: Request):
    """
    رفع/إعادة **العودة الإجبارية بسبب الجهد** وحدها (10.0–10.8V).

    🔴 لا يمسّ الإيقاف الفوري دون الأرضية، ولا الإطفاء المنظَّم، ولا
    الحاجز الزمني. الشرح الكامل في `MissionSim.set_batt_rth_override`.
    ⚠ مُسجَّل قبل مسار `/api/mission/{action}` العام عمداً — وإلا ابتلعه.
    """
    d = await req.json()
    return mission.set_batt_rth_override(bool(d.get("enabled", False)))


@app.post("/api/mission/nav_override")
async def api_nav_override(req: Request):
    """
    خيارا القيادة الذاتية اليدويان: تجاهل حساسات العوائق · قوة المحركات.

    🔴 خيار خطر بثمن معلَن — الشرح الكامل في `MissionSim.set_nav_override`،
    ويُرفض أثناء جريان المهمة (تبديل عقد السلامة في المنتصف).
    ⚠ مُسجَّل قبل مسار `/api/mission/{action}` العام عمداً — وإلا ابتلعه.
    """
    d = await req.json()
    return mission.set_nav_override(
        ignore_obstacles=d.get("ignore_obstacles"), power=d.get("power"))


@app.post("/api/mission/training_source")
async def api_training_source(req: Request):
    """
    🎯 مصدر تدريبي افتراضي فوق قيادة حقيقية (بروفة الاختبار): العدّ وحده
    يُصنَّع من التربيع العكسي، والقيادة والسلامة حقيقيتان. معلَن في السجل
    وكل بثّ حالة، ويزول بإعادة تشغيل السيرفر. `{"clear": true}` يمسحه.
    ⚠ مُسجَّل قبل مسار `/api/mission/{action}` العام عمداً — وإلا ابتلعه.
    """
    d = await req.json()
    if d.get("clear"):
        return mission.set_training_source()
    return mission.set_training_source(d.get("x"), d.get("y"), d.get("usvh_1m"))


@app.post("/api/sim/battery")
async def api_sim_battery(req: Request):
    """
    أداة اختبار: ضبط جهد البطارية الوهمي لتجربة العتبات (RTH/إيقاف).

    ⚠ التجاوز يتقدّم على قراءة INA219 الحقيقية عمداً — ولهذا يجب أن يكون
    **الرجوع عنه ممكناً**: `{"clear": true}` يُعيد المصدر إلى العتاد. وبلا
    هذا يبقى النظام على جهد وهمي بعد الاختبار ويظنّه حقيقياً.
    """
    d = await req.json()
    if d.get("clear"):
        mission.rover.sim_clear_voltage_override()
    else:
        mission.rover.sim_set_voltage(float(d["v"]))
    mission._rth_triggered = False
    mission._batt_source = None            # يُعاد تسجيل المصدر بعد التبديل
    return {"ok": True, "battery": mission.rover.battery_state()}


@app.post("/api/calibration/select")
async def api_calib_select(req: Request):
    d = await req.json()
    try:
        prof = calib_store.load(d["name"])
        mission.set_calibration(prof)
        _active_calib["name"] = prof.name
        return {"ok": True, "name": prof.name}
    except Exception as e:                     # noqa: BLE001
        return JSONResponse({"ok": False, "error": str(e)}, status_code=400)


@app.post("/api/mission/{action}")
async def api_mission(action: str, req: Request):
    if action == "start":
        return mission.start()
    if action == "pause":
        mission.pause(); return {"ok": True}
    if action == "resume":
        mission.resume(); return {"ok": True}
    if action == "estop":
        mission.estop(); return {"ok": True}
    if action == "return_home":
        mission.return_home(); return {"ok": True}
    if action == "speed":
        d = await req.json(); mission.set_speed(d.get("mult", 1)); return {"ok": True}
    return JSONResponse({"ok": False, "error": "أمر غير معروف"}, status_code=400)


# ═══ الأنماط الثلاثة ═════════════════════════════════════════════
@app.get("/api/mode")
async def api_mode_get():
    return modes.state()


@app.post("/api/mode")
async def api_mode_set(req: Request):
    """
    تبديل النمط. مهمة جارية ⇒ يُعيد `needs_confirm` فتسأل الواجهة، ثم
    يُعاد النداء بـ`confirm: true` فيُنفَّذ **إيقافاً مؤقتاً لا إلغاء**.
    """
    d = await req.json()
    res = modes.switch(str(d.get("mode", "")), confirm=bool(d.get("confirm")))
    if not res.get("ok") and not res.get("needs_confirm"):
        return JSONResponse(res, status_code=400)
    return res


# ═══ القيادة اليدوية والراديو (بوابة واحدة) ══════════════════════
class ManualCmdReq(BaseModel):
    """أمر قيادة يدوية — نموذج صريح ليعمل على كل إصدارات FastAPI."""
    cmd: str = "STOP"
    power: float = None


@app.post("/api/manual/command")
def api_manual_command(body: ManualCmdReq):
    """
    🔴 **المسار الوحيد** للقيادة اليدوية من الواجهة — نفس بوابة الراديو.
    طبقة السلامة تسبق الأمر: عائق أمامي ⇒ رفض التقدّم وإيقاف المحركات.

    `def` لا `async def`: النداء يكتب على السيريال (حاجب) فيعمل في
    threadpool بلا تجميد حلقة البثّ.
    """
    res = manual.command(body.cmd, power=body.power, source=SOURCE_MANUAL)
    # 🔴 مع كل ردّ: وضع الجسر والقيم المرسلة فعلاً — عطل مقاس (2026-08-06):
    #    السيرفر أُقلع بلا RMS_ROVER_MODE=real فقبل كل أمر «بنجاح» وحرّك
    #    روبوتاً وهمياً، والمستخدم أمام روبوت ساكن بلا أي رسالة. القبول
    #    الصامت على جسر sim أسوأ من الرفض الصريح.
    rover = mission.rover
    res["rover"] = {"mode": rover.mode, "error": rover.error,
                    "cmd_lr": list(getattr(rover, "_cmd_lr", (0.0, 0.0))),
                    "link_ok": getattr(rover, "link_ok", None)}
    return res


@app.post("/api/manual/enable")
async def api_manual_enable(req: Request):
    """يفتح/يغلق نمط القيادة اليدوية (الإغلاق يوقف المحركات دائماً)."""
    d = await req.json()
    return manual.set_enabled(bool(d.get("on", False)))


@app.post("/api/manual/ignore_sensors")
async def api_manual_ignore(req: Request):
    """
    🔴 تجاوز حساسات العوائق — **قيادة يدوية فقط وبطلب صريح**: المشغّل يقود
    بالكاميرا ويتحمّل المسؤولية. heartbeat وحدّ القوة وESTOP لا يتأثرون،
    ويُصفَّر تلقائياً عند إغلاق القيادة اليدوية (لا يورَّث لوضع آخر).
    """
    d = await req.json()
    manual.ignore_sensors = bool(d.get("on", False))
    return {"ok": True, "ignore_sensors": manual.ignore_sensors}


# ═══ قيادة ذاتية بالليدار (نسخة اختبار — pi/nav/lidar_drive.py) ════════
# 🔴 لا تُمنع أبداً (قرار المشغّل): كل مشكلة تحذير في الحالة. آمر واحد على
#    السيريال: البدء يغلق اليدوية ويوقف المسح مؤقتاً إن كان جارياً.
_lidar = {"sensor": None, "driver": None, "log": []}


def _lidar_echo(msg: str) -> None:
    _lidar["log"] = (_lidar["log"] + [f"{time.strftime('%H:%M:%S')} {msg}"])[-30:]


class LidarDriveReq(BaseModel):
    max_speed: float = None
    dry_run: bool = False


@app.post("/api/lidar_drive/start")
def api_lidar_drive_start(body: LidarDriveReq):
    from pi.config import LIDAR_MAX_SPEED
    from pi.nav.lidar_drive import LidarDriver
    from pi.sensors.lidar_c1 import LidarC1
    drv = _lidar["driver"]
    if drv is not None and drv.running:
        return {"ok": True, "already": True, "state": drv.state()}
    manual.set_enabled(False)
    if mission.state in ("running", "returning"):
        mission.pause()
        _lidar_echo("⚠ المسح الجاري أُوقف مؤقتاً (آمر واحد على السيريال)")
    if _lidar["sensor"] is None:
        _lidar["sensor"] = LidarC1().start()
    drv = LidarDriver(mission.rover, _lidar["sensor"],
                      body.max_speed or LIDAR_MAX_SPEED,
                      dry_run=bool(body.dry_run), echo=_lidar_echo)
    _lidar["driver"] = drv
    drv.start_thread()
    return {"ok": True, "state": drv.state()}


@app.post("/api/lidar_drive/stop")
def api_lidar_drive_stop():
    drv = _lidar["driver"]
    if drv is not None:
        drv.request_stop()
        drv.join(3.0)
    mission.rover.stop()                 # ⚠ صفر مضمون حتى لو علق الخيط
    return {"ok": True, "state": drv.state() if drv else None}


@app.get("/api/lidar_drive/status")
def api_lidar_drive_status():
    drv = _lidar["driver"]
    sen = _lidar["sensor"]
    return {"state": drv.state() if drv else None,
            "lidar": sen.state() if sen else None, "log": _lidar["log"][-12:]}


@app.get("/api/manual/status")
async def api_manual_status():
    return {"manual": manual.state(), "lora": lora.state()}


@app.get("/api/lora/status")
async def api_lora_status():
    """
    حالة وصلة الراديو — **تفرّق بين أسباب التعطّل** (معطّل بالإعداد ·
    pyserial غائبة · منفذ مفقود) فيظهر السبب في الواجهة لا «لا يعمل».
    """
    return lora.state()


@app.post("/api/lora/start")
async def api_lora_start():
    """محاولة فتح الوصلة يدوياً (زر «أعِد المحاولة» في الواجهة)."""
    lora.enabled = True
    return lora.start()


@app.post("/api/lora/stop")
async def api_lora_stop():
    lora.stop()
    lora.enabled = False
    return lora.state()


@app.post("/api/obstacle")
async def api_obstacle(req: Request):
    d = await req.json()
    return mission.toggle_obstacle(int(d["row"]), int(d["col"]))


@app.get("/api/mission/estimate")
async def api_mission_estimate():
    """
    زمن المهمة التقديري **قبل البدء**: خلايا حرّة × (توقف + حركة)، مقارناً
    بالحاجز الزمني والبطارية — المستخدم يعرف قبل الضغط هل ستكتمل.

    ⚠ تقدير صريح الحدود: لا يشمل مراحل التأكيد/الاقتراب (تعتمد على ما
    يُكتشف) — يُقال ذلك في الرد لا يُخفى.
    """
    from pi.config import CELL_DWELL_S, MISSION_TIME_LIMIT_S, DRIVE_POWER_DEFAULT
    if mission.grid is None:
        return {"ok": False, "reason": "عرّف الغرفة أولاً"}
    if mission.profile is None:
        return {"ok": False, "reason": "لا ملف معايرة — السرعة مجهولة"}
    counts = mission.grid.counts()
    free = int(counts.get("total", 0)) - int(counts.get("blocked", 0))
    spacing = float(getattr(mission.room, "scan_spacing_m", 0.5) or 0.5)
    speed = float(mission.profile.speed_for_power(DRIVE_POWER_DEFAULT) or 0.3)
    move_s = spacing / max(speed, 0.05)
    survey_s = free * (CELL_DWELL_S + move_s)
    batt = mission.rover.battery_state()
    return {
        "ok": True,
        "cells_free": free,
        "dwell_s": CELL_DWELL_S,
        "move_s_per_cell": round(move_s, 2),
        "survey_s": round(survey_s),
        "time_limit_s": MISSION_TIME_LIMIT_S,
        "fits_time_limit": survey_s <= MISSION_TIME_LIMIT_S,
        "battery": {"v": batt.get("v"), "percent": batt.get("percent"),
                    "text": batt.get("text"), "source": batt.get("source")},
        "note": "التقدير للمسح وحده — التأكيد والاقتراب يعتمدان على ما يُكتشف",
    }


@app.get("/api/doc/image/{idx}")
async def api_doc_image(idx: int):
    """صور التوثيق الثلاث — كانت بايتات حبيسة الذاكرة بلا أي مسار يقدّمها."""
    d = mission.documentation or {}
    imgs = d.get("images") or []
    if not imgs:
        return JSONResponse({"ok": False, "error": "لا صور توثيق بعد — "
                             "تُلتقط في مرحلة التوثيق آخر الدورة"},
                            status_code=404)
    if not 0 <= idx < len(imgs):
        return JSONResponse({"ok": False,
                             "error": f"الفهرس {idx} خارج المدى 0..{len(imgs)-1}"},
                            status_code=404)
    return Response(content=imgs[idx], media_type="image/jpeg")


@app.get("/api/mission/csv")
async def api_mission_csv():
    return Response(content=mission.csv_bytes(), media_type="text/csv",
                    headers={"Content-Disposition": "attachment; filename=mission.csv"})


@app.get("/api/mission/report")
async def api_mission_report():
    return mission.report()


@app.get("/api/sim/status")
async def api_sim_status():
    return _full_state(include_full_grid=True)


@app.websocket("/ws")
async def ws_sim(ws: WebSocket):
    await ws.accept()
    _sim_clients.add(ws)
    try:
        # ⚠ المصافحة أثقل حمولة (الشبكة كاملة) — وفشلها هنا يقتل الاتصال
        #   قبل أول إطار، فيُعيد المتصفح الوصل بلا توقف بلا أي تفسير.
        _hello = dumps_or_report({**platform_banner(),
                                  **_full_state(include_full_grid=True)},
                                 "مصافحة /ws")
        if _hello is None:
            _hello = json.dumps({"t": "sim", "state": "broadcast_error",
                                 "error": "تعذّر تسلسل حالة المهمة — راجع "
                                          "طرفية السيرفر (broadcast_error)"})
        await ws.send_text(_hello)
        while True:
            await ws.receive_text()            # الأوامر عبر REST؛ نبقي الاتصال حيّاً
    except WebSocketDisconnect:
        pass
    except Exception:                          # noqa: BLE001
        pass
    finally:
        _sim_clients.discard(ws)


# ═══ REST/WS — الحساسات (M1) ═════════════════════════════════════
@app.get("/snapshot.jpg")
def snapshot() -> Response:
    data = camera.snapshot_jpeg()
    if data is None:
        return Response(status_code=503, content="camera unavailable")
    return Response(content=data, media_type="image/jpeg")


@app.get("/stream.mjpg")
def stream(res: str = None) -> Response:
    """
    بثّ MJPEG **في كل الأنماط** — ويتوقف مؤقتاً في مرحلة التوثيق وحدها.

    🔴 كان مقصوراً على نمط القيادة اليدوية، فكان يُرفض بـ409 بعد انتهاء أي
    مهمة ذاتية (مقاس 2026-08-11: «الكاميرا ما تشتغل» عند شرح الروبوت بعد
    المهمة). والقيد لم يكن له مبرّر أمني: الخطر الحقيقي هو مزاحمة
    **الصور التوثيقية** — و`snapshot_jpeg` يرفض لقطة ثانية أثناء لقطة
    جارية، فبثّ أثناء التوثيق كان سيُفقد صور التقرير.

    ⇒ المنع صار **على المرحلة لا على النمط**، وهو **توقّف مؤقت لا قطع**:
      الاتصال يبقى مفتوحاً والبثّ يستأنف فور انتهاء التوثيق — بدل قطعٍ
      يتبعه سيل إعادة اتصال من المتصفح.

    ⚠ وسبب التعذّر **مقروء** لا 503 صامتة.
    """
    if not camera.state()["available"]:
        return Response(status_code=503,
                        content=f"الكاميرا غير متاحة: {camera.error or 'سبب غير معروف'}",
                        media_type="text/plain; charset=utf-8")
    return StreamingResponse(
        mjpeg_frames(camera, res,
                     pause_fn=lambda: getattr(mission, "phase", "") == "document"),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/api/camera/options")
async def api_camera_options():
    """الدقّات المتاحة + حالة الكاميرا (وسبب تعذّرها إن وُجد)."""
    return {**resolution_options(), "camera": camera.state(),
            "error": camera.error, "streaming": modes.camera_streaming}


@app.post("/api/snapshot/save")
def save_snapshot() -> JSONResponse:
    name = camera.save_snapshot(CAPTURES_DIR)
    if not name:
        return JSONResponse({"ok": False, "error": "camera unavailable"}, status_code=503)
    return JSONResponse({"ok": True, "file": name})


@app.get("/api/status")
async def api_status() -> dict:
    return _sensor_telemetry()


@app.websocket("/ws/sensors")
async def ws_sensors(ws: WebSocket) -> None:
    await ws.accept()
    _sensor_clients.add(ws)
    try:
        await ws.send_text(json.dumps(_sensor_telemetry()))
        while True:
            raw = await ws.receive_text()
            try:
                cmd = json.loads(raw)
                # ⚠⚠ **أُزيل مسار تجاوز طبقة السلامة** ⚠⚠
                # كان هنا `rover.command(...)` يقود **نسخة جسر ثانية**
                # (`RoverBridge(mode="sim")`) لا جسر المهمة — بلا فحص
                # حساسات ولا heartbeat ولا حدّ قوة. صار كل شيء يمرّ
                # ببوابة `manual` الواحدة.
                if cmd.get("c") == "drive":
                    manual.command(_dir_to_cmd(cmd.get("dir", "S")),
                                   power=cmd.get("power"),
                                   source=SOURCE_MANUAL)
                elif cmd.get("c") == "estop":
                    manual.command("ESTOP", source=SOURCE_MANUAL)
            except Exception:                  # noqa: BLE001
                continue
    except WebSocketDisconnect:
        pass
    except Exception:                          # noqa: BLE001
        pass
    finally:
        _sensor_clients.discard(ws)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=WEB_HOST, port=WEB_PORT)
