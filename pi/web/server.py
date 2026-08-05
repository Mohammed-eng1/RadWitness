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
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from pydantic import BaseModel

from pi.config import (
    WEB_HOST, WEB_PORT, BROADCAST_S, CAPTURES_DIR, BATTERY_MONITOR_ENABLED,
)
from pi.platform_detect import banner as platform_banner
from pi.ai.risk import classify
from pi.nav.mission import MissionSim, default_sim_profile, legacy_low_battery_profile
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


def _boot(label: str, factory):
    """يبني نظاماً فرعياً ويطبع زمنه فوراً (flush) — لا يُسقط الإقلاع بفشله."""
    t0 = time.time()
    print(f"[إقلاع] {label} …", flush=True)
    try:
        obj = factory()
        dt = time.time() - t0
        _BOOT.append({"name": label, "seconds": round(dt, 2), "ok": True})
        print(f"[إقلاع] {label}: تم في {dt:.2f}ث", flush=True)
        return obj
    except Exception as e:                     # noqa: BLE001
        dt = time.time() - t0
        _BOOT.append({"name": label, "seconds": round(dt, 2),
                      "ok": False, "error": str(e)})
        print(f"[إقلاع] {label}: ⚠ فشل بعد {dt:.2f}ث — {e}", flush=True)
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
imu = _boot("BNO055 (القارئ المشترك)", get_imu)
camera = _boot("الكاميرا (فتحها كسول)", CameraReader)
rover = RoverBridge(mode="sim")
# حساسات القرب الحقيقية + مصدر الإشعاع → محرّك المهمة (المرحلة 2)
ultrasonic = _boot("ألترا سونيك", UltrasonicReader)
ir_sensors = _boot("حسّاسا IR", IRReader)
mission.set_proximity(ultrasonic, ir_sensors)
mission.set_geiger(geiger)
_sensor_clients: set[WebSocket] = set()
_last_sensor_loop = time.time()


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
        },
    }


print(f"[إقلاع] اكتمل تجهيز الأنظمة في "
      f"{time.time() - _T_BOOT:.2f}ث — يبدأ uvicorn الآن", flush=True)
for _k, _v in _boot_summary()["health"].items():
    if _v.get("error"):
        print(f"[إقلاع] ⚠ {_k}: {_v['error']}", flush=True)


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
            if (now - last_bcast) >= 0.2:
                last_bcast = now
                mission.poll_battery()     # الجهد يُعرض دائماً لا أثناء المسح فقط
                mission.poll_ground_echo()  # اشتباه صدى الأرض (إعلان لا معالجة)
                mission.poll_power_clamp() # أحداث الجسر + heartbeat → السجل
                mission.poll_reactive()    # بثّ السرعة وسببها (البند 3)
                if _sim_clients:
                    msg = json.dumps(mission.state_dict(include_full_grid=False))
                    for ws in list(_sim_clients):
                        try:
                            await ws.send_text(msg)
                        except Exception:      # noqa: BLE001
                            _sim_clients.discard(ws)
        except Exception:                      # noqa: BLE001 — لا نُسقط الحلقة
            pass
        await asyncio.sleep(0.01)


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
                msg = json.dumps(_sensor_telemetry())
                for ws in list(_sensor_clients):
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
    geiger.close(); gps.close(); camera.close()
    ultrasonic.close(); ir_sensors.close()


# ⚠ حقن الكاميرا في المهمة: التوثيق البصري بعد المسح يحتاجها، وبلا هذا
#   السطر يعمل كل شيء **عدا التقاط الصور** بلا أي رسالة خطأ.
mission.camera = camera

app = FastAPI(title="RMS Rover v2", lifespan=lifespan)


def _page(name: str) -> HTMLResponse:
    return HTMLResponse((_STATIC / name).read_text(encoding="utf-8"))


# ═══ الصفحات ═════════════════════════════════════════════════════
@app.get("/")
async def index():
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
    prof = default_sim_profile(battery_v=mission.rover.voltage() or 0.0)
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

    ⛔ **لا حسّاس جهد على هذا العتاد** — الحقل `v` لا يصل والحسّاس معطّل. لا
       تبنِ عليه شيئاً. الحماية من الاستنزاف قائمة على بديلين لا يحتاجان
       فولتميتر: حدّ زمني، و**ذروة معدل الدوران** (الدوران بالمكان أثقل
       مناورة فهو أول ما يسقط مع ضعف البطارية).
    """
    st = mission.rover.read_status()
    rv = mission.rover
    return {
        "raw": st, "mode": rv.mode,
        "link_ok": rv.link_ok, "link_error": rv.link_error,
        "voltage_sensor": "معطّل — لا قراءة جهد على هذا العتاد",
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


@app.post("/api/obstacle")
async def api_obstacle(req: Request):
    d = await req.json()
    return mission.toggle_obstacle(int(d["row"]), int(d["col"]))


@app.get("/api/mission/csv")
async def api_mission_csv():
    return Response(content=mission.csv_bytes(), media_type="text/csv",
                    headers={"Content-Disposition": "attachment; filename=mission.csv"})


@app.get("/api/mission/report")
async def api_mission_report():
    return mission.report()


@app.get("/api/sim/status")
async def api_sim_status():
    return mission.state_dict(include_full_grid=True)


@app.websocket("/ws")
async def ws_sim(ws: WebSocket):
    await ws.accept()
    _sim_clients.add(ws)
    try:
        await ws.send_text(json.dumps({**platform_banner(),
                                       **mission.state_dict(include_full_grid=True)}))
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
def stream() -> Response:
    if not camera.state()["available"]:
        return Response(status_code=503, content="camera unavailable")
    return StreamingResponse(camera.mjpeg_frames(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


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
                if cmd.get("c") == "drive":
                    rover.command(cmd.get("dir", "S"), cmd.get("power", 70))
                elif cmd.get("c") == "estop":
                    rover.command("S", 0)
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
