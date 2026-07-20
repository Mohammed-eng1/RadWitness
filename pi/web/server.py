# -*- coding: utf-8 -*-
"""
server.py — سيرفر RMS Rover v2 (FastAPI + WebSocket)  [M1]
=========================================================
- حلقة قراءة async غير حاجبة كل ثانية: جيجر + GPS + BNO055 + تحديث روفر sim.
- بث WebSocket موحّد كل ثانية لكل العملاء.
- أوامر قيادة sim من الواجهة عبر WebSocket.
- لقطة كاميرا عند الطلب (/snapshot.jpg). بث MJPEG وطبقة الذكاء الكاملة في M2.

التشغيل (من جذر المستودع، والبيئة مفعّلة):
    python -m pi.web.server
    # أو:  uvicorn pi.web.server:app --host 0.0.0.0 --port 8000
ثم افتح:  http://therover:8000  (محلياً أو عبر Tailscale)
"""
import asyncio
import json
import os
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, Response

from pi.config import WEB_HOST, WEB_PORT, BROADCAST_S
from pi.sensors.geiger import GeigerReader
from pi.sensors.gps import GPSReader
from pi.sensors.imu import IMUReader
from pi.sensors.camera import CameraReader
from pi.rover.bridge import RoverBridge
from pi.ai.risk import classify

# ── الحساسات والجسر (تُهيّأ عند الاستيراد؛ الأعطال غير قاتلة) ──────
geiger = GeigerReader()
gps = GPSReader()
imu = IMUReader()
camera = CameraReader()
rover = RoverBridge(mode="sim")

_clients: set[WebSocket] = set()
_last_loop = time.time()
_STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")


def build_telemetry() -> dict:
    """يبني حزمة التيليمتري الموحّدة من كل المصادر."""
    g = geiger.state()
    p = gps.state()
    m = imu.state()
    r = rover.state()
    risk = classify(g["usvh"])
    return {
        "t": "telemetry",
        "ts": int(time.time()),
        # الجيجر + الخطر
        "cpm_raw": g["cpm_raw"], "cpm": g["cpm"], "usvh": g["usvh"],
        "high_rate": g["high_rate"], "total": g["total"],
        "risk": risk["risk"], "lvl": risk["lvl"], "risk_color": risk["color"],
        # GPS
        "fix": p["fix"], "lat": p["lat"], "lng": p["lng"],
        "sats": p["sats"], "hdop": p["hdop"],
        # IMU
        "heading": m["heading"], "mag_cal": m["mag_cal"],
        "sys_cal": m["sys_cal"], "gyro_cal": m["gyro_cal"], "accel_cal": m["accel_cal"],
        "mag_warn": m["mag_warn"],
        # الروفر (sim)
        "rover": r,
        # صحة المكونات
        "health": {
            "geiger": g["ok"],
            "gps": p["ok"] and p["fix"],
            "gps_link": p["ok"],
            "imu": m["ok"],
            "camera": camera.state()["available"],
        },
        "sim": rover.mode == "sim",
    }


def _handle_command(cmd: dict) -> None:
    """أوامر الواجهة عبر WebSocket (قيادة sim + إيقاف طوارئ)."""
    c = cmd.get("c")
    if c == "drive":
        rover.command(cmd.get("dir", "S"), cmd.get("power", 70))
    elif c == "estop":
        rover.command("S", 0)


async def _sensor_loop() -> None:
    """الحلقة الرئيسية: تحدّث الحساسات وتبثّ كل ثانية."""
    global _last_loop
    while True:
        try:
            now = time.time()
            dt = now - _last_loop
            _last_loop = now

            geiger.sample()
            imu.read()
            p = gps.state()
            if p["fix"]:
                rover.set_home_from_gps(p["lat"], p["lng"])
            rover.update(dt)

            msg = json.dumps(build_telemetry())
            for ws in list(_clients):
                try:
                    await ws.send_text(msg)
                except Exception:          # noqa: BLE001 — عميل مقطوع
                    _clients.discard(ws)
        except Exception:                  # noqa: BLE001 — لا نُسقط الحلقة أبداً
            pass
        await asyncio.sleep(BROADCAST_S)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    task = asyncio.create_task(_sensor_loop())
    yield
    task.cancel()
    geiger.close()
    gps.close()
    camera.close()


app = FastAPI(title="RMS Rover v2", lifespan=lifespan)


@app.get("/")
async def index() -> HTMLResponse:
    path = os.path.join(_STATIC_DIR, "index.html")
    with open(path, encoding="utf-8") as f:
        return HTMLResponse(f.read())


@app.get("/snapshot.jpg")
def snapshot() -> Response:
    """لقطة كاميرا واحدة (sync → FastAPI يشغّلها في threadpool)."""
    data = camera.snapshot_jpeg()
    if data is None:
        return Response(status_code=503, content="camera unavailable")
    return Response(content=data, media_type="image/jpeg")


@app.get("/api/status")
async def api_status() -> dict:
    """لقطة حالة كاملة (تشخيص)."""
    return build_telemetry()


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    _clients.add(ws)
    try:
        await ws.send_text(json.dumps(build_telemetry()))   # لقطة فورية
        while True:
            raw = await ws.receive_text()
            try:
                _handle_command(json.loads(raw))
            except Exception:              # noqa: BLE001 — رسالة تالفة
                continue
    except WebSocketDisconnect:
        pass
    except Exception:                      # noqa: BLE001
        pass
    finally:
        _clients.discard(ws)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=WEB_HOST, port=WEB_PORT)
