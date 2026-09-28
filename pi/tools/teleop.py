#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop.py — تحكم سريع بـUGV01 من المتصفح + عرض الليدار ثنائي الأبعاد
====================================================================
    python3 -m pi.tools.teleop            # على الراسبيري ثم افتح http://IP:8080
    python3 -m pi.tools.teleop --sim      # بلا عتاد (ليدار وقاعدة وهميان — ويندوز)

مبني على lidar_teleop.py (مجرّب على محاكي). مكتبات بايثون القياسية + pyserial
فقط، والصفحة ملف واحد بلا أي تحميل من الإنترنت.

الأمان:
- الروبوت يتحرك فقط أثناء الضغط: الصفحة ترسل كل 100ms، والسيرفر يصفّر السرعة
  إن لم يصله أمر خلال 0.3ث. أقصى سرعة 0.30 م/ث.
- مهلة الفيرموير 500ms أثناء التشغيل (T:136)، وتُعاد 3000 عند الخروج بعد
  ثلاثة أصفار.
- «أوقف الأمام إذا فيه عائق» يُطبَّق في الصفحة **وفي السيرفر** معاً (الثاني
  يحمي حتى لو تأخّر رسم الصفحة): يمنع التقدّم فقط، واللفّ والرجوع مسموحة.
- 🔴 أوامر الصفحة بإطار **الروبوت** (L/R = جنزير يسار/يمين، موجب = تقدّم)،
  وتُحوَّل لإطار **السلك** بخريطة المحركات المقاسة في config قبل الإرسال
  (UGV01: تبديل + نفي — السلك (+,+) يرجع للخلف فيزيائياً، مقاس 2026-09-26).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial                                   # pyserial
except ImportError:                                 # --sim يعمل بدونها
    serial = None

from pi.config import MOTOR_INVERT, MOTOR_SWAP_LR
from pi.sensors.lidar_c1 import LidarC1, SimLidar, sim_room_scene

BASE_BAUD = 115200
CMD_WATCHDOG_S = 0.30          # لا أمر من الصفحة خلال هذا ⇒ صفر
SEND_PERIOD_S = 0.10           # أمر السرعة للقاعدة كل 100ms
HEARTBEAT_MS = 500             # القاعدة تقف وحدها بعد 0.5ث من آخر أمر
RESTORE_HEARTBEAT_MS = 3000    # افتراضي الفيرموير عند الخروج
MAX_SPEED = 0.30               # م/ث حدّ صارم
FRONT_HALF_DEG = 25.0          # قطاع «الأمام» لخيار الوقف (نفس رسم الصفحة)
MIN_RANGE_MM = 50              # ما دون 5سم يُهمل (مقاس)


def to_wire(l: float, r: float) -> tuple[float, float]:
    """إطار الروبوت ⇒ إطار السلك (نفس تحويل bridge.motors: تبديل ثم نفي)."""
    wl, wr = (r, l) if MOTOR_SWAP_LR else (l, r)
    return wl * MOTOR_INVERT, wr * MOTOR_INVERT


# ─────────────────────────────── الليدار ───────────────────────────────
class LidarView:
    """غلاف فوق LidarC1/SimLidar: نقاط (زاوية°، مم) + معدل اللفّات + الحالة."""

    def __init__(self, lidar):
        self.lidar = lidar
        self._hist = deque(maxlen=12)      # (وقت، عدد اللفّات)

    def snapshot(self):
        s = self.lidar.latest()
        now = time.time()
        self._hist.append((now, self.lidar.revs))
        rate = 0.0
        if len(self._hist) >= 2 and self._hist[-1][0] > self._hist[0][0]:
            rate = ((self._hist[-1][1] - self._hist[0][1])
                    / (self._hist[-1][0] - self._hist[0][0]))
        if s is None:
            return [], 0.0, rate
        pts = [(round(a, 1), round(d * 1000)) for a, d in s[1]
               if d and d * 1000 >= MIN_RANGE_MM]
        return pts, s[0], rate

    def status(self) -> str:
        l = self.lidar
        if getattr(l, "connected", False):
            return f"ok ({l.port})"
        return f"reconnecting: {l.error}" if l.error else "starting"

    def front_min_mm(self):
        pts, t, _ = self.snapshot()
        if not t or time.time() - t > 1.0:
            return None                    # لا بيانات حديثة ⇒ مجهول
        m = None
        for a, d in pts:
            x = (a + 180.0) % 360.0 - 180.0
            if abs(x) <= FRONT_HALF_DEG and (m is None or d < m):
                m = d
        return m

    def stop(self):
        self.lidar.stop()                  # 🔴 STOP للمحرك (لا يقف بغيره)


# ─────────────────────────────── القاعدة ───────────────────────────────
class FakeSerial:
    """منفذ وهمي لـ--sim: يسجّل ما يُكتب ويردّ على T:130 بجهد ثابت."""

    def __init__(self):
        self.lines = deque(maxlen=400)
        self._out = bytearray()
        self._lock = threading.Lock()

    def write(self, b):
        line = b.decode().strip()
        with self._lock:
            self.lines.append(line)
            if '"T":130' in line:
                self._out += b'{"T":1001,"L":0,"R":0,"v":11.84}\n'
        return len(b)

    def read(self, n):
        time.sleep(0.05)
        with self._lock:
            out, self._out = bytes(self._out[:n]), self._out[n:]
        return out

    def close(self):
        pass


class Base(threading.Thread):
    """يرسل {"T":1,"L","R"} كل 100ms؛ صفر إن سكتت الصفحة."""

    def __init__(self, port, ser=None):
        super().__init__(daemon=True)
        self.ser = ser or serial.Serial(port, BASE_BAUD, timeout=0.1)
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)
        self.t_cmd = 0.0
        self.volt = None
        self.running = True
        self.send({"T": 900, "main": 3, "module": 0})    # UGV01
        time.sleep(0.05)
        self.send({"T": 136, "cmd": HEARTBEAT_MS})       # مؤقت أمان قصير
        threading.Thread(target=self._reader, daemon=True).start()

    def send(self, obj):
        self.ser.write((json.dumps(obj, separators=(",", ":")) + "\n").encode())

    def set_cmd(self, l, r):
        l = max(-MAX_SPEED, min(MAX_SPEED, float(l)))
        r = max(-MAX_SPEED, min(MAX_SPEED, float(r)))
        with self.lock:
            self.cmd = (l, r)
            self.t_cmd = time.time()

    def current(self):
        with self.lock:
            if time.time() - self.t_cmd > CMD_WATCHDOG_S:
                return 0.0, 0.0
            return self.cmd

    def _reader(self):
        buf = b""
        while self.running:
            try:
                buf += self.ser.read(256)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        m = json.loads(line)
                        if isinstance(m, dict) and m.get("T") == 1001 and "v" in m:
                            self.volt = float(m["v"])
                    except Exception:                  # noqa: BLE001
                        pass
            except Exception:                          # noqa: BLE001
                time.sleep(0.2)

    def run(self):
        k = 0
        while self.running:
            l, r = self.current()
            wl, wr = to_wire(l, r)
            try:
                # أرقام عشرية دائماً (حتى الصفر 0.0)
                # (+ 0.0 يحوّل −0.0 الناتج عن النفي إلى 0.0)
                self.send({"T": 1, "L": float(round(wl, 3)) + 0.0,
                           "R": float(round(wr, 3)) + 0.0})
                k += 1
                if k % 10 == 0:
                    self.send({"T": 130})                # البطارية مرة كل ثانية
            except Exception:                          # noqa: BLE001
                pass
            time.sleep(SEND_PERIOD_S)

    def stop(self):
        self.running = False
        try:
            for _ in range(3):
                self.send({"T": 1, "L": 0.0, "R": 0.0})
                time.sleep(0.05)
            self.send({"T": 136, "cmd": RESTORE_HEARTBEAT_MS})
            self.ser.close()
        except Exception:                              # noqa: BLE001
            pass


# ─────────────────────────── إضافات اختيارية ───────────────────────────
class GeigerTicker:
    """GeigerReader + استدعاء sample() كل ثانية. بلا عتاد ⇒ None بصمت."""

    def __init__(self):
        self.g = None
        try:
            from pi.sensors.geiger import GeigerReader
            g = GeigerReader()
            if g.ok:
                self.g = g
                threading.Thread(target=self._tick, daemon=True).start()
        except Exception:                              # noqa: BLE001
            self.g = None

    def _tick(self):
        while True:
            try:
                self.g.sample()
            except Exception:                          # noqa: BLE001
                pass
            time.sleep(1.0)

    def cpm(self):
        return None if self.g is None else self.g.state().get("cpm")

    def close(self):
        if self.g is not None:
            self.g.close()


def open_camera():
    try:
        from pi.sensors.camera import CameraReader
        cam = CameraReader()
        return cam if getattr(cam, "available", False) else None
    except Exception:                                  # noqa: BLE001
        return None


# ─────────────────────────────── الصفحة ───────────────────────────────
PAGE = r"""<!doctype html><html lang="ar" dir="rtl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UGV01 LiDAR</title>
<style>
:root{--bg:#0f1417;--panel:#172024;--ink:#e6eef0;--mute:#8aa0a6;--acc:#39d0b0;--warn:#ff6b5a;--grid:#243238}
*{box-sizing:border-box}[hidden]{display:none!important}body{margin:0;background:var(--bg);color:var(--ink);font:15px system-ui,"Segoe UI",Tahoma,sans-serif}
.wrap{display:flex;gap:16px;padding:16px;flex-wrap:wrap;justify-content:center}
canvas{background:#0b0f11;border-radius:12px;width:min(640px,100%);height:auto;aspect-ratio:1;touch-action:none}
.side{display:flex;flex-direction:column;gap:12px;min-width:240px;max-width:340px;flex:1}
.card{background:var(--panel);border-radius:12px;padding:12px}
.row{display:flex;justify-content:space-between;gap:8px;font-variant-numeric:tabular-nums}
.mute{color:var(--mute);font-size:13px}
.pad{display:grid;grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,72px);gap:6px;justify-content:center;direction:ltr}
.pad button{font-size:26px;border:0;border-radius:10px;background:#22343a;color:var(--ink);cursor:pointer;user-select:none;-webkit-user-select:none;touch-action:none}
.pad button:active,.pad button.on{background:var(--acc);color:#062}
.stop{background:var(--warn)!important;color:#fff!important;font-size:15px!important}
input[type=range]{width:100%}
.big{font-size:20px;font-weight:600}
.warn{color:var(--warn);font-weight:600}
label{display:flex;gap:8px;align-items:center}
#cam{width:100%;border-radius:10px;background:#0b0f11;display:block}
</style></head><body>
<div class="wrap">
  <canvas id="c" width="640" height="640"></canvas>
  <div class="side">
    <div class="card">
      <div class="big">UGV01 · LiDAR</div>
      <div class="row"><span>أقرب جسم <span class="mute">nearest</span></span><span id="near" dir="ltr">—</span></div>
      <div class="row"><span>لفات/ث <span class="mute">scan rate</span></span><span id="rate" dir="ltr">—</span></div>
      <div class="row"><span>البطارية <span class="mute">battery</span></span><span id="volt" dir="ltr">—</span></div>
      <div class="row" id="cpmRow" hidden><span>الإشعاع <span class="mute">CPM</span></span><span id="cpm" dir="ltr">—</span></div>
      <div class="row"><span>الليدار <span class="mute">lidar</span></span><span id="lst" class="mute" dir="ltr">—</span></div>
      <div id="blk" class="warn" hidden>⛔ عائق أمامي — الأمام موقوف</div>
    </div>
    <div class="card">
      <div class="pad">
        <span></span><button data-k="f">▲</button><span></span>
        <button data-k="l">◀</button><button data-k="x" class="stop">STOP</button><button data-k="r">▶</button>
        <span></span><button data-k="b">▼</button><span></span>
      </div>
      <p class="mute">اضغط باستمرار · الكيبورد: W A S D أو الأسهم · Space = وقوف</p>
    </div>
    <div class="card">
      <div class="row"><span>السرعة <span class="mute">speed</span></span><span id="spv" dir="ltr">0.15 m/s</span></div>
      <input id="sp" type="range" min="0.05" max="0.30" step="0.01" value="0.15">
      <div class="row"><span>المدى المعروض <span class="mute">view</span></span><span id="rgv" dir="ltr">3 m</span></div>
      <input id="rg" type="range" min="1" max="8" step="0.5" value="3">
      <label><input id="safe" type="checkbox" checked> أوقف الأمام إذا فيه عائق أقرب من <span id="sdv" dir="ltr">30</span> سم</label>
      <input id="sd" type="range" min="15" max="80" step="5" value="30">
      <label><input id="flip" type="checkbox"> اعكس يمين/يسار <span class="mute">flip L/R</span></label>
    </div>
    <div class="card" id="camCard" hidden>
      <label><input id="camOn" type="checkbox" checked> الكاميرا <span class="mute">camera</span></label>
      <img id="cam" alt="">
    </div>
  </div>
</div>
<script>
const c=document.getElementById('c'),g=c.getContext('2d');c.style.direction='ltr';g.direction='ltr';
const $=id=>document.getElementById(id);
let scan=[],keys=new Set(),blocked=false,srvBlocked=false;
function frontMin(){let m=1e9;for(const[a,d]of scan){let x=((a+180)%360+360)%360-180;if(Math.abs(x)<=25&&d>50)m=Math.min(m,d)}return m}
function draw(){
  const W=c.width,H=c.height,cx=W/2,cy=H/2,R=+$('rg').value,k=(W/2-10)/(R*1000),flip=$('flip').checked?-1:1;
  g.clearRect(0,0,W,H);
  g.strokeStyle='#243238';g.fillStyle='#5f777d';g.font='12px sans-serif';g.lineWidth=1;
  for(let r=0.5;r<=R+1e-6;r+=0.5){g.beginPath();g.arc(cx,cy,r*1000*k,0,7);g.stroke();if(r%1===0)g.fillText(r+' m',cx+4,cy-r*1000*k-3)}
  g.beginPath();g.moveTo(cx,0);g.lineTo(cx,H);g.moveTo(0,cy);g.lineTo(W,cy);g.stroke();
  const sd=+$('sd').value*10;
  g.fillStyle='rgba(255,107,90,0.10)';g.beginPath();g.moveTo(cx,cy);g.arc(cx,cy,sd*k,-Math.PI/2-25*Math.PI/180,-Math.PI/2+25*Math.PI/180);g.fill();
  for(const[a,d]of scan){const t=a*Math.PI/180;const x=cx+flip*d*k*Math.sin(t),y=cy-d*k*Math.cos(t);
    g.fillStyle=d<sd?'#ff6b5a':'#39d0b0';g.fillRect(x-1.5,y-1.5,3,3)}
  g.fillStyle='#e6eef0';g.beginPath();g.moveTo(cx,cy-14);g.lineTo(cx-9,cy+10);g.lineTo(cx+9,cy+10);g.fill();
  g.fillStyle='#8aa0a6';g.fillText('أمام / front',cx+8,30);
}
async function poll(){
  try{const r=await fetch('/scan');const j=await r.json();scan=j.pts;
    $('rate').textContent=j.rate.toFixed(2);$('volt').textContent=j.v==null?'—':j.v.toFixed(2)+' V';
    $('lst').textContent=j.age>0.5?('stale · '+j.status):j.status;
    if(j.cpm!=null){$('cpmRow').hidden=false;$('cpm').textContent=j.cpm.toFixed(0)}
    srvBlocked=!!j.blocked;
    let n=null;for(const[a,d]of scan){if(d>50&&(!n||d<n[1]))n=[a,d]}
    $('near').textContent=n?(n[1]/10).toFixed(1)+' cm @ '+n[0].toFixed(0)+'°':'—';
    draw();}catch(e){$('lst').textContent='no connection'}
  setTimeout(poll,120);
}
function speeds(){
  const v=+$('sp').value;let l=0,r=0;
  if(keys.has('f')){l+=v;r+=v} if(keys.has('b')){l-=v;r-=v}
  if(keys.has('l')){l-=v*0.8;r+=v*0.8} if(keys.has('r')){l+=v*0.8;r-=v*0.8}
  blocked=false;
  if($('safe').checked&&(l+r)>0&&frontMin()<+$('sd').value*10){blocked=true;const turn=(r-l)/2;l=-turn;r=turn}
  return[l,r];
}
async function tick(){
  if(keys.size&&!keys.has('x')){const[l,r]=speeds();
    try{await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({l,r,safe:$('safe').checked,stop_cm:+$('sd').value})})}catch(e){}}
  else{blocked=false}
  $('blk').hidden=!(blocked||(srvBlocked&&keys.has('f')));
  setTimeout(tick,100);
}
const map={ArrowUp:'f',KeyW:'f',ArrowDown:'b',KeyS:'b',ArrowLeft:'l',KeyA:'l',ArrowRight:'r',KeyD:'r',Space:'x'};
addEventListener('keydown',e=>{const k=map[e.code];if(k){e.preventDefault();if(k==='x'){keys.clear();stopNow()}else keys.add(k);mark()}});
addEventListener('keyup',e=>{const k=map[e.code];if(k){keys.delete(k);mark()}});
addEventListener('blur',()=>{keys.clear();mark()});
document.addEventListener('visibilitychange',()=>{if(document.hidden){keys.clear();mark();stopNow()}});
document.querySelectorAll('.pad button').forEach(b=>{const k=b.dataset.k;
  const on=e=>{e.preventDefault();if(k==='x'){keys.clear();stopNow()}else keys.add(k);mark()};
  const off=e=>{e.preventDefault();keys.delete(k);mark()};
  b.addEventListener('pointerdown',on);b.addEventListener('pointerup',off);b.addEventListener('pointerleave',off);b.addEventListener('pointercancel',off);
  b.addEventListener('contextmenu',e=>e.preventDefault())});
function mark(){document.querySelectorAll('.pad button').forEach(b=>b.classList.toggle('on',keys.has(b.dataset.k)))}
function stopNow(){fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:'{"l":0.0,"r":0.0}'}).catch(()=>{})}
for(const id of['sp','rg','sd'])$(id).addEventListener('input',()=>{$('spv').textContent=(+$('sp').value).toFixed(2)+' m/s';$('rgv').textContent=$('rg').value+' m';$('sdv').textContent=$('sd').value;draw()});
/* الكاميرا: إطار تلو إطار (بلا بثّ متراكم) — تُخفى إن لم تتوفر */
function camLoop(){const img=$('cam');
  if(!$('camOn').checked){setTimeout(camLoop,500);return}
  const nxt=new Image();nxt.onload=()=>{img.src=nxt.src;$('camCard').hidden=false;setTimeout(camLoop,150)};
  nxt.onerror=()=>{setTimeout(camLoop,3000)};nxt.src='/cam.jpg?t='+Date.now()}
fetch('/caps').then(r=>r.json()).then(j=>{if(j.camera){$('camCard').hidden=false;camLoop()}}).catch(()=>{});
poll();tick();
</script></body></html>"""


def make_handler(view, base, geiger=None, camera=None, sim_ser=None):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode(), "application/json")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/scan":
                pts, t, rate = view.snapshot()
                self._json({"pts": pts, "rate": rate,
                            "age": time.time() - t if t else 99,
                            "status": view.status(),
                            "v": base.volt if base else None,
                            "cpm": geiger.cpm() if geiger else None,
                            "blocked": H.blocked})
            elif path in ("/", "/index.html"):
                self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/caps":
                self._json({"camera": camera is not None,
                            "geiger": bool(geiger and geiger.g), "sim": sim_ser is not None})
            elif path == "/cam.jpg":
                data = camera.latest_jpeg() if camera else None
                if data:
                    self._send(200, data, "image/jpeg")
                else:
                    self._send(503, b"no camera", "text/plain")
            elif path == "/sim/log" and sim_ser is not None:
                self._json(list(sim_ser.lines)[-30:])
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            if self.path != "/cmd":
                return self._send(404, b"not found", "text/plain")
            try:
                n = int(self.headers.get("Content-Length", 0))
                j = json.loads(self.rfile.read(n) or b"{}")
                l, r = float(j.get("l", 0.0)), float(j.get("r", 0.0))
                # 🔴 وقف الأمام في السيرفر أيضاً: التقدّم فقط يُمنع، واللفّ يبقى
                H.blocked = False
                if j.get("safe", True) and (l + r) > 0:
                    fm = view.front_min_mm()
                    stop_mm = float(j.get("stop_cm", 30)) * 10.0
                    if fm is not None and fm < stop_mm:
                        turn = (r - l) / 2.0
                        l, r = -turn, turn
                        H.blocked = True
                if base:
                    base.set_cmd(l, r)
                self._send(200, b"ok", "text/plain")
            except Exception as e:                     # noqa: BLE001
                self._send(400, str(e).encode(), "text/plain")
    H.blocked = False
    return H


def local_ipv4s() -> list[str]:
    """كل عناوين IPv4 غير المحلية (hostname -I ثم احتياط بالمقبس)."""
    ips = []
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True,
                             timeout=2).stdout
        ips = [x for x in out.split() if re.fullmatch(r"\d+\.\d+\.\d+\.\d+", x)]
    except Exception:                                  # noqa: BLE001
        pass
    if not ips:
        try:   # لا يرسل شيئاً: يختار الواجهة الافتراضية فقط
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("10.255.255.255", 1))
            ips = [s.getsockname()[0]]
            s.close()
        except Exception:                              # noqa: BLE001
            pass
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                ip = info[4][0]
                if not ip.startswith("127.") and ip not in ips:
                    ips.append(ip)
        except Exception:                              # noqa: BLE001
            pass
    return ips


def main(argv=None):
    ap = argparse.ArgumentParser(description="UGV01 live LiDAR view + driving")
    ap.add_argument("--base", default="/dev/ttyAMA4", help="منفذ قاعدة الروبوت")
    ap.add_argument("--lidar", default=None, help="منفذ الليدار (تلقائي: CP210x)")
    ap.add_argument("--http", type=int, default=8080)
    ap.add_argument("--no-base", action="store_true", help="عرض فقط بلا قيادة")
    ap.add_argument("--sim", action="store_true", help="ليدار وقاعدة وهميان (بلا منافذ)")
    a = ap.parse_args(argv)

    sim_ser = None
    if a.sim:
        view = LidarView(SimLidar(sim_room_scene(1.5, 1.0)).start())
    else:
        if serial is None:
            sys.exit("[FAIL] pyserial missing:  sudo apt install -y python3-serial")
        view = LidarView(LidarC1(port=a.lidar).start())
    base = None
    if not a.no_base:
        try:
            if a.sim:
                sim_ser = FakeSerial()
                base = Base(None, ser=sim_ser)
            else:
                base = Base(a.base)
            base.start()
        except Exception as e:                         # noqa: BLE001
            view.stop()
            sys.exit(f"[FAIL] cannot open base {a.base}: {e}\n"
                     "       another program using it? (stop lidar_drive / web server first)")
    geiger = None if a.sim else GeigerTicker()
    camera = None if a.sim else open_camera()

    srv = ThreadingHTTPServer(("0.0.0.0", a.http),
                              make_handler(view, base, geiger, camera, sim_ser))
    srv.daemon_threads = True
    # SIGTERM (systemd/kill) يمرّ بنفس مسار الإيقاف الآمن كـCtrl+C
    def _on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)
    print("=" * 56)
    print("  افتح في المتصفح / open in the browser:")
    ips = local_ipv4s()
    for ip in ips:
        print(f"    http://{ip}:{a.http}")
    if not ips:
        print(f"    http://<pi-ip>:{a.http}   (hostname -I)")
    print(f"    http://localhost:{a.http}")
    extras = []
    if geiger and geiger.g:
        extras.append("geiger")
    if camera:
        extras.append("camera")
    print(f"  {'SIM · ' if a.sim else ''}base={'off' if base is None else ('sim' if a.sim else a.base)}"
          f" · extras={','.join(extras) or '-'} · Ctrl+C = stop everything")
    print("=" * 56, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nstopping robot and lidar...", flush=True)
        if base:
            base.stop()
        view.stop()
        if geiger:
            geiger.close()
        if camera:
            try:
                camera.close()
            except Exception:                          # noqa: BLE001
                pass
        srv.server_close()
        if sim_ser is not None:
            print("sim base, last lines sent:")
            for line in list(sim_ser.lines)[-5:]:
                print("   ", line)


if __name__ == "__main__":
    main()
