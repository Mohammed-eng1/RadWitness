#!/usr/bin/env python3
"""
lidar_teleop.py - live LiDAR view + driving for UGV01, from any browser.

On the Raspberry Pi:   python3 lidar_teleop.py
On the laptop:         open  http://<pi-ip>:8080   (get the IP with: hostname -I)

Keys: W/S or Up/Down = forward/back, A/D or Left/Right = turn, Space = stop.
The robot only moves while you hold a key/button. If the page stops talking
(closed tab, WiFi drop) the robot stops within ~0.3 s.

Needs only pyserial   (sudo apt install -y python3-serial)
Options:  --base /dev/ttyAMA4   --lidar /dev/ttyUSB0   --http 8080   --no-base
"""
import argparse
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("[FAIL] pyserial missing:  sudo apt install -y python3-serial")

LIDAR_BAUD = 460800
BASE_BAUD = 115200
CMD_WATCHDOG_S = 0.30          # no command from the page for this long -> stop
SEND_PERIOD_S = 0.10           # speed command to the base every 100 ms
HEARTBEAT_MS = 500             # base stops by itself 0.5 s after the last command
MAX_SPEED = 0.30               # m/s hard limit


# ─────────────────────────────── LiDAR ───────────────────────────────
class NodeParser:
    """RPLIDAR standard scan: 5-byte nodes, check bits, resync on error."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out, i, n = [], 0, len(self.buf)
        while n - i >= 5:
            b0, b1, b2, b3, b4 = self.buf[i:i + 5]
            s, ns = b0 & 1, (b0 >> 1) & 1
            if s == ns or (b1 & 1) != 1:
                i += 1
                continue
            out.append((s, ((b2 << 7) | (b1 >> 1)) / 64.0, (b3 | (b4 << 8)) / 4.0))
            i += 5
        del self.buf[:i]
        return out


def find_lidar():
    for p in list_ports.comports():
        if p.vid == 0x10C4 and p.pid == 0xEA60:
            return p.device
    return None


class Lidar(threading.Thread):
    def __init__(self, port=None):
        super().__init__(daemon=True)
        self.port_arg = port
        self.lock = threading.Lock()
        self.scan = []              # [(angle_deg, dist_mm), ...] last full turn
        self.t_scan = 0.0
        self.rate = 0.0
        self.status = "starting"
        self.ser = None
        self.running = True

    def _send(self, cmd):
        self.ser.write(bytes([0xA5, cmd]))
        self.ser.flush()

    def run(self):
        while self.running:
            port = self.port_arg or find_lidar()
            if not port:
                self.status = "lidar not found (USB?)"
                time.sleep(1)
                continue
            try:
                self.ser = serial.Serial(port, LIDAR_BAUD, timeout=0.5)
                self._send(0x25)                      # STOP
                time.sleep(0.05)
                self.ser.reset_input_buffer()
                self._send(0x20)                      # SCAN
                d = self.ser.read(7)
                if len(d) != 7 or d[0] != 0xA5 or d[1] != 0x5A:
                    raise IOError("no scan reply")
                self.status = f"ok ({port})"
                parser, turn, starts = NodeParser(), [], []
                while self.running:
                    data = self.ser.read(2048)
                    if not data:
                        raise IOError("lidar silent")
                    for s, a, dist in parser.feed(data):
                        if s == 1 and turn:
                            now = time.time()
                            starts = (starts + [now])[-11:]
                            with self.lock:
                                self.scan = turn
                                self.t_scan = now
                                if len(starts) > 1:
                                    self.rate = (len(starts) - 1) / (starts[-1] - starts[0])
                            turn = []
                        if dist > 0:
                            turn.append((round(a, 1), round(dist)))
            except Exception as e:                     # noqa: BLE001
                self.status = f"reconnecting: {e}"
                try:
                    self.ser.close()
                except Exception:                      # noqa: BLE001
                    pass
                time.sleep(1)

    def snapshot(self):
        with self.lock:
            return list(self.scan), self.t_scan, self.rate

    def stop(self):
        self.running = False
        try:
            self._send(0x25)
            self.ser.close()
        except Exception:                              # noqa: BLE001
            pass


# ─────────────────────────────── Base ───────────────────────────────
class Base(threading.Thread):
    """Sends {"T":1,"L","R"} every 100 ms; zero if the page went quiet."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, BASE_BAUD, timeout=0.1)
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)
        self.t_cmd = 0.0
        self.volt = None
        self.running = True
        self.send({"T": 900, "main": 3, "module": 0})    # UGV01
        time.sleep(0.05)
        self.send({"T": 136, "cmd": HEARTBEAT_MS})       # short safety timer
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
                        if m.get("T") == 1001 and "v" in m:
                            self.volt = float(m["v"])
                    except Exception:                  # noqa: BLE001
                        pass
            except Exception:                          # noqa: BLE001
                time.sleep(0.2)

    def run(self):
        k = 0
        while self.running:
            l, r = self.current()
            try:
                self.send({"T": 1, "L": round(l, 3), "R": round(r, 3)})
                k += 1
                if k % 10 == 0:
                    self.send({"T": 130})                # ask for battery once a second
            except Exception:                          # noqa: BLE001
                pass
            time.sleep(SEND_PERIOD_S)

    def stop(self):
        self.running = False
        try:
            for _ in range(3):
                self.send({"T": 1, "L": 0.0, "R": 0.0})
                time.sleep(0.05)
            self.send({"T": 136, "cmd": 3000})
            self.ser.close()
        except Exception:                              # noqa: BLE001
            pass


# ─────────────────────────────── Web page ───────────────────────────────
PAGE = r"""<!doctype html><html lang="ar" dir="rtl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UGV01 LiDAR</title>
<style>
:root{--bg:#0f1417;--panel:#172024;--ink:#e6eef0;--mute:#8aa0a6;--acc:#39d0b0;--warn:#ff6b5a;--grid:#243238}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px system-ui,"Segoe UI",Tahoma,sans-serif}
.wrap{display:flex;gap:16px;padding:16px;flex-wrap:wrap;justify-content:center}
canvas{background:#0b0f11;border-radius:12px;max-width:100%;height:auto;touch-action:none}
.side{display:flex;flex-direction:column;gap:12px;min-width:240px;max-width:320px;flex:1}
.card{background:var(--panel);border-radius:12px;padding:12px}
.row{display:flex;justify-content:space-between;gap:8px;font-variant-numeric:tabular-nums}
.mute{color:var(--mute);font-size:13px}
.pad{display:grid;grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,64px);gap:6px;justify-content:center;direction:ltr}
.pad button{font-size:24px;border:0;border-radius:10px;background:#22343a;color:var(--ink);cursor:pointer;user-select:none}
.pad button:active,.pad button.on{background:var(--acc);color:#062}
.stop{background:var(--warn)!important;color:#fff!important;font-size:15px!important}
input[type=range]{width:100%}
.big{font-size:20px;font-weight:600}
.warn{color:var(--warn);font-weight:600}
label{display:flex;gap:8px;align-items:center}
</style></head><body>
<div class="wrap">
  <canvas id="c" width="640" height="640"></canvas>
  <div class="side">
    <div class="card">
      <div class="big">UGV01 · LiDAR</div>
      <div class="row"><span>أقرب جسم <span class="mute">nearest</span></span><span id="near" dir="ltr">—</span></div>
      <div class="row"><span>لفات/ث <span class="mute">scan rate</span></span><span id="rate" dir="ltr">—</span></div>
      <div class="row"><span>البطارية <span class="mute">battery</span></span><span id="volt" dir="ltr">—</span></div>
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
      <label><input id="safe" type="checkbox" checked> أوقف الأمام إذا فيه عائق أقرب من <span id="sdv">30</span> سم</label>
      <input id="sd" type="range" min="15" max="80" step="5" value="30">
      <label><input id="flip" type="checkbox"> اعكس يمين/يسار (إذا طلع الرسم معكوس)</label>
    </div>
  </div>
</div>
<script>
const c=document.getElementById('c'),g=c.getContext('2d');g.direction='ltr';
const $=id=>document.getElementById(id);
let scan=[],keys=new Set(),blocked=false;
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
    $('lst').textContent=j.age>0.5?('متوقف '+j.status):j.status;
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
  $('blk').hidden=!blocked;return[l,r];
}
async function tick(){
  if(keys.size&&!keys.has('x')){const[l,r]=speeds();
    try{await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({l,r})})}catch(e){}}
  else{$('blk').hidden=true}
  setTimeout(tick,100);
}
const map={ArrowUp:'f',KeyW:'f',ArrowDown:'b',KeyS:'b',ArrowLeft:'l',KeyA:'l',ArrowRight:'r',KeyD:'r',Space:'x'};
addEventListener('keydown',e=>{const k=map[e.code];if(k){e.preventDefault();if(k==='x'){keys.clear();stopNow()}else keys.add(k);mark()}});
addEventListener('keyup',e=>{const k=map[e.code];if(k){keys.delete(k);mark()}});
addEventListener('blur',()=>{keys.clear();mark()});
document.querySelectorAll('.pad button').forEach(b=>{const k=b.dataset.k;
  const on=e=>{e.preventDefault();if(k==='x'){keys.clear();stopNow()}else keys.add(k);mark()};
  const off=e=>{e.preventDefault();keys.delete(k);mark()};
  b.addEventListener('pointerdown',on);b.addEventListener('pointerup',off);b.addEventListener('pointerleave',off);b.addEventListener('pointercancel',off)});
function mark(){document.querySelectorAll('.pad button').forEach(b=>b.classList.toggle('on',keys.has(b.dataset.k)))}
function stopNow(){fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:'{"l":0,"r":0}'}).catch(()=>{})}
for(const id of['sp','rg','sd'])$(id).addEventListener('input',()=>{$('spv').textContent=(+$('sp').value).toFixed(2)+' m/s';$('rgv').textContent=$('rg').value+' m';$('sdv').textContent=$('sd').value;draw()});
poll();tick();
</script></body></html>"""


def make_handler(lidar, base):
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

        def do_GET(self):
            if self.path == "/scan":
                pts, t, rate = lidar.snapshot()
                body = json.dumps({"pts": pts, "rate": rate,
                                   "age": time.time() - t if t else 99,
                                   "status": lidar.status,
                                   "v": base.volt if base else None}).encode()
                self._send(200, body, "application/json")
            elif self.path in ("/", "/index.html"):
                self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            if self.path != "/cmd":
                return self._send(404, b"not found", "text/plain")
            try:
                n = int(self.headers.get("Content-Length", 0))
                j = json.loads(self.rfile.read(n) or b"{}")
                if base:
                    base.set_cmd(j.get("l", 0), j.get("r", 0))
                self._send(200, b"ok", "text/plain")
            except Exception as e:                     # noqa: BLE001
                self._send(400, str(e).encode(), "text/plain")
    return H


def main():
    ap = argparse.ArgumentParser(description="UGV01 live LiDAR view + driving")
    ap.add_argument("--base", default="/dev/ttyAMA4", help="robot base serial port")
    ap.add_argument("--lidar", default=None, help="lidar port (auto: CP210x)")
    ap.add_argument("--http", type=int, default=8080)
    ap.add_argument("--no-base", action="store_true", help="view only, no driving")
    a = ap.parse_args()

    lidar = Lidar(a.lidar)
    lidar.start()
    base = None
    if not a.no_base:
        try:
            base = Base(a.base)
            base.start()
        except Exception as e:                         # noqa: BLE001
            sys.exit(f"[FAIL] cannot open base {a.base}: {e}\n"
                     "       another program using it? (stop lidar_drive / web server first)")

    srv = ThreadingHTTPServer(("0.0.0.0", a.http), make_handler(lidar, base))
    print("=" * 56)
    print(f"  Open in the laptop browser:  http://<pi-ip>:{a.http}")
    print("  (Pi IP:  hostname -I)      Ctrl+C here = stop everything")
    print("=" * 56)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nstopping robot and lidar...")
        if base:
            base.stop()
        lidar.stop()
        srv.server_close()


if __name__ == "__main__":
    main()
