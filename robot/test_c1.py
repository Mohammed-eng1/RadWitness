#!/usr/bin/env python3
"""
test_c1.py — اختبار ليدار RPLIDAR C1 على اللابتوب (ويندوز / لينكس / ماك)

يحتاج:  pip install pyserial matplotlib      (matplotlib فقط لخيار --plot)

أمثلة:
  python test_c1.py                      فحص كامل 10 ثواني (يختار المنفذ تلقائياً)
  python test_c1.py --port COM5          منفذ محدد
  python test_c1.py --plot               رسم حي من فوق (دائرة 360°)
  python test_c1.py --warmup 120 --seconds 20 --csv   قياس دقة المسافة بعد تسخين دقيقتين

C1 يتكلم بسرعة 460800. المحرك لا يدور إلا بعد أمر المسح — طبيعي أن يكون واقفاً أول ما تشبكه.
"""
import argparse
import csv
import os
import statistics
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("[FAIL] Install pyserial first:  pip install pyserial")

BAUD = 460800
CMD_STOP, CMD_RESET, CMD_SCAN, CMD_INFO, CMD_HEALTH = 0x25, 0x40, 0x20, 0x50, 0x52
HEALTH_TXT = {0: "Good", 1: "Warning", 2: "Error"}


# ───────────────────────── المنفذ ─────────────────────────
def find_port():
    ports = list(list_ports.comports())
    for p in ports:  # محوّل C1 شريحته CP2102 من Silicon Labs
        if p.vid == 0x10C4 and p.pid == 0xEA60:
            return p.device, ports
    return None, ports


def send(ser, cmd):
    ser.write(bytes([0xA5, cmd]))
    ser.flush()


def stop(ser):
    try:
        send(ser, CMD_STOP)
        time.sleep(0.05)
        ser.reset_input_buffer()
    except Exception:
        pass


def read_exact(ser, n, what):
    b = ser.read(n)
    if len(b) != n:
        raise TimeoutError(f"No reply for {what} (got {len(b)} of {n} bytes)")
    return b


def read_descriptor(ser, what):
    d = read_exact(ser, 7, what)
    if d[0] != 0xA5 or d[1] != 0x5A:
        raise ValueError(f"Bad reply header for {what}: {d.hex(' ')}")
    length = int.from_bytes(d[2:6], "little") & 0x3FFFFFFF
    return length, d[6]


def get_info(ser):
    stop(ser)
    send(ser, CMD_INFO)
    length, dtype = read_descriptor(ser, "INFO")
    p = read_exact(ser, length, "INFO")
    return {"model": p[0], "firmware": f"{p[2]}.{p[1]:02d}", "hardware": p[3],
            "serial": p[4:20].hex().upper()}


def get_health(ser):
    send(ser, CMD_HEALTH)
    length, dtype = read_descriptor(ser, "HEALTH")
    p = read_exact(ser, length, "HEALTH")
    return p[0], int.from_bytes(p[1:3], "little")


# ───────────────────── تفكيك نقاط المسح ─────────────────────
class NodeParser:
    """كل نقطة 5 بايت. يتحقق من بتّات الفحص، ويعيد المزامنة إذا انزاح البث."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_bytes = 0

    def feed(self, data):
        self.buf += data
        out = []
        i, n = 0, len(self.buf)
        while n - i >= 5:
            b0, b1, b2, b3, b4 = self.buf[i:i + 5]
            s, ns = b0 & 1, (b0 >> 1) & 1
            if s == ns or (b1 & 1) != 1:
                i += 1
                self.bad_bytes += 1
                continue
            angle = ((b2 << 7) | (b1 >> 1)) / 64.0
            dist = (b3 | (b4 << 8)) / 4.0
            out.append((s, b0 >> 2, angle, dist))
            i += 5
        del self.buf[:i]
        return out


def ang_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


# ───────────────────────── الرسم ─────────────────────────
class Plot:
    def __init__(self, max_m):
        import matplotlib.pyplot as plt
        import numpy as np
        self.plt, self.np = plt, np
        plt.ion()
        self.fig = plt.figure("RPLIDAR C1")
        self.ax = self.fig.add_subplot(111, projection="polar")
        self.ax.set_theta_zero_location("N")   # 0° للأعلى
        self.ax.set_theta_direction(-1)         # الزاوية تزيد مع عقارب الساعة (مثل الليدار)
        self.ax.set_rmax(max_m)
        self.sc = self.ax.scatter([], [], s=3)
        self.last = 0.0

    def update(self, rot):
        now = time.time()
        if now - self.last < 0.25 or not self.plt.fignum_exists(self.fig.number):
            return
        self.last = now
        pts = [(a, d) for a, d, q in rot if d > 0]
        if not pts:
            return
        th = self.np.radians([a for a, d in pts])
        r = [d / 1000.0 for a, d in pts]
        self.sc.set_offsets(self.np.c_[th, r])
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)


# ───────────────────────── المسح ─────────────────────────
def scan(ser, seconds, warmup, front, want_csv, plot):
    send(ser, CMD_SCAN)
    length, dtype = read_descriptor(ser, "SCAN")
    if dtype != 0x81 or length != 5:
        print(f"  [WARN] Unexpected SCAN reply (type=0x{dtype:02X}, len={length}) - continuing")

    parser = NodeParser()
    if warmup > 0:
        print(f"... warm-up {warmup:.0f} s (motor spinning, not measuring)")
        t_end = time.time() + warmup
        while time.time() < t_end:
            parser.feed(ser.read(4096))
            if plot is None:
                left = int(t_end - time.time())
                print(f"\r   {left:4d} s left", end="", flush=True)
        print()
        parser.bad_bytes = 0

    rows, rotations, rot = [], [], []
    starts, total, valid, front_d = [], 0, 0, []
    t0 = time.time()
    t_end = t0 + seconds
    last_print = 0.0
    print(f"... scanning {seconds:.0f} s")
    while time.time() < t_end:
        for s, q, a, d in parser.feed(ser.read(4096)):
            now = time.time()
            if s == 1:
                starts.append(now)
                if rot:
                    rotations.append(rot)
                    if plot:
                        plot.update(rot)
                    if now - last_print > 1.0:
                        v = [(dd, aa) for aa, dd, qq in rot if dd > 0]
                        if v:
                            dmin, amin = min(v)
                            print(f"   nearest object: {dmin/10:6.1f} cm at angle {amin:6.1f} deg"
                                  f"   ({len(rot)} points/turn)")
                        last_print = now
                rot = []
            rot.append((a, d, q))
            total += 1
            if d > 0:
                valid += 1
                if ang_diff(a, front) <= 3.0:
                    front_d.append(d)
            if want_csv:
                rows.append((round(now - t0, 4), len(rotations), round(a, 3), d, q))
    stop(ser)
    return rotations, starts, total, valid, front_d, parser.bad_bytes, rows


# ───────────────────────── التشغيل ─────────────────────────
def main():
    ap = argparse.ArgumentParser(description="RPLIDAR C1 test")
    ap.add_argument("--port", help="e.g. COM5 or /dev/ttyUSB0 (auto if omitted)")
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument("--warmup", type=float, default=0, help="warm-up seconds before measuring (maker suggests 120+)")
    ap.add_argument("--front", type=float, default=0.0, help="angle to measure distance at (default 0)")
    ap.add_argument("--plot", action="store_true", help="live top-view plot")
    ap.add_argument("--max-m", type=float, default=4.0, help="plot radius in metres")
    ap.add_argument("--csv", action="store_true", help="save all points to logs/")
    a = ap.parse_args()

    print("=" * 56)
    print("  RPLIDAR C1 test")
    print("=" * 56)

    port, ports = find_port()
    if a.port:
        port = a.port
    print("[1] Serial ports found:")
    for p in ports:
        tag = "  <- lidar adapter (CP210x)" if (p.vid, p.pid) == (0x10C4, 0xEA60) else ""
        print(f"     {p.device:12s} {p.description}{tag}")
    if not port:
        print("[FAIL] Lidar adapter not found.")
        print("   - Use a DATA cable (some cables are charge-only)")
        print("   - Install the Silicon Labs CP210x driver, then unplug/replug")
        print("   - Or give the port yourself:  --port COM5")
        sys.exit(1)
    print(f"     using: {port} @ {BAUD}")

    results = {}
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"[FAIL] Cannot open {port}: {e}\n   (another program using it? close RoboStudio)")
    if hasattr(ser, "set_buffer_size"):
        try:
            ser.set_buffer_size(rx_size=65536)
        except Exception:
            pass

    plot = None
    try:
        print("[2] Device info:")
        info = get_info(ser)
        print(f"     model {info['model']} | firmware {info['firmware']}"
              f" | hardware {info['hardware']}")
        print(f"     serial {info['serial']}")
        results["info"] = True

        print("[3] Health:")
        st, err = get_health(ser)
        print(f"     {HEALTH_TXT.get(st, st)} | error code {err}")
        results["health"] = (st == 0)

        if a.plot:
            try:
                plot = Plot(a.max_m)
            except Exception as e:
                print(f"  [WARN] Plot unavailable ({e}) - pip install matplotlib. Continuing without plot.")

        print("[4] Scan:")
        rots, starts, total, valid, front_d, bad, rows = scan(
            ser, a.seconds, a.warmup, a.front, a.csv, plot)
    except (TimeoutError, ValueError) as e:
        print(f"[FAIL] {e}")
        print("   - baud must be 460800 | unplug 5 s and replug | close other programs on the port")
        stop(ser)
        ser.close()
        sys.exit(1)
    except KeyboardInterrupt:
        stop(ser)
        ser.close()
        sys.exit("\nScan stopped.")
    ser.close()

    # ── النتائج ──
    print("=" * 56)
    print("  RESULTS (measured)")
    print("=" * 56)
    if len(starts) >= 2:
        rate = (len(starts) - 1) / (starts[-1] - starts[0])
    else:
        rate = 0.0
    full = [r for r in rots[1:]]  # أول لفة ناقصة غالباً
    ppr = statistics.mean(len(r) for r in full) if full else 0
    pct = 100.0 * valid / total if total else 0.0
    print(f"  rotation speed:     {rate:5.2f} turns/s  (maker: about 10)")
    print(f"  points per turn:    {ppr:6.0f}")
    print(f"  valid points:       {pct:5.1f}%   (should be high inside a room)")
    print(f"  out-of-sync bytes:  {bad}")
    if front_d:
        med = statistics.median(front_d)
        sd = statistics.pstdev(front_d) if len(front_d) > 1 else 0.0
        print(f"  distance at {a.front:.0f} deg (+-3): median {med/10:.1f} cm | spread {sd/10:.1f} cm | {len(front_d)} points")
    else:
        print(f"  distance at {a.front:.0f} deg: no valid readings (nothing in front?)")

    results["scan"] = 5 <= rate <= 15 and ppr > 100
    print("-" * 56)
    for k, label in (("info", "device info read"), ("health", "health is Good"),
                     ("scan", "scanning works (5-15 turns/s and >100 points/turn)")):
        print(f"  {'[OK]  ' if results.get(k) else '[FAIL]'} {label}")
    ok = all(results.get(k) for k in ("info", "health", "scan"))
    print("=" * 56)
    print("  GO - lidar is healthy" if ok else "  NO-GO - copy the whole output to Claude")
    print("=" * 56)

    if a.csv and rows:
        os.makedirs("logs", exist_ok=True)
        fn = time.strftime("logs/c1_%Y%m%d_%H%M%S.csv")
        with open(fn, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "rotation", "angle_deg", "dist_mm", "quality"])
            w.writerows(rows)
        print(f"  points saved: {fn}")

    if plot:
        print("  Close the plot window to exit.")
        plot.plt.ioff()
        plot.plt.show()


if __name__ == "__main__":
    main()
