#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_slip.py — الانزلاق أثناء الدوران بالمكان: ما يقوله الإنكودر مقابل الجايرو
================================================================================
🔴 الروبوت **على الأرض** في مساحة خالية (نصف متر حوله على الأقل).

الدوران بالمكان أثقل مناورة: العجلات تُجرّ جانبياً على الأرض، فما يدّعيه
الإنكودر من دوران لا يحدث كلّه فعلاً. هذا السكربت يضع الرقمين جنباً إلى جنب:

    ω_enc = (R − L) / 0.141          [rad/s — من الإنكودر · 0.141 = TRACK_WIDTH]
    ω_gyr = gz × π / 180             [rad/s — من الجايرو، بافتراض gz بـ°/ث]
    slip  = |ω_enc − ω_gyr|

لكل سرعة في --speeds (افتراضياً 0.1 و0.2 و0.3 م/ث لكل عجلة) خمس لفّات
بالمكان، وفي كل عيّنة يُسجَّل معها **الفارق الزمني بين وصول T:1001 وT:1002**.

⚠ **القراءتان من لحظتين مختلفتين**: L/R من ردّ، وgz من ردّ آخر يصل بعده.
فجزء من «الانزلاق» المقاس قد يكون **فارق زمن** لا انزلاقاً — خاصةً أثناء
التسارع حيث تتغيّر السرعة بين الردّين. هذا ما سيعالجه تعديل الفيرموير لاحقاً،
وtest_sync يقيس الفارق وحده والروبوت ساكن.

⚠ **لا يستنتج معاملاً ولا يقترح تعديلاً — أرقام فقط.** معامل انزلاق من
خمس لفّات على أرضية واحدة ببطارية واحدة يُتبنّى كأنه ثابت فيزيائي ثم يُطارَد
شهراً حين يكذب على أرضية أخرى. الاشتقاق جلسة مستقلة تقرأ ملف CSV.

⚠ **الإشارة لا تُفترض**: لو كانت اصطلاحات إشارة الإنكودر والجايرو متعاكسة
لصار slip ≈ |ω_enc| + |ω_gyr| بلا معنى. السكربت يعدّ العيّنات المتوافقة
الإشارة ويرسب إن كانت الأغلبية متعاكسة — **ولا يقلب شيئاً بنفسه**.

الجدول يُحسب على عيّنات **الاستقرار** (بعد --settle-s من بدء كل لفّة)، لأن
التسارع يضخّم أثر الفارق الزمني. والملف يحوي **كل** العيّنات مع عمود
phase = ramp|steady لمن أراد إعادة الحساب.

الأمان:
  • تأكيد "yes" قبل أي حركة
  • --timeout حدّ أقصى صارم لكل لفّة (افتراضي 3ث)؛ لفّة أطول منه تُرفض لا تُقصّ
  • إعادة إرسال أمر السرعة كل 800ms (نبضة القلب في الفيرموير 3000ms)
  • finally: {"T":1,"L":0,"R":0} دائماً — حتى مع Ctrl+C أو الخطأ
  • اتجاه اللفّ يتناوب افتراضياً فيبقى الروبوت في مكانه ولا يلتفّ كابل

التشغيل:
    python3 -m pi.tests.ugv01.test_slip
    python3 -m pi.tests.ugv01.test_slip --speeds 0.1,0.2,0.3 --reps 5 --spin-s 2.0
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys
import threading
import time

try:
    import serial  # pyserial — الاعتماد الوحيد لهذه السكربتات
except ImportError:
    sys.exit("خطأ: pyserial غير مثبّتة.  pip3 install pyserial")


# ═══ أدوات مستقلة ═══════════════════════════════════════════════════════
# مكرّرة عمداً في كل سكربت هنا: هذه اختبارات يوم أول، ولو فشل استيراد من
# pi.* لأي سبب وجب أن تبقى عاملة. الاستقلال ميزة لا تكرار.
BAUD = 115200
# المنفذ المثبت على العتاد (قياس لا افتراض): UART4 على GPIO8/9، والبقية
# احتياط فقط — serial0 على هذه الراسبري ليس وصلة اللوحة.
PRIMARY_PORT = "/dev/ttyAMA4"
PORT_CANDIDATES = (PRIMARY_PORT, "/dev/serial0", "/dev/ttyUSB*", "/dev/ttyACM*")
PRIMARY_MISSING_HINT = (
    "⚠ %s غير موجود — وهو منفذ لوحة UGV01 المثبت (UART4)." % PRIMARY_PORT,
    "   • /boot/firmware/config.txt تحت [all] يجب أن يحوي:  dtoverlay=uart4",
    "     (مع enable_uart=1 · dtoverlay=disable-bt · dtparam=uart0=on) ثم أعد الإقلاع.",
    "   • الأسلاك: TX: GPIO8 → الدبوس 24 (إلى RX اللوحة) · RX: GPIO9 → الدبوس 21 (من TX اللوحة) · GND مشترك.",
    "   • تحقّق:  ls -l /dev/ttyAMA*   و   grep -n uart4 /boot/firmware/config.txt",
)
BOOT_WAIT_S = 3.0
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

INIT_CMDS = (
    ({"T": 900, "main": 3, "module": 0}, "نوع الروبوت UGV01 (mainType=3) — إلزامي"),
    ({"T": 143, "cmd": 0}, "إطفاء صدى السيريال"),
    ({"T": 605, "cmd": 0}, "إطفاء طباعة التشخيص"),
    ({"T": 131, "cmd": 0}, "إطفاء البثّ المستمر — إطاراته تُقرن بالسؤال الخطأ"),
)


def find_port(explicit=None):
    """يعيد --port إن مُرِّر، وإلا أول مرشّح موجود بالترتيب، وNone إن لم يوجد شيء.
    غياب ttyAMA4 يُطبع تشخيصه صراحةً: السقوط الصامت إلى serial0 يفتح منفذاً
    حيّاً لجهاز آخر فيبدو العطل «اللوحة لا تردّ» وهو في config.txt."""
    if explicit:
        return explicit
    if not os.path.exists(PRIMARY_PORT):
        for line in PRIMARY_MISSING_HINT:
            print(line)
    for pattern in PORT_CANDIDATES:
        hits = sorted(glob.glob(pattern))
        if hits:
            if hits[0] != PRIMARY_PORT:
                print("⚠ سقوط احتياطي إلى %s — ليس المنفذ المثبت؛ تأكّد أنه اللوحة فعلاً."
                      % hits[0])
            return hits[0]
    return None


class Link:
    """غلاف رفيع حول pyserial مع **طابع زمني لكل سطر** عند وصول نهايته.

    ⚠ الفرق عن بقية السكربتات مقصود: `read(256)` ينتظر امتلاء 256 بايت أو
    انقضاء المهلة، فيُكمّم زمن الوصول بمقدار المهلة كلها — وهو هنا المقيس
    نفسه. لذلك تُقرأ البايتات المتاحة فوراً (`in_waiting`، وبايت واحد على
    الأقل) فتعود القراءة مع أول بايت يصل."""

    def __init__(self, ser):
        self.ser = ser
        self._buf = b""
        self._lock = threading.Lock()

    def send(self, obj):
        payload = (json.dumps(obj, separators=(",", ":")) + "\n").encode("ascii")
        with self._lock:
            self.ser.write(payload)
            self.ser.flush()

    def lines(self, seconds):
        """يولّد (نص، كائن، لحظة الوصول، طول السطر بالبايت) حتى انقضاء المهلة."""
        t_end = time.monotonic() + seconds
        while True:
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            t_arr = time.monotonic()
            if chunk:
                self._buf += chunk
                while b"\n" in self._buf:
                    raw, self._buf = self._buf.split(b"\n", 1)
                    nbytes = len(raw) + 1
                    text = raw.decode("ascii", errors="replace").strip()
                    if not text:
                        continue
                    obj = None
                    if text.startswith("{"):
                        try:
                            parsed = json.loads(text)
                        except ValueError:
                            parsed = None
                        if isinstance(parsed, dict):
                            obj = parsed
                    yield text, obj, t_arr, nbytes
            if time.monotonic() >= t_end:
                return

    def drain(self):
        """يرمي المتراكم قبل السؤال — الردّ القديم يُقرن بالسؤال الجديد فيكذب."""
        self._buf = b""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def ask_pair(self, seconds):
        """يرسل T:130 ثم T:126 متتاليين، وينتظر T:1001 وT:1002 معاً.

        يعيد قاموساً: لحظتا الإرسال، ولحظتا الوصول (None لما لم يصل)،
        والكائنان، وطولا السطرين، والأسطر الشاردة."""
        self.drain()
        out = {"t_send130": None, "t_send126": None, "t1001": None, "t1002": None,
               "o1001": None, "o1002": None, "n1001": 0, "n1002": 0, "strays": []}
        out["t_send130"] = time.monotonic()
        self.send({"T": 130})
        out["t_send126"] = time.monotonic()
        self.send({"T": 126})
        for text, obj, t_arr, nbytes in self.lines(seconds):
            tag = obj.get("T") if obj is not None else None
            if tag == 1001 and out["o1001"] is None:
                out["o1001"], out["t1001"], out["n1001"] = obj, t_arr, nbytes
            elif tag == 1002 and out["o1002"] is None:
                out["o1002"], out["t1002"], out["n1002"] = obj, t_arr, nbytes
            else:
                out["strays"].append(text)
            if out["o1001"] is not None and out["o1002"] is not None:
                break
        return out


def open_link(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.05, exclusive=True)
    except TypeError:                 # إصدارات pyserial قديمة بلا exclusive
        ser = serial.Serial(port, baud, timeout=0.05)
    return Link(ser)


def init_board(link, gap=0.30, verbose=True):
    """تهيئة الإقلاع — تُرسل في كل سكربت لأن اللوحة تنساها مع كل إقلاع."""
    for cmd, why in INIT_CMDS:
        if verbose:
            print("   → %-32s  %s" % (json.dumps(cmd, separators=(",", ":")), why))
        link.send(cmd)
        time.sleep(gap)
    link.ser.reset_input_buffer()


def new_csv(name, header):
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, "%s_%s.csv" % (name, time.strftime("%Y%m%d_%H%M%S")))
    fh = open(path, "w", newline="", encoding="utf-8")
    writer = csv.writer(fh)
    writer.writerow(header)
    return fh, writer, path


def verdict(ok, reasons):
    print()
    print("═" * 66)
    for line in reasons:
        print("  " + line)
    print("═" * 66)
    print("GO" if ok else "NO-GO")
    return 0 if ok else 1


STOP_CMD = {"T": 1, "L": 0, "R": 0}
MAX_SPEED_MPS = 0.5          # أقصى سرعة عملية عبر T:1 على UGV01
RESEND_MS = 800              # تجديد أمر السرعة — نبضة القلب في الفيرموير 3000ms
TRACK_WIDTH_M = 0.141        # UGV01 (mainType=3) من مصدر الفيرموير
ENC_MOVING_RAD_S = 0.05      # تحتها ω_enc يُعدّ «لا دوران» ولا تُحكم إشارته
GYRO_RANGES_DPS = (250.0, 500.0, 1000.0, 2000.0)   # مدى جايرو شائعة — لكشف التشبّع


def num(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def hard_stop(link, times=3):
    """إيقاف لا يرفع استثناءً أبداً — مسار الإيقاف لا يجوز أن يفشل بنفسه."""
    for _ in range(times):
        try:
            link.send(STOP_CMD)
        except Exception:
            pass
        time.sleep(0.05)


def confirm_motion(placement):
    print()
    print("⚠" * 33)
    print("⚠ هذا السكربت **يشغّل المحركات** (دوران بالمكان بعدة سرعات).")
    print("⚠ وضع الروبوت المطلوب: %s" % placement)
    print("⚠" * 33)
    try:
        answer = input('اكتب "yes" للمتابعة (أي شيء آخر يُلغي): ').strip().lower()
    except EOFError:
        answer = ""
    if answer != "yes":
        print("أُلغي بناء على طلبك.")
        return False
    return True


class Keepalive(threading.Thread):
    """خيط يجدّد أمر السرعة كل RESEND_MS — اللوحة توقف المحركات بلا أوامر."""

    def __init__(self, link, period_s):
        threading.Thread.__init__(self, daemon=True)
        self.link = link
        self.period = period_s
        self._cmd = dict(STOP_CMD)
        self._cmd_lock = threading.Lock()
        # ⚠ `_stop_evt` لا `_stop`: ‏threading.Thread يملك `_stop()` داخلياً
        # ويستدعيها join()، فتظليلها يجعل join يرمي داخل finally قبل الإيقاف.
        self._stop_evt = threading.Event()

    def set_speed(self, left, right):
        with self._cmd_lock:
            self._cmd = {"T": 1, "L": left, "R": right}
        self.link.send(self._cmd)

    def run(self):
        while not self._stop_evt.wait(self.period):
            with self._cmd_lock:
                cmd = dict(self._cmd)
            try:
                self.link.send(cmd)
            except Exception:
                pass

    def shutdown(self):
        self._stop_evt.set()


def sign(x):
    return (x > 0) - (x < 0)


def spin_once(link, keeper, writer, speed, rep, direction, spin_s, settle_s, reply_timeout):
    """لفّة واحدة بالمكان: يأمر، ويجمع أزواج T:1001/T:1002 حتى انقضاء spin_s.

    direction = +1 ⇒ L=−v وR=+v (R−L موجب) · ‏−1 ⇒ العكس.
    يعيد قائمة العيّنات (قواميس). لا يوقف المحركات — المستدعي يفعل."""
    left, right = -direction * speed, direction * speed
    samples = []
    t0 = time.monotonic()
    keeper.set_speed(left, right)
    while True:
        remaining = spin_s - (time.monotonic() - t0)
        if remaining <= 0:
            break
        # المهلة لا تتجاوز ما بقي: فلا تمتدّ الحركة بعد spin_s (ومنه بعد --timeout)
        r = link.ask_pair(min(reply_timeout, remaining))
        if r["o1001"] is None or r["o1002"] is None:
            continue
        L, R = num(r["o1001"].get("L")), num(r["o1001"].get("R"))
        gz = num(r["o1002"].get("gz"))
        if L is None or R is None or gz is None:
            continue
        t_s = r["t1001"] - t0
        w_enc = (R - L) / TRACK_WIDTH_M
        w_gyr = gz * math.pi / 180.0
        slip = abs(w_enc - w_gyr)
        dt_ms = (r["t1002"] - r["t1001"]) * 1000.0
        phase = "steady" if t_s >= settle_s else "ramp"
        s = {"t_s": t_s, "L": L, "R": R, "gz": gz, "w_enc": w_enc, "w_gyr": w_gyr,
             "slip": slip, "dt_ms": dt_ms, "phase": phase}
        samples.append(s)
        writer.writerow([speed, rep, direction, left, right, round(t_s, 4), phase,
                         L, R, gz, round(w_enc, 5), round(w_gyr, 5), round(slip, 5),
                         round(dt_ms, 3)])
    return samples


def summarize(speed, runs):
    """إحصاء سرعة واحدة على عيّنات الاستقرار. runs = [قائمة عيّنات لكل لفّة]."""
    steady = [s for run in runs for s in run if s["phase"] == "steady"]
    per_rep = [statistics.mean(s["slip"] for s in run if s["phase"] == "steady")
               for run in runs if any(s["phase"] == "steady" for s in run)]
    out = {"speed": speed, "n": len(steady), "reps": len(per_rep)}
    if not steady:
        return out
    slips = [s["slip"] for s in steady]
    dts = [s["dt_ms"] for s in steady]
    moving = [s for s in steady if abs(s["w_enc"]) >= ENC_MOVING_RAD_S]
    agree = sum(1 for s in moving if sign(s["w_enc"]) == sign(s["w_gyr"]))
    out.update({
        "slip_mean": statistics.mean(slips),
        "slip_sd": statistics.stdev(slips) if len(slips) >= 2 else 0.0,
        "rep_sd": statistics.stdev(per_rep) if len(per_rep) >= 2 else 0.0,
        "abs_enc": statistics.mean(abs(s["w_enc"]) for s in steady),
        "abs_gyr": statistics.mean(abs(s["w_gyr"]) for s in steady),
        "dt_mean": statistics.mean(dts),
        "dt_max": max(dts),
        "moving": len(moving),
        "agree": agree,
        "gz_absmax": max(abs(s["gz"]) for s in steady),
        "gz_top": 0,
    })
    top = out["gz_absmax"]
    out["gz_top"] = sum(1 for s in steady if abs(s["gz"]) >= 0.99 * top)
    return out


# ═══ الاختبار ════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="UGV01: الانزلاق أثناء الدوران بالمكان")
    ap.add_argument("--port", default=None, help="افتراضياً: بحث تلقائي")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--boot-wait", type=float, default=BOOT_WAIT_S)
    ap.add_argument("--speeds", default="0.1,0.2,0.3",
                    help="سرعات العجلة م/ث مفصولة بفواصل")
    ap.add_argument("--reps", type=int, default=5, help="لفّات لكل سرعة")
    ap.add_argument("--spin-s", type=float, default=2.0, help="مدة كل لفّة (ث)")
    ap.add_argument("--settle-s", type=float, default=0.5,
                    help="ما قبله من كل لفّة «تسارع» ولا يدخل الجدول (ث)")
    ap.add_argument("--rest-s", type=float, default=2.0,
                    help="سكون بين اللفّات — يمنع دخول قصور لفّة في التالية (ث)")
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="الحدّ الأقصى الصارم لكل لفّة (ث)")
    ap.add_argument("--direction", choices=("alt", "ccw", "cw"), default="alt",
                    help="alt يتناوب · ccw: ‏R−L موجب · cw: ‏R−L سالب")
    ap.add_argument("--reply-timeout", type=float, default=0.3)
    a = ap.parse_args()

    print("═" * 66)
    print("  UGV01 · الانزلاق أثناء الدوران بالمكان (test_slip)")
    print("═" * 66)

    # ── فحص المدخلات قبل أي منفذ أو حركة ────────────────────────────────
    try:
        speeds = [float(x) for x in a.speeds.split(",") if x.strip()]
    except ValueError:
        return verdict(False, ["--speeds غير صالحة: %r" % a.speeds])
    bad = [v for v in speeds if not (0 < v <= MAX_SPEED_MPS)]
    if not speeds or bad:
        return verdict(False, ["سرعات خارج النطاق العملي (0, %.2f] م/ث: %s"
                               % (MAX_SPEED_MPS, bad or "لا شيء")])
    if a.spin_s > a.timeout:
        return verdict(False, [
            "--spin-s %.1f أطول من --timeout %.1f — اللفّة تُرفض ولا تُقصّ في صمت."
            % (a.spin_s, a.timeout),
            "ارفع --timeout صراحةً إن كان ذلك مقصوداً."])
    if a.settle_s >= a.spin_s:
        return verdict(False, ["--settle-s %.1f ≥ --spin-s %.1f ⇒ لا عيّنة استقرار."
                               % (a.settle_s, a.spin_s)])
    if a.reps < 1:
        return verdict(False, ["--reps يجب أن يكون 1 على الأقل."])

    port = find_port(a.port)
    if not port:
        return verdict(False, ["لم يُعثر على أي منفذ: %s" % ", ".join(PORT_CANDIDATES)])
    print("المنفذ: %s @ %d" % (port, a.baud))
    total = len(speeds) * a.reps
    print("الخطة: سرعات %s م/ث × %d لفّة = %d لفّة · %.1fث لكل لفّة · اتجاه %s"
          % (", ".join("%.2f" % v for v in speeds), a.reps, total, a.spin_s, a.direction))
    for v in speeds:
        w = 2 * v / TRACK_WIDTH_M
        print("   %.2f م/ث ⇒ ω_enc المتوقَّع ≈ %.2f rad/s ≈ %.0f °/ث"
              % (v, w, math.degrees(w)))

    if not confirm_motion("🔴 **على الأرض** في مساحة خالية (نصف متر حوله على الأقل)"):
        return 2

    try:
        link = open_link(port, a.baud)
    except Exception as exc:
        return verdict(False, ["تعذّر فتح المنفذ: %s" % exc])

    fh, keeper = None, None
    completed = 0
    try:
        print("\n[1] انتظار إقلاع اللوحة %.1fث…" % a.boot_wait)
        for _ in link.lines(a.boot_wait):
            pass
        link.ser.reset_input_buffer()

        print("\n[2] تهيئة الإقلاع:")
        init_board(link)

        fh, writer, path = new_csv("slip", [
            "speed", "rep", "direction", "cmd_L", "cmd_R", "t_s", "phase",
            "L", "R", "gz", "w_enc", "w_gyr", "slip", "dt_1001_1002_ms"])

        keeper = Keepalive(link, RESEND_MS / 1000.0)
        keeper.start()

        results = {}
        print("\n[3] اللفّات:")
        for v in speeds:
            runs = []
            for rep in range(1, a.reps + 1):
                if a.direction == "alt":
                    direction = 1 if rep % 2 else -1
                else:
                    direction = 1 if a.direction == "ccw" else -1
                run = spin_once(link, keeper, writer, v, rep, direction,
                                a.spin_s, a.settle_s, a.reply_timeout)
                keeper.set_speed(0, 0)
                hard_stop(link, times=1)
                completed += 1
                runs.append(run)
                st = [s for s in run if s["phase"] == "steady"]
                if st:
                    print("   v=%.2f  #%d  %s  عيّنات %3d  slip وسطي %.3f rad/s  Δt وسطي %.1f ms"
                          % (v, rep, "R−L>0" if direction > 0 else "R−L<0", len(st),
                             statistics.mean(s["slip"] for s in st),
                             statistics.mean(s["dt_ms"] for s in st)))
                else:
                    print("   v=%.2f  #%d  ✗ لا عيّنات استقرار" % (v, rep))
                fh.flush()
                time.sleep(a.rest_s)
            results[v] = summarize(v, runs)

        # ── الجدول ──────────────────────────────────────────────────────
        print("\n[4] الجدول (عيّنات الاستقرار فقط · rad/s · ms):\n")
        print("   سرعة   عيّنات  slip وسطي   σ(عيّنات)  σ(بين اللفّات)   |ω_enc|  |ω_gyr|   Δt وسطي  Δt أقصى")
        print("   " + "─" * 96)
        for v in speeds:
            s = results[v]
            if not s["n"]:
                print("   %.2f   %6d   —" % (v, 0))
                continue
            print("   %.2f   %6d   %8.3f   %9.3f   %13.3f   %7.3f  %7.3f   %7.1f  %7.1f"
                  % (v, s["n"], s["slip_mean"], s["slip_sd"], s["rep_sd"],
                     s["abs_enc"], s["abs_gyr"], s["dt_mean"], s["dt_max"]))
        print()
        print("   ⚠ القراءتان من لحظتين مختلفتين (العمودان Δt): جزء من «الانزلاق» أعلاه")
        print("     قد يكون فارق زمن لا انزلاقاً. تعديل الفيرموير سيعالج ذلك لاحقاً.")
        print("   ⚠ أرقام فقط — لا معامل مشتقّ ولا تعديل مقترح.")

        # ── الحكم: صلاحية البيانات لا جودة الروبوت ──────────────────────
        ok = True
        reasons = ["لفّات مكتملة: %d/%d" % (completed, total)]
        for v in speeds:
            s = results[v]
            if not s["n"]:
                ok = False
                reasons.append("🔴 v=%.2f: لا عيّنات استقرار ⇒ لا رقم لهذه السرعة." % v)
                continue
            if s["moving"] == 0:
                ok = False
                reasons.append("🔴 v=%.2f: ω_enc ≈ 0 ⇒ الإنكودر لا يرى دوراناً"
                               " (شغّل test_encoder)." % v)
                continue
            if s["agree"] * 2 <= s["moving"]:
                ok = False
                reasons.append("🔴 v=%.2f: الإشارة متعاكسة في %d/%d عيّنة ⇒ slip المحسوب"
                               % (v, s["moving"] - s["agree"], s["moving"]))
                reasons.append("   بلا معنى حتى تُحسم اصطلاحات الإشارة (لا يُقلب شيء هنا).")
            if s["gz_top"] >= 3 and any(abs(s["gz_absmax"] - g) <= 0.02 * g
                                        for g in GYRO_RANGES_DPS):
                reasons.append("⚠ v=%.2f: |gz| يتسطّح عند %.1f °/ث (%d عيّنة) ⇒ احتمال"
                               " تشبّع الجايرو؛ ω_gyr ناقص." % (v, s["gz_absmax"], s["gz_top"]))
            if s["abs_enc"] > 0 and s["abs_gyr"] < 0.05 * s["abs_enc"]:
                reasons.append("⚠ v=%.2f: |ω_gyr| أصغر من 5%% من |ω_enc| ⇒ تحقّق من وحدة gz"
                               " (الصيغة تفترض °/ث)." % v)
        if ok:
            reasons.append("✅ كل سرعة لها عيّنات استقرار وإشارتان متوافقتان.")
        reasons.append("⚠ القراءتان من لحظتين مختلفتين — جزء من الانزلاق قد يكون فارق زمن.")
        reasons.append("السجل: " + path)
        return verdict(ok, reasons)

    except KeyboardInterrupt:
        print("\nأُوقف بـCtrl+C — يُرسل الإيقاف الآن.")
        return 2
    finally:
        # لا استثناء يعبر مسار الإيقاف: كل خطوة معزولة، والإيقاف يُنفَّذ
        # مهما فشل ما قبله.
        if keeper:
            try:
                keeper.set_speed(0, 0)   # أي إرسال متأخّر من الخيط يصير إيقافاً
            except Exception:
                pass
            try:
                keeper.shutdown()
                keeper.join(timeout=2.0)
            except Exception:
                pass
        try:
            hard_stop(link)
        except Exception:
            pass
        if fh:
            fh.close()
        try:
            link.ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
