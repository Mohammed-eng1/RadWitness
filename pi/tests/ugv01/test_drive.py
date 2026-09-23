#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_drive.py — حركة على الأرض: ما يقوله الروبوت مقابل ما تقوله المسطرة
================================================================================
🔴 الروبوت **على الأرض** وحوله مساحة خالية بمقدار المسافة المطلوبة + هامش.

هذا السكربت يجمع بيانات خام لا أكثر. لكل شوط ثلاثة أرقام تُحفظ جنباً إلى جنب:
  المأمور  ·  ما يقوله الروبوت عن نفسه  ·  ما تقوله المسطرة/المنقلة

  --straight <متر>  يمشي بأمر سرعة ومدة، ويكامل L/R من الإنكودر، ثم يسألك
                    المسافة الحقيقية بالمسطرة.
  --turn <درجة>     يلفّ بالمكان ويكامل gz، ثم يسألك الزاوية الحقيقية المقاسة.

خمسة تكرارات لكل أمر، ويُطبع تشتّت كل عمود على حدة.

⚠ **لا يُحسب هنا أي معامل ولا يُقترح أي تعديل إعدادات.** الأرقام تُطبع وتُحفظ
في CSV فقط. سبب هذا صريح: معامل يُشتقّ من خمسة أشواط على أرضية واحدة ببطارية
واحدة يُتبنّى كأنه ثابت فيزيائي، ثم يُطارَد شهراً حين يكذب على أرضية أخرى.
الاشتقاق جلسة معايرة مستقلة تقرأ هذه الملفات.

⚠ تكامل gz يفترض أن الوحدة **درجة/ثانية**. إن كانت رادياناً/ث فالرقم سيصغر
~57 ضعفاً عن قراءة المنقلة — وهذا ما ستكشفه المقارنة نفسها. لا يصحّح
السكربت شيئاً بناءً عليه.

⚠ إشارة اللفّ (أي اتجاه يقابل L=+ و R=−) **غير مفترضة**: الأمر المُرسل يُطبع
ويُحفظ كما هو، فتُستخرج الخريطة من البيانات لا من التخمين.

الأمان: تأكيد "yes" · --timeout حدّ صارم لكل شوط · finally يرسل الإيقاف دائماً.

التشغيل:
    python3 -m pi.tests.ugv01.test_drive --straight 1.0 --timeout 8
    python3 -m pi.tests.ugv01.test_drive --turn 90 --timeout 5
    python3 -m pi.tests.ugv01.test_drive --straight 0.5 --turn 90 --repeats 5 --timeout 6
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import statistics
import sys
import threading
import time

try:
    import serial
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
)

STOP_CMD = {"T": 1, "L": 0, "R": 0}
MAX_SPEED_MPS = 0.5
RESEND_MS = 800
BIAS_SECONDS = 1.5        # نافذة قياس انحياز gz قبل كل لفّة
BIAS_OUTLIER_SIGMA = 3.0  # في نافذة الانحياز: ما بعد 3σ عن الوسيط شاذّ يُستبعد
# قفزات gz المنفردة المقاسة ساكناً (2026-09-23): ±15.9°/ث في 3 من 200 عيّنة
# (= 256 عدّة بالضبط عند 16 عدّة لكل °/ث). عتبة العدّ نصفها: فوق ضجيج σ≈0.23
# بمراحل، ودون القفزة بهامش. تُعدّ فقط — الاستبدال يفعله الوسيط نفسه.
SPIKE_JUMP_DPS = 8.0


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
    """غلاف رفيع حول pyserial: أسطر JSON داخلة/خارجة، بلا أي اعتماد على pi.*"""

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
        t_end = time.monotonic() + seconds
        while True:
            chunk = self.ser.read(256)
            if chunk:
                self._buf += chunk
                while b"\n" in self._buf:
                    raw, self._buf = self._buf.split(b"\n", 1)
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
                    yield text, obj
            if time.monotonic() >= t_end:
                return

    def drain(self):
        """يرمي كل ما تراكم قبل السؤال. الردّ **القديم** أخطر من غياب الردّ:
        إطاراً بثٍّ متراكماً أو ردّاً وصل بعد انتهاء مهلته يجعل كل قراءة
        لاحقة متأخّرة إطاراً كاملاً، فتقرأ حالةً مضت وتظنّها الآن."""
        self._buf = b""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def ask(self, cmd, want_t, seconds):
        self.drain()
        strays = []
        t0 = time.monotonic()
        self.send(cmd)
        for text, obj in self.lines(seconds):
            if obj is not None and obj.get("T") == want_t:
                return obj, time.monotonic() - t0, strays
            strays.append(text)
        return None, time.monotonic() - t0, strays


def open_link(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.05, exclusive=True)
    except TypeError:
        ser = serial.Serial(port, baud, timeout=0.05)
    return Link(ser)


def init_board(link, gap=0.30, verbose=True):
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
    print("⚠ هذا السكربت **يشغّل المحركات**.")
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


def ask_float(prompt):
    """يقرأ رقماً من المستخدم؛ السطر الفارغ = «لم أقس» ويُحفظ فراغاً لا صفراً."""
    while True:
        try:
            raw = input(prompt).strip()
        except EOFError:
            return None
        if not raw:
            return None
        raw = raw.replace("٫", ".").replace(",", ".")
        try:
            return float(raw)
        except ValueError:
            print("      رقم غير صالح — أعد الإدخال (أو اتركه فارغاً للتخطّي).")


class Keepalive(threading.Thread):
    """خيط يجدّد أمر السرعة كل RESEND_MS — اللوحة توقف المحركات بلا أوامر."""

    def __init__(self, link, period_s):
        threading.Thread.__init__(self, daemon=True)
        self.link = link
        self.period = period_s
        self._cmd = dict(STOP_CMD)
        self._cmd_lock = threading.Lock()
        # ⚠ الاسم `_stop_evt` لا `_stop`: ‏threading.Thread يملك `_stop()`
        # داخلياً ويستدعيها join()، فتظليلها بـEvent يجعل join يرمي
        # TypeError — **داخل finally وقبل الإيقاف**، فتبقى المحركات تدور.
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


def stats_line(label, values, unit):
    """تشتّت عمود واحد — وصفٌ للبيانات لا اشتقاق معامل منها."""
    clean = [v for v in values if v is not None]
    if not clean:
        return "   %-26s —  (لا قيم)" % label
    mean = statistics.mean(clean)
    sd = statistics.stdev(clean) if len(clean) > 1 else 0.0
    return ("   %-26s متوسّط %+8.3f %s · σ %6.3f · أدنى %+8.3f · أقصى %+8.3f · ن=%d"
            % (label, mean, unit, sd, min(clean), max(clean), len(clean)))


# ═══ شوط سير مستقيم ══════════════════════════════════════════════════════
def run_straight(link, keeper, distance, speed, timeout):
    """يمشي مدةً محسوبة من السرعة، ويكامل L/R المقاستين. يعيد قاموس القياس."""
    duration = abs(distance) / abs(speed)
    if duration > timeout:
        return {"error": "المدة اللازمة %.2fث تتجاوز --timeout %.2fث" % (duration, timeout)}

    direction = 1.0 if distance >= 0 else -1.0
    cmd_speed = direction * abs(speed)
    print("      الأمر: L=%+.3f R=%+.3f م/ث لمدة %.2fث" % (cmd_speed, cmd_speed, duration))

    keeper.set_speed(cmd_speed, cmd_speed)
    t0 = time.monotonic()
    t_prev = t0
    int_l = int_r = 0.0
    samples = 0
    deadline = t0 + min(duration, timeout)
    while time.monotonic() < deadline:
        obj, _, _ = link.ask({"T": 130}, 1001, 0.2)
        now = time.monotonic()
        dt = now - t_prev
        t_prev = now
        if obj is None:
            continue
        left, right = num(obj.get("L")), num(obj.get("R"))
        if left is None or right is None:
            continue
        int_l += left * dt
        int_r += right * dt
        samples += 1
    keeper.set_speed(0, 0)
    elapsed = time.monotonic() - t0
    hard_stop(link, times=1)
    time.sleep(0.6)                       # القصور الذاتي ينتهي قبل القياس بالمسطرة
    return {"duration_s": elapsed, "cmd_speed": cmd_speed,
            "enc_L_m": int_l, "enc_R_m": int_r,
            "enc_mean_m": (int_l + int_r) / 2.0, "samples": samples}


# ═══ شوط لفّ بالمكان ═════════════════════════════════════════════════════
def robust_bias(values, k=BIAS_OUTLIER_SIGMA):
    """انحياز بالوسيط بعد استبعاد ما خرج عن k·σ حول الوسيط.

    ⚠ لا بالمتوسط: قفزة ±16°/ث واحدة في نافذة ~27 عيّنة تزيح المتوسط ~0.6°/ث،
    وهذا ما حدث فعلاً في لفّتين من جلسة 2026-09-23 (−1.34 و−1.44 بدل ~−0.25) —
    ثم يُطرح الانحياز الخاطئ من كل عيّنة فيتراكم خطأً في الزاوية.
    يعيد (الانحياز، عدد المستعمَل، عدد المستبعَد)."""
    med = statistics.median(values)
    if len(values) < 3:
        return med, len(values), 0
    sd = statistics.stdev(values)
    kept = [v for v in values if abs(v - med) <= k * sd] or values
    return statistics.median(kept), len(kept), len(values) - len(kept)


class Median3Integrator:
    """تكامل gz بعد مرشّح وسيط ثلاثي: كل عيّنة تُستبدل بوسيط (السابقة، هي، التالية).

    قفزة منفردة لا تنجو من الوسيط أبداً، ومنحدر رتيب (تسارع/تباطؤ حقيقي) يمرّ
    كما هو لأن العيّنة الوسطى فيه هي الوسيط أصلاً.
    ⚠ الثمن: عيّنة تأخير — العيّنة تُثبَّت بعد وصول جارتها. فقرار «بلغنا الهدف»
    يقرأ `provisional()` = المثبَّت + العيّنة المعلّقة خاماً، وإلا تأخّر القطع
    عيّنةً كاملة (~5° عند 88°/ث و18 هرتز). والعيّنة الأخيرة تُثبَّت خاماً في
    `finish()` إذ لا جارة لها — قفزة فيها بالذات (~1.5٪) لا تُلتقط."""

    def __init__(self, bias, seed=None):
        self.bias = bias
        self.prev = seed          # سياق فقط من طور سابق — لا يُكامَل مرتين
        self.pend = None          # (القيمة، dt) تنتظر جارتها التالية
        self.total = 0.0
        self.rejected = 0

    def add(self, value, dt):
        if self.pend is not None:
            b, db = self.pend
            if self.prev is None:
                v = b                                  # الحافّة الأولى بلا جارة سابقة
            else:
                v = sorted((self.prev, b, value))[1]
                if abs(b - v) > SPIKE_JUMP_DPS:
                    self.rejected += 1
            self.total += (v - self.bias) * db
            self.prev = b
        self.pend = (value, dt)

    def provisional(self):
        if self.pend is None:
            return self.total
        return self.total + (self.pend[0] - self.bias) * self.pend[1]

    def finish(self):
        if self.pend is not None:
            self.total += (self.pend[0] - self.bias) * self.pend[1]
            self.prev = self.pend[0]
            self.pend = None
        return self.total


def measure_gz_bias(link, seconds):
    """انحياز gz والروبوت ساكن — يُقاس في كل شوط ولا يُثبَّت في الكود أبداً.
    يعيد (الانحياز، المستعمَل، المستبعَد) — بالوسيط لا بالمتوسط."""
    values = []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        obj, _, _ = link.ask({"T": 126}, 1002, 0.3)
        if obj is None:
            continue
        gz = num(obj.get("gz"))
        if gz is not None:
            values.append(gz)
    if not values:
        return None, 0, 0
    return robust_bias(values)


def median3_series(values):
    """سلسلة بعد وسيط ثلاثي (الطرفان كما هما) — للذروة لا للتكامل."""
    if len(values) < 3:
        return list(values)
    return [values[0]] + [sorted(values[k - 1:k + 2])[1]
                          for k in range(1, len(values) - 1)] + [values[-1]]


def run_turn(link, keeper, degrees, speed, timeout):
    """يلفّ بالمكان ويكامل gz خاماً، ومرشَّحاً بالوسيط الثلاثي ومطروحاً منه
    الانحياز. يعيد قاموس القياس."""
    print("      قياس انحياز gz %0.1fث والروبوت ساكن…" % BIAS_SECONDS)
    bias, bias_n, bias_out = measure_gz_bias(link, BIAS_SECONDS)
    if bias is None:
        return {"error": "لا قراءات gz — اللوحة لا تردّ T:1002"}
    print("      الانحياز (وسيط): %+.4f (من %d عيّنة · استُبعد %d شاذّ خارج %.0fσ)"
          % (bias, bias_n, bias_out, BIAS_OUTLIER_SIGMA))

    # الإشارة الموجبة تقابل L=+ و R=− اصطلاحاً هنا، وهي **غير مفترضة**:
    # الأمر يُحفظ كما أُرسل والزاوية الحقيقية تأتي من المنقلة.
    direction = 1.0 if degrees >= 0 else -1.0
    cmd_l = direction * abs(speed)
    cmd_r = -direction * abs(speed)
    print("      الأمر: L=%+.3f R=%+.3f م/ث (الهدف %+.1f°، حدّ %.1fث)"
          % (cmd_l, cmd_r, degrees, timeout))

    keeper.set_speed(cmd_l, cmd_r)
    t0 = time.monotonic()
    t_prev = t0
    raw_deg = 0.0
    integ = Median3Integrator(bias)
    turn_vals = []
    deadline = t0 + timeout
    hit_target = False
    while time.monotonic() < deadline:
        obj, _, _ = link.ask({"T": 126}, 1002, 0.2)
        now = time.monotonic()
        dt = now - t_prev
        t_prev = now
        if obj is None:
            continue
        gz = num(obj.get("gz"))
        if gz is None:
            continue
        raw_deg += gz * dt
        integ.add(gz, dt)
        turn_vals.append(gz)
        if abs(integ.provisional()) >= abs(degrees):
            hit_target = True
            break
    keeper.set_speed(0, 0)
    elapsed = time.monotonic() - t0
    hard_stop(link, times=1)
    corr_deg = integ.finish()

    # القصور الذاتي بعد قطع الطاقة: يُقاس ولا يُفترض. المرشّح يُكمل بسياق
    # آخر عيّنة من اللفّة فلا تُعامَل أول عيّنة قصور كحافّة بلا جارة.
    coast = Median3Integrator(bias, seed=turn_vals[-1] if turn_vals else None)
    t_prev = time.monotonic()
    t_end = t_prev + 1.0
    while time.monotonic() < t_end:
        obj, _, _ = link.ask({"T": 126}, 1002, 0.2)
        now = time.monotonic()
        dt = now - t_prev
        t_prev = now
        if obj is None:
            continue
        gz = num(obj.get("gz"))
        if gz is not None:
            coast.add(gz, dt)
    coast_deg = coast.finish()
    time.sleep(0.4)
    peak_rate = max((abs(v) for v in median3_series(turn_vals)), default=0.0)
    return {"duration_s": elapsed, "cmd_L": cmd_l, "cmd_R": cmd_r,
            "gz_bias": bias, "bias_n": bias_n, "bias_excluded": bias_out,
            "gz_raw_deg": raw_deg, "gz_corr_deg": corr_deg,
            "coast_deg": coast_deg, "peak_rate_dps": peak_rate,
            "spikes_rejected": integ.rejected + coast.rejected,
            "samples": len(turn_vals), "hit_target": hit_target}


# ═══ الاختبار ════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="UGV01: حركة على الأرض وقياس بالمسطرة")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--boot-wait", type=float, default=BOOT_WAIT_S)
    ap.add_argument("--straight", type=float, default=None, metavar="متر",
                    help="مسافة سير تقديرية بالمتر")
    ap.add_argument("--turn", type=float, default=None, metavar="درجة",
                    help="زاوية لفّ بالمكان بالدرجة")
    ap.add_argument("--speed", type=float, default=0.2, help="م/ث للسير")
    ap.add_argument("--turn-speed", type=float, default=0.15, help="م/ث للفّ")
    ap.add_argument("--repeats", type=int, default=5, help="تكرارات كل أمر")
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="الحدّ الأقصى الصارم لكل شوط (ث)")
    a = ap.parse_args()

    print("═" * 66)
    print("  UGV01 · حركة على الأرض (test_drive) — جمع بيانات خام")
    print("═" * 66)

    if a.straight is None and a.turn is None:
        return verdict(False, ["حدّد --straight <متر> و/أو --turn <درجة>."])
    for name, value in (("--speed", a.speed), ("--turn-speed", a.turn_speed)):
        if abs(value) > MAX_SPEED_MPS or value == 0:
            return verdict(False, ["%s = %.3f خارج النطاق العملي (0, %.2f] م/ث."
                                   % (name, value, MAX_SPEED_MPS)])
    if a.straight is not None:
        need = abs(a.straight) / abs(a.speed)
        if need > a.timeout:
            return verdict(False, [
                "شوط %.2fم عند %.2f م/ث يحتاج %.2fث، و--timeout = %.2fث."
                % (a.straight, a.speed, need, a.timeout),
                "ارفع --timeout صراحةً — الحدّ لا يُقصّ الشوط في صمت،",
                "لأن شوطاً مقصوصاً يُسجَّل كأنه شوط كامل فيفسد البيانات.",
            ])

    port = find_port(a.port)
    if not port:
        return verdict(False, ["لم يُعثر على أي منفذ: %s" % ", ".join(PORT_CANDIDATES)])
    print("المنفذ: %s @ %d" % (port, a.baud))

    if not confirm_motion("🔴 **على الأرض** وحوله مساحة خالية (المسافة + هامش)"):
        return 2

    try:
        link = open_link(port, a.baud)
    except Exception as exc:
        return verdict(False, ["تعذّر فتح المنفذ: %s" % exc])

    fh, keeper = None, None
    try:
        print("\n[1] انتظار إقلاع اللوحة %.1fث…" % a.boot_wait)
        for _ in link.lines(a.boot_wait):
            pass
        link.ser.reset_input_buffer()

        print("\n[2] تهيئة الإقلاع:")
        init_board(link)

        fh, writer, path = new_csv("drive", [
            "mode", "run", "commanded", "cmd_L", "cmd_R", "duration_s",
            "enc_L_m", "enc_R_m", "enc_mean_m",
            "gz_bias", "gz_raw_deg", "gz_corr_deg", "coast_deg", "peak_rate_dps",
            "samples", "measured_truth", "bias_n", "bias_excluded", "spikes_rejected"])

        keeper = Keepalive(link, RESEND_MS / 1000.0)
        keeper.start()

        enc_vals, straight_truth = [], []
        gz_vals, turn_truth, coast_vals = [], [], []
        aborted = False

        # ── أشواط السير ─────────────────────────────────────────────────
        if a.straight is not None:
            print("\n[3] سير مستقيم %.2fم × %d تكرار" % (a.straight, a.repeats))
            for i in range(1, a.repeats + 1):
                print("\n   ▶ شوط %d/%d" % (i, a.repeats))
                print("      ضع الروبوت عند خط البداية وعلّمه، ثم:")
                try:
                    if input('      اضغط Enter للانطلاق (أو "q" للإنهاء): ').strip().lower() == "q":
                        aborted = True
                        break
                except EOFError:
                    aborted = True
                    break
                result = run_straight(link, keeper, a.straight, a.speed, a.timeout)
                if "error" in result:
                    print("      ✗ %s" % result["error"])
                    aborted = True
                    break
                print("      تكامل الإنكودر: L=%.3f م · R=%.3f م · المتوسّط=%.3f م  (%d عيّنة)"
                      % (result["enc_L_m"], result["enc_R_m"],
                         result["enc_mean_m"], result["samples"]))
                truth = ask_float("      📏 المسافة الحقيقية بالمسطرة (م، فارغ=تخطّي): ")
                enc_vals.append(result["enc_mean_m"])
                straight_truth.append(truth)
                writer.writerow(["straight", i, a.straight,
                                 result["cmd_speed"], result["cmd_speed"],
                                 round(result["duration_s"], 3),
                                 round(result["enc_L_m"], 4), round(result["enc_R_m"], 4),
                                 round(result["enc_mean_m"], 4),
                                 "", "", "", "", "", result["samples"],
                                 "" if truth is None else truth, "", "", ""])
                fh.flush()

        # ── أشواط اللفّ ─────────────────────────────────────────────────
        if a.turn is not None and not aborted:
            print("\n[4] لفّ بالمكان %+.1f° × %d تكرار" % (a.turn, a.repeats))
            for i in range(1, a.repeats + 1):
                print("\n   ▶ شوط %d/%d" % (i, a.repeats))
                print("      علّم اتجاه الروبوت الحالي (شريط لاصق/منقلة)، ثم:")
                try:
                    if input('      اضغط Enter للانطلاق (أو "q" للإنهاء): ').strip().lower() == "q":
                        aborted = True
                        break
                except EOFError:
                    aborted = True
                    break
                result = run_turn(link, keeper, a.turn, a.turn_speed, a.timeout)
                if "error" in result:
                    print("      ✗ %s" % result["error"])
                    aborted = True
                    break
                print("      تكامل gz: خام=%+.2f° · بعد الوسيط الثلاثي وطرح الانحياز=%+.2f°"
                      % (result["gz_raw_deg"], result["gz_corr_deg"]))
                print("      قفزات gz مرفوضة بالوسيط (>%.0f°/ث): %d"
                      % (SPIKE_JUMP_DPS, result["spikes_rejected"]))
                print("      قصور بعد قطع الطاقة=%+.2f° · ذروة المعدل=%.1f · %d عيّنة"
                      % (result["coast_deg"], result["peak_rate_dps"], result["samples"]))
                if not result["hit_target"]:
                    print("      ⚠ انتهت المهلة قبل بلوغ الهدف — الشوط غير مكتمل.")
                truth = ask_float("      📐 الزاوية الحقيقية المقاسة (درجة، فارغ=تخطّي): ")
                gz_vals.append(result["gz_corr_deg"])
                coast_vals.append(result["coast_deg"])
                turn_truth.append(truth)
                writer.writerow(["turn", i, a.turn,
                                 result["cmd_L"], result["cmd_R"],
                                 round(result["duration_s"], 3),
                                 "", "", "",
                                 round(result["gz_bias"], 5),
                                 round(result["gz_raw_deg"], 3),
                                 round(result["gz_corr_deg"], 3),
                                 round(result["coast_deg"], 3),
                                 round(result["peak_rate_dps"], 2),
                                 result["samples"],
                                 "" if truth is None else truth,
                                 result["bias_n"], result["bias_excluded"],
                                 result["spikes_rejected"]])
                fh.flush()

        keeper.set_speed(0, 0)

        # ── التشتّت ─────────────────────────────────────────────────────
        print("\n[5] التشتّت (وصفٌ للبيانات — بلا اشتقاق أي معامل):\n")
        if enc_vals:
            print("   ── سير مستقيم · المأمور %.3f م ──" % a.straight)
            print(stats_line("تكامل الإنكودر", enc_vals, "م"))
            print(stats_line("المسطرة", straight_truth, "م"))
        if gz_vals:
            print("   ── لفّ بالمكان · المأمور %+.1f° ──" % a.turn)
            print(stats_line("تكامل gz (بعد الانحياز)", gz_vals, "°"))
            print(stats_line("قصور بعد قطع الطاقة", coast_vals, "°"))
            print(stats_line("المنقلة", turn_truth, "°"))
        print()
        print("   ⚠ المقارنة بين السطرين متروكة لك ولجلسة المعايرة. هذا السكربت")
        print("     لا يشتقّ معاملاً ولا يقترح تعديل أي إعداد — خمسة أشواط على")
        print("     أرضية واحدة ببطارية واحدة لا تصنع ثابتاً فيزيائياً.")

        # ── الحكم ───────────────────────────────────────────────────────
        done = len(enc_vals) + len(gz_vals)
        want = (a.repeats if a.straight is not None else 0) + \
               (a.repeats if a.turn is not None else 0)
        ok = (done == want) and not aborted
        reasons = ["أشواط مكتملة: %d من %d" % (done, want)]
        truths = [t for t in straight_truth + turn_truth if t is not None]
        reasons.append("قياسات يدوية مُدخلة: %d من %d" % (len(truths), done))
        if aborted:
            reasons.append("أُنهيت الجلسة مبكراً — البيانات المجمّعة محفوظة كاملة.")
        if done and not truths:
            reasons.append("⚠ لا قياس يدوي واحد ⇒ الملف يحوي رأي الروبوت عن نفسه")
            reasons.append("   فقط، ولا يصلح مرجعاً لأي معايرة لاحقة.")
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
                keeper.shutdown()        # أوقف التجديد قبل الإيقاف النهائي
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
