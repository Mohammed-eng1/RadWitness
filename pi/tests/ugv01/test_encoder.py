#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_encoder.py — هل الإنكودر يردّ، وبإشارة صحيحة؟
================================================================================
🔴 الروبوت **مرفوع عن الأرض** والعجلات حرّة.

الحقلان L و R في `{"T":1001}` هما السرعة **المقاسة من الإنكودر**، لا المأمورة.
والفرق جوهري: لوحة بإنكودر ميت تردّ صفراً هادئاً، أو تردّ الرقم المأمور نفسه،
وكلاهما يبدو «سليماً» في سطر سجل. ولا يوجد في هذا الفيرموير عدّاد مسافة
تراكمي — من أراد مسافة فليكامل السرعة بنفسه.

ما يفحصه:
  1. أمام +0.1 م/ث ثانيتين → L و R ليستا صفراً، وإشارتهما **موجبة**
  2. خلف  −0.1 م/ث ثانيتين → L و R ليستا صفراً، وإشارتهما **سالبة**
  3. يطبع المتوسّط المقاس مقابل المأمور لكل عجلة
  4. **التماثل**: |أمام| ≟ |خلف| لكل عجلة، و|L| ≟ |R| في كل اتجاه

والفحص الرابع هو ما لا يصنعه الأول والثاني: هما يسألان كل قراءة على حدة
«أليست صفراً؟ أإشارتها صحيحة؟»، فإنكودر يعطي ‎+0.100 أماماً و‎−0.070 خلفاً
**يمرّ بهما معاً** — وفرقُ 30% يفسد كل حساب مسافة لاحق بصمت. ثلاثة أسباب
تنتج هذا: عدّ الإنكودر يختلف باتجاه الدوران (قناة تربيع معطوبة)، أو
احتكاك ميكانيكي في اتجاه واحد، أو لا تماثل في PID الفيرموير — والأول
وحده كافٍ لإفساد كل ملاحة لاحقة.

⚠ **لا يُشتقّ هنا أي معامل تصحيح.** الأرقام تُطبع وتُحفظ فقط. المعايرة جلسة
أخرى: معامل يُشتقّ من عجلات تدور في الهواء بلا حمل ولا احتكاك أرض رقمٌ لا
يصلح للأرض أصلاً.

أمر السرعة يُجدَّد من **خيط مستقل كل 800ms** لأن اللوحة توقف المحركات عند
انقطاع الأوامر ~3000ms (قِسْ هذا الرقم بـtest_heartbeat).

── وضع الأرض: --floor ────────────────────────────────────────────────
🔴 الروبوت **على الأرض**: أمام --seconds (افتراضي 8ث) ثم خلف بالمدة نفسها
فيعود قرب نقطة البداية (مساحة ≈ السرعة × المدة + هامش، أماماً وخلفاً).
في الهواء لا حمل ولا احتكاك، فالفرق بين الجانبين هناك لا يقول شيئاً عن
الأرض. هنا يُطبع **الفرق بين الجانبين بعد استقرار أول ثانيتين فقط**
(اندفاع البدء والتسارع خارج الحساب): متوسّط L وR، والفرق L−R بالم/ث وبالنسبة،
والفرق في المسافة المتكاملة خلال نافذة الاستقرار.
--timeout في هذا الوضع افتراضياً = --seconds، وقيمة أصغر منه **تُرفض** لا تقصّ.

الأمان: تأكيد "yes" · --timeout لكل طور (افتراضي 3ث، أو --seconds مع --floor) ·
finally يرسل الإيقاف دائماً.

التشغيل:
    python3 -m pi.tests.ugv01.test_encoder
    python3 -m pi.tests.ugv01.test_encoder --speed 0.15 --timeout 3
    python3 -m pi.tests.ugv01.test_encoder --floor               # على الأرض 8ث
    python3 -m pi.tests.ugv01.test_encoder --floor --seconds 10
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
MAX_SPEED_MPS = 0.5        # أقصى سرعة عملية عبر T:1 على UGV01
MOVING_MPS = 0.02          # دونها تُعدّ العجلة «لا تدور»
RESEND_MS = 800            # تجديد الأمر — دون مهلة النبضة بهامش واضح
# ⚠ **غير معاير**: تخمين يوم أول لا رقم مقاس على هذه المنصّة. مهمّته أن
# يلفت النظر إلى لا تماثل صارخ، لا أن يحكم بدقّة. عدّله بـ--symmetry-tol
# بعد أن تصير لديك عدّة قراءات من هذا العتاد.
SYMMETRY_TOL_PCT = 15.0    # فرق أكبر منه بين الذهاب والعودة (أو بين L وR) ⇒ رفض
FLOOR_SETTLE_S = 2.0       # وضع الأرض: أول ثانيتين خارج حساب الفرق بين الجانبين
LIFTED_TIMEOUT_S = 3.0     # الحدّ الافتراضي لكل طور والروبوت مرفوع


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
        self._lock = threading.Lock()   # خيط التجديد يكتب بالتوازي مع الرئيسي

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


def pct_diff(first, second):
    """الفرق النسبي بين **مقدارين** منسوباً إلى متوسّطهما (٪، موجب إن كان
    الأول أكبر). القسمة على المتوسّط لا على أحد الطرفين: نسبة لا تنقلب
    قيمتها بتبديل الطرفين. ويعيد None إن كان المقداران أصغر من أن تُنسب
    بينهما نسبة (القسمة على ما يقارب الصفر تصنع أرقاماً هائلة بلا معنى).
    """
    denom = (abs(first) + abs(second)) / 2.0
    if denom < MOVING_MPS:
        return None
    return 100.0 * (abs(first) - abs(second)) / denom


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
        self.link.send(self._cmd)       # فوري، لا ينتظر دورة الخيط

    def run(self):
        while not self._stop_evt.wait(self.period):
            with self._cmd_lock:
                cmd = dict(self._cmd)
            try:
                self.link.send(cmd)
            except Exception:
                pass                    # الخيط لا يُسقط البرنامج بخطأ منفذ

    def shutdown(self):
        self._stop_evt.set()


# ═══ طور واحد ════════════════════════════════════════════════════════════
def run_phase(link, keeper, label, speed, seconds, settle_s, writer, print_every=0.0):
    """يأمر بسرعة ويجمع L/R المقاستين. يعيد قائمة (t, L, R) بعد زمن الاستقرار."""
    print("\n   ▶ %s: أمر L=%+.3f R=%+.3f م/ث لمدة %.1fث"
          % (label, speed, speed, seconds))
    keeper.set_speed(speed, speed)
    t0 = time.monotonic()
    steady, allsamp = [], []
    last_print = -1e9
    while time.monotonic() - t0 < seconds:
        obj, _, _ = link.ask({"T": 130}, 1001, 0.3)
        if obj is None:
            continue
        left, right = num(obj.get("L")), num(obj.get("R"))
        if left is None or right is None:
            continue
        t = time.monotonic() - t0
        allsamp.append((t, left, right))
        writer.writerow([label, round(t, 3), speed, speed, left, right])
        if t >= settle_s:
            steady.append((t, left, right))
        if t - last_print >= print_every:       # الطور الطويل على الأرض لا يُغرق الشاشة
            last_print = t
            print("      %4.2fث   L=%+7.3f   R=%+7.3f" % (t, left, right))
    keeper.set_speed(0, 0)
    return steady, allsamp


def side_difference(label, steady):
    """الفرق بين الجانبين في نافذة الاستقرار — أرقام فقط. يعيد (L−R، ٪) أو None."""
    if len(steady) < 2:
        print("      ⚠ %s: عيّنات استقرار غير كافية للفرق بين الجانبين." % label)
        return None
    mean_l = statistics.mean(s[1] for s in steady)
    mean_r = statistics.mean(s[2] for s in steady)
    dist_l = dist_r = 0.0
    for (t0, l0, r0), (t1, l1, r1) in zip(steady, steady[1:]):
        dist_l += 0.5 * (l0 + l1) * (t1 - t0)
        dist_r += 0.5 * (r0 + r1) * (t1 - t0)
    window = steady[-1][0] - steady[0][0]
    pct = pct_diff(abs(mean_l), abs(mean_r))
    print("      ── %s · الفرق بين الجانبين (بعد %.0fث، نافذة %.1fث، %d عيّنة) ──"
          % (label, FLOOR_SETTLE_S, window, len(steady)))
    print("         L %+.4f · R %+.4f م/ث  ⇒  L−R = %+.4f م/ث (%s)"
          % (mean_l, mean_r, mean_l - mean_r,
             ("%+.1f٪ من متوسّطهما" % pct) if pct is not None else "—"))
    print("         المسافة المتكاملة: L %+.3f · R %+.3f م  ⇒  الفرق %+.1f سم"
          % (dist_l, dist_r, (dist_l - dist_r) * 100.0))
    return mean_l - mean_r, pct


def summarize(label, speed, steady):
    """يطبع المقاس مقابل المأمور — أرقاماً فقط، بلا اشتقاق أي معامل."""
    if not steady:
        print("      ⚠ لا عيّنات بعد زمن الاستقرار في طور %s." % label)
        return None, None
    lefts = [s[1] for s in steady]
    rights = [s[2] for s in steady]
    mean_l, mean_r = statistics.mean(lefts), statistics.mean(rights)
    sd_l = statistics.stdev(lefts) if len(lefts) > 1 else 0.0
    sd_r = statistics.stdev(rights) if len(rights) > 1 else 0.0
    print("      ── %s (%d عيّنة مستقرّة) ──" % (label, len(steady)))
    print("         المأمور : %+.3f م/ث للعجلتين" % speed)
    print("         L المقاس: %+.3f ± %.3f" % (mean_l, sd_l))
    print("         R المقاس: %+.3f ± %.3f" % (mean_r, sd_r))
    return mean_l, mean_r


# ═══ الاختبار ════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="UGV01: فحص ردّ الإنكودر وإشارته")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--boot-wait", type=float, default=BOOT_WAIT_S)
    ap.add_argument("--speed", type=float, default=0.1, help="م/ث (|s| ≤ 0.5)")
    ap.add_argument("--timeout", type=float, default=None,
                    help="الحدّ الأقصى لكل طور حركة (ث) — افتراضي 3، أو --seconds مع --floor")
    ap.add_argument("--floor", action="store_true",
                    help="الروبوت على الأرض: طوران بمدة --seconds والفرق بين الجانبين")
    ap.add_argument("--seconds", type=float, default=8.0,
                    help="مدة كل طور في وضع --floor (ث)")
    ap.add_argument("--duration", type=float, default=2.0, help="مدة كل طور (ث)")
    ap.add_argument("--settle", type=float, default=0.5,
                    help="يُستبعد من المتوسّط أول هذه الثواني (اندفاع البدء)")
    ap.add_argument("--symmetry-tol", type=float, default=SYMMETRY_TOL_PCT,
                    metavar="٪", help="أقصى فرق مقبول بين الذهاب والعودة وبين "
                                      "العجلتين (⚠ الافتراضي غير معاير)")
    a = ap.parse_args()

    print("═" * 66)
    print("  UGV01 · فحص الإنكودر (test_encoder)")
    print("═" * 66)

    if abs(a.speed) > MAX_SPEED_MPS:
        return verdict(False, ["--speed %.2f يتجاوز الحدّ العملي %.2f م/ث."
                               % (a.speed, MAX_SPEED_MPS)])
    if a.floor:
        timeout = a.seconds if a.timeout is None else a.timeout
        if timeout < a.seconds:
            return verdict(False, [
                "--seconds %.1f أطول من --timeout %.1f — الطور يُرفض ولا يُقصّ في صمت."
                % (a.seconds, timeout),
                "ارفع --timeout صراحةً أو قصّر --seconds."])
        duration = a.seconds
        settle = FLOOR_SETTLE_S
    else:
        timeout = LIFTED_TIMEOUT_S if a.timeout is None else a.timeout
        duration = min(a.duration, timeout)
        if duration < a.duration:
            print("ℹ المدة قُصّت إلى %.1fث بحدّ --timeout." % duration)
        settle = a.settle
    if settle >= duration:
        return verdict(False, ["زمن الاستقرار %.1f ≥ مدة الطور %.1f ⇒ لا عيّنات للمتوسّط."
                               % (settle, duration)])
    print("الوضع: %s · %.1fث لكل طور · استقرار %.1fث · حدّ %.1fث"
          % ("🔴 على الأرض (--floor)" if a.floor else "مرفوع", duration, settle, timeout))

    port = find_port(a.port)
    if not port:
        return verdict(False, ["لم يُعثر على أي منفذ: %s" % ", ".join(PORT_CANDIDATES)])
    print("المنفذ: %s @ %d" % (port, a.baud))

    if a.floor:
        placement = ("🔴 **على الأرض**، خالية ≥ %.1fم أمامه **وخلفه** (أمام %.0fث ثم خلف)"
                     % (abs(a.speed) * duration + 0.5, duration))
    else:
        placement = "🔴 **مرفوع عن الأرض** والعجلات حرّة تدور في الهواء"
    if not confirm_motion(placement):
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

        fh, writer, path = new_csv(
            "encoder_floor" if a.floor else "encoder", ["phase", "t_s", "cmd_L", "cmd_R", "meas_L", "meas_R"])

        keeper = Keepalive(link, RESEND_MS / 1000.0)
        keeper.start()
        print("\n[3] خيط تجديد الأمر يعمل كل %dms." % RESEND_MS)

        every = 0.5 if a.floor else 0.0
        fwd_steady, _ = run_phase(link, keeper, "forward", +a.speed,
                                  duration, settle, writer, every)
        mean_fl, mean_fr = summarize("أمام", +a.speed, fwd_steady)
        side_fwd = side_difference("أمام", fwd_steady) if a.floor else None

        print("\n   ⏸ توقّف ثانية واحدة…")
        time.sleep(1.0)

        rev_steady, _ = run_phase(link, keeper, "reverse", -a.speed,
                                  duration, settle, writer, every)
        mean_rl, mean_rr = summarize("خلف", -a.speed, rev_steady)
        side_rev = side_difference("خلف", rev_steady) if a.floor else None

        keeper.set_speed(0, 0)
        fh.flush()

        # ── الحكم ───────────────────────────────────────────────────────
        ok = True
        reasons = []
        if None in (mean_fl, mean_fr, mean_rl, mean_rr):
            return verdict(False, [
                "طور واحد على الأقل بلا عيّنات ⇒ اللوحة لا تردّ T:1001 بانتظام.",
                "شغّل test_link أولاً.",
                "السجل: " + path])

        reasons.append("أمام: المأمور %+.3f ← L %+.3f · R %+.3f"
                       % (a.speed, mean_fl, mean_fr))
        reasons.append("خلف : المأمور %+.3f ← L %+.3f · R %+.3f"
                       % (-a.speed, mean_rl, mean_rr))

        dead = []
        for name, value in (("L أمام", mean_fl), ("R أمام", mean_fr),
                            ("L خلف", mean_rl), ("R خلف", mean_rr)):
            if abs(value) <= MOVING_MPS:
                dead.append(name)
        if dead:
            ok = False
            reasons.append("🔴 قراءة صفرية عملياً في: %s" % "، ".join(dead))
            reasons.append("   ⇒ إنكودر غير موصول، أو عجلة لا تدور، أو الفيرموير")
            reasons.append("     يردّ المأمور لا المقاس. افحص كابل الإنكودر أولاً.")

        wrong = []
        if mean_fl <= 0:
            wrong.append("L لا تُبلّغ موجباً عند الأمام")
        if mean_fr <= 0:
            wrong.append("R لا تُبلّغ موجباً عند الأمام")
        if mean_rl >= 0:
            wrong.append("L لا تُبلّغ سالباً عند الخلف")
        if mean_rr >= 0:
            wrong.append("R لا تُبلّغ سالباً عند الخلف")
        if wrong:
            ok = False
            reasons.append("🔴 إشارة معكوسة: %s" % "، ".join(wrong))
            reasons.append("   ⚠ خطأ إشارة في عجلة **واحدة** يُلغي نفسه لاحقاً مع خطأ")
            reasons.append("     إشارة في الجايرو فيبدو كل شيء سليماً — سجّل هذا الآن.")
        elif not dead:
            reasons.append("✅ العجلتان تردّان بقيم غير صفرية وبإشارة صحيحة اتجاهين.")

        # ── التماثل: المقارنة التي لا تصنعها الفحوص أعلاه ────────────────
        # الفحصان السابقان يسألان كل قراءة على حدة: أليست صفراً؟ أإشارتها
        # صحيحة؟ فإنكودر يعطي +0.100 أماماً و−0.070 خلفاً **يمرّ بهما
        # معاً**، وفرقُ 30% يفسد كل حساب مسافة لاحق. والمقارنة هنا بين
        # المقادير: الإشارة حُكم عليها أصلاً، وخلطها بالمقدار يخفي أحدهما.
        pairs = (
            ("ذهاب/عودة · L", abs(mean_fl), abs(mean_rl), "|أمام|", "|خلف|"),
            ("ذهاب/عودة · R", abs(mean_fr), abs(mean_rr), "|أمام|", "|خلف|"),
            ("توازن أمام", abs(mean_fl), abs(mean_fr), "|L|", "|R|"),
            ("توازن خلف", abs(mean_rl), abs(mean_rr), "|L|", "|R|"),
        )
        print("\n   ── التماثل (٪ منسوبة إلى متوسّط الطرفين) ──")
        print("      %-16s %9s %9s %9s" % ("المقارنة", "الأول", "الثاني", "الفرق"))
        asym_bad = []
        for label, first, second, n1, n2 in pairs:
            diff = pct_diff(first, second)
            if diff is None:
                print("      %-16s %9.3f %9.3f %9s  (أصغر من أن تُنسب)"
                      % (label, first, second, "—"))
                continue
            over = abs(diff) > a.symmetry_tol
            print("      %-16s %9.3f %9.3f %+8.1f٪  %s"
                  % (label, first, second, diff, "❌" if over else "✅"))
            if over:
                asym_bad.append((label, diff))

        if asym_bad:
            ok = False
            reasons.append("🔴 لا تماثل يتجاوز %.0f٪: %s"
                           % (a.symmetry_tol,
                              "، ".join("%s %+.1f٪" % (n, d) for n, d in asym_bad)))
            if any(n.startswith("ذهاب/عودة") for n, _ in asym_bad):
                reasons.append("   ذهاب ≠ عودة ⇒ أحد ثلاثة: عدّ الإنكودر يختلف")
                reasons.append("   باتجاه الدوران (قناة تربيع معطوبة) · احتكاك")
                reasons.append("   ميكانيكي في اتجاه واحد · لا تماثل في PID الفيرموير.")
                reasons.append("   والأول وحده يفسد كل مسافة محسوبة لاحقاً بصمت.")
            if any(n.startswith("توازن") for n, _ in asym_bad):
                reasons.append("   L ≠ R ⇒ محرك أو درايفر أضعف، أو إنكودر بمقياس")
                reasons.append("   مختلف. والأثر مباشر: الروبوت ينحرف عن الخط")
                reasons.append("   المستقيم، والحلقة المغلقة لا تُخفيه لأنها تلاحق")
                reasons.append("   سرعةً مقاسة كاذبة أصلاً.")
            reasons.append("   ⚠ العتبة %.0f٪ **غير معايرة** — تخمين يوم أول."
                           % a.symmetry_tol)
            reasons.append("     أعد بـ--symmetry-tol بعد عدّة قراءات من هذا العتاد.")
        elif not dead:
            reasons.append("✅ التماثل ضمن %.0f٪: ذهابٌ كعودة، وL كـR."
                           % a.symmetry_tol)

        if a.floor:
            for lab, sd in (("أمام", side_fwd), ("خلف", side_rev)):
                if sd is not None:
                    reasons.append("على الأرض · %s: L−R = %+.4f م/ث (%s) بعد أول %.0fث"
                                   % (lab, sd[0], ("%+.1f٪" % sd[1]) if sd[1] is not None
                                      else "—", FLOOR_SETTLE_S))
            reasons.append("⚠ **تشخيص لا معايرة**: لا معامل يُشتقّ ولا إعداد يُكتب —")
            reasons.append("   أرضية واحدة وبطارية واحدة. والإنكودر يقيس دوران العجلة لا")
            reasons.append("   حركة الروبوت: انزلاق جانب لا يظهر هنا (قارن بالمسطرة).")
        else:
            reasons.append("⚠ كل ما سبق **تشخيص لا معايرة**: لا معامل يُشتقّ ولا إعداد")
            reasons.append("   يُكتب. وعجلات تدور في الهواء بلا حمل ولا احتكاك أرض لا")
            reasons.append("   تصلح مرجع معايرة أصلاً.")
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
