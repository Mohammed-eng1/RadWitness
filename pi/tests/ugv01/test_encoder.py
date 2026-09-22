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

الأمان: تأكيد "yes" · --timeout لكل طور (افتراضي 3ث) · finally يرسل الإيقاف دائماً.

التشغيل:
    python3 -m pi.tests.ugv01.test_encoder
    python3 -m pi.tests.ugv01.test_encoder --speed 0.15 --timeout 3
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
PORT_CANDIDATES = ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial0")
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


def find_port(explicit=None):
    if explicit:
        return explicit
    for pattern in PORT_CANDIDATES:
        hits = sorted(glob.glob(pattern))
        if hits:
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
def run_phase(link, keeper, label, speed, seconds, settle_s, writer):
    """يأمر بسرعة ويجمع L/R المقاستين. يعيد قائمة (t, L, R) بعد زمن الاستقرار."""
    print("\n   ▶ %s: أمر L=%+.3f R=%+.3f م/ث لمدة %.1fث"
          % (label, speed, speed, seconds))
    keeper.set_speed(speed, speed)
    t0 = time.monotonic()
    steady, allsamp = [], []
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
        print("      %4.2fث   L=%+7.3f   R=%+7.3f" % (t, left, right))
    keeper.set_speed(0, 0)
    return steady, allsamp


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
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="الحدّ الأقصى لكل طور حركة (ث) — حدّ سلامة")
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
    duration = min(a.duration, a.timeout)
    if duration < a.duration:
        print("ℹ المدة قُصّت إلى %.1fث بحدّ --timeout." % duration)
    if a.settle >= duration:
        return verdict(False, ["--settle %.1f ≥ مدة الطور %.1f ⇒ لا عيّنات للمتوسّط."
                               % (a.settle, duration)])

    port = find_port(a.port)
    if not port:
        return verdict(False, ["لم يُعثر على أي منفذ: %s" % ", ".join(PORT_CANDIDATES)])
    print("المنفذ: %s @ %d" % (port, a.baud))

    if not confirm_motion("🔴 **مرفوع عن الأرض** والعجلات حرّة تدور في الهواء"):
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
            "encoder", ["phase", "t_s", "cmd_L", "cmd_R", "meas_L", "meas_R"])

        keeper = Keepalive(link, RESEND_MS / 1000.0)
        keeper.start()
        print("\n[3] خيط تجديد الأمر يعمل كل %dms." % RESEND_MS)

        fwd_steady, _ = run_phase(link, keeper, "forward", +a.speed,
                                  duration, a.settle, writer)
        mean_fl, mean_fr = summarize("أمام", +a.speed, fwd_steady)

        print("\n   ⏸ توقّف ثانية واحدة…")
        time.sleep(1.0)

        rev_steady, _ = run_phase(link, keeper, "reverse", -a.speed,
                                  duration, a.settle, writer)
        mean_rl, mean_rr = summarize("خلف", -a.speed, rev_steady)

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
