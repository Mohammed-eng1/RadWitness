#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_battery.py — الجهد تحت الحمل: كم يهبط، ومتى يعود؟
================================================================================
🔴 الروبوت **على الأرض** في مساحة خالية (أو --lifted مع قبول قراءة متفائلة).

جهد ساكن لا يقول شيئاً مفيداً. البطارية تنهار تحت **الحمل**، والدوران بالمكان
أثقل مناورة ممكنة: أربعة محركات تعمل معاً ضدّ احتكاك جانبي كامل. فهو أوّل ما
يسقط مع ضعف البطارية بينما يبقى السير ممكناً — وهذا بالضبط ما يجعل العطل
مخادعاً: المسح يعمل واللفّات تفشل.

⚠ السبب المباشر لوجود هذا السكربت: على المنصّة السابقة، وعند **11.04 فولت**،
انعكست لفّة مهمة كاملة — والبطارية كانت «تعمل». أُريد سلوك البطارية الجديدة
مقيساً مبكراً، لا مكتشفاً وسط مهمة.

ثلاث مراحل:
  أ. ساكن 5ث        → خطّ الأساس
  ب. دوران بالمكان 3ث → أدنى جهد تحت الحمل
  ج. ساكن 5ث        → هل يتعافى، وإلى أين؟

ويُطبع الهبوط الأقصى = متوسّط الأساس − أدنى قراءة تحت الحمل.

🔴 **القراءة الخارجة عن 5–14 فولت تُرفض ولا تُقبل بصمت**: حزمة 3S لا تصل إليها
أصلاً، فالرقم حينها خطأ قياس لا حالة بطارية — وقبوله يصنع إنذاراً كاذباً أخطر
من غياب القراءة.

⚠ العتبات أدناه موروثة من منصّة 3S السابقة وتُمرَّر كخيارات: تحقّق من مواصفة
حزمة UGV01 قبل الاعتماد عليها.

الأمان: تأكيد "yes" · --timeout حدّ مرحلة الحمل · finally يرسل الإيقاف دائماً.

التشغيل:
    python3 -m pi.tests.ugv01.test_battery
    python3 -m pi.tests.ugv01.test_battery --lifted --load-seconds 3
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
MAX_SPEED_MPS = 0.5
RESEND_MS = 800
V_SANE_MIN, V_SANE_MAX = 5.0, 14.0     # خارجها: خطأ قياس لا حالة بطارية


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
    print("⚠ هذا السكربت **يشغّل المحركات** (دوران بالمكان).")
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


def sample_phase(link, label, seconds, writer, rejects):
    """يجمع الجهد خلال مرحلة. القراءة الشاذة تُعدّ ولا تدخل الإحصاء."""
    volts = []
    t0 = time.monotonic()
    last_print = -1.0
    while time.monotonic() - t0 < seconds:
        obj, _, _ = link.ask({"T": 130}, 1001, 0.3)
        if obj is None:
            continue
        v = num(obj.get("v"))
        t = time.monotonic() - t0
        if v is None:
            rejects.append(("missing", None))
            continue
        if not (V_SANE_MIN <= v <= V_SANE_MAX):
            rejects.append((label, v))
            writer.writerow([label, round(t, 3), v, "rejected",
                             obj.get("L"), obj.get("R")])
            continue
        volts.append(v)
        writer.writerow([label, round(t, 3), v, "ok", obj.get("L"), obj.get("R")])
        if t - last_print >= 1.0:
            last_print = t
            print("      %4.1fث   v=%.2f فولت" % (t, v))
    return volts


# ═══ الاختبار ════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="UGV01: هبوط جهد البطارية تحت الحمل")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--boot-wait", type=float, default=BOOT_WAIT_S)
    ap.add_argument("--idle-seconds", type=float, default=5.0)
    ap.add_argument("--load-seconds", type=float, default=3.0)
    ap.add_argument("--speed", type=float, default=0.2, help="م/ث لكل عجلة في الدوران")
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="الحدّ الأقصى الصارم لمرحلة الحمل (ث)")
    ap.add_argument("--lifted", action="store_true",
                    help="الروبوت مرفوع — الحمل أخفّ والهبوط سيبدو متفائلاً")
    ap.add_argument("--warn-below", type=float, default=11.0,
                    help="تحذير إن نزل خطّ الأساس عنه (موروث من منصّة 3S)")
    ap.add_argument("--floor", type=float, default=10.0,
                    help="رفض إن نزل الجهد تحت الحمل عنه (موروث من منصّة 3S)")
    a = ap.parse_args()

    print("═" * 66)
    print("  UGV01 · جهد البطارية تحت الحمل (test_battery)")
    print("═" * 66)

    if abs(a.speed) > MAX_SPEED_MPS or a.speed == 0:
        return verdict(False, ["--speed %.3f خارج النطاق العملي (0, %.2f] م/ث."
                               % (a.speed, MAX_SPEED_MPS)])
    load_s = min(a.load_seconds, a.timeout)
    if load_s < a.load_seconds:
        print("ℹ مرحلة الحمل قُصّت إلى %.1fث بحدّ --timeout." % load_s)

    port = find_port(a.port)
    if not port:
        return verdict(False, ["لم يُعثر على أي منفذ: %s" % ", ".join(PORT_CANDIDATES)])
    print("المنفذ: %s @ %d" % (port, a.baud))

    placement = ("⚠ **مرفوع** — حمل أخفّ من الحقيقي" if a.lifted
                 else "🔴 **على الأرض** في مساحة خالية (الحمل الحقيقي)")
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
            "battery", ["phase", "t_s", "v", "status", "L", "R"])
        rejects = []

        print("\n[3] المرحلة أ — ساكن %.1fث (خطّ الأساس):" % a.idle_seconds)
        base = sample_phase(link, "idle_before", a.idle_seconds, writer, rejects)

        keeper = Keepalive(link, RESEND_MS / 1000.0)
        keeper.start()

        print("\n[4] المرحلة ب — دوران بالمكان %.1fث (أثقل مناورة):" % load_s)
        keeper.set_speed(+a.speed, -a.speed)
        load = sample_phase(link, "load_spin", load_s, writer, rejects)
        keeper.set_speed(0, 0)
        hard_stop(link, times=1)

        print("\n[5] المرحلة ج — ساكن %.1fث (التعافي):" % a.idle_seconds)
        recovery = sample_phase(link, "idle_after", a.idle_seconds, writer, rejects)
        fh.flush()

        # ── النتائج ─────────────────────────────────────────────────────
        print("\n[6] النتائج:\n")
        if not base or not load:
            missing = sum(1 for r in rejects if r[0] == "missing")
            bad = [r for r in rejects if r[0] != "missing"]
            reasons = ["قراءات صالحة: أساس %d · حمل %d · تعافٍ %d"
                       % (len(base), len(load), len(recovery)),
                       "قراءات بلا حقل v: %d · قراءات خارج %.0f–%.0f فولت: %d"
                       % (missing, V_SANE_MIN, V_SANE_MAX, len(bad))]
            if missing and not bad:
                reasons.append("🔴 لا حقل v في ردّ T:1001 ⇒ لا قراءة جهد من هذه")
                reasons.append("   اللوحة. الحماية الجهدية تحتاج مصدراً آخر (INA219).")
            elif bad:
                reasons.append("🔴 كل القراءات خارج النطاق المعقول ⇒ خطأ قياس لا")
                reasons.append("   حالة بطارية. أمثلة: %s"
                               % ", ".join("%.2f" % r[1] for r in bad[:5]))
            if not load:
                reasons.append("🔴 لا قراءات أثناء الحمل ⇒ الهبوط غير مقيس.")
            reasons.append("السجل: " + path)
            return verdict(False, reasons)

        base_mean = statistics.mean(base)
        base_min = min(base)
        load_min = min(load)
        load_mean = statistics.mean(load)
        sag = base_mean - load_min
        rec_mean = statistics.mean(recovery) if recovery else None

        print("    خطّ الأساس (ساكن)   : متوسّط %.2f · أدنى %.2f فولت (%d قراءة)"
              % (base_mean, base_min, len(base)))
        print("    تحت الحمل (دوران)   : متوسّط %.2f · **أدنى %.2f** فولت (%d قراءة)"
              % (load_mean, load_min, len(load)))
        if rec_mean is not None:
            print("    بعد التعافي (ساكن)  : متوسّط %.2f فولت (%d قراءة)"
                  % (rec_mean, len(recovery)))
        print()
        print("    🔻 **الهبوط الأقصى = %.2f فولت**  (الأساس − أدنى حمل)" % sag)
        if rec_mean is not None:
            print("    ↩ الفارق بعد التعافي عن الأساس: %+.2f فولت"
                  % (rec_mean - base_mean))

        # ── الحكم ───────────────────────────────────────────────────────
        ok = True
        reasons = ["الأساس %.2f · أدنى حمل %.2f · الهبوط %.2f فولت"
                   % (base_mean, load_min, sag)]
        if rejects:
            reasons.append("قراءات مرفوضة (خارج %.0f–%.0f فولت أو بلا v): %d — عُدّت"
                           " ولم تدخل الإحصاء." % (V_SANE_MIN, V_SANE_MAX, len(rejects)))
        if load_min < a.floor:
            ok = False
            reasons.append("🔴 الجهد تحت الحمل %.2f نزل تحت الحدّ %.2f فولت."
                           % (load_min, a.floor))
            reasons.append("   لا تُكمل اختبارات الحركة قبل الشحن.")
        elif base_mean < a.warn_below:
            reasons.append("⚠ خطّ الأساس %.2f دون %.2f فولت: البطارية ليست ممتلئة."
                           % (base_mean, a.warn_below))
            reasons.append("   على المنصّة السابقة انعكست لفّة مهمة كاملة عند 11.04")
            reasons.append("   فولت والبطارية «تعمل» — فلا تُعاير حركةً على هذا الشحن.")
        else:
            reasons.append("✅ الجهد بقي فوق الحدود طوال الحمل.")
        if a.lifted:
            reasons.append("⚠ قيس والروبوت **مرفوع**: الحمل الحقيقي على الأرض أثقل")
            reasons.append("   والهبوط الفعلي أكبر من %.2f فولت. أعده على الأرض." % sag)
        reasons.append("⚠ العتبات (%.1f تحذير · %.1f رفض) موروثة من منصّة 3S — تحقّق"
                       % (a.warn_below, a.floor))
        reasons.append("   من مواصفة حزمة UGV01 قبل الاعتماد عليها.")
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
