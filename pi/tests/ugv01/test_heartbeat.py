#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_heartbeat.py — إثبات فخّ الثلاث ثوانٍ **بالقياس**
================================================================================
🔴 الروبوت **مرفوع عن الأرض** والعجلات حرّة.

اللوحة توقف المحركات إن لم يصلها أمر خلال ~3000 مللي ثانية. هذا مكتوب في
الفيرموير — وهذا السكربت لا يصدّقه ولا يكذّبه، بل **يقيسه**: بعد كم ملّي ثانية
من آخر أمر تصفر السرعة المقاسة فعلاً؟

⚠ **الفخّ داخل الفخّ**: قياس هذا بالاستعلام `{"T":130}` يفسده من أصله — الاستعلام
نفسه أمرٌ يصل إلى اللوحة، وأي أمر يصل قد يجدّد مؤقّت النبضة. فتقيس صمتاً وأنت
تتكلّم. لذلك المنهج الأساسي هنا **البثّ المستمر** `{"T":131,"cmd":1}`: اللوحة
تبثّ من طرفها ونحن نصمت صمتاً تاماً، فيبقى المقياس نظيفاً.

  المرحلة أ: أمر {"T":1,"L":0.1,"R":0.1} متجدّد حتى تتحرّك العجلات فعلاً.
  المرحلة ب: **صمت تام** ومراقبة L/R في البثّ حتى تصفر ⇒ الزمن المقاس.

وبـ--probe-poll تُضاف مرحلة ثالثة تجيب سؤالاً منفصلاً مهمّاً للهجرة:
هل استعلام الحالة T:130 وحده يُبقي المحركات حيّة؟ (لو نعم، فحلقة تليمتري
بريئة تكفي لإلغاء حارس السلامة كلّه بلا أن يشعر أحد.)

دقّة القياس = فترة البثّ نفسها، وتُطبع صراحةً: لا معنى لرقم بلا مقياسه.

الأمان:
  • تأكيد "yes" قبل أي حركة
  • --timeout يحدّ **مرحلة الأمر** (افتراضي 3ث)
  • --watch يحدّ مرحلة الصمت، وعند انتهائها بلا توقّف يُرسل إيقاف قسري
  • finally: {"T":1,"L":0,"R":0} دائماً — حتى مع Ctrl+C أو الخطأ

التشغيل:
    python3 -m pi.tests.ugv01.test_heartbeat
    python3 -m pi.tests.ugv01.test_heartbeat --probe-poll
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
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
MOVING_MPS = 0.02      # فوقها تُعدّ العجلة «تدور» فعلاً
STOPPED_MPS = 0.005    # دونها تُعدّ «واقفة»
STOPPED_STREAK = 2     # عيّنتان متتاليتان دون العتبة ⇒ توقّف مؤكَّد لا ضجيج


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
        ser = serial.Serial(port, baud, timeout=0.02, exclusive=True)
    except TypeError:
        ser = serial.Serial(port, baud, timeout=0.02)
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
    """تحذير + تأكيد يدوي بكلمة yes. لا علم يتجاوزه."""
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


# ═══ مراحل القياس ════════════════════════════════════════════════════════
def drive_until_moving(link, speed, resend_s, timeout_s, writer, sink):
    """يرسل أمر السرعة متجدّداً حتى تدور العجلتان فعلاً أو تنتهي المهلة.

    يعيد (تحرّكت؟، زمن آخر أمر مُرسل، عيّنات البثّ المرصودة).
    """
    t0 = time.monotonic()
    last_cmd = 0.0
    moving = False
    while time.monotonic() - t0 < timeout_s:
        if time.monotonic() - last_cmd >= resend_s:
            link.send({"T": 1, "L": speed, "R": speed})
            last_cmd = time.monotonic()
        for _, obj in link.lines(0.05):
            if obj is None or obj.get("T") != 1001:
                continue
            left, right = num(obj.get("L")), num(obj.get("R"))
            if left is None or right is None:
                continue
            sink.append((time.monotonic(), left, right))
            writer.writerow(["drive", round((time.monotonic() - t0) * 1000, 1),
                             left, right])
            if abs(left) > MOVING_MPS and abs(right) > MOVING_MPS:
                moving = True
    return moving, last_cmd, sink


def watch_until_stopped(link, last_cmd_t, watch_s, writer, phase):
    """يصمت تماماً ويراقب البثّ حتى تصفر L/R. يعيد (زمن التوقّف ms أو None، عيّنات)."""
    samples = []
    streak = 0
    stop_ms = None
    for _, obj in link.lines(watch_s):
        if obj is None or obj.get("T") != 1001:
            continue
        left, right = num(obj.get("L")), num(obj.get("R"))
        if left is None or right is None:
            continue
        now = time.monotonic()
        ms = (now - last_cmd_t) * 1000.0
        samples.append((now, left, right))
        writer.writerow([phase, round(ms, 1), left, right])
        if abs(left) <= STOPPED_MPS and abs(right) <= STOPPED_MPS:
            streak += 1
            if streak >= STOPPED_STREAK and stop_ms is None:
                # الزمن يُنسب إلى **أول** عيّنة صفرية لا إلى تأكيدها
                stop_ms = (samples[-STOPPED_STREAK][0] - last_cmd_t) * 1000.0
                break
        else:
            streak = 0
    return stop_ms, samples


def watch_until_stopped_polled(link, last_cmd_t, watch_s, writer, phase, interval):
    """بديل **ملوَّث** حين لا يعمل البثّ: يستعلم دورياً بدل أن يصمت.

    الاستعلام نفسه قد يجدّد مؤقّت النبضة، فنتيجته لا تُقرأ كنتيجة البثّ:
    توقُّفٌ رغم الاستعلام قياسٌ صالح، وعدمُ توقّف **ملتبس** لا يميّز بين
    «الاستعلام يجدّد المؤقّت» و«الحارس لا يعمل».
    """
    samples = []
    streak = 0
    stop_ms = None
    t_end = time.monotonic() + watch_s
    while time.monotonic() < t_end:
        obj, _, _ = link.ask({"T": 130}, 1001, 0.3)
        if obj is not None:
            left, right = num(obj.get("L")), num(obj.get("R"))
            if left is not None and right is not None:
                now = time.monotonic()
                ms = (now - last_cmd_t) * 1000.0
                samples.append((now, left, right))
                writer.writerow([phase, round(ms, 1), left, right])
                if abs(left) <= STOPPED_MPS and abs(right) <= STOPPED_MPS:
                    streak += 1
                    if streak >= STOPPED_STREAK and stop_ms is None:
                        stop_ms = (samples[-STOPPED_STREAK][0] - last_cmd_t) * 1000.0
                        break
                else:
                    streak = 0
        time.sleep(interval)
    return stop_ms, samples


def stream_period_ms(samples):
    """فترة البثّ المرصودة = دقّة القياس. رقمٌ بلا مقياسه ادّعاء."""
    if len(samples) < 3:
        return None
    gaps = [(samples[i][0] - samples[i - 1][0]) * 1000.0
            for i in range(1, len(samples))]
    gaps.sort()
    return gaps[len(gaps) // 2]


# ═══ الاختبار ════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="UGV01: قياس مهلة نبضة القلب")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--boot-wait", type=float, default=BOOT_WAIT_S)
    ap.add_argument("--speed", type=float, default=0.1, help="م/ث للعجلتين")
    ap.add_argument("--resend", type=float, default=0.3, help="تجديد الأمر (ث)")
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="الحدّ الأقصى لمرحلة الأمر (ث) — حدّ سلامة")
    ap.add_argument("--watch", type=float, default=6.0,
                    help="الحدّ الأقصى لمرحلة الصمت (ث)")
    ap.add_argument("--probe-poll", action="store_true",
                    help="مرحلة إضافية: هل T:130 وحده يُبقي المحركات حيّة؟")
    a = ap.parse_args()

    print("═" * 66)
    print("  UGV01 · قياس مهلة نبضة القلب (test_heartbeat)")
    print("═" * 66)

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

    fh = None
    try:
        print("\n[1] انتظار إقلاع اللوحة %.1fث…" % a.boot_wait)
        for _ in link.lines(a.boot_wait):
            pass
        link.ser.reset_input_buffer()

        print("\n[2] تهيئة الإقلاع:")
        init_board(link)

        print("\n[3] تشغيل البثّ المستمر {\"T\":131,\"cmd\":1}")
        print("    (المنهج كلّه يقوم عليه: اللوحة تتكلّم ونحن نصمت،")
        print("     فلا يجدّد قياسُنا المؤقّتَ الذي نقيسه.)")
        link.send({"T": 131, "cmd": 1})
        time.sleep(0.4)

        # عمود ms: في طور drive زمنٌ منذ بدء الطور، وفي silence/poll_probe
        # زمنٌ منذ **آخر أمر مُرسل** — وهو الرقم المقصود بالقياس.
        fh, writer, path = new_csv(
            "heartbeat", ["phase", "ms", "L", "R"])

        probe = [_ for _ in link.lines(1.0)]
        stream_alive = any(o is not None and o.get("T") == 1001 for _, o in probe)
        if stream_alive:
            print("    ✓ البثّ يعمل — القياس نظيف.")
        else:
            print("    ✗ لا بثّ. سيُقاس بالاستعلام T:130 (منهج ملوَّث — انظر الحكم).")

        # ── المرحلة أ: تحريك ──────────────────────────────────────────
        print("\n[4] المرحلة أ: أمر %.2f م/ث متجدّداً كل %.0fms (حدّ %.1fث)…"
              % (a.speed, a.resend * 1000, a.timeout))
        samples = []
        if stream_alive:
            moving, last_cmd, samples = drive_until_moving(
                link, a.speed, a.resend, a.timeout, writer, samples)
        else:
            # بلا بثّ: نستعلم مع الأمر لنعرف هل تحرّكت أصلاً
            t0 = time.monotonic()
            moving, last_cmd = False, 0.0
            while time.monotonic() - t0 < a.timeout:
                link.send({"T": 1, "L": a.speed, "R": a.speed})
                last_cmd = time.monotonic()
                obj, _, _ = link.ask({"T": 130}, 1001, 0.3)
                if obj:
                    left, right = num(obj.get("L")), num(obj.get("R"))
                    if left is not None and right is not None:
                        samples.append((time.monotonic(), left, right))
                        writer.writerow(["drive", 0, left, right])
                        if abs(left) > MOVING_MPS and abs(right) > MOVING_MPS:
                            moving = True
                time.sleep(max(0.0, a.resend - 0.3))

        if samples:
            peak_l = max(abs(s[1]) for s in samples)
            peak_r = max(abs(s[2]) for s in samples)
            print("    ذروة السرعة المقاسة: L=%.3f  R=%.3f م/ث" % (peak_l, peak_r))
        else:
            peak_l = peak_r = 0.0
            print("    ⚠ لا عيّنات سرعة وصلت أثناء الأمر.")

        if not moving:
            hard_stop(link)
            return verdict(False, [
                "المحركات لم تدر أصلاً (ذروة L=%.3f R=%.3f م/ث)." % (peak_l, peak_r),
                "لا يمكن قياس مهلة توقّف لشيء لم يبدأ.",
                "شغّل test_link ثم test_encoder، وتحقّق من مفتاح طاقة المحركات",
                "ومن شحن البطارية.",
                "السجل: " + path])

        period = stream_period_ms(samples)
        # ── المرحلة ب: صمت تام ────────────────────────────────────────
        if stream_alive:
            print("\n[5] المرحلة ب: **صمت تام** — لا بايت واحد نحو اللوحة.")
            print("    مراقبة L/R في البثّ حتى تصفر (حدّ %.1fث)…" % a.watch)
            stop_ms, watch_samples = watch_until_stopped(
                link, last_cmd, a.watch, writer, "silence")
            if period is None:
                period = stream_period_ms(watch_samples)
        else:
            print("\n[5] المرحلة ب: لا بثّ ⇒ مراقبة بالاستعلام (منهج ملوَّث).")
            print("    مراقبة L/R كل %.1fث حتى تصفر (حدّ %.1fث)…" % (0.3, a.watch))
            stop_ms, watch_samples = watch_until_stopped_polled(
                link, last_cmd, a.watch, writer, "silence_polled", 0.3)
            period = None
        fh.flush()

        # ── المرحلة ج (اختيارية): هل T:130 يجدّد النبضة؟ ──────────────
        poll_kept_alive = None
        if a.probe_poll:
            print("\n[6] المرحلة ج: هل استعلام T:130 وحده يُبقي المحركات حيّة؟")
            # 🔴 يُطفأ البثّ هنا: السؤال هو «هل الاستعلام وحده يكفي»، وبثٌّ
            # يعمل بالتوازي يملأ المخزن بإطارات تُقرأ بعد حين فتبدو حيّة.
            link.send({"T": 131, "cmd": 0})
            time.sleep(0.3)
            hard_stop(link)
            time.sleep(0.8)
            link.drain()
            t0 = time.monotonic()
            last_cmd = 0.0
            while time.monotonic() - t0 < min(a.timeout, 2.0):
                link.send({"T": 1, "L": a.speed, "R": a.speed})
                last_cmd = time.monotonic()
                for _ in link.lines(0.2):
                    pass
            print("    الآن: لا أوامر حركة — استعلام T:130 كل 500ms لمدة 5ث…")
            print("    (البثّ مطفأ، وكل استعلام يبدأ من مخزن نظيف.)")
            alive_after = []
            t0 = time.monotonic()
            while time.monotonic() - t0 < 5.0:
                obj, _, _ = link.ask({"T": 130}, 1001, 0.3)
                if obj:
                    left, right = num(obj.get("L")), num(obj.get("R"))
                    if left is not None and right is not None:
                        elapsed_ms = (time.monotonic() - last_cmd) * 1000.0
                        writer.writerow(["poll_probe", round(elapsed_ms, 1), left, right])
                        alive_after.append((elapsed_ms, abs(left), abs(right)))
                        print("      %6.0f ms   L=%+.3f R=%+.3f" % (elapsed_ms, left, right))
                time.sleep(0.5)
            late = [s for s in alive_after if s[0] > 4000.0]
            poll_kept_alive = bool(late) and any(
                s[1] > MOVING_MPS or s[2] > MOVING_MPS for s in late)
            hard_stop(link)
            fh.flush()

        # ── الحكم ───────────────────────────────────────────────────────
        ok = stop_ms is not None
        reasons = []
        if stream_alive and period:
            reasons.append("فترة البثّ المرصودة (= دقّة القياس): %.0f ms" % period)
        if stop_ms is None and not watch_samples:
            reasons.append("🔴 **لم تصل أي عيّنة** أثناء مرحلة الصمت.")
            reasons.append("   هذا ليس «المحركات لم تتوقف» — نحن لم نرَ شيئاً أصلاً،")
            reasons.append("   فلا قياس هنا لا سلباً ولا إيجاباً. شغّل test_link.")
        elif stop_ms is None and stream_alive:
            reasons.append("🔴 المحركات **لم تتوقف** خلال %.1fث من صمت تام." % a.watch)
            reasons.append("   القياس نظيف (بثّ، بلا أي بايت خارج) ⇒ حارس النبضة لا")
            reasons.append("   يعمل كما هو موصوف. لا تعتمد عليه كطبقة سلامة حتى يُفهم")
            reasons.append("   السبب. (أُرسل إيقاف قسري.)")
        elif stop_ms is None:
            reasons.append("⚠ المحركات لم تتوقف خلال %.1fث — والنتيجة **ملتبسة**:"
                           % a.watch)
            reasons.append("   القياس تمّ بالاستعلام، فلا تمييز بين «الاستعلام يجدّد")
            reasons.append("   المؤقّت» و«الحارس لا يعمل». أصلح البثّ (T:131) أولاً،")
            reasons.append("   أو شغّل --probe-poll للإجابة عن الشقّ الأول وحده.")
        else:
            reasons.append("✅ توقّفت المحركات بعد **%.0f ms** من آخر أمر." % stop_ms)
            reasons.append("   (المواصفة تقول ~3000 ms — هذا هو المقاس على جهازك،")
            reasons.append("    وهو ما يُبنى عليه لا المواصفة.)")
            reasons.append("   ⇒ أي مُرسِل أوامر لاحق يجب أن يجدّد أسرع من هذا الرقم")
            reasons.append("     بهامش واضح (مثلاً 800ms مقابل 3000ms).")
        if not stream_alive:
            ok = False
            reasons.append("⚠ القياس تمّ بالاستعلام لا بالبثّ: T:131 لم يعطِ شيئاً.")
            reasons.append("   الاستعلام نفسه قد يجدّد المؤقّت، فالرقم أعلاه غير موثوق.")
        if poll_kept_alive is True:
            reasons.append("🔴 استعلام T:130 وحده **أبقى المحركات تدور** بعد 4ث.")
            reasons.append("   ⇒ الاستعلام يجدّد نبضة القلب: حلقة تليمتري بريئة كفيلة")
            reasons.append("     بإلغاء حارس السلامة كلّه بلا أن يشعر أحد. احسب حساب")
            reasons.append("     هذا في أي كود هجرة.")
        elif poll_kept_alive is False:
            reasons.append("✅ استعلام T:130 وحده **لم يُبقِ** المحركات تدور ⇒ الحارس")
            reasons.append("   يميّز أوامر الحركة عن الاستعلام.")
        reasons.append("السجل: " + path)
        return verdict(ok, reasons)

    except KeyboardInterrupt:
        print("\nأُوقف بـCtrl+C — يُرسل الإيقاف الآن.")
        return 2
    finally:
        # الإيقاف أولاً ثم إطفاء البثّ — الترتيب مقصود
        try:
            hard_stop(link)
            link.send({"T": 131, "cmd": 0})
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
