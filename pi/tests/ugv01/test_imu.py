#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_imu.py — هل جايرو UGV01 حيّ؟  (لا يحرّك شيئاً)
================================================================================
الاتجاه في هذا المشروع من **تكامل gz حصراً**، فالجايرو هو الحسّاس الذي تقف
عليه الملاحة كلها. وهذا السكربت يسأل سؤالاً واحداً: هل يعطي بيانات حيّة، أم
يعطي رقماً ثابتاً يشبه البيانات؟

🔴 **σ = 0.0 بالضبط معناه ميت أو معلّق — لا «هادئ»**
درس مقاس على المنصّة السابقة: حسّاس عاد إلى وضع CONFIG بعد إعادة تشغيل ذاتية
صار يردّ بهويّته على الناقل ويقرأ صفراً **مضبوطاً** إلى الأبد. البصمة مميّزة:
الضجيج الحراري الحقيقي لا يكون صفراً أبداً — أي جايرو سليم يتذبذب في خانته
الأخيرة ولو كان ساكناً على طاولة رخام. فانعدام الضجيج تماماً ليس جودةً عالية،
بل دليل أن الرقم لا يأتي من فيزياء أصلاً.

ما يقيسه خلال 10 ثوانٍ من استعلام {"T":126} ← {"T":1002}:
  • معدل القراءة (هرتز)
  • σ و المتوسّط لـ gx/gy/gz  ← الحكم على σ(gz)
  • أطول سلسلة أصفار مضبوطة متتابعة في gz (تعليق مؤقّت لا يظهر في σ)
  • مقدار الجاذبية من ax,ay,az — يجب أن يقع بين 9.0 و10.6
  • يطبع y مع تحذير صريح

⚠ الروبوت **ساكن تماماً** أثناء الاختبار: أي تحريك باليد يضخّم σ ويُنجح
اختباراً كان يجب أن يرسب.

الحكم:
  NO-GO  لا ردود · σ(gz)=0.0 بالضبط · الجاذبية خارج 9.0–10.6

التشغيل:
    python3 -m pi.tests.ugv01.test_imu
    python3 -m pi.tests.ugv01.test_imu --seconds 20
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

# حدود الحكم — كلها هنا لا مبعثرة في الكود
GRAVITY_MIN, GRAVITY_MAX = 9.0, 10.6
ZERO_RUN_WARN = 5          # سلسلة أصفار بهذا الطول ⇒ تحذير تعليق
RATE_WARN_HZ = 10.0        # دون هذا المعدل: تحذير لا رفض


def find_port(explicit=None):
    """يعيد المنفذ المطلوب أو أول مرشّح موجود، وNone إن لم يوجد شيء."""
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
        """يولّد (نص، كائن) لكل سطر يصل خلال المهلة؛ الكائن None إن لم يكن JSON."""
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

    def ask(self, cmd, want_t, seconds):
        """يرسل أمراً وينتظر ردّاً بـT المطلوب. يعيد (كائن، زمن الردّ، أسطر شاردة)."""
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
    """تهيئة الإقلاع الثلاثية — تُرسل في كل سكربت لأن اللوحة تنساها مع كل إقلاع."""
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
    """يحوّل إلى float أو يعيد None — حقل ناقص ليس صفراً."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def longest_zero_run(values):
    """أطول سلسلة أصفار **مضبوطة** متتابعة (0.0 حرفياً لا ما قاربه)."""
    best = run = 0
    for v in values:
        run = run + 1 if v == 0.0 else 0
        best = max(best, run)
    return best


# ═══ الاختبار ════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="UGV01: فحص حياة الجايرو والتسارع")
    ap.add_argument("--port", default=None, help="افتراضياً: بحث تلقائي")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--boot-wait", type=float, default=BOOT_WAIT_S)
    ap.add_argument("--seconds", type=float, default=10.0, help="مدة الجمع")
    a = ap.parse_args()

    print("═" * 66)
    print("  UGV01 · فحص وحدة القصور الذاتي (test_imu) — لا يحرّك المحركات")
    print("═" * 66)
    print("⚠ أبقِ الروبوت **ساكناً تماماً** طوال %.0f ثانية." % a.seconds)
    print("   تحريكه باليد يضخّم σ فيُنجح اختباراً كان يجب أن يرسب.")

    port = find_port(a.port)
    if not port:
        return verdict(False, ["لم يُعثر على أي منفذ: %s" % ", ".join(PORT_CANDIDATES)])
    print("\nالمنفذ: %s @ %d" % (port, a.baud))

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

        print("\n[3] جمع {\"T\":126} لمدة %.0fث…" % a.seconds)
        fh, writer, path = new_csv(
            "imu", ["t_s", "gx", "gy", "gz", "ax", "ay", "az", "r", "p", "y", "temp"])

        gx, gy, gz, gmag, yaws = [], [], [], [], []
        misses = 0
        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < a.seconds:
            obj, _, _ = link.ask({"T": 126}, 1002, 0.5)
            if obj is None:
                misses += 1
                continue
            t = time.monotonic() - t0
            vgx, vgy, vgz = num(obj.get("gx")), num(obj.get("gy")), num(obj.get("gz"))
            vax, vay, vaz = num(obj.get("ax")), num(obj.get("ay")), num(obj.get("az"))
            if vgz is not None:
                gz.append(vgz)
            if vgx is not None:
                gx.append(vgx)
            if vgy is not None:
                gy.append(vgy)
            if None not in (vax, vay, vaz):
                gmag.append(math.sqrt(vax * vax + vay * vay + vaz * vaz))
            vy = num(obj.get("y"))
            if vy is not None:
                yaws.append(vy)
            writer.writerow([round(t, 4), vgx, vgy, vgz, vax, vay, vaz,
                             obj.get("r"), obj.get("p"), obj.get("y"), obj.get("temp")])
            if t - last_print >= 1.0:
                last_print = t
                print("    %4.1fث   gz=%-10s  |a|=%s"
                      % (t, vgz, ("%.3f" % gmag[-1]) if gmag else "—"))
        elapsed = time.monotonic() - t0
        fh.flush()

        # ── التحليل ─────────────────────────────────────────────────────
        n = len(gz)
        if n < 2:
            return verdict(False, [
                "قراءات gz صالحة: %d (والمطلوب ≥2)" % n,
                "ردود ضائعة: %d" % misses,
                "سبب الرفض: اللوحة لا تردّ T:1002 — شغّل test_link أولاً.",
                "السجل: " + path])

        rate = n / elapsed if elapsed > 0 else 0.0
        sigma_gz = statistics.stdev(gz)
        mean_gz = statistics.mean(gz)
        zero_run = longest_zero_run(gz)
        zeros = sum(1 for v in gz if v == 0.0)

        print("\n[4] النتائج:\n")
        print("    عيّنات gz            : %d خلال %.2fث" % (n, elapsed))
        print("    معدل القراءة         : %.1f هرتز%s"
              % (rate, "   ⚠ منخفض" if rate < RATE_WARN_HZ else ""))
        print("    gz  متوسّط / σ        : %+.4f / %.4f" % (mean_gz, sigma_gz))
        if gx:
            print("    gx  متوسّط / σ        : %+.4f / %.4f"
                  % (statistics.mean(gx), statistics.stdev(gx) if len(gx) > 1 else 0.0))
        if gy:
            print("    gy  متوسّط / σ        : %+.4f / %.4f"
                  % (statistics.mean(gy), statistics.stdev(gy) if len(gy) > 1 else 0.0))
        print("    أصفار gz المضبوطة    : %d من %d · أطول سلسلة %d"
              % (zeros, n, zero_run))

        grav = statistics.mean(gmag) if gmag else None
        if grav is None:
            print("    مقدار الجاذبية       : — (لا حقول ax/ay/az في الردّ)")
        else:
            print("    مقدار الجاذبية |a|   : %.3f   (المقبول %.1f–%.1f)"
                  % (grav, GRAVITY_MIN, GRAVITY_MAX))

        if yaws:
            print("    y (yaw) آخر قراءة    : %+.2f°" % yaws[-1])
            print("    ⚠ y مدموجة بالمغنيتومتر AK09918 — **لا تُستعمل للملاحة**.")
            print("      الهيكل معدني وحوله أربعة محركات بمغانط، فالبوصلة تقرأ مجال")
            print("      الروبوت نفسه وتدور معه. الاتجاه من تكامل gz حصراً.")

        # ── الحكم ───────────────────────────────────────────────────────
        ok = True
        reasons = ["عيّنات gz: %d · معدل %.1f هرتز · ردود ضائعة %d" % (n, rate, misses)]

        if sigma_gz == 0.0:
            ok = False
            reasons.append("🔴 σ(gz) = 0.0 **بالضبط** ⇒ الحسّاس ميت أو معلّق.")
            reasons.append("   الضجيج الحراري الحقيقي لا يكون صفراً أبداً: رقم بلا")
            reasons.append("   تذبذب هو رقم لا يأتي من فيزياء. افحص تغذية الحسّاس")
            reasons.append("   وناقله، وأعد إقلاع اللوحة، ثم أعد الاختبار.")
        else:
            reasons.append("σ(gz) = %.4f ≠ 0 ⇒ إشارة حيّة." % sigma_gz)

        if zero_run >= ZERO_RUN_WARN:
            reasons.append("⚠ أطول سلسلة أصفار مضبوطة = %d ⇒ تعليق متقطّع محتمل"
                           " (σ وحدها تخفيه)." % zero_run)

        if grav is None:
            ok = False
            reasons.append("🔴 لا حقول ax/ay/az في T:1002 ⇒ لا يمكن التحقق من الجاذبية.")
        elif not (GRAVITY_MIN <= grav <= GRAVITY_MAX):
            ok = False
            reasons.append("🔴 مقدار الجاذبية %.3f خارج %.1f–%.1f."
                           % (grav, GRAVITY_MIN, GRAVITY_MAX))
            # تلميح وحدات: لا يغيّر الحكم، لكنه يمنع تشخيص «حسّاس تالف» وهو سليم
            if 0.90 <= grav <= 1.06:
                reasons.append("   تلميح: القيمة ≈1 ⇒ الوحدة g لا م/ث². الحسّاس على"
                               " الأرجح سليم والمقياس مختلف — تحقّق قبل استبداله.")
            elif 900.0 <= grav <= 1060.0:
                reasons.append("   تلميح: القيمة ≈1000 ⇒ الوحدة mg. الحسّاس على الأرجح"
                               " سليم والمقياس مختلف — تحقّق قبل استبداله.")
            else:
                reasons.append("   الروبوت مائل أثناء القياس؟ أعِده مستوياً وكرّر.")

        if rate < RATE_WARN_HZ:
            reasons.append("⚠ معدل %.1f هرتز منخفض (تحذير لا رفض) — تكامل الاتجاه"
                           " يخشن بمعدل بطيء." % rate)

        reasons.append("السجل: " + path)
        return verdict(ok, reasons)

    except KeyboardInterrupt:
        print("\nأُوقف بـCtrl+C.")
        return 2
    finally:
        if fh:
            fh.close()
        try:
            link.ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
