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
  • مقدار الجاذبية من ax,ay,az — **بعد تحويل الوحدة إلى م/ث²** — بين 9.0 و10.6

⚠ **وحدة التسارع تُكتشف من المقدار، لا تُفترض**: فيرموير UGV01 يرسلها بوحدة
×100 (مقاس 2026-09-23: ‏|a| = 980.03 ساكناً = 9.80 م/ث²). المقارنة الخام بـ
9.0–10.6 كانت ترسب حسّاساً سليماً. المرشّحات: ≈980 (÷100) · ≈9.8 (م/ث²) ·
≈1.0 (g). مقدار خارجها كلها ⇒ رفض بالقيمة الخام، لا تحويل مخمَّن.
  • يطبع y مع تحذير صريح

⚠ الروبوت **ساكن تماماً** أثناء الاختبار: أي تحريك باليد يضخّم σ ويُنجح
اختباراً كان يجب أن يرسب.

الحكم:
  NO-GO  لا ردود · σ(gz)=0.0 بالضبط · وحدة تسارع غير معروفة ·
         الجاذبية بعد التحويل خارج 9.0–10.6

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

# حدود الحكم — كلها هنا لا مبعثرة في الكود
GRAVITY_MIN, GRAVITY_MAX = 9.0, 10.6       # م/ث² — بعد تحويل الوحدة
G_STD = 9.80665
# (المدى الخام، القاسم إلى م/ث²، الوصف). المدى واسع عمداً (±50٪ حول القيمة
# المتوقَّعة) ليلتقط الوحدة لا ليحكم: الحكم بعد التحويل على 9.0–10.6.
ACCEL_UNITS = (
    ((490.0, 1470.0), 100.0, "×100 (سم/ث²) — 980 ⇒ 9.80 م/ث²"),
    ((4.9, 14.7), 1.0, "م/ث²"),
    ((0.5, 1.5), 1.0 / G_STD, "g"),
)
ZERO_RUN_WARN = 5          # سلسلة أصفار بهذا الطول ⇒ تحذير تعليق
RATE_WARN_HZ = 10.0        # دون هذا المعدل: تحذير لا رفض
BIAS_OUTLIER_SIGMA = 3.0   # الانحياز بالوسيط بعد استبعاد ما خرج عن 3σ
# قفزات gz منفردة مقاسة ساكناً (2026-09-23): ±15.9°/ث في 3 من 200 عيّنة —
# 256 عدّة بالضبط. عتبة العدّ نصفها: فوق ضجيج σ≈0.23 بمراحل.
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
        """يرسل أمراً وينتظر ردّاً بـT المطلوب. يعيد (كائن، زمن الردّ، أسطر شاردة)."""
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
def robust_bias(values, k=BIAS_OUTLIER_SIGMA):
    """(الانحياز بالوسيط بعد استبعاد ما خرج عن k·σ حول الوسيط، المستبعَد).
    لا بالمتوسط: قفزة ±16 واحدة تزيح المتوسط وتبقى في كل تكامل لاحق."""
    med = statistics.median(values)
    if len(values) < 3:
        return med, 0
    sd = statistics.stdev(values)
    kept = [v for v in values if abs(v - med) <= k * sd] or values
    return statistics.median(kept), len(values) - len(kept)


def median3_series(values):
    """(السلسلة بعد وسيط ثلاثي، عدد القفزات المرفوضة فوق SPIKE_JUMP_DPS)."""
    if len(values) < 3:
        return list(values), 0
    out = [values[0]]
    rejected = 0
    for k in range(1, len(values) - 1):
        m = sorted(values[k - 1:k + 2])[1]
        if abs(values[k] - m) > SPIKE_JUMP_DPS:
            rejected += 1
        out.append(m)
    out.append(values[-1])
    return out, rejected


def detect_accel_unit(raw_mag):
    """يعيد (القاسم إلى م/ث²، الوصف) من المقدار الخام الساكن، أو None."""
    for (lo, hi), divisor, label in ACCEL_UNITS:
        if lo <= raw_mag <= hi:
            return divisor, label
    return None


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
        bias, bias_out = robust_bias(gz)
        gz_f, spikes = median3_series(gz)
        print("    انحياز gz (وسيط)     : %+.4f   (استُبعد %d شاذّ خارج %.0fσ)"
              % (bias, bias_out, BIAS_OUTLIER_SIGMA))
        print("    قفزات gz مرفوضة       : %d   (وسيط ثلاثي، عتبة %.0f°/ث)"
              % (spikes, SPIKE_JUMP_DPS))
        print("    σ(gz) بعد الوسيط      : %.4f   (الخام أعلاه %.4f)"
              % (statistics.stdev(gz_f), sigma_gz))

        grav_raw = statistics.mean(gmag) if gmag else None
        unit = detect_accel_unit(grav_raw) if grav_raw is not None else None
        grav = grav_raw / unit[0] if unit else None
        if grav_raw is None:
            print("    مقدار الجاذبية       : — (لا حقول ax/ay/az في الردّ)")
        else:
            print("    مقدار الجاذبية الخام : %.3f" % grav_raw)
            if unit:
                print("    الوحدة المكتشفة      : %s" % unit[1])
                print("    مقدار الجاذبية م/ث²  : %.3f   (المقبول %.1f–%.1f)"
                      % (grav, GRAVITY_MIN, GRAVITY_MAX))
            else:
                print("    الوحدة المكتشفة      : ❌ لا مرشّح يطابق %.3f" % grav_raw)

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
        reasons.append("انحياز gz (وسيط) %+.4f · قفزات مرفوضة %d · σ بعد الوسيط %.4f"
                       % (bias, spikes, statistics.stdev(gz_f)))

        if zero_run >= ZERO_RUN_WARN:
            reasons.append("⚠ أطول سلسلة أصفار مضبوطة = %d ⇒ تعليق متقطّع محتمل"
                           " (σ وحدها تخفيه)." % zero_run)

        if grav_raw is None:
            ok = False
            reasons.append("🔴 لا حقول ax/ay/az في T:1002 ⇒ لا يمكن التحقق من الجاذبية.")
        elif unit is None:
            ok = False
            reasons.append("🔴 مقدار التسارع الخام %.3f لا يطابق أي وحدة معروفة"
                           " (≈980 · ≈9.8 · ≈1.0)." % grav_raw)
            reasons.append("   لا تحويل مخمَّن: تحقّق من الحقول ومن سكون الروبوت.")
        elif not (GRAVITY_MIN <= grav <= GRAVITY_MAX):
            ok = False
            reasons.append("🔴 الجاذبية %.3f م/ث² (الخام %.3f، الوحدة %s) خارج %.1f–%.1f."
                           % (grav, grav_raw, unit[1], GRAVITY_MIN, GRAVITY_MAX))
            reasons.append("   الروبوت مائل أثناء القياس؟ أعِده مستوياً وكرّر.")
        else:
            reasons.append("الجاذبية %.3f م/ث² (الخام %.3f · الوحدة %s) ✅"
                           % (grav, grav_raw, unit[1]))

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
