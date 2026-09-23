#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_sync.py — كم ملّي ثانية بين ردّ الحالة وردّ الـIMU؟  (لا يحرّك شيئاً)
================================================================================
الإنكودر (`L/R` في T:1001) والجايرو (`gz` في T:1002) يصلان في **ردّين
منفصلين**، أي من لحظتين مختلفتين. وكل حساب يقارن بينهما (الانزلاق، دمج
الاتجاه) يفترض ضمناً أنهما من اللحظة نفسها. هذا السكربت يقيس ذلك الافتراض.

في كل دورة:
  يرسل {"T":130} ثم {"T":126} متتاليين بلا فاصل، ويسجّل لحظة **وصول** السطر
  الكامل لكلٍّ من T:1001 وT:1002، والفارق Δ = وصول(1002) − وصول(1001).
  ‏200 دورة افتراضياً ⇒ الوسيط · التشتت (σ و IQR) · الأقصى — بالملّي ثانية.

الغرض: **رقم أساس قبل تعديل الفيرموير (أكتوبر)** ليُقارن بعده بالسكربت نفسه
وبالإعدادات نفسها. لذلك لا يحكم على الرقم بجيد أو سيّئ — يقيسه ويحفظه.

⚠ ما يقيسه بدقّة: زمن **الوصول** إلى الراسبري، لا لحظة أخذ العيّنة داخل
اللوحة. والفارق يحوي حتماً زمن إرسال سطر T:1002 نفسه على السلك (~10 بت لكل
بايت عند 115200) — يُحسب لكل دورة ويُطبع «أرضيةً» لا يمكن النزول تحتها.

⚠ دقّة الطابع الزمني: القراءة تُعيد عند أول بايت متاح (لا تنتظر امتلاء مخزن
ثابت الحجم)، لكن كمون مشغّل UART في النواة يبقى بضعة ملّي ثوانٍ غير مقيسة.

⚠ البثّ المستمر {"T":131} لو تُرك مفعّلاً (من test_heartbeat مقطوع مثلاً)
يدسّ إطارات T:1001 غير مطلوبة فتُقرن بالسؤال الخطأ — لذلك يُطفأ في التهيئة.

الحكم:
  GO     ≥ 95% من الدورات وصل فيها الردّان، وصفر أسطر غير JSON.
  NO-GO  غير ذلك — الرقم المحسوب من عيّنة ناقصة لا يصلح أساساً للمقارنة.

التشغيل:
    python3 -m pi.tests.ugv01.test_sync
    python3 -m pi.tests.ugv01.test_sync --cycles 500 --gap 0.05
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

GO_MIN_FRACTION = 0.95      # نسبة الدورات المكتملة المطلوبة لقبول الرقم أساساً


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


def describe(values):
    """الوسيط · σ · الربيعان · الأدنى · الأقصى — والعيّنة الواحدة بلا تشتت."""
    vals = sorted(values)
    n = len(vals)
    q1, q3 = (statistics.quantiles(vals, n=4)[0::2] if n >= 2 else (vals[0], vals[0]))
    return {
        "n": n,
        "median": statistics.median(vals),
        "stdev": statistics.stdev(vals) if n >= 2 else 0.0,
        "q1": q1, "q3": q3,
        "min": vals[0], "max": vals[-1],
    }


def ms(t):
    return round(t * 1000.0, 3) if t is not None else ""


# ═══ الاختبار ════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="UGV01: الفارق الزمني بين T:1001 وT:1002")
    ap.add_argument("--port", default=None, help="افتراضياً: بحث تلقائي")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--boot-wait", type=float, default=BOOT_WAIT_S)
    ap.add_argument("--cycles", type=int, default=200, help="عدد الدورات")
    ap.add_argument("--gap", type=float, default=0.05,
                    help="استراحة بين الدورات (ث) — ثبّتها عند المقارنة")
    ap.add_argument("--reply-timeout", type=float, default=0.5,
                    help="مهلة انتظار الردّين في الدورة الواحدة (ث)")
    a = ap.parse_args()

    print("═" * 66)
    print("  UGV01 · تزامن T:1001/T:1002 (test_sync) — لا يحرّك المحركات إطلاقاً")
    print("═" * 66)

    port = find_port(a.port)
    if not port:
        return verdict(False, ["لم يُعثر على أي منفذ: %s" % ", ".join(PORT_CANDIDATES)])
    print("المنفذ: %s @ %d" % (port, a.baud))

    try:
        link = open_link(port, a.baud)
    except Exception as exc:
        return verdict(False, ["تعذّر فتح المنفذ: %s" % exc,
                               "تحقّق من الصلاحيات (مجموعة dialout) ومن عدم شغله ببرنامج آخر."])

    fh = None
    try:
        print("\n[1] انتظار إقلاع اللوحة %.1fث…" % a.boot_wait)
        for _ in link.lines(a.boot_wait):
            pass
        link.ser.reset_input_buffer()

        print("\n[2] تهيئة الإقلاع:")
        init_board(link)

        print("\n[3] %d دورة: {\"T\":130} ثم {\"T\":126} → وصول T:1001 ووصول T:1002\n"
              % a.cycles)
        fh, writer, path = new_csv("sync", [
            "cycle", "ok", "send_gap_ms", "lat_1001_ms", "lat_1002_ms", "delta_ms",
            "wire_1002_ms", "bytes_1001", "bytes_1002", "L", "R", "gz"])

        deltas, wires, lat1, lat2 = [], [], [], []
        miss_1001 = miss_1002 = swapped = 0
        junk = []
        t_start = time.monotonic()
        for i in range(1, a.cycles + 1):
            r = link.ask_pair(a.reply_timeout)
            junk.extend(s for s in r["strays"] if not s.startswith("{"))
            got1, got2 = r["t1001"] is not None, r["t1002"] is not None
            miss_1001 += 0 if got1 else 1
            miss_1002 += 0 if got2 else 1
            send_gap = r["t_send126"] - r["t_send130"]
            if got1 and got2:
                delta = r["t1002"] - r["t1001"]
                wire = r["n1002"] * 10.0 / a.baud      # 8N1 = 10 بت لكل بايت
                deltas.append(delta * 1000.0)
                wires.append(wire * 1000.0)
                lat1.append((r["t1001"] - r["t_send130"]) * 1000.0)
                lat2.append((r["t1002"] - r["t_send126"]) * 1000.0)
                if delta < 0:
                    swapped += 1
                writer.writerow([
                    i, 1, ms(send_gap), ms(r["t1001"] - r["t_send130"]),
                    ms(r["t1002"] - r["t_send126"]), ms(delta), ms(wire),
                    r["n1001"], r["n1002"], r["o1001"].get("L"), r["o1001"].get("R"),
                    r["o1002"].get("gz")])
            else:
                writer.writerow([
                    i, 0, ms(send_gap),
                    ms(r["t1001"] - r["t_send130"]) if got1 else "",
                    ms(r["t1002"] - r["t_send126"]) if got2 else "",
                    "", "", r["n1001"], r["n1002"], "", "", ""])
            if i % 20 == 0 or i == a.cycles:
                last = ("Δ=%+.2f ms" % deltas[-1]) if (got1 and got2) else "✗ ناقصة"
                print("   %4d/%d   مكتملة %d   آخرها %s"
                      % (i, a.cycles, len(deltas), last))
            time.sleep(a.gap)
        elapsed = time.monotonic() - t_start
        fh.flush()

        # ── النتائج ─────────────────────────────────────────────────────
        print("\n[4] النتائج (بالملّي ثانية):\n")
        complete = len(deltas)
        if complete:
            d = describe(deltas)
            w = describe(wires)
            print("    Δ = وصول(T:1002) − وصول(T:1001)   —  %d دورة مكتملة" % complete)
            print("      الوسيط   : %8.2f" % d["median"])
            print("      التشتت σ : %8.2f      (IQR: %.2f … %.2f)"
                  % (d["stdev"], d["q1"], d["q3"]))
            print("      الأقصى   : %8.2f      (الأدنى %.2f)" % (d["max"], d["min"]))
            print()
            print("    أرضية السلك (زمن إرسال سطر T:1002 وحده @ %d): وسيط %.2f"
                  % (a.baud, w["median"]))
            print("      ⇒ ما فوقها من Δ هو معالجة اللوحة + كمون المشغّل، لا السلك.")
            l1, l2 = describe(lat1), describe(lat2)
            print("    زمن الردّ من الإرسال: T:1001 وسيط %.2f · T:1002 وسيط %.2f"
                  % (l1["median"], l2["median"]))
            if swapped:
                print("    ⚠ وصل T:1002 **قبل** T:1001 في %d دورة (Δ سالب) — مسجّل كما هو."
                      % swapped)
        else:
            print("    (لا دورة مكتملة — لا رقم)")

        if junk:
            print("\n[!] أسطر غير JSON وصلت أثناء القياس (%d):" % len(junk))
            for text in junk[:8]:
                print("      | " + text[:100])

        # ── الحكم ───────────────────────────────────────────────────────
        frac = complete / float(a.cycles) if a.cycles else 0.0
        ok = frac >= GO_MIN_FRACTION and not junk
        reasons = ["دورات مكتملة: %d/%d (%.1f%%) خلال %.1fث"
                   % (complete, a.cycles, frac * 100.0, elapsed),
                   "ردود ضائعة: T:1001 %d · T:1002 %d · أسطر غير JSON: %d"
                   % (miss_1001, miss_1002, len(junk))]
        if complete:
            reasons.append("Δ: وسيط %.2f · σ %.2f · أقصى %.2f ms   ← رقم الأساس"
                           % (d["median"], d["stdev"], d["max"]))
        if frac < GO_MIN_FRACTION:
            reasons.append("سبب الرفض: أقلّ من %.0f%% مكتملة ⇒ الرقم لا يصلح أساساً."
                           " شغّل test_link أولاً." % (GO_MIN_FRACTION * 100))
        if junk:
            reasons.append("سبب الرفض: خرج غير JSON ⇒ T:143 أو T:605 لم يُطفأ فعلياً.")
        reasons.append("⚠ يقيس زمن **الوصول** لا لحظة أخذ العيّنة داخل اللوحة.")
        reasons.append("⚠ للمقارنة بعد الفيرموير: نفس --cycles و--gap ونفس المنفذ.")
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
