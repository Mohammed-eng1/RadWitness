#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_link.py — هل لوحة UGV01 ترد أصلاً؟  (اختبار الإقلاع الأول · لا يحرّك شيئاً)
================================================================================
أوّل سؤال في يوم الاستلام، وقبله لا معنى لأي اختبار آخر: **هل الطرف الآخر حيّ
ويتكلّم JSON نظيفاً؟** فتح منفذ UART ينجح دائماً — حتى لو كان الطرف الآخر ميتاً
أو أصمّ — فنجاح `serial.Serial()` ليس شهادة حياة.

ما يفعله بالترتيب:
  1. يفتح المنفذ وينتظر إقلاع ESP32 (فتح المنفذ يضرب DTR/RTS فيعيد إقلاع
     اللوحة على معظم محوّلات USB) ويجمع ضجيج الإقلاع للعرض لا للحكم.
  2. يرسل تهيئة الإقلاع الثلاثية:
       {"T":900,"main":3,"module":0}  ← إلزامي: بدونه تستعمل اللوحة ثوابت
                                        روبوت آخر **بصمت** (قطر عجلة وعرض
                                        مسار مختلفين) فتكذب كل سرعة مقاسة.
       {"T":143,"cmd":0}              ← إطفاء صدى السيريال
       {"T":605,"cmd":0}              ← إطفاء طباعة التشخيص
     وبلا الأخيرين يختلط خرج بشري مع JSON فينكسر أي تحليل سطري.
  3. عشر جولات `{"T":130}` وينتظر `{"T":1001,...}`: يقيس زمن الردّ، ويطبع
     الجهد وكل حقل في الردّ، ويعدّ الإخفاقات والأسطر غير الـJSON.

الحكم:
  GO     ‏10/10 ردود صالحة **و** صفر أسطر غير JSON أثناء القياس.
  NO-GO  أي ردّ ضائع، أو أي سطر غير JSON (الصدى/التشخيص لم يُطفأ).

التشغيل:
    python3 -m pi.tests.ugv01.test_link
    python3 -m pi.tests.ugv01.test_link --port /dev/ttyUSB0 --rounds 20
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
    import serial  # pyserial — الاعتماد الوحيد لهذه السكربتات
except ImportError:
    sys.exit("خطأ: pyserial غير مثبّتة.  pip3 install pyserial")


# ═══ أدوات مستقلة ═══════════════════════════════════════════════════════
# مكرّرة عمداً في كل سكربت هنا: هذه اختبارات يوم أول، ولو فشل استيراد من
# pi.* لأي سبب وجب أن تبقى عاملة. الاستقلال ميزة لا تكرار.
BAUD = 115200
PORT_CANDIDATES = ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial0")
BOOT_WAIT_S = 3.0          # إقلاع ESP32 المقاس على المنصّة السابقة ~3ث
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

INIT_CMDS = (
    ({"T": 900, "main": 3, "module": 0}, "نوع الروبوت UGV01 (mainType=3) — إلزامي"),
    ({"T": 143, "cmd": 0}, "إطفاء صدى السيريال"),
    ({"T": 605, "cmd": 0}, "إطفاء طباعة التشخيص"),
)


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
        self._lock = threading.Lock()   # الكتابة قد تأتي من خيط نبضة القلب

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
    """يفتح المنفذ بمهلة قراءة قصيرة (القراءة نفسها هي مؤقّت الحلقات)."""
    try:
        ser = serial.Serial(port, baud, timeout=0.05, exclusive=True)
    except TypeError:                 # إصدارات pyserial قديمة بلا exclusive
        ser = serial.Serial(port, baud, timeout=0.05)
    return Link(ser)


def init_board(link, gap=0.30, verbose=True):
    """تهيئة الإقلاع الثلاثية — تُرسل في كل سكربت لأن اللوحة تنساها مع كل إقلاع."""
    for cmd, why in INIT_CMDS:
        if verbose:
            print("   → %-32s  %s" % (json.dumps(cmd, separators=(",", ":")), why))
        link.send(cmd)
        time.sleep(gap)
    link.ser.reset_input_buffer()     # ما ردّته التهيئة لا يخصّ القياس


def new_csv(name, header):
    """يفتح ملف CSV موسوماً بالوقت داخل logs/ ويعيد (الملف، الكاتب، المسار)."""
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, "%s_%s.csv" % (name, time.strftime("%Y%m%d_%H%M%S")))
    fh = open(path, "w", newline="", encoding="utf-8")
    writer = csv.writer(fh)
    writer.writerow(header)
    return fh, writer, path


def verdict(ok, reasons):
    """يطبع الأسباب ثم يختم بسطر GO/NO-GO **وحده**، ويعيد رمز الخروج."""
    print()
    print("═" * 66)
    for line in reasons:
        print("  " + line)
    print("═" * 66)
    print("GO" if ok else "NO-GO")
    return 0 if ok else 1


# ═══ الاختبار ════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="UGV01: فحص وصلة السيريال والردّ")
    ap.add_argument("--port", default=None, help="افتراضياً: بحث تلقائي")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--boot-wait", type=float, default=BOOT_WAIT_S,
                    help="انتظار إقلاع ESP32 بعد فتح المنفذ (ث)")
    ap.add_argument("--rounds", type=int, default=10, help="عدد جولات T:130")
    ap.add_argument("--reply-timeout", type=float, default=1.0,
                    help="مهلة انتظار ردّ الجولة الواحدة (ث)")
    a = ap.parse_args()

    print("═" * 66)
    print("  UGV01 · فحص الوصلة (test_link) — لا يحرّك المحركات إطلاقاً")
    print("═" * 66)

    port = find_port(a.port)
    if not port:
        return verdict(False, [
            "لم يُعثر على أي منفذ: %s" % ", ".join(PORT_CANDIDATES),
            "وصّل اللوحة بالـUSB أو مرّر --port صراحةً.",
        ])
    print("المنفذ: %s @ %d" % (port, a.baud))

    try:
        link = open_link(port, a.baud)
    except Exception as exc:
        return verdict(False, ["تعذّر فتح المنفذ: %s" % exc,
                               "تحقّق من الصلاحيات (مجموعة dialout) ومن عدم شغله ببرنامج آخر."])

    fh = None
    try:
        # ── 1. ضجيج الإقلاع: يُعرض ولا يُحاكم عليه ────────────────────────
        print("\n[1] انتظار إقلاع اللوحة %.1fث (فتح المنفذ يعيد إقلاع ESP32)…"
              % a.boot_wait)
        boot_lines = [t for t, _ in link.lines(a.boot_wait)]
        if boot_lines:
            print("    وصل %d سطراً أثناء الإقلاع (طبيعي — لا يدخل الحكم):" % len(boot_lines))
            for text in boot_lines[:6]:
                print("      | " + text[:100])
            if len(boot_lines) > 6:
                print("      | … و%d سطراً آخر" % (len(boot_lines) - 6))
        else:
            print("    صمت تام أثناء الإقلاع (طبيعي أيضاً — ليس حكماً).")
        link.ser.reset_input_buffer()

        # ── 2. التهيئة الإلزامية ────────────────────────────────────────
        print("\n[2] تهيئة الإقلاع:")
        init_board(link)

        # ── 3. جولات الاستعلام ──────────────────────────────────────────
        print("\n[3] %d جولة {\"T\":130} → انتظار {\"T\":1001}:\n" % a.rounds)
        fh, writer, path = new_csv(
            "link", ["round", "ok", "latency_ms", "L", "R", "r", "p", "y", "temp", "v"])

        replies, latencies, junk, first_obj = 0, [], [], None
        for i in range(1, a.rounds + 1):
            obj, dt, strays = link.ask({"T": 130}, 1001, a.reply_timeout)
            junk.extend(s for s in strays if not s.startswith("{"))
            if obj is None:
                print("   %2d/%d   ✗ لا ردّ خلال %.1fث" % (i, a.rounds, a.reply_timeout))
                writer.writerow([i, 0, round(dt * 1000, 1), "", "", "", "", "", "", ""])
            else:
                replies += 1
                latencies.append(dt * 1000.0)
                first_obj = first_obj or obj
                print("   %2d/%d   ✓ %6.1f ms   v=%s  L=%s R=%s  temp=%s"
                      % (i, a.rounds, dt * 1000.0, obj.get("v"), obj.get("L"),
                         obj.get("R"), obj.get("temp")))
                writer.writerow([i, 1, round(dt * 1000, 1),
                                 obj.get("L"), obj.get("R"), obj.get("r"),
                                 obj.get("p"), obj.get("y"), obj.get("temp"),
                                 obj.get("v")])
            time.sleep(0.10)
        fh.flush()

        # ── 4. عرض الردّ كاملاً ─────────────────────────────────────────
        print("\n[4] حقول أول ردّ T:1001 كاملةً:")
        if first_obj is None:
            print("    (لا يوجد ردّ لعرضه)")
        else:
            labels = {"L": "سرعة اليسار المقاسة م/ث", "R": "سرعة اليمين المقاسة م/ث",
                      "r": "roll درجة", "p": "pitch درجة", "y": "yaw درجة",
                      "temp": "حرارة °م", "v": "جهد البطارية فولت"}
            for key, val in first_obj.items():
                if key == "T":
                    continue
                print("      %-6s = %-12s %s" % (key, val, labels.get(key, "")))
            if "y" in first_obj:
                print("    ⚠ الحقل y مدموج بالمغنيتومتر AK09918 — لا يُستعمل للملاحة.")
            if "v" not in first_obj:
                print("    ⚠ لا حقل v في الردّ — لا قراءة جهد من هذه اللوحة.")

        if junk:
            print("\n[!] أسطر غير JSON وصلت أثناء القياس (%d):" % len(junk))
            for text in junk[:8]:
                print("      | " + text[:100])

        # ── الحكم ───────────────────────────────────────────────────────
        ok = (replies == a.rounds) and not junk
        reasons = ["الردود: %d/%d" % (replies, a.rounds),
                   "أسطر غير JSON أثناء القياس: %d" % len(junk)]
        if latencies:
            reasons.append("زمن الردّ: أدنى %.1f · وسطي %.1f · أقصى %.1f ms"
                           % (min(latencies), sum(latencies) / len(latencies),
                              max(latencies)))
        if replies == 0:
            reasons.append("سبب الرفض: صمت كامل ⇒ سلك TX/RX/GND، أو سرعة سيريال"
                           " غير 115200، أو فيرموير لا يعمل.")
        elif replies < a.rounds:
            reasons.append("سبب الرفض: ردود ضائعة ⇒ وصلة غير مستقرة أو لوحة مشغولة.")
        if junk:
            reasons.append("سبب الرفض: خرج غير JSON ⇒ T:143 أو T:605 لم يُطفأ فعلياً.")
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
