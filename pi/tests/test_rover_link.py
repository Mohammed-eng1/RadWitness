#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_rover_link.py — أداة خارجية: هل ESP32 الروفر يسمعنا ويردّ أصلاً؟
======================================================================
كُتب للعطل المقاس 2026-08-06: أوامر الحركة تُرسل بلا أي خطأ والروبوت لا
يتحرك إطلاقاً — فتح UART ينجح وإن كان الطرف الآخر ميتاً أو أصمّ، ولا
شيء في مسار الإرسال يكشف ذلك. هذا المسبار يفصل الاحتمالات طبقة طبقة:

  1. **استماع سلبي** (3ث): بعض إصدارات الفيرموير تبثّ تلقائياً — أي بايت
     يصل يثبت أن RX الراسبري وسلك TX↔RX سليمان.
  2. **استعلام** `{"T":126}` خمس مرات: الردّ `{"T":1002,...}` يثبت الوصلة
     كاملة بالاتجاهين (TX الراسبري ← RX الروفر والعكس).
  3. (اختياري `--spin`) أمر دوران فعلي `{"T":1,"L":p,"R":-p}` ثانية واحدة
     مع تجديد الأمر — أنت تراقب العجلات: دارت والوصلة تردّ؟ المشكلة إذن
     ليست الوصلة بل ما بعدها.

تفسير النتائج:
  صمت كامل (لا سلبي ولا ردّ)  ⇒ سلك TX/RX/GND، أو الفيرموير لا يعمل.
     ⚠ احتمال موثَّق يستحق الفحص: **فُلِش فيرموير شاشة المتحكم على
     ESP32 الروفر بالخطأ** (المسار الآخر يبني firmware/controller_display
     في نفس الفترة) — فيرموير آخر = صمت JSON كامل رغم عتاد سليم.
  يردّ 1002 والعجلات لا تدور مع --spin ⇒ الوصلة سليمة والعطل في مسار
     المحركات: درايفر، تغذية محركات، أو حماية فيرموير (جهد منخفض؟).

التشغيل:
    python3 -m pi.tests.test_rover_link
    python3 -m pi.tests.test_rover_link --spin --power 0.3   # ⚠ يدير الروبوت
"""
from __future__ import annotations

import argparse
import json
import sys
import time

from pi.config import ROVER_PORT, ROVER_BAUD, MAX_MOTOR_POWER

try:
    import serial
except ImportError:
    sys.exit("خطأ: pyserial غير مثبّتة (pip install pyserial).")


def read_lines(ser, seconds: float) -> list:
    """يجمع الأسطر الواصلة خلال المهلة (خاماً، بلا افتراض صلاحية JSON)."""
    out, buf = [], b""
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                out.append(line.decode("ascii", errors="replace").strip())
        else:
            time.sleep(0.02)
    if buf.strip():
        out.append(buf.decode("ascii", errors="replace").strip() + " …(بلا \\n)")
    return [l for l in out if l]


def main() -> int:
    ap = argparse.ArgumentParser(description="مسبار وصلة الروفر (UART ثنائي الاتجاه)")
    ap.add_argument("--port", default=ROVER_PORT)
    ap.add_argument("--baud", type=int, default=ROVER_BAUD)
    ap.add_argument("--spin", action="store_true",
                    help="⚠ أمر دوران فعلي 1ث — يدير الروبوت بالمكان")
    ap.add_argument("--power", type=float, default=0.3)
    a = ap.parse_args()

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.05)
    except Exception as e:                    # noqa: BLE001
        sys.exit(f"⛔ تعذّر فتح {a.port}: {e}\n"
                 f"   (سيرفر يحجزه؟ sudo lsof {a.port})")
    print(f"المنفذ {a.port} @ {a.baud} مفتوح ✅")

    try:
        # ── 1) استماع سلبي ───────────────────────────────────────
        ser.reset_input_buffer()
        print("\n[1/3] استماع سلبي 3ث (بلا إرسال)…")
        passive = read_lines(ser, 3.0)
        if passive:
            print(f"  ✅ وصل {len(passive)} سطر تلقائياً — RX يعمل. أول سطرين:")
            for l in passive[:2]:
                print(f"     {l[:100]}")
        else:
            print("  (صمت — ليس حكماً بعد: بعض الفيرموير لا يبثّ إلا سؤالاً)")

        # ── 2) استعلام T=126 ─────────────────────────────────────
        print("\n[2/3] استعلام {\"T\":126} × 5…")
        replies = []
        for _ in range(5):
            ser.write(b'{"T":126}\n')
            replies += read_lines(ser, 0.4)
        got_1002 = False
        for l in replies:
            try:
                d = json.loads(l)
                if isinstance(d, dict) and d.get("T") == 1002:
                    got_1002 = True
                    print(f"  ✅ ردّ 1002 سليم: gz={d.get('gz')} · v={d.get('v')}")
                    break
            except Exception:                 # noqa: BLE001
                continue
        if not got_1002:
            if replies:
                print(f"  ⚠ وصل {len(replies)} سطر لكن لا 1002 مفهوماً. عيّنة:")
                for l in replies[:3]:
                    print(f"     {l[:100]}")
                print("  ⇒ شيء يبثّ لكنه ليس بروتوكول Wave Rover — فيرموير "
                      "مختلف؟ سرعة باود خاطئة؟")
            else:
                print("  ⛔ **صمت كامل بالاتجاهين** — الوصلة أو الفيرموير:")
                print("     • سلك TX/RX معكوس أو مفصول (تحرّك أثناء تركيب الحساسات؟)")
                print("     • GND غير مشترك بين الراسبري والروفر")
                print("     • ⚠ هل فُلِش فيرموير شاشة المتحكم على ESP32 الروفر؟")
            return 1

        # ── 3) أمر حركة فعلي ─────────────────────────────────────
        if a.spin:
            p = min(abs(a.power), MAX_MOTOR_POWER)
            print(f"\n[3/3] ⚠ دوران بالمكان 1ث بقوة {p} — أفرغ حوله (3ث)…")
            time.sleep(3.0)
            t0 = time.time()
            while time.time() - t0 < 1.0:
                ser.write(json.dumps(
                    {"T": 1, "L": p, "R": -p}).encode() + b"\n")
                time.sleep(0.1)
            ser.write(b'{"T":1,"L":0,"R":0}\n')   # إيقاف مضمون
            ser.write(b'{"T":1,"L":0,"R":0}\n')
            ans = input("  هل دارت العجلات فعلاً؟ [y/n] ").strip().lower()
            if ans.startswith("y"):
                print("  ✅ الوصلة والمحركات سليمتان معاً — عطل «لم يتحرك» "
                      "كان في طبقة أعلى (جسر/قوة/heartbeat)")
            else:
                print("  🔴 الوصلة تردّ والمحركات صامتة ⇒ العطل بعد ESP32:")
                print("     تغذية المحركات · الدرايفر · حماية جهد في الفيرموير")
        else:
            print("\n[3/3] ⏭ (أضف --spin لاختبار المحركات مباشرة عبر الوصلة)")
        print("\n✅ الوصلة ثنائية الاتجاه سليمة.")
        return 0
    finally:
        try:
            ser.write(b'{"T":1,"L":0,"R":0}\n')  # لا نترك أمراً معلّقاً
            ser.close()
        except Exception:                     # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
