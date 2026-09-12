# -*- coding: utf-8 -*-
"""
calibrate_adc.py — معايرة قراءة جهد حزمة المحركات (ADS7830)
=============================================================
**أداة خارجية** (سكربت معايرة على العتاد — البند 8).

🔴 **لماذا لزمت**: معامل التحويل يتبع **إصدار لوح Freenove**، و
   Freenove **لا يكتشفه من العتاد** — يسأل المستخدم ويحفظه في
   `params.json` (مقروء من مصدره: `parameter.py` → `get_valid_input`).
   فلا سجلّ يُقرأ ولا هوية تُستعلم. الاحتمالان من `adc.py`:

       إصدار 1 :  V = raw/255 × 3.3 × 3   (مدى كامل 9.90V)
       إصدار 2 :  V = raw/255 × 5.2 × 2   (مدى كامل 10.40V)

   وبينهما ~5% — تكفي لتُخفي بطارية منخفضة أو تُطلق إنذاراً كاذباً.

✅ **والحلّ لا يحتاج معرفة الإصدار أصلاً**: قياس واحد بالملتيميتر يعطي
   المعامل **المركَّب** مباشرةً:

       K = الجهد المقاس × 255 ÷ البايت الخام

   ثم `ADS7830_VREF_V × ADS7830_DIVIDER = K`. فالسؤال «أي إصدار؟» يسقط.

⚠ أوقف السيرفر أولاً (يمسك الناقل):  pkill -f "pi.web.server"
"""
from __future__ import annotations

import argparse
import sys

from pi.config import (ADS7830_I2C_BUS, ADS7830_ADDR, ADS7830_BATT_CHANNEL,
                       ADS7830_VREF_V, ADS7830_DIVIDER,
                       MOTOR_BATT_MIN_V, MOTOR_BATT_MAX_V)

_CMD_BASE = 0x84
#: الاحتمالان المنسوخان من `adc.py` في مصدر Freenove — (اسم، vref، مقسّم)
_CANDIDATES = (("PCB v1", 3.3, 3.0), ("PCB v2", 5.2, 2.0))


def read_raw(bus, addr: int, channel: int):
    """بايت مستقرّ (قراءتان متطابقتان) — نفس منهج Freenove."""
    cmd = _CMD_BASE | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
    bus.write_byte(addr, cmd)
    prev = None
    for _ in range(6):
        v = bus.read_byte(addr)
        if v == prev:
            return v
        prev = v
    return prev


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Calibrate the motor-battery ADC against a multimeter")
    ap.add_argument("--samples", type=int, default=8,
                    help="how many raw readings to average (default 8)")
    ap.add_argument("--channel", type=int, default=ADS7830_BATT_CHANNEL)
    a = ap.parse_args()

    try:
        import smbus2
    except ImportError:
        print("⛔ smbus2 missing. Run inside the venv.")
        return 2
    try:
        bus = smbus2.SMBus(ADS7830_I2C_BUS)
    except Exception as e:                    # noqa: BLE001
        print(f"⛔ Cannot open i2c-{ADS7830_I2C_BUS}: {e}")
        return 2

    print(f"ADS7830 on i2c-{ADS7830_I2C_BUS} @ 0x{ADS7830_ADDR:02x}, "
          f"channel {a.channel}")
    raws = []
    for _ in range(max(1, a.samples)):
        try:
            r = read_raw(bus, ADS7830_ADDR, a.channel)
        except Exception as e:                # noqa: BLE001
            print(f"⛔ Read failed: {e}")
            return 2
        if r is not None:
            raws.append(r)
    if not raws:
        print("⛔ No stable reading. Check wiring and power.")
        return 3
    raw = sum(raws) / len(raws)
    print(f"\nRaw byte: {raw:.1f}  (min {min(raws)}, max {max(raws)}, "
          f"{len(raws)} samples)")
    if raw <= 0:
        print("🔴 Raw is 0 — the channel reads nothing. Wrong channel, or")
        print("   the motor battery is disconnected.")
        return 3

    print("\n  What each Freenove PCB version would say:")
    for name, vref, div in _CANDIDATES:
        print(f"    {name}:  {raw / 255.0 * vref * div:5.2f} V   "
              f"(vref {vref} x divider {div:.0f})")
    cur = raw / 255.0 * ADS7830_VREF_V * ADS7830_DIVIDER
    print(f"\n  Current config says: {cur:.2f} V  "
          f"(vref {ADS7830_VREF_V} x divider {ADS7830_DIVIDER})")

    print("\n🔎 Now measure the MOTOR battery with a multimeter.")
    print("   (The 2S pack that powers the Freenove board — not the Pi.)")
    try:
        txt = input("   Measured volts (Enter to skip): ").strip()
    except (EOFError, KeyboardInterrupt):
        print("\n⛔ Cancelled."); return 1
    except Exception:                         # noqa: BLE001
        print("  ⚠ Could not read input."); return 1
    if not txt:
        print("\n⏭ Skipped. Nothing changed — the value stays UNCALIBRATED.")
        return 0
    try:
        measured = float(txt.replace(",", "."))
    except ValueError:
        print("  ⚠ Not a number. Nothing changed.")
        return 1
    if not (MOTOR_BATT_MIN_V <= measured <= MOTOR_BATT_MAX_V):
        # 🔴 خارج نطاق 2S المعقول = غالباً قِست الحزمة الخطأ
        print(f"\n🔴 {measured} V is outside the sane 2S range "
              f"({MOTOR_BATT_MIN_V}-{MOTOR_BATT_MAX_V} V).")
        print("   Did you measure the Pi's UPS pack by mistake? That one is")
        print("   3S (~11 V) and is read by the INA219, not this ADC.")
        return 3

    k = measured * 255.0 / raw
    print(f"\n  ── RESULT ──")
    print(f"  Combined factor K = {measured} x 255 / {raw:.1f} = {k:.3f}")
    # أقرب احتمال معروف — للتوثيق فقط، والقرار للقياس
    best = min(_CANDIDATES, key=lambda c: abs(c[1] * c[2] - k))
    err = abs(best[1] * best[2] - k) / k * 100.0
    print(f"  Closest known layout: {best[0]} "
          f"({best[1]} x {best[2]:.0f} = {best[1]*best[2]:.2f}), off by {err:.1f}%")
    print(f"\n  Put these in pi/config.py:")
    print(f"     ADS7830_VREF_V  = {k:.3f}")
    print(f"     ADS7830_DIVIDER = 1.0")
    print(f"  (Any pair whose product is {k:.3f} works — this is the simple one.)")
    print(f"\n  Then check it:")
    print(f"     python3 -m pi.tests.calibrate_adc --samples 8")
    print(f"  The printed 'Current config says' should match your multimeter.")
    if err > 10.0:
        print(f"\n  ⚠ {err:.0f}% away from both known layouts. Re-check that you")
        print(f"     measured the 2S motor pack, and that the robot is ON.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
