# -*- coding: utf-8 -*-
"""
motors_off.py — EMERGENCY STOP: force every motor channel off
===============================================================
**أداة خارجية** (طوارئ — البند 8).

Run this if a wheel keeps spinning and will not stop.

    python3 -m pi.tests.motors_off

🔴 **لماذا لا تكفي إعادة تشغيل الراسبري**: PCA9685 **شريحة مستقلة لها
سجلاتها الخاصة**. تُغذّى من خط 3.3V/5V، وتحتفظ بآخر قيمة PWM كُتبت فيها.
إعادة إقلاع الراسبري تعيد تشغيل البرنامج **ولا تمسّ الشريحة** — فتبقى
القناة مكتوبة والعجلة تدور. لا بدّ من كتابة «أطفئ الكل» صراحةً، أو قطع
التغذية عن الشريحة نفسها.

⚠ هذا السكربت **يتجاهل Ctrl+C عمداً** حتى يكتمل الإطفاء: عطل مقاس
   (2026-09-12) أن ضغطات Ctrl+C المتتابعة أجهضت مسار التنظيف نفسه
   فبقيت عجلة تدور بلا توقف.
"""
from __future__ import annotations

import signal
import sys

from pi.config import (FREENOVE_I2C_BUS, FREENOVE_PCA9685_ADDR,
                       FREENOVE_WHEEL_CHANNELS)

MODE1       = 0x00
LED0_ON_L   = 0x06
ALLLED_ON_L = 0xFA


def _ignore_interrupts() -> None:
    """
    🔴 يُصمّ Ctrl+C أثناء الإطفاء. الخطر ليس الإزعاج: المقاطعة **وسط
       مسار الإيقاف** تتركه ناقصاً، والعجلة تبقى تدور — وهو بالضبط
       العطل الذي وُجد لأجله هذا السكربت.
    """
    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            try:
                signal.signal(sig, signal.SIG_IGN)
            except Exception:                 # noqa: BLE001
                pass


def force_all_off(bus_num: int = FREENOVE_I2C_BUS,
                  addr: int = FREENOVE_PCA9685_ADDR) -> int:
    """يكتب «أطفئ الكل» ثم يصفّر كل قناة على حدة. يعيد 0 عند النجاح."""
    try:
        import smbus2
    except ImportError:
        print("⛔ smbus2 not installed. Run inside the venv:")
        print("   cd ~/RMS-Rover-v2 && source venv/bin/activate")
        return 2
    try:
        bus = smbus2.SMBus(bus_num)
    except Exception as e:                    # noqa: BLE001
        print(f"⛔ Cannot open i2c-{bus_num}: {e}")
        print("   If the board is unpowered this is expected — and then the")
        print("   motor cannot be spinning from the driver either.")
        return 2

    ok = True
    # ① أمر النداء العام: أطفئ كل القنوات دفعة واحدة (بت «إطفاء كامل»)
    try:
        bus.write_i2c_block_data(addr, ALLLED_ON_L, [0x00, 0x00, 0x00, 0x10])
        print("✅ ALL_LED_OFF sent (all 16 channels off)")
    except Exception as e:                    # noqa: BLE001
        ok = False
        print(f"⚠ ALL_LED_OFF failed: {e}")

    # ② ثم **كل قناة على حدة** — الحزام والحمّالة: لو رفضت الشريحة أمر
    #    النداء العام (بت ALLCALL مطفأ في MODE1) بقيت القنوات مكتوبة.
    for ch in range(16):
        try:
            bus.write_i2c_block_data(addr, LED0_ON_L + 4 * ch,
                                     [0x00, 0x00, 0x00, 0x10])
        except Exception as e:                # noqa: BLE001
            ok = False
            print(f"⚠ channel {ch} failed: {e}")
    if ok:
        print("✅ All 16 channels individually forced off")

    # ③ التحقّق بالقراءة — لا نكتفي بنجاح الكتابة (درس MPU المقلَّدة:
    #    شريحة تقبل الكتابة ولا تحتفظ بها).
    stuck = []
    for side in ("left", "right"):
        for ch_a, ch_b in FREENOVE_WHEEL_CHANNELS[side]:
            for ch in (ch_a, ch_b):
                try:
                    off_h = bus.read_byte_data(addr, LED0_ON_L + 4 * ch + 3)
                    if not (off_h & 0x10):
                        stuck.append(ch)
                except Exception:             # noqa: BLE001
                    pass
    try:
        bus.close()
    except Exception:                         # noqa: BLE001
        pass

    print()
    if stuck:
        print(f"🔴 Channels still NOT off after write-back check: {sorted(set(stuck))}")
        print("   🔴 PULL THE BATTERY CONNECTOR NOW. Do not rely on the switch.")
        print("   Then tell me which channel numbers are listed above.")
        return 3
    print("✅ Verified by reading back: motor channels are off.")
    print("   If a wheel is STILL spinning, the driver board is latched or")
    print("   miswired — pull the battery connector; software cannot fix it.")
    return 0 if ok else 1


def main() -> int:
    _ignore_interrupts()
    print("🛑 EMERGENCY MOTOR STOP")
    print(f"   PCA9685 on i2c-{FREENOVE_I2C_BUS} "
          f"@ 0x{FREENOVE_PCA9685_ADDR:02x}")
    print("   (Ctrl+C is disabled here on purpose — let it finish.)\n")
    return force_all_off()


if __name__ == "__main__":
    sys.exit(main())
