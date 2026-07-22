#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_lora.py — اختبار منفرد لوصلة اللورا HC-14 على الراسبري
===========================================================
يبثّ إطار PING دورياً ويستقبل الإطارات الواردة، متحققاً من الـchecksum،
ويعرض أي رد (PONG) مع زمن الذهاب-والإياب (RTT). اختبار حيّة الوصلة قبل
الدمج (الشاشة تردّ من فيرمويرها في M4).

⚠ العتاد: HC-14 عبر محول USB-Serial → منفذ USB (/dev/ttyUSB*)، 9600 باود
   (الخيار الموصى — أبسط من UART ثانٍ على الراسبري، ويحرّر UART العتاد
   للـGPS). أبعِد هوائي 433MHz عن وحدة GPS.

صيغة الإطار (v2 — ≤60 بايت + checksum XOR):
    $<payload>*<HH>\n
    <payload>  نص ASCII، ≤ 56 بايت (يترك مجالاً للتأطير)
    <HH>       بايتان hex = XOR لكل بايتات payload (على طراز NMEA)
    مثال:  $PING,42*3A\n
    (المواصفة الكاملة للرسائل الثلاث — روفر/لورا/WebSocket — تُوثَّق في
     docs/protocol.md ضمن M4؛ هنا نثبّت التأطير ونختبر الوصلة فقط.)

التشغيل على الراسبري:
    python3 pi/tests/test_lora.py                 # حلقة PING (تحتاج طرفاً ثانياً يردّ)
    python3 pi/tests/test_lora.py /dev/ttyUSB1    # لتحديد منفذ آخر
    python3 pi/tests/test_lora.py at              # ★ فحص وحدة واحدة عبر أوامر AT

★ فحص الوحدة الواحدة (at): بوحدة HC-14 واحدة لا يمكن اختبار الإرسال اللاسلكي
   (يلزم طرف ثانٍ في M4). لكن أوامر AT تؤكد أن الوحدة حيّة والتوصيل (TX/RX)
   سليم: **اربط دبوس SET بـGND** (وضع الأوامر)، ثم شغّل `... at` → يجب أن
   تردّ الوحدة `OK` على `AT`. أعد SET حراً بعدها للوضع الشفاف.
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("خطأ: مكتبة pyserial غير مثبّتة. ثبّتها عبر setup_pi.sh.")

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 9600
MAX_PAYLOAD = 56


def xor_checksum(payload: str) -> int:
    """XOR لكل بايتات payload (بترميز ASCII)."""
    c = 0
    for b in payload.encode("ascii"):
        c ^= b
    return c


def build_frame(payload: str) -> bytes:
    """يبني إطار v2: $<payload>*<HH>\\n مع فرض حد الطول."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload يتجاوز {MAX_PAYLOAD} بايت")
    return f"${payload}*{xor_checksum(payload):02X}\n".encode("ascii")


def parse_frame(line: str):
    """يفكّ إطاراً ويتحقق من الـchecksum. يُعيد payload أو None عند الخطأ."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, cksum = line[1:].rpartition("*")
    try:
        if int(cksum, 16) != xor_checksum(body):
            return None                    # checksum لا يطابق → إطار تالف
    except ValueError:
        return None
    return body


def at_diagnostics(port: str) -> None:
    """فحص وحدة HC-14 واحدة عبر أوامر AT (يتطلب SET→GND)."""
    try:
        ser = serial.Serial(port, BAUD, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"خطأ: تعذّر فتح {port} ({e}). تحقق من محول USB-Serial (ls /dev/ttyUSB*).")

    print(f"وضع AT على {port} @ {BAUD}.")
    print("⚠ تأكد أن دبوس SET موصول بـGND (وإلا الوحدة في الوضع الشفاف ولن تردّ).\n")
    any_reply = False
    for cmd in ("AT", "AT+V", "AT+RX"):            # حيّة / إصدار / الإعدادات
        ser.reset_input_buffer()
        ser.write(cmd.encode("ascii"))
        time.sleep(0.6)
        n = ser.in_waiting
        resp = ser.read(n).decode("ascii", errors="replace").strip() if n else ""
        print(f"→ {cmd:8s}  ←  {resp!r}")
        if resp:
            any_reply = True
    ser.close()
    if any_reply:
        print("\n✅ الوحدة تردّ — السيريال والتوصيل (TX/RX) سليمان. أعد SET حراً للوضع الشفاف.")
    else:
        print("\n⚠ لا ردّ. تحقّق بالترتيب: SET→GND، TX↔RX غير معكوسين، التغذية 3.3-5V، الباود 9600.")


def main() -> None:
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"خطأ: تعذّر فتح {PORT} ({e}). تحقق من توصيل محول USB-Serial (lsusb / ls /dev/ttyUSB*).")

    print(f"وصلة اللورا على {PORT} @ {BAUD}. يبثّ PING كل ثانية ويصغي. Ctrl-C للإيقاف.\n")
    seq = 0
    last_ping_ms = 0.0
    ping_sent_at = {}
    try:
        while True:
            now = time.time()
            if (now - last_ping_ms) >= 1.0:
                last_ping_ms = now
                seq += 1
                payload = f"PING,{seq}"
                ser.write(build_frame(payload))
                ping_sent_at[seq] = now
                print(f"→ أُرسل  {payload}")

            raw = ser.readline().decode("ascii", errors="replace")
            if not raw.strip():
                continue
            payload = parse_frame(raw)
            if payload is None:
                print(f"← تالف/checksum خاطئ: {raw.strip()!r}")
                continue

            # رد PONG,<seq> → احسب RTT
            if payload.startswith("PONG,"):
                try:
                    rseq = int(payload.split(",")[1])
                    rtt_ms = (now - ping_sent_at.pop(rseq, now)) * 1000.0
                    print(f"← PONG seq={rseq}  RTT={rtt_ms:.0f}ms ✅")
                except (IndexError, ValueError):
                    print(f"← {payload}")
            else:
                print(f"← استُقبل {payload}")
    except KeyboardInterrupt:
        print("\nتوقّف.")
    finally:
        ser.close()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "at":
        at_diagnostics(sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0")
    else:
        main()
