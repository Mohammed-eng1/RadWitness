#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_c1.py — فحص RPLIDAR C1 وحده (pyserial فقط، بلا استيراد من pi.*)
=====================================================================
⚠ نسخة مكتوبة من البروتوكول القياسي لأن ملفك المجرَّب على الجهاز
  (rms-clean/Claude outputs/test_c1.py على ويندوز) لم يكن متاحاً هنا.
  إن اختلف عنه فاستبدل هذا الملف بملفك — و`pi/sensors/lidar_c1.py` يستعمل
  نفس الأوامر (STOP/SCAN وعُقد 5 بايت).

يطبع: المنفذ · معلومات الجهاز والصحّة · لفّات/ث · نقاط/لفّة · أقرب نقطة.
ويرسل STOP عند الخروج دائماً (المحرك لا يقف بغيره).

    python3 -m pi.tests.ugv01.test_c1                # 5 ثوانٍ
    python3 -m pi.tests.ugv01.test_c1 --seconds 10 --port /dev/ttyUSB0
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("خطأ: pyserial غير مثبّتة.  pip3 install pyserial")

VID, PID, BAUD = 0x10C4, 0xEA60, 460800        # CP210x · مقاس على الجهاز
STOP, SCAN = b"\xA5\x25", b"\xA5\x20"
GET_INFO, GET_HEALTH = b"\xA5\x50", b"\xA5\x52"
SCAN_DESC = b"\xA5\x5A\x05\x00\x00\x40\x81"


def find_port() -> str | None:
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    for pat in ("/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*Silicon_Labs*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return os.path.realpath(hits[0])
    return None


def request(ser, cmd: bytes, n_payload: int, timeout: float = 1.0):
    ser.reset_input_buffer()
    ser.write(cmd)
    buf = bytearray()
    t_end = time.time() + timeout
    while time.time() < t_end and len(buf) < 7 + n_payload:
        buf.extend(ser.read(7 + n_payload - len(buf)))
    k = buf.find(b"\xA5\x5A")
    if k < 0 or len(buf) < k + 7 + n_payload:
        return None
    return bytes(buf[k + 7:k + 7 + n_payload])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args()
    port = a.port or find_port()
    if not port:
        print("❌ لم يُعثر على الليدار (CP210x 10C4:EA60). ls /dev/serial/by-id")
        return 1
    print(f"المنفذ: {port} @ {BAUD}")
    ser = serial.Serial(port, BAUD, timeout=0.2)
    try:
        ser.write(STOP)
        time.sleep(0.05)
        info = request(ser, GET_INFO, 20)
        if info:
            print(f"الطراز {info[0]} · فيرموير {info[2]}.{info[1]:02d} · "
                  f"عتاد {info[3]} · تسلسلي {info[4:].hex().upper()}")
        else:
            print("⚠ لا ردّ على GET_INFO (يكمل)")
        health = request(ser, GET_HEALTH, 3)
        if health:
            st = {0: "سليم ✅", 1: "تحذير ⚠", 2: "خطأ ❌"}.get(health[0], health[0])
            print(f"الصحّة: {st} (رمز {health[1] | health[2] << 8})")
        ser.reset_input_buffer()
        ser.write(SCAN)
        buf = bytearray()
        t_end = time.time() + 2.0
        while time.time() < t_end and SCAN_DESC not in buf:
            buf.extend(ser.read(64))
        k = buf.find(SCAN_DESC)
        if k < 0:
            print("❌ لا واصف SCAN خلال 2ث")
            return 1
        buf = buf[k + 7:]
        revs, cur, counts, resync = 0, [], [], 0
        nearest = None
        t0 = time.time()
        while time.time() - t0 < a.seconds:
            buf.extend(ser.read(1024))
            i = 0
            while len(buf) - i >= 5:
                b0, b1 = buf[i], buf[i + 1]
                if (b0 & 1) == ((b0 >> 1) & 1) or not (b1 & 1):
                    i += 1
                    resync += 1
                    continue
                ang = ((buf[i + 2] << 7) | (b1 >> 1)) / 64.0
                dist = (buf[i + 3] | buf[i + 4] << 8) / 4000.0
                if (b0 & 1) and cur:
                    revs += 1
                    counts.append(len(cur))
                    cur = []
                if dist >= 0.05:
                    cur.append((ang, dist))
                    if nearest is None or dist < nearest[1]:
                        nearest = (ang, dist)
                i += 5
            del buf[:i]
        dt = time.time() - t0
        print(f"لفّات: {revs} في {dt:.1f}ث = {revs / dt:.1f} لفّة/ث "
              f"(مقاس سابقاً ~10.3)")
        if counts:
            print(f"نقاط/لفّة (≥5سم): وسيط {sorted(counts)[len(counts) // 2]} "
                  f"(مقاس سابقاً ~500) · إعادة تزامن {resync}")
        if nearest:
            print(f"أقرب نقطة: {nearest[1]:.3f}م عند {nearest[0]:.1f}° "
                  f"(زاوية الشركة: 0=السهم، مع عقارب الساعة)")
        return 0 if revs > 0 else 1
    finally:
        try:
            ser.write(STOP)                       # 🔴 المحرك لا يقف بغيره
            time.sleep(0.05)
        finally:
            ser.close()
            print("STOP أُرسل.")


if __name__ == "__main__":
    sys.exit(main())
