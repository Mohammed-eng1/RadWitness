#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_camera.py — اختبار منفرد للويب كام (USB) على الراسبري
==========================================================
يفتح الكاميرا عبر OpenCV، يقرأ إطاراً، يطبع الأبعاد، ويحفظ لقطة موسومة
بالوقت في captures/ (نفس مجلد لقطات الشذوذ التلقائية لاحقاً).

⚠ العتاد: ويب كام USB على أي منفذ — cv2.VideoCapture(0). إن تعددت
   الكاميرات جرّب فهرساً آخر: python3 test_camera.py 1

التشغيل على الراسبري:
    python3 pi/tests/test_camera.py        # يحفظ لقطة في captures/
"""
import sys
import os
import time

try:
    import cv2
except ImportError:
    sys.exit("خطأ: مكتبة opencv-python غير مثبّتة. ثبّتها عبر setup_pi.sh.")

INDEX = int(sys.argv[1]) if len(sys.argv) > 1 else 0
# مجلد اللقطات في جذر المستودع: pi/tests/ → ../../captures
CAPTURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "captures"))


def main() -> None:
    cap = cv2.VideoCapture(INDEX)
    if not cap.isOpened():
        sys.exit(f"خطأ: تعذّر فتح الكاميرا على الفهرس {INDEX}. تحقق من التوصيل أو جرّب فهرساً آخر.")

    # التقاط بضعة إطارات لتستقر التعريضة قبل الحفظ
    frame = None
    for _ in range(5):
        ok, frame = cap.read()
        time.sleep(0.1)
    if not ok or frame is None:
        cap.release()
        sys.exit("خطأ: فُتحت الكاميرا لكن تعذّرت قراءة إطار.")

    h, w = frame.shape[:2]
    os.makedirs(CAPTURES_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(CAPTURES_DIR, f"test_{stamp}.jpg")
    cv2.imwrite(path, frame)

    print(f"الكاميرا تعمل ✅  الأبعاد={w}×{h}")
    print(f"حُفظت لقطة: {path}")
    cap.release()


if __name__ == "__main__":
    main()
