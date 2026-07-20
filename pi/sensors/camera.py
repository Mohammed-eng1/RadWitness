# -*- coding: utf-8 -*-
"""
camera.py — قارئ الويب كام (منقول من pi/tests/test_camera.py المثبت)
====================================================================
cv2.VideoCapture(0). يُفتح كسولاً عند أول طلب لقطة (M1: لقطة عند الطلب؛
بث MJPEG في M2). آمن: غياب opencv/الكاميرا لا يُسقط السيرفر.
"""
import threading

from pi.config import CAMERA_INDEX, CAMERA_W, CAMERA_H

try:
    import cv2
    _CV2_OK = True
except Exception:                     # noqa: BLE001
    _CV2_OK = False


class CameraReader:
    def __init__(self, index: int = CAMERA_INDEX):
        self.available = _CV2_OK
        self.error = None if _CV2_OK else "opencv غير مثبّت"
        self._index = index
        self._cap = None
        self._opened = False
        self._lock = threading.Lock()

    def _ensure_open(self) -> bool:
        if self._cap is not None:
            return self._cap.isOpened()
        try:
            cap = cv2.VideoCapture(self._index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_H)
            self._cap = cap
            self._opened = cap.isOpened()
            return self._opened
        except Exception as e:            # noqa: BLE001
            self.error = str(e)
            return False

    def snapshot_jpeg(self):
        """يُعيد بايتات JPEG للقطة واحدة، أو None عند التعذّر."""
        if not _CV2_OK:
            return None
        with self._lock:
            if not self._ensure_open():
                return None
            ok, frame = self._cap.read()
            if not ok:
                return None
            ok, buf = cv2.imencode(".jpg", frame)
            if not ok:
                return None
            return buf.tobytes()

    def state(self) -> dict:
        # الصحة الحقيقية تُعرف بعد أول لقطة؛ قبلها نبلّغ توفّر المكتبة فقط
        return {"available": self.available, "opened": self._opened}

    def close(self) -> None:
        with self._lock:
            try:
                if self._cap:
                    self._cap.release()
            except Exception:             # noqa: BLE001
                pass
