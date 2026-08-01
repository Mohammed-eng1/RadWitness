# -*- coding: utf-8 -*-
"""
camera.py — قارئ الويب كام (منقول من pi/tests/test_camera.py المثبت)
====================================================================
cv2.VideoCapture(0). يُفتح كسولاً عند أول طلب. آمن: غياب opencv/الكاميرا لا
يُسقط السيرفر.

⚠ سياسة الخصوصية: **لا يُكتب أي إطار على القرص تلقائياً**. اللقطة والبث
   يُرسلان إلى المتصفح كبايتات فقط. الحفظ يتم حصراً عبر save_snapshot()
   باستدعاء صريح من زر «حفظ» (ولاحقاً لقطة الشذوذ في M2).
"""
import os
import threading
import time

from pi.config import CAMERA_INDEX, CAMERA_W, CAMERA_H, CAMERA_STREAM_FPS

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

    def frame_array(self):
        """
        إطار خام كمصفوفة numpy (H, W, 3) أو None — للاتجاه البصري.
        ⚠ **بلا ترميز JPEG**: `snapshot_jpeg` يرمّز ثم يفكّ الترميز عند
           الاستعمال، وهو ضياع وقت وجودة في مسار يُقرأ عند كل خلية.
        ⚠ يشارك نفس القفل: الكاميرا تُقرأ من خيط البثّ وخيط المحركات معاً.
        """
        if not _CV2_OK:
            return None
        with self._lock:
            if not self._ensure_open():
                return None
            ok, frame = self._cap.read()
            return frame if ok else None

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

    def mjpeg_frames(self):
        """مولّد إطارات multipart للبث الحي (لا يُكتب شيء على القرص)."""
        interval = 1.0 / max(1, CAMERA_STREAM_FPS)
        while True:
            data = self.snapshot_jpeg()
            if data is None:
                break                     # الكاميرا غير متاحة → أنهِ البث
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n"
                   + data + b"\r\n")
            time.sleep(interval)

    def save_snapshot(self, directory: str):
        """يحفظ لقطة على القرص — **بطلب صريح فقط**. يُعيد اسم الملف أو None."""
        data = self.snapshot_jpeg()
        if data is None:
            return None
        try:
            os.makedirs(directory, exist_ok=True)
            name = "snap_" + time.strftime("%Y%m%d_%H%M%S") + ".jpg"
            with open(os.path.join(directory, name), "wb") as f:
                f.write(data)
            return name
        except Exception as e:            # noqa: BLE001
            self.error = str(e)
            return None

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
