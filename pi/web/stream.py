# -*- coding: utf-8 -*-
"""
stream.py — بثّ MJPEG بدقة قابلة للاختيار (طبقة الواجهة)
=========================================================
البثّ **عند الطلب فقط**: يبدأ بدخول نمط القيادة اليدوية ويتوقف بالخروج
منه — كاميرا USB مفتوحة تستهلك المعالج والناقل حتى بلا مشاهد، وعلى
راسبري 2GB يُحسب ذلك.

────────────────────────────────────────────────────────────────────
## ⚠ لماذا يُعاد الترميز هنا بدل ضبط الكاميرا؟

`CameraReader` يثبّت الدقة عند فتح الجهاز من `CAMERA_W/CAMERA_H`، وتغييرها
فعلياً يحتاج تعديل `pi/sensors/camera.py` — **خارج نطاق هذا المسار**.
فالتصغير يتم هنا: فكّ ترميز الإطار وتصغيره وإعادة ترميزه.

**ما يكسبه هذا وما يكلّفه — بلا تجميل**:
- ✅ يقلّل **النطاق** فعلاً (إطار 320×240 أصغر كثيراً من 640×480).
- ❌ لا يقلّل حمل **الالتقاط** (الكاميرا ما زالت تلتقط بالدقة الكاملة)،
  ويضيف فكّ ترميز + تصغير + ترميز لكل إطار.

⇒ مكسب صافٍ على وصلة بطيئة (الهدف الأصلي)، وخسارة معالج على وصلة سريعة.
ولهذا `640×480` هو الافتراضي و`320×240` خيار صريح للوصلات الضعيفة.
**الحلّ الأنظف** ضبط دقة الالتقاط نفسها — يحتاج `set_resolution()` في
`camera.py` (مطلوب من مسار الحساسات، وعندها يصير هذا الملف تمريراً بلا
إعادة ترميز).

## 🔴 غياب الكاميرا لا يُسقط شيئاً
لا انهيار ولا صفحة بيضاء: البثّ ينتهي بلا إطارات وتعرض الواجهة صورة
بديلة برسالة السبب.
"""
from __future__ import annotations

import time

from pi.config import CAMERA_W, CAMERA_H, CAMERA_STREAM_FPS

# ⚠ استيراد كسول خلف try/except — يعمل على ويندوز بلا opencv
try:
    import cv2                                       # type: ignore
    import numpy as np                               # type: ignore
    _CV2_OK = True
    _CV2_ERR = None
except ImportError as _e:                            # pragma: no cover
    cv2 = None
    np = None
    _CV2_OK = False
    _CV2_ERR = f"opencv غير مثبّت ({_e})"

#: الدقّات المتاحة — الافتراضي أولاً.
RESOLUTIONS = {
    "640x480": (640, 480),
    "320x240": (320, 240),
}
DEFAULT_RES = f"{CAMERA_W}x{CAMERA_H}" if f"{CAMERA_W}x{CAMERA_H}" in RESOLUTIONS \
    else "640x480"


def resolution_options() -> dict:
    """الدقّات المعروضة في الواجهة + سبب تعذّر التصغير إن وُجد."""
    return {
        "options": list(RESOLUTIONS.keys()),
        "default": DEFAULT_RES,
        "native": f"{CAMERA_W}x{CAMERA_H}",
        "resize_ok": _CV2_OK,
        # لا فشل صامت: بلا opencv يُبثّ بالدقة الأصلية ويُقال ذلك صراحةً
        "note": (None if _CV2_OK else
                 f"{_CV2_ERR} — يُبثّ بالدقة الأصلية بلا تصغير"),
    }


def _resize_jpeg(data: bytes, size) -> bytes:
    """
    يصغّر إطار JPEG. عند أي تعذّر **يُعيد الأصل** لا None: إطار أكبر من
    المطلوب أفضل من بثّ ينقطع.
    """
    if not _CV2_OK or size is None:
        return data
    try:
        arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return data
        h, w = img.shape[:2]
        if (w, h) == size:
            return data
        small = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", small)
        return buf.tobytes() if ok else data
    except Exception:                                # noqa: BLE001
        return data


#: إطارات مفقودة متتالية قبل إعلان الكاميرا غائبة وإنهاء البثّ.
#  ⚠ **لا يُنهى البثّ من أول إطار مفقود**: `snapshot_jpeg` يُعيد `None` حين
#  تكون لقطة أخرى جارية (حارس تكديس الخيوط) — ومع وجود مستهلك ثانٍ
#  (مسجّل المهمة) صار هذا حدثاً عادياً كل بضع ثوانٍ. الإنهاء الفوري كان
#  يقتل البثّ لأتفه سبب ويُظهره «معطّلاً» وهو سليم.
STREAM_MAX_MISSES = 24                               # ~2ث عند 12 إطار/ث


def mjpeg_frames(camera, res: str = None, is_active=None, pause_fn=None):
    """
    مولّد إطارات multipart.

    `is_active`: تُسأل قبل كل إطار — **البثّ يتوقف فور انتفائها** ولا ينتظر
                 انقطاع المتصفح.
    `pause_fn` : تُسأل قبل كل إطار — `True` ⇒ **توقّف مؤقت بلا إنهاء**: لا
                 تُلمس الكاميرا ولا يُقطع الاتصال. تُستعمل لمرحلة التوثيق:
                 الكاميرا هناك ملك التوثيق وحده (لقطة ثانية أثناء لقطة
                 جارية تُرفض، فالمزاحمة تُفقد **صور التقرير** نفسها).
    """
    size = RESOLUTIONS.get(res or DEFAULT_RES)
    interval = 1.0 / max(1, CAMERA_STREAM_FPS)
    misses = 0
    while True:
        if is_active is not None and not is_active():
            break                                    # انتفت الصلاحية ⇒ أنهِ
        if pause_fn is not None and pause_fn():
            time.sleep(interval)                     # توقّف مؤقت لا إنهاء
            continue
        data = camera.snapshot_jpeg()
        if data is None:
            misses += 1
            if misses >= STREAM_MAX_MISSES:
                break                                # الكاميرا غائبة فعلاً
            time.sleep(interval)
            continue
        misses = 0
        data = _resize_jpeg(data, size)
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n"
               + data + b"\r\n")
        time.sleep(interval)
