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

from pi.config import (CAMERA_INDEX, CAMERA_W, CAMERA_H, CAMERA_STREAM_FPS,
                       CAMERA_SNAPSHOT_TIMEOUT_S, CAMERA_CLOSE_LOCK_TIMEOUT_S)

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
        self._worker = None           # خيط اللقطة (قد يعلق على جهاز معطوب)
        self.timeouts = 0             # مرات تجاوز مهلة اللقطة — تظهر في state()
        self._gen = 0                 # جيل المقبض: يمنع عاملاً مهجوراً من
                                      # تحرير مقبض فُتح **بعده**
        self.abandoned = 0            # مقابض هُجرت بلا تحرير (خيط عالق يملكها)

    def _ensure_open(self) -> bool:
        """
        🔴 **يتخلّص من المقبض الميت ويعيد المحاولة** — لا يعود False للأبد.

        عطل مقاس (2026-08-10): كان الشرط `if self._cap is not None: return
        self._cap.isOpened()` بلا تنظيف، فأي مقبض محرَّر أو جهاز اختفى
        لحظةً يقفل الكاميرا **لبقية عمر العملية**: كل لقطة تالية `None`،
        والتوثيق يخرج بصفر صور فيبدو عطلاً في التوثيق لا في الجهاز.
        وأشيع مسبّب: فشل فتح عابر عند الإقلاع (الجهاز مشغول أو لم يُعدّ بعد).
        """
        if self._cap is not None:
            try:
                if self._cap.isOpened():
                    return True
            except Exception:             # noqa: BLE001
                pass
            self._drop_handle()           # مقبض ميت — تخلّص وأعد الفتح
        try:
            cap = cv2.VideoCapture(self._index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_H)
            self._cap = cap
            self._opened = cap.isOpened()
            if not self._opened:
                self._drop_handle()       # لا تحتفظ بمقبض فاشل يمنع المحاولة
                self.error = f"تعذّر فتح الكاميرا (index={self._index})"
            return self._opened
        except Exception as e:            # noqa: BLE001
            self.error = str(e)
            self._drop_handle()
            return False

    def _drop_handle(self) -> None:
        """يحرّر المقبض **ويُصفّر الحالة** — التصفير جزء من التحرير لا زينة."""
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:                 # noqa: BLE001
            pass
        finally:
            self._cap = None
            self._opened = False

    def _snapshot_blocking(self):
        """اللقطة الفعلية — قد **تحجب** على جهاز معطوب (انظر `snapshot_jpeg`)."""
        with self._lock:
            gen = self._gen
            if not self._ensure_open():
                return None
            cap = self._cap
            ok, frame = cap.read()
            # ⚠ هُجرنا أثناء الحجب: نحرّر **مقبضنا نحن** ولا نلمس الجديد.
            #    (تحرير مقبض خيطٍ آخر داخل `read()` عليه انهيار في libc لا
            #     استثناء بايثون.)
            if gen != self._gen:
                try:
                    cap.release()
                except Exception:         # noqa: BLE001
                    pass
                return None
            if not ok:
                self._drop_handle()       # قراءة فاشلة = مقبض مشبوه
                return None
            ok, buf = cv2.imencode(".jpg", frame)
            return buf.tobytes() if ok else None

    def snapshot_jpeg(self, timeout_s: float = CAMERA_SNAPSHOT_TIMEOUT_S):
        """
        🔴 لقطة **بمهلة صلبة** — `cv2` يحجب بلا نهاية على جهاز معطوب.

        عطل مقاس (2026-08-10): المهمة بلغت «المرحلة ٦ توثيق» ثم تجمّدت
        دقائق بلا سطر سجل واحد ولا صورة. و`VideoCapture()`/`read()` نداءان
        إلى V4L2 **بلا أي مهلة**: جهاز في حالة سيئة (أو مشغول) يوقفهما إلى
        الأبد، فيتجمّد **خيط المهمة كله** — والروبوت واقف بلا تفسير.

        العلاج: تنفيذ اللقطة في خيط عامل ومهلة عليه. الخيط العالق يبقى
        عالقاً (لا يمكن قتل نداء C) لكن **المهمة تمضي**، ونمنع تكديس
        الخيوط: ما دام عاملٌ سابق عالقاً لا نُطلق آخر ونُعلن السبب.
        ⚠ الفشل يُعلَن ولا يُبتلع: `timeouts` يتصاعد ويظهر في `state()`.
        """
        if not _CV2_OK:
            return None
        if not timeout_s or timeout_s <= 0:
            return self._snapshot_blocking()
        w = self._worker
        if w is not None and w.is_alive():
            self.error = ("لقطة سابقة ما زالت عالقة — الجهاز لا يستجيب "
                          "(أعد تشغيل السيرفر أو افحص /dev/video0)")
            return None
        box = {}

        def _work():
            try:
                box["data"] = self._snapshot_blocking()
            except Exception as e:        # noqa: BLE001
                box["err"] = str(e)

        w = threading.Thread(target=_work, daemon=True)
        self._worker = w
        w.start()
        w.join(float(timeout_s))
        if w.is_alive():
            self.timeouts += 1
            self.error = (f"تعذّرت اللقطة خلال {timeout_s:.0f}ث — الجهاز "
                          f"لا يستجيب (تعليق V4L2)")
            return None
        if "err" in box:
            self.error = box["err"]
            return None
        return box.get("data")

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
        # الصحة الحقيقية تُعرف بعد أول لقطة؛ قبلها نبلّغ توفّر المكتبة فقط.
        # ⚠ تُقرأ من المقبض لا من راية محفوظة: الراية القديمة كانت تُبقي
        #   نقطة الصحة **خضراء على كاميرا محرَّرة**، فيبدو العطل في التوثيق.
        opened = False
        try:
            opened = bool(self._cap is not None and self._cap.isOpened())
        except Exception:                 # noqa: BLE001
            opened = False
        self._opened = opened
        return {"available": self.available, "opened": opened,
                "timeouts": self.timeouts, "abandoned": self.abandoned,
                "stuck": bool(self._worker is not None
                              and self._worker.is_alive()),
                "error": self.error}

    def close(self) -> None:
        """
        🔴 التحرير **يُصفّر الحالة** — وإلا ماتت الكاميرا لبقية عمر العملية.

        عطل مقاس (2026-08-10) وهو سبب غياب الصور فعلياً: `ModeManager` ينادي
        `close()` عند **مغادرة نمط القيادة اليدوية** — أي عند كل انتقال
        «يدوي → ذاتي»، وهو بالضبط ما يسبق كل مهمة. وبلا تصفير `_cap` كان
        `_ensure_open` يعود `isOpened()=False` أبداً، فيخرج التوثيق بصفر
        صورة ويبدو العطل في التوثيق لا في الجهاز.

        🔴 **ولا ينتظر القفل بلا مهلة**: العامل المهجور (انظر `snapshot_jpeg`)
        يبقى ممسكاً بالقفل بحكم التصميم، و`close()` يُنادى من **حلقة
        asyncio** (تبديل النمط عبر `/api/mode` وإغلاق السيرفر). فانتظار بلا
        مهلة ينقل التجمّد من خيط المهمة إلى الواجهة كلها: لا تيليمتري ولا
        زرّ إيقاف — بينما خيط المهمة يواصل قيادة المحركات. نهجر ونُعلن.
        """
        if not self._lock.acquire(timeout=CAMERA_CLOSE_LOCK_TIMEOUT_S):
            # ⚠ ممنوع `release()` هنا: المقبض ملك خيطٍ عالق داخل `read()`.
            #    نرفع الجيل فيتخلّص هو منه إن استفاق، ونُسرّبه **معلَناً**.
            self._gen += 1
            self._cap = None
            self._opened = False
            self.abandoned += 1
            self.error = ("مقبض الكاميرا هُجر — لقطة عالقة تملك القفل "
                          "(أعد تشغيل السيرفر أو افحص /dev/video0)")
            return
        try:
            self._drop_handle()
            self._gen += 1
        finally:
            self._lock.release()
