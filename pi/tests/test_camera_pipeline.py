#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_camera_pipeline.py — تشخيص `CameraReader.snapshot_jpeg()` على العتاد
==========================================================================
يختبر **مسار الأنبوب الفعلي** الذي يستعمله التوثيق البصري، لا cv2 مباشرةً
(ذاك في `test_camera.py`). الفرق مهم: التوثيق يستدعي `snapshot_jpeg()`
ويعتمد على بايتات JPEG جاهزة للإرسال إلى Gemini.

يقيس ويطبع:
  • زمن الالتقاط لكل إطار (وأثر الإحماء — أول إطار غالباً أغمق أو أسود).
  • حجم البايتات والأبعاد **الفعلية** (قد تخالف CAMERA_W/H إن رفضها السائق).
  • **جودة الصورة**: أسوداء؟ مسطّحة؟ مشوّشة؟ مشبَعة؟ — بأرقام لا انطباع.
  • حجم الصورة بعد تجهيزها لـGemini (512px) ونسبة التوفير.

التشغيل على الراسبري:
    python3 -m pi.tests.test_camera_pipeline
    python3 -m pi.tests.test_camera_pipeline --frames 8 --save
    python3 -m pi.tests.test_camera_pipeline --index 1     # كاميرا أخرى
"""
import argparse
import io
import os
import sys
import time

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")))

from pi.config import CAMERA_INDEX, CAMERA_W, CAMERA_H, CAPTURES_DIR  # noqa: E402
from pi.sensors.camera import CameraReader                            # noqa: E402

try:
    import numpy as np
    _NP = True
except ImportError:
    _NP = False

try:
    from PIL import Image
    _PIL = True
except ImportError:
    _PIL = False


def analyze(jpeg: bytes) -> dict:
    """
    تشخيص الجودة بأرقام. المعايير وحدودها:
      • `mean`  < 12  ⇒ **صورة سوداء** (كاميرا لم تُحمَّ أو الغطاء مغلق).
      • `std`   < 6   ⇒ **مسطّحة بلا محتوى** (جدار فارغ أو تعرّض فاشل).
      • `sharp` (تباين لابلاس) < 60 ⇒ **مشوّشة/خارج البؤرة**.
      • `sat`   > 0.25 ⇒ ربع الصورة **مشبَع** (إضاءة زائدة تُفقد التفاصيل).
    الحدود إرشادية للتشخيص السريع لا عتبات قبول صارمة.
    """
    if not (_PIL and _NP):
        return {"ok": None, "note": "PIL/numpy غير متوفرة — لا تشخيص جودة"}
    img = Image.open(io.BytesIO(jpeg))
    w, h = img.size
    g = np.asarray(img.convert("L"), dtype=np.float32)
    # تباين لابلاس: مقياس حِدّة معياري (فرق كل بكسل عن جيرانه الأربعة)
    lap = (g[:-2, 1:-1] + g[2:, 1:-1] + g[1:-1, :-2] + g[1:-1, 2:]
           - 4.0 * g[1:-1, 1:-1])
    mean, std, sharp = float(g.mean()), float(g.std()), float(lap.var())
    sat = float((g > 250).mean())
    flags = []
    if mean < 12:
        flags.append("🔴 سوداء")
    if std < 6:
        flags.append("⚠ مسطّحة بلا محتوى")
    if sharp < 60:
        flags.append("⚠ مشوّشة/خارج البؤرة")
    if sat > 0.25:
        flags.append("⚠ مشبَعة (إضاءة زائدة)")
    return {"ok": not flags, "w": w, "h": h, "mean": mean, "std": std,
            "sharp": sharp, "sat_pct": 100.0 * sat, "flags": flags}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=CAMERA_INDEX)
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--save", action="store_true", help="احفظ آخر لقطة")
    a = ap.parse_args()

    print("=" * 64)
    print("  تشخيص مسار الكاميرا — CameraReader.snapshot_jpeg()")
    print("=" * 64)
    print(f"  المنفذ: index={a.index} · الإعداد المطلوب {CAMERA_W}x{CAMERA_H}")

    t0 = time.time()
    cam = CameraReader(index=a.index)
    print(f"  الفتح: {time.time() - t0:.2f}ث · available={cam.available}"
          + (f" · خطأ: {cam.error}" if cam.error else ""))
    if not cam.available:
        print("  ✗ opencv غير مثبّت — ثبّته:  sudo apt install python3-opencv")
        return 1

    print(f"\n  {'#':<3} {'زمن':>7} {'بايت':>9} {'الأبعاد':>11} "
          f"{'سطوع':>7} {'تباين':>7} {'حِدّة':>9}  الحالة")
    print("  " + "-" * 70)
    last, results = None, []
    for i in range(1, a.frames + 1):
        t = time.time()
        jpeg = cam.snapshot_jpeg()
        dt = time.time() - t
        if jpeg is None:
            print(f"  {i:<3} {dt:>6.2f}ث {'—':>9} {'—':>11}   🔴 فشل الالتقاط")
            results.append(None)
            continue
        last = jpeg
        q = analyze(jpeg)
        results.append(q)
        if q.get("ok") is None:
            print(f"  {i:<3} {dt:>6.2f}ث {len(jpeg):>9,} {'?':>11}   "
                  f"{q['note']}")
            continue
        state = "✅ واضحة" if q["ok"] else " ".join(q["flags"])
        print(f"  {i:<3} {dt:>6.2f}ث {len(jpeg):>9,} "
              f"{q['w']}x{q['h']:<6} {q['mean']:>7.1f} {q['std']:>7.1f} "
              f"{q['sharp']:>9.0f}  {state}")

    good = [r for r in results if r and r.get("ok")]
    print(f"\n  إطارات واضحة: {len(good)}/{a.frames}")
    if results and results[0] and not results[0].get("ok") and good:
        print("  ⚠ **أثر الإحماء**: أول إطار دون المستوى وما بعده سليم — "
              "التقط إطاراً أو إطارين تمهيديين قبل الاعتماد على اللقطة.")

    if last:
        ok_dim = (results[-1] or {}).get("w") == CAMERA_W
        print(f"  الأبعاد الفعلية تطابق الإعداد: "
              f"{'نعم' if ok_dim else '**لا** — السائق فرض مقاساً آخر'}")
        try:
            from pi.ai.vision_nav import prepare_image
            from pi.config import VISION_IMAGE_MAX_PX
            prep = prepare_image(last)
            print(f"  التجهيز لـGemini ({VISION_IMAGE_MAX_PX}px): "
                  f"{len(last):,} → {len(prep):,} بايت "
                  f"(توفير {100 * (1 - len(prep) / max(len(last), 1)):.0f}%)")
        except Exception as e:                     # noqa: BLE001
            print(f"  تعذّر التجهيز: {e}")

        if a.save:
            os.makedirs(CAPTURES_DIR, exist_ok=True)
            p = os.path.join(CAPTURES_DIR,
                             time.strftime("cam_pipeline_%Y%m%d_%H%M%S.jpg"))
            open(p, "wb").write(last)
            print(f"  حُفظت: {p}")

    cam.close()
    return 0 if good else 2


if __name__ == "__main__":
    sys.exit(main())
