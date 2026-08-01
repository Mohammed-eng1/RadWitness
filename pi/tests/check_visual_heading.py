#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_visual_heading.py — معايرة وفحص الاتجاه البصري (الويب كام)
================================================================
الكاميرا مرجع اتجاه **مطلق** يكسر تراكم انحراف الجايرو (انظر
`pi/sensors/visual_heading.py`). لكنها تحتاج رقمين **مقاسين لا مخمَّنين**:

    CAMERA_HFOV_DEG   مجال الرؤية الأفقي بالدرجات
    VISUAL_YAW_SIGN   اتجاه انزلاق المشهد عند الدوران يميناً

كلاهما يُضرب في إزاحة البكسل، فرقم مفترض خاطئ يُنتج تصحيحاً خاطئاً **واثقاً**
— أسوأ من غياب التصحيح أصلاً (نفس درس `TURN_COAST_TAU_S`).

المراحل:
  1. **جودة المشهد** (بلا حركة): هل فيه معالم كافية للمطابقة أصلاً؟
  2. **المعايرة** (⚠ يلفّ الروبوت): يلفّ زاوية معلومة بالجايرو ويجمع إزاحة
     البكسل تراكمياً → درجة/بكسل → HFOV، والإشارة من اتجاه الإزاحة.
  3. **التحقّق** (⚠ يلفّ): لفّة ثانية — كم تقول الكاميرا وكم يقول الجايرو؟

التشغيل على الراسبري (⚠ أوقف السيرفر أولاً — الكاميرا والمنفذ مشتركان):
    python3 -m pi.tests.check_visual_heading              # المرحلة 1 فقط
    python3 -m pi.tests.check_visual_heading --calibrate  # ⚠ يحرّك الروبوت

⚠ اجعل أمام الروبوت **مشهداً ثابتاً وفيه تفاصيل** (رفّ، باب، أثاث). جدار
   أبيض أملس لا يعطي شيئاً — وهذا نفسه ما تقيسه المرحلة 1.
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from pi.config import (
    ROVER_MODE, VISUAL_MIN_CONTRAST, VISUAL_MIN_CORR, VISUAL_MIN_MARGIN,
    CAMERA_HFOV_DEG, VISUAL_YAW_SIGN, TURN_POWER,
)
from pi.sensors.camera import CameraReader
from pi.sensors.heading import PHASES
from pi.sensors.visual_heading import (
    column_signature, signature_contrast, match_shift,
)

CALIB_TURN_DEG = 40.0       # زاوية المعايرة: كبيرة للدقة، ودون مجال الرؤية
SAMPLE_GAP_S = 0.05


def diagnose_camera(cam) -> None:
    """
    «لا إطار» ثلاثة أعطال مختلفة تماماً — تسميتها بالاسم توفّر جولة تخمين:
      • cv2 غير مستورَد   → شُغّل خارج البيئة الافتراضية (الأشيع بفارق كبير).
      • لا جهاز /dev/video* → الكاميرا غير موصولة.
      • الجهاز موجود ولا يفتح → عملية أخرى تمسكه (السيرفر، mjpg-streamer).
    """
    import glob
    import os
    print("\n⛔ الكاميرا لا تعطي إطاراً — التشخيص:")
    st = cam.state()
    if not st.get("available"):
        print(f"  ⛔ مكتبة opencv غير متاحة لهذا المفسّر ({cam.error})")
        print(f"     المفسّر الحالي: {sys.executable}")
        if "venv" not in sys.executable:
            print("     ⇒ **أنت خارج البيئة الافتراضية**. شغّل أولاً:")
            print("         source venv/bin/activate")
        else:
            print("     ⇒ pip install opencv-python-headless")
        return
    devs = sorted(glob.glob("/dev/video*"))
    if not devs:
        print("  ⛔ لا يوجد أي /dev/video* — الكاميرا غير موصولة أو لم تُعرَّف.")
        print("     افحص: lsusb   ثم أعد توصيل الكابل.")
        return
    print(f"  الأجهزة الموجودة: {' '.join(devs)}")
    print("  ⇒ الجهاز موجود ولا يُفتح: **عملية أخرى تمسكه**.")
    print("     أوقف السيرفر، وتحقّق من mjpg-streamer:")
    print("         pkill -f 'pi.web.server' ; pkill -f mjpg_streamer")
    print("     ولمعرفة مَن يمسكه:  sudo fuser -v /dev/video0")
    if os.path.exists("/dev/video0"):
        print("     (⚠ بعض الويب كامات تُظهر عدّة أجهزة وأولها فقط هو الصورة)")


def scene_quality(cam, seconds: float = 3.0) -> dict:
    """جودة المشهد بلا حركة: تباين التوقيع + ثبات المطابقة بين لقطتين."""
    print(f"\n── 1) جودة المشهد ({seconds:.0f}ث، الروبوت ساكن) ──")
    sigs, contrasts = [], []
    deadline = time.time() + seconds
    while time.time() < deadline:
        f = cam.frame_array()
        if f is not None:
            s = column_signature(f)
            sigs.append(s)
            contrasts.append(signature_contrast(s))
        time.sleep(0.1)
    if len(sigs) < 3:
        print("  ⛔ لا لقطات — الكاميرا لا تقرأ (هل السيرفر يمسكها؟)")
        return {"ok": False}
    c = float(np.median(contrasts))
    m = match_shift(sigs[0], sigs[-1])
    drift = abs(m.get("shift_px", 0.0)) if m.get("ok") else float("nan")
    print(f"  لقطات: {len(sigs)} · عرض الصورة: {len(sigs[0])} بكسل")
    print(f"  التباين (وسيط): {c:.1f}  "
          + ("✅" if c >= VISUAL_MIN_CONTRAST else
             f"⛔ دون {VISUAL_MIN_CONTRAST} — المشهد بلا معالم"))
    if m.get("ok"):
        lo, hi = m["lobe"]
        print(f"  ثبات المطابقة والروبوت ساكن: إزاحة {drift:.2f} بكسل · "
              f"ارتباط {m['peak']:.2f} · حدّة {m['margin']:.2f}")
        print(f"  عرض الفصّ الرئيسي: {hi - lo + 1} خطوة · "
              f"أقرب ذروة منافسة عند إزاحة {m.get('rival_shift_px')} بكسل")
        if m["margin"] < VISUAL_MIN_MARGIN:
            print(f"  ⚠ حدّة الذروة دون {VISUAL_MIN_MARGIN} — توجد ذروة "
                  f"منافسة بنفس القوة تقريباً عند إزاحة مختلفة: نمط متكرّر "
                  f"(بلاط/ستائر/أرفف)؟ وجّه الكاميرا إلى مشهد أقلّ دورية.")
        if drift > 1.5:
            print("  ⚠ إزاحة ملموسة والروبوت **ساكن**: اهتزاز أو تغيّر إضاءة "
                  "أو حركة في المشهد — سيُترجَم لاحقاً إلى دوران وهمي.")
    ok = (c >= VISUAL_MIN_CONTRAST and m.get("ok")
          and m["peak"] >= VISUAL_MIN_CORR and m["margin"] >= VISUAL_MIN_MARGIN)
    print("  " + ("✅ المشهد صالح للاتجاه البصري" if ok else
                  "⛔ المشهد غير صالح — عالج ما سبق قبل المعايرة"))
    return {"ok": ok, "contrast": c, "width": len(sigs[0])}


def _turn_and_track(rover, cam, degrees: float) -> dict:
    """
    يلفّ بالجايرو ويجمع إزاحة البكسل **تراكمياً بين اللقطات المتتابعة**.

    ⚠ التراكم بين لقطات متقاربة لا بمقارنة الأولى بالأخيرة: لفّة 40° تُخرج
       المعالم من مجال الرؤية فتفشل المطابقة المباشرة تماماً. الخطوات
       الصغيرة تُبقي تداخلاً كافياً في كل خطوة.
    """
    total_px = 0.0
    weak = 0
    prev = None
    f = cam.frame_array()
    if f is not None:
        prev = column_signature(f)

    import threading
    res = {}
    th = threading.Thread(
        target=lambda: res.update(rover.turn_by_angle(degrees)), daemon=True)
    th.start()
    while th.is_alive():
        f = cam.frame_array()
        if f is not None:
            sig = column_signature(f)
            if prev is not None:
                m = match_shift(prev, sig)
                if m.get("ok") and m["peak"] >= VISUAL_MIN_CORR \
                        and m["margin"] >= VISUAL_MIN_MARGIN:
                    total_px += m["shift_px"]
                else:
                    weak += 1
            prev = sig
        time.sleep(SAMPLE_GAP_S)
    th.join(timeout=2.0)

    # ⚠ اللقطات بعد توقف المحركات مقصودة: القصور الذاتي يضيف 5–8° بعد قطع
    #    الطاقة (CLAUDE.md §1.1.1)، والكاميرا تراه كما يراه الجايرو.
    settle = time.time() + 0.6
    while time.time() < settle:
        f = cam.frame_array()
        if f is not None:
            sig = column_signature(f)
            if prev is not None:
                m = match_shift(prev, sig)
                if m.get("ok") and m["peak"] >= VISUAL_MIN_CORR:
                    total_px += m["shift_px"]
            prev = sig
        time.sleep(SAMPLE_GAP_S)
    return {"px": total_px, "weak": weak, "turn": res}


def motor_probe(rover, seconds: float = 1.2) -> dict:
    """
    فحص فاصل **قبل أي معايرة**: هل تصل الأوامر وهل يدور الروبوت؟

    ⚠ «الجايرو 0° والكاميرا 0 بكسل» تصف العرَض لا السبب. ثلاثة أسباب مختلفة
       تُنتجها كلها: (أ) البرمجية رفضت الحركة أصلاً (مصدر اتجاه معطّل)، (ب)
       الأوامر لا تصل الفيرموير (منفذ/أسلاك)، (ج) تصل ولا يدور (بطارية/عائق).
       هذا الفحص **بحلقة مفتوحة بلا أي شرط برمجي** يفصل الثلاثة.
    """
    src = rover.heading_source
    print(f"\n── فحص فاصل: أمر لفّ خام {seconds:.1f}ث ⚠ يتحرّك ──")
    print(f"  مصدر الاتجاه: {src.name} · سليم={src.ok} · "
          f"انحياز مُعاير={src.bias_calibrated}"
          + (f" · ⛔ {src.error}" if src.error else ""))
    print(f"  وصلة الروفر: mode={rover.mode} · link_ok={rover.link_ok}"
          + (f" · ⛔ {rover.link_error}" if rover.link_error else ""))
    src.set_phase("turn")
    spikes0 = src.cond.spikes
    peak = 0.0
    total = 0.0
    sent = (0.0, 0.0)
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            rover.turn("R", TURN_POWER)       # أمر خام متجدّد (heartbeat)
            sent = rover._cmd_lr               # ⚠ تُلتقط **قبل** stop() وإلا
            d = src.update()                   #    طبعنا أمر الإيقاف (0,0)
            peak = max(peak, abs(d.get("dps", 0.0)))
            total += d.get("delta", 0.0)
            time.sleep(0.02)
    finally:
        rover.stop()
        src.set_phase("drive")
    spikes = src.cond.spikes - spikes0
    print(f"  أُرسل L={sent[0]:+.2f} R={sent[1]:+.2f} · "
          f"ذروة الدوران {peak:.1f}°/ث · تراكم {total:+.1f}°")
    # ⚠ الحارس الذي كشف العطل: ذروة تلامس العتبة تعني قراءات تُرمى صامتةً
    thr = PHASES["turn"][0]
    if spikes or peak > 0.8 * thr:
        print(f"  ⚠ عتبة رفض القفزة {thr:.0f}°/ث والذروة {peak:.0f}°/ث "
              f"({100.0 * peak / thr:.0f}%) · رُفضت {spikes} قراءة")
        print("     فوق العتبة يُعيد المرشّح صفراً لا القراءة ⇒ يتجمّد التكامل "
              "في أسرع لحظة من اللفّة. ارفع HEADING_SPIKE_DPS_TURN.")
    ok = peak > 5.0
    if not ok:
        print("  ⛔ **لم يدر الروبوت بأمر خام**. الأوامر تُرسل (انظر L/R أعلاه)")
        print("     والجايرو يقرأ صفراً ⇒ إمّا لا يتحرّك فعلياً (بطارية/عائق/"
              "أسلاك محرّك) وإمّا لا يصل الأمر إلى الفيرموير.")
        print("     افحص: هل سمعتَ المحركات؟ هل تحرّك الروبوت أصلاً؟")
        print("     وللفصل: python3 -m pi.tests.check_directions")
    else:
        print("  ✅ الروبوت يدور بأمر خام — المشكلة ليست في العتاد.")
    return {"ok": ok, "peak": peak, "total": total}


def calibrate(rover, cam, width: int) -> None:
    print(f"\n── 2) المعايرة: لفّة {CALIB_TURN_DEG:.0f}° ⚠ الروبوت يتحرّك ──")
    t_start = time.time()
    r = _turn_and_track(rover, cam, CALIB_TURN_DEG)
    t = r["turn"]
    gyro = float(t.get("turned_deg", 0.0))
    px = r["px"]
    print(f"  الجايرو: {gyro:+.1f}° (طُلب {CALIB_TURN_DEG:+.0f}°) · "
          f"الكاميرا: {px:+.1f} بكسل تراكمياً · مطابقات ضعيفة: {r['weak']}")
    # ⚠ **الحصيلة كاملة**: «دار 0°» وحدها لا تفرّق بين إجهاض برمجي قبل أي
    #    حركة وبين محركات دارت ولم تُنتج دوراناً. الفرق كله في هذه الحقول.
    print(f"  الحصيلة: أُجهض={t.get('aborted')} · مهلة={t.get('timed_out')} · "
          f"لا دوران={t.get('no_rotation')} · ذروة={t.get('peak_rate_dps')}°/ث · "
          f"قفزات مرفوضة={t.get('spikes')} · تصحيحات={t.get('corrections')} · "
          f"قصور={t.get('coast_deg')}° · τ={t.get('coast_tau_s')}ث · "
          f"مراحل={t.get('segments')} · استغرق {time.time() - t_start:.1f}ث")
    if t.get("spikes"):
        print("  ⚠ قراءات رُفضت كقفزة ⇒ الزاوية المقروءة **أقلّ من الحقيقية**، "
              "فالمعايرة أدناه مبنية على مرجع ناقص. ارفع HEADING_SPIKE_DPS_TURN "
              "وأعد التجربة قبل اعتماد الرقم.")
    if t.get("aborted"):
        print(f"  ⛔ اللفّة **أُجهضت برمجياً قبل الحركة**: {t['aborted']}")
        print(f"     مصدر الاتجاه: {rover.heading_source.error}")
        return
    if abs(gyro) < 10.0:
        print("  ⛔ الروبوت لم يلفّ فعلياً — لا معايرة. (بطارية؟ عائق؟)")
        return
    if abs(px) < 20.0:
        print("  ⛔ الكاميرا لم تر دوراناً — تحقّق من تثبيتها ومن المشهد.")
        return

    deg_per_px = abs(gyro) / abs(px)
    hfov = deg_per_px * width
    sign = -1 if (px * gyro) < 0 else +1
    print(f"\n  درجة/بكسل = {deg_per_px:.4f}")
    print(f"  ➜ CAMERA_HFOV_DEG = {hfov:.1f}")
    print(f"  ➜ VISUAL_YAW_SIGN = {sign:+d}")
    if not (30.0 <= hfov <= 120.0):
        print(f"  ⚠ {hfov:.0f}° خارج المدى المعقول لويب كام (30–120°) — "
              f"الأرجح أن اللفّة أو المطابقة غير موثوقة. أعد التجربة.")
        return

    print(f"\n── 3) التحقّق: لفّة ثانية {-CALIB_TURN_DEG:.0f}° ⚠ يتحرّك ──")
    r2 = _turn_and_track(rover, cam, -CALIB_TURN_DEG)
    g2 = float(r2["turn"].get("turned_deg", 0.0))
    cam_deg = sign * r2["px"] * deg_per_px
    err = cam_deg - g2
    print(f"  الجايرو: {g2:+.1f}°  ·  الكاميرا: {cam_deg:+.1f}°  ·  "
          f"الفرق: {err:+.1f}°")
    print(f"  (مطابقات ضعيفة: {r2['weak']} · قفزات مرفوضة: "
          f"{r2['turn'].get('spikes')} · ذروة {r2['turn'].get('peak_rate_dps')}°/ث)")
    if r2["weak"] > 2:
        print("  ⚠ مطابقات ضعيفة كثيرة: الدوران أسرع ممّا تلحقه الكاميرا "
              "(ضبابية حركة + إزاحة ضخمة بين لقطتين). أعد بـ--power أقلّ.")
    if abs(err) <= 4.0:
        print("  ✅ المصدران متفقان — القياس موثوق.")
    else:
        print("  ⚠ فرق كبير: انزلاق العجلات (الجايرو أدقّ للدوران) أو مشهد "
              "قريب جداً (تزيّح المنظر). أعد التجربة أمام مشهد أبعد.")

    print("\n" + "═" * 58)
    print("ضع في pi/config.py ثم فعّل VISUAL_HEADING_ENABLED = True:")
    print(f"    CAMERA_HFOV_DEG  = {hfov:.1f}")
    print(f"    VISUAL_YAW_SIGN  = {sign:+d}")
    print("═" * 58)


def main() -> int:
    ap = argparse.ArgumentParser(description="معايرة الاتجاه البصري")
    ap.add_argument("--calibrate", action="store_true",
                    help="⚠ يلفّ الروبوت لقياس HFOV والإشارة")
    args = ap.parse_args()

    cam = CameraReader()
    if cam.frame_array() is None:
        diagnose_camera(cam)
        return 1
    print(f"الحالي في config: CAMERA_HFOV_DEG={CAMERA_HFOV_DEG} · "
          f"VISUAL_YAW_SIGN={VISUAL_YAW_SIGN:+d}"
          + ("  (غير مقاس بعد)" if not CAMERA_HFOV_DEG else ""))

    q = scene_quality(cam)
    if not args.calibrate:
        print("\nأضف --calibrate لقياس HFOV والإشارة (⚠ يحرّك الروبوت).")
        return 0 if q.get("ok") else 1
    if not q.get("ok"):
        print("\n⛔ لن أعاير على مشهد غير صالح — النتيجة ستكون رقماً واثقاً "
              "وخاطئاً، وهو أسوأ من لا شيء.")
        return 1

    from pi.rover.bridge import WaveRoverBridge
    rover = WaveRoverBridge(mode=ROVER_MODE)
    if rover.mode != "real":
        print(f"\n⚠ الجسر في وضع {rover.mode} — لن تتحرّك المحركات. "
              f"شغّل بـ RMS_ROVER_MODE=real")
    try:
        info = rover.calibrate_gyro_bias()
        bi = rover.bias_info
        print(f"\n  انحياز الجايرو: {info:+.4f} (σ={bi.get('std', 0):.3f}، "
              f"{bi.get('n', 0)} عينة)"
              + ("" if bi.get("ok") else f" ⛔ رُفض: {bi.get('reason')}"))
        # الفحص الفاصل قبل المعايرة: عطل عتاد يُنتج نفس أرقام عطل البرمجية
        if not motor_probe(rover)["ok"]:
            print("\n⛔ لا معنى لمعايرة بصرية والروبوت لا يدور — عالج ما سبق.")
            return 1
        calibrate(rover, cam, q["width"])
    finally:
        rover.stop()
        cam.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nتوقّف.")
