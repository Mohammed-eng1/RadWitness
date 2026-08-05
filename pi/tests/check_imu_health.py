#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_imu_health.py — لماذا يقول السجل «الحسّاس ميت» وأنا أراه شغّالاً؟
=======================================================================
يجيب على السؤال بقراءة **الشريحة نفسها** لا بالاستنتاج من بياناتها.

المشكلة التي كُتب لها: مصدر الاتجاه أعلن
    «40 قراءة صفر مضبوط متتابعة — الحسّاس لا يرسل شيئاً (ميت؟)»
بينما BNO055 يعمل ويُقرأ بلا شكوى قبل بدء المهمة. السبب المرشّح الأول:

  ⚠ في وضع **CONFIG** تقرأ كل سجلات بيانات BNO055 **0x00**، والشريحة تعود
    إلى CONFIG **وحدها** بعد أي إعادة تشغيل ذاتية — وأشيع مسبّب لها هبوط
    جهد لحظي على خط 3.3V عند إقلاع أربعة محركات. فالشريحة حيّة على الناقل
    وتردّ بهويتها، والجايرو صفر مضبوط إلى الأبد. «ميتة» و«عادت إلى CONFIG»
    تبدوان متطابقتين من فوق، وعلاجهما مختلف تماماً.

المراحل:
  1. حالة الشريحة الآن: CHIP_ID / OPR_MODE / SYS_STAT / SYS_ERR + المعايرة.
  2. مراقبة gz والمحركات **مطفأة** — أرضية مرجعية (يجب ألا تكون كلها صفراً).
  3. (اختيارية، تحرّك الروبوت) نبضات لفّ متكرّرة مع مراقبة gz: إن سقطت
     القراءة إلى صفر مضبوط **عند إقلاع المحركات** فالتشخيص محسوم — تغذية.
  4. عند الاشتباه: `recover()` ثم قراءة تحقّق — هل تعود الحياة بإعادة تهيئة؟

التشغيل على الراسبري (لا يحرّك شيئاً افتراضياً):
    python3 -m pi.tests.check_imu_health
    python3 -m pi.tests.check_imu_health --motors   # ⚠ يشغّل المحركات نبضاً

⚠ ارفع الروبوت على حامل أو أفرغ حوله مساحة قبل `--motors`.
"""
from __future__ import annotations

import argparse
import sys
import time

from pi.config import BNO055_I2C_BUS, BNO055_ADDR, TURN_POWER, ROVER_MODE
from pi.sensors.imu import get_imu

# ── لافتات SYS_STAT/SYS_ERR من ورقة بيانات BNO055 ────────────────
SYS_STAT_TXT = {
    0: "خامل (idle)", 1: "خطأ نظام", 2: "تهيئة الأطراف",
    3: "تهيئة النظام", 4: "اختبار ذاتي", 5: "**الدمج يعمل**",
    6: "يعمل بلا دمج",
}
SYS_ERR_TXT = {
    0: "بلا خطأ", 1: "خطأ تهيئة الطرفية", 2: "خطأ تهيئة النظام",
    3: "فشل الاختبار الذاتي", 4: "قيمة سجل خارج المدى",
    5: "عنوان سجل خارج المدى", 6: "كتابة سجل مرفوضة",
    7: "وضع منخفض الطاقة غير متاح", 8: "وضع تسريع غير متاح",
    9: "تعذّر ضبط معدّل الاستطلاع",
}
MODE_TXT = {0x00: "**CONFIG** ⚠ (كل سجلات البيانات تقرأ 0)",
            0x08: "IMUPLUS (بلا مغنيتومتر) ✅", 0x0C: "NDOF"}


def show_health(imu, title: str) -> dict:
    h = imu.health()
    print(f"\n── {title} ──")
    if not h.get("ok"):
        print(f"  ⛔ {h.get('reason')}")
        return h
    chip, mode = h["chip_id"], h["opr_mode"]
    st, err = h["sys_stat"], h["sys_err"]
    ok_chip = "✅" if chip == 0xA0 else "⛔ ليست BNO055 / ناقل خاطئ"
    print(f"  CHIP_ID  = {hex(chip)}  {ok_chip}")
    print(f"  OPR_MODE = {hex(mode)}  {MODE_TXT.get(mode, 'وضع آخر')}")
    print(f"  SYS_STAT = {st}  {SYS_STAT_TXT.get(st, '?')}")
    print(f"  SYS_ERR  = {err}  {SYS_ERR_TXT.get(err, '?')}")
    s = imu.state()
    print(f"  المعايرة: sys={s['sys_cal']} gyro={s['gyro_cal']} "
          f"accel={s['accel_cal']} mag={s['mag_cal']}  "
          f"(mag=0 متوقَّع في IMUPLUS)")
    return h


def watch(imu, seconds: float, label: str) -> dict:
    """يراقب gz ويحصي **الصفر المضبوط**: أطول سلسلة متتابعة هي المؤشر."""
    n = zeros = run = worst = nones = 0
    peak = 0.0
    deadline = time.time() + seconds
    while time.time() < deadline:
        v = imu.gyro_z_dps()
        if v is None:
            nones += 1
        else:
            n += 1
            peak = max(peak, abs(v))
            if v == 0.0:
                zeros += 1
                run += 1
                worst = max(worst, run)
            else:
                run = 0
        time.sleep(0.02)
    pct = (100.0 * zeros / n) if n else 0.0
    print(f"  {label}: {n} قراءة · صفر مضبوط {zeros} ({pct:.0f}%) · "
          f"أطول سلسلة {worst} · ذروة {peak:.1f}°/ث · فشل قراءة {nones}")
    return {"n": n, "zeros": zeros, "worst_run": worst, "peak": peak,
            "nones": nones}


def main() -> int:
    ap = argparse.ArgumentParser(description="تشخيص صحّة BNO055")
    ap.add_argument("--motors", action="store_true",
                    help="⚠ يشغّل المحركات نبضاً لإعادة إنتاج انهيار التغذية")
    ap.add_argument("--pulses", type=int, default=6, help="عدد نبضات اللفّ")
    args = ap.parse_args()

    print(f"BNO055 المتوقَّع على i2c-{BNO055_I2C_BUS} @ {hex(BNO055_ADDR)}")
    imu = get_imu()
    if not imu.ok:
        print(f"⛔ لم يُفتح الحسّاس: {imu.error}")
        print(f"   افحص:  i2cdetect -y {BNO055_I2C_BUS}   (يجب أن يظهر "
              f"{hex(BNO055_ADDR)[2:]})")
        return 1
    print(f"✅ مفتوح: سائق {imu.driver} · i2c-{imu.bus_num} @ {hex(imu.addr)} "
          f"· وضع {imu.mode_name}")

    show_health(imu, "1) حالة الشريحة الآن")

    print("\n── 2) مرجع: 3ث والمحركات مطفأة ──")
    base = watch(imu, 3.0, "ساكن")
    if base["n"] and base["zeros"] == base["n"]:
        print("  ⛔ كل القراءات صفر مضبوط **والمحركات مطفأة** — الشريحة لا "
              "ترسل أصلاً (راجع OPR_MODE أعلاه: CONFIG؟).")
    elif base["worst_run"] >= 10:
        print(f"  ⚠ سلسلة صفر طولها {base['worst_run']} في السكون — غير طبيعية "
              f"لحسّاس بدقة 1/16°/ث وضجيج σ≈0.07.")
    else:
        print("  ✅ ضجيج طبيعي (لا سلاسل صفر طويلة) — الحسّاس حيّ قبل المحركات.")

    if args.motors:
        if ROVER_MODE != "real":
            print("\n⚠ RMS_ROVER_MODE ليس real — لن تتحرّك المحركات فعلياً.")
        print(f"\n── 3) {args.pulses} نبضة لفّ (⚠ الروبوت يتحرّك) ──")
        from pi.rover.bridge import WaveRoverBridge
        rv = WaveRoverBridge(mode=ROVER_MODE)
        worst = 0
        try:
            for i in range(args.pulses):
                rv.turn("R", TURN_POWER)
                r = watch(imu, 0.8, f"نبضة {i + 1}/{args.pulses}")
                rv.stop()
                worst = max(worst, r["worst_run"])
                if r["n"] and r["zeros"] == r["n"]:
                    print("  ⛔ سقطت القراءة إلى صفر مضبوط **مع المحركات** — "
                          "هذا هو العطل بعينه.")
                    break
                time.sleep(0.4)
        finally:
            rv.stop()
        show_health(imu, "حالة الشريحة بعد نبضات المحركات")
        if worst >= 40:
            print("\n⛔ **التشخيص**: الشريحة تسقط عند إقلاع المحركات.\n"
                  "   إن كان OPR_MODE أعلاه = 0x00 فقد أعادت تشغيل نفسها:\n"
                  "   خطّ 3.3V ينهار مع اندفاع تيار المحركات. العلاج عتادي:\n"
                  "   مكثّف تفريغ (100µF + 100nF) عند تغذية الحسّاس، وفصل\n"
                  "   تغذيته عن خط المحركات، وأرضي مشترك قصير وسميك.")
        elif worst == 0:
            print("\n✅ لم تسقط القراءة مع المحركات في هذه الجولة — أعِد "
                  "التجربة ببطارية أقل شحناً (الانهيار يشتدّ مع انخفاض الجهد).")

    print("\n── 4) اختبار الإحياء (recover) ──")
    res = imu.recover()
    print(("  ✅ " if res.get("recovered") else "  ℹ️ ") + str(res.get("detail")))
    print("\nخلاصة: الاتجاه ينتقل تلقائياً إلى الإحياء أثناء المهمة الآن، "
          "والعطل لا يُعلَن إلا بعد فشل الإحياء وبحالة الشريحة المقروءة.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nتوقّف.")
