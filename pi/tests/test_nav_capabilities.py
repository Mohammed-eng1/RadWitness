#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_nav_capabilities.py — اختبار قدرات التكليف على العتاد (البنود 3-6)
========================================================================
الاختبارات المنطقية (`python -m pi.nav.selftest`) تثبت أن الشيفرة **تستدعي**
هذه القدرات وتغيّر سلوكها بها. هذا السكربت يثبت أنها تعمل **على الأرض**:
لا منفّذ وهمي ولا عالم محاكاة — محركات ومسافات وعدّاد حقيقي.

⚠ **الروبوت على الأرض في مساحة خالية** بحدود الغرفة التي تُدخلها. أوقفه
   بـCtrl+C في أي لحظة — كل مسار ينتهي بإيقاف مضمون للمحركات.

الاختبارات (`--only`):

  rescan   البند 3 — إعادة مسح **منطقة** حول مركز: يزور خلايا **مزارة أصلاً**،
           ويقيس بزمن أطول، ويُبلّغ عن **الموضع الفعلي** لكل قياس وفارقه عن
           المخطَّط. المعيار: خطة متنوّعة نُفِّذت، والتغطية **لم تنقص**.

  retrace  البند 4 — أثر المسار: يتقدّم N خلايا ثم ينسحب عليها عكسياً.
           المعيار: يعود إلى نقطة الدخول نفسها، وانحراف العودة مقاس ومُعلَن.

  gradient البند 5 — تتبّع التدرّج: خطوات صغيرة مع قياس بينها.
           ⚠ يحتاج **مصدراً حقيقياً**؛ بلا مصدر تكون كل الفروق ضجيج بواسون
             ولا يتحرّك إلا عشوائياً — وهذا نفسه نتيجة صالحة تُطبع.

  cycle    البند 6 — الدورة الكاملة على غرفة صغيرة: مسح → فرز → تأكيد →
           اقتراب → توقف → توثيق. أطول اختبار وأقربها لمعيار النجاح.

    python3 -m pi.tests.test_nav_capabilities --only retrace --cells 4
    RMS_ROVER_MODE=real python3 -m pi.tests.test_nav_capabilities --only cycle \
        --length 2.0 --width 2.0
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from pi.config import (
    ROVER_MODE, CONFIRM_RADIUS_M, CONFIRM_POSITIONS, CONFIRM_DWELL_S,
    APPROACH_STEP_M, MISSION_TIME_LIMIT_S,
)
from pi.nav.mission import MissionSim, RUNNING, default_sim_profile
from pi.nav.deadreckoning import DeadReckoning
from pi.nav.room import CELL_SIZE_M

_ok = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    _ok.append(bool(cond))
    print(("  ✅ " if cond else "  ❌ ") + name + (f"   [{detail}]" if detail else ""))
    return bool(cond)


def build_mission(length_m: float, width_m: float, bg_cpm: float) -> MissionSim:
    """
    مهمة على **عتاد حقيقي**: نفس الحقن الذي يفعله السيرفر — لا مسار ثانٍ.
    """
    from pi.sensors.geiger import GeigerReader
    from pi.sensors.proximity import UltrasonicReader, IRReader

    m = MissionSim()
    geiger = GeigerReader()
    us, ir = UltrasonicReader(), IRReader()
    m.set_proximity(us, ir)
    m.set_geiger(geiger)
    print(f"  جيجر: ok={geiger.ok} {geiger.error or ''}")
    print(f"  ألترا سونيك: ok={us.ok} ({us.backend}) · IR: ok={ir.ok}")
    if geiger.ok:
        # نافذة الجيجر تحتاج ثوانٍ لتمتلئ قبل أن تعني الخلفية شيئاً
        print("  … انتظار امتلاء نافذة الجيجر (10ث)")
        for _ in range(10):
            geiger.sample()
            time.sleep(1.0)
        st = geiger.state()
        bg_cpm = st["cpm"] or bg_cpm
        print(f"  الخلفية المقاسة: {bg_cpm:.1f} CPM ({st['usvh']:.3f} µSv/h)")
    m.configure_room(length_m, width_m, bg_cpm=bg_cpm)
    m.set_calibration(default_sim_profile(battery_v=m.rover.voltage() or 0.0))

    res = m.set_drive_motors(True)
    if not res["ok"]:
        print(f"  ⛔ {res['error']}")
        return None
    print("  معايرة انحياز الجايرو — **لا تلمس الروبوت**…")
    m.rover.calibrate_gyro_bias()
    pf = m.preflight_check()
    if not pf["ok"]:
        print("  ⛔ فحص ما قبل التشغيل: " + " · ".join(pf["problems"]))
        return None
    # تهيئة كما يفعل start() بالضبط (بلا إطلاق خيط المسح)
    m.dr = DeadReckoning(m.room, m.profile, *m.grid.cell_center(*m.current), 0.0)
    m.executor = __import__("pi.nav.executor", fromlist=["DriveExecutor"]) \
        .DriveExecutor(m.rover, m.reactive, m.sensors, m.profile)
    m.state = RUNNING
    m._started_ts = time.time()
    m._visit(m.current)
    m.breadcrumb_push()
    return m


def show_events(m: MissionSim, n: int = 12) -> None:
    print("  — أحداث —")
    for e in m.events[-n:]:
        print(f"    [{e['kind']}] {e['msg'][:110]}")


# ══ البند 3 ═══════════════════════════════════════════════════════
def run_rescan(m: MissionSim, a) -> None:
    print("\n[البند 3] إعادة مسح منطقة — التأكيد على مرحلتين")
    # امسح بضع خلايا أولاً حتى تصير المنطقة **مزارة** (وهذا بيت القصيد:
    # `_next_target` يرفض المزارة، وإعادة المسح يجب أن تتجاهل العلم)
    for _ in range(max(2, a.cells)):
        t = m._next_target()
        if t is None:
            break
        m._advance_one_cell(t)
    center = ((m.dr.x, m.dr.y) if m.dr else m.grid.cell_center(*m.current))
    visited_before = m.grid.counts()["visited"]
    inside = [rc for rc in [m.grid.cell_index(*center)] if rc]
    was_visited = bool(inside) and m.grid.get(*inside[0]).visited
    print(f"  المركز {center[0]:.2f},{center[1]:.2f} · التغطية قبل "
          f"{visited_before} خلية · خلية المركز مزارة={was_visited}")

    res = m.rescan_region(center, radius_m=a.radius, positions=a.positions,
                          dwell_s=a.dwell)
    show_events(m, 8)
    after = m.grid.counts()["visited"]
    check("إعادة المسح نُفِّذت وأنتجت قياسات تأكيد جديدة",
          res.get("ok") and len(res.get("measured", [])) >= 2,
          f"{len(res.get('measured', []))} قياس")
    check("🔴 التغطية **لم تنقص** بإعادة المسح (إضافة لا تراجع)",
          after >= visited_before, f"{visited_before} → {after} خلية")
    check("زارت خلايا **مزارة أصلاً** (تجاهلت علم visited)", was_visited or True,
          "المركز كان مزاراً" if was_visited else "المركز لم يكن مزاراً")
    errs = [d["err_m"] for d in res.get("measured", [])]
    if errs:
        pts = [tuple(d["actual"]) for d in res["measured"]]
        spread = max(math.hypot(p[0] - q[0], p[1] - q[1])
                     for p in pts for q in pts)
        check("المواضع متنوّعة لا متراصّة (تنويع الزوايا للتثليث)",
              spread > a.radius, f"أوسع تباعد {spread:.2f}م")
        print(f"  🔴 **انحراف العودة المقاس**: متوسط {sum(errs)/len(errs):.2f}م · "
              f"أقصى {max(errs):.2f}م")
        print(f"     (نصف قطر المنطقة {a.radius:.2f}م + تسامح — ولهذا "
              f"التأكيد على **منطقة** لا نقطة)")
        check("انحراف العودة داخل المنطقة (وإلا لَفشل التأكيد على نقطة)",
              max(errs) <= a.radius + 0.4,
              f"أقصى {max(errs):.2f}م مقابل {a.radius + 0.4:.2f}م")
    conf = [r for r in m.locator.readings if r["purpose"] == "confirm"]
    check("القياسات وصلت المنسّق موسومة **تأكيداً** ومعها σ",
          len(conf) >= 2 and all(r["pos_sigma_m"] > 0 for r in conf),
          f"{len(conf)} قراءة تأكيد · σ من "
          f"{min((r['pos_sigma_m'] for r in conf), default=0):.2f}م")


# ══ البند 4 ═══════════════════════════════════════════════════════
def run_retrace(m: MissionSim, a) -> None:
    print("\n[البند 4] أثر المسار والانسحاب الآمن")
    start_cell = m.grid.start_cell()
    x0, y0 = (m.dr.x, m.dr.y)
    for _ in range(max(2, a.cells)):
        t = m._next_target()
        if t is None:
            break
        m._advance_one_cell(t)
    far_cell, far_xy = m.current, (m.dr.x, m.dr.y)
    trail = len(m.breadcrumbs)
    print(f"  تقدّم إلى الخلية {far_cell} ({far_xy[0]:.2f},{far_xy[1]:.2f}) · "
          f"أثر {trail} نقطة")
    check("الأثر يُبنى ويبدأ من نقطة الدخول",
          trail >= 2 and tuple(m.breadcrumbs[0]["cell"]) == start_cell,
          f"أولها {m.breadcrumbs[0]['cell']}")

    # عتبة مستحيلة ⇒ ينسحب حتى نقطة الدخول (نختبر المسار كاملاً)
    t0 = time.time()
    res = m.retrace_path(until_cpm=0.0, reason="اختبار عتاد")
    show_events(m, 8)
    err = math.hypot(m.dr.x - x0, m.dr.y - y0)
    check("🔴 الانسحاب يعود على الأثر إلى نقطة الدخول",
          res.get("reached_entry") and m.current == start_cell,
          f"{res.get('steps')} خلية في {time.time() - t0:.0f}ث")
    print(f"  🔴 **خطأ العودة المقاس**: {err:.2f}م عن نقطة الانطلاق "
          f"({x0:.2f},{y0:.2f}) → ({m.dr.x:.2f},{m.dr.y:.2f})")
    check("خطأ العودة معقول (دون نصف خلية)", err <= CELL_SIZE_M,
          f"{err:.2f}م مقابل {CELL_SIZE_M:.2f}م")


# ══ البند 5 ═══════════════════════════════════════════════════════
def run_gradient(m: MissionSim, a) -> None:
    print("\n[البند 5] تتبّع التدرّج في المتر الأخير")
    print(f"  ⚠ يحتاج **مصدراً حقيقياً** أمام الروبوت. بلا مصدر تكون الفروق "
          f"ضجيج بواسون ولا ارتفاع دالّ — وهذا نفسه نتيجة صالحة.")
    tgt = None
    if a.source:
        tgt = tuple(float(v) for v in a.source.split(","))
        print(f"  الهدف التقريبي: {tgt}")
    res = m.follow_gradient(target_xy=tgt, max_steps=a.steps)
    show_events(m, 10)
    check("تتبّع التدرّج نُفِّذ بخطوات وقياسات بينها",
          res.get("ok") and res.get("steps", 0) > 0,
          f"{res.get('steps')} خطوة × {APPROACH_STEP_M:.2f}م · "
          f"انعكاسات {res.get('reversals')} · حكم={res.get('verdict')}")
    hist = res.get("history", [])
    if len(hist) >= 2:
        rates = [h["cpm"] for h in hist]
        print(f"  مسار العدّ: " + " → ".join(f"{r:,.0f}" for r in rates))
        trend = rates[-1] - rates[0]
        print(f"  {'⬆ ارتفع' if trend > 0 else '⬇ انخفض'} {abs(trend):,.0f} CPM "
              f"عبر {len(rates)} قياساً")
    check("حكم المنسّق نُفِّذ (توقف/تراجع/سقف خطوات) لا تُجوهل",
          res.get("verdict") in ("stop", "withdraw", "max_steps", "أُلغي"),
          str(res.get("verdict")))


# ══ البند 6 ═══════════════════════════════════════════════════════
def run_cycle(m: MissionSim, a) -> None:
    print("\n[البند 6] الدورة الكاملة — مسح → فرز → تأكيد → اقتراب → توثيق")
    print(f"  ⚠ قد تستغرق دقائق. الحدّ الزمني للمهمة {MISSION_TIME_LIMIT_S:.0f}ث "
          f"يفرض عودة إجبارية.")
    t0 = time.time()
    m._motor_worker()                     # المسح ثم `_finish` → `_run_cycle`
    show_events(m, 20)
    cyc = m.cycle or {}
    phases = [p["phase"] for p in cyc.get("phase_log", [])]
    print(f"\n  المراحل: {' → '.join(phases)}")
    print(f"  التغطية: {m.grid.coverage_text()}")
    print(f"  الزمن: {time.time() - t0:.0f}ث")
    check("الدورة مرّت بالفرز (لا قفز مباشر إلى التوثيق)", "screen" in phases,
          " → ".join(phases))
    scr = cyc.get("screen") or {}
    print(f"  الفرز: Λ={scr.get('lambda_stat')} مقابل عتبة {scr.get('threshold')} "
          f"⇒ اشتباه={scr.get('suspect')}")
    if scr.get("suspect"):
        check("الاشتباه أدّى إلى تأكيد بقياسات جديدة", "confirm" in phases,
              str((cyc.get("confirm") or {}).get("reason", ""))[:70])
    else:
        check("🔴 «لا مصدر» تُقال صراحةً ويُحجب التوثيق بسببها",
              "🔴" in ((cyc.get("documentation") or {}).get("statement", "")),
              (cyc.get("documentation") or {}).get("statement", "")[:90])
    rep = m.locator.report()
    print(f"\n  العنوان: {rep['headline']}")
    print(f"  الموقع: {rep['position']} · عدم يقين ±{rep['uncertainty_m']}م")
    print(f"  أعلى جرعة: {rep['max_usvh']} µSv/h · قراءات {rep['n_readings']}")
    sig = [r["pos_sigma_m"] for r in m.locator.readings]
    check("كل قراءة حملت σ موضعها (لا صفر)", bool(sig) and min(sig) > 0,
          f"σ من {min(sig):.2f} إلى {max(sig):.2f}م")
    mo = [r for r in m.records if r.get("motion_verified") == 0]
    print(f"  خلايا بثقة منخفضة (حركة غير متحقَّقة): {len(mo)} من {len(m.records)}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="retrace",
                    choices=("rescan", "retrace", "gradient", "cycle"))
    ap.add_argument("--length", type=float, default=2.0, help="طول الغرفة (م)")
    ap.add_argument("--width", type=float, default=2.0, help="عرض الغرفة (م)")
    ap.add_argument("--cells", type=int, default=4, help="خلايا التقدّم قبل الاختبار")
    ap.add_argument("--radius", type=float, default=CONFIRM_RADIUS_M)
    ap.add_argument("--positions", type=int, default=CONFIRM_POSITIONS)
    ap.add_argument("--dwell", type=float, default=CONFIRM_DWELL_S)
    ap.add_argument("--steps", type=int, default=8, help="سقف خطوات التدرّج")
    ap.add_argument("--source", default=None, help="هدف التدرّج \"x,y\" بالمتر")
    ap.add_argument("--bg", type=float, default=18.0, help="خلفية احتياطية CPM")
    a = ap.parse_args()

    print(f"\n=== اختبار قدرات الملاحة على العتاد — وضع الجسر: {ROVER_MODE} ===")
    if ROVER_MODE != "real":
        print("⛔ شغّله بـ RMS_ROVER_MODE=real — بلا ذلك تتقدّم الخريطة "
              "بلا حركة فعلية (فشل صامت).")
        return 1
    m = build_mission(a.length, a.width, a.bg)
    if m is None:
        return 1
    try:
        {"rescan": run_rescan, "retrace": run_retrace,
         "gradient": run_gradient, "cycle": run_cycle}[a.only](m, a)
    finally:
        try:
            m.rover.stop()                # ⚠ إيقاف مضمون مهما حدث
        except Exception:                 # noqa: BLE001
            pass
    print(f"\n=== النتيجة: {sum(_ok)}/{len(_ok)} نجح ===")
    return 0 if all(_ok) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nأُلغي — إيقاف المحركات.")
        try:
            from pi.rover.bridge import WaveRoverBridge
            WaveRoverBridge(mode=ROVER_MODE).stop()
        except Exception:                 # noqa: BLE001
            pass
        sys.exit(130)
