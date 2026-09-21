#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bringup.py — المشغّل الرئيسي: هل الروبوت الجديد حيّ بالكامل؟
================================================================================
يشغّل اختبارات الإقلاع بالترتيب ويطبع جدول GO/NO-GO واحداً في النهاية.

  بلا خيارات   →  1. test_link   ثم  2. test_imu     (**لا حركة إطلاقاً**)
  مع --motors  →  تُضاف 3. test_heartbeat · 4. test_encoder · 6. test_battery
                  بعد تأكيد منفصل

🔴 **لا يشغّل test_drive (5) تلقائياً أبداً.** ذاك يقود الروبوت على الأرض
لمسافات حقيقية ويحتاج إنساناً بمسطرة عند كل شوط؛ يُبدأ بقرار صريح لا ضمن
تسلسل آلي.

🔴 **بوابة**: إن رسب 1 أو 2 فلا تُشغَّل اختبارات الحركة. قيادة روبوت وصلته
معطوبة أو جايروه ميت ليست اختباراً بل مخاطرة — والنتيجة لن تعني شيئاً أصلاً.

ترتيب الوضع المادي (يُطبع قبل كل اختبار أيضاً):
  1, 2  → أي وضع، ساكن تماماً
  3, 4  → **مرفوع عن الأرض** والعجلات حرّة
  6     → **على الأرض** في مساحة خالية

⚠ كل سكربت يسأل تأكيده الخاص بكلمة "yes" قبل تشغيل المحركات. تأكيد bringup
لا يلغي ذلك ولا يمرّ عنه: لا يوجد في هذه المجموعة علم يتجاوز التأكيد.

يستدعي bringup الاختبارات كعمليات مستقلة (subprocess) لا يستوردها: فسقوط
سكربت لا يُسقط الجدول، ويبقى كل ملف صالحاً للتشغيل وحده.

التشغيل:
    python3 -m pi.tests.ugv01.bringup
    python3 -m pi.tests.ugv01.bringup --motors --port /dev/ttyUSB0
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

# جذر المستودع: pi/tests/ugv01/bringup.py ⇒ ثلاثة مستويات فوق
REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))

# (المفتاح، الوحدة، الوصف، وضع الروبوت، أيحرّك المحركات؟)
QUIET_TESTS = (
    ("1", "test_link", "وصلة السيريال وردّ اللوحة", "أي وضع — ساكن", False),
    ("2", "test_imu", "حياة الجايرو والتسارع", "ساكن تماماً", False),
)
MOTOR_TESTS = (
    ("3", "test_heartbeat", "مهلة نبضة القلب", "🔴 مرفوع عن الأرض", True),
    ("4", "test_encoder", "ردّ الإنكودر وإشارته", "🔴 مرفوع عن الأرض", True),
    ("6", "test_battery", "هبوط الجهد تحت الحمل", "🔴 على الأرض، مساحة خالية", True),
)

RESULT_GO, RESULT_NOGO, RESULT_ABORT, RESULT_SKIP = "GO", "NO-GO", "أُجهض", "تُخطّي"


def run_test(module, passthrough):
    """يشغّل اختباراً كعملية مستقلة ويرث الطرفية (فيعمل سؤال yes فيه)."""
    cmd = [sys.executable, "-m", "pi.tests.ugv01." + module] + passthrough
    print()
    print("─" * 66)
    print("  ▶ %s" % " ".join(cmd))
    print("─" * 66)
    t0 = time.monotonic()
    try:
        code = subprocess.call(cmd, cwd=REPO_ROOT)
    except KeyboardInterrupt:
        return RESULT_ABORT, time.monotonic() - t0
    except Exception as exc:
        print("  ✗ تعذّر تشغيل %s: %s" % (module, exc))
        return RESULT_ABORT, time.monotonic() - t0
    elapsed = time.monotonic() - t0
    if code == 0:
        return RESULT_GO, elapsed
    if code == 1:
        return RESULT_NOGO, elapsed
    return RESULT_ABORT, elapsed


def print_table(rows):
    """جدول واحد في النهاية — هو ما يُنظر إليه، لا شلال السجلات فوقه."""
    print()
    print("═" * 66)
    print("  جدول الإقلاع — UGV01")
    print("═" * 66)
    print("  %-3s %-16s %-26s %-8s %s" % ("#", "الاختبار", "ماذا يفحص", "النتيجة", "الزمن"))
    print("  " + "─" * 62)
    for key, module, what, result, elapsed in rows:
        mark = {RESULT_GO: "✅", RESULT_NOGO: "❌",
                RESULT_ABORT: "⚠", RESULT_SKIP: "⏭"}.get(result, "?")
        span = "—" if elapsed is None else "%.1fث" % elapsed
        print("  %-3s %-16s %-26s %s %-6s %s" % (key, module, what, mark, result, span))
    print("═" * 66)


def confirm(question):
    try:
        return input(question).strip().lower() == "yes"
    except EOFError:
        return False


def main():
    ap = argparse.ArgumentParser(description="UGV01: مشغّل اختبارات الإقلاع")
    ap.add_argument("--port", default=None, help="يُمرَّر لكل اختبار")
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--boot-wait", type=float, default=None)
    ap.add_argument("--motors", action="store_true",
                    help="⚠ يضيف اختبارات تشغّل المحركات (3، 4، 6)")
    a = ap.parse_args()

    passthrough = []
    if a.port:
        passthrough += ["--port", a.port]
    if a.baud:
        passthrough += ["--baud", str(a.baud)]
    if a.boot_wait is not None:
        passthrough += ["--boot-wait", str(a.boot_wait)]

    print("═" * 66)
    print("  UGV01 · إقلاع اليوم الأول (bringup)")
    print("═" * 66)
    print("  المرحلة الأولى بلا حركة: %s"
          % "، ".join(m for _, m, _, _, _ in QUIET_TESTS))
    if a.motors:
        print("  ثم مرحلة المحركات: %s"
              % "، ".join(m for _, m, _, _, _ in MOTOR_TESTS))
    else:
        print("  (مرحلة المحركات معطّلة — أضف --motors لتشغيلها.)")
    print("  🔴 test_drive لا يُشغَّل من هنا أبداً — يُبدأ بقرار صريح.")

    rows = []
    gate_ok = True

    # ── المرحلة الأولى: بلا حركة ────────────────────────────────────────
    for key, module, what, placement, _moves in QUIET_TESTS:
        print("\n  وضع الروبوت المطلوب: %s" % placement)
        result, elapsed = run_test(module, passthrough)
        rows.append((key, module, what, result, elapsed))
        if result != RESULT_GO:
            gate_ok = False
        if result == RESULT_ABORT:
            print("\n  ⚠ أُجهض %s — يتوقف التسلسل." % module)
            for k2, m2, w2, _p, _mv in QUIET_TESTS + MOTOR_TESTS:
                if not any(r[0] == k2 for r in rows):
                    rows.append((k2, m2, w2, RESULT_SKIP, None))
            print_table(rows)
            print("NO-GO")
            return 2

    # ── المرحلة الثانية: المحركات ───────────────────────────────────────
    if a.motors:
        if not gate_ok:
            print("\n  🔴 رسب اختبار بلا حركة ⇒ **لن تُشغَّل اختبارات المحركات**.")
            print("     قيادة روبوت وصلته معطوبة أو جايروه ميت ليست اختباراً،")
            print("     ونتيجتها لن تعني شيئاً على أي حال. أصلح الأعلى أولاً.")
            for key, module, what, _p, _mv in MOTOR_TESTS:
                rows.append((key, module, what, RESULT_SKIP, None))
        else:
            print()
            print("⚠" * 33)
            print("⚠ المرحلة التالية **تشغّل المحركات** في %d اختبارات."
                  % len(MOTOR_TESTS))
            print("⚠ وسيسألك كل اختبار تأكيده الخاص أيضاً — هذا التأكيد لا يلغيه.")
            print("⚠" * 33)
            if not confirm('اكتب "yes" للمتابعة إلى اختبارات المحركات: '):
                print("  تُخطّيت مرحلة المحركات بناء على طلبك.")
                for key, module, what, _p, _mv in MOTOR_TESTS:
                    rows.append((key, module, what, RESULT_SKIP, None))
            else:
                for key, module, what, placement, _moves in MOTOR_TESTS:
                    print("\n  وضع الروبوت المطلوب: %s" % placement)
                    if not confirm('   الروبوت في هذا الوضع الآن؟ اكتب "yes": '):
                        print("   تُخطّي %s." % module)
                        rows.append((key, module, what, RESULT_SKIP, None))
                        continue
                    result, elapsed = run_test(module, passthrough)
                    rows.append((key, module, what, result, elapsed))
                    if result == RESULT_ABORT:
                        print("\n  ⚠ أُجهض %s — يتوقف التسلسل." % module)
                        break
                for key, module, what, _p, _mv in MOTOR_TESTS:
                    if not any(r[0] == key for r in rows):
                        rows.append((key, module, what, RESULT_SKIP, None))

    # ── الجدول والحكم ───────────────────────────────────────────────────
    # اختبارات لم تُشغَّل تظهر «تُخطّي» صراحةً: جدولٌ يُسقط صفوفه يوحي بتغطية
    # لم تحدث، والغائب من الجدول أخطر من الراسب فيه.
    if not a.motors:
        for key, module, what, _p, _mv in MOTOR_TESTS:
            rows.append((key, module, what, RESULT_SKIP, None))
    rows.append(("5", "test_drive", "حركة على الأرض بمسطرة", RESULT_SKIP, None))
    rows.sort(key=lambda r: r[0])
    print_table(rows)

    ran = [r for r in rows if r[3] in (RESULT_GO, RESULT_NOGO)]
    failed = [r for r in rows if r[3] == RESULT_NOGO]
    aborted = [r for r in rows if r[3] == RESULT_ABORT]

    print("  شُغّل %d · نجح %d · رسب %d · أُجهض %d"
          % (len(ran), len(ran) - len(failed), len(failed), len(aborted)))
    if failed:
        print("  الراسب: %s" % "، ".join(r[1] for r in failed))
        print("  راجع pi/tests/ugv01/README.md — لكل NO-GO معناه وخطوته التالية.")
    if not a.motors and not failed and not aborted:
        print("  التالي: --motors لاختبارات المحركات، ثم test_drive يدوياً.")
    print("  🔴 test_drive (5) لم يُشغَّل — بالتصميم.")
    print("═" * 66)

    if aborted:
        print("NO-GO")
        return 2
    print("GO" if ran and not failed else "NO-GO")
    return 0 if (ran and not failed) else 1


if __name__ == "__main__":
    sys.exit(main())
