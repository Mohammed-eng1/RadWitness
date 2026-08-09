#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_rover_link.py — أداة خارجية: هل ESP32 الروفر يسمعنا ويردّ أصلاً؟
======================================================================
كُتب للعطل المقاس 2026-08-06: أوامر الحركة تُرسل بلا أي خطأ والروبوت لا
يتحرك إطلاقاً — فتح UART ينجح وإن كان الطرف الآخر ميتاً أو أصمّ، ولا
شيء في مسار الإرسال يكشف ذلك. هذا المسبار يفصل الاحتمالات طبقة طبقة:

  1. **استماع سلبي** (3ث): بعض إصدارات الفيرموير تبثّ تلقائياً — أي بايت
     يصل يثبت أن RX الراسبري وسلك TX↔RX سليمان.
  2. **استعلام** `{"T":126}` خمس مرات: الردّ `{"T":1002,...}` يثبت الوصلة
     كاملة بالاتجاهين (TX الراسبري ← RX الروفر والعكس).
  3. (اختياري `--spin`) أمر دوران فعلي `{"T":1,"L":p,"R":-p}` ثانية واحدة
     مع تجديد الأمر — أنت تراقب العجلات: دارت والوصلة تردّ؟ المشكلة إذن
     ليست الوصلة بل ما بعدها.

تفسير النتائج:
  صمت كامل (لا سلبي ولا ردّ)  ⇒ سلك TX/RX/GND، أو الفيرموير لا يعمل.
     ⚠ احتمال موثَّق يستحق الفحص: **فُلِش فيرموير شاشة المتحكم على
     ESP32 الروفر بالخطأ** (المسار الآخر يبني firmware/controller_display
     في نفس الفترة) — فيرموير آخر = صمت JSON كامل رغم عتاد سليم.
  يردّ 1002 والعجلات لا تدور مع --spin ⇒ الوصلة سليمة والعطل في مسار
     المحركات: درايفر، تغذية محركات، أو حماية فيرموير (جهد منخفض؟).

التشغيل:
    python3 -m pi.tests.test_rover_link
    python3 -m pi.tests.test_rover_link --spin --power 0.3   # ⚠ يدير الروبوت
"""
from __future__ import annotations

import argparse
import json
import sys
import time

from pi.config import ROVER_PORT, ROVER_BAUD, MAX_MOTOR_POWER

try:
    import serial
except ImportError:
    sys.exit("خطأ: pyserial غير مثبّتة (pip install pyserial).")


def read_lines(ser, seconds: float) -> list:
    """يجمع الأسطر الواصلة خلال المهلة (خاماً، بلا افتراض صلاحية JSON)."""
    out, buf = [], b""
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                out.append(line.decode("ascii", errors="replace").strip())
        else:
            time.sleep(0.02)
    if buf.strip():
        out.append(buf.decode("ascii", errors="replace").strip() + " …(بلا \\n)")
    return [l for l in out if l]


def main() -> int:
    ap = argparse.ArgumentParser(description="مسبار وصلة الروفر (UART ثنائي الاتجاه)")
    ap.add_argument("--port", default=ROVER_PORT)
    ap.add_argument("--baud", type=int, default=ROVER_BAUD)
    ap.add_argument("--spin", action="store_true",
                    help="⚠ أمر دوران فعلي 1ث — يدير الروبوت بالمكان")
    ap.add_argument("--truth-table", action="store_true",
                    help="🔬 4 نبضات خام والدوران يُقاس بالجايرو — جدول حقيقة الفيرموير")
    ap.add_argument("--wheels", action="store_true",
                    help="🔍 فحص العجلات الموجّه: نبضة ثم سؤال بسيط (f/b/n/w) — "
                         "الروبوت مرفوع والعجلات حرة")
    ap.add_argument("--map", action="store_true",
                    help="🧭 حسم الخريطة بمعلم فيزيائي (أنبوب الجيجر) — "
                         "حقل واحد لكل نبضة، بلا كلمتي يمين/يسار")
    ap.add_argument("--power", type=float, default=0.3)
    a = ap.parse_args()

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.05)
    except Exception as e:                    # noqa: BLE001
        sys.exit(f"⛔ تعذّر فتح {a.port}: {e}\n"
                 f"   (سيرفر يحجزه؟ sudo lsof {a.port})")
    print(f"المنفذ {a.port} @ {a.baud} مفتوح ✅")

    try:
        # ── 1) استماع سلبي ───────────────────────────────────────
        ser.reset_input_buffer()
        print("\n[1/3] استماع سلبي 3ث (بلا إرسال)…")
        passive = read_lines(ser, 3.0)
        if passive:
            print(f"  ✅ وصل {len(passive)} سطر تلقائياً — RX يعمل. أول سطرين:")
            for l in passive[:2]:
                print(f"     {l[:100]}")
        else:
            print("  (صمت — ليس حكماً بعد: بعض الفيرموير لا يبثّ إلا سؤالاً)")

        # ── 2) استعلام T=126 ─────────────────────────────────────
        print("\n[2/3] استعلام {\"T\":126} × 5…")
        replies = []
        for _ in range(5):
            ser.write(b'{"T":126}\n')
            replies += read_lines(ser, 0.4)
        got_1002 = False
        for l in replies:
            try:
                d = json.loads(l)
                if isinstance(d, dict) and d.get("T") == 1002:
                    got_1002 = True
                    print(f"  ✅ ردّ 1002 سليم: gz={d.get('gz')} · v={d.get('v')}")
                    break
            except Exception:                 # noqa: BLE001
                continue
        if not got_1002:
            if replies:
                print(f"  ⚠ وصل {len(replies)} سطر لكن لا 1002 مفهوماً. عيّنة:")
                for l in replies[:3]:
                    print(f"     {l[:100]}")
                print("  ⇒ شيء يبثّ لكنه ليس بروتوكول Wave Rover — فيرموير "
                      "مختلف؟ سرعة باود خاطئة؟")
            else:
                print("  ⛔ **صمت كامل بالاتجاهين** — الوصلة أو الفيرموير:")
                print("     • سلك TX/RX معكوس أو مفصول (تحرّك أثناء تركيب الحساسات؟)")
                print("     • GND غير مشترك بين الراسبري والروفر")
                print("     • ⚠ هل فُلِش فيرموير شاشة المتحكم على ESP32 الروفر؟")
            return 1

        # ── 3د) 🧭 حسم الخريطة بمعلم فيزيائي — لا «يمين/يسار» إطلاقاً ──
        # شهادات الجهة انقلبت مرتين في هذه القصة (النظر من الأمام يعكسها).
        # المعلم الذي لا يلتبس: أنبوب الجيجر مثبّت على **يسار** الروبوت
        # (قاسه المشغّل بالشريط 2026-08-09) — والسؤال يصير «جهة الأنبوب أم
        # المقابلة؟» فلا مجال للمرآة. حقل واحد لكل نبضة يعزل كل قناة.
        if a.map:
            p = min(abs(a.power), MAX_MOTOR_POWER)

            def pulse2(lp, rp, sec=2.0):
                t0 = time.time()
                while time.time() - t0 < sec:
                    ser.write(json.dumps({"T": 1, "L": round(lp, 3),
                                          "R": round(rp, 3)}).encode() + b"\n")
                    time.sleep(0.1)
                ser.write(b'{"T":1,"L":0,"R":0}\n')
                ser.write(b'{"T":1,"L":0,"R":0}\n')

            def ask2(q, opts):
                while True:
                    print(q)
                    try:
                        v = input(f"[{'/'.join(opts)}] ").strip().lower()
                    except Exception:          # noqa: BLE001
                        print("  ⚠ أعد الكتابة — الحرف وحده ثم Enter")
                        continue
                    if v in opts:
                        return v
                    print(f"  حرف من: {' · '.join(opts)}")

            print(f"\n🧭 حسم الخريطة — قوة {p}. ⚠ الروبوت **مرفوع** والعجلات "
                  f"حرة.\n  المعلم: **أنبوب الجيجر** (على أحد جانبي الروبوت) "
                  f"— لا نستعمل يمين/يسار أبداً.")
            results = {}
            for fname, lp, rp in (("L", p, 0.0), ("R", 0.0, p)):
                input(f"\n  ▶ نبضة الحقل {fname} وحده — Enter…")
                pulse2(lp, rp)
                side = ask2("  أي جهة دارت عجلاتها؟ g = جهة أنبوب الجيجر · "
                            "o = الجهة المقابلة · n = لا شيء دار",
                            ("g", "o", "n"))
                dirn = "n"
                if side != "n":
                    dirn = ask2("  ودارت نحو: f = مقدمة الروبوت (جهة الكاميرا) "
                                "· b = مؤخرته", ("f", "b"))
                results[fname] = (side, dirn)
            ar_side = {"g": "جهة الجيجر (يسار الروبوت)",
                       "o": "الجهة المقابلة (يمين الروبوت)", "n": "لا دوران"}
            ar_dir = {"f": "أمام", "b": "خلف", "n": "—"}
            print("\n  ── الخريطة المرصودة (الجيجر = يسار الروبوت) ──")
            for fname in ("L", "R"):
                s, d = results[fname]
                print(f"    الحقل {fname}=+p ⇒ {ar_side[s]} · {ar_dir[d]}")
            # الاشتقاق: side_g = يسار حقيقي. wheel(field)=±
            print("\n  ── الاشتقاق ──")
            ls, ld = results["L"]
            rs, rd = results["R"]
            if "n" in (ls, rs):
                print("  🔴 حقل بلا استجابة — جانب ميت/قناة غير مقودة. أرسل الخرج.")
            elif ls == rs:
                print("  🔴 الحقلان يحرّكان نفس الجهة — قناتا الفيرموير على "
                      "جانب واحد. أرسل الخرج.")
            else:
                # أي حقل يقود جهة الجيجر (يسار الروبوت)؟ وبأي قطبية؟
                left_field = "L" if ls == "g" else "R"
                left_dir = ld if ls == "g" else rd
                right_dir = rd if ls == "g" else ld
                swap_needed = (left_field == "R")
                inv_needed = (left_dir == "b")   # ‎+p يجب أن يقدّم العجلة
                same_pol = (left_dir == right_dir)
                if not same_pol:
                    print("  🔴 قطبية الجانبين مختلفة — انعكاس محرك جانب واحد "
                          "(أسلاك) لا يصلحه ثابت. أرسل الخرج.")
                else:
                    print(f"  ✅ الوصفة النهائية في config:")
                    print(f"     MOTOR_SWAP_LR = {swap_needed}")
                    print(f"     MOTOR_INVERT  = {-1 if inv_needed else +1}")
                    print("  طبّقها ثم تحقق بـcheck_directions (من خلف الروبوت).")
            return 0

        # ── 3ج) 🔍 فحص العجلات الموجّه — سؤال واحد بسيط في كل مرة ────
        # الروبوت مرفوع والعجلات حرة: مراقبة عجلة عن قرب لا تلتبس، والسكربت
        # يبني الجدول ويحكم بنفسه (المشغّل لم يفهم جدولاً يملؤه يدوياً — عدّل
        # الأسلوب لا تلُم المشغّل، 2026-08-09).
        if a.wheels:
            p = min(abs(a.power), MAX_MOTOR_POWER)

            def pulse(lp, rp, sec=2.0):
                t0 = time.time()
                while time.time() - t0 < sec:
                    ser.write(json.dumps({"T": 1, "L": round(lp, 3),
                                          "R": round(rp, 3)}).encode() + b"\n")
                    time.sleep(0.1)
                ser.write(b'{"T":1,"L":0,"R":0}\n')
                ser.write(b'{"T":1,"L":0,"R":0}\n')

            def ask(side_ar):
                while True:
                    print(f"  عجلتا {side_ar}: f = أمام · b = خلف · "
                          f"n = ساكنة · w = ضعيفة/متقطعة")
                    try:
                        v = input("[f/b/n/w] ").strip().lower()
                    except Exception:          # noqa: BLE001 — ترميز طرفية
                        print("  ⚠ تعذّرت القراءة — اكتب الحرف وحده وEnter")
                        continue
                    if v in ("f", "b", "n", "w"):
                        return v
                    print("  حرف واحد فقط من الأربعة.")

            print(f"\n🔍 فحص العجلات الموجّه — قوة {p}. ⚠ الروبوت **مرفوع** "
                  f"والعجلات في الهواء.")
            print("  ثماني نبضات (2ث لكل نبضة): في كل نبضة راقب الجانب الذي "
                  "أطلبه **فقط** وأجب بحرف واحد.")
            pats = [("1", p, -p), ("2", -p, p), ("3", p, p), ("4", -p, -p)]
            obs = {}
            for name, lp, rp in pats:
                for side_ar, key in (("اليسار", "L"), ("اليمين", "R")):
                    input(f"\n  ▶ راقب عجلتي **{side_ar}** الآن — Enter للنبضة…")
                    pulse(lp, rp)
                    obs[(name, key)] = ask(side_ar)
            ar = {"f": "أمام", "b": "خلف", "n": "ساكنة", "w": "ضعيفة"}
            print("\n  ── الجدول المرصود (النمط: يسار | يمين) ──")
            for name, lp, rp in pats:
                print(f"    نمط {name} (L={'+' if lp>0 else '-'}p R="
                      f"{'+' if rp>0 else '-'}p): "
                      f"{ar[obs[(name,'L')]]} | {ar[obs[(name,'R')]]}")
            l_dead = all(obs[(n, "L")] in ("n", "w") for n, _, _ in pats)
            r_dead = all(obs[(n, "R")] in ("n", "w") for n, _, _ in pats)
            l_nosign = (obs[("1", "L")] == obs[("2", "L")]
                        and obs[("1", "L")] in ("f", "b"))
            r_nosign = (obs[("1", "R")] == obs[("2", "R")]
                        and obs[("1", "R")] in ("f", "b"))
            print("\n  ── الحكم ──")
            verdicts = False
            if l_dead:
                verdicts = True
                print("  🔴 عجلات **اليسار** ميتة/ضعيفة في كل الأنماط ⇒ "
                      "موصلا محركي اليسار على اللوحة، أو درايفر اليسار، أو "
                      "فيرموير لا يقود أطرافه. ابدأ بإعادة تثبيت الموصلين.")
            if r_dead:
                verdicts = True
                print("  🔴 عجلات **اليمين** ميتة/ضعيفة في كل الأنماط ⇒ "
                      "موصلا محركي اليمين، أو درايفر اليمين، أو الفيرموير. "
                      "ابدأ بإعادة تثبيت الموصلين.")
            if l_nosign:
                verdicts = True
                print("  🔴 اليسار يدور بنفس الاتجاه في النمطين المتعاكسين ⇒ "
                      "يتجاهل إشارة أمره — فيرموير (بناء لوحة خاطئ) أو درايفر.")
            if r_nosign:
                verdicts = True
                print("  🔴 اليمين يتجاهل إشارة أمره — فيرموير أو درايفر.")
            if not verdicts:
                print("  ✅ الجانبان يستجيبان ويعكسان الإشارة — أرسل الجدول "
                      "لاشتقاق الثابتين نهائياً.")
            print("  أرسل الخرج كاملاً كما هو.")
            return 0

        # ── 3ب) 🔬 جدول حقيقة الفيرموير — القياس بالجايرو لا بالعين ──
        # وُلد من مأزق مقاس (2026-08-09): أمر «يمين» أنتج يساراً في كلا وضعي
        # التبديل رغم أن النمطين السلكيين متعاكسان — مستحيل على عتاد خطي
        # سليم. الاحتمالان الباقيان: فيرموير غير خطي مع الإشارة السالبة، أو
        # جانب محركات ميت/ضعيف (يكشفه النمط المتماثل). العين خارج الحلقة
        # نهائياً: الجايرو المثبت يدوياً هو الحكم.
        if a.truth_table:
            try:
                from pi.sensors.mpu6050 import get_mpu
                from pi.config import MPU6050_GYRO_Z_SIGN as _ZS
            except Exception as e:            # noqa: BLE001
                sys.exit(f"⛔ يحتاج MPU: {e}")
            mpu = get_mpu()
            if not mpu.ok:
                sys.exit(f"⛔ MPU غير متاح ({mpu.error}) — الجدول يقيس بالجايرو")
            p = min(abs(a.power), MAX_MOTOR_POWER)
            print(f"\n[3/3] 🔬 جدول الحقيقة: 4 أنماط خام × 1ث بقوة {p} — "
                  f"أفرغ حول الروبوت (3ث)…")
            time.sleep(3.0)
            pats = [("L=+p R=-p", p, -p), ("L=-p R=+p", -p, p),
                    ("L=+p R=+p", p, p), ("L=-p R=-p", -p, -p)]
            res = []
            for name, lp, rp in pats:
                t0 = time.time()
                bias, n = 0.0, 0
                while time.time() - t0 < 1.0:      # انحياز سريع ساكناً
                    z = mpu.gyro_z_dps()
                    if z is not None:
                        bias += z
                        n += 1
                    time.sleep(0.01)
                bias /= max(n, 1)
                yaw, last, t0 = 0.0, time.time(), time.time()
                while time.time() - t0 < 1.0:
                    ser.write(json.dumps(
                        {"T": 1, "L": round(lp, 3),
                         "R": round(rp, 3)}).encode() + b"\n")
                    z = mpu.gyro_z_dps()
                    now = time.time()
                    if z is not None:
                        yaw += (z - bias) * (now - last) * _ZS
                    last = now
                    time.sleep(0.02)
                ser.write(b'{"T":1,"L":0,"R":0}\n')
                ser.write(b'{"T":1,"L":0,"R":0}\n')
                d = "يمين" if yaw > 15 else ("يسار" if yaw < -15 else "≈بلا دوران")
                res.append((name, yaw))
                print(f"    {name}: {yaw:+7.1f}° ⇒ {d}")
                time.sleep(1.5)                    # خمود بين الأنماط
            y_pm, y_mp, y_pp, y_mm = (r[1] for r in res)
            print("\n  ── قراءة الجدول ──")
            if abs(y_pp) > 25 or abs(y_mm) > 25:
                print("  🔴 أمر **متماثل** يُدير الروبوت ⇒ جانب كامل ضعيف/ميت "
                      "(أسلاك أو درايفر جانب) — لا يصلحه أي ثابت في config.")
            if y_pm * y_mp > 0 and (abs(y_pm) > 15 or abs(y_mp) > 15):
                print("  🔴 النمطان المتعاكسان يدوران بنفس الجهة ⇒ فيرموير "
                      "غير خطي مع الإشارة السالبة — أرسل الجدول كاملاً.")
            print("  أرسل الأسطر الأربعة كما هي — الثوابت تُشتق منها مباشرة.")
            return 0

        # ── 3) أمر حركة فعلي ─────────────────────────────────────
        if a.spin:
            p = min(abs(a.power), MAX_MOTOR_POWER)
            print(f"\n[3/3] ⚠ دوران بالمكان 1ث بقوة {p} — أفرغ حوله (3ث)…")
            time.sleep(3.0)
            t0 = time.time()
            while time.time() - t0 < 1.0:
                ser.write(json.dumps(
                    {"T": 1, "L": p, "R": -p}).encode() + b"\n")
                time.sleep(0.1)
            ser.write(b'{"T":1,"L":0,"R":0}\n')   # إيقاف مضمون
            ser.write(b'{"T":1,"L":0,"R":0}\n')
            ans = input("  هل دارت العجلات فعلاً؟ [y/n] ").strip().lower()
            if ans.startswith("y"):
                print("  ✅ الوصلة والمحركات سليمتان معاً — عطل «لم يتحرك» "
                      "كان في طبقة أعلى (جسر/قوة/heartbeat)")
            else:
                print("  🔴 الوصلة تردّ والمحركات صامتة ⇒ العطل بعد ESP32:")
                print("     تغذية المحركات · الدرايفر · حماية جهد في الفيرموير")
        else:
            print("\n[3/3] ⏭ (أضف --spin لاختبار المحركات مباشرة عبر الوصلة)")
        print("\n✅ الوصلة ثنائية الاتجاه سليمة.")
        return 0
    finally:
        try:
            ser.write(b'{"T":1,"L":0,"R":0}\n')  # لا نترك أمراً معلّقاً
            ser.close()
        except Exception:                     # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
