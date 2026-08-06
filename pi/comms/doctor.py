# -*- coding: utf-8 -*-
"""
doctor.py — «أداة خارجية»: تشخيص أمر قيادة واحد من طرفه لطرفه
================================================================
يجيب بدقة عن أسئلة «الروبوت لا يتحرك»: هل الأمر وصل البوابة؟ بأي نمط
وجسر؟ ماذا قررت طبقة السلامة ولماذا؟ وما القيم التي خرجت فعلاً للمنفذ؟

الاستعمال (والسيرفر شغّال):
    python3 -m pi.comms.doctor            # يشخّص أمر FWD
    python3 -m pi.comms.doctor BACK       # أو أي أمر من القائمة المغلقة
    python3 -m pi.comms.doctor FWD 0.4    # بقوة محددة

⚠ يتكلم مع السيرفر عبر HTTP (لا يفتح المنفذ التسلسلي بنفسه) — فهو آمن
   أثناء عمل كل شيء ولا يتنازع مع أحد على العتاد.
"""
from __future__ import annotations

import json
import sys
import urllib.request

BASE = "http://localhost:8000"


def _get(path: str) -> dict:
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.load(r)


def _post(path: str, body: dict) -> dict:
    req = urllib.request.Request(
        BASE + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r)


def _yesno(v) -> str:
    return {True: "نعم", False: "لا", None: "مجهول"}.get(v, str(v))


def main() -> int:
    cmd = (sys.argv[1] if len(sys.argv) > 1 else "FWD").upper()
    power = float(sys.argv[2]) if len(sys.argv) > 2 else None

    try:
        st = _get("/api/sim/status")
    except Exception as e:                       # noqa: BLE001
        print(f"⛔ السيرفر لا يستجيب على {BASE} — شغّله أولاً ({e})")
        return 1

    ui, man, rov = st.get("ui_mode", {}), st.get("manual", {}), st.get("rover", {})
    sens = st.get("sensors", {})
    print("═══ ما قبل الأمر ═══")
    print(f"  النمط: {ui.get('mode_ar')} ({ui.get('mode')})"
          f" · بوابة القيادة: {'مفتوحة' if man.get('enabled') else 'مغلقة'}")
    print(f"  🔴 جسر الروفر: {rov.get('mode')}"
          + (" — المحركات الحقيقية **لن تتحرك** على جسر sim!"
             if rov.get("mode") != "real" else " ✅")
          + (f" · خطأ الجسر: {rov.get('error')}" if rov.get("error") else ""))
    print(f"  حالة المهمة: {st.get('state')} · محركات القيادة الذاتية: "
          f"{_yesno(st.get('drive_motors'))}")
    print(f"  المسافة الأمامية: "
          + (f"{sens.get('ultrasonic_cm')} سم" if sens.get('ultrasonic_cm') is not None
             else "مجهولة (الغائب مجهول لا خالٍ — قد تُقصّ السرعة أو يُرفض التقدّم)"))
    print(f"  IR (يسار/وسط/يمين): {sens.get('ir_left')}/{sens.get('ir_mid')}/{sens.get('ir_right')}")

    print(f"\n═══ إرسال {cmd}" + (f" بقوة {power}" if power else "") + " ═══")
    body = {"cmd": cmd}
    if power is not None:
        body["power"] = power
    try:
        res = _post("/api/manual/command", body)
    except Exception as e:                       # noqa: BLE001
        print(f"⛔ فشل النداء نفسه: {e}")
        return 1

    ok = res.get("ok")
    print(f"  القرار: {'✅ قُبل' if ok else '⛔ رُفض'} · ack={res.get('ack')}")
    print(f"  السبب: {res.get('reason')}")
    if res.get("safety"):
        s = res["safety"]
        print(f"  طبقة السلامة: allow={_yesno(s.get('allow'))}"
              + (f" · سقف السرعة={s.get('speed_cap')}" if s.get('speed_cap') is not None else "")
              + (f" · درجة السلّم={s.get('rung')}" if s.get('rung') else "")
              + (f" · مجهول={s.get('unknown')}" if s.get('unknown') else ""))
    if res.get("power") is not None:
        print(f"  القوة بعد القصّ: {res['power']}")
    rv = res.get("rover", {})
    lr = rv.get("cmd_lr")
    print(f"  🔴 ما خرج للجسر فعلاً: L={lr[0] if lr else '؟'} R={lr[1] if lr else '؟'}"
          f" · الجسر: {rv.get('mode')} · الوصلة سليمة: {_yesno(rv.get('link_ok'))}")

    print("\n═══ الخلاصة ═══")
    if ok and rv.get("mode") == "real" and lr and any(abs(v) > 0 for v in lr):
        print("  الأمر خرج للعتاد الحقيقي بقيم غير صفرية — إن لم يتحرك الروبوت")
        print("  فالمشكلة بعد المنفذ: كبل الروفر، فيرموير Wave Rover، أو التغذية.")
    elif ok and rv.get("mode") != "real":
        print("  🔴 الأمر «نجح» لكن على جسر محاكاة — الروبوت الحقيقي لن يتحرك.")
        print("  الحل: أعد تشغيل السيرفر هكذا:")
        print("    RMS_ROVER_MODE=real python3 -m uvicorn pi.web.server:app --host 0.0.0.0 --port 8000")
        print("  أو بدّل حيّاً: curl -X POST localhost:8000/api/rover/mode "
              "-H 'Content-Type: application/json' -d '{\"mode\":\"real\"}'")
    elif not ok:
        print(f"  الأمر رُفض قبل المحركات — السبب أعلاه هو التفسير الكامل: {res.get('ack')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
