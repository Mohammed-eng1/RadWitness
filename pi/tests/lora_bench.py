# -*- coding: utf-8 -*-
"""
lora_bench.py — «أداة خارجية»: مقعد اختبار اللورا الكامل بلا راسبري
=====================================================================
يشغّل **المكدس الحقيقي نفسه** (MissionSim + بوابة ManualControl +
LoRaLink بالبروتوكول والإقرارات) فوق محوّل USB-TTL على أي حاسوب —
فتتعامل شاشة CYD معه كأنه الروبوت: تستقبل تيليمتري دورياً (LoRa OK
وقراءات على شاشتها) وترتد إقرارات أزرارها (STOP OK · FWD BUSY…).

الغاية: إغلاق اختبار قبول قناة الطوارئ من المكتب والروبوت يشحن —
كل شيء حقيقي إلا المحركات (جسر محاكاة، فلا حركة فيزيائية بالتعريف).

الاستعمال (ويندوز أو لينكس):
    python pi/tests/lora_bench.py COM5            # منفذ المحوّل
    python pi/tests/lora_bench.py COM5 --drive    # يفتح بوابة القيادة
                                                  # فيرد FWD بـ OK لا BUSY
    python -m serial.tools.list_ports -v          # لمعرفة اسم المنفذ

المتوقع على الشاشة خلال ~8 ثوانٍ: NO LINK → LoRa OK + قراءات المحاكاة.
ثم اضغط أزرارها وراقب السطرين (هنا وعلى الشاشة):
    STOP  → OK دائماً        FWD  → BUSY (بلا --drive) أو OK (معه)
    WDRAW → BUSY (لا مهمة نشطة — سلوك صحيح)
"""
from __future__ import annotations

import pathlib
import sys
import time

# تشغيل مباشر `python pi/tests/lora_bench.py` يضع مجلد tests لا جذر
# المستودع على المسار — أضف الجذر كي تعمل استيرادات `pi.*`
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

# طباعة آمنة على طرفية ويندوز cp1256 (الرموز تُستبدل لا تُسقط الأداة)
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:                                  # noqa: BLE001
    pass

from pi.nav.mission import MissionSim              # noqa: E402
from pi.comms.control import ManualControl        # noqa: E402
from pi.comms.lora import LoRaLink                # noqa: E402


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    drive = "--drive" in sys.argv
    port = args[0] if args else "COM3"

    print(f"مقعد اختبار اللورا — المنفذ {port} @ 9600"
          + (" · بوابة القيادة مفتوحة (--drive)" if drive else ""))
    print("المكدس حقيقي بالكامل عدا المحركات (جسر محاكاة). Ctrl-C للإيقاف.\n")

    mission = MissionSim()
    gate = ManualControl(mission)
    if drive:
        gate.set_enabled(True)
    link = LoRaLink(mission, gate, port=port, baud=9600, enabled=True)
    st = link.start()
    if not st["ok"]:
        print(f"تعذر فتح الوصلة: {st['error']}")
        print("تلميح: python -m serial.tools.list_ports -v لمعرفة اسم المنفذ،")
        print("وتأكد ألا يمسكه برنامج آخر (شاشة تسلسلية/Arduino Serial Monitor).")
        return 1
    print(f"الوصلة مفتوحة على {port} — الشاشة يجب أن تتحول إلى LoRa OK "
          "خلال ~8 ثوانٍ (تيليمتري كل ثانيتين).\n")

    last = {}
    try:
        while True:
            time.sleep(1.0)
            s = link.state()
            snap = (s["frames_rx"], s["frames_bad"], s["echoes"],
                    s["commands_ok"], gate.last_cmd, gate.last_reason)
            if snap != last.get("snap"):
                last["snap"] = snap
                print(f"مستقبلة={s['frames_rx']} تالفة={s['frames_bad']} "
                      f"صدى={s['echoes']} أوامر مقبولة={s['commands_ok']}"
                      + (f" | آخر أمر: {gate.last_cmd} — {gate.last_reason}"
                         if gate.last_cmd else ""))
            # مهلة أوامر الراديو تعمل هنا أيضاً — نفس حارس السيرفر
            gate.poll()
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()

    s = link.state()
    print("\n─── الخلاصة ─────────────────────────────────")
    print(f"  مستقبلة: {s['frames_rx']} · تالفة: {s['frames_bad']}"
          f" · صدى: {s['echoes']} · أوامر مقبولة: {s['commands_ok']}")
    if s["commands_ok"] > 0:
        print("نجاح: أوامر الشاشة عبرت البروتوكول والبوابة وارتدت إقراراتها —")
        print("قناة الطوارئ مثبتة من طرفها لطرفها (بقي فقط تكرارها مرة على الراسبري).")
    elif s["frames_rx"] > 0:
        print("وصلت أطر لكن لم يُقبل أمر — راجع الأسباب المطبوعة أعلاه.")
    else:
        print("لا شيء وصل: راجع توصيلة HC-14 بالمحوّل (تقاطع TX/RX) والقناة.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
