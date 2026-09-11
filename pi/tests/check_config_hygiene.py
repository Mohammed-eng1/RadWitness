# -*- coding: utf-8 -*-
"""
check_config_hygiene.py — حارس نظافة الإعدادات (منطق خالص، بلا عتاد)
======================================================================
**أداة خارجية** (سكربت تشخيص — البند 8) — ويُستدعى أيضاً من
`pi/nav/selftest.py` فيصير جزءاً من شبكة الأمان لا اختياراً.

ثلاثة فحوص، كلٌّ منها بعد خلل حقيقي:

① 🐞 **ثابت مُعرَّف مرتين**: وُجد `GYRO_SCALE` مكرّراً بنفس القيمة
   **والثاني يفوز صامتاً**. فمن يعايره — وهو أول ما يُعاير على منصّة
   جديدة — كان سيغيّر رقماً **لا يُقرأ**، ويرى نتيجة لا تتغيّر، ويستنتج
   أن الحسّاس معطوب. الفحص يمنع عودته.

② 🔴 **إرث منصّة ميتة**: هذا الفرع منصّة Freenove وحدها. أي عودة لثوابت
   Wave Rover/ESP32 تعني قيداً عتادياً ميتاً يُطبَّق على عتاد حيّ.

③ ⚠ **ثابت مشتقّ كُتب يدوياً**: `HEADING_SPIKE_DPS_STEER` تُشتقّ من سقف
   القوة. كتابتها رقماً تجعلها تتخلّف عنه حين يُعاد قياسه، فيرفض المرشّح
   دوران الروبوت الذي أمر به المتحكّم نفسه (البند 1.1.3).

الاستعمال: `python3 -m pi.tests.check_config_hygiene`
"""
from __future__ import annotations

import collections
import os
import re
import sys

CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "config.py")

#: ثوابت منصّة ميتة — عودتها تعني إرثاً عتادياً يُطبَّق على عتاد حيّ
DEAD_PLATFORM_NAMES = (
    "ESP32_BOOT_WAIT_S", "ESP32_STUCK_ZERO_BYTES", "ESP32_STUCK_RETRY_S",
    "ESP32_STUCK_RETRIES", "ROVER_PORT", "ROVER_BAUD", "ROVER_LINK_REOPEN_S",
)

#: ثوابت **مشتقّة**: يجب ألّا تُسنَد قيمة حرفية
DERIVED_NAMES = ("HEADING_SPIKE_DPS_STEER",)

_ASSIGN = re.compile(r"^([A-Z_][A-Z0-9_]*)\s*=")
_LITERAL = re.compile(r"^[A-Z_][A-Z0-9_]*\s*=\s*-?[\d.]+\s*(#.*)?$")


def duplicate_constants(text: str) -> dict:
    """يعيد {الاسم: [أرقام الأسطر]} لكل ثابت مُعرَّف أكثر من مرة."""
    seen = collections.defaultdict(list)
    for i, line in enumerate(text.split("\n"), 1):
        m = _ASSIGN.match(line)
        if m:
            seen[m.group(1)].append(i)
    return {k: v for k, v in seen.items() if len(v) > 1}


def dead_platform_constants(text: str) -> dict:
    """ثوابت المنصّة الميتة إن عادت — {الاسم: رقم السطر}."""
    out = {}
    for i, line in enumerate(text.split("\n"), 1):
        m = _ASSIGN.match(line)
        if m and m.group(1) in DEAD_PLATFORM_NAMES:
            out[m.group(1)] = i
    return out


def hardcoded_derived(text: str) -> dict:
    """ثابت مشتقّ أُسنِدت له قيمة حرفية — {الاسم: رقم السطر}."""
    out = {}
    for i, line in enumerate(text.split("\n"), 1):
        m = _ASSIGN.match(line)
        if m and m.group(1) in DERIVED_NAMES and _LITERAL.match(line.strip()):
            out[m.group(1)] = i
    return out


def run(text: str = None) -> list:
    """يعيد قائمة المشاكل (فارغة = نظيف). منطق خالص — يُستدعى من selftest."""
    if text is None:
        with open(CONFIG_PATH, encoding="utf-8") as f:
            text = f.read()
    problems = []
    for name, lines in sorted(duplicate_constants(text).items()):
        problems.append(
            f"🐞 `{name}` مُعرَّف {len(lines)} مرات (الأسطر {lines}) — "
            f"الأخير يفوز صامتاً، فمن يعايره يغيّر رقماً لا يُقرأ")
    for name, ln in sorted(dead_platform_constants(text).items()):
        problems.append(
            f"🔴 `{name}` (س{ln}) ثابت منصّة ميتة — قيد عتادي لا يخصّ Freenove")
    for name, ln in sorted(hardcoded_derived(text).items()):
        problems.append(
            f"⚠ `{name}` (س{ln}) **مشتقّ** وكُتب رقماً — سيتخلّف عن سقف "
            f"القوة حين يُعاد قياسه (البند 1.1.3)")
    return problems


def main() -> int:
    problems = run()
    if not problems:
        print("✅ إعدادات نظيفة: لا تكرار · لا إرث منصّة ميتة · "
              "لا ثابت مشتقّ مكتوب يدوياً")
        return 0
    print(f"🔴 {len(problems)} مشكلة في pi/config.py:\n")
    for p in problems:
        print(f"  {p}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
