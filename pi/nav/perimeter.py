# -*- coding: utf-8 -*-
"""
perimeter.py — تتبّع المحيط لتحديد نطاق الغرفة (منطق خالص)
============================================================
سلوك يسبق المسح الداخلي: يتبع الروبوت جداراً على **جهة ثابتة** ويقيس
أضلاع الغرفة فعلياً بدل الاعتماد على أبعاد مُدخلة يدوياً.

    ابحث عن جدار → اتبعه بمسافة ثابتة (وصحّح الاتجاه أثناءه)
    → جدار أمامي ⇒ سجّل طول الضلع + لفّ 90° → أربع لفّات = دورة

⚠ **الجهة ثابتة طوال الدورة** (يميناً افتراضياً): قاعدة بسيطة تمنع الالتباس
   عند الزوايا الداخلية والخارجية.

⚠ **اكتمال الدورة بعدّ اللفّات لا بالموضع** — أصدق مع موضع تقديري: الروبوت
   قد لا يعود إلى نقطة البداية بالضبط، وانتظار ذلك يجعل الدورة لا تنتهي.

⚠ **«لم أجد جداراً» ليس فشلاً**: يُكمل الضلع بالمسافة المقدَّرة ويُوسم
   `no_reference` — ويُرفع فيه شكّ الاتجاه والموضع، لأن الضلع بلا مرجع
   يعني عودةً إلى الحساب المفتوح تماماً.

⚠ **الفتحات لا تُدخَل الآن**: قفزة مفاجئة في المسافة الجانبية = باب أو
   ممر. تُسجَّل ويُكمل المحيط — الدخول قرار مسح لا قرار تتبّع.

منطق خالص بلا عتاد: يستقبل قراءات ويُصدر أوامر، فيُختبر كاملاً على ويندوز.
"""
from __future__ import annotations

import time

from pi.config import (
    PERIMETER_WALL_SIDE, PERIMETER_TARGET_CM, PERIMETER_KP,
    PERIMETER_TURNS_FOR_CYCLE, PERIMETER_MAX_SIDE_M, PERIMETER_MAX_CYCLE_S,
    PERIMETER_OPENING_JUMP_CM, PERIMETER_DIM_TOLERANCE_M,
    WALL_FOLLOW_MIN_CM, WALL_FOLLOW_MAX_CM, STOP_CM,
)

# أوامر المخرَج — تنفّذها الملاحة
FOLLOW, TURN, DONE, ABORT, SEEK = "follow", "turn", "done", "abort", "seek"


class PerimeterTracker:
    """
    آلة حالة تتبّع المحيط. `step(...)` بعد كل تقدّم، وتنفيذ ما تُعيده.

    الحالة الداخلية أضلاع مقاسة + لفّات + فتحات — كلها تُخرَج في `summary()`.
    """

    def __init__(self, side: str = PERIMETER_WALL_SIDE,
                 target_cm: float = PERIMETER_TARGET_CM,
                 turns_for_cycle: int = PERIMETER_TURNS_FOR_CYCLE):
        self.side = side
        self.target_cm = float(target_cm)
        self.turns_for_cycle = int(turns_for_cycle)
        self.sides: list[dict] = []       # الأضلاع المكتملة
        self.openings: list[dict] = []    # الفتحات المسجَّلة
        self.turns = 0
        self.started_ts = time.time()
        self.side_start_odom = 0.0
        self.side_has_reference = False   # هل رأينا جداراً في هذا الضلع؟
        self.side_samples = 0
        self._last_side_cm = None
        self.events: list[dict] = []
        self.finished = False
        self.abort_reason = None

    # ── مساعدات ──────────────────────────────────────────────────
    def _log(self, kind: str, msg: str) -> None:
        self.events.append({"kind": kind, "msg": msg,
                            "t": round(time.time() - self.started_ts, 1)})

    def _close_side(self, odom_m: float, front_cm=None) -> dict:
        """
        يُغلق الضلع. ⚠ **الطول ليس المسافة المقطوعة وحدها**: الروبوت يتوقف
        قبل الجدار بمسافة `front_cm`، فالبُعد الحقيقي = المقطوع + تلك الفجوة.
        إغفالها يُنقص كل ضلع بمقدار ثابت (~0.4م) فتخرج الغرفة أصغر من واقعها
        وتُطلق مقارنةُ الأبعاد تحذيراً كاذباً.

        ⚠ ويبقى تقريب من الدرجة الثانية غير مُعوَّض: الضلع يبدأ أيضاً مزاحاً
           عن الجدار السابق. أثره أصغر ويُترك معلَناً بدل تعويضه بتخمين —
           ولهذا `PERIMETER_DIM_TOLERANCE_M` يستوعبه.
        """
        travelled = max(0.0, float(odom_m) - self.side_start_odom)
        gap_m = (float(front_cm) / 100.0) if front_cm is not None else 0.0
        length = travelled + gap_m
        rec = {"index": len(self.sides) + 1, "length_m": round(length, 3),
               "has_reference": self.side_has_reference,
               "samples": self.side_samples}
        self.sides.append(rec)
        self._log("side_done",
                  f"الضلع {rec['index']}: {rec['length_m']:.2f}م"
                  + ("" if self.side_has_reference else
                     " ⚠ **بلا مرجع جدار** — شكّ الاتجاه والموضع مرفوعان"))
        self.side_start_odom = float(odom_m)
        self.side_has_reference = False
        self.side_samples = 0
        self._last_side_cm = None
        return rec

    # ── الخطوة ───────────────────────────────────────────────────
    def step(self, side_cm, front_cm, odom_m: float) -> dict:
        """
        `side_cm` المسافة الجانبية العمودية · `front_cm` الأمامية ·
        `odom_m` المسافة التراكمية المقطوعة.
        """
        if self.finished:
            return {"command": DONE, "reason": "الدورة اكتملت"}
        elapsed = time.time() - self.started_ts
        if elapsed > PERIMETER_MAX_CYCLE_S:
            return self._abort(f"تجاوزت الدورة مهلتها ({elapsed:.0f}ث) — "
                               f"فتحة كبيرة أو فقدان جدار")
        side_len = float(odom_m) - self.side_start_odom
        if side_len > PERIMETER_MAX_SIDE_M:
            return self._abort(f"الضلع تجاوز {PERIMETER_MAX_SIDE_M:.0f}م "
                               f"({side_len:.1f}م) — فُقد الجدار أو فتحة كبيرة")

        # ① جدار أمامي ⇒ نهاية ضلع
        if front_cm is not None and front_cm <= max(STOP_CM, self.target_cm):
            self._close_side(odom_m, front_cm)
            self.turns += 1
            if self.turns >= self.turns_for_cycle:
                self.finished = True
                self._log("cycle_done",
                          f"دورة كاملة — {self.turns} لفّات و"
                          f"{len(self.sides)} أضلاع")
                return {"command": DONE, "turns": self.turns,
                        "reason": "أربع لفّات = دورة كاملة"}
            # الجهة تحدّد اتجاه اللفّ: جدار يمين ⇒ نلفّ يساراً حوله
            deg = -90.0 if self.side == "right" else 90.0
            self._log("turn", f"جدار أمامي على {front_cm:.0f}سم ⇒ لفّ {deg:+.0f}°")
            return {"command": TURN, "turn_deg": deg, "turns": self.turns,
                    "reason": f"جدار مقابل على {front_cm:.0f}سم"}

        # ② لا قراءة جانبية ⇒ ابحث عن جدار (الضلع بلا مرجع حتى الآن)
        if side_cm is None or not (WALL_FOLLOW_MIN_CM <= side_cm <= WALL_FOLLOW_MAX_CM):
            prev = self._last_side_cm
            self._last_side_cm = None
            if prev is not None:
                # 🔴 قفزة مفاجئة من قراءة صالحة إلى لا شيء = **فتحة**
                op = {"at_odom_m": round(float(odom_m), 2),
                      "side_index": len(self.sides) + 1,
                      "from_cm": round(prev, 1)}
                self.openings.append(op)
                self._log("opening",
                          f"فتحة/باب عند {op['at_odom_m']}م — سُجّلت "
                          f"**ولا تُدخَل الآن** (المحيط أولاً)")
            return {"command": SEEK, "reason": "لا جدار في المدى — أكمل مقدَّراً",
                    "no_reference": True}

        # ③ اتباع بمسافة ثابتة — تحكّم تناسبي على الخطأ الجانبي
        self.side_has_reference = True
        self.side_samples += 1
        if self._last_side_cm is not None:
            jump = abs(side_cm - self._last_side_cm)
            if jump >= PERIMETER_OPENING_JUMP_CM:
                op = {"at_odom_m": round(float(odom_m), 2),
                      "side_index": len(self.sides) + 1,
                      "jump_cm": round(jump, 1)}
                self.openings.append(op)
                self._log("opening",
                          f"قفزة {jump:.0f}سم عند {op['at_odom_m']}م = فتحة — "
                          f"سُجّلت ولا تُدخَل")
        self._last_side_cm = float(side_cm)
        err_cm = float(side_cm) - self.target_cm      # موجب = بعيد عن الجدار
        # موجب ⇒ اقترب من الجدار: جهة اليمين تعني انحرافاً يميناً
        steer = PERIMETER_KP * err_cm * (1.0 if self.side == "right" else -1.0)
        return {"command": FOLLOW, "steer": round(steer, 4),
                "error_cm": round(err_cm, 1), "side_cm": round(side_cm, 1),
                "reason": f"اتباع على {side_cm:.0f}سم (هدف {self.target_cm:.0f})"}

    def _abort(self, reason: str) -> dict:
        self.finished = True
        self.abort_reason = reason
        self._log("abort", f"⛔ {reason}")
        return {"command": ABORT, "reason": reason}

    # ── الحصيلة ──────────────────────────────────────────────────
    def measured_dimensions(self):
        """
        (العرض، الطول) بالمتر من الأضلاع المقاسة، أو None قبل اكتمال الدورة.
        الأضلاع المتقابلة تُتوسَّط — وفارقها مؤشّر جودة بحدّ ذاته.
        """
        if len(self.sides) < 4:
            return None
        a = [s["length_m"] for s in self.sides[:4]]
        return (round((a[0] + a[2]) / 2.0, 2), round((a[1] + a[3]) / 2.0, 2))

    def compare_to(self, width_m: float, length_m: float) -> dict:
        """
        **فحص ذاتي مجاني**: الأبعاد المقاسة مقابل المُدخلة.
        تطابق تقريبي ⇒ الملاحة موثوقة. اختلاف كبير ⇒ انزلاق أو انحراف
        — ويُعلَن **قبل** بدء المسح بدل اكتشافه بعد مهمة كاملة.
        """
        dims = self.measured_dimensions()
        if dims is None:
            return {"ok": False, "reason": "الدورة لم تكتمل — لا أبعاد مقاسة"}
        mw, ml = dims
        # الترتيب قد ينعكس حسب نقطة البدء — نأخذ أفضل مطابقة
        d_direct = abs(mw - width_m) + abs(ml - length_m)
        d_swap = abs(mw - length_m) + abs(ml - width_m)
        if d_swap < d_direct:
            mw, ml = ml, mw
        dw, dl = abs(mw - width_m), abs(ml - length_m)
        worst = max(dw, dl)
        ok = worst <= PERIMETER_DIM_TOLERANCE_M
        no_ref = [s["index"] for s in self.sides if not s["has_reference"]]
        return {
            "ok": ok, "measured": (mw, ml), "declared": (width_m, length_m),
            "delta_m": (round(dw, 2), round(dl, 2)),
            "tolerance_m": PERIMETER_DIM_TOLERANCE_M,
            "sides_without_reference": no_ref,
            "reason": (f"✅ الأبعاد المقاسة {mw}×{ml}م تطابق المُدخلة "
                       f"ضمن {PERIMETER_DIM_TOLERANCE_M}م — الملاحة موثوقة"
                       if ok else
                       f"⚠ **فارق {worst:.2f}م** بين المقاس {mw}×{ml}م "
                       f"والمُدخل {width_m}×{length_m}م — انزلاق أو انحراف "
                       f"اتجاه. راجع قبل بدء المسح."),
        }

    def summary(self) -> dict:
        return {"side": self.side, "turns": self.turns,
                "sides": list(self.sides), "openings": list(self.openings),
                "measured": self.measured_dimensions(),
                "finished": self.finished, "abort_reason": self.abort_reason,
                "elapsed_s": round(time.time() - self.started_ts, 1),
                "events": self.events[-20:]}
