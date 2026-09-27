# -*- coding: utf-8 -*-
"""
lidar_avoid.py — قرار التفادي من لفّة ليدار واحدة (منطق خالص، بلا عتاد)
========================================================================
المدخل: نقاط بإطار الروبوت من `lidar_c1.to_robot_frame` (a زاوية يسار موجب،
x أمام، y يسار). المخرج: قرار واحد (dict) لحلقة `lidar_drive`.

الترتيب:
1. **تصفية**: القناع (أجزاء الروبوت) · النقطة المنفردة (لا جارة خلال 2° و5سم
   = ضجيج). ما يقع **داخل** مستطيل الروبوت ولم يحجبه القناع = عائق ملاصق
   (خلوص 0) لا يُحذف — ويُعدّ في `inside` للتحذير.
2. **الخلوص من حافة الروبوت** (مستطيل ROBOT_LENGTH_M × ROBOT_WIDTH_M) لا من
   الليدار: جسم على الجانب يبعد عن الليدار 20سم قد يلامس الجنزير.
3. **الأمام = ممرّ السير** (عرض الروبوت + هامش) لا مخروط ±30°: مخروط ±30°
   يوقف الروبوت لجدار جانبي لا يقع في طريقه. والقطاعات الستة تُحسب للعرض
   والخلف والجوانب.
4. **السرعة** من سلّم السلامة القائم (`SPEED_LADDER` + `STOP_CM`) مقيَّساً إلى
   سقف هذا المسار، بأرضية أدنى سرعة تتحرّك فعلاً.
5. **أوسع فتحة** في −90..+90 بخانات 5°: قريبة ⇒ قوس، بعيدة ⇒ لفّ بالمكان.
6. **مسدود** ⇒ رجوع إن كان الخلف فاضياً ثم لفّ؛ بعد ESCAPE_MAX_ATTEMPTS ⇒ «محصور».
"""
from __future__ import annotations

import bisect
import math

from pi.config import (
    ROBOT_LENGTH_M, ROBOT_WIDTH_M, LIDAR_NEIGHBOR_DEG, LIDAR_NEIGHBOR_M,
    LIDAR_CORRIDOR_MARGIN_M, LIDAR_FRONT_HALF_DEG, LIDAR_SIDE_FROM_DEG,
    LIDAR_BACK_FROM_DEG, LIDAR_MAX_SPEED, LIDAR_MIN_SPEED, LIDAR_AVOID_START_M,
    LIDAR_GAP_RANGE_DEG, LIDAR_GAP_BIN_DEG, LIDAR_GAP_FREE_M, LIDAR_GAP_MIN_BINS,
    LIDAR_ARC_MAX_DEG, LIDAR_ARC_GAIN, LIDAR_BACK_CLEAR_M, LIDAR_ESCAPE_RESET_S,
    LIDAR_BLOCK_CONFIRM_SCANS,
    SPEED_LADDER, STOP_CM, ESCAPE_MAX_ATTEMPTS, TURN_MIN_ACHIEVABLE_DEG,
)

SECTORS = ("front", "front_left", "left", "back", "right", "front_right")
SECTOR_AR = {"front": "أمام", "front_left": "أمام-يسار", "left": "يسار",
             "back": "خلف", "right": "يمين", "front_right": "أمام-يمين"}


# ═══ القناع ═══════════════════════════════════════════════════════
def mask_set(mask: dict | None) -> set:
    """زوايا محجوبة (أعداد صحيحة −180..179 بإطار الليدار في الروبوت)."""
    if not mask:
        return set()
    return {int(d) for d in mask.get("blocked_deg", [])}


def mask_front_overlap(blocked: set, half_deg: float = LIDAR_FRONT_HALF_DEG) -> list:
    """زوايا القناع داخل قطاع الأمام — تحذير (قناع يعمي الأمام)."""
    return sorted(d for d in blocked if abs(d) <= half_deg)


def _deg_key(la: float) -> int:
    return int(math.floor(la + 0.5 + 180.0)) % 360 - 180


# ═══ الهندسة ═══════════════════════════════════════════════════════
def edge_clearance(x: float, y: float, length: float = ROBOT_LENGTH_M,
                   width: float = ROBOT_WIDTH_M) -> float:
    """بُعد النقطة عن **حافة** مستطيل الروبوت (0 = داخله أو عليه)."""
    dx = max(abs(x) - length / 2.0, 0.0)
    dy = max(abs(y) - width / 2.0, 0.0)
    return math.hypot(dx, dy)


def sector_of(a: float) -> str:
    aa = abs(a)
    if aa <= LIDAR_FRONT_HALF_DEG:
        return "front"
    if aa <= LIDAR_SIDE_FROM_DEG:
        return "front_left" if a > 0 else "front_right"
    if aa <= LIDAR_BACK_FROM_DEG:
        return "left" if a > 0 else "right"
    return "back"


def drop_isolated(points: list, nb_deg: float = LIDAR_NEIGHBOR_DEG,
                  nb_m: float = LIDAR_NEIGHBOR_M) -> tuple[list, int]:
    """يُبقي النقطة إن كان لها جارة خلال `nb_deg` و`nb_m` (مع التفاف 360°)."""
    if len(points) < 2:
        return [], len(points)
    pts = sorted(points, key=lambda p: p["a"])
    angs = [p["a"] for p in pts]
    n = len(pts)
    keep = []
    for i, p in enumerate(pts):
        ok = False
        for shift in (0.0, 360.0, -360.0):
            lo = bisect.bisect_left(angs, p["a"] + shift - nb_deg)
            hi = bisect.bisect_right(angs, p["a"] + shift + nb_deg)
            for j in range(lo, hi):
                if j != i and abs(pts[j]["r"] - p["r"]) <= nb_m:
                    ok = True
                    break
            if ok:
                break
        if ok:
            keep.append(p)
    return keep, n - len(keep)


def filter_points(points: list, blocked: set) -> tuple[list, dict]:
    """القناع ثم المنفردة؛ داخل-الروبوت يبقى بخلوص 0. ⇒ (النقاط، العدّادات)."""
    stats = {"raw": len(points), "masked": 0, "inside": 0, "isolated": 0}
    kept = []
    for p in points:
        if blocked and _deg_key(p["la"]) in blocked:
            stats["masked"] += 1
            continue
        c = edge_clearance(p["x"], p["y"])
        if c <= 0.0:
            # 🔴 **عائق لا جزء من الروبوت**: مقاس 2026-09-27 — كوب أمام
            #    الليدار بـ~10سم وقع داخل المستطيل المفترض (الليدار ليس في
            #    المركز) فأُسقط، وحجب الجدار خلفه ⇒ «الممرّ خالٍ» والكوب
            #    أمامه. أجزاء الروبوت يحجبها **القناع** المعاير؛ ما بقي داخل
            #    المستطيل يُعامل ملاصقاً (خلوص 0) ويُعدّ للتحذير.
            stats["inside"] += 1
        kept.append(dict(p, c=c))
    kept, stats["isolated"] = drop_isolated(kept)
    stats["used"] = len(kept)
    return kept, stats


def sector_clearance(points: list) -> dict:
    """أدنى خلوص من الحافة لكل قطاع (None = لا نقطة = مجهول لا «خالٍ»)."""
    out = {s: None for s in SECTORS}
    for p in points:
        s = sector_of(p["a"])
        if out[s] is None or p["c"] < out[s]:
            out[s] = p["c"]
    return out


def corridor_clearance(points: list, length: float = ROBOT_LENGTH_M,
                       width: float = ROBOT_WIDTH_M,
                       margin: float = LIDAR_CORRIDOR_MARGIN_M,
                       forward: bool = True):
    """أقرب نقطة داخل ممرّ السير (أمام أو خلف) — بُعدها عن الحافة، أو None."""
    half_w = width / 2.0 + margin
    best = None
    for p in points:
        x = p["x"] if forward else -p["x"]
        # ⚠ من المركز لا من الحافة: نقطة داخل المستطيل في هذا النصف = 0
        if x >= 0.0 and abs(p["y"]) <= half_w:
            d = max(0.0, x - length / 2.0)
            if best is None or d < best:
                best = d
    return best


def ladder_speed(front_m, max_speed: float = LIDAR_MAX_SPEED,
                 min_speed: float = LIDAR_MIN_SPEED) -> float:
    """
    سلّم السلامة القائم (SPEED_LADDER بالسنتيمتر + STOP_CM) مقيَّساً: أعلى درجة
    = `max_speed`، والباقي بنسبته. None (لا شيء في الممرّ) = طريق مفتوح.
    ⚠ أرضية `min_speed`: درجة دونها لا تحرّك الروبوت = توقّف صامت.
    """
    if front_m is None:
        return max_speed
    cm = front_m * 100.0
    if cm < STOP_CM:
        return 0.0
    top = max(p for _, p in SPEED_LADDER)
    frac = SPEED_LADDER[-1][1] / top
    for threshold, pw in SPEED_LADDER:
        if cm >= threshold:
            frac = pw / top
            break
    return max(min(max_speed, min_speed), min(max_speed, max_speed * frac))


def widest_gap(points: list, rng: float = LIDAR_GAP_RANGE_DEG,
               bin_deg: float = LIDAR_GAP_BIN_DEG,
               free_m: float = LIDAR_GAP_FREE_M,
               min_bins: int = LIDAR_GAP_MIN_BINS):
    """
    أوسع سلسلة خانات حرّة في −rng..+rng. خانة بلا نقاط = حرّة (لا صدى داخل
    المدى). التعادل ⇒ الأقرب إلى الأمام. يُعيد dict أو None.
    """
    nb = int(round(2 * rng / bin_deg))
    worst = [None] * nb
    for p in points:
        if -rng <= p["a"] < rng:
            k = min(nb - 1, int((p["a"] + rng) // bin_deg))
            if worst[k] is None or p["c"] < worst[k]:
                worst[k] = p["c"]
    free = [w is None or w > free_m for w in worst]
    best = None
    i = 0
    while i < nb:
        if not free[i]:
            i += 1
            continue
        j = i
        while j < nb and free[j]:
            j += 1
        width = j - i
        center = -rng + (i + j) / 2.0 * bin_deg
        cand = (width, -abs(center))
        if width >= min_bins and (best is None or cand > best[0]):
            lo, hi = -rng + i * bin_deg, -rng + j * bin_deg
            # الهدف: أقرب زاوية إلى الأمام **داخل** الفتحة بهامش نصف عرض
            # الروبوت مرئياً على بُعد `free_m` — مركز فتحة 5..90 (47°) يلفّ
            # الروبوت نصف دورة لصندوق يلامس طرف الممرّ.
            m = min(math.degrees(math.atan2(ROBOT_WIDTH_M / 2.0
                                            + LIDAR_CORRIDOR_MARGIN_M, free_m)),
                    (hi - lo) / 2.0)
            target = min(max(0.0, lo + m), hi - m)
            best = (cand, {"angle": target, "center": center,
                           "width_deg": width * bin_deg, "from": lo, "to": hi})
        i = j
    return best[1] if best else None


# ═══ صاحب القرار (حالة محاولات التحرر) ════════════════════════════
class LidarAvoider:
    """
    `decide(points, now)` ⇒ dict فيه `action`:
      go (l, r) · arc (l, r) · spin (deg، موجب=يسار) · backup (m) ·
      stop · trapped.
    الحالة الوحيدة: عدّاد محاولات التحرر، يُصفَّر بعد سير حرّ مستمر.
    """

    def __init__(self, blocked: set | None = None, max_speed: float = LIDAR_MAX_SPEED,
                 max_attempts: int = ESCAPE_MAX_ATTEMPTS):
        self.blocked = blocked or set()
        self.max_speed = min(float(max_speed), LIDAR_MAX_SPEED) if max_speed else LIDAR_MAX_SPEED
        self.max_attempts = max_attempts
        self.attempts = 0
        self._free_since = None
        self._just_backed = False
        self._blocked_run = 0

    def reset(self) -> None:
        self._blocked_run = 0
        self.attempts = 0
        self._free_since = None
        self._just_backed = False

    def note_backed_up(self) -> None:
        """يُستدعى بعد تنفيذ الرجوع: القرار التالي لفّ لا رجوع آخر."""
        self._just_backed = True

    def decide(self, points: list, now: float) -> dict:
        pts, stats = filter_points(points, self.blocked)
        sectors = sector_clearance(pts)
        front = corridor_clearance(pts, forward=True)
        rear = corridor_clearance(pts, forward=False)
        inside = [(round(p["a"], 1), round(p["d"], 3)) for p in pts if p["c"] <= 0.0][:6]
        base = {"stats": stats, "sectors": sectors, "front_m": front, "inside_pts": inside,
                "rear_m": rear, "attempts": self.attempts}
        stop_m = STOP_CM / 100.0

        # ── طريق مفتوح أو عائق بعيد: سِر (وقوس نحو فتحة إن اقترب) ──
        if front is None or front >= stop_m:
            self._blocked_run = 0
            v = ladder_speed(front, self.max_speed)
            if front is None or front >= LIDAR_AVOID_START_M:
                self._note_free(now)
                return dict(base, action="go", l=v, r=v, speed=v,
                            reason=("الممرّ خالٍ" if front is None else
                                    f"الممرّ خالٍ حتى {front:.2f}م"))
            self._free_since = None
            gap = widest_gap(pts)
            if gap is not None and abs(gap["angle"]) <= LIDAR_ARC_MAX_DEG:
                if abs(gap["angle"]) < LIDAR_GAP_BIN_DEG:
                    return dict(base, action="go", l=v, r=v, speed=v, gap=gap,
                                reason=f"عائق {front:.2f}م والفتحة أمامنا — إبطاء")
                d = LIDAR_ARC_GAIN * math.radians(gap["angle"])
                l = max(0.0, min(self.max_speed, v - d))
                r = max(0.0, min(self.max_speed, v + d))
                return dict(base, action="arc", l=l, r=r, speed=v, gap=gap,
                            reason=f"عائق {front:.2f}م ⇒ قوس نحو فتحة "
                                   f"{gap['angle']:+.0f}° (عرض {gap['width_deg']:.0f}°)")
            if gap is not None:
                return dict(base, action="spin", deg=gap["angle"], gap=gap,
                            reason=f"عائق {front:.2f}م ⇒ لفّ بالمكان نحو فتحة "
                                   f"{gap['angle']:+.0f}°")
            # لا فتحة ولا خطر فوري: ازحف حتى حدّ التوقف
            return dict(base, action="go", l=v, r=v, speed=v,
                        reason=f"عائق {front:.2f}م ولا فتحة أوسع — زحف")

        # ── الأمام مسدود ──
        self._free_since = None
        self._blocked_run += 1
        if self._blocked_run < LIDAR_BLOCK_CONFIRM_SCANS and not self._just_backed:
            # توقف فوري، والمحاولة تُحتسب فقط إن تكرّر الانسداد (لا نقطة عابرة)
            return dict(base, action="stop",
                        reason=f"الأمام مسدود ({front:.2f}م) — توقف للتأكيد")
        gap = widest_gap(pts)
        if self._just_backed:
            self._just_backed = False
            if gap is not None and abs(gap["angle"]) >= TURN_MIN_ACHIEVABLE_DEG:
                return dict(base, action="spin", deg=gap["angle"], gap=gap,
                            reason=f"بعد الرجوع ⇒ لفّ لأوسع فتحة {gap['angle']:+.0f}°")
            side = self._open_side(sectors)
            if side is not None:
                return dict(base, action="spin", deg=side,
                            reason=f"بعد الرجوع ولا فتحة أمامية ⇒ لفّ {side:+.0f}° "
                                   f"نحو الجهة الأوسع")
        self.attempts += 1
        base["attempts"] = self.attempts
        if self.attempts > self.max_attempts:
            return dict(base, action="trapped",
                        reason=f"⛔ محصور: {self.max_attempts} محاولات تحرّر فشلت")
        rear_ok = rear is None or rear >= LIDAR_BACK_CLEAR_M
        if rear_ok:
            return dict(base, action="backup",
                        reason=f"الأمام مسدود ({front:.2f}م) والخلف فاضٍ ⇒ رجوع بطيء "
                               f"(محاولة {self.attempts}/{self.max_attempts})")
        if gap is not None and abs(gap["angle"]) >= TURN_MIN_ACHIEVABLE_DEG:
            return dict(base, action="spin", deg=gap["angle"], gap=gap,
                        reason=f"الأمام والخلف مسدودان ⇒ لفّ لفتحة {gap['angle']:+.0f}°")
        side = self._open_side(sectors)
        if side is not None:
            return dict(base, action="spin", deg=side,
                        reason=f"الأمام والخلف مسدودان ⇒ لفّ {side:+.0f}°")
        return dict(base, action="stop",
                    reason=f"مسدود من كل الجهات (محاولة {self.attempts})")

    # ── مساعدات ──
    def _note_free(self, now: float) -> None:
        if self._free_since is None:
            self._free_since = now
        elif now - self._free_since >= LIDAR_ESCAPE_RESET_S:
            self.attempts = 0

    @staticmethod
    def _open_side(sectors: dict):
        """90° نحو الجانب الأوسع (None = الجانبان أضيق من خلوص الرجوع)."""
        def val(s):
            v = sectors.get(s)
            return float("inf") if v is None else v
        left = min(val("left"), val("front_left"))
        right = min(val("right"), val("front_right"))
        if max(left, right) < LIDAR_BACK_CLEAR_M:
            return None
        return 90.0 if left >= right else -90.0
