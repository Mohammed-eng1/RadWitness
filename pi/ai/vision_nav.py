# -*- coding: utf-8 -*-
"""
vision_nav.py — طبقة الرؤية المشتركة (مزوّد واحد للمسارين)
===========================================================
⚠ **وحدة واحدة لمسارَي المشروع**: الملاحة البصرية (`BRIEF_VISUAL_NAV.md`)
والتوثيق البصري للمصدر (`approach_document.py`) يستعملان **نفس المزوّد ونفس
المفتاح ونفس عدّاد الرصيد**. نسختان من نفس التكامل تعني صيانة مزدوجة
واستهلاك رصيد مضاعف.

**النقل مشترك، والمهام منفصلة تماماً**: كل مهمة برومتها ومخططها الخاص. ولا
يُخلط بينهما إطلاقاً — برومت الملاحة يشدّد صراحةً «لا تكشف إشعاعاً»، وبرومت
التوثيق يصف ما يُرى **بعد** أن أثبت العدّاد الإشعاع فيزيائياً.

────────────────────────────────────────────────────────────────────
قواعد ملزمة (من الفلسفة الحاكمة — CLAUDE.md §5):

  **الرؤية تقترح والحساسات تحمي.** خطأ النموذج أو تأخّره أو انقطاع الإنترنت
  **لا يوقف المهمة ولا يغيّر السلوك**: تُسجَّل ملاحظة ويكمل الروبوت محلياً.

  لذلك كل مسار فشل هنا **يُعيد نتيجة، لا يرمي استثناءً**: بلا مفتاح، بلا
  إنترنت، مهلة، رد فاسد، سقف رصيد — كلها `ok=False` بسبب مقروء.

────────────────────────────────────────────────────────────────────
النقل عبر **REST مباشرة** لا عبر SDK: يتفادى اقتران إصدارات المكتبات
(google-genai مقابل google-generativeai) ويُبقي سطح التبعية على الراسبري
عند `requests` وحدها — وهي موجودة أصلاً. وسهل الاعتراض في الاختبار.

منطق خالص القابلية للاختبار: بلا عتاد وبلا شبكة في الاختبارات (مزوّد محاكاة).
"""
from __future__ import annotations

import base64
import importlib.util
import io
import json
import os
import threading
import time

from pi.config import (
    VISION_CONNECT_TIMEOUT_S, VISION_CALL_DEADLINE_S,
    VISION_ENABLED, VISION_MODEL, VISION_API_BASE, VISION_TIMEOUT_S,
    VISION_IMAGE_MAX_PX, VISION_JPEG_QUALITY, VISION_THINKING_BUDGET,
    VISION_COOLDOWN_S, VISION_MAX_CALLS_PER_MISSION, VISION_MAX_IMAGES_PER_CALL,
    VISION_COST_PER_CALL_USD, _REPO_ROOT,
)

# ── تبعيات اختيارية خلف حراسة (القاعدة 7: لا استيراد مكشوف) ───────
try:
    import requests
    _REQUESTS_OK = True
except ImportError:                       # noqa: BLE001
    _REQUESTS_OK = False

try:
    from PIL import Image
    _PIL_OK = True
except ImportError:                       # noqa: BLE001
    _PIL_OK = False


# ── المفتاح من secrets.py ────────────────────────────────────────
# ⚠⚠ **موضع ملف الأسرار: `pi/secrets.py` لا جذر المستودع** ⚠⚠
# `secrets` وحدة **قياسية في بايثون**، وملفٌ بهذا الاسم في جذر المستودع
# **يُظلّلها على البرنامج كله** (الجذر أول عنصر في sys.path عند التشغيل منه).
# العطل مقاس لا نظري: `numpy.random` يستورد `randbits` من الوحدة القياسية،
# فبمجرّد وجود secrets.py في الجذر ينهار numpy كله:
#     ImportError: cannot import name randbits
# ولأن الملف **مستثنى من git** فالعطل يظهر عند من يملأ أسراره فقط — أي على
# الراسبري وقت التشغيل الحقيقي، لا عند المطوّر. أخبث صنف أعطال.
# ⇒ داخل الحزمة `pi/` لا يُظلّل شيئاً (الحزمة ليست على sys.path مباشرةً)،
#   و`.gitignore` يطابق `secrets.py` **بأي عمق** فيبقى مستثنى تلقائياً.
_SECRETS_PATHS = (
    os.path.join(_REPO_ROOT, "pi", "secrets.py"),      # الموضع الصحيح
    os.path.join(_REPO_ROOT, "secrets.py"),            # قديم — يعمل مع تحذير
)


def secrets_location() -> dict:
    """أين وُجد ملف الأسرار، ومع تحذير إن كان في الموضع الخطر."""
    for i, p in enumerate(_SECRETS_PATHS):
        if os.path.exists(p):
            return {"path": p, "ok": i == 0,
                    "warning": (None if i == 0 else
                                "🔴 secrets.py في **جذر المستودع** يُظلّل وحدة "
                                "بايثون القياسية `secrets` ويكسر numpy.random "
                                "(cannot import name randbits). انقله إلى "
                                "pi/secrets.py.")}
    return {"path": None, "ok": False, "warning": "لا ملف أسرار"}


def _load_api_key():
    """
    يقرأ `GEMINI_API_KEY` من `pi/secrets.py`.

    ⚠ **لا نستعمل `import secrets`** إطلاقاً (انظر التحذير أعلاه): التحميل
    بالمسار الصريح تحت اسم مستقل يزيل الالتباس ولا يضيف الملف إلى فضاء
    الأسماء العام.

    متغيّر البيئة `RMS_GEMINI_API_KEY` يتقدّم على الملف (تشغيل مؤقت بلا ملف).
    """
    env = os.environ.get("RMS_GEMINI_API_KEY", "").strip()
    if env:
        return env
    loc = secrets_location()
    if not loc["path"]:
        return None
    try:
        spec = importlib.util.spec_from_file_location("rms_secrets", loc["path"])
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        key = (getattr(mod, "GEMINI_API_KEY", "") or "").strip()
        return key or None
    except Exception:                     # noqa: BLE001 — ملف أسرار معطوب
        return None


# ═══ تجهيز الصور ═════════════════════════════════════════════════
def prepare_image(jpeg_bytes: bytes, max_px: int = VISION_IMAGE_MAX_PX) -> bytes:
    """
    يقلّص الصورة إلى `max_px` على الضلع الأطول — **توفير تكلفة وزمن**، ودقة
    512px كافية تماماً لوصف المشهد (لا نقرأ نصاً دقيقاً). بلا PIL تُرسل كما هي
    مع تكلفة أعلى (لا فشل).
    """
    if not jpeg_bytes or not _PIL_OK:
        return jpeg_bytes
    try:
        img = Image.open(io.BytesIO(jpeg_bytes))
        w, h = img.size
        if max(w, h) <= max_px:
            return jpeg_bytes
        scale = max_px / float(max(w, h))
        img = img.resize((max(1, int(w * scale)), max(1, int(h * scale))))
        out = io.BytesIO()
        img.convert("RGB").save(out, format="JPEG", quality=VISION_JPEG_QUALITY)
        return out.getvalue()
    except Exception:                     # noqa: BLE001 — صورة تالفة
        return jpeg_bytes


# ═══ التحقق الصارم من الـJSON ════════════════════════════════════
def validate_json(text: str, required: dict) -> dict:
    """
    تحقّق **صارم** قبل الاستخدام: رد فاسد يُتجاهل ويكمل الروبوت محلياً.

    `required` = {مفتاح: None للحقل الحرّ، أو مجموعة القيم المسموحة}.
    يقبل تغليف ```json ...``` لأن النماذج تضيفه أحياناً رغم التعليمات.
    """
    if not text:
        return {"ok": False, "reason": "رد فارغ"}
    s = text.strip()
    if s.startswith("```"):
        s = s.split("```")[1] if "```" in s[3:] else s.lstrip("`")
        if s.lstrip().lower().startswith("json"):
            s = s.lstrip()[4:]
    i, j = s.find("{"), s.rfind("}")
    if i < 0 or j <= i:
        return {"ok": False, "reason": "لا كائن JSON في الرد"}
    try:
        data = json.loads(s[i:j + 1])
    except json.JSONDecodeError as e:
        return {"ok": False, "reason": f"JSON غير صالح: {e}"}
    if not isinstance(data, dict):
        return {"ok": False, "reason": "الجذر ليس كائناً"}
    for key, allowed in required.items():
        if key not in data:
            return {"ok": False, "reason": f"حقل ناقص: {key}"}
        if allowed is not None and data[key] not in allowed:
            return {"ok": False, "reason":
                    f"قيمة غير مسموحة في {key}: {data[key]!r}"}
    return {"ok": True, "data": data}


# ═══ المزوّدون ═══════════════════════════════════════════════════
class VisionProvider:
    """
    الواجهة المجرّدة. أي مزوّد يُعيد **قاموساً** فيه `ok` وسبب — ولا يرمي
    استثناءً أبداً: الرؤية طبقة اقتراح، وسقوطها لا يوقف المهمة.
    """

    name = "base"

    def __init__(self):
        self.calls = 0
        self.failures = 0
        self.last_latency_s = None
        self.last_error = None
        self.last_usage = None         # توكنات آخر استدعاء (من الرد لا تقديراً)
        self.total_tokens = 0
        self._last_call_at = {}        # {مفتاح الموقف: زمن آخر استدعاء}

    # ── حماية الرصيد ─────────────────────────────────────────────
    def cooldown_ok(self, situation_key: str,
                    cooldown_s: float = VISION_COOLDOWN_S) -> bool:
        """
        أداة خارجية حتى توصيل BRIEF_VISUAL_NAV (بند 2 — محفّزات الاستدعاء).

        مانع تكرار: لا نستدعي لنفس الموقف مرتين خلال المهلة.
        """
        t = self._last_call_at.get(situation_key)
        return t is None or (time.time() - t) >= cooldown_s

    def mark_called(self, situation_key: str) -> None:
        """أداة خارجية حتى توصيل BRIEF_VISUAL_NAV (بند 2 — محفّزات الاستدعاء)."""
        self._last_call_at[situation_key] = time.time()

    def budget_left(self) -> int:
        return max(VISION_MAX_CALLS_PER_MISSION - self.calls, 0)

    def stats(self) -> dict:
        """عدّاد الاستدعاءات والتكلفة التقديرية — للواجهة (وعي الرصيد)."""
        return {"provider": self.name, "calls": self.calls,
                "failures": self.failures,
                # نداءات قُطعت بالسقف الصلب — تصاعدها يعني **شبكة** لا رصيداً
                "timeouts": int(getattr(self, "timeouts", 0)),
                "budget_left": self.budget_left(),
                "estimated_cost_usd": round(self.calls * VISION_COST_PER_CALL_USD, 4),
                "total_tokens": self.total_tokens,
                "last_usage": self.last_usage,
                "last_latency_s": self.last_latency_s,
                "last_error": self.last_error,
                "available": self.available()}

    def available(self) -> bool:
        return True

    def analyze(self, images, prompt, required) -> dict:
        raise NotImplementedError


class SimulatedVisionProvider(VisionProvider):
    """
    مزوّد محاكاة — **ليس كعباً للاختبار فقط**: هو مسار التشغيل الشرعي بلا
    مفتاح أو بلا إنترنت. يُعيد `ok=False` بسبب واضح فتعرض الواجهة «التحليل
    البصري غير متاح» ويكمل النظام كل شيء آخر.

    `canned` يسمح بحقن رد جاهز في الاختبارات وفي **وضع العرض المسجّل**
    (REPLAY_VISION في بريف الملاحة — حماية ليوم العرض بلا إنترنت).
    """

    name = "simulated"

    def __init__(self, canned: dict = None, reason: str = None):
        super().__init__()
        self.canned = canned
        self.reason = reason or "لا مفتاح Gemini — التحليل البصري غير متاح"

    def available(self) -> bool:
        return self.canned is not None

    def analyze(self, images, prompt, required) -> dict:
        self.calls += 1
        self.last_latency_s = 0.0
        if self.canned is None:
            self.failures += 1
            self.last_error = self.reason
            return {"ok": False, "reason": self.reason, "provider": self.name}
        out = validate_json(json.dumps(self.canned, ensure_ascii=False), required)
        out["provider"] = self.name
        if not out["ok"]:
            self.failures += 1
            self.last_error = out["reason"]
        return out


class GeminiVisionProvider(VisionProvider):
    """
    المزوّد الحقيقي عبر REST. كل فشل يُترجم إلى `ok=False` بسبب مقروء —
    لا استثناء يتسرّب إلى حلقة الملاحة.
    """

    name = "gemini"

    def __init__(self, api_key: str = None, model: str = VISION_MODEL,
                 timeout_s: float = VISION_TIMEOUT_S):
        super().__init__()
        self.api_key = api_key or _load_api_key()
        self.model = model
        self.timeout_s = float(timeout_s)
        self.deadline_s = float(VISION_CALL_DEADLINE_S)
        self.timeouts = 0             # نداءات قُطعت بالسقف الصلب (للواجهة)

    def _post_with_deadline(self, url: str, body: dict):
        """
        🔴 سقف صلب للنداء كله — مهلة `requests` **لكل عملية مقبس** لا للنداء.

        urllib3 يجرّب كل عنوان من `getaddrinfo` بمهلة مستقلة (AAAA ثم A)،
        و**حلّ الاسم خارجها كلياً**. فعلى شبكة تحجب أو تبتلع الحزم يمتدّ
        النداء الواحد دقائق — وهو يقع على **خيط المهمة** بعد التقاط الصور
        وقبل التقرير، فتتجمّد المهمة بلا سطر سجل ولا صورة معروضة.

        الخيط العالق يبقى عالقاً (daemon) لكن **المهمة تمضي** — نفس علاج
        لقطة V4L2 في `pi/sensors/camera.py`. والقاعدة محفوظة: الرؤية
        تقترح والحساسات تحمي، فانقطاعها لا يوقف شيئاً.
        """
        box = {}

        def _work():
            try:
                box["resp"] = requests.post(
                    url, json=body,
                    # (اتصال، قراءة) لا رقماً مفرداً: يقصّ زمن كل عنوان فاشل
                    timeout=(VISION_CONNECT_TIMEOUT_S, self.timeout_s),
                    headers={"x-goog-api-key": self.api_key,
                             "Content-Type": "application/json"})
            except Exception as e:        # noqa: BLE001
                box["err"] = e

        w = threading.Thread(target=_work, daemon=True)
        w.start()
        w.join(self.deadline_s)
        if w.is_alive():
            self.timeouts += 1
            raise TimeoutError(
                f"تجاوز السقف الصلب {self.deadline_s:.0f}ث (حلّ اسم/اتصال/"
                f"قراءة) — النداء مقطوع والمهمة تمضي")
        if "err" in box:
            raise box["err"]
        return box["resp"]

    def available(self) -> bool:
        return bool(VISION_ENABLED and self.api_key and _REQUESTS_OK)

    def analyze(self, images, prompt, required) -> dict:
        if not VISION_ENABLED:
            return {"ok": False, "reason": "الرؤية معطّلة في config",
                    "provider": self.name}
        if not self.api_key:
            return {"ok": False, "reason": "لا مفتاح Gemini في secrets.py",
                    "provider": self.name}
        if not _REQUESTS_OK:
            return {"ok": False, "reason": "requests غير مثبّتة",
                    "provider": self.name}
        if self.budget_left() <= 0:
            return {"ok": False, "provider": self.name,
                    "reason": (f"بلغ سقف {VISION_MAX_CALLS_PER_MISSION} استدعاء "
                               f"للمهمة — حماية الرصيد")}

        parts = [{"text": prompt}]
        for img in list(images or [])[:VISION_MAX_IMAGES_PER_CALL]:
            parts.append({"inline_data": {
                "mime_type": "image/jpeg",
                "data": base64.b64encode(prepare_image(img)).decode("ascii")}})

        body = {
            "contents": [{"role": "user", "parts": parts}],
            "generationConfig": {
                "responseMimeType": "application/json",   # JSON فقط
                "temperature": 0.0,                        # وصف لا تأليف
                "thinkingConfig": {"thinkingBudget": VISION_THINKING_BUDGET},
            },
        }
        url = f"{VISION_API_BASE}/{self.model}:generateContent"
        t0 = time.time()
        self.calls += 1
        try:
            resp = self._post_with_deadline(url, body)
            self.last_latency_s = round(time.time() - t0, 2)
            if resp.status_code != 200:
                self.failures += 1
                self.last_error = f"HTTP {resp.status_code}"
                return {"ok": False, "provider": self.name,
                        "reason": f"HTTP {resp.status_code}: {resp.text[:120]}"}
            payload = resp.json()
            # عدّاد التوكنات **من الرد نفسه** لا تقديراً: هو الأساس الوحيد
            # الصادق لتكلفة معروضة في الواجهة (التقدير الثابت للاطمئنان فقط).
            um = payload.get("usageMetadata") or {}
            self.last_usage = {
                "prompt_tokens": um.get("promptTokenCount"),
                "output_tokens": um.get("candidatesTokenCount"),
                "total_tokens": um.get("totalTokenCount"),
            }
            self.total_tokens += int(um.get("totalTokenCount") or 0)
            text = (payload["candidates"][0]["content"]["parts"][0]["text"])
        except Exception as e:            # noqa: BLE001 — مهلة/شبكة/رد ناقص
            self.last_latency_s = round(time.time() - t0, 2)
            self.failures += 1
            self.last_error = str(e)[:120]
            return {"ok": False, "provider": self.name,
                    "reason": f"فشل الاستدعاء ({type(e).__name__}): {str(e)[:100]}"}

        out = validate_json(text, required)
        out["provider"] = self.name
        out["latency_s"] = self.last_latency_s
        if not out["ok"]:
            self.failures += 1
            self.last_error = out["reason"]
        return out


_PROVIDER = None


def get_provider(force_simulated: bool = False, canned: dict = None):
    """
    المزوّد المشترك (مفرد). يسقط تلقائياً إلى المحاكاة بلا مفتاح أو بلا
    `requests` — **بلا فشل صامت**: السبب يظهر في `stats()["last_error"]`.
    """
    global _PROVIDER
    if force_simulated or canned is not None:
        return SimulatedVisionProvider(canned=canned)
    if _PROVIDER is None:
        g = GeminiVisionProvider()
        _PROVIDER = g if g.available() else SimulatedVisionProvider(
            reason=("لا مفتاح Gemini في secrets.py" if not g.api_key
                    else "requests غير مثبّتة"))
    return _PROVIDER


def reset_provider() -> None:
    """أداة خارجية (اختبارات/تشخيص) — إعادة التقاط المفتاح بعد تعديل secrets.py."""
    global _PROVIDER
    _PROVIDER = None


# ═══ المهمة (أ): الملاحة البصرية — BRIEF_VISUAL_NAV.md ═══════════
# ⚠ برومت الملاحة **يمنع صراحةً** أي ادعاء عن الإشعاع: الإشعاع غير مرئي،
#   ومهمة هذه الطبقة المساحة القابلة للمرور فقط.
_NAV_RULES = ("أنت عين ملاحة لروبوت أرضي. صف المساحة القابلة للمرور فقط. "
              "لا تكشف إشعاعاً — الإشعاع غير مرئي ولا يُستدل عليه من صورة. "
              "أخرج JSON فقط بلا أي نص خارجه. عند الغموض قل confidence=low "
              "ولا تخمّن.")

OBSTACLE_SCHEMA = {
    "obstacle_type": {"room_wall", "separate_object", "doorway",
                      "furniture", "unclear"},
    "can_pass_around": None,
    "pass_side": {"left", "right", "none"},
    "confidence": {"low", "medium", "high"},
    "reason": None,
}
DIRECTIONS_SCHEMA = {
    "directions": None, "best_index": None,
    "confidence": {"low", "medium", "high"}, "reason": None,
}


def analyze_obstacle(images, provider=None) -> dict:
    """
    أداة خارجية حتى توصيل BRIEF_VISUAL_NAV (بند 2 — المحفّز الرئيسي).

    «هل هذا جدار الغرفة أم عائق منفصل نلتف حوله لتغطية ما خلفه؟»
    """
    p = provider or get_provider()
    prompt = (_NAV_RULES + "\n\nالسؤال: هل ما أمام الروبوت جدار غرفة "
              "(نمشي بمحاذاته) أم عائق منفصل (نلتف حوله لتغطية ما خلفه)؟\n"
              "المخطط: {\"obstacle_type\": \"room_wall|separate_object|"
              "doorway|furniture|unclear\", \"can_pass_around\": true/false, "
              "\"pass_side\": \"left|right|none\", \"confidence\": "
              "\"low|medium|high\", \"reason\": \"وصف قصير\"}")
    return p.analyze(images, prompt, OBSTACLE_SCHEMA)


def analyze_directions(images, provider=None) -> dict:
    """
    أداة خارجية حتى توصيل BRIEF_VISUAL_NAV (بند 3 — المسح البصري 360°).

    مسح 360°: عدة لقطات في **طلب واحد** (أوفر) واختيار الاتجاه الأفضل.
    """
    p = provider or get_provider()
    prompt = (_NAV_RULES + f"\n\nلديك {len(images or [])} صور مرتّبة حول "
              "الروبوت (index 0 هو الأمام، ثم بترتيب الدوران). قيّم انفتاح كل "
              "اتجاه واختر الأفضل للمرور.\nالمخطط: {\"directions\": "
              "[{\"index\": 0, \"openness\": \"open|partial|blocked\", "
              "\"hazard\": \"none|furniture|wall|dropoff\", \"note\": \"..\"}], "
              "\"best_index\": 0, \"confidence\": \"low|medium|high\", "
              "\"reason\": \"..\"}")
    return p.analyze(images, prompt, DIRECTIONS_SCHEMA)


# ═══ المهمة (ب): التوثيق البصري للمصدر ═══════════════════════════
# ⚠ **مهمة مختلفة تماماً عن الملاحة** رغم اشتراك النقل. وهنا أيضاً القاعدة
#   نفسها بصياغة أدق: العدّاد أثبت الإشعاع **فيزيائياً**، والرؤية تصف ما
#   يوجد **بصرياً**. الوصف ليس إثباتاً ولا نفياً للإشعاع.
SOURCE_SCENE_SCHEMA = {
    "indicators": None,          # قائمة من المؤشرات المرصودة
    "description": None,
    "confidence": {"low", "medium", "high"},
    "note": None,
}
INDICATOR_VALUES = ("barrel", "warning_sign", "radiation_symbol",
                    "industrial_equipment", "restricted_area",
                    "container", "none")


def analyze_source_scene(images, provider=None) -> dict:
    """
    وصف المشهد عند الموقع المقدَّر للمصدر.

    البرومت يمنع صراحةً استنتاج وجود الإشعاع من الصورة أو نفيه — وهذا ليس
    تزيّداً: لو قال النموذج «لا أرى إشعاعاً» لأوحى بنفي ما أثبته العدّاد،
    ولو قال «هذا مصدر مشع» لادّعى ما لا تراه الكاميرا. كلٌّ في مجاله.
    """
    p = provider or get_provider()
    prompt = (
        "أنت تصف صورة التُقطت عند موقع حدّده عدّاد جيجر فيزيائياً.\n"
        "مهمتك: **وصف ما يُرى فقط**. ما الذي تراه؟ هل توجد مؤشرات بصرية على "
        "مواد مشعة — براميل، لافتات تحذير، رمز إشعاع (☢)، معدات صناعية، "
        "حاويات، مناطق محظورة؟\n"
        "⚠ لا تستنتج وجود إشعاع من الصورة ولا تنفيه: الإشعاع **غير مرئي**، "
        "وقد أثبته العدّاد أصلاً. غياب المؤشرات البصرية لا يعني غياب المصدر "
        "(قد يكون داخل حاوية أو خلف جدار).\n"
        "أخرج JSON فقط. عند الغموض confidence=low.\n"
        "المخطط: {\"indicators\": [" + "|".join(INDICATOR_VALUES) + "], "
        "\"description\": \"وصف قصير لما يُرى\", \"confidence\": "
        "\"low|medium|high\", \"note\": \"أي ملاحظة\"}")
    out = p.analyze(images, prompt, SOURCE_SCENE_SCHEMA)
    if out.get("ok"):
        ind = out["data"].get("indicators")
        if not isinstance(ind, list):
            return {"ok": False, "provider": out.get("provider"),
                    "reason": "indicators ليست قائمة"}
        unknown = [v for v in ind if v not in INDICATOR_VALUES]
        if unknown:
            return {"ok": False, "provider": out.get("provider"),
                    "reason": f"مؤشرات خارج المخطط: {unknown}"}
    return out
