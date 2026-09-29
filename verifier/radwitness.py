#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
radwitness.py — سجل ميداني مقاوم للتزوير: سلسلة بصمات (SHA-256) + ختم رقمي (Ed25519).

ماذا يثبت:   أن السجل لم يتغير بعد كتابته، وأين تغيّر بالضبط، وأن حامل المفتاح هو من ختمه.
ماذا لا يثبت: أن الحسّاس كان صادقاً — ولهذا يُسجَّل مع كل قراءة هل كانت موثوقة.
ليس «بلوك تشين»: جهاز واحد، بلا شبكة ولا إجماع.

الأوامر:
  python3 radwitness.py keygen                     # مرة واحدة: مفتاح الروبوت
  python3 radwitness.py from-csv MISSION.csv LOG   # بناء سجل من بيانات مهمة حقيقية
  python3 radwitness.py append LOG --cpm 18 --x 1.2 --y 0.4 --reliable 1 [--img photo.jpg]
  python3 radwitness.py seal LOG                   # الختم في نهاية المهمة
  python3 radwitness.py verify LOG                 # التحقق — يشغّله أي أحد بالمفتاح العام
  python3 radwitness.py show LOG                   # عرض السجل مرقّماً (للفيديو)
  python3 radwitness.py tamper-test LOG --n 1000   # تجربة: تعديل بايت عشوائي ألف مرة
"""
import argparse, base64, csv, hashlib, json, os, random, shutil, sys, tempfile, time
from datetime import datetime, timezone

try:
    from cryptography.hazmat.primitives.asymmetric.ed25519 import (
        Ed25519PrivateKey, Ed25519PublicKey)
    from cryptography.hazmat.primitives import serialization
    from cryptography.exceptions import InvalidSignature
except ImportError:
    sys.exit("⛔ مكتبة cryptography غير مثبّتة:  pip install cryptography")

GENESIS = "0" * 64
KEY_DIR = "keys"
PRIV = os.path.join(KEY_DIR, "robot.key")
PUB = os.path.join(KEY_DIR, "robot.pub")


# ─── أساسيات ─────────────────────────────────────────────────────────────
def canon(obj) -> bytes:
    """تمثيل ثابت للقيد: نفس المحتوى ⇒ نفس البايتات ⇒ نفس البصمة دائماً."""
    return json.dumps(obj, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False).encode("utf-8")


def sha256_hex(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def file_sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def read_raw_lines(log):
    """الأسطر كبايتات — حتى لا يُسقط بايتٌ معطوب قراءةَ الملف كله."""
    if not os.path.exists(log):
        return []
    with open(log, "rb") as f:
        return [ln.rstrip(b"\r\n") for ln in f.read().split(b"\n") if ln.strip()]


def read_lines(log):
    return [ln.decode("utf-8", "replace") for ln in read_raw_lines(log)]


def last_hash(log) -> str:
    lines = read_lines(log)
    if not lines:
        return GENESIS
    return json.loads(lines[-1])["hash"]


def make_entry(i: int, prev: str, payload: dict) -> dict:
    body = {"i": i, "t": now_iso(), "prev": prev, "data": payload}
    body["hash"] = sha256_hex(canon(body))
    return body


def seal_message(head: str, count: int) -> bytes:
    return f"RADWITNESS|v1|{count}|{head}".encode()


# ─── الأوامر ─────────────────────────────────────────────────────────────
def cmd_keygen(a):
    os.makedirs(KEY_DIR, exist_ok=True)
    if os.path.exists(PRIV) and not a.force:
        sys.exit(f"⚠ المفتاح موجود مسبقاً في {PRIV} — استعمل --force لاستبداله")
    k = Ed25519PrivateKey.generate()
    with open(PRIV, "wb") as f:
        f.write(k.private_bytes(serialization.Encoding.PEM,
                                serialization.PrivateFormat.PKCS8,
                                serialization.NoEncryption()))
    os.chmod(PRIV, 0o600)
    with open(PUB, "wb") as f:
        f.write(k.public_key().public_bytes(serialization.Encoding.PEM,
                                            serialization.PublicFormat.SubjectPublicKeyInfo))
    fp = sha256_hex(k.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw))[:16]
    print(f"✅ الختم الأصلي (سري):  {PRIV}")
    print(f"✅ صورة الختم (للتوزيع): {PUB}   بصمتها: {fp}")
    print("⚠ لا تشارك robot.key مع أحد، ولا ترفعه إلى GitHub.")


def cmd_append(a):
    if os.path.exists(a.log + ".seal"):
        sys.exit("⛔ السجل مختوم — لا يُضاف إليه بعد الختم. ابدأ سجلاً جديداً.")
    payload = {"cpm": a.cpm, "x": a.x, "y": a.y, "reliable": bool(a.reliable)}
    if a.img:
        payload["img"] = os.path.basename(a.img)
        payload["img_sha256"] = file_sha256(a.img)
    n = len(read_lines(a.log))
    e = make_entry(n, last_hash(a.log), payload)
    with open(a.log, "a", encoding="utf-8") as f:
        f.write(json.dumps(e, ensure_ascii=False) + "\n")
    print(f"✅ القيد {n} · بصمته {e['hash'][:12]}… · بصمة سابقه {e['prev'][:12]}…")


def cmd_from_csv(a):
    if os.path.exists(a.log):
        sys.exit(f"⛔ {a.log} موجود — اختر اسماً جديداً")
    with open(a.csv, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("⛔ ملف CSV فارغ")
    prev = GENESIS
    with open(a.log, "w", encoding="utf-8") as out:
        for i, r in enumerate(rows):
            payload = {k: v for k, v in r.items() if k is not None}
            payload["source_file"] = os.path.basename(a.csv)
            e = make_entry(i, prev, payload)
            out.write(json.dumps(e, ensure_ascii=False) + "\n")
            prev = e["hash"]
    print(f"✅ {len(rows)} قيداً من {a.csv} ← {a.log}")
    print(f"   رأس السلسلة: {prev[:16]}…   (اختمه الآن: seal {a.log})")


def cmd_seal(a):
    lines = read_lines(a.log)
    if not lines:
        sys.exit("⛔ السجل فارغ")
    with open(a.key, "rb") as f:
        k = serialization.load_pem_private_key(f.read(), password=None)
    head = json.loads(lines[-1])["hash"]
    sig = k.sign(seal_message(head, len(lines)))
    seal = {"version": 1, "count": len(lines), "head": head,
            "sealed_at": now_iso(), "sig": base64.b64encode(sig).decode()}
    with open(a.log + ".seal", "w", encoding="utf-8") as f:
        json.dump(seal, f, indent=2)
    print(f"✅ خُتم {len(lines)} قيداً · الرأس {head[:16]}… ← {a.log}.seal")


def verify(log, pub_path, quiet=False):
    """يُرجع (ok, bad_index_or_None, reason)."""
    raw_lines = read_raw_lines(log)
    if not raw_lines:
        return False, None, "السجل فارغ"
    lines = raw_lines
    prev = GENESIS
    for n, raw in enumerate(raw_lines):
        try:
            e = json.loads(raw.decode("utf-8"))
        except Exception:
            return False, n, "السطر تالف — لم يعد نصاً أو JSON صالحاً"
        if not isinstance(e, dict) or "hash" not in e:
            return False, n, "بنية القيد غير صحيحة"
        stored = e.pop("hash")
        if e.get("prev") != prev:
            return False, n, "الربط بالقيد السابق مكسور"
        if sha256_hex(canon(e)) != stored:
            return False, n, "محتوى القيد لا يطابق بصمته"
        if e.get("i") != n:
            return False, n, "ترقيم القيد غير متسلسل"
        prev = stored
    seal_path = log + ".seal"
    if not os.path.exists(seal_path):
        return False, None, "لا يوجد ختم — السلسلة سليمة لكن غير مختومة"
    try:
        with open(seal_path, encoding="utf-8") as f:
            seal = json.load(f)
        with open(pub_path, "rb") as f:
            pub = serialization.load_pem_public_key(f.read())
        if seal["head"] != prev or seal["count"] != len(lines):
            return False, None, "الختم لا يطابق السجل (أُضيف أو حُذف منه قيود)"
        pub.verify(base64.b64decode(seal["sig"]), seal_message(prev, len(lines)))
    except InvalidSignature:
        return False, None, "التوقيع غير صحيح — الختم مزوّر أو المفتاح مختلف"
    except Exception as ex:
        return False, None, f"الختم تالف: {ex}"
    return True, None, f"{len(lines)} قيداً سليماً ومختوماً"


def cmd_verify(a):
    ok, bad, why = verify(a.log, a.pub)
    print("═" * 52)
    if ok:
        print(f"  ✅ السجل سليم — {why}")
    else:
        where = f" عند القيد {bad}" if bad is not None else ""
        print(f"  ⛔ السجل عُدّل{where}")
        print(f"     السبب: {why}")
    print("═" * 52)
    sys.exit(0 if ok else 1)


def cmd_show(a):
    for ln in read_lines(a.log)[: a.limit]:
        e = json.loads(ln)
        d = e["data"]
        short = ", ".join(f"{k}={v}" for k, v in list(d.items())[:4])
        print(f"[{e['i']:>4}] {short:<48} ← {e['prev'][:8]}…  = {e['hash'][:8]}…")


def cmd_tamper_test(a):
    """تجربة التنفيذ: هل يُكشف تعديل بايت واحد عشوائي؟ وهل يُحدَّد القيد الصحيح؟"""
    rng = random.Random(a.seed)
    base_ok, _, why = verify(a.log, a.pub)
    if not base_ok:
        sys.exit(f"⛔ السجل الأصلي لا ينجح في التحقق أصلاً: {why}")
    raw = open(a.log, "rb").read()
    offsets = []
    pos = 0
    for ln in raw.split(b"\n"):
        offsets.append((pos, pos + len(ln)))
        pos += len(ln) + 1
    detected = located = 0
    rows = []
    with tempfile.TemporaryDirectory() as td:
        tlog = os.path.join(td, "t.jsonl")
        shutil.copy(a.log + ".seal", tlog + ".seal")
        for k in range(a.n):
            b = bytearray(raw)
            j = rng.randrange(len(b))
            if b[j] == 0x0A:                      # لا نلمس فواصل الأسطر
                j = max(0, j - 1)
            old = b[j]
            new = old
            while new == old or new == 0x0A:
                new = rng.randrange(256)
            b[j] = new
            with open(tlog, "wb") as f:
                f.write(b)
            true_line = next(i for i, (s, e) in enumerate(offsets) if s <= j <= e)
            ok, bad, why = verify(tlog, a.pub)
            det = not ok
            loc = det and bad == true_line
            detected += det
            located += loc
            rows.append((k, j, true_line, int(det), bad if bad is not None else "", int(loc), why))
    # الضابط: السجل السليم يجب أن ينجح دائماً
    clean_pass = sum(verify(a.log, a.pub)[0] for _ in range(a.clean))
    out = f"tamper_test_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["trial", "byte_offset", "true_entry", "detected", "reported_entry", "located", "reason"])
        w.writerows(rows)
    print("═" * 60)
    n_entries = len(read_lines(a.log))
    print(f"  تجربة التعديل العشوائي — {a.n} محاولة على {n_entries} قيداً")
    print(f"  كُشف التعديل:            {detected}/{a.n}")
    print(f"  حُدّد القيد الصحيح:       {located}/{a.n}")
    print(f"  السجل السليم نجح (ضابط): {clean_pass}/{a.clean}")
    print(f"  التفاصيل: {out}")
    print("═" * 60)
    print("  ملاحظة أمانة: التعديل الذي يغيّر مسافة بين الحقول فقط (مسافة إلى tab مثلاً)")
    print("  لا يغيّر أي قيمة، فلا يُعد تعديلاً ولا يُكشف — كل تعديل غيّر قيمة يجب أن يُكشف.")


def main():
    p = argparse.ArgumentParser(description="RadWitness — سجل ميداني مقاوم للتزوير")
    s = p.add_subparsers(dest="cmd", required=True)

    k = s.add_parser("keygen"); k.add_argument("--force", action="store_true"); k.set_defaults(f=cmd_keygen)

    ap = s.add_parser("append"); ap.add_argument("log")
    ap.add_argument("--cpm", type=float, required=True); ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0); ap.add_argument("--reliable", type=int, default=1)
    ap.add_argument("--img"); ap.set_defaults(f=cmd_append)

    fc = s.add_parser("from-csv"); fc.add_argument("csv"); fc.add_argument("log"); fc.set_defaults(f=cmd_from_csv)

    se = s.add_parser("seal"); se.add_argument("log"); se.add_argument("--key", default=PRIV); se.set_defaults(f=cmd_seal)

    v = s.add_parser("verify"); v.add_argument("log"); v.add_argument("--pub", default=PUB); v.set_defaults(f=cmd_verify)

    sh = s.add_parser("show"); sh.add_argument("log"); sh.add_argument("--limit", type=int, default=30); sh.set_defaults(f=cmd_show)

    t = s.add_parser("tamper-test"); t.add_argument("log"); t.add_argument("--pub", default=PUB)
    t.add_argument("--n", type=int, default=1000); t.add_argument("--clean", type=int, default=100)
    t.add_argument("--seed", type=int, default=2026); t.set_defaults(f=cmd_tamper_test)

    a = p.parse_args()
    a.f(a)


if __name__ == "__main__":
    main()
