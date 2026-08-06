#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_lora.py — اختبار منفرد لوصلة اللورا HC-14 على الراسبري
===========================================================
يبثّ إطار PING دورياً ويستقبل الإطارات الواردة، متحققاً من الـchecksum،
ويعرض أي رد (PONG) مع زمن الذهاب-والإياب (RTT). اختبار حيّة الوصلة قبل
الدمج (الشاشة تردّ من فيرمويرها في M4).

⚠ العتاد: HC-14 عبر محول USB-Serial → منفذ USB (/dev/ttyUSB*)، 9600 باود
   (الخيار الموصى — أبسط من UART ثانٍ على الراسبري، ويحرّر UART العتاد
   للـGPS). أبعِد هوائي 433MHz عن وحدة GPS.

صيغة الإطار (v2 — ≤60 بايت + checksum XOR):
    $<payload>*<HH>\n
    <payload>  نص ASCII، ≤ 56 بايت (يترك مجالاً للتأطير)
    <HH>       بايتان hex = XOR لكل بايتات payload (على طراز NMEA)
    مثال:  $PING,42*3A\n
    (المواصفة الكاملة للرسائل الثلاث — روفر/لورا/WebSocket — تُوثَّق في
     docs/protocol.md ضمن M4؛ هنا نثبّت التأطير ونختبر الوصلة فقط.)

التشغيل على الراسبري:
    python3 pi/tests/test_lora.py                 # حلقة PING (تحتاج طرفاً ثانياً يردّ)
    python3 pi/tests/test_lora.py /dev/ttyUSB1    # لتحديد منفذ آخر
    python3 pi/tests/test_lora.py listen          # ★★ إصغاء صرف — أقوى إثبات للوصلة
    python3 pi/tests/test_lora.py at              # ★ فحص وحدة واحدة عبر أوامر AT
    python3 pi/tests/test_lora.py hex             # ◆ بايتات خام hex — تشريح التلف
    python3 pi/tests/test_lora.py probe           # ◆ hex + بثّ $PROBE,<n> كل ثانيتين

◆ وضعا التشريح البايتي (يقابلان firmware/lora_probe على الشاشة):
   «إطار مرفوض» يخفي شكل العطل؛ الـhex يفرّقه: خربشة عشوائية = باود/سلك/
   تغذية · نص سليم مبتور = تأطير · نص شبه سليم ببتات مقلوبة = RF (الأرجح
   تشبّع: وحدتان +20dBm على نفس الطاولة — باعد مترين أو AT+P6 مؤقتاً).
   شاشة→راسبري: lora_probe على الشاشة + `hex` هنا.
   راسبري→شاشة: `probe` هنا + راقب RX< في شاشة lora_probe التسلسلية.

⚠⚠ **درس مقاس (2026-08-06): الصدى ليس استقبالاً** ⚠⚠
   نتيجة سابقة (PING,1 → «استُقبل PING,1») ظُنّت رداً من الشاشة — والشاشة
   **لم تكن مبرمَجة أصلاً**. كان صدى: HC-14 أو محوّل USB-TTL يعيد ما
   أُرسل إليه (نفس درس صدى فيرموير Wave Rover الموثَّق). لذلك:
   - أي حمولة واردة **مطابقة لما أرسلناه للتوّ** تُعرض «صدى» تحذيراً
     لا استقبالاً، وتُحصى منفصلة.
   - الوصلة لا تُعلَن سليمة إلا برد **مختلف الصيغة** عن كل ما أرسلناه
     (PONG، أو إطار C من أزرار الشاشة، أو أي حمولة لم نبثّها).
   - وضع `listen` لا يبثّ شيئاً إطلاقاً ⇒ كل إطار صالح يصل فيه هو
     حتماً من الطرف الآخر: اضغط أزرار الشاشة وراقب وصول إطارات C.

★ فحص الوحدة الواحدة (at): بوحدة HC-14 واحدة لا يمكن اختبار الإرسال اللاسلكي
   (يلزم طرف ثانٍ في M4). لكن أوامر AT تؤكد أن الوحدة حيّة والتوصيل (TX/RX)
   سليم: **اربط دبوس KEY بـGND** (وضع الأوامر)، ثم شغّل `... at` → يجب أن
   تردّ الوحدة `OK` على `AT`. أعد KEY حراً بعدها للوضع الشفاف.

⚠⚠ **أسماء دبابيس HC-14 — درس ضلّل تشخيصاً فعلاً (2026-08-06)**:
   - دبوس الأوامر اسمه **KEY** (بسحب داخلي لأعلى؛ إنزاله = وضع AT).
     تسمية «SET» تخصّ HC-12 الأقدم ولا وجود لها على HC-14 — وكانت هنا
     خطأً فذهب GND إلى الدبوس الخطأ وبقيت الوحدة شفافة لا تردّ على AT.
   - **STA خرجٌ لا دخل**: حالة الوحدة (HIGH جاهزة/LOW مشغولة) لإيقاظ
     متحكم نائم — **لا يوصل بـGND أبداً**.
   - حيلة استرجاع: إنزال KEY **قبل** التغذية يدخل AT على 9600 دائماً
     مهما كان الباود المضبوط المنسي.
   - التغذية تحتاج ≥250mA — خرج 3.3V في بعض محولات USB لا يكفي بثّة
     الإرسال؛ استعمل 5V المحوّل (المدى المسموح 3.0–5.5V).
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("خطأ: مكتبة pyserial غير مثبّتة. ثبّتها عبر setup_pi.sh.")

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 9600
MAX_PAYLOAD = 56


def xor_checksum(payload: str) -> int:
    """XOR لكل بايتات payload (بترميز ASCII)."""
    c = 0
    for b in payload.encode("ascii"):
        c ^= b
    return c


def build_frame(payload: str) -> bytes:
    """يبني إطار v2: $<payload>*<HH>\\n مع فرض حد الطول."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload يتجاوز {MAX_PAYLOAD} بايت")
    return f"${payload}*{xor_checksum(payload):02X}\n".encode("ascii")


def parse_frame(line: str):
    """يفكّ إطاراً ويتحقق من الـchecksum. يُعيد payload أو None عند الخطأ."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, cksum = line[1:].rpartition("*")
    try:
        if int(cksum, 16) != xor_checksum(body):
            return None                    # checksum لا يطابق → إطار تالف
    except ValueError:
        return None
    return body


def at_diagnostics(port: str) -> None:
    """فحص وحدة HC-14 واحدة عبر أوامر AT (يتطلب **KEY**→GND — لا STA!)."""
    try:
        ser = serial.Serial(port, BAUD, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"خطأ: تعذّر فتح {port} ({e}). تحقق من محول USB-Serial (ls /dev/ttyUSB*).")

    print(f"وضع AT على {port} @ {BAUD}.")
    print("⚠ اربط دبوس **KEY** بـGND (وإلا الوحدة في الوضع الشفاف ولن تردّ).")
    print("⚠ STA خرجُ حالة لا يوصل بـGND — الالتباس بينهما ضلّل تشخيصاً فعلاً.\n")
    any_reply = False
    # ⚠ أوامر AT تُرسل **بلا نهاية سطر** (متطلب HC-14 الموثَّق)
    for cmd in ("AT", "AT+V?", "AT+RX"):           # حيّة / إصدار / الإعدادات
        ser.reset_input_buffer()
        ser.write(cmd.encode("ascii"))
        time.sleep(0.6)
        n = ser.in_waiting
        resp = ser.read(n).decode("ascii", errors="replace").strip() if n else ""
        print(f"→ {cmd:8s}  ←  {resp!r}")
        if resp:
            any_reply = True
    ser.close()
    if any_reply:
        print("\n✅ الوحدة تردّ — السيريال والتوصيل (TX/RX) سليمان. أعد KEY حراً للوضع الشفاف.")
    else:
        print("\n⚠ لا ردّ. تحقّق بالترتيب: KEY→GND (لا STA!)، TX↔RX غير معكوسين،"
              "\n   التغذية 5V من المحوّل (بثّة الإرسال تحتاج ≥250mA)، الباود 9600."
              "\n   حيلة: أنزل KEY ثم صِل التغذية — يدخل AT على 9600 مهما كان الضبط.")


#: نافذة اعتبار الحمولة الواردة صدىً لإرسالنا (ثوانٍ)
ECHO_WINDOW_S = 5.0


def _verdict(real_rx: int, echoes: int, corrupt: int) -> None:
    """
    الحكم النهائي — 🔴 **الصدى لا يشهد بشيء**: وحدة واحدة بلا طرف ثانٍ
    تُنتج نفس المشهد تماماً. السليم الوحيد = رد مختلف الصيغة.
    """
    print("\n─── الخلاصة ─────────────────────────────────────")
    print(f"  ردود حقيقية (صيغة مختلفة): {real_rx}")
    print(f"  صدى (نفس ما أرسلناه):      {echoes}")
    print(f"  أطر تالفة:                  {corrupt}")
    if real_rx > 0:
        print("✅ الوصلة سليمة: وصل ردّ **مختلف الصيغة** عمّا أُرسل —"
              " لا يمكن أن يكون صدى.")
    elif echoes > 0:
        print("⚠ **لا دليل على وصلة**: كل الوارد صدى لما أرسلناه.")
        print("   الأرجح: HC-14/محوّل USB-TTL يعيد المرسَل، أو الطرف الآخر")
        print("   غير مبرمَج/مطفأ. هذا **ليس** استقبالاً لاسلكياً.")
        print("   جرّب: python3 pi/tests/test_lora.py listen ثم اضغط أزرار الشاشة.")
    else:
        print("⚠ لا شيء وصل إطلاقاً — راجع التوصيل والباود وضبط الوحدتين.")


def _open(port: str):
    try:
        return serial.Serial(port, BAUD, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"خطأ: تعذّر فتح {port} ({e}). "
                 "تحقق من توصيل محول USB-Serial (lsusb / ls /dev/ttyUSB*).")


class LineBuffer:
    """
    تجميع يدوي حتى `\\n` — لا `readline` بمهلة.

    ⚠ LoRa بطيء المعدل الهوائي والوحدة تسلّم الإطار على دفعات؛ `readline`
    بمهلة يقطع الإطار نصفين عند أي فجوة فيفشل كل نصف في الـchecksum
    ويبدو الإطار السليم ضجيجاً (نفس إصلاح `pi/comms/lora.py` و`pump()`
    في فيرموير الشاشة).
    """

    def __init__(self, ser):
        self.ser = ser
        self.buf = b""

    def lines(self):
        """يُعيد قائمة الأسطر المكتملة المتاحة الآن (قد تكون فارغة)."""
        chunk = self.ser.read(64)
        out = []
        if chunk:
            self.buf += chunk
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                if line.strip():
                    out.append(line.decode("ascii", errors="replace"))
            if len(self.buf) > 512:
                self.buf = b""
        return out


def listen_mode(port: str) -> None:
    """
    ★★ إصغاء صرف — **لا يبثّ بايتاً واحداً** ⇒ يستحيل الصدى بالبناء.
    كل إطار صالح يصل هنا هو حتماً من الطرف الآخر. الاستعمال: شغّله ثم
    اضغط أزرار شاشة CYD وراقب وصول إطارات `C,<seq>,<cmd>,…`.
    """
    ser = _open(port)
    print(f"إصغاء صرف على {port} @ {BAUD} — لا إرسال إطلاقاً. "
          "اضغط أزرار الشاشة الآن. Ctrl-C للإيقاف.\n")
    real_rx = corrupt = 0
    rd = LineBuffer(ser)
    try:
        while True:
            for raw in rd.lines():
                payload = parse_frame(raw)
                if payload is None:
                    corrupt += 1
                    print(f"← تالف/checksum خاطئ: {raw.strip()!r}")
                    continue
                real_rx += 1
                print(f"← استُقبل {payload} ✅ (لا إرسال منا ⇒ ليس صدى)")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    _verdict(real_rx, 0, corrupt)


def hex_mode(port: str, send_probe: bool = False) -> None:
    """
    ◆ تشريح بايتي: يطبع **كل** ما يصل hex + ASCII مع فارق الزمن، ويحكم
    على كل سطر مكتمل (سليم/تالف وسبب التلف). مع `send_probe` يبثّ أيضاً
    `$PROBE,<n>` كل ثانيتين — فيُختبر الاتجاهان بأداة واحدة.

    لماذا hex؟ «إطار مرفوض» يخفي البصمة. شكل البايتات يفرّق العلل:
    خربشة = باود/كهرباء · بتر = تأطير · بتات مقلوبة قليلة = RF/تشبّع.
    """
    ser = _open(port)
    role = "probe (بثّ + تشريح)" if send_probe else "hex (تشريح صرف — لا إرسال)"
    print(f"وضع {role} على {port} @ {BAUD}. Ctrl-C للإيقاف.\n")
    rd = LineBuffer(ser)
    n_probe = 0
    last_tx = 0.0
    last_rx = None
    frames_ok = frames_bad = 0
    try:
        while True:
            now = time.time()
            if send_probe and (now - last_tx) >= 2.0:
                last_tx = now
                n_probe += 1
                f = build_frame(f"PROBE,{n_probe}")
                ser.write(f)
                print(f"TX> {f.decode('ascii').strip()}")

            chunk = ser.read(64)
            if chunk:
                dt = "" if last_rx is None else f" (+{(now - last_rx)*1000:.0f}ms)"
                last_rx = now
                hx = " ".join(f"{b:02X}" for b in chunk)
                asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
                print(f"RX<{dt} hex: {hx}")
                print(f"    ascii: \"{asc}\"")
                # حكم على الأسطر المكتملة عبر نفس المجمّع
                rd.buf += chunk
                while b"\n" in rd.buf:
                    line, rd.buf = rd.buf.split(b"\n", 1)
                    s = line.decode("ascii", errors="replace").strip()
                    if not s:
                        continue
                    p = parse_frame(s)
                    if p is None:
                        frames_bad += 1
                        print(f"    ⇒ ❌ إطار تالف: {s!r}")
                    else:
                        frames_ok += 1
                        print(f"    ⇒ ✅ إطار سليم: {p}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print(f"\n─── الخلاصة: سليمة {frames_ok} · تالفة {frames_bad} ───")
    if frames_bad and not frames_ok:
        print("كل الأطر تالفة — انظر شكل الـhex أعلاه:")
        print("  خربشة عشوائية ⇒ باود/سلك/تغذية · نص مبتور ⇒ تأطير ·")
        print("  بتات مقلوبة قليلة ⇒ RF: باعد الوحدتين مترين أو AT+P6 مؤقتاً.")


def main() -> None:
    ser = _open(PORT)
    print(f"وصلة اللورا على {PORT} @ {BAUD}. يبثّ PING كل ثانية ويصغي. "
          "Ctrl-C للإيقاف.\n")
    seq = 0
    last_ping_ms = 0.0
    ping_sent_at = {}
    sent_recently = {}            # payload → وقت الإرسال (لكشف الصدى)
    real_rx = echoes = corrupt = 0
    echo_explained = False
    rd = LineBuffer(ser)
    try:
        while True:
            now = time.time()
            if (now - last_ping_ms) >= 1.0:
                last_ping_ms = now
                seq += 1
                payload = f"PING,{seq}"
                ser.write(build_frame(payload))
                ping_sent_at[seq] = now
                sent_recently[payload] = now
                print(f"→ أُرسل  {payload}")
            # تنظيف نافذة الصدى
            sent_recently = {p: t for p, t in sent_recently.items()
                             if (now - t) <= ECHO_WINDOW_S}

            for raw in rd.lines():
                payload = parse_frame(raw)
                if payload is None:
                    corrupt += 1
                    print(f"← تالف/checksum خاطئ: {raw.strip()!r}")
                    continue

                # 🔴 نفس ما أرسلناه للتوّ = صدى — تحذير لا استقبال
                if payload in sent_recently:
                    echoes += 1
                    print(f"← ⚠ صدى: {payload} (نفس ما أرسلناه — ليس رداً)")
                    if not echo_explained:
                        echo_explained = True
                        print("   ⚠ الصدى من HC-14/المحوّل نفسه ولا يثبت أي"
                              " وصلة لاسلكية (درس 2026-08-06).")
                    continue

                # رد PONG,<seq> → مختلف الصيغة عن PING ⇒ حقيقي، واحسب RTT
                if payload.startswith("PONG,"):
                    real_rx += 1
                    try:
                        rseq = int(payload.split(",")[1])
                        rtt_ms = (now - ping_sent_at.pop(rseq, now)) * 1000.0
                        print(f"← PONG seq={rseq}  RTT={rtt_ms:.0f}ms ✅")
                    except (IndexError, ValueError):
                        print(f"← {payload}")
                else:
                    real_rx += 1
                    print(f"← استُقبل {payload} ✅ (صيغة مختلفة عن المرسَل)")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    _verdict(real_rx, echoes, corrupt)


if __name__ == "__main__":
    _mode = sys.argv[1] if len(sys.argv) > 1 else ""
    _port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0"
    if _mode == "at":
        at_diagnostics(_port)
    elif _mode == "listen":
        listen_mode(_port)
    elif _mode == "hex":
        hex_mode(_port, send_probe=False)
    elif _mode == "probe":
        hex_mode(_port, send_probe=True)
    else:
        main()
