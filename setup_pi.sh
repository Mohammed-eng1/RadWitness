#!/usr/bin/env bash
# setup_pi.sh — إعداد راسبري باي 4 كامل وغير تفاعلي لـ RMS Rover v2
# ═══════════════════════════════════════════════════════════════════
# يفعل: تحديث النظام، تفعيل UART (للـGPS) وI2C (للـBNO055)، تثبيت مكتبة
#        lgpio (لعدّ نبضات الجيجر — بديل pigpio الحديث على Debian trixie،
#        بلا daemon)، وإنشاء بيئة افتراضية + تثبيت المتطلبات.
#
# التشغيل على الراسبري (من جذر المستودع):
#     chmod +x setup_pi.sh && ./setup_pi.sh
# ثم انسخ ملفات الأسرار:
#     cp secrets.example.py secrets.py   # واملأه
#
# آمن للإعادة: يتخطى ما هو مثبت/مفعّل مسبقاً. يتطلب صلاحية sudo.
set -euo pipefail

echo "══════════════════════════════════════════════════════════"
echo "  RMS Rover v2 — إعداد الراسبري"
echo "══════════════════════════════════════════════════════════"

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

# ── 1) تحديث النظام وحزم النظام اللازمة ────────────────────────────
echo "[1/4] تحديث النظام وتثبيت حزم النظام…"
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -y
# الأساسية: python3-lgpio لعدّ الجيجر (بلا daemon)، i2c-tools لفحص BNO055
sudo apt-get install -y \
    python3 python3-venv python3-dev python3-pip \
    python3-lgpio \
    i2c-tools \
    git
# مكتبات تشغيل OpenCV (أفضل جهد — أسماؤها تختلف بين إصدارات ديبيان بسبب انتقال
# t64، فلا نُفشل الإعداد إن تعذّرت؛ الكاميرا تُستخدم في M3):
sudo apt-get install -y libgl1 libglib2.0-0t64 2>/dev/null || \
    sudo apt-get install -y libgl1 libglib2.0-0 2>/dev/null || \
    echo "    ⚠ تعذّر تثبيت مكتبات OpenCV الاختيارية — تُعالَج في M3 عند الحاجة"

# ── 2) تفعيل UART (GPS على GPIO15 RXD) وتحرير المنفذ من كونسول النظام ─
echo "[2/4] تفعيل UART للـGPS…"
# تحديد ملف إعداد الإقلاع (Bookworm/trixie: /boot/firmware، الأقدم: /boot)
if [ -f /boot/firmware/config.txt ]; then
    BOOT_CFG=/boot/firmware/config.txt
else
    BOOT_CFG=/boot/config.txt
fi
if ! grep -q "^enable_uart=1" "$BOOT_CFG"; then
    echo "enable_uart=1" | sudo tee -a "$BOOT_CFG" >/dev/null
    echo "    أُضيف enable_uart=1 إلى $BOOT_CFG"
fi
# تعطيل كونسول السيريال حتى يتفرّغ المنفذ للـGPS (تجاهل الفشل على بعض الإصدارات)
sudo raspi-config nonint do_serial_cons 1 2>/dev/null || \
    sudo raspi-config nonint do_serial 1  2>/dev/null || \
    echo "    ⚠ عطّل كونسول السيريال يدوياً عبر raspi-config إن لزم"

# ── 3) تفعيل I2C: الناقل 1 (المحركات والبطارية) + الناقل 4 (MPU) ───
echo "[3/4] تفعيل I2C (ناقل 1 للمحركات/البطارية · ناقل 4 للـMPU)…"
# الناقل 1 — عليه PCA9685 @0x40 (محركات Freenove) وINA219 @0x42 (البطارية)
sudo raspi-config nonint do_i2c 0 2>/dev/null || \
    echo "    ⚠ فعّل I2C يدوياً عبر raspi-config إن لزم"
# 🔴 الناقل 4 — وحدة MPU على GPIO6/7. **بلا هذا السطر لا يوجد /dev/i2c-4**
#    فيسقط مصدر الاتجاه إلى المحاكاة، ويُرفض المسح الذاتي بسبب معلن.
#    (كان ناقصاً من هذا السكربت فيُضبط يدوياً — أُضيف 2026-09-12.)
if ! grep -q "^dtoverlay=i2c4" "$BOOT_CFG"; then
    echo "dtoverlay=i2c4,pins_6_7,baudrate=100000" | sudo tee -a "$BOOT_CFG" >/dev/null
    echo "    أُضيف dtoverlay=i2c4,pins_6_7 إلى $BOOT_CFG (⚠ يلزم إعادة إقلاع)"
    NEED_REBOOT=1
fi

# ── 4) بيئة افتراضية + متطلبات بايثون ──────────────────────────────
echo "[4/4] إنشاء بيئة افتراضية وتثبيت المتطلبات…"
# --system-site-packages: لتظهر حزم النظام (lgpio/opencv) للبيئة عند الحاجة
if [ ! -d venv ]; then
    python3 -m venv --system-site-packages venv
fi
# shellcheck disable=SC1091
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
deactivate

echo "══════════════════════════════════════════════════════════"
echo "  اكتمل الإعداد ✅  (lgpio لا يحتاج daemon — عدّ الجيجر جاهز مباشرة)"
echo "──────────────────────────────────────────────────────────"
echo "  الخطوات التالية:"
echo "   1) انسخ الأسرار:  cp secrets.example.py secrets.py  ثم املأه"
echo "   2) أعد الإقلاع لتفعيل UART/I2C:  sudo reboot"
echo "   3) فعّل البيئة واختبر الحساسات:"
echo "        source venv/bin/activate"
echo "        python3 pi/tests/test_geiger.py"
echo "══════════════════════════════════════════════════════════"

if [ "${NEED_REBOOT:-0}" = "1" ]; then
    echo
    echo "🔴 أُضيف dtoverlay جديد — **أعد الإقلاع** ثم تحقّق:"
    echo "      sudo reboot"
    echo "      ls /dev/i2c-*        # يجب أن يظهر i2c-1 و i2c-4"
    echo "      i2cdetect -y 4       # توقّع 0x68 (وحدة MPU)"
fi
