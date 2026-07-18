#!/usr/bin/env bash
# setup_pi.sh — إعداد راسبري باي 4 كامل وغير تفاعلي لـ RMS Rover v2
# ═══════════════════════════════════════════════════════════════════
# يفعل: تحديث النظام، تفعيل UART (للـGPS) وI2C (للـBNO055)، تثبيت وتشغيل
#        pigpiod (لعدّ نبضات الجيجر)، إنشاء بيئة افتراضية + تثبيت المتطلبات.
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
echo "[1/5] تحديث النظام وتثبيت حزم النظام…"
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -y
# libatlas/libjpeg لـopencv و numpy، i2c-tools لفحص BNO055، pigpio لعدّ الجيجر
sudo apt-get install -y \
    python3 python3-venv python3-dev python3-pip \
    pigpio python3-pigpio \
    i2c-tools \
    libatlas-base-dev libjpeg-dev libopenjp2-7 \
    git

# ── 2) تفعيل UART (GPS على GPIO15 RXD) وتحرير المنفذ من كونسول النظام ─
echo "[2/5] تفعيل UART للـGPS…"
# تحديد ملف إعداد الإقلاع (Bookworm: /boot/firmware، الأقدم: /boot)
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

# ── 3) تفعيل I2C (BNO055 على GPIO2/3) ──────────────────────────────
echo "[3/5] تفعيل I2C للـBNO055…"
sudo raspi-config nonint do_i2c 0 2>/dev/null || \
    echo "    ⚠ فعّل I2C يدوياً عبر raspi-config إن لزم"

# ── 4) تفعيل وتشغيل pigpiod (لازم لعدّ نبضات الجيجر عبر pigpio) ──────
echo "[4/5] تفعيل وتشغيل pigpiod…"
sudo systemctl enable pigpiod
sudo systemctl start  pigpiod

# ── 5) بيئة افتراضية + متطلبات بايثون ──────────────────────────────
echo "[5/5] إنشاء بيئة افتراضية وتثبيت المتطلبات…"
# --system-site-packages: لتظهر حزم النظام (pigpio/opencv) للبيئة عند الحاجة
if [ ! -d venv ]; then
    python3 -m venv --system-site-packages venv
fi
# shellcheck disable=SC1091
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
deactivate

echo "══════════════════════════════════════════════════════════"
echo "  اكتمل الإعداد ✅"
echo "──────────────────────────────────────────────────────────"
echo "  الخطوات التالية:"
echo "   1) انسخ الأسرار:  cp secrets.example.py secrets.py  ثم املأه"
echo "   2) أعد الإقلاع لتفعيل UART/I2C:  sudo reboot"
echo "   3) فعّل البيئة واختبر الحساسات:"
echo "        source venv/bin/activate"
echo "        python3 pi/tests/test_geiger.py"
echo "══════════════════════════════════════════════════════════"
