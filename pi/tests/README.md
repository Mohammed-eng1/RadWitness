# سكربتات اختبار الحساسات المنفردة — `pi/tests/`

> فلسفة المشروع: **كل حساس يُقرأ منفرداً بسكربته قبل أي دمج**. هذه السكربتات
> تعمل على الراسبري (تحتاج العتاد الحقيقي المتصل)؛ من الطبيعي أن تفشل على جهاز
> التطوير المكتبي لغياب المكتبات/العتاد.

قبل التشغيل: نفّذ `setup_pi.sh` وفعّل البيئة الافتراضية:

```bash
source venv/bin/activate
```

| السكربت | يختبر | التشغيل | التوصيل |
|---|---|---|---|
| `test_geiger.py` | عدّ جيجر (lgpio) + CPM + معايرة µSv/h | `python3 pi/tests/test_geiger.py` | GPIO17، ⚠ تغذية 3.3V |
| `test_gps.py` | قفل GPS، إحداثيات، أقمار، HDOP | `python3 pi/tests/test_gps.py` | GPIO15 RXD، 9600 |
| `test_bno055.py` | heading + حالة المعايرة | `python3 pi/tests/test_bno055.py` | I2C GPIO2/3، 0x28 |
| `test_camera.py` | فتح الكاميرا + حفظ لقطة | `python3 pi/tests/test_camera.py` | USB (index 0) |
| `test_lora.py` | حيّة وصلة HC-14 + RTT | `python3 pi/tests/test_lora.py` | USB-Serial /dev/ttyUSB0 |
| `test_ultrasonic.py` | مسافة HC-SR04 (اختبار مكتبي) | `python3 pi/tests/test_ultrasonic.py` | TRIG=GPIO23، ECHO=GPIO24 ⚠ **مقسّم جهد** |

ملاحظات:
- **الجيجر** يستخدم `lgpio` (بلا daemon — بديل pigpio المحذوف من Debian trixie).
  إن ظهر خطأ صلاحيات، تأكد أن مستخدمك ضمن مجموعة `gpio`: `groups | grep gpio`.
- **GPS** قد يستغرق دقائق للقفل الأول في العراء.
- **BNO055**: افحص الوجود أولاً `i2cdetect -y 1`، ولوّح على شكل ∞ حتى `mag ≥ 2`.
- **الكاميرا** تحفظ اللقطة في `captures/`.
- ثوابت معايرة الجيجر (K=111، τ=200µs) مثبتة مخبرياً ضد Cs-137 — لا تُغيَّر.

جدول التوصيلات الكامل والتحذيرات الحرجة في [`docs/wiring.md`](../../docs/wiring.md).
