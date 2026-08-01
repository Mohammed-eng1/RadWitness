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
| `test_ir.py` | حساس عائق IR (اختبار مكتبي) | `python3 pi/tests/test_ir.py` | OUT=GPIO25 ⚠ **تغذية 3.3V** |
| `calibrate_heading.py` | **معايرة الاتجاه** (انحياز/معامل/KP) — يحرّك المحركات في المرحلتين 2 و3 | `python3 -m pi.tests.calibrate_heading` | BNO055 I2C **0x29** + الروفر على `/dev/serial0` |
| `check_imu_health.py` | **«الحسّاس ميت» أم عاد إلى CONFIG؟** يقرأ CHIP_ID/OPR_MODE/SYS_ERR ويحصي الصفر المضبوط | `python3 -m pi.tests.check_imu_health` (أضف `--motors` لإعادة إنتاج انهيار التغذية ⚠ يحرّك الروبوت) | BNO055 على i2c-**4** @ 0x29 |
| `check_visual_heading.py` | **معايرة الاتجاه البصري**: جودة المشهد ثم قياس `CAMERA_HFOV_DEG` و`VISUAL_YAW_SIGN` | `python3 -m pi.tests.check_visual_heading` (أضف `--calibrate` ⚠ يلفّ الروبوت) | ويب كام + الروفر. ⚠ **أوقف السيرفر** (يمسك `/dev/video0`) |

ملاحظات:
- **الجيجر** يستخدم `lgpio` (بلا daemon — بديل pigpio المحذوف من Debian trixie).
  إن ظهر خطأ صلاحيات، تأكد أن مستخدمك ضمن مجموعة `gpio`: `groups | grep gpio`.
- **GPS** قد يستغرق دقائق للقفل الأول في العراء.
- ⚠ **«40 قراءة صفر مضبوط متتابعة — الحسّاس ميت؟» في سجل المهمة لا تعني الموت
  بالضرورة**: في وضع **CONFIG** تقرأ كل سجلات بيانات BNO055 `0x00`، والشريحة
  تعود إلى CONFIG وحدها بعد أي إعادة تشغيل ذاتية (أشيع سبب: هبوط جهد 3.3V عند
  اندفاع تيار المحركات). فهي حيّة على الناقل وتردّ بهويتها والجايرو صفر إلى
  الأبد. مصدر الاتجاه يحاول الإحياء تلقائياً قبل إعلان العطل؛ ولحسم السبب:
  `python3 -m pi.tests.check_imu_health --motors`.
- **BNO055**: افحص الوجود أولاً `i2cdetect -y 4` (يجب أن يظهر **29** — الناقل
  4 لا 1، انظر `BNO055_I2C_BUS`). صار **مصدر
  الاتجاه الأساسي** بعد موت جايرو الروفر، ويعمل في وضع **IMUPLUS بلا مغنيتومتر**
  → لا حاجة للتلويح على شكل ∞، و`mag_cal = 0` **متوقَّع** (المهم `gyro_cal ≥ 2`).
  إجراء المعايرة الكامل في [`docs/PATCH_HEADING_SOURCE.md`](../../docs/PATCH_HEADING_SOURCE.md).
- **الكاميرا** تحفظ اللقطة في `captures/`.
- ثوابت معايرة الجيجر (K=111، τ=200µs) مثبتة مخبرياً ضد Cs-137 — لا تُغيَّر.

جدول التوصيلات الكامل والتحذيرات الحرجة في [`docs/wiring.md`](../../docs/wiring.md).
