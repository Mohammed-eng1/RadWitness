# RUNBOOK — القيادة الذاتية بالليدار (RPLIDAR C1) · نسخة اختبار

تفادي عوائق في غرفة بلا خريطة. **التشغيل لا يُمنع أبداً** — كل مشكلة تحذير
مطبوع ثم يكمل. السجلات في `logs/lidar_drive_*.jsonl`.

```bash
cd ~/RMS-Rover-v2 && git pull && source venv/bin/activate
```

## 1) الليدار وحده
```bash
python3 -m pi.tests.ugv01.test_c1
```
المتوقع: المنفذ (CP210x) · صحّة «سليم» · ~10 لفّات/ث · ~500 نقطة/لفّة · «STOP أُرسل».

## 2) اتجاه الإطار (LIDAR_ANGLE_SIGN)
```bash
python3 -m pi.tests.ugv01.test_lidar_frame
```
ضع يدك/علبة على **يسار** الروبوت ⇒ يجب «يسار» وزاوية **موجبة** (~+90°).
- طبع «يمين» ⇒ اقلب `LIDAR_ANGLE_SIGN` في `pi/config.py`.
- جسم أمامه يظهر منحرفاً ثابتاً ⇒ `LIDAR_YAW_OFFSET_DEG`.

## 3) قناع أجزاء الروبوت (الكاميرا/الهوائي)
```bash
python3 -m pi.tests.ugv01.calibrate_lidar_mask
```
مكان مفتوح (لا شيء أقرب من ~0.5م) ولا تقف بجانبه. يطبع القطاعات المحجوبة
ويحفظ `pi/data/lidar_mask.json`. قطاع داخل ±30° = تحذير (أعد المعايرة).

## 4) تجربة بلا حركة — الروبوت **مرفوع**
```bash
python3 -m pi.nav.lidar_drive --dry-run
```
حرّك يدك/علبة أمامه ويمينه ويساره وراقب القرار: `go` · `arc` · `spin` ·
`backup` · `trapped`. لا أمر حركة يُرسل.

## 5) على الأرض ببطء
```bash
python3 -m pi.nav.lidar_drive --max-speed 0.10
```
غرفة فارغة، وأنت بجانبه. **Ctrl+C** يوقف (T:1 صفر + STOP لليدار + مهلة
الفيرموير تعود 3000ms). أول ثانية: تأكد أنه يتقدّم لا يرجع (خريطة المحركات
صُحّحت 2026-09-26 ولم تُؤكَّد بعد). ثم جرّب `--max-speed 0.15`.

من الواجهة: بطاقة «🛰 قيادة ذاتية بالليدار» (ابدأ/⛔ توقف، و«تجربة بلا حركة»)
مع `RMS_ROVER_MODE=real python -m pi.web.server`.

## اختبارات بلا عتاد
```bash
python3 -m pi.nav.lidar_selftest            # ثوانٍ
python3 -m pi.nav.lidar_drive --sim --dry-run
```

## القيم المؤقتة (كلها في `pi/config.py`، كتلة «ليدار RPLIDAR C1»)
`ROBOT_LENGTH_M` 0.30 · `ROBOT_WIDTH_M` 0.25 · `LIDAR_X_M`/`LIDAR_Y_M` 0 ·
`LIDAR_YAW_OFFSET_DEG` 0 · `LIDAR_AVOID_START_M` 0.80 · `LIDAR_GAP_FREE_M` 0.60 ·
`LIDAR_ARC_MAX_DEG` 25 · `LIDAR_BACK_CLEAR_M` 0.30 · `LIDAR_BACKUP_M` 0.15.
