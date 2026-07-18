# RadiationRover

روفر فحص إشعاعي ذاتي على منصة SunFounder GalaxyRVR. التفاصيل الكاملة: `CLAUDE.md` (القواعد) و`docs/BRIEF.md` (المهمة) و`docs/wiring.md` (التوصيلات).

## الترجمة — Arduino IDE

### الهَب `firmware/esp32_main/` — ESP32-S3 DevKitC N16R8

إعدادات اللوحة (Tools):

| الإعداد | القيمة |
|---------|--------|
| Board | **ESP32S3 Dev Module** |
| Flash Size | **16MB (128Mb)** |
| Partition Scheme | مخطط 16MB بمساحة تطبيق ≥ 3MB (مثل: `16M Flash (3MB APP/9.9MB FATFS)` أو `Default 16MB`) |
| PSRAM | **OPI PSRAM** (الشريحة الثمانية في N16R8) |
| USB CDC On Boot | Enabled (للمونيتور عبر USB المدمج) |

> ⚠ قيود منافذ S3 — انظر `docs/wiring.md`: ممنوع 19/20 (USB) و26-32 (فلاش) و33-37 (PSRAM).

المكتبات: ESPAsyncWebServer، AsyncTCP، TinyGPSPlus، Adafruit BNO055، Adafruit Unified Sensor، ArduinoJson (+ SD/SPI/Wire/LittleFS مدمجة).

### فيرموير الشاشة `firmware/controller_display/` — ESP32-3248S035R (أو 2432S028)

- Board: **ESP32 Dev Module** (الشاشتان WROOM-32 عادي).
- اختر الموديل بعلم `DISPLAY_MODEL` في `config.h` الخاص بها.
- **إعداد TFT_eSPI**: انسخ محتوى ملف الإعداد المطابق لموديلك من `firmware/controller_display/tft_setup/` فوق ملف `User_Setup.h` في مجلد مكتبة TFT_eSPI:
  - `Setup_3248S035R.h` — الافتراضي (ST7796، 320×480)
  - `Setup_2432S028.h` — البديل (ILI9341، 240×320)
- المكتبات: TFT_eSPI، XPT2046_Touchscreen، WebSockets (arduinoWebSockets — عميل فقط)، ArduinoJson (+ WiFi/ESPmDNS مدمجة).
- **الشبكة (STA فقط)**: الشاشة تتصل بنفس شبكة الهَب — لا تصنع شبكتها. بيانات الشبكة في `secrets.h` بنفس أسماء الهَب (`WIFI_SSID`/`WIFI_PASSWORD`) — **متغيّر شبكة واحد**. الشاشة وحدة تحكم لمسية فقط؛ الموقع الشامل (خرائط/نطاق/شات بوت/قيادة) على الهَب `rover.local` يُفتح من أي متصفح على نفس الشبكة.

### البقية

- `firmware/arduino_rover/` — Arduino Uno R3 (سائق المحركات).
- `firmware/esp32cam_bridge/` — ESP32-CAM (AI Thinker).
- `firmware/reference/` — سكتشات اختبار مثبتة على العتاد (مرجع للمنافذ والحيل — لا تُرفع للروفر).

## الأدوات

- `tools/log_parser.py` و`tools/heatmap_generator.py` — تحليل سجلات المهمات CSV.
