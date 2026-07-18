/*
 * config.h — فيرموير الشاشة (بوابة AP + جسر للهَب)
 * ==================================================
 * كل المنافذ والثوابت هنا حصراً. أسرار الشبكة في secrets.h فقط.
 *
 * اختيار الموديل: DISPLAY_MODEL
 *   1 = ESP32-3248S035R (الافتراضي): ST7796 بدقة 320×480، لمس XPT2046،
 *       الإضاءة الخلفية IO27 — لا تستخدمه لغير PWM الإضاءة.
 *   2 = ESP32-2432S028 (البديل): ILI9341 بدقة 240×320، الإضاءة IO21.
 * ملف إعداد TFT_eSPI المطابق في tft_setup/ — انظر README.
 */
#pragma once

#define DISPLAY_MODEL 1        // 1 = 3248S035R (افتراضي) | 2 = 2432S028

// ═══ منافذ حسب الموديل ═══════════════════════════════════════
#if DISPLAY_MODEL == 1
  // ESP32-3248S035R — ST7796 320×480
  #define TFT_BL_PIN     27    // ⚠ الإضاءة الخلفية حصراً — PWM
  #define LORA_RX_PIN    21    // ← HC-14 TX (مع 3.3V على نفس الموصل)
  #define LORA_TX_PIN    22    // → HC-14 RX
#else
  // ESP32-2432S028 — ILI9341 240×320
  #define TFT_BL_PIN     21    // الإضاءة الخلفية
  #define LORA_RX_PIN    35    // ← HC-14 TX
  #define LORA_TX_PIN    27    // → HC-14 RX
#endif
#define LORA_BAUD       9600   // يجب أن يتطابق مع باود HC-14 في الهَب
#define LORA_TIMEOUT_MS 12000  // بلا استقبال LoRa خلالها = القناة ميتة (الهَب يبثّ كل 5s)
#define LORA_ASSUME_LINK 0     // 1 = اعتبر لورا حيّة دائماً (اختبار فقط)

// اللمس XPT2046 — على SPI مستقل (مشترك بين الموديلين، عائلة CYD)
#define TOUCH_CLK_PIN  25
#define TOUCH_MOSI_PIN 32
#define TOUCH_MISO_PIN 39
#define TOUCH_CS_PIN   33
#define TOUCH_IRQ_PIN  36

// حدود اللمس الخام (XPT2046 يعطي 0-4095) — اضبطها عملياً إن انحرف اللمس
#define TOUCH_RAW_MIN  200
#define TOUCH_RAW_MAX  3700

// عتبة الضغط (z): فوقها = لمسة حقيقية. السكون ~10-17، فاجعلها فوقه بهامش.
// ارفعها لو ظهرت لمسات وهمية، أو اخفضها لو لم تُسجَّل ضغطاتك.
#define TOUCH_PRESSURE_TH  40

// معايرة اللمس (موديل 1): تُحفظ في NVS وتُعاد تلقائياً. اجعله 1 لإعادة
// المعايرة مرة واحدة (المس الأركان الأربعة كما تُطلب)، ثم أرجعه 0.
#define TOUCH_FORCE_CALIBRATE 0

// ضبط اتجاه اللمس (تُطبَّق بعد القراءة — راقب [TOUCH] على السيريال إن احتجت):
//   موديل 1 (getTouch): يتولى الاتجاه غالباً؛ اقلب أدناه فقط لو ظهر انعكاس.
//   موديل 2 (XPT2046 خام): عادةً INVERT_X=1 وINVERT_Y=1، وSWAP لو صار قطرياً.
#define TOUCH_SWAP_XY  0   // بدّل محوري X/Y (موديل 2 فقط)
#define TOUCH_INVERT_X 0   // اعكس الأفقي (يمين↔يسار)
#define TOUCH_INVERT_Y 0   // اعكس الرأسي (فوق↔تحت)

// ═══ الشبكة ══════════════════════════════════════════════════
//   AP+STA: الشاشة تبث شبكتها (RMS-CONTROL) لجوالك القريب، وتتصل بشبكة
//   الهَب (secrets.h WIFI_*) لما تكون قريبة. الموقع الخفيف (قيادة+قراءات)
//   يُستضاف على الشاشة؛ يجسر للهَب عبر WiFi (قريب) أو LoRa (بعيد).
#define AP_SSID        "RMS-CONTROL"   // شبكة الشاشة (كلمة المرور في secrets.h)
#define AP_CHANNEL     6
#define DNS_PORT       53              // captive portal: كل النطاقات → 192.168.4.1
#define HUB_MDNS_NAME  "rover"         // اسم الهَب على شبكة الراوتر
#define HUB_WS_PORT    80              // WebSocket الهَب /ws
#define HUB_HTTP_PORT  80
#define LOCAL_STATUS_MS 1000           // بث حالة الجسر لعملاء شبكة الشاشة

// ═══ إيقاعات وحدود ═══════════════════════════════════════════
#define TFT_BACKLIGHT_PCT   60         // سطوع افتراضي %
#define TFT_UPDATE_MS       250        // تحديث الشاشة من المتغيرات المحفوظة
#define GRAPH_N             200        // طول حلقة عيّنات CPM لشاشة الرسم البياني
#define GRAPH_SAMPLE_MS     2000       // فاصل أخذ عيّنة CPM للرسم البياني
#define HUB_TIMEOUT_MS      3000       // لا رسالة من الهَب خلالها = الوصلة مقطوعة
#define HUB_RESOLVE_MS      10000      // إعادة محاولة mDNS للهَب
#define DRIVE_HEARTBEAT_MS  250        // تجديد أمر حركة لمسي مضغوط (WiFi) — أسرع من مهلة 1.5s
#define LORA_DRIVE_HB_MS    500        // تجديد اللورا: 500ms (فقد إطار واحد = فجوة 1s < 1.5s
                                       // مهلة الأردوينو، فلا تقطّع). الهَب يصمت أثناء التحكم فلا تكدّس.
