/*
 * config.h — فيرموير شاشة CYD (وحدة تحكم LoRa)
 * ==================================================================
 * كل المنافذ والثوابت هنا حصراً (نفس قاعدة `pi/config.py` على الراسبري).
 *
 * ⚠ المنافذ **منقولة من الفيرموير القديم المجرَّب** على نفس اللوحات:
 *   `legacy/RadiationRover/firmware/controller_display/config.h`.
 *   لم تُخترع أرقام جديدة — هذه عملت فعلاً على هذا العتاد.
 *
 * اختيار الموديل: DISPLAY_MODEL
 *   1 = ESP32-3248S035R: ST7796 320×480، لمس XPT2046، الإضاءة IO27
 *   2 = ESP32-2432S028 : ILI9341 240×320، الإضاءة IO21
 * ملف إعداد TFT_eSPI المطابق في legacy/.../tft_setup/.
 */
#pragma once

#define DISPLAY_MODEL 1        // 1 = 3248S035R (افتراضي) | 2 = 2432S028

// ═══ منافذ حسب الموديل (مجرَّبة) ══════════════════════════════
#if DISPLAY_MODEL == 1
  #define TFT_BL_PIN     27    // ⚠ الإضاءة الخلفية حصراً — PWM
  #define LORA_RX_PIN    21    // ← HC-14 TX
  #define LORA_TX_PIN    22    // → HC-14 RX
#else
  #define TFT_BL_PIN     21
  #define LORA_RX_PIN    35
  #define LORA_TX_PIN    27
#endif

// 🔴 يجب أن يطابق `LORA_BAUD` في pi/config.py — عدم التطابق يعطي
//    أطراً تالفة باستمرار وهو أشيع سبب لـ«الراديو لا يعمل».
#define LORA_BAUD       9600

// اللمس XPT2046 — SPI مستقل (مشترك بين الموديلين، عائلة CYD)
#define TOUCH_CLK_PIN  25
#define TOUCH_MOSI_PIN 32
#define TOUCH_MISO_PIN 39
#define TOUCH_CS_PIN   33
#define TOUCH_IRQ_PIN  36
#define TOUCH_RAW_MIN  200
#define TOUCH_RAW_MAX  3700
#define TOUCH_PRESSURE_TH 40   // فوقها = لمسة حقيقية (السكون ~10-17)
#define TOUCH_SWAP_XY  0
#define TOUCH_INVERT_X 0
#define TOUCH_INVERT_Y 0

// ═══ البروتوكول (يطابق pi/comms/protocol.py) ══════════════════
#define MAX_PAYLOAD     56     // = LORA_MAX_PAYLOAD
#define SEQ_MODULO      1000   // = LORA_SEQ_MODULO

// ═══ إيقاعات ══════════════════════════════════════════════════
#define TFT_BACKLIGHT_PCT   60
#define UI_UPDATE_MS        250    // إعادة رسم الحقول المتغيّرة فقط
// 🔴 تجديد أمر الحركة المضغوط: **أقصر من مهلة الراديو** (2000ms في
//    pi/config.py). 500ms يعني أن فقدان إطارين متتاليين لا يقطع الأمر.
#define DRIVE_REPEAT_MS     500
// بلا أي تيليمتري خلالها ⇒ القناة تُعلَن ميتة (الروبوت يبثّ كل 2ث)
#define LORA_TIMEOUT_MS     8000

// ═══ كشف وضع الجسر ════════════════════════════════════════════
// ⚠⚠ **ESP32 الكلاسيكي لا يكشف وصل USB مباشرةً** ⚠⚠
// اللوحة تستعمل محوّل CH340/CP2102 على UART0، ولا يوجد مكدّس USB أصيل
// (بخلاف ESP32-S2/S3) — فلا حدث «وُصِل» ولا خط DTR مقروء. و`Serial`
// تُعيد `true` دائماً على UART عادي، فاستعمالها كـ«هل USB موصول؟» يعطي
// **جسراً دائماً** ويقتل الوضع المستقل تماماً.
// ⇒ الكشف **بالحركة لا بالكهرباء**: أول إطار صالح من UART0 يُدخل وضع
//   الجسر، وصمتٌ بطول BRIDGE_IDLE_MS يُعيد الوضع المستقل. وهذا هو
//   السلوك المطلوب فعلاً: «هل يقودني حاسوب الآن؟» لا «هل الكبل مركّب؟»
#define BRIDGE_IDLE_MS      5000
#define USB_BAUD            115200 // باود المتصفح ↔ الشاشة (Web Serial)

// ═══ ألوان الواجهة (داكنة — تطابق لوحة الموقع) ════════════════
#define COL_BG      0x0861     // #0b0f17
#define COL_PANEL   0x18C3     // #131a26
#define COL_LINE    0x2247     // #243149
#define COL_FG      0xE71C     // #e6ecf7
#define COL_DIM     0x8410     // #8a93a6
#define COL_OK      0x1DE6     // #22c55e
#define COL_WARN    0xFB40     // #f59e0b
#define COL_BAD     0xE9A6     // #ef4444
#define COL_ACCENT  0x3C1F     // #3b82f6
