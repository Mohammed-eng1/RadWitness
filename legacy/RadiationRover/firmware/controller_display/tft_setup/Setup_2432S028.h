// ============================================================
//  إعداد TFT_eSPI — ESP32-2432S028 (البديل)
//  ILI9341 240×320 — عائلة CYD 2.8"
//  انسخ محتوى هذا الملف فوق User_Setup.h في مجلد مكتبة TFT_eSPI،
//  أو أضف سطر تضمينه في User_Setup_Select.h. انظر README.
//
//  الشاشة على HSPI (USE_HSPI_PORT) — اللمس XPT2046 على VSPI منفصل
//  يُدار بمكتبة XPT2046_Touchscreen في controller_display.ino (لا هنا).
//  الإضاءة الخلفية IO21 تُدار يدوياً (PWM) في السكتش — لا نعرّف TFT_BL هنا.
// ============================================================
#define USER_SETUP_ID 2432028

#define ILI9341_2_DRIVER   // بعض ألواح 2432S028 تحتاج ILI9341_2 (لو انقلبت الألوان جرّب ILI9341_DRIVER)
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

#define USE_HSPI_PORT

#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1

#define TFT_RGB_ORDER TFT_BGR   // لوح CYD 2.8" غالباً BGR

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_GFXFF
#define SMOOTH_FONT

#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY  16000000
