// ============================================================
//  إعداد TFT_eSPI — ESP32-3248S035R (الافتراضي)
//  ST7796 320×480 — عائلة CYD 3.5"
//  انسخ محتوى هذا الملف فوق User_Setup.h في مجلد مكتبة TFT_eSPI،
//  أو أضف سطر تضمينه في User_Setup_Select.h. انظر README.
//
//  ★ مطابق للإعداد المُجرَّب عملياً على العتاد ★
//  اللمس XPT2046 يشارك ناقل SPI الشاشة (12/13/14)، CS مستقل = 33.
//  TFT_eSPI يتولى اللمس عبر tft.getTouch() — بلا USE_HSPI_PORT ولا ناقل منفصل.
//  الإضاءة الخلفية IO27 تُدار يدوياً (PWM) في السكتش أيضاً.
// ============================================================
#define USER_SETUP_ID 3248035

#define ST7796_DRIVER

// دبابيس الشاشة (SPI)
#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1        // موصول بـEN — لا منفذ مخصص
#define TFT_BL   27        // الإضاءة الخلفية

// شريحة اللمس XPT2046 — على نفس ناقل الشاشة، CS مستقل
#define TOUCH_CS 33

// الخطوط
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4

// الترددات (كما في الإعداد المُجرَّب)
#define SPI_FREQUENCY        65000000
#define SPI_TOUCH_FREQUENCY  2500000
