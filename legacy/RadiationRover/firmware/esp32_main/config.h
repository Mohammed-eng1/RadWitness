/*
 * config.h — كل المنافذ والثوابت والعتبات في مكان واحد حصراً
 * ============================================================
 * أسرار الشبكة (SSID/كلمة المرور) في secrets.h فقط — ليست هنا.
 *
 * منطق الأعلام:
 *   SIMULATION_MODE   وضع Demo الدائم — يُبلَّغ في التيليمتري ويُفعّل
 *                     نموذج المصدر الافتراضي لأي حساس وهمي.
 *   USE_REAL_x        كل حساس يُفعَّل على حدة (المرحلة B) — الحساس
 *                     الحقيقي يستبدل الوهمي دون لمس أي كود فوقه.
 */
#pragma once

// ═══ أوضاع التشغيل ═══════════════════════════════════════════
#define SIMULATION_MODE   1

// أعلام العتاد الحقيقي — تُقلب واحدة واحدة عند وصول القطع (المرحلة B)
#define USE_REAL_GEIGER1  1   // ✅ B1: لوحة CAJOE (J305) — نبضات GPIO15 RISING عائم
#define USE_REAL_GEIGER2  0   // لم يصل بعد (انظر SINGLE_TUBE_MODE)
#define USE_REAL_IMU      1   // ✅ B1: BNO055 وصل — I2C 21/9، العنوان يُكتشف تلقائياً (B2)
#define USE_REAL_SD       1   // ✅ B2: كرت SD على FSPI — عند غيابه fallback تلقائي لLittleFS

// وضع الأنبوب الواحد (B1): أنبوب حقيقي واحد فقط متوفر — عطّل الثاني كلياً
// حتى لا تختلط محاكاة موازية بالقراءة الحقيقية. قلبه إلى 0 عند وصول الأنبوب الثاني.
#define SINGLE_TUBE_MODE  1
#define USE_REAL_GPS      1   // الـGPS الحقيقي موجود منذ البداية
#define SIM_GPS_WALK      0   // 1 = تجاهل موقع GPS وحاكِ الحركة من أوامر القيادة
                              //     (Demo مكتبي/اختبار داخلي) — صفّره للاختبار الخارجي

// ═══ الشبكة: الأسماء والمنافذ (البروتوكول الثابت — لا يُكسر) ═══
#define MDNS_NAME         "rover"
#define CAM_HOST_NAME     "rovercam"
#define CAM_STREAM_PORT   81      // بث الكاميرا :81/stream
#define UDP_CAM_CMD_PORT  4210    // هَب → كام: أوامر حركة نصية
#define UDP_LOCAL_PORT    4211    // منفذ الإرسال المحلي على الهَب
#define UDP_TELE_PORT     4212    // كام → هَب: بطارية "BV:x.xx"
#define UDP_DISPLAY_PORT  4220    // هَب → شاشة: تيليمتري JSON broadcast

// ═══ منافذ العتاد على الهَب ESP32-S3 DevKitC N16R8 ═══════════
//   (المرجع: docs/wiring.md — الخريطة مثبتة عملياً من
//    firmware/reference/gps_bno055_sd_logger_final.ino)
//   قيود S3: ممنوع 19/20 (USB D-/D+)، و26-32 (فلاش)، و33-37 (PSRAM الثماني)
#define GEIGER1_PIN  4            // نبضات أنبوب 1 (كان 15، وقبلها 32 — غير موجود في S3)
#define GEIGER2_PIN  5            // نبضات أنبوب 2 (لم يصل — SINGLE_TUBE_MODE؛ كان 33 = PSRAM في S3)
#define GPS_RX_PIN   16           // ← GPS TX (نقرأ فقط)
#define GPS_TX_PIN   -1           // غير موصول — كما في المرجع
#define GPS_BAUD     9600
#define I2C_SDA_PIN  21           // BNO055 (SDA)
#define I2C_SCL_PIN  9            // BNO055 (SCL) — ⚠ لا تُرجعه إلى 20:
                                  //   GPIO20 = USB D+ في S3، يقتل I2C عند التوصيل عبر USB
                                  //   (فشل حقيقي موثق في السكتش المرجعي)
#define SD_SCK_PIN   12           // SD عبر SPIClass spiSD(FSPI) — أسلوب المرجع حرفياً
#define SD_MOSI_PIN  11
#define SD_MISO_PIN  13
#define SD_CS_PIN    10
// لورا HC-14 على UART2 (GPS يشغل UART1) — قناة تحكم يدوية أولوية قصوى
#define LORA_RX_PIN  18           // ← HC-14 TX
#define LORA_TX_PIN  17           // → HC-14 RX
#define LORA_BAUD    9600         // باود HC-14 الافتراضي (يجب أن يتطابق الطرفان)
#define LORA_STATUS_MS 5000       // بث حالة نادر (نصف مزدوج: لا يخنق قناة الأوامر)
#define LORA_TX_STATUS 1          // 1 = بثّ الحالة مفعّل (كل LORA_STATUS_MS)
#define LORA_QUIET_AFTER_RX_MS 3000  // بعد أمر لورا يدوي: اصمت هذه المدة (أولوية قصوى
                                     // للأوامر على القناة نصف-المزدوجة — لا يزاحمها بثّ الحالة)

// حساسات العوائق موصولة بشيلد الأردوينو (لا بالهَب): الفوق-صوتي=منفذ 10
// (TRIG+ECHO سلك واحد)، IR يسار=8، IR يمين=7 — توصيل GalaxyRVR الرسمي.
// الأردوينو يقرأها ويرسلها "OBS:d,l,r" على Serial؛ الكام تمرّرها UDP 4212.
// المنافذ نفسها معرّفة في firmware/arduino_rover/arduino_rover.ino.

// ═══ الحركة: افتراضات فيزيائية (للمحاكاة وطبقة السلامة) ═══════
#define CMD_DEFAULT_POWER     70
#define DRIVE_SPEED_MPS       0.45f  // سرعة تقريبية عند قوة 70
#define TURN_RATE_DPS         70.0f  // معدل الدوران بالمكان عند قوة 70 (درجة/ث)
#define CMD_HEARTBEAT_MS      250    // إعادة إرسال أمر مستمر (مهلة الأردوينو 1.5s) —
                                     // أسرع = يتحمّل فقد حزم UDP دون تقطّع الحركة
#define UNO_SAFETY_TIMEOUT_MS 1500
// مزلاج الأمر اليدوي: الهَب يكرّر آخر أمر حركة من حلقته الموثوقة (لا من مؤقّت
// المتصفح الذي يتلعثم تحت حمل الكاميرا). ينتهي إن لم يصل تحديث خلال هذه المدة (أمان).
#define MANUAL_LATCH_EXPIRE_MS 1300

// ═══ الجيجر ═══════════════════════════════════════════════════
#define CPM_PER_USVH      111.0f  // مُعاير ضد مصدر Cs-137 مرجعي (662 keV)، 2026-07
                                  // (القيمة المقاسة عند أدنى معدل عد، الأصدق — كان 123 تقديرياً)
#define GEIGER_BUCKET_MS  10000   // نافذة CPM منزلقة: GEIGER_BUCKETS دلو × 10 ثوانٍ
#define GEIGER_BUCKETS    6
#define GEIGER_SIM_TICK_MS 100    // خطوة توليد النبضات الوهمية (بواسون)

// تصحيح الزمن الميت — يطبَّق على قراءات الأنبوب الحقيقي فقط قبل التحويل لجرعة
// (المحاكاة تولّد قيماً "حقيقية" أصلاً فلا تُصحَّح). τ مقاس من معايرة Cs-137.
#define USE_DEADTIME_CORRECTION 1
#define GEIGER_DEADTIME_TAU_S   0.0002f  // 200 µs

// صحة الأنبوب الحقيقي — كشف انفصال السلك (الخلفية الطبيعية لا تصمت أبداً)
#define GEIGER_ALIVE_MS   60000   // آخر نبضة خلال 60s = أخضر
#define GEIGER_SILENT_MS  120000  // صمت تام > 120s = أحمر (سلك النبضات مفصول)

// حدّ القراءة المنطقية: J305 لا يتجاوزها فيزيائياً. فوقها = الدخل العائم
// يلتقط ضجيجاً (سلك الإشارة مفصول) → نبلّغها عطلاً لا جرعةً، ولا نبني
// عليها قرار سلامة (مؤكد من SentinelLab: نفس اللوحة/الدخل العائم).
#define GEIGER_MAX_PLAUSIBLE_CPM 10000.0f

// المصدر الإشعاعي الافتراضي (المحاكاة / وضع Demo)
#define SIM_BG_CPM        22.0f   // خلفية طبيعية 20-25 CPM
#define SIM_SRC_LAT       24.713900
#define SIM_SRC_LNG       46.675600
#define SIM_SRC_CPM_1M    30000.0f // شدة المصدر على بعد 1م (قانون التربيع العكسي)
#define SIM_SHIELD_MIN    0.25f   // معامل الحجب عند إدارة الظهر للمصدر (درع الرصاص)
#define SIM_TUBE2_FACTOR  0.93f   // فرق حساسية الأنبوب الثاني (واقعية المعايرة)
#define SIM_HOME_LAT      24.713600 // الموقع الافتراضي عند غياب fix حقيقي
#define SIM_HOME_LNG      46.675300

// ═══ كشف الشذوذ (anomaly.h) ══════════════════════════════════
#define CALIB_DURATION_MS   60000  // معايرة الخلفية عند بدء المهمة
#define CALIB_MIN_SAMPLES   5      // أقل عدد عينات (دلاء) لاعتماد المعايرة
#define ANOMALY_SIGMA       3.0f   // قراءة > μ+3σ = شذوذ

// ═══ تصنيف الخطر µSv/h (risk.h) ══════════════════════════════
#define RISK_LOW_USVH       0.5f   // Safe < 0.5 | Low 0.5-2
#define RISK_MED_USVH       2.0f   // Medium 2-10
#define RISK_HIGH_USVH      10.0f  // High 10-100
#define RISK_CRIT_USVH      100.0f // Critical > 100
#define CRITICAL_RETREAT_MS 5000   // زمن التراجع ≈ 2م بسرعة DRIVE_SPEED_MPS
#define CRITICAL_RETREAT_POWER 60
#define CRITICAL_COOLDOWN_MS 10000 // مهلة تهدئة قبل تكرار التراجع إن بقي Critical

// ═══ BNO055 ═══════════════════════════════════════════════════
#define BNO_STABILITY_TH  0.05f  // rad/s — norm الجايرو دونه = الجهاز ثابت (من المرجع)
// كريستال خارجي: 1 فقط للوحات فيها كريستال (Adafruit الأصلية). الوحدات
// الرخيصة (GY-BNO055) بلا كريستال — تفعيله يُبقي الحساس في CONFIG (كل القراءات صفر).
#define IMU_EXT_CRYSTAL   0

// ═══ تشخيص السيريال ══════════════════════════════════════════
#define IMU_DEBUG_SERIAL  0   // 1 = اطبع حالة IMU (اتجاه/ميل/معايرة) كل ثانية
#define NET_DEBUG_SERIAL  1   // 1 = اطبع حالة الشبكة/الكام كل ثانيتين (تشخيص الاتصال)

// ═══ طبقة السلامة الثانية: انحشار وميل (failsafe.h) ══════════
//   تعتمد على IMU الحقيقي — تعمل بلا إنترنت وخارج سيطرة أي AI.
// كشف الميل الخطير: 0 = معطَّل كلياً (لا إيقاف ولا إنذار ميل). عطّله ما دام
// BNO055 غير مُعاير/تركيبه مائل (pitch/roll غير صفرية وهو مستوٍ) لأنه يعطي
// إنذاراً كاذباً ويوقف الروفر. أعِده إلى 1 بعد ضبط المستوي ومعايرة الحساس.
#define FAILSAFE_TILT_CHECK   0
#define TILT_LIMIT_DEG        25.0f  // |pitch| أو |roll| فوقه = ميل خطير
#define JAM_ACCEL_THRESH_MS2  0.30f  // تسارع خطي دونه أثناء أمر حركة = لا تقدم فعلي
#define JAM_DETECT_MS         3000   // استمرار الحالة أعلاه = انحشار → إيقاف
// سلوك الميل: 0 = إيقاف + إنذار فقط (آمن — الافتراضي)، 1 = تراجع خلفي تلقائي.
// ابقِه 0 حتى تُعاير الـBNO وتؤكد أن pitch/roll ≈ 0 وهو مستوٍ (خريطة المحاور
// الخاطئة أو عدم المعايرة تجعله يقود للخلف بلا توقف — انفلات).
#define FAILSAFE_TILT_DRIVE_BACK 0
#define TILT_RETREAT_MS       1500   // مدة التراجع (تُستخدم فقط عند تفعيل الأعلى)
#define TILT_RETREAT_POWER    60

// ═══ حساسات العوائق (obstacles.h — حامل تيليمتري من الأردوينو) ═
#define OBSTACLE_MAX_CM          300.0f  // أقصى مدى الفوق-صوتي (خالٍ فوقه)
#define OBSTACLE_WARN_CM         50.0f   // عائق أمامي أقرب = تحذير أصفر بالواجهة
#define OBSTACLE_NEAR_CM         20.0f   // أقرب = خطر (يفعّل الإيقاف الطارئ إن مُكِّن)
#define OBSTACLE_STALE_MS        1500    // بلا "OBS:" خلاله = بيانات قديمة (أحمر/لا بيانات)
// إيقاف طارئ عند عائق أمامي قريب — قرار في الهَب من التيليمتري المستلم. مطفأ
// افتراضياً: بيانات العوائق تصل عبر WiFi (كام) فلا يُعتمد عليها كسلامة صلبة، وكي
// لا يقاطع القيادة اليدوية/الرجوع. فعّله بعد معايرة مدى الحساسات على العتاد.
#define OBSTACLE_ESTOP           0

// ═══ محدد اتجاه المصدر (source_locator.h) ════════════════════
#define SCAN_BINS           12     // 12 سلة × 30°
#define SCAN_PULSE_MS       250    // نبضة دوران R قصيرة (≈18° عند SCAN_TURN_POWER)
#define SCAN_TURN_POWER     55
#define SCAN_SETTLE_MS      2000   // توقف لتجميع النبضات في كل سلة
#define SCAN_TIMEOUT_MS     120000
#define SCAN_MIN_COUNTS_FULLCONF 300  // مجموع نبضات المسح اللازم لثقة كاملة
#define GRADIENT_POINTS     16     // آخر N نقاط GPS لتدرّج الجرعة
#define GRADIENT_MIN_STEP_M 1.0f   // أقل مسافة بين نقطتي تدرّج

// ═══ التسجيل (sd_logger.h) ═══════════════════════════════════
#define LOG_INTERVAL_MS 1000
#define CSV_HEADER "ms,lat,lng,fix,cpm1,cpm2,cpm,usvh,risk,heading,anomaly"

// ═══ إيقاعات البث ════════════════════════════════════════════
#define GPS_BROADCAST_MS  1000
#define RAD_BROADCAST_MS  1000
#define TELE_BROADCAST_MS 2000

// ═══ أنواع مشتركة ════════════════════════════════════════════
// دالة إرسال أمر حركة للروفر — تحقنها الوحدات الذكية (risk, source_locator)
typedef void (*RoverCmdFn)(const char*);
