/*
 * arduino_rover.ino — GalaxyRVR Motor Controller
 * ===============================================
 * يعمل على: Arduino Uno R3 داخل GalaxyRVR
 *
 * يستقبل أوامر من ESP32 (الجديد) عبر Serial (115200)
 * البروتوكول: نص بسيط منتهي بـ '\n'
 *
 *   "F\n"     → Forward بقوة افتراضية
 *   "B\n"     → Backward
 *   "L\n"     → Left
 *   "R\n"     → Right
 *   "S\n"     → Stop
 *   "Fxx\n"   → Forward  بقوة xx (1-100)  مثال: F80
 *   "Bxx\n"   → Backward بقوة xx
 *   "Lxx\n"   → Left بقوة xx
 *   "Rxx\n"   → Right بقوة xx
 *
 * ★ ميزة جديدة: Safety Timeout ★
 *   لو لم يصل أمر لأكثر من 1.5 ثانية، الروبوت يتوقف تلقائياً.
 *   هذا يحمي لو فقد الاتصال بـ ESP32 أو الواي فاي.
 *
 * Pins (SunFounder GalaxyRVR shield):
 *   IN1=2, IN2=3  → 3 محركات يسار
 *   IN3=4, IN4=5  → 3 محركات يمين
 *   الفوق-صوتي = 10 (TRIG+ECHO سلك واحد)، IR يسار = 8، IR يمين = 7
 *   (توصيل شيلد GalaxyRVR الرسمي — راجع docs/wiring.md)
 *
 * ★ تيليمتري الحساسات ★
 *   الأردوينو يقرأ العوائق ويرسلها على Serial المشترك بصيغة:
 *     "OBS:<مسافة سم>,<IR يسار 0/1>,<IR يمين 0/1>"   مثال: OBS:34.5,0,1
 *   (المسافة -1 = خالٍ/خارج المدى). لا يتخذ الأردوينو أي قرار — العقل في الهَب.
 *
 * مكتبة مطلوبة: SoftPWM (Brett Hagman)
 */

#include <SoftPWM.h>

// ── Motor pins ──────────────────────────────────────────────
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;

// ★ اعكس مجموعتي المحركات (يسار/يمين) برمجياً إذا كان التوصيل معكوساً ★
#define MOTORS_SWAP_LR 0

// أدوار المنافذ (أمام/خلف لكل جهة) تُشتق حسب MOTORS_SWAP_LR
#if MOTORS_SWAP_LR
const int L_FWD_PIN = IN4, L_BWD_PIN = IN3, R_FWD_PIN = IN1, R_BWD_PIN = IN2;
#else
const int L_FWD_PIN = IN1, L_BWD_PIN = IN2, R_FWD_PIN = IN4, R_BWD_PIN = IN3;
#endif

// ── Battery pin (GalaxyRVR: A3 عبر voltage divider) ─────────
const int BATTERY_PIN = A3;

// ── حساسات العوائق (شيلد GalaxyRVR) ─────────────────────────
const int ULTRASONIC_PIN = 10;   // TRIG وECHO على منفذ واحد (توفير أطراف)
const int IR_LEFT_PIN    = 8;    // خرج رقمي: LOW عند وجود عائق
const int IR_RIGHT_PIN   = 7;
const long ULTRASONIC_TIMEOUT_US = 18000;  // ≈300سم (2·300/34000·1e6)
const int  ULTRASONIC_MAX_CM     = 300;

// ── إعدادات ─────────────────────────────────────────────────
const int DEFAULT_POWER = 75;
const unsigned long SAFETY_TIMEOUT_MS = 1500;  // 1.5 ثانية
const unsigned long BATTERY_INTERVAL  = 2000;  // أرسل الجهد كل ثانيتين
const unsigned long OBSTACLE_INTERVAL = 200;   // أرسل قراءة العوائق كل 200ms (5Hz)

// ★ تشخيص: اطبع كل أمر يصل على RX والإجراء المتخذ (لتأكيد وصول الأوامر).
//   الكام تتجاهل هذه الأسطر (ليست BV:/OBS:)، فلا تؤثر على التيليمتري. صفّره لاحقاً.
#define CMD_DEBUG 1

// ── Command state ───────────────────────────────────────────
String cmdBuffer = "";
unsigned long lastCmdTime = 0;
unsigned long lastBatterySend = 0;
unsigned long lastObstacleSend = 0;
bool motorsActive = false;     // هل المحركات شغّالة الآن؟

// ============================================================
void setup() {
    Serial.begin(115200);

    pinMode(BATTERY_PIN, INPUT);
    pinMode(IR_LEFT_PIN,  INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);
    SoftPWMBegin();
    stopMotors();

    // LED يومض 3 مرات عند الجاهزية
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(180);
        digitalWrite(LED_BUILTIN, LOW);  delay(180);
    }
}

// ============================================================
void loop() {
    // اقرأ كل ما وصل على Serial حتى '\n'
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            processCommand(cmdBuffer);
            cmdBuffer = "";
        } else if (c != '\r') {
            cmdBuffer += c;
            if (cmdBuffer.length() > 10) cmdBuffer = "";   // حماية overflow
        }
    }

    // ★ Safety: لو ما وصل أمر منذ فترة وأنت متحرك، توقف
    if (motorsActive && (millis() - lastCmdTime > SAFETY_TIMEOUT_MS)) {
        stopMotors();
        motorsActive = false;
        // وميض LED سريع كإشارة safety stop
        for (int i=0; i<2; i++){
            digitalWrite(LED_BUILTIN, HIGH); delay(50);
            digitalWrite(LED_BUILTIN, LOW);  delay(50);
        }
    }

    // ── أرسل جهد البطارية كل ثانيتين ──────────────────────
    if (millis() - lastBatterySend > BATTERY_INTERVAL) {
        lastBatterySend = millis();
        sendBatteryVoltage();
    }

    // ── أرسل قراءة العوائق كل 200ms ───────────────────────
    if (millis() - lastObstacleSend > OBSTACLE_INTERVAL) {
        lastObstacleSend = millis();
        sendObstacles();
    }

    // ★ تشخيص: LED مضيء طوال تنفيذ أمر حركة (motorsActive).
    //   اضغط اتجاهاً: يضيء = الأمر وصل للأردوينو ويُنفَّذ → لو ما تحرّك = عتاد/تغذية
    //   المحركات. لا يضيء = الأمر لا يصل من الكام.
    digitalWrite(LED_BUILTIN, motorsActive ? HIGH : LOW);
}

// ── قراءة الفوق-صوتي (منفذ واحد TRIG+ECHO) وإرسال العوائق ────
//   الطريقة الرسمية من شيلد GalaxyRVR: نبضة 10µs ثم pulseIn بمهلة.
//   pulseIn حاجز (~≤18ms) لكن يُستدعى كل 200ms فقط، فلا يؤثر على مهلة
//   الأمان (1.5s) ولا على استقبال الأوامر (تُخزَّن في FIFO للـUART).
float readUltrasonicCm() {
    pinMode(ULTRASONIC_PIN, OUTPUT);
    digitalWrite(ULTRASONIC_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_PIN, LOW);
    pinMode(ULTRASONIC_PIN, INPUT);

    long duration = pulseIn(ULTRASONIC_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
    float distance = duration * 0.017;   // سم = t(µs) × 0.017
    if (distance == 0 || distance > ULTRASONIC_MAX_CM) return -1;  // خالٍ/خارج المدى
    return distance;
}

void sendObstacles() {
    float dist = readUltrasonicCm();
    // معظم وحدات IR: OUT=LOW عند وجود عائق
    int irL = (digitalRead(IR_LEFT_PIN)  == LOW) ? 1 : 0;
    int irR = (digitalRead(IR_RIGHT_PIN) == LOW) ? 1 : 0;

    Serial.print("OBS:");
    Serial.print(dist, 1);
    Serial.print(',');
    Serial.print(irL);
    Serial.print(',');
    Serial.println(irR);
}

// ── قراءة وإرسال جهد البطارية ───────────────────────────────
void sendBatteryVoltage() {
    // متوسط 5 قراءات لتقليل الضوضاء
    long sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += analogRead(BATTERY_PIN);
        delay(2);
    }
    float adcValue = sum / 5.0;
    // معادلة GalaxyVR الرسمية: x2 بسبب الـ voltage divider
    float voltage = adcValue / 1023.0 * 5.0 * 2.0;

    // أرسل بصيغة "BV:7.85" — ESP32-CAM يميّزها عن الأوامر
    Serial.print("BV:");
    Serial.println(voltage, 2);
}

// ── تنفيذ الأمر ──────────────────────────────────────────────
void processCommand(String cmd) {
#if CMD_DEBUG
    Serial.print("RX:["); Serial.print(cmd); Serial.println("]");   // كل سطر يصل على RX
#endif
    // ★ تصليب البارسر ★: الطول ≤ 4، أول حرف أمر معروف، والبقية أرقام فقط.
    //   أي شيء آخر يُتجاهل تماماً (بلا تحديث lastCmdTime) — رسائل إقلاع
    //   الكاميرا على الـSerial المشترك يجب ألا تحرك الروفر أبداً.
    if (cmd.length() == 0 || cmd.length() > 4) return;

    char dir = cmd.charAt(0);
    if (dir != 'F' && dir != 'B' && dir != 'L' && dir != 'R' && dir != 'S') return;
    for (unsigned int i = 1; i < cmd.length(); i++) {
        if (!isDigit(cmd.charAt(i))) return;
    }

    lastCmdTime = millis();

    // استخرج القوة إذا موجودة: "F80" → 80
    int power = DEFAULT_POWER;
    if (cmd.length() > 1) {
        int parsed = cmd.substring(1).toInt();
        if (parsed > 0 && parsed <= 100) power = parsed;
    }

    switch (dir) {
        case 'F': moveForward(power);  motorsActive = true;  break;
        case 'B': moveBackward(power); motorsActive = true;  break;
        case 'L': turnLeft(power);     motorsActive = true;  break;
        case 'R': turnRight(power);    motorsActive = true;  break;
        case 'S': stopMotors();        motorsActive = false; break;
        default: break;
    }
#if CMD_DEBUG
    // أمر مقبول ونُفّذ: الاتجاه والقوة والـPWM المكتوب على منافذ المحركات
    Serial.print("MOVE:"); Serial.print(dir);
    Serial.print(" pw="); Serial.print(power);
    Serial.print(" pwm="); Serial.println(toPWM(power));
#endif
}

// ── دوال الحركة ──────────────────────────────────────────────
// pwm موجب = أمام، سالب = خلف (لجهة واحدة)
void setSide(int fwdPin, int bwdPin, int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) { SoftPWMSet(fwdPin, pwm); SoftPWMSet(bwdPin, 0);    }
    else          { SoftPWMSet(fwdPin, 0);   SoftPWMSet(bwdPin, -pwm); }
}

int toPWM(int power) {
    return map(power, 0, 100, 0, 255);
}

void moveForward(int power) {
    int p = toPWM(power);
    setSide(L_FWD_PIN, L_BWD_PIN,  p); setSide(R_FWD_PIN, R_BWD_PIN,  p);
}

void moveBackward(int power) {
    int p = toPWM(power);
    setSide(L_FWD_PIN, L_BWD_PIN, -p); setSide(R_FWD_PIN, R_BWD_PIN, -p);
}

void turnLeft(int power) {
    int p = toPWM(power);
    setSide(L_FWD_PIN, L_BWD_PIN, -p); setSide(R_FWD_PIN, R_BWD_PIN,  p);
}

void turnRight(int power) {
    int p = toPWM(power);
    setSide(L_FWD_PIN, L_BWD_PIN,  p); setSide(R_FWD_PIN, R_BWD_PIN, -p);
}

void stopMotors() {
    SoftPWMSet(IN1, 0); SoftPWMSet(IN2, 0);
    SoftPWMSet(IN3, 0); SoftPWMSet(IN4, 0);
}
