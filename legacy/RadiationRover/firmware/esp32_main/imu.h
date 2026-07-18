/*
 * imu.h — واجهة تجريد الـIMU (HAL)
 * =================================
 * حقيقي: BNO055 على I2C (SDA=21/SCL=9) عبر مكتبة Adafruit، العنوان يُكتشف تلقائياً.
 * وهمي:  الـheading يتغير منطقياً مع أوامر الحركة المرسلة
 *        (لف يمين = زيادة الزاوية بمعدل TURN_RATE_DPS الواقعي)،
 *        مع محاكاة مهلة أمان الأردوينو (أمر بلا تجديد يتوقف بعد 1.5s).
 *
 * imuNotifyCommand() تُستدعى لكل أمر حركة صادر (يدوي أو ذاتي) —
 * تسجَّل دائماً حتى مع IMU حقيقي، لأن محاكاة GPS تحتاجها أيضاً.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

#if USE_REAL_IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Preferences.h>                // حفظ مرجع "المستوي"
static Adafruit_BNO055* bno = nullptr;   // يُنشأ بعد اكتشاف العنوان تلقائياً (من المرجع)
static uint8_t bnoAddr = 0;              // العنوان المكتشَف (للمراقب/الاستعادة)
#endif

static bool  imuOkFlag = false;
static float imuHeadingDeg = 0.0f;
static float imuPitchDeg = 0.0f;
static float imuRollDeg = 0.0f;
static float imuAccelMagMS2 = 0.0f;   // لكشف الانحشار (failsafe.h)
static bool  imuStableFlag = true;    // norm الجايرو < BNO_STABILITY_TH (من المرجع)
// معايرة مخزّنة (تُقرأ في imuUpdate — لا I2C في كل استدعاء)
static uint8_t imuCalS = 0, imuCalG = 0, imuCalA = 0, imuCalM = 0;

// ── ضبط المستوي: متجه الجاذبية الحالي + المرجع "الطبيعي" للروبوت ──
//   الميل يُقاس كزاوية بين المتجهين → يعمل مهما كان اتجاه تركيب الحساس.
static float imuGravX = 0, imuGravY = 0, imuGravZ = -9.8f;   // الحالي
static float imuGrefX = 0, imuGrefY = 0, imuGrefZ = -9.8f;   // المرجع (افتراضي: لأسفل)
static bool  imuLevelSet = false;
float imuTiltFromLevel();   // تعريف لاحقاً — إعلان مسبق لاستخدامه في imuUpdate

// حالة الأمر النشط (تُغذّي المحاكاة)
static char  imuCmdDir = 'S';
static int   imuCmdPower = 0;
static unsigned long imuCmdLastMs = 0;
static unsigned long imuSimLastStepMs = 0;

// سجّل أمر الحركة الصادر (F/B/L/R/S + قوة)
void imuNotifyCommand(char dir, int power) {
    imuCmdDir = dir;
    imuCmdPower = power;
    imuCmdLastMs = millis();
}

// الأمر الفعّال الآن — 'S' إذا انتهت مهلة أمان الأردوينو دون تجديد
char imuSimActiveDir() {
    if (imuCmdDir == 'S') return 'S';
    if (millis() - imuCmdLastMs >= UNO_SAFETY_TIMEOUT_MS) return 'S';
    return imuCmdDir;
}

int imuSimActivePower() { return imuCmdPower; }

#if USE_REAL_IMU
// يمسح ناقل I2C ويرجّع عنوان BNO (0x28 أو 0x29) أو 0 إذا غير موجود — من المرجع
static uint8_t i2cFindBNO() {
    Serial.print("[IMU] I2C scan:");
    uint8_t addr = 0;
    for (byte a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf(" 0x%02X", a);
            if (a == 0x28 || a == 0x29) addr = a;
        }
    }
    Serial.println(addr ? "" : "  (لا شيء! تحقق SDA=21 / SCL=9 / التغذية 3V3)");
    return addr;
}

// يضبط الحساس على وضع NDOF (الفيوجن) — إلزامي وإلا كل القراءات صفر (CONFIG).
// لا كريستال خارجي افتراضياً؛ الوحدات الرخيصة بلا كريستال وتفعيله يعطّل الفيوجن.
static void imuConfigureBNO() {
    if (!bno) return;
    delay(30);
#if IMU_EXT_CRYSTAL
    bno->setExtCrystalUse(true);
    delay(30);
#endif
    bno->setMode(OPERATION_MODE_NDOF);
    delay(30);
}
#endif

void imuInit() {
#if USE_REAL_IMU
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    uint8_t addr = i2cFindBNO();
    if (addr) {
        bnoAddr = addr;
        bno = new Adafruit_BNO055(55, addr, &Wire);
        imuOkFlag = bno->begin(OPERATION_MODE_NDOF);
        if (imuOkFlag) {
            imuConfigureBNO();
            Serial.printf("[IMU] BNO055 OK @ 0x%02X mode=0x%02X\n", addr, bno->getMode());
        } else Serial.println("[IMU] BNO ظاهر على الناقل لكن begin() فشل");
    } else imuOkFlag = false;
    // حمّل مرجع "المستوي" المحفوظ (إن ضبطه المستخدم سابقاً)
    {
        Preferences p; p.begin("imu", true);
        if (p.isKey("gx")) {
            imuGrefX = p.getFloat("gx", 0); imuGrefY = p.getFloat("gy", 0); imuGrefZ = p.getFloat("gz", -9.8f);
            imuLevelSet = true;
            Serial.println("[IMU] مرجع المستوي محمّل من الذاكرة");
        }
        p.end();
    }
#else
    imuOkFlag = true;
#endif
    imuSimLastStepMs = millis();
}

void imuUpdate(unsigned long now) {
#if USE_REAL_IMU
    // مراقب الحساس: افحص وجوده على I2C كل 2s — يكشف فصل السلك ويستعيد تلقائياً.
    // (يجعل imuOk يعكس الواقع بدل قيمة الإقلاع، ويعيد التهيئة عند عودة التوصيل.)
    static unsigned long lastProbe = 0;
    if (now - lastProbe >= 2000) {
        lastProbe = now;
        Wire.beginTransmission(bnoAddr ? bnoAddr : (uint8_t)0x28);
        bool present = (Wire.endTransmission() == 0);
        if (!present) {
            if (imuOkFlag) Serial.println("[IMU] فُقد BNO055 (تحقق GND/SDA=21/SCL=9)");
            imuOkFlag = false;
        } else if (!imuOkFlag) {
            uint8_t a = i2cFindBNO();
            if (a) {
                bnoAddr = a;
                if (bno) { delete bno; bno = nullptr; }
                bno = new Adafruit_BNO055(55, a, &Wire);
                imuOkFlag = bno->begin(OPERATION_MODE_NDOF);
                if (imuOkFlag) { imuConfigureBNO(); Serial.printf("[IMU] استُعيد @ 0x%02X\n", a); }
            }
        } else if (bno != nullptr) {
            // موصول ويعمل — أكّد NDOF، لكن بعد قراءتين خاطئتين متتاليتين فقط
            // (تفادي إعادة الضبط على وميض getMode العابر الذي يشوّش الفيوجن).
            static uint8_t badModeCount = 0;
            if (bno->getMode() != OPERATION_MODE_NDOF) {
                if (++badModeCount >= 2) {
                    bno->setMode(OPERATION_MODE_NDOF);
                    delay(20);
                    badModeCount = 0;
                    Serial.println("[IMU] أعدت ضبط NDOF");
                }
            } else badModeCount = 0;
        }
    }
    // تشخيص كل ثانية على السيريال — يطبع الاتجاه والميل والمعايرة ووضع التشغيل.
    // mode=0x0C يعني NDOF (الفيوجن يعمل، الاتجاه صالح). أي قيمة أخرى = المشكلة.
    // معطَّل افتراضياً (IMU_DEBUG_SERIAL=0) لإبقاء السيريال نظيفاً؛ فعّله عند الحاجة.
#if IMU_DEBUG_SERIAL
    static unsigned long lastImuDiag = 0;
    if (now - lastImuDiag >= 1000) {
        lastImuDiag = now;
        uint8_t mode = (imuOkFlag && bno != nullptr) ? bno->getMode() : 0xFF;
        Serial.printf("[IMU] ok=%d mode=0x%02X hdg=%.1f pitch=%.1f roll=%.1f tiltLv=%.0f cal(sgam)=%d%d%d%d\n",
                      imuOkFlag ? 1 : 0, mode, imuHeadingDeg, imuPitchDeg, imuRollDeg,
                      imuTiltFromLevel(), imuCalS, imuCalG, imuCalA, imuCalM);
    }
#endif
    if (!imuOkFlag || bno == nullptr) return;
    static unsigned long lastRead = 0;
    if (now - lastRead < 100) return;
    lastRead = now;
    sensors_event_t orient, accel, gyro;
    bno->getEvent(&orient, Adafruit_BNO055::VECTOR_EULER);
    imuHeadingDeg = orient.orientation.x;
    if (imuHeadingDeg < 0) imuHeadingDeg += 360.0f;
    // تخطيط المحاور حسب اتجاه تركيب اللوح — يُضبط عملياً على العتاد
    imuRollDeg  = orient.orientation.y;
    imuPitchDeg = orient.orientation.z;
    bno->getEvent(&accel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    imuAccelMagMS2 = sqrtf(accel.acceleration.x * accel.acceleration.x +
                           accel.acceleration.y * accel.acceleration.y +
                           accel.acceleration.z * accel.acceleration.z);
    // علم الثبات من norm الجايرو (من المرجع) — إشارة ثانية لكشف الانحشار
    bno->getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    float gx = gyro.gyro.x, gy = gyro.gyro.y, gz = gyro.gyro.z;
    imuStableFlag = sqrtf(gx * gx + gy * gy + gz * gz) < BNO_STABILITY_TH;
    // خزّن المعايرة (تُستخدم لبوابة السلامة والعرض — بلا I2C إضافي)
    bno->getCalibration(&imuCalS, &imuCalG, &imuCalA, &imuCalM);
    // متجه الجاذبية — أساس قياس الميل نسبةً لمرجع "المستوي"
    sensors_event_t grav;
    bno->getEvent(&grav, Adafruit_BNO055::VECTOR_GRAVITY);
    imuGravX = grav.acceleration.x; imuGravY = grav.acceleration.y; imuGravZ = grav.acceleration.z;
#else
    float dt = (now - imuSimLastStepMs) / 1000.0f;
    imuSimLastStepMs = now;
    if (dt <= 0 || dt > 2.0f) return;   // قفزة زمنية غير منطقية — تجاهل

    char d = imuSimActiveDir();
    if (d == 'L' || d == 'R') {
        float rate = TURN_RATE_DPS * (imuCmdPower / 70.0f);
        imuHeadingDeg += (d == 'R' ? rate : -rate) * dt;
        while (imuHeadingDeg >= 360.0f) imuHeadingDeg -= 360.0f;
        while (imuHeadingDeg < 0.0f)    imuHeadingDeg += 360.0f;
    }
    // "تسارع" رمزي أثناء الحركة — فوق عتبة الانحشار فلا إنذارات كاذبة في المحاكاة
    imuAccelMagMS2 = (d != 'S') ? 0.5f : 0.0f;
    imuStableFlag = (d == 'S');
#endif
}

float imuHeading()  { return imuHeadingDeg; }
float imuPitch()    { return imuPitchDeg; }
float imuRoll()     { return imuRollDeg; }
float imuAccelMag() { return imuAccelMagMS2; }
bool  imuOk()       { return imuOkFlag; }
bool  imuStable()   { return imuStableFlag; }   // ثبات من الجايرو (norm < 0.05)

// هل قراءة الاتجاه/الميل موثوقة؟ الميل يأتي من التسارع → نشترط معايرة التسارع.
// حتى لا تتصرف طبقة السلامة على IMU غير مُعاير (كان سبب انفلات الروفر).
bool imuTrusted() {
#if USE_REAL_IMU
    return imuOkFlag && imuCalA >= 1;
#else
    return true;
#endif
}

// زاوية ميل الروبوت عن وضعه "المستوي" المرجعي (بالدرجات) — بمتجه الجاذبية،
// فتعمل مهما كان اتجاه تركيب الحساس (مائل/على جنبه/مقلوب). المحاكاة: 0.
float imuTiltFromLevel() {
#if USE_REAL_IMU
    float m1 = sqrtf(imuGravX*imuGravX + imuGravY*imuGravY + imuGravZ*imuGravZ);
    float m2 = sqrtf(imuGrefX*imuGrefX + imuGrefY*imuGrefY + imuGrefZ*imuGrefZ);
    if (m1 < 0.5f || m2 < 0.5f) return 0.0f;
    float dp = (imuGravX*imuGrefX + imuGravY*imuGrefY + imuGravZ*imuGrefZ) / (m1 * m2);
    dp = constrain(dp, -1.0f, 1.0f);
    return acosf(dp) * RAD_TO_DEG;
#else
    return 0.0f;
#endif
}

bool imuLevelIsSet() { return imuLevelSet; }

// ضبط الوضع الحالي كـ"المستوي" (الروبوت في وضعه الطبيعي) — يُحفظ في NVS.
// يرجّع false لو الجاذبية غير صالحة (الحساس في CONFIG لحظتها) — لا يضبط مرجعاً فاسداً.
bool imuSetLevel() {
#if USE_REAL_IMU
    if (!imuOkFlag || bno == nullptr) return false;
    float m = sqrtf(imuGravX*imuGravX + imuGravY*imuGravY + imuGravZ*imuGravZ);
    if (m < 0.5f) return false;                 // قراءة جاذبية غير صالحة الآن
    imuGrefX = imuGravX; imuGrefY = imuGravY; imuGrefZ = imuGravZ;
    imuLevelSet = true;
    Preferences p; p.begin("imu", false);
    p.putFloat("gx", imuGrefX); p.putFloat("gy", imuGrefY); p.putFloat("gz", imuGrefZ);
    p.end();
    Serial.printf("[IMU] ضُبط المستوي: g=(%.2f,%.2f,%.2f)\n", imuGrefX, imuGrefY, imuGrefZ);
    return true;
#else
    return true;
#endif
}

// استعادة المرجع الافتراضي (لأسفل) — الميل يصبح مطلقاً، ويُمسح المحفوظ.
void imuResetLevel() {
#if USE_REAL_IMU
    imuGrefX = 0; imuGrefY = 0; imuGrefZ = -9.8f;
    imuLevelSet = false;
    Preferences p; p.begin("imu", false);
    p.remove("gx"); p.remove("gy"); p.remove("gz");
    p.end();
    Serial.println("[IMU] أُعيد المستوي للافتراضي");
#endif
}

// حالة معايرة BNO055 (0-3 لكل محور): sys/gyro/accel/mag — من الكاش.
// المحاكاة تُبلّغ معايرة كاملة (3) — لا حساس فعلي يُعاير.
void imuGetCalibration(uint8_t &sys, uint8_t &gyro, uint8_t &accel, uint8_t &mag) {
#if USE_REAL_IMU
    sys = imuCalS; gyro = imuCalG; accel = imuCalA; mag = imuCalM;
#else
    sys = gyro = accel = mag = 3;
#endif
}
