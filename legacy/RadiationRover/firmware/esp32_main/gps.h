/*
 * gps.h — واجهة تجريد GPS (HAL) + دوال جغرافية مشتركة
 * =====================================================
 * حقيقي: NEO-6M/7M/8M على Serial1 (RX=16, TX=17) عبر TinyGPSPlus.
 * SIM_GPS_WALK: يتجاهل موقع الـGPS ويحاكي حركة واقعية من أوامر
 * القيادة (heading من imu.h، سرعة DRIVE_SPEED_MPS) — للاختبار
 * الداخلي وDemo المكتب بلا سماء مفتوحة.
 */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "imu.h"

#if USE_REAL_GPS
#include <TinyGPSPlus.h>
static HardwareSerial GPSSerial(1);
static TinyGPSPlus tinyGps;
#endif

// هل الموقع المعتمد وهمي؟ (المشي الوهمي يتجاوز حتى الـGPS الحقيقي)
#define GPS_POS_IS_SIM (SIM_GPS_WALK || !USE_REAL_GPS)

static double simGpsLat = SIM_HOME_LAT;
static double simGpsLng = SIM_HOME_LNG;
static float  simGpsSpeedKmph = 0.0f;
static unsigned long simGpsLastStepMs = 0;

// ── دوال جغرافية (مسافات قصيرة — تقريب مستوٍ كافٍ) ──────────
float geoDistanceM(double lat1, double lng1, double lat2, double lng2) {
    double dLat = (lat2 - lat1) * 111320.0;
    double dLng = (lng2 - lng1) * 111320.0 * cos(lat1 * DEG_TO_RAD);
    return (float)sqrt(dLat * dLat + dLng * dLng);
}

// الاتجاه من النقطة 1 إلى 2: صفر = شمال، مع عقارب الساعة
float geoBearingDeg(double lat1, double lng1, double lat2, double lng2) {
    double dLat = (lat2 - lat1) * 111320.0;
    double dLng = (lng2 - lng1) * 111320.0 * cos(lat1 * DEG_TO_RAD);
    float b = atan2(dLng, dLat) * RAD_TO_DEG;
    if (b < 0) b += 360.0f;
    return b;
}

// فرق زاويتين في المدى [-180, 180]
float angDiffDeg(float a, float b) {
    return fmodf(a - b + 540.0f, 360.0f) - 180.0f;
}

void gpsInit() {
#if USE_REAL_GPS
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif
    simGpsLastStepMs = millis();
}

void gpsUpdate(unsigned long now) {
#if USE_REAL_GPS
    while (GPSSerial.available()) tinyGps.encode(GPSSerial.read());
#endif
#if GPS_POS_IS_SIM
    float dt = (now - simGpsLastStepMs) / 1000.0f;
    simGpsLastStepMs = now;
    if (dt <= 0 || dt > 2.0f) return;

    char d = imuSimActiveDir();
    if (d == 'F' || d == 'B') {
        float v = DRIVE_SPEED_MPS * (imuSimActivePower() / 70.0f) * (d == 'B' ? -1.0f : 1.0f);
        double hd = imuHeading() * DEG_TO_RAD;
        simGpsLat += (v * dt * cos(hd)) / 111320.0;
        simGpsLng += (v * dt * sin(hd)) / (111320.0 * cos(simGpsLat * DEG_TO_RAD));
        simGpsSpeedKmph = fabsf(v) * 3.6f;
    } else {
        simGpsSpeedKmph = 0.0f;
    }
#endif
}

bool gpsFix() {
#if GPS_POS_IS_SIM
    return true;
#else
    return tinyGps.location.isValid() && tinyGps.location.age() < 5000;
#endif
}

double gpsLat() {
#if GPS_POS_IS_SIM
    return simGpsLat;
#else
    return tinyGps.location.lat();
#endif
}

double gpsLng() {
#if GPS_POS_IS_SIM
    return simGpsLng;
#else
    return tinyGps.location.lng();
#endif
}

int gpsSats() {
#if GPS_POS_IS_SIM
    return 12;
#else
    return tinyGps.satellites.isValid() ? (int)tinyGps.satellites.value() : 0;
#endif
}

float gpsSpeedKmph() {
#if GPS_POS_IS_SIM
    return simGpsSpeedKmph;
#else
    return tinyGps.speed.kmph();
#endif
}

float gpsAltM() {
#if GPS_POS_IS_SIM
    return 600.0f;
#else
    return tinyGps.altitude.isValid() ? tinyGps.altitude.meters() : 0.0f;
#endif
}

// موقع الروفر الحالي أو الافتراضي — لا يفشل أبداً (للمحاكاة والتقارير)
void gpsPosOrHome(double &lat, double &lng) {
    if (gpsFix()) { lat = gpsLat(); lng = gpsLng(); }
    else          { lat = SIM_HOME_LAT; lng = SIM_HOME_LNG; }
}

// حساس GPS نفسه سليم؟ (في المحاكاة دائماً نعم)
bool gpsOk() {
#if GPS_POS_IS_SIM
    return true;
#else
    return gpsFix();
#endif
}
