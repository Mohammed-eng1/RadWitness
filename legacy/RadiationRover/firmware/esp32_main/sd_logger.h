/*
 * sd_logger.h — واجهة تجريد التسجيل (HAL)
 * =========================================
 * حقيقي (USE_REAL_SD): كرت SD على SPI مخصّص — بأسلوب السكتش المرجعي
 *   حرفياً: SPIClass spiSD(FSPI) ثم SD.begin(CS, spiSD)، كتابة الرأس مرة
 *   واحدة عند إنشاء الملف، وفتح/إلحاق/إغلاق لكل صف (آمن ضد فصل الطاقة).
 *
 * تهبيط رشيق: إن غاب الكرت عند الإقلاع أو نُزع أثناء التشغيل → تحويل
 *   تلقائي إلى LittleFS بنفس الصيغ والمسارات — النظام لا يتجمد أبداً.
 *   loggerHealth(): 2 = SD يعمل (أخضر) | 1 = fallback على LittleFS (أصفر)
 *                   0 = لا تخزين إطلاقاً (أحمر)
 *
 * كل مهمة = ملف /LOG_NNN.csv + تقرير /MRPT_NNN.json بنفس الرقم.
 */
#pragma once
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include "config.h"

#if USE_REAL_SD
  #include <SPI.h>
  #include <SD.h>
  static SPIClass spiSD(FSPI);          // ناقل مخصّص — أسلوب المرجع المثبت
#endif

static fs::FS* logFs = nullptr;         // الوجهة الحالية (SD أو LittleFS)
static uint8_t loggerHealthVal = 0;     // 2=SD | 1=LittleFS fallback | 0=لا شيء
static char logCsvPath[24] = "";
static char logReportPath[24] = "";
static int  logMissionIdx = 0;
static bool logOpenFlag = false;        // مهمة جارية (الملف يُفتح لكل صف فقط)
static uint8_t logFailCount = 0;        // فشل كتابة متتالٍ على SD → fallback

static const char* LOG_CSV_HEADER = CSV_HEADER;

// الرأس مرة واحدة عند إنشاء الملف (أسلوب المرجع)
static void loggerWriteHeaderIfNew(const char* path) {
    if (logFs == nullptr || logFs->exists(path)) return;
    File f = logFs->open(path, FILE_WRITE);
    if (f) { f.println(LOG_CSV_HEADER); f.close(); }
}

// التحويل إلى LittleFS (عند فشل SD إقلاعاً أو أثناء التشغيل)
static void loggerFallbackToFlash() {
    if (!LittleFS.begin(true)) {        // true = هيّئ الفلاش أول مرة
        logFs = nullptr;
        loggerHealthVal = 0;
        Serial.println("[LOG] LittleFS fallback FAILED — no storage!");
        return;
    }
    logFs = &LittleFS;
    loggerHealthVal = 1;
    // لو في مهمة جارية: أنشئ نفس الملف على الفلاش بالرأس وأكمل التسجيل
    if (logOpenFlag && logCsvPath[0]) loggerWriteHeaderIfNew(logCsvPath);
    Serial.println("[LOG] SD lost -> falling back to LittleFS (yellow)");
}

bool loggerInit() {
#if USE_REAL_SD
    spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (SD.begin(SD_CS_PIN, spiSD)) {
        logFs = &SD;
        loggerHealthVal = 2;
        Serial.println("[LOG] SD OK (FSPI)");
        return true;
    }
    Serial.println("[LOG] SD init FAILED — falling back to LittleFS");
    loggerFallbackToFlash();
    return logFs != nullptr;
#else
    logFs = LittleFS.begin(true) ? &LittleFS : nullptr;
    loggerHealthVal = logFs ? 1 : 0;
    return logFs != nullptr;
#endif
}

bool loggerOk()     { return logFs != nullptr; }
int  loggerHealth() { return loggerHealthVal; }

// افتح ملف CSV جديداً بأول رقم تسلسلي غير مستخدم (الرأس فقط — لا يبقى مفتوحاً)
bool loggerStartMission() {
    if (logFs == nullptr || logOpenFlag) return false;
    for (int i = 1; i < 1000; i++) {
        snprintf(logCsvPath, sizeof(logCsvPath), "/LOG_%03d.csv", i);
        logMissionIdx = i;
        if (!logFs->exists(logCsvPath)) break;
    }
    snprintf(logReportPath, sizeof(logReportPath), "/MRPT_%03d.json", logMissionIdx);
    loggerWriteHeaderIfNew(logCsvPath);
    if (!logFs->exists(logCsvPath)) return false;   // حتى الإنشاء فشل
    logOpenFlag = true;
    logFailCount = 0;
    return true;
}

void loggerLog(unsigned long ms, double lat, double lng, bool fix,
               float cpm1, float cpm2, float cpm, float usvh,
               const char* risk, float heading, bool anomaly) {
    if (!logOpenFlag || logFs == nullptr) return;
    // فتح/إلحاق/إغلاق لكل صف — أسلوب المرجع، آمن ضد فصل الطاقة ونزع الكرت
    File f = logFs->open(logCsvPath, FILE_APPEND);
    if (!f) {
        // فشل الفتح: لو على SD فالكرت غالباً نُزع → 3 محاولات ثم fallback
        if (loggerHealthVal == 2 && ++logFailCount >= 3) loggerFallbackToFlash();
        return;
    }
    logFailCount = 0;
    f.printf("%lu,%.6f,%.6f,%d,%.1f,%.1f,%.1f,%.3f,%s,%.1f,%d\n",
             ms, lat, lng, fix ? 1 : 0, cpm1, cpm2, cpm, usvh, risk,
             heading, anomaly ? 1 : 0);
    f.close();
}

void loggerEndMission() { logOpenFlag = false; }

// حفظ نص كامل إلى ملف (تقرير مهمة، خطة مهمة...)
bool loggerSaveText(const char* path, const String& content) {
    if (logFs == nullptr || path[0] == 0) return false;
    File f = logFs->open(path, FILE_WRITE);
    if (!f) {
        if (loggerHealthVal == 2) { loggerFallbackToFlash(); f = logFs ? logFs->open(path, FILE_WRITE) : File(); }
        if (!f) return false;
    }
    f.print(content);
    f.close();
    return true;
}

const char* loggerCsvPath()    { return logCsvPath; }
const char* loggerReportPath() { return logReportPath; }
int loggerMissionIdx()         { return logMissionIdx; }
fs::FS& loggerFS()             { return *logFs; }
