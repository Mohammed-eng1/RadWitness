/*
 * mission_report.h — تتبع المهمة وتقريرها النهائي
 * =================================================
 * يجمع أثناء المهمة: أعلى جرعة وموقعها، المسافة والمدة، مناطق
 * الخطر، عدد الشذوذات، نتيجة المسح الدوراني وتدرّج الجرعة.
 * عند الإنهاء: JSON كامل + توصيات سلامة نصية بقواعد صريحة
 * (بلا أي AI — قواعد if/switch فقط).
 */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "gps.h"
#include "risk.h"

#define REPORT_MAX_ZONES 8

static bool repActive = false;
static unsigned long repBeganMs = 0;
static double repLastLat = 0, repLastLng = 0;
static bool  repHaveLast = false;
static float repDistanceM = 0;
static float repMaxUsvh = 0;
static double repMaxLat = 0, repMaxLng = 0;
static int repAnomalies = 0;
static RiskLevel repMaxRisk = RISK_SAFE;
static float repSrcDir = -1, repSrcConf = 0;
static float repGradDir = -1, repGradSlope = 0;

static double repZoneLat[REPORT_MAX_ZONES], repZoneLng[REPORT_MAX_ZONES];
static float  repZoneUsvh[REPORT_MAX_ZONES];
static int    repZoneN = 0;

void reportMissionStart(unsigned long now) {
    repActive = true;
    repBeganMs = now;
    repHaveLast = false;
    repDistanceM = 0;
    repMaxUsvh = 0; repMaxLat = 0; repMaxLng = 0;
    repAnomalies = 0;
    repMaxRisk = RISK_SAFE;
    repZoneN = 0;
    repSrcDir = -1; repSrcConf = 0;
    repGradDir = -1; repGradSlope = 0;
}

void reportMissionEnd()   { repActive = false; }
bool reportMissionActive() { return repActive; }

void reportTrack(double lat, double lng, bool fix, float usvh, RiskLevel lvl, bool anomalyEvent) {
    if (!repActive) return;
    if (fix) {
        if (repHaveLast) {
            float d = geoDistanceM(lat, lng, repLastLat, repLastLng);
            if (d < 50.0f) repDistanceM += d;   // تجاهل قفزات GPS الشاذة
        }
        repLastLat = lat; repLastLng = lng; repHaveLast = true;
    }
    if (usvh > repMaxUsvh) { repMaxUsvh = usvh; repMaxLat = lat; repMaxLng = lng; }
    if (lvl > repMaxRisk) repMaxRisk = lvl;
    if (anomalyEvent) repAnomalies++;

    // مناطق الخطر: High فأعلى، بفاصل ≥ 5م عن المسجل سابقاً
    if (lvl >= RISK_HIGH && fix) {
        for (int i = 0; i < repZoneN; i++) {
            if (geoDistanceM(lat, lng, repZoneLat[i], repZoneLng[i]) < 5.0f) {
                if (usvh > repZoneUsvh[i]) repZoneUsvh[i] = usvh;
                return;
            }
        }
        if (repZoneN < REPORT_MAX_ZONES) {
            repZoneLat[repZoneN] = lat;
            repZoneLng[repZoneN] = lng;
            repZoneUsvh[repZoneN] = usvh;
            repZoneN++;
        }
    }
}

void reportSetScanResult(float dirDeg, float conf) { repSrcDir = dirDeg; repSrcConf = conf; }
void reportSetGradient(float dirDeg, float slope)  { repGradDir = dirDeg; repGradSlope = slope; }

String reportBuildJson(unsigned long now) {
    String j;
    j.reserve(1400);
    j += "{\"duration_s\":" + String((now - repBeganMs) / 1000);
    j += ",\"distance_m\":" + String(repDistanceM, 1);
    j += ",\"max_usvh\":" + String(repMaxUsvh, 3);
    j += ",\"max_risk\":\"" + String(riskName(repMaxRisk)) + "\"";
    j += ",\"max_pos\":{\"lat\":" + String(repMaxLat, 6) + ",\"lng\":" + String(repMaxLng, 6) + "}";
    j += ",\"anomalies\":" + String(repAnomalies);
    j += ",\"source\":{\"found\":" + String(repSrcConf > 0 ? "true" : "false");
    j += ",\"dir_deg\":" + String(repSrcDir, 1) + ",\"confidence\":" + String(repSrcConf, 2) + "}";
    j += ",\"gradient\":{\"found\":" + String(repGradSlope > 0 ? "true" : "false");
    j += ",\"dir_deg\":" + String(repGradDir, 1) + ",\"slope_usvh_per_m\":" + String(repGradSlope, 4) + "}";
    j += ",\"danger_zones\":[";
    for (int i = 0; i < repZoneN; i++) {
        if (i) j += ",";
        j += "{\"lat\":" + String(repZoneLat[i], 6) + ",\"lng\":" + String(repZoneLng[i], 6);
        j += ",\"usvh\":" + String(repZoneUsvh[i], 2) + "}";
    }
    j += "],\"recommendations\":[";

    // توصيات سلامة بقواعد صريحة
    bool first = true;
    auto add = [&](const String& s) {
        if (!first) j += ",";
        j += "\"" + s + "\"";
        first = false;
    };
    switch (repMaxRisk) {
        case RISK_SAFE:
            add("المستويات ضمن الخلفية الطبيعية — لا إجراءات مطلوبة.");
            break;
        case RISK_LOW:
            add("ارتفاع طفيف فوق الخلفية — يُنصح بمراقبة دورية للمنطقة.");
            break;
        case RISK_MEDIUM:
            add("مستوى متوسط — قلّل زمن البقاء قرب نقطة القراءة الأعلى ووثّق محيطها.");
            break;
        case RISK_HIGH:
            add("منطقة خطرة — يمنع الاقتراب دون وقاية، ويلزم تطويق نصف قطر 10م حول النقاط المسجلة.");
            break;
        case RISK_CRITICAL:
            add("خطر جسيم — إخلاء فوري وتطويق واسع وإبلاغ الجهات المختصة قبل أي اقتراب.");
            break;
    }
    if (repSrcConf >= 0.5f) {
        add("الاتجاه المرجح للمصدر " + String(repSrcDir, 0) + "° بثقة " +
            String(repSrcConf * 100.0f, 0) + "% — تقدّم نحوه بحذر وبقراءات متدرجة.");
    } else if (repSrcConf > 0) {
        add("نتيجة المسح الدوراني غير حاسمة — كرر المسح من موقع أقرب للقراءة الأعلى.");
    }
    if (repAnomalies > 0) {
        add("سُجّل " + String(repAnomalies) + " حدث شذوذ فوق μ+3σ — راجع ملف CSV لمواقعها.");
    }
    if (repZoneN > 0) {
        add("حُدّدت " + String(repZoneN) + " منطقة خطر — تجنب المرور بها في المهمات القادمة.");
    }
    j += "]}";
    return j;
}
