/*
 * risk.h — تصنيف الخطر + طبقة السلامة المحلية
 * =============================================
 * التحويل: µSv/h = CPM ÷ CPM_PER_USVH (قابل للمعايرة في config.h).
 * المستويات: Safe <0.5 | Low 0.5-2 | Medium 2-10 | High 10-100 | Critical >100.
 *
 * طبقة السلامة المحلية (التراجع التلقائي عند Critical) خارج سيطرة
 * أي AI/شات بوت كلياً وتعمل بلا إنترنت — تحقن دالة الإرسال مرة
 * واحدة عند الإقلاع ولا تستشير أحداً قبل التصرف.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

enum RiskLevel { RISK_SAFE = 0, RISK_LOW, RISK_MEDIUM, RISK_HIGH, RISK_CRITICAL };

inline float cpmToUsvh(float cpm) { return cpm / CPM_PER_USVH; }

inline RiskLevel riskClassify(float usvh) {
    if (usvh >= RISK_CRIT_USVH) return RISK_CRITICAL;
    if (usvh >= RISK_HIGH_USVH) return RISK_HIGH;
    if (usvh >= RISK_MED_USVH)  return RISK_MEDIUM;
    if (usvh >= RISK_LOW_USVH)  return RISK_LOW;
    return RISK_SAFE;
}

inline const char* riskName(RiskLevel r) {
    switch (r) {
        case RISK_SAFE:     return "Safe";
        case RISK_LOW:      return "Low";
        case RISK_MEDIUM:   return "Medium";
        case RISK_HIGH:     return "High";
        case RISK_CRITICAL: return "Critical";
    }
    return "?";
}

// ── التراجع التلقائي عند Critical: توقف، ارجع ~2م، أنذر ──────
static RoverCmdFn riskSendCmd = nullptr;
static bool retreatActiveFlag = false;
static unsigned long retreatStartMs = 0;
static unsigned long retreatLastHbMs = 0;
static unsigned long retreatCooldownStartMs = 0;
static bool riskAlarmFlag = false;

void riskSafetyInit(RoverCmdFn fn) { riskSendCmd = fn; }

void riskSafetyUpdate(unsigned long now, RiskLevel level) {
    if (riskSendCmd == nullptr) return;

    if (retreatActiveFlag) {
        if (now - retreatStartMs >= CRITICAL_RETREAT_MS) {
            riskSendCmd("S");
            retreatActiveFlag = false;
            retreatCooldownStartMs = now;
        } else if (now - retreatLastHbMs >= CMD_HEARTBEAT_MS) {
            // heartbeat: أمر B متجدد كل 400ms (مهلة أمان الأردوينو 1.5s)
            retreatLastHbMs = now;
            char c[8];
            snprintf(c, sizeof(c), "B%d", CRITICAL_RETREAT_POWER);
            riskSendCmd(c);
        }
        return;
    }

    // مهلة تهدئة بعد كل تراجع: إن بقي المستوى Critical يتراجع مجدداً بعدها
    bool cooled = (retreatCooldownStartMs == 0) ||
                  (now - retreatCooldownStartMs >= CRITICAL_COOLDOWN_MS);
    if (level == RISK_CRITICAL && cooled) {
        retreatActiveFlag = true;
        riskAlarmFlag = true;
        retreatStartMs = now;
        retreatLastHbMs = 0;
    }
}

bool riskRetreatActive() { return retreatActiveFlag; }

// تدخل يدوي (أولوية القنوات: اليدوي فوق الذاتي) — يوقف التراجع، الإنذار يبقى
void riskCancelRetreat() {
    if (retreatActiveFlag) {
        retreatActiveFlag = false;
        retreatCooldownStartMs = millis();
    }
}

// إنذار Critical جديد منذ آخر استهلاك؟
bool riskConsumeAlarm() {
    bool f = riskAlarmFlag;
    riskAlarmFlag = false;
    return f;
}
