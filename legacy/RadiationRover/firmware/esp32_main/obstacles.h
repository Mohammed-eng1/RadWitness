/*
 * obstacles.h — حامل تيليمتري حساسات العوائق (الهَب)
 * ===================================================
 * الحساسات الثلاثة (فوق-صوتي أمامي + IR يسار + IR يمين) موصولة فيزيائياً
 * بشيلد الأردوينو (منافذ 10/8/7)، والأردوينو يقرأها ويرسلها على Serial
 * المشترك بصيغة "OBS:<سم>,<L>,<R>". الكاميرا تمرّرها عبر UDP 4212، والهَب
 * (هنا) يخزّن آخر قراءة ويعرضها — بلا قرار ذاتي وبلا محاكاة.
 *
 * لا حساس على الهَب نفسه: عند غياب الأردوينو ببساطة لا تصل بيانات، فتُبلَّغ
 * الحالة "قديمة/لا بيانات" (obstacleHealth=0) — النظام يترجم ويعمل دون تجميد.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

// آخر قراءة مستلمة من الأردوينو (عبر UDP 4212)
static float         obstacleDistRaw = -1.0f;   // سم؛ سالب = خالٍ/خارج المدى
static bool          obstacleIrLeft  = false;
static bool          obstacleIrRight = false;
static unsigned long obstacleLastRxMs = 0;
static bool          obstacleEverRx   = false;

void obstacleInit() {
    obstacleDistRaw = -1.0f;
    obstacleIrLeft = obstacleIrRight = false;
    obstacleEverRx = false;
}

// يُستدعى من محلّل UDP عند وصول سطر "OBS:d,l,r"
void obstacleSet(float distCm, bool irL, bool irR) {
    obstacleDistRaw  = distCm;
    obstacleIrLeft   = irL;
    obstacleIrRight  = irR;
    obstacleLastRxMs = millis();
    obstacleEverRx   = true;
}

// يحلّل الحمولة النصية "d,l,r" (ما بعد "OBS:") ويخزّنها. يعيد true عند النجاح.
bool obstacleParse(const char* payload) {
    float d; int l = 0, r = 0;
    if (sscanf(payload, "%f,%d,%d", &d, &l, &r) >= 1) {
        obstacleSet(d, l != 0, r != 0);
        return true;
    }
    return false;
}

// المسافة للعرض: الخالٍ/خارج المدى (سالب) يُعرض كأقصى مدى منطقي.
float obstacleDistanceCm() {
    if (obstacleDistRaw < 0) return OBSTACLE_MAX_CM;
    return obstacleDistRaw;
}
bool obstacleLeft()  { return obstacleIrLeft; }
bool obstacleRight() { return obstacleIrRight; }

// هل البيانات حديثة؟ (وصلت خلال OBSTACLE_STALE_MS)
bool obstacleFresh() {
    return obstacleEverRx && (millis() - obstacleLastRxMs) < OBSTACLE_STALE_MS;
}

// عائق أمامي حاجز: مسافة فعلية موجبة دون الخطر، أو أي IR مفعّل — وبيانات حديثة.
bool obstacleFrontBlocked() {
    if (!obstacleFresh()) return false;   // بلا بيانات موثوقة لا نبني قراراً
    bool ultraNear = obstacleDistRaw >= 0 && obstacleDistRaw < OBSTACLE_NEAR_CM;
    return ultraNear || obstacleIrLeft || obstacleIrRight;
}

// صحة الحساس: 2=أخضر (بيانات حديثة)، 0=أحمر (قديمة/لا شيء — الأردوينو/الكام مفصول).
int obstacleHealth() { return obstacleFresh() ? 2 : 0; }
bool obstacleOk()    { return obstacleFresh(); }
