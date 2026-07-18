/*
 * geiger.h — واجهة تجريد عداد جيجر مزدوج (HAL)
 * ==============================================
 * حقيقي: نبضات على GEIGER1_PIN (RISING عائم، CAJOE/J305) و
 *        GEIGER2_PIN (FALLING). الـISR يزيد عدّاداً volatile فقط —
 *        IRAM_ATTR وبلا أي حسابات.
 * وهمي:  خلفية SIM_BG_CPM بتوزيع بواسون حقيقي + مصدر افتراضي عند
 *        SIM_SRC_LAT/LNG بقانون التربيع العكسي، وتتأثر الشدة بزاوية
 *        الـheading (محاكاة حجب درع الرصاص) ليُختبر المسح الدوراني.
 *
 * ثلاث قراءات CPM لأغراض مختلفة:
 *   geigerCPM1/2()     نافذة منزلقة 60 ثانية — عرض هادئ لكل أنبوب
 *   geigerFastCPM()    EMA سريع (~5 ثوانٍ) — تصنيف الخطر والعرض الحي
 *   geigerInstantCPM() آخر دلو 10 ثوانٍ — عينات مستقلة لكشف الشذوذ
 */
#pragma once
#include <Arduino.h>
#include <esp_system.h>
#include "config.h"
#include "imu.h"
#include "gps.h"

// عدّادات النبض التراكمية (ISR حقيقي أو مولّد بواسون وهمي)
static volatile uint32_t geigerCount1 = 0;
static volatile uint32_t geigerCount2 = 0;

#if USE_REAL_GEIGER1
static void IRAM_ATTR geigerISR1() { geigerCount1++; }
#endif
#if USE_REAL_GEIGER2
static void IRAM_ATTR geigerISR2() { geigerCount2++; }
#endif

// عدد الأنابيب الفعّالة — يقسم عليه لحساب متوسط CPM (وضع الأنبوب الواحد = 1)
#if SINGLE_TUBE_MODE
  #define GEIGER_ACTIVE_TUBES 1
#else
  #define GEIGER_ACTIVE_TUBES 2
#endif

// تصحيح الزمن الميت: CPM_true = CPM_meas / (1 - CPM_meas·τ/60)
// τ مقاس من جلسة معايرة Cs-137 — يُطبَّق على الأنبوب الحقيقي فقط.
static inline float correctDeadTime(float cpmMeas) {
    float d = 1.0f - cpmMeas * GEIGER_DEADTIME_TAU_S / 60.0f;
    if (d < 0.1f) d = 0.1f;               // حماية من القسمة عند التشبع الشديد
    return cpmMeas / d;
}

// يطبّق التصحيح فقط حين يكون مصدر العدّ أنبوباً حقيقياً
static inline float geigerApplyDeadTime(float cpmMeas) {
#if (USE_REAL_GEIGER1 || USE_REAL_GEIGER2) && USE_DEADTIME_CORRECTION
    return correctDeadTime(cpmMeas);
#else
    return cpmMeas;                        // المحاكاة: القيم "حقيقية" أصلاً
#endif
}

// صحة الأنبوب الحقيقي: آخر لحظة تغيّر فيها العدّاد (كشف انفصال السلك)
static unsigned long geigerLastPulseMs = 0;
static uint32_t geigerHealthPrevTotal = 0;

// النافذة المنزلقة: GEIGER_BUCKETS دلو × GEIGER_BUCKET_MS
static uint32_t g1Buckets[GEIGER_BUCKETS];
static uint32_t g2Buckets[GEIGER_BUCKETS];
static uint32_t g1LastTotal = 0, g2LastTotal = 0;
static uint8_t  geigerBucketIdx = 0, geigerBucketsFilled = 0;
static unsigned long geigerLastRoll = 0;
static float geigerCpm1Val = 0, geigerCpm2Val = 0, geigerInstantVal = 0;
static bool  geigerNewSampleFlag = false;

// EMA سريع (يتحدث كل ثانية)
static unsigned long geigerLastFastTick = 0;
static uint32_t geigerFastLastTotal = 0;
static float geigerFastVal = 0;

static unsigned long geigerSimLastTick = 0;

// ── مولّد العشوائيات للمحاكاة ────────────────────────────────
static float simUniform() {   // (0,1)
    return ((float)esp_random() + 0.5f) / 4294967296.0f;
}

static float simGauss() {     // Box-Muller
    float u1 = simUniform(), u2 = simUniform();
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

static uint32_t simPoisson(float lambda) {
    if (lambda <= 0.0f) return 0;
    if (lambda > 30.0f) {
        // تقريب طبيعي — حلقة Knuth تطول (وexp تتجاوز الدقة) مع λ الكبيرة
        float v = lambda + sqrtf(lambda) * simGauss();
        return (v < 0.0f) ? 0 : (uint32_t)(v + 0.5f);
    }
    float L = expf(-lambda), p = 1.0f;
    uint32_t k = 0;
    do { k++; p *= simUniform(); } while (p > L);
    return k - 1;
}

// أدوات المحاكاة (تُستدعى من لوحة التشخيص — فعّالة في SIMULATION_MODE فقط)
static bool geigerSimSrcEnabled = true;         // المصدر الوهمي مفعّل افتراضياً
void geigerSimSetSource(bool on) { geigerSimSrcEnabled = on; }
bool geigerSimSourceOn()         { return geigerSimSrcEnabled; }
void geigerSimInjectSpike()      { geigerCount1 += 150; }   // قفزة شذوذ مفتعلة (دلو واحد)

// معدل CPM المتوقع من نموذج المصدر الافتراضي عند وضعية الروفر الحالية
static float geigerSimExpectedCPM() {
    if (!geigerSimSrcEnabled) return SIM_BG_CPM;   // مصدر مُطفأ = خلفية فقط
    double lat, lng;
    gpsPosOrHome(lat, lng);
    float d = geoDistanceM(lat, lng, SIM_SRC_LAT, SIM_SRC_LNG);
    if (d < 0.5f) d = 0.5f;
    float srcCPM = SIM_SRC_CPM_1M / (d * d);          // التربيع العكسي
    // حجب درع الرصاص: حساسية قصوى عند مواجهة المصدر، دنيا عند إدارة الظهر
    float diff = angDiffDeg(imuHeading(), geoBearingDeg(lat, lng, SIM_SRC_LAT, SIM_SRC_LNG));
    float facing = 0.5f * (1.0f + cosf(diff * DEG_TO_RAD));   // 1 مواجه ← 0 ظهر
    float shield = SIM_SHIELD_MIN + (1.0f - SIM_SHIELD_MIN) * facing;
    return SIM_BG_CPM + srcCPM * shield;
}

void geigerInit() {
#if USE_REAL_GEIGER1
    // لوحة CAJOE + J305 على GPIO15: خرج VIN نبضة موجبة ضعيفة من مخرج عالي
    // الممانعة — ثبت عملياً (SentinelLab) أن أي مقاومة رفع/سحب تقتل الإشارة
    // (CPM=0). الحل المؤكد: INPUT عائم تماماً + مقاطعة RISING.
    pinMode(GEIGER1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(GEIGER1_PIN), geigerISR1, RISING);
#endif
#if USE_REAL_GEIGER2
    pinMode(GEIGER2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(GEIGER2_PIN), geigerISR2, FALLING);
#endif
    for (int i = 0; i < GEIGER_BUCKETS; i++) { g1Buckets[i] = 0; g2Buckets[i] = 0; }
    unsigned long now = millis();
    geigerLastRoll = now;
    geigerLastFastTick = now;
    geigerSimLastTick = now;
    geigerLastPulseMs = now;          // لا نعتبره "صامتاً" قبل مرور نافذة الصمت
    geigerHealthPrevTotal = 0;
}

void geigerUpdate(unsigned long now) {
    // 1) توليد النبضات الوهمية (لكل أنبوب غير حقيقي وغير معطَّل)
    //    في SINGLE_TUBE_MODE الأنبوب الثاني معطَّل كلياً — لا محاكاة موازية.
#if !USE_REAL_GEIGER1 || (!USE_REAL_GEIGER2 && !SINGLE_TUBE_MODE)
    if (now - geigerSimLastTick >= GEIGER_SIM_TICK_MS) {
        geigerSimLastTick = now;
        float lambda = geigerSimExpectedCPM() * (GEIGER_SIM_TICK_MS / 60000.0f);
    #if !USE_REAL_GEIGER1
        geigerCount1 += simPoisson(lambda);
    #endif
    #if !USE_REAL_GEIGER2 && !SINGLE_TUBE_MODE
        geigerCount2 += simPoisson(lambda * SIM_TUBE2_FACTOR);
    #endif
    }
#endif

    // صحة الأنبوب الحقيقي: سجّل لحظة أي زيادة في العدّاد (نبضة استُقبلت)
    {
        uint32_t healthTot = geigerCount1 + geigerCount2;
        if (healthTot != geigerHealthPrevTotal) {
            geigerHealthPrevTotal = healthTot;
            geigerLastPulseMs = now;
        }
    }

    // 2) الـEMA السريع كل ثانية (متوسط الأنبوبين → CPM)
    if (now - geigerLastFastTick >= 1000) {
        geigerLastFastTick = now;
        uint32_t tot = geigerCount1 + geigerCount2;
        float countsLastSec = (float)(tot - geigerFastLastTotal);
        geigerFastLastTotal = tot;
        float inst = countsLastSec * 60.0f / GEIGER_ACTIVE_TUBES;
        geigerFastVal += 0.2f * (inst - geigerFastVal);
    }

    // 3) دلاء النافذة المنزلقة (كل 10 ثوانٍ)
    if (now - geigerLastRoll >= GEIGER_BUCKET_MS) {
        geigerLastRoll = now;
        uint32_t t1 = geigerCount1, t2 = geigerCount2;
        g1Buckets[geigerBucketIdx] = t1 - g1LastTotal; g1LastTotal = t1;
        g2Buckets[geigerBucketIdx] = t2 - g2LastTotal; g2LastTotal = t2;
        geigerInstantVal = (float)(g1Buckets[geigerBucketIdx] + g2Buckets[geigerBucketIdx])
                           * (60000.0f / GEIGER_BUCKET_MS) / GEIGER_ACTIVE_TUBES;
        geigerBucketIdx = (geigerBucketIdx + 1) % GEIGER_BUCKETS;
        if (geigerBucketsFilled < GEIGER_BUCKETS) geigerBucketsFilled++;

        uint32_t s1 = 0, s2 = 0;
        for (int i = 0; i < geigerBucketsFilled; i++) { s1 += g1Buckets[i]; s2 += g2Buckets[i]; }
        float windowMin = geigerBucketsFilled * (GEIGER_BUCKET_MS / 60000.0f);
        geigerCpm1Val = s1 / windowMin;
        geigerCpm2Val = s2 / windowMin;
        geigerNewSampleFlag = true;
    }
}

float geigerCPM1()       { return geigerCpm1Val; }
float geigerCPM2()       { return geigerCpm2Val; }
// CPM المجمّع: مصحَّح للزمن الميت (يغذّي الجرعة)، مع نسخة خام للشفافية العلمية
float geigerFastCPM()    { return geigerApplyDeadTime(geigerFastVal); }
float geigerFastCPMRaw() { return geigerFastVal; }
float geigerInstantCPM() { return geigerApplyDeadTime(geigerInstantVal); }
uint32_t geigerTotalCounts() { return geigerCount1 + geigerCount2; }

// اكتمل دلو جديد منذ آخر استدعاء؟ (عينة مستقلة لكشف الشذوذ)
bool geigerNewSample() {
    bool f = geigerNewSampleFlag;
    geigerNewSampleFlag = false;
    return f;
}

// صحة الأنبوب الحقيقي (B1): 2=أخضر (نبضة خلال GEIGER_ALIVE_MS)،
// 1=أصفر (صمت مؤقت)، 0=أحمر (صمت > GEIGER_SILENT_MS = سلك مفصول).
// المحاكاة دائماً خضراء (لا سلك يُفصل).
int geigerHealth() {
#if USE_REAL_GEIGER1
    unsigned long dt = millis() - geigerLastPulseMs;
    if (dt < GEIGER_ALIVE_MS)  return 2;
    if (dt < GEIGER_SILENT_MS) return 1;
    return 0;
#else
    return 2;
#endif
}

bool geigerOk() { return geigerHealth() > 0; }

// قراءة غير منطقية فيزيائياً = سلك الإشارة مفصول والدخل العائم يلتقط ضجيجاً.
// نبلّغها عطلاً (لا جرعة حقيقية) ولا نبني عليها قرار سلامة. المحاكاة لا تعلق.
bool geigerImplausible() {
#if USE_REAL_GEIGER1
    return geigerFastVal > GEIGER_MAX_PLAUSIBLE_CPM;
#else
    return false;
#endif
}
