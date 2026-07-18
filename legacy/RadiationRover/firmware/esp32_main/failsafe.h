/*
 * failsafe.h — طبقة سلامة محلية ثانية: انحشار وميل خطير
 * ======================================================
 * تعتمد على IMU الحقيقي (BNO055). مثل risk.h تماماً:
 *   - non-blocking (millis فقط)، تُستدعى failsafeUpdate() من اللوب.
 *   - خارج سيطرة أي AI/شات بوت، وتعمل بلا إنترنت.
 *   - تُحقن دالة الإرسال مرة واحدة عند الإقلاع.
 *   - التدخل اليدوي يقاطعها (failsafeCancel) — أولوية اليدوي فوق الذاتي.
 *
 * كشفان:
 *   (أ) الانحشار: أمر حركة F/B نشط + تسارع خطي ≈ 0 لمدة JAM_DETECT_MS
 *       (العجلات تدور بلا تقدم — مثل رفع الروفر عن الأرض) ← إيقاف + إنذار.
 *   (ب) الميل الخطير: |pitch| أو |roll| > TILT_LIMIT_DEG ← تراجع فوري + إنذار.
 *
 * في وضع المحاكاة: pitch/roll = 0 دائماً (لا ميل) والتسارع الرمزي أثناء
 * الحركة (0.5) فوق العتبة (لا انحشار) — فلا تتفعّل أبداً (لا انحدار).
 */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "imu.h"

static RoverCmdFn failsafeSend = nullptr;

// حالة كشف الانحشار
static unsigned long jamBelowSinceMs = 0;   // متى بدأ سكون التسارع تحت أمر حركة
static bool jamActiveFlag = false;

// حالة تراجع الميل الخطير
static bool tiltRetreatActive = false;
static unsigned long tiltRetreatStartMs = 0, tiltRetreatHbMs = 0;

// إنذار جديد للاستهلاك من اللوب
static bool failsafeAlarmFlag = false;
static const char* failsafeReasonStr = "";

void failsafeInit(RoverCmdFn fn) {
    failsafeSend = fn;
    jamBelowSinceMs = 0;
    jamActiveFlag = false;
    tiltRetreatActive = false;
    failsafeAlarmFlag = false;
}

bool failsafeJam()  { return jamActiveFlag; }
bool failsafeTilt() { return tiltRetreatActive; }
bool failsafeActive() { return jamActiveFlag || tiltRetreatActive; }

// إنذار سلامة جديد منذ آخر استهلاك؟ (السبب عبر reason)
bool failsafeConsumeAlarm(const char** reason) {
    bool f = failsafeAlarmFlag;
    failsafeAlarmFlag = false;
    if (reason) *reason = failsafeReasonStr;
    return f;
}

// تدخل يدوي: يلغي تصرفات السلامة الجارية (الإنذار يُستهلك مستقلاً)
void failsafeCancel() {
    jamActiveFlag = false;
    jamBelowSinceMs = 0;
    tiltRetreatActive = false;
}

void failsafeUpdate(unsigned long now) {
    if (failsafeSend == nullptr) return;

    // بوابة الثقة: لا نتصرف على IMU غير مُعاير (قراءة ميل/تسارع غير موثوقة).
    // بدونها كان الروفر ينفلت: BNO غير مُعاير → "ميل" وهمي → تراجع بلا توقف.
    if (!imuTrusted()) {
        if (tiltRetreatActive) { failsafeSend("S"); tiltRetreatActive = false; }
        jamActiveFlag = false; jamBelowSinceMs = 0;
        return;
    }

    // ── (ب) الميل الخطير له الأولوية ──────────────────────────
    //   يُقاس عن وضع "المستوي" المرجعي (بمتجه الجاذبية) لا مطلقاً —
    //   فيعمل مهما كان اتجاه تركيب الـBNO (بعد ضبط المستوي مرة).
#if FAILSAFE_TILT_CHECK
    bool tilted = (imuTiltFromLevel() > TILT_LIMIT_DEG);

  #if FAILSAFE_TILT_DRIVE_BACK
    // تراجع خلفي تلقائي (يُفعَّل فقط بعد تأكيد معايرة/خريطة محاور صحيحة)
    if (tiltRetreatActive) {
        if (!tilted && now - tiltRetreatStartMs >= TILT_RETREAT_MS) {
            failsafeSend("S");
            tiltRetreatActive = false;
        } else if (now - tiltRetreatHbMs >= CMD_HEARTBEAT_MS) {
            tiltRetreatHbMs = now;
            char c[8];
            snprintf(c, sizeof(c), "B%d", TILT_RETREAT_POWER);
            failsafeSend(c);
        }
        return;
    }
    if (tilted) {
        tiltRetreatActive = true;
        tiltRetreatStartMs = now;
        tiltRetreatHbMs = 0;
        failsafeAlarmFlag = true;
        failsafeReasonStr = "tilt";
        return;
    }
  #else
    // السلوك الآمن الافتراضي: الميل يوقف الروفر وينذر فقط (لا قيادة تلقائية)
    if (tilted) {
        if (!tiltRetreatActive) {                 // حافة صاعدة = إنذار + إيقاف مرة
            failsafeSend("S");
            failsafeAlarmFlag = true;
            failsafeReasonStr = "tilt";
        }
        tiltRetreatActive = true;                 // يُستخدم كوسم "مائل الآن" للعرض
        return;
    }
    tiltRetreatActive = false;
  #endif  // FAILSAFE_TILT_DRIVE_BACK
#endif    // FAILSAFE_TILT_CHECK

    // ── (أ) كشف الانحشار ──────────────────────────────────────
    //   إشارتان معاً (B2 — يقلل الإنذارات الكاذبة): تسارع ≈ 0 *و* جايرو
    //   مستقر رغم أمر حركة نشط = العجلات تدور بلا أي تقدم أو دوران فعلي.
    char d = imuSimActiveDir();                   // الأمر الفعّال (يحترم مهلة الأردوينو)
    bool moving = (d == 'F' || d == 'B');

    if (moving && imuAccelMag() < JAM_ACCEL_THRESH_MS2 && imuStable()) {
        if (jamBelowSinceMs == 0) jamBelowSinceMs = now;
        else if (now - jamBelowSinceMs >= JAM_DETECT_MS && !jamActiveFlag) {
            jamActiveFlag = true;
            failsafeSend("S");                    // أوقف: تقدّم صفري رغم أمر الحركة
            failsafeAlarmFlag = true;
            failsafeReasonStr = "jam";
        }
    } else {
        jamBelowSinceMs = 0;
        if (moving) jamActiveFlag = false;        // استؤنفت حركة فعلية → زال الانحشار
    }
}
