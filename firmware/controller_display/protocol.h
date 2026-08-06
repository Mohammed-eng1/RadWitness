/*
 * protocol.h — بروتوكول الراديو (مرآة pi/comms/protocol.py)
 * ==================================================================
 * 🔴 **أي تغيير هنا يجب أن يقابله تغيير هناك.** الطرفان يتكلّمان نفس
 *    اللغة حرفياً، والانحراف بينهما يعطي «أطراً تالفة» بلا سبب ظاهر.
 *
 *   الإطار:   $<payload>*<HH>\n      (HH = XOR لبايتات payload، طراز NMEA)
 *   أمر:      C,<seq>,<cmd>,<p1>,<p2>
 *   تيليمتري: T,<seq>,<cpm>,<mv>,<state>     (mv ميلي فولت، -1 = مجهول)
 *   إقرار:    A,<seq>,<code>
 *
 * 🔴 قائمة الأوامر **مغلقة**: FWD BACK LEFT RIGHT STOP ESTOP STATUS RTH
 */
#pragma once
#include <Arduino.h>
#include "config.h"

// ── XOR checksum على بايتات الحمولة ──────────────────────────
inline uint8_t xorChecksum(const char *payload) {
  uint8_t c = 0;
  for (const char *p = payload; *p; ++p) c ^= (uint8_t)(*p);
  return c;
}

// ── بناء إطار كامل جاهز للإرسال ──────────────────────────────
// يُعيد false إن تجاوزت الحمولة الحدّ (لا يقصّ بصمت).
inline bool buildFrame(const char *payload, char *out, size_t outSize) {
  size_t n = strlen(payload);
  if (n > MAX_PAYLOAD || outSize < n + 6) return false;
  snprintf(out, outSize, "$%s*%02X\n", payload, xorChecksum(payload));
  return true;
}

// ── فكّ إطار والتحقق من الـchecksum ──────────────────────────
// يكتب الحمولة في payloadOut ويُعيد true عند الصحة فقط.
inline bool parseFrame(const char *line, char *payloadOut, size_t outSize) {
  if (!line || line[0] != '$') return false;
  const char *star = strrchr(line, '*');
  if (!star || star[1] == 0 || star[2] == 0) return false;
  size_t n = (size_t)(star - line - 1);
  if (n == 0 || n > MAX_PAYLOAD || n >= outSize) return false;
  memcpy(payloadOut, line + 1, n);
  payloadOut[n] = 0;
  char hex[3] = { star[1], star[2], 0 };
  uint8_t got = (uint8_t)strtol(hex, nullptr, 16);
  if (got != xorChecksum(payloadOut)) { payloadOut[0] = 0; return false; }
  return true;
}

// ── 🔴 القائمة المغلقة ───────────────────────────────────────
inline bool isAllowedCommand(const char *cmd) {
  static const char *ALLOWED[] = { "FWD", "BACK", "LEFT", "RIGHT",
                                   "STOP", "ESTOP", "STATUS", "RTH" };
  for (uint8_t i = 0; i < 8; ++i)
    if (strcmp(cmd, ALLOWED[i]) == 0) return true;
  return false;
}

// ── بناء إطار أمر (يرفض ما هو خارج القائمة) ──────────────────
inline bool buildCommand(uint16_t seq, const char *cmd, float p1,
                         char *out, size_t outSize) {
  if (!isAllowedCommand(cmd)) return false;
  char payload[MAX_PAYLOAD + 1];
  // 🔴 يطابق `_num()` في بايثون **حرفياً**: صحيح بلا فاصلة، وإلا خانتان
  //    مع **تجريد الأصفار اللاحقة** (0.30 → 0.3). بلا التجريد يختلف
  //    البايت فيختلف الـchecksum، وتفترق النسختان بصمت — وهو ما كشفه
  //    `test_protocol.cpp` فعلاً عند أول تشغيل.
  char pbuf[12];
  if (p1 == (int)p1) {
    snprintf(pbuf, sizeof(pbuf), "%d", (int)p1);
  } else {
    snprintf(pbuf, sizeof(pbuf), "%.2f", p1);
    size_t L = strlen(pbuf);
    while (L > 0 && pbuf[L - 1] == '0') pbuf[--L] = 0;      // أصفار لاحقة
    if (L > 0 && pbuf[L - 1] == '.') pbuf[--L] = 0;         // فاصلة معلّقة
  }
  snprintf(payload, sizeof(payload), "C,%u,%s,%s,0",
           (unsigned)(seq % SEQ_MODULO), cmd, pbuf);
  return buildFrame(payload, out, outSize);
}

// ── تيليمتري مفكوك ───────────────────────────────────────────
struct Telemetry {
  uint16_t seq;
  long     cpm;
  long     mv;        // -1 = مجهول (**لا يُعرض صفراً**)
  char     state[10];
  bool     valid;
};

inline Telemetry parseTelemetry(const char *payload) {
  Telemetry t; t.valid = false; t.seq = 0; t.cpm = 0; t.mv = -1; t.state[0] = 0;
  if (!payload || payload[0] != 'T' || payload[1] != ',') return t;
  char buf[MAX_PAYLOAD + 1];
  strncpy(buf, payload, sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
  char *save = nullptr;
  char *tok = strtok_r(buf, ",", &save);      // "T"
  if (!tok) return t;
  tok = strtok_r(nullptr, ",", &save); if (!tok) return t; t.seq = atoi(tok);
  tok = strtok_r(nullptr, ",", &save); if (!tok) return t; t.cpm = atol(tok);
  tok = strtok_r(nullptr, ",", &save); if (!tok) return t; t.mv  = atol(tok);
  tok = strtok_r(nullptr, ",", &save); if (!tok) return t;
  strncpy(t.state, tok, sizeof(t.state) - 1); t.state[sizeof(t.state) - 1] = 0;
  t.valid = true;
  return t;
}

// ── إقرار مفكوك (يجعل الرفض مرئياً على الشاشة لا صامتاً) ─────
inline bool parseAck(const char *payload, char *codeOut, size_t outSize) {
  if (!payload || payload[0] != 'A' || payload[1] != ',') return false;
  const char *c = strchr(payload + 2, ',');
  if (!c) return false;
  strncpy(codeOut, c + 1, outSize - 1);
  codeOut[outSize - 1] = 0;
  return true;
}
