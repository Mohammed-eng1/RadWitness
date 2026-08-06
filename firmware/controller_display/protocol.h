/*
 * protocol.h -- radio protocol (mirror of pi/comms/protocol.py)
 * ==================================================================
 * ANY change here MUST be mirrored there. Both ends speak the exact
 * same bytes; any drift between them produces "corrupt frames" with no
 * visible cause, and the symptom looks electrical, not software.
 * pi/comms/selftest.py compiles test_protocol.cpp and compares this
 * implementation against the Python one automatically.
 *
 *   Frame:     $<payload>*<HH>\n   (HH = XOR of payload bytes, NMEA style)
 *   Command:   C,<seq>,<cmd>,<p1>,<p2>
 *   Telemetry: T,<seq>,<cpm>,<mv>,<state>   (mv millivolts, -1 = unknown)
 *   Ack:       A,<seq>,<code>
 *
 * CLOSED command list: FWD BACK LEFT RIGHT STOP ESTOP STATUS RTH WDRAW
 *
 * ASCII only in this file (comments AND literals) -- Arabic BiDi text
 * broke the Arduino build before. See the note in controller_display.ino.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

// -- XOR checksum over the payload bytes -----------------------
inline uint8_t xorChecksum(const char *payload) {
  uint8_t c = 0;
  for (const char *p = payload; *p; ++p) c ^= (uint8_t)(*p);
  return c;
}

// -- Build a complete frame ready to transmit ------------------
// Returns false if the payload exceeds the cap (no silent truncation).
inline bool buildFrame(const char *payload, char *out, size_t outSize) {
  size_t n = strlen(payload);
  if (n > MAX_PAYLOAD || outSize < n + 6) return false;
  snprintf(out, outSize, "$%s*%02X\n", payload, xorChecksum(payload));
  return true;
}

// -- Parse a frame and verify its checksum ---------------------
// Writes the payload into payloadOut; returns true only when valid.
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

// -- The CLOSED command list -----------------------------------
inline bool isAllowedCommand(const char *cmd) {
  static const char *ALLOWED[] = { "FWD", "BACK", "LEFT", "RIGHT",
                                   "STOP", "ESTOP", "STATUS", "RTH",
                                   "WDRAW" };
  for (uint8_t i = 0; i < 9; ++i)
    if (strcmp(cmd, ALLOWED[i]) == 0) return true;
  return false;
}

// -- Build a command frame (rejects anything outside the list) --
inline bool buildCommand(uint16_t seq, const char *cmd, float p1,
                         char *out, size_t outSize) {
  if (!isAllowedCommand(cmd)) return false;
  char payload[MAX_PAYLOAD + 1];
  // Must match Python's _num() BYTE FOR BYTE: integers without a decimal
  // point, otherwise two decimals with TRAILING ZEROS STRIPPED
  // (0.30 -> 0.3). Without the stripping the byte differs, so the
  // checksum differs, and the two sides drift apart silently -- which is
  // exactly what test_protocol.cpp caught on its first run.
  char pbuf[12];
  if (p1 == (int)p1) {
    snprintf(pbuf, sizeof(pbuf), "%d", (int)p1);
  } else {
    snprintf(pbuf, sizeof(pbuf), "%.2f", p1);
    size_t L = strlen(pbuf);
    while (L > 0 && pbuf[L - 1] == '0') pbuf[--L] = 0;      // trailing zeros
    if (L > 0 && pbuf[L - 1] == '.') pbuf[--L] = 0;         // dangling dot
  }
  snprintf(payload, sizeof(payload), "C,%u,%s,%s,0",
           (unsigned)(seq % SEQ_MODULO), cmd, pbuf);
  return buildFrame(payload, out, outSize);
}

// -- Decoded telemetry ------------------------------------------
struct Telemetry {
  uint16_t seq;
  long     cpm;
  long     mv;        // -1 = unknown (NEVER displayed as zero)
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

// -- Decoded ack (rejections stay visible on screen, not swallowed)
inline bool parseAck(const char *payload, char *codeOut, size_t outSize) {
  if (!payload || payload[0] != 'A' || payload[1] != ',') return false;
  const char *c = strchr(payload + 2, ',');
  if (!c) return false;
  strncpy(codeOut, c + 1, outSize - 1);
  codeOut[outSize - 1] = 0;
  return true;
}
