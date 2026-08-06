/*
 * test_protocol.cpp -- proves protocol.h matches pi/comms/protocol.py
 * ==================================================================
 * WHY THIS FILE EXISTS: the protocol is written twice (Python on the
 * Pi, C++ on the display). Any drift between them -- one byte in the
 * checksum or a field order -- produces "corrupt frames" on a radio
 * channel that is miserable to debug, and the symptom looks electrical
 * rather than software. So instead of relying on eyeball review, the
 * vectors below are printed and compared against Python's output
 * automatically in pi/comms/selftest.py (firmware section).
 *
 * Not compiled into the firmware -- a host-side verification tool:
 *     g++ -std=c++17 -I test_stub -o test_protocol test_protocol.cpp
 *     ./test_protocol
 *
 * ASCII only in this file -- see the BiDi note in controller_display.ino.
 */
// The Arduino IDE compiles EVERY root-level .cpp in the sketch folder,
// and this host-only tool (with its own main()) must never be part of
// the firmware build. ARDUINO is defined by the IDE/arduino-cli and is
// absent in the plain g++ host build, so this guard excludes the file
// from the firmware while leaving the host test untouched.
#if !defined(ARDUINO)
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>

// Minimal Arduino.h stand-in lives in test_stub/ (strtol/strchr only)


#include "protocol.h"

int main() {
  char out[128], payload[128];

  // 1) Command frames -- compared byte-for-byte with encode_command in Python
  struct { uint16_t seq; const char *cmd; float p1; } cases[] = {
    {1, "FWD", 0.30f}, {7, "FWD", 0.0f}, {42, "ESTOP", 0.0f},
    {999, "STOP", 0.5f}, {123, "RTH", 0.25f}, {0, "STATUS", 0.0f},
    {500, "LEFT", 0.4f}, {12, "BACK", 0.1f}, {1001, "RIGHT", 1.0f},
    {77, "WDRAW", 0.0f},
  };
  for (auto &c : cases) {
    if (buildCommand(c.seq, c.cmd, c.p1, out, sizeof(out)))
      printf("CMD\t%s", out);
    else
      printf("CMD\tREJECT %s\n", c.cmd);
  }

  // 2) The closed command list
  const char *evil[] = {"REBOOT", "rm -rf /", "FWD;BACK", "fwd", "", "EXEC"};
  for (auto e : evil)
    printf("ALLOW\t%s\t%d\n", e, isAllowedCommand(e) ? 1 : 0);

  // 3) Raw checksums
  const char *cks[] = {"PING,42", "C,1,FWD,0.3,0", "T,3,120,11320,RUN", "A,7,OK"};
  for (auto s : cks) printf("XOR\t%s\t%02X\n", s, xorChecksum(s));

  // 4) Frame parsing: valid / bad checksum / no prefix / no star
  const char *frames[] = {
    "$C,1,FWD,0.3,0*4F", "$PING,42*3A", "$PING,42*00",
    "PING,42*3A", "$PING,42", "$T,3,120,11320,RUN*0C", ""
  };
  for (auto f : frames)
    printf("PARSE\t%s\t%s\n", f,
           parseFrame(f, payload, sizeof(payload)) ? payload : "REJECT");

  // 5) Telemetry -- including unknown voltage (-1)
  const char *tels[] = {"T,3,120,11320,RUN", "T,4,0,-1,IDLE", "T,9,99999,12600,ESTOP"};
  for (auto t : tels) {
    Telemetry x = parseTelemetry(t);
    printf("TEL\t%s\t%d\t%u\t%ld\t%ld\t%s\n", t, x.valid ? 1 : 0,
           x.seq, x.cpm, x.mv, x.state);
  }
  return 0;
}

#endif  // !defined(ARDUINO)
