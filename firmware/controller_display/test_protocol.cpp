/*
 * test_protocol.cpp — يُثبت أن `protocol.h` يطابق `pi/comms/protocol.py`
 * ==================================================================
 * 🔴 **لماذا هذا الملف موجود**: البروتوكول مكتوب مرتين (بايثون على
 *    الراسبري، C++ على الشاشة). أي انحراف بينهما — بايت واحد في
 *    الـchecksum أو ترتيب حقل — يُنتج «أطراً تالفة» على قناة راديو
 *    لا يمكن تنقيحها بسهولة، والعرَض يبدو عطلاً كهربائياً لا برمجياً.
 *    فبدل الاعتماد على المراجعة البصرية، تُطبع المخرجات هنا وتُقارن
 *    بمخرجات بايثون آلياً في `pi/comms/selftest.py` (قسم الفيرموير).
 *
 * لا يُترجَم ضمن الفيرموير — أداة تحقّق على الحاسوب:
 *     g++ -std=c++17 -o test_protocol test_protocol.cpp && ./test_protocol
 */
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>

// ── بديل Arduino.h الأدنى (نحتاج strtol/strchr وحدها فعلياً) ──


#include "protocol.h"

int main() {
  char out[128], payload[128];

  // ١) أطر أوامر — تُقارن حرفياً بمخرجات encode_command في بايثون
  struct { uint16_t seq; const char *cmd; float p1; } cases[] = {
    {1, "FWD", 0.30f}, {7, "FWD", 0.0f}, {42, "ESTOP", 0.0f},
    {999, "STOP", 0.5f}, {123, "RTH", 0.25f}, {0, "STATUS", 0.0f},
    {500, "LEFT", 0.4f}, {12, "BACK", 0.1f}, {1001, "RIGHT", 1.0f},
  };
  for (auto &c : cases) {
    if (buildCommand(c.seq, c.cmd, c.p1, out, sizeof(out)))
      printf("CMD\t%s", out);
    else
      printf("CMD\tREJECT %s\n", c.cmd);
  }

  // ٢) القائمة المغلقة
  const char *evil[] = {"REBOOT", "rm -rf /", "FWD;BACK", "fwd", "", "EXEC"};
  for (auto e : evil)
    printf("ALLOW\t%s\t%d\n", e, isAllowedCommand(e) ? 1 : 0);

  // ٣) checksum خام
  const char *cks[] = {"PING,42", "C,1,FWD,0.3,0", "T,3,120,11320,RUN", "A,7,OK"};
  for (auto s : cks) printf("XOR\t%s\t%02X\n", s, xorChecksum(s));

  // ٤) فكّ الأطر: سليم · checksum خاطئ · بلا بادئة · بلا نجمة
  const char *frames[] = {
    "$C,1,FWD,0.3,0*4F", "$PING,42*3A", "$PING,42*00",
    "PING,42*3A", "$PING,42", "$T,3,120,11320,RUN*0C", ""
  };
  for (auto f : frames)
    printf("PARSE\t%s\t%s\n", f,
           parseFrame(f, payload, sizeof(payload)) ? payload : "REJECT");

  // ٥) تيليمتري — بما فيه الجهد المجهول (-1)
  const char *tels[] = {"T,3,120,11320,RUN", "T,4,0,-1,IDLE", "T,9,99999,12600,ESTOP"};
  for (auto t : tels) {
    Telemetry x = parseTelemetry(t);
    printf("TEL\t%s\t%d\t%u\t%ld\t%ld\t%s\n", t, x.valid ? 1 : 0,
           x.seq, x.cpm, x.mv, x.state);
  }
  return 0;
}
