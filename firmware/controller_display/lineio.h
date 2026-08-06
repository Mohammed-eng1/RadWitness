/*
 * lineio.h -- non-blocking line reader shared by USB and LoRa ports
 * ==================================================================
 * WHY THIS LIVES IN A HEADER, NOT IN THE .INO:
 * The Arduino IDE sketch preprocessor auto-generates a prototype for
 * every function it finds in the .ino. For this function it chokes --
 * the combination of `template <typename S>` and a function-pointer
 * parameter produces a mangled prototype, injected with a #line
 * directive that blames the innocent COMMENT line right above the
 * function ("expected ')' before ';' token"). That is exactly the
 * error this project hit twice: once blamed on an Arabic comment
 * (line 200), once on its English replacement (line 219) -- same
 * error after the comment was deleted, which is the tell.
 * Header files are compiled verbatim, no prototype generation, so the
 * problem cannot recur here.
 *
 * ASCII only in this file -- see the note in controller_display.ino.
 */
#pragma once
#include <Arduino.h>

// Callback invoked once per complete received line (without \r\n).
typedef void (*LineCb)(const char *);

// Reads whatever bytes are available on `port`, splits on \r or \n,
// and invokes `cb` for each complete non-empty line. Never blocks.
template <typename S>
void pump(S &port, char *buf, size_t cap, size_t &len, LineCb cb) {
  while (port.available()) {
    char ch = (char)port.read();
    if (ch == '\n' || ch == '\r') {
      if (len > 0) { buf[len] = 0; cb(buf); len = 0; }
    } else if (len < cap - 1) {
      buf[len++] = ch;
    } else {
      len = 0;                                // line exceeds cap => discard it
    }
  }
}
