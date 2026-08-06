/*
 * lora_probe.ino -- byte-level HC-14 link probe (runs on the CYD ESP32)
 * ==========================================================================
 * PURPOSE: isolate the radio path from ALL display code. No TFT, no touch,
 * no protocol state -- just the HC-14 on UART2 and the USB serial monitor.
 * Use it to answer exactly one question per direction:
 *   "what BYTES actually cross this link, and how are they damaged?"
 *
 * What it does:
 *  - Every 2s transmits a valid frame  $PROBE,<n>*HH\n  over the radio
 *    and logs it to USB as  TX> ...
 *  - Everything received from the radio is dumped to USB as raw HEX plus
 *    printable ASCII (RX< ...), so corruption is visible byte by byte:
 *      * random garbage bytes            => UART baud / wiring / power
 *      * clean text but frames cut short => framing / buffering
 *      * mostly-right text, few flipped  => RF (saturation: two +20dBm
 *        bytes                              modules on the same desk)
 *  - Complete lines are checksum-verified:  FRAME OK / FRAME BAD.
 *  - Any line you type into the serial monitor starting with '$' is sent
 *    to the radio verbatim (manual injection).
 *
 * Pairing with the Pi tools (pi/tests/test_lora.py):
 *   display->Pi : run this probe, on the Pi run  ... test_lora.py hex
 *   Pi->display : on the Pi run  ... test_lora.py probe , watch RX< here
 *
 * Build: board "ESP32 Dev Module" -- no libraries needed at all.
 * ASCII only in this file (see firmware/controller_display/README.md).
 */
#include <Arduino.h>

// Same proven pins as controller_display/config.h, model 1 (3248S035R).
// For model 2 (2432S028) use RX 35 / TX 27 instead.
#define LORA_RX_PIN     21     // <- HC-14 TX
#define LORA_TX_PIN     22     // -> HC-14 RX
#define LORA_BAUD       9600
#define USB_BAUD        115200
#define PROBE_PERIOD_MS 2000

HardwareSerial loraSerial(2);

uint32_t lastTxMs = 0;
uint32_t probeN   = 0;

char lineBuf[96];  size_t lineLen = 0;   // radio line assembly
char usbBuf[96];   size_t usbLen  = 0;   // USB line assembly

uint8_t xorChecksum(const char *p) {
  uint8_t c = 0;
  while (*p) c ^= (uint8_t)(*p++);
  return c;
}

// Verify "$<payload>*<HH>" -- prints the verdict for each complete line.
void checkFrame(const char *line) {
  if (line[0] != '$') { Serial.printf("FRAME BAD (no '$'): %s\n", line); return; }
  const char *star = strrchr(line, '*');
  if (!star || !star[1] || !star[2]) {
    Serial.printf("FRAME BAD (no *HH): %s\n", line); return;
  }
  char payload[90];
  size_t n = (size_t)(star - line - 1);
  if (n == 0 || n >= sizeof(payload)) { Serial.printf("FRAME BAD (len): %s\n", line); return; }
  memcpy(payload, line + 1, n); payload[n] = 0;
  char hex[3] = { star[1], star[2], 0 };
  uint8_t got = (uint8_t)strtol(hex, nullptr, 16);
  uint8_t want = xorChecksum(payload);
  if (got == want) Serial.printf("FRAME OK : %s\n", payload);
  else Serial.printf("FRAME BAD (ck %02X != %02X): %s\n", got, want, line);
}

// Dump a chunk as hex + printable ASCII, and feed the line assembler.
void dumpAndAssemble(const uint8_t *d, size_t n) {
  Serial.print("RX< hex:");
  for (size_t i = 0; i < n; ++i) Serial.printf(" %02X", d[i]);
  Serial.print("  ascii:\"");
  for (size_t i = 0; i < n; ++i) {
    char c = (char)d[i];
    Serial.print((c >= 32 && c < 127) ? c : '.');
  }
  Serial.println("\"");
  for (size_t i = 0; i < n; ++i) {
    char c = (char)d[i];
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = 0; checkFrame(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                       // over-long junk => discard
    }
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  loraSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(300);
  Serial.println();
  Serial.println("=== lora_probe: HC-14 byte-level link probe ===");
  Serial.printf("UART2 rx=%d tx=%d @ %d. Sends $PROBE,<n> every %dms.\n",
                LORA_RX_PIN, LORA_TX_PIN, LORA_BAUD, PROBE_PERIOD_MS);
  Serial.println("Type a $...*HH line here to inject it to the radio.");
}

void loop() {
  // Periodic probe transmission
  uint32_t now = millis();
  if (now - lastTxMs >= PROBE_PERIOD_MS) {
    lastTxMs = now;
    char payload[24], frame[32];
    snprintf(payload, sizeof(payload), "PROBE,%lu", (unsigned long)++probeN);
    snprintf(frame, sizeof(frame), "$%s*%02X\n", payload, xorChecksum(payload));
    loraSerial.print(frame);
    Serial.printf("TX> %s", frame);
  }

  // Radio -> hex dump + frame check
  uint8_t chunk[64];
  size_t n = 0;
  while (loraSerial.available() && n < sizeof(chunk)) chunk[n++] = (uint8_t)loraSerial.read();
  if (n > 0) dumpAndAssemble(chunk, n);

  // USB -> manual frame injection
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbLen > 0) {
        usbBuf[usbLen] = 0; usbLen = 0;
        if (usbBuf[0] == '$') {
          loraSerial.print(usbBuf); loraSerial.print('\n');
          Serial.printf("TX> %s (manual)\n", usbBuf);
        }
      }
    } else if (usbLen < sizeof(usbBuf) - 1) {
      usbBuf[usbLen++] = c;
    } else {
      usbLen = 0;
    }
  }
}
