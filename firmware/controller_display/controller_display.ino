/*
 * controller_display.ino -- LoRa controller on a CYD (Cheap Yellow Display)
 * ==========================================================================
 * Role in the system: an EMERGENCY CHANNEL fully independent of the
 * internet and Tailscale.
 *
 *     Browser --USB--> this display --LoRa--> HC-14 on the Pi --> robot
 *
 * NOTE: this is NOT a copy of the legacy firmware. The legacy one
 * (legacy/RadiationRover/firmware/controller_display) builds WiFi +
 * WebServer + mDNS + WebSocket + an embedded site -- all of which depend
 * on the network, i.e. on exactly what this channel must work WITHOUT.
 * What was carried over is what deserved it: the proven pin map and the
 * touch handling logic (config.h).
 *
 * -- Two modes, detected by traffic ---------------------------------
 *  1) STANDALONE: touch buttons send commands directly over LoRa
 *     (a complete handheld controller).
 *  2) BRIDGE: when driven by a computer over USB, it relays frames
 *     between the browser and the radio in both directions.
 *  Transition: first valid frame from USB => bridge; silence longer
 *  than BRIDGE_IDLE_MS => back to standalone. (Why we cannot detect the
 *  USB cable electrically is explained in config.h.)
 *
 * -- Shown on screen: CPM, voltage, link state, last command, mode --
 *
 * IMPORTANT: all comments and string literals in this firmware must be
 * plain ASCII English. Arabic (RTL/BiDi) text in C++ sources reorders
 * invisibly in editors and has broken the Arduino build before
 * ("expected ')' before ';'" pointing at a comment line). Arabic is
 * fine in Python and Markdown -- never here. An automated ASCII guard
 * in pi/comms/selftest.py enforces this.
 *
 * Libraries: TFT_eSPI (+ XPT2046_Touchscreen on DISPLAY_MODEL 2 only -- see
 * the touch note below; no WiFi, no JSON -- the channel is narrow).
 * Build: board "ESP32 Dev Module" + the TFT_eSPI setup file from
 *        legacy/RadiationRover/firmware/controller_display/tft_setup/
 */
#include <SPI.h>
#include <TFT_eSPI.h>

#include "config.h"
#include "protocol.h"
#include "lineio.h"   // pump() lives in a header ON PURPOSE -- see its note

TFT_eSPI tft = TFT_eSPI();
HardwareSerial loraSerial(2);

// === Touch input: the path DIFFERS BY BOARD MODEL ==============
// Model 1 (3248S035R, 3.5"): the XPT2046 shares the DISPLAY SPI bus and is
//   read through TFT_eSPI (tft.getTouch), with TOUCH_CS set in the TFT_eSPI
//   Setup file. Calibration is stored in NVS.
// Model 2 (2432S028, 2.8"): raw XPT2046 on its own SPI bus.
// Measured on hardware 2026-08-09: driving model 1 down the model-2 path
// reads a floating MISO -- z=4095, y=8191, touched() true on every sample,
// IRQ still working -- and every button is dead. Same symptom as a broken
// panel, entirely different cause.
#if DISPLAY_MODEL == 1
  #include <Preferences.h>
#else
  #include <XPT2046_Touchscreen.h>
  SPIClass touchSpi(VSPI);
  XPT2046_Touchscreen ts(TOUCH_CS_PIN, TOUCH_IRQ_PIN);
#endif

// === Displayed state ==========================================
long     dCpm      = 0;
long     dMv       = -1;        // -1 = unknown (NEVER shown as zero)
char     dState[10] = "?";
char     dLastCmd[12] = "-";
char     dLastAck[10] = "";
bool     bridgeMode = false;
uint32_t lastUsbFrameMs = 0;
uint32_t lastLoraRxMs   = 0;
uint16_t txSeq          = 0;
uint32_t lastUiMs       = 0;
bool     uiDirty        = true;

// Currently held button (press-and-hold keeps the robot moving)
int      heldBtn        = -1;
uint32_t lastRepeatMs   = 0;

// === Buttons ==================================================
struct Btn { int x, y, w, h; const char *label; const char *cmd; uint16_t col; };
#define NBTN 8
Btn btns[NBTN];
int HDR = 0, PANEL_H = 0;

bool loraLink() { return (millis() - lastLoraRxMs) < LORA_TIMEOUT_MS; }

// === Layout ===================================================
void layout() {
  int W = tft.width(), H = tft.height();
  HDR = 96;                                  // readings panel on top
  int gy = HDR + 6;
  int gh = H - gy - 6;
  int cw = W / 3, ch = gh / 3;

  // Row 1: < | ^ | >   Row 2: WDRW | STOP | RTH   Row 3: v | ESTOP(x2)
  // WDRW (withdraw) replaces STAT deliberately: telemetry is broadcast
  // periodically anyway, while withdraw is a safety order the operator
  // must be able to issue from the standalone handheld -- this screen
  // may be their only channel when the network is down.
  int i = 0;
  btns[i++] = { 0,      gy,          cw, ch, "<",     "LEFT",   COL_PANEL };
  btns[i++] = { cw,     gy,          cw, ch, "^",     "FWD",    COL_PANEL };
  btns[i++] = { cw * 2, gy,          cw, ch, ">",     "RIGHT",  COL_PANEL };
  btns[i++] = { 0,      gy + ch,     cw, ch, "WDRW",  "WDRAW",  0x6300 };
  btns[i++] = { cw,     gy + ch,     cw, ch, "STOP",  "STOP",   0x5000 };
  btns[i++] = { cw * 2, gy + ch,     cw, ch, "RTH",   "RTH",    COL_PANEL };
  btns[i++] = { 0,      gy + ch * 2, cw, ch, "v",     "BACK",   COL_PANEL };
  btns[i++] = { cw,     gy + ch * 2, cw * 2, ch, "ESTOP", "ESTOP", COL_BAD };
}

void drawButtons() {
  for (int i = 0; i < NBTN; ++i) {
    Btn &b = btns[i];
    uint16_t bg = (i == heldBtn) ? COL_ACCENT : b.col;
    tft.fillRoundRect(b.x + 3, b.y + 3, b.w - 6, b.h - 6, 7, bg);
    tft.drawRoundRect(b.x + 3, b.y + 3, b.w - 6, b.h - 6, 7, COL_LINE);
    tft.setTextColor(COL_FG, bg);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(strlen(b.label) <= 1 ? 3 : 2);
    tft.drawString(b.label, b.x + b.w / 2, b.y + b.h / 2);
  }
  tft.setTextDatum(TL_DATUM);
}

// === Readings panel ===========================================
void drawHeader() {
  tft.fillRect(0, 0, tft.width(), HDR, COL_PANEL);
  tft.drawFastHLine(0, HDR - 1, tft.width(), COL_LINE);
  tft.setTextDatum(TL_DATUM);

  // Mode indicator -- always prominent (explicit requirement)
  tft.setTextSize(2);
  tft.setTextColor(bridgeMode ? COL_ACCENT : COL_OK, COL_PANEL);
  tft.drawString(bridgeMode ? "BRIDGE" : "STANDALONE", 8, 8);

  // Link state
  bool up = loraLink();
  tft.setTextColor(up ? COL_OK : COL_BAD, COL_PANEL);
  tft.setTextSize(2);
  tft.drawString(up ? "LoRa OK" : "NO LINK", tft.width() - 118, 8);

  // CPM
  tft.setTextSize(3);
  tft.setTextColor(COL_FG, COL_PANEL);
  char buf[28];
  snprintf(buf, sizeof(buf), "%ld CPM", dCpm);
  tft.drawString(buf, 8, 34);

  // Voltage -- unknown is printed "-- V", NEVER "0.00V" (a zero reading
  // is a catastrophe and "don't know" is not; the difference is the
  // whole point of the -1 sentinel).
  tft.setTextSize(2);
  if (dMv < 0) {
    tft.setTextColor(COL_DIM, COL_PANEL);
    tft.drawString("-- V", tft.width() - 118, 40);
  } else {
    float v = dMv / 1000.0f;
    // 3S threshold colors (must match pi/config.py)
    uint16_t c = (v >= 11.5f) ? COL_OK : (v >= 10.8f) ? COL_WARN : COL_BAD;
    tft.setTextColor(c, COL_PANEL);
    snprintf(buf, sizeof(buf), "%.2f V", v);
    tft.drawString(buf, tft.width() - 118, 40);
  }

  // Last command + its ack
  tft.setTextSize(2);
  tft.setTextColor(COL_DIM, COL_PANEL);
  snprintf(buf, sizeof(buf), "%s %s  [%s]", dLastCmd,
           dLastAck[0] ? dLastAck : "", dState);
  tft.drawString(buf, 8, 68);
}

void redraw() { drawHeader(); drawButtons(); }

// === Transmit =================================================
void sendCommand(const char *cmd, float p1) {
  if (!isAllowedCommand(cmd)) return;         // nothing outside the closed list
  char frame[MAX_PAYLOAD + 8];
  txSeq = (txSeq + 1) % SEQ_MODULO;
  if (!buildCommand(txSeq, cmd, p1, frame, sizeof(frame))) return;
  loraSerial.print(frame);
  strncpy(dLastCmd, cmd, sizeof(dLastCmd) - 1);
  dLastCmd[sizeof(dLastCmd) - 1] = 0;
  dLastAck[0] = 0;                            // await a fresh ack
  uiDirty = true;
}

// === Receive from the radio ===================================
void handleLoraLine(const char *line) {
  char payload[MAX_PAYLOAD + 1];
  if (!parseFrame(line, payload, sizeof(payload))) return;   // corrupt => drop
  lastLoraRxMs = millis();

  if (payload[0] == 'T') {
    Telemetry t = parseTelemetry(payload);
    if (t.valid) {
      dCpm = t.cpm; dMv = t.mv;
      strncpy(dState, t.state, sizeof(dState) - 1);
      dState[sizeof(dState) - 1] = 0;
      uiDirty = true;
    }
  } else if (payload[0] == 'A') {
    char code[10];
    if (parseAck(payload, code, sizeof(code))) {
      strncpy(dLastAck, code, sizeof(dLastAck) - 1);
      dLastAck[sizeof(dLastAck) - 1] = 0;
      uiDirty = true;                         // rejections are shown, not swallowed
    }
  }
  // In bridge mode: relay everything received to the browser verbatim
  if (bridgeMode) { Serial.print(line); Serial.print('\n'); }
}

// === Receive from USB (the browser) ===========================
void handleUsbLine(const char *line) {
  char payload[MAX_PAYLOAD + 1];
  if (!parseFrame(line, payload, sizeof(payload))) return;
  // A valid frame from USB means "a computer is driving me now" => bridge
  lastUsbFrameMs = millis();
  if (!bridgeMode) { bridgeMode = true; uiDirty = true; }
  // Relayed VERBATIM -- no interpretation, no command synthesis. Final
  // validation happens on the Pi (sequence + checksum + closed list +
  // safety layer). The bridge is a carrier, not an authority -- any
  // "smarts" here would become a second guard that someone must
  // maintain, and one of the two would eventually be forgotten.
  loraSerial.print(line);
  loraSerial.print('\n');
}

// (line reader pump() is in lineio.h -- the sketch preprocessor mangles
//  its prototype when it sits in the .ino; see the note there)

char usbBuf[96];  size_t usbLen  = 0;
char loraBuf[96]; size_t loraLen = 0;

// === Touch ====================================================
int btnAt(int x, int y) {
  for (int i = 0; i < NBTN; ++i) {
    Btn &b = btns[i];
    if (x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) return i;
  }
  return -1;
}

// Returns true and fills SCREEN coordinates when the panel is pressed.
#if DISPLAY_MODEL == 1
static bool readTouch(int &x, int &y) {
  uint16_t tx = 0, ty = 0;
  // getTouch applies the stored calibration, so this is already in screen
  // space -- no raw mapping, no swap/invert (those are model-2 knobs).
  bool ok = tft.getTouch(&tx, &ty, TOUCH_PRESSURE_TH);
#if TOUCH_DEBUG
  // Prints what the panel and the calibration ACTUALLY produce. Needed
  // because "buttons dead" has two very different causes that look
  // identical: the panel not reading (rawz stays 0), or a calibration whose
  // converted point lands outside the screen -- TFT_eSPI's getTouch rejects
  // out-of-bounds points silently, so a bad calibration reads as "no touch".
  // Silent in BRIDGE mode: those bytes belong to the browser's frame stream.
  static uint32_t lastDbg = 0;
  uint16_t rz = tft.getTouchRawZ();
  if (rz > 0 && !bridgeMode && (millis() - lastDbg) > 150) {
    lastDbg = millis();
    uint16_t rx = 0, ry = 0;
    tft.getTouchRaw(&rx, &ry);
    Serial.printf("TOUCH rawz=%u rawx=%u rawy=%u | getTouch=%s sx=%u sy=%u"
                  " | screen=%dx%d btn=%d\n",
                  rz, rx, ry, ok ? "YES" : "NO", tx, ty,
                  tft.width(), tft.height(),
                  ok ? btnAt((int)tx, (int)ty) : -2);
  }
#endif
  if (!ok) return false;
  x = (int)tx; y = (int)ty;
  return true;
}
#else
static bool readTouch(int &x, int &y) {
  if (!ts.touched()) return false;
  TS_Point p = ts.getPoint();
  if (p.z < TOUCH_PRESSURE_TH) return false;
  x = map(p.x, TOUCH_RAW_MIN, TOUCH_RAW_MAX, 0, tft.width());
  y = map(p.y, TOUCH_RAW_MIN, TOUCH_RAW_MAX, 0, tft.height());
#if TOUCH_SWAP_XY
  int t = x; x = y; y = t;
#endif
#if TOUCH_INVERT_X
  x = tft.width() - x;
#endif
#if TOUCH_INVERT_Y
  y = tft.height() - y;
#endif
  return true;
}
#endif

void handleTouch() {
  int x = 0, y = 0;
  bool touched = readTouch(x, y);
  if (touched) {
    int i = btnAt(x, y);
    if (i >= 0 && i != heldBtn) {
      heldBtn = i;
      sendCommand(btns[i].cmd, 0.30f);
      lastRepeatMs = millis();
      uiDirty = true;
    } else if (i >= 0) {
      // Press-and-hold keeps moving: refresh the command before the
      // radio command timeout (2s on the Pi) can cut it off.
      if (millis() - lastRepeatMs >= DRIVE_REPEAT_MS) {
        sendCommand(btns[i].cmd, 0.30f);
        lastRepeatMs = millis();
      }
    }
  }
  if (!touched && heldBtn >= 0) {
    // Finger lifted off a motion button => IMMEDIATE stop, not "wait
    // for the timeout to notice".
    const char *c = btns[heldBtn].cmd;
    bool motion = (strcmp(c, "FWD") == 0 || strcmp(c, "BACK") == 0 ||
                   strcmp(c, "LEFT") == 0 || strcmp(c, "RIGHT") == 0);
    heldBtn = -1;
    if (motion) sendCommand("STOP", 0);
    uiDirty = true;
  }
}

// === setup / loop =============================================
void setup() {
  Serial.begin(USB_BAUD);                       // <- browser (Web Serial)
  loraSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  pinMode(TFT_BL_PIN, OUTPUT);
  analogWrite(TFT_BL_PIN, (255 * TFT_BACKLIGHT_PCT) / 100);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(COL_BG);

#if DISPLAY_MODEL == 1
  // Calibration persists in NVS; the first run (or TOUCH_FORCE_CALIBRATE=1)
  // asks for the four corners. Mirrors the proven legacy firmware.
  {
    uint16_t calData[5];
    Preferences prefs;
    // Namespace is OURS, not the legacy firmware's "disp": that one is still
    // in NVS (flashing with Erase Flash disabled keeps it) and its blob was
    // recorded at a different rotation/layout. Loading it silently skipped
    // the corner prompt and mapped every press to the wrong place -- seen on
    // hardware 2026-08-09. The tag below adds a second guard: any calibration
    // not written by THIS layout/rotation is discarded and redone, so future
    // layout changes cannot resurrect the same bug.
    prefs.begin("cydctl", false);
    bool haveCal = (prefs.getBytesLength("touchcal") == sizeof(calData))
                   && (prefs.getUInt("caltag", 0) == TOUCH_CAL_TAG);
  #if TOUCH_FORCE_CALIBRATE
    haveCal = false;
  #endif
    if (haveCal) {
      prefs.getBytes("touchcal", calData, sizeof(calData));
      tft.setTouch(calData);
    } else {
      tft.fillScreen(COL_BG);
      tft.setTextColor(COL_FG, COL_BG);
      tft.setTextSize(2);
      tft.setCursor(10, 10); tft.println("Touch each corner");
      tft.setCursor(10, 40); tft.println("marker in turn");
      tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 20);
      prefs.putBytes("touchcal", calData, sizeof(calData));
      prefs.putUInt("caltag", TOUCH_CAL_TAG);
    }
    prefs.end();
  }
#else
  touchSpi.begin(TOUCH_CLK_PIN, TOUCH_MISO_PIN, TOUCH_MOSI_PIN, TOUCH_CS_PIN);
  ts.begin(touchSpi);
  ts.setRotation(0);
#endif

  layout();
  redraw();
}

void loop() {
  pump(Serial, usbBuf, sizeof(usbBuf), usbLen, handleUsbLine);
  pump(loraSerial, loraBuf, sizeof(loraBuf), loraLen, handleLoraLine);

  // USB silence elapsed => back to standalone (detection by traffic)
  if (bridgeMode && (millis() - lastUsbFrameMs) > BRIDGE_IDLE_MS) {
    bridgeMode = false;
    uiDirty = true;
  }

  // Touch works in BOTH modes: the on-device emergency button must not
  // go dead just because a computer is attached -- it is the most
  // reliable path exactly when the browser freezes.
  handleTouch();

  uint32_t now = millis();
  if (uiDirty || (now - lastUiMs) >= UI_UPDATE_MS) {
    lastUiMs = now;
    uiDirty = false;
    redraw();
  }
}
