/*
 * controller_display.ino — وحدة تحكم LoRa على شاشة CYD
 * ==================================================================
 * دورها في المنظومة: **قناة طوارئ مستقلة عن الإنترنت وTailscale**.
 *
 *     المتصفح ──USB──> هذه الشاشة ──LoRa──> HC-14 على الراسبري ──> الروبوت
 *
 * ⚠ **ليست نسخة من الفيرموير القديم**: القديم (legacy/…/controller_display)
 *   يبني WiFi + WebServer + mDNS + WebSocket + موقعاً مضمّناً — وكلها
 *   تعتمد على الشبكة، أي على ما وُجدت هذه القناة لتعمل **بدونه**. المنقول
 *   منه هو ما يستحق النقل: خريطة المنافذ المجرَّبة ومنطق اللمس (config.h).
 *
 * ── وضعان، والكشف بالحركة ─────────────────────────────────────
 *  1) **مستقل**: أزرار اللمس ترسل أوامر عبر LoRa مباشرة (وحدة تحكم كاملة).
 *  2) **جسر**:   يمرّر بين المتصفح (USB) والراديو في الاتجاهين.
 *  الانتقال: أول إطار صالح من USB ⇒ جسر · صمت BRIDGE_IDLE_MS ⇒ مستقل.
 *  (سبب عدم الاعتماد على كشف USB كهربائياً مشروح في config.h.)
 *
 * ── تُعرض: CPM · الجهد · حالة الاتصال · آخر أمر · مؤشّر الوضع ──
 *
 * المكتبات: TFT_eSPI + XPT2046_Touchscreen (لا WiFi ولا JSON — القناة ضيقة)
 * الترجمة: لوحة "ESP32 Dev Module" + إعداد TFT_eSPI من
 *          legacy/RadiationRover/firmware/controller_display/tft_setup/
 */
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#include "config.h"
#include "protocol.h"

TFT_eSPI tft = TFT_eSPI();
SPIClass touchSpi(VSPI);
XPT2046_Touchscreen ts(TOUCH_CS_PIN, TOUCH_IRQ_PIN);
HardwareSerial loraSerial(2);

// ═══ الحالة المعروضة ══════════════════════════════════════════
long     dCpm      = 0;
long     dMv       = -1;        // -1 = مجهول (**لا يُعرض صفراً**)
char     dState[10] = "?";
char     dLastCmd[12] = "—";
char     dLastAck[10] = "";
bool     bridgeMode = false;
uint32_t lastUsbFrameMs = 0;
uint32_t lastLoraRxMs   = 0;
uint16_t txSeq          = 0;
uint32_t lastUiMs       = 0;
bool     uiDirty        = true;

// الزرّ المضغوط حالياً (الضغط المستمر يحرّك)
int      heldBtn        = -1;
uint32_t lastRepeatMs   = 0;

// ═══ الأزرار ══════════════════════════════════════════════════
struct Btn { int x, y, w, h; const char *label; const char *cmd; uint16_t col; };
#define NBTN 8
Btn btns[NBTN];
int HDR = 0, PANEL_H = 0;

bool loraLink() { return (millis() - lastLoraRxMs) < LORA_TIMEOUT_MS; }

// ═══ التخطيط ══════════════════════════════════════════════════
void layout() {
  int W = tft.width(), H = tft.height();
  HDR = 96;                                  // لوحة القراءات أعلى
  int gy = HDR + 6;
  int gh = H - gy - 6;
  int cw = W / 3, ch = gh / 3;

  // صفّ 1: ⟲ | ▲ | ⟳          صفّ 2: ◀ | STOP | ▶
  // صفّ 3: ▼ | ESTOP | RTH
  int i = 0;
  btns[i++] = { 0,      gy,          cw, ch, "<",     "LEFT",   COL_PANEL };
  btns[i++] = { cw,     gy,          cw, ch, "^",     "FWD",    COL_PANEL };
  btns[i++] = { cw * 2, gy,          cw, ch, ">",     "RIGHT",  COL_PANEL };
  btns[i++] = { 0,      gy + ch,     cw, ch, "STAT",  "STATUS", COL_PANEL };
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

// ═══ لوحة القراءات ════════════════════════════════════════════
void drawHeader() {
  tft.fillRect(0, 0, tft.width(), HDR, COL_PANEL);
  tft.drawFastHLine(0, HDR - 1, tft.width(), COL_LINE);
  tft.setTextDatum(TL_DATUM);

  // الوضع — بارز دائماً (مطلوب صراحةً)
  tft.setTextSize(2);
  tft.setTextColor(bridgeMode ? COL_ACCENT : COL_OK, COL_PANEL);
  tft.drawString(bridgeMode ? "BRIDGE" : "STANDALONE", 8, 8);

  // حالة القناة
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

  // الجهد — 🔴 مجهول يُكتب "-- V" لا "0.00V"
  tft.setTextSize(2);
  if (dMv < 0) {
    tft.setTextColor(COL_DIM, COL_PANEL);
    tft.drawString("-- V", tft.width() - 118, 40);
  } else {
    float v = dMv / 1000.0f;
    // ألوان عتبات 3S (تطابق pi/config.py)
    uint16_t c = (v >= 11.5f) ? COL_OK : (v >= 10.8f) ? COL_WARN : COL_BAD;
    tft.setTextColor(c, COL_PANEL);
    snprintf(buf, sizeof(buf), "%.2f V", v);
    tft.drawString(buf, tft.width() - 118, 40);
  }

  // آخر أمر + إقراره
  tft.setTextSize(2);
  tft.setTextColor(COL_DIM, COL_PANEL);
  snprintf(buf, sizeof(buf), "%s %s  [%s]", dLastCmd,
           dLastAck[0] ? dLastAck : "", dState);
  tft.drawString(buf, 8, 68);
}

void redraw() { drawHeader(); drawButtons(); }

// ═══ الإرسال ══════════════════════════════════════════════════
void sendCommand(const char *cmd, float p1) {
  if (!isAllowedCommand(cmd)) return;         // 🔴 لا شيء خارج القائمة
  char frame[MAX_PAYLOAD + 8];
  txSeq = (txSeq + 1) % SEQ_MODULO;
  if (!buildCommand(txSeq, cmd, p1, frame, sizeof(frame))) return;
  loraSerial.print(frame);
  strncpy(dLastCmd, cmd, sizeof(dLastCmd) - 1);
  dLastCmd[sizeof(dLastCmd) - 1] = 0;
  dLastAck[0] = 0;                            // ننتظر إقراراً جديداً
  uiDirty = true;
}

// ═══ استقبال من الراديو ═══════════════════════════════════════
void handleLoraLine(const char *line) {
  char payload[MAX_PAYLOAD + 1];
  if (!parseFrame(line, payload, sizeof(payload))) return;   // تالف ⇒ تجاهل
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
      uiDirty = true;                         // الرفض يظهر لا يُبتلع
    }
  }
  // في وضع الجسر: مرّر كل ما وصل إلى المتصفح كما هو
  if (bridgeMode) { Serial.print(line); Serial.print('\n'); }
}

// ═══ استقبال من USB (المتصفح) ═════════════════════════════════
void handleUsbLine(const char *line) {
  char payload[MAX_PAYLOAD + 1];
  if (!parseFrame(line, payload, sizeof(payload))) return;
  // ⚠ إطار صالح من USB = «يقودني حاسوب الآن» ⇒ وضع الجسر
  lastUsbFrameMs = millis();
  if (!bridgeMode) { bridgeMode = true; uiDirty = true; }
  // 🔴 يُمرَّر كما هو **بلا تفسير ولا توليد أوامر**: التحقق النهائي عند
  //    الراسبري (تسلسل + checksum + قائمة مغلقة + طبقة سلامة). الجسر
  //    ناقل لا سلطة — وأي «ذكاء» هنا يصير حارساً ثانياً يجب صيانته.
  loraSerial.print(line);
  loraSerial.print('\n');
}

// ═══ قراءة سطر من منفذ (بلا حجب) ══════════════════════════════
template <typename S>
void pump(S &port, char *buf, size_t cap, size_t &len, void (*cb)(const char *)) {
  while (port.available()) {
    char ch = (char)port.read();
    if (ch == '\n' || ch == '\r') {
      if (len > 0) { buf[len] = 0; cb(buf); len = 0; }
    } else if (len < cap - 1) {
      buf[len++] = ch;
    } else {
      len = 0;                                // سطر أطول من الحدّ ⇒ اطرحه
    }
  }
}

char usbBuf[96];  size_t usbLen  = 0;
char loraBuf[96]; size_t loraLen = 0;

// ═══ اللمس ════════════════════════════════════════════════════
int btnAt(int x, int y) {
  for (int i = 0; i < NBTN; ++i) {
    Btn &b = btns[i];
    if (x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) return i;
  }
  return -1;
}

void handleTouch() {
  bool touched = ts.touched();
  if (touched) {
    TS_Point p = ts.getPoint();
    if (p.z < TOUCH_PRESSURE_TH) { touched = false; }
    else {
      int x = map(p.x, TOUCH_RAW_MIN, TOUCH_RAW_MAX, 0, tft.width());
      int y = map(p.y, TOUCH_RAW_MIN, TOUCH_RAW_MAX, 0, tft.height());
#if TOUCH_SWAP_XY
      int t = x; x = y; y = t;
#endif
#if TOUCH_INVERT_X
      x = tft.width() - x;
#endif
#if TOUCH_INVERT_Y
      y = tft.height() - y;
#endif
      int i = btnAt(x, y);
      if (i >= 0 && i != heldBtn) {
        heldBtn = i;
        sendCommand(btns[i].cmd, 0.30f);
        lastRepeatMs = millis();
        uiDirty = true;
      } else if (i >= 0) {
        // الضغط المستمر يحرّك: جدّد الأمر قبل انتهاء مهلة الراديو
        if (millis() - lastRepeatMs >= DRIVE_REPEAT_MS) {
          sendCommand(btns[i].cmd, 0.30f);
          lastRepeatMs = millis();
        }
      }
    }
  }
  if (!touched && heldBtn >= 0) {
    // 🔴 رفع الإصبع عن زرّ حركة ⇒ **إيقاف فوري** لا انتظار المهلة
    const char *c = btns[heldBtn].cmd;
    bool motion = (strcmp(c, "FWD") == 0 || strcmp(c, "BACK") == 0 ||
                   strcmp(c, "LEFT") == 0 || strcmp(c, "RIGHT") == 0);
    heldBtn = -1;
    if (motion) sendCommand("STOP", 0);
    uiDirty = true;
  }
}

// ═══ setup / loop ═════════════════════════════════════════════
void setup() {
  Serial.begin(USB_BAUD);                       // ← المتصفح (Web Serial)
  loraSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  pinMode(TFT_BL_PIN, OUTPUT);
  analogWrite(TFT_BL_PIN, (255 * TFT_BACKLIGHT_PCT) / 100);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(COL_BG);

  touchSpi.begin(TOUCH_CLK_PIN, TOUCH_MISO_PIN, TOUCH_MOSI_PIN, TOUCH_CS_PIN);
  ts.begin(touchSpi);
  ts.setRotation(0);

  layout();
  redraw();
}

void loop() {
  pump(Serial, usbBuf, sizeof(usbBuf), usbLen, handleUsbLine);
  pump(loraSerial, loraBuf, sizeof(loraBuf), loraLen, handleLoraLine);

  // انقضاء صمت USB ⇒ العودة إلى الوضع المستقل (الكشف بالحركة)
  if (bridgeMode && (millis() - lastUsbFrameMs) > BRIDGE_IDLE_MS) {
    bridgeMode = false;
    uiDirty = true;
  }

  // ⚠ اللمس يعمل في **الوضعين**: زرّ الطوارئ على الجهاز يجب ألّا يتعطّل
  //   لأن حاسوباً موصول — وهو أضمن مسار حين يتجمّد المتصفح.
  handleTouch();

  uint32_t now = millis();
  if (uiDirty || (now - lastUiMs) >= UI_UPDATE_MS) {
    lastUiMs = now;
    uiDirty = false;
    redraw();
  }
}
