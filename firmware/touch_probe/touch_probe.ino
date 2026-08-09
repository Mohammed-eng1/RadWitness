/*
 * touch_probe.ino -- XPT2046 touch-panel probe for the CYD display
 * ==========================================================================
 * PURPOSE: isolate the touch controller from EVERYTHING else. No TFT, no
 * radio, no protocol -- just the XPT2046 on its own SPI bus and the USB
 * serial monitor. It answers one question per run:
 *
 *   "does the touch chip see my finger, and with what raw numbers?"
 *
 * Why a separate sketch: in controller_display.ino a dead touch panel and a
 * mis-calibrated one look identical (buttons never light up), yet the fixes
 * are completely different -- one is wiring/hardware, the other is three
 * numbers in config.h. This probe tells them apart in ten seconds.
 *
 * It reports, continuously:
 *   - RAW z (pressure), x, y straight from the chip -- printed even when the
 *     library's touched() says false, which is the whole point: a panel that
 *     works but reads z=25 with a threshold of 40 looks "dead" and is not.
 *   - touched()  : the library's own verdict (polling mode, no IRQ)
 *   - IRQ pin    : digital level of TOUCH_IRQ_PIN. controller_display builds
 *                  XPT2046_Touchscreen WITH the IRQ pin, so a dead/miswired
 *                  IRQ line makes touched() never fire there even though the
 *                  chip itself is perfectly fine.
 *   - running min/max of x and y -> the calibration numbers to paste back
 *     into config.h (TOUCH_RAW_MIN / TOUCH_RAW_MAX).
 *
 * HOW TO USE
 *   1. Upload, open Serial Monitor at 115200.
 *   2. Press and hold anywhere. Watch the RAW line.
 *   3. Then press each of the four screen corners in turn, holding ~1s each.
 *   4. Read the SUMMARY block it prints every 5 seconds.
 *
 * READING THE RESULT
 *   a) RAW z stays 0 and x/y never move  -> the chip is not responding:
 *      wiring or a wrong pin in config.h (CLK/MISO/MOSI/CS), or a dead panel.
 *   b) RAW z moves (say 15..90) but touched()=NO -> the panel WORKS; the
 *      pressure threshold is too high. Lower TOUCH_PRESSURE_TH in config.h
 *      below the z you actually measure while pressing.
 *   c) RAW is fine and touched()=YES here, but buttons still dead in
 *      controller_display -> the IRQ line. Check the "IRQ" column: if it
 *      never goes LOW while pressing, drop the IRQ argument from the
 *      XPT2046_Touchscreen constructor there (polling mode) or fix the pin.
 *   d) RAW works but corner min/max differ a lot from TOUCH_RAW_MIN/MAX
 *      (200/3700) -> pure calibration: paste the measured numbers.
 *
 * Build: board "ESP32 Dev Module", library XPT2046_Touchscreen. No TFT_eSPI.
 * ASCII only in this file (see firmware/controller_display/README.md).
 */
#include <Arduino.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>

// Same pins as controller_display/config.h (CYD family, shared across models)
#define TOUCH_CLK_PIN   25
#define TOUCH_MOSI_PIN  32
#define TOUCH_MISO_PIN  39
#define TOUCH_CS_PIN    33
#define TOUCH_IRQ_PIN   36
#define USB_BAUD        115200

// Values currently compiled into the firmware -- printed for comparison
#define CFG_RAW_MIN     200
#define CFG_RAW_MAX     3700
#define CFG_PRESSURE_TH 40

SPIClass touchSpi(VSPI);
// NOTE: constructed WITHOUT the IRQ pin on purpose -> polling mode. This
// asks the chip directly instead of trusting the interrupt line, so a broken
// IRQ cannot hide a working panel. The IRQ pin is read separately below.
XPT2046_Touchscreen ts(TOUCH_CS_PIN);

uint16_t minX = 0xFFFF, maxX = 0, minY = 0xFFFF, maxY = 0;
uint16_t maxZ = 0;
uint32_t samples = 0, touches = 0, irqLows = 0, rails = 0;
uint32_t lastSummaryMs = 0;
uint16_t lastZ = 0xFFFF;

void setup() {
  Serial.begin(USB_BAUD);
  pinMode(TOUCH_IRQ_PIN, INPUT);
  touchSpi.begin(TOUCH_CLK_PIN, TOUCH_MISO_PIN, TOUCH_MOSI_PIN, TOUCH_CS_PIN);
  ts.begin(touchSpi);
  ts.setRotation(0);
  delay(300);
  Serial.println();
  Serial.println("=== touch_probe: XPT2046 raw diagnostic ===");
  Serial.printf("pins  clk=%d miso=%d mosi=%d cs=%d irq=%d\n",
                TOUCH_CLK_PIN, TOUCH_MISO_PIN, TOUCH_MOSI_PIN,
                TOUCH_CS_PIN, TOUCH_IRQ_PIN);
  Serial.printf("config in firmware: RAW_MIN=%d RAW_MAX=%d PRESSURE_TH=%d\n",
                CFG_RAW_MIN, CFG_RAW_MAX, CFG_PRESSURE_TH);
  Serial.println("Press and hold the screen. Then press each corner ~1s.");
  Serial.println("A SUMMARY block prints every 5 seconds.");
  Serial.println();
}

void loop() {
  TS_Point p = ts.getPoint();     // raw read, regardless of touched()
  bool lib = ts.touched();        // library verdict (polling mode)
  int irq = digitalRead(TOUCH_IRQ_PIN);

  samples++;
  if (lib) touches++;
  if (irq == LOW) irqLows++;
  // Rail signature: the XPT2046 is 12-bit, so anything above 4095 -- or a
  // negative x, or z pinned at exactly 4095 -- is not a measurement at all.
  // It is a floating MISO read as all ones, i.e. the chip is not answering
  // on this SPI bus. Counted separately so the summary can say so plainly
  // instead of "calibrating" garbage. (Measured on hardware 2026-08-09.)
  bool rail = (p.z >= 4095) || (p.y > 4095) || (p.x > 4095) || (p.x < 0);
  if (rail) rails++;

  if (p.z > 0) {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
    if (p.z > maxZ) maxZ = p.z;
  }

  // Print on meaningful change only -- a flooded monitor hides the signal
  if (p.z != lastZ && (p.z > 0 || lastZ > 0)) {
    lastZ = p.z;
    Serial.printf("RAW z=%4d x=%4d y=%4d | touched()=%-3s | IRQ=%s%s\n",
                  p.z, p.x, p.y, lib ? "YES" : "NO",
                  irq == LOW ? "LOW " : "HIGH",
                  (p.z > 0 && p.z < CFG_PRESSURE_TH)
                      ? "  <-- below PRESSURE_TH, firmware would ignore it" : "");
  }

  uint32_t now = millis();
  if (now - lastSummaryMs >= 5000) {
    lastSummaryMs = now;
    Serial.println("--- SUMMARY (last 5s window is cumulative) ---");
    Serial.printf("  samples=%lu  touched()=%lu  IRQ_low=%lu\n",
                  (unsigned long)samples, (unsigned long)touches,
                  (unsigned long)irqLows);
    if (rails > samples / 2) {
      Serial.println("  *** SPI NOT RESPONDING -- these are not measurements. ***");
      Serial.println("  Values are stuck at the digital rails (z=4095, y=8191,");
      Serial.println("  x negative): MISO is floating and reads all ones, so the");
      Serial.println("  XPT2046 is not on the bus these pins describe.");
      if (irqLows > 0)
        Serial.println("  BUT the IRQ line pulses on your presses -> the panel and");
      else
        Serial.println("  The IRQ line never pulsed either -> also check the panel and");
      Serial.println("  the chip are ALIVE; only the data path is wrong.");
      Serial.println("  On the 3.5\" board (3248S035R) the touch chip shares the");
      Serial.println("  DISPLAY SPI bus (sck 14, miso 12, mosi 13, cs 33) and must be");
      Serial.println("  read via TFT_eSPI tft.getTouch() -- pins 25/32/39 are the");
      Serial.println("  2.8\" board only. Ignore any calibration numbers below.");
    }
    if (maxZ == 0) {
      Serial.println("  z never rose above 0 -> chip not responding.");
      Serial.println("  Check touch wiring / pins (clk,miso,mosi,cs), then the panel.");
    } else if (rails <= samples / 2) {
      Serial.printf("  peak z=%d   x range=%d..%d   y range=%d..%d\n",
                    maxZ, minX, maxX, minY, maxY);
      if (maxZ < CFG_PRESSURE_TH)
        Serial.printf("  peak z is BELOW TOUCH_PRESSURE_TH (%d): panel works,"
                      " threshold too high.\n", CFG_PRESSURE_TH);
      if (touches == 0)
        Serial.println("  library touched() never fired -> see the z note above.");
      if (irqLows == 0)
        Serial.println("  IRQ never went LOW -> IRQ line suspect; polling mode"
                       " is the workaround.");
      Serial.println("  After pressing ALL FOUR corners, paste these into config.h:");
      Serial.printf("    #define TOUCH_RAW_MIN  %d\n", (minX < minY ? minX : minY));
      Serial.printf("    #define TOUCH_RAW_MAX  %d\n", (maxX > maxY ? maxX : maxY));
      Serial.printf("    #define TOUCH_PRESSURE_TH %d   (about a third of peak z)\n",
                    maxZ / 3 > 5 ? maxZ / 3 : 5);
    }
    Serial.println();
  }

  delay(50);
}
