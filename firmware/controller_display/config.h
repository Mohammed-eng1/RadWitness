/*
 * config.h -- CYD display firmware (LoRa controller)
 * ==================================================================
 * ALL pins and constants live here exclusively (same rule as
 * pi/config.py on the Pi).
 *
 * The pins are CARRIED OVER FROM THE PROVEN LEGACY FIRMWARE on these
 * exact boards: legacy/RadiationRover/firmware/controller_display/
 * config.h. No new numbers were invented -- these actually worked on
 * this hardware.
 *
 * Model select: DISPLAY_MODEL
 *   1 = ESP32-3248S035R: ST7796 320x480, XPT2046 touch, backlight IO27
 *   2 = ESP32-2432S028 : ILI9341 240x320, backlight IO21
 * The matching TFT_eSPI setup file is in legacy/.../tft_setup/.
 *
 * ASCII only in this file -- see the BiDi note in controller_display.ino.
 */
#pragma once

#define DISPLAY_MODEL 1        // 1 = 3248S035R (default) | 2 = 2432S028

// === Pins per model (proven) ==================================
#if DISPLAY_MODEL == 1
  #define TFT_BL_PIN     27    // backlight ONLY -- PWM
  #define LORA_RX_PIN    21    // <- HC-14 TX
  #define LORA_TX_PIN    22    // -> HC-14 RX
#else
  #define TFT_BL_PIN     21
  #define LORA_RX_PIN    35
  #define LORA_TX_PIN    27
#endif

// MUST match LORA_BAUD in pi/config.py -- a mismatch yields endless
// corrupt frames and is the most common cause of "the radio is dead".
#define LORA_BAUD       9600

// === Touch -- THE PATH DIFFERS BY MODEL, this is not a shared pinout =====
// Model 1 (3248S035R, 3.5"): the XPT2046 sits on the DISPLAY SPI bus
//   (sck 14, miso 12, mosi 13) with CS 33, and is read through TFT_eSPI's
//   own tft.getTouch(). TOUCH_CS is declared in the TFT_eSPI Setup file, and
//   calibration is stored in NVS. The four pins below are NOT used here.
// Model 2 (2432S028, 2.8"): raw XPT2046 on its own SPI bus -- the pins below.
//
// Getting this wrong is a silent, misleading failure: driving model 1 down
// the model-2 path talks to pins nothing is attached to, so MISO floats and
// reads all ones -- z=4095, y=8191, x=-4096, touched() true on every single
// sample, while the IRQ line still pulses correctly on every press. Buttons
// simply never respond, looking exactly like a dead panel. Measured on
// hardware 2026-08-09 with firmware/touch_probe.
#define TOUCH_CLK_PIN  25      // model 2 only
#define TOUCH_MOSI_PIN 32      // model 2 only
#define TOUCH_MISO_PIN 39      // model 2 only
#define TOUCH_CS_PIN   33      // model 2 only (model 1 sets TOUCH_CS in Setup)
#define TOUCH_IRQ_PIN  36      // model 2 only (the one pin common to both)
#define TOUCH_RAW_MIN  200     // model 2 only -- model 1 uses NVS calibration
#define TOUCH_RAW_MAX  3700    // model 2 only
#define TOUCH_PRESSURE_TH 40   // both: above = real touch (idle noise ~10-17)
#define TOUCH_SWAP_XY  0       // model 2 only -- model 1 orientation comes
#define TOUCH_INVERT_X 0       //   from the calibration itself
#define TOUCH_INVERT_Y 0
// Model 1: set to 1 for ONE boot to redo the four-corner calibration
// (it is stored in NVS and reloaded automatically), then set it back to 0.
#define TOUCH_FORCE_CALIBRATE 0
// Stamped next to the stored calibration. A calibration is only reused when
// this value matches, so a blob recorded under a different rotation or button
// layout is discarded and redone instead of silently mis-mapping every press.
// BUMP THIS whenever setRotation() or the layout changes.
#define TOUCH_CAL_TAG 0x43594431UL   // 'CYD1' -- rotation 0, 3x3 button grid
// Model 1: print raw + converted touch values over USB while STANDALONE.
// Leave at 1 while proving the panel; set to 0 once buttons respond.
#define TOUCH_DEBUG 1

// === Protocol (must match pi/comms/protocol.py) ===============
#define MAX_PAYLOAD     56     // = LORA_MAX_PAYLOAD
#define SEQ_MODULO      1000   // = LORA_SEQ_MODULO

// === Timing ===================================================
#define TFT_BACKLIGHT_PCT   60
#define UI_UPDATE_MS        250    // redraw changed fields only
// Held-button command refresh: MUST be shorter than the radio command
// timeout on the Pi (2000ms in pi/config.py). 500ms means losing two
// consecutive frames still does not cut the motion off.
#define DRIVE_REPEAT_MS     500
// No telemetry for this long => the link is declared dead (the robot
// broadcasts every 2s).
#define LORA_TIMEOUT_MS     8000

// === Bridge-mode detection ====================================
// WARNING: classic ESP32 CANNOT detect a USB cable electrically.
// These boards use a CH340/CP2102 adapter on UART0 and there is no
// native USB stack (unlike ESP32-S2/S3) -- no "attached" event, no
// readable DTR line. And `Serial` evaluates true ALWAYS on a plain
// UART, so using it as "is USB plugged in?" gives permanent bridge
// mode and kills standalone mode entirely.
// => Detect by TRAFFIC, not electricity: the first valid frame on
//    UART0 enters bridge mode; BRIDGE_IDLE_MS of silence returns to
//    standalone. This is also the semantically right question:
//    "is a computer driving me right now?" not "is a cable attached?"
#define BRIDGE_IDLE_MS      5000
#define USB_BAUD            115200 // browser <-> display baud (Web Serial)

// === UI colors (dark -- matches the web dashboard) ============
#define COL_BG      0x0861     // #0b0f17
#define COL_PANEL   0x18C3     // #131a26
#define COL_LINE    0x2247     // #243149
#define COL_FG      0xE71C     // #e6ecf7
#define COL_DIM     0x8410     // #8a93a6
#define COL_OK      0x1DE6     // #22c55e
#define COL_WARN    0xFB40     // #f59e0b
#define COL_BAD     0xE9A6     // #ef4444
#define COL_ACCENT  0x3C1F     // #3b82f6
