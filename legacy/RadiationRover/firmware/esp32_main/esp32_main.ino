/*
 * esp32_main.ino — Mars Rover Control Hub
 * ========================================
 * يعمل على: ESP32-S3 DevKitC N16R8 (فلاش 16MB + PSRAM 8MB)
 * الترجمة: لوحة "ESP32S3 Dev Module"، فلاش 16MB، قسم تطبيق ≥3MB، PSRAM: OPI
 *
 * ★ هذا هو الهَب — العقل المركزي ★
 *
 * يستضيف:
 *   - الواجهة على بورت 80
 *   - WebSocket /ws للأوامر و GPS (تأخير <50ms)
 *   - mDNS: http://rover.local
 *
 * يتصل بـ:
 *   - GPS module (NEO-6M/7M/8M) عبر Serial1 → يستقبل NMEA
 *   - ESP32-CAM عبر UDP/WiFi (بورت 4210) → يمرّر الأوامر
 *     (الـ ESP32-CAM موصولة فيزيائياً بالأردوينو على البورد)
 *
 * ═══════════════════════════════════════════════════════════
 *  التوصيلات
 * ═══════════════════════════════════════════════════════════
 *
 *   الخريطة الكاملة في docs/wiring.md (S3: جيجر=4، I2C=21/9،
 *   GPS RX=16، SD=12/11/13/10) — مثبتة من السكتش المرجعي.
 *
 *   الهَب: يحصل على الطاقة من البطارية (VIN=5V) أو USB.
 *
 *   ⚠ لا حاجة لتوصيل سلكي بين ESP32 الجديد والروبوت —
 *     التواصل عبر WiFi مع الـ ESP32-CAM.
 *
 * ═══════════════════════════════════════════════════════════
 *  المكتبات المطلوبة
 * ═══════════════════════════════════════════════════════════
 *   1. ESPAsyncWebServer  (by lacamera)
 *   2. AsyncTCP           (by me-no-dev)
 *   3. TinyGPSPlus        (by Mikal Hart)
 *   (WiFi, WiFiUdp, ESPmDNS, LittleFS مضمّنات)
 *   مع USE_REAL_IMU: Adafruit_BNO055 + Adafruit_Sensor
 *   مع USE_REAL_SD:  SD (مضمّنة)
 *
 *  البنية: كل حساس خلف واجهة HAL في ملف .h (وهمي/حقيقي حسب
 *  أعلام config.h) + طبقة ذكية محلية: anomaly / risk /
 *  source_locator / mission_report — كلها non-blocking.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>    // تحليل خطة المهمة القادمة من الشات بوت

#include "secrets.h"        // WIFI_SSID / WIFI_PASSWORD — خارج نطاق git
#include "config.h"         // كل المنافذ والثوابت والعتبات هنا حصراً

// وحدات الحساسات (HAL وهمي/حقيقي) والطبقة الذكية — الترتيب مهم:
// imu ← gps (يحتاج heading) ← geiger (يحتاج الموقع والاتجاه)
#include "imu.h"
#include "gps.h"
#include "geiger.h"
#include "obstacles.h"
#include "sd_logger.h"
#include "anomaly.h"
#include "risk.h"
#include "failsafe.h"
#include "source_locator.h"
#include "mission_report.h"

// ── Globals ─────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

WiFiUDP udp;
IPAddress camIP;
bool camResolved = false;
bool camViaUdp = false;          // تعرّفنا على IP الكام من حزمة UDP (BV:/OBS:) — لا نحتاج mDNS الحاجز بعدها
unsigned long lastCamCheck = 0;
unsigned long lastCamRxMs = 0;   // آخر حزمة UDP وصلت من الكام (تشخيص الاتصال)

// ── مزلاج الأمر اليدوي (يكرّره الهَب بنفسه — لا يعتمد على مؤقّت المتصفح) ──
//   مخزَّن كمصفوفة char ثابتة: يُكتب من مهمة WebSocket (AsyncTCP) ويُقرأ من loop —
//   المصفوفة الثابتة لا يُعاد تخصيصها فلا خطر تعطّل (بخلاف String عبر المهام).
char             manualCmd[8] = "";     // آخر أمر حركة يدوي (F/B/L/R + قوة)
volatile bool    manualActive = false;
volatile unsigned long manualLastRxMs = 0;  // آخر وصول للأمر (متصفح/لورا)
unsigned long    manualLastFwdMs = 0;   // آخر تكرار للكام (يُقرأ/يُكتب في loop فقط)

WiFiUDP teleUdp;                  // استقبال telemetry (البطارية) من CAM
float batteryV = 0.0;
unsigned long lastBatteryTime = 0;

unsigned long lastGPSBroadcast = 0;
unsigned long lastTeleBroadcast = 0;
unsigned long lastRadBroadcast = 0;
unsigned long lastLogWrite = 0;

// ── لورا HC-14 (UART2): تحكم يدوي أولوية قصوى + بث حالة مضغوطة ──
HardwareSerial loraSerial(2);
String loraRxBuf;
unsigned long lastLoraRxMs = 0;      // آخر أمر LoRa (لمؤشر القناة)
unsigned long lastLoraTxMs = 0;

// ── HTML/JS Page ────────────────────────────────────────────
const char HTML_PAGE[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="ar" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Mars Rover Control</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" crossorigin=""/>
<style>
:root{
  --bg:#05070f; --bg-2:#0a0e1a;
  --panel:#0e1422; --panel-2:#151d30;
  --border:#212c49; --border-2:#1a2338;
  --text:#eaeef7; --muted:#8493ae;
  --mars:#f2592d; --mars-glow:#ff8158;
  --cyan:#2dd4ee; --good:#12c583;
  --bad:#f5484a; --warn:#f9a825;
  --radius:16px;
}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
html,body{height:100%}
body{
  background:var(--bg); color:var(--text);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","Tajawal",sans-serif;
  min-height:100vh; line-height:1.4;
  background-image:
    radial-gradient(1200px 600px at 12% -5%, rgba(242,89,45,.10), transparent 60%),
    radial-gradient(1000px 500px at 100% 0%, rgba(45,212,238,.06), transparent 55%),
    linear-gradient(180deg, var(--bg), var(--bg-2));
  background-attachment:fixed;
}
.appbar{position:sticky;top:0;z-index:20;background:rgba(8,11,20,.82);backdrop-filter:blur(14px) saturate(1.2);border-bottom:1px solid var(--border)}
.topbar{display:flex;justify-content:space-between;align-items:center;gap:12px;padding:11px 18px;border-bottom:1px solid var(--border-2)}
.brand{display:flex;align-items:center;gap:10px;font-weight:700;letter-spacing:.14em;font-size:.92rem;white-space:nowrap}
.brand .logo{font-size:1.35rem;filter:drop-shadow(0 0 10px var(--mars-glow))}
.brand .ver{font-size:.6rem;color:var(--mars);background:rgba(242,89,45,.14);border:1px solid rgba(242,89,45,.25);padding:2px 7px;border-radius:6px;letter-spacing:.06em}
.cluster{display:flex;gap:6px;flex-wrap:wrap;justify-content:flex-end}
.pill{font-size:.68rem;padding:5px 10px;border-radius:999px;background:rgba(255,255,255,.03);border:1px solid var(--border);display:flex;align-items:center;gap:6px;color:var(--muted);transition:.2s;white-space:nowrap}
.pill .dot{width:6px;height:6px;border-radius:50%;background:var(--muted);transition:.2s}
.pill.ok{color:var(--good);border-color:rgba(18,197,131,.35);background:rgba(18,197,131,.06)}
.pill.ok .dot{background:var(--good);box-shadow:0 0 8px var(--good)}
.pill.bad{color:var(--bad);border-color:rgba(245,72,74,.4);background:rgba(245,72,74,.06)}
.pill.bad .dot{background:var(--bad);box-shadow:0 0 8px var(--bad)}
.pill.warn{color:var(--warn);border-color:rgba(249,168,37,.38);background:rgba(249,168,37,.06)}
.pill.warn .dot{background:var(--warn);box-shadow:0 0 6px var(--warn)}
.pill .batt-icon{font-size:.85rem;line-height:1}
#batt-pill{font-family:ui-monospace,monospace;font-weight:600}
#batt-pill.full{color:var(--good);border-color:rgba(18,197,131,.35)}
#batt-pill.mid{color:var(--warn);border-color:rgba(249,168,37,.38)}
#batt-pill.low{color:var(--bad);border-color:rgba(245,72,74,.45);animation:battpulse 1.5s infinite}
@keyframes battpulse{0%,100%{opacity:1}50%{opacity:.45}}
#imu-pill #imu-txt{font-family:ui-monospace,monospace;letter-spacing:.06em}
#geiger-pill{font-family:ui-monospace,monospace}
.tabbar{display:flex;gap:6px;padding:10px 14px;overflow-x:auto;scrollbar-width:none}
.tabbar::-webkit-scrollbar{display:none}
.tab{flex:0 0 auto;display:flex;align-items:center;gap:7px;font-family:inherit;cursor:pointer;font-size:.82rem;font-weight:600;color:var(--muted);background:transparent;border:1px solid transparent;padding:9px 15px;border-radius:11px;transition:.18s;letter-spacing:.02em;white-space:nowrap}
.tab:hover{color:var(--text);background:rgba(255,255,255,.03)}
.tab.active{color:#fff;background:linear-gradient(180deg,rgba(242,89,45,.22),rgba(242,89,45,.08));border-color:rgba(242,89,45,.5);box-shadow:0 4px 18px -6px rgba(242,89,45,.5),inset 0 1px 0 rgba(255,255,255,.06)}
.tab .ico{font-size:1rem}
#alarm-banner{display:none;margin:12px 16px 0;padding:11px 16px;border-radius:12px;background:rgba(245,72,74,.14);border:1px solid var(--bad);color:var(--bad);font-size:.85rem;font-weight:700;text-align:center;letter-spacing:.02em;animation:battpulse 1s infinite}
#alarm-banner.show{display:block}
.content{max-width:1180px;margin:0 auto;padding:16px}
.tabpage{display:none;animation:fade .28s ease}
.tabpage.active{display:block}
#tab-drive.active{display:grid;grid-template-columns:1.55fr 1fr;gap:16px;align-items:start}
@media(max-width:860px){#tab-drive.active{grid-template-columns:1fr}}
#tab-rad.active{max-width:780px;margin:0 auto}
#tab-ai.active,#tab-diag.active{max-width:860px;margin:0 auto}
@keyframes fade{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:none}}
.panel{position:relative;background:linear-gradient(180deg,var(--panel),rgba(14,20,34,.6));border:1px solid var(--border);border-radius:var(--radius);padding:16px;box-shadow:0 12px 30px -18px rgba(0,0,0,.9)}
.panel+.panel{margin-top:16px}
.panel::before{content:"";position:absolute;inset:0 0 auto 0;height:1px;border-radius:var(--radius) var(--radius) 0 0;background:linear-gradient(90deg,transparent,rgba(255,255,255,.10),transparent)}
.panel h2{font-size:.68rem;letter-spacing:.2em;color:var(--muted);text-transform:uppercase;margin-bottom:12px;font-weight:600;display:flex;align-items:center;gap:8px}
.panel h2::before{content:"";width:3px;height:13px;background:linear-gradient(var(--mars),var(--mars-glow));border-radius:2px;box-shadow:0 0 8px var(--mars-glow)}
.cam-wrap{position:relative;aspect-ratio:4/3;background:#000;border-radius:12px;overflow:hidden;border:1px solid var(--border)}
.cam-wrap img{width:100%;height:100%;object-fit:cover;display:block}
.hud{position:absolute;top:9px;left:9px;background:rgba(0,0,0,.6);padding:4px 10px;border-radius:7px;font-size:.68rem;font-family:ui-monospace,monospace;color:var(--mars);border:1px solid rgba(242,89,45,.35);backdrop-filter:blur(4px)}
.cam-fail{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;flex-direction:column;gap:10px;color:var(--muted);font-size:.85rem;background:rgba(0,0,0,.86)}
.cam-fail.hidden{display:none}
.cam-fail button{background:var(--mars);color:#fff;border:none;padding:7px 18px;border-radius:8px;font-size:.8rem;cursor:pointer;font-family:inherit}
.cam-controls{display:flex;justify-content:space-between;align-items:center;margin-top:12px;gap:10px;flex-wrap:wrap}
.cam-modes{display:flex;gap:6px}
.cam-mode-btn{background:var(--panel-2);border:1px solid var(--border);color:var(--muted);padding:7px 13px;border-radius:9px;font-size:.72rem;cursor:pointer;transition:.15s;font-family:inherit}
.cam-mode-btn.active{background:rgba(242,89,45,.16);border-color:var(--mars);color:var(--mars)}
.snap-speed-row{display:flex;align-items:center;gap:5px;font-size:.7rem;color:var(--muted);flex-wrap:wrap}
.snap-btn{background:var(--panel-2);border:1px solid var(--border);color:var(--muted);padding:6px 10px;border-radius:8px;font-size:.7rem;cursor:pointer;font-family:inherit;transition:.15s}
.snap-btn:active,.snap-btn.active{background:rgba(45,212,238,.14);border-color:var(--cyan);color:var(--cyan)}
.speed-row{display:flex;align-items:center;gap:12px;margin-bottom:16px;font-size:.8rem;color:var(--muted)}
.speed-row input{flex:1;accent-color:var(--mars)}
#spd-lbl{font-family:ui-monospace,monospace;color:var(--mars);min-width:34px;text-align:end;font-weight:700;font-size:1rem}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;max-width:290px;margin:0 auto}
.empty{visibility:hidden}
.btn{aspect-ratio:1;background:linear-gradient(180deg,var(--panel-2),rgba(21,29,48,.5));border:1px solid var(--border);color:var(--text);border-radius:14px;font-size:1.5rem;cursor:pointer;user-select:none;touch-action:none;display:flex;align-items:center;justify-content:center;transition:.08s;font-family:ui-monospace,monospace}
.btn:active,.btn.active{background:rgba(242,89,45,.2);border-color:var(--mars);color:var(--mars);transform:scale(.94);box-shadow:0 0 16px rgba(242,89,45,.3)}
.btn.stop{background:rgba(245,72,74,.08);border-color:rgba(245,72,74,.3);color:var(--bad)}
.btn.stop:active{background:rgba(245,72,74,.26);border-color:var(--bad);box-shadow:0 0 16px rgba(245,72,74,.35)}
.hint{text-align:center;font-size:.7rem;color:var(--muted);margin-top:12px;letter-spacing:.04em}
#map{width:100%;aspect-ratio:16/10;border-radius:12px;background:var(--bg-2);border:1px solid var(--border)}
@media(max-width:860px){#map{aspect-ratio:4/3}}
.geo-row{display:flex;align-items:center;gap:8px;margin-top:12px;flex-wrap:wrap;font-size:.72rem;color:var(--muted)}
.geo-row input[type=range]{flex:1;min-width:90px;accent-color:var(--mars)}
#geo-btn.on{background:rgba(18,197,131,.16);border-color:var(--good);color:var(--good)}
#geo-state{color:var(--warn);font-weight:600}
.map-legend{display:flex;gap:16px;align-items:center;margin-top:12px;font-size:.68rem;color:var(--muted);flex-wrap:wrap}
.map-legend span{display:flex;align-items:center;gap:6px}
.map-legend i{width:16px;height:3px;border-radius:2px;display:inline-block}
.leg-src{background:var(--cyan);box-shadow:0 0 6px var(--cyan)}
.leg-grad{background:var(--warn)}
.leg-heat{width:10px;height:10px;border-radius:50%;background:conic-gradient(var(--good),var(--warn),var(--bad),var(--good))}
.src-arrowhead{width:0;height:0;border-left:6px solid transparent;border-right:6px solid transparent;border-bottom:12px solid var(--cyan);filter:drop-shadow(0 0 4px var(--cyan));transform-origin:50% 60%}
.leaflet-container{background:var(--bg-2)!important;font-family:inherit!important}
.leaflet-control-attribution{background:rgba(0,0,0,.5)!important;color:var(--muted)!important;font-size:.6rem!important}
.leaflet-control-attribution a{color:var(--cyan)!important}
.leaflet-control-zoom a{background:var(--panel)!important;color:var(--text)!important;border:1px solid var(--border)!important}
.rover-marker{background:transparent;border:none}
.rov-arrow{width:0;height:0;border-left:7px solid transparent;border-right:7px solid transparent;border-bottom:16px solid var(--mars);filter:drop-shadow(0 0 5px var(--mars-glow)) drop-shadow(0 0 2px #000);transform-origin:50% 65%;transition:transform .3s ease-out}
.dose-hero{display:flex;flex-direction:column;align-items:center;gap:14px;margin-bottom:16px;padding:16px 10px 6px;background:radial-gradient(ellipse at 50% 0%,rgba(242,89,45,.07),transparent 70%);border-radius:14px}
.gauge-wrap{position:relative;width:190px;height:190px}
.gauge{width:100%;height:100%;overflow:visible}
.gauge .track{fill:none;stroke:var(--panel-2);stroke-width:13;stroke-linecap:round}
.gauge .fill{fill:none;stroke-width:13;stroke-linecap:round;transition:stroke-dashoffset .6s cubic-bezier(.4,0,.2,1),stroke .4s;filter:drop-shadow(0 0 8px currentColor)}
.gauge-center{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:1px}
.gauge-val{font-size:2.2rem;font-weight:800;font-family:ui-monospace,monospace;line-height:1;transition:color .4s}
.gauge-unit{font-size:.58rem;color:var(--muted);letter-spacing:.15em;text-transform:uppercase}
.gauge-risk{margin-top:8px;font-size:.72rem;font-weight:700;letter-spacing:.06em;padding:3px 14px;border-radius:999px;border:1px solid currentColor;transition:color .4s}
.gauge-cpm{margin-top:6px;font-size:.66rem;color:var(--muted);font-family:ui-monospace,monospace}
.gauge-cpm b{color:var(--text)}
.lvlbar{display:flex;width:100%;gap:5px}
.lvl-seg{flex:1;text-align:center;font-size:.56rem;letter-spacing:.02em;padding:7px 2px;border-radius:8px;background:var(--panel-2);border:1px solid var(--border);color:var(--muted);transition:.35s;text-transform:uppercase;font-weight:700}
.lvl-seg.on{color:#fff;border-color:currentColor;box-shadow:0 0 14px -2px currentColor,inset 0 0 22px -12px currentColor}
.cpm-chart{width:100%;height:100px;display:block;background:linear-gradient(180deg,#0b1120,#080c16);border:1px solid var(--border);border-radius:12px}
.chart-cap{display:flex;justify-content:space-between;width:100%;font-size:.6rem;color:var(--muted);letter-spacing:.05em;margin-top:-6px}
.chart-cap b{color:var(--cyan);font-family:ui-monospace,monospace}
.tele-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:9px}
@media(min-width:560px){.tele-grid{grid-template-columns:repeat(3,1fr)}}
.tele{background:linear-gradient(180deg,var(--panel-2),rgba(21,29,48,.4));padding:11px 12px;border-radius:11px;border:1px solid var(--border-2)}
.tele .lbl{display:block;font-size:.62rem;color:var(--muted);text-transform:uppercase;letter-spacing:.07em;margin-bottom:5px}
.tele .val{font-size:1.02rem;font-weight:600;font-family:ui-monospace,monospace;color:var(--text)}
.tele .val em{font-style:normal;font-size:.68rem;color:var(--muted);margin-inline-start:3px}
.tele.dim .val{color:var(--muted);font-style:italic;font-size:.82rem}
.tele.crit{border-color:rgba(245,72,74,.5);background:rgba(245,72,74,.06)}
.tele.crit .val{color:var(--bad)}
.ai-cfg{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px}
.ai-cfg input{background:var(--panel-2);border:1px solid var(--border);color:var(--text);border-radius:9px;padding:9px 12px;font-size:.78rem;font-family:inherit}
.ai-cfg input#gem-key{flex:2;min-width:180px}.ai-cfg input#gem-model{flex:1;min-width:130px}
#ai-log{background:var(--bg-2);border:1px solid var(--border);border-radius:12px;padding:12px;height:280px;overflow:auto;display:flex;flex-direction:column;gap:9px;margin-bottom:10px}
.ai-msg{max-width:85%;padding:9px 13px;border-radius:12px;font-size:.83rem;line-height:1.55;white-space:pre-wrap}
.ai-msg.u{align-self:flex-start;background:var(--panel-2);border:1px solid var(--border)}
.ai-msg.a{align-self:flex-end;background:rgba(45,212,238,.09);border:1px solid rgba(45,212,238,.32)}
.ai-msg.sys{align-self:center;color:var(--muted);font-size:.72rem;font-style:italic}
.ai-input-row{display:flex;gap:8px}
.ai-input-row textarea{flex:1;background:var(--panel-2);border:1px solid var(--border);color:var(--text);border-radius:10px;padding:11px;font-size:.83rem;font-family:inherit;resize:none;min-height:46px}
.ai-input-row button,.ai-plan-btns button{background:var(--mars);color:#fff;border:none;border-radius:10px;padding:0 18px;font-size:.82rem;font-weight:600;cursor:pointer;font-family:inherit}
.ai-input-row button:disabled{opacity:.5;cursor:default}
#ai-plan{display:none;margin-top:14px;background:var(--panel-2);border:1px solid var(--cyan);border-radius:12px;padding:14px}
#ai-plan.show{display:block}
#ai-plan h3{font-size:.8rem;color:var(--cyan);margin-bottom:8px}
#ai-plan ol{margin:0 18px;font-size:.79rem;line-height:1.7}
.ai-plan-btns{display:flex;gap:8px;margin-top:12px;flex-wrap:wrap}
.ai-plan-btns button{padding:9px 16px}
.ai-plan-btns button.run{background:var(--good)}.ai-plan-btns button.abort{background:var(--bad)}
#ai-plan-status{margin-top:8px;font-size:.78rem;color:var(--warn)}
.diag-grid{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:10px;align-items:center}
.diag-btn{background:var(--panel-2);border:1px solid var(--border);color:var(--text);padding:9px 14px;border-radius:10px;font-size:.78rem;cursor:pointer;font-family:inherit;transition:.12s}
.diag-btn:hover{border-color:var(--cyan);color:var(--cyan)}
.diag-btn:active{background:rgba(45,212,238,.14)}
.diag-btn.danger{border-color:rgba(245,72,74,.4);color:var(--bad)}
.diag-btn.sim{border-color:rgba(249,168,37,.4);color:var(--warn)}
#sim-tools{display:none}
.scanbar{height:8px;background:var(--panel-2);border-radius:5px;overflow:hidden;border:1px solid var(--border);flex:1;min-width:120px}
.scanbar>div{height:100%;width:0;background:linear-gradient(90deg,var(--cyan),var(--mars));transition:width .4s}
#diag-out{margin-top:10px;background:var(--bg-2);border:1px solid var(--border);border-radius:10px;padding:11px;font-family:ui-monospace,monospace;font-size:.7rem;color:var(--text);white-space:pre-wrap;direction:ltr;max-height:300px;overflow:auto;display:none}
#estop-btn{position:fixed;bottom:18px;left:18px;z-index:50;background:var(--bad);color:#fff;border:2px solid rgba(255,255,255,.2);border-radius:14px;padding:14px 22px;font-size:1rem;font-weight:800;cursor:pointer;letter-spacing:.03em;font-family:inherit;box-shadow:0 6px 26px rgba(245,72,74,.5)}
#estop-btn:active{transform:scale(.95)}
#toasts{position:fixed;bottom:18px;right:18px;z-index:60;display:flex;flex-direction:column;gap:8px}
.toast{background:var(--panel);border:1px solid var(--cyan);color:var(--text);padding:10px 16px;border-radius:11px;font-size:.8rem;box-shadow:0 6px 20px rgba(0,0,0,.5);animation:toastin .2s ease-out}
.toast.err{border-color:var(--bad);color:var(--bad)}
@keyframes toastin{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:none}}
</style>
</head>
<body>

<div class="appbar">
<header class="topbar">
  <div class="brand">
    <span class="logo">🚀</span>
    <span>MARS ROVER</span>
    <span class="ver">v2.0</span>
  </div>
  <div class="cluster">
    <div class="pill" id="batt-pill"><span class="batt-icon" id="batt-icon">🔋</span><span id="batt-text">—</span></div>
    <div class="pill" id="rad-pill"><span class="dot"></span><span id="rad-txt">— µSv/h</span></div>
    <div class="pill" id="ws-pill"><span class="dot"></span><span>Link</span></div>
    <div class="pill" id="cam-pill"><span class="dot"></span><span>Cam</span></div>
    <div class="pill" id="gps-pill"><span class="dot"></span><span>GPS</span></div>
    <div class="pill" id="geiger-pill"><span class="dot"></span><span id="geiger-txt">G1</span></div>
    <div class="pill" id="imu-pill"><span class="dot"></span><span id="imu-txt">IMU</span></div>
    <div class="pill" id="sd-pill"><span class="dot"></span><span id="sd-txt">SD</span></div>
    <div class="pill" id="obs-pill"><span class="dot"></span><span id="obs-txt">OBS</span></div>
  </div>
</header>
<nav class="tabbar">
  <button class="tab" data-tab="drive"><span class="ico">🎮</span> قيادة</button>
  <button class="tab" data-tab="rad"><span class="ico">☢️</span> الإشعاع</button>
  <button class="tab" data-tab="map"><span class="ico">🗺️</span> الخريطة</button>
  <button class="tab" data-tab="ai"><span class="ico">🧠</span> الكابتن</button>
  <button class="tab" data-tab="diag"><span class="ico">🛠️</span> التشخيص</button>
</nav>
</div>

<div id="alarm-banner"></div>

<main class="content">
  <div class="tabpage" id="tab-drive">
    <section class="panel cam-pan"><h2>Live Feed</h2>
    <div class="cam-wrap">
      <img id="stream" alt="">
      <div class="hud" id="hud">— fps</div>
      <div class="cam-fail" id="cam-fail">
        <div>الكاميرا غير متصلة</div>
        <button onclick="retryCam()">إعادة المحاولة</button>
      </div>
    </div>
    <div class="cam-controls">
      <div class="cam-modes">
        <button class="cam-mode-btn active" data-mode="snapshot" onclick="setCamMode('snapshot')">📷 صور</button>
        <button class="cam-mode-btn" data-mode="live" onclick="setCamMode('live')">🎥 بث مباشر</button>
      </div>
      <div class="snap-speed-row" id="snap-speed-row">
        <span>السرعة:</span>
        <button class="snap-btn active" onclick="setSnapInterval(400)">سريع</button>
        <button class="snap-btn" onclick="setSnapInterval(700)">متوسط</button>
        <button class="snap-btn" onclick="setSnapInterval(1500)">موفّر</button>
        <button class="snap-btn" onclick="grabSnapshot()">⟳ الآن</button>
      </div>
    </div></section>
    <section class="panel ctl-pan"><h2>Controls</h2>
    <div class="speed-row">
      <span>Power</span>
      <input type="range" id="speed" min="20" max="100" value="70" step="5">
      <span id="spd-lbl">70</span>
    </div>
    <div class="dpad">
      <div class="empty"></div>
      <button class="btn" data-dir="F">▲</button>
      <div class="empty"></div>
      <button class="btn" data-dir="L">◀</button>
      <button class="btn stop" data-dir="S">■</button>
      <button class="btn" data-dir="R">▶</button>
      <div class="empty"></div>
      <button class="btn" data-dir="B">▼</button>
      <div class="empty"></div>
    </div>
    <div class="hint">WASD أو ← → ↑ ↓</div></section>
  </div>
  <div class="tabpage" id="tab-rad">
    <section class="panel tel-pan"><h2>Radiation</h2>
    <div class="dose-hero">
      <div class="gauge-wrap">
        <svg class="gauge" viewBox="0 0 186 186">
          <circle class="track" id="g-track" cx="93" cy="93" r="76" transform="rotate(135 93 93)"></circle>
          <circle class="fill"  id="g-fill"  cx="93" cy="93" r="76" transform="rotate(135 93 93)" style="stroke:var(--good)"></circle>
        </svg>
        <div class="gauge-center">
          <div class="gauge-val" id="g-dose">—</div>
          <div class="gauge-unit">µSv/h</div>
          <div class="gauge-risk" id="g-risk" style="color:var(--muted)">—</div>
          <div class="gauge-cpm">CPM <b id="g-cpm">—</b></div>
        </div>
      </div>
      <div class="lvlbar" id="lvlbar">
        <div class="lvl-seg" data-l="0">Safe</div>
        <div class="lvl-seg" data-l="1">Low</div>
        <div class="lvl-seg" data-l="2">Med</div>
        <div class="lvl-seg" data-l="3">High</div>
        <div class="lvl-seg" data-l="4">Crit</div>
      </div>
      <canvas class="cpm-chart" id="cpm-chart"></canvas>
      <div class="chart-cap"><span>آخر 5 دقائق</span><span>ذروة <b id="cpm-peak">—</b></span></div>
    </div>
    <div class="tele-grid">
      <div class="tele crit" id="tile-dose"><span class="lbl">Dose</span><span class="val" id="t-dose">—<em>µSv/h</em></span></div>
      <div class="tele"><span class="lbl">Risk</span><span class="val" id="t-risk">—</span></div>
      <div class="tele"><span class="lbl">CPM · tube 1</span><span class="val" id="t-cpm">—</span></div>
      <div class="tele" id="tile-cpm2"><span class="lbl">CPM · tube 2</span><span class="val" id="t-cpm2">—</span></div>
      <div class="tele"><span class="lbl">CPM raw</span><span class="val" id="t-cpmraw">—</span></div>
      <div class="tele"><span class="lbl">IMU calib</span><span class="val" id="t-imucal">—<em>s/g/a/m</em></span></div>
      <div class="tele" id="tile-tilt"><span class="lbl">IMU Tilt (P/R)</span><span class="val" id="t-tilt">—<em>°</em></span></div>
    </div>
    <h2 style="margin-top:14px">Telemetry</h2>
    <div class="tele-grid">
      <div class="tele"><span class="lbl">Latitude</span><span class="val" id="t-lat">—</span></div>
      <div class="tele"><span class="lbl">Longitude</span><span class="val" id="t-lng">—</span></div>
      <div class="tele"><span class="lbl">Satellites</span><span class="val" id="t-sat">0</span></div>
      <div class="tele"><span class="lbl">Speed</span><span class="val" id="t-spd">0.0<em>km/h</em></span></div>
      <div class="tele"><span class="lbl">Heading</span><span class="val" id="t-hdg">—<em>°</em></span></div>
      <div class="tele"><span class="lbl">Altitude</span><span class="val" id="t-alt">—<em>m</em></span></div>
      <div class="tele"><span class="lbl">Battery</span><span class="val" id="t-bv">—<em>V</em></span></div>
      <div class="tele"><span class="lbl">Charge</span><span class="val" id="t-bpct">—<em>%</em></span></div>
      <div class="tele" id="tile-obs"><span class="lbl">Obstacle · front</span><span class="val" id="t-obs">—<em>cm</em></span></div>
      <div class="tele"><span class="lbl">IR · L / R</span><span class="val" id="t-ir">— / —</span></div>
    </div></section>
  </div>
  <div class="tabpage" id="tab-map">
    <section class="panel map-pan"><h2>Surface Map</h2>
    <div id="map"></div>
    <div class="geo-row">
      <button class="diag-btn" id="geo-btn" onclick="geoToggle()">🎯 تحديد النطاق</button>
      <span class="geo-lbl">نصف القطر</span>
      <input type="range" id="geo-r" min="5" max="200" value="30" step="5" oninput="geoRadius()">
      <span id="geo-rv">30م</span>
      <span id="geo-state"></span>
    </div>
    <div class="map-legend">
      <span><i class="leg-heat"></i> قياسات</span>
      <span><i class="leg-src"></i> اتجاه المصدر</span>
      <span><i class="leg-grad"></i> تدرّج الجرعة</span>
    </div></section>
  </div>
  <div class="tabpage" id="tab-ai">
    <section class="panel ai-pan"><h2>مخطِّط المهمات — OpenAI</h2>
    <div class="ai-cfg">
      <input type="password" id="gem-key" placeholder="مفتاح OpenAI API (يُحفظ في متصفحك فقط)">
      <input type="text" id="gem-model" placeholder="gpt-4o-mini">
    </div>
    <div id="ai-log"></div>
    <div class="ai-input-row">
      <textarea id="ai-in" placeholder="مثال: خطّط مسحاً إشعاعياً للمنطقة شمال الروفر، أو اسأل عن القراءات الحالية..."></textarea>
      <button id="ai-send" onclick="aiSend()">إرسال</button>
    </div>
    <div id="ai-plan">
      <h3>الخطة المقترحة — راجِعها قبل الحفظ</h3>
      <div id="ai-plan-name"></div>
      <ol id="ai-plan-steps"></ol>
      <div class="ai-plan-btns">
        <button onclick="aiSavePlan()">💾 حفظ على SD</button>
        <button class="run" id="ai-run" onclick="cmd('plan_start')" disabled>▶ بدء المهمة</button>
        <button class="abort" onclick="cmd('plan_abort')">⏹ إيقاف</button>
      </div>
      <div id="ai-plan-status"></div>
    </div></section>
  </div>
  <div class="tabpage" id="tab-diag">
    <section class="panel diag-pan">
      <h2>الاختبارات والتشخيص</h2>
      <div class="diag-grid">
        <button class="diag-btn" onclick="cmd('mission_start')">▶ بدء مهمة</button>
        <button class="diag-btn" onclick="cmd('mission_end')">⏹ إنهاء مهمة</button>
        <button class="diag-btn" onclick="cmd('scan_start')">🧭 مسح 360°</button>
        <button class="diag-btn" onclick="cmd('scan_cancel')">إلغاء المسح</button>
        <div class="scanbar"><div id="scan-fill"></div></div>
        <span id="scan-res" style="align-self:center;font-size:.72rem;color:var(--muted)">—</span>
      </div>
      <div class="diag-grid">
        <button class="diag-btn" onclick="cmd('set_level')">🎚️ ضبط المستوي (BNO)</button>
        <button class="diag-btn" onclick="cmd('reset_level')">↺ استعادة الافتراضي</button>
        <button class="diag-btn" onclick="cmd('rth')">🏠 عودة للمنزل RTH</button>
        <button class="diag-btn" onclick="showReport()">📄 عرض التقرير</button>
        <button class="diag-btn" onclick="window.open('/api/log')">⬇ تنزيل آخر CSV</button>
        <button class="diag-btn" onclick="showStatus()">🩺 حالة النظام الكاملة</button>
      </div>
      <div class="diag-grid" id="sim-tools">
        <button class="diag-btn sim" onclick="cmd('sim_src_on')">☢ تفعيل المصدر الوهمي</button>
        <button class="diag-btn sim" onclick="cmd('sim_src_off')">إطفاء المصدر الوهمي</button>
        <button class="diag-btn sim" onclick="cmd('sim_spike')">⚡ قفزة شذوذ مفتعلة</button>
      </div>
      <div id="diag-out"></div>
    </section>
  </div>
</main>

<button id="estop-btn" onclick="cmd('estop')">🛑 إيقاف طوارئ</button>
<div id="toasts"></div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" crossorigin=""></script>
<script>
const $ = id => document.getElementById(id);
let ws=null, map=null, marker=null, trail=null, trailPts=[];
let fpsCount=0, lastFpsT=Date.now();

function setPill(id, state){
  const el = $(id);
  el.classList.remove('ok','bad','warn');
  if(state) el.classList.add(state);
}

let camBase = '', snapTimer = null, snapInterval = 700, snapMode = 'snapshot', camRunning = false;

async function loadConfig(){
  try{
    const r = await fetch('/config');
    const c = await r.json();
    camBase = c.camBase;
    startCam();
  }catch(e){ console.error('config fail', e); }
}

function startCam(){
  $('cam-fail').classList.add('hidden');
  stopCam();
  camRunning = true;
  if(snapMode === 'live'){
    // بث مباشر (MJPEG) — يسحب تيار عالي
    const img = $('stream');
    img.src = camBase + '/stream?t=' + Date.now();
    img.onload = () => { setPill('cam-pill','ok'); };
    img.onerror = () => { setPill('cam-pill','bad'); $('cam-fail').classList.remove('hidden'); };
  } else {
    // snapshot متسلسل: كل صورة تبدأ فور انتهاء السابقة + التأخير المحدد
    snapLoop();
  }
}

function snapLoop(){
  if(!camRunning || snapMode !== 'snapshot') return;
  const img = $('stream');
  const tmp = new Image();
  tmp.onload = () => {
    img.src = tmp.src;
    setPill('cam-pill','ok');
    fpsCount++;
    // جدول الصورة التالية بعد التأخير
    snapTimer = setTimeout(snapLoop, snapInterval);
  };
  tmp.onerror = () => {
    setPill('cam-pill','bad');
    snapTimer = setTimeout(snapLoop, snapInterval);
  };
  tmp.src = camBase + '/capture?t=' + Date.now();
}

function grabSnapshot(){
  // صورة فورية عند الطلب (زر ⟳ الآن)
  const img = $('stream');
  const tmp = new Image();
  tmp.onload = () => { img.src = tmp.src; setPill('cam-pill','ok'); fpsCount++; };
  tmp.onerror = () => { setPill('cam-pill','bad'); };
  tmp.src = camBase + '/capture?t=' + Date.now();
}

function stopCam(){
  camRunning = false;
  if(snapTimer){ clearTimeout(snapTimer); snapTimer = null; }
  const img = $('stream');
  img.src = '';   // يوقف أي بث MJPEG جارٍ
}

function retryCam(){ startCam(); }

// تبديل وضع الكاميرا
function setCamMode(mode){
  snapMode = mode;
  startCam();
  // حدّث أزرار الوضع
  document.querySelectorAll('.cam-mode-btn').forEach(b => {
    b.classList.toggle('active', b.dataset.mode === mode);
  });
  // أظهر/أخفِ خيارات السرعة (للـ snapshot فقط)
  $('snap-speed-row').style.display = (mode === 'snapshot') ? 'flex' : 'none';
}

function setSnapInterval(ms){
  snapInterval = ms;
  document.querySelectorAll('.snap-btn').forEach(b => {
    const t = b.textContent;
    b.classList.toggle('active',
      (ms===400 && t==='سريع') || (ms===700 && t==='متوسط') || (ms===1500 && t==='موفّر'));
  });
  // ما نحتاج نعيد التشغيل — snapLoop ياخذ snapInterval الجديد تلقائياً
}

setInterval(() => {
  const dt = (Date.now() - lastFpsT)/1000;
  const fps = fpsCount/dt;
  $('hud').textContent = (fps > 0 ? fps.toFixed(0) : '—') + ' fps';
  fpsCount = 0; lastFpsT = Date.now();
}, 1000);

function connectWS(){
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  ws = new WebSocket(`${proto}//${location.host}/ws`);
  ws.onopen = () => setPill('ws-pill','ok');
  ws.onclose = () => { setPill('ws-pill','bad'); setTimeout(connectWS, 1500); };
  ws.onerror = () => setPill('ws-pill','bad');
  ws.onmessage = (e) => {
    try{
      const m = JSON.parse(e.data);
      if(m.t === 'gps') handleGPS(m);
      else if(m.t === 'batt') handleBattery(m);
      else if(m.t === 'rad') handleRad(m);
      else if(m.t === 'alarm') handleAlarm(m);
      else if(m.t === 'info') handleInfo(m);
      else if(m.t === 'scan') handleScanResult(m);
      else if(m.t === 'report') { toast('📄 تقرير المهمة جاهز'); showReportData(m.data); }
    }catch(_){}
  };
}
function send(cmd){ if(ws && ws.readyState === 1) ws.send(cmd); }

const spd = $('speed'), spdLbl = $('spd-lbl');
spd.oninput = () => spdLbl.textContent = spd.value;

let activeDir = null;
let heartbeat = null;

function startCmd(dir){
  if(activeDir === dir) return;
  activeDir = dir;
  const msg = dir === 'S' ? 'S' : dir + spd.value;
  send(msg);
  clearInterval(heartbeat);
  if(dir !== 'S'){
    // keep-alive فقط لتجديد مزلاج الهَب — الهَب هو من يكرّر الأمر للكام من حلقته
    // الموثوقة، فلا تتأثر استمرارية القيادة بتلعثم مؤقّت المتصفح تحت حمل الكاميرا.
    heartbeat = setInterval(() => send(dir + spd.value), 300);
  }
}
function stopCmd(){
  clearInterval(heartbeat); heartbeat = null;
  activeDir = null;
  // كرّر التوقف عدة مرات: لو ضاعت حزمة 'S' يبقى الروفر متحركاً حتى مهلة الأمان.
  send('S'); setTimeout(() => send('S'), 70); setTimeout(() => send('S'), 150);
}

document.querySelectorAll('.btn').forEach(btn => {
  const d = btn.dataset.dir;
  const press = (e) => {
    e.preventDefault();
    btn.classList.add('active');
    if(d === 'S') send('S'); else startCmd(d);
  };
  const release = (e) => {
    if(e) e.preventDefault();
    btn.classList.remove('active');
    if(d !== 'S') stopCmd();
  };
  btn.addEventListener('touchstart', press, {passive:false});
  btn.addEventListener('touchend', release, {passive:false});
  btn.addEventListener('touchcancel', release, {passive:false});
  btn.addEventListener('mousedown', press);
  btn.addEventListener('mouseup', release);
  btn.addEventListener('mouseleave', () => {
    if(btn.classList.contains('active')) release();
  });
});

const keyMap = {ArrowUp:'F',KeyW:'F',ArrowDown:'B',KeyS:'B',
                ArrowLeft:'L',KeyA:'L',ArrowRight:'R',KeyD:'R',Space:'S'};
const held = {};
// لا تخطف مفاتيح القيادة أثناء الكتابة في حقل نصّي (الشات، الإعدادات...) —
// وإلا لن تُكتب المسافة ولا حروف W/A/S/D. تجاهل الحدث عندما يكون التركيز في إدخال.
function typingInField(e){
  const el = e.target;
  return el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.isContentEditable);
}
document.addEventListener('keydown', e => {
  if(typingInField(e)) return;
  const d = keyMap[e.code];
  if(!d) return;
  e.preventDefault();
  if(held[e.code]) return;
  held[e.code] = true;
  if(d === 'S'){ send('S'); return; }
  startCmd(d);
});
document.addEventListener('keyup', e => {
  if(typingInField(e)) return;
  const d = keyMap[e.code];
  if(!d) return;
  delete held[e.code];
  if(d !== 'S' && Object.keys(held).length === 0) stopCmd();
});

// ── لوحة التشخيص: أوامر + توست تأكيد ────────────────────────
function cmd(c){
  if(!ws || ws.readyState !== 1){ toast('⚠ غير متصل بالهَب', true); return; }
  ws.send(JSON.stringify({c:c}));
  toast('أُرسل: ' + (CMD_TXT[c] || c));
}
const CMD_TXT = {mission_start:'بدء مهمة', mission_end:'إنهاء مهمة',
  scan_start:'مسح 360°', scan_cancel:'إلغاء المسح', estop:'إيقاف طوارئ',
  rth:'RTH', sim_spike:'قفزة شذوذ', sim_src_on:'تفعيل المصدر', sim_src_off:'إطفاء المصدر',
  plan_start:'بدء المهمة', plan_abort:'إيقاف المهمة', set_level:'ضبط المستوي',
  reset_level:'استعادة المستوي الافتراضي'};
const INFO_TXT = {mission_started:'✅ بدأت المهمة — التسجيل جارٍ',
  mission_ended:'✅ انتهت المهمة', logger_fail:'⚠ فشل فتح ملف التسجيل!',
  estop:'🛑 إيقاف طوارئ منفَّذ', scan_started:'🧭 بدأ المسح الدوراني',
  scan_cancelled:'أُلغي المسح', rth_unavailable:'RTH غير متاح بعد (مرحلة قادمة)',
  sim_spike:'⚡ حُقنت قفزة شذوذ', sim_src_on:'☢ المصدر الوهمي مفعَّل',
  sim_src_off:'المصدر الوهمي مُطفأ',
  level_set:'✅ ضُبط المستوي — الميل يُقاس من هذا الوضع الآن',
  level_fail:'⚠ تعذّر الضبط — الـBNO غير مستقر لحظتها، أعد المحاولة',
  level_reset:'↺ عاد المستوي للافتراضي (ميل مطلق)'};
function toast(txt, isErr){
  const t = document.createElement('div');
  t.className = 'toast' + (isErr ? ' err' : '');
  t.textContent = txt;
  $('toasts').appendChild(t);
  setTimeout(() => t.remove(), 3500);
}
function handleInfo(m){
  const msg = m.msg || '';
  if(msg.startsWith('plan_saved')){
    $('ai-run').disabled = false;
    $('ai-plan-status').textContent = '✅ حُفظت على SD ('+(msg.split(':')[1]||'')+'). جاهزة — اضغط بدء المهمة.';
    toast('✅ حُفظت الخطة على SD'); return;
  }
  if(msg.startsWith('plan_abort')){ $('ai-plan-status').textContent = '⏹ أُوقفت المهمة ('+(msg.split(':')[1]||'')+')'; toast('⏹ أُوقفت المهمة', true); return; }
  if(msg === 'plan_running'){ $('ai-plan-status').textContent = '▶ المهمة تعمل...'; toast('▶ بدأت المهمة'); return; }
  if(msg === 'plan_done'){ $('ai-plan-status').textContent = '✅ اكتملت المهمة'; toast('✅ اكتملت المهمة'); return; }
  if(msg.startsWith('plan_')){ $('ai-plan-status').textContent = '⚠ '+msg; toast('⚠ خطة مرفوضة: '+msg, true); return; }
  toast(INFO_TXT[msg] || msg, /fail|unavail|down/.test(msg));
}
function handleScanResult(m){
  $('scan-fill').style.width = '0%';
  $('scan-res').textContent = m.ok ? ('الاتجاه: ' + m.dir + '° (ثقة ' + Math.round(m.conf*100) + '%)') : 'غير حاسم';
  if(m.ok) drawSourceArrow(m.dir, m.conf);   // سهم اتجاه المصدر على الخريطة
  toast(m.ok ? ('🧭 اتجاه المصدر: ' + m.dir + '°') : '🧭 المسح غير حاسم', !m.ok);
}
async function showReport(){
  try{
    const r = await fetch('/api/report');
    if(!r.ok){ toast('لا يوجد تقرير محفوظ بعد', true); return; }
    showReportData(await r.json());
  }catch(_){ toast('تعذر جلب التقرير', true); }
}
function showReportData(d){
  const o = $('diag-out');
  o.style.display = 'block';
  o.textContent = JSON.stringify(d, null, 2);
  const _d=o.closest('details'); if(_d) _d.open=true;
}
async function showStatus(){
  try{
    const r = await fetch('/api/status');
    const d = await r.json();
    const o = $('diag-out');
    o.style.display = 'block';
    o.textContent = JSON.stringify(d, null, 2);
  }catch(_){ toast('تعذر جلب الحالة', true); }
}

// ── مخطِّط المهمات OpenAI (النداء من متصفحك — المفتاح لا يغادره) ──
const GEM_SYS = `أنت مساعد لروفر فحص إشعاعي (عداد جيجر + GPS + IMU/ميل + حساسات عوائق: فوق-صوتي أمامي + IR يسار/يمين). مهمتك:
(1) تخطيط مهمات حركة آمنة قصيرة، (2) الإجابة عن قراءات الإشعاع والحالة.
الأوامر المسموحة حصراً في الخطة (لا شيء غيرها):
- move: dir أحد F/B/L/R، power 20-100، duration_s ثوانٍ.
- scan: مسح دوراني 360° لتحديد اتجاه المصدر.
- stop: توقف/انتظار duration_s ثوانٍ.
اجعل الحركات حذرة: power ≤ 70 و duration_s ≤ 15، وابدأ ثم امسح ثم تحرّك بخطوات صغيرة.
رُدّ بصيغة JSON فقط بلا أي نص خارجها:
{"reply":"ردك بالعربية","plan":{"name":"اسم قصير","steps":[{"action":"move","dir":"F","power":60,"duration_s":5,"note":"وصف"}]}}
إن كان الطلب سؤالاً عن القراءات فقط، اجعل "plan": null.`;
let gemHistory = [], lastTele = {}, currentPlan = null;

const AI_DEFAULT_MODEL = 'gpt-4o-mini';   // رخيص وكفؤ ويدعم إخراج JSON منظّم
function aiCfgLoad(){
  $('gem-key').value = localStorage.getItem('gemKey') || '';
  // رحّل أي موديل Gemini محفوظ سابقاً إلى موديل OpenAI الافتراضي.
  let sm = localStorage.getItem('gemModel') || AI_DEFAULT_MODEL;
  if(/gemini/i.test(sm) || !sm){ sm = AI_DEFAULT_MODEL; localStorage.setItem('gemModel', sm); }
  $('gem-model').value = sm;
  $('gem-key').onchange = () => localStorage.setItem('gemKey', $('gem-key').value.trim());
  $('gem-model').onchange = () => localStorage.setItem('gemModel', $('gem-model').value.trim());
}
function aiLog(text, cls){
  const d = document.createElement('div');
  d.className = 'ai-msg ' + cls; d.textContent = text;
  $('ai-log').appendChild(d); $('ai-log').scrollTop = $('ai-log').scrollHeight;
}
function readingsCtx(){
  const t = lastTele;
  return `[قراءات حية] CPM=${t.cpm??'?'} جرعة=${t.usvh??'?'}µSv/h خطر=${t.risk??'?'} `
    + `GPS=${t.fix?'مثبّت':'بلا'}(${t.lat??'?'},${t.lng??'?'}) اتجاه=${t.hdg??'?'}° `
    + `ميل=(p${t.pitch??'?'}/r${t.roll??'?'}) IMUcal=${t.imucal??'?'} `
    + `عائق أمامي=${t.obs??'?'}سم IR(يسار/يمين)=${t.obl?'عائق':'خالٍ'}/${t.obr?'عائق':'خالٍ'}`;
}
async function aiSend(){
  const key = ($('gem-key').value||'').trim();
  if(!key){ aiLog('أدخل مفتاح OpenAI API أولاً (يُحفظ في متصفحك فقط).', 'sys'); return; }
  const model = ($('gem-model').value||AI_DEFAULT_MODEL).trim();
  const msg = ($('ai-in').value||'').trim();
  if(!msg) return;
  $('ai-in').value = ''; aiLog(msg, 'u');
  gemHistory.push({role:'user', content: msg + '\n' + readingsCtx()});
  $('ai-send').disabled = true;
  try{
    // OpenAI Chat Completions — إخراج JSON منظّم عبر response_format
    const body = {
      model,
      messages: [{role:'system', content: GEM_SYS}, ...gemHistory],
      temperature: 0.4,
      response_format: {type:'json_object'}
    };
    const r = await fetch('https://api.openai.com/v1/chat/completions', {
      method:'POST',
      headers:{'Content-Type':'application/json', 'Authorization':'Bearer '+key},
      body: JSON.stringify(body)
    });
    if(!r.ok){
      let detail=''; try{ detail = (await r.json())?.error?.message || ''; }catch(_){}
      const hint = r.status===401 ? ' (المفتاح غير صحيح أو غير مفعّل)'
                 : r.status===404 ? ' (اسم الموديل غير صالح — جرّب gpt-4o-mini)'
                 : r.status===429 ? ' (تجاوزت الحصة/الرصيد — تحقق من رصيدك في platform.openai.com)'
                 : ' — تحقق من المفتاح/الموديل/الإنترنت.';
      aiLog('خطأ OpenAI '+r.status+(detail?' — '+detail:'')+hint, 'sys');
      $('ai-send').disabled=false; return;
    }
    const j = await r.json();
    const txt = j.choices?.[0]?.message?.content || '{}';
    gemHistory.push({role:'assistant', content: txt});
    let obj; try{ obj = JSON.parse(txt); }catch(_){ obj = {reply: txt, plan:null}; }
    aiLog(obj.reply || '(بلا رد)', 'a');
    if(obj.plan && obj.plan.steps && obj.plan.steps.length) showPlan(obj.plan);
  }catch(e){ aiLog('تعذّر الاتصال بـOpenAI — هل المتصفح على شبكة فيها إنترنت؟ ('+e.message+')', 'sys'); }
  $('ai-send').disabled = false;
}
function showPlan(plan){
  currentPlan = plan;
  $('ai-plan-name').textContent = 'المهمة: ' + (plan.name||'');
  const ol = $('ai-plan-steps'); ol.innerHTML = '';
  plan.steps.forEach(s => {
    let d = s.action;
    if(s.action==='move') d += ` ${s.dir} قوة ${s.power||60} لمدة ${s.duration_s||3}ث`;
    else if(s.action==='stop') d += ` ${s.duration_s||3}ث`;
    if(s.note) d += ` — ${s.note}`;
    const li = document.createElement('li'); li.textContent = d; ol.appendChild(li);
  });
  $('ai-run').disabled = true;
  $('ai-plan-status').textContent = 'احفظ الخطة على SD قبل بدء المهمة.';
  $('ai-plan').classList.add('show');
}
function aiSavePlan(){
  if(!currentPlan) return;
  if(!ws || ws.readyState !== 1){ toast('⚠ غير متصل بالهَب', true); return; }
  ws.send(JSON.stringify({c:'plan_save', name: currentPlan.name||'plan', steps: currentPlan.steps}));
  toast('أُرسلت الخطة للهَب...');
}

// ══ عناصر الجرعة البصرية: عداد دائري + شريط مستويات + رسم CPM ══
const LVL_COLOR = ['#10b981','#a3e635','#f59e0b','#fb923c','#ef4444'];
const G_CIRC = 2*Math.PI*76;          // محيط دائرة العداد (r=76)
const G_ARC  = G_CIRC*0.75;           // قوس مرئي 270°
const G_LO = Math.log10(0.05), G_HI = Math.log10(200);   // خريطة لوغاريتمية تغطي المستويات الخمسة
function gaugeFrac(usvh){
  const v = Math.max(usvh, 0.01);
  return Math.min(1, Math.max(0, (Math.log10(v)-G_LO)/(G_HI-G_LO)));
}
function initRadVisuals(){
  $('g-track').style.strokeDasharray = G_ARC+' '+G_CIRC;
  $('g-fill').style.strokeDasharray  = '0 '+G_CIRC;
  sizeChart();
  window.addEventListener('resize', sizeChart);
}
function updateRadVisuals(m){
  const col = LVL_COLOR[m.lvl] || LVL_COLOR[0];
  const fill = $('g-fill');
  fill.style.strokeDasharray = (G_ARC*gaugeFrac(m.usvh))+' '+G_CIRC;
  fill.style.stroke = col;
  $('g-dose').textContent = m.usvh.toFixed(m.usvh<10?3:1);
  $('g-dose').style.color = col;
  $('g-cpm').textContent = m.cpm.toFixed(0);
  const rk = $('g-risk'); rk.textContent = m.risk; rk.style.color = col;
  document.querySelectorAll('.lvl-seg').forEach(s=>{
    const on = (+s.dataset.l) === m.lvl;
    s.classList.toggle('on', on);
    s.style.color = on ? LVL_COLOR[+s.dataset.l] : '';
  });
  pushCpm(m.cpm);
}

// رسم CPM الزمني — canvas خفيف بلا مكتبة، آخر 5 دقائق
let cpmHist = [];
const CPM_WINDOW = 5*60*1000;
function sizeChart(){
  const c = $('cpm-chart'); if(!c) return;
  const dpr = window.devicePixelRatio||1;
  c.width = Math.max(1, c.clientWidth*dpr); c.height = Math.max(1, c.clientHeight*dpr);
  c.getContext('2d').setTransform(dpr,0,0,dpr,0,0);
  drawCpmChart();
}
function pushCpm(cpm){
  const now = Date.now();
  cpmHist.push({t:now, cpm});
  const cut = now - CPM_WINDOW;
  while(cpmHist.length && cpmHist[0].t < cut) cpmHist.shift();
  drawCpmChart();
}
function drawCpmChart(){
  const c = $('cpm-chart'); if(!c) return;
  const ctx = c.getContext('2d');
  const W = c.clientWidth, H = c.clientHeight;
  ctx.clearRect(0,0,W,H);
  if(cpmHist.length < 2) return;
  const now = Date.now();
  let peak = 0; cpmHist.forEach(p=>{ if(p.cpm>peak) peak=p.cpm; });
  $('cpm-peak').textContent = peak.toFixed(0);
  const top = Math.max(peak*1.15, 30);
  ctx.strokeStyle = 'rgba(124,138,168,.14)'; ctx.lineWidth = 1;
  for(let i=1;i<3;i++){ const y=H*i/3; ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); }
  const xOf = t => W*(1-(now-t)/CPM_WINDOW);
  const yOf = v => H - (v/top)*(H-6) - 3;
  ctx.beginPath(); ctx.moveTo(xOf(cpmHist[0].t), H);
  cpmHist.forEach(p=> ctx.lineTo(xOf(p.t), yOf(p.cpm)));
  ctx.lineTo(xOf(cpmHist[cpmHist.length-1].t), H); ctx.closePath();
  const g = ctx.createLinearGradient(0,0,0,H);
  g.addColorStop(0,'rgba(34,211,238,.30)'); g.addColorStop(1,'rgba(34,211,238,0)');
  ctx.fillStyle = g; ctx.fill();
  ctx.beginPath();
  cpmHist.forEach((p,i)=>{ const x=xOf(p.t), y=yOf(p.cpm); i?ctx.lineTo(x,y):ctx.moveTo(x,y); });
  ctx.strokeStyle = '#22d3ee'; ctx.lineWidth = 2; ctx.lineJoin='round'; ctx.stroke();
}

// ── طبقات الخريطة: قياسات ملوّنة + سهم المصدر + سهم التدرّج ──
let heatLayer=null, heatPts=[], srcArrow=null, srcHead=null, gradArrow=null;
let lastHeatT=0, lastHeatLL=null;
function destPoint(lat,lng,brgDeg,distM){
  const R=6378137, br=brgDeg*Math.PI/180;
  const dLat=(distM*Math.cos(br))/R;
  const dLng=(distM*Math.sin(br))/(R*Math.cos(lat*Math.PI/180));
  return [lat+dLat*180/Math.PI, lng+dLng*180/Math.PI];
}
function addHeatPoint(lat,lng){
  if(!heatLayer || lastTele.usvh===undefined) return;
  const now=Date.now();
  if(lastHeatLL){                            // خفّف: نقطة كل 2.5s أو بعد تحرّك ~2م
    const moved = map.distance([lat,lng], lastHeatLL);
    if(now-lastHeatT < 2500 && moved < 2) return;
  }
  lastHeatT=now; lastHeatLL=[lat,lng];
  const lvl = +lastTele.lvl||0, usvh=+lastTele.usvh||0;
  const mk = L.circleMarker([lat,lng], {radius:6, weight:0, fillColor:LVL_COLOR[lvl], fillOpacity:.5});
  mk.addTo(heatLayer);
  heatPts.push({lat,lng,usvh,mk});
  if(heatPts.length>300){ heatLayer.removeLayer(heatPts.shift().mk); }
  updateGradientArrow(lat,lng);
}
function drawSourceArrow(dir,conf){
  if(!marker) return;
  const o=marker.getLatLng();
  const tip=destPoint(o.lat,o.lng,dir,8+conf*24);   // الطول ∝ الثقة
  if(srcArrow) map.removeLayer(srcArrow);
  if(srcHead)  map.removeLayer(srcHead);
  srcArrow=L.polyline([[o.lat,o.lng],tip], {color:'#22d3ee', weight:4, opacity:.9}).addTo(map);
  const head=L.divIcon({className:'rover-marker',
    html:'<div class="src-arrowhead" style="transform:rotate('+dir+'deg)"></div>',
    iconSize:[12,12], iconAnchor:[6,6]});
  srcHead=L.marker(tip, {icon:head, interactive:false}).addTo(map);
}
function updateGradientArrow(lat,lng){
  if(heatPts.length<6) return;
  let mean=0; heatPts.forEach(p=>mean+=p.usvh); mean/=heatPts.length;
  let vx=0, vy=0, n=0;
  heatPts.forEach(p=>{
    const dLat=p.lat-lat, dLng=(p.lng-lng)*Math.cos(lat*Math.PI/180);
    const d=Math.hypot(dLat,dLng); if(d<1e-8) return;
    const w=p.usvh-mean; vx+=w*dLng/d; vy+=w*dLat/d; n++;
  });
  if(n<4 || (Math.abs(vx)<1e-12 && Math.abs(vy)<1e-12)){
    if(gradArrow){ map.removeLayer(gradArrow); gradArrow=null; } return;
  }
  const brg=(Math.atan2(vx,vy)*180/Math.PI+360)%360;   // اتجاه ارتفاع الجرعة
  const tip=destPoint(lat,lng,brg,14);
  if(gradArrow) map.removeLayer(gradArrow);
  gradArrow=L.polyline([[lat,lng],tip], {color:'#f59e0b', weight:3, opacity:.85, dashArray:'6 5'}).addTo(map);
}

// ── حالة الإشعاع + صحة المكونات ─────────────────────────────
function handleRad(m){
  updateRadVisuals(m);
  $('t-dose').innerHTML = m.usvh.toFixed(3) + '<em>µSv/h</em>';
  $('t-risk').textContent = m.risk;
  $('t-cpm').textContent = m.cpm.toFixed(1);
  if(m.cpm_raw !== undefined) $('t-cpmraw').textContent = m.cpm_raw.toFixed(1);

  // بطاقة الأنبوب الثاني: "غير مركّب" في وضع الأنبوب الواحد
  const t2 = $('tile-cpm2');
  if(m.single){ $('t-cpm2').textContent = 'غير مركّب'; t2.classList.add('dim'); }
  else { $('t-cpm2').textContent = m.cpm2.toFixed(1); t2.classList.remove('dim'); }

  // شريحة الجرعة بلون الخطر (lvl: 0 Safe .. 4 Critical)
  $('rad-txt').textContent = m.usvh.toFixed(2) + ' µSv/h';
  setPill('rad-pill', m.lvl >= 3 ? 'bad' : (m.lvl >= 1 ? 'warn' : 'ok'));
  $('tile-dose').classList.toggle('crit', m.lvl >= 3);

  // عطل الجيجر: قراءة مستحيلة فيزيائياً = سلك الإشارة مفصول (دخل عائم يلتقط ضجيجاً)
  const gFault = !!m.g1fault;
  $('t-cpm').parentElement.classList.toggle('crit', gFault);
  if(gFault){
    $('t-risk').textContent = '⚠ سلك مفصول؟';
    $('geiger-txt').textContent = 'G1 ⚠';
    setPill('geiger-pill', 'bad');
    handleAlarm({msg:'geiger_fault'});   // يُعاد كل ثانية فيبقى ظاهراً حتى يزول العطل
  } else {
    $('geiger-txt').textContent = 'G1';
    // صحة الأنبوب: 2 أخضر / 1 أصفر / 0 أحمر (صمت = سلك مفصول)
    if(m.g1h !== undefined)
      setPill('geiger-pill', m.g1h === 2 ? 'ok' : (m.g1h === 1 ? 'warn' : 'bad'));
  }

  // معايرة BNO055: 4 مؤشرات (0-3) + تحذير أصفر إذا mag < 2
  if(m.imucs !== undefined){
    $('imu-txt').textContent = 'IMU ' + m.imucs + m.imucg + m.imuca + m.imucm;
    $('t-imucal').innerHTML = m.imucs+'/'+m.imucg+'/'+m.imuca+'/'+m.imucm+'<em>s/g/a/m</em>';
    setPill('imu-pill', !m.imuok ? 'bad' : (m.imucm < 2 ? 'warn' : 'ok'));
  }

  // ميل الجهاز عن وضعه "المستوي" المرجعي (حماية الميل تعتمد عليه)
  if(m.tiltlv !== undefined){
    $('t-tilt').innerHTML = m.tiltlv.toFixed(0)+'°<em>'+(m.levelset?' من المستوي':' (اضبط المستوي)')+'</em>';
    $('tile-tilt').classList.toggle('crit', m.tiltlv > 25);
  }

  // صحة التخزين: 2 = SD (أخضر) / 1 = fallback على LittleFS (أصفر) / 0 = لا شيء
  if(m.sdh !== undefined){
    $('sd-txt').textContent = m.sdh === 2 ? 'SD' : (m.sdh === 1 ? 'SD→Flash' : 'SD ✗');
    setPill('sd-pill', m.sdh === 2 ? 'ok' : (m.sdh === 1 ? 'warn' : 'bad'));
  }

  // حساسات العوائق (تصل من الأردوينو عبر الكام): مسافة أمامية + IR يسار/يمين
  if(m.obs !== undefined){
    if(m.obh === 0){                       // بيانات قديمة/لا تصل (الأردوينو أو الكام مفصول)
      $('t-obs').innerHTML = '—<em>cm</em>';
      $('t-ir').textContent = '— / —';
      $('obs-txt').textContent = 'OBS ✗';
      setPill('obs-pill', 'bad');
      $('tile-obs').classList.remove('crit');
    } else {
      $('t-obs').innerHTML = (m.obs >= 300 ? '≥300' : m.obs.toFixed(0)) + '<em>cm</em>';
      $('t-ir').textContent = (m.obl?'🔴':'⚪') + ' / ' + (m.obr?'🔴':'⚪');
      const near = !!m.obk, warn = m.obs < 50;
      $('tile-obs').classList.toggle('crit', near);
      $('obs-txt').textContent = near ? 'OBS ⚠' : 'OBS';
      setPill('obs-pill', near ? 'bad' : (warn ? 'warn' : 'ok'));  // أحمر عائق/أصفر تحذير/أخضر خالٍ
      if(near) handleAlarm({msg:'obstacle'});
    }
  }

  // شريط تقدم المسح الدوراني + أزرار المحاكاة (تظهر في SIMULATION_MODE فقط)
  $('scan-fill').style.width = (m.scan ? m.scanpct : 0) + '%';
  $('sim-tools').style.display = m.sim ? 'flex' : 'none';

  // سياق للشات بوت + تقدّم خطة المهمة
  lastTele.cpm = m.cpm; lastTele.usvh = m.usvh; lastTele.risk = m.risk; lastTele.lvl = m.lvl;
  lastTele.pitch = m.pitch; lastTele.roll = m.roll;
  if(m.obs !== undefined){ lastTele.obs = m.obs; lastTele.obl = m.obl; lastTele.obr = m.obr; }
  if(m.imucs !== undefined) lastTele.imucal = ''+m.imucs+m.imucg+m.imuca+m.imucm;
  if(m.plan){ $('ai-plan-status').textContent = `▶ المهمة تعمل — الخطوة ${m.plan_step}/${m.plan_total}`; }
}

// ── إنذارات السلامة (تراجع Critical / انحشار / ميل) ─────────
let alarmTimer = null;
const ALARM_TXT = {
  critical_retreat:'⚠ إشعاع حرج — تراجع تلقائي',
  failsafe_jam:'⚠ انحشار — إيقاف تلقائي',
  failsafe_tilt:'⚠ ميل خطير — تراجع تلقائي',
  geiger_fault:'⚠ الجيجر: قراءة مستحيلة — تحقق من سلك الإشارة (GPIO 15)',
  obstacle:'⚠ عائق أمامي قريب — توقف'
};
function handleAlarm(m){
  const b = $('alarm-banner');
  b.textContent = ALARM_TXT[m.msg] || ('⚠ ' + m.msg);
  b.classList.add('show');
  clearTimeout(alarmTimer);
  alarmTimer = setTimeout(() => b.classList.remove('show'), 6000);
}

function handleBattery(m){
  const v = m.v;
  // بطارية 2x18650: ممتلئة ~8.4V، فاضية ~6.0V
  const VMAX = 8.4, VMIN = 6.0;
  let pct = Math.round((v - VMIN) / (VMAX - VMIN) * 100);
  pct = Math.max(0, Math.min(100, pct));

  $('t-bv').innerHTML = v.toFixed(2) + '<em>V</em>';
  $('t-bpct').innerHTML = pct + '<em>%</em>';
  $('batt-text').textContent = pct + '%';

  const pill = $('batt-pill');
  pill.classList.remove('full','mid','low');
  let icon = '🔋';
  if(pct >= 50){ pill.classList.add('full'); }
  else if(pct >= 20){ pill.classList.add('mid'); }
  else { pill.classList.add('low'); icon = '🪫'; }
  $('batt-icon').textContent = icon;
}

function initMap(){
  map = L.map('map', {zoomControl:true, attributionControl:true}).setView([24.7136, 46.6753], 16);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{
    maxZoom:19, attribution:'© OSM'
  }).addTo(map);

  const icon = L.divIcon({
    className:'rover-marker',
    html:'<div class="rov-arrow" id="rov-arrow"></div>',
    iconSize:[14,16], iconAnchor:[7,8]
  });
  marker = L.marker([24.7136, 46.6753], {icon}).addTo(map);
  trail = L.polyline([], {color:'#e5532b', weight:3, opacity:.75}).addTo(map);
  heatLayer = L.layerGroup().addTo(map);   // طبقة قياسات الإشعاع الملوّنة
  // نقر الخريطة يحرّك مركز النطاق (عند تفعيله)
  map.on('click', e => { if(geoActive){ geoCenter = e.latlng; geoCircle.setLatLng(e.latlng); } });
}

// ── النطاق الجغرافي (geofence): دائرة حدود عمل الروفر ────────
let geoActive=false, geoCircle=null, geoCenter=null, geoRadiusM=30, geoOut=false;
function geoToggle(){
  geoActive = !geoActive;
  $('geo-btn').classList.toggle('on', geoActive);
  if(geoActive){
    geoCenter = marker ? marker.getLatLng() : map.getCenter();
    geoRadiusM = +$('geo-r').value;
    if(!geoCircle) geoCircle = L.circle(geoCenter, {radius:geoRadiusM, color:'#22d3ee', weight:2, fillColor:'#22d3ee', fillOpacity:.06}).addTo(map);
    else { geoCircle.setLatLng(geoCenter).setRadius(geoRadiusM); geoCircle.addTo(map); }
    $('geo-state').textContent = 'النطاق مفعّل — انقر الخريطة لتحريك المركز';
  }else{
    if(geoCircle) map.removeLayer(geoCircle);
    $('geo-state').textContent = ''; geoOut = false;
  }
}
function geoRadius(){
  geoRadiusM = +$('geo-r').value;
  $('geo-rv').textContent = geoRadiusM + 'م';
  if(geoActive && geoCircle) geoCircle.setRadius(geoRadiusM);
}
function geoCheck(lat, lng){
  if(!geoActive || !geoCenter) return;
  const d = map.distance([lat,lng], geoCenter);
  const out = d > geoRadiusM;
  if(out !== geoOut){                       // حافة: تنبيه مرة عند العبور
    geoOut = out;
    if(out){ $('geo-state').textContent = '⚠ الروفر خارج النطاق ('+Math.round(d)+'م)'; toast('⚠ الروفر خرج النطاق المحدد', true); }
    else $('geo-state').textContent = '✅ داخل النطاق';
  } else if(out){ $('geo-state').textContent = '⚠ خارج النطاق ('+Math.round(d)+'م)'; }
}

function handleGPS(m){
  lastTele.fix = m.fix; lastTele.hdg = m.hdg !== undefined ? Math.round(m.hdg) : lastTele.hdg;
  if(m.fix){ lastTele.lat = m.lat.toFixed(6); lastTele.lng = m.lng.toFixed(6);
    setPill('gps-pill','ok');
    $('t-lat').textContent = m.lat.toFixed(6);
    $('t-lng').textContent = m.lng.toFixed(6);
    $('t-sat').textContent = m.sat;
    $('t-spd').innerHTML = m.spd.toFixed(1) + '<em>km/h</em>';
    $('t-hdg').innerHTML = Math.round(m.hdg) + '<em>°</em>';
    $('t-alt').innerHTML = Math.round(m.alt) + '<em>m</em>';
    const ll = [m.lat, m.lng];
    marker.setLatLng(ll);
    // سهم الاتجاه على الخريطة يتبع heading (0°=شمال، دوران عقارب الساعة)
    const el = marker.getElement();
    if(el){ const a = el.querySelector('.rov-arrow'); if(a) a.style.transform = 'rotate(' + m.hdg + 'deg)'; }
    trailPts.push(ll);
    if(trailPts.length > 800) trailPts.shift();
    trail.setLatLngs(trailPts);
    if(trailPts.length === 1) map.setView(ll, 18);
    addHeatPoint(m.lat, m.lng);      // قياس ملوّن + تحديث سهم التدرّج
    geoCheck(m.lat, m.lng);          // تحقق النطاق الجغرافي
  }else{
    setPill('gps-pill','warn');
    $('t-sat').textContent = m.sat || 0;
  }
}

initMap();
initRadVisuals();
loadConfig();
connectWS();
aiCfgLoad();
</script>
<script>
// ── تبديل التبويبات ──────────────────────────────────────────
(function(){
  function switchTab(name){
    document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('active', t.dataset.tab===name));
    document.querySelectorAll('.tabpage').forEach(p=>p.classList.toggle('active', p.id==='tab-'+name));
    // خريطة Leaflet داخل حاوية كانت مخفية → أعد حساب مقاسها عند إظهارها
    if(name==='map' && typeof map!=='undefined' && map){ setTimeout(()=>{ try{ map.invalidateSize(); }catch(_){}} , 90); }
    try{ localStorage.setItem('activeTab', name); }catch(_){}
  }
  window.switchTab = switchTab;
  document.querySelectorAll('.tab').forEach(t=> t.addEventListener('click', ()=> switchTab(t.dataset.tab)));
  let init='drive'; try{ init = localStorage.getItem('activeTab') || 'drive'; }catch(_){}
  if(!document.getElementById('tab-'+init)) init='drive';
  switchTab(init);
})();
</script>
</body>
</html>
)rawhtml";

// ── إرسال أمر UDP إلى ESP32-CAM ─────────────────────────────
void sendCmdUDP(const String& cmd) {
    if (camResolved) {
        udp.beginPacket(camIP, UDP_CAM_CMD_PORT);
        udp.print(cmd);
        udp.endPacket();
    } else {
        // Fallback: subnet broadcast
        IPAddress local = WiFi.localIP();
        IPAddress bcast(local[0], local[1], local[2], 255);
        udp.beginPacket(bcast, UDP_CAM_CMD_PORT);
        udp.print(cmd);
        udp.endPacket();
    }
}

// ── كل أوامر الحركة الصادرة (يدوية أو ذاتية) تمر من هنا ──────
//   ترسل للكام عبر UDP وتغذّي المحاكاة (heading وموقع وهميان)
void roverSend(const char* cmd) {
    sendCmdUDP(String(cmd));
    char dir = cmd[0];
    int power = CMD_DEFAULT_POWER;
    if (cmd[1] != 0) {
        int p = atoi(cmd + 1);
        if (p > 0 && p <= 100) power = p;
    }
    imuNotifyCommand(dir, power);
}

// ── إدارة المهمة ─────────────────────────────────────────────
void wsBroadcastInfo(const char* msg) {
    char j[96];
    snprintf(j, sizeof(j), "{\"t\":\"info\",\"msg\":\"%s\"}", msg);
    ws.textAll(j);
}

void startMission() {
    if (reportMissionActive()) return;
    unsigned long now = millis();
    // فشل التسجيل لا يجمّد النظام — المهمة تستمر بلا ملف CSV
    if (!loggerStartMission()) wsBroadcastInfo("logger_fail");
    anomalyStartCalibration(now);
    gradientReset();
    reportMissionStart(now);
    wsBroadcastInfo("mission_started");
    Serial.printf("[MISSION] started, log=%s\n", loggerCsvPath());
}

void endMission() {
    if (!reportMissionActive()) return;
    unsigned long now = millis();
    if (scanActive()) scanCancel();
    float gDir, gSlope;
    if (gradientCompute(gDir, gSlope)) reportSetGradient(gDir, gSlope);
    String rep = reportBuildJson(now);
    reportMissionEnd();
    loggerSaveText(loggerReportPath(), rep);
    loggerEndMission();
    ws.textAll("{\"t\":\"report\",\"data\":" + rep + "}");
    wsBroadcastInfo("mission_ended");
    Serial.printf("[MISSION] ended, report=%s\n", loggerReportPath());
}

void emergencyStop() {
    manualActive = false;            // أوقف مزلاج التكرار اليدوي
    if (scanActive()) scanCancel();
    riskCancelRetreat();
    failsafeCancel();
    roverSend("S");
    wsBroadcastInfo("estop");
}

// ═══ منفّذ خطة المهمة (من الشات بوت OpenAI) ═══════════════════
//   الخطة = تسلسل خطوات بدائية آمنة فقط: move / scan / stop. تُحفظ على
//   SD ويشغّلها زر "بدء المهمة". طبقة السلامة تبقى فوقها، وأي أمر يدوي
//   أو حدث سلامة (ميل/انحشار/حرج) يوقفها فوراً. الهَب يتحقق من كل خطوة.
#define PLAN_MAX_STEPS 24
struct PlanStep { char action[6]; char dir; int power; int durS; };
PlanStep planSteps[PLAN_MAX_STEPS];
int  planCount = 0;
bool planLoaded = false, planRunning = false;
int  planCurStep = 0;
char planName[32] = "";
unsigned long planStepStartMs = 0, planStepHbMs = 0;

bool planActive() { return planRunning; }

static bool planActionAllowed(const char* a) {   // القائمة المسموحة حصراً
    return !strcmp(a, "move") || !strcmp(a, "scan") || !strcmp(a, "stop");
}

// حفظ خطة من الشات بوت: تحقق كامل + تحميل للذاكرة + حفظ على SD
void planSave(const String& json) {
    DynamicJsonDocument doc(6144);
    if (deserializeJson(doc, json)) { wsBroadcastInfo("plan_bad_json"); return; }
    JsonArray steps = doc["steps"].as<JsonArray>();
    if (steps.isNull() || steps.size() == 0) { wsBroadcastInfo("plan_empty"); return; }
    int n = 0;
    for (JsonObject s : steps) {
        if (n >= PLAN_MAX_STEPS) break;
        const char* a = s["action"] | "";
        if (!planActionAllowed(a)) { wsBroadcastInfo("plan_bad_action"); return; }   // رفض كامل
        strlcpy(planSteps[n].action, a, sizeof(planSteps[n].action));
        const char* d = s["dir"] | "S";
        char dc = d[0];
        planSteps[n].dir   = (dc=='F'||dc=='B'||dc=='L'||dc=='R') ? dc : 'S';
        planSteps[n].power = constrain((int)(s["power"] | 60), 20, 100);
        planSteps[n].durS  = constrain((int)(s["duration_s"] | 3), 1, 60);
        n++;
    }
    planCount = n; planLoaded = true;
    strlcpy(planName, doc["name"] | "plan", sizeof(planName));
    char path[24];
    for (int i = 1; i < 1000; i++) {
        snprintf(path, sizeof(path), "/PLAN_%03d.json", i);
        if (!loggerFS().exists(path)) break;
    }
    loggerSaveText(path, json);
    char info[72];
    snprintf(info, sizeof(info), "plan_saved:%s:%d", path, planCount);
    wsBroadcastInfo(info);
    Serial.printf("[PLAN] saved %s (%d steps)\n", path, planCount);
}

static void planBeginStep(unsigned long now) {
    planStepStartMs = now; planStepHbMs = 0;
    PlanStep& s = planSteps[planCurStep];
    if      (!strcmp(s.action, "scan")) scanStart(now);
    else if (!strcmp(s.action, "stop")) roverSend("S");
    // move: يُرسَل بالنبض في planUpdate
}

void planStart() {
    if (!planLoaded || planRunning || planCount == 0) { wsBroadcastInfo("plan_none"); return; }
    if (!reportMissionActive()) startMission();     // سجّل قراءات الخطة على SD
    manualActive = false;                           // لا يزاحم المزلاجُ اليدوي الخطةَ الذاتية
    planRunning = true; planCurStep = 0;
    planBeginStep(millis());
    wsBroadcastInfo("plan_running");
    Serial.printf("[PLAN] start '%s' (%d steps)\n", planName, planCount);
}

void planAbort(const char* reason) {
    if (!planRunning) return;
    planRunning = false;
    if (scanActive()) scanCancel();
    roverSend("S");
    char info[48];
    snprintf(info, sizeof(info), "plan_abort:%s", reason ? reason : "");
    wsBroadcastInfo(info);
    Serial.printf("[PLAN] aborted (%s)\n", reason ? reason : "");
}

void planUpdate(unsigned long now) {
    if (!planRunning) return;
    // السلامة فوق الخطة: أي حدث سلامة يوقفها فوراً
    if (riskRetreatActive() || failsafeActive()) { planAbort("safety"); return; }

    PlanStep& s = planSteps[planCurStep];
    bool done = false;
    if (!strcmp(s.action, "move")) {
        if (now - planStepHbMs >= CMD_HEARTBEAT_MS) {
            planStepHbMs = now;
            char c[8]; snprintf(c, sizeof(c), "%c%d", s.dir, s.power);
            roverSend(c);
        }
        if (now - planStepStartMs >= (unsigned long)s.durS * 1000) { roverSend("S"); done = true; }
    } else if (!strcmp(s.action, "scan")) {
        if (!scanActive()) done = true;              // اكتمل المسح
    } else {                                          // stop / wait
        if (now - planStepStartMs >= (unsigned long)s.durS * 1000) done = true;
    }

    if (done) {
        planCurStep++;
        if (planCurStep >= planCount) {
            planRunning = false; roverSend("S");
            endMission();
            wsBroadcastInfo("plan_done");
            Serial.println("[PLAN] complete");
        } else planBeginStep(now);
    }
}

// أوامر تحكم JSON بسيطة من الواجهة: {"c":"mission_start"} ...
// كل أمر يرد بـinfo — لوحة التشخيص تعرضه toast (تأكيد بصري لكل ضغطة)
void handleControlMsg(const String& msg) {
    if      (msg.indexOf("plan_save")     >= 0) planSave(msg);            // خطة الشات بوت
    else if (msg.indexOf("plan_start")    >= 0) planStart();              // زر "بدء المهمة"
    else if (msg.indexOf("plan_abort")    >= 0) planAbort("user");
    else if (msg.indexOf("mission_start") >= 0) startMission();
    else if (msg.indexOf("mission_end")   >= 0) endMission();
    else if (msg.indexOf("scan_start")    >= 0) { scanStart(millis()); wsBroadcastInfo("scan_started"); }
    else if (msg.indexOf("scan_cancel")   >= 0) { scanCancel(); wsBroadcastInfo("scan_cancelled"); }
    else if (msg.indexOf("estop")         >= 0) { planAbort("estop"); emergencyStop(); }
    else if (msg.indexOf("reset_level")   >= 0) { imuResetLevel(); wsBroadcastInfo("level_reset"); }
    else if (msg.indexOf("set_level")     >= 0) wsBroadcastInfo(imuSetLevel() ? "level_set" : "level_fail");
    else if (msg.indexOf("rth")           >= 0) wsBroadcastInfo("rth_unavailable");  // تُنفَّذ في مرحلة قادمة
    else if (msg.indexOf("sim_spike")     >= 0) {
        if (SIMULATION_MODE) { geigerSimInjectSpike(); wsBroadcastInfo("sim_spike"); }
    }
    else if (msg.indexOf("sim_src_on")    >= 0) { if (SIMULATION_MODE) { geigerSimSetSource(true);  wsBroadcastInfo("sim_src_on"); } }
    else if (msg.indexOf("sim_src_off")   >= 0) { if (SIMULATION_MODE) { geigerSimSetSource(false); wsBroadcastInfo("sim_src_off"); } }
}

// ── محاولة العثور على ESP32-CAM عبر mDNS ───────────────────
//   ⚠ queryHost حاجزة — نمرّر مهلة قصيرة (300ms) كي لا تجمّد الحلقة طويلاً،
//   ولا تُستدعى إطلاقاً بعد أن نتعلّم IP الكام من UDP (camViaUdp).
void resolveCam() {
    IPAddress ip = MDNS.queryHost(CAM_HOST_NAME, 300);
    if (ip != IPAddress(0, 0, 0, 0)) {
        if (!camResolved || ip != camIP) {
            camIP = ip;
            camResolved = true;
            Serial.printf("[CAM] resolved: %s\n", camIP.toString().c_str());
        }
    } else {
        if (camResolved) {
            Serial.println("[CAM] lost mDNS, will use broadcast");
        }
        camResolved = false;
    }
}

// ── معالج أوامر التحكم المشترك (WS أو LoRa) ─────────────────
//   الحركة يدوية بأولوية قصوى: تقاطع المسح/التراجع/السلامة فوراً.
void applyRemoteCommand(const String& cmd) {
    if (cmd.length() == 0) return;
    char dir = cmd.charAt(0);
    if (dir=='S') {
        manualActive = false;                        // رفع الإصبع → أوقف المزلاج فوراً
        roverSend("S");
        return;
    }
    if (dir=='F'||dir=='B'||dir=='L'||dir=='R') {
        // أمر حركة يدوي: خزّنه في المزلاج ليكرّره الهَب من حلقته الموثوقة.
        // الأوامر المتكررة (keep-alive) تجدّد الوقت فقط؛ الهَب هو من يمرّر للكام.
        char buf[8];
        strncpy(buf, cmd.c_str(), sizeof(buf)-1); buf[sizeof(buf)-1] = 0;
        bool isNew = (!manualActive || strcmp(manualCmd, buf) != 0);
        strncpy(manualCmd, buf, sizeof(manualCmd));  // مصفوفة ثابتة (آمنة عبر المهام)
        manualLastRxMs = millis();
        manualActive = true;
        if (isNew) {
            if (planActive()) planAbort("manual");   // اليدوي يقاطع خطة الشات بوت
            if (scanActive()) scanCancel();
            if (riskRetreatActive()) riskCancelRetreat();
            if (failsafeActive()) failsafeCancel();
            roverSend(manualCmd);                    // أرسل فوراً عند الضغط/تغيّر الاتجاه
            manualLastFwdMs = manualLastRxMs;
        }
    } else if (dir == '{') {
        handleControlMsg(cmd);
    }
}

// ── استقبال أوامر LoRa (HC-14): أسطر منتهية بـ\n ────────────
void loraReceive(unsigned long now) {
    while (loraSerial.available()) {
        char c = loraSerial.read();
        if (c == '\n' || c == '\r') {
            loraRxBuf.trim();
            if (loraRxBuf.length() > 0) {
                Serial.printf("[LoRa RX] '%s'\n", loraRxBuf.c_str());   // تشخيص وصول الأوامر
                applyRemoteCommand(loraRxBuf);
                lastLoraRxMs = now;      // القناة حيّة
            }
            loraRxBuf = "";
        } else {
            loraRxBuf += c;
            if (loraRxBuf.length() > 90) loraRxBuf = "";   // حماية من التخمة
        }
    }
}

// ── بث حالة مضغوطة عبر LoRa (تصل الشاشة حتى بلا WiFi) ────────
void loraBroadcastStatus() {
    float cpm = geigerFastCPM();
    float usvh = cpmToUsvh(cpm);
    RiskLevel lvl = riskClassify(usvh);
    char l[96];
    snprintf(l, sizeof(l),
        "{\"lora\":1,\"cpm\":%.0f,\"usvh\":%.3f,\"risk\":\"%s\",\"lvl\":%d,\"hdg\":%.0f,\"fix\":%d}\n",
        cpm, usvh, riskName(lvl), (int)lvl, imuHeading(), gpsFix() ? 1 : 0);
    loraSerial.print(l);
}

// ── WebSocket events ─────────────────────────────────────────
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Client #%u connected\n", client->id());
        client->text("{\"t\":\"hello\"}");
    }
    else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
    else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            String cmd = String((char*)data);
            cmd.trim();
            applyRemoteCommand(cmd);
        }
    }
}

// ── بث GPS ──────────────────────────────────────────────────
void broadcastGPS() {
    if (ws.count() == 0) return;
    char json[256];
    if (gpsFix()) {
        // heading من الـIMU (وهمي أو BNO055) — أدق من مسار GPS عند السرعات البطيئة
        snprintf(json, sizeof(json),
            "{\"t\":\"gps\",\"fix\":true,\"lat\":%.6f,\"lng\":%.6f,"
            "\"sat\":%d,\"spd\":%.2f,\"hdg\":%.1f,\"alt\":%.1f}",
            gpsLat(), gpsLng(), gpsSats(), gpsSpeedKmph(), imuHeading(), gpsAltM());
    } else {
        snprintf(json, sizeof(json), "{\"t\":\"gps\",\"fix\":false,\"sat\":%d}", gpsSats());
    }
    ws.textAll(json);
}

// ── بث حالة الإشعاع والمهمة للواجهة ─────────────────────────
void broadcastRad() {
    if (ws.count() == 0) return;
    float cpm = geigerFastCPM();
    float usvh = cpmToUsvh(cpm);
    RiskLevel lvl = riskClassify(usvh);
    uint8_t cs, cg, ca, cm;
    imuGetCalibration(cs, cg, ca, cm);
    char json[896];
    snprintf(json, sizeof(json),
        "{\"t\":\"rad\",\"cpm1\":%.1f,\"cpm2\":%.1f,\"cpm\":%.1f,\"cpm_raw\":%.1f,"
        "\"usvh\":%.3f,\"risk\":\"%s\",\"lvl\":%d,\"anom\":%d,\"calib\":%d,"
        "\"mission\":%d,\"scan\":%d,\"scanpct\":%d,\"retreat\":%d,\"sim\":%d,"
        "\"single\":%d,\"g1h\":%d,\"g1fault\":%d,\"sdh\":%d,\"imuok\":%d,\"imucs\":%d,\"imucg\":%d,"
        "\"imuca\":%d,\"imucm\":%d,\"jam\":%d,\"tilt\":%d,\"pitch\":%.1f,\"roll\":%.1f,"
        "\"tiltlv\":%.0f,\"levelset\":%d,\"plan\":%d,\"plan_step\":%d,\"plan_total\":%d,"
        "\"obs\":%.0f,\"obl\":%d,\"obr\":%d,\"obh\":%d,\"obk\":%d}",
        geigerCPM1(), geigerCPM2(), cpm, geigerFastCPMRaw(), usvh,
        riskName(lvl), (int)lvl,
        anomalyActive() ? 1 : 0,
        anomalyCalibrating() ? anomalyCalibProgress(millis()) : (anomalyCalibrated() ? 100 : -1),
        reportMissionActive() ? 1 : 0,
        scanActive() ? 1 : 0, scanProgressPct(),
        riskRetreatActive() ? 1 : 0,
        SIMULATION_MODE,
        SINGLE_TUBE_MODE, geigerHealth(), geigerImplausible() ? 1 : 0, loggerHealth(),
        imuOk() ? 1 : 0,
        cs, cg, ca, cm,
        failsafeJam() ? 1 : 0, failsafeTilt() ? 1 : 0,
        imuPitch(), imuRoll(),
        imuTiltFromLevel(), imuLevelIsSet() ? 1 : 0,
        planActive() ? 1 : 0, planActive() ? planCurStep + 1 : 0, planCount,
        obstacleDistanceCm(), obstacleLeft() ? 1 : 0, obstacleRight() ? 1 : 0,
        obstacleHealth(), obstacleFrontBlocked() ? 1 : 0);
    ws.textAll(json);
}

// ── بث البطارية ─────────────────────────────────────────────
void broadcastBattery() {
    if (ws.count() == 0) return;
    if (batteryV <= 0.0) return;   // ما استلمنا قراءة بعد
    char json[64];
    snprintf(json, sizeof(json), "{\"t\":\"batt\",\"v\":%.2f}", batteryV);
    ws.textAll(json);
}

// ── بث التيليمتري للشاشة (UDP broadcast على المنفذ 4220) ──────
//   يُرسل حتى لو ما فيه متصفح مفتوح — الشاشة جهاز مستقل.
//   حقول الإشعاع إضافة فوق البروتوكول القائم (لا تكسره).
void broadcastToDisplay() {
    IPAddress bcast(255, 255, 255, 255);
    float cpm = geigerFastCPM();
    float usvh = cpmToUsvh(cpm);
    RiskLevel lvl = riskClassify(usvh);
    char json[360];
    if (gpsFix()) {
        snprintf(json, sizeof(json),
            "{\"fix\":1,\"lat\":%.6f,\"lng\":%.6f,\"sat\":%d,"
            "\"spd\":%.2f,\"hdg\":%.1f,\"alt\":%.1f,\"bv\":%.2f,"
            "\"cpm\":%.1f,\"usvh\":%.3f,\"risk\":\"%s\",\"obs\":%.0f}",
            gpsLat(), gpsLng(), gpsSats(), gpsSpeedKmph(), imuHeading(), gpsAltM(),
            batteryV, cpm, usvh, riskName(lvl), obstacleDistanceCm());
    } else {
        snprintf(json, sizeof(json),
            "{\"fix\":0,\"sat\":%d,\"bv\":%.2f,"
            "\"cpm\":%.1f,\"usvh\":%.3f,\"risk\":\"%s\",\"obs\":%.0f}",
            gpsSats(), batteryV, cpm, usvh, riskName(lvl), obstacleDistanceCm());
    }
    udp.beginPacket(bcast, UDP_DISPLAY_PORT);
    udp.print(json);
    udp.endPacket();
}

// ═══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32 Rover Hub starting ===");

    // وحدات الحساسات (HAL) والطبقة الذكية
    imuInit();
    gpsInit();
    geigerInit();
    obstacleInit();
    if (!loggerInit()) Serial.println("[LOG] filesystem FAILED — continuing without logging");
    riskSafetyInit(roverSend);
    failsafeInit(roverSend);
    scanInit(roverSend);
    Serial.printf("[HAL] sim=%d geiger(%d,%d) imu=%d sd=%d gps=%d walk=%d\n",
                  SIMULATION_MODE, USE_REAL_GEIGER1, USE_REAL_GEIGER2,
                  USE_REAL_IMU, USE_REAL_SD, USE_REAL_GPS, SIM_GPS_WALK);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Connecting");
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
        delay(400); Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] FAILED — restarting in 5s");
        delay(5000); ESP.restart();
    }

    Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

    if (MDNS.begin(MDNS_NAME)) {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("[mDNS] http://%s.local\n", MDNS_NAME);
    }

    // OTA — رفع لاسلكي للتحديثات
    ArduinoOTA.setHostname(MDNS_NAME);
    // ArduinoOTA.setPassword("rover");   // فعّلها لو تبي حماية
    ArduinoOTA.onStart([](){ Serial.println("[OTA] update start"); });
    ArduinoOTA.onEnd([](){ Serial.println("[OTA] update done"); });
    ArduinoOTA.onError([](ota_error_t e){ Serial.printf("[OTA] err %u\n", e); });
    ArduinoOTA.begin();
    Serial.println("[OTA] ready");

    // UDP
    udp.begin(UDP_LOCAL_PORT);      // بورت محلي
    teleUdp.begin(UDP_TELE_PORT);   // استقبال البطارية من CAM

    // لورا HC-14 على UART2 — قناة تحكم يدوية أولوية قصوى (تعمل بلا WiFi)
    loraSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    Serial.printf("[LoRa] HC-14 على UART2 RX=%d TX=%d @ %d\n", LORA_RX_PIN, LORA_TX_PIN, LORA_BAUD);
    Serial.printf("[UDP] sender ready, target=%s.local:%d\n", CAM_HOST_NAME, UDP_CAM_CMD_PORT);
    Serial.printf("[UDP] telemetry listener on :%d\n", UDP_TELE_PORT);

    // محاولة أولى لـ resolve ESP32-CAM
    delay(1000);   // أعطي ESP32-CAM وقت لو في setup
    resolveCam();
    lastCamCheck = millis();

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){
        AsyncWebServerResponse *res = r->beginResponse_P(200, "text/html", HTML_PAGE);
        res->addHeader("Cache-Control", "no-cache");
        r->send(res);
    });

    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *r){
        String host;
        if (camResolved && camIP != IPAddress(0,0,0,0)) {
            host = camIP.toString();          // IP رقمي مؤكد
        } else {
            host = String(CAM_HOST_NAME) + ".local";   // fallback
        }
        // نعطي base فقط — الواجهة تبني /capture أو /stream حسب الوضع
        String json = "{\"camBase\":\"http://";
        json += host; json += ":"; json += String(CAM_STREAM_PORT); json += "\"}";
        r->send(200, "application/json", json);
    });

    // لقطة حالة كاملة — للاختبار والتشخيص (A2 ستبني الواجهة فوق نفس البيانات)
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *r){
        float cpm = geigerFastCPM();
        float usvh = cpmToUsvh(cpm);
        uint8_t cs, cg, ca, cm;
        imuGetCalibration(cs, cg, ca, cm);
        char json[820];
        snprintf(json, sizeof(json),
            "{\"cpm1\":%.1f,\"cpm2\":%.1f,\"cpm\":%.1f,\"cpm_raw\":%.1f,\"usvh\":%.3f,\"risk\":\"%s\","
            "\"fix\":%d,\"lat\":%.6f,\"lng\":%.6f,\"sat\":%d,\"hdg\":%.1f,"
            "\"bv\":%.2f,\"mission\":%d,\"scan\":%d,\"scanpct\":%d,\"calib\":%d,"
            "\"anom\":%d,\"mu\":%.1f,\"sigma\":%.1f,"
            "\"scan_dir\":%.1f,\"scan_conf\":%.2f,\"retreat\":%d,"
            "\"single\":%d,\"g1h\":%d,\"g1fault\":%d,\"imucal\":\"%d%d%d%d\",\"imutrust\":%d,"
            "\"pitch\":%.1f,\"roll\":%.1f,\"jam\":%d,\"tilt\":%d,"
            "\"sd\":%d,\"imu\":%d,\"sim\":%d,\"heap\":%u,"
            "\"obs\":%.0f,\"obl\":%d,\"obr\":%d,\"obh\":%d,\"obk\":%d}",
            geigerCPM1(), geigerCPM2(), cpm, geigerFastCPMRaw(), usvh, riskName(riskClassify(usvh)),
            gpsFix() ? 1 : 0, gpsLat(), gpsLng(), gpsSats(), imuHeading(),
            batteryV, reportMissionActive() ? 1 : 0,
            scanActive() ? 1 : 0, scanProgressPct(),
            anomalyCalibrating() ? anomalyCalibProgress(millis()) : (anomalyCalibrated() ? 100 : -1),
            anomalyActive() ? 1 : 0, anomalyMu(), anomalySigma(),
            scanResultDirDeg(), scanResultConf(), riskRetreatActive() ? 1 : 0,
            SINGLE_TUBE_MODE, geigerHealth(), geigerImplausible() ? 1 : 0, cs, cg, ca, cm,
            imuTrusted() ? 1 : 0, imuPitch(), imuRoll(),
            failsafeJam() ? 1 : 0, failsafeTilt() ? 1 : 0,
            loggerOk() ? 1 : 0, imuOk() ? 1 : 0, SIMULATION_MODE,
            (unsigned)ESP.getFreeHeap(),
            obstacleDistanceCm(), obstacleLeft() ? 1 : 0, obstacleRight() ? 1 : 0,
            obstacleHealth(), obstacleFrontBlocked() ? 1 : 0);
        r->send(200, "application/json", json);
    });

    // آخر تقرير مهمة محفوظ
    server.on("/api/report", HTTP_GET, [](AsyncWebServerRequest *r){
        if (loggerOk() && loggerReportPath()[0] != 0 && loggerFS().exists(loggerReportPath()))
            r->send(loggerFS(), loggerReportPath(), "application/json");
        else
            r->send(404, "application/json", "{\"err\":\"no report\"}");
    });

    // ملف CSV للمهمة الحالية/الأخيرة
    server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest *r){
        if (loggerOk() && loggerCsvPath()[0] != 0 && loggerFS().exists(loggerCsvPath()))
            r->send(loggerFS(), loggerCsvPath(), "text/csv");
        else
            r->send(404, "application/json", "{\"err\":\"no log\"}");
    });

    server.onNotFound([](AsyncWebServerRequest *r){
        r->send(404, "text/plain", "Not found");
    });

    server.begin();
    Serial.println("[HTTP] Server started on :80");
    Serial.println("=== Ready ===\n");
}

// ═══════════════════════════════════════════════════════════
void loop() {
    ArduinoOTA.handle();
    unsigned long now = millis();

    // ── تحديث الحساسات (كلها non-blocking) ──
    imuUpdate(now);
    gpsUpdate(now);
    geigerUpdate(now);

    // ── مزلاج الأمر اليدوي: كرّر آخر أمر حركة من حلقة الهَب الموثوقة ──
    //   يفصل استمرارية القيادة عن مؤقّت المتصفح (الذي يتلعثم تحت حمل الكاميرا).
    //   ينتهي تلقائياً إن توقّف المتصفح عن التحديث (أمان) → الأردوينو يتوقف.
    if (manualActive) {
        if (now - manualLastRxMs > MANUAL_LATCH_EXPIRE_MS) {
            manualActive = false;
            roverSend("S");                 // انقطع مصدر الأمر → توقف آمن
        } else if (now - manualLastFwdMs >= CMD_HEARTBEAT_MS) {
            manualLastFwdMs = now;
            roverSend(manualCmd);            // تكرار ثابت 250ms للكام
        }
    }

    // ── لورا: استقبل الأوامر (أولوية قصوى) + بث حالة مضغوطة ──
    loraReceive(now);
#if LORA_TX_STATUS
    // لا تبثّ الحالة إن وصل أمر يدوي حديثاً — القناة نصف-مزدوجة، فبثّ الهَب
    // يحجب استقبال الأوامر ويسبب تأخّر الاستجابة. الأولوية القصوى للتحكم اليدوي.
    if (now - lastLoraTxMs >= LORA_STATUS_MS &&
        (lastLoraRxMs == 0 || now - lastLoraRxMs > LORA_QUIET_AFTER_RX_MS)) {
        lastLoraTxMs = now;
        loraBroadcastStatus();
    }
#endif

    ws.cleanupClients();

    // ── استقبل تيليمتري الأردوينو من CAM عبر UDP (BV: بطارية / OBS: عوائق) ──
    int plen = teleUdp.parsePacket();
    if (plen > 0) {
        // الكام هي مصدر هذا الـ packet → احفظ IP الحقيقي (تعارف ذاتي)
        camIP = teleUdp.remoteIP();
        camResolved = true;
        camViaUdp = true;          // عرفنا الكام مباشرة → أوقف استعلامات mDNS الحاجزة
        lastCamRxMs = now;         // آخر إشارة حياة من الكام

        char buf[32];
        int n = teleUdp.read(buf, sizeof(buf)-1);
        if (n > 0) {
            buf[n] = 0;
            if (strncmp(buf, "BV:", 3) == 0) {
                batteryV = atof(buf + 3);
                lastBatteryTime = now;
            } else if (strncmp(buf, "OBS:", 4) == 0) {
                obstacleParse(buf + 4);   // "d,l,r" → حامل العوائق
            }
        }
    }

    // mDNS للإقلاع فقط: بمجرد وصول أول حزمة UDP من الكام نعرف IP-ها ونتوقف عن
    // الاستعلام الحاجز نهائياً (كان يجمّد الحلقة ~ثانيتين فيتوقف الروفر فجأة).
    if (!camViaUdp && now - lastCamCheck > 5000) {
        resolveCam();
        lastCamCheck = now;
    }

    // ── تشخيص حالة الاتصال على السيريال (كل ثانيتين) ──
    //   يوضّح: واي فاي الهَب، هل وصلت أي حزمة من الكام (وعمرها)، البطارية،
    //   طزاجة بيانات العوائق، عدد عملاء الويب، وآخر أمر لورا. لتشخيص "لا بيانات".
#if NET_DEBUG_SERIAL
    static unsigned long lastNetStatus = 0;
    if (now - lastNetStatus >= 2000) {
        lastNetStatus = now;
        Serial.printf(
            "[NET] wifi=%s ip=%s rssi=%d | cam=%s ip=%s rx_age=%s | batt=%.2fV | obs=%s | ws=%u | manual=%s\n",
            WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
            WiFi.localIP().toString().c_str(),
            (int)WiFi.RSSI(),
            camViaUdp ? "LINKED" : (camResolved ? "mDNS-only" : "UNKNOWN"),
            camResolved ? camIP.toString().c_str() : "-",
            lastCamRxMs ? String((now - lastCamRxMs) / 1000.0, 1).c_str() : "never",
            batteryV,
            obstacleFresh() ? "fresh" : "stale/none",
            (unsigned)ws.count(),
            manualActive ? manualCmd : "-");
    }
#endif

    // ── الطبقة الذكية المحلية ──
    float cpm = geigerFastCPM();
    float usvh = cpmToUsvh(cpm);
    RiskLevel lvl = riskClassify(usvh);
    bool geigerFault = geigerImplausible();   // قراءة مستحيلة = سلك مفصول/ضجيج

    // طبقة السلامة (خارج أي AI): تراجع تلقائي ~2م عند Critical + إنذار.
    // عند عطل الجيجر لا نبني القرار على الضجيج (لئلا يتراجع الروفر بلا سبب).
    riskSafetyUpdate(now, geigerFault ? RISK_SAFE : lvl);
    if (riskConsumeAlarm()) {
        char j[96];
        snprintf(j, sizeof(j), "{\"t\":\"alarm\",\"usvh\":%.2f,\"msg\":\"critical_retreat\"}", usvh);
        ws.textAll(j);
        Serial.printf("[SAFETY] CRITICAL %.2f uSv/h — auto retreat\n", usvh);
    }

    // طبقة السلامة الثانية (IMU): كشف الانحشار وحد الميل الخطير
    failsafeUpdate(now);
    const char* fsReason;
    if (failsafeConsumeAlarm(&fsReason)) {
        char j[96];
        snprintf(j, sizeof(j), "{\"t\":\"alarm\",\"msg\":\"failsafe_%s\"}", fsReason);
        ws.textAll(j);
        Serial.printf("[SAFETY] failsafe: %s (pitch=%.0f roll=%.0f acc=%.2f)\n",
                      fsReason, imuPitch(), imuRoll(), imuAccelMag());
    }

    // طبقة السلامة (عوائق): إيقاف طارئ عند عائق أمامي قريب — خارج أي AI.
    // مطفأة افتراضياً (OBSTACLE_ESTOP=0) حتى لا تقاطع القيادة اليدوية/الرجوع.
#if OBSTACLE_ESTOP
    {
        static bool obsBlockedPrev = false;
        bool blocked = obstacleFrontBlocked();
        if (blocked && !obsBlockedPrev) {          // حافة: نفّذ مرة واحدة
            roverSend("S");
            ws.textAll("{\"t\":\"alarm\",\"msg\":\"obstacle\"}");
            Serial.printf("[SAFETY] obstacle %.0fcm (L%d R%d) — stop\n",
                          obstacleDistanceCm(), obstacleLeft(), obstacleRight());
        }
        obsBlockedPrev = blocked;
    }
#endif

    // المسح الدوراني 360°
    scanUpdate(now);
    if (scanConsumeFinished()) {
        if (scanResultValid()) {
            reportSetScanResult(scanResultDirDeg(), scanResultConf());
            char j[96];
            snprintf(j, sizeof(j), "{\"t\":\"scan\",\"ok\":1,\"dir\":%.1f,\"conf\":%.2f}",
                     scanResultDirDeg(), scanResultConf());
            ws.textAll(j);
            Serial.printf("[SCAN] dir=%.1f conf=%.2f\n", scanResultDirDeg(), scanResultConf());
        } else {
            ws.textAll("{\"t\":\"scan\",\"ok\":0}");
            Serial.println("[SCAN] inconclusive");
        }
    }

    // منفّذ خطة الشات بوت (بعد المسح — يعتمد على اكتماله)
    planUpdate(now);

    // كشف الشذوذ: عينة مستقلة عند اكتمال كل دلو جيجر (10 ثوانٍ)
    if (geigerNewSample()) {
        anomalyUpdate(now, geigerInstantCPM());
        if (anomalyConsumeEvent()) {
            double lat, lng;
            gpsPosOrHome(lat, lng);
            float aUsvh = cpmToUsvh(geigerInstantCPM());
            char j[160];
            snprintf(j, sizeof(j),
                "{\"t\":\"anomaly\",\"cpm\":%.1f,\"usvh\":%.3f,\"lat\":%.6f,\"lng\":%.6f}",
                geigerInstantCPM(), aUsvh, lat, lng);
            ws.textAll(j);   // بث فوري
            reportTrack(lat, lng, gpsFix(), aUsvh, lvl, true);
            Serial.printf("[ANOMALY] %.1f CPM (thr %.1f)\n", geigerInstantCPM(), anomalyThreshold());
        }
    }

    // تسجيل CSV + تتبع التقرير + تدرّج الجرعة أثناء المهمة
    if (reportMissionActive() && now - lastLogWrite >= LOG_INTERVAL_MS) {
        lastLogWrite = now;
        double lat, lng;
        gpsPosOrHome(lat, lng);
        loggerLog(now, lat, lng, gpsFix(), geigerCPM1(), geigerCPM2(), cpm, usvh,
                  riskName(lvl), imuHeading(), anomalyActive());
        reportTrack(lat, lng, gpsFix(), usvh, lvl, false);
        if (gpsFix()) gradientAddPoint(lat, lng, usvh);
    }

    // ── البث الدوري ──
    if (now - lastGPSBroadcast >= GPS_BROADCAST_MS) {
        lastGPSBroadcast = now;
        broadcastGPS();
        broadcastToDisplay();   // بث للشاشة المستقلة (UDP 4220)
    }

    if (now - lastRadBroadcast >= RAD_BROADCAST_MS) {
        lastRadBroadcast = now;
        broadcastRad();
    }

    // بث البطارية للواجهة كل ثانيتين
    if (now - lastTeleBroadcast >= TELE_BROADCAST_MS) {
        lastTeleBroadcast = now;
        broadcastBattery();
    }
}
