/*
 * controller_display.ino — شاشة تحكم لمسية (STA فقط)
 * =================================================================
 * يعمل على: ESP32-3248S035R (افتراضي) أو ESP32-2432S028 — علم DISPLAY_MODEL
 * الترجمة: لوحة "ESP32 Dev Module" + إعداد TFT_eSPI من tft_setup/ (انظر README)
 *
 * المعمارية (بعد التوحيد):
 *   الشاشة **تتصل بنفس شبكة الهَب** (STA فقط، بيانات secrets.h) — لا تصنع
 *   شبكتها ولا تستضيف موقعاً. الموقع الشامل (خرائط/نطاق/شات بوت/قيادة) على
 *   الهَب وحده (rover.local) يُفتح من أي متصفح على نفس الشبكة.
 *   دور الشاشة: **وحدة تحكم لمسية فيزيائية** — تفتح WebSocket للهَب كعميل
 *   لعرض التيليمتري، وترسل القيادة عبر WS + LoRa (قناة أولوية بلا WiFi).
 *
 *   قاعدة الأمان: انقطعت الوصلتان (WS وLoRa) → تعطيل أزرار الحركة فوراً.
 *
 * المكتبات: TFT_eSPI + XPT2046_Touchscreen + WebSockets (arduinoWebSockets)
 *           + ArduinoJson + Preferences (+ WiFi/ESPmDNS مدمجة)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>   // حفظ معايرة اللمس في NVS
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#include "secrets.h"
#include "config.h"

// ── Globals ──────────────────────────────────────────────────
WebServer         server(80);        // موقع خفيف لجوالك (قيادة + قراءات)
DNSServer         dns;               // captive portal
WebSocketsServer  wsLocal(81);       // عملاء شبكة الشاشة (جوالك)
WebSocketsClient  wsHub;             // وصلتنا بالهَب (WiFi، لما قريب)

TFT_eSPI tft = TFT_eSPI();
SPIClass touchSpi(VSPI);
XPT2046_Touchscreen ts(TOUCH_CS_PIN, TOUCH_IRQ_PIN);
IPAddress apIP(192, 168, 4, 1);

// مستطيل زر لمسي على الـTFT — يُعرّف هنا (فوق أي دالة) لأن Arduino يولّد
// نماذج الدوال في رأس السكتش، فلو عُرّف لاحقاً لفشلت hit()/drawBtn().
struct Btn { int x, y, w, h; };
Btn bTab0, bTab1, bTab2, bTab3, bEstop, bUp, bDown, bLeft, bRight, bStop, bSpdBar;
Btn bCommBoth, bCommWifi, bCommLora;   // أزرار اختيار قناة التحكم (شاشة COMMS)

// قناة التحكم المختارة: 0 = كلاهما (تلقائي)، 1 = WiFi فقط، 2 = LoRa فقط
int commMode = 0;

// حالة الوصلة بالهَب
bool          staUp = false, mdnsUp = false;
bool          hubWsUp = false;
IPAddress     hubIP;
bool          hubResolved = false;
unsigned long lastHubMsgMs = 0, lastResolveMs = 0;

// لورا HC-14 (UART2): قناة تحكم/حالة احتياطية تعمل بلا WiFi
HardwareSerial loraSerial(2);
String        loraRxBuf;
unsigned long lastLoraRxMs = 0;      // آخر حالة LoRa من الهَب

// آخر تيليمتري (للـTFT — تحديث من متغيرات محفوظة، لا وميض)
float  dCpm = 0, dUsvh = 0, dBv = 0;
int    dLvl = 0, dSats = 0, dHdg = 0, dTilt = 0;
bool   dFix = false;
char   dRisk[12] = "—";
int    dG1h = 2, dSdh = 0;
bool   dSim = false;

// شاشة الرسم البياني: حلقة عيّنات CPM عبر الزمن (GRAPH_N/GRAPH_SAMPLE_MS في config.h)
uint16_t cpmRing[GRAPH_N];
int      cpmRingCount = 0;             // عدد العيّنات المخزّنة (يتشبّع عند GRAPH_N)
unsigned long lastGraphSample = 0;
bool     graphDirty = true;            // أُضيفت عيّنة → أعد رسم المنحنى

// القيادة اللمسية
char          driveDir = 'S';
int           drivePower = 70;
unsigned long driveLastHb = 0;

// TFT
int  scrW, scrH, uiScreen = 0;       // 0 = حالة، 1 = قيادة، 2 = قنوات، 3 = رسم بياني
int  HDR = 26, TABH = 32;            // ارتفاع شريط العنوان وشريط التبويبات (يُضبط في layoutButtons)
int  gaugeCx = 0, gaugeCy = 0, gaugeR = 0;   // هندسة العداد القوسي (شاشة الحالة)
unsigned long lastTftMs = 0, lastStatusMs = 0;
bool tftForceRedraw = true;

// قناة لورا حيّة؟ (استقبلنا حالة الهَب خلال المهلة)
bool loraLink() {
#if LORA_ASSUME_LINK
    return true;                                          // اختبار: قُد بلا اشتراط استقبال
#else
    return (millis() - lastLoraRxMs) < LORA_TIMEOUT_MS;
#endif
}
// قناة WiFi حيّة؟ (WS متصل ووصلت رسالة حديثاً)
bool wifiLink() { return hubWsUp && (millis() - lastHubMsgMs < HUB_TIMEOUT_MS); }

// الوصلة حية حسب القناة المختارة (commMode: 0 كلاهما، 1 WiFi، 2 LoRa)
bool hubLink() {
    if (commMode == 1) return wifiLink();
    if (commMode == 2) return loraLink();
    return wifiLink() || loraLink();
}

// ── موقع الشاشة الخفيف (PROGMEM، بلا CDN) — قيادة + قراءات فقط ──
//   للكاميرا/الخرائط/الشات بوت: افتح rover.local عند القرب (WiFi).
const char PAGE[] PROGMEM = R"H(<!DOCTYPE html><html lang="ar" dir="rtl"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>RMS Control</title><style>
:root{--bg:#070912;--pnl:#111827;--pnl2:#1a2236;--bd:#1f2a44;--tx:#e5e9f0;--mut:#7c8aa8;
--mars:#e5532b;--cy:#22d3ee;--good:#10b981;--bad:#ef4444;--warn:#f59e0b}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:var(--bg);color:var(--tx);font-family:-apple-system,"Segoe UI","Tajawal",sans-serif;min-height:100vh}
.top{display:flex;justify-content:space-between;align-items:center;padding:10px 14px;background:#0d1220cc;
border-bottom:1px solid var(--bd);position:sticky;top:0;flex-wrap:wrap;gap:6px}
.brand{font-weight:700;letter-spacing:.1em}.brand em{color:var(--mars);font-style:normal}
.pill{font-size:.68rem;padding:4px 10px;border-radius:999px;background:var(--pnl);border:1px solid var(--bd);color:var(--mut)}
.pill.ok{color:var(--good);border-color:#10b98155}.pill.bad{color:var(--bad);border-color:#ef444466}
#ban{display:none;margin:10px 14px;padding:9px;border-radius:9px;background:#ef444422;border:1px solid var(--bad);
color:var(--bad);font-weight:700;text-align:center;font-size:.82rem}#ban.show{display:block}
main{display:grid;gap:12px;padding:12px;max-width:560px;margin:auto}
.pnl{background:var(--pnl);border:1px solid var(--bd);border-radius:13px;padding:13px}
.pnl h2{font-size:.66rem;letter-spacing:.2em;color:var(--mut);margin-bottom:9px}
#dose{font-size:2.4rem;font-weight:700;font-family:ui-monospace,monospace;text-align:center}
#dose em{font-size:.85rem;color:var(--mut);font-style:normal}#risk{text-align:center;font-weight:600}
.lv0{color:var(--good)}.lv1{color:#a3e635}.lv2{color:var(--warn)}.lv3{color:#fb923c}.lv4{color:var(--bad)}
.row{display:flex;justify-content:space-between;font-size:.8rem;padding:4px 0;color:var(--mut)}
.row b{color:var(--tx);font-family:ui-monospace,monospace}
.cm{display:flex;gap:6px;margin-bottom:10px}.cm button{flex:1;background:var(--pnl2);border:1px solid var(--bd);
color:var(--mut);padding:7px;border-radius:8px;font-size:.72rem;cursor:pointer;font-family:inherit}
.cm button.on{background:#22d3ee18;border-color:var(--cy);color:var(--cy)}
.spd{display:flex;align-items:center;gap:10px;font-size:.75rem;color:var(--mut);margin-bottom:6px}.spd input{flex:1;accent-color:var(--mars)}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;max-width:240px;margin:6px auto}
.btn{aspect-ratio:1;background:var(--pnl2);border:1px solid var(--bd);color:var(--tx);border-radius:11px;
font-size:1.3rem;cursor:pointer;user-select:none;touch-action:none}.btn:active{background:#e5532b2e;border-color:var(--mars);color:var(--mars)}
.btn.st{color:var(--bad);border-color:#ef444444}.empty{visibility:hidden}
#dw.off{opacity:.35;pointer-events:none}
#estop{position:fixed;bottom:14px;left:14px;background:var(--bad);color:#fff;border:2px solid #fff3;border-radius:13px;
padding:12px 18px;font-size:.9rem;font-weight:700;cursor:pointer;box-shadow:0 4px 20px #ef444466}
.note{font-size:.68rem;color:var(--mut);text-align:center;margin-top:6px}
</style></head><body>
<div class="top"><div class="brand">RMS <em>CONTROL</em></div>
<div style="display:flex;gap:5px"><span class="pill" id="p-hub">HUB</span><span class="pill" id="p-ch">—</span></div></div>
<div id="ban">⚠ الهَب غير متصل — القيادة معطلة</div>
<main>
<section class="pnl"><h2>القناة</h2><div class="cm">
<button id="cm0" onclick="setCm(0)">تلقائي</button><button id="cm1" onclick="setCm(1)">WiFi</button><button id="cm2" onclick="setCm(2)">LoRa</button></div>
<div class="row"><span>الوصلة</span><b id="lnk">—</b></div></section>
<section class="pnl"><h2>الإشعاع</h2><div id="dose">—<em> µSv/h</em></div><div id="risk">—</div>
<div class="row"><span>CPM</span><b id="cpm">—</b></div><div class="row"><span>الميل من المستوي</span><b id="tilt">—°</b></div></section>
<section class="pnl"><h2>GPS</h2><div class="row"><span>Fix / أقمار</span><b id="fix">—</b></div>
<div class="row"><span>الاتجاه</span><b id="hdg">—°</b></div><div class="row"><span>الموقع</span><b id="pos">—</b></div></section>
<section class="pnl"><h2>القيادة</h2><div id="dw">
<div class="spd"><span>القوة</span><input type="range" id="pw" min="20" max="100" value="70" step="5"><b id="pwv">70</b></div>
<div class="dpad"><div class="empty"></div><button class="btn" data-d="F">▲</button><div class="empty"></div>
<button class="btn" data-d="L">◀</button><button class="btn st" data-d="S">■</button><button class="btn" data-d="R">▶</button>
<div class="empty"></div><button class="btn" data-d="B">▼</button><div class="empty"></div></div></div>
<div class="note">للكاميرا والخرائط والشات بوت: افتح <b>rover.local</b> عند قربك من الروبوت (WiFi)</div></section>
</main><button id="estop" onclick="send(JSON.stringify({c:'estop'}))">🛑 إيقاف طوارئ</button>
<script>
const $=i=>document.getElementById(i);let ws=null,hubOk=false;
function pill(id,s){const e=$(id);e.classList.remove('ok','bad');if(s)e.classList.add(s)}
function setHub(ok){hubOk=ok;pill('p-hub',ok?'ok':'bad');$('ban').classList.toggle('show',!ok);$('dw').classList.toggle('off',!ok)}
function connect(){ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onclose=()=>{setHub(false);setTimeout(connect,1500)};
ws.onmessage=e=>{try{const m=JSON.parse(e.data);
if(m.t==='disp'){setHub(m.hub===1);$('lnk').textContent=(m.wifi?'WiFi ':'')+(m.lora?'LoRa':'')||'مقطوع';
['cm0','cm1','cm2'].forEach((x,i)=>$(x).classList.toggle('on',m.cm===i));$('p-ch').textContent=['تلقائي','WiFi','LoRa'][m.cm]}
else if(m.t==='rad'){$('dose').innerHTML=(m.usvh||0).toFixed(3)+'<em> µSv/h</em>';$('dose').className='lv'+(m.lvl||0);
$('risk').textContent=m.risk||'—';$('risk').className='lv'+(m.lvl||0);if(m.cpm!==undefined)$('cpm').textContent=m.cpm.toFixed(1);
if(m.tiltlv!==undefined)$('tilt').textContent=m.tiltlv.toFixed(0)+'°'}
else if(m.t==='gps'){$('fix').textContent=(m.fix?'FIX / ':'لا / ')+(m.sat||0);if(m.fix){$('hdg').textContent=Math.round(m.hdg)+'°';$('pos').textContent=m.lat.toFixed(5)+', '+m.lng.toFixed(5)}}
}catch(_){}}}
function send(s){if(ws&&ws.readyState===1)ws.send(s)}
function setCm(n){send(JSON.stringify({cm:n}))}
const pw=$('pw');pw.oninput=()=>$('pwv').textContent=pw.value;let hb=null,cur=null;
function go(d){if(!hubOk&&d!=='S')return;cur=d;send(d==='S'?'S':d+pw.value);clearInterval(hb);if(d!=='S')hb=setInterval(()=>send(d+pw.value),500)}
function halt(){clearInterval(hb);hb=null;if(cur&&cur!=='S')send('S');cur=null}
document.querySelectorAll('.btn').forEach(b=>{const d=b.dataset.d;
const p=e=>{e.preventDefault();d==='S'?send('S'):go(d)};const r=e=>{if(e)e.preventDefault();if(d!=='S')halt()};
b.addEventListener('touchstart',p,{passive:false});b.addEventListener('touchend',r,{passive:false});
b.addEventListener('mousedown',p);b.addEventListener('mouseup',r);b.addEventListener('mouseleave',()=>halt())});
setHub(false);connect();
</script></body></html>)H";

// ── إرسال للهَب عبر القناتين: WiFi(WS) + LoRa(HC-14) ──────────
//   القناتان معاً: WS للزمن الحقيقي، ولورا احتياطي أولوية يعمل بلا WiFi.
//   الأمر متكرر على الطرفين لا يضرّ (آخر أمر يفوز، والحركة مُحدَّثة دورياً).
void sendToHub(const char* msg) {
    if (commMode != 2 && hubWsUp) wsHub.sendTXT(msg);   // WiFi (إلا في وضع LoRa فقط)
    if (commMode != 1) {                                 // LoRa (إلا في وضع WiFi فقط)
        loraSerial.print(msg);
        loraSerial.print('\n');        // فاصل السطر = إطار أمر على لورا
        static unsigned long lastTxLog = 0;
        if (millis() - lastTxLog > 500) { lastTxLog = millis(); Serial.printf("[LoRa TX] %s\n", msg); }
    }
}

// أمر حركة من أي مصدر (موقع الشاشة أو لمس TFT) — قاعدة أمان الجسر:
// لا حركة والهَب مقطوع (أمر متأخر في طابور = خطر). S يمرر دائماً.
bool forwardDrive(const char* cmd) {
    char d = cmd[0];
    if (d != 'S' && !hubLink()) return false;
    sendToHub(cmd);
    return true;
}

// ── أوامر جوالك (على موقع الشاشة) ────────────────────────────
void onLocalWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
    if (type != WStype_TEXT || len == 0) return;
    char d = (char)payload[0];
    String msg = String((char*)payload).substring(0, len);
    if (d == 'F' || d == 'B' || d == 'L' || d == 'R' || d == 'S') {
        if (!forwardDrive(msg.c_str()))
            wsLocal.sendTXT(num, "{\"t\":\"disp\",\"hub\":0}");
    } else if (d == '{') {
        if (msg.indexOf("\"cm\"") >= 0) {          // تغيير القناة محلياً على الشاشة
            StaticJsonDocument<64> j;
            if (!deserializeJson(j, msg) && j.containsKey("cm")) commMode = constrain((int)j["cm"], 0, 2);
        } else sendToHub(msg.c_str());             // estop وغيره → الهَب
    }
}

// ── Captive Portal: أي طلب مجهول → صفحتنا ───────────────────
void captiveRedirect() {
    server.sendHeader("Location", "http://" + apIP.toString() + "/", true);
    server.send(302, "text/plain", "");
}

// ── أحداث وصلة الهَب: مرآة للجوال + التقاط ما يحتاجه الـTFT ──
void onHubWsEvent(WStype_t type, uint8_t* payload, size_t len) {
    if (type == WStype_CONNECTED)    { hubWsUp = true;  lastHubMsgMs = millis(); Serial.println("[HUB] WS connected"); }
    else if (type == WStype_DISCONNECTED) { hubWsUp = false; Serial.println("[HUB] WS disconnected"); }
    else if (type == WStype_TEXT) {
        lastHubMsgMs = millis();
        wsLocal.broadcastTXT(payload, len);        // الجسر: مرآة التيليمتري لجوالك (WiFi)
        StaticJsonDocument<768> m;
        if (deserializeJson(m, payload, len)) return;
        const char* t = m["t"] | "";
        if (!strcmp(t, "rad")) {
            dCpm  = m["cpm"]  | 0.0f;
            dUsvh = m["usvh"] | 0.0f;
            dLvl  = m["lvl"]  | 0;
            strlcpy(dRisk, m["risk"] | "?", sizeof(dRisk));
            dG1h  = m["g1h"]  | 2;
            dSdh  = m["sdh"]  | 0;
            dSim  = (m["sim"] | 0) != 0;
            dTilt = (int)(m["tiltlv"] | 0.0f);
        } else if (!strcmp(t, "batt")) {
            dBv   = m["v"] | 0.0f;
        } else if (!strcmp(t, "gps")) {
            dFix  = m["fix"] | false;
            dSats = m["sat"] | 0;
            dHdg  = (int)(m["hdg"] | 0.0f);
        }
    }
}

// ============================================================
//  TFT — شاشتان: (0) حالة + E-STOP كبير، (1) قيادة لمسية
//  تحديث كل TFT_UPDATE_MS من متغيرات محفوظة (رسم المتغير فقط — لا وميض)
// ============================================================
// ── لوحة ألوان Mars (RGB565، مطابقة لثيم الويب) ──────────────
#define C_BG      0x0000   // خلفية سوداء
#define C_PANEL   0x10C4   // #111827
#define C_PANEL2  0x1906   // #1a2236
#define C_BORDER  0x1948   // #1f2a44
#define C_MARS    0xE2A5   // #e5532b
#define C_CYAN    0x269D   // #22d3ee
#define C_MUTED   0x7C55   // #7c8aa8
#define C_TEXT    0xE75E   // #e5e9f0
#define C_GOOD    0x15B0   // #10b981
#define C_WARN    0xF4E1   // #f59e0b
#define C_BAD     0xEA28   // #ef4444
#define C_TABACT  0x0902   // تعبئة التبويب النشط (سماوي غامق)
#define DEG2RAD   0.017453292f

uint16_t riskColor(int lvl) {
    switch (lvl) {
        case 0: return C_GOOD;
        case 1: return 0xA726;   // greenyellow #a3e635
        case 2: return C_WARN;
        case 3: return 0xFC87;   // orange #fb923c
        default: return C_BAD;
    }
}

// قوس ممتلئ بين نصفي قطر (مثلثات) — لرسم العداد بلا مكتبة إضافية
void fillArc(int cx, int cy, float startDeg, float sweepDeg, int rOut, int rIn, uint16_t color) {
    const float step = 3.0f;
    for (float a = startDeg; a < startDeg + sweepDeg - 0.01f; a += step) {
        float a2 = a + step; if (a2 > startDeg + sweepDeg) a2 = startDeg + sweepDeg;
        float c1 = cosf(a*DEG2RAD),  s1 = sinf(a*DEG2RAD);
        float c2 = cosf(a2*DEG2RAD), s2 = sinf(a2*DEG2RAD);
        int x0 = cx + rIn*c1,  y0 = cy + rIn*s1;
        int x1 = cx + rOut*c1, y1 = cy + rOut*s1;
        int x2 = cx + rOut*c2, y2 = cy + rOut*s2;
        int x3 = cx + rIn*c2,  y3 = cy + rIn*s2;
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, color);
        tft.fillTriangle(x0, y0, x2, y2, x3, y3, color);
    }
}

bool hit(const Btn& b, int x, int y) {
    return x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h;
}

void drawBtn(const Btn& b, const char* label, uint16_t border, uint16_t txt, uint8_t size) {
    tft.drawRoundRect(b.x, b.y, b.w, b.h, 8, border);
    tft.setTextColor(txt, TFT_BLACK);
    tft.setTextSize(size);
    int tw = strlen(label) * 6 * size;
    tft.setCursor(b.x + (b.w - tw) / 2, b.y + (b.h - 8 * size) / 2);
    tft.print(label);
}

// تبويب أنيق: النشط بتعبئة داكنة وحد سماوي، والخامل نص باهت
void drawTab(const Btn& b, const char* label, bool active) {
    tft.fillRoundRect(b.x + 2, b.y + 2, b.w - 4, b.h - 4, 6, active ? C_TABACT : C_BG);
    if (active) tft.drawRoundRect(b.x + 2, b.y + 2, b.w - 4, b.h - 4, 6, C_CYAN);
    tft.setTextSize(2);
    tft.setTextColor(active ? C_CYAN : C_MUTED, active ? C_TABACT : C_BG);
    int tw = strlen(label) * 12;
    tft.setCursor(b.x + (b.w - tw) / 2, b.y + (b.h - 16) / 2);
    tft.print(label);
}

void layoutButtons() {
    int W = scrW, H = scrH;
    HDR = 26; TABH = 32;
    int CY = HDR + TABH;                 // بداية منطقة المحتوى
    // أربعة تبويبات بعرض W/4 تحت شريط العنوان
    bTab0 = { 0,         HDR, W / 4, TABH };
    bTab1 = { W / 4,     HDR, W / 4, TABH };
    bTab2 = { 2 * W / 4, HDR, W / 4, TABH };
    bTab3 = { 3 * W / 4, HDR, W / 4, TABH };
    // العداد القوسي (شاشة الحالة): يسار المحتوى
    gaugeR  = min(88, (H - CY - 54) / 2);
    gaugeCx = max(gaugeR + 14, (int)(W * 0.26f));
    gaugeCy = CY + gaugeR + 10;
    // شاشة COMMS: ثلاثة أزرار اختيار القناة
    int cbW = (W - 40 - 20) / 3;
    bCommBoth = { 20,                CY + 44, cbW, 60 };
    bCommWifi = { 20 + cbW + 10,     CY + 44, cbW, 60 };
    bCommLora = { 20 + 2*(cbW + 10), CY + 44, cbW, 60 };
    // شاشة الحالة: E-STOP شريط سفلي كبير دائم
    bEstop = { 8, H - 50, W - 16, 42 };
    // شاشة القيادة: أسهم وسط الشاشة + شريط سرعة أسفل
    int cx = W / 2, cy = CY + (H - CY - 50) / 2, bs = min(W, H) / 4;
    bUp    = { cx - bs / 2,          cy - bs - 6,   bs, bs };
    bDown  = { cx - bs / 2,          cy + 6,        bs, bs };
    bLeft  = { cx - bs - bs / 2 - 8, cy - bs / 2,   bs, bs };
    bRight = { cx + bs / 2 + 8,      cy - bs / 2,   bs, bs };
    bStop  = { cx - bs / 2,          cy - bs / 2,   bs, bs };
    bSpdBar = { 20, H - 44, W - 40, 30 };
}

// قيم مرسومة سابقاً (لإعادة رسم المتغير فقط — لا وميض)
float pUsvh = -1; int pLvl = -1, pSats = -1, pClients = -1, pHdg = -1, pTilt = -999;
bool  pFix = false; int pPower = -1; char pDir = '?';
bool  pWsA = false, pLoA = false; int pBattPct = -999;   // شريط العنوان
bool  pCWsA = false, pCLoA = false; int pCMode = -1;      // شاشة COMMS
bool  pDriveHub = false;                                  // حالة الوصلة على شاشة القيادة

// نسبة شحن البطارية من الجهد (2×18650: ممتلئة 8.4V، فارغة 6.0V)
int battPct(float v) {
    if (v <= 0.1f) return -1;
    int p = (int)((v - 6.0f) / (8.4f - 6.0f) * 100.0f);
    return p < 0 ? 0 : (p > 100 ? 100 : p);
}

void drawCommButtons() {   // يُميَّز الوضع المختار بالسماوي
    for (int i = 0; i < 3; i++) {
        const Btn& b = (i==0) ? bCommBoth : (i==1) ? bCommWifi : bCommLora;
        bool on = (commMode == i);
        tft.fillRoundRect(b.x, b.y, b.w, b.h, 8, on ? C_TABACT : C_PANEL2);
        tft.drawRoundRect(b.x, b.y, b.w, b.h, 8, on ? C_CYAN : C_BORDER);
        tft.setTextSize(2);
        tft.setTextColor(on ? C_CYAN : C_MUTED, on ? C_TABACT : C_PANEL2);
        const char* lbl = (i==0) ? "BOTH" : (i==1) ? "WiFi" : "LoRa";
        int tw = strlen(lbl) * 12;
        tft.setCursor(b.x + (b.w - tw)/2, b.y + (b.h - 16)/2);
        tft.print(lbl);
    }
}

// عنوان الشاشة الثابت (يسار) — الجزء المتغيّر (بطارية/Wi/Lo) في tftTick
void drawHeaderStatic() {
    tft.fillRect(0, 0, scrW, HDR, C_BG);
    tft.setTextSize(2);
    tft.setTextColor(C_MARS, C_BG);
    tft.setCursor(6, (HDR - 16) / 2);
    tft.print("RAD ROVER");
}

void drawGraph();   // إعلان مسبق (شاشة الرسم البياني)

void drawStatic() {
    tft.fillScreen(C_BG);
    drawHeaderStatic();
    drawTab(bTab0, "STATUS", uiScreen == 0);
    drawTab(bTab1, "DRIVE",  uiScreen == 1);
    drawTab(bTab2, "COMMS",  uiScreen == 2);
    drawTab(bTab3, "GRAPH",  uiScreen == 3);
    int CY = HDR + TABH;
    if (uiScreen == 0) {
        // مسار العداد الرمادي (يُرسم مرة، والقيمة فوقه في tftTick)
        int rIn = gaugeR - max(10, gaugeR / 6);
        fillArc(gaugeCx, gaugeCy, 135, 270, gaugeR, rIn, C_PANEL2);
        tft.fillRoundRect(bEstop.x, bEstop.y, bEstop.w, bEstop.h, 8, C_BAD);
        tft.setTextSize(3); tft.setTextColor(TFT_WHITE, C_BAD);
        tft.setCursor(bEstop.x + (bEstop.w - 6*18)/2, bEstop.y + (bEstop.h - 24)/2);
        tft.print("E-STOP");
    } else if (uiScreen == 1) {
        drawBtn(bUp,    "^",    C_BORDER, C_TEXT, 3);
        drawBtn(bDown,  "v",    C_BORDER, C_TEXT, 3);
        drawBtn(bLeft,  "<",    C_BORDER, C_TEXT, 3);
        drawBtn(bRight, ">",    C_BORDER, C_TEXT, 3);
        drawBtn(bStop,  "STOP", C_BAD,    C_BAD,  2);
    } else if (uiScreen == 2) {
        tft.setTextColor(C_MUTED, C_BG); tft.setTextSize(2);
        tft.setCursor(20, CY + 12); tft.print("CONTROL CHANNEL");
        drawCommButtons();
        tft.setTextSize(1); tft.setTextColor(C_MUTED, C_BG);
        tft.setCursor(20, CY + 118); tft.print("WiFi / WebSocket:");
        tft.setCursor(20, CY + 158); tft.print("LoRa HC-14:");
    } else {
        tft.setTextColor(C_MUTED, C_BG); tft.setTextSize(2);
        tft.setCursor(20, CY + 6); tft.print("CPM HISTORY");
    }
    // أجبر إعادة رسم كل القيم المتغيّرة
    pUsvh = -1; pLvl = -1; pSats = -1; pClients = -1; pHdg = -1; pTilt = -999;
    pFix = !dFix; pPower = -1; pDir = '?'; pDriveHub = !hubLink();
    pWsA = !wifiLink(); pLoA = !loraLink(); pBattPct = -999;
    pCWsA = !wifiLink(); pCLoA = !loraLink(); pCMode = -1;
    graphDirty = true;
}

// صف قراءة معنون في عمود شاشة الحالة اليمنى
void statRow(int x, int y, const char* label, const char* val, uint16_t vcol) {
    tft.setTextSize(1); tft.setTextColor(C_MUTED, C_BG);
    tft.setCursor(x, y); tft.print(label);
    tft.setTextSize(2); tft.setTextColor(vcol, C_BG);
    tft.setCursor(x, y + 11); tft.print(val);
}

void tftTick() {
    bool wsA = wifiLink(), loA = loraLink();
    bool hub = hubLink();
    int  bp  = battPct(dBv);

    // ── شريط العنوان: بطارية + Wi/Lo (يمين) — يُعاد عند أي تغيّر ──
    if (wsA != pWsA || loA != pLoA || bp != pBattPct || tftForceRedraw) {
        pWsA = wsA; pLoA = loA; pBattPct = bp;
        int rx = scrW - 150;
        tft.fillRect(rx, 0, scrW - rx, HDR, C_BG);
        // بطارية
        tft.setTextSize(1);
        uint16_t bc = bp < 0 ? C_MUTED : (bp >= 50 ? C_GOOD : (bp >= 20 ? C_WARN : C_BAD));
        tft.setTextColor(bc, C_BG);
        tft.setCursor(rx, 3);
        if (bp < 0) tft.print("BATT --");
        else { tft.print(dBv, 1); tft.print("V "); tft.print(bp); tft.print("%"); }
        // Wi / Lo
        tft.setTextSize(2);
        tft.setTextColor(wsA ? C_GOOD : C_BAD, C_BG);
        tft.setCursor(scrW - 74, (HDR - 16)/2);  tft.print("Wi");
        tft.setTextColor(loA ? C_GOOD : C_BAD, C_BG);
        tft.setCursor(scrW - 34, (HDR - 16)/2);  tft.print("Lo");
    }

    if (uiScreen == 0) {
        // ── العداد القوسي + النص المركزي ──
        if (dUsvh != pUsvh || dLvl != pLvl || tftForceRedraw) {
            pUsvh = dUsvh; pLvl = dLvl;
            uint16_t rc = riskColor(dLvl);
            int rIn = gaugeR - max(10, gaugeR / 6);
            // خريطة لوغاريتمية 0.05 → 200 µSv/h على قوس 270°
            float lv = log10f(dUsvh < 0.01f ? 0.01f : dUsvh);
            float frac = (lv - log10f(0.05f)) / (log10f(200.0f) - log10f(0.05f));
            if (frac < 0) frac = 0; if (frac > 1) frac = 1;
            fillArc(gaugeCx, gaugeCy, 135, 270, gaugeR, rIn, C_PANEL2);         // مسار
            if (frac > 0) fillArc(gaugeCx, gaugeCy, 135, 270*frac, gaugeR, rIn, rc);  // قيمة
            // نص مركزي
            tft.fillCircle(gaugeCx, gaugeCy, rIn - 2, C_BG);
            char buf[12];
            dtostrf(dUsvh, 0, dUsvh < 10 ? 3 : 1, buf);
            tft.setTextSize(3); tft.setTextColor(rc, C_BG);
            tft.setCursor(gaugeCx - strlen(buf)*9, gaugeCy - 20); tft.print(buf);
            tft.setTextSize(1); tft.setTextColor(C_MUTED, C_BG);
            tft.setCursor(gaugeCx - 15, gaugeCy + 6); tft.print("uSv/h");
            tft.setTextSize(2); tft.setTextColor(rc, C_BG);
            int tw = strlen(dRisk) * 12;
            tft.setCursor(gaugeCx - tw/2, gaugeCy + 20); tft.print(dRisk);
        }
        // ── عمود القراءات اليمنى ──
        if (dFix != pFix || dSats != pSats || dHdg != pHdg || (int)dCpm != pClients
            || dTilt != pTilt || tftForceRedraw) {
            pFix = dFix; pSats = dSats; pHdg = dHdg; pClients = (int)dCpm; pTilt = dTilt;
            int rx = gaugeCx + gaugeR + 18;
            int CY = HDR + TABH;
            int colH = scrH - CY - 54;
            tft.fillRect(rx - 2, CY + 2, scrW - rx, colH, C_BG);
            int rowY = CY + 8, dy = colH / 4;
            char v[20];
            snprintf(v, sizeof(v), "%d", (int)dCpm);
            statRow(rx, rowY, "CPM", v, C_TEXT); rowY += dy;
            snprintf(v, sizeof(v), "%d deg", dTilt);
            statRow(rx, rowY, "TILT", v, dTilt > 25 ? C_BAD : C_TEXT); rowY += dy;
            snprintf(v, sizeof(v), "%s %d", dFix ? "FIX" : "no", dSats);
            statRow(rx, rowY, "GPS / SAT", v, dFix ? C_GOOD : C_WARN); rowY += dy;
            snprintf(v, sizeof(v), "%d deg / %s", dHdg,
                     commMode == 0 ? "AUTO" : commMode == 1 ? "WiFi" : "LoRa");
            statRow(rx, rowY, "HDG / CH", v, C_CYAN);
        }
    } else if (uiScreen == 1) {
        // ── شاشة القيادة: القوة + الاتجاه ──
        if (drivePower != pPower || tftForceRedraw) {
            pPower = drivePower;
            tft.drawRoundRect(bSpdBar.x, bSpdBar.y, bSpdBar.w, bSpdBar.h, 6, C_BORDER);
            int fillW = (bSpdBar.w - 4) * (drivePower - 20) / 80;
            tft.fillRect(bSpdBar.x + 2, bSpdBar.y + 2, bSpdBar.w - 4, bSpdBar.h - 4, C_BG);
            tft.fillRect(bSpdBar.x + 2, bSpdBar.y + 2, fillW, bSpdBar.h - 4, hub ? C_MARS : C_BORDER);
            tft.setTextSize(2); tft.setTextColor(TFT_WHITE, hub ? C_MARS : C_BG);
            tft.setCursor(bSpdBar.x + bSpdBar.w / 2 - 18, bSpdBar.y + 8);
            tft.print(drivePower);
        }
        if (driveDir != pDir || hub != pDriveHub || tftForceRedraw) {
            pDir = driveDir; pDriveHub = hub;
            int CY = HDR + TABH;
            tft.fillRect(0, CY + 4, scrW, 20, C_BG);
            tft.setTextSize(2);
            tft.setTextColor(hub ? C_GOOD : C_BAD, C_BG);
            tft.setCursor(8, CY + 4);
            tft.printf("DIR:%c  %s", driveDir, hub ? "LINK OK" : "NO LINK");
        }
    } else if (uiScreen == 2) {
        // ── شاشة COMMS ──
        if (wsA != pCWsA || loA != pCLoA || commMode != pCMode || tftForceRedraw) {
            pCWsA = wsA; pCLoA = loA; pCMode = commMode;
            int CY = HDR + TABH;
            drawCommButtons();
            tft.fillRect(20, CY + 134, scrW - 40, 18, C_BG);
            tft.setTextSize(2); tft.setTextColor(wsA ? C_GOOD : C_BAD, C_BG);
            tft.setCursor(20, CY + 134); tft.print(wsA ? "CONNECTED" : "DOWN");
            if (hubResolved) { tft.setTextColor(C_MUTED, C_BG); tft.print("  "); tft.print(hubIP.toString()); }
            tft.fillRect(20, CY + 174, scrW - 40, 18, C_BG);
            tft.setTextColor(loA ? C_GOOD : C_BAD, C_BG);
            tft.setCursor(20, CY + 174); tft.print(loA ? "ALIVE" : "SILENT");
            tft.fillRect(20, CY + 208, scrW - 40, 18, C_BG);
            tft.setTextColor(C_CYAN, C_BG);
            tft.setCursor(20, CY + 208);
            const char* mn = commMode == 0 ? "BOTH (auto)" : commMode == 1 ? "WiFi only" : "LoRa only";
            tft.print("Active: "); tft.print(mn); tft.print(hub ? "  [UP]" : "  [DOWN]");
        }
    } else {
        // ── شاشة الرسم البياني ──
        if (graphDirty || tftForceRedraw) { graphDirty = false; drawGraph(); }
    }
    tftForceRedraw = false;
}

// ── شاشة الرسم البياني: منحنى CPM عبر الزمن من حلقة العيّنات ──
void drawGraph() {
    int CY = HDR + TABH;
    int gx0 = 46, gy0 = CY + 30, gx1 = scrW - 12, gy1 = scrH - 26;
    int gw = gx1 - gx0, gh = gy1 - gy0;
    tft.fillRect(gx0 - 44, gy0 - 4, scrW - (gx0 - 44) - 6, gh + 30, C_BG);
    tft.drawRect(gx0, gy0, gw, gh, C_BORDER);
    // مقياس رأسي تلقائي
    int mx = 30;
    for (int i = 0; i < cpmRingCount; i++) if (cpmRing[i] > mx) mx = cpmRing[i];
    tft.setTextSize(1);
    for (int i = 0; i <= 2; i++) {
        int y = gy0 + gh * i / 2;
        tft.drawFastHLine(gx0, y, gw, C_PANEL);
        tft.setTextColor(C_MUTED, C_BG);
        tft.setCursor(6, y - 3); tft.print(mx * (2 - i) / 2);
    }
    // المنحنى (index 0 = الأقدم → gx0، الأحدث → gx1)
    if (cpmRingCount >= 2) {
        int prevX = -1, prevY = 0;
        for (int i = 0; i < cpmRingCount; i++) {
            int x = gx0 + gw * i / (GRAPH_N - 1);
            int y = gy1 - (int)((long)cpmRing[i] * gh / mx);
            if (y < gy0) y = gy0;
            if (prevX >= 0) tft.drawLine(prevX, prevY, x, y, C_CYAN);
            prevX = x; prevY = y;
        }
    } else {
        tft.setTextColor(C_MUTED, C_BG); tft.setTextSize(2);
        tft.setCursor(gx0 + 20, gy0 + gh/2 - 8); tft.print("collecting...");
    }
    tft.setTextSize(1); tft.setTextColor(C_MUTED, C_BG);
    tft.setCursor(gx0, gy1 + 6); tft.print("older");
    tft.setCursor(gx1 - 20, gy1 + 6); tft.print("now");
}

// إضافة عيّنة CPM لحلقة الرسم البياني (تُزاح عند الامتلاء)
void graphPush(int v) {
    if (v < 0) v = 0; if (v > 65535) v = 65535;
    if (cpmRingCount < GRAPH_N) cpmRing[cpmRingCount++] = (uint16_t)v;
    else { memmove(cpmRing, cpmRing + 1, (GRAPH_N - 1) * sizeof(uint16_t)); cpmRing[GRAPH_N - 1] = (uint16_t)v; }
    graphDirty = true;
}

// لمس → أزرار (مع مهلة استقرار بسيطة)
// يقرأ اللمس ويرجّع إحداثيات الشاشة (بعد الاتجاه) — true = ملموس.
// موديل 1 (3.5"): لمس TFT_eSPI على ناقل الشاشة المشترك (tft.getTouch).
// موديل 2 (2.8"): XPT2046 على ناقل مستقل.
bool readTouch(int &x, int &y) {
    bool touched = false;
    int rx = 0, ry = 0, rz = 0;
    x = 0; y = 0;
#if DISPLAY_MODEL == 1
    // لمس مُعاير عبر TFT_eSPI — يرجّع إحداثيات الشاشة مباشرةً (الاتجاه مضبوط بالمعايرة)
    uint16_t tx = 0, ty = 0;
    rz = tft.getTouchRawZ();
    touched = tft.getTouch(&tx, &ty, TOUCH_PRESSURE_TH);
    rx = tx; ry = ty;
    x = tx; y = ty;
#else
    touched = ts.touched();
    if (touched) {
        TS_Point p = ts.getPoint();
        rx = p.x; ry = p.y; rz = p.z;
        int aX = p.x, aY = p.y;
  #if TOUCH_SWAP_XY
        int t = aX; aX = aY; aY = t;
  #endif
        x = map(aX, TOUCH_RAW_MIN, TOUCH_RAW_MAX, 0, scrW);
        y = map(aY, TOUCH_RAW_MIN, TOUCH_RAW_MAX, 0, scrH);
    #if TOUCH_INVERT_X
        x = scrW - 1 - x;
    #endif
    #if TOUCH_INVERT_Y
        y = scrH - 1 - y;
    #endif
    }
#endif
    if (touched) { x = constrain(x, 0, scrW - 1); y = constrain(y, 0, scrH - 1); }
    (void)rx; (void)ry; (void)rz;   // (بقايا تشخيص اللمس — أُزيلت الطباعة)
    return touched;
}

void handleTouch() {
    static unsigned long lastTouchMs = 0;
    int x = 0, y = 0;
    if (!readTouch(x, y)) {
        // أُفلت إصبع القيادة → توقف فوري (لا أمر معلقاً)
        if (uiScreen == 1 && driveDir != 'S') { driveDir = 'S'; forwardDrive("S"); }
        return;
    }

    unsigned long now = millis();
    // التبويبات وE-STOP وأزرار القناة: بمهلة نقر (وليس استمراراً)
    if (now - lastTouchMs > 250) {
        if (hit(bTab0, x, y) && uiScreen != 0) { uiScreen = 0; drawStatic(); lastTouchMs = now; return; }
        if (hit(bTab1, x, y) && uiScreen != 1) { uiScreen = 1; drawStatic(); lastTouchMs = now; return; }
        if (hit(bTab2, x, y) && uiScreen != 2) { uiScreen = 2; drawStatic(); lastTouchMs = now; return; }
        if (hit(bTab3, x, y) && uiScreen != 3) { uiScreen = 3; drawStatic(); lastTouchMs = now; return; }
        if (uiScreen == 0 && hit(bEstop, x, y)) {
            sendToHub("{\"c\":\"estop\"}"); sendToHub("S");
            lastTouchMs = now; return;
        }
        if (uiScreen == 2) {   // اختيار قناة التحكم
            int nm = -1;
            if      (hit(bCommBoth, x, y)) nm = 0;
            else if (hit(bCommWifi, x, y)) nm = 1;
            else if (hit(bCommLora, x, y)) nm = 2;
            if (nm >= 0 && nm != commMode) {
                commMode = nm;
                drawCommButtons();          // تمييز فوري للمختار
                Serial.printf("[COMM] mode=%d\n", commMode);
            }
            if (nm >= 0) { lastTouchMs = now; return; }
        }
    }
    if (uiScreen == 1) {
        if (hit(bSpdBar, x, y)) {
            drivePower = constrain(20 + (x - bSpdBar.x) * 80 / bSpdBar.w, 20, 100);
            drivePower = (drivePower / 5) * 5;
            return;
        }
        char d = 0;
        if      (hit(bUp, x, y))    d = 'F';
        else if (hit(bDown, x, y))  d = 'B';
        else if (hit(bLeft, x, y))  d = 'L';
        else if (hit(bRight, x, y)) d = 'R';
        else if (hit(bStop, x, y))  d = 'S';
        if (d) {
            char c[8];
            if (d == 'S') strcpy(c, "S");
            else snprintf(c, sizeof(c), "%c%d", d, drivePower);
            // أمر جديد يُرسل فوراً؛ المضغوط يتجدد أبطأ على لورا (يقلل التكدّس والتأخير)
            unsigned long hbMs = (commMode == 2) ? LORA_DRIVE_HB_MS : DRIVE_HEARTBEAT_MS;
            if (d != driveDir || now - driveLastHb >= hbMs) {
                if (forwardDrive(c)) { driveDir = d; driveLastHb = now; }
            }
        }
    }
}

// ============================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n== RMS Controller Display (B2) ==");

    // TFT + إضاءة خلفية PWM (الافتراضي 60%)
    tft.init();
    tft.setRotation(1);                       // أفقي
    scrW = tft.width(); scrH = tft.height();
    pinMode(TFT_BL_PIN, OUTPUT);
    analogWrite(TFT_BL_PIN, 255 * TFT_BACKLIGHT_PCT / 100);

    // اللمس: موديل 2 (2.8") على ناقل SPI مستقل عبر XPT2046_Touchscreen.
    // موديل 1 (3.5") يشارك ناقل الشاشة ويُقرأ عبر tft.getTouch().
#if DISPLAY_MODEL == 2
    touchSpi.begin(TOUCH_CLK_PIN, TOUCH_MISO_PIN, TOUCH_MOSI_PIN, TOUCH_CS_PIN);
    ts.begin(touchSpi);
    ts.setRotation(1);
#else
    // معايرة لمس TFT_eSPI — تُحفظ في NVS وتُطبَّق تلقائياً بعد أول مرة.
    // أول تشغيل (أو TOUCH_FORCE_CALIBRATE=1): المس العلامات في الأركان.
    {
        uint16_t calData[5];
        Preferences prefs;
        prefs.begin("disp", false);
        bool haveCal = (prefs.getBytesLength("touchcal") == sizeof(calData));
    #if TOUCH_FORCE_CALIBRATE
        haveCal = false;
    #endif
        if (haveCal) {
            prefs.getBytes("touchcal", calData, sizeof(calData));
            tft.setTouch(calData);
            Serial.println("[TOUCH] loaded saved calibration");
        } else {
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextSize(2);
            tft.setCursor(10, 10);
            tft.println("Touch each corner");
            tft.setCursor(10, 40);
            tft.println("marker in turn");
            tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 20);
            prefs.putBytes("touchcal", calData, sizeof(calData));
            Serial.printf("[TOUCH] calibrated & saved: %u %u %u %u %u\n",
                          calData[0], calData[1], calData[2], calData[3], calData[4]);
        }
        prefs.end();
    }
#endif

    layoutButtons();
    drawStatic();

    // AP+STA: نبثّ شبكة الشاشة لجوالك، ونتصل بشبكة الهَب (secrets) لما قريب
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[AP] '%s' على %s | STA→'%s'\n", AP_SSID, apIP.toString().c_str(), WIFI_SSID);

    // Captive portal + الموقع الخفيف
    dns.setErrorReplyCode(DNSReplyCode::NoError);
    dns.start(DNS_PORT, "*", apIP);
    server.on("/", []() { server.send_P(200, "text/html", PAGE); });
    server.on("/generate_204",        captiveRedirect);
    server.on("/gen_204",             captiveRedirect);
    server.on("/hotspot-detect.html", captiveRedirect);
    server.on("/ncsi.txt",            captiveRedirect);
    server.onNotFound(captiveRedirect);
    server.begin();
    wsLocal.begin();
    wsLocal.onEvent(onLocalWsEvent);

    // وصلة الهَب (WiFi) تُفتح بعد resolve (في اللوب)
    wsHub.onEvent(onHubWsEvent);
    wsHub.setReconnectInterval(3000);
    wsHub.enableHeartbeat(15000, 3000, 2);

    // لورا HC-14 على UART2 — قناة تحكم/حالة أولوية تعمل بلا WiFi
    loraSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    Serial.printf("[LoRa] HC-14 RX=%d TX=%d @ %d\n", LORA_RX_PIN, LORA_TX_PIN, LORA_BAUD);

    Serial.println("[DISPLAY] ready — AP+STA gateway");
}

// ── استقبال حالة الهَب عبر لورا (تعمل بلا WiFi) ──────────────
void loraReceive() {
    while (loraSerial.available()) {
        char c = loraSerial.read();
        if (c == '\n' || c == '\r') {
            if (loraRxBuf.length() > 0) {
                StaticJsonDocument<256> m;
                if (!deserializeJson(m, loraRxBuf) && m.containsKey("cpm")) {
                    lastLoraRxMs = millis();
                    dCpm  = m["cpm"]  | 0.0f;
                    dUsvh = m["usvh"] | 0.0f;
                    dLvl  = m["lvl"]  | 0;
                    strlcpy(dRisk, m["risk"] | "?", sizeof(dRisk));
                    dHdg  = (int)(m["hdg"] | 0.0f);
                    dFix  = (m["fix"] | 0) != 0;
                    // WS مقطوع (بعيد): مرّر الحالة لجوالك كـrad+gps مبسّطين
                    bool wsAlive = hubWsUp && (millis() - lastHubMsgMs < HUB_TIMEOUT_MS);
                    if (!wsAlive) {
                        char j[176];
                        snprintf(j, sizeof(j),
                          "{\"t\":\"rad\",\"cpm\":%.0f,\"usvh\":%.3f,\"risk\":\"%s\",\"lvl\":%d,\"single\":1,\"tiltlv\":%d}",
                          dCpm, dUsvh, dRisk, dLvl, (int)(m["tiltlv"] | 0));
                        wsLocal.broadcastTXT(j);
                        snprintf(j, sizeof(j), "{\"t\":\"gps\",\"fix\":%d,\"hdg\":%d,\"sat\":0}", dFix ? 1 : 0, dHdg);
                        wsLocal.broadcastTXT(j);
                    }
                }
            }
            loraRxBuf = "";
        } else {
            loraRxBuf += c;
            if (loraRxBuf.length() > 120) loraRxBuf = "";
        }
    }
}

// ============================================================
void loop() {
    unsigned long now = millis();

    dns.processNextRequest();
    server.handleClient();
    wsLocal.loop();
    wsHub.loop();
    loraReceive();

    // إدارة جهة STA: عند أول اتصال شغّل mDNS، ثم resolve الهَب دورياً
    if (WiFi.status() == WL_CONNECTED) {
        if (!staUp) {
            staUp = true;
            Serial.printf("[STA] IP: %s\n", WiFi.localIP().toString().c_str());
            if (MDNS.begin("rms-display")) mdnsUp = true;
        }
        if (!hubResolved && now - lastResolveMs > HUB_RESOLVE_MS) {
            lastResolveMs = now;
            IPAddress ip = mdnsUp ? MDNS.queryHost(HUB_MDNS_NAME) : IPAddress(0, 0, 0, 0);
            if (ip != IPAddress(0, 0, 0, 0)) {
                hubIP = ip;
                hubResolved = true;
                Serial.printf("[HUB] resolved %s.local -> %s\n", HUB_MDNS_NAME, ip.toString().c_str());
                wsHub.begin(hubIP.toString(), HUB_WS_PORT, "/ws");
            } else Serial.println("[HUB] mDNS resolve failed, retrying...");
        }
    } else staUp = false;

    handleTouch();

    // عيّنة CPM لحلقة الرسم البياني (من آخر تيليمتري محفوظ)
    if (now - lastGraphSample >= GRAPH_SAMPLE_MS) {
        lastGraphSample = now;
        graphPush((int)dCpm);
    }

    // بث حالة الجسر لجوالك (يفعّل/يعطّل أزراره + القناة + حالة الوصلة)
    if (now - lastStatusMs >= LOCAL_STATUS_MS) {
        lastStatusMs = now;
        char j[112];
        snprintf(j, sizeof(j), "{\"t\":\"disp\",\"hub\":%d,\"wifi\":%d,\"lora\":%d,\"cm\":%d}",
                 hubLink() ? 1 : 0, wifiLink() ? 1 : 0, loraLink() ? 1 : 0, commMode);
        wsLocal.broadcastTXT(j);
    }

    // تحديث TFT كل 250ms من المتغيرات المحفوظة
    if (now - lastTftMs >= TFT_UPDATE_MS) {
        lastTftMs = now;
        tftTick();
    }
}
