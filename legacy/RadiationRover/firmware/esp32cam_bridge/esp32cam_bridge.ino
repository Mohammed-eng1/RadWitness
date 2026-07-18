/*
 * esp32cam_bridge.ino — ESP32-CAM Stream + UDP Bridge + Battery
 * =============================================================
 * مبني على النسخة المبسّطة الشغّالة (cam:true).
 *
 * ★ شِيلنا المشتبه بهم: ★
 *   - تعطيل brownout (WRITE_PERI_REG) — كان يخرب استقرار الكاميرا
 *   - ArduinoOTA — كان يزاحم الكاميرا على الموارد
 *
 * ★ أبقينا الضروري للروبوت: ★
 *   - بث الكاميرا (بورت 81)
 *   - UDP: أوامر داخلة (4210) → الأردوينو عبر Serial
 *   - UDP: البطارية والعوائق خارجة (4212) ← من الأردوينو (BV: / OBS:)
 *
 * Board: AI Thinker ESP32-CAM | Core: 2.0.17
 */

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

#include "secrets.h"   // WIFI_SSID / WIFI_PASSWORD — خارج نطاق git

const char* MDNS_NAME     = "rovercam";
const int   UDP_CMD_PORT  = 4210;
const int   UDP_TELE_PORT = 4212;

// AI Thinker pins
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

httpd_handle_t streamServer = NULL;
WiFiUDP udp;       // استقبال الأوامر فقط (منفذ 4210)
WiFiUDP txUdp;     // إرسال التيليمتري فقط (منفذ 4212) — سوكِت منفصل حتى لا
                   // يعطّل الإرسالُ المتكرر (OBS كل 200ms) استقبالَ الأوامر
IPAddress hubIP;
bool hubKnown = false;
float lastBV = 0.0;

esp_err_t camInitErr = ESP_FAIL;
int camSensorPID = -1;
bool camInitOk = false;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    if (psramFound()) {
        config.jpeg_quality = 10;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    camInitErr = esp_camera_init(&config);
    if (camInitErr != ESP_OK) { camInitOk = false; return false; }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        camSensorPID = s->id.PID;
        s->set_framesize(s, FRAMESIZE_VGA);
        // قلب الصورة (الكاميرا مركّبة مقلوبة على الروبوت)
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
        // تحسين الجودة والإضاءة
        s->set_brightness(s, 1);      // أنصع (-2..2)
        s->set_contrast(s, 1);
        s->set_saturation(s, 0);
        s->set_gain_ctrl(s, 1);       // AGC تلقائي
        s->set_exposure_ctrl(s, 1);   // AEC تلقائي
        s->set_aec2(s, 1);            // تحسين التعريض في الإضاءة الخافتة
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_lenc(s, 1);            // تصحيح العدسة
        s->set_bpc(s, 1);
        s->set_wpc(s, 1);
    }
    camInitOk = true;
    return true;
}

esp_err_t streamHandler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    struct timeval _timestamp;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char part_buf[128];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            res = ESP_FAIL;
        } else {
            _timestamp.tv_sec = fb->timestamp.tv_sec;
            _timestamp.tv_usec = fb->timestamp.tv_usec;
            if (fb->format != PIXFORMAT_JPEG) {
                bool ok = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb); fb = NULL;
                if (!ok) res = ESP_FAIL;
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, 128, _STREAM_PART, _jpg_buf_len, (int)_timestamp.tv_sec, (int)_timestamp.tv_usec);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, (const char*)_jpg_buf, _jpg_buf_len);
        if (fb) { esp_camera_fb_return(fb); fb = NULL; _jpg_buf = NULL; }
        else if (_jpg_buf) { free(_jpg_buf); _jpg_buf = NULL; }
        if (res != ESP_OK) break;
    }
    return res;
}

esp_err_t captureHandler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

esp_err_t healthHandler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char resp[256];
    camera_fb_t *fb = esp_camera_fb_get();
    bool camOk = (fb != NULL);
    size_t flen = camOk ? fb->len : 0;
    if (fb) esp_camera_fb_return(fb);
    snprintf(resp, sizeof(resp),
        "{\"cam\":%s,\"framelen\":%u,\"initOk\":%s,\"pid\":\"0x%x\",\"psram\":%s,\"bv\":%.2f,\"heap\":%u}",
        camOk ? "true" : "false", flen, camInitOk ? "true" : "false",
        camSensorPID, psramFound() ? "true" : "false", lastBV, ESP.getFreeHeap());
    return httpd_resp_send(req, resp, strlen(resp));
}

void startServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 4;

    httpd_uri_t stream_uri = {.uri="/stream", .method=HTTP_GET, .handler=streamHandler, .user_ctx=NULL};
    httpd_uri_t capture_uri = {.uri="/capture", .method=HTTP_GET, .handler=captureHandler, .user_ctx=NULL};
    httpd_uri_t health_uri = {.uri="/health", .method=HTTP_GET, .handler=healthHandler, .user_ctx=NULL};

    if (httpd_start(&streamServer, &config) == ESP_OK) {
        httpd_register_uri_handler(streamServer, &stream_uri);
        httpd_register_uri_handler(streamServer, &capture_uri);
        httpd_register_uri_handler(streamServer, &health_uri);
    }
}

void setup() {
    Serial.begin(115200);   // للأردوينو

    if (!initCamera()) {
        pinMode(4, OUTPUT);
        while (true) { digitalWrite(4, HIGH); delay(100); digitalWrite(4, LOW); delay(100); }
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) delay(400);

    if (WiFi.status() != WL_CONNECTED) {
        pinMode(4, OUTPUT);
        while (true) { digitalWrite(4, HIGH); delay(800); digitalWrite(4, LOW); delay(800); }
    }

    MDNS.begin(MDNS_NAME);
    MDNS.addService("http", "tcp", 81);

    udp.begin(UDP_CMD_PORT);        // استقبال الأوامر على 4210
    txUdp.begin(UDP_CMD_PORT + 3);  // منفذ محلي منفصل للإرسال (4213) — لا يمسّ الاستقبال
    startServer();

    pinMode(4, OUTPUT);
    for (int i=0;i<2;i++){ digitalWrite(4,HIGH); delay(150); digitalWrite(4,LOW); delay(150); }
}

char udpBuf[24];
String arduinoLine = "";

void loop() {
    // 1) استلام UDP (أوامر) → الأردوينو
    int len = udp.parsePacket();
    if (len > 0) {
        hubIP = udp.remoteIP();
        hubKnown = true;
        int n = udp.read(udpBuf, sizeof(udpBuf) - 1);
        if (n > 0) {
            udpBuf[n] = 0;
            char* p = udpBuf;
            while (*p==' '||*p=='\r'||*p=='\n'||*p=='\t') p++;
            char* e = p + strlen(p);
            while (e > p && (e[-1]==' '||e[-1]=='\r'||e[-1]=='\n')) { e--; *e=0; }
            if (strlen(p) > 0) {
                char dir = p[0];
                if (dir=='F'||dir=='B'||dir=='L'||dir=='R'||dir=='S') {
                    Serial.print(p);
                    Serial.print('\n');
                    // تشخيص: بدّل حالة الفلاش عند كل أمر — يومض ~2Hz أثناء القيادة
                    // (الأوامر تصل كل 250ms) = دليل مرئي أن الكام تتلقى أوامر الهَب.
                    static bool camLed = false;
                    camLed = !camLed;
                    digitalWrite(4, camLed ? HIGH : LOW);
                }
            }
        }
    }

    // 2) استلام من الأردوينو (BV:x.xx / OBS:d,l,r) → UDP للـ hub
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            arduinoLine.trim();
            // الوجهة: بعد معرفة الهَب أرسل له مباشرة، وقبلها broadcast على
            // الشبكة (الهَب يلتقطه ويتعلم IP الكاميرا — تعارف ذاتي بالاتجاهين)
            IPAddress local = WiFi.localIP();
            IPAddress dest = hubKnown ? hubIP
                                      : IPAddress(local[0], local[1], local[2], 255);
            if (arduinoLine.startsWith("BV:")) {
                lastBV = arduinoLine.substring(3).toFloat();
                char msg[24];
                snprintf(msg, sizeof(msg), "BV:%.2f", lastBV);
                txUdp.beginPacket(dest, UDP_TELE_PORT);   // سوكِت الإرسال المنفصل
                txUdp.print(msg);
                txUdp.endPacket();
            } else if (arduinoLine.startsWith("OBS:")) {
                // مرّر سطر العوائق كما هو — الهَب يحلّله
                txUdp.beginPacket(dest, UDP_TELE_PORT);   // سوكِت الإرسال المنفصل
                txUdp.print(arduinoLine);
                txUdp.endPacket();
            }
            arduinoLine = "";
        } else if (c != '\r') {
            arduinoLine += c;
            if (arduinoLine.length() > 30) arduinoLine = "";
        }
    }
}
