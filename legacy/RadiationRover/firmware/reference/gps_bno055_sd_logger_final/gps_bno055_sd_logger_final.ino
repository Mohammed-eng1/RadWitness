/* ============================================================
 *  ESP32-S3 (N16R8) — GPS + BNO055 + MicroSD Logger  (نهائي)
 *  ------------------------------------------------------------
 *  BNO055 : SDA = 21 , SCL = 9    (I2C)  ← SCL نُقل من 20 إلى 9
 *           السبب: على S3 الطرف GPIO20 = USB D+ فلا يعمل I2C
 *           لمّا اللوحة موصولة عبر USB المدمج.
 *  GPS    : ESP32 RX = GPIO16     (سلك GPS-TX -> 16) ، نقرأ فقط
 *  SD     : SCK=12  MOSI=11  MISO=13  CS=10  (SPI)
 *
 *  المكتبات: Adafruit BNO055 + Adafruit Unified Sensor +
 *            TinyGPSPlus + SD/SPI/Wire (مدمجة)
 * ============================================================ */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPS++.h>
#include <math.h>

// ---------------- الأطراف (Pins) ----------------
#define I2C_SDA   21
#define I2C_SCL   9         // ← كان 20 (USB D+). لا تُرجعه إلى 20.

#define GPS_RX    16
#define GPS_TX    -1
#define GPS_BAUD  9600

#define SD_SCK    12
#define SD_MOSI   11
#define SD_MISO   13
#define SD_CS     10

// ---------------- BNO055 ----------------
#define BNO_STABILITY_TH  0.05f
Adafruit_BNO055* bno = nullptr;      // يُنشأ بعد اكتشاف العنوان تلقائياً
bool  bnoOK      = false;
float bnoHeading = 0;
bool  bnoStable  = true;

// ---------------- GPS ----------------
TinyGPSPlus    gps;
HardwareSerial GPSserial(1);

// ---------------- SD ----------------
SPIClass    spiSD(FSPI);
bool        sdOK = false;
const char* LOG_PATH = "/log.csv";

unsigned long lastLog = 0;

// يمسح ناقل I2C ويرجّع عنوان BNO (0x28 أو 0x29) أو 0 إذا غير موجود
uint8_t i2cFindBNO(){
  Serial.print("I2C scan:");
  uint8_t addr = 0;
  for(byte a = 1; a < 127; a++){
    Wire.beginTransmission(a);
    if(Wire.endTransmission() == 0){
      Serial.printf(" 0x%02X", a);
      if(a == 0x28 || a == 0x29) addr = a;
    }
  }
  Serial.println(addr ? "" : "  (لا شيء! تحقق SDA=21 / SCL=9 / التغذية 3V3)");
  return addr;
}

void sdWriteHeaderIfNew(){
  if(!SD.exists(LOG_PATH)){
    File f = SD.open(LOG_PATH, FILE_WRITE);
    if(f){
      f.println("millis,fix,sats,lat,lng,alt_m,speed_kmh,gps_course,heading,stable");
      f.close();
    }
  }
}

void setup(){
  Serial.begin(115200);
  delay(300);
  Serial.println("\n== ESP32-S3 GPS + BNO055 + SD Logger ==");

  // --- I2C + BNO055 ---
  Wire.begin(I2C_SDA, I2C_SCL);
  uint8_t addr = i2cFindBNO();
  if(addr){
    bno = new Adafruit_BNO055(55, addr, &Wire);
    bnoOK = bno->begin();
    if(bnoOK){ delay(50); bno->setExtCrystalUse(true); Serial.printf("BNO055 OK @ 0x%02X\n", addr); }
    else       Serial.println("BNO ظاهر على الناقل لكن begin() فشل");
  } else {
    Serial.println("BNO055 not found");
  }

  // --- GPS UART ---
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS UART started on GPIO16");

  // --- SD (SPI مخصّص) ---
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOK = SD.begin(SD_CS, spiSD);
  if(sdOK){ Serial.println("SD OK"); sdWriteHeaderIfNew(); }
  else       Serial.println("SD init FAILED — تحقق الأسلاك/التغذية/FAT32");
}

void loop(){
  // اقرأ ما يصل من الـ GPS باستمرار
  while(GPSserial.available()) gps.encode(GPSserial.read());

  // اقرأ BNO055 (اتجاه + ثبات)
  if(bnoOK){
    sensors_event_t oriEv, gyroEv;
    bno->getEvent(&oriEv,  Adafruit_BNO055::VECTOR_EULER);
    bno->getEvent(&gyroEv, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bnoHeading = oriEv.orientation.x;
    if(bnoHeading < 0) bnoHeading += 360;
    float gx = gyroEv.gyro.x, gy = gyroEv.gyro.y, gz = gyroEv.gyro.z;
    bnoStable = sqrtf(gx*gx + gy*gy + gz*gz) < BNO_STABILITY_TH;
  }

  // سجّل صفّاً كل ثانية
  if(millis() - lastLog >= 1000){
    lastLog = millis();

    bool   fix  = gps.location.isValid();
    double lat  = fix ? gps.location.lat() : 0.0;
    double lng  = fix ? gps.location.lng() : 0.0;
    double alt  = gps.altitude.isValid()   ? gps.altitude.meters() : 0.0;
    double spd  = gps.speed.isValid()      ? gps.speed.kmph()      : 0.0;
    double crs  = gps.course.isValid()     ? gps.course.deg()      : 0.0;
    int    sats = gps.satellites.isValid() ? gps.satellites.value(): 0;

    Serial.printf("bno=%d fix=%d sats=%d lat=%.6f lng=%.6f alt=%.1f hdg=%.1f stable=%d\n",
                  bnoOK, fix, sats, lat, lng, alt, bnoHeading, bnoStable);

    if(sdOK){
      File f = SD.open(LOG_PATH, FILE_APPEND);
      if(f){
        f.printf("%lu,%d,%d,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%d\n",
                 millis(), fix, sats, lat, lng, alt, spd, crs, bnoHeading, bnoStable);
        f.close();
      }
    }
  }
}
