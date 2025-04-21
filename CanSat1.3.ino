#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <HMC5883L.h>
#include <esp_now.h>
#include <WiFi.h>

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;
HMC5883L compass;
TinyGPSPlus gps;
HardwareSerial mySerial(1); // GPS UART1

const int buzzerPin = 4;
const int SD_CS_PIN = 5;
const int RXPin = 16;
const int TXPin = 17;
float previousAltitude = -999;
bool buzzerActive = false;

unsigned long previousMillis = 0;
const long interval = 1000;

bool bmpOK = false;
bool mpuOK = false;
bool compassOK = false;
bool sdOK = false;

// ESP-NOW veri yapısı
typedef struct struct_message {
  float Temperature;
  float Altitude;
  float Pressure;
} struct_message;
struct_message myData;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nESP-NOW Gönderim Durumu:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Başarılı" : "Başarısız");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  mySerial.begin(9600, SERIAL_8N1, RXPin, TXPin);
  Wire.begin();

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // BMP085 başlat
  if (bmp.begin()) {
    bmpOK = true;
    Serial.println("BMP085 OK");
  } else {
    Serial.println("BMP085 BAŞARISIZ");
  }

  if (mpu.begin()) {
    mpuOK = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 BAŞARISIZ");
  }

  if (compass.begin()) {
    compassOK = true;
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_30HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
    compass.setOffset(0, 0, 0);
    Serial.println("HMC5883L OK");
  } else {
    Serial.println("HMC5883L BAŞARISIZ");
  }

  if (SD.begin(SD_CS_PIN)) {
    sdOK = true;
    if (!SD.exists("/flight_log.csv")) {
      File file = SD.open("/flight_log.csv", FILE_WRITE);
      file.println("OrBeet");
      file.close();
    }
    Serial.println("SD Kart OK");
  } else {
    Serial.println("SD Kart BAŞARISIZ");
  }

  // ESP-NOW Setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW başlatılamadı");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP-NOW alıcı eklenemedi");
    return;
  }

  Serial.println("Sistem başlatıldı.");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    String temperature = "N/A", pressure = "N/A", altitude = "N/A";
    String ax = "N/A", ay = "N/A", az = "N/A";
    String gx = "N/A", gy = "N/A", gz = "N/A";
    String mpuTemp = "N/A";
    String headingStr = "N/A";
    String gpsLat = "N/A", gpsLon = "N/A", gpsDate = "N/A", gpsTime = "N/A";

    float altValue = 999;
    if (bmpOK) {
      float temp = bmp.readTemperature();
      float pres = bmp.readPressure();
      altValue = bmp.readAltitude(101500);

      temperature = String(temp, 2);
      pressure = String(pres);
      altitude = String(altValue, 2);

      // ESP-NOW veri gönder
      myData.Temperature = temp;
      myData.Pressure = pres;
      myData.Altitude = altValue;
      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    }

    float altitudeDiff = abs(altValue - previousAltitude);
    if (previousAltitude != -999) {
      if (altitudeDiff < 0.5 && !buzzerActive) {
        tone(buzzerPin, 1000);
        buzzerActive = true;
        Serial.println(">>> İNİŞ TAMAMLANDI - BUZZER AKTİF <<<");
      } else if (altitudeDiff >= 0.05) {
        noTone(buzzerPin);
        buzzerActive = false;
      }
    }
    previousAltitude = altValue;

    if (mpuOK) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      ax = String(a.acceleration.x, 2);
      ay = String(a.acceleration.y, 2);
      az = String(a.acceleration.z, 2);
      gx = String(g.gyro.x, 2);
      gy = String(g.gyro.y, 2);
      gz = String(g.gyro.z, 2);
      mpuTemp = String(t.temperature, 2);
    }

    if (compassOK) {
      Vector mag = compass.readNormalize();
      float heading = atan2(mag.YAxis, mag.XAxis);
      float declination = (4.0 + 26.0 / 60.0) / (180 / M_PI);
      heading += declination;
      if (heading < 0) heading += 2 * PI;
      if (heading > 2 * PI) heading -= 2 * PI;
      headingStr = String(heading * 180 / M_PI, 2);
    }

    while (mySerial.available() > 0) gps.encode(mySerial.read());
    if (gps.location.isValid()) {
      gpsLat = String(gps.location.lat(), 6);
      gpsLon = String(gps.location.lng(), 6);
    }
    if (gps.date.isValid()) {
      gpsDate = String(gps.date.day()) + "-" + String(gps.date.month()) + "-" + String(gps.date.year());
    }
    if (gps.time.isValid()) {
      gpsTime = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
    }

    String csv = "Time: " + String(millis()) + "th millisecond\n" +
                 "Temperature: " + temperature + " °C\n" +
                 "Pressure: " + pressure + "Pa\n" +
                 "Altitude: " + altitude + "m\n" +
                 "AccelX: " + ax + "m/s^2\n" +
                 "AccelY: " + ay + "m/s^2\n" +
                 "AccelZ: " + az + "m/s^2\n" +
                 "GyroX: " + gx + "°/s\n" +
                 "GyroY: " + gy + "°/s\n" +
                 "GyroZ: " + gz + "°/s\n" +
                 "MPU Temp: " + mpuTemp + "°C\n" +
                 "Heading: " + headingStr + "°\n" +
                 "Latitude: " + gpsLat + "°\n" +
                 "Longitude: " + gpsLon + "°\n" +
                 "Date: " + gpsDate + "\n" +
                 "Time: " + gpsTime + "\n";

    Serial.println("Kayıt: " + csv);

    if (sdOK) {
      File file = SD.open("/flight_log.csv", FILE_APPEND);
      if (file) {
        file.println(csv);
        file.close();
      } else {
        Serial.println("SD yazma hatası!");
      }
    }
  }
}