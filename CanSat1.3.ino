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
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float Heading;
  String GPSLat;
  String GPSLon;
  String GPSDate;
  String GPSTime;
} struct_message;
struct_message myData;

uint8_t broadcastAddress[] = {0xe4, 0x65, 0xb8, 0xda, 0x73, 0x04};
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nESP-NOW Gönderim Durumu:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Başarılı" : "Başarısız");
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXPin, TXPin);
  Wire.begin();

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

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

    float accX = 0, accY = 0, accZ = 0;
    float gyrX = 0, gyrY = 0, gyrZ = 0;
    float headingDeg = 0;
    float temp = 0, pres = 0, altValue = 999;
    String gpsLat = "N/A", gpsLon = "N/A", gpsDate = "N/A", gpsTime = "N/A";
    String mpuTemp = "N/A";

    if (bmpOK) {
      temp = bmp.readTemperature();
      pres = bmp.readPressure();
      altValue = bmp.readAltitude(101500);
    }

    if (mpuOK) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      accX = a.acceleration.x;
      accY = a.acceleration.y;
      accZ = a.acceleration.z;
      gyrX = g.gyro.x;
      gyrY = g.gyro.y;
      gyrZ = g.gyro.z;
      mpuTemp = String(t.temperature, 2);
    }

    if (compassOK) {
      Vector mag = compass.readNormalize();
      float heading = atan2(mag.YAxis, mag.XAxis);
      float declination = (4.0 + 26.0 / 60.0) / (180 / M_PI);
      heading += declination;
      if (heading < 0) heading += 2 * PI;
      if (heading > 2 * PI) heading -= 2 * PI;
      headingDeg = heading * 180 / M_PI;
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

    // Buzzer mantığı
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

    // Verileri myData'ya yaz
    myData.Temperature = temp;
    myData.Pressure = pres;
    myData.Altitude = altValue;
    myData.ax = accX;
    myData.ay = accY;
    myData.az = accZ;
    myData.gx = gyrX;
    myData.gy = gyrY;
    myData.gz = gyrZ;
    myData.Heading = headingDeg;
    myData.GPSLat = gpsLat;
    myData.GPSLon = gpsLon;
    myData.GPSDate = gpsDate;
    myData.GPSTime = gpsTime;

    // ESP-NOW ile gönder
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    // CSV formatında veriyi hazırla
    String csv = "Time: " + String(millis()) + "ms\n" +
                 "Temperature: " + String(temp, 2) + " °C\n" +
                 "Pressure: " + String(pres, 2) + " Pa\n" +
                 "Altitude: " + String(altValue, 2) + " m\n" +
                 "AccelX: " + String(accX, 2) + " m/s^2\n" +
                 "AccelY: " + String(accY, 2) + " m/s^2\n" +
                 "AccelZ: " + String(accZ, 2) + " m/s^2\n" +
                 "GyroX: " + String(gyrX, 2) + " °/s\n" +
                 "GyroY: " + String(gyrY, 2) + " °/s\n" +
                 "GyroZ: " + String(gyrZ, 2) + " °/s\n" +
                 "MPU Temp: " + mpuTemp + " °C\n" +
                 "Heading: " + String(headingDeg, 2) + "°\n" +
                 "Latitude: " + gpsLat + "\n" +
                 "Longitude: " + gpsLon + "\n" +
                 "Date: " + gpsDate + "\n" +
                 "Time: " + gpsTime + "\n";

    Serial.println("Kayıt: \n" + csv);

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
