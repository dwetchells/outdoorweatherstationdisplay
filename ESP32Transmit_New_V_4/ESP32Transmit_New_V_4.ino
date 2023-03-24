#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_VEML7700.h"

#define CHANNEL 1
#define SEALEVELPRESSURE_HPA (1000.25)
#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60 /* Time ESP32 will go to sleep (in seconds) */
#define TEMPOFFSET 20   /* BEM280 Temp Offset for board temp */   


esp_now_peer_info_t slave;

uint8_t data = 0;
float myTemp = 0;
unsigned int readingId = 0;
unsigned long delayTime;

// I2C Configuration
Adafruit_BME280 bme;  // BME280 Temp Sensor
Adafruit_VEML7700 veml = Adafruit_VEML7700();  // Light sensor to shut station down at night

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

typedef struct struct_message {
  int id;
  float a;  // Temp
  float b;  // Humdity
  float c;  // Pressuer
  float d;  // Alt.
  float e;  // Wind Speed
  float f;  // Wind Direction
  int readingId;
} struct_message;

struct_message myData;

void setup() {
  //Serial.begin(115200);
  //Serial.println(F("BME280 test"));

  if (!bme.begin(0x77, &Wire)) {
    //Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

    // Setup Light Sensor for Sleep
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

//Serial.println("Adafruit VEML7700 Test");

  if (!veml.begin()) {
    //Serial.println("Sensor not found");
    while (true) {
        pixels.setPixelColor(0, pixels.Color(10, 0, 10));
        pixels.show();
        delay(delayTime);
    }
      
  }
  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);

  //Serial.println("-- Default Test --");
  //Serial.println("normal mode, 16x oversampling for all, filter off,");
  //Serial.println("0.5ms standby period");

  // Setup GPIO pins for wind sensors and power relay.
  pinMode(12, OUTPUT);  // Relay Set
  pinMode(13, OUTPUT);  // Relay UnSet

  pinMode(A4, INPUT);  // Wind Speed
  pinMode(A2, INPUT);  // Wind Dir

  delayTime = 5000;  
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);
  ScanForSlave();
  esp_now_add_peer(&slave);

  // Reset Relay  
  digitalWrite(12, 1);
  delay(500);
  digitalWrite(12, 0);
  delay(500);
  digitalWrite(13, 1);
  delay(500);
  digitalWrite(13, 0);

  //Serial.println();
}

void loop() {
  // Only needed in forced mode! In normal mode, you can remove the next line.
  bme.takeForcedMeasurement();  // has no effect in normal mode

  getWindDir();
  getWindSpeed();
  getBME280();

  myData.readingId = readingId++;

  esp_now_send(slave.peer_addr, (uint8_t *)&myData, sizeof(myData));
  //data++;
  
  pixels.setPixelColor(0, pixels.Color(0, 10, 0));
  pixels.show();
  delay(delayTime);
}

/** Scan for slaves in AP mode and add as peer if found **/
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();

  for (int i = 0; i < scanResults; ++i) {
    String SSID = WiFi.SSID(i);
    String BSSIDstr = WiFi.BSSIDstr(i);

    if (SSID.indexOf("RX") == 0) {

      int mac[6];
      if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5], &mac[6])) {
        for (int ii = 0; ii < 6; ++ii) {
          slave.peer_addr[ii] = (uint8_t)mac[ii];
        }
      }

      slave.channel = CHANNEL;
      slave.encrypt = 0;
      break;
    }
  }
}

/** callback when daa is sent from Master to Slave **/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("I sent my data --> ");
  //Serial.println(readingId);
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  LighSensor();
}

void getWindSpeed() {
  float voltage = 0;
  float new_voltage = 0;
  for (int i = 0; i <= 49; i++) {
    float voltage = analogRead(A4);
    if (voltage == 0) {
      i--;
    }
    new_voltage = voltage + new_voltage;
    // Serial.println(new_voltage);
    delay(20);
  }
  new_voltage = new_voltage / 50;
  // Serial.print("New_Voltage : ");
  //Serial.print("Wind Speed Voltage average: ");
 // Serial.println(new_voltage);

  float windSpeedRaw = (new_voltage / 8190) * 3;
  myData.e = windSpeedRaw;
  //Serial.print("WindSpeedRaw : ");
  //Serial.println(myData.e);
}

void getWindDir() {
  float windDirRaw = analogRead(A2);
  myData.f = windDirRaw;
  //Serial.print("Wind Direction : ");
  //Serial.println(myData.f);
}

void getBME280() {
  //Serial.print("Temperature = ");
  //Serial.print(bme.readTemperature());
  myData.a = ((bme.readTemperature() * 1.8 + 32) - TEMPOFFSET);
  //Serial.print(myData.a);
  //Serial.println(" *F");

  //Serial.print("Humidity = ");
  //Serial.print(bme.readHumidity());
  myData.b = bme.readHumidity();
  //Serial.print(myData.b);
  //Serial.println(" %");

  //Serial.print("Pressure = ");
  //Serial.print(bme.readPressure() / 100.0F);
  myData.c = bme.readPressure() / 100.0F;
  //Serial.print(myData.c);
  //Serial.println(" hPa");

  //Serial.print("Approx. Altitude = ");
  //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA * 3.28));
  myData.d = bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.28;
  //Serial.print(myData.d);
  //Serial.println(" feet");
}

void LighSensor() {
  //Serial.print("lux: ");
  //Serial.println(veml.readLux());

  uint16_t irq = veml.interruptStatus();
  if (irq & VEML7700_INTERRUPT_LOW) {
    //Serial.println("** Low threshold");
  }
  if (irq & VEML7700_INTERRUPT_HIGH) {
    //Serial.println("** High threshold");
  }
  if (veml.readLux() <= 1.5) {
    digitalWrite(12, 1);
    delay(500);
    digitalWrite(12, 0);

    pixels.setPixelColor(0, pixels.Color(10, 0, 0));
    pixels.show();
    delay(500);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();

    //Serial.println("Going to sleep now");
    esp_deep_sleep_start();
    }
}