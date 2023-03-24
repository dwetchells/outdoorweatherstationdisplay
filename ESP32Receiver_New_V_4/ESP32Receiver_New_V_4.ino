#include <esp_now.h>
#include <WiFi.h>

//TFT Libraries
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <SPI.h>
// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#define CHANNEL 1

// Define a data structure
typedef struct struct_message {
  int id;
  float a;  // Temp
  float b;  // Humdity
  float c;  // Pressure
  float d;  // ALt
  float e;  // Windspeed
  float f;  // Wind Direction
  unsigned int readingId;
} struct_message;
// Create a structured object
struct_message incomingReadings;
bool dataready;

void setup() {
  Serial.begin(115200);

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  tft.init(135, 240);  // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  uint16_t time = millis();
  tft.fillScreen(ST77XX_BLACK);
  time = millis() - time;

  delay(1000);
  // Set ESP32 as a Wi-Fi Statio

  WiFi.mode(WIFI_AP);
  WiFi.softAP("RX_1", "RX_1_Password", CHANNEL, 0);

  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {

  // Call Sensor Read Routines
  if (dataready == 1) {
    temp_sensor(incomingReadings.a, incomingReadings.b, incomingReadings.c, incomingReadings.d, incomingReadings.readingId);
    windSpeedSensor(incomingReadings.e);
    wind_direction(incomingReadings.f);
    dataready = 0;
  }
  delay(1000);
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  //Serial.print("I just received data --> ");
  dataready = 1;
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  clearScreen();
}

float windSpeedSensor(float windSpeed) {
  Serial.print("Wind Speed received: ");
  Serial.println(windSpeed);
  float wind_speed = mapfloat(windSpeed, 0.47, 2.0, 0.0, 32.4);
  //float speed_mph = ((wind_speed ));
  float speed_mph = ((wind_speed * 3600) / 1609.344);

  if (speed_mph <= 0.2) {
    speed_mph = 0.0;
  }

  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.print("Wind Speed: ");
  tft.print(int(speed_mph));
  tft.print(" mph ");

  //windspeed = 0;

  return speed_mph;
}

char *wind_direction(int winddir) {
  char *set_Direction[] = { "North", "North-East", "East", "South-East", "South", "South-West", "West", "North-West" };
  int i;
  int adc = winddir;
  Serial.print("Wind Direction: ");
  Serial.println(winddir);
  adc = map(adc, 0, 8060, 0, 360);
  Serial.print("Wind Pointing ");
  Serial.println(adc);
  tft.setCursor(0, 20);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.print("Wind Dir: ");
  if ((adc >= 338) || (adc <= 22)) {
    i = 0;
  } else if ((adc >= 23) && (adc <= 67)) {
    i = 1;
  } else if ((adc >= 68) && (adc <= 112)) {
    i = 2;
  } else if ((adc >= 113) && (adc <= 158)) {
    i = 3;
  } else if ((adc >= 159) && (adc <= 202)) {
    i = 4;
  } else if ((adc >= 203) && (adc <= 247)) {
    i = 5;
  } else if ((adc >= 248) && (adc <= 292)) {
    i = 6;
  } else if ((adc >= 293) && (adc <= 337)) {
    i = 7;
  }
  tft.print(set_Direction[i]);
  return set_Direction[i];
}


// ==============================================================================
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// ==============================================================================

void temp_sensor(float temp, float humidity, float pressure, float alt, int id) {
  //Serial.println(" ");
  //Serial.println(temp);
  //Serial.println(humidity);
  //Serial.println(pressure);
  //Serial.println(alt);
  //Serial.print(" Record : ");
  //Serial.println(id);

  // Display info to TFT screen
  tft.setCursor(0, 60);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(2);
  tft.print("Temp: ");
  tft.print(temp);
  tft.print(" F");

  tft.setCursor(0, 80);
  tft.print("Humdity: ");
  tft.print(humidity);
  tft.print(" %");

  tft.setCursor(0, 100);
  tft.print("Pressure: ");
  tft.print(pressure);
  tft.print(" hPa");

  tft.setCursor(0, 120);
  tft.print("Alt: ");
  tft.print(alt);
  tft.print(" Feet");
}


// Clear Screen for new update info
void clearScreen() {
  tft.fillScreen(ST77XX_BLACK);
}