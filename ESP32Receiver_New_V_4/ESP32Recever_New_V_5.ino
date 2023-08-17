/**************************************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  Update: 05-23-2023

  The 2.0" TFT breakout
    ----> https://www.adafruit.com/product/4311

  Check out the links above for our tutorials and wiring diagrams.
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional).

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <Adafruit_BME280.h> // Temp & Humitidy Sensor
#include <SPI.h>
#include <time.h>


// For the breakout board, you can use any 2 or 3 pins.
// These pins will also work for the 1.8" TFT shield.
#define TFT_CS 5
#define TFT_RST 13 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC 6
#define TFT_MOSI 35 // Data out
#define TFT_SCLK 36 // Clock out
#define TFT_LED 18
#define SEALEVELPRESSURE_HPA (1000.25)
#define TEMPOFFSET 30 /* BEM280 Temp Offset for board temp */
#define CHANNEL 1
#define timezone -5 // US Eastern Time Zone

// Network stuff
const char *ssid = "Buttons";
const char *password = "KD23cusslerpit";
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
const boolean PREVENING=true;   //Big Bang theory.  https://www.urbandictionary.com/define.php?term=prevening
const boolean DISPLAY_DIGITAL=true;  //turn on displaying digital time after scrolling.
int timeinfo;

// I2C Configuration
Adafruit_BME280 bme; // BME280 Temp Sensor

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

typedef struct struct_message
{
  int id;
  float a; // Temp
  float b; // Humdity
  float c; // Pressuer
  float d; // Alt.
  float e; // Wind Speed
  float f; // Wind Direction
  int readingId;
} struct_message;

struct_message incomingReadings;
;

bool dataready;

float inhumidity;

int maxSpeed = 0;


// Clear Screen for new update info
void clearScreen()
{
  tft.fillScreen(ST77XX_BLACK);
}

// ==============================================================================
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// ==============================================================================

void printLocalTime() {
  struct tm timeinfo;
  static int values[2];
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
 
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor( 240, 105);
  if (timeinfo.tm_hour - 2 < 10) {
    tft.print("0");
    tft.print(timeinfo.tm_hour %24 - 2);
  } else {
    tft.print(timeinfo.tm_hour %24 - 2);
  }

  tft.print(":");

  
  if (timeinfo.tm_min < 10) {
    tft.print("0");
    tft.print(timeinfo.tm_min);
  } else {
    tft.print(timeinfo.tm_min);
  }
}


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len)
{
  // Serial.print("I just received data --> ");
  dataready = 1;
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  // clearScreen();
}

float windSpeedSensor(float windSpeed)
{
  int max_Speed = 0;
  Serial.print("Wind Speed received: ");
  Serial.println(windSpeed);
  float wind_speed = mapfloat(windSpeed, 0.47, 2.0, 0.0, 32.4);
  float speed_mph = ((wind_speed * 3600) / 1609.344);

  if (speed_mph <= 0.2)
  {
    speed_mph = 0.0;
  }

  // Display Wind Speed

 if (speed_mph <= 0.2)
  {
    speed_mph = 0.0;
  }
  max_Speed = maxSpeed;
  if (speed_mph > maxSpeed) {
    maxSpeed = speed_mph;
    max_Speed = speed_mph;
    tft.setTextSize(2);
    tft.setCursor(240, 125);
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.print(max_Speed, 1);
    printLocalTime();
  }
  // Display Wind Speed

  tft.setTextSize(4);
  tft.setCursor(90, 125);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(int(speed_mph));
  tft.print(" mph ");

  tft.setTextSize(2);
  tft.setCursor(240, 125);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.print(max_Speed, 1);

  return speed_mph;
}


void wind_direction(int winddir)
{
  String set_Direction[] = {"N ", "NE", "E ", "SE", "S ", "SW", "W ", "NW"};
  int i;
  int adc = winddir;

  adc = map(adc, 0, 8060, 0, 360);

  if ((adc >= 338) || (adc <= 22))
  {
    i = 0;
  }
  else if ((adc >= 23) && (adc <= 67))
  {
    i = 1;
  }
  else if ((adc >= 68) && (adc <= 112))
  {
    i = 2;
  }
  else if ((adc >= 113) && (adc <= 158))
  {
    i = 3;
  }
  else if ((adc >= 159) && (adc <= 202))
  {
    i = 4;
  }
  else if ((adc >= 203) && (adc <= 247))
  {
    i = 5;
  }
  else if ((adc >= 248) && (adc <= 292))
  {
    i = 6;
  }
  else if ((adc >= 293) && (adc <= 337))
  {
    i = 7;
  }

  // Display Wind Direction

  tft.setTextSize(8);

  tft.setCursor(118, 60);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  tft.print(set_Direction[i]);
}

// Display Temp and humidity reading outdoor sensor
void temp_sensor(float temp, float humidity, float outtemp, float outhumidity)
{
  tft.setTextSize(2);

  // Display Outdoor temp and humidity
  tft.setCursor(10, 198);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.print((incomingReadings.a * 1.05)+32, 1);
  tft.print(char(0xF7)); // Display temp degree symbol
  tft.print("F"); tft.print(" ");

  tft.setCursor(10, 220);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(incomingReadings.b, 1);
  tft.print(" %"); tft.print(" ");

  // Display Indoor temp and humidity

  tft.setCursor(240, 198);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
 
  tft.print((outtemp * 1.05)+32, 1);
  tft.print(char(0xF7)); // Display temp degree symbol
  tft.print("F");

  tft.setCursor(240, 220);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(outhumidity, 1);
  tft.print(" %");
}

// ================================================================
//                   END Subroutines
// ================================================================

void setup(void)
{
  Serial.begin(115200);
  delay(5000);
  // connect to WiFi Just long enough to get the current Time.
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");
  // init and get the time
  configTime(timezone * 3600, daylightOffset_sec, ntpServer);
  printLocalTime();

  if (!bme.begin(0x77, &Wire)) {
    //Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_AP);
  WiFi.softAP("RX_1", "RX_1_Password", CHANNEL, 0);

  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  // use this initializer (uncomment) if using a 2.0" 320x240 TFT:
  tft.init(240, 320); // Init ST7789 320x240

  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setCursor(65, 10);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.print("Weather Station");

  tft.setCursor(3, 170);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.print("Outdoor");
  tft.setCursor(240, 170);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.print("Indoor");

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(270, 125);
  tft.print("max");

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(270, 145);
  tft.print("mph");
    
  delay(1000);

}
void loop()
{
  // 
  bme.takeForcedMeasurement();  // has no effect in normal mode
  float boardintemp = bme.readTemperature();
  float boardinhumidity = bme.readHumidity();

  // Call Sensor Read Routines
  if (dataready == 1)
  {
    temp_sensor(incomingReadings.a, incomingReadings.b, boardintemp, boardinhumidity);
    windSpeedSensor(incomingReadings.e);
    wind_direction(incomingReadings.f);
    dataready = 0;
  }
  tft.setTextSize(2);
  tft.setCursor(135, 220);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print(incomingReadings.readingId); tft.print("   ");

}
