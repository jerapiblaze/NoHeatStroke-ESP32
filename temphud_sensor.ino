/*
NoHeatStroke-ESP32
Author: j12t (jerapiblannett)
References:
- [Basic http request -- randomnerdtutorials.com](https://randomnerdtutorials.com/esp32-http-get-post-arduino/)
- [Format C string -- delftstack.com](https://www.delftstack.com/howto/c/c-string-formatting/)
- [Heat index levels -- weather.gov](https://www.weather.gov/arx/heat_index)
- [Connect 1602 led using I2C module to esp32 -- randomnerdtutorials.com](https://randomnerdtutorials.com/esp32-esp8266-i2c-lcd-arduino-ide/)
- [ESP32 with DHT11/DHT22 -- randomnerdtutorials.com](https://randomnerdtutorials.com/esp32-dht11-dht22-temperature-humidity-sensor-arduino-ide/)
- [ESP32 sleep modes -- deepbluembedded.com](https://deepbluembedded.com/esp32-sleep-modes-power-consumption/)
- [ESP32 external interrupts pins -- deepbluembedded.com](https://deepbluembedded.com/esp32-external-interrupts-pins-arduino-examples/)
- [UTF-8 character with 1602 LCD -- LiquidCrystal_I2C_UTF8 (locple)](https://github.com/locple/LiquidCrystal_I2C_UTF8)
- [HeatIndex Calculator -- calculator.net](https://www.calculator.net/heat-index-calculator.html)
- [The HeatIndex Equation -- ncep.noaa.gov](http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml)
*/
#include <WiFi.h>
#include "DHT.h"
#include <esp_sleep.h>
#include "driver/adc.h"
#include <Wire.h>
#include <LCDI2C_Multilingual.h>
#include <HTTPClient.h>

#define DEVIVE_ID "j12t_esp32_0" // max 16
#define SERVER_SECRET "123456"
#define SERVER_ENDPOINT "http://192.168.137.1:3000/api/v1/sensor0"

#define STA_SSID "Wrioteatime"
#define STA_PASS "chai-tea"

#define DHTPIN 25
#define DHTTYPE DHT11 

#define PSDA 33
#define PSDL 32
#define LCD_ADDR 0x27
#define LCD_W 16
#define LCD_H 2

#define A_KEY 34
#define PBUSY 

#define PSAFE 13
#define PWARN 12
#define PDANG 14
#define PEXTR 27
#define PEEXT 26

DHT dht(DHTPIN, DHTTYPE);
LCDI2C_Latin_Symbols lcd (LCD_ADDR, LCD_W, LCD_H); 
HTTPClient http;
WiFiClient client;

RTC_DATA_ATTR volatile int _n_samples = 5;
RTC_DATA_ATTR volatile float _humi = 0;
RTC_DATA_ATTR volatile float _temp = 0;
RTC_DATA_ATTR volatile float _hind = 0;
volatile bool will_sleep = false;

void ReadSensor(){
  // Read _n_samples times to get maximum _n_samples
  // If any sample is corrupted, ignore it. 
  int i = 0;
  while(i < _n_samples){
    delay(2000);
    i++;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t)){
      Serial.println("[SENS] FAIL TO READ DATA!");
      continue;
    }
    if (_humi == 0 || _temp == 0){
      _humi = h;
      _temp = t;
    } else {
      _humi = (_humi+h)/2;
      _temp = (_temp+t)/2;
    }
    _hind = dht.computeHeatIndex(_temp, _humi, false);
  }
  return;
}

void DisplayValues(){
  Serial.printf("[SENS] TEMP=%5.2f HUMID=%5.2f HEAT=%5.2f COND=", _temp, _humi, _hind);
  lcd.setCursor(0,0); // faster than clear because we use all of the cells anyway
  lcd.printf("%6.2f", _temp);
  lcd.print("°C ");
  lcd.printf("%6.2f", _humi);
  lcd.print("%");
  lcd.println();
  lcd.printf("%6.2fH ", _hind);
  // Heat index levels
  if (_hind < 27){ // SAFE
    digitalWrite(PEEXT, LOW);
    digitalWrite(PEXTR, LOW);
    digitalWrite(PDANG, LOW);
    digitalWrite(PWARN, LOW);
    digitalWrite(PSAFE, HIGH);
    Serial.printf("NORMAL");
    lcd.println("  NORMAL");
  }
  if (_hind >= 27){ // CAUTION
    digitalWrite(PEEXT, LOW);
    digitalWrite(PEXTR, LOW);
    digitalWrite(PDANG, LOW);
    digitalWrite(PWARN, HIGH);
    digitalWrite(PSAFE, LOW);
    Serial.println("CAUTION");
    lcd.printf(" CAUTION");
  }
  if (_hind >= 32){
    digitalWrite(PEEXT, LOW);
    digitalWrite(PEXTR, LOW);
    digitalWrite(PDANG, HIGH);
    Serial.println("EXTRA_CAUTION");
    lcd.printf("XCAUTION");
  }
  if (_hind >= 41){
    digitalWrite(PEEXT, LOW);
    digitalWrite(PEXTR, HIGH);
    Serial.println("DANGER");
    lcd.printf("  DANGER");
  }
  if (_hind >= 54){
    digitalWrite(PEEXT, HIGH);
    Serial.println("EXTREME_DANGER");
    lcd.printf(" XDANGER");
  }
  Serial.println();
  return;
}

void SendToServer(){
  if (WiFi.status() != WL_CONNECTED){
    return; // If not connected, no need to try to send request because it will fail anyway
  }
  // Preparing the request body
  char payload[64];
  snprintf(payload, 64, "{\"did\":\"%16s\",\"t\":%6.2f,\"h\":%6.2f,\"hi\":%6.2f}", DEVIVE_ID, _temp, _humi, _hind);
  // Send the request
  int responseCode = http.POST(payload);
  Serial.print("[HTTP] ");
  Serial.println(responseCode);
  return;
}

void INT_KEYA_RISE(){
  // Set the flag 
  will_sleep = true;
  Serial.println("[POWR] Sleep confirmed, finishing remaining tasks...");
  return;
}

void setup() {
  // Initialize serial connection
  Serial.begin(9600);
  
  Serial.println("==== Initializing internals ====");
  // Setup pins
  pinMode(A_KEY, INPUT);
  pinMode(PSAFE, OUTPUT);
  pinMode(PWARN, OUTPUT);
  pinMode(PDANG, OUTPUT);
  pinMode(PEXTR, OUTPUT);
  pinMode(PEEXT, OUTPUT);
  Serial.println("[PINS] Ready");
  // Setup GPIO interrupts
  attachInterrupt(34, INT_KEYA_RISE, RISING);
  Serial.println("[INTR] Ready");

  Serial.println("==== Initializing peripherals ====");
  // Setup I2C pins
  Wire.begin(PSDA, PSDL);
  // Setup LCD
  lcd.init();           // Initialize the LCD
  lcd.backlight();      // Turn on the LCD backlight
  lcd.noCursor();       // Disable LCD cursors
  // Start DHT module
  dht.begin();
  // Setup wifi
  Serial.println("[WIFI] Start");
  WiFi.begin(STA_SSID, STA_PASS);  // We don't do internet-critial task, no need to block the program until the device is connected
  esp_log_level_set("wifi", ESP_LOG_NONE); // Disable logs for wifi too
  // Setup http client
  http.begin(client, "http://192.168.137.1:3000/api/v1/sensor0?did=j12t_sensor0");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("token", SERVER_SECRET);
  Serial.println("[HTTP] Ok");

  Serial.println("==== Device ready ====");
  lcd.println("START");
  lcd.println("Hello");
  lcd.clear();
}

void loop() {
  ReadSensor();
  DisplayValues(); // Prefer to display first then send
  SendToServer();
  
  if (will_sleep){
    Serial.println("==== Sleep sequences ====");
    // Prepare to sleep
    esp_sleep_enable_ext0_wakeup((gpio_num_t)A_KEY, HIGH); // Enable wakeup with GPIO pin input
    // Turn off wifi
    Serial.println("[WIFI] Stop");
    WiFi.mode(WIFI_OFF);
    // Leave user a message
    lcd.noBacklight();
    lcd.clear();
    lcd.println("!  DEEP SLEEP  !");
    lcd.println("!  Press2wake  !");
    // Deep sleep
    Serial.println("[POWR] Entering deep sleep");
    delay(1000);
    esp_deep_sleep_start();
  }
}