# NoHeatStroke-ESP32 -- Microprocesors Project

This is the source code to be compiled and add to NoHeatStroke-ESP32 device. The device will read temperatures and humidity from [DHT11](https://components101.com/sensors/dht11-temperature-sensor) sensor, then calculate the [Heat Index](https://www.weather.gov/arx/heat_index) (`HI`) value. Based on the calulated `HI` value, the device will show warnings about potenial heatstrokes to user via the built-in LCD and leds. In addition, the device will send the values to [NoHeatStroke-Server](https://github.com/jerapiblaze/NoHeatStroke-Server), where the values will be stored and serve users on-demand.

The device will go to [deep-sleep mode](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html) when pushing the button, and will wake from sleep when pushing the button again.

## Circuit diagram

*to-be updated*

## Setup

Before uploading the code to your own ESP32, you will need to define some variables.

```cpp
#define DEVIVE_ID "j12t_esp32_0" // DEVICE_ID (unique for each device, max 16)
#define SERVER_SECRET "123456"   // same as in server
#define SERVER_ENDPOINT "http://192.168.137.1:3000/api/v1/sensor0" // SERVER API ENDPOINT
#define STA_SSID "Wriotheley-de-teatime"   // Your wifi ssid
#define STA_PASS "chainese-tea"            // Your wifi password
```

## References

- [Basic http request -- randomnerdtutorials.com](https://randomnerdtutorials.com/esp32-http-get-post-arduino/)
- [Format C string -- delftstack.com](https://www.delftstack.com/howto/c/c-string-formatting/)
- [Heat index levels -- weather.gov](https://www.weather.gov/arx/heat_index)
- [Connect 1602 led using I2C module to esp32 -- randomnerdtutorials.com](https://randomnerdtutorials.com/esp32-esp8266-i2c-lcd-arduino-ide/)
- [ESP32 with DHT11/DHT22 -- randomnerdtutorials.com](https://randomnerdtutorials.com/esp32-dht11-dht22-temperature-humidity-sensor-arduino-ide/)
- [ESP32 sleep modes -- deepbluembedded.com](https://deepbluembedded.com/esp32-sleep-modes-power-consumption/)
- [UTF-8 character with 1602 LCD](https://github.com/locple/LiquidCrystal_I2C_UTF8)