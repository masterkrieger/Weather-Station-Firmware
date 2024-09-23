# Weather Station Firmware

## WeMOS LOLIN D1 mini
The hookup guide for the WeMOS D1 Mini.
https://www.wemos.cc/en/latest/d1/d1_mini.html

### Pinout
220 kΩ resistor - A0 to GND
Deep sleep jumper - D0 to RST (remove for programming)

| BME280 SPI Sensor | WEMOS |
| ----------------- | ----- |
| SDA               | D2    |
| SCL               | D1    |
| GND               | GND   |
| VIN               | 3V3   |

## WiFi Credentials

Create a `WiFiCredentials.h` header file with the following lines to include your protected Wifi Credentials.

`WiFiCredentials.h`
```
#define SSID myssidwhatever
#define PASSWORD mypassword
```