/*
 Weather Station
 by Jeremy Barr
 Sept 23, 2024
 Version: 1.3.1
 - Added battery check before initializing hardware.
 - updated firmware code based on library updates (ex: ArduinoJson 5 to 7)
 - added Battery level sensor
 - removed SparkFun BME280 library
 - added Adafruit Sensor & BME280 libraries
 - renamed function initAtmosphereSensor() to initBME280Sensor()
*/

// Include the ESP8266 WiFi library. (Works a lot like the
// Arduino WiFi library.)
#include <ESP8266WiFi.h>
// Include the ESP8266 HTTPClient library.
#include <ESP8266HTTPClient.h>
// Include the ArduinoJson library to encode data in JSON
#include <ArduinoJson.h>

// Include the Adafruit BME280 library.
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <stdint.h>
#include "Wire.h"
#include "SPI.h"

char firmwareVer[] = "1.2.1"; // Firmware version installed

//////////////////////////
// Global Sensor Object //
//////////////////////////
Adafruit_BME280 bme;

#define SEALEVELPRESSURE_HPA (1013.25)
#define FEET_PER_METER (3.28084)
#define MINITE_MS (60000)

unsigned long delayTime;
float tempC = 0;
float tempF = 0;
float pressure = 0;
float humidity = 0;
float altitudeFeet = 0;
float altitudeMeters = 0;
float voltage = 0;

//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiSSID[] = "";
const char WiFiPSK[] = "";
const char WiFiHostname = "Weather Station 1";

/////////////////////
// Battery Voltage //
/////////////////////
// VOLTAGE = ((A0-value)*ANALOG_VOLT_COEF)-ANALOG_VOLT_CONST;
const byte ANALOG_PIN = A0;
const float ANALOG_VOLT_COEF =  0.0051705;
const float ANALOG_VOLT_CONST = -0.014703;
const float TEMPC_CORRECTION = 6.414402;

/////////////////
// Server info //
/////////////////
//const char ServerHost[] = "frozencreeks.com";
const char ServerHost[] = "http://10.0.0.102";
const int httpPort = 3000;

// Static IP settings
IPAddress staticIP(10,0,0,43); //static IP address
IPAddress gateway(10,0,0,1);   //Router's IP address
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);
IPAddress subnet(255, 255, 255, 0);

/////////////////
// Post Timing //
/////////////////
const int sleepTimeS = 900;    // 900 seconds for deep sleep 15min*(60sec/min)

void setup()
{
  // check battery level before initialization
  batteryCheck();
  
  initHardware();
  initBME280Sensor();
  
  connectWiFi();
  
  readSensorData();   // Read Sensor data and pass to WeatherStation
  delay(delayTime);   // delayTime assigned from readSensorData()
  printSensorData();  // Print the sensor data to serial
   
  while (postToServer() != 1)
  {
    delay(100);
  }
  digitalWrite(LED_BUILTIN, HIGH);

  // deepSleep time is defined in microseconds.
  // Multiply seconds by 1e6
  //ESP.deepSleep(sleepTimeS * 1e6);
  //delay(1000); //for above sleep
  deepSleepSec(sleepTimeS);
}

void loop()
{}

void connectWiFi()
{
  byte ledStatus = HIGH; // HIGH=OFF, LOW=ON

  // DHCP Hostname
  WiFi.hostname(WiFiHostname);
  
  if (!WiFi.config(staticIP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  // Set WiFi mode to station (as opposed to AP or AP_STA)
  WiFi.mode(WIFI_STA);

  // WiFI.begin([ssid], [passkey]) initiates a WiFI connection
  // to the stated [ssid], using the [passkey] as a WPA, WPA2,
  // or WEP passphrase.
  WiFi.begin(WiFiSSID, WiFiPSK);

  // Use the WiFi.status() function to check if the ESP8266
  // is connected to a WiFi network.
  while (WiFi.status() != WL_CONNECTED)
  {
    if (millis() > MINITE_MS) {
      Serial.print("60 sec Timeout connecting to ");
      Serial.print(WiFiSSID);
      Serial.println("./nGoing back to sleep...");
      // deepSleep time is defined in microseconds.
      // Multiply seconds by 1e6
      //ESP.deepSleep(60 * 1e6); //(seconds * 1e6)
      //delay(1000); //for above sleep
      deepSleepSec(60);
    }
    // Blink the LED
    digitalWrite(LED_BUILTIN, ledStatus); // Write LED high/low
    ledStatus = (ledStatus == LOW) ? HIGH : LOW;

    // Delays allow the ESP8266 to perform critical tasks
    // defined outside of the sketch. These tasks include
    // setting up, and maintaining, a WiFi connection.
    delay(100);
    // Potentially infinite loops are generally dangerous.
    // Add delays -- allowing the processor to perform other
    // tasks -- wherever possible.
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void initHardware()
{
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output. (onboard LED)
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH); // Arduino: turn the LED on (HIGH)
                                   // D1 Mini: turns the LED *off*
  // Don't need to set ANALOG_PIN as input,
  // that's all it can be.
}

void initBME280Sensor()
{
   if (! bme.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    Serial.println("-- Default Test --");
    Serial.println("normal mode, 16x oversampling for all, filter off,");
    Serial.println("0.5ms standby period");
    delayTime = 5000;
    
    
    // For more details on the following scenarious, see chapter
    // 3.5 "Recommended modes of operation" in the datasheet
    
/*
    // weather monitoring
    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
                      
    // suggested rate is 1/60Hz (1m)
    delayTime = 60000; // in milliseconds
*/

/*    
    // humidity sensing
    Serial.println("-- Humidity Sensing Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 0x pressure oversampling");
    Serial.println("= pressure off, filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_NONE, // pressure
                    Adafruit_BME280::SAMPLING_X1,   // humidity
                    Adafruit_BME280::FILTER_OFF );
                      
    // suggested rate is 1Hz (1s)
    delayTime = 1000;  // in milliseconds
*/

/*    
    // indoor navigation
    Serial.println("-- Indoor Navigation Scenario --");
    Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
    Serial.println("0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
    
    // suggested rate is 25Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
    // T_ovs = 2
    // P_ovs = 16
    // H_ovs = 1
    // = 40ms (25Hz)
    // with standby time that should really be 24.16913... Hz
    delayTime = 41;
    */
    
    /*
    // gaming
    Serial.println("-- Gaming Scenario --");
    Serial.println("normal mode, 4x pressure / 1x temperature / 0x humidity oversampling,");
    Serial.println("= humidity off, 0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_X4,   // pressure
                    Adafruit_BME280::SAMPLING_NONE, // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
                      
    // Suggested rate is 83Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5)
    // T_ovs = 1
    // P_ovs = 4
    // = 11.5ms + 0.5ms standby
    delayTime = 12;
*/

    Serial.println();  
}


int postToServer()
{
  // LED turns on when we enter, it'll go off when we
  // successfully post.
  digitalWrite(LED_BUILTIN, LOW);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "WeMos-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String postedID = "WeMos-" + macID; // macID = F833

  //Declaring static JSON buffer
  //StaticJsonBuffer<300> JSONbuffer; // arduinojson 5
  StaticJsonDocument<256> JSONbuffer;
  //JsonObject& JSONdata = JSONbuffer.createObject(); // arduinojson 5
  JsonObject JSONdata = JSONbuffer.to<JsonObject>();

  // Build the JSON data by adding the field/value pairs defined by our stream:
  JSONdata["station_id"] = postedID;
  JSONdata["tempf"] = tempF;
  JSONdata["tempc"] = tempC;
  JSONdata["humidity"] = humidity;
  JSONdata["pressure"] = pressure;
  JSONdata["altitude_ft"] = altitudeFeet;
  JSONdata["altitude_m"] = altitudeMeters;
  JSONdata["time"] = millis() / 1000;
  JSONdata["firmware_version"] = firmwareVer;
  JSONdata["battery"] = voltage;

  char JSONmessageBuffer[256]; // arduinojson 5
  //JSONdata.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer)); // arduinojson 5
  //serializeJsonPretty(JSONdata, Serial);
  serializeJsonPretty(JSONdata,JSONmessageBuffer);
  Serial.println(JSONmessageBuffer);

  // Now connect to Frozen Creek Weather, and post our data:
  HTTPClient http;
  WiFiClient client;
  String serverUrl = String(ServerHost) + ":" + String(httpPort) + "/api/weather";
  http.begin(client, serverUrl);
  http.addHeader("Content-Type", "application/json");

  // POST the JSON encoded data in the JSON message buffer and initialize response variable
  int response = http.POST(JSONmessageBuffer);
  // Response payload as String
  String payload = http.getString();

  Serial.println(serverUrl);   //Print HTTP return code

  Serial.println(response);   //Print HTTP return code
  Serial.println(payload);    //Print request response payload

  // Close connection
  http.end();

  // Before we exit, turn the LED off.
  digitalWrite(LED_BUILTIN, HIGH);

  return 1; // Return success
}

void printSensorData()
{
  Serial.print("\nTemperature F: ");
  Serial.print(tempF, 2);
  Serial.println(" degrees F");

  Serial.print("Temperature C: ");
  Serial.print(tempC, 2);
  Serial.println(" degrees C");

  Serial.print("Pressure: ");
  Serial.print(pressure, 2);
  Serial.println(" Pa");

  Serial.print("Altitude: ");
  Serial.print(altitudeFeet, 2);
  Serial.println("ft");

  Serial.print("Altitude: ");
  Serial.print(altitudeMeters, 2);
  Serial.println("m");

  Serial.print("%RH: ");
  Serial.print(humidity, 2);
  Serial.println(" %");

  Serial.print("time: ");
  Serial.print(millis() / 1000);
  Serial.println(" sec");

  Serial.print("battery voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  Serial.println();
}

void readSensorData()
{
  tempC = bme.readTemperature() - TEMPC_CORRECTION; //runs too hot
  tempF = (tempC * 9/5) + 32; //convert to Fahrenheit 
  humidity = bme.readHumidity();
  pressure = bme.readPressure();
  altitudeMeters = bme.readAltitude(SEALEVELPRESSURE_HPA);
  altitudeFeet = altitudeMeters/FEET_PER_METER;
  voltage = getBatteryVoltage(); //analogRead(ANALOG_PIN);
}

float getBatteryVoltage()
{
  //Serial.println(analogRead(ANALOG_PIN));
  return ((analogRead(ANALOG_PIN)*ANALOG_VOLT_COEF)+ANALOG_VOLT_CONST);
}

void batteryCheck()
{
  // Check battery voltage and goto sleep if battery is low.
  int minVoltage = 3.2;
  
  // read battery voltage
  // if undervoltage sleep for 1hr (3600sec)
  if (getBatteryVoltage() < minVoltage) {
    //Serial.println("Battery Voltage below 3.2V... Going to sleep: 1hr");
    deepSleepSec(3600);
  }
}

void deepSleepSec(int sleepSeconds)
{
  // deepSleep time is defined in microseconds.
  // Multiply seconds by 1e6
  ESP.deepSleep(sleepSeconds * 1e6);
  delay(1000); //for above sleep
}
