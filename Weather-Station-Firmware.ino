// Include the ESP8266 WiFi library. (Works a lot like the
// Arduino WiFi library.)
#include <ESP8266WiFi.h>
// Include the ESP8266 HTTPClient library.
#include <ESP8266HTTPClient.h>
// Include the ArduinoJson library to encode data in JSON
#include <ArduinoJson.h>

// Include the SparkFun BME280 library.
#include "SparkFunBME280.h"

#include <stdint.h>
#include "Wire.h"
#include "SPI.h"

//////////////////////////
// Global Sensor Object //
//////////////////////////
ADC_MODE(ADC_VCC);  // Allows voltage read on pin VCC
BME280 mySensor;

char firmwareVer[] = "1.0.0"; // Software version installed

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

/////////////////////
// Pin Definitions //
/////////////////////
const int LED_PIN = 5; // Thing's onboard, green LED
const int ANALOG_PIN = A0; // The only analog pin on the Thing
const int DIGITAL_PIN = 12; // Digital pin to be read

/////////////////
// Server info //
/////////////////
const char ServerHost[] = "http://##.##.##.##";
const int httpPort = 3000;

/////////////////
// Post Timing //
/////////////////
const unsigned long postRate = 15 * 60000; // 15min*(60sec/min)=900sec*1000ms
const int sleepTimeS = 900;    // seconds
unsigned long lastPost = 0;

void setup()
{
  initHardware();
  initAtmosphereSensor();
  connectWiFi();
  digitalWrite(LED_PIN, HIGH);
  readSensorData();   // Read Sensor data and pass to WeatherStation
  //printSensorData();  // Print the sensor data to serial
  delay(100);
  while (postToServer() != 1)
  {
    delay(100);
  }
  digitalWrite(LED_PIN, LOW);

  // deepSleep time is defined in microseconds. Multiply
  // seconds by 1e6
  ESP.deepSleep(sleepTimeS * 1e6);
  delay(1000); //for above sleep
}

void loop()
{
  /*
    // Post to Server every 900 seconds (postRate/1000)
    if (lastPost + postRate <= millis())
    {
    readSensorData();   // Read Sensor data and pass to WeatherStation
    printSensorData();  // Print the sensor data to serial
    delay(100);
    if (postToServer()) {
      lastPost = millis();
      Serial.println(lastPost);
    }
    else
      delay(100);
    }*/

}

void connectWiFi()
{
  byte ledStatus = LOW;

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
    // Blink the LED
    digitalWrite(LED_PIN, ledStatus); // Write LED high/low
    ledStatus = (ledStatus == HIGH) ? LOW : HIGH;

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
  pinMode(DIGITAL_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Don't need to set ANALOG_PIN as input,
  // that's all it can be.
}

void initAtmosphereSensor()
{
  //***Driver settings********************************//
  //commInterface can be I2C_MODE or SPI_MODE
  //specify chipSelectPin using arduino pin names
  //specify I2C address.  Can be 0x77(default) or 0x76

  //For I2C, enable the following and disable the SPI section
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x77;

  //For SPI enable the following and dissable the I2C section
  //mySensor.settings.commInterface = SPI_MODE;
  //mySensor.settings.chipSelectPin = 10;

  //***Operation settings*****************************//

  //runMode can be:
  //  0, Sleep mode
  //  1 or 2, Forced mode
  //  3, Normal mode
  mySensor.settings.runMode = 3; //Normal mode

  //tStandby can be:
  //  0, 0.5ms
  //  1, 62.5ms
  //  2, 125ms
  //  3, 250ms
  //  4, 500ms
  //  5, 1000ms
  //  6, 10ms
  //  7, 20ms
  mySensor.settings.tStandby = 0;

  //filter can be off or number of FIR coefficients to use:
  //  0, filter off
  //  1, coefficients = 2
  //  2, coefficients = 4
  //  3, coefficients = 8
  //  4, coefficients = 16
  mySensor.settings.filter = 0;

  //tempOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.tempOverSample = 1;

  //pressOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.pressOverSample = 1;

  //humidOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.humidOverSample = 1;
  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.

  Serial.begin(57600);
  Serial.print("Starting BME280... result of .begin(): 0x");
  //Calling .begin() causes the settings to be loaded
  Serial.println(mySensor.begin(), HEX);
}

int postToServer()
{
  // LED turns on when we enter, it'll go off when we
  // successfully post.
  digitalWrite(LED_PIN, HIGH);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String postedID = "Thing-" + macID;

  //Declaring static JSON buffer
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONdata = JSONbuffer.createObject();

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
  JSONdata["battery"] = voltage / 1000;

  char JSONmessageBuffer[300];
  JSONdata.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println(JSONmessageBuffer);

  // Now connect to Frozen Creek Weather, and post our data:
  HTTPClient http;
  String serverUrl = String(ServerHost) + ":" + String(httpPort) + "/api/weather";
  http.begin(serverUrl);
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
  digitalWrite(LED_PIN, LOW);

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
  tempF = mySensor.readTempF();
  tempC = mySensor.readTempC();
  altitudeMeters = mySensor.readFloatAltitudeMeters();
  altitudeFeet = mySensor.readFloatAltitudeFeet();
  humidity = mySensor.readFloatHumidity();
  pressure = mySensor.readFloatPressure();
  voltage = ESP.getVcc();
}
