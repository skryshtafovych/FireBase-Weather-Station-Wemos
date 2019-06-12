/*
  Cloud State change detection (cloud edge detection)

  Often, you don't need to know the state of a digital input all the time, but
  you just need to know when the input changes from one state to another.
  For example, you want to know when a Hall Sensor goes from OFF to ON. This is called
  state change detection, or edge detection.

  This example shows how to detect when a hall Sensor or button changes from off to on
  and on to off or Up to Down or Down to Up ...

  The circuit:
  - hallSensor attached to pin 2 from +5V
  - Relay attached from pin 5 to ground
  - DHT11 Temp/Humidity Sensor attached pin 4

  created  27 Sep 2005 -by Tom Igoe
  modified 30 Aug 2011 -by Tom Igoe
  modified 09 May 2019 -by Stepan Kryshtafovych

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
// - Firebase Arduino Lib: https://github.com/googlesamples/firebase-arduino/archive/master.zip
// - Json for Firebase http://downloads.arduino.cc/libraries/github.com/bblanchon/ArduinoJson-5.13.2.zip



  You Can find modified example on Github.
  https://github.com/skryshtafovych/CustomHomeGarage

  This was based on below example code is in the public domain.
  http://www.arduino.cc/en/Tutorial/ButtonStateChange
*/
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include "secretValues.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


#define DHTPIN 4     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11     // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;



// this constant won't change:
const int  hallSensorPin = 2;    // the pin that the pushbutton is attached to
const int relayPin = 5;       // the pin that the LED is attached to


// Variables will change:
int hallSensorCounter = 0;   // counter for the number of button presses
int hallSensorState = 0;         // current state of the button
int lastHallSensorState = 0;     // previous state of the button

void setup() {
  // initialize the button pin as a input:
  pinMode(hallSensorPin, INPUT);
  // initialize the LED as an output:
  pinMode(relayPin, OUTPUT);
  // initialize serial communication:
  Serial.begin(9600);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // connect to wifi.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
    // Initialize device.
  dht.begin();
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
   // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);



}


void loop() {


  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Firebase.setInt("GarageT", event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Firebase.setInt("GarageH", event.relative_humidity);
    Serial.println(F("%"));
    Serial.println(Firebase.getString("hallSensorGarageH"));

  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // read the hall sensor input pin:
  hallSensorState = digitalRead(hallSensorPin);

  // compare the hallSensorState to its previous state
  if (hallSensorState != lastHallSensorState) {
    // if the state has changed, increment the counter
    if (hallSensorState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      hallSensorCounter++;
      Serial.println("on");
      Serial.print("number of button pushes: ");
      Serial.println(hallSensorCounter);
      Firebase.setString("hallSensorGarageH", "true");


    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("off");
      Firebase.setString("hallSensorGarageH", "false");
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastHallSensorState = hallSensorState;


  // turns on the LED every four button pushes by checking the modulo of the
  // button push counter. the modulo function gives you the remainder of the
  // division of two numbers:
  if (hallSensorCounter % 2 == 0) {
    digitalWrite(relayPin, HIGH);
      Firebase.setString("hallSensorGarage", "true");
  } else {
    digitalWrite(relayPin, LOW);
          Firebase.setString("hallSensorGarage", "false");

  }

}
