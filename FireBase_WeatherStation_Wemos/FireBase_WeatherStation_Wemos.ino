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
#include "secrets.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <EEPROM.h>


#include "SSD1306.h"
#include "SH1106.h"

extern "C" {
#include "user_interface.h"
}



#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE    DHT11     // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;


/*===== SETTINGS =====*/
/* create display(Adr, SDA-pin, SCL-pin) */
SSD1306 display(0x3c, 5, 4);   // GPIO 5 = D1, GPIO 4 = D2
//SH1106 display(0x3c, 5, 4);

/* select the button for your board */
#define btn D3         // GPIO 0 = FLASH BUTTON 

#define maxCh 13       // max Channel -> US = 11, EU = 13, Japan = 14
#define ledPin 2       // led pin ( 2 = built-in LED)
#define packetRate 5   // min. packets before it gets recognized as an attack

#define flipDisplay true

/* Display settings */
#define minRow       0              /* default =   0 */
#define maxRow     127              /* default = 127 */
#define minLine      0              /* default =   0 */
#define maxLine     63              /* default =  63 */

/* render settings */
#define Row1         0
#define Row2        30
#define Row3        35
#define Row4        80
#define Row5        85
#define Row6       125

#define LineText     0
#define Line        12
#define LineVal     47

//===== Run-Time variables =====//
unsigned long prevTime   = 0;
unsigned long curTime    = 0;
unsigned long pkts       = 0;
unsigned long no_deauths = 0;
unsigned long deauths    = 0;
int curChannel           = 1;
unsigned long maxVal     = 0;
double multiplicator     = 0.0;
bool canBtnPress         = true;

unsigned int val[128];

unsigned long timeDude;




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

    display.init();
  if (flipDisplay) display.flipScreenVertically();

  /* show start screen */
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Weather-");
  display.drawString(0, 16, "Station");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 40, "Copyright (c) 2019");
  display.drawString(0, 50, "Stepan Kryshtafovych");
  display.display();
  delay(2500);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  // Set delay between sensor readings based on sensor details.



}


void loop() {

  timeDude = millis();

  /* show start screen */
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Temperature: 72°");
  display.drawString(0, 16, "Humidity: 32%");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 40, "upTimer");
  display.drawString(0, 50, (String)timeDude);
  display.display();




}
