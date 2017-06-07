/*
   Written by Hans Perera

   This is designed to control tempurature inside a smoker.

   Features:
   #Pid controlled
   #LCD display
   #OTA updates

   TODO:
   Fuel warnning
   MQTT and openhab
   WEB interface
   Flash storage
*/

#define LCDON
#define WIFION
#define WEBPAGE

#include <Wire.h>
#include <max6675.h>
#include "ESP8266WiFi.h"
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <ESP_Adafruit_SSD1306.h>

#ifdef WIFION
//Access point config
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Dns.h>

//Remote firmware update
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//MQTT stuff
//#include <PubSubClient.h>
//WiFiClient espClient;
//PubSubClient client(espClient);
#endif

#define OLED_RESET_PIN 4
#define FAN_PIN 15
unsigned long previousMillis = 0; // For the timer
const long interval = 2000;  // Tempurature update rate
boolean error;

MAX6675 ktc(14, 13, 12);
#ifdef LCDON
Adafruit_SSD1306 display(OLED_RESET_PIN);
#endif

//Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;
double m_setPoint, Temp, Output;
//Specify the links and initial tuning parameters
PID myPID(&Temp, &Output, &m_setPoint, consKp, consKi, consKd, DIRECT);

// Web page stuff
#ifdef WEBPAGE
#include "web_server.h"
WebInterface webPage (Temp, m_setPoint);
#endif

void setup() {
  // Debugging serial
  Serial.begin(9600);

  // Pin setup
  pinMode(FAN_PIN, OUTPUT);
  analogWriteFreq(10000);

  // PID setup
  myPID.SetOutputLimits(2, 1023);
  myPID.SetMode(AUTOMATIC);
  m_setPoint = 210.0; // Default set point

#ifdef WIFION
  // WIFI connect
  WiFiManager wifiManager;
  wifiManager.autoConnect("Smoker"); // This will hold the connection till it get a connection
  WiFi.hostname("Smoker");

  // OTA stuff
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#else
  WiFi.mode(WIFI_OFF);
#endif

  // LCD init
#ifdef LCDON
  display.begin(SSD1306_SWITCHCAPVCC, 0x78 >> 1);
  display.display();
  display.clearDisplay();
#endif

  // Web page stuff
#ifdef WEBPAGE
  webPage.Begin ();
#endif
}

void loop() {

#ifdef WIFION
  ArduinoOTA.handle();
#endif

  // Update the fan speed only so often.
  unsigned long currentMillis = millis();// Time now
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read the temperature
    Temp = ktc.readFahrenheit();

    displayData();

    // Set how aggresive we want the pid controling to be.
    if ((m_setPoint - Temp) < 10)
    { //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }

    // Calculate the pid output.
    myPID.Compute();
    if (Output <= 2) // Make sure to turn off the fan completely if the value is too low.
      analogWrite(FAN_PIN, 0);
    else
      analogWrite (FAN_PIN, (int) Output);
  }

#ifdef WEBPAGE
  webPage.dataLogging ();
  webPage.serve ();
#endif
}

void displayData() {
  // Debugging stuff
  Serial.print("C = ");
  Serial.print(ktc.readCelsius());
  Serial.print("\t F = ");
  Serial.println(ktc.readFahrenheit());
  Serial.print ("pid output: ");
  Serial.println (Output);

#ifdef LCDON
  display.setTextSize(1);
  display.setCursor(5, 1);
  display.print (Output);
  if (Output <= 2)
    display.println (" |OFF");
  else
    display.println (" |ON");

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(18, 20);
  display.print((char)248);
  display.print("S:");
  display.print(m_setPoint);


  display.setCursor(18, 40);
  display.print((char)248);
  display.print("F:");
  display.println(Temp);
  display.setTextSize(1);
  if (error)
    display.println("ERROR!");
  else
    display.println("ok!");
  display.display();
  display.clearDisplay();
#endif
}
