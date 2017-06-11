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

//#define LCDON
#define WIFION
#define WEBPAGE

#include <Wire.h>
#include <max6675.h>
#include "ESP8266WiFi.h"
#include <PID_v1.h>
//#include <Adafruit_GFX.h>
#include <ESP_Adafruit_SSD1306.h>

#ifdef WIFION
//Access point config
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Dns.h>

//Remote firmware update
#include <ESP8266mDNS.h>
//#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//MQTT stuff
#include <PubSubClient.h>
WiFiClient espClient;
PubSubClient client(espClient);
#define MQTT_SERVER "172.168.1.10"

#endif

#define OLED_RESET_PIN 4
#define FAN_PIN 15
#define FAN_RAMP_PERIOD 2
unsigned long prevMilForInputRead = 0; // For the timer
const long interval = 2000;  // Tempurature update rate
unsigned int actualValue = 0;
bool fanOff = true;

MAX6675 ktc(14, 13, 12);
#ifdef LCDON
Adafruit_SSD1306 display(OLED_RESET_PIN);
#endif

//Define the aggressive and conservative Tuning Parameters
const double aggKp = 4, aggKi = 0.2, aggKd = 1;
const double consKp = 1, consKi = 0.05, consKd = 0.25;
double m_setPoint, m_temp, Output;
//Specify the links and initial tuning parameters
PID myPID(&m_temp, &Output, &m_setPoint, aggKp, aggKi, aggKd, DIRECT);

static int preHeatingTime = 1800000; // 30 minutes of pre-heating time
static int notificationTreshhold = 5400000; // Use to send notifications every 1.5 hours (90 Minutes)
int notificationCounter = 1; // Will increase every time a notification is send
bool timeNotification = false;
bool heatNotification = false;

unsigned int pwmValue = 1023;
bool fast = false;
char macAddressChar[18];
char* heatStatusStr = "openhab/home/smoker/heatStatus";
char* timeStatusStr = "openhab/home/smoker/timeStatus";
unsigned long reconnectPreviousMillis = 0;

// Web page stuff
#ifdef WEBPAGE
#include <FS.h>
#include <NtpClientLib.h>
#include <TimeLib.h>
#include <ArduinoJson.h>

#define HISTORY_FILE "/history.json"

ESP8266WebServer m_server (80);

StaticJsonBuffer<5000> jsonBuffer;                 // Buffer static contenant le JSON courant - Current JSON static buffer
JsonObject& root = jsonBuffer.createObject();
JsonArray& timestamp = root.createNestedArray("timestamp");
JsonArray& hist_t = root.createNestedArray("t");
JsonArray& hist_s = root.createNestedArray("s");

const long intervalHist = 20000;
unsigned long previousMillis = intervalHist;  // Dernier point enregistré dans l'historique - time of last point added
int     sizeHist = 20 ; //Number of records
char json[5000];                                   // Buffer pour export du JSON - JSON export buffer
#endif

void setup() {
  // Debugging serial
  Serial.begin(115200);

  // Pin setup
  pinMode(FAN_PIN, OUTPUT);
  //  analogWriteFreq(5000);

  // PID setup
  myPID.SetOutputLimits(30, 1023);
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
  
  //mqqt config
  DNSClient dns;
  IPAddress dns_ip(172, 168, 1, 1);
  IPAddress out_ip;
  dns.begin(dns_ip);
  Serial.print("mqtt server: ");
  dns.getHostByName(MQTT_SERVER, out_ip);

  Serial.println(out_ip);
  client.setServer(out_ip, 1883);
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
  // Real time
  NTP.onNTPSyncEvent([](NTPSyncEvent_t error) {
    if (error) {
      Serial.print("Time Sync error: ");
      if (error == noResponse)
        Serial.println("NTP server not reachable");
      else if (error == invalidAddress)
        Serial.println("Invalid NTP server address");
    }
    else {
      Serial.print("Got NTP time: ");
      Serial.println(NTP.getTimeDateString(NTP.getLastNTPSync()));
    }
  });
  // Serveur NTP, decalage horaire, heure été - NTP Server, time offset, daylight
  NTP.begin("pool.ntp.org", -6, true);
  NTP.setInterval(60000);

  // Load the flash
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount failed");        // Problème avec le stockage SPIFFS - Serious problem with SPIFFS
  } else {
    Serial.println("SPIFFS Mount succesfull");
    loadHistory();
  }
  delay(50);

  // Config the web server
  m_server.on("/mesures.json", sendMesures);
  m_server.on("/graph_temp.json", sendHistory);
  m_server.on("/set", setTemperature);

  m_server.serveStatic("/js", SPIFFS, "/js");
  m_server.serveStatic("/css", SPIFFS, "/css");
  m_server.serveStatic("/img", SPIFFS, "/img");
  m_server.serveStatic("/", SPIFFS, "/index.html");

  m_server.begin();
  Serial.println ( "HTTP server started" );

  Serial.print("Uptime :");
  Serial.println(NTP.getUptime());
  Serial.print("LastBootTime :");
  Serial.println(NTP.getLastBootTime());

#endif
}

void loop() {

#ifdef WIFION
  ArduinoOTA.handle();
  m_server.handleClient();
#endif

  // Update the fan speed only so often.
  unsigned long currentMillis = millis();// Time now
  if (currentMillis - prevMilForInputRead >= interval) {
    prevMilForInputRead = currentMillis;

    // Read the temperature
    m_temp = ktc.readFahrenheit();
    m_temp = 0;

    displayData();

    setNotifications();
    sendNotifications();

    // Set how aggresive we want the pid controling to be.
    if ((m_setPoint - m_temp) < 10)
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
  }

  fanSpeed ((int)Output);
#ifdef WEBPAGE
  addPtToHist ();
#endif
}

void displayData() {
  // Debugging stuff
  //Serial.print("C = ");
  //Serial.print(ktc.readCelsius());
  //Serial.print("\t F = ");
  //Serial.println(ktc.readFahrenheit());
  //Serial.print ("pid output: ");
  //Serial.println (Output);  

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
  display.println(m_temp);
  display.setTextSize(1);
  //  if (error)
  //    display.println("ERROR!");
  //  else
  //    display.println("ok!");
  display.display();
  display.clearDisplay();
#endif
}

#ifdef WEBPAGE
void addPtToHist() {
  unsigned long currentMillis = millis();

  if ( currentMillis - previousMillis > intervalHist ) 
  {
    long int tps = NTP.getTime();
    previousMillis = currentMillis;
    //Serial.println(NTP.getTime());
    if ( tps > 0 ) 
    {
      timestamp.add(tps);
      hist_t.add(m_temp);
      hist_s.add(m_setPoint);

      //root.printTo(Serial);
      if ( hist_t.size() > sizeHist ) 
      {
        //Serial.println("efface anciennes mesures");
        timestamp.remove(0);
        hist_t.remove(0);
        hist_s.remove(0);
      }
      Serial.print("size hist_t "); Serial.println(hist_t.size());
      //      calcStat();

      saveHistory();
      Serial.print("after the save");
      root.printTo(Serial);
    }
  }
}

void sendMesures() {
  String tempJson = "{\"t\":\"" + String(m_temp) + "\",";
  tempJson += "\"s\":\"" + String(m_setPoint);
  tempJson += "\"}";

  m_server.send(200, "application/json", tempJson);
  Serial.println("Send measures");
}

void sendHistory() {
  root.printTo(json, sizeof(json));             // Export du JSON dans une chaine - Export JSON object as a string
  m_server.send(200, "application/json", json);   // Envoi l'historique au client Web - Send history data to the web client
  Serial.println("Send History");
}

void loadHistory()
{
  File file = SPIFFS.open(HISTORY_FILE, "r");
  if (!file) {
    Serial.println("No History Exist");
    return;
  } else {
    size_t size = file.size();
    if ( size == 0 ) {
      Serial.println("History file empty !");
    } else {
      std::unique_ptr<char[]> buf (new char[size]);
      file.readBytes(buf.get(), size);
      JsonObject& root = jsonBuffer.parseObject(buf.get());
      if (!root.success()) {
        Serial.println("Impossible to read JSON file");
      } else {
        Serial.println("History loaded");
        root.prettyPrintTo(Serial);
        Serial.print("setting the set temp : ");
        if (root["s"][root["s"].size() - 1] != 0)
          m_setPoint = root["s"][root["s"].size() - 1];
        Serial.println (m_setPoint);
      }
    }
    file.close();
  }
}

void saveHistory()
{
  Serial.println("Save History");
  File historyFile = SPIFFS.open(HISTORY_FILE, "w");
  root.printTo(historyFile); // Exporte et enregsitre le JSON dans la zone SPIFFS - Export and save JSON object to SPIFFS area
  historyFile.close();
}

void setTemperature() {
  String type = m_server.arg("type");
  String temp = m_server.arg("temp");
  Serial.print("Setting temp: ");

  if (temp == "0")
  {
    if (type == "1")
      m_setPoint = m_setPoint + 5;
    else if (type == "0")
      m_setPoint = m_setPoint - 5;
  }
  else if (String(temp).toInt() > 0)
  {
    m_setPoint = String(temp).toFloat();
  }

  Serial.println(m_setPoint);
  m_server.send(200, "text/plain", "success");          //Returns the HTTP response
}
// control the light intensity.
void fanSpeed (unsigned short pwmValue)
{
  if (pwmValue > 30)
  {
    if (actualValue < 500 && fanOff)
    {

      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= FAN_RAMP_PERIOD)
      {

        previousMillis = currentMillis;
        actualValue++;
        analogWrite (FAN_PIN, actualValue);
        if (actualValue >= 500)
        {
          Serial.println("ramping.....");
          fanOff = false;
        }
      }
    }
    else if (pwmValue > actualValue)
    {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= FAN_RAMP_PERIOD)
      {
        previousMillis = currentMillis;
        actualValue++;
        analogWrite (FAN_PIN, actualValue);
      }
    }
    else if (pwmValue < actualValue)
    {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= FAN_RAMP_PERIOD)
      {
        previousMillis = currentMillis;
        actualValue--;
        analogWrite (FAN_PIN, actualValue);
      }
    }
  }
  else if (actualValue > 0)
  {
    fanOff = true;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= FAN_RAMP_PERIOD)
    {
      previousMillis = currentMillis;
      actualValue--;
      analogWrite (FAN_PIN, actualValue);
    }
  }

}
#endif


void setNotifications ()
{
    if (millis() > preHeatingTime) // Pass the pre-heating step
    {
        long notificationTimer = notificationTreshhold * notificationCounter;
        if (millis() > notificationTimer) // if the current time 
        {
            notificationCounter = notificationCounter + 1;
            timeNotification = true;
        }

        if ((m_setPoint - m_temp) > 15) // if after 40 minutes the temp diffrrent between current and set is more than 15 send notification
        {
            heatNotification = true;
        }
    }
}

void sendNotifications()
{
    connectToOpenHab();
    
    if (timeNotification)
    {
        // send time notification
        client.publish(timeStatusStr, "true");
        timeNotification = false;
    }

    if (heatNotification)
    { 
        // send heat notification
        client.publish(heatStatusStr, "true");
        heatNotification = false;
    }
}

#ifdef WIFION
void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  // if (length > 0)
  // {
    //payload[length] = '\0';
    //updateThePwmValue ((char*) payload);
    //    pwmValue = String((char*) payload).toInt ();
    //      pwmValue = map(precentage, 0, 100, 0, 1023);
    //      Serial.print(precentage);
    //      Serial.print("/");
    // Serial.println(pwmValue);
    // fast = false;
  // }
}

void reconnect() 
{
  // Loop until we're reconnected
  if (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(macAddressChar, "openhabian", "openhabian")) 
    {
      Serial.println("connected");
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}

void connectToOpenHab()
{
  unsigned long currentMillis = millis();
  // Run only if wifi is on
  
  #ifdef WIFION
  ArduinoOTA.handle();
  
  // MQTT stuff
  if (!client.connected()) 
  {
    if (currentMillis - reconnectPreviousMillis >= 5000) 
    {
      reconnectPreviousMillis = currentMillis;
      reconnect();
    }
  }
  client.loop();
  #endif
}

#endif

