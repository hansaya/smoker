/*
   Written by Hans Perera

   This is designed to control tempurature inside a smoker.

   Features:
   #Pid controlled
   #LCD display
   #OTA updates
   #MQTT and openhab
   #WEB interface
   #Flash storage

   TODO:
   Fuel warnning (buggy)

*/

#define LCDON
#define WIFION
#define WEBPAGE
#define MQTT
// #define DEBUG

#include <Wire.h>
#include <max6675.h>
#include "ESP8266WiFi.h"
#include <PID_v1.h>
#include <ESP_Adafruit_SSD1306.h>
#include <SimpleList.h>

#ifdef WIFION
//Access point config
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <Dns.h>
#include <WiFiManager.h>

//Remote firmware update
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>



#endif

#ifdef MQTT
//MQTT stuff
#include <PubSubClient.h>
WiFiClient espClient;
PubSubClient client(espClient);
#define MQTT_SERVER "172.168.1.10"
#endif

//Pins
#define OLED_RESET_PIN 4
#define FAN_PIN 15
#define FAN_RAMP_PERIOD 2
#define MINFAN 30
unsigned long prevMilForInputRead = 0; // For the timer
const long interval = 2000;  // Tempurature update rate
unsigned int actualValue = 0;
bool fanOff = true;

// Temp sensor stuff
MAX6675 ktc(14, 13, 12);
#ifdef LCDON
Adafruit_SSD1306 display(OLED_RESET_PIN);
#endif

//Define the aggressive and conservative Tuning Parameters
 double Kp = 40, Ki = 0.2, Kd = 0.3;
//ouble Kp = 0.09, Ki = 0.006, Kd = 0.2;
//const double consKp = 3, consKi = 0.1, consKd = 0.4;
double m_setPoint, m_temp, m_output;
//Specify the links and initial tuning parameters
PID myPID(&m_temp, &m_output, &m_setPoint, Kp, Ki, Kd, DIRECT);


//Notification sendout to openhab
#ifdef DEBUG
static unsigned long preHeatingTime = 20000;// Make it faster for debugging
#else
static unsigned long preHeatingTime = 900000; // 15 minutes of pre-heating time
#endif
static unsigned long notificationTreshhold = 5400000; // Use to send notifications every 1.5 hours (90 Minutes)
static unsigned long timeNotificationTimeOut = 240000; // Leave the notification for 4 mins
unsigned long timeNotificationPreviousMillis = 0;
int notificationCounter = 1; // Will increase every time a notification is send
bool timeNotification = false;
bool heatNotification = false;
bool previous_timeNotification = false;
bool previous_heatNotification = false;
unsigned long reconnectPreviousMillis = 0;

char* mqttHeatStatusTopic = "openhab/home/smoker/heat_status";
char* mqttTimeStatusTopic = "openhab/home/smoker/time_status";
char* mqttTempTopic = "openhab/home/smoker/temp";
char* mqttSetTempTopic = "openhab/home/smoker/set_temp";
char* mqttSetTempStateTopic = "openhab/home/smoker/set_temp_state";
char* mqttFanSpeedTopic = "openhab/home/smoker/fan_speed";

// Web page stuff
#ifdef WEBPAGE
#include <FS.h>
#include <NtpClientLib.h>
#include <TimeLib.h>
#include <ArduinoJson.h>

#define HISTORY_FILE "/history.json"

ESP8266WebServer m_server (80); //Webserver

StaticJsonBuffer<200> histroyJsonBuffer;                 // Buffer static contenant le JSON courant for last saved values
JsonObject& history = histroyJsonBuffer.createObject();

//Saving the histroy for the histogram
SimpleList<double> tempHist;
SimpleList<int> timeHist;
SimpleList<double> setTempHist;

// Make sure to check the last time memeory got updated
unsigned long lastSaveTime = 0;

#ifdef DEBUG
const long intervalHist = 1000; //Faster History saving for debugging
#else
const long intervalHist = 30000; //Save a history record every 30sec
#endif
unsigned long previousMillis = intervalHist;  // Dernier point enregistré dans l'historique - time of last point added
int sizeHist = 120 ; //Number of records
char json[2400];                             // Buffer for json output string. This will increase by sizeHist * 166
#endif

void setup() {
  // Debugging serial
  Serial.begin(115200);

  // Pin setup
  pinMode(FAN_PIN, OUTPUT);
  //  analogWriteFreq(5000);

  // PID setup
  myPID.SetOutputLimits(MINFAN, 1023);
  myPID.SetMode(AUTOMATIC);
  m_setPoint = 210.0; // Default set point

  // WIFI connect
  WiFiManager wifiManager;
  wifiManager.autoConnect("Smoker"); // This will hold the connection till it get a connection
  WiFi.hostname("Smoker");
#ifdef WIFION


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

#ifdef MQTT
  //mqqt config
  Serial.print("mqtt server: ");
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);

  Serial.println(MQTT_SERVER);
#endif

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
  NTP.begin("pool.ntp.org", -1, true);
  NTP.setInterval(60000);

  // Load the flash
  history["s"] = 210;
  history["t"] = 0;
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

#ifdef DEBUG
  randomSeed(analogRead(0)); // To mimic temperature
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
    #ifdef DEBUG
      m_temp = random(175, 210);
    #else
      m_temp = round(ktc.readFahrenheit()*10)/10.0;
    #endif

    displayData(); // Display the data on the LCD


    // Send notifications to openhab
    setNotifications();
    #ifdef MQTT
    sendNotifications();
    #endif

    myPID.Compute(); // PID loop calculations

    // Update openhab with the latest updates
    publishDouble (mqttTempTopic, m_temp);
    publishDouble (mqttSetTempStateTopic, m_setPoint);
    publishInt (mqttFanSpeedTopic, map (m_output, MINFAN, 1023, 0, 100));
  }

if(m_temp > (m_setPoint+15))
  fanSpeed (MINFAN);
else
  fanSpeed ((int)m_output);
#ifdef WEBPAGE
  addPtToHist ();
#endif
}

void displayData() {
  // Debugging stuff
  Serial.print("C = ");
  Serial.print(ktc.readCelsius());
  Serial.print("\t F = ");
  Serial.println(ktc.readFahrenheit());
  Serial.print ("pid output: ");
  Serial.println (m_output);

#ifdef LCDON
  display.setTextSize(1);
  display.setCursor(5, 1);
  display.print (m_output);
  if (m_output <= MINFAN)
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
    if ( tps > 0 )
    {
      tempHist.push_back (m_temp);
      timeHist.push_back (tps);
      setTempHist.push_back (m_setPoint);

      if ( tempHist.size() > sizeHist )
      {
        tempHist.erase (tempHist.begin ());
        timeHist.erase (timeHist.begin ());
        setTempHist.erase (setTempHist.begin ());
      }
      Serial.print("size hist_t "); Serial.println(tempHist.size());
      if (lastSaveTime < history["t"])
      {
        saveHistory();
        lastSaveTime = history["t"];
        Serial.println("After the save");

      }
    }
  }
}

void sendMesures() {
  Serial.println("Send measures");
  String tempJson = "{\"t\":\"" + String(m_temp) + "\",";
  tempJson += "\"s\":\"" + String(m_setPoint) + "\",";
  tempJson += "\"o\":\"" + String(map (m_output, MINFAN, 1023, 0, 100)) + "\",";
  tempJson += "\"p\":\"" + String(Kp) + "\",";
  tempJson += "\"i\":\"" + String(Ki) + "\",";
  tempJson += "\"w\":\"" + String(heatNotification);
  tempJson += "\"}";

  m_server.send(200, "application/json", tempJson);
}

void sendHistory() {
  Serial.println("Send History");
  DynamicJsonBuffer jsonBuffer;                 // Buffer static contenant le JSON courant - Current JSON static buffer
  JsonObject& root = jsonBuffer.createObject();
  JsonArray& timestamp = root.createNestedArray("timestamp");
  JsonArray& hist_t = root.createNestedArray("t");
  JsonArray& hist_s = root.createNestedArray("s");

  for (SimpleList<int>::iterator it = timeHist.begin() ; it != timeHist.end(); ++it)
    timestamp.add(*it);

  for (SimpleList<double>::iterator it = tempHist.begin() ; it != tempHist.end(); ++it)
    hist_t.add(*it);

  for (SimpleList<double>::iterator it = setTempHist.begin() ; it != setTempHist.end(); ++it)
    hist_s.add(*it);

  root.printTo(json, sizeof(json));             // Export du JSON dans une chaine - Export JSON object as a string
  m_server.send(200, "application/json", json);   // Envoi l'historique au client Web - Send history data to the web client
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
      StaticJsonBuffer<150> jsonBuffer;
      JsonObject& tempRoot = jsonBuffer.parseObject(buf.get());
      if (!tempRoot.success()) {
        Serial.println("Impossible to read JSON file");
      } else {
        Serial.println("History loaded");
        tempRoot.prettyPrintTo(Serial);
        Serial.print("setting the set temp : ");
        if (tempRoot["s"] != 0)
          m_setPoint = tempRoot["s"];
        Serial.print (m_setPoint);
        Serial.print (" Last updated: ");
        Serial.print (NTP.getTimeDateString(tempRoot["t"]));
      }
    }
    file.close();
  }
}

void saveHistory()
{
  Serial.println("Save History");
  File historyFile = SPIFFS.open(HISTORY_FILE, "w");
  history.printTo(historyFile); // Exporte et enregsitre le JSON dans la zone SPIFFS - Export and save JSON object to SPIFFS area
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
  history["s"] = m_setPoint;
  history["t"] = NTP.getTime();

  Serial.println(m_setPoint);
  publishDouble (mqttSetTempStateTopic, m_setPoint);
  m_server.send(200, "text/plain", "success");          //Returns the HTTP response
}
// control the light intensity.
void fanSpeed (unsigned short pwmValue)
{
  if (pwmValue > MINFAN)
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
          Serial.println ("running out of time!!!!");
            notificationCounter = notificationCounter + 1;
            timeNotification = true;
            timeNotificationPreviousMillis  = millis();
        }

        if ((m_setPoint - m_temp) > 15) // if after 40 minutes the temp diffrrent between current and set is more than 15 send notification
        {
            heatNotification = true;
        }
        else if ((m_setPoint-5) <= m_temp)
        {
            heatNotification = false;
        }
    }
}

#ifdef MQTT
void sendNotifications()
{
    connectToOpenHab();
    #if !defined(DEBUG)
    // send time notification
    if (timeNotification && !previous_timeNotification)
    {
      client.publish(mqttTimeStatusTopic, "CLOSED");
      previous_timeNotification = timeNotification;
      timeNotification = false;
    }
    else if (!timeNotification && previous_timeNotification && (millis() - timeNotificationPreviousMillis) >= 500)
    {
      Serial.println ("setting it back!!!");
      client.publish(mqttTimeStatusTopic, "OPEN");
      previous_timeNotification = timeNotification;
    }

    // send heat notification
    if (heatNotification && !previous_heatNotification)
    {
      client.publish(mqttHeatStatusTopic, "CLOSED");
      previous_heatNotification = heatNotification;
    }
    else if (!heatNotification && previous_heatNotification)
    {
      client.publish(mqttHeatStatusTopic, "OPEN");
      previous_heatNotification = heatNotification;
    }
    #endif
}


void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  if(length > 0)
  {
    payload[length] = '\0';
    m_setPoint = String((char*) payload).toFloat ();
    history["s"] = m_setPoint;
    history["t"] = NTP.getTime();
    Serial.print("MQTT Set: ");
    Serial.println(m_setPoint);
  }
}

void reconnect()
{
  // Loop until we're reconnected
  if (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("smoky", "openhabian", "openhabian"))
    {
      Serial.println("connected");
      publishDouble (mqttSetTempStateTopic, m_setPoint);
      client.subscribe(mqttSetTempTopic);
      client.publish(mqttHeatStatusTopic, "OPEN");
      client.publish(mqttTimeStatusTopic, "OPEN");
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


void publishDouble (char* topic, double value)
{
  #ifdef MQTT
  char buf[10];
  String (value).toCharArray(buf, 5);
  client.publish(topic, buf);
  #endif
}

void publishInt (char* topic, int value)
{
  #ifdef MQTT
  char buf[10];
  String (value).toCharArray(buf, 5);
  client.publish(topic, buf);
  #endif
}
