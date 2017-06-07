#include "web_server.h"

WiFiServer server(80);

// needed to avoid link error on ram check
extern "C"
{
#include "user_interface.h"
}

// ntp timestamp
unsigned long ulSecs2000_timer = 0;

#define KEEP_MEM_FREE 20240
#define MEAS_SPAN_H 6
unsigned long ulMeasCount = 0;  // values already measured
unsigned long ulNoMeasValues = 0; // size of array
unsigned long ulMeasDelta_ms;   // distance to next meas time
unsigned long ulNextMeas_ms;    // next meas time
unsigned long *pulTime;         // array for time points of measurements
float *pfTemp;           // array for temperature and humidity measurements

unsigned long ulReqcount;       // how often has a valid page been requested
unsigned long ulReconncount;    // how often did we connect to WiFi

void WebInterface::Begin ()
{
  // setup globals
  ulReqcount = 0;
  ulReconncount = 0;

  // allocate ram for data storage
  uint32_t free = system_get_free_heap_size() - KEEP_MEM_FREE;
  ulNoMeasValues = free / (sizeof(float) + sizeof(unsigned long)); // humidity & temp --> 2 + time
  pulTime = new unsigned long[ulNoMeasValues];
  pfTemp = new float[ulNoMeasValues];

  if (pulTime == NULL || pfTemp == NULL)
  {
    ulNoMeasValues = 0;
    Serial.println("Error in memory allocation!");
  }
  else
  {
    Serial.print("Allocated storage for ");
    Serial.print(ulNoMeasValues);
    Serial.println(" data points.");

    float fMeasDelta_sec = MEAS_SPAN_H * 3600. / ulNoMeasValues;
    ulMeasDelta_ms = ( (unsigned long)(fMeasDelta_sec + 0.5) ) * 1000; // round to full sec
    Serial.print("Measurements will happen each ");
    Serial.print(ulMeasDelta_ms);
    Serial.println(" ms.");

    ulNextMeas_ms = millis() + ulMeasDelta_ms;
  }

  ulReconncount++;

  ///////////////////////////////
  // connect to NTP and get time
  ///////////////////////////////
  ulSecs2000_timer = getNTPTimestamp();
  Serial.print("Current Time UTC from NTP server: " );
  Serial.println(epoch_to_string(ulSecs2000_timer).c_str());

  ulSecs2000_timer -= millis() / 1000; // keep distance to millis() counter

  // Start the server
  server.begin();
}

void WebInterface::dataLogging ()
{
  if (millis() >= ulNextMeas_ms)
  {
    ulNextMeas_ms = millis() + ulMeasDelta_ms;

    //    pfHum[ulMeasCount % ulNoMeasValues] = 0.0;     /////////////////////////////HUMEDAD
    pfTemp[ulMeasCount % ulNoMeasValues] = *m_temp; ///////////////////////////////TEMPERATURA  Value data
    pulTime[ulMeasCount % ulNoMeasValues] = millis() / 1000 + ulSecs2000_timer;

    Serial.print("Logging Temperature: ");
    Serial.print(pfTemp[ulMeasCount % ulNoMeasValues]);
    Serial.print(" deg Celsius");
    Serial.print("% - Time: ");
    Serial.println(pulTime[ulMeasCount % ulNoMeasValues]);

    ulMeasCount++;
  }

}

void WebInterface::serve ()
{
  ///////////////////////////////////
  // Check if a client has connected
  ///////////////////////////////////
  WiFiClient client = server.available();
  if (!client)
  {
    return;
  }

  // Wait until the client sends some data
  Serial.println("new client");
  unsigned long ultimeout = millis() + 250;
  while (!client.available() && (millis() < ultimeout) )
  {
    delay(1);
  }
  if (millis() > ultimeout)
  {
    Serial.println("client connection time-out!");
    return;
  }

  /////////////////////////////////////
  // Read the first line of the request
  /////////////////////////////////////
  String sRequest = client.readStringUntil('\r');
  //Serial.println(sRequest);
  client.flush();

  // stop client, if request is empty
  if (sRequest == "")
  {
    Serial.println("empty request! - stopping client");
    client.stop();
    return;
  }

  // get path; end of path is either space or ?
  // Syntax is e.g. GET /?show=1234 HTTP/1.1
  String sPath = "", sParam = "", sCmd = "";
  String sGetstart = "GET ";
  int iStart, iEndSpace, iEndQuest;
  iStart = sRequest.indexOf(sGetstart);
  if (iStart >= 0)
  {
    iStart += +sGetstart.length();
    iEndSpace = sRequest.indexOf(" ", iStart);
    iEndQuest = sRequest.indexOf("?", iStart);

    // are there parameters?
    if (iEndSpace > 0)
    {
      if (iEndQuest > 0)
      {
        // there are parameters
        sPath  = sRequest.substring(iStart, iEndQuest);
        sParam = sRequest.substring(iEndQuest, iEndSpace);
      }
      else
      {
        // NO parameters
        sPath  = sRequest.substring(iStart, iEndSpace);
      }
    }
  }


  ///////////////////////////
  // format the html response
  ///////////////////////////
  String sResponse, sResponse2, sHeader;

  /////////////////////////////
  // format the html page for /
  /////////////////////////////
  if (sPath == "/")
  {
    ulReqcount++;
    int iIndex = (ulMeasCount - 1) % ulNoMeasValues;
    sResponse  = F("<html>\n<head>\n<title>Smoky  </title>\n<script type=\"text/javascript\" src=\"https://www.google.com/jsapi?autoload={'modules':[{'name':'visualization','version':'1','packages':['gauge']}]}\"></script>\n<script type=\"text/javascript\">\nvar temp=");
    sResponse += pfTemp[iIndex];
    sResponse += F(",setP=");
    sResponse += *m_setPoint;
    sResponse += F(";\ngoogle.load('visualization', '1', {packages: ['gauge']});google.setOnLoadCallback(drawgaugetemp);google.setOnLoadCallback(drawgaugehum);\nvar gaugetempOptions = {min: -20, max: 400, yellowFrom: -20, yellowTo: 150,redFrom: 300, redTo: 400, minorTicks: 10, majorTicks: ['50','100','150','200','250','300','350','400']};\n");
    sResponse += F("var gaugehumOptions = {min: 0, max: 400, yellowFrom: 0, yellowTo: 175, redFrom: 325, redTo: 400, minorTicks: 10, majorTicks: ['50','100','150','200','250','300','350','400']};\nvar gaugetemp,gaugehum;\n\nfunction drawgaugetemp() {\ngaugetempData = new google.visualization.DataTable();\n");
    sResponse += F("gaugetempData.addColumn('number', '\260F');\ngaugetempData.addRows(1);\ngaugetempData.setCell(0, 0, temp);\ngaugetemp = new google.visualization.Gauge(document.getElementById('gaugetemp_div'));\ngaugetemp.draw(gaugetempData, gaugetempOptions);\n}\n\n");
    sResponse += F("function drawgaugehum() {\ngaugehumData = new google.visualization.DataTable();\ngaugehumData.addColumn('number', 'S \260F');\ngaugehumData.addRows(1);\ngaugehumData.setCell(0, 0, setP);\ngaugehum = new google.visualization.Gauge(document.getElementById('gaugehum_div'));\ngaugehum.draw(gaugehumData, gaugehumOptions);\n}\n");
    sResponse += F("</script>\n</head>\n<body>\n<font color=\"#000000\"><body bgcolor=\"#d0d0f0\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\"><h1>Smoky  </h1>Smoker Regulator <BR><BR><FONT SIZE=+1> Last measurement to ");
    sResponse += epoch_to_string(pulTime[iIndex]).c_str();
    sResponse += F(" UTC<BR>\n<div id=\"gaugetemp_div\" style=\"float:left; width:160px; height: 160px;\"></div> \n<div id=\"gaugehum_div\" style=\"float:left; width:160px; height: 160px;\"></div>\n<div style=\"clear:both;\"></div>");

    sResponse2 = F("<p>Temperature load pages :<BR><a href=\"/grafik\">Grafic</a>     <a href=\"/tabelle\">Table</a></p>");
    sResponse2 += MakeHTTPFooter().c_str();

    // Send the response to the client
    client.print(MakeHTTPHeader(sResponse.length() + sResponse2.length()).c_str());
    client.print(sResponse);
    client.print(sResponse2);
  }
  else if (sPath == "/tabelle")
    ////////////////////////////////////
    // format the html page for /tabelle
    ////////////////////////////////////
  {
    ulReqcount++;
    unsigned long ulSizeList = MakeTable(&client, false); // get size of table first

    sResponse  = F("<html><head><title>WLAN Logger Temperature ESP8266 - Google Charts </title></head><body>");
    sResponse += F("<font color=\"#000000\"><body bgcolor=\"#b0b0b0\">");
    sResponse += F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\">");
    sResponse += F("<h1>WLAN Logger Temperature </h1>");
    sResponse += F("<FONT SIZE=+1>");
    sResponse += F("<a href=\"/\">Dashboard</a><BR><BR>Recent measurements every");
    sResponse += ulMeasDelta_ms;
    sResponse += F("ms<BR>");
    // here the big table will follow later - but let us prepare the end first

    // part 2 of response - after the big table
    sResponse2 = MakeHTTPFooter().c_str();

    // Send the response to the client - delete strings after use to keep mem low
    client.print(MakeHTTPHeader(sResponse.length() + sResponse2.length() + ulSizeList).c_str());
    client.print(sResponse); sResponse = "";
    MakeTable(&client, true);
    client.print(sResponse2);
  }
  else if (sPath == "/grafik")
    ///////////////////////////////////
    // format the html page for /grafik
    ///////////////////////////////////
  {
    ulReqcount++;
    unsigned long ulSizeList = MakeList(&client, false); // get size of list first

    sResponse  = F("<html>\n<head>\n<title>Worlds best smoker </title>\n<script type=\"text/javascript\" src=\"https://www.google.com/jsapi?autoload={'modules':[{'name':'visualization','version':'1','packages':['corechart']}]}\"></script>\n");
    sResponse += F("<script type=\"text/javascript\"> google.setOnLoadCallback(drawChart);\nfunction drawChart() {var data = google.visualization.arrayToDataTable([\n['Zeit / UTC', 'Temperature', 'Set Temp'],\n");
    // here the big list will follow later - but let us prepare the end first

    // part 2 of response - after the big list
    sResponse2  = F("]);\nvar options = {title: 'course',vAxes:{0:{viewWindowMode:'explicit',gridlines:{color:'black'},format:\"##.##\260C\"},1: {gridlines:{color:'transparent'},format:\"##.##\260C\"},},series:{0:{targetAxisIndex:0},1:{targetAxisIndex:1},},curveType:'none',legend:{ position: 'bottom'}};");
    sResponse2 += F("var chart = new google.visualization.LineChart(document.getElementById('curve_chart'));chart.draw(data, options);}\n</script>\n</head>\n");
    sResponse2 += F("<body>\n<font color=\"#000000\"><body bgcolor=\"#d0d0f0\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\"><h1>Smoker Temperature Logger </h1><a href=\"/\">Dashboard</a><BR>");
    sResponse2 += F("<BR>\n<div id=\"curve_chart\" style=\"width: 600px; height: 400px\"></div>");
    sResponse2 += MakeHTTPFooter().c_str();

    // Send the response to the client - delete strings after use to keep mem low
    client.print(MakeHTTPHeader(sResponse.length() + sResponse2.length() + ulSizeList).c_str());
    client.print(sResponse); sResponse = "";
    MakeList(&client, true);
    client.print(sResponse2);
  }
  else
    ////////////////////////////
    // 404 for non-matching path
    ////////////////////////////
  {
    sResponse = "<html><head><title>404 Not Found</title></head><body><h1>Not Found</h1><p>The requested URL was not found on this server.</p></body></html>";

    sHeader  = F("HTTP/1.1 404 Not found\r\nContent-Length: ");
    sHeader += sResponse.length();
    sHeader += F("\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");

    // Send the response to the client
    client.print(sHeader);
    client.print(sResponse);
  }

  // and stop the client
  client.stop();
  Serial.println("Client disconnected");
}

////////////////////////////////////////////////////
// make google chart object table for measured data
////////////////////////////////////////////////////
unsigned long WebInterface::MakeList (WiFiClient *pclient, bool bStream)
{
  unsigned long ulLength = 0;

  // here we build a big list.
  // we cannot store this in a string as this will blow the memory
  // thus we count first to get the number of bytes and later on
  // we stream this out
  if (ulMeasCount > 0)
  {
    unsigned long ulBegin;
    if (ulMeasCount > ulNoMeasValues)
    {
      ulBegin = ulMeasCount - ulNoMeasValues;
    }
    else
    {
      ulBegin = 0;
    }

    String sTable = "";
    for (unsigned long li = ulBegin; li < ulMeasCount; li++)
    {
      // result shall be ['18:24:08 - 21.5.2015',21.10,49.00],
      unsigned long ulIndex = li % ulNoMeasValues;
      sTable += "['";
      sTable += epoch_to_string(pulTime[ulIndex]).c_str();
      sTable += "',";
      sTable += pfTemp[ulIndex];
      sTable += ",";
      sTable += *m_setPoint;
      sTable += "],\n";

      // play out in chunks of 1k
      if (sTable.length() > 1024)
      {
        if (bStream)
        {
          pclient->print(sTable);
          //pclient->write(sTable.c_str(),sTable.length());
        }
        ulLength += sTable.length();
        sTable = "";
      }
    }

    // remaining chunk
    if (bStream)
    {
      pclient->print(sTable);
      //pclient->write(sTable.c_str(),sTable.length());
    }
    ulLength += sTable.length();
  }

  return (ulLength);
}

/////////////////////////////////////
// make html table for measured data
/////////////////////////////////////
unsigned long WebInterface::MakeTable (WiFiClient *pclient, bool bStream)
{
  unsigned long ulLength = 0;

  // here we build a big table.
  // we cannot store this in a string as this will blow the memory
  // thus we count first to get the number of bytes and later on
  // we stream this out
  if (ulMeasCount == 0)
  {
    String sTable = "No data feature navigation use gbar.<BR>";      ///
    if (bStream)
    {
      pclient->print(sTable);
    }
    ulLength += sTable.length();
  }
  else
  {
    unsigned long ulEnd;
    if (ulMeasCount > ulNoMeasValues)
    {
      ulEnd = ulMeasCount - ulNoMeasValues;
    }
    else
    {
      ulEnd = 0;
    }

    String sTable;
    sTable = "<table style=\"width:100%\"><tr><th>Time  / UTC</th><th>T &deg;C</th><th>Set Temp &#037;</th></tr>";
    sTable += "<style>table, th, td {border: 2px solid black; border-collapse: collapse;} th, td {padding: 5px;} th {text-align: left;}</style>";
    for (unsigned long li = ulMeasCount; li > ulEnd; li--)
    {
      unsigned long ulIndex = (li - 1) % ulNoMeasValues;
      sTable += "<tr><td>";
      sTable += epoch_to_string(pulTime[ulIndex]).c_str();
      sTable += "</td><td>";
      sTable += pfTemp[ulIndex];
      sTable += "</td><td>";
      sTable += *m_setPoint;
      sTable += "</td></tr>";

      // play out in chunks of 1k
      if (sTable.length() > 1024)
      {
        if (bStream)
        {
          pclient->print(sTable);
          //pclient->write(sTable.c_str(),sTable.length());
        }
        ulLength += sTable.length();
        sTable = "";
      }
    }

    // remaining chunk
    sTable += "</table>";
    ulLength += sTable.length();
    if (bStream)
    {
      pclient->print(sTable);
      //pclient->write(sTable.c_str(),sTable.length());
    }
  }

  return (ulLength);
}

////////////////////
// make html footer
////////////////////
String WebInterface::MakeHTTPFooter()
{
  String sResponse;

  sResponse  = F("<FONT SIZE=-2><BR> call Navigation use counter=");
  sResponse += ulReqcount;
  sResponse += F(" - connection  Navigation use counter=");
  sResponse += ulReconncount;
  sResponse += F(" - Free RAM=");
  sResponse += (uint32_t)system_get_free_heap_size();
  sResponse += F(" - Max.data points=");
  sResponse += ulNoMeasValues;
  sResponse += F("<BR>Hans Perera 06/2017<BR></body></html>");

  return (sResponse);
}

//////////////////////////
// create HTTP 1.1 header
//////////////////////////
String WebInterface::MakeHTTPHeader(unsigned long ulLength)
{
  String sHeader;

  sHeader  = F("HTTP/1.1 200 OK\r\nContent-Length: ");
  sHeader += ulLength;
  sHeader += F("\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");

  return (sHeader);
}
