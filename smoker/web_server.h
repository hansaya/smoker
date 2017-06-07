#ifndef _WEB_SERVER_
#define _WEB_SERVER_

#include "time_ntp.h"
#include <WiFiUdp.h>

class WebInterface
{
  public:
    WebInterface (double &temp, double &setPoint):
      m_temp (&temp),
      m_setPoint (&setPoint) {}

    void Begin ();
    void dataLogging ();
    void serve ();

  private:

    String MakeHTTPFooter();
    String MakeHTTPHeader(unsigned long ulLength);
    unsigned long MakeTable (WiFiClient *pclient, bool bStream);
    unsigned long MakeList (WiFiClient *pclient, bool bStream);
    double *m_setPoint;
    double *m_temp;
};

#endif
