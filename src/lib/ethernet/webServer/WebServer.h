// -----------------------------------------------------------------------------------
// Web server
#pragma once

#include "../../../Common.h"

#if OPERATIONAL_MODE == ETHERNET_W5100 || OPERATIONAL_MODE == ETHERNET_W5500

  #include "../Ethernet.h"

  // Turn ON to allow webserver debug messages
  #define WEBSERVER_DEBUG OFF

  #if WEBSERVER_DEBUG == ON
    #define W(x) V(x)
    #define WF(x) VF(x)
    #define WL(x) VL(x)
    #define WLF(x) VLF(x)
  #else
    #define W(x)
    #define WF(x)
    #define WL(x)
    #define WLF(x)
  #endif

  // macros to help with sending webpage data
  #define sendHtmlStart()
  #define sendHtml(x) client->print(x);
  #define sendHtmlDone(x) client->print(x);

  // misc.
  #define WEB_SOCKET_TIMEOUT    10000
  #define HANDLER_COUNT_MAX     15
  #define PARAMETER_COUNT_MAX   6
  
  typedef void (* webFunction) (EthernetClient *client);
  
  class WebServer {
    public:
      void init();
      void handleClient();
      void setResponseHeader(const char *str);
      void on(String fn, webFunction handler);
      #if SD_CARD == ON
        void on(String fn);
      #endif
      void onNotFound(webFunction handler);
      String arg(String id);
  
      bool SDfound = false;

    private:
      int  getHandler(String* line);
      void processGet(String* line);
      void processPost(String* line);
 
      #if SD_CARD == ON
        void sdPage(String fn, EthernetClient* client);
      #endif
  
      char responseHeader[200] = "";
      #if SD_CARD == ON
        bool modifiedSinceFound = false;
      #endif
  
      webFunction notFoundHandler = NULL;
      webFunction handlers[HANDLER_COUNT_MAX];
      String handlers_fn[HANDLER_COUNT_MAX];
      int handler_count;
      
      String parameters[PARAMETER_COUNT_MAX];
      String values[PARAMETER_COUNT_MAX];
      int parameter_count;
  };

  extern WebServer www;

#endif
