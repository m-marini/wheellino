#ifndef ApiServer_h
#define ApiServer_h

#include <Arduino.h>
#include <HTTPClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h>
#include <ArduinoJson.h>

#include "WiFiModule.h"

/*
   Handles the api server used to manage wifi module configuration
*/
class ApiServerClass {
  public:
    /*
       Creates api server
    */
    ApiServerClass(const unsigned port = 80) :
      _apiServer(port) {}

    /*
       Initializes api server
    */
    void begin(void);

    /*
       Sets the WiFiModule
    */
    void wiFiModule(WiFiModuleClass* module) {
      _wiFiModule = module;
    }
    /*
       Handlers of api requests
    */
    void handleNotFound(AsyncWebServerRequest& request);
    void handleGetWiFiConfig(AsyncWebServerRequest& request);
    void handlePostWiFiConfig(AsyncWebServerRequest& request, JsonVariant& jsonDoc);
    void handleGetNetworkList(AsyncWebServerRequest& request);

    /*
       Sets callback on activity
    */
    void onActivity(void (*callback)(void *context, ApiServerClass&), void* context = NULL) {
      _onActivity = callback;
      _context = context;
    }
    /*
       Reloads the network list if required
    */
    void polling(const unsigned long t0 = millis());

  private:
    AsyncWebServer _apiServer;
    WiFiModuleClass* _wiFiModule;
    void* _context;
    void (*_onActivity)(void *, ApiServerClass&);
    AsyncWebServerRequest* _networkListRequest;

    void sendError(AsyncWebServerRequest& request, const t_http_codes code, const String& msg);
    const String methodName(AsyncWebServerRequest& request);
    void setActivity(void);
};

extern ApiServerClass ApiServer;

#endif
