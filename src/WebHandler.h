#ifndef WEB_HANDLER_H
#define WEB_HANDLER_H

#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

extern ESP8266WebServer server;

struct DataPoint {
    unsigned long timestamp;
    float voltage;
    bool outputActive;
    uint16_t pwmValue;
    bool outputMode; // true for PWM, false for Relay
    bool engineRunning;
};

extern DataPoint history[];
extern byte historyIndex;
extern bool relay_state;
extern bool engine_running;

void setupWebServer();
void handleRoot();
void handleData();
void handleHistory();

#endif