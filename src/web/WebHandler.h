#ifndef WEB_HANDLER_H
#define WEB_HANDLER_H

#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include "../storage/UsageCounters.h"
#include "../storage/MaintenanceLog.h"

extern ESP8266WebServer server;

#define HISTORY_SIZE 120 // ~10 minutes at the 5s store_data() cadence

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
extern bool autotuneActive;
void start_pid_autotune();
extern UsageCounters usageCounters;
extern MaintenanceLog maintenanceLog;

void handleAutotuneStart();
void handleMaintenanceReset();
void handleMaintenanceInterval();
void handleMaintenancePage();
void handleMaintenanceLogList();
void handleMaintenanceLogAdd();
void handleGpsEnable();
void handleGpsDisable();
void handleGpsStatus();
void handleRestart();
void handleMqttTest();

void setupWebServer();
void handleRoot();
void handleData();
void handleHistory();

#endif