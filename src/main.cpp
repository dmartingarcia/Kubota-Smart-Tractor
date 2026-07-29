#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "web/WebHandler.h"
#include "charging/output_component.h"
#include "charging/VoltageSensor.h"
#include "charging/AlternatorLogic.h"
#include "connectivity/WifiManager.h"
#include "connectivity/Esp8266WifiDriver.h"
#include "charging/PidAutotuner.h"
#include "storage/UsageCounters.h"
#include "storage/LittleFsStore.h"
#include "storage/MaintenanceLog.h"
#include "storage/LittleFsMaintenanceLogStore.h"
#include "connectivity/MqttPublisher.h"
#include "connectivity/PubSubMqttTransport.h"
#include "web/HttpsGpsServer.h"
#include <PID_v1.h>

// Extern WiFi credentials from secrets.h
extern const char* ap_ssid;
extern const char* ap_password;
extern const char* sta_ssid;
extern const char* sta_password;
extern const char* mqtt_host;
extern const uint16_t mqtt_port;
extern const char* mqtt_user;
extern const char* mqtt_password;

#define STA_CONNECT_TIMEOUT_MS   15000  // give up on home WiFi and fall back to AP after this
#define AP_RETRY_INTERVAL_MS     60000  // while in AP fallback, retry home WiFi this often

Esp8266WifiDriver wifiDriver;
WifiManager wifiManager(wifiDriver, sta_ssid, sta_password, ap_ssid, ap_password,
                         STA_CONNECT_TIMEOUT_MS, AP_RETRY_INTERVAL_MS);

// Configuration
#define CALIBRATION_IN_VOLTAGE        15.25
#define CALIBRATION_A0_VOLTAGE        2.90
#define VOLTAGE_THRESHOLD_HIGH        14.6
#define VOLTAGE_THRESHOLD_LOW         13.0
#define INPUT_VOLTAGE                 A0
#define SAMPLES                       5
#define LED_PIN                       D4
#define RELAY_PIN                     D3
#define RELAY_ACTIVATION_DELAY        20000
#define ALTERNATOR_ACTIVE_STATE       HIGH
#define USE_PWM                       true
#define MAX_CHARGE_CURRENT_PWM        1024

// PID Configuration
#define PID_SAMPLE_TIME       20   // ms
double pidInput, pidOutput;
double Setpoint=140.0, Kp=30, Ki=3, Kd=1;
PID chargePID(&pidInput, &pidOutput, &Setpoint, Kp, Ki, Kd, DIRECT);

// Relay-feedback autotune: swings output +-25% of full scale around mid-scale,
// noiseBand 0.2V (Setpoint is voltage*10), 6 half-cycles (~3 periods), 3min cap.
// Its output still goes through decide_pwm_safety_action every cycle, same as normal PID.
PidAutotuner pidAutotuner(Setpoint, MAX_CHARGE_CURRENT_PWM / 2.0, MAX_CHARGE_CURRENT_PWM / 4.0,
                          2.0, 6, 180000);
bool autotuneActive = false;

void start_pid_autotune() {
  pidAutotuner.begin(millis());
  autotuneActive = true;
}

// Charging-hours + maintenance counter, persisted to flash (throttled: saves every
// 300s of *accumulated active* runtime, not every loop iteration or wall-clock tick).
// Worst case on abrupt power loss (no shutdown signal on this tractor): up to 300s of
// unsaved hours, harmless for a maintenance counter (undercounts slightly, never over).
// Wear: LittleFS wear-levels across its partition; at this cadence even continuous
// charging stays far under typical NOR flash erase-cycle budgets over the unit's life.
LittleFsStore usageStore("/usage.bin");
UsageCounters usageCounters(usageStore, 250, 300); // default: service every 250h

// Maintenance logbook: append-only, written only on real (rare) user-logged service
// events, so flash wear is a non-issue here regardless of UsageCounters' cadence above.
LittleFsMaintenanceLogStore maintenanceLogStore("/maintenance_log.csv");
MaintenanceLog maintenanceLog(maintenanceLogStore);

// MQTT/Home Assistant: only reachable in CONNECTED_STA mode (see loop()). Buffers
// readings while offline/AP-fallback and flushes them once STA + broker are back.
PubSubMqttTransport mqttTransport(mqtt_host, mqtt_port, "smarttractor", mqtt_user, mqtt_password);
MqttPublisher mqttPublisher(mqttTransport, 20, 30000); // 20 readings buffered, retry every 30s

// Global state
OutputComponent alternator(RELAY_PIN, USE_PWM, ALTERNATOR_ACTIVE_STATE);
DataPoint history[120];
byte historyIndex = 0;
bool connected = false;
static bool last_state = false;
unsigned long next_relay_check = 0;
static unsigned long lastPidUpdate = 0;
float current_voltage = 0.0;
bool engine_running = false; // TODO: determine engine state from the voltage increment when enabling alternator

void setup() {
  Serial.begin(115200);

  // Hardware setup
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(RELAY_PIN, HIGH);
  delay(5000);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  delay(5000);


  // Initialize PID
  if(USE_PWM) {
    chargePID.SetMode(AUTOMATIC);
    chargePID.SetSampleTime(PID_SAMPLE_TIME);
    chargePID.SetOutputLimits(0, MAX_CHARGE_CURRENT_PWM);
  }

  // WiFi: try home network (STA) first, non-blocking; falls back to AP if it can't connect.
  // Web server/OTA are started once here since ESP8266WebServer/ArduinoOTA work over
  // whichever WiFi interface (STA or AP) ends up active.
  ArduinoOTA.setHostname("smarttractor");
  ArduinoOTA.begin();
  setupWebServer();
  wifiManager.begin(millis());
  usageCounters.begin(millis());

  // Best-effort NTP sync (non-blocking): only resolves once/if STA has internet.
  // Maintenance log entries use epochSeconds=0 (shown as "unknown date") until this lands.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

float read_voltage() {
  static int samples[SAMPLES];
  float total = 0;

  // Collect samples
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = analogRead(INPUT_VOLTAGE);
  }

  // Calculate moving average
  for (int i = 0; i < SAMPLES; i++) total += samples[i];
  float avg_reading = total / SAMPLES;

  return calibrate_voltage(avg_reading, 1024.0, 3.3, CALIBRATION_IN_VOLTAGE, CALIBRATION_A0_VOLTAGE);
}

int lastTargetPWM = 0;

void manage_alternator() {
  if(USE_PWM) {
    // PID-based control, on a short/stable cadence regardless of what else runs in loop()
    if(should_run_cycle(millis(), lastPidUpdate, PID_SAMPLE_TIME)) {
      pidInput = current_voltage * 10;
      lastPidUpdate = millis();
      if(!autotuneActive) chargePID.Compute();

      switch(decide_pwm_safety_action(current_voltage, VOLTAGE_THRESHOLD_HIGH, VOLTAGE_THRESHOLD_LOW)) {
        case ChargeAction::OFF:
          alternator.off();
          lastTargetPWM = 0;
          break;
        case ChargeAction::MAX_CHARGE:
          alternator.pwm(MAX_CHARGE_CURRENT_PWM);
          lastTargetPWM = MAX_CHARGE_CURRENT_PWM;
          break;
        case ChargeAction::RUN_PID: {
          if(autotuneActive) {
            double out = pidAutotuner.update(pidInput, millis());
            int targetPWM = constrain(static_cast<int>(out), 0, MAX_CHARGE_CURRENT_PWM);
            alternator.pwm(targetPWM);
            lastTargetPWM = targetPWM;

            if(pidAutotuner.state() == AutotuneState::SUCCEEDED) {
              AutotuneGains g = pidAutotuner.gains();
              chargePID.SetTunings(g.kp, g.ki, g.kd);
              Serial.printf("Autotune done: Kp=%.2f Ki=%.2f Kd=%.2f (Ku=%.2f Pu=%.0fms)\n",
                            g.kp, g.ki, g.kd, g.ku, g.pu);
              autotuneActive = false;
            } else if(pidAutotuner.state() == AutotuneState::FAILED) {
              Serial.println("Autotune failed, keeping previous tunings");
              autotuneActive = false;
            }
          } else {
            int targetPWM = static_cast<int>(pidOutput);
            alternator.pwm(targetPWM);
            analogWrite(LED_PIN, map(targetPWM, 0, MAX_CHARGE_CURRENT_PWM, 300, 1023));
            lastTargetPWM = targetPWM;
          }
          break;
        }
      }
    }
  } else {
    // Relay mode - hysteresis control
    RelayDecision decision = decide_relay_state(last_state, current_voltage, millis(),
                                                 next_relay_check, VOLTAGE_THRESHOLD_HIGH,
                                                 VOLTAGE_THRESHOLD_LOW, RELAY_ACTIVATION_DELAY);
    if(decision.changed) {
      last_state = decision.state;
      next_relay_check = decision.nextCheck;
      if(last_state) alternator.set(true);
      else alternator.off();
    }
  }
}

// Modified store_data to include PWM value
void store_data() {
  history[historyIndex] = {
    millis(),
    current_voltage,
    alternator.isActive(),
    alternator.getPWM(),  // Store PWM value
  };

  historyIndex = (historyIndex + 1) % 120;

  Serial.printf("Current Voltage: %.2fV\n", current_voltage);
  Serial.printf("Alternator State: %s\n", alternator.isActive() ? "ON" : "OFF");
  Serial.printf("PWM Value: %d\n", alternator.getPWM());
  Serial.printf("Relay State: %s\n", last_state ? "ON" : "OFF");
  Serial.printf("Next Relay Check: %lu\n", next_relay_check);
  Serial.printf("History Index: %d\n", historyIndex);
  Serial.printf("Connected: %s\n", connected ? "Yes" : "No");
  Serial.println();
}

unsigned long lastDataStore = 0;

void loop() {
  unsigned long currentMillis = millis();
  current_voltage = read_voltage();

  // Non-blocking: just polls state/kicks off connects, never waits.
  wifiManager.update(currentMillis);
  connected = (wifiManager.mode() == WifiMode::CONNECTED_STA);

  ArduinoOTA.handle();
  server.handleClient();
  gps_https_handle_client(); // no-op unless GPS mode was enabled from the dashboard

  manage_alternator();
  usageCounters.tick(alternator.isActive(), currentMillis);

  // Data storage + MQTT publish (5000ms interval, not every fast PID cycle)
  if(currentMillis - lastDataStore >= 5000) {
    store_data();
    lastDataStore = currentMillis;

    if(mqtt_host[0] != '\0') {
      MqttReading reading{currentMillis, current_voltage, alternator.getPWM(), alternator.isActive(),
                          engine_running, usageCounters.totalRunSeconds(), usageCounters.isMaintenanceDue()};
      mqttPublisher.update(wifiManager.mode() == WifiMode::CONNECTED_STA, reading, currentMillis);
    }
  }
}
