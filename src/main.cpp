#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "WebHandler.h"
#include "output_component.h"
#include <PID_v1.h>

// Extern AP credentials from secrets.h
extern const char* ap_ssid;
extern const char* ap_password;

// Configuration
#define CALIBRATION_IN_VOLTAGE        15.25
#define CALIBRATION_A0_VOLTAGE        2.90
#define VOLTAGE_THRESHOLD_HIGH        14.8
#define VOLTAGE_THRESHOLD_LOW         14.0
#define INPUT_VOLTAGE                 A0
#define SAMPLES                       5
#define LED_PIN                       D4
#define RELAY_PIN                     D3
#define RELAY_ACTIVATION_DELAY        20000
#define ALTERNATOR_ACTIVE_STATE       LOW
#define USE_PWM                      true
#define MAX_CHARGE_CURRENT_PWM       1024

// PID Configuration
#define PID_SAMPLE_TIME       1000   // ms
double pidInput, pidOutput;
double Setpoint=14.4, Kp=2, Ki=5, Kd=1;
PID chargePID(&pidInput, &pidOutput, &Setpoint, Kp, Ki, Kd, DIRECT);

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

void setupWiFiAP() {
  if (WiFi.getMode() == WIFI_AP) {
    return; // AP is already set up
  }

  // Configure the ESP as an access point
  WiFi.mode(WIFI_AP);

  // Set static IP for AP
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);

  bool apStarted = WiFi.softAP(ap_ssid, ap_password);

  if (apStarted) {
    Serial.println("\nAccess Point Started");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // Configure OTA
    ArduinoOTA.setHostname("smarttractor");
    ArduinoOTA.begin();

    setupWebServer();
  } else {
    Serial.println("\nFailed to start AP mode!");
  }
}

void setup() {
  Serial.begin(9600);

  // Hardware setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(RELAY_PIN, OUTPUT);
  alternator.off();

  // Initialize PID
  if(USE_PWM) {
    chargePID.SetMode(AUTOMATIC);
    chargePID.SetSampleTime(PID_SAMPLE_TIME);
    chargePID.SetOutputLimits(0, MAX_CHARGE_CURRENT_PWM);
  }

  // WiFi Access Point setup
  Serial.println("\nSetting up WiFi Access Point...");
  setupWiFiAP();
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
  float avg_reading = (total / SAMPLES) * (3.3 / 1024.0);

  // Apply calibration
  return (CALIBRATION_IN_VOLTAGE / CALIBRATION_A0_VOLTAGE) * avg_reading;
}

void manage_alternator() {
  if(USE_PWM) {
    // PID-based control
    if(millis() - lastPidUpdate >= PID_SAMPLE_TIME) {
      pidInput = current_voltage * 10;
      chargePID.Compute();
      lastPidUpdate = millis();
      // Safety limits
      if(current_voltage >= VOLTAGE_THRESHOLD_HIGH) {
        alternator.off();
      } else if(current_voltage <= VOLTAGE_THRESHOLD_LOW) {
        alternator.pwm(MAX_CHARGE_CURRENT_PWM);
      } else {
        int targetPWM = static_cast<int>(pidOutput);
        Serial.printf("PID - Current Voltage: %.2fV, Target PWM: %d\n", pidInput, targetPWM);
        alternator.pwm(targetPWM);
      }
    }
  } else {
    // Relay mode - hysteresis control
    if (last_state && current_voltage > VOLTAGE_THRESHOLD_HIGH) {
      last_state = false;
      next_relay_check = millis() + RELAY_ACTIVATION_DELAY;
      alternator.off();
    } else if (next_relay_check < millis() && !last_state && current_voltage < VOLTAGE_THRESHOLD_LOW) {
      last_state = true;
      next_relay_check = millis() + RELAY_ACTIVATION_DELAY;
      alternator.set(true);
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

// Modified toggle_led to reflect PWM status
void toggle_led() {
  const unsigned long now = millis();

  // Pattern configuration
  uint8_t blink_pattern =
    (!connected) ? 3 :                     // WiFi Failed: Double flash
    (alternator.isActive()) ?
      (alternator.isPWMEnabled() ? 2 : 1)  // PWM: Fast blink, Relay: Solid
      : 0;                                 // Inactive: Slow blink

  // Timing configuration
  const uint16_t intervals[] = {
    2000,  // Pattern 0: Slow blink (0.2Hz)
    1000,  // Pattern 1: Solid ON (1Hz)
    500,   // Pattern 2: Fast blink (2Hz)
    250    // Pattern 4: Double flash (0.5Hz)
  };

  // State machine
  if(now - last_led_toggle >= intervals[blink_pattern]) {
    switch(blink_pattern) {
      case 1: // Solid ON (Relay Active)
        digitalWrite(LED_PIN, alternator.isActive() ? LOW : HIGH);
        break;
      case 2: // Fast blink (2Hz) PWM active
        analogWrite(LED_PIN, alternator.getPWM());
      case 0: // Slow blink (1Hz) Alternator inactive
      case 3: // Double flash (0.5Hz) WiFi connection failed
        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;
        break;
    }
    last_led_toggle = now;
  }
}

unsigned long lastDataStore = 0;
unsigned long lastWifiConnection = 0;

void loop() {
  unsigned long currentMillis = millis();
  float current_voltage = read_voltage();

  // Check AP status
  if (WiFi.getMode() != WIFI_AP) {
    connected = false;
    if(currentMillis - lastWifiConnection >= 60000) { // Try to setup AP every 60 seconds if needed
      setupWiFiAP();
      lastWifiConnection = currentMillis;
    }
  } else {
    connected = true;
    ArduinoOTA.handle();
    server.handleClient();
  }

  manage_alternator();
  toggle_led();

  // Data Storage (5000ms interval)
  if(currentMillis - lastDataStore >= 5000) {
    store_data();
    lastDataStore = currentMillis;
  }
}
