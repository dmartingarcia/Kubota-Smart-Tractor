#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "WebHandler.h"
#include "output_component.h"
#include <PID_v1.h>

// Configuration
#define CALIBRATION_IN_VOLTAGE        15.25
#define CALIBRATION_A0_VOLTAGE        2.90
#define VOLTAGE_THRESHOLD_HIGH        14.8
#define VOLTAGE_THRESHOLD_LOW         14.0
#define ENGINE_VOLTAGE_RISE_THRESHOLD 1.0    // Minimum voltage jump to detect engine
#define INPUT_VOLTAGE                 A0
#define SAMPLES                       10
#define LED_PIN                       D4
#define RELAY_PIN                     D1
#define RELAY_ACTIVATION_DELAY        2000
#define ALTERNATOR_ACTIVE_STATE       LOW
#define USE_PWM                      false
#define MAX_CHARGE_CURRENT_PWM       800

// PID Configuration
#define PID_SETPOINT          14.6  // Optimal charging voltage
#define PID_SAMPLE_TIME       100   // ms
double pidInput, pidOutput, pidSetpoint = PID_SETPOINT;
PID chargePID(&pidInput, &pidOutput, &pidSetpoint, 100.0, 10.0, 1.0, DIRECT);

// Global state
OutputComponent alternator(RELAY_PIN, USE_PWM, ALTERNATOR_ACTIVE_STATE);
DataPoint history[120];
byte historyIndex = 0;
bool engine_running = false;
bool connected = true;
static bool last_state = false;
unsigned long next_relay_check = 0;
static unsigned long lastPidUpdate = 0;
static uint16_t lastPWM = 0;
static unsigned long last_toggle = 0;
static bool led_state = HIGH;

void connectToWiFi() {
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int retry = 15;
  while (WiFi.status() != WL_CONNECTED && retry-- > 0) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    connected = false;
    Serial.println("\nWiFi failed!");
  } else {
    connected = true;
    ArduinoOTA.setHostname("smarttractor");
    ArduinoOTA.begin();

    Serial.println("\nWiFi connected\nIP: " + WiFi.localIP().toString());

    setupWebServer();
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

  // WiFi connection
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  connectToWiFi();
}

float read_voltage() {
  static int samples[SAMPLES];
  static int index = 0;
  float total = 0;

  // Collect samples
  samples[index] = analogRead(INPUT_VOLTAGE);
  index = (index + 1) % SAMPLES;

  // Calculate moving average
  for (int i = 0; i < SAMPLES; i++) total += samples[i];
  float avg_reading = (total / SAMPLES) * (3.3 / 1024.0);

  // Apply calibration
  return (CALIBRATION_IN_VOLTAGE / CALIBRATION_A0_VOLTAGE) * avg_reading;
}

void manage_alternator() {
  float current_voltage = read_voltage();
  Serial.printf("Current Voltage: %.2fV\n", current_voltage);
  Serial.printf("Alternator State: %s\n", alternator.isActive() ? "ON" : "OFF");
  Serial.printf("PWM Value: %d\n", alternator.getPWM());
  Serial.printf("Engine Running: %s\n", engine_running ? "YES" : "NO");
  Serial.printf("Relay State: %s\n", last_state ? "ON" : "OFF");
  Serial.printf("Next Relay Check: %lu\n", next_relay_check);
  Serial.println();

  if(USE_PWM) {
    // PID-based control
    if(millis() - lastPidUpdate >= PID_SAMPLE_TIME) {
      pidInput = current_voltage;
      chargePID.Compute();
      lastPidUpdate = millis();

      // Safety limits
      if(current_voltage >= VOLTAGE_THRESHOLD_HIGH) {
        alternator.off();
        lastPWM = 0;
      } else if(current_voltage <= VOLTAGE_THRESHOLD_LOW) {
        alternator.pwm(MAX_CHARGE_CURRENT_PWM);
        lastPWM = MAX_CHARGE_CURRENT_PWM;
      } else {

        // Apply PID-calculated PWM with smooth transitions
        int targetPWM = static_cast<int>(pidOutput);

        // Smooth transition (max 1% change per cycle)
        if(targetPWM > lastPWM) {
          targetPWM = min(lastPWM + MAX_CHARGE_CURRENT_PWM/10, targetPWM);
        } else {
          targetPWM = max(lastPWM - MAX_CHARGE_CURRENT_PWM/10, targetPWM);
        }

        alternator.pwm(targetPWM);
        lastPWM = targetPWM;
      }
    }
  } else {
    // Relay mode - hysteresis control
    if (last_state && current_voltage > VOLTAGE_THRESHOLD_HIGH) {
      last_state = false;
      next_relay_check = millis() + RELAY_ACTIVATION_DELAY;
      alternator.off();
    } else if (!last_state && current_voltage < VOLTAGE_THRESHOLD_LOW) {
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
    read_voltage(),
    alternator.isActive(),
    alternator.getPWM(),  // Store PWM value
    engine_running
  };
  historyIndex = (historyIndex + 1) % 120;
}

// Modified toggle_led to reflect PWM status
void toggle_led() {
  const unsigned long now = millis();

  // Pattern configuration
  const uint8_t blink_pattern =
    (!connected) ? 4 :                     // WiFi Failed: Double flash
    (alternator.isActive()) ?
      (alternator.isPWMEnabled() ? 2 : 1)  // PWM: Fast blink, Relay: Solid
      : 0;                                 // Inactive: Slow blink

  // Timing configuration
  const uint16_t intervals[] = {
    5000,  // Pattern 0: Slow blink (0.2Hz)
    1000,  // Pattern 1: Solid ON (1Hz)
    500,   // Pattern 2: Fast blink (2Hz)
    200    // Pattern 4: Double flash (0.5Hz)
  };

  // State machine
  if(now - last_toggle >= intervals[blink_pattern]) {
    switch(blink_pattern) {
      case 0: // Slow blink (1Hz) Alternator inactive
        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;
        break;

      case 1: // Solid ON (Relay Active)
        digitalWrite(LED_PIN, alternator.isActive() ? LOW : HIGH);
        break;

      case 2: // Fast blink (2Hz) PWM active
        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;
        break;

      case 4: // Double flash (0.5Hz) WiFi connection failed
        static uint8_t flash_count = 0;
        if(flash_count < 2) {
          digitalWrite(LED_PIN, LOW);
          delay(50);
          digitalWrite(LED_PIN, HIGH);
          flash_count++;
        } else {
          if(now - last_toggle >= 2000) {
            flash_count = 0;
          }
        }
        break;
    }
    last_toggle = now;
  }
}

unsigned long lastDataStore = 0;
unsigned long lastLedUpdate = 0;

void loop() {
  unsigned long currentMillis = millis();

  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  } else {
    ArduinoOTA.handle();
    server.handleClient();
  }

  if(engine_running) {
    manage_alternator();
  } else {
    // Single engine check
    float initial_voltage = read_voltage();
    digitalWrite(RELAY_PIN, ALTERNATOR_ACTIVE_STATE);
    delay(1000);
    float current_voltage = read_voltage();
    digitalWrite(RELAY_PIN, !ALTERNATOR_ACTIVE_STATE);
    engine_running = (current_voltage - initial_voltage) > ENGINE_VOLTAGE_RISE_THRESHOLD;
  }

  // LED Update (100ms interval)
  if(currentMillis - lastLedUpdate >= 100) {
    toggle_led();
    lastLedUpdate = currentMillis;
  }

  // Data Storage (500ms interval)
  if(currentMillis - lastDataStore >= 500) {
    store_data();
    lastDataStore = currentMillis;
  }
}
