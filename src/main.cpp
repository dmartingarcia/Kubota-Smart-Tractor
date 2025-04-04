#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "secrets.h"
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

// Configuration
#define CALIBRATION_IN_VOLTAGE    15.25    // Measured actual voltage during calibration
#define CALIBRATION_A0_VOLTAGE    2.90     // Voltage read at A0 during calibration
#define VOLTAGE_THRESHOLD_HIGH    14.8     // Upper threshold (turn off alternator)
#define VOLTAGE_THRESHOLD_LOW     14.0     // Lower threshold (turn on alternator)
#define VOLTAGE_HYSTERESIS        0.8      // Prevent rapid switching
#define INPUT_VOLTAGE             A0
#define SAMPLES                   10       // Moving average samples
#define LED_PIN                   D4
#define RELAY_PIN                 D1
#define RELAY_ACTIVATION_DELAY    2000     // 2-second delay for relay changes
#define ALTERNATOR_ACTIVE_STATE   LOW      // Verify relay logic (LOW/HIGH)

// Global state
bool relay_state = ALTERNATOR_ACTIVE_STATE;
unsigned long next_relay_check = 0;
bool connected = true;

void setup() {
  Serial.begin(9600);
  
  // Hardware setup
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, ALTERNATOR_ACTIVE_STATE);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // WiFi connection
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int retry = 15;
  while (WiFi.status() != WL_CONNECTED && retry-- > 0) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.setHostname("smarttractor");
    ArduinoOTA.begin();
    Serial.println("\nWiFi connected\nIP: " + WiFi.localIP().toString());
  } else {
    connected = false;
    Serial.println("\nWiFi failed!");
  }
}

void toggle_led() {
  static bool led_state = HIGH;
  digitalWrite(LED_PIN, relay_state == ALTERNATOR_ACTIVE_STATE ? LOW : led_state);
  led_state = !led_state;
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

void manage_relay(float voltage) {
  static bool last_state = false;
  bool should_activate = voltage > (last_state ? VOLTAGE_THRESHOLD_LOW : VOLTAGE_THRESHOLD_HIGH);

  if (millis() > next_relay_check) {
    if (should_activate != last_state) {
      relay_state = should_activate ? !ALTERNATOR_ACTIVE_STATE : ALTERNATOR_ACTIVE_STATE;
      digitalWrite(RELAY_PIN, relay_state);
      next_relay_check = millis() + RELAY_ACTIVATION_DELAY;
      last_state = should_activate;
      Serial.println("State changed: " + String(relay_state ? "OFF" : "ON"));
    }
  }
}

void loop() {
  if (connected) {
    ArduinoOTA.handle();
  }

  toggle_led();
  float current_voltage = read_voltage();
  
  Serial.print("Voltage: ");
  Serial.print(current_voltage, 2);
  Serial.println("V");

  manage_relay(current_voltage);
  delay(500);
}
