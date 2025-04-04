#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "WebHandler.h"

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

// Global state
DataPoint history[120];
byte historyIndex = 0;
bool relay_state = ALTERNATOR_ACTIVE_STATE;
unsigned long next_relay_check = 0;
bool engine_running = false;
bool connected = true;
static bool last_state = false;

void setup() {
  Serial.begin(9600);

  // Hardware setup
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, ALTERNATOR_ACTIVE_STATE);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.print("Engine status: ");
  Serial.println(engine_running ? "RUNNING" : "STOPPED");

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
    setupWebServer();
    Serial.println("\nWiFi connected\nIP: " + WiFi.localIP().toString());
  } else {
    connected = false;
    Serial.println("\nWiFi failed!");
  }
}

float read_voltage() {
  static int samples[SAMPLES];
  static int index = 0;
  float total = 0;

  samples[index] = analogRead(INPUT_VOLTAGE);
  index = (index + 1) % SAMPLES;

  for (int i = 0; i < SAMPLES; i++) total += samples[i];
  float avg_reading = (total / SAMPLES) * (3.3 / 1024.0);
  return (CALIBRATION_IN_VOLTAGE / CALIBRATION_A0_VOLTAGE) * avg_reading;
}

void manage_alternator() {
  float current_voltage = read_voltage();
  bool old_state = last_state;

  if (last_state && current_voltage > VOLTAGE_THRESHOLD_HIGH) {
    last_state = false;
    next_relay_check = millis() + RELAY_ACTIVATION_DELAY;
    Serial.println("Over-voltage detected: " + String(current_voltage));
  }else if (millis() > next_relay_check && current_voltage < VOLTAGE_THRESHOLD_LOW) {
    last_state = true;
    Serial.println("Under-voltage detected: " + String(current_voltage));
    next_relay_check = millis() + RELAY_ACTIVATION_DELAY;
  }

  if (last_state == old_state) {
    // Toggle relay state
    relay_state = last_state ? !ALTERNATOR_ACTIVE_STATE : ALTERNATOR_ACTIVE_STATE;
    digitalWrite(RELAY_PIN, relay_state);
  }
}


void store_data() {
  history[historyIndex] = { millis(), read_voltage(), relay_state };
  historyIndex = (historyIndex + 1) % 120;
}

void toggle_led() {
  static bool led_state = HIGH;
  digitalWrite(LED_PIN, relay_state == ALTERNATOR_ACTIVE_STATE ? LOW : led_state);
  led_state = !led_state;
}

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
  }
}

void setupOTA() {
  ArduinoOTA.setHostname("smarttractor");
  ArduinoOTA.begin();
  Serial.println("\nWiFi connected\nIP: " + WiFi.localIP().toString());
}


void loop() {
  if (connected) {
    ArduinoOTA.handle();
    server.handleClient();
  }

  toggle_led();

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

  store_data();
  delay(500);
}
