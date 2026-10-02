#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "web/WebHandler.h"
#include "charging/output_component.h"
#include "charging/VoltageSensor.h"
#include "charging/AlternatorLogic.h"
#include "charging/EngineDetector.h"
#include "charging/EngineMode.h"
#include "charging/VoltageCalibration.h"
#include "charging/PidSettings.h"
#include "charging/PwmMirror.h"
#include "charging/StatusLed.h"
#include "charging/PidAutotuner.h"
#include "charging/RealTimePid.h"
#include "storage/UsageCounters.h"
#include "storage/LittleFsStore.h"
#include "storage/MaintenanceLog.h"
#include "storage/LittleFsMaintenanceLogStore.h"
#include "connectivity/MqttPublisher.h"
#include "connectivity/PubSubMqttTransport.h"
#include "gps/GpsReader.h"
#include "gps/GpsJumpFilter.h"
#include "gps/SpeedAverager.h"

// Extern WiFi credentials from secrets.h
extern const char* ap_ssid;
extern const char* ap_password;
extern const char* sta_ssid;
extern const char* sta_password;
extern const char* mqtt_host;
extern const uint16_t mqtt_port;
extern const char* mqtt_user;
extern const char* mqtt_password;

// Plain ESP8266 WiFi handling, no abstraction layer. Exclusive AP/STA (not AP_STA
// concurrent - that looked fine at first but proved unreliable across reboots on
// this hardware: the SDK reported success but the AP silently stopped broadcasting).
// The AP is up by default and stays up; a STA attempt briefly switches away from it,
// but only when nobody is currently connected to the AP.
#define STA_CONNECT_TIMEOUT_MS 30000  // give up on home WiFi and go back to AP after this
#define STA_RETRY_INTERVAL_MS  120000 // if not connected to home WiFi, retry this often

bool wifiConnectedSta = false;
bool staAttemptInProgress = false;
unsigned long staAttemptDeadline = 0;
unsigned long nextStaAttempt = 0;
// Diagnostics for the dashboard (no USB needed): how many STA attempts, and why the last one
// failed - WiFi.status(): 1 SSID not found, 4 connect failed (wrong password?), 6 disconnected.
int staAttempts = 0;
int staLastFailStatus = -1;

// Captive portal: redirects any DNS lookup from an AP client back to our own IP, so
// the phone's connectivity probe fails/redirects and it auto-shows the sign-in page.
DNSServer dnsServer;
const IPAddress apIP(192, 168, 4, 1);

// The ESP8266 defaults to modem sleep, which parks the radio between beacons: ~100ms pings and
// retransmission stalls of 1-2s on the dashboard. Powered from the tractor, so keep it awake.
void keepRadioAwake() {
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
}

void startAP() {
  WiFi.persistent(false);
  WiFi.disconnect(true);
  bool modeOk = WiFi.mode(WIFI_AP);
  keepRadioAwake();
  IPAddress gateway = apIP;
  IPAddress subnet(255, 255, 255, 0);
  bool cfgOk = WiFi.softAPConfig(apIP, gateway, subnet);
  bool apOk = WiFi.softAP(ap_ssid, ap_password);
  dnsServer.start(53, "*", apIP);
  Serial.printf("AP up: mode=%d cfg=%d ap=%d ip=%s heap=%u\n",
                modeOk, cfgOk, apOk, WiFi.softAPIP().toString().c_str(), ESP.getFreeHeap());
}

void updateWifi(unsigned long currentMillis) {
  if (staAttemptInProgress) {
    if (WiFi.status() == WL_CONNECTED) {
      staAttemptInProgress = false;
      wifiConnectedSta = true;
      keepRadioAwake();
      Serial.println("STA connected");
    } else if (currentMillis - staAttemptDeadline < (1UL << 31)) { // deadline reached (non-wrapping compare)
      staAttemptInProgress = false;
      staLastFailStatus = WiFi.status();
      startAP(); // STA attempt used the radio exclusively - bring the AP back
      nextStaAttempt = currentMillis + STA_RETRY_INTERVAL_MS;
      Serial.println("STA connection failed, back to AP");
    }
    return;
  }

  if (wifiConnectedSta) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnectedSta = false;
      startAP();
      nextStaAttempt = currentMillis + STA_RETRY_INTERVAL_MS;
      Serial.println("STA dropped, back to AP");
    }
    return;
  }

  // Sitting on AP only: consider retrying STA, but never interrupt someone actively
  // using the AP just to try the periodic reconnect.
  if (sta_ssid[0] != '\0' && currentMillis - nextStaAttempt < (1UL << 31)) {
    if (WiFi.softAPgetStationNum() > 0) {
      nextStaAttempt = currentMillis + STA_RETRY_INTERVAL_MS; // postpone, AP is in use
      return;
    }
    WiFi.mode(WIFI_STA);
    keepRadioAwake();
    WiFi.begin(sta_ssid, sta_password);
    staAttemptInProgress = true;
    staAttempts++;
    staAttemptDeadline = currentMillis + STA_CONNECT_TIMEOUT_MS;
    Serial.println("Trying STA...");
  }
}

// Configuration
#define CALIBRATION_IN_VOLTAGE        15.25
#define CALIBRATION_A0_VOLTAGE        2.90
#define VOLTAGE_THRESHOLD_HIGH        14.4
#define VOLTAGE_THRESHOLD_LOW         13.0
#define INPUT_VOLTAGE                 A0
#define SAMPLES                       5
#define LED_PIN                       D4
#define RELAY_PIN                     D3
#define GPS_RX_PIN                    D5 // ESP8266 RX, wired to the GPS module's TX
#define GPS_TX_PIN                    D6 // ESP8266 TX, wired to the GPS module's RX (optional, only needed to send it commands)
#define GPS_BAUD                      9600
#define RELAY_ACTIVATION_DELAY        20000
#define ALTERNATOR_ACTIVE_STATE       HIGH
#define USE_PWM                       true
#define MAX_CHARGE_CURRENT_PWM        1023 // matches analogWriteRange in OutputComponent
#define BOOT_BLINK_HALF_PERIOD_MS     200   // power-on LED blink: 3 pulses, 200ms on / 200ms off
#define BOOT_BLINK_COUNT              3
#define FAULT_BLINK_HALF_PERIOD_MS    120   // fault code: N quick pulses (see StatusLed.h), then FAULT_BLINK_PAUSE_MS dark
#define FAULT_BLINK_PAUSE_MS          1000
#define FAULT_BLINK_REPEATS           5     // show each new fault code this many times, then hand the LED back
#define LED_ACTIVE_LOW                true // Wemos D1 Mini on-board LED lights when D4 is LOW
#define ENGINE_PROBE_PULSE_MS         3000  // hold full output this long before checking for a voltage rise
#define ENGINE_PROBE_COOLDOWN_MS      8000  // wait this long before the next pulse if the engine looks off
#define ENGINE_PROBE_RISE_VOLTS       0.05  // minimum voltage rise during a pulse to call the engine running
#define ENGINE_GPS_MIN_SPEED_KMH      3.0   // average speed above this (valid GPS fix) means the engine is on
#define ENGINE_GPS_AVG_WINDOW_MS      15000 // ...averaged over this window, sampled once per second
#define ENGINE_RUNNING_GRACE_MS       300000 // after seeing the engine charging, ride out voltage sags this long before probing

// PID Configuration
#define PID_SAMPLE_TIME       20   // ms
double pidInput, pidOutput;
double Setpoint=140.0, Kp=30, Ki=3, Kd=1;
RealTimePid chargePID(Kp, Ki, Kd, 0, MAX_CHARGE_CURRENT_PWM);

// Kp/Ki/Kd above are the FACTORY gains. Autotune results are saved and loaded on boot; with
// nothing saved (or after a reset from the dashboard) the factory gains apply.
LittleFsStore pidStore("/pid.bin");
PidSettings pidSettings(pidStore, PidGains{Kp, Ki, Kd});

// Calibration aid: while paused the field is held off so the voltage settles and the user can
// compare the dashboard with a multimeter. Capped, and it simply expires on its own.
#define ALTERNATOR_PAUSE_MAX_MS 30000
unsigned long alternatorPauseUntil = 0;
void pause_alternator(unsigned long ms) {
  if(ms > ALTERNATOR_PAUSE_MAX_MS) ms = ALTERNATOR_PAUSE_MAX_MS;
  alternatorPauseUntil = millis() + ms;
}
void resume_alternator() { alternatorPauseUntil = millis(); }
unsigned long alternator_pause_remaining_ms() {
  unsigned long now = millis();
  return is_before_deadline(now, alternatorPauseUntil) ? alternatorPauseUntil - now : 0;
}

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
// 60s of *accumulated active* runtime, not every loop iteration or wall-clock tick).
// Worst case on abrupt power loss (no shutdown signal on this tractor): up to 60s of
// unsaved hours, harmless for a maintenance counter (undercounts slightly, never over).
// Wear: LittleFS wear-levels across its partition; at this cadence even continuous
// charging stays far under typical NOR flash erase-cycle budgets over the unit's life.
LittleFsStore usageStore("/usage.bin");
UsageCounters usageCounters(usageStore, 250, 60) ; // default: service every 250h

// Maintenance logbook: append-only, written only on real (rare) user-logged service
// events, so flash wear is a non-issue here regardless of UsageCounters' cadence above.
LittleFsMaintenanceLogStore maintenanceLogStore("/maintenance_log.csv");
MaintenanceLog maintenanceLog(maintenanceLogStore);

// MQTT/Home Assistant: only reachable in CONNECTED_STA mode (see loop()). Buffers
// readings while offline/AP-fallback and flushes them once STA + broker are back.
PubSubMqttTransport mqttTransport(mqtt_host, mqtt_port, "smarttractor", mqtt_user, mqtt_password);
MqttPublisher mqttPublisher(mqttTransport, 180, 30000); // 30min of samples at MQTT_SAMPLE_INTERVAL_MS, retry every 30s

// Sampling (buffer a reading) and publishing (actually send) run on separate cadences:
// sampling stays fast for good resolution/history even while offline, publishing is
// slower so a healthy connection doesn't get spammed with one message per sample.
#define MQTT_SAMPLE_INTERVAL_MS  10000
#define MQTT_PUBLISH_INTERVAL_MS 30000
unsigned long lastMqttSample = 0;
unsigned long lastMqttPublish = 0;

// GPS: optional hardware, gracefully absent if not wired up. isConnected()/hasFix()
// gate everything - dashboard/MQTT only ever show GPS data once it's actually valid.
GpsReader gpsReader(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

// Rejects momentary GPS position jumps a tractor can't physically make (60km/h implied
// speed cap; resyncs after 3 consecutive rejects rather than getting stuck on a bad seed).
GpsJumpFilter gpsJumpFilter(60.0, 3);
SpeedAverager gpsSpeedAvg(ENGINE_GPS_AVG_WINDOW_MS, 1000);

// User one-point voltage calibration (typed in from a multimeter on the dashboard), applied on
// top of the fixed divider calibration. unscaledVoltage is the reading BEFORE that scale, which
// is what the calibration is computed against.
LittleFsStore voltageCalStore("/voltage_cal.bin");
VoltageCalibration voltageCalibration(voltageCalStore);
float unscaledVoltage = 0;

// Which signals may declare "engine on" (always / GPS motion / alternator probing, any
// combination), user-selectable from the dashboard and persisted.
LittleFsStore engineModeStore("/engine_mode.bin");
EngineModeSettings engineModeSettings(engineModeStore);
bool gpsHasAcceptedFix = false;

// Global state
// LED_PIN mirrors the alternator output PWM 1:1 (same duty, every write path).
OutputComponent alternator(RELAY_PIN, USE_PWM, ALTERNATOR_ACTIVE_STATE, MAX_CHARGE_CURRENT_PWM, LED_PIN, LED_ACTIVE_LOW);
EngineDetector engineDetector(ENGINE_PROBE_PULSE_MS, ENGINE_PROBE_COOLDOWN_MS, ENGINE_PROBE_RISE_VOLTS, ENGINE_RUNNING_GRACE_MS);
DataPoint history[HISTORY_SIZE];
byte historyIndex = 0;
bool connected = false;
static bool last_state = false;
unsigned long next_relay_check = 0;
static unsigned long lastPidUpdate = 0;
static unsigned long lastVoltageRead = 0;
float current_voltage = 0.0;
bool engine_running = false; // updated each manage_alternator() cycle
bool engine_probing = false; // true while a MAX_CHARGE probe pulse has no verdict yet
// Latched alert: set immediately on overvoltage, stays true for OVERVOLTAGE_ALERT_HOLD_MS
// after the LAST time it triggered (refreshed every cycle while still active) so a brief
// spike survives long enough to actually get picked up by the next MQTT publish, instead
// of clearing before anyone ever sees it.
#define OVERVOLTAGE_ALERT_HOLD_MS 60000
bool overvoltage_alert = false;
unsigned long overvoltageAlertMillis = 0;
unsigned long maxLoopDurationUs = 0; // high-water mark, reset every store_data() cycle
// Per-section high-water marks (same reset), so the dashboard can say WHO made the loop slow.
unsigned long maxWifiUs = 0, maxWebUs = 0, maxMqttUs = 0, maxFlashUs = 0;
// 60s peak-hold copy of the above (what the dashboard shows, so a 5s poll can't miss a spike).
unsigned long peakLoopUs = 0, peakWifiUs = 0, peakWebUs = 0, peakMqttUs = 0, peakFlashUs = 0;
unsigned long peakWindowStart = 0;
static inline void trackMax(unsigned long& maxUs, unsigned long startUs) {
  unsigned long d = micros() - startUs;
  if(d > maxUs) maxUs = d;
}
// Fault tracking for the LED codes (StatusLed.h) and the dashboard: what is wrong now, and
// the last fault seen since boot (stays after it clears, so a short blip can still be read).
LedFault activeFault = LedFault::NONE;
LedFault lastFault = LedFault::NONE;
unsigned long lastFaultMillis = 0;
bool lastEngineRunning = false; // edge-detects engine shutdown to force an immediate save

// NTP wall-clock seconds, 0 until synced (before ~2023 time() is near zero).
uint32_t epochOrZero() {
  time_t now = time(nullptr);
  return now >= 1700000000 ? static_cast<uint32_t>(now) : 0;
}

void setup() {
  Serial.begin(115200);

  // Hardware setup: the alternator output (and the LED mirroring it) are already configured
  // and driven off by OutputComponent's constructor - no blocking delays here, so the AP,
  // web server and the PID loop are up within a second of power-on.

  // WiFi: AP up first (always reachable), STA attempted alongside it. Web server/OTA
  // started here since they just work over whichever WiFi interface is active.
  startAP();
  nextStaAttempt = millis();
  ArduinoOTA.setHostname("smarttractor");
  ArduinoOTA.begin();
  setupWebServer();
  usageCounters.begin(millis());
  engineModeSettings.begin();
  voltageCalibration.begin();
  pidSettings.begin();
  PidGains pidStart = pidSettings.gains();
  chargePID.setTunings(pidStart.kp, pidStart.ki, pidStart.kd);
  gpsReader.begin();

  // Best-effort NTP sync (non-blocking): only resolves once/if STA has internet.
  // Maintenance log entries use epochSeconds=0 (shown as "unknown date") until this lands.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

float read_voltage() {
  static int samples[SAMPLES];

  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = analogRead(INPUT_VOLTAGE);
  }

  // Median, not mean: alternator ripple/spikes at high RPM must not move the reading.
  unscaledVoltage = calibrate_voltage(median_reading(samples, SAMPLES), 1024.0, 3.3, CALIBRATION_IN_VOLTAGE, CALIBRATION_A0_VOLTAGE);
  return unscaledVoltage * voltageCalibration.scale();
}

int lastTargetPWM = 0;

void manage_alternator() {
  if(overvoltage_alert && millis() - overvoltageAlertMillis >= OVERVOLTAGE_ALERT_HOLD_MS) {
    overvoltage_alert = false; // condition cleared and MQTT has had time to pick it up
  }

  if(USE_PWM && is_before_deadline(millis(), alternatorPauseUntil)) {
    alternator.off(); // paused for a calibration measurement: field off, nothing else decides
    lastTargetPWM = 0;
    chargePID.hold(0, current_voltage * 10, millis());
    return;
  }

  if(USE_PWM) {
    // PID-based control, on a short/stable cadence regardless of what else runs in loop()
    if(should_run_cycle(millis(), lastPidUpdate, PID_SAMPLE_TIME)) {
      pidInput = current_voltage * 10;
      lastPidUpdate = millis();
      EngineSourceDecision src = decide_engine_sources(
          engineModeSettings.sources(),
          gps_indicates_engine_running(gpsReader.hasFix(), gpsReader.fixAgeMs(),
                                       gpsSpeedAvg.average(millis()), ENGINE_GPS_MIN_SPEED_KMH));
      bool engineEvidence = src.forceRunning || src.useAlternator; // may charging voltage prove the engine?

      ChargeAction action = decide_pwm_safety_action(current_voltage, VOLTAGE_THRESHOLD_HIGH, VOLTAGE_THRESHOLD_LOW);

      // The PID only computes inside the regulation band. While the safety envelope drives the
      // output (full charge / cutoff) it is held aligned with that output, so entering the band
      // continues from the same output: 100% after full charge (no drop to 0), 0% after a cutoff
      // (no stale integral to overshoot with) - and no integrating of the time spent outside.
      if(action == ChargeAction::MAX_CHARGE) chargePID.hold(MAX_CHARGE_CURRENT_PWM, pidInput, millis());
      else if(action == ChargeAction::OFF) chargePID.hold(0, pidInput, millis());

      switch(action) {
        case ChargeAction::OFF:
          alternator.off();
          lastTargetPWM = 0;
          if(engineEvidence) engineDetector.noteRunning(millis()); else engineDetector.reset();
          engine_running = engineEvidence; // voltage this high means something is charging it
          engine_probing = false;
          overvoltage_alert = true;
          overvoltageAlertMillis = millis(); // refreshed every cycle while still active
          break;
        case ChargeAction::MAX_CHARGE:
          // Voltage this low is also where a resting, engine-off battery sits, so don't just
          // hold the field coil at 100% and drain it further — probe with pulses instead.
          if(src.forceRunning) {
            engineDetector.noteRunning(millis()); // engine known on: charge now, no probing
            alternator.pwm(MAX_CHARGE_CURRENT_PWM);
            lastTargetPWM = MAX_CHARGE_CURRENT_PWM;
            engine_running = true;
            engine_probing = false;
          } else if(src.useAlternator) {
            if(engineDetector.update(current_voltage, millis())) {
              alternator.pwm(MAX_CHARGE_CURRENT_PWM);
              lastTargetPWM = MAX_CHARGE_CURRENT_PWM;
            } else {
              alternator.off();
              lastTargetPWM = 0;
            }
            engine_running = engineDetector.engineRunning();
            engine_probing = engineDetector.isProbing();
          } else {
            engineDetector.reset(); // only GPS selected and standing still: engine considered off
            alternator.off();
            lastTargetPWM = 0;
            engine_running = false;
            engine_probing = false;
          }
          break;
        case ChargeAction::RUN_PID: {
          if(engineEvidence) engineDetector.noteRunning(millis()); else engineDetector.reset();
          engine_running = engineEvidence; // alternator has already raised voltage out of the probe zone
          engine_probing = false;
          if(autotuneActive) {
            double out = pidAutotuner.update(pidInput, millis());
            int targetPWM = constrain(static_cast<int>(out), 0, MAX_CHARGE_CURRENT_PWM);
            alternator.pwm(targetPWM);
            lastTargetPWM = targetPWM;

            if(pidAutotuner.state() == AutotuneState::SUCCEEDED) {
              AutotuneGains g = pidAutotuner.gains();
              if(pidSettings.save(PidGains{g.kp, g.ki, g.kd})) chargePID.setTunings(g.kp, g.ki, g.kd);
              else Serial.println("Autotune gains rejected as implausible, keeping the previous ones");
              Serial.printf("Autotune done: Kp=%.2f Ki=%.2f Kd=%.2f (Ku=%.2f Pu=%.0fms)\n",
                            g.kp, g.ki, g.kd, g.ku, g.pu);
              autotuneActive = false;
            } else if(pidAutotuner.state() == AutotuneState::FAILED) {
              Serial.println("Autotune failed, keeping previous tunings");
              autotuneActive = false;
            }
          } else {
            pidOutput = chargePID.compute(Setpoint, pidInput, millis());
            int targetPWM = static_cast<int>(pidOutput);
            alternator.pwm(targetPWM);
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
    engine_running = alternator.isActive();
    engine_probing = false;
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

  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  Serial.printf("Current Voltage: %.2fV\n", current_voltage);
  Serial.printf("Alternator State: %s\n", alternator.isActive() ? "ON" : "OFF");
  Serial.printf("PWM Value: %d\n", alternator.getPWM());
  Serial.printf("Relay State: %s\n", last_state ? "ON" : "OFF");
  Serial.printf("Next Relay Check: %lu\n", next_relay_check);
  Serial.printf("History Index: %d\n", historyIndex);
  if (connected) {
    Serial.printf("Connected: Yes (%s)\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("Connected: No");
  }
  Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
  Serial.printf("Max loop time: %lu us (wifi %lu, web %lu, mqtt %lu, flash %lu)\n",
                maxLoopDurationUs, maxWifiUs, maxWebUs, maxMqttUs, maxFlashUs);
  if(millis() - peakWindowStart >= 60000) {
    peakLoopUs = peakWifiUs = peakWebUs = peakMqttUs = peakFlashUs = 0;
    peakWindowStart = millis();
  }
  if(maxLoopDurationUs > peakLoopUs) peakLoopUs = maxLoopDurationUs;
  if(maxWifiUs > peakWifiUs) peakWifiUs = maxWifiUs;
  if(maxWebUs > peakWebUs) peakWebUs = maxWebUs;
  if(maxMqttUs > peakMqttUs) peakMqttUs = maxMqttUs;
  if(maxFlashUs > peakFlashUs) peakFlashUs = maxFlashUs;
  maxLoopDurationUs = 0;
  maxWifiUs = maxWebUs = maxMqttUs = maxFlashUs = 0;
  Serial.println();
}

unsigned long lastDataStore = 0;

void loop() {
  unsigned long loopStartUs = micros();
  unsigned long currentMillis = millis();
  if(should_run_cycle(currentMillis, lastVoltageRead, PID_SAMPLE_TIME)) {
    current_voltage = read_voltage();
    lastVoltageRead = currentMillis;
  }

  // "I'm alive" blink on power-on; owns the LED (mirror paused) until it finishes.
  static unsigned long bootBlinkStart = currentMillis;
  static bool bootBlinkDone = false;
  if(!bootBlinkDone) {
    unsigned long elapsed = currentMillis - bootBlinkStart;
    if(boot_blink_active(elapsed, BOOT_BLINK_HALF_PERIOD_MS, BOOT_BLINK_COUNT)) {
      alternator.setMirrorEnabled(false);
      alternator.writeMirrorRaw(boot_blink_on(elapsed, BOOT_BLINK_HALF_PERIOD_MS, BOOT_BLINK_COUNT));
    } else {
      alternator.setMirrorEnabled(true);
      bootBlinkDone = true;
    }
  }

  // Fault code on the LED (after the boot blink): takes the LED over from the PWM mirror
  // while a fault is active, hands it back as soon as it clears.
  activeFault = decide_led_fault(current_voltage, overvoltage_alert, ESP.getFreeHeap());
  if(activeFault != LedFault::NONE) { lastFault = activeFault; lastFaultMillis = currentMillis; }

  if(bootBlinkDone) {
    static LedFault shownFault = LedFault::NONE;
    static unsigned long faultStart = 0;
    static bool faultLedOwned = false;
    LedFault fault = activeFault;
    if(fault != shownFault) faultStart = currentMillis; // new code (or cleared): restart the pattern
    unsigned long faultElapsed = currentMillis - faultStart;
    int pulses = static_cast<int>(fault);
    if(fault != LedFault::NONE &&
       fault_blink_active(faultElapsed, pulses, FAULT_BLINK_HALF_PERIOD_MS, FAULT_BLINK_PAUSE_MS, FAULT_BLINK_REPEATS)) {
      alternator.setMirrorEnabled(false);
      alternator.writeMirrorRaw(fault_blink_on(faultElapsed, pulses, FAULT_BLINK_HALF_PERIOD_MS, FAULT_BLINK_PAUSE_MS));
      faultLedOwned = true;
    } else if(faultLedOwned) {
      alternator.setMirrorEnabled(true); // shown enough (or cleared): back to mirroring the output
      faultLedOwned = false;
    }
    shownFault = fault;
  }

  unsigned long sectionUs = micros();
  updateWifi(currentMillis);
  trackMax(maxWifiUs, sectionUs);
  connected = wifiConnectedSta;
  dnsServer.processNextRequest();
  gpsReader.update();
  if(gpsReader.hasFix() && gpsJumpFilter.accept(gpsReader.latitude(), gpsReader.longitude(), currentMillis)) {
    gpsHasAcceptedFix = true;
  }

  if(gpsReader.hasFix()) gpsSpeedAvg.add(gpsReader.speedKmh(), currentMillis);
  else gpsHasAcceptedFix = false; // fix lost: stop showing a stale position/speed

  ArduinoOTA.handle();
  sectionUs = micros();
  server.handleClient();
  trackMax(maxWebUs, sectionUs);

  manage_alternator();
  sectionUs = micros();
  usageCounters.tick(engine_running, currentMillis);

  // This board only keeps running ~3-4s after the tractor's engine (and its own
  // power feed) is switched off, so the engine-off edge is the last reliable chance
  // to persist anything - don't wait for the normal throttled save.
  if(lastEngineRunning && !engine_running) {
    usageCounters.forceSave();
  }
  lastEngineRunning = engine_running;
  trackMax(maxFlashUs, sectionUs);

  // Data storage (5000ms interval, not every fast PID cycle)
  if(currentMillis - lastDataStore >= 5000) {
    store_data();
    lastDataStore = currentMillis;
  }

  sectionUs = micros();
  if(mqtt_host[0] != '\0') {
    mqttTransport.loop(); // keepalive/ping - needs to run well under the 15s broker keepalive
                           // regardless of the slower sample/publish cadences below

    if(currentMillis - lastMqttSample >= MQTT_SAMPLE_INTERVAL_MS) {
      lastMqttSample = currentMillis;
      MqttReading reading{currentMillis, current_voltage, alternator.getPWM(), alternator.getPWMPercent(),
                          alternator.isActive(), engine_running, usageCounters.totalRunSeconds(),
                          usageCounters.isMaintenanceDue(), ESP.getFreeHeap(), overvoltage_alert, epochOrZero()};
      mqttPublisher.recordSample(reading);
      mqttPublisher.setLiveState(MqttLiveState{gpsHasAcceptedFix, gpsJumpFilter.lastLat(), gpsJumpFilter.lastLon(),
                                               static_cast<float>(gpsReader.speedKmh()),
                                               static_cast<float>(gpsSpeedAvg.average(currentMillis)),
                                               engine_probing, engineModeSettings.sources(),
                                               static_cast<uint8_t>(activeFault)});
    }

    if(currentMillis - lastMqttPublish >= MQTT_PUBLISH_INTERVAL_MS) {
      lastMqttPublish = currentMillis;
      mqttPublisher.update(wifiConnectedSta, currentMillis);
    }
  }

  trackMax(maxMqttUs, sectionUs);

  unsigned long loopDurationUs = micros() - loopStartUs;
  if (loopDurationUs > maxLoopDurationUs) maxLoopDurationUs = loopDurationUs;
}
