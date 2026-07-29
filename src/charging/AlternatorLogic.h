#ifndef ALTERNATOR_LOGIC_H
#define ALTERNATOR_LOGIC_H

// Pure decision logic for alternator control, no hardware access — testable on host.
// main.cpp applies the returned decisions to OutputComponent/PID.

enum class ChargeAction { OFF, MAX_CHARGE, RUN_PID };

// Fixed-cadence scheduler check, handles millis() rollover via unsigned wraparound.
// Used to keep the PID cycle on a short, stable interval regardless of what else
// runs in loop() (web server, WiFi, MQTT, ...).
bool should_run_cycle(unsigned long currentMillis, unsigned long lastRunMillis, unsigned long intervalMillis);

// Safety envelope around PID control (PWM mode): forces OFF above the high
// threshold and MAX below the low threshold, otherwise defers to PID.
ChargeAction decide_pwm_safety_action(float voltage, float thresholdHigh, float thresholdLow);

struct RelayDecision {
  bool state;
  unsigned long nextCheck;
  bool changed;
};

// Hysteresis control for relay mode: turns off above the high threshold (then
// waits activationDelay before allowing another switch), turns on below the
// low threshold once that delay has elapsed.
RelayDecision decide_relay_state(bool currentState, float voltage, unsigned long currentMillis,
                                  unsigned long nextRelayCheck, float thresholdHigh,
                                  float thresholdLow, unsigned long activationDelay);

#endif
