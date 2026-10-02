#include "AlternatorLogic.h"

bool should_run_cycle(unsigned long currentMillis, unsigned long lastRunMillis, unsigned long intervalMillis) {
  return (currentMillis - lastRunMillis) >= intervalMillis;
}

ChargeAction decide_pwm_safety_action(float voltage, float thresholdHigh, float thresholdLow) {
  if (voltage >= thresholdHigh) return ChargeAction::OFF;
  if (voltage <= thresholdLow) return ChargeAction::MAX_CHARGE;
  return ChargeAction::RUN_PID;
}

RelayDecision decide_relay_state(bool currentState, float voltage, unsigned long currentMillis,
                                  unsigned long nextRelayCheck, float thresholdHigh,
                                  float thresholdLow, unsigned long activationDelay) {
  if (currentState && voltage > thresholdHigh) {
    return RelayDecision{false, currentMillis + activationDelay, true};
  }
  if (!currentState && nextRelayCheck < currentMillis && voltage < thresholdLow) {
    return RelayDecision{true, currentMillis + activationDelay, true};
  }
  return RelayDecision{currentState, nextRelayCheck, false};
}

bool gps_indicates_engine_running(bool hasFix, unsigned long fixAgeMs, double speedKmh, double minSpeedKmh) {
  const unsigned long kMaxFixAgeMs = 3000;
  return hasFix && fixAgeMs <= kMaxFixAgeMs && speedKmh >= minSpeedKmh;
}
