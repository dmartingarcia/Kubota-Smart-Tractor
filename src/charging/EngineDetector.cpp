#include "EngineDetector.h"

namespace {
// Rise above resting voltage, with the alternator NOT driven, that ends cooldown early.
// Deliberately larger than the in-pulse threshold, which can be tiny at idle RPM.
constexpr float kCooldownExitRiseVolts = 0.3f;
}

EngineDetector::EngineDetector(unsigned long pulseDurationMs, unsigned long cooldownMs, float voltageRiseVolts, unsigned long graceMs)
  : pulseDurationMs_(pulseDurationMs), cooldownMs_(cooldownMs), voltageRiseVolts_(voltageRiseVolts),
    started_(false), phase_(Phase::PULSING), phaseStartMillis_(0), baselineVoltage_(0), engineRunning_(false),
    probing_(false), graceMs_(graceMs), lastRunningMillis_(0), hasRunning_(false) {}

void EngineDetector::noteRunning(unsigned long currentMillis) {
  hasRunning_ = true;
  lastRunningMillis_ = currentMillis;
  started_ = false; // next probe (after grace) starts fresh
  engineRunning_ = true;
  probing_ = false;
}

bool EngineDetector::update(float voltage, unsigned long currentMillis) {
  if (hasRunning_) {
    if (currentMillis - lastRunningMillis_ < graceMs_) {
      engineRunning_ = true;
      probing_ = false;
      return true;
    }
    hasRunning_ = false;
    engineRunning_ = false;
  }

  if (!started_) {
    started_ = true;
    phase_ = Phase::PULSING;
    phaseStartMillis_ = currentMillis;
    baselineVoltage_ = voltage;
    probing_ = true;
    return true;
  }

  if (phase_ == Phase::COOLDOWN) {
    bool voltageRoseOnItsOwn = voltage - baselineVoltage_ >= kCooldownExitRiseVolts;
    if (!voltageRoseOnItsOwn && currentMillis - phaseStartMillis_ < cooldownMs_) { probing_ = false; return false; }
    phase_ = Phase::PULSING;
    phaseStartMillis_ = currentMillis;
    baselineVoltage_ = voltage;
    probing_ = true;
    return true;
  }

  // Phase::PULSING
  if (voltage - baselineVoltage_ >= voltageRiseVolts_) {
    engineRunning_ = true;
    probing_ = false;
    return true;
  }

  if (currentMillis - phaseStartMillis_ < pulseDurationMs_) { probing_ = true; return true; } // still watching this pulse

  // Pulse window elapsed with no voltage rise: engine looks off, back off and cool down.
  engineRunning_ = false;
  probing_ = false;
  phase_ = Phase::COOLDOWN;
  phaseStartMillis_ = currentMillis;
  // baselineVoltage_ stays the resting level from pulse start: the battery recovering
  // from the pulse's sag returns to it, so only a real rise above it ends the cooldown.
  return false;
}

bool EngineDetector::engineRunning() const {
  return engineRunning_;
}

bool EngineDetector::isProbing() const {
  return probing_;
}

void EngineDetector::reset() {
  hasRunning_ = false;
  started_ = false;
  engineRunning_ = false;
  probing_ = false;
}
