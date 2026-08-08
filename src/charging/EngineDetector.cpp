#include "EngineDetector.h"

EngineDetector::EngineDetector(unsigned long pulseDurationMs, unsigned long cooldownMs, float voltageRiseVolts)
  : pulseDurationMs_(pulseDurationMs), cooldownMs_(cooldownMs), voltageRiseVolts_(voltageRiseVolts),
    started_(false), phase_(Phase::PULSING), phaseStartMillis_(0), baselineVoltage_(0), engineRunning_(false),
    probing_(false) {}

bool EngineDetector::update(float voltage, unsigned long currentMillis) {
  if (!started_) {
    started_ = true;
    phase_ = Phase::PULSING;
    phaseStartMillis_ = currentMillis;
    baselineVoltage_ = voltage;
    probing_ = true;
    return true;
  }

  if (phase_ == Phase::COOLDOWN) {
    if (currentMillis - phaseStartMillis_ < cooldownMs_) { probing_ = false; return false; }
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
  return false;
}

bool EngineDetector::engineRunning() const {
  return engineRunning_;
}

bool EngineDetector::isProbing() const {
  return probing_;
}

void EngineDetector::reset() {
  started_ = false;
  engineRunning_ = false;
  probing_ = false;
}
