#include "PidAutotuner.h"

namespace {
constexpr double kPi = 3.14159265358979323846;
}

PidAutotuner::PidAutotuner(double setpoint, double outputBase, double outputStep, double noiseBand,
                           int cyclesToRecord, unsigned long maxRuntimeMs)
  : setpoint_(setpoint), outputBase_(outputBase), outputStep_(outputStep), noiseBand_(noiseBand),
    cyclesToRecord_(cyclesToRecord), maxRuntimeMs_(maxRuntimeMs),
    relayHigh_(true), peakMin_(0), peakMax_(0),
    lastSwitchMillis_(0), startMillis_(0), halfCyclesSeen_(0),
    periodSumMs_(0), fullPeriodsRecorded_(0),
    state_(AutotuneState::RUNNING), gains_{0, 0, 0, 0, 0} {}

void PidAutotuner::begin(unsigned long currentMillis) {
  relayHigh_ = true;
  halfCyclesSeen_ = 0;
  periodSumMs_ = 0;
  fullPeriodsRecorded_ = 0;
  startMillis_ = currentMillis;
  lastSwitchMillis_ = currentMillis;
  state_ = AutotuneState::RUNNING;
}

double PidAutotuner::update(double input, unsigned long currentMillis) {
  if (state_ != AutotuneState::RUNNING) {
    return relayHigh_ ? (outputBase_ + outputStep_) : (outputBase_ - outputStep_);
  }

  if (halfCyclesSeen_ >= 1) {
    if (input > peakMax_) peakMax_ = input;
    if (input < peakMin_) peakMin_ = input;
  }

  bool shouldSwitch = relayHigh_ ? (input >= setpoint_ + noiseBand_) : (input <= setpoint_ - noiseBand_);

  if (shouldSwitch) {
    halfCyclesSeen_++;
    if (halfCyclesSeen_ == 1) {
      peakMin_ = peakMax_ = input; // start of recording window; discard warm-up half-cycle
    } else {
      periodSumMs_ += (currentMillis - lastSwitchMillis_);
      fullPeriodsRecorded_++;
    }
    relayHigh_ = !relayHigh_;
    lastSwitchMillis_ = currentMillis;

    if (fullPeriodsRecorded_ >= cyclesToRecord_) {
      finish(currentMillis);
    }
  }

  if (state_ == AutotuneState::RUNNING && (currentMillis - startMillis_) >= maxRuntimeMs_) {
    state_ = AutotuneState::FAILED;
  }

  return relayHigh_ ? (outputBase_ + outputStep_) : (outputBase_ - outputStep_);
}

void PidAutotuner::finish(unsigned long currentMillis) {
  (void)currentMillis;
  double amplitude = (peakMax_ - peakMin_) / 2.0;
  if (amplitude <= 1e-6 || fullPeriodsRecorded_ == 0) {
    state_ = AutotuneState::FAILED;
    return;
  }

  double avgHalfPeriodMs = static_cast<double>(periodSumMs_) / fullPeriodsRecorded_;
  double puMs = avgHalfPeriodMs * 2.0;
  double puSec = puMs / 1000.0;
  double ku = (4.0 * outputStep_) / (kPi * amplitude);
  double kp = 0.6 * ku;
  double ki = 2.0 * kp / puSec;
  double kd = kp * puSec / 8.0;

  gains_ = AutotuneGains{kp, ki, kd, ku, puMs};
  state_ = AutotuneState::SUCCEEDED;
}

AutotuneState PidAutotuner::state() const {
  return state_;
}

AutotuneGains PidAutotuner::gains() const {
  return gains_;
}
