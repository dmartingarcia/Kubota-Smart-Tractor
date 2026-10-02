#include "RealTimePid.h"

namespace {
double clamp(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
}

RealTimePid::RealTimePid(double kp, double ki, double kd, double outputMin, double outputMax)
  : kp_(kp), ki_(ki), kd_(kd), outputMin_(outputMin), outputMax_(outputMax),
    integral_(0), lastInput_(0), lastMillis_(0), hasLast_(false) {}

void RealTimePid::setTunings(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

double RealTimePid::compute(double setpoint, double input, unsigned long currentMillis) {
  if (!hasLast_) {
    hasLast_ = true;
    lastInput_ = input;
    lastMillis_ = currentMillis;
    return 0;
  }

  double dtSeconds = (currentMillis - lastMillis_) / 1000.0;
  lastMillis_ = currentMillis;
  if (dtSeconds <= 0) return clamp(kp_ * (setpoint - input) + integral_, outputMin_, outputMax_);

  double error = setpoint - input;
  integral_ = clamp(integral_ + ki_ * error * dtSeconds, outputMin_, outputMax_); // anti-windup
  double derivativeOnMeasurement = (input - lastInput_) / dtSeconds; // avoids derivative kick on setpoint changes
  lastInput_ = input;

  return clamp(kp_ * error + integral_ - kd_ * derivativeOnMeasurement, outputMin_, outputMax_);
}

void RealTimePid::reset() {
  integral_ = 0;
  hasLast_ = false;
}

void RealTimePid::hold(double output, double input, unsigned long currentMillis) {
  integral_ = clamp(output, outputMin_, outputMax_);
  lastInput_ = input;
  lastMillis_ = currentMillis;
  hasLast_ = true;
}
