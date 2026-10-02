#include "StatusLed.h"

LedFault decide_led_fault(float voltage, bool overvoltageAlert, uint32_t freeHeapBytes) {
  if (voltage < kLedSensorMinVolts || voltage > kLedSensorMaxVolts) return LedFault::SENSOR_RANGE;
  if (overvoltageAlert) return LedFault::OVERVOLTAGE;
  if (freeHeapBytes < kLedLowHeapBytes) return LedFault::LOW_HEAP;
  return LedFault::NONE;
}

bool fault_blink_on(unsigned long elapsedMs, int pulses, unsigned long halfPeriodMs, unsigned long pauseMs) {
  if (pulses <= 0) return false;
  unsigned long pulsesSpan = 2UL * halfPeriodMs * static_cast<unsigned long>(pulses);
  unsigned long t = elapsedMs % (pulsesSpan + pauseMs);
  if (t >= pulsesSpan) return false; // pause between repeats
  return (t % (2UL * halfPeriodMs)) < halfPeriodMs;
}

bool fault_blink_active(unsigned long elapsedMs, int pulses, unsigned long halfPeriodMs, unsigned long pauseMs, int repeats) {
  if (pulses <= 0 || repeats <= 0) return false;
  unsigned long pulsesSpan = 2UL * halfPeriodMs * static_cast<unsigned long>(pulses);
  unsigned long total = static_cast<unsigned long>(repeats) * (pulsesSpan + pauseMs) - pauseMs;
  return elapsedMs < total;
}
