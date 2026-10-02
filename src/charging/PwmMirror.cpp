#include "PwmMirror.h"

uint16_t mirror_duty(uint16_t outputDuty, uint16_t maxDuty, bool activeLow) {
  uint16_t duty = outputDuty > maxDuty ? maxDuty : outputDuty;
  return activeLow ? maxDuty - duty : duty;
}

bool boot_blink_active(unsigned long elapsedMs, unsigned long halfPeriodMs, int blinks) {
  return elapsedMs < 2UL * halfPeriodMs * static_cast<unsigned long>(blinks);
}

bool boot_blink_on(unsigned long elapsedMs, unsigned long halfPeriodMs, int blinks) {
  return boot_blink_active(elapsedMs, halfPeriodMs, blinks) && (elapsedMs % (2UL * halfPeriodMs)) < halfPeriodMs;
}
