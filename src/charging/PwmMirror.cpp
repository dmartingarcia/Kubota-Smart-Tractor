#include "PwmMirror.h"

uint16_t mirror_duty(uint16_t outputDuty, uint16_t maxDuty, bool activeLow) {
  uint16_t duty = outputDuty > maxDuty ? maxDuty : outputDuty;
  return activeLow ? maxDuty - duty : duty;
}
