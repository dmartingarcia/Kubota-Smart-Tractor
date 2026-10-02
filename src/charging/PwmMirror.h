#ifndef PWM_MIRROR_H
#define PWM_MIRROR_H

#include <stdint.h>

// Duty to write on a mirror pin (e.g. status LED) so it replicates the duty on the
// alternator output pin one-to-one. activeLow inverts it for LEDs wired to sink current
// (the Wemos D1 Mini on-board LED lights when its pin is LOW). Pure, no hardware access.
uint16_t mirror_duty(uint16_t outputDuty, uint16_t maxDuty, bool activeLow);

#endif
