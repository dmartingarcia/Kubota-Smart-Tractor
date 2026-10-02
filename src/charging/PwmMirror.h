#ifndef PWM_MIRROR_H
#define PWM_MIRROR_H

#include <stdint.h>

// Duty to write on a mirror pin (e.g. status LED) so it replicates the duty on the
// alternator output pin one-to-one. activeLow inverts it for LEDs wired to sink current
// (the Wemos D1 Mini on-board LED lights when its pin is LOW). Pure, no hardware access.
uint16_t mirror_duty(uint16_t outputDuty, uint16_t maxDuty, bool activeLow);

// Power-on "I'm alive" blink on the status LED: `blinks` pulses of halfPeriodMs on /
// halfPeriodMs off, measured from the first loop() iteration. boot_blink_active() says
// whether the pattern is still running (the LED belongs to it, not to the PWM mirror).
bool boot_blink_on(unsigned long elapsedMs, unsigned long halfPeriodMs, int blinks);
bool boot_blink_active(unsigned long elapsedMs, unsigned long halfPeriodMs, int blinks);

#endif
