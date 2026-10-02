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

// Remembers the last duty written to a pin so unchanged values are not rewritten: the PID asks
// for a duty every 20 ms, but analogWrite() on the ESP8266 reprograms the software-PWM waveform
// each time it is called, which is wasted CPU (and can disturb the running waveform).
class DutyLatch {
  public:
    bool changed(uint16_t duty) { // true when the pin needs writing (first write always counts)
      if (valid_ && duty == last_) return false;
      last_ = duty;
      valid_ = true;
      return true;
    }
    void invalidate() { valid_ = false; }

  private:
    uint16_t last_ = 0;
    bool valid_ = false;
};

#endif
