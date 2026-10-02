#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>

// Fault codes shown on the status LED as N pulses, a pause, repeat. Pure logic, no hardware.
// While a fault is active the LED shows the code instead of mirroring the output PWM.
enum class LedFault : uint8_t {
  NONE = 0,
  OVERVOLTAGE = 2,   // overvoltage cutoff active/latched
  SENSOR_RANGE = 3,  // battery voltage outside a plausible range: sense wiring / divider fault
  LOW_HEAP = 4,      // free RAM nearly exhausted: restart soon (heap leak/fragmentation)
};

constexpr float kLedSensorMinVolts = 8.0f;
constexpr float kLedSensorMaxVolts = 17.0f;
constexpr uint32_t kLedLowHeapBytes = 6000;

// Highest priority first: sensor fault (the other readings can't be trusted), overvoltage, heap.
LedFault decide_led_fault(float voltage, bool overvoltageAlert, uint32_t freeHeapBytes);

// `pulses` blinks of halfPeriodMs on / halfPeriodMs off, then pauseMs dark, repeating.
bool fault_blink_on(unsigned long elapsedMs, int pulses, unsigned long halfPeriodMs, unsigned long pauseMs);

// Whether the code is still being shown: `repeats` showings of the pattern, then the LED is
// handed back (the fault may persist - it was already shown, no need to keep flashing).
bool fault_blink_active(unsigned long elapsedMs, int pulses, unsigned long halfPeriodMs, unsigned long pauseMs, int repeats);

#endif
