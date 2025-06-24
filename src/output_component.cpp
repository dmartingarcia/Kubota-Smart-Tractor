#include <Arduino.h>

#define OUTPUT_PIN              D1
#define PWM_FREQUENCY           5000    // 5kHz for MOSFET
#define PWM_RESOLUTION          1023      // 10-bit resolution (0-1023)
#define USE_PWM                 true    // Set to false for relay control
#include "output_component.h"
#include <Arduino.h>

OutputComponent::OutputComponent(uint8_t outputPin, bool pwmEnabled, bool activeLow, uint16_t maxPWMValue)
  : pin(outputPin),
    isPWM(pwmEnabled),
    activeState(!activeLow),
    currentPWM(0),
    maxPWM(maxPWMValue)
{
  pinMode(pin, OUTPUT);
  if(isPWM) {
    analogWriteFreq(PWM_FREQUENCY);  // 5kHz frequency
    analogWriteRange(PWM_RESOLUTION);  // 10-bit resolution
  }
  off();
}

void OutputComponent::set(bool state) {
  if(isPWM) {
    analogWrite(pin, state ? maxPWM : 0);
    currentPWM = state ? maxPWM : 0;
  } else {
    digitalWrite(pin, state ? activeState : !activeState);
  }
}

void OutputComponent::pwm(uint16_t value) {
  if(isPWM) {
    currentPWM = constrain(value, 0, maxPWM);
    analogWrite(pin, currentPWM);
  }
}

void OutputComponent::off() {
  if(isPWM) {
    analogWrite(pin, 0);
  } else {
    digitalWrite(pin, !activeState);
  }
  currentPWM = 0;
}

uint16_t OutputComponent::getPWM() const {
  return currentPWM;
}

uint8_t OutputComponent::getPWMPercent() const {
  return map(currentPWM, 0, maxPWM, 0, 100);
}

bool OutputComponent::isActive() const {
  return isPWM ? (currentPWM > 0) : (digitalRead(pin) == activeState);
}

bool OutputComponent::isPWMEnabled() const {
  return isPWM;
}

uint16_t OutputComponent::getMaxPWM() const {
  return maxPWM;
}