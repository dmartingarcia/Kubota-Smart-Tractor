#include <Arduino.h>
#include "PwmMirror.h"

#define OUTPUT_PIN              D1
#define PWM_FREQUENCY           5000    // 5kHz for MOSFET
#define PWM_RESOLUTION          1023      // 10-bit resolution (0-1023)
#define USE_PWM                 true    // Set to false for relay control
#include "output_component.h"
#include <Arduino.h>

OutputComponent::OutputComponent(uint8_t outputPin, bool pwmEnabled, bool activeState, uint16_t maxPWMValue, int8_t mirrorPin, bool mirrorActiveLow)
  : pin(outputPin),
    isPWM(pwmEnabled),
    activeState(activeState),
    currentPWM(0),
    maxPWM(maxPWMValue),
    mirrorPin(mirrorPin),
    mirrorActiveLow(mirrorActiveLow),
    mirrorEnabled(true)
{
  pinMode(pin, OUTPUT);
  if(mirrorPin >= 0) pinMode(mirrorPin, OUTPUT);
  if(isPWM) {
    analogWriteFreq(PWM_FREQUENCY);  // 5kHz frequency
    analogWriteRange(PWM_RESOLUTION);  // 10-bit resolution
  }
  off();
}

void OutputComponent::write(uint16_t duty) {
  analogWrite(pin, duty);
  if(mirrorPin >= 0 && mirrorEnabled) analogWrite(mirrorPin, mirror_duty(duty, maxPWM, mirrorActiveLow));
}

void OutputComponent::set(bool state) {
  if(isPWM) {
    write(state ? maxPWM : 0);
    currentPWM = state ? maxPWM : 0;
  } else {
    digitalWrite(pin, state ? activeState : !activeState);
  }
}

void OutputComponent::pwm(uint16_t value) {
  if(isPWM) {
    currentPWM = constrain(value, 0, maxPWM);
    write(currentPWM);
  }
}

void OutputComponent::off() {
  if(isPWM) {
    write(0);
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

void OutputComponent::setMirrorEnabled(bool enabled) {
  mirrorEnabled = enabled;
  if(enabled && mirrorPin >= 0 && isPWM) analogWrite(mirrorPin, mirror_duty(currentPWM, maxPWM, mirrorActiveLow));
}

void OutputComponent::writeMirrorRaw(bool on) {
  if(mirrorPin >= 0) analogWrite(mirrorPin, mirror_duty(on ? maxPWM : 0, maxPWM, mirrorActiveLow));
}
