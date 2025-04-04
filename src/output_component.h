#ifndef OUTPUT_COMPONENT_H
#define OUTPUT_COMPONENT_H

#include <Arduino.h>

class OutputComponent {
  private:
    uint8_t pin;
    bool isPWM;
    bool activeState;
    uint16_t currentPWM;
    uint16_t maxPWM;

  public:
    OutputComponent(uint8_t outputPin, bool pwmEnabled, bool activeLow, uint16_t maxPWMValue = 800);

    void set(bool state);
    void pwm(uint16_t value);
    void off();

    uint16_t getPWM() const;
    uint8_t getPWMPercent() const;
    bool isActive() const;
    bool isPWMEnabled() const;
    uint16_t getMaxPWM() const;
};

#endif