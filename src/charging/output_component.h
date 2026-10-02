#ifndef OUTPUT_COMPONENT_H
#define OUTPUT_COMPONENT_H

#include <Arduino.h>
#include "PwmMirror.h"

class OutputComponent {
  private:
    uint8_t pin;
    bool isPWM;
    bool activeState;
    uint16_t currentPWM;
    uint16_t maxPWM;
    int8_t mirrorPin;
    bool mirrorActiveLow;
    bool mirrorEnabled;
    DutyLatch outLatch;
    DutyLatch mirrorLatch;
    void writeMirror(uint16_t duty);

    void write(uint16_t duty); // single choke point: output pin + mirror pin get the same duty

  public:
    OutputComponent(uint8_t outputPin, bool pwmEnabled, bool activeState, uint16_t maxPWMValue = 1023,
                    int8_t mirrorPin = -1, bool mirrorActiveLow = false);

    void set(bool state);
    void pwm(uint16_t value);
    void off();

    // While disabled the mirror pin is left alone (e.g. boot blink owns the LED); enabling
    // it again immediately re-syncs the mirror pin to the current output duty.
    void setMirrorEnabled(bool enabled);
    void writeMirrorRaw(bool on); // drive the mirror pin fully on/off (active-low aware), independent of the output

    uint16_t getPWM() const;
    uint8_t getPWMPercent() const;
    bool isActive() const;
    bool isPWMEnabled() const;
    uint16_t getMaxPWM() const;
};

#endif