#ifndef VOLTAGE_CALIBRATION_H
#define VOLTAGE_CALIBRATION_H

#include "../storage/UsageCounters.h" // IFlashStore

// One-point user calibration on top of the fixed divider calibration (calibrate_voltage):
// the user types what a reference multimeter reads and the device derives a persistent
// scale factor = measured / unscaled reading. Pure logic + HAL store, natively testable.

constexpr float kVoltageScaleMin = 0.8f;
constexpr float kVoltageScaleMax = 1.25f;

// False (and *scaleOut untouched) if the inputs are implausible: reference outside 6-20V,
// unscaled reading below 1V, or a resulting scale outside [kVoltageScaleMin, kVoltageScaleMax]
// (that would mean wiring/divider trouble, not drift).
bool compute_voltage_scale(float measuredVolts, float unscaledVolts, float* scaleOut);

class VoltageCalibration {
  public:
    explicit VoltageCalibration(IFlashStore& store);
    void begin();
    float scale() const { return scale_; }
    bool calibrate(float measuredVolts, float unscaledVolts); // false = rejected, nothing changed
    // Same, but from the value the dashboard SHOWED (already scaled): the voltage moves, so the
    // user reports what the UI read at the moment they read the multimeter.
    bool calibrateFromShown(float measuredVolts, float shownVolts);
    void reset();                                              // back to 1.0

  private:
    struct Blob { uint32_t magic; float scale; };
    IFlashStore& store_;
    float scale_;
    void save();
};

#endif
