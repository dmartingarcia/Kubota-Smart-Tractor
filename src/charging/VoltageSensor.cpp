#include "VoltageSensor.h"

float calibrate_voltage(float rawAdcAverage, float adcMax, float adcReferenceVolts,
                         float calInVoltage, float calA0Voltage) {
  float measuredVolts = rawAdcAverage * (adcReferenceVolts / adcMax);
  return (calInVoltage / calA0Voltage) * measuredVolts;
}
