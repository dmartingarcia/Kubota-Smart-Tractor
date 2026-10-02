#include "VoltageSensor.h"

float calibrate_voltage(float rawAdcAverage, float adcMax, float adcReferenceVolts,
                         float calInVoltage, float calA0Voltage) {
  float measuredVolts = rawAdcAverage * (adcReferenceVolts / adcMax);
  return (calInVoltage / calA0Voltage) * measuredVolts;
}

int median_reading(const int* samples, int n) {
  int sorted[15];
  if (n > 15) n = 15;
  for (int i = 0; i < n; i++) {
    int v = samples[i];
    int j = i;
    while (j > 0 && sorted[j - 1] > v) { sorted[j] = sorted[j - 1]; j--; }
    sorted[j] = v;
  }
  return sorted[n / 2];
}
