#ifndef VOLTAGE_SENSOR_H
#define VOLTAGE_SENSOR_H

// Pure calibration math, no hardware access — testable on host.

// Converts a raw ADC average (0..adcMax) into a calibrated real-world voltage,
// using a known-good reference reading (calInVoltage measured at calA0Voltage).
float calibrate_voltage(float rawAdcAverage, float adcMax, float adcReferenceVolts,
                         float calInVoltage, float calA0Voltage);

// Median of n raw ADC samples (n <= 15). Rejects the ripple/ignition spikes an
// alternator at high RPM puts on the sense line, which a plain average smears into
// the reading (and which can trip the 14.4V cutoff for a single bogus sample).
int median_reading(const int* samples, int n);

#endif
