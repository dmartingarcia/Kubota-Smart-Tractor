#ifndef VOLTAGE_SENSOR_H
#define VOLTAGE_SENSOR_H

// Pure calibration math, no hardware access — testable on host.

// Converts a raw ADC average (0..adcMax) into a calibrated real-world voltage,
// using a known-good reference reading (calInVoltage measured at calA0Voltage).
float calibrate_voltage(float rawAdcAverage, float adcMax, float adcReferenceVolts,
                         float calInVoltage, float calA0Voltage);

#endif
