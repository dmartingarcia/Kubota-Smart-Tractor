#include "VoltageCalibration.h"

namespace {
constexpr uint32_t kMagic = 0x564F4C31; // "VOL1"
}

bool compute_voltage_scale(float measuredVolts, float unscaledVolts, float* scaleOut) {
  if (measuredVolts < 6.0f || measuredVolts > 20.0f || unscaledVolts < 1.0f) return false;
  float scale = measuredVolts / unscaledVolts;
  if (scale < kVoltageScaleMin || scale > kVoltageScaleMax) return false;
  *scaleOut = scale;
  return true;
}

VoltageCalibration::VoltageCalibration(IFlashStore& store) : store_(store), scale_(1.0f) {}

void VoltageCalibration::begin() {
  Blob b;
  if (store_.readBlob(&b, sizeof(b)) && b.magic == kMagic && b.scale >= kVoltageScaleMin && b.scale <= kVoltageScaleMax) {
    scale_ = b.scale;
  }
}

bool VoltageCalibration::calibrate(float measuredVolts, float unscaledVolts) {
  float scale;
  if (!compute_voltage_scale(measuredVolts, unscaledVolts, &scale)) return false;
  scale_ = scale;
  save();
  return true;
}

void VoltageCalibration::reset() {
  if (scale_ == 1.0f) return;
  scale_ = 1.0f;
  save();
}

void VoltageCalibration::save() {
  Blob b{kMagic, scale_};
  store_.writeBlob(&b, sizeof(b));
}
