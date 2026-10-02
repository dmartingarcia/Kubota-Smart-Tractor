#include "PidSettings.h"

namespace {
constexpr uint32_t kMagic = 0x50494431;  // "PID1": custom gains stored
constexpr uint32_t kCleared = 0;         // factory restored
constexpr double kMaxGain = 5000.0;
}

bool pid_gains_valid(const PidGains& g) {
  auto sane = [](double v) { return v == v && v >= 0.0 && v <= kMaxGain; }; // v == v is false for NaN
  return sane(g.kp) && sane(g.ki) && sane(g.kd) && g.kp > 0.0;
}

PidSettings::PidSettings(IFlashStore& store, PidGains factory)
  : store_(store), factory_(factory), custom_(factory), hasCustom_(false) {}

void PidSettings::begin() {
  Blob b;
  if (store_.readBlob(&b, sizeof(b)) && b.magic == kMagic) {
    PidGains g{b.kp, b.ki, b.kd};
    if (pid_gains_valid(g)) {
      custom_ = g;
      hasCustom_ = true;
    }
  }
}

bool PidSettings::save(PidGains g) {
  if (!pid_gains_valid(g)) return false;
  custom_ = g;
  hasCustom_ = true;
  write(kMagic, g);
  return true;
}

void PidSettings::resetToFactory() {
  if (!hasCustom_) return;
  hasCustom_ = false;
  write(kCleared, factory_);
}

void PidSettings::write(uint32_t magic, const PidGains& g) {
  Blob b{magic, static_cast<float>(g.kp), static_cast<float>(g.ki), static_cast<float>(g.kd)};
  store_.writeBlob(&b, sizeof(b));
}
