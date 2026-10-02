#include "EngineMode.h"

namespace {
constexpr uint32_t kMagic = 0x454E4731; // "ENG1"
}

uint8_t sanitize_engine_sources(uint8_t mask) {
  mask &= ENGINE_SRC_ALL;
  return mask == 0 ? ENGINE_SRC_DEFAULT : mask;
}

EngineSourceDecision decide_engine_sources(uint8_t mask, bool gpsMoving) {
  mask = sanitize_engine_sources(mask);
  bool force = (mask & ENGINE_SRC_ALWAYS) || ((mask & ENGINE_SRC_GPS) && gpsMoving);
  return EngineSourceDecision{force, (mask & ENGINE_SRC_ALTERNATOR) != 0};
}

EngineModeSettings::EngineModeSettings(IFlashStore& store) : store_(store), sources_(ENGINE_SRC_DEFAULT) {}

void EngineModeSettings::begin() {
  Blob b;
  if (store_.readBlob(&b, sizeof(b)) && b.magic == kMagic) sources_ = sanitize_engine_sources(b.sources);
}

void EngineModeSettings::setSources(uint8_t mask) {
  mask = sanitize_engine_sources(mask);
  if (mask == sources_) return;
  sources_ = mask;
  Blob b{kMagic, sources_, {0, 0, 0}};
  store_.writeBlob(&b, sizeof(b));
}
