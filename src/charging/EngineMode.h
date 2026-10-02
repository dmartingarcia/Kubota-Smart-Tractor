#ifndef ENGINE_MODE_H
#define ENGINE_MODE_H

#include <stdint.h>
#include "../storage/UsageCounters.h" // IFlashStore

// Which signals may declare "engine is on". Any combination (bitmask):
//   ALWAYS     - engine is considered on permanently (fixed mode, no probing)
//   GPS        - on while the tractor moves under a valid GPS fix
//   ALTERNATOR - on when the alternator proves it (voltage probe pulses / charging voltage)
enum EngineSource : uint8_t {
  ENGINE_SRC_ALWAYS     = 1,
  ENGINE_SRC_GPS        = 2,
  ENGINE_SRC_ALTERNATOR = 4,
};
constexpr uint8_t ENGINE_SRC_ALL = ENGINE_SRC_ALWAYS | ENGINE_SRC_GPS | ENGINE_SRC_ALTERNATOR;
constexpr uint8_t ENGINE_SRC_DEFAULT = ENGINE_SRC_GPS | ENGINE_SRC_ALTERNATOR;

// Drops unknown bits; an empty selection falls back to the default (otherwise the
// engine could never be considered on and the alternator would never be driven).
uint8_t sanitize_engine_sources(uint8_t mask);

struct EngineSourceDecision {
  bool forceRunning;  // engine is on regardless of voltage: drive at full output, no probing
  bool useAlternator; // alternator evidence (probing / charging voltage) is allowed to decide
};
EngineSourceDecision decide_engine_sources(uint8_t mask, bool gpsMoving);

// Persists the selection (1 byte + magic) behind the flash HAL. Written only when the
// user changes it, so wear is a non-issue.
class EngineModeSettings {
  public:
    explicit EngineModeSettings(IFlashStore& store);
    void begin();
    uint8_t sources() const { return sources_; }
    void setSources(uint8_t mask);

  private:
    struct Blob { uint32_t magic; uint8_t sources; uint8_t pad[3]; };
    IFlashStore& store_;
    uint8_t sources_;
};

#endif
