#ifndef USAGE_COUNTERS_H
#define USAGE_COUNTERS_H

#include <stdint.h>
#include <stddef.h>

// HAL boundary: no Arduino/filesystem includes here, so this is natively testable
// against a fake store. See LittleFsStore for the real hardware implementation.
class IFlashStore {
  public:
    virtual ~IFlashStore() = default;
    // Returns false if no valid data is present yet (e.g. first boot).
    virtual bool readBlob(void* buffer, size_t size) = 0;
    virtual bool writeBlob(const void* buffer, size_t size) = 0;
};

struct UsageData {
  uint32_t magic;
  uint32_t totalRunSeconds;
  uint32_t secondsSinceService;
  uint32_t serviceIntervalHours;
  uint32_t bootCount;
};

// Tracks engine/alternator running hours and a resettable maintenance counter,
// persisted to flash with throttled writes (to limit flash wear). Call tick() every
// loop() iteration; it's cheap and only touches the store when accumulated unsaved
// time crosses saveThresholdSeconds, a reset/interval change happens, or on load.
class UsageCounters {
  public:
    UsageCounters(IFlashStore& store, uint32_t defaultServiceIntervalHours,
                  uint32_t saveThresholdSeconds);

    void begin(unsigned long currentMillis);
    void tick(bool engineActive, unsigned long currentMillis);

    void resetMaintenanceCounter();
    void setServiceIntervalHours(uint32_t hours);
    void forceSave(); // bypass the throttle - call right when the engine turns off,
                       // since this hardware only has a few seconds of power left then

    uint32_t totalRunSeconds() const;
    uint32_t secondsSinceService() const;
    uint32_t serviceIntervalHours() const;
    bool isMaintenanceDue() const;
    uint32_t bootCount() const;

  private:
    IFlashStore& store_;
    uint32_t defaultServiceIntervalHours_;
    uint32_t saveThresholdSeconds_;

    UsageData data_;
    unsigned long lastTickMillis_;
    uint32_t unsavedSeconds_;

    void save();
};

#endif
