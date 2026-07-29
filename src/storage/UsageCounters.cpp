#include "UsageCounters.h"
#include <string.h>

namespace {
constexpr uint32_t kMagic = 0x54524B31; // "TRK1"
}

UsageCounters::UsageCounters(IFlashStore& store, uint32_t defaultServiceIntervalHours,
                             uint32_t saveThresholdSeconds)
  : store_(store), defaultServiceIntervalHours_(defaultServiceIntervalHours),
    saveThresholdSeconds_(saveThresholdSeconds),
    data_{0, 0, 0, 0}, lastTickMillis_(0), unsavedSeconds_(0) {}

void UsageCounters::begin(unsigned long currentMillis) {
  lastTickMillis_ = currentMillis;
  unsavedSeconds_ = 0;

  UsageData loaded;
  if (store_.readBlob(&loaded, sizeof(loaded)) && loaded.magic == kMagic) {
    data_ = loaded;
  } else {
    data_ = UsageData{kMagic, 0, 0, defaultServiceIntervalHours_, 0};
  }
  data_.bootCount++; // a real power-on is rare; always save it immediately, no throttling
  save();
}

void UsageCounters::tick(bool engineActive, unsigned long currentMillis) {
  unsigned long elapsedMs = currentMillis - lastTickMillis_;
  lastTickMillis_ = currentMillis;

  if (!engineActive || elapsedMs == 0) return;

  uint32_t elapsedSeconds = static_cast<uint32_t>(elapsedMs / 1000);
  if (elapsedSeconds == 0) return; // accumulate only whole seconds; sub-second remainder is dropped

  data_.totalRunSeconds += elapsedSeconds;
  data_.secondsSinceService += elapsedSeconds;
  unsavedSeconds_ += elapsedSeconds;

  if (unsavedSeconds_ >= saveThresholdSeconds_) {
    save();
  }
}

void UsageCounters::resetMaintenanceCounter() {
  data_.secondsSinceService = 0;
  save();
}

void UsageCounters::setServiceIntervalHours(uint32_t hours) {
  data_.serviceIntervalHours = hours;
  save();
}

void UsageCounters::save() {
  store_.writeBlob(&data_, sizeof(data_));
  unsavedSeconds_ = 0;
}

uint32_t UsageCounters::totalRunSeconds() const { return data_.totalRunSeconds; }
uint32_t UsageCounters::secondsSinceService() const { return data_.secondsSinceService; }
uint32_t UsageCounters::serviceIntervalHours() const { return data_.serviceIntervalHours; }

bool UsageCounters::isMaintenanceDue() const {
  return data_.secondsSinceService >= (static_cast<uint64_t>(data_.serviceIntervalHours) * 3600);
}

uint32_t UsageCounters::bootCount() const { return data_.bootCount; }
