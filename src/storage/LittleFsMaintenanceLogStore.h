#ifndef LITTLE_FS_MAINTENANCE_LOG_STORE_H
#define LITTLE_FS_MAINTENANCE_LOG_STORE_H

#include "MaintenanceLog.h"

// Real hardware IMaintenanceLogStore impl: append-only CSV file on LittleFS.
// Entries are rare (real maintenance events, not periodic ticks), so unlike
// UsageCounters this has no wear-throttling concern - see LittleFsStore for that.
class LittleFsMaintenanceLogStore : public IMaintenanceLogStore {
  public:
    explicit LittleFsMaintenanceLogStore(const char* path);
    bool append(const MaintenanceLogEntry& entry) override;
    size_t readAll(MaintenanceLogEntry* out, size_t maxEntries) override;

  private:
    const char* path_;
};

#endif
