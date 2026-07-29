#ifndef MAINTENANCE_LOG_H
#define MAINTENANCE_LOG_H

#include <stdint.h>
#include <stddef.h>

struct MaintenanceLogEntry {
  uint32_t epochSeconds; // 0 if unknown (no NTP sync yet - e.g. AP-only, never had internet)
  uint32_t runHours;     // total charging hours at time of entry (always available, no RTC needed)
  char note[40];
};

// Sanitizes free-text input for safe storage in the CSV-ish log file: commas become
// semicolons, newlines become spaces, result is truncated to fit outSize (incl. NUL).
void sanitize_note(const char* raw, char* out, size_t outSize);

// HAL boundary: no filesystem includes here, natively testable against a fake store.
class IMaintenanceLogStore {
  public:
    virtual ~IMaintenanceLogStore() = default;
    virtual bool append(const MaintenanceLogEntry& entry) = 0;
    // Reads up to maxEntries in file order (oldest first); returns count read.
    virtual size_t readAll(MaintenanceLogEntry* out, size_t maxEntries) = 0;
};

// Thin wrapper: sanitizes notes before handing entries to the store.
class MaintenanceLog {
  public:
    explicit MaintenanceLog(IMaintenanceLogStore& store);
    bool addEntry(uint32_t epochSeconds, uint32_t runHours, const char* rawNote);
    size_t getEntries(MaintenanceLogEntry* out, size_t maxEntries);

  private:
    IMaintenanceLogStore& store_;
};

#endif
