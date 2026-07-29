#include "MaintenanceLog.h"

void sanitize_note(const char* raw, char* out, size_t outSize) {
  if (outSize == 0) return;
  size_t i = 0;
  for (; raw[i] != '\0' && i < outSize - 1; i++) {
    char c = raw[i];
    if (c == ',') c = ';';
    if (c == '\n' || c == '\r') c = ' ';
    out[i] = c;
  }
  out[i] = '\0';
}

MaintenanceLog::MaintenanceLog(IMaintenanceLogStore& store) : store_(store) {}

bool MaintenanceLog::addEntry(uint32_t epochSeconds, uint32_t runHours, const char* rawNote) {
  MaintenanceLogEntry entry{epochSeconds, runHours, {0}};
  sanitize_note(rawNote, entry.note, sizeof(entry.note));
  return store_.append(entry);
}

size_t MaintenanceLog::getEntries(MaintenanceLogEntry* out, size_t maxEntries) {
  return store_.readAll(out, maxEntries);
}
