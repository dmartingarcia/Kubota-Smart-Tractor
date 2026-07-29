#include "LittleFsMaintenanceLogStore.h"
#include <LittleFS.h>

LittleFsMaintenanceLogStore::LittleFsMaintenanceLogStore(const char* path) : path_(path) {
  LittleFS.begin();
}

bool LittleFsMaintenanceLogStore::append(const MaintenanceLogEntry& entry) {
  File f = LittleFS.open(path_, "a");
  if (!f) return false;
  f.printf("%lu,%lu,%s\n", static_cast<unsigned long>(entry.epochSeconds),
           static_cast<unsigned long>(entry.runHours), entry.note);
  f.close();
  return true;
}

size_t LittleFsMaintenanceLogStore::readAll(MaintenanceLogEntry* out, size_t maxEntries) {
  File f = LittleFS.open(path_, "r");
  if (!f) return 0;

  size_t count = 0;
  while (f.available() && count < maxEntries) {
    String line = f.readStringUntil('\n');
    if (line.length() == 0) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) continue;

    out[count].epochSeconds = line.substring(0, c1).toInt();
    out[count].runHours = line.substring(c1 + 1, c2).toInt();
    String note = line.substring(c2 + 1);
    note.toCharArray(out[count].note, sizeof(out[count].note));
    count++;
  }
  f.close();
  return count;
}
