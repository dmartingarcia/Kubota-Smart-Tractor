#include "LittleFsTrackStore.h"
#include <LittleFS.h>

LittleFsTrackStore::LittleFsTrackStore(const char* path) : path_(path) {
  LittleFS.begin();
}

size_t LittleFsTrackStore::count() {
  File f = LittleFS.open(path_, "r");
  if (!f) return 0;
  size_t n = f.size() / sizeof(TrackPoint);
  f.close();
  return n;
}

bool LittleFsTrackStore::append(const TrackPoint* points, size_t n) {
  File f = LittleFS.open(path_, "a");
  if (!f) return false;
  size_t bytes = n * sizeof(TrackPoint);
  size_t written = f.write(reinterpret_cast<const uint8_t*>(points), bytes);
  f.close();
  return written == bytes;
}

size_t LittleFsTrackStore::read(size_t first, TrackPoint* out, size_t maxPoints) {
  File f = LittleFS.open(path_, "r");
  if (!f) return 0;
  size_t got = 0;
  if (f.seek(first * sizeof(TrackPoint))) {
    got = f.read(reinterpret_cast<uint8_t*>(out), maxPoints * sizeof(TrackPoint)) / sizeof(TrackPoint);
  }
  f.close();
  return got;
}

void LittleFsTrackStore::clear() {
  LittleFS.remove(path_);
}
