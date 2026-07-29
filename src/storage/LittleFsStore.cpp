#include "LittleFsStore.h"
#include <LittleFS.h>

LittleFsStore::LittleFsStore(const char* path) : path_(path) {
  LittleFS.begin();
}

bool LittleFsStore::readBlob(void* buffer, size_t size) {
  File f = LittleFS.open(path_, "r");
  if (!f) return false;
  size_t read = f.read(reinterpret_cast<uint8_t*>(buffer), size);
  f.close();
  return read == size;
}

bool LittleFsStore::writeBlob(const void* buffer, size_t size) {
  File f = LittleFS.open(path_, "w");
  if (!f) return false;
  size_t written = f.write(reinterpret_cast<const uint8_t*>(buffer), size);
  f.close();
  return written == size;
}
