#ifndef LITTLE_FS_STORE_H
#define LITTLE_FS_STORE_H

#include "UsageCounters.h"

// Real hardware IFlashStore impl (LittleFS). Excluded from native tests.
class LittleFsStore : public IFlashStore {
  public:
    explicit LittleFsStore(const char* path);
    bool readBlob(void* buffer, size_t size) override;
    bool writeBlob(const void* buffer, size_t size) override;

  private:
    const char* path_;
};

#endif
