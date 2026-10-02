#ifndef LITTLEFS_TRACK_STORE_H
#define LITTLEFS_TRACK_STORE_H

#include "TrackRecorder.h"

// Real hardware ITrackStore: an append-only file of 12-byte records on LittleFS. Appends are
// small and batched by TrackRecorder, so flash wear is negligible. Excluded from native tests.
class LittleFsTrackStore : public ITrackStore {
  public:
    explicit LittleFsTrackStore(const char* path);
    size_t count() override;
    bool append(const TrackPoint* points, size_t n) override;
    size_t read(size_t first, TrackPoint* out, size_t maxPoints) override;
    void clear() override;

  private:
    const char* path_;
};

#endif
