#ifndef TRACK_RECORDER_H
#define TRACK_RECORDER_H

#include "GpsTrack.h"

// HAL: where the simplified track lives (flash on the device, memory in tests). Append-only.
class ITrackStore {
  public:
    virtual ~ITrackStore() = default;
    virtual size_t count() = 0;
    virtual bool append(const TrackPoint* points, size_t n) = 0;
    virtual size_t read(size_t first, TrackPoint* out, size_t maxPoints) = 0; // returns points read
    virtual void clear() = 0;
};

struct TrackConfig {
  double epsilonM = 1.5;        // a point is kept when dropping it would bend the line by more than this
  double turnAngleDeg = 15.0;   // ...or when the heading changes by more than this (sets the detail of turns)
  double minTurnChordM = 4.0;   // ...measured over at least this much path, so GPS wobble isn't read as turns
  double maxSegmentM = 60.0;    // ...or when a straight piece gets longer than this
  double minRawSpacingM = 2.0;  // raw fixes closer than this to the previous one are GPS jitter
  size_t flushEvery = 16;       // pending points written to the store in batches (flash wear)
  size_t maxPoints = 20000;     // hard cap on stored points (12 bytes each)
};

// Records the path of the tractor with ONLINE line simplification (Opening-window style): a
// straight pass costs a couple of points, a turn is stored in detail - so a whole working day
// fits in flash with lines and turns visible. A segment-break marker is written at the start
// of every session so the map/GPX never join the end of one session to the start of the next.
class TrackRecorder {
  public:
    TrackRecorder(ITrackStore& store, TrackConfig config = TrackConfig());

    void begin();                                            // loads count + distance from the store
    bool add(double lat, double lon, uint32_t epochSeconds); // feed raw fixes while moving; true if the fix was used
    void flush();                                            // push pending points to the store
    void clear();

    size_t size() const;                                     // stored + pending + the current head point
    size_t read(size_t first, TrackPoint* out, size_t maxPoints) const; // unified view of all of them
    double distanceMeters() const { return distanceM_; }     // true path length (raw, not simplified)
    bool full() const { return storedPlusPending() >= config_.maxPoints; }

  private:
    static const size_t kPendingMax = 32;
    ITrackStore& store_;
    TrackConfig config_;
    TrackPoint pending_[kPendingMax];
    size_t pendingCount_;
    size_t storedCount_;
    TrackPoint anchor_, head_, lastRaw_;
    bool hasAnchor_, hasHead_, hasLastRaw_, sessionStarted_;
    double distanceM_;

    size_t storedPlusPending() const { return storedCount_ + pendingCount_; }
    void record(const TrackPoint& p);
};

#endif
