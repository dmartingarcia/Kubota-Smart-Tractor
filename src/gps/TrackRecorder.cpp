#include "TrackRecorder.h"
#include <algorithm>

namespace {
int32_t toE6(double deg) { return static_cast<int32_t>(deg >= 0 ? deg * 1e6 + 0.5 : deg * 1e6 - 0.5); }
double dist(const TrackPoint& a, const TrackPoint& b) { return track_distance_meters(a.latE6, a.lonE6, b.latE6, b.lonE6); }
}

TrackRecorder::TrackRecorder(ITrackStore& store, TrackConfig config)
  : store_(store), config_(config), pendingCount_(0), storedCount_(0), anchor_{0, 0, 0}, head_{0, 0, 0},
    lastRaw_{0, 0, 0}, hasAnchor_(false), hasHead_(false), hasLastRaw_(false), sessionStarted_(false), distanceM_(0) {}

void TrackRecorder::begin() {
  storedCount_ = store_.count();
  distanceM_ = 0;
  TrackPoint chunk[16];
  TrackPoint prev{0, 0, 0};
  bool havePrev = false;
  for (size_t first = 0; first < storedCount_;) {
    size_t n = store_.read(first, chunk, 16);
    if (n == 0) break;
    for (size_t i = 0; i < n; i++) {
      if (is_segment_break(chunk[i])) { havePrev = false; continue; }
      if (havePrev) distanceM_ += dist(prev, chunk[i]);
      prev = chunk[i];
      havePrev = true;
    }
    first += n;
  }
}

void TrackRecorder::record(const TrackPoint& p) {
  if (full()) return; // out of room: keep measuring distance, stop storing
  pending_[pendingCount_++] = p;
  if (pendingCount_ >= config_.flushEvery || pendingCount_ >= kPendingMax) flush();
}

void TrackRecorder::flush() {
  if (pendingCount_ == 0) return;
  if (store_.append(pending_, pendingCount_)) {
    storedCount_ += pendingCount_;
    pendingCount_ = 0;
  } else if (pendingCount_ >= kPendingMax) {
    pendingCount_ = 0; // store keeps failing and the buffer is full: drop rather than block the loop
  }
}

bool TrackRecorder::add(double lat, double lon, uint32_t epochSeconds) {
  TrackPoint n{toE6(lat), toE6(lon), epochSeconds};

  if (!hasAnchor_) {
    if (!sessionStarted_) {
      record(TrackPoint{kSegmentBreak, 0, 0});
      sessionStarted_ = true;
    }
    record(n);
    anchor_ = n;
    hasAnchor_ = true;
    lastRaw_ = n;
    hasLastRaw_ = true;
    return true;
  }

  double step = dist(lastRaw_, n);
  if (step < config_.minRawSpacingM) return false; // jitter
  distanceM_ += step;

  if (!hasHead_) {
    head_ = n;
    hasHead_ = true;
  } else {
    // Keep the head as a real vertex only if skipping it would bend the line, or the straight
    // piece has got long; otherwise it is simply replaced by the newer point.
    bool bends = track_cross_track_meters(anchor_, n, head_) > config_.epsilonM || dist(anchor_, n) > config_.maxSegmentM;
    if (!bends && dist(anchor_, head_) >= config_.minTurnChordM) {
      bends = track_turn_angle_degrees(anchor_, head_, n) > config_.turnAngleDeg;
    }
    if (bends) {
      record(head_);
      anchor_ = head_;
    }
    head_ = n;
  }
  lastRaw_ = n;
  return true;
}

size_t TrackRecorder::size() const {
  return storedCount_ + pendingCount_ + (hasHead_ ? 1 : 0);
}

size_t TrackRecorder::read(size_t first, TrackPoint* out, size_t maxPoints) const {
  size_t total = size(), n = 0;
  while (n < maxPoints && first + n < total) {
    size_t idx = first + n;
    if (idx < storedCount_) {
      size_t got = store_.read(idx, out + n, std::min(maxPoints - n, storedCount_ - idx));
      if (got == 0) break;
      n += got;
    } else if (idx < storedCount_ + pendingCount_) {
      out[n++] = pending_[idx - storedCount_];
    } else {
      out[n++] = head_;
    }
  }
  return n;
}

void TrackRecorder::clear() {
  store_.clear();
  storedCount_ = pendingCount_ = 0;
  hasAnchor_ = hasHead_ = hasLastRaw_ = sessionStarted_ = false;
  distanceM_ = 0;
}
