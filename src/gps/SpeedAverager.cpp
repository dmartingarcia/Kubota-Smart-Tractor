#include "SpeedAverager.h"

SpeedAverager::SpeedAverager(unsigned long windowMs, unsigned long sampleIntervalMs)
  : head_(0), count_(0), windowMs_(windowMs), sampleIntervalMs_(sampleIntervalMs), lastSampleMs_(0) {}

void SpeedAverager::add(double speedKmh, unsigned long nowMs) {
  if (count_ > 0 && nowMs - lastSampleMs_ < sampleIntervalMs_) return;
  lastSampleMs_ = nowMs;
  speeds_[head_] = speedKmh;
  times_[head_] = nowMs;
  head_ = (head_ + 1) % kCapacity;
  if (count_ < kCapacity) count_++;
}

double SpeedAverager::average(unsigned long nowMs) const {
  double total = 0;
  int used = 0;
  for (int i = 0; i < count_; i++) {
    if (nowMs - times_[i] < windowMs_) { total += speeds_[i]; used++; }
  }
  return used ? total / used : 0.0;
}

void SpeedAverager::clear() {
  head_ = 0;
  count_ = 0;
}
