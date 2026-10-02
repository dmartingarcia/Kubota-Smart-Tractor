#ifndef SPEED_AVERAGER_H
#define SPEED_AVERAGER_H

// Rolling mean of GPS speed over the last windowMs, sampled at most once per
// sampleIntervalMs. Smooths the 0.5-2 km/h jitter a parked receiver reports and
// bridges momentary stops. Pure logic, no hardware access.
class SpeedAverager {
  public:
    SpeedAverager(unsigned long windowMs, unsigned long sampleIntervalMs);

    void add(double speedKmh, unsigned long nowMs); // call whenever a valid fix is available
    double average(unsigned long nowMs) const;      // 0 when no sample is inside the window
    void clear();

  private:
    static const int kCapacity = 32;
    double speeds_[kCapacity];
    unsigned long times_[kCapacity];
    int head_;
    int count_;
    unsigned long windowMs_;
    unsigned long sampleIntervalMs_;
    unsigned long lastSampleMs_;
};

#endif
