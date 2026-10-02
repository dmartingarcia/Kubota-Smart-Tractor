#ifndef PID_SETTINGS_H
#define PID_SETTINGS_H

#include "../storage/UsageCounters.h" // IFlashStore

struct PidGains {
  double kp, ki, kd;
};

// Rejects NaN/inf, negatives, kp <= 0 and absurd magnitudes (a bad autotune must never end up
// persisted and then loaded on every boot).
bool pid_gains_valid(const PidGains& g);

// PID gains that survive reboots: the factory defaults until something valid is saved (e.g.
// by a successful autotune), then the saved custom ones. resetToFactory() drops the custom set.
class PidSettings {
  public:
    PidSettings(IFlashStore& store, PidGains factory);
    void begin();
    PidGains gains() const { return hasCustom_ ? custom_ : factory_; }
    bool hasCustom() const { return hasCustom_; }
    bool save(PidGains g); // false = rejected as invalid, nothing changed
    void resetToFactory();

  private:
    struct Blob { uint32_t magic; float kp, ki, kd; };
    IFlashStore& store_;
    PidGains factory_;
    PidGains custom_;
    bool hasCustom_;
    void write(uint32_t magic, const PidGains& g);
};

#endif
