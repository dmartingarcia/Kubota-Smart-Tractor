#ifndef PID_AUTOTUNER_H
#define PID_AUTOTUNER_H

// Relay-feedback PID autotune (Åström–Hägglund), pure logic, no hardware access.
// Drives the output between (base - step) and (base + step) around setpoint, measures
// the resulting oscillation's amplitude/period, and derives Ziegler-Nichols gains.
// Caller MUST still pass the returned output through the normal safety envelope
// (decide_pwm_safety_action) — autotune never bypasses the over-voltage cutoff.

enum class AutotuneState { RUNNING, SUCCEEDED, FAILED };

struct AutotuneGains {
  double kp;
  double ki;
  double kd;
  double ku; // ultimate gain
  double pu; // ultimate period, ms
};

class PidAutotuner {
  public:
    PidAutotuner(double setpoint, double outputBase, double outputStep, double noiseBand,
                 int cyclesToRecord, unsigned long maxRuntimeMs);

    void begin(unsigned long currentMillis);
    double update(double input, unsigned long currentMillis); // returns relay output level
    AutotuneState state() const;
    AutotuneGains gains() const; // valid once state() == SUCCEEDED

  private:
    double setpoint_, outputBase_, outputStep_, noiseBand_;
    int cyclesToRecord_;
    unsigned long maxRuntimeMs_;

    bool relayHigh_;
    double peakMin_, peakMax_;
    unsigned long lastSwitchMillis_;
    unsigned long startMillis_;
    int halfCyclesSeen_;
    unsigned long periodSumMs_;
    int fullPeriodsRecorded_;
    AutotuneState state_;
    AutotuneGains gains_;

    void finish(unsigned long currentMillis);
};

#endif
