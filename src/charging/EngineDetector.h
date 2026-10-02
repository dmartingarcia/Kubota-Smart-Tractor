#ifndef ENGINE_DETECTOR_H
#define ENGINE_DETECTOR_H

// Detects whether the engine is actually running before committing to full
// alternator output. The safety envelope calls for MAX_CHARGE whenever voltage
// is low, but a resting (engine-off) battery sits below that threshold too —
// holding the field coil at 100% in that case never charges anything, it just
// drains the battery further. So instead of driving continuously, this pulses
// the alternator on for pulseDurationMs and checks for a voltage rise. No rise
// means the engine is off: back off for cooldownMs before trying another pulse.
class EngineDetector {
  public:
    EngineDetector(unsigned long pulseDurationMs, unsigned long cooldownMs, float voltageRiseVolts,
                   unsigned long graceMs = 0);

    // Call whenever other evidence says the engine is running (voltage in the PID band or
    // over the cutoff). For graceMs afterwards, a voltage sag below the low threshold keeps
    // the alternator driven and the engine "running" instead of triggering a probe: at high
    // RPM the alternator is already at its limit, so a probe pulse can't show a further
    // rise and would wrongly declare the engine stopped.
    void noteRunning(unsigned long currentMillis);

    // Call every cycle while the safety envelope calls for full charge output.
    // Returns whether the alternator should be driven this cycle.
    bool update(float voltage, unsigned long currentMillis);

    bool engineRunning() const;

    // True while actively driving a probe pulse with no verdict yet (voltage hasn't
    // risen, but the pulse window hasn't elapsed either) - distinct from a confirmed
    // "stopped" verdict, since the alternator IS being driven at full output right now.
    bool isProbing() const;

    // Call when leaving the low-voltage/full-charge regime, so the next time
    // it's entered, detection starts from a fresh pulse instead of stale state.
    void reset();

  private:
    enum class Phase { PULSING, COOLDOWN };

    unsigned long pulseDurationMs_;
    unsigned long cooldownMs_;
    float voltageRiseVolts_;

    bool started_;
    Phase phase_;
    unsigned long phaseStartMillis_;
    float baselineVoltage_;
    bool engineRunning_;
    bool probing_;
    unsigned long graceMs_;
    unsigned long lastRunningMillis_;
    bool hasRunning_;
};

#endif
