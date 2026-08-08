#ifndef REAL_TIME_PID_H
#define REAL_TIME_PID_H

// Standard PID with derivative-on-measurement and integral clamped to the output
// range (anti-windup), pure logic, no hardware access.
//
// Unlike the PID_v1 library (see git history), this scales the integral/derivative
// terms by the ACTUAL elapsed time between compute() calls, not a fixed assumed
// SampleTime - PID_v1 bakes SampleTime into ki/kd once and ignores real jitter, so a
// slow loop() iteration (e.g. a blocking MQTT connect) gets silently mis-integrated
// as if only the configured sample time had passed.
class RealTimePid {
  public:
    RealTimePid(double kp, double ki, double kd, double outputMin, double outputMax);

    void setTunings(double kp, double ki, double kd);

    // setpoint/input in the caller's units; currentMillis for real elapsed-time scaling.
    // First call after construction/reset() has no valid dt yet and returns 0.
    double compute(double setpoint, double input, unsigned long currentMillis);

    void reset(); // clears integral/derivative history, e.g. on mode transitions

  private:
    double kp_, ki_, kd_;
    double outputMin_, outputMax_;
    double integral_;
    double lastInput_;
    unsigned long lastMillis_;
    bool hasLast_;
};

#endif
