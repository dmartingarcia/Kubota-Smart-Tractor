#include <unity.h>
#include "../src/charging/VoltageSensor.h"
#include "../src/charging/AlternatorLogic.h"
#include "../src/charging/PidAutotuner.h"
#include "../src/charging/EngineDetector.h"
#include "../src/charging/PwmMirror.h"
#include "../src/charging/StatusLed.h"
#include "../src/charging/RealTimePid.h"
#include "../src/charging/EngineMode.h"
#include "../src/charging/VoltageCalibration.h"
#include "../src/charging/PidSettings.h"
#include "../src/storage/UsageCounters.h"
#include "../src/connectivity/MqttPublisher.h"
#include "../src/storage/MaintenanceLog.h"
#include "../src/gps/GpsJumpFilter.h"
#include "../src/gps/SpeedAverager.h"
#include "../src/gps/GpsTrack.h"
#include "../src/gps/TrackRecorder.h"
#include <string.h>
#include <string>
#include <math.h>
#include <vector>

void setUp() {}
void tearDown() {}

void test_voltage_calibration() {
    // 2.90V at A0 corresponds to the calibrated 15.25V reference.
    float raw = (2.90f / 3.3f) * 1024.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.05, 15.25, calibrate_voltage(raw, 1024.0, 3.3, 15.25, 2.90));

    // Half the reference reading should give half the reference voltage.
    raw = (1.45f / 3.3f) * 1024.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.05, 7.625, calibrate_voltage(raw, 1024.0, 3.3, 15.25, 2.90));
}

void test_pwm_safety_thresholds() {
    TEST_ASSERT_EQUAL(static_cast<int>(ChargeAction::OFF), static_cast<int>(decide_pwm_safety_action(14.8, 14.6, 13.0)));
    TEST_ASSERT_EQUAL(static_cast<int>(ChargeAction::OFF), static_cast<int>(decide_pwm_safety_action(14.6, 14.6, 13.0))); // boundary is inclusive
    TEST_ASSERT_EQUAL(static_cast<int>(ChargeAction::MAX_CHARGE), static_cast<int>(decide_pwm_safety_action(12.5, 14.6, 13.0)));
    TEST_ASSERT_EQUAL(static_cast<int>(ChargeAction::RUN_PID), static_cast<int>(decide_pwm_safety_action(14.0, 14.6, 13.0)));
}

void test_relay_overvoltage_turns_off_and_starts_delay() {
    RelayDecision d = decide_relay_state(true, 15.0, 1000, 0, 14.8, 14.0, 20000);
    TEST_ASSERT_TRUE(d.changed);
    TEST_ASSERT_FALSE(d.state);
    TEST_ASSERT_EQUAL_UINT32(21000, d.nextCheck);
}

void test_relay_stays_off_during_delay_window() {
    // Between thresholds, delay not yet elapsed -> no change.
    RelayDecision d = decide_relay_state(false, 14.5, 5000, 21000, 14.8, 14.0, 20000);
    TEST_ASSERT_FALSE(d.changed);
    TEST_ASSERT_FALSE(d.state);
}

void test_relay_turns_on_below_low_threshold_after_delay() {
    RelayDecision d = decide_relay_state(false, 13.5, 25000, 21000, 14.8, 14.0, 20000);
    TEST_ASSERT_TRUE(d.changed);
    TEST_ASSERT_TRUE(d.state);
    TEST_ASSERT_EQUAL_UINT32(45000, d.nextCheck);
}

void test_gps_motion_means_engine_running() {
    TEST_ASSERT_TRUE(gps_indicates_engine_running(true, 500, 5.0, 3.0));
    TEST_ASSERT_TRUE(gps_indicates_engine_running(true, 500, 3.0, 3.0)); // boundary inclusive
}

void test_gps_standstill_noise_is_not_motion() {
    TEST_ASSERT_FALSE(gps_indicates_engine_running(true, 500, 1.2, 3.0));
}

void test_gps_motion_ignored_without_fresh_fix() {
    TEST_ASSERT_FALSE(gps_indicates_engine_running(false, 500, 20.0, 3.0));  // no fix
    TEST_ASSERT_FALSE(gps_indicates_engine_running(true, 10000, 20.0, 3.0)); // stale speed
}

void test_should_run_cycle_respects_interval() {
    TEST_ASSERT_FALSE(should_run_cycle(10, 0, 20));   // 10ms elapsed, interval 20ms -> not yet
    TEST_ASSERT_TRUE(should_run_cycle(20, 0, 20));     // exactly on interval -> run
    TEST_ASSERT_TRUE(should_run_cycle(25, 0, 20));     // past interval -> run
}

void test_should_run_cycle_handles_millis_rollover() {
    // lastRunMillis close to ULONG_MAX, currentMillis wrapped to a small value:
    // unsigned subtraction wraps around correctly and should still fire on schedule.
    unsigned long lastRun = static_cast<unsigned long>(-5);  // ULONG_MAX - 4
    unsigned long current = 15;                              // 20ms later after wraparound
    TEST_ASSERT_TRUE(should_run_cycle(current, lastRun, 20));
    TEST_ASSERT_FALSE(should_run_cycle(10, lastRun, 20));
}

void test_autotune_computes_gains_from_relay_oscillation() {
    // setpoint=140, noiseBand=2 -> switches at 142 (up) / 138 (down).
    // Feeding a clean 130<->150 triangle wave, period 1000ms, amplitude 10.
    PidAutotuner tuner(140.0, 500.0, 200.0, 2.0, 4, 60000);
    tuner.begin(0);
    double inputs[] = {130, 150, 130, 150, 130, 150};
    unsigned long times[] = {0, 500, 1000, 1500, 2000, 2500};
    double lastOutput = 0;
    for (int i = 0; i < 6; i++) {
        lastOutput = tuner.update(inputs[i], times[i]);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(AutotuneState::SUCCEEDED), static_cast<int>(tuner.state()));

    AutotuneGains g = tuner.gains();
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1000.0, g.pu);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 25.46, g.ku);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 15.28, g.kp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 30.56, g.ki);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1.91, g.kd);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 300.0, lastOutput); // last switch was to relayHigh_=false -> base-step
}

void test_autotune_fails_on_zero_amplitude() {
    // noiseBand=0 -> input==setpoint satisfies both switch conditions every call,
    // so peakMax_/peakMin_ never diverge from setpoint: genuine zero amplitude.
    PidAutotuner tuner(140.0, 500.0, 200.0, 0.0, 2, 60000);
    tuner.begin(0);
    for (int i = 0; i < 4; i++) {
        tuner.update(140.0, i * 100);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(AutotuneState::FAILED), static_cast<int>(tuner.state()));
}

void test_autotune_update_after_completion_is_a_no_op() {
    PidAutotuner tuner(140.0, 500.0, 200.0, 2.0, 4, 60000);
    tuner.begin(0);
    double inputs[] = {130, 150, 130, 150, 130, 150};
    unsigned long times[] = {0, 500, 1000, 1500, 2000, 2500};
    for (int i = 0; i < 6; i++) tuner.update(inputs[i], times[i]);
    TEST_ASSERT_EQUAL(static_cast<int>(AutotuneState::SUCCEEDED), static_cast<int>(tuner.state()));
    AutotuneGains before = tuner.gains();

    double out = tuner.update(9999.0, 999999); // must not recompute/crash once finished
    TEST_ASSERT_EQUAL(static_cast<int>(AutotuneState::SUCCEEDED), static_cast<int>(tuner.state()));
    TEST_ASSERT_EQUAL_FLOAT(before.kp, tuner.gains().kp);
    TEST_ASSERT_TRUE(out == 300.0 || out == 700.0); // still just the relay output level
}

void test_autotune_fails_if_no_oscillation_within_runtime() {
    PidAutotuner tuner(140.0, 500.0, 200.0, 2.0, 4, 1000);
    tuner.begin(0);
    tuner.update(140.0, 500);  // sits right at setpoint, never crosses noise band
    tuner.update(140.0, 1500); // past maxRuntimeMs with zero switches
    TEST_ASSERT_EQUAL(static_cast<int>(AutotuneState::FAILED), static_cast<int>(tuner.state()));
}

void test_engine_detector_starts_with_a_pulse() {
    EngineDetector detector(2000, 60000, 0.3);
    TEST_ASSERT_TRUE(detector.update(12.5, 0)); // first call always pulses
    TEST_ASSERT_FALSE(detector.engineRunning());
}

void test_engine_detector_confirms_engine_running_on_voltage_rise() {
    EngineDetector detector(2000, 60000, 0.3);
    detector.update(12.5, 0);              // baseline captured at pulse start
    TEST_ASSERT_TRUE(detector.update(12.9, 500)); // rose 0.4V mid-pulse -> keep driving
    TEST_ASSERT_TRUE(detector.engineRunning());
}

void test_engine_detector_backs_off_after_a_dry_pulse() {
    EngineDetector detector(2000, 60000, 0.3);
    detector.update(12.5, 0);                       // pulse starts
    TEST_ASSERT_TRUE(detector.update(12.5, 1000));  // still within pulse window, no rise yet
    TEST_ASSERT_FALSE(detector.update(12.5, 2000)); // pulse window elapsed, no rise -> cooldown
    TEST_ASSERT_FALSE(detector.engineRunning());
}

void test_engine_detector_waits_full_cooldown_before_next_pulse() {
    EngineDetector detector(2000, 60000, 0.3);
    detector.update(12.5, 0);
    detector.update(12.5, 2000); // enters cooldown at t=2000
    TEST_ASSERT_FALSE(detector.update(12.5, 40000)); // only 38s into 60s cooldown
    TEST_ASSERT_TRUE(detector.update(12.5, 62000));  // 60s elapsed -> pulses again
}

void test_engine_detector_reset_clears_stale_state() {
    EngineDetector detector(2000, 60000, 0.3);
    detector.update(12.5, 0);
    detector.update(12.9, 100); // engine confirmed running
    TEST_ASSERT_TRUE(detector.engineRunning());

    detector.reset();
    TEST_ASSERT_FALSE(detector.engineRunning());
    TEST_ASSERT_TRUE(detector.update(12.0, 200000)); // starts a fresh pulse, not stuck in old state
}


void test_engine_detector_grace_keeps_running_through_voltage_sag() {
    EngineDetector detector(2000, 60000, 0.3, 15000);
    detector.noteRunning(1000);                       // PID/OFF branch saw the engine charging
    TEST_ASSERT_TRUE(detector.update(12.8, 2000));    // sag below low threshold, no rise possible
    TEST_ASSERT_TRUE(detector.engineRunning());
    TEST_ASSERT_FALSE(detector.isProbing());
    TEST_ASSERT_TRUE(detector.update(12.8, 9000));    // still inside grace
    TEST_ASSERT_TRUE(detector.engineRunning());
}

void test_engine_detector_grace_expires_into_probe() {
    EngineDetector detector(2000, 60000, 0.3, 15000);
    detector.noteRunning(1000);
    TEST_ASSERT_TRUE(detector.update(12.8, 17000));   // grace over -> fresh probe pulse
    TEST_ASSERT_TRUE(detector.isProbing());
    TEST_ASSERT_FALSE(detector.update(12.8, 19000));  // dry pulse -> stopped
    TEST_ASSERT_FALSE(detector.engineRunning());
}

void test_engine_detector_reset_drops_grace() {
    EngineDetector detector(2000, 60000, 0.3, 15000);
    detector.noteRunning(1000);
    detector.reset();
    TEST_ASSERT_TRUE(detector.update(12.8, 2000));
    TEST_ASSERT_TRUE(detector.isProbing());
}

void test_median_reading_rejects_spike() {
    int s[5] = {500, 502, 1023, 501, 499};
    TEST_ASSERT_EQUAL_INT(501, median_reading(s, 5));
    int single[1] = {7};
    TEST_ASSERT_EQUAL_INT(7, median_reading(single, 1));
}

void test_mirror_duty_follows_output_exactly() {
    TEST_ASSERT_EQUAL_UINT16(0, mirror_duty(0, 1023, false));
    TEST_ASSERT_EQUAL_UINT16(512, mirror_duty(512, 1023, false));
    TEST_ASSERT_EQUAL_UINT16(1023, mirror_duty(1023, 1023, false));
}

void test_mirror_duty_inverts_for_active_low_led() {
    TEST_ASSERT_EQUAL_UINT16(1023, mirror_duty(0, 1023, true));   // output off -> LED off (pin high)
    TEST_ASSERT_EQUAL_UINT16(0, mirror_duty(1023, 1023, true));   // output full -> LED full (pin low)
    TEST_ASSERT_EQUAL_UINT16(511, mirror_duty(512, 1023, true));
}

void test_mirror_duty_clamps_overrange() {
    TEST_ASSERT_EQUAL_UINT16(1023, mirror_duty(2000, 1023, false));
    TEST_ASSERT_EQUAL_UINT16(0, mirror_duty(2000, 1023, true));
}

void test_engine_detector_small_rise_is_enough_during_pulse() {
    EngineDetector detector(3000, 8000, 0.05);
    detector.update(12.60, 0);
    TEST_ASSERT_TRUE(detector.update(12.66, 800)); // idle-RPM sized rise
    TEST_ASSERT_TRUE(detector.engineRunning());
}

void test_engine_detector_cooldown_ignores_voltage_until_it_elapses() {
    EngineDetector detector(3000, 8000, 0.05);
    detector.update(12.60, 0);          // resting level
    detector.update(12.30, 3000);       // sagged under field load, dry pulse -> cooldown
    TEST_ASSERT_FALSE(detector.update(12.60, 4000)); // recovers to rest: not a start
    TEST_ASSERT_FALSE(detector.update(12.62, 5000));
}

// Test-only fake store: in-memory blob, no real flash.
class InMemoryFlashStore : public IFlashStore {
  public:
    uint8_t buffer[64];
    bool hasData = false;
    int writeCalls = 0;

    bool readBlob(void* out, size_t size) override {
        if (!hasData || size > sizeof(buffer)) return false;
        memcpy(out, buffer, size);
        return true;
    }
    bool writeBlob(const void* in, size_t size) override {
        if (size > sizeof(buffer)) return false;
        memcpy(buffer, in, size);
        hasData = true;
        writeCalls++;
        return true;
    }
};

void test_usage_counters_first_boot_uses_defaults() {
    InMemoryFlashStore store;
    UsageCounters counters(store, 250, 300);
    counters.begin(0);
    TEST_ASSERT_EQUAL_UINT32(0, counters.totalRunSeconds());
    TEST_ASSERT_EQUAL_UINT32(250, counters.serviceIntervalHours());
    TEST_ASSERT_EQUAL(1, store.writeCalls); // initial defaults get persisted
}

void test_usage_counters_accumulate_only_while_engine_active() {
    InMemoryFlashStore store;
    UsageCounters counters(store, 250, 300);
    counters.begin(0);
    counters.tick(true, 10000);   // 10s active
    counters.tick(false, 20000);  // 10s idle, not counted
    counters.tick(true, 25000);   // 5s active
    TEST_ASSERT_EQUAL_UINT32(15, counters.totalRunSeconds());
    TEST_ASSERT_EQUAL_UINT32(15, counters.secondsSinceService());
}

void test_usage_counters_count_with_fast_loop_ticks() {
    // Real loop() ticks every few ms, far below 1s: sub-second time must carry over, not be dropped.
    InMemoryFlashStore store;
    UsageCounters counters(store, 250, 300);
    counters.begin(0);
    for (unsigned long t = 20; t <= 10000; t += 20) counters.tick(true, t);
    TEST_ASSERT_EQUAL_UINT32(10, counters.totalRunSeconds());
}

void test_usage_counters_idle_time_not_carried_into_next_run() {
    InMemoryFlashStore store;
    UsageCounters counters(store, 250, 300);
    counters.begin(0);
    for (unsigned long t = 20; t <= 5000; t += 20) counters.tick(true, t);   // 5s running
    for (unsigned long t = 5020; t <= 65000; t += 20) counters.tick(false, t); // 60s stopped
    counters.tick(true, 65500);                                              // engine back on
    TEST_ASSERT_EQUAL_UINT32(5, counters.totalRunSeconds());                 // idle minute not counted
}

void test_usage_counters_throttles_writes() {
    InMemoryFlashStore store;
    UsageCounters counters(store, 250, 300); // save every 300s of accumulated runtime
    counters.begin(0);
    int callsAfterInit = store.writeCalls;
    counters.tick(true, 100000); // 100s active, below threshold
    TEST_ASSERT_EQUAL(callsAfterInit, store.writeCalls);
    counters.tick(true, 400000); // +300s -> crosses threshold
    TEST_ASSERT_EQUAL(callsAfterInit + 1, store.writeCalls);
}

void test_usage_counters_reset_and_reload_across_reboot() {
    InMemoryFlashStore store;
    {
        UsageCounters counters(store, 250, 300);
        counters.begin(0);
        counters.tick(true, 500000); // 500s active
        counters.setServiceIntervalHours(100);
        TEST_ASSERT_FALSE(counters.isMaintenanceDue());
    }
    // Simulate reboot: new instance, same underlying store.
    UsageCounters reloaded(store, 250, 300);
    reloaded.begin(0);
    TEST_ASSERT_EQUAL_UINT32(500, reloaded.totalRunSeconds());
    TEST_ASSERT_EQUAL_UINT32(500, reloaded.secondsSinceService());
    TEST_ASSERT_EQUAL_UINT32(100, reloaded.serviceIntervalHours());

    reloaded.resetMaintenanceCounter();
    TEST_ASSERT_EQUAL_UINT32(0, reloaded.secondsSinceService());
    TEST_ASSERT_EQUAL_UINT32(500, reloaded.totalRunSeconds()); // total hours unaffected by service reset
}

void test_usage_counters_boot_count_increments_each_boot() {
    InMemoryFlashStore store;
    {
        UsageCounters counters(store, 250, 300);
        counters.begin(0);
        TEST_ASSERT_EQUAL_UINT32(1, counters.bootCount());
    }
    UsageCounters rebooted(store, 250, 300);
    rebooted.begin(1000);
    TEST_ASSERT_EQUAL_UINT32(2, rebooted.bootCount());
}

void test_usage_counters_maintenance_due() {
    InMemoryFlashStore store;
    UsageCounters counters(store, 1, 300); // 1-hour service interval
    counters.begin(0);
    counters.tick(true, 3599 * 1000UL);
    TEST_ASSERT_FALSE(counters.isMaintenanceDue());
    counters.tick(true, 3601 * 1000UL);
    TEST_ASSERT_TRUE(counters.isMaintenanceDue());
}

// Test-only fake MQTT transport: no real network, records every publish.
struct SentMessage { std::string topic; std::string payload; bool retain; };
class FakeMqttTransport : public IMqttTransport {
  public:
    bool connectedState = false;
    bool connectResult = true;
    int connectCalls = 0;
    int publishCalls = 0;
    std::vector<SentMessage> sent;

    bool connected() override { return connectedState; }
    bool connect() override {
        connectCalls++;
        connectedState = connectResult;
        return connectResult;
    }
    bool publish(const char* topic, const char* payload, bool retain) override {
        publishCalls++;
        sent.push_back({topic, payload, retain});
        return true;
    }
    void loop() override {}

    int countTopic(const char* topic) const {
        int n = 0;
        for (const auto& m : sent) if (m.topic == topic) n++;
        return n;
    }
    const SentMessage* lastOn(const char* topic) const {
        for (size_t i = sent.size(); i-- > 0;) if (sent[i].topic == topic) return &sent[i];
        return nullptr;
    }
    int discoveryCount() const {
        int n = 0;
        for (const auto& m : sent) if (m.topic.rfind("homeassistant/", 0) == 0) n++;
        return n;
    }
};

MqttReading make_reading(unsigned long ts) {
    // timestamp, voltage, pwmValue, pwmPercent, active, engineRunning, totalRunSeconds, maintenanceDue
    return MqttReading{ts, 14.2f, 500, 49, true, false, 3600, false};
}

void test_mqtt_buffers_when_network_unavailable() {
    FakeMqttTransport t;
    MqttPublisher pub(t, 5, 30000);
    pub.recordSample(make_reading(1000));
    pub.update(false, 1000);
    TEST_ASSERT_EQUAL(0, t.connectCalls);
    TEST_ASSERT_EQUAL(0, t.publishCalls);
    TEST_ASSERT_EQUAL(1, static_cast<int>(pub.bufferedCount()));
}

void test_mqtt_throttles_reconnect_attempts() {
    FakeMqttTransport t;
    t.connectResult = false; // always fails
    MqttPublisher pub(t, 5, 30000);
    pub.update(true, 0);       // first call -> attempts immediately
    TEST_ASSERT_EQUAL(1, t.connectCalls);
    pub.update(true, 10000); // within interval -> no retry
    TEST_ASSERT_EQUAL(1, t.connectCalls);
    pub.update(true, 31000); // past interval -> retries
    TEST_ASSERT_EQUAL(2, t.connectCalls);
}

void test_mqtt_flushes_buffer_and_publishes_on_reconnect() {
    FakeMqttTransport t;
    MqttPublisher pub(t, 5, 30000);
    pub.recordSample(make_reading(0));
    pub.update(true, 0); // not connected: triggers successful connect, doesn't publish yet
    TEST_ASSERT_EQUAL(1, static_cast<int>(pub.bufferedCount()));
    TEST_ASSERT_EQUAL(0, pub.publishCount());

    pub.update(true, 100); // now connected: discovery + batch(1)
    TEST_ASSERT_EQUAL(0, static_cast<int>(pub.bufferedCount()));
    TEST_ASSERT_EQUAL(1, pub.publishCount());
    TEST_ASSERT_EQUAL(13, t.discoveryCount());
    TEST_ASSERT_EQUAL(15, t.publishCalls); // 13 discovery configs + 1 state + 1 history batch

    pub.recordSample(make_reading(200));
    pub.update(true, 200); // already connected, discovery not repeated
    TEST_ASSERT_EQUAL(2, pub.publishCount());
    TEST_ASSERT_EQUAL(17, t.publishCalls); // + state + history batch, discovery not repeated
}

void test_mqtt_buffer_drops_oldest_when_full() {
    FakeMqttTransport t;
    MqttPublisher pub(t, 2, 30000); // capacity 2
    pub.recordSample(make_reading(1));
    pub.recordSample(make_reading(2));
    pub.recordSample(make_reading(3)); // drops reading #1
    TEST_ASSERT_EQUAL(2, static_cast<int>(pub.bufferedCount()));
}


MqttReading make_reading_v(unsigned long ts, float volts) {
    MqttReading r = make_reading(ts);
    r.voltage = volts;
    return r;
}

void test_mqtt_state_topic_is_latest_reading_even_with_backlog() {
    FakeMqttTransport t;
    t.connectedState = true;
    MqttPublisher pub(t, 50, 30000);
    for (int i = 0; i < 20; i++) pub.recordSample(make_reading_v(1000UL * i, 12.0f + i * 0.1f)); // 20 buffered, last is 13.90V
    pub.update(true, 100000);

    const SentMessage* state = t.lastOn("kubotio/tractor/state");
    TEST_ASSERT_NOT_NULL(state);
    TEST_ASSERT_TRUE(state->retain);
    TEST_ASSERT_EQUAL_CHAR('{', state->payload[0]);                       // one object, not an array
    TEST_ASSERT_NOT_EQUAL(std::string::npos, state->payload.find("\"voltage\":13.90")); // newest, not oldest
}

void test_mqtt_history_topic_drains_oldest_first_in_capped_batches() {
    FakeMqttTransport t;
    t.connectedState = true;
    MqttPublisher pub(t, 50, 30000);
    for (int i = 0; i < 20; i++) pub.recordSample(make_reading_v(1000UL * i, 12.0f));
    pub.update(true, 100000);
    TEST_ASSERT_EQUAL(8, pub.publishCount()); // kMaxPerBatch
    const SentMessage* hist = t.lastOn("kubotio/tractor/history");
    TEST_ASSERT_NOT_NULL(hist);
    TEST_ASSERT_FALSE(hist->retain);
    TEST_ASSERT_EQUAL_CHAR('[', hist->payload[0]);
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hist->payload.find("\"timestamp\":0,"));    // oldest first
    TEST_ASSERT_EQUAL(std::string::npos, hist->payload.find("\"timestamp\":8000,"));     // 9th waits for the next cycle
}

void test_mqtt_state_not_republished_without_new_sample() {
    FakeMqttTransport t;
    t.connectedState = true;
    MqttPublisher pub(t, 50, 30000);
    for (int i = 0; i < 20; i++) pub.recordSample(make_reading(1000UL * i));
    pub.update(true, 100000);
    pub.update(true, 130000); // no new sample, backlog still draining
    TEST_ASSERT_EQUAL(1, t.countTopic("kubotio/tractor/state"));
    TEST_ASSERT_EQUAL(2, t.countTopic("kubotio/tractor/history"));
}

void test_mqtt_gps_topic_only_with_fix_and_carries_position() {
    FakeMqttTransport t;
    t.connectedState = true;
    MqttPublisher pub(t, 50, 30000);

    pub.recordSample(make_reading(0));
    pub.update(true, 1000);
    TEST_ASSERT_EQUAL(0, t.countTopic("kubotio/tractor/gps")); // default live state: no fix

    MqttLiveState live{};
    live.gpsHasFix = true; live.latitude = 41.123456; live.longitude = 2.654321;
    live.speedKmh = 7.5f; live.speedAvgKmh = 6.0f; live.engineProbing = true; live.engineSources = 6; live.fault = 3;
    pub.setLiveState(live);
    pub.recordSample(make_reading(10000));
    pub.update(true, 11000);

    const SentMessage* gps = t.lastOn("kubotio/tractor/gps");
    TEST_ASSERT_NOT_NULL(gps);
    TEST_ASSERT_TRUE(gps->retain);
    TEST_ASSERT_NOT_EQUAL(std::string::npos, gps->payload.find("\"latitude\":41.123456"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, gps->payload.find("\"longitude\":2.654321"));

    const SentMessage* state = t.lastOn("kubotio/tractor/state");
    TEST_ASSERT_NOT_EQUAL(std::string::npos, state->payload.find("\"gpsSpeedKmh\":7.5"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, state->payload.find("\"engineProbing\":true"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, state->payload.find("\"engineSources\":6"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, state->payload.find("\"fault\":3"));
}

void test_mqtt_readings_carry_epoch_when_known() {
    FakeMqttTransport t;
    t.connectedState = true;
    MqttPublisher pub(t, 50, 30000);
    MqttReading r = make_reading(0);
    r.epochSeconds = 1780000000;
    pub.recordSample(r);
    pub.update(true, 1000);
    TEST_ASSERT_NOT_EQUAL(std::string::npos, t.lastOn("kubotio/tractor/state")->payload.find("\"epoch\":1780000000"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, t.lastOn("kubotio/tractor/history")->payload.find("\"epoch\":1780000000"));
}

void test_mqtt_discovery_entities_read_the_state_object() {
    FakeMqttTransport t;
    t.connectedState = true;
    MqttPublisher pub(t, 5, 30000);
    pub.recordSample(make_reading(0));
    pub.update(true, 0);
    int stateEntities = 0;
    for (const auto& m : t.sent) {
        if (m.topic.rfind("homeassistant/", 0) != 0) continue;
        TEST_ASSERT_TRUE(m.retain);
        TEST_ASSERT_EQUAL(std::string::npos, m.payload.find("[-1]")); // no more array indexing
        if (m.payload.find("\"state_topic\":\"kubotio/tractor/state\"") != std::string::npos) stateEntities++;
    }
    TEST_ASSERT_EQUAL(12, stateEntities);                                  // 12 entities on the state topic
    TEST_ASSERT_NOT_NULL(t.lastOn("homeassistant/device_tracker/kubotio_tractor/config"));
    TEST_ASSERT_NOT_NULL(t.lastOn("homeassistant/binary_sensor/kubotio_engine_running/config"));
    TEST_ASSERT_NOT_NULL(t.lastOn("homeassistant/sensor/kubotio_gps_speed/config"));
    TEST_ASSERT_NOT_NULL(t.lastOn("homeassistant/binary_sensor/kubotio_fault_active/config"));
    TEST_ASSERT_NOT_NULL(t.lastOn("homeassistant/sensor/kubotio_fault_text/config"));
}

void test_mqtt_worst_case_batch_fits_transport_buffer() {
    FakeMqttTransport t;
    t.connectedState = true;
    MqttPublisher pub(t, 50, 30000);
    MqttReading r{4294967295UL, 14.40f, 1023, 100, false, false, 4000000000UL, false, 81920, false, 4294967295UL};
    for (int i = 0; i < 20; i++) pub.recordSample(r);
    pub.update(true, 1);
    TEST_ASSERT_TRUE(t.lastOn("kubotio/tractor/history")->payload.size() < 2100);
    TEST_ASSERT_EQUAL(8, pub.publishCount());
}

void test_mqtt_reconnect_backs_off_while_broker_is_unreachable() {
    FakeMqttTransport t;
    t.connectResult = false;
    MqttPublisher pub(t, 5, 30000);
    pub.update(true, 0);        // attempt 1
    pub.update(true, 30000);    // attempt 2 (after 30s)
    TEST_ASSERT_EQUAL(2, t.connectCalls);
    pub.update(true, 60000);    // only 30s since attempt 2, backoff is now 60s -> no attempt
    TEST_ASSERT_EQUAL(2, t.connectCalls);
    pub.update(true, 90000);    // 60s since attempt 2 -> attempt 3
    TEST_ASSERT_EQUAL(3, t.connectCalls);
    pub.update(true, 150000);   // backoff 120s -> not yet
    TEST_ASSERT_EQUAL(3, t.connectCalls);
}

void test_mqtt_backoff_is_capped_and_resets_on_success() {
    FakeMqttTransport t;
    t.connectResult = false;
    MqttPublisher pub(t, 5, 30000);
    unsigned long now = 0;
    for (int i = 0; i < 12; i++) { pub.update(true, now); now += 600000; } // far apart: every call attempts
    int attempts = t.connectCalls;
    TEST_ASSERT_EQUAL(12, attempts);
    pub.update(true, now);                 // 0s later than last attempt
    now += 300000;                         // cap is 5 min
    pub.update(true, now);
    TEST_ASSERT_EQUAL(attempts + 2, t.connectCalls);

    t.connectResult = true;
    pub.update(true, now + 300000);        // connects
    t.connectedState = false;              // drop again
    t.connectResult = false;
    int before = t.connectCalls;
    pub.update(true, now + 330000);        // backoff reset: base interval (30s) applies again
    TEST_ASSERT_EQUAL(before + 1, t.connectCalls);
}

void test_gps_jump_filter_accepts_first_fix() {
    GpsJumpFilter f(60.0, 3);
    TEST_ASSERT_TRUE(f.accept(0.0, 0.0, 1000));
    TEST_ASSERT_EQUAL_DOUBLE(0.0, f.lastLat());
}

void test_gps_jump_filter_accepts_plausible_movement() {
    GpsJumpFilter f(60.0, 3);
    f.accept(0.0, 0.0, 0);
    // ~11m over 5s = ~8km/h, well under the 60km/h cap
    TEST_ASSERT_TRUE(f.accept(0.0, 0.0001, 5000));
    TEST_ASSERT_EQUAL_DOUBLE(0.0001, f.lastLon());
}

void test_gps_jump_filter_rejects_impossible_jump() {
    GpsJumpFilter f(60.0, 3);
    f.accept(0.0, 0.0, 0);
    // ~111km in 1s - no tractor does that
    TEST_ASSERT_FALSE(f.accept(1.0, 0.0, 1000));
    TEST_ASSERT_EQUAL_DOUBLE(0.0, f.lastLat()); // reference point unchanged
}

void test_gps_jump_filter_resyncs_after_consecutive_rejects() {
    GpsJumpFilter f(60.0, 2); // resync after 2 rejects
    f.accept(0.0, 0.0, 0);
    TEST_ASSERT_FALSE(f.accept(1.0, 0.0, 1000)); // reject 1
    TEST_ASSERT_FALSE(f.accept(1.0, 0.0, 2000)); // reject 2
    TEST_ASSERT_TRUE(f.accept(1.0, 0.0, 3000));  // resync: accepted unconditionally
    TEST_ASSERT_EQUAL_DOUBLE(1.0, f.lastLat());
}

void test_sanitize_note_escapes_commas_and_newlines() {
    char out[40];
    sanitize_note("Oil change, filter\nreplaced", out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("Oil change; filter replaced", out);
}

void test_sanitize_note_truncates_to_buffer() {
    char out[8];
    sanitize_note("Way too long a note", out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("Way too", out); // 7 chars + NUL
}

// Test-only fake log store: in-memory, append-order.
class InMemoryLogStore : public IMaintenanceLogStore {
  public:
    MaintenanceLogEntry entries[10];
    size_t count = 0;

    bool append(const MaintenanceLogEntry& e) override {
        if (count >= 10) return false;
        entries[count++] = e;
        return true;
    }
    size_t readAll(MaintenanceLogEntry* out, size_t maxEntries) override {
        size_t n = count < maxEntries ? count : maxEntries;
        for (size_t i = 0; i < n; i++) out[i] = entries[i];
        return n;
    }
};

void test_maintenance_log_add_and_read_roundtrip() {
    InMemoryLogStore store;
    MaintenanceLog log(store);
    TEST_ASSERT_TRUE(log.addEntry(1700000000, 128, "Oil change"));
    TEST_ASSERT_TRUE(log.addEntry(0, 140, "Filter, replaced"));

    MaintenanceLogEntry out[10];
    size_t n = log.getEntries(out, 10);
    TEST_ASSERT_EQUAL(2, static_cast<int>(n));
    TEST_ASSERT_EQUAL_UINT32(1700000000, out[0].epochSeconds);
    TEST_ASSERT_EQUAL_UINT32(128, out[0].runHours);
    TEST_ASSERT_EQUAL_STRING("Oil change", out[0].note);
    TEST_ASSERT_EQUAL_UINT32(0, out[1].epochSeconds);
    TEST_ASSERT_EQUAL_STRING("Filter; replaced", out[1].note);
}


void test_engine_sources_always_forces_running() {
    EngineSourceDecision d = decide_engine_sources(ENGINE_SRC_ALWAYS, false);
    TEST_ASSERT_TRUE(d.forceRunning);
    TEST_ASSERT_FALSE(d.useAlternator);
}

void test_engine_sources_gps_only_follows_motion() {
    TEST_ASSERT_TRUE(decide_engine_sources(ENGINE_SRC_GPS, true).forceRunning);
    EngineSourceDecision still = decide_engine_sources(ENGINE_SRC_GPS, false);
    TEST_ASSERT_FALSE(still.forceRunning);
    TEST_ASSERT_FALSE(still.useAlternator);
}

void test_engine_sources_alternator_only_never_forces() {
    EngineSourceDecision d = decide_engine_sources(ENGINE_SRC_ALTERNATOR, true); // GPS motion ignored
    TEST_ASSERT_FALSE(d.forceRunning);
    TEST_ASSERT_TRUE(d.useAlternator);
}

void test_engine_sources_multicheck_combines() {
    uint8_t m = ENGINE_SRC_GPS | ENGINE_SRC_ALTERNATOR;
    EngineSourceDecision moving = decide_engine_sources(m, true);
    TEST_ASSERT_TRUE(moving.forceRunning);
    TEST_ASSERT_TRUE(moving.useAlternator);
    EngineSourceDecision still = decide_engine_sources(m, false);
    TEST_ASSERT_FALSE(still.forceRunning);
    TEST_ASSERT_TRUE(still.useAlternator);
}

void test_engine_sources_sanitize() {
    TEST_ASSERT_EQUAL_UINT8(ENGINE_SRC_DEFAULT, sanitize_engine_sources(0));
    TEST_ASSERT_EQUAL_UINT8(ENGINE_SRC_DEFAULT, sanitize_engine_sources(0xF8)); // only unknown bits
    TEST_ASSERT_EQUAL_UINT8(ENGINE_SRC_GPS, sanitize_engine_sources(ENGINE_SRC_GPS | 0x80));
}

void test_engine_mode_settings_default_and_persist() {
    InMemoryFlashStore store;
    EngineModeSettings a(store);
    a.begin();
    TEST_ASSERT_EQUAL_UINT8(ENGINE_SRC_DEFAULT, a.sources()); // first boot
    a.setSources(ENGINE_SRC_ALWAYS);
    TEST_ASSERT_EQUAL_INT(1, store.writeCalls);

    EngineModeSettings b(store);                // "reboot"
    b.begin();
    TEST_ASSERT_EQUAL_UINT8(ENGINE_SRC_ALWAYS, b.sources());
}

void test_engine_mode_settings_skips_redundant_and_invalid_writes() {
    InMemoryFlashStore store;
    EngineModeSettings s(store);
    s.begin();
    s.setSources(ENGINE_SRC_DEFAULT);           // unchanged -> no write
    TEST_ASSERT_EQUAL_INT(0, store.writeCalls);
    s.setSources(0);                            // invalid -> sanitized to default -> unchanged
    TEST_ASSERT_EQUAL_INT(0, store.writeCalls);
    TEST_ASSERT_EQUAL_UINT8(ENGINE_SRC_DEFAULT, s.sources());
}


void test_speed_average_over_window() {
    SpeedAverager avg(15000, 1000);
    avg.add(0.0, 0);
    avg.add(6.0, 1000);
    avg.add(9.0, 2000);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 5.0, avg.average(2000));
}

void test_speed_average_drops_samples_older_than_window() {
    SpeedAverager avg(15000, 1000);
    avg.add(30.0, 0);
    avg.add(0.0, 14000);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 15.0, avg.average(14500));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, avg.average(16000)); // the 30 km/h sample aged out
}

void test_speed_average_throttles_samples() {
    SpeedAverager avg(15000, 1000);
    avg.add(10.0, 0);
    avg.add(100.0, 200); // too soon, ignored
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 10.0, avg.average(300));
}

void test_speed_average_empty_and_clear() {
    SpeedAverager avg(15000, 1000);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, avg.average(1000));
    avg.add(8.0, 0);
    avg.clear();
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, avg.average(500));
}

void test_speed_average_survives_ring_overflow() {
    SpeedAverager avg(1000000, 1);
    for (int i = 0; i < 100; i++) avg.add(i < 68 ? 0.0 : 10.0, i * 10UL); // last 32 slots are all 10
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 10.0, avg.average(1000));
}

void test_pid_reset_discards_windup() {
    RealTimePid pid(30, 3, 1, 0, 1023);
    pid.compute(140, 120, 0);
    for (unsigned long t = 20; t <= 60000; t += 20) pid.compute(140, 120, t); // 60s deep below setpoint
    TEST_ASSERT_EQUAL_DOUBLE(1023, pid.compute(140, 120, 60020));            // saturated

    pid.reset();                                                             // left the regulation band
    TEST_ASSERT_EQUAL_DOUBLE(0, pid.compute(140, 130, 70000));               // first call after reset
    double out = pid.compute(140, 130, 70020);
    TEST_ASSERT_TRUE(out < 400);                                             // P-term only, no wound-up integral
}

void test_boot_blink_pattern() {
    // 3 blinks, 200ms on / 200ms off
    TEST_ASSERT_TRUE(boot_blink_on(0, 200, 3));
    TEST_ASSERT_TRUE(boot_blink_on(199, 200, 3));
    TEST_ASSERT_FALSE(boot_blink_on(200, 200, 3));
    TEST_ASSERT_TRUE(boot_blink_on(400, 200, 3));   // second blink
    TEST_ASSERT_TRUE(boot_blink_on(800, 200, 3));   // third blink
    TEST_ASSERT_FALSE(boot_blink_on(1000, 200, 3));
}

void test_boot_blink_ends_after_last_blink() {
    TEST_ASSERT_TRUE(boot_blink_active(1199, 200, 3));
    TEST_ASSERT_FALSE(boot_blink_active(1200, 200, 3));
    TEST_ASSERT_FALSE(boot_blink_on(1200, 200, 3));   // would be "on" in a 4th cycle, but pattern is over
    TEST_ASSERT_FALSE(boot_blink_on(1600, 200, 3));
}

void test_led_fault_none_when_everything_is_normal() {
    TEST_ASSERT_EQUAL(static_cast<int>(LedFault::NONE), static_cast<int>(decide_led_fault(13.8f, false, 30000)));
}

void test_led_fault_priority() {
    TEST_ASSERT_EQUAL(static_cast<int>(LedFault::SENSOR_RANGE), static_cast<int>(decide_led_fault(0.0f, true, 1000)));  // sensor beats the rest
    TEST_ASSERT_EQUAL(static_cast<int>(LedFault::SENSOR_RANGE), static_cast<int>(decide_led_fault(18.0f, false, 30000)));
    TEST_ASSERT_EQUAL(static_cast<int>(LedFault::OVERVOLTAGE), static_cast<int>(decide_led_fault(14.6f, true, 1000)));  // then overvoltage
    TEST_ASSERT_EQUAL(static_cast<int>(LedFault::LOW_HEAP), static_cast<int>(decide_led_fault(13.8f, false, 5000)));
    TEST_ASSERT_EQUAL(static_cast<int>(LedFault::NONE), static_cast<int>(decide_led_fault(12.4f, false, 6000))); // boundaries are fine
}

void test_fault_blink_pattern_pulses_then_pause() {
    // 3 pulses, 120ms half period, 1000ms pause: pulses occupy 0..720, dark until 1720
    TEST_ASSERT_TRUE(fault_blink_on(0, 3, 120, 1000));
    TEST_ASSERT_FALSE(fault_blink_on(120, 3, 120, 1000));
    TEST_ASSERT_TRUE(fault_blink_on(240, 3, 120, 1000));
    TEST_ASSERT_TRUE(fault_blink_on(480, 3, 120, 1000));   // third pulse
    TEST_ASSERT_FALSE(fault_blink_on(600, 3, 120, 1000));
    TEST_ASSERT_FALSE(fault_blink_on(1000, 3, 120, 1000)); // pause
    TEST_ASSERT_TRUE(fault_blink_on(1720, 3, 120, 1000));  // repeats
}

void test_fault_blink_zero_pulses_stays_dark() {
    TEST_ASSERT_FALSE(fault_blink_on(0, 0, 120, 1000));
}

void test_fault_blink_shows_code_a_couple_of_times_then_stops() {
    // 3 pulses (720ms) + 1000ms pause, 2 repeats: active until 720+1000+720 = 2440
    TEST_ASSERT_TRUE(fault_blink_active(0, 3, 120, 1000, 2));
    TEST_ASSERT_TRUE(fault_blink_active(2439, 3, 120, 1000, 2));
    TEST_ASSERT_FALSE(fault_blink_active(2440, 3, 120, 1000, 2));
    TEST_ASSERT_TRUE(fault_blink_on(1720, 3, 120, 1000));            // second showing
    TEST_ASSERT_FALSE(fault_blink_on(1720 + 1000 + 10, 3, 120, 1000)); // would repeat, but only on within active window
}

void test_pid_hold_at_max_keeps_output_at_100_percent_on_band_entry() {
    RealTimePid pid(30, 3, 1, 0, 1023);
    pid.hold(1023, 130, 5000);                                  // was in full-charge (V < 13.0), forced to 100%
    TEST_ASSERT_EQUAL_DOUBLE(1023, pid.compute(140, 130, 5020)); // enters the band: no drop to 0, no ramp from scratch
    TEST_ASSERT_EQUAL_DOUBLE(1023, pid.compute(140, 132, 5040)); // still below setpoint -> stays at 100%
}

void test_pid_hold_after_long_gap_does_not_jump() {
    RealTimePid pid(30, 3, 1, 0, 1023);
    pid.compute(140, 130, 0);
    pid.hold(1023, 130, 600000);                                // 10 min in full charge without compute()
    double out = pid.compute(140, 130, 600020);                 // dt is 20ms, not 10 minutes
    TEST_ASSERT_TRUE(out > 1000);
}

void test_pid_hold_at_zero_after_cutoff_starts_from_nothing() {
    RealTimePid pid(30, 3, 1, 0, 1023);
    pid.hold(0, 146, 5000);                                     // cutoff (V >= 14.4): field off
    double out = pid.compute(140, 144, 5020);                   // back in band, still above setpoint
    TEST_ASSERT_EQUAL_DOUBLE(0, out);                           // no stale 100% integral -> no new overshoot
}

void test_voltage_scale_from_multimeter_reading() {
    float scale = 0;
    TEST_ASSERT_TRUE(compute_voltage_scale(13.8f, 14.0f, &scale));   // device says 14.00, multimeter 13.8
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.98571f, scale);
}

void test_voltage_scale_rejects_implausible_input() {
    float scale = 1.0f;
    TEST_ASSERT_FALSE(compute_voltage_scale(5.0f, 5.0f, &scale));    // reference outside 6-20V
    TEST_ASSERT_FALSE(compute_voltage_scale(21.0f, 20.0f, &scale));
    TEST_ASSERT_FALSE(compute_voltage_scale(13.8f, 0.5f, &scale));   // nothing read
    TEST_ASSERT_FALSE(compute_voltage_scale(10.0f, 14.0f, &scale));  // 29% off: wiring, not drift
    TEST_ASSERT_FALSE(compute_voltage_scale(18.0f, 12.0f, &scale));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, scale);                            // untouched on rejection
}

void test_voltage_calibration_persists_across_reboot() {
    InMemoryFlashStore store;
    VoltageCalibration a(store);
    a.begin();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, a.scale());
    TEST_ASSERT_TRUE(a.calibrate(13.8f, 14.0f));
    VoltageCalibration b(store);                                     // "reboot"
    b.begin();
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.98571f, b.scale());
}

void test_voltage_calibration_rejected_keeps_previous_and_does_not_write() {
    InMemoryFlashStore store;
    VoltageCalibration c(store);
    c.begin();
    c.calibrate(13.8f, 14.0f);
    int writes = store.writeCalls;
    TEST_ASSERT_FALSE(c.calibrate(3.0f, 14.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.98571f, c.scale());
    TEST_ASSERT_EQUAL_INT(writes, store.writeCalls);
}

void test_voltage_calibration_reset() {
    InMemoryFlashStore store;
    VoltageCalibration c(store);
    c.begin();
    c.reset();                                                       // already 1.0: no write
    TEST_ASSERT_EQUAL_INT(0, store.writeCalls);
    c.calibrate(13.8f, 14.0f);
    c.reset();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, c.scale());
    VoltageCalibration d(store);
    d.begin();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, d.scale());
}

void test_pid_exposes_current_tunings() {
    RealTimePid pid(30, 3, 1, 0, 1023);
    TEST_ASSERT_EQUAL_DOUBLE(30, pid.kp());
    pid.setTunings(12.5, 0.4, 2.0);                                  // e.g. after autotune
    TEST_ASSERT_EQUAL_DOUBLE(12.5, pid.kp());
    TEST_ASSERT_EQUAL_DOUBLE(0.4, pid.ki());
    TEST_ASSERT_EQUAL_DOUBLE(2.0, pid.kd());
}

void test_deadline_pending_until_reached() {
    TEST_ASSERT_TRUE(is_before_deadline(1000, 5000));
    TEST_ASSERT_FALSE(is_before_deadline(5000, 5000));   // reached
    TEST_ASSERT_FALSE(is_before_deadline(6000, 5000));   // passed
}

void test_deadline_handles_millis_rollover() {
    unsigned long deadline = 10;                         // wrapped past ULONG_MAX
    TEST_ASSERT_TRUE(is_before_deadline(static_cast<unsigned long>(-5), deadline));
    TEST_ASSERT_FALSE(is_before_deadline(20, deadline));
}

void test_calibrate_from_shown_uses_what_the_ui_displayed() {
    InMemoryFlashStore store;
    VoltageCalibration c(store);
    c.begin();
    TEST_ASSERT_TRUE(c.calibrateFromShown(13.8f, 14.0f));            // UI said 14.00 (scale 1.0), meter 13.8
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.98571f, c.scale());
    // Now the UI (already scaled) shows 13.9 while the meter says 13.8: the correction is relative to what was SHOWN.
    TEST_ASSERT_TRUE(c.calibrateFromShown(13.8f, 13.9f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.98571f * 13.8f / 13.9f, c.scale());
}

void test_pid_settings_factory_when_nothing_saved() {
    InMemoryFlashStore store;
    PidSettings ps(store, PidGains{30, 3, 1});
    ps.begin();
    TEST_ASSERT_FALSE(ps.hasCustom());
    TEST_ASSERT_EQUAL_DOUBLE(30, ps.gains().kp);
    TEST_ASSERT_EQUAL_DOUBLE(3, ps.gains().ki);
    TEST_ASSERT_EQUAL_DOUBLE(1, ps.gains().kd);
    TEST_ASSERT_EQUAL_INT(0, store.writeCalls);
}

void test_pid_settings_custom_survives_reboot() {
    InMemoryFlashStore store;
    PidSettings a(store, PidGains{30, 3, 1});
    a.begin();
    TEST_ASSERT_TRUE(a.save(PidGains{12.5, 0.75, 2.25}));            // e.g. autotune result
    PidSettings b(store, PidGains{30, 3, 1});                        // "reboot"
    b.begin();
    TEST_ASSERT_TRUE(b.hasCustom());
    TEST_ASSERT_EQUAL_DOUBLE(12.5, b.gains().kp);
    TEST_ASSERT_EQUAL_DOUBLE(0.75, b.gains().ki);
    TEST_ASSERT_EQUAL_DOUBLE(2.25, b.gains().kd);
}

void test_pid_settings_rejects_invalid_gains() {
    InMemoryFlashStore store;
    PidSettings ps(store, PidGains{30, 3, 1});
    ps.begin();
    double nan = 0.0 / 0.0;
    TEST_ASSERT_FALSE(ps.save(PidGains{nan, 1, 1}));
    TEST_ASSERT_FALSE(ps.save(PidGains{-1, 1, 1}));
    TEST_ASSERT_FALSE(ps.save(PidGains{0, 1, 1}));                   // kp must be > 0
    TEST_ASSERT_FALSE(ps.save(PidGains{1e9, 1, 1}));
    TEST_ASSERT_FALSE(ps.hasCustom());
    TEST_ASSERT_EQUAL_DOUBLE(30, ps.gains().kp);
    TEST_ASSERT_EQUAL_INT(0, store.writeCalls);
}

void test_pid_settings_reset_returns_to_factory_and_persists() {
    InMemoryFlashStore store;
    PidSettings a(store, PidGains{30, 3, 1});
    a.begin();
    a.save(PidGains{12.5, 0.75, 2.25});
    a.resetToFactory();
    TEST_ASSERT_FALSE(a.hasCustom());
    TEST_ASSERT_EQUAL_DOUBLE(30, a.gains().kp);
    PidSettings b(store, PidGains{30, 3, 1});
    b.begin();
    TEST_ASSERT_FALSE(b.hasCustom());                                // stays factory after reboot
    TEST_ASSERT_EQUAL_DOUBLE(30, b.gains().kp);
}

void test_pid_settings_corrupt_blob_falls_back_to_factory() {
    InMemoryFlashStore store;
    PidSettings a(store, PidGains{30, 3, 1});
    a.begin();
    a.save(PidGains{12.5, 0.75, 2.25});
    store.buffer[4] = 0xFF; store.buffer[5] = 0xFF; store.buffer[6] = 0xFF; store.buffer[7] = 0x7F; // kp -> NaN/garbage
    PidSettings b(store, PidGains{30, 3, 1});
    b.begin();
    TEST_ASSERT_FALSE(b.hasCustom());
    TEST_ASSERT_EQUAL_DOUBLE(30, b.gains().kp);
}

void test_utc_to_epoch() {
    TEST_ASSERT_EQUAL_UINT32(1790944496UL, utc_to_epoch(2026, 10, 2, 12, 34, 56));
    TEST_ASSERT_EQUAL_UINT32(0UL, utc_to_epoch(1970, 1, 1, 0, 0, 0));
    TEST_ASSERT_EQUAL_UINT32(1709251199UL, utc_to_epoch(2024, 2, 29, 23, 59, 59));      // leap day
    TEST_ASSERT_EQUAL_UINT32(951868800UL, utc_to_epoch(2000, 3, 1, 0, 0, 0));
}

void test_epoch_to_iso8601_roundtrip() {
    char buf[32];
    epoch_to_iso8601(1790944496UL, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("2026-10-02T12:34:56Z", buf);
    epoch_to_iso8601(1709251199UL, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("2024-02-29T23:59:59Z", buf);
}

void test_gpx_point_with_and_without_time() {
    char buf[160];
    size_t n = gpx_format_point(buf, sizeof(buf), TrackPoint{41123456, -2654321, 1790944496UL});
    TEST_ASSERT_EQUAL_STRING("<trkpt lat=\"41.123456\" lon=\"-2.654321\"><time>2026-10-02T12:34:56Z</time></trkpt>\n", buf);
    TEST_ASSERT_EQUAL(static_cast<int>(strlen(buf)), static_cast<int>(n));
    gpx_format_point(buf, sizeof(buf), TrackPoint{-500000, 7, 0});
    TEST_ASSERT_EQUAL_STRING("<trkpt lat=\"-0.500000\" lon=\"0.000007\"></trkpt>\n", buf);
}

void test_gpx_point_reports_when_it_does_not_fit() {
    char tiny[20];
    TEST_ASSERT_EQUAL(0, static_cast<int>(gpx_format_point(tiny, sizeof(tiny), TrackPoint{1, 2, 3})));
}

void test_gpx_document_is_well_formed_xml_wrapper() {
    TEST_ASSERT_NOT_NULL(strstr(kGpxHeader, "<gpx"));
    TEST_ASSERT_NOT_NULL(strstr(kGpxHeader, "<trkseg>"));
    TEST_ASSERT_EQUAL_STRING("</trkseg></trk></gpx>\n", kGpxFooter);
    TEST_ASSERT_EQUAL_STRING("</trkseg>\n<trkseg>\n", kGpxSegmentBreak);
}

class InMemoryTrackStore : public ITrackStore {
  public:
    std::vector<TrackPoint> pts;
    int appendCalls = 0;
    size_t count() override { return pts.size(); }
    bool append(const TrackPoint* p, size_t n) override { appendCalls++; pts.insert(pts.end(), p, p + n); return true; }
    size_t read(size_t first, TrackPoint* out, size_t maxPoints) override {
        size_t n = 0;
        for (size_t i = first; i < pts.size() && n < maxPoints; i++) out[n++] = pts[i];
        return n;
    }
    void clear() override { pts.clear(); }
};

// All points of the unified (stored + pending + head) view.
std::vector<TrackPoint> track_all(const TrackRecorder& r) {
    std::vector<TrackPoint> all(r.size());
    if (!all.empty()) r.read(0, all.data(), all.size());
    return all;
}

// 0.00001 degrees of latitude is 1.1119 m.
const double kStepDeg = 0.00001;

void test_cross_track_distance() {
    TrackPoint a{0, 0, 0}, b{1000, 0, 0}, p{500, 100, 0};            // line north, p 100e-6 deg (11.1m) east
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 11.12, track_cross_track_meters(a, b, p));
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 11.12, track_cross_track_meters(a, a, TrackPoint{0, 100, 0})); // degenerate: distance to a
}

void test_turn_angle() {
    TrackPoint a{0, 0, 0}, b{1000, 0, 0};
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, track_turn_angle_degrees(a, b, TrackPoint{2000, 0, 0}));      // straight on
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 90.0, track_turn_angle_degrees(a, b, TrackPoint{1000, 1000, 0}));   // right angle (equator: 1:1)
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 180.0, track_turn_angle_degrees(a, b, a));                          // reversal
}

void test_recorder_straight_pass_costs_a_few_points() {
    InMemoryTrackStore store;
    TrackRecorder r(store);
    r.begin();
    for (int i = 0; i <= 100; i++) r.add(41.0 + i * 3 * kStepDeg / 1.1119 * 1.1119, 2.0, 0); // ~300 m north, 3.3 m steps
    auto pts = track_all(r);
    TEST_ASSERT_TRUE(pts.size() <= 10);                              // break + anchor + ~5 segments + head
    TEST_ASSERT_TRUE(pts.size() >= 4);
    TEST_ASSERT_DOUBLE_WITHIN(8.0, 333.0, r.distanceMeters());       // true length, not the simplified one
}

void test_recorder_keeps_the_corner() {
    InMemoryTrackStore store;
    TrackRecorder r(store);
    r.begin();
    double lat = 41.0, lon = 2.0;
    for (int i = 0; i < 40; i++) { r.add(lat, lon, 0); lat += 3 * kStepDeg; }          // ~133 m north
    double cornerLat = lat;
    for (int i = 0; i < 40; i++) { lon += 3 * kStepDeg / 0.7547; r.add(lat, lon, 0); } // ~133 m east
    bool nearCorner = false;
    for (const auto& p : track_all(r))
        if (!is_segment_break(p) && track_distance_meters(p.latE6, p.lonE6, static_cast<int32_t>(cornerLat * 1e6), 2000000) < 6.0) nearCorner = true;
    TEST_ASSERT_TRUE(nearCorner);                                    // the 90 degree turn survives simplification
}

void test_recorder_stores_a_u_turn_in_detail() {
    InMemoryTrackStore store;
    TrackRecorder r(store);
    r.begin();
    const double radiusM = 15.0, pi = 3.14159265358979;
    for (int i = 0; i <= 30; i++) {                                  // semicircle, ~3 m per step
        double ang = pi * i / 30.0;
        r.add(41.0 + radiusM * sin(ang) / 111195.0, 2.0 + radiusM * (1 - cos(ang)) / (111195.0 * 0.7547), 0);
    }
    TEST_ASSERT_TRUE(track_all(r).size() >= 7);                      // a straight pass of the same length is <= 4
}

void test_recorder_ignores_gps_jitter_when_parked() {
    InMemoryTrackStore store;
    TrackRecorder r(store);
    r.begin();
    r.add(41.0, 2.0, 0);
    for (int i = 0; i < 50; i++) r.add(41.0 + (i % 2) * 0.000012, 2.0, 0);  // +-1.3 m wobble
    TEST_ASSERT_EQUAL(2, static_cast<int>(r.size()));                // break + first point only
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, r.distanceMeters());
}

void test_recorder_flushes_in_batches() {
    InMemoryTrackStore store;
    TrackConfig cfg; cfg.flushEvery = 4;
    TrackRecorder r(store, cfg);
    r.begin();
    for (int i = 0; i < 40; i++) r.add(41.0 + i * 0.0004, 2.0 + (i % 2) * 0.0004, 0);   // zig-zag: every point is a corner
    TEST_ASSERT_TRUE(store.appendCalls >= 5);
    TEST_ASSERT_TRUE(store.appendCalls < 40);                        // batched, not one flash write per point
    size_t viewSize = r.size();
    r.flush();
    TEST_ASSERT_EQUAL(static_cast<int>(viewSize), static_cast<int>(store.pts.size() + 1)); // + the head (not stored until it becomes a corner)
}

void test_recorder_session_break_after_reboot() {
    InMemoryTrackStore store;
    {
        TrackRecorder first(store);
        first.begin();
        for (int i = 0; i < 20; i++) first.add(41.0 + i * 0.0004, 2.0, 0);
        first.flush();
    }
    TrackRecorder second(store);                                     // "reboot": same flash, new object
    second.begin();
    size_t before = second.size();
    TEST_ASSERT_TRUE(before > 2);                                    // previous session still there
    TEST_ASSERT_TRUE(second.distanceMeters() > 500.0);                // recomputed from flash
    second.add(41.5, 2.5, 0);                                        // far away: tomorrow, other field
    auto pts = track_all(second);
    int breaks = 0;
    for (const auto& p : pts) breaks += is_segment_break(p);
    TEST_ASSERT_EQUAL(2, breaks);                                    // one per session
    TEST_ASSERT_TRUE(is_segment_break(pts[0]));
}

void test_recorder_respects_max_points_but_keeps_measuring() {
    InMemoryTrackStore store;
    TrackConfig cfg; cfg.maxPoints = 10; cfg.flushEvery = 2;
    TrackRecorder r(store, cfg);
    r.begin();
    for (int i = 0; i < 200; i++) r.add(41.0 + i * 0.0004, 2.0 + (i % 2) * 0.0004, 0);
    r.flush();
    TEST_ASSERT_TRUE(r.full());
    TEST_ASSERT_TRUE(store.pts.size() <= 10);
    TEST_ASSERT_TRUE(r.distanceMeters() > 5000.0);                   // distance keeps counting
}

void test_recorder_clear_starts_fresh() {
    InMemoryTrackStore store;
    TrackRecorder r(store);
    r.begin();
    for (int i = 0; i < 20; i++) r.add(41.0 + i * 0.0004, 2.0, 0);
    r.flush();
    r.clear();
    TEST_ASSERT_EQUAL(0, static_cast<int>(r.size()));
    TEST_ASSERT_EQUAL(0, static_cast<int>(store.pts.size()));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, r.distanceMeters());
    r.add(41.0, 2.0, 0);
    TEST_ASSERT_TRUE(is_segment_break(track_all(r)[0]));             // fresh break after a clear
}

void test_recorder_read_is_a_consistent_window() {
    InMemoryTrackStore store;
    TrackConfig cfg; cfg.flushEvery = 4;
    TrackRecorder r(store, cfg);
    r.begin();
    for (int i = 0; i < 30; i++) r.add(41.0 + i * 0.0004, 2.0 + (i % 2) * 0.0004, 0);
    auto all = track_all(r);
    TrackPoint window[5];
    size_t n = r.read(3, window, 5);
    TEST_ASSERT_EQUAL(5, static_cast<int>(n));
    for (size_t i = 0; i < n; i++) TEST_ASSERT_EQUAL_INT32(all[3 + i].latE6, window[i].latE6);
    TEST_ASSERT_EQUAL(0, static_cast<int>(r.read(all.size(), window, 5)));   // past the end
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_cross_track_distance);
    RUN_TEST(test_turn_angle);
    RUN_TEST(test_recorder_straight_pass_costs_a_few_points);
    RUN_TEST(test_recorder_keeps_the_corner);
    RUN_TEST(test_recorder_stores_a_u_turn_in_detail);
    RUN_TEST(test_recorder_ignores_gps_jitter_when_parked);
    RUN_TEST(test_recorder_flushes_in_batches);
    RUN_TEST(test_recorder_session_break_after_reboot);
    RUN_TEST(test_recorder_respects_max_points_but_keeps_measuring);
    RUN_TEST(test_recorder_clear_starts_fresh);
    RUN_TEST(test_recorder_read_is_a_consistent_window);
    RUN_TEST(test_utc_to_epoch);
    RUN_TEST(test_epoch_to_iso8601_roundtrip);
    RUN_TEST(test_gpx_point_with_and_without_time);
    RUN_TEST(test_gpx_point_reports_when_it_does_not_fit);
    RUN_TEST(test_gpx_document_is_well_formed_xml_wrapper);
    RUN_TEST(test_pid_settings_factory_when_nothing_saved);
    RUN_TEST(test_pid_settings_custom_survives_reboot);
    RUN_TEST(test_pid_settings_rejects_invalid_gains);
    RUN_TEST(test_pid_settings_reset_returns_to_factory_and_persists);
    RUN_TEST(test_pid_settings_corrupt_blob_falls_back_to_factory);
    RUN_TEST(test_deadline_pending_until_reached);
    RUN_TEST(test_deadline_handles_millis_rollover);
    RUN_TEST(test_calibrate_from_shown_uses_what_the_ui_displayed);
    RUN_TEST(test_pid_exposes_current_tunings);
    RUN_TEST(test_voltage_scale_from_multimeter_reading);
    RUN_TEST(test_voltage_scale_rejects_implausible_input);
    RUN_TEST(test_voltage_calibration_persists_across_reboot);
    RUN_TEST(test_voltage_calibration_rejected_keeps_previous_and_does_not_write);
    RUN_TEST(test_voltage_calibration_reset);
    RUN_TEST(test_pid_hold_at_max_keeps_output_at_100_percent_on_band_entry);
    RUN_TEST(test_pid_hold_after_long_gap_does_not_jump);
    RUN_TEST(test_pid_hold_at_zero_after_cutoff_starts_from_nothing);
    RUN_TEST(test_fault_blink_shows_code_a_couple_of_times_then_stops);
    RUN_TEST(test_led_fault_none_when_everything_is_normal);
    RUN_TEST(test_led_fault_priority);
    RUN_TEST(test_fault_blink_pattern_pulses_then_pause);
    RUN_TEST(test_fault_blink_zero_pulses_stays_dark);
    RUN_TEST(test_boot_blink_pattern);
    RUN_TEST(test_boot_blink_ends_after_last_blink);
    RUN_TEST(test_pid_reset_discards_windup);
    RUN_TEST(test_speed_average_over_window);
    RUN_TEST(test_speed_average_drops_samples_older_than_window);
    RUN_TEST(test_speed_average_throttles_samples);
    RUN_TEST(test_speed_average_empty_and_clear);
    RUN_TEST(test_speed_average_survives_ring_overflow);
    RUN_TEST(test_engine_sources_always_forces_running);
    RUN_TEST(test_engine_sources_gps_only_follows_motion);
    RUN_TEST(test_engine_sources_alternator_only_never_forces);
    RUN_TEST(test_engine_sources_multicheck_combines);
    RUN_TEST(test_engine_sources_sanitize);
    RUN_TEST(test_engine_mode_settings_default_and_persist);
    RUN_TEST(test_engine_mode_settings_skips_redundant_and_invalid_writes);
    RUN_TEST(test_voltage_calibration);
    RUN_TEST(test_pwm_safety_thresholds);
    RUN_TEST(test_relay_overvoltage_turns_off_and_starts_delay);
    RUN_TEST(test_relay_stays_off_during_delay_window);
    RUN_TEST(test_relay_turns_on_below_low_threshold_after_delay);
    RUN_TEST(test_gps_motion_means_engine_running);
    RUN_TEST(test_gps_standstill_noise_is_not_motion);
    RUN_TEST(test_gps_motion_ignored_without_fresh_fix);
    RUN_TEST(test_should_run_cycle_respects_interval);
    RUN_TEST(test_should_run_cycle_handles_millis_rollover);
    RUN_TEST(test_autotune_computes_gains_from_relay_oscillation);
    RUN_TEST(test_autotune_fails_if_no_oscillation_within_runtime);
    RUN_TEST(test_autotune_fails_on_zero_amplitude);
    RUN_TEST(test_autotune_update_after_completion_is_a_no_op);
    RUN_TEST(test_engine_detector_starts_with_a_pulse);
    RUN_TEST(test_engine_detector_confirms_engine_running_on_voltage_rise);
    RUN_TEST(test_engine_detector_backs_off_after_a_dry_pulse);
    RUN_TEST(test_engine_detector_waits_full_cooldown_before_next_pulse);
    RUN_TEST(test_engine_detector_reset_clears_stale_state);
    RUN_TEST(test_engine_detector_grace_keeps_running_through_voltage_sag);
    RUN_TEST(test_engine_detector_grace_expires_into_probe);
    RUN_TEST(test_engine_detector_reset_drops_grace);
    RUN_TEST(test_engine_detector_small_rise_is_enough_during_pulse);
    RUN_TEST(test_engine_detector_cooldown_ignores_voltage_until_it_elapses);
    RUN_TEST(test_median_reading_rejects_spike);
    RUN_TEST(test_mirror_duty_follows_output_exactly);
    RUN_TEST(test_mirror_duty_inverts_for_active_low_led);
    RUN_TEST(test_mirror_duty_clamps_overrange);
    RUN_TEST(test_usage_counters_first_boot_uses_defaults);
    RUN_TEST(test_usage_counters_accumulate_only_while_engine_active);
    RUN_TEST(test_usage_counters_count_with_fast_loop_ticks);
    RUN_TEST(test_usage_counters_idle_time_not_carried_into_next_run);
    RUN_TEST(test_usage_counters_throttles_writes);
    RUN_TEST(test_usage_counters_reset_and_reload_across_reboot);
    RUN_TEST(test_usage_counters_maintenance_due);
    RUN_TEST(test_usage_counters_boot_count_increments_each_boot);
    RUN_TEST(test_mqtt_buffers_when_network_unavailable);
    RUN_TEST(test_mqtt_throttles_reconnect_attempts);
    RUN_TEST(test_mqtt_flushes_buffer_and_publishes_on_reconnect);
    RUN_TEST(test_mqtt_buffer_drops_oldest_when_full);
    RUN_TEST(test_mqtt_reconnect_backs_off_while_broker_is_unreachable);
    RUN_TEST(test_mqtt_backoff_is_capped_and_resets_on_success);
    RUN_TEST(test_mqtt_state_topic_is_latest_reading_even_with_backlog);
    RUN_TEST(test_mqtt_history_topic_drains_oldest_first_in_capped_batches);
    RUN_TEST(test_mqtt_state_not_republished_without_new_sample);
    RUN_TEST(test_mqtt_gps_topic_only_with_fix_and_carries_position);
    RUN_TEST(test_mqtt_readings_carry_epoch_when_known);
    RUN_TEST(test_mqtt_discovery_entities_read_the_state_object);
    RUN_TEST(test_mqtt_worst_case_batch_fits_transport_buffer);
    RUN_TEST(test_gps_jump_filter_accepts_first_fix);
    RUN_TEST(test_gps_jump_filter_accepts_plausible_movement);
    RUN_TEST(test_gps_jump_filter_rejects_impossible_jump);
    RUN_TEST(test_gps_jump_filter_resyncs_after_consecutive_rejects);
    RUN_TEST(test_sanitize_note_escapes_commas_and_newlines);
    RUN_TEST(test_sanitize_note_truncates_to_buffer);
    RUN_TEST(test_maintenance_log_add_and_read_roundtrip);
    return UNITY_END();
}
