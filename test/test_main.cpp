#include <unity.h>
#include "../src/charging/VoltageSensor.h"
#include "../src/charging/AlternatorLogic.h"
#include "../src/charging/PidAutotuner.h"
#include "../src/charging/EngineDetector.h"
#include "../src/charging/PwmMirror.h"
#include "../src/charging/RealTimePid.h"
#include "../src/charging/EngineMode.h"
#include "../src/storage/UsageCounters.h"
#include "../src/connectivity/MqttPublisher.h"
#include "../src/storage/MaintenanceLog.h"
#include "../src/gps/GpsJumpFilter.h"
#include "../src/gps/SpeedAverager.h"
#include <string.h>

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

// Test-only fake MQTT transport: no real network.
class FakeMqttTransport : public IMqttTransport {
  public:
    bool connectedState = false;
    bool connectResult = true;
    int connectCalls = 0;
    int publishCalls = 0;

    bool connected() override { return connectedState; }
    bool connect() override {
        connectCalls++;
        connectedState = connectResult;
        return connectResult;
    }
    bool publish(const char*, const char*, bool) override {
        publishCalls++;
        return true;
    }
    void loop() override {}
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
    TEST_ASSERT_EQUAL(7, t.publishCalls); // 6 discovery configs + 1 batch publish

    pub.recordSample(make_reading(200));
    pub.update(true, 200); // already connected, discovery not repeated
    TEST_ASSERT_EQUAL(2, pub.publishCount());
    TEST_ASSERT_EQUAL(8, t.publishCalls);
}

void test_mqtt_buffer_drops_oldest_when_full() {
    FakeMqttTransport t;
    MqttPublisher pub(t, 2, 30000); // capacity 2
    pub.recordSample(make_reading(1));
    pub.recordSample(make_reading(2));
    pub.recordSample(make_reading(3)); // drops reading #1
    TEST_ASSERT_EQUAL(2, static_cast<int>(pub.bufferedCount()));
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

int main(int argc, char **argv) {
    UNITY_BEGIN();
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
    RUN_TEST(test_gps_jump_filter_accepts_first_fix);
    RUN_TEST(test_gps_jump_filter_accepts_plausible_movement);
    RUN_TEST(test_gps_jump_filter_rejects_impossible_jump);
    RUN_TEST(test_gps_jump_filter_resyncs_after_consecutive_rejects);
    RUN_TEST(test_sanitize_note_escapes_commas_and_newlines);
    RUN_TEST(test_sanitize_note_truncates_to_buffer);
    RUN_TEST(test_maintenance_log_add_and_read_roundtrip);
    return UNITY_END();
}
