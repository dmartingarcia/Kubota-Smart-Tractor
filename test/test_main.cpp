#include <Arduino.h>
#include <unity.h>
#include "../src/main.cpp"
#include "../src/WebHandler.h"

// Mock replacements
unsigned long fakeMillis = 0;
int mockAnalogRead = 0;
uint8_t relayPinState;

void setUp() {
    // Reset state before each test
    fakeMillis = 0;
    historyIndex = 0;
    relay_state = ALTERNATOR_ACTIVE_STATE;
    next_relay_check = 0;
    memset(history, 0, sizeof(history));
    relayPinState = ALTERNATOR_ACTIVE_STATE;
}

void tearDown() {
    // Clean up after each test
}

// Mock implementations
unsigned long millis() {
    return fakeMillis;
}

int analogRead(uint8_t) {
    return mockAnalogRead;
}

void digitalWrite(uint8_t pin, uint8_t val) {
    if(pin == RELAY_PIN) relayPinState = val;
}

void test_voltage_calculation() {
    // Test calibration calculation
    CALIBRATION_IN_VOLTAGE = 15.25;
    CALIBRATION_A0_VOLTAGE = 2.90;

    // Simulate 2.9V at A0 (15.25V real voltage)
    mockAnalogRead = (2.90 / 3.3) * 1023;
    TEST_ASSERT_FLOAT_WITHIN(0.1, 15.25, read_voltage());

    // Test lower voltage
    mockAnalogRead = (2.0 / 3.3) * 1023;
    float expected = (15.25 / 2.90) * 2.0;
    TEST_ASSERT_FLOAT_WITHIN(0.1, expected, read_voltage());
}

void test_relay_control() {
    VOLTAGE_THRESHOLD_HIGH = 14.8;
    VOLTAGE_THRESHOLD_LOW = 14.0;

    // Initial state
    TEST_ASSERT_EQUAL(ALTERNATOR_ACTIVE_STATE, relay_state);

    // Test over-voltage detection
    manage_relay(15.0);
    TEST_ASSERT_EQUAL(!ALTERNATOR_ACTIVE_STATE, relay_state);
    TEST_ASSERT_EQUAL(RELAY_ACTIVATION_DELAY, next_relay_check - fakeMillis);

    // Test hysteresis
    fakeMillis += RELAY_ACTIVATION_DELAY;
    manage_relay(14.5);  // Between thresholds
    TEST_ASSERT_EQUAL(!ALTERNATOR_ACTIVE_STATE, relay_state);
}

void test_data_storage() {
    // Test history buffer
    for(int i=0; i<130; i++) {
        fakeMillis = i * 500;
        store_data();
    }

    // Should wrap around after 120 entries
    TEST_ASSERT_EQUAL(10, historyIndex);
    TEST_ASSERT_EQUAL(129*500, history[9].timestamp);
}

void test_web_handlers() {
    // Initialize test data
    history[0] = {1000, 14.5, true};
    historyIndex = 1;
    relay_state = true;

    // Test /data endpoint
    handleData();
    TEST_ASSERT_TRUE(server.hasArg("voltage"));

    // Test /history endpoint
    handleHistory();
    TEST_ASSERT_TRUE(server.hasArg("timestamp"));
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_voltage_calculation);
    RUN_TEST(test_relay_control);
    RUN_TEST(test_data_storage);
    RUN_TEST(test_web_handlers);
    UNITY_END();
}

void loop() {
    // No loop logic needed for tests
    // In a real application, this would be the main loop
}