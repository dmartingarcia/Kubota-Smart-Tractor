#include "MqttPublisher.h"
#include "../charging/AlternatorLogic.h"
#include <stdio.h>

namespace {
constexpr const char* kStateTopic = "kubotio/tractor/state";
}

MqttPublisher::MqttPublisher(IMqttTransport& transport, size_t bufferCapacity, unsigned long reconnectIntervalMs)
  : transport_(transport),
    bufferCapacity_(bufferCapacity > kMaxBuffered ? kMaxBuffered : bufferCapacity),
    reconnectIntervalMs_(reconnectIntervalMs),
    bufferHead_(0), bufferCount_(0),
    lastConnectAttempt_(0 - reconnectIntervalMs), // wraps so the first update() attempts a connect immediately
    discoveryPublished_(false), publishCount_(0), lastPublished_{},
    hasLastPublished_(false), lastPublishMillis_(0) {}

void MqttPublisher::enqueue(const MqttReading& r) {
  if (bufferCount_ < bufferCapacity_) {
    size_t idx = (bufferHead_ + bufferCount_) % bufferCapacity_;
    buffer_[idx] = r;
    bufferCount_++;
  } else {
    buffer_[bufferHead_] = r; // full: drop oldest
    bufferHead_ = (bufferHead_ + 1) % bufferCapacity_;
  }
}

void MqttPublisher::flushBuffer(unsigned long currentMillis) {
  while (bufferCount_ > 0) {
    publishReading(buffer_[bufferHead_], currentMillis);
    bufferHead_ = (bufferHead_ + 1) % bufferCapacity_;
    bufferCount_--;
  }
}

void MqttPublisher::publishReading(const MqttReading& r, unsigned long currentMillis) {
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"timestamp\":%lu,\"voltage\":%.2f,\"pwm\":%u,\"active\":%s,"
           "\"engineRunning\":%s,\"runHours\":%.1f,\"maintenanceDue\":%s,\"freeHeap\":%lu,"
           "\"overvoltageAlert\":%s}",
           r.timestamp, r.voltage, r.pwmValue, r.active ? "true" : "false",
           r.engineRunning ? "true" : "false", r.totalRunSeconds / 3600.0,
           r.maintenanceDue ? "true" : "false", static_cast<unsigned long>(r.freeHeap),
           r.overvoltageAlert ? "true" : "false");
  if (transport_.publish(kStateTopic, payload, false)) {
    publishCount_++;
    lastPublished_ = r;
    hasLastPublished_ = true;
    lastPublishMillis_ = currentMillis;
  }
}

void MqttPublisher::publishDiscovery() {
  transport_.publish("homeassistant/sensor/kubotio_voltage/config",
    "{\"name\":\"Tractor Battery Voltage\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json.voltage }}\",\"unit_of_measurement\":\"V\","
    "\"unique_id\":\"kubotio_voltage\"}", true);
  transport_.publish("homeassistant/sensor/kubotio_pwm/config",
    "{\"name\":\"Tractor Charge PWM\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json.pwm }}\",\"unit_of_measurement\":\"%\","
    "\"unique_id\":\"kubotio_pwm\"}", true);
  transport_.publish("homeassistant/sensor/kubotio_run_hours/config",
    "{\"name\":\"Tractor Charging Hours\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json.runHours }}\",\"unit_of_measurement\":\"h\","
    "\"unique_id\":\"kubotio_run_hours\"}", true);
  transport_.publish("homeassistant/binary_sensor/kubotio_maintenance_due/config",
    "{\"name\":\"Tractor Maintenance Due\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json.maintenanceDue }}\",\"payload_on\":true,\"payload_off\":false,"
    "\"device_class\":\"problem\",\"unique_id\":\"kubotio_maintenance_due\"}", true);
  transport_.publish("homeassistant/sensor/kubotio_free_heap/config",
    "{\"name\":\"Tractor Free Heap\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json.freeHeap }}\",\"unit_of_measurement\":\"B\","
    "\"entity_category\":\"diagnostic\",\"unique_id\":\"kubotio_free_heap\"}", true);
  transport_.publish("homeassistant/binary_sensor/kubotio_overvoltage/config",
    "{\"name\":\"Tractor Overvoltage Alert\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json.overvoltageAlert }}\",\"payload_on\":true,\"payload_off\":false,"
    "\"device_class\":\"safety\",\"unique_id\":\"kubotio_overvoltage\"}", true);
}

void MqttPublisher::update(bool networkAvailable, const MqttReading& latest, unsigned long currentMillis) {
  transport_.loop();

  if (!networkAvailable) {
    enqueue(latest);
    return;
  }

  if (!transport_.connected()) {
    enqueue(latest);
    if (should_run_cycle(currentMillis, lastConnectAttempt_, reconnectIntervalMs_)) {
      lastConnectAttempt_ = currentMillis;
      if (transport_.connect()) {
        discoveryPublished_ = false; // republish discovery after (re)connect
      }
    }
    return;
  }

  if (!discoveryPublished_) {
    publishDiscovery();
    discoveryPublished_ = true;
  }
  flushBuffer(currentMillis);
  publishReading(latest, currentMillis);
}

size_t MqttPublisher::bufferedCount() const { return bufferCount_; }
int MqttPublisher::publishCount() const { return publishCount_; }
bool MqttPublisher::isConnected() { return transport_.connected(); }
void MqttPublisher::forceReconnectNow() { lastConnectAttempt_ = 0 - reconnectIntervalMs_; }
