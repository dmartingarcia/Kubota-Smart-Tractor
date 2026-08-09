#include "MqttPublisher.h"
#include "../charging/AlternatorLogic.h"
#include <stdio.h>

namespace {
constexpr const char* kStateTopic = "kubotio/tractor/state";

// One reading as a JSON object, written at buf+offset. Returns the new offset, or
// the old offset unchanged if it didn't fit (caller stops appending in that case).
size_t appendReadingJson(char* buf, size_t bufSize, size_t offset, const MqttReading& r) {
  int written = snprintf(buf + offset, bufSize - offset,
           "{\"timestamp\":%lu,\"voltage\":%.2f,\"pwm\":%u,\"pwmPercent\":%u,\"active\":%s,"
           "\"engineRunning\":%s,\"runHours\":%.1f,\"maintenanceDue\":%s,\"freeHeap\":%lu,"
           "\"overvoltageAlert\":%s}",
           r.timestamp, r.voltage, r.pwmValue, r.pwmPercent, r.active ? "true" : "false",
           r.engineRunning ? "true" : "false", r.totalRunSeconds / 3600.0,
           r.maintenanceDue ? "true" : "false", static_cast<unsigned long>(r.freeHeap),
           r.overvoltageAlert ? "true" : "false");
  if (written < 0 || static_cast<size_t>(written) >= bufSize - offset) return offset; // didn't fit
  return offset + static_cast<size_t>(written);
}
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

void MqttPublisher::recordSample(const MqttReading& reading) {
  enqueue(reading);
}

void MqttPublisher::publishDiscovery() {
  transport_.publish("homeassistant/sensor/kubotio_voltage/config",
    "{\"name\":\"Tractor Battery Voltage\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json[-1].voltage }}\",\"unit_of_measurement\":\"V\","
    "\"unique_id\":\"kubotio_voltage\"}", true);
  transport_.publish("homeassistant/sensor/kubotio_pwm/config",
    "{\"name\":\"Tractor Charge PWM\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json[-1].pwmPercent }}\",\"unit_of_measurement\":\"%\","
    "\"unique_id\":\"kubotio_pwm\"}", true);
  transport_.publish("homeassistant/sensor/kubotio_run_hours/config",
    "{\"name\":\"Tractor Charging Hours\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json[-1].runHours }}\",\"unit_of_measurement\":\"h\","
    "\"unique_id\":\"kubotio_run_hours\"}", true);
  transport_.publish("homeassistant/binary_sensor/kubotio_maintenance_due/config",
    "{\"name\":\"Tractor Maintenance Due\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json[-1].maintenanceDue }}\",\"payload_on\":true,\"payload_off\":false,"
    "\"device_class\":\"problem\",\"unique_id\":\"kubotio_maintenance_due\"}", true);
  transport_.publish("homeassistant/sensor/kubotio_free_heap/config",
    "{\"name\":\"Tractor Free Heap\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json[-1].freeHeap }}\",\"unit_of_measurement\":\"B\","
    "\"entity_category\":\"diagnostic\",\"unique_id\":\"kubotio_free_heap\"}", true);
  transport_.publish("homeassistant/binary_sensor/kubotio_overvoltage/config",
    "{\"name\":\"Tractor Overvoltage Alert\",\"state_topic\":\"kubotio/tractor/state\","
    "\"value_template\":\"{{ value_json[-1].overvoltageAlert }}\",\"payload_on\":true,\"payload_off\":false,"
    "\"device_class\":\"safety\",\"unique_id\":\"kubotio_overvoltage\"}", true);
}

void MqttPublisher::publishBatch(unsigned long currentMillis) {
  size_t batchSize = bufferCount_ < kMaxPerBatch ? bufferCount_ : kMaxPerBatch;
  if (batchSize == 0) return;

  char payload[kMaxPerBatch * 200 + 8];
  size_t offset = 0;
  payload[offset++] = '[';
  for (size_t i = 0; i < batchSize; i++) {
    if (i > 0) payload[offset++] = ',';
    offset = appendReadingJson(payload, sizeof(payload), offset, buffer_[(bufferHead_ + i) % bufferCapacity_]);
  }
  payload[offset++] = ']';
  payload[offset] = '\0';

  if (transport_.publish(kStateTopic, payload, false)) {
    publishCount_ += static_cast<int>(batchSize);
    lastPublished_ = buffer_[(bufferHead_ + batchSize - 1) % bufferCapacity_];
    hasLastPublished_ = true;
    lastPublishMillis_ = currentMillis;
    bufferHead_ = (bufferHead_ + batchSize) % bufferCapacity_;
    bufferCount_ -= batchSize;
  }
}

void MqttPublisher::update(bool networkAvailable, unsigned long currentMillis) {
  // transport_.loop() (keepalive) is pumped directly by the caller every loop()
  // iteration now, independent of this slower publish cadence.
  if (!networkAvailable) return; // recordSample() already buffered whatever came in

  if (!transport_.connected()) {
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
  publishBatch(currentMillis);
}

size_t MqttPublisher::bufferedCount() const { return bufferCount_; }
int MqttPublisher::publishCount() const { return publishCount_; }
bool MqttPublisher::isConnected() { return transport_.connected(); }
void MqttPublisher::forceReconnectNow() { lastConnectAttempt_ = 0 - reconnectIntervalMs_; }
