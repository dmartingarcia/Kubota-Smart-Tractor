#include "MqttPublisher.h"
#include "../charging/AlternatorLogic.h"
#include <stdio.h>

namespace {
constexpr const char* kStateTopic = "kubotio/tractor/state";
constexpr const char* kGpsTopic = "kubotio/tractor/gps";
constexpr const char* kHistoryTopic = "kubotio/tractor/history";
constexpr size_t kReadingSlot = 240; // worst-case reading JSON is ~210 chars

// One reading as a JSON object, written at buf+offset. Returns the new offset, or
// the old offset unchanged if it didn't fit (caller stops appending in that case).
size_t appendReadingJson(char* buf, size_t bufSize, size_t offset, const MqttReading& r) {
  int written = snprintf(buf + offset, bufSize - offset,
           "{\"timestamp\":%lu,\"voltage\":%.2f,\"pwm\":%u,\"pwmPercent\":%u,\"active\":%s,"
           "\"engineRunning\":%s,\"runHours\":%.1f,\"maintenanceDue\":%s,\"freeHeap\":%lu,"
           "\"overvoltageAlert\":%s,\"epoch\":%lu}",
           r.timestamp, r.voltage, r.pwmValue, r.pwmPercent, r.active ? "true" : "false",
           r.engineRunning ? "true" : "false", r.totalRunSeconds / 3600.0,
           r.maintenanceDue ? "true" : "false", static_cast<unsigned long>(r.freeHeap),
           r.overvoltageAlert ? "true" : "false", static_cast<unsigned long>(r.epochSeconds));
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
    hasLastPublished_(false), lastPublishMillis_(0), latest_{}, latestDirty_(false), live_{} {}

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
  latest_ = reading;
  latestDirty_ = true;
}

void MqttPublisher::publishDiscovery() {
  // expire_after: entities go "unavailable" if the state topic stops updating, instead of
  // showing the last retained value as if it were current.
  static const char* const kCommon = "\"state_topic\":\"kubotio/tractor/state\",\"expire_after\":180";
  char payload[320];
  auto sensor = [&](const char* id, const char* name, const char* field, const char* extra) {
    char topic[96];
    snprintf(topic, sizeof(topic), "homeassistant/sensor/kubotio_%s/config", id);
    snprintf(payload, sizeof(payload), "{\"name\":\"%s\",%s,\"value_template\":\"{{ value_json.%s }}\",\"unique_id\":\"kubotio_%s\"%s}",
             name, kCommon, field, id, extra);
    transport_.publish(topic, payload, true);
  };
  auto binary = [&](const char* id, const char* name, const char* field, const char* deviceClass) {
    char topic[96];
    snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/kubotio_%s/config", id);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"%s\",%s,\"value_template\":\"{{ value_json.%s }}\",\"payload_on\":true,\"payload_off\":false,"
             "\"device_class\":\"%s\",\"unique_id\":\"kubotio_%s\"}",
             name, kCommon, field, deviceClass, id);
    transport_.publish(topic, payload, true);
  };

  sensor("voltage", "Tractor Battery Voltage", "voltage", ",\"unit_of_measurement\":\"V\"");
  sensor("pwm", "Tractor Charge PWM", "pwmPercent", ",\"unit_of_measurement\":\"%\"");
  sensor("run_hours", "Tractor Charging Hours", "runHours", ",\"unit_of_measurement\":\"h\"");
  sensor("free_heap", "Tractor Free Heap", "freeHeap", ",\"unit_of_measurement\":\"B\",\"entity_category\":\"diagnostic\"");
  sensor("gps_speed", "Tractor Speed", "gpsSpeedKmh", ",\"unit_of_measurement\":\"km/h\"");
  binary("maintenance_due", "Tractor Maintenance Due", "maintenanceDue", "problem");
  binary("overvoltage", "Tractor Overvoltage Alert", "overvoltageAlert", "safety");
  binary("engine_running", "Tractor Engine Running", "engineRunning", "running");
  binary("charging_active", "Tractor Charging", "active", "battery_charging");
  binary("fault_active", "Tractor Fault", "fault > 0", "problem");
  sensor("fault", "Tractor Fault Code", "fault", "");
  transport_.publish("homeassistant/sensor/kubotio_fault_text/config",
    "{\"name\":\"Tractor Fault Reason\",\"state_topic\":\"kubotio/tractor/state\",\"expire_after\":180,"
    "\"value_template\":\"{{ {0:'none',2:'overvoltage',3:'voltage sensor range',4:'low memory'}[value_json.fault] | default('unknown') }}\","
    "\"unique_id\":\"kubotio_fault_text\"}", true);

  transport_.publish("homeassistant/device_tracker/kubotio_tractor/config",
    "{\"name\":\"Tractor\",\"json_attributes_topic\":\"kubotio/tractor/gps\",\"source_type\":\"gps\","
    "\"unique_id\":\"kubotio_tractor_tracker\"}", true);
}

void MqttPublisher::publishState(unsigned long currentMillis) {
  if (!latestDirty_) return; // nothing new since the last state publish
  const MqttReading& r = latest_;
  char payload[480];
  snprintf(payload, sizeof(payload),
           "{\"timestamp\":%lu,\"epoch\":%lu,\"voltage\":%.2f,\"pwm\":%u,\"pwmPercent\":%u,\"active\":%s,"
           "\"engineRunning\":%s,\"engineProbing\":%s,\"engineSources\":%u,\"fault\":%u,\"runHours\":%.1f,"
           "\"maintenanceDue\":%s,\"freeHeap\":%lu,\"overvoltageAlert\":%s,"
           "\"gpsFix\":%s,\"gpsSpeedKmh\":%.1f,\"gpsSpeedAvgKmh\":%.1f}",
           r.timestamp, static_cast<unsigned long>(r.epochSeconds), r.voltage, r.pwmValue, r.pwmPercent,
           r.active ? "true" : "false", r.engineRunning ? "true" : "false",
           live_.engineProbing ? "true" : "false", live_.engineSources, live_.fault, r.totalRunSeconds / 3600.0,
           r.maintenanceDue ? "true" : "false", static_cast<unsigned long>(r.freeHeap),
           r.overvoltageAlert ? "true" : "false", live_.gpsHasFix ? "true" : "false",
           live_.gpsHasFix ? live_.speedKmh : 0.0f, live_.gpsHasFix ? live_.speedAvgKmh : 0.0f);
  if (!transport_.publish(kStateTopic, payload, true)) return;

  if (live_.gpsHasFix) {
    snprintf(payload, sizeof(payload), "{\"latitude\":%.6f,\"longitude\":%.6f,\"gps_accuracy\":10}",
             live_.latitude, live_.longitude);
    transport_.publish(kGpsTopic, payload, true);
  }
  latestDirty_ = false;
  lastPublished_ = r;
  hasLastPublished_ = true;
  lastPublishMillis_ = currentMillis;
}

void MqttPublisher::publishBatch() {
  size_t batchSize = bufferCount_ < kMaxPerBatch ? bufferCount_ : kMaxPerBatch;
  if (batchSize == 0) return;

  char payload[kMaxPerBatch * kReadingSlot + 8];
  size_t offset = 0;
  payload[offset++] = '[';
  for (size_t i = 0; i < batchSize; i++) {
    if (i > 0) payload[offset++] = ',';
    offset = appendReadingJson(payload, sizeof(payload), offset, buffer_[(bufferHead_ + i) % bufferCapacity_]);
  }
  payload[offset++] = ']';
  payload[offset] = '\0';

  if (transport_.publish(kHistoryTopic, payload, false)) {
    publishCount_ += static_cast<int>(batchSize);
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
  publishState(currentMillis);
  publishBatch();
}

size_t MqttPublisher::bufferedCount() const { return bufferCount_; }
int MqttPublisher::publishCount() const { return publishCount_; }
bool MqttPublisher::isConnected() { return transport_.connected(); }
void MqttPublisher::forceReconnectNow() { lastConnectAttempt_ = 0 - reconnectIntervalMs_; }
