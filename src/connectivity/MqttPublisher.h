#ifndef MQTT_PUBLISHER_H
#define MQTT_PUBLISHER_H

#include <stddef.h>
#include <stdint.h>

// HAL boundary: no network/library includes here, so this is natively testable
// against a fake transport. See PubSubMqttTransport for the real implementation.
class IMqttTransport {
  public:
    virtual ~IMqttTransport() = default;
    virtual bool connected() = 0;
    virtual bool connect() = 0; // may block briefly (network timeout) - caller throttles calls
    virtual bool publish(const char* topic, const char* payload, bool retain) = 0;
    virtual void loop() = 0; // pump keepalive etc, must be cheap
};

struct MqttReading {
  unsigned long timestamp;
  float voltage;
  uint16_t pwmValue;    // raw 0-maxPWM count
  uint8_t pwmPercent;   // 0-100, for display (HA etc.) - pwmValue alone isn't a percentage
  bool active;
  bool engineRunning;
  uint32_t totalRunSeconds;
  bool maintenanceDue;
  uint32_t freeHeap;
  bool overvoltageAlert;
  uint32_t epochSeconds; // wall-clock time (NTP), 0 when unknown
};

// Latest-only extras for the state topic: not buffered (RAM), so they are never history.
struct MqttLiveState {
  bool gpsHasFix;
  double latitude;
  double longitude;
  float speedKmh;
  float speedAvgKmh;
  bool engineProbing;
  uint8_t engineSources;
  uint8_t fault; // LED fault code (StatusLed.h), 0 = none
};

// Publishes telemetry to MQTT/Home Assistant. Sampling and publishing are decoupled
// on purpose: recordSample() buffers a reading (call it often, e.g. every 10s, for
// good resolution even while offline), update() sends (call it less often, e.g. every 30s).
// Two channels per update():
//   kubotio/tractor/state   - retained, ONE JSON object: the newest reading plus live extras
//                             (GPS, engine state). What HA entities read, so after an outage
//                             they show the present immediately, not the backlog.
//   kubotio/tractor/gps     - retained lat/lon attributes for the HA device_tracker (fix only).
//   kubotio/tractor/history - JSON array, oldest first, up to kMaxPerBatch readings per call:
//                             store-and-forward of every sample (with epoch when known).
// Publishes HA MQTT discovery configs once per (re)connect.
// update() must stay cheap: reconnect attempts are throttled to reconnectIntervalMs,
// since IMqttTransport::connect() can block briefly on a real socket.
class MqttPublisher {
  public:
    // Buffered readings are stored packed (voltage in centivolts, heap in 4-byte units, flags in one
    // byte): 20 bytes instead of ~40, i.e. ~3.6KB less RAM for the 30-minute store-and-forward buffer.
    // What is published is identical (same 2-decimal voltage, same fields).
    struct Sample {
      uint32_t timestamp;
      uint32_t epoch;
      uint32_t totalRunSeconds;
      uint16_t voltageCv;
      uint16_t pwmValue;
      uint16_t freeHeap4;
      uint8_t pwmPercent;
      uint8_t flags;
    };
    static constexpr size_t kSampleBytes = sizeof(Sample);

    static constexpr size_t kMaxBuffered = 180; // matches the capacity main.cpp actually passes
    static constexpr size_t kMaxPerBatch = 8;  // bounds a single publish() payload's size

    MqttPublisher(IMqttTransport& transport, size_t bufferCapacity, unsigned long reconnectIntervalMs);

    void recordSample(const MqttReading& reading); // always buffers; call at the fast/sample cadence
    void setLiveState(const MqttLiveState& live) { live_ = live; } // latest GPS/engine extras for the state topic

    // Call at the slower publish cadence: connects if needed (throttled), and if
    // connected, publishes up to kMaxPerBatch buffered readings as one batch.
    void update(bool networkAvailable, unsigned long currentMillis);

    size_t bufferedCount() const;
    int publishCount() const; // total individual readings successfully published, for tests/diagnostics
    bool isConnected(); // wraps transport_.connected(), for dashboard/diagnostics

    bool hasLastPublished() const { return hasLastPublished_; }
    const MqttReading& lastPublished() const { return lastPublished_; }
    unsigned long lastPublishMillis() const { return lastPublishMillis_; }

    // Bypasses the reconnect throttle so the next update() retries immediately -
    // for a dashboard "test connection now" button.
    void forceReconnectNow();

  private:
    IMqttTransport& transport_;
    size_t bufferCapacity_;
    unsigned long reconnectIntervalMs_;

    Sample buffer_[kMaxBuffered];
    size_t bufferHead_;
    size_t bufferCount_;
    unsigned long lastConnectAttempt_;
    bool discoveryPublished_;
    int publishCount_;
    MqttReading lastPublished_;
    bool hasLastPublished_;
    unsigned long lastPublishMillis_;
    MqttReading latest_;
    bool latestDirty_;
    MqttLiveState live_;
    int connectFailures_;

    void enqueue(const MqttReading& r);
    static Sample pack(const MqttReading& r);
    static MqttReading unpack(const Sample& s);
    void publishState(unsigned long currentMillis);
    void publishBatch();
    void publishDiscovery();
};

#endif
