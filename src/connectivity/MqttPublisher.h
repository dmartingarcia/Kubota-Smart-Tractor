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
  uint16_t pwmValue;
  bool active;
  bool engineRunning;
  uint32_t totalRunSeconds;
  bool maintenanceDue;
};

// Publishes telemetry to MQTT/Home Assistant, buffering readings while offline
// (no WiFi STA / broker unreachable) and flushing them once connectivity is back
// (store-and-forward). Publishes HA MQTT discovery configs once per (re)connect.
// update() must stay cheap: reconnect attempts are throttled to reconnectIntervalMs,
// since IMqttTransport::connect() can block briefly on a real socket.
class MqttPublisher {
  public:
    static constexpr size_t kMaxBuffered = 40;

    MqttPublisher(IMqttTransport& transport, size_t bufferCapacity, unsigned long reconnectIntervalMs);

    void update(bool networkAvailable, const MqttReading& latest, unsigned long currentMillis);

    size_t bufferedCount() const;
    int publishCount() const; // total successful publish() calls, for tests/diagnostics

  private:
    IMqttTransport& transport_;
    size_t bufferCapacity_;
    unsigned long reconnectIntervalMs_;

    MqttReading buffer_[kMaxBuffered];
    size_t bufferHead_;
    size_t bufferCount_;
    unsigned long lastConnectAttempt_;
    bool discoveryPublished_;
    int publishCount_;

    void enqueue(const MqttReading& r);
    void flushBuffer();
    void publishReading(const MqttReading& r);
    void publishDiscovery();
};

#endif
