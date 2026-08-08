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
  uint32_t freeHeap;
  bool overvoltageAlert;
};

// Publishes telemetry to MQTT/Home Assistant, buffering readings while offline
// (no WiFi STA / broker unreachable) and flushing them once connectivity is back
// (store-and-forward). Publishes HA MQTT discovery configs once per (re)connect.
// update() must stay cheap: reconnect attempts are throttled to reconnectIntervalMs,
// since IMqttTransport::connect() can block briefly on a real socket.
class MqttPublisher {
  public:
    static constexpr size_t kMaxBuffered = 20; // matches the capacity main.cpp actually passes

    MqttPublisher(IMqttTransport& transport, size_t bufferCapacity, unsigned long reconnectIntervalMs);

    void update(bool networkAvailable, const MqttReading& latest, unsigned long currentMillis);

    size_t bufferedCount() const;
    int publishCount() const; // total successful publish() calls, for tests/diagnostics
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

    MqttReading buffer_[kMaxBuffered];
    size_t bufferHead_;
    size_t bufferCount_;
    unsigned long lastConnectAttempt_;
    bool discoveryPublished_;
    int publishCount_;
    MqttReading lastPublished_;
    bool hasLastPublished_;
    unsigned long lastPublishMillis_;

    void enqueue(const MqttReading& r);
    void flushBuffer(unsigned long currentMillis);
    void publishReading(const MqttReading& r, unsigned long currentMillis);
    void publishDiscovery();
};

#endif
