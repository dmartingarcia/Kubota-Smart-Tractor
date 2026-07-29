#ifndef PUBSUB_MQTT_TRANSPORT_H
#define PUBSUB_MQTT_TRANSPORT_H

#include "MqttPublisher.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Real hardware IMqttTransport impl (PubSubClient over TCP). Excluded from native tests.
// Socket timeout is kept short so a broker that's unreachable can't stall the caller
// (and therefore the PID loop) for long - see WifiManager/should_run_cycle for the
// same non-blocking-cadence concern.
class PubSubMqttTransport : public IMqttTransport {
  public:
    PubSubMqttTransport(const char* host, uint16_t port, const char* clientId,
                        const char* user, const char* password);

    bool connected() override;
    bool connect() override;
    bool publish(const char* topic, const char* payload, bool retain) override;
    void loop() override;

  private:
    WiFiClient wifiClient_;
    PubSubClient mqttClient_;
    const char* clientId_;
    const char* user_;
    const char* password_;
};

#endif
