#include "PubSubMqttTransport.h"

PubSubMqttTransport::PubSubMqttTransport(const char* host, uint16_t port, const char* clientId,
                                         const char* user, const char* password)
  : mqttClient_(wifiClient_), clientId_(clientId), user_(user), password_(password) {
  mqttClient_.setServer(host, port);
  mqttClient_.setSocketTimeout(1); // seconds - bounds worst-case blocking on connect()/publish()
  mqttClient_.setBufferSize(2200); // batched publishes (MqttPublisher::kMaxPerBatch) need more than the 256B default
}

bool PubSubMqttTransport::connected() {
  return mqttClient_.connected();
}

bool PubSubMqttTransport::connect() {
  if (user_ != nullptr && user_[0] != '\0') {
    return mqttClient_.connect(clientId_, user_, password_);
  }
  return mqttClient_.connect(clientId_);
}

bool PubSubMqttTransport::publish(const char* topic, const char* payload, bool retain) {
  return mqttClient_.publish(topic, payload, retain);
}

void PubSubMqttTransport::loop() {
  mqttClient_.loop();
}
