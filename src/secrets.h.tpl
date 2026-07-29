// Access Point settings (fallback when home WiFi isn't reachable)
const char* ap_ssid = "Kubotio-AP";  // Access Point SSID
const char* ap_password = "12345678";  // Access Point password (min 8 characters)

// Home WiFi (station mode) - tried first on boot; needed to reach Home Assistant/MQTT.
// Leave as-is (empty) to skip straight to AP mode.
const char* sta_ssid = "";
const char* sta_password = "";

// MQTT broker (Home Assistant) - only reachable while on STA WiFi, not AP fallback.
// Leave mqtt_host empty to disable MQTT entirely.
const char* mqtt_host = "";
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";
