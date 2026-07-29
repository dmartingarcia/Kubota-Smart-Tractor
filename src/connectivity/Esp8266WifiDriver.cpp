#include "Esp8266WifiDriver.h"
#include <ESP8266WiFi.h>

void Esp8266WifiDriver::beginSTA(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // non-blocking: returns immediately, status polled via staStatus()
}

StaLinkStatus Esp8266WifiDriver::staStatus() {
  switch (WiFi.status()) {
    case WL_CONNECTED: return StaLinkStatus::CONNECTED;
    case WL_NO_SSID_AVAIL:
    case WL_CONNECT_FAILED:
    case WL_CONNECTION_LOST:
    case WL_DISCONNECTED:
      return StaLinkStatus::FAILED;
    default: return StaLinkStatus::CONNECTING;
  }
}

void Esp8266WifiDriver::beginAP(const char* ssid, const char* password) {
  WiFi.mode(WIFI_AP);
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
}
