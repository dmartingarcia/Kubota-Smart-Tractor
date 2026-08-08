#ifndef ESP8266_WIFI_DRIVER_H
#define ESP8266_WIFI_DRIVER_H

#include "WifiManager.h"

// Real hardware IWifiDriver impl. Excluded from native tests (uses ESP8266WiFi).
class Esp8266WifiDriver : public IWifiDriver {
  public:
    void beginSTA(const char* ssid, const char* password) override;
    StaLinkStatus staStatus() override;
    void beginAP(const char* ssid, const char* password) override;
    void stopSTA() override;
};

#endif
