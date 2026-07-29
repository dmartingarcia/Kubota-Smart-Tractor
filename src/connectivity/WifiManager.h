#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

// HAL boundary: no Arduino/ESP8266 includes here, so this is natively testable
// against a fake driver. See Esp8266WifiDriver for the real hardware implementation.

enum class StaLinkStatus { CONNECTING, CONNECTED, FAILED };

class IWifiDriver {
  public:
    virtual ~IWifiDriver() = default;
    virtual void beginSTA(const char* ssid, const char* password) = 0;
    virtual StaLinkStatus staStatus() = 0;
    virtual void beginAP(const char* ssid, const char* password) = 0;
};

enum class WifiMode { CONNECTING_STA, CONNECTED_STA, AP_FALLBACK };

// Non-blocking WiFi manager: tries the home network (STA) first, falls back to a
// local AP if STA doesn't connect within staTimeoutMs, and periodically retries STA
// while in AP fallback. update() must be cheap and non-blocking — call it every
// loop() iteration so the PID cadence (should_run_cycle) is never stalled by WiFi.
class WifiManager {
  public:
    WifiManager(IWifiDriver& driver,
                const char* staSsid, const char* staPassword,
                const char* apSsid, const char* apPassword,
                unsigned long staTimeoutMs, unsigned long apRetryIntervalMs);

    void begin(unsigned long currentMillis);
    void update(unsigned long currentMillis);
    WifiMode mode() const;

  private:
    IWifiDriver& driver_;
    const char* staSsid_;
    const char* staPassword_;
    const char* apSsid_;
    const char* apPassword_;
    unsigned long staTimeoutMs_;
    unsigned long apRetryIntervalMs_;

    WifiMode mode_;
    unsigned long staDeadline_;
    unsigned long nextApRetry_;

    void startSTA(unsigned long currentMillis);
};

#endif
