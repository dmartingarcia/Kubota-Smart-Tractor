#include "WifiManager.h"

WifiManager::WifiManager(IWifiDriver& driver,
                          const char* staSsid, const char* staPassword,
                          const char* apSsid, const char* apPassword,
                          unsigned long staTimeoutMs, unsigned long apRetryIntervalMs)
  : driver_(driver),
    staSsid_(staSsid), staPassword_(staPassword),
    apSsid_(apSsid), apPassword_(apPassword),
    staTimeoutMs_(staTimeoutMs), apRetryIntervalMs_(apRetryIntervalMs),
    mode_(WifiMode::CONNECTING_STA), staDeadline_(0), nextApRetry_(0) {}

void WifiManager::startSTA(unsigned long currentMillis) {
  driver_.beginSTA(staSsid_, staPassword_);
  mode_ = WifiMode::CONNECTING_STA;
  staDeadline_ = currentMillis + staTimeoutMs_;
}

void WifiManager::begin(unsigned long currentMillis) {
  if (staSsid_ == nullptr || staSsid_[0] == '\0') {
    driver_.beginAP(apSsid_, apPassword_);
    mode_ = WifiMode::AP_FALLBACK;
    nextApRetry_ = currentMillis + apRetryIntervalMs_;
    return;
  }
  startSTA(currentMillis);
}

void WifiManager::update(unsigned long currentMillis) {
  switch (mode_) {
    case WifiMode::CONNECTING_STA: {
      StaLinkStatus status = driver_.staStatus();
      if (status == StaLinkStatus::CONNECTED) {
        mode_ = WifiMode::CONNECTED_STA;
      } else if (currentMillis - staDeadline_ < (1UL << 31)) { // deadline reached (non-wrapping compare)
        driver_.beginAP(apSsid_, apPassword_);
        mode_ = WifiMode::AP_FALLBACK;
        nextApRetry_ = currentMillis + apRetryIntervalMs_;
      }
      break;
    }
    case WifiMode::CONNECTED_STA: {
      if (driver_.staStatus() != StaLinkStatus::CONNECTED) {
        startSTA(currentMillis);
      }
      break;
    }
    case WifiMode::AP_FALLBACK: {
      if (currentMillis - nextApRetry_ < (1UL << 31)) {
        startSTA(currentMillis);
      }
      break;
    }
  }
}

WifiMode WifiManager::mode() const {
  return mode_;
}
