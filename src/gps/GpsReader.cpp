#include "GpsReader.h"

namespace {
constexpr uint32_t kMaxFixAgeMs = 5000; // older than this, treat the fix as stale/unsafe to use
constexpr unsigned long kMinCharsForConnected = 10; // a few bytes proves *something* is wired
}

GpsReader::GpsReader(uint8_t rxPin, uint8_t txPin, uint32_t baud)
  : serial_(rxPin, txPin), baud_(baud) {}

void GpsReader::begin() {
  serial_.begin(baud_);
}

void GpsReader::update() {
  while (serial_.available() > 0) {
    gps_.encode(serial_.read());
  }
}

bool GpsReader::isConnected() const {
  // Sticky once true: if the module stops sending entirely, hasFix() going stale
  // (via fixAgeMs()) is the signal that actually matters for whether to trust data.
  return gps_.charsProcessed() > kMinCharsForConnected;
}

bool GpsReader::hasFix() const {
  return gps_.location.isValid() && gps_.location.age() < kMaxFixAgeMs;
}

double GpsReader::latitude() { return gps_.location.lat(); }
double GpsReader::longitude() { return gps_.location.lng(); }
double GpsReader::speedKmh() { return gps_.speed.kmph(); }
uint32_t GpsReader::fixAgeMs() { return gps_.location.age(); }
