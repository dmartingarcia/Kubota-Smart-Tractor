#ifndef GPS_READER_H
#define GPS_READER_H

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Real hardware GPS reader (SoftwareSerial + TinyGPS++). No HAL boundary/interface
// here (unlike WifiDriver/MqttTransport) - there's no meaningful pure-logic decision
// to unit test, TinyGPS++ itself already does the NMEA parsing/validation.
//
// Two independent availability signals, since a module can be wired but not yet have
// a satellite fix (cold start can take 30s-a few minutes outdoors):
//   isConnected() - module is physically present and sending SOME data, fix or not.
//   hasFix()      - module has a recent, valid location (safe to actually use/display).
class GpsReader {
  public:
    GpsReader(uint8_t rxPin, uint8_t txPin, uint32_t baud);

    void begin();
    void update(); // non-blocking: drains whatever bytes are available, call every loop()

    bool isConnected() const;
    bool hasFix() const;

    // Not const: TinyGPS++'s accessors aren't const-qualified themselves.
    double latitude();
    double longitude();
    double speedKmh();
    uint32_t fixAgeMs(); // milliseconds since the last valid location update
    uint32_t epochUtc();  // UTC seconds from the GPS date/time, 0 if not valid yet (no NTP/WiFi needed)

  private:
    SoftwareSerial serial_;
    TinyGPSPlus gps_;
    uint32_t baud_;
};

#endif
