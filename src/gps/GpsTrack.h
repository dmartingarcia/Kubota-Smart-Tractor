#ifndef GPS_TRACK_H
#define GPS_TRACK_H

#include <stddef.h>
#include <stdint.h>

// Session track of where the tractor has been, plus GPX helpers. Pure logic, no hardware.
//
// Fixed RAM (kCapacity points x 12 bytes). A point is only recorded once it is at least
// minDistanceMeters from the last recorded one (parked jitter adds nothing). When the buffer
// fills up it is halved (every other point kept) and the spacing doubles, so the WHOLE session
// stays covered at a coarser resolution instead of the oldest part being thrown away.

struct TrackPoint {
  int32_t latE6;   // degrees * 1e6
  int32_t lonE6;
  uint32_t epoch;  // UTC seconds, 0 = unknown
};

class GpsTrack {
  public:
    static const size_t kCapacity = 500;

    explicit GpsTrack(double minDistanceMeters);

    bool add(double lat, double lon, uint32_t epochSeconds); // true if recorded
    size_t size() const { return count_; }
    TrackPoint at(size_t index) const { return points_[index]; } // oldest first
    double distanceMeters() const { return distanceM_; }         // true path length, survives compaction
    double minDistanceMeters() const { return minDistanceM_; }
    void clear();

  private:
    TrackPoint points_[kCapacity];
    size_t count_;
    double baseMinDistanceM_;
    double minDistanceM_;
    double distanceM_;
    int32_t lastLatE6_, lastLonE6_;
    bool hasLast_;

    void compact();
};

// Metres between two points given in 1e-6 degrees (equirectangular: plenty for tractor distances).
double track_distance_meters(int32_t latE6a, int32_t lonE6a, int32_t latE6b, int32_t lonE6b);

// Calendar UTC -> epoch seconds (GPS gives date+time, so no NTP/WiFi is needed for timestamps).
uint32_t utc_to_epoch(int year, int month, int day, int hour, int minute, int second);

// "2026-10-02T12:34:56Z" into buf (needs >= 21 bytes); returns the length written.
size_t epoch_to_iso8601(uint32_t epoch, char* buf, size_t bufSize);

// One <trkpt> line (with <time> when epoch != 0). Returns the length written, 0 if it did not fit.
size_t gpx_format_point(char* buf, size_t bufSize, const TrackPoint& p);

extern const char kGpxHeader[]; // XML prolog + <gpx><trk><trkseg>
extern const char kGpxFooter[]; // </trkseg></trk></gpx>

#endif
