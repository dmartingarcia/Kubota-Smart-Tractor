#ifndef GPS_TRACK_H
#define GPS_TRACK_H

#include <stddef.h>
#include <stdint.h>

// Track point + geometry/time/GPX helpers. Pure logic, no hardware access.

struct TrackPoint {
  int32_t latE6;   // degrees * 1e6, or kSegmentBreak
  int32_t lonE6;
  uint32_t epoch;  // UTC seconds, 0 = unknown
};

// Marker record: "a new session starts here" (the line must not join the previous point).
constexpr int32_t kSegmentBreak = INT32_MIN;
inline bool is_segment_break(const TrackPoint& p) { return p.latE6 == kSegmentBreak; }

// Metres between two points given in 1e-6 degrees (equirectangular: plenty for tractor distances).
double track_distance_meters(int32_t latE6a, int32_t lonE6a, int32_t latE6b, int32_t lonE6b);

// Perpendicular distance in metres from p to the infinite line through a and b (local planar
// approximation). Falls back to |a-p| when a and b coincide.
double track_cross_track_meters(const TrackPoint& a, const TrackPoint& b, const TrackPoint& p);

// Heading change in degrees (0 = straight on, 180 = reversal) at b when going a -> b -> c.
double track_turn_angle_degrees(const TrackPoint& a, const TrackPoint& b, const TrackPoint& c);

// Calendar UTC -> epoch seconds (GPS gives date+time, so no NTP/WiFi is needed for timestamps).
uint32_t utc_to_epoch(int year, int month, int day, int hour, int minute, int second);

// "2026-10-02T12:34:56Z" into buf (needs >= 21 bytes); returns the length written.
size_t epoch_to_iso8601(uint32_t epoch, char* buf, size_t bufSize);

// One <trkpt> line (with <time> when epoch != 0). Returns the length written, 0 if it did not fit.
size_t gpx_format_point(char* buf, size_t bufSize, const TrackPoint& p);

extern const char kGpxHeader[];       // XML prolog + <gpx><trk><trkseg>
extern const char kGpxSegmentBreak[]; // </trkseg><trkseg>
extern const char kGpxFooter[];       // </trkseg></trk></gpx>

#endif
