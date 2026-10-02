#include "GpsTrack.h"
#include <math.h>
#include <stdio.h>

const char kGpxHeader[] =
  "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
  "<gpx version=\"1.1\" creator=\"Kubota Smart Tractor\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
  "<trk><name>Tractor session</name><trkseg>\n";
const char kGpxSegmentBreak[] = "</trkseg>\n<trkseg>\n";
const char kGpxFooter[] = "</trkseg></trk></gpx>\n";

namespace {
constexpr double kMetersPerDegree = 111194.9266;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}

double track_distance_meters(int32_t latE6a, int32_t lonE6a, int32_t latE6b, int32_t lonE6b) {
  double lat1 = latE6a / 1e6, lat2 = latE6b / 1e6;
  double dy = (lat2 - lat1) * kMetersPerDegree;
  double dx = (lonE6b - lonE6a) / 1e6 * cos((lat1 + lat2) / 2.0 * kDegToRad) * kMetersPerDegree;
  return sqrt(dx * dx + dy * dy);
}

double track_cross_track_meters(const TrackPoint& a, const TrackPoint& b, const TrackPoint& p) {
  double kx = cos((a.latE6 / 1e6) * kDegToRad) * kMetersPerDegree / 1e6;
  double ky = kMetersPerDegree / 1e6;
  double bx = (b.lonE6 - a.lonE6) * kx, by = (b.latE6 - a.latE6) * ky;
  double px = (p.lonE6 - a.lonE6) * kx, py = (p.latE6 - a.latE6) * ky;
  double len = sqrt(bx * bx + by * by);
  if (len < 1e-6) return sqrt(px * px + py * py);
  return fabs(bx * py - by * px) / len;
}

// Howard Hinnant's days-from-civil / civil-from-days.
uint32_t utc_to_epoch(int year, int month, int day, int hour, int minute, int second) {
  year -= month <= 2;
  int era = (year >= 0 ? year : year - 399) / 400;
  unsigned yoe = static_cast<unsigned>(year - era * 400);
  unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = static_cast<long>(era) * 146097 + static_cast<long>(doe) - 719468;
  return static_cast<uint32_t>(days * 86400L + hour * 3600L + minute * 60L + second);
}

size_t epoch_to_iso8601(uint32_t epoch, char* buf, size_t bufSize) {
  long days = static_cast<long>(epoch / 86400UL);
  unsigned secs = epoch % 86400UL;
  days += 719468;
  long era = days / 146097;
  unsigned doe = static_cast<unsigned>(days - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  long y = static_cast<long>(yoe) + era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  unsigned d = doy - (153 * mp + 2) / 5 + 1;
  unsigned m = mp < 10 ? mp + 3 : mp - 9;
  y += m <= 2;
  int n = snprintf(buf, bufSize, "%04ld-%02u-%02uT%02u:%02u:%02uZ", y, m, d, secs / 3600, (secs / 60) % 60, secs % 60);
  return n > 0 ? static_cast<size_t>(n) : 0;
}

size_t gpx_format_point(char* buf, size_t bufSize, const TrackPoint& p) {
  auto fmt = [](int32_t v, char* out, size_t outSize) {
    const char* sign = v < 0 ? "-" : "";
    long a = v < 0 ? -static_cast<long>(v) : v;
    snprintf(out, outSize, "%s%ld.%06ld", sign, a / 1000000L, a % 1000000L);
  };
  char lat[16], lon[16];
  fmt(p.latE6, lat, sizeof(lat));
  fmt(p.lonE6, lon, sizeof(lon));
  int n;
  if (p.epoch != 0) {
    char iso[24];
    epoch_to_iso8601(p.epoch, iso, sizeof(iso));
    n = snprintf(buf, bufSize, "<trkpt lat=\"%s\" lon=\"%s\"><time>%s</time></trkpt>\n", lat, lon, iso);
  } else {
    n = snprintf(buf, bufSize, "<trkpt lat=\"%s\" lon=\"%s\"></trkpt>\n", lat, lon);
  }
  return (n > 0 && static_cast<size_t>(n) < bufSize) ? static_cast<size_t>(n) : 0;
}

double track_turn_angle_degrees(const TrackPoint& a, const TrackPoint& b, const TrackPoint& c) {
  double kx = cos((b.latE6 / 1e6) * kDegToRad) * kMetersPerDegree / 1e6;
  double ky = kMetersPerDegree / 1e6;
  double v1x = (b.lonE6 - a.lonE6) * kx, v1y = (b.latE6 - a.latE6) * ky;
  double v2x = (c.lonE6 - b.lonE6) * kx, v2y = (c.latE6 - b.latE6) * ky;
  return fabs(atan2(v1x * v2y - v1y * v2x, v1x * v2x + v1y * v2y)) / kDegToRad;
}
