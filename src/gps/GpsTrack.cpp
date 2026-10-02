#include "GpsTrack.h"
#include <math.h>
#include <stdio.h>

const char kGpxHeader[] =
  "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
  "<gpx version=\"1.1\" creator=\"Kubota Smart Tractor\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
  "<trk><name>Tractor session</name><trkseg>\n";
const char kGpxFooter[] = "</trkseg></trk></gpx>\n";

namespace {
constexpr double kMetersPerDegree = 111194.9266;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
int32_t toE6(double deg) { return static_cast<int32_t>(deg >= 0 ? deg * 1e6 + 0.5 : deg * 1e6 - 0.5); }
}

double track_distance_meters(int32_t latE6a, int32_t lonE6a, int32_t latE6b, int32_t lonE6b) {
  double lat1 = latE6a / 1e6, lat2 = latE6b / 1e6;
  double dy = (lat2 - lat1) * kMetersPerDegree;
  double dx = (lonE6b - lonE6a) / 1e6 * cos((lat1 + lat2) / 2.0 * kDegToRad) * kMetersPerDegree;
  return sqrt(dx * dx + dy * dy);
}

GpsTrack::GpsTrack(double minDistanceMeters)
  : count_(0), baseMinDistanceM_(minDistanceMeters), minDistanceM_(minDistanceMeters), distanceM_(0),
    lastLatE6_(0), lastLonE6_(0), hasLast_(false) {}

bool GpsTrack::add(double lat, double lon, uint32_t epochSeconds) {
  int32_t latE6 = toE6(lat), lonE6 = toE6(lon);
  double step = 0;
  if (hasLast_) {
    step = track_distance_meters(lastLatE6_, lastLonE6_, latE6, lonE6);
    if (step < minDistanceM_) return false;
  }
  if (count_ == kCapacity) compact();
  points_[count_++] = TrackPoint{latE6, lonE6, epochSeconds};
  distanceM_ += step;
  lastLatE6_ = latE6;
  lastLonE6_ = lonE6;
  hasLast_ = true;
  return true;
}

void GpsTrack::compact() {
  size_t kept = 0;
  for (size_t i = 0; i < count_; i += 2) points_[kept++] = points_[i];
  count_ = kept;
  minDistanceM_ *= 2.0;
}

void GpsTrack::clear() {
  count_ = 0;
  distanceM_ = 0;
  minDistanceM_ = baseMinDistanceM_;
  hasLast_ = false;
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
