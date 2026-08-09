#include "GpsJumpFilter.h"
#include <cmath>

namespace {
constexpr double kEarthRadiusMeters = 6371000.0;

double toRadians(double degrees) { return degrees * M_PI / 180.0; }

double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(toRadians(lat1)) * cos(toRadians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return kEarthRadiusMeters * c;
}
}

GpsJumpFilter::GpsJumpFilter(double maxPlausibleSpeedKmh, int maxConsecutiveRejects)
  : maxPlausibleSpeedKmh_(maxPlausibleSpeedKmh), maxConsecutiveRejects_(maxConsecutiveRejects),
    hasLast_(false), lastLat_(0), lastLon_(0), lastMillis_(0), consecutiveRejects_(0) {}

bool GpsJumpFilter::accept(double lat, double lon, unsigned long fixMillis) {
  if (!hasLast_) {
    hasLast_ = true;
    lastLat_ = lat;
    lastLon_ = lon;
    lastMillis_ = fixMillis;
    consecutiveRejects_ = 0;
    return true;
  }

  double dtSeconds = (fixMillis - lastMillis_) / 1000.0;
  bool implausible = true;
  if (dtSeconds > 0) {
    double distanceMeters = haversineMeters(lastLat_, lastLon_, lat, lon);
    double impliedSpeedKmh = (distanceMeters / dtSeconds) * 3.6;
    implausible = impliedSpeedKmh > maxPlausibleSpeedKmh_;
  }

  if (implausible && consecutiveRejects_ < maxConsecutiveRejects_) {
    consecutiveRejects_++;
    return false;
  }

  lastLat_ = lat;
  lastLon_ = lon;
  lastMillis_ = fixMillis;
  consecutiveRejects_ = 0;
  return true;
}
