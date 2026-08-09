#ifndef GPS_JUMP_FILTER_H
#define GPS_JUMP_FILTER_H

// Pure logic, no hardware access - natively testable.
//
// Rejects a GPS fix if the implied speed from the last ACCEPTED fix is physically
// implausible for a tractor (a momentary GPS glitch/multipath spike, not real
// movement). A rejected fix doesn't update the reference point, so the next fix is
// judged against the same last-known-good position - a single-sample spike self-heals
// on the next good reading.
//
// Rejecting forever would be worse than an occasional bad fix getting through, so
// after maxConsecutiveRejects in a row, the next fix is accepted unconditionally
// (resync) - covers a genuinely bad seed point, or a real position change after a
// long fix outage (e.g. driving under dense tree cover) that would otherwise look
// like an implausible jump from the stale reference forever.
class GpsJumpFilter {
  public:
    GpsJumpFilter(double maxPlausibleSpeedKmh, int maxConsecutiveRejects);

    // lat/lon in degrees, fixMillis the time this fix was obtained. Returns true if
    // the fix is accepted (and becomes the new reference point), false if rejected.
    bool accept(double lat, double lon, unsigned long fixMillis);

    double lastLat() const { return lastLat_; }
    double lastLon() const { return lastLon_; }

  private:
    double maxPlausibleSpeedKmh_;
    int maxConsecutiveRejects_;

    bool hasLast_;
    double lastLat_, lastLon_;
    unsigned long lastMillis_;
    int consecutiveRejects_;
};

#endif
