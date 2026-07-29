#ifndef HTTPS_GPS_SERVER_H
#define HTTPS_GPS_SERVER_H

// On-demand HTTPS server (self-signed) just for the GPS page: Geolocation only
// works in a secure context, which plain HTTP on a LAN IP never satisfies. Kept
// separate from the always-on HTTP dashboard (WebHandler) and started/stopped
// explicitly (via the dashboard's "Enable GPS mode" toggle) so its RAM/TLS cost
// is paid only while actually in use, not for the whole tractor's operating time.

bool gps_https_start();  // false if cert missing or already running
void gps_https_stop();
bool gps_https_active();
void gps_https_handle_client();  // call every loop() iteration when active; cheap no-op otherwise

#endif
