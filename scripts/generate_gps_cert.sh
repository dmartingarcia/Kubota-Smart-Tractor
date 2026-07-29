#!/usr/bin/env bash
# Generates a self-signed cert/key for the GPS HTTPS server and embeds them as
# PROGMEM C++ literals in src/web/GpsHttpsCert.h (gitignored, like secrets.h).
#
# Why self-signed + baked-in: Geolocation only works in a "secure context"
# (HTTPS or localhost) - browsers block it outright on plain HTTP LAN IPs, even
# fully offline in AP mode. There's no way to get a CA-trusted cert for a local
# IP with no internet, so a self-signed cert (with a one-time per-device
# "accept the risk" browser warning) is the only way to unlock it here.
#
# Run this once per device if you want a cert unique to your unit; otherwise
# every clone of this repo that skips this step shares no cert at all (HTTPS
# GPS mode just won't build/work until you run it).
set -euo pipefail
cd "$(dirname "$0")/.."

TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT

openssl req -x509 -nodes -newkey rsa:2048 \
  -keyout "$TMP_DIR/key.pem" -out "$TMP_DIR/cert.pem" \
  -days 3650 -subj "/CN=kubotio.local"

{
  echo "#ifndef GPS_HTTPS_CERT_H"
  echo "#define GPS_HTTPS_CERT_H"
  echo
  echo "// Self-signed cert/key for the on-demand GPS HTTPS server, generated locally by"
  echo "// scripts/generate_gps_cert.sh - not committed to git (see .gitignore)."
  echo
  echo 'static const char GPS_HTTPS_CERT[] PROGMEM = R"EOF('
  cat "$TMP_DIR/cert.pem"
  echo ')EOF";'
  echo
  echo 'static const char GPS_HTTPS_KEY[] PROGMEM = R"EOF('
  cat "$TMP_DIR/key.pem"
  echo ')EOF";'
  echo
  echo "#endif"
} > src/web/GpsHttpsCert.h

echo "Wrote src/web/GpsHttpsCert.h"
