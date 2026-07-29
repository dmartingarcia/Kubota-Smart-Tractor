# Smart Tractor Alternator Controller 🚜⚡

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CI](https://github.com/dmartingarcia/Kubota-Smart-Tractor/actions/workflows/ci.yml/badge.svg)](https://github.com/dmartingarcia/Kubota-Smart-Tractor/actions/workflows/ci.yml)

ESP8266-based intelligent charging system with adaptive PID control and web monitoring. Maintains optimal battery health using advanced charge management.

## Features ✨
- 🎮 **Adaptive PID Control** - Maintains precise voltage regulation, with one-shot
  **relay-feedback autotune** to derive Kp/Ki/Kd automatically
- ⚡ **Dual Output Modes** - Relay or PWM MOSFET control (configurable)
- 📶 **WiFi STA + AP fallback** - Tries the home network first (non-blocking), falls
  back to a local AP if it can't connect; retries STA periodically in the background
- 🏠 **Home Assistant via MQTT** - Publishes telemetry + HA MQTT discovery configs;
  buffers readings while offline and flushes them once connectivity is back
- 🌐 **Web Dashboard** - Real-time monitoring, historical chart (no external
  dependencies - works fully offline in AP mode), autotune trigger, WiFi/MQTT status
- 🛠️ **Maintenance Logbook** - Charging-hours + power-on counters and a resettable
  service-interval counter, plus a logbook page to record maintenance events, all
  persisted to flash
- 🛰️ **GPS Tracking** - On-demand HTTPS page (needed for browser Geolocation): live
  speed, track recording with GPX export, straight-line (AB) driving guidance
- 🔄 **OTA Updates** - Wireless firmware upgrades
- 📊 **Advanced Telemetry** - Voltage, PWM%, PID output, and engine status
- 🔒 **Safety envelope** - Over-voltage cutoff always enforced, including during
  autotune (see [Safety Systems](#safety-systems-️))

## Hardware Requirements 🔨
| Component              | Specification                           |
|------------------------|-----------------------------------------|
| Microcontroller        | ESP8266 (NodeMCU/Wemos D1 Mini)         |
| Voltage Divider        | R1=220kΩ, R2=68kΩ (1% tolerance)        |
| Output Device          | 5V SPDT Relay **or** 30A MOSFET + Heatsink |
| Power Input            | 12V Tractor Battery                     |
| Protection             | 10A Fuse, Reverse Polarity Diode        |

**Wiring Diagrams:**

*Relay Mode:*

Battery+ → Fuse → [220k] → A0 → [68k] → GND

Relay COM → Alternator Field Circuit

*MOSFET Mode:*

Battery+ → MOSFET Source → Alternator Field

MOSFET Gate → PWM Pin (D1)

MOSFET Drain → GND (with heatsink)


## Installation 📥

### PlatformIO Setup
1. Install [PlatformIO Core](https://platformio.org/install)
2. Clone repository:
   ```bash
   git clone https://github.com/dmartingarcia/Kubota-Smart-Tractor.git
   cd Kubota-Smart-Tractor
   ```
3. Install dependencies:
   ```bash
   pio pkg install
   ```

### Configuration
1. Create `src/secrets.h` (see `src/secrets.h.tpl`):
   ```cpp
   const char* ap_ssid = "Kubotio-AP";      // AP fallback SSID
   const char* ap_password = "12345678";    // AP fallback password (min 8 characters)

   const char* sta_ssid = "";               // Home WiFi - empty skips straight to AP
   const char* sta_password = "";

   const char* mqtt_host = "";              // Home Assistant MQTT broker - empty disables MQTT
   const uint16_t mqtt_port = 1883;
   const char* mqtt_user = "";
   const char* mqtt_password = "";
   ```
2. Configure operation mode in `src/main.cpp`:
   ```cpp
   #define USE_PWM true      // false for relay mode
   #define OUTPUT_PIN D1     // PWM-capable pin
   #define PID_SETPOINT 14.6 // Optimal charging voltage
   ```

## Calibration Guide 🔧
### Voltage Calibration
1. Connect known good battery (12.6V+)
2. Measure actual voltage (V_true) with multimeter
3. Update calibration constants:
   ```cpp
   #define CALIBRATION_IN_VOLTAGE 12.65  // Measured voltage
   #define CALIBRATION_A0_VOLTAGE 2.89   // Serial monitor reading
   ```

### PID Tuning Procedure
1. **Initial Setup** (P-only control):
   ```cpp
   PID chargePID(&pidInput, &pidOutput, &pidSetpoint, 80.0, 0, 0, DIRECT);
   ```
2. **Tuning Steps**:
   - Increase P until system oscillates, then reduce by 50%
   - Add Integral control (start with 0.1*P)
   - Add Derivative control (start with 0.01*P)
3. **Example Safe Values**:
   ```cpp
   PID chargePID(&pidInput, &pidOutput, &pidSetpoint, 80.0, 5.0, 0.5, DIRECT);
   ```

## Testing 🧪
All non-trivial logic is split from hardware access behind small interfaces (a HAL),
so it runs natively on your machine — no ESP8266 board needed:
```bash
pio test -e native
```
| Module | What it covers |
|---|---|
| `VoltageSensor` / `AlternatorLogic` | Calibration math, safety thresholds, relay hysteresis, PID cycle scheduling |
| `WifiManager` (+ `IWifiDriver`) | STA/AP fallback state machine, against a fake driver |
| `PidAutotuner` | Relay-feedback autotune math (Ku/Pu → Kp/Ki/Kd) |
| `UsageCounters` (+ `IFlashStore`) | Charging-hours/boot-count accumulation, throttled saves, against an in-memory fake |
| `MqttPublisher` (+ `IMqttTransport`) | Offline buffering/flush, HA discovery, reconnect throttling, against a fake transport |
| `MaintenanceLog` (+ `IMaintenanceLogStore`) | Note sanitization, log round-trip, against a fake store |

`main.cpp` wires these pure/HAL-backed decisions into the real hardware calls (PID,
PWM, relay, WiFi, flash, MQTT). When touching any of this behavior, add/extend a
native test first (TDD) — the real hardware implementations (`Esp8266WifiDriver`,
`LittleFsStore`, `PubSubMqttTransport`, `LittleFsMaintenanceLogStore`) are thin and
intentionally left untested here (no board attached to CI/dev machine); verify them
on real hardware after native tests pass.

**Coverage:**
```bash
pio test -e coverage   # same suite, instrumented
gcovr --root . --filter 'src/charging/.*' --filter 'src/connectivity/.*' --filter 'src/storage/.*' \
  --object-directory .pio/build/coverage -s
```
(On macOS, add `--gcov-executable "xcrun llvm-cov gcov"`.) CI runs this on every push
and fails if line/function coverage drops below 90%.

**CI:** GitHub Actions (`.github/workflows/ci.yml`) runs the native test suite, the
coverage gate, and a firmware compile check (`pio run -e d1_mini`, with a placeholder
`secrets.h` - no real credentials needed to verify it builds) on every push/PR.

## Web Interface 🌐
If `sta_ssid` is set and reachable, the dashboard is available on your home network's
IP. Otherwise (or always, as a fallback), connect to the AP WiFi network and browse to
`http://192.168.4.1`. The dashboard has no external dependencies (chart is drawn with
plain `<canvas>`, no CDN) so it renders correctly even fully offline in AP mode.

**Dashboard (`/`):**
- Real-time voltage chart (last ~10min of history)
- PWM%/Relay status, engine status
- WiFi mode (home network vs. AP fallback) and what that means for HA/MQTT reachability
- PID autotune trigger + status
- Charging hours, power-on count, maintenance due indicator, service interval editor

**Maintenance logbook (`/maintenance`):**
- Table of logged maintenance events (date - if NTP has synced, otherwise "unknown
  date" - hours at time of service, free-text note)
- Form to log a new entry

## GPS Tracking 🛰️
Browsers only allow the Geolocation API in a "secure context" (HTTPS, or
`localhost`) - they block it outright on a plain-HTTP LAN IP like the dashboard's,
even fully offline in AP mode. To make GPS features possible at all, there's a
second, separate HTTPS server (self-signed cert, port 443) that only runs the GPS
page - it's started/stopped on demand from the dashboard's "GPS Mode" toggle, so its
RAM/TLS cost is only paid while actually using it, not for the whole time the tractor
is charging.

**Setup:**
```bash
./scripts/generate_gps_cert.sh   # once per checkout/device; writes src/web/GpsHttpsCert.h (gitignored)
```
Without this, the firmware still builds and runs fine - GPS mode just isn't usable
until the cert exists.

**Using it:** enable "GPS Mode" on the dashboard, then open the HTTPS link it shows.
The browser will warn the certificate isn't trusted (expected - it's self-signed,
there's no CA reachable for a LAN-only device); accept it once per device. From there:
- Live speed and GPS accuracy
- Start/stop track recording, stored in the browser's IndexedDB (not on the device);
  download any recorded track as a `.gpx` file, or delete it
- Straight-line (AB-line) driving guidance: mark point A, drive, mark point B, and a
  bar shows how far off that line you are as you go

**Known limitations:**
- The self-signed cert is generated once per checkout and baked into the firmware; it
  is *not* unique per physical device unless you re-run the script per device you flash.
  It exists only to satisfy the browser's secure-context check, not to protect anything
  sensitive on this LAN-only server.
- TLS handshakes are RAM-hungry on an ESP8266. This was only verified to *compile* and
  report a plausible RAM/flash footprint (see below); it has not been exercised on real
  hardware this session (no board attached). If it turns out to be too tight alongside
  WiFi/MQTT/the main dashboard, the on-demand start/stop design at least means it can
  only affect things while GPS mode is actively toggled on.
- `ESP8266WebServerSecure` supports one simultaneous client - fine for a single phone.

**Parcel &amp; coverage:** on the same GPS page, enter a Spanish cadastral reference
(*referencia catastral*) to fetch the field's boundary from Sede del Catastro's public
INSPIRE WFS and an aerial photo background from IGN's PNOA WMS - both free, no API
key, fetched directly from the phone's browser (needs internet, so do this on home
WiFi). The boundary and photo are cached in `localStorage` for offline reuse afterward.
Set a working width and, as you drive with GPS enabled, it fills in a coverage grid
(cell size = working width) over the field and shows % covered. Verified working
end-to-end against a real parcel this session.

## Home Assistant / MQTT 🏠
- On boot, the device tries the home WiFi (`sta_ssid`) first; if it can't connect
  within ~15s it falls back to its own AP, and keeps retrying the home network in the
  background every ~60s. WiFi connects are non-blocking so the PID loop's cadence
  never stalls waiting on a handshake.
- MQTT/Home Assistant is only reachable while on the home network - the AP fallback is
  offline-only by design (it's meant for direct access when there's no home WiFi in range).
- Set `mqtt_host` in `secrets.h` to enable; leave it empty to disable MQTT entirely.
- Readings are buffered in RAM (up to 20) while offline/disconnected and flushed to the
  broker once reconnected, so brief outages don't lose data. HA MQTT discovery configs
  are published on (re)connect, so entities show up automatically (voltage, PWM%,
  charging hours, maintenance-due).
- **Known limitation:** the underlying MQTT client's `connect()` can still block
  briefly on an unreachable broker; it's throttled to at most once per ~30s and its
  socket timeout is capped at 1s to bound the worst case, but it isn't fully
  non-blocking like the WiFi manager. Acceptable given the cadence, but worth knowing.

## System Indicators 💡
The status LED tracks PID output brightness while in PWM mode (dim = low charge
output, bright = high); there are no distinct blink patterns for WiFi/relay state.
Use the web dashboard for WiFi/mode status instead.

## OTA Updates 🛠️
1. Connect to the AP WiFi network (`ap_ssid`/`ap_password` from `secrets.h`)
2. In PlatformIO:
   ```bash
   pio run --target upload --environment wemos_d1_mini_ota
   ```
   or
   ```bash
   pio run --target upload --upload-port 192.168.4.1
   ```
3. Monitor serial output for progress

## Troubleshooting 🔍
| Issue                  | Solution                      |
|------------------------|-------------------------------|
| No PWM output          | Verify MOSFET wiring & PWM config |
| Voltage oscillations   | Reduce PID P value            |
| Web interface offline  | Check WiFi mode (dashboard) & that `server.begin()` ran |
| Overheating MOSFET     | Add heatsink & verify current |

## Safety Systems ⚠️
- **Implemented today**: hard cutoff at `VOLTAGE_THRESHOLD_HIGH` (14.6V by default) -
  the alternator is forced off above this regardless of PID/autotune output
  (`decide_pwm_safety_action` in `AlternatorLogic`, covered by native tests). Autotune
  swings output but is still subject to this same cutoff every cycle, never bypasses it.
- **Not implemented yet** (aspirational, don't rely on these): PWM rate limiting,
  thermal shutdown (no temperature sensor wired up), watchdog timer. Treat the
  hardware notes below as mandatory regardless.

**Critical Notes**:
- 🔥 Always use appropriately rated components
- 🛑 Double-check polarity before power-on
- 🔋 Maintain battery temperature monitoring
- 🧯 Enclose in IP67-rated waterproof case

## Persistence design notes 💾
- **Usage counters** (`UsageCounters`, `/usage.bin`): saved every 300s of *accumulated
  active charging time* (not wall-clock, not every loop). Worst case on an abrupt power
  cut (this tractor has no shutdown signal) is losing the last <300s of hours - harmless
  for a maintenance counter, since it only ever undercounts. At this cadence, even
  continuous charging stays far under typical NOR flash erase-cycle budgets over the
  unit's lifetime; LittleFS also wear-levels across its partition rather than hammering
  one sector. Boot count is saved once per real power-on (rare, unthrottled).
- **Maintenance logbook** (`MaintenanceLog`, `/maintenance_log.csv`): append-only,
  written only when a user logs a real service event (a handful of times a year at
  most) - flash wear is a non-issue here regardless of the counters' cadence above.

## Roadmap 🗺️
- **MQTT `connect()` blocking caveat** - see [Home Assistant / MQTT](#home-assistant--mqtt-) above; a
  fully non-blocking MQTT client would need a custom async TCP state machine.
- **Engine-vs-charging detection** - `engine_running` is currently always `false`
  (see the `TODO` in `main.cpp`); usage-hour tracking currently uses
  *alternator-active* time as a proxy, which is reasonable but not identical to engine
  runtime.
- Real-hardware validation of WiFi STA/AP fallback, MQTT, LittleFS persistence, and
  the GPS HTTPS server - everything above is native-unit-tested and compiles for
  `d1_mini`, but hasn't been run on an actual board this session (none attached to the
  dev machine). The HTTPS/TLS RAM footprint is the one most worth checking first.
## License 📄
MIT License - See [LICENSE](LICENSE) for details
*PID Library:* BSD 3-Clause (included in dependencies)