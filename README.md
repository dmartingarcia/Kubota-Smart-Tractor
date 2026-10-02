# Smart Tractor Alternator Controller 🚜⚡

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CI](https://github.com/dmartingarcia/Kubota-Smart-Tractor/actions/workflows/ci.yml/badge.svg)](https://github.com/dmartingarcia/Kubota-Smart-Tractor/actions/workflows/ci.yml)

ESP8266-based intelligent charging system with adaptive PID control and web monitoring. Maintains optimal battery health using advanced charge management.

## Features ✨
- 🎮 **Real-time-aware PID Control** - Maintains precise voltage regulation, scaled by
  actual elapsed time between cycles (not an assumed fixed sample time), with one-shot
  **relay-feedback autotune** to derive Kp/Ki/Kd automatically
- ⚡ **Dual Output Modes** - Relay or PWM MOSFET control (configurable)
- 📶 **Exclusive AP/STA WiFi** - The AP is up by default and always reachable; a STA
  attempt briefly takes the radio and falls back to AP on failure/drop. The periodic
  STA retry never interrupts someone actively connected to the AP. Captive portal
  (auto-redirects AP clients to the dashboard) and a `/restart` endpoint + dashboard
  button for remote recovery without USB access
- 🏠 **Home Assistant via MQTT** - Samples telemetry every 10s, publishes in batches
  every 30s (buffers up to 30 minutes of samples while offline/disconnected and sends
  the backlog as soon as connectivity is back); HA MQTT discovery configs published on
  (re)connect; dashboard "test connection" button
- 🌐 **Web Dashboard** - Real-time monitoring, historical voltage chart (no external
  dependencies - works fully offline in AP mode), autotune trigger, WiFi/MQTT status,
  RAM/loop-time diagnostics, configurable refresh rate
- 🛠️ **Maintenance Logbook** - Charging-hours + power-on counters and a resettable
  service-interval counter, plus a logbook page to record maintenance events, all
  persisted to flash
- 🛰️ **GPS (optional hardware)** - Reads a serial NMEA GPS module directly (no phone/
  browser involved); dashboard only ever shows a GPS card once the module has a valid
  fix - gracefully absent if it's not wired up
- 🔄 **OTA Updates** - Wireless firmware upgrades
- 📊 **Advanced Telemetry** - Voltage, PWM%, PID output, and engine status (including a
  distinct "PROBING" state while testing for a running engine; once charging has been seen
  the engine stays "RUNNING" through voltage sags for `ENGINE_RUNNING_GRACE_MS` before any probe; when the engine looks off, probes repeat every 8s (a 0.05V rise during a pulse counts as running), so charging starts soon after RPM picks up). A valid GPS fix with speed above `ENGINE_GPS_MIN_SPEED_KMH` (3 km/h) also counts as engine running and skips probing
- 🔒 **Safety envelope** - Over-voltage cutoff (14.4V) always enforced, including during
  autotune, with a 60s-latched dashboard/MQTT alert so a brief spike doesn't go
  unnoticed (see [Safety Systems](#safety-systems-️))

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

MOSFET Gate → PWM Pin (D3, `RELAY_PIN` in `main.cpp` - same pin drives the relay or the MOSFET depending on `USE_PWM`)

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
   #define RELAY_PIN D3      // drives the relay or the MOSFET gate, depending on USE_PWM
   double Setpoint = 140.0;  // target charge voltage * 10 (140.0 = 14.0V)
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
`chargePID` is a `RealTimePid` (see [`src/charging/RealTimePid.h`](src/charging/RealTimePid.h)) - a
standard PID scaled by the *actual* elapsed time between `compute()` calls (derivative-on-
measurement, integral clamped to the output range for anti-windup), not the `PID_v1` library.
1. **Initial Setup** (P-only control):
   ```cpp
   RealTimePid chargePID(80.0, 0, 0, 0, MAX_CHARGE_CURRENT_PWM);
   ```
2. **Tuning Steps**:
   - Increase P until system oscillates, then reduce by 50%
   - Add Integral control (start with 0.1*P)
   - Add Derivative control (start with 0.01*P)
3. **Example Safe Values**:
   ```cpp
   RealTimePid chargePID(80.0, 5.0, 0.5, 0, MAX_CHARGE_CURRENT_PWM);
   ```
   Or skip manual tuning entirely and use the dashboard's one-shot autotune trigger
   (relay-feedback / Åström–Hägglund, see `PidAutotuner`).

## Testing 🧪
All non-trivial logic is split from hardware access behind small interfaces (a HAL),
so it runs natively on your machine — no ESP8266 board needed:
```bash
pio test -e native
```
| Module | What it covers |
|---|---|
| `VoltageSensor` / `AlternatorLogic` | Calibration math, safety thresholds, relay hysteresis, PID cycle scheduling |
| `RealTimePid` | Elapsed-time-scaled PID math, anti-windup clamping |
| `EngineDetector` | Probe-pulse engine detection state machine (probing/running/stopped), plus the running-grace window that rides out voltage sags at high RPM |
| `PwmMirror` | Duty mirrored from the alternator output onto the LED (clamping, active-low inversion) |
| `VoltageSensor` (median) | Median-of-N ADC filter that rejects alternator ripple/spikes |
| `PidAutotuner` | Relay-feedback autotune math (Ku/Pu → Kp/Ki/Kd) |
| `UsageCounters` (+ `IFlashStore`) | Charging-hours/boot-count accumulation, throttled saves, against an in-memory fake |
| `MqttPublisher` (+ `IMqttTransport`) | Sample/publish decoupling, batched payloads, offline buffering/flush, HA discovery, reconnect throttling, against a fake transport |
| `MaintenanceLog` (+ `IMaintenanceLogStore`) | Note sanitization, log round-trip, against a fake store |
| `WifiManager` (+ `IWifiDriver`) | STA/AP fallback state machine, against a fake driver - **no longer used by `main.cpp`** (which now does plain exclusive AP/STA directly), kept only because it's still covered here |

`main.cpp` wires these pure/HAL-backed decisions into the real hardware calls (PID,
PWM, relay, WiFi, flash, MQTT, GPS). When touching any of this behavior, add/extend a
native test first (TDD) — the real hardware implementations (`LittleFsStore`,
`PubSubMqttTransport`, `LittleFsMaintenanceLogStore`, `GpsReader`) are thin and
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
- Large colour-coded battery voltage, light/dark theme (follows the device), lost-connection
  and maintenance-due banners
- Real-time voltage chart (last ~10min of history) with the 13.0V/14.4V thresholds marked
- PWM%/Relay status, engine status (RUNNING/PROBING/STOPPED)
- WiFi mode (home network vs. AP) and what that means for HA/MQTT reachability
- PID autotune trigger + status
- Charging hours, power-on count, maintenance due indicator, service interval editor
- MQTT status + last-published reading + "test connection" button
- GPS card (hidden until the module has a fix - see [GPS](#gps-optional-hardware-))
- RAM used / peak loop time, configurable refresh rate, "restart device" button

**Maintenance logbook (`/maintenance`):**
- Table of logged maintenance events (date - if NTP has synced, otherwise "unknown
  date" - hours at time of service, free-text note)
- Form to log a new entry

## GPS (optional hardware) 🛰️
A serial NMEA GPS module wires directly to the ESP8266 - no browser, no HTTPS, no
per-phone permission prompt, works even with nobody's phone in range.

**Wiring:** a u-blox NEO-6M/NEO-M8N-class module (or anything else that speaks NMEA
over UART), via `SoftwareSerial`:
| ESP8266 pin | GPIO | Wire to |
|---|---|---|
| D5 | GPIO14 | GPS module TX (ESP8266 RX) |
| D6 | GPIO12 | GPS module RX (ESP8266 TX - only needed to send the module commands) |

Neither pin is a boot-strapping pin, so no conflict with normal startup (unlike D3/D4,
already used for the relay/LED - see [`GpsReader.h`](src/gps/GpsReader.h) for the full
pin rationale). Baud rate defaults to 9600 (`GPS_BAUD` in `main.cpp`), the standard for
these modules.

**Availability, not assumed:** `GpsReader` exposes two independent signals -
`isConnected()` (the module is physically present and sending *something*, fix or not)
and `hasFix()` (a recent, valid location - safe to actually display/use). The dashboard's
GPS card stays hidden until `hasFix()` is true; nothing GPS-related is shown until
there's real data to show. A cold GPS fix can take 30s to a few minutes outdoors.

**Not implemented:** on-device track recording/storage (plan: buffer points on LittleFS,
forward to a small backend once connectivity allows, mirroring the MQTT store-and-forward
pattern - see [Roadmap](#roadmap-)) and heading (no magnetometer/IMU - no feature needs
it yet).

## Home Assistant / MQTT 🏠
- WiFi is exclusive AP/STA (see [Features](#features-)) - MQTT is only reachable
  while a STA connection is up; it's offline-only while sitting on the AP.
- Set `mqtt_host` in `secrets.h` to enable; leave it empty to disable MQTT entirely.
- **Sampling and publishing are decoupled:** a reading is sampled (buffered) every
  10s (`MQTT_SAMPLE_INTERVAL_MS`) regardless of connectivity, giving good resolution
  even while offline; the buffer is actually sent every 30s (`MQTT_PUBLISH_INTERVAL_MS`),
  as one batched JSON array (capped at 10 readings per MQTT message -
  `MqttPublisher::kMaxPerBatch` - so a long outage's backlog drains over a few publish
  cycles instead of one huge message). Buffer holds up to 30 minutes of samples
  (`MqttPublisher::kMaxBuffered = 180`).
- HA MQTT discovery configs are published on (re)connect (voltage, PWM%, charging
  hours, maintenance-due, free heap, overvoltage alert). Since the state payload is
  now an array, templates read the last element: `{{ value_json[-1].voltage }}`.
- Dashboard has a "test connection" button (`/mqtt/test`, forces an immediate
  reconnect attempt) and shows the last-published reading + how long ago.
- **Known limitation:** the underlying MQTT client's `connect()` can still block
  briefly on an unreachable broker; it's throttled to at most once per ~30s and its
  socket timeout is capped at 1s to bound the worst case, but it isn't fully
  non-blocking. Acceptable given the cadence, but worth knowing.

## Engine detection & charging logic 🔌
Every PID cycle (20 ms), `manage_alternator()` first picks the voltage regime
(`decide_pwm_safety_action`), then asks which sources may declare the engine on
(`decide_engine_sources`, selected from the dashboard "Engine detection" card, persisted
in `/engine_mode.bin`; any combination, empty falls back to GPS + alternator).

| Source | Engine counts as on when |
|---|---|
| Always on | permanently (no probing) |
| GPS | valid fresh fix and mean speed of the last 15s >= 3 km/h (`SpeedAverager`) |
| Alternator | voltage in the PID band / over the cutoff, or a probe pulse shows a >= 0.05V rise |

| Voltage | Action |
|---|---|
| >= 14.4V | field **off**, overvoltage alert latched 60s; engine on if "always"/"alternator" selected |
| 13.0V - 14.4V | PID regulates to 14.0V (autotune swings output here); engine on if "always"/"alternator" selected |
| <= 13.0V, engine forced on (always, or GPS moving) | field at **100%** immediately, no probing |
| <= 13.0V, alternator source only | probe: 3s full-field pulse; rise >= 0.05V = running, else off for 8s; 5 min grace after any sign of charging so sags at high RPM don't stop the drive |
| <= 13.0V, only GPS selected and standing still | field off, engine off |

The LED mirrors the final duty written to the field pin in every case.

## System Indicators 💡
The status LED (`LED_PIN`, D4) replicates the alternator output PWM one-to-one: every
duty written to the output pin is written, with the same value, to the LED
(`OutputComponent` mirror pin, `mirror_duty()` in `PwmMirror`). LED brightness is
therefore the real field drive - off when the alternator is off or probing is in its
cooldown, full during a probe pulse/`MAX_CHARGE`, proportional while the PID runs.
`LED_ACTIVE_LOW` (default `true`, the Wemos D1 Mini on-board LED) inverts the duty so
brightness still follows the output; set it to `false` for an external active-high LED.
Use the web dashboard for WiFi/mode status.

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
- **Implemented**: hard cutoff at `VOLTAGE_THRESHOLD_HIGH` (14.4V) - the alternator is
  forced off above this regardless of PID/autotune output (`decide_pwm_safety_action`
  in `AlternatorLogic`, covered by native tests). Autotune swings output but is still
  subject to this same cutoff every cycle, never bypasses it. The alert latches for
  60s (`OVERVOLTAGE_ALERT_HOLD_MS`) after the last trigger, shown on the dashboard and
  published via MQTT/HA, so a brief spike doesn't go unnoticed even if it clears before
  the next publish cycle.
- **Not implemented** (aspirational, don't rely on these): PWM rate limiting,
  thermal shutdown (no temperature sensor wired up), watchdog timer. Treat the
  hardware notes below as mandatory regardless.

**Critical Notes**:
- 🔥 Always use appropriately rated components
- 🛑 Double-check polarity before power-on
- 🔋 Maintain battery temperature monitoring
- 🧯 Enclose in IP67-rated waterproof case

## Persistence design notes 💾
- **Usage counters** (`UsageCounters`, `/usage.bin`): saved every 60s of *accumulated
  active charging time* (not wall-clock, not every loop). Worst case on an abrupt power
  cut (this tractor has no shutdown signal) is losing the last <60s of hours - harmless
  for a maintenance counter, since it only ever undercounts. At this cadence, even
  continuous charging stays far under typical NOR flash erase-cycle budgets over the
  unit's lifetime; LittleFS also wear-levels across its partition rather than hammering
  one sector. Boot count is saved once per real power-on (rare, unthrottled).
- **Maintenance logbook** (`MaintenanceLog`, `/maintenance_log.csv`): append-only,
  written only when a user logs a real service event (a handful of times a year at
  most) - flash wear is a non-issue here regardless of the counters' cadence above.

## Roadmap 🗺️
- **MQTT `connect()` blocking caveat** - see [Home Assistant / MQTT](#home-assistant--mqtt-)
  above; a fully non-blocking MQTT client would need a custom async TCP state machine.
- **GPS track storage** - no on-device recording yet. Plan: buffer points on LittleFS,
  forward to a small backend (Ruby, under consideration) once STA connectivity is up,
  same store-and-forward shape as `MqttPublisher`.
- **Heading** - no magnetometer/IMU. Only worth adding once a concrete feature needs
  it (e.g. straight-line driving guidance).
- **Heap fragmentation over long uptime** - `ESP.getHeapFragmentation()` can climb
  high enough after many hours of runtime to break the dashboard even with healthy
  total free heap (largest contiguous block is the real constraint). `/restart` +
  the dashboard button work around it; an automatic periodic restart is a candidate
  fix, not yet implemented.
## License 📄
MIT License - See [LICENSE](LICENSE) for details
*PID Library:* BSD 3-Clause (included in dependencies)