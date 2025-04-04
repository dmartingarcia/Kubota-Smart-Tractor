# Smart Tractor Alternator Controller 🚜⚡

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ESP8266-based intelligent charging system with adaptive PID control and web monitoring. Maintains optimal battery health using advanced charge management.

## Features ✨
- 🎮 **Adaptive PID Control** - Maintains precise voltage regulation
- ⚡ **Dual Output Modes** - Relay or PWM MOSFET control (configurable)
- 📶 **WiFi Web Interface** - Real-time monitoring & historical charts
- 🔄 **OTA Updates** - Wireless firmware upgrades
- 📊 **Advanced Telemetry** - Voltage, PWM%, PID output, and engine status
- 🔒 **Multi-layer Protection** - Over-voltage cutoff, rate limiting, and thermal safety

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
   git clone https://github.com/yourrepo/smart-tractor-charger.git
   cd smart-tractor-charger
   ```
3. Install dependencies:
   ```bash
   pio pkg install
   ```

### Configuration
1. Create `include/secrets.h`:
   ```cpp
   const char* ssid = "YOUR_SSID";
   const char* password = "YOUR_PASSWORD";
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

## Web Interface 🌐
Access via `http://smarttractor.local` or device IP:


**Features:**
- Real-time voltage graph (60s history)
- PID output visualization
- PWM %/Relay status indicator
- Engine running detection status
- System mode (PWM/Relay) display

## System Indicators 💡
| LED Pattern            | System State               |
|------------------------|----------------------------|
| Slow Blink (0.2Hz)     | Alternator OFF             |
| Rapid Blink (1Hz)      | PWM Active                 |
| Solid ON (2Hz)         | Relay Active               |
| Double Flash (0.5Hz)   | WiFi Connection Failed     |

## OTA Updates 🛠️
1. Connect to device WiFi
2. In PlatformIO:
   ```bash
   pio run --target upload --upload-port smarttractor.local
   ```
3. Monitor serial output for progress

## Troubleshooting 🔍
| Issue                  | Solution                      |
|------------------------|-------------------------------|
| No PWM output          | Verify MOSFET wiring & PWM config |
| Voltage oscillations   | Reduce PID P value            |
| Web interface offline  | Check mDNS & server.begin()   |
| Overheating MOSFET     | Add heatsink & verify current |

## Safety Systems ⚠️
- **Multi-stage Protection**:
  - Hard-cutoff at 15.0V (absolute maximum)
  - PWM rate limiting (max 10%/s change)
  - Thermal shutdown (requires temp sensor)
  - Watchdog timer (auto-reset on lockup)

**Critical Notes**:
- 🔥 Always use appropriately rated components
- 🛑 Double-check polarity before power-on
- 🔋 Maintain battery temperature monitoring
- 🧯 Enclose in IP67-rated waterproof case

## License 📄
MIT License - See [LICENSE](LICENSE) for details
*PID Library:* BSD 3-Clause (included in dependencies)