# Smart Tractor Alternator Controller 🚜⚡

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ESP8266-based system to automatically control tractor alternator charging based on battery voltage monitoring. Prevents overcharging while maintaining optimal battery levels.

## Features ✨
- 🎚️ Dual-threshold hysteresis control (14.0V-14.8V)
- 📶 WiFi connectivity for OTA updates
- 🔋 Real-time battery voltage monitoring
- 🔒 Relay protection with activation delay
- 💡 Visual status LED indicators
- 📊 Moving average noise reduction

## Hardware Requirements 🔨
| Component              | Specification           |
|------------------------|-------------------------|
| Microcontroller        | ESP8266 (NodeMCU/Wemos) |
| Voltage Divider        | R1=220kΩ, R2=68kΩ       |
| Relay                  | 5V SPDT Relay           |
| Power Input            | 12V Tractor Battery     |

**Wiring Diagram:**

Battery+ ----[220k]---- A0 ----[68k]---- GND  
Relay COM ---- Alternator Field Circuit  
NO/NC Contacts ---- Configured for fail-safe operation  

## Installation 📥

1. **Arduino IDE Setup**
   Board Manager URL: http://arduino.esp8266.com/stable/package_esp8266com_index.json
   Install ESP8266 platform (2.7.4+)

### Library Dependencies

All of them are detailed in the `platformio.ini`

- *ESP8266WiFi*
- *ESP8266mDNS*
- *ArduinoOTA*

## Configuration

Create secrets.h with:
```cpp
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
```
Set calibration values in main.cpp

`Verify ALTERNATOR_ACTIVE_STATE matches relay logic`

    
## Calibration Guide 🔧

Initial Setup:
1. Connect fully charged battery (12.6V+)
2. Upload initial code with default calibration values
3. Monitor serial output at 9600 baud

### Voltage Adjustment

1. Measure actual battery voltage with multimeter (V_true)
2. Note serial output voltage (V_reading)
3. Update calibration constants:
       CALIBRATION_IN_VOLTAGE = V_true
       CALIBRATION_A0_VOLTAGE = V_reading

#### Threshold Testing
   
Use variable power supply to verify:
- Relay opens at 14.8V ±0.2V
- Relay closes at 14.0V ±0.2V

## System Behavior Indicators 💡
- LED Pattern	System State
- Slow Blink (1Hz)	Alternator OFF
- Solid ON	Alternator Active
- Rapid Blink (5Hz)	WiFi Connection Failed

# OTA Update Instructions 🛠️

Connect to device WiFi AP (if configured)

```bash
Tools > Port: "smarttractor.local"
Sketch > Export Compiled Binary
Sketch > Upload Over Network
```

# Troubleshooting 🔍

Issue	Solution
No relay activation	Verify ALTERNATOR_ACTIVE_STATE logic
Inaccurate readings	Check voltage divider resistors
WiFi connection fails	Confirm 2.4GHz network credentials
OTA not working	Ensure mDNS responder active

# Safety Notes ⚠️

Batteries contains acid, and overvoltage could cause damages to the batteries and they can also explode, so, **be kind with the calibration and settings and double/triple check that it works as expected**
    
🔥 Always fuse battery connections

🛑 Use automotive-rated relay for high current paths

🔌 Double-check polarity before connecting

🧯 Keep ESP8266 in waterproof enclosure

# License 📄

MIT License - See LICENSE for details

