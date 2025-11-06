# TohrAirTag - ESP32-C3 Bluetooth AirTag Tracker

> **Advanced DIY Personal Item Tracker** with BLE proximity detection, motion sensing, and smart audio/visual feedback

[![Hardware: ESP32-C3](https://img.shields.io/badge/Hardware-ESP32--C3-blue)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/)
[![Language: C/C++](https://img.shields.io/badge/Language-C%2FC%2B%2B-brightgreen)](https://en.wikipedia.org/wiki/C%2B%2B)
[![BLE: Bluetooth 5.0](https://img.shields.io/badge/BLE-Bluetooth%205.0-blue)](https://www.bluetooth.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Components](#hardware-components)
- [Pin Configuration](#pin-configuration)
- [Installation & Setup](#installation--setup)
- [Quick Start](#quick-start)
- [Code Architecture](#code-architecture)
- [Configuration Guide](#configuration-guide)
- [Operating Modes](#operating-modes)
- [Audio/Visual Feedback Patterns](#audiovisual-feedback-patterns)
- [RSSI Proximity Detection](#rssi-proximity-detection)
- [Troubleshooting](#troubleshooting)
- [Performance Specifications](#performance-specifications)
- [Future Roadmap](#future-roadmap)
- [Contributing](#contributing)

---

## 🎯 Overview

**TohrAirTag** is a DIY Bluetooth Low Energy proximity tracker built on the ultra-compact **ESP32-C3 Super Mini** microcontroller. It functions similarly to Apple AirTag or Tile devices, helping you find lost items by measuring Bluetooth signal strength (RSSI) and providing proximity alerts through audio (buzzer), visual (LED), and digital (serial) feedback.

### Use Cases
- Track personal items (keys, wallet, bag, etc.)
- Find lost devices within Bluetooth range
- Alert when item moves out of usable range
- Monitor device movement via motion sensor
- Educational platform for BLE and embedded systems

---

## ✨ Features

### Core Functionality
- **BLE Proximity Detection** - Measures RSSI signal strength for distance estimation
- **Smart Proximity States** - NEAR (in range) and FAR (out of range) with hysteresis filtering
- **Hardware Motion Detection** - MPU6050 6-axis IMU with interrupt-based detection
- **Dual Feedback System** - Audio (buzzer) + Visual (LED) + Serial output
- **Auto Reconnection** - Continuous advertising for instant reconnection
- **RSSI Averaging** - 3-sample rolling average eliminates noise

### Audio Patterns
- ✓ Startup sequence (3 distinct beeps)
- ✓ Connection confirmation (2 beeps)
- ✓ NEAR state alert (1 single beep)
- ✓ FAR state alert (continuous pulse)
- ✓ Disconnection feedback (1 sad beep)

### Visual Indicators
- ✓ Startup flash sequence (4 flashes)
- ✓ Connected = Solid ON
- ✓ Connected & NEAR = LED ON
- ✓ Connected & FAR = Rapid pulse
- ✓ Disconnected = Slow blink

---

## 🔧 Hardware Components

### Required Components

| Component | Model | Pin Connection | Purpose |
|-----------|-------|-----------------|---------|
| **Microcontroller** | ESP32-C3 Super Mini | - | Main processor |
| **Motion Sensor** | MPU-6050 IMU | GPIO4 (SCL), GPIO5 (SDA), GPIO3 (INT) | Detects device movement |
| **Buzzer** | Active Piezo Buzzer | GPIO2 (via NPN transistor) | Audio feedback |
| **LED** | Built-in Blue LED | GPIO8 (active LOW) | Visual status |
| **Resistor** | 2.6kΩ | Base of NPN transistor | Buzzer driver circuit |
| **Transistor** | BC547 (NPN) | Buzzer driver | Controls buzzer power |
| **Power** | USB-C or Battery | 3.3V/5V | Device power |

### Hardware Diagram

```
ESP32-C3 Super Mini
┌───────────────────────────┐
│      GPIO8 (LED)  ◇ Blue   │  Active LOW (LOW=ON, HIGH=OFF)
│      GPIO2 (Buzzer)        │  Control transistor base
│      GPIO3 (MPU INT)       │  Motion interrupt
│      GPIO4 (I2C SCL) ──────┼─→ MPU6050 SCL
│      GPIO5 (I2C SDA) ──────┼─→ MPU6050 SDA
└───────────────────────────┘
         │
    ┌─────┴────────┐
    │   BUZZER     │
   (+)    ◇       (-)
    │    NPN       │
    │   BC547      │
    └──(base)──────┼─→ 2.6kΩ resistor to GPIO2
                   │
              GND (0V)
```

---

## 📍 Pin Configuration

### GPIO Pinout

```c
#define BLUE_LED_PIN       8      // Built-in LED (active LOW)
#define BUZZER_PIN         2      // Buzzer via NPN transistor
#define BOOT_BUTTON_PIN    9      // Built-in BOOT button
#define MPU_INT_PIN        3      // MPU6050 interrupt
#define MPU_SDA_PIN        5      // MPU6050 I2C data
#define MPU_SCL_PIN        4      // MPU6050 I2C clock
#define MPU_ADDR           0x68   // MPU6050 I2C address
```

### MPU6050 I2C Connection

```
MPU6050 Pin          → ESP32-C3 Pin
─────────────────────────────────
VCC                  → 3.3V
GND                  → GND
SCL (I2C Clock)      → GPIO4
SDA (I2C Data)       → GPIO5
INT (Interrupt)      → GPIO3
```

---

## 📦 Installation & Setup

### Prerequisites
- ESP32-C3 Super Mini board
- Arduino IDE or PlatformIO
- USB-C cable for programming
- BLE-capable smartphone or device

### Step 1: Install ESP32-C3 Board in Arduino IDE

1. Open Arduino IDE → **File** → **Preferences**
2. Add board manager URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Go to **Tools** → **Board Manager** → Search "ESP32"
4. Install "esp32 by Espressif Systems"
5. Select **Tools** → **Board** → **ESP32-C3 (choose your variant)**

### Step 2: Install Required Libraries

In Arduino IDE → **Sketch** → **Include Library** → **Manage Libraries**, install:

- `BLEDevice` (by Espressif) - Already included with ESP32 core
- `Wire` - I2C communication (built-in)
- `Preferences` - EEPROM storage (built-in)

### Step 3: Upload Code

1. Connect ESP32-C3 to computer via USB-C
2. Select correct **Board** and **COM Port** in Arduino IDE
3. Copy the firmware code into IDE
4. Click **Upload** (Ctrl+U)
5. Monitor serial output to verify initialization

### Step 4: Verify Installation

```
Open Serial Monitor (Tools → Serial Monitor)
Set baud rate to 115200
Press RST button on device

Expected output:
╔════════════════════════════════════════════════════════╗
║  ESP32-C3 Advanced AirTag Tracker                     ║
║  v2.0 - FIXED Deep Sleep & Stable BLE                 ║
╚════════════════════════════════════════════════════════╝

[BOOT] Device starting...
[BLE] ✓ BLE initialized and advertising
[BOOT] ✓ Initialization complete. Waiting for connection...
```

---

## 🚀 Quick Start

### Minimal Setup (5 minutes)

1. **Hardware Assembly**
   - Connect MPU6050: SCL→GPIO4, SDA→GPIO5, INT→GPIO3
   - Connect Buzzer: Signal→GPIO2 (via transistor base), GND
   - LED is built-in on GPIO8
   - Power via USB-C

2. **Upload Firmware**
   - Compile and upload code to ESP32-C3
   - Wait for "Initialization complete" message in Serial Monitor

3. **Connect via BLE**
   - Open BLE app on phone (e.g., nRF Connect)
   - Scan for "TohrAirTag"
   - Connect → hear 2 beeps, LED turns ON

4. **Test Proximity Detection**
   - Move phone away from device (5+ meters)
   - At ~6 meters: buzzer starts, LED pulses
   - Move back close: 1 beep, LED solid, buzzer stops

5. **Monitor Serial Output**
   - Open Serial Monitor (115200 baud)
   - See real-time RSSI values and state changes

---

## 🏗️ Code Architecture

The firmware is organized into key functional segments:

### 1. **Initialization Phase** (`setup()`)
```
GPIO Setup
  ↓
I2C Initialization
  ↓
MPU6050 Configuration
  ↓
Startup Feedback (3 beeps + LED flash)
  ↓
BLE Setup (advertise)
  ↓
Ready for connection
```

### 2. **Main Loop** (`loop()`)
```
Check BLE connection state
  ↓
Read MPU6050 motion interrupt
  ↓
Scan for RSSI (if connected)
  ↓
Process RSSI averaging & proximity
  ↓
Update LED status
  ↓
Log status every 5 seconds
```

### 3. **Functional Modules**

| Module | Purpose | Key Functions |
|--------|---------|---------------|
| **MPU6050 I2C** | Motion detection | `writeMPURegister()`, `readMPUAccel()` |
| **Buzzer Control** | Audio feedback | `beep()`, `beepStartup()`, `beepPairing()` |
| **LED Control** | Visual status | `ledOn()`, `ledBlinkFast()`, `ledBlinkSlow()` |
| **RSSI Processing** | Distance estimation | `pushRSSI()`, `getAverageRSSI()`, `updateProximity()` |
| **BLE Server** | Bluetooth communication | `setupBLE()`, `ServerCallbacks` class |

---

## ⚙️ Configuration Guide

### Adjustable Parameters in Code

#### 1. **RSSI Proximity Thresholds**
```cpp
const int RSSI_NEAR_THRESHOLD = -68;    // Signal strength for "NEAR"
const int RSSI_HYSTERESIS = 3;          // ±3 dB buffer (prevent flipping)
const int RSSI_AVG_WINDOW = 3;          // Average last 3 samples
const int SCAN_SECONDS = 1;             // Check RSSI every 1 second
```

**How to adjust:**
- Decrease `RSSI_NEAR_THRESHOLD` (e.g., -75) for larger tracking range
- Increase `RSSI_HYSTERESIS` (e.g., 5) to prevent false state changes
- Increase `RSSI_AVG_WINDOW` (e.g., 5) for smoother but slower response

#### 2. **Motion Detection Sensitivity**
```cpp
const uint8_t MOTION_THRESHOLD = 15;    // ~15 mg acceleration
const uint8_t MOTION_DURATION = 1;      // 10 ms duration
```

**How to adjust:**
- Decrease `MOTION_THRESHOLD` for more sensitive motion detection
- Increase for less sensitive (ignore vibrations)

#### 3. **Timing Parameters**
```cpp
const unsigned long IDLE_TIMEOUT_MS = 180000;         // 3 minutes (for future deep sleep)
const unsigned long LED_BLINK_FAST = 150;             // 150 ms (disconnected)
const unsigned long LED_BLINK_SLOW = 500;             // 500 ms (searching)
const unsigned long BLE_RECONNECT_DELAY = 2000;       // 2 second wait before re-advertise
```

#### 4. **BLE Advertising**
```cpp
BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);   // Max power (+9 dBm)
pScan->setInterval(30);                                       // 30 ms scan interval
pScan->setWindow(30);                                         // 30 ms scan window
```

---

## 🎮 Operating Modes

### Mode 1: Disconnected (Searching)
```
State:   Not connected to phone
LED:     Fast blink (150 ms cycle)
Buzzer:  Silent
RSSI:    Not measured
Action:  Broadcasting, waiting for connection
```

### Mode 2: Connected & NEAR (In Range)
```
State:   Phone connected & signal strong
LED:     Solid ON (continuous)
Buzzer:  Silent (happy state)
RSSI:    ≥ -65 dBm
Action:  Continuous RSSI monitoring, no alerts
```

### Mode 3: Connected & FAR (Out of Range)
```
State:   Phone connected but signal weak
LED:     Rapid pulse (150 ms cycle)
Buzzer:  Continuous pulse (alert mode)
RSSI:    ≤ -75 dBm
Action:  Continuous alerts, user should locate device
```

### Mode 4: Disconnected (After being connected)
```
State:   Lost BLE connection
LED:     Slow blink (500 ms cycle)
Buzzer:  Single sad beep (immediate feedback)
RSSI:    Unknown
Action:  Auto re-advertise after 2 seconds
```

---

## 🔊 Audio/Visual Feedback Patterns

### Startup Sequence
- **LED:** 4 rapid flashes (100ms each)
- **Buzzer:** 3 beeps (200ms + 200ms + 300ms with 150ms pauses)
- **Duration:** ~2 seconds total
- **Meaning:** Device initialized and ready

### Connection Established
- **LED:** Immediate solid ON
- **Buzzer:** 2 quick beeps (150ms each, 100ms pause)
- **Meaning:** Phone successfully connected, proximity monitoring active

### Proximity State: NEAR
- **LED:** Solid ON (no change)
- **Buzzer:** SILENT
- **Meaning:** Phone is close, everything is good

### Proximity State: FAR
- **LED:** Rapid pulse (150ms ON/OFF)
- **Buzzer:** Continuous pulse (150ms ON/OFF)
- **Meaning:** Phone is far away, alert user!

### Back to NEAR (State Recovery)
- **LED:** Solid ON (from pulse)
- **Buzzer:** 1 confirmation beep (120ms)
- **Meaning:** Phone came back into range

### Disconnection
- **LED:** Slow blink (500ms ON/OFF)
- **Buzzer:** Single sad beep (300ms)
- **Meaning:** Lost connection, will auto-reconnect

---

## 📡 RSSI Proximity Detection

### How RSSI Works

**RSSI** = Received Signal Strength Indicator (in dBm)

| RSSI Range | Distance | Description |
|-----------|----------|-------------|
| -30 to -50 | 0.5-2m | Very close (same room, line-of-sight) |
| -50 to -65 | 2-8m | **NEAR range** (normal use) |
| -65 to -80 | 8-15m | Boundary zone (hysteresis applies) |
| -80 to -100 | 15-30m | **FAR range** (out of tracking) |
| < -100 | >30m | Very weak or blocked signal |

### Threshold Logic with Hysteresis

```
State Machine:
╔═══════════════════════════════════════════════════════╗
║  NEAR State                                           ║
║  RSSI ≥ -65 dBm (THRESHOLD - HYSTERESIS)             ║
║  LED: ON, Buzzer: OFF                                ║
╚═══════════════════════════════════════════════════════╝
        ↓ (RSSI drops below -75)
┌────────────────────────────────────────────────────┐
│  HYSTERESIS ZONE: -65 to -75 dBm                   │
│  State remains unchanged (prevents flipping)        │
└────────────────────────────────────────────────────┘
        ↓
╔═══════════════════════════════════════════════════════╗
║  FAR State                                            ║
║  RSSI ≤ -75 dBm (THRESHOLD + HYSTERESIS)             ║
║  LED: PULSE, Buzzer: ON                              ║
╚═══════════════════════════════════════════════════════╝
        ↓ (RSSI rises above -65)
┌────────────────────────────────────────────────────┐
│  HYSTERESIS ZONE: -75 to -65 dBm                   │
│  State remains unchanged (prevents flipping)        │
└────────────────────────────────────────────────────┘
        ↓
        Back to NEAR State
```

### Averaging Algorithm

```cpp
// Stores last 3 RSSI readings
int rssiBuf[3];

// Push new RSSI sample
pushRSSI(currentRSSI);

// Calculate average of all samples in buffer
int avgRSSI = getAverageRSSI();  
// = (sample1 + sample2 + sample3) / 3
```

**Benefits:**
- Smooths wireless signal fluctuations
- Reduces false state transitions
- 3-sample window updates every ~3 seconds

---

## 🔍 Troubleshooting

### Problem: Device won't boot

**Symptoms:** No serial output, LED doesn't flash

**Solutions:**
1. Check USB cable (must support data transfer, not just power)
2. Try different USB port on computer
3. Press **RST** button on ESP32-C3
4. Try different USB cable (charging-only cables won't work)
5. Verify correct board selected in Arduino IDE

---

### Problem: Can't find "TohrAirTag" via BLE scan

**Symptoms:** BLE app shows no device

**Solutions:**
1. Check Serial Monitor - should show "BLE initialized and advertising"
2. Verify BLE is enabled on your phone
3. Try different BLE app (nRF Connect, BLE Scanner)
4. Restart ESP32-C3 (press RST button)
5. Check if phone is in Airplane Mode

---

### Problem: Device keeps disconnecting

**Symptoms:** Connects briefly, then drops

**Solutions:**
1. Check phone distance (within 10 meters)
2. Check for WiFi interference (same 2.4 GHz band)
3. Try moving away from WiFi router
4. Reduce distance between phone and device
5. Check USB power (unstable power causes disconnects)
6. Verify wire connections are secure

---

### Problem: Buzzer doesn't work

**Symptoms:** No sound, but serial shows "beeping"

**Solutions:**
1. Verify GPIO2 connections
2. Check transistor connections:
   - Base → GPIO2 (through 2.6kΩ resistor)
   - Collector → Buzzer negative
   - Emitter → GND
3. Test buzzer directly with 3.3V (should beep immediately)
4. Check if transistor is burned out (replace with BC547)
5. Verify 3.3V supply at buzzer positive terminal

---

### Problem: LED doesn't turn on

**Symptoms:** LED stays dark

**Solutions:**
1. Remember: GPIO8 is **active LOW** (LOW=ON, HIGH=OFF)
2. Check LED brightness by looking in dark room
3. Test LED directly with battery
4. Try different LED (might be burned out)
5. Check if GPIO8 is correctly configured as OUTPUT

---

### Problem: Motion detection not working

**Symptoms:** No "Motion Detected" messages in Serial

**Solutions:**
1. Check I2C connections:
   - SCL on GPIO4
   - SDA on GPIO5
   - INT on GPIO3
2. Serial should show "[MPU] ✓ Motion interrupt configured"
3. Try physically shaking device hard
4. Reduce `MOTION_THRESHOLD` (e.g., from 15 to 10) for more sensitivity
5. Check if MPU6050 is detected on startup

---

### Problem: RSSI values are always -127 or stay same

**Symptoms:** Proximity detection not working, no RSSI changes

**Solutions:**
1. Ensure device is **connected** first (2 beeps confirm)
2. Check if phone is discoverable
3. Try different phone/device for scanning
4. Check distance between phone and device
5. Verify BLE connection is active (LED should be ON or pulsing)

---

### Problem: False proximity state changes

**Symptoms:** LED/buzzer keeps flipping between states rapidly

**Solutions:**
1. Increase `RSSI_HYSTERESIS` from 3 to 5 dB
2. Increase `RSSI_AVG_WINDOW` from 3 to 5 samples
3. Move away from WiFi interference
4. Check USB power stability
5. Reduce movement to stabilize RSSI readings

---

## 📊 Performance Specifications

### Technical Metrics

| Parameter | Value | Notes |
|-----------|-------|-------|
| **BLE Range** | 100+ meters | Line-of-sight, open space |
| **Indoor Range** | 20-30 meters | Through walls/obstruction |
| **Proximity Detection Range** | 5-15 meters | Configurable |
| **RSSI Update Frequency** | 1 second | Continuous scan |
| **Proximity Response Time** | 3-5 seconds | Due to 3-sample averaging |
| **BLE Connection Latency** | <100 ms | Immediate connection |
| **Auto-Reconnect Time** | <2 seconds | Advertising never stops |
| **Motion Detection Latency** | ~100 ms | Hardware interrupt |
| **Startup Time** | ~2 seconds | Full initialization |
| **Memory Usage** | ~45% RAM | 180 KB of 400 KB |
| **Boot Message Rate** | Every 5 seconds | Status updates |

---

### Power Consumption (Typical)

| Mode | Current | Duration | Use Case |
|------|---------|----------|----------|
| **Active (USB)** | 50-100 mA | Continuous | Normal operation |
| **Idle (USB)** | 20-30 mA | When disconnected |
| **Deep Sleep** | <100 µA | Future feature | Extended battery life |
| **Motion Detected** | ~100 mA | Momentary | Interrupt handling |

---

## 🗺️ Future Roadmap

### Phase 2: Enhanced Features
- [ ] Deep sleep mode (<100 µA idle consumption)
- [ ] Battery voltage monitoring & low battery alerts
- [ ] Configurable thresholds via smartphone app
- [ ] BLE notification characteristics (push alerts to phone)
- [ ] Motion-triggered wake from deep sleep
- [ ] Multiple device tracking (pair with multiple phones)
- [ ] Find-by-sound (press "Find Me" button on phone)

### Phase 3: Advanced Features
- [ ] Cloud integration (track across networks)
- [ ] GPS backup for outdoor tracking
- [ ] Temperature & humidity sensors
- [ ] Tamper detection (case opened)
- [ ] Cryptographic identification (anti-spoofing)
- [ ] Indoor localization via BLE triangulation
- [ ] Machine learning for better distance estimation

### Phase 4: Commercial
- [ ] Custom PCB design & manufacturing
- [ ] Enclosure design (waterproof case)
- [ ] FCC certification
- [ ] Companion mobile app
- [ ] Battery optimization (3+ days on single charge)

---

## 🤝 Contributing

Contributions are welcome! Here's how to help:

1. **Report Issues** - Found a bug? Open an issue with details
2. **Feature Requests** - Have ideas? Suggest improvements
3. **Code Improvements** - Submit pull requests with optimizations
4. **Documentation** - Help improve guides and comments
5. **Testing** - Test on different devices and report results

### Development Guidelines
- Follow existing code style and formatting
- Add comments for complex logic
- Test thoroughly before submitting PR
- Update README if adding new features
- Document any new configuration parameters

---

## 📝 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Espressif** - ESP32-C3 hardware and Arduino core
- **Bluetooth SIG** - BLE specifications
- **Arduino Community** - Libraries and support
- **Contributors** - For improvements and testing

---

## 📞 Support & Contact

- **Issues:** Open an issue on GitHub
- **Discussions:** Use GitHub Discussions for general questions
- **Documentation:** Check this README and inline code comments
- **Serial Debug:** Enable Serial Monitor at 115200 baud for troubleshooting

---

## 🎓 Educational Value

This project demonstrates:
- **Embedded Systems**: Real-time task scheduling, interrupt handling
- **Wireless Communication**: BLE protocol, RSSI measurement, advertising
- **Sensor Integration**: I2C communication, motion detection (MPU6050)
- **Firmware Design**: State machines, signal processing, user feedback
- **Power Management**: GPIO control, peripheral configuration
- **Debugging**: Serial monitoring, protocol analysis

Perfect for students learning electronics, embedded systems, or IoT development!

---

**Enjoy building and tracking! 🚀**

---

*Last Updated: November 6, 2025*  
*Firmware Version: v2.0 (Stable & Fixed)*  
*Device: ESP32-C3 Super Mini*
