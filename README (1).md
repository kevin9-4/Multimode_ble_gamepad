# 🎮 GamePad V2 — ESP32 BLE Multi-Mode Controller

> A custom dual-mode Bluetooth gamepad built on ESP32 with a gyroscope, OLED UI, and support for both Generic HID and Xbox Series X HID profiles.

---

## 📸 Overview

GamePad V2 is the 3rd hardware prototype of a fully handmade ESP32-based Bluetooth controller. It supports two independent BLE HID modes — a **Generic Gamepad** (works on Windows ) and an **Xbox Series X** controller profile (works natively on Windows and Xbox Cloud Gaming) — switchable from an on-device OLED menu without any software tools.

**Built over 6 months | Firmware v1.0 | Author: Kevin Mistry**

---

## ✨ Features

- **Dual BLE HID Mode** — Generic Gamepad HID + Xbox Series X HID, each with a separate Bluetooth MAC address so they pair independently
- **Gyroscope Aiming** — MPU-6050 gyro blended into the right stick for motion aiming; toggle on/off mid-game with a single button
- **Dual Analog Joysticks** — with deadzone, center calibration, and 3-level sensitivity (Low / Default / High)
- **Full D-Pad** — 8 directions including all four diagonals
- **14 Buttons** — face buttons, bumpers, digital triggers (with analog ramp in Xbox mode), Start, Back, L3, R3
- **OLED UI** — scrollable menus for mode selection, settings, and live sensor readout (Developer Mode)
- **Adjustable Brightness** — 10-level OLED brightness control
- **Adjustable Gyro Sensitivity** — 3 levels (Low / Default / High)
- **Deep Sleep** — press Sleep in settings; wakes on MODE button press
- **NVS Boot Persistence** — selected mode is saved to flash and survives power cycles
- **Quick Reboot** — mode switching skips the 8-second boot splash on reboot

---

## 🔧 Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32 (38-pin, dual-core 240 MHz) |
| Display | SSD1306 OLED 128×64 (I2C) |
| IMU | MPU-6050 6-axis IMU (I2C) |
| Joysticks | 2× Analog thumbstick modules |
| Buttons | 14× digital buttons (active-LOW with pull-ups) |
| Power | LiPo / USB (3.3V regulated) |

### Pin Map

| Function | GPIO |
|----------|------|
| Left Stick X | 34 |
| Left Stick Y | 35 |
| Left Stick Button (L3) | 15 |
| Right Stick X | 39 |
| Right Stick Y | 36 |
| Right Stick Button (R3) | 32 |
| D-Pad Up | 5 |
| D-Pad Down | 16 |
| D-Pad Right | 17 |
| D-Pad Left | 4 |
| Button X (Yellow) | 3 |
| Button Y (Blue) | 1 |
| Button A (Red) | 19 |
| Button B (White) | 18 |
| Left Trigger (LT) | 27 |
| Left Bumper (LB) | 26 |
| Right Trigger (RT) | 33 |
| Right Bumper (RB) | 25 |
| Mode Button | 14 |
| Select / Gyro Toggle | 23 |
| Start | 12 |
| Back | 13 |
| I2C SDA (OLED + MPU) | 21 |
| I2C SCL (OLED + MPU) | 22 |

---

## 🛠️ Software Requirements

### Arduino IDE / ESP-IDF

| Tool | Version |
|------|---------|
| Arduino IDE | 2.x recommended |
| **ESP32 Arduino Core** | **2.0.14** ⚠️ (see note below) |

> ⚠️ **Important:** This firmware is developed and tested against **ESP32 Arduino Core 2.0.14**. Using a newer core (3.x) may break BLE HID functionality due to breaking changes in the NimBLE / Arduino BLE stack used by BleCompositeHID. Stick to **2.0.14** until the library is updated.

To install the correct core version in Arduino IDE:
1. Go to **Tools → Board → Boards Manager**
2. Search for `esp32` by Espressif
3. Select version **2.0.14** and install

---

### Required Libraries

Install all libraries via **Arduino IDE → Sketch → Include Library → Manage Libraries** or the links below.

| Library | Version Tested | Install Method | Link |
|---------|---------------|----------------|------|
| **ESP32-BLE-CompositeHID** | Latest (main branch) | GitHub ZIP / Manual | [GitHub](https://github.com/Mystfit/ESP32-BLE-CompositeHID) |
| **Adafruit SSD1306** | 2.5.x | Library Manager | [GitHub](https://github.com/adafruit/Adafruit_SSD1306) |
| **Adafruit GFX Library** | 1.11.x | Library Manager | [GitHub](https://github.com/adafruit/Adafruit-GFX-Library) |
| **MPU6050_WE** | 1.1.x | Library Manager | [GitHub](https://github.com/wollewald/MPU6050_WE) |
| **Preferences** | Built-in | ESP32 Core built-in | — |
| **Wire** | Built-in | ESP32 Core built-in | — |

> ⚠️ **BleCompositeHID is NOT on the Arduino Library Manager.** You must download it manually from GitHub and place it in your Arduino `libraries/` folder. It also pulls in its own dependencies — check its README for the exact dependency list.

---

## 🚀 Getting Started

### 1. Clone / Download

```bash
git clone https://github.com/kevin9-4/Multimode_ble_gamepad
```

### 2. Install Dependencies

- Install ESP32 core **2.0.14** in Boards Manager
- Install all libraries listed in the table above
- Download **BleCompositeHID** from GitHub and copy to your `Arduino/libraries/` folder

### 3. Open and Flash

- Open `GamePad_V2.ino` in Arduino IDE
- Select your ESP32 board (e.g. **ESP32 Dev Module**)
- Set **Partition Scheme** to `Minimal SPIFFS (1.9MB APP)` or larger for BLE headroom
- Upload

### 4. Pair

- On first boot, the gamepad advertises as `GamePad V2 GP` (Gamepad mode) or `GamePad V2 XBOX` (Xbox mode)
- Open Bluetooth settings on your host and pair normally
- The two modes have **separate Bluetooth MACs** so they can be paired to the same host independently

---

## 🎛️ Using the Controller

### OLED Menu Navigation

| Button | Action |
|--------|--------|
| D-Pad Up / Down | Navigate menu |
| A (Red) | Select / Confirm |
| D-Pad Left | Back / Exit submenu |
| MODE button | Exit active mode → return to menu |

### Main Menu Options

| Option | Description |
|--------|-------------|
| GamePad | Generic HID mode (Windows / Android / Linux) |
| XBOX\_X | Xbox Series X HID mode |
| Developer Mode | Live ADC + gyro readout for debugging / calibration |
| Settings | Brightness, sensitivity, gyro sensitivity, sleep |
| About | Credits |

### Gyro Aiming

- In active mode, press **SELECT** to toggle gyro on/off
- Gyro values from the MPU-6050 are added to the right stick axis
- Adjust sensitivity in **Settings → Gyro Sensitivity** (Low / Default / High)

### Xbox HOME Button

In Xbox mode, press **BACK + START** simultaneously to trigger the Xbox HOME/Guide button.

### Mode Switching

Selecting a mode that differs from the current boot mode will save the new mode to flash and reboot automatically. The device comes back up in the new mode within a few seconds (splash screen is skipped on mode-switch reboots).

---

## ⚙️ Calibration

The joystick center offsets are hardcoded based on the specific modules used:

```cpp
const int joy1xcenter = 1880;
const int joy1ycenter = 1830;
const int joy2center  = 1890;
```

If your joysticks drift at rest, read the raw ADC values in **Developer Mode** and update these constants to match your hardware.

Gyro offsets are auto-calibrated at every boot via `gyro.autoOffsets()` — keep the controller flat and still for the first 2 seconds after powering on.

---

## 📁 File Structure

```
GamePad-V2/
├── GamePad_V2.ino      # Main firmware (single-file Arduino sketch)
└── README.md           # This file
```

---

## 🐛 Known Issues / Limitations

- **Digital triggers only** — LT/RT are digital buttons with a software ramp in Xbox mode; not true analog potentiometers
- **No rumble / haptic feedback** — `haptic_enable` flag is reserved for a future hardware revision
- **Gyro calibration is static** — `autoOffsets()` runs once at boot; thermal drift over long sessions is not corrected
- **BLE stack sensitivity** — the firmware depends on ESP32 core 2.0.14; newer cores may require library updates

---

## 📜 License

This project is open source. Feel free to use, modify, and build on it — credit appreciated.

---

## 🙏 Credits

- **BleCompositeHID** by [Mystfit](https://github.com/Mystfit/ESP32-BLE-CompositeHID) — the BLE HID foundation this entire project is built on
- **Adafruit** — SSD1306 and GFX libraries
- **wollewald** — MPU6050\_WE library
- Built with ❤️ by **Kevin Mistry**
