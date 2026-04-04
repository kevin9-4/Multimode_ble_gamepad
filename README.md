# Multimode_ble_gamepad
Developed a BLE gamepad using ESP32 and MPU6050 with gyroscope-based motion control. Implemented Xbox and custom HID modes using BLECompositeHID, with an OLED interface for mode selection. Optimized for low-latency wireless performance.

🕹 BLE Gamepad Controller (ESP32 + MPU6050)
📌 Overview

This project implements a Bluetooth Low Energy (BLE) gamepad using ESP32 and MPU6050. It uses motion-based input from the gyroscope to control gameplay and functions as a wireless Human Interface Device (HID).

⚙️ Features
Dual modes: Xbox Mode & Custom Gamepad Mode
Wireless control using BLE HID protocol
Motion-based input using MPU6050 (gyroscope)
OLED display for mode selection and feedback
Optimized for low-latency real-time performance
🧠 Working

MPU6050 captures motion data, which is processed by ESP32 and converted into control inputs. These inputs are mapped to HID signals and transmitted via BLE using the BLECompositeHID library, allowing the ESP32 to function as a wireless gamepad.

🔧 Hardware
ESP32
MPU6050
OLED Display
Push Buttons
💻 Software
Arduino IDE
BLECompositeHID Library
Wire Library (I2C)
🚀 Applications
Wireless gaming controller
Motion-based control systems
Embedded HID devices
🔮 Future Improvements
Add joystick support
Improve sensor filtering
Optimize latency further

Author: Kevin Mistry
