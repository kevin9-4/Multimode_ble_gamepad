/**
 * ============================================================
 *  GamePad V2 — ESP32 BLE Multi-Mode Controller Firmware
 * ============================================================
 *  Author  : Kevin Mistry
 *  Version : 1.0
 *  Board   : ESP32 (38-pin)
 *
 *  Description:
 *    A dual-mode Bluetooth Low Energy (BLE) gamepad built on
 *    the BleCompositeHID library. Supports:
 *      • Generic HID Gamepad mode  (Windows )
 *      • Xbox Series X HID mode    (Windows / Xbox )
 *
 *    Features:
 *      - Dual analog joysticks with deadzone & sensitivity
 *      - MPU-6050 gyroscope for motion aiming (toggle on/off)
 *      - D-Pad with diagonal support
 *      - Shoulder buttons + analog triggers (Xbox mode)
 *      - OLED UI with multi-level menus
 *      - Adjustable brightness, gyro sensitivity, stick sensitivity
 *      - Deep-sleep mode with wake-on-button
 *      - Boot mode persistence via NVS (Preferences)
 *
 *  Libraries Required:
 *    - Adafruit_SSD1306     (OLED display driver)
 *    - Adafruit_GFX         (graphics primitives)
 *    - BleCompositeHID      (https://github.com/Mystfit/ESP32-BLE-CompositeHID)
 *    - MPU6050_WE           (gyroscope driver)
 *    - Preferences          (ESP32 NVS key-value store)
 * ============================================================
 */

// ─────────────────────────────────────────────────────────────
//  System & Library Includes
// ─────────────────────────────────────────────────────────────
#include "esp_system.h"
#include "esp_mac.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <BleConnectionStatus.h>
#include <BleCompositeHID.h>
#include <XboxGamepadDevice.h>
#include <GamepadDevice.h>
#include <MPU6050_WE.h>

// ─────────────────────────────────────────────────────────────
//  OLED Display Configuration
// ─────────────────────────────────────────────────────────────
#define SCREEN_ADDRESS  0x3C    // I2C address of the SSD1306 OLED
#define SCREEN_RESET    -1      // No dedicated reset pin (share Arduino reset)
#define SCREEN_WIDTH    128     // Display width in pixels
#define SCREEN_HEIGHT   64      // Display height in pixels
#define MENU_START_Y    16      // Y pixel where the menu list begins
#define LINE_HEIGHT     9       // Pixels per menu row

// ─────────────────────────────────────────────────────────────
//  IMU (Gyroscope) Configuration
// ─────────────────────────────────────────────────────────────
#define MPU6050_ADDR    0x68    // Default I2C address for MPU-6050
#define GYRO_DEADZONE   1.5f    // Ignore gyro readings below this (°/s)

// ─────────────────────────────────────────────────────────────
//  HID Axis Range Constants
//    Generic HID uses 0 → 32767 (unsigned) with 16383 as center.
//    Xbox HID uses -32768 → 32767 (signed).
// ─────────────────────────────────────────────────────────────
#define HID_MIN_X  -32768   // Xbox signed minimum
#define HID_MIN     0       // Generic HID minimum
#define HID_MID     16383   // Generic HID center / half-scale
#define HID_MAX     32767   // Full-scale maximum (both modes)
#define DEADZONE    20      // ADC counts to ignore near joystick center
#define RAMP        300     // Increment per loop tick for trigger ramping

// ─────────────────────────────────────────────────────────────
//  Boot Mode IDs  (stored in NVS so the choice persists reboot)
// ─────────────────────────────────────────────────────────────
#define BOOT_GAMEPAD  0
#define BOOT_XBOX     1

// ─────────────────────────────────────────────────────────────
//  GPIO Pin Assignments
// ─────────────────────────────────────────────────────────────

// Analog joysticks
#define JOY1_X    34   // Left  stick X-axis  (ADC1_CH6)
#define JOY1_Y    35   // Left  stick Y-axis  (ADC1_CH7)
#define JOY1_BTN  15   // Left  stick click

#define JOY2_X    39   // Right stick X-axis  (ADC1_CH3)
#define JOY2_Y    36   // Right stick Y-axis  (ADC1_CH0)
#define JOY2_BTN  32   // Right stick click

// D-Pad directional buttons
#define DPAD_UP    5
#define DPAD_DOWN  16
#define DPAD_RIGHT 17
#define DPAD_LEFT   4

// Face action buttons  (colour-coded for reference)
#define BTN_X   3   // Yellow
#define BTN_Y   1   // Blue
#define BTN_A  19   // Red
#define BTN_B  18   // White

// Shoulder / trigger buttons
#define BTN_LT  27   // Left  Trigger  (digital in this design)
#define BTN_LB  26   // Left  Bumper
#define BTN_RT  33   // Right Trigger
#define BTN_RB  25   // Right Bumper

// System / UI control buttons
#define MODE_BTN     14   // Enter sleep / exit active mode
#define SELECT_MODE  23   // Toggle gyro aiming

// Start / Back (also used as Xbox View / Menu)
#define START  12
#define BACK   13

// I2C bus pins (shared by OLED and MPU-6050)
#define SDA  21
#define SCL  22

// ─────────────────────────────────────────────────────────────
//  Global Object Instances
// ─────────────────────────────────────────────────────────────
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, SCREEN_RESET);
MPU6050_WE       gyro(MPU6050_ADDR);
BleCompositeHID* compositeHID;   // Parent BLE HID composite device
GamepadDevice*   gamepad;        // Generic HID child device
XboxGamepadDevice* XBOX_X;       // Xbox HID child device
Preferences      my_mcu;         // NVS key-value store

// ─────────────────────────────────────────────────────────────
//  Runtime State Variables
// ─────────────────────────────────────────────────────────────
bool Previous_state    = false;  // Tracks last BLE connection state
bool Xbox_Home_latch   = false;  // Prevents repeated HOME presses
int  trigger_lt        = 0;      // Left  trigger current value  (ramped)
int  trigger_rt        = 0;      // Right trigger current value  (ramped)
int  bootMode          = BOOT_GAMEPAD; // Active boot mode (loaded from NVS)

bool GyroEnable        = false;  // Whether gyro aiming is active
bool gyro_BTN_Latch    = false;  // Debounce latch for gyro toggle button

bool haptic_enable     = true;   // Reserved: haptic feedback flag

// ─────────────────────────────────────────────────────────────
//  OLED Menu State
// ─────────────────────────────────────────────────────────────
int Menu_Count      = 5;   // Total items in the main menu
int Selected_menu   = 0;   // Highlighted index in main menu
int current_menu    = 0;   // Currently active mode index

int Selected_setting = 0;  // Highlighted index in settings menu
int setting_count    = 5;  // Total items in settings menu

uint8_t brightnessLevel  = 1;           // OLED brightness 1–10
const uint8_t BRIGHTNESS_MIN = 30;      // Minimum SSD1306 contrast value
const uint8_t BRIGHTNESS_MAX = 200;     // Maximum SSD1306 contrast value

// ─────────────────────────────────────────────────────────────
//  UI State Machine
// ─────────────────────────────────────────────────────────────
/**
 * UIState — all possible screens the OLED can display.
 * The main loop dispatches rendering and input handling
 * based on the current uiState value.
 */
enum UIState {
  UI_MENU,              // Main mode-selection menu
  UI_ACTIVE,            // Gamepad actively sending HID reports
  UI_SETTINGS,          // Settings sub-menu
  UI_BRIGHTNESS,        // Brightness adjustment screen
  UI_GYRO_SENSITIVITY,  // Gyro sensitivity adjustment screen
  UI_SENSITIVITY,       // Stick sensitivity adjustment screen
  UI_DEVIVE_INFO,       // Device information screen
  UI_SLEEP,             // Transitional state before deep sleep
  UI_DEVLOPER_MODE,     // Live ADC / gyro readout for debugging
  UI_ABOUT              // About / credits screen
};
UIState uiState = UI_MENU;

// ─────────────────────────────────────────────────────────────
//  Sensitivity Levels  — Joystick / Stick Axis
// ─────────────────────────────────────────────────────────────
/**
 * SensitivityLevel — scales joystick HID output.
 *   LOW     → 50 % of raw value
 *   DEFAULT → 100 % (no change)
 *   HIGH    → 150 % of raw value
 */
enum SensitivityLevel {
  SENS_LOW     = 0,
  SENS_DEFAULT = 1,
  SENS_HIGH    = 2
};
SensitivityLevel sensitivity = SENS_DEFAULT;

/**
 * applySensitivity()
 * Scales a raw HID axis value by the current sensitivity preset.
 *
 * @param rawValue  Signed integer HID axis value before scaling.
 * @return          Scaled integer (may exceed HID range; caller must constrain).
 */
int applySensitivity(int rawValue) {
  switch (sensitivity) {
    case SENS_LOW:  return (rawValue * 5)  / 10;   // 50 %
    case SENS_HIGH: return (rawValue * 15) / 10;   // 150 %
    default:        return rawValue;               // 100 %
  }
}

// ─────────────────────────────────────────────────────────────
//  Gyro Sensitivity Levels
// ─────────────────────────────────────────────────────────────
/**
 * GyroSensitivityLevel — controls how aggressively gyro motion
 * moves the virtual right-stick.
 */
enum GyroSensitivityLevel {
  GYRO_LOW     = 0,
  GYRO_DEFAULT = 1,
  GYRO_HIGH    = 2
};
GyroSensitivityLevel gyroSensitivity = GYRO_DEFAULT;

/**
 * applyGyroSensitivity()
 * Converts a raw gyro rate (°/s) to a HID-scale delta.
 *
 * @param rawValue  Raw gyroscope angular rate in °/s.
 * @return          Scaled float ready to be cast to int and added to axis.
 */
float applyGyroSensitivity(float rawValue) {
  switch (gyroSensitivity) {
    case GYRO_LOW:     return rawValue * 20.0f;
    case GYRO_DEFAULT: return rawValue * 35.0f;
    case GYRO_HIGH:    return rawValue * 50.0f;
  }
  return rawValue * 35.0f; // Fallback
}

// ─────────────────────────────────────────────────────────────
//  Splash-screen Bitmap — 128 × 64 px (stored in Flash)
// ─────────────────────────────────────────────────────────────
const unsigned char MY_LOGO[] PROGMEM = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0x87, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0xe0, 0x03, 0xe1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0x00, 0x00, 0x78, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3c, 0x00, 0x00, 0x1e, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x70, 0x00, 0x00, 0x07, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xe0, 0x00, 0x00, 0x03, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0x80, 0x00, 0x00, 0x00, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x67, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xe6, 0x00, 0x00, 0x00, 0x00, 0x33, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x19, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x98, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x98, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x30, 0x1f, 0x00, 0x7c, 0x0c, 0x06, 0x7f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x30, 0x1f, 0x00, 0xfc, 0x1c, 0x06, 0x7f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfe, 0x60, 0x1f, 0x01, 0xf8, 0x3c, 0x03, 0x3f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfe, 0x60, 0x1f, 0x03, 0xf0, 0x3c, 0x03, 0x3f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfc, 0xc0, 0x1f, 0x07, 0xe0, 0x7c, 0x01, 0x9f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfc, 0xc0, 0x1f, 0x0f, 0xc0, 0xfc, 0x01, 0x9f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfc, 0x80, 0x1f, 0x0f, 0x81, 0xfc, 0x00, 0x9f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfd, 0x80, 0x1f, 0x1f, 0x03, 0xfc, 0x00, 0xdf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0x3f, 0x07, 0xfc, 0x00, 0xdf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0x7e, 0x0f, 0xfc, 0x00, 0xcf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0xfc, 0x0f, 0xfc, 0x00, 0xcf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0xfc, 0x1f, 0x7c, 0x00, 0xcf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0xfe, 0x3e, 0x7c, 0x00, 0xcf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0xff, 0x7e, 0x7c, 0x00, 0xcf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0xff, 0xfc, 0x7c, 0x00, 0xcf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0xcf, 0xf8, 0x7c, 0x00, 0xcf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0x87, 0xf0, 0x7c, 0x00, 0xcf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x1f, 0x03, 0xe0, 0x7c, 0x00, 0xdf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfd, 0x80, 0x1f, 0x03, 0xc0, 0x7c, 0x00, 0xdf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfc, 0xc0, 0x1f, 0x01, 0xc0, 0x7c, 0x01, 0x9f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfc, 0xc0, 0x1f, 0x00, 0x00, 0x7c, 0x01, 0x9f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfc, 0xc0, 0x1f, 0x00, 0x00, 0x7c, 0x01, 0xbf, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfe, 0x60, 0x1f, 0x00, 0x00, 0x7c, 0x03, 0x3f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfe, 0x60, 0x1f, 0x00, 0x00, 0x7c, 0x03, 0x3f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x30, 0x1f, 0x00, 0x00, 0x7c, 0x06, 0x7f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x30, 0x1f, 0x00, 0x00, 0x7c, 0x06, 0x7f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x98, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x9c, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x19, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xe6, 0x00, 0x00, 0x00, 0x00, 0x33, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0x00, 0x00, 0x00, 0x00, 0x67, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x00, 0x00, 0x00, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xe0, 0x00, 0x00, 0x03, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x78, 0x00, 0x00, 0x0f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1e, 0x00, 0x00, 0x3c, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0x80, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe1, 0xf8, 0x0f, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xfe, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

// ─────────────────────────────────────────────────────────────
//  "Connected" bitmap — controller silhouette shown on connect
// ─────────────────────────────────────────────────────────────
const unsigned char video_game_controller[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x03, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x07, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x3e, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xf8, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00,
  0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00,
  0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00,
  0x01, 0xf0, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xe0, 0x00, 0x0f, 0x80,
  0x07, 0xc0, 0x00, 0x1f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x00, 0x03, 0xe0,
  0x0f, 0x80, 0x00, 0x3e, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x78, 0x00, 0x01, 0xf0,
  0x1e, 0x00, 0x00, 0x3e, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x78,
  0x3c, 0x00, 0x02, 0xbe, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x10, 0x00, 0x7c,
  0x3c, 0x01, 0xff, 0xfc, 0x1f, 0xff, 0xc0, 0x00, 0x00, 0x03, 0xff, 0x00, 0x01, 0xff, 0x80, 0x3c,
  0x3c, 0x03, 0xe0, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x07, 0x8f, 0x80, 0x03, 0xc7, 0xc0, 0x3c,
  0x3c, 0x03, 0xf7, 0xc0, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x07, 0xff, 0x80, 0x03, 0xff, 0xc0, 0x3c,
  0x3c, 0x00, 0xff, 0xfe, 0x1f, 0xff, 0x80, 0x00, 0x00, 0x01, 0xfe, 0x00, 0x00, 0xff, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x3e, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x3e, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x1e, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x78, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x00, 0x03, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x3c,
  0x3c, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x3c,
  0x3e, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x7c,
  0x0f, 0xc0, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x03, 0xf0,
  0x03, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xc0,
  0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ─────────────────────────────────────────────────────────────
//  Menu / Settings String Tables
// ─────────────────────────────────────────────────────────────
/** Main menu item labels (index matches Selected_menu). */
const char* menuItems[] = {
  "GamePad",       // 0 — Generic HID mode
  "XBOX_X",        // 1 — Xbox HID mode
  "Developer Mode",// 2 — Live sensor readout
  "Settings",      // 3 — Settings sub-menu
  "About"          // 4 — Credits screen
};

/** Settings sub-menu item labels (index matches Selected_setting). */
const char* settingItems[] = {
  "Brightness",       // 0
  "Gyro Sensitivity", // 1
  "Sensitivity",      // 2
  "Device Info",      // 3
  "Sleep"             // 4
};

// ─────────────────────────────────────────────────────────────
//  HID Button Mapping Tables
// ─────────────────────────────────────────────────────────────
/**
 * Generic HID button map.
 * hidButtonID[i] is pressed/released when buttonPins[i] is LOW.
 */
int hidButtonID[] = {
  BUTTON_1,  BUTTON_2,  BUTTON_3,  BUTTON_4,   // X, A, B, Y
  BUTTON_5,  BUTTON_6,  BUTTON_7,  BUTTON_8,   // LB, RB, LT, RT
  BUTTON_9,  BUTTON_10,                         // Back, Start
  BUTTON_11, BUTTON_12                          // L3, R3
};
int buttonPins[] = {
  BTN_X, BTN_A, BTN_B, BTN_Y,     // Face buttons
  BTN_LB, BTN_RB, BTN_LT, BTN_RT, // Shoulder / trigger
  BACK, START,                      // System buttons
  JOY1_BTN, JOY2_BTN              // Stick clicks
};

/**
 * Xbox HID button map.
 * Xbox_BTN_ID[i] is pressed/released when XBOX_BTN_PIN[i] is LOW.
 * Note: HOME is handled separately via a chord (BACK + START).
 */
int XBOX_BTN_PIN[] = {
  BTN_A, BTN_B, BTN_X, BTN_Y,
  BTN_LB, BTN_RB,
  BACK, START,
  JOY1_BTN, JOY2_BTN
};
const int Xbox_BTN_ID[] = {
  XBOX_BUTTON_A,  XBOX_BUTTON_B,
  XBOX_BUTTON_X,  XBOX_BUTTON_Y,
  XBOX_BUTTON_LB, XBOX_BUTTON_RB,
  XBOX_BUTTON_START, XBOX_BUTTON_SELECT,
  XBOX_BUTTON_LS, XBOX_BUTTON_RS
};

// ═══════════════════════════════════════════════════════════════
//  ██  HARDWARE INIT
// ═══════════════════════════════════════════════════════════════

/**
 * BTN_init()
 * Configures all digital input pins with internal pull-ups.
 * Active-LOW logic — buttons connect GPIO to GND.
 */
void BTN_init() {
  pinMode(JOY1_BTN,    INPUT_PULLUP);
  pinMode(JOY2_BTN,    INPUT_PULLUP);
  pinMode(DPAD_UP,     INPUT_PULLUP);
  pinMode(DPAD_DOWN,   INPUT_PULLUP);
  pinMode(DPAD_LEFT,   INPUT_PULLUP);
  pinMode(DPAD_RIGHT,  INPUT_PULLUP);
  pinMode(BTN_X,       INPUT_PULLUP);
  pinMode(BTN_Y,       INPUT_PULLUP);
  pinMode(BTN_A,       INPUT_PULLUP);
  pinMode(BTN_B,       INPUT_PULLUP);
  pinMode(BTN_LT,      INPUT_PULLUP);
  pinMode(BTN_LB,      INPUT_PULLUP);
  pinMode(BTN_RT,      INPUT_PULLUP);
  pinMode(BTN_RB,      INPUT_PULLUP);
  pinMode(MODE_BTN,    INPUT_PULLUP);
  pinMode(SELECT_MODE, INPUT_PULLUP);
  pinMode(START,       INPUT_PULLUP);
  pinMode(BACK,        INPUT_PULLUP);
}

// ═══════════════════════════════════════════════════════════════
//  ██  SELF-TEST HELPERS  (run once at boot)
// ═══════════════════════════════════════════════════════════════

/**
 * Test_BTN()
 * Verifies all buttons are released (HIGH) at boot.
 * Prints a pass/fail result to the OLED.
 * NOTE: Call oled.display() after this function.
 */
void Test_BTN() {
  bool allOK = true;
  int BTN[] = {
    JOY1_BTN, JOY2_BTN,
    DPAD_UP, DPAD_DOWN, DPAD_RIGHT, DPAD_LEFT,
    BTN_X, BTN_Y, BTN_A, BTN_B,
    BTN_LT, BTN_LB, BTN_RT, BTN_RB,
    MODE_BTN, SELECT_MODE, START, BACK
  };
  int total = sizeof(BTN) / sizeof(BTN[0]);

  for (int i = 0; i < total; i++) {
    if (digitalRead(BTN[i]) == LOW) {
      allOK = false;
      break;
    }
  }
  oled.println(allOK ? "ALL INPUTS - OK" : "INPUT HAVE ISSUE !");
}

/**
 * TEST_GYRO()
 * Checks that the MPU-6050 responds on the I2C bus.
 * Prints a pass/fail result to the OLED.
 */
void TEST_GYRO() {
  if (gyro.init()) {
    oled.print("GYRO - OK ");
  } else {
    oled.print("GYRO - FAILED !");
  }
}

// ═══════════════════════════════════════════════════════════════
//  ██  OLED UI SCREENS
// ═══════════════════════════════════════════════════════════════

/**
 * ui_boot()
 * Displays the startup splash logo then runs self-tests for
 * OLED, BLE, gyroscope, and all buttons.
 */
void ui_boot() {
  oled.clearDisplay();
  oled.drawBitmap(0, 0, MY_LOGO, SCREEN_WIDTH, 64, 1);
  oled.display();
  delay(2500);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.setTextSize(1);
  oled.println("Initialization...");
  oled.drawLine(0, 15, 125, 15, SSD1306_WHITE);
  oled.display();
  delay(100);

  oled.setCursor(0, 20);  oled.println("OLED - OK");    oled.display();
  oled.setCursor(0, 30);  oled.println("BLE  - OK");    delay(100);
  oled.setCursor(0, 40);  TEST_GYRO();                   oled.display();
  delay(100);
  oled.setCursor(0, 50);  Test_BTN();                    oled.display();
  delay(2000);
}

/**
 * ui_Welcome()
 * Brief animated welcome screen shown on every power-on.
 */
void ui_Welcome() {
  oled.clearDisplay();
  oled.setCursor(24, 20);
  oled.setTextSize(2);
  oled.setTextColor(SSD1306_WHITE);
  oled.println("WELCOME");
  oled.display();
  delay(1000);

  oled.setTextSize(1);
  oled.setCursor(30, 40);  oled.println("GamePad by");
  oled.setCursor(48, 50);  oled.println("KEVIN");
  oled.display();
  delay(1500);
}

/**
 * ui_boot_mode()
 * Shows which HID mode was loaded from NVS so the user
 * knows what profile is active before the BLE connection.
 */
void ui_boot_mode() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("BOOT MODE :");
  oled.drawLine(0, 14, 127, 14, SSD1306_WHITE);
  oled.setCursor(0, 20);
  oled.println(bootMode == BOOT_XBOX ? "XBOX HID" : "GAMEPAD HID");
  oled.display();
  delay(200);
}

/**
 * ui_connection()
 * "Waiting for connection" screen while BLE advertising.
 */
void ui_connection() {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.setTextSize(1);
  oled.println("GAMEPAD V2");
  oled.drawLine(0, 15, 125, 15, SSD1306_WHITE);
  oled.setCursor(0, 20);
  oled.print("Wait For Connection..");
  oled.display();
}

/**
 * BLE_Status()
 * Called every loop iteration. Detects BLE connect/disconnect
 * transitions and updates the OLED accordingly.
 * Uses a previous-state variable so the screen only redraws
 * on change, not every frame.
 */
void BLE_Status() {
  bool currentState = compositeHID->isConnected();
  if (currentState == Previous_state) return;   // No change — skip

  Previous_state = currentState;
  oled.clearDisplay();

  if (currentState) {
    // ── Connected ──────────────────────────────────────────
    oled.setCursor(0, 0);
    oled.println("CONNECTED...");
    oled.drawLine(0, 14, 127, 14, SSD1306_WHITE);
    oled.drawBitmap(0, 16, video_game_controller, 128, 48, 1);
    oled.display();
    delay(3000);
  } else {
    // ── Disconnected ────────────────────────────────────────
    oled.setCursor(0, 0);  oled.println("Disconnected!");
    oled.setCursor(0, 18); oled.println("TRY AGAIN !");
    oled.setCursor(0, 36); oled.println("RESET ESP32 !");
    oled.display();
  }
}

/**
 * draw_menu()
 * Renders the main mode-selection menu.
 * The currently highlighted item is drawn inverted
 * (black text on white background).
 */
void draw_menu() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("MODES");
  oled.drawLine(0, 13, 127, 13, SSD1306_WHITE);

  for (int i = 0; i < Menu_Count; i++) {
    int y = MENU_START_Y + (i * LINE_HEIGHT);

    if (i == Selected_menu) {
      // Highlight selected row
      oled.fillRect(0, y, 128, LINE_HEIGHT, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
    } else {
      oled.setTextColor(SSD1306_WHITE);
    }
    oled.setCursor(5, y + 1);
    oled.println(menuItems[i]);
    oled.setTextColor(SSD1306_WHITE); // Reset after each row
  }
  oled.display();
}

/**
 * ui_active_mode()
 * Status screen shown while the gamepad is actively sending
 * HID reports.  Displays current mode and gyro on/off state.
 */
void ui_active_mode() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("ACTIVE MODE :");
  oled.drawLine(0, 16, 128, 16, SSD1306_WHITE);

  oled.setCursor(0, 20);
  oled.println(menuItems[current_menu]);

  oled.setCursor(0, 35);
  oled.print("GYRO : ");
  oled.println(GyroEnable ? "ON" : "OFF");

  oled.setCursor(0, 48);
  oled.println("SELECT_BTN : TOGGLE");
  oled.display();
}

/**
 * Ui_Settings()
 * Renders the settings sub-menu with highlight on Selected_setting.
 */
void Ui_Settings() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("SETTINGS");
  oled.drawLine(0, 13, 127, 13, SSD1306_WHITE);

  for (int i = 0; i < setting_count; i++) {
    int y = MENU_START_Y + (i * LINE_HEIGHT);

    if (i == Selected_setting) {
      oled.fillRect(0, y, 128, LINE_HEIGHT, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
    } else {
      oled.setTextColor(SSD1306_WHITE);
    }
    oled.setCursor(5, y + 1);
    oled.println(settingItems[i]);
    oled.setTextColor(SSD1306_WHITE);
  }
  oled.display();
}

/**
 * ui_brightness()
 * Shows the current brightness level as a bar of '#' characters.
 */
void ui_brightness() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("BRIGHTNESS :");
  oled.drawLine(0, 13, 127, 13, SSD1306_WHITE);

  oled.setCursor(0, 22);
  oled.print("LEVEL : ");
  for (int i = 1; i <= 10; i++) {
    oled.print(i <= brightnessLevel ? "#" : "-");
  }
  oled.setCursor(0, 40);
  oled.print(brightnessLevel);
  oled.print(" / 10");
  oled.display();
}

/**
 * ui_sensitivity()
 * Shows the current joystick stick sensitivity preset.
 */
void ui_sensitivity() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("SENSITIVITY :");
  oled.drawLine(0, 13, 127, 13, SSD1306_WHITE);

  oled.setCursor(0, 25);
  oled.print("LEVEL - ");
  switch (sensitivity) {
    case SENS_LOW:     oled.println("LOW");     break;
    case SENS_DEFAULT: oled.println("DEFAULT"); break;
    case SENS_HIGH:    oled.println("HIGH");    break;
  }
  oled.setCursor(0, 45);
  oled.println("BLUE / RED : CHANGES");
  oled.display();
}

/**
 * ui_Gyro_Sensitivity()
 * Shows the current gyro sensitivity preset.
 */
void ui_Gyro_Sensitivity() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("GYRO SENSITIVITY");
  oled.drawLine(0, 13, 127, 13, SSD1306_WHITE);

  oled.setCursor(0, 25);
  oled.print("LEVEL : ");
  switch (gyroSensitivity) {
    case GYRO_LOW:     oled.println("LOW");     break;
    case GYRO_DEFAULT: oled.println("DEFAULT"); break;
    case GYRO_HIGH:    oled.println("HIGH");    break;
  }
  oled.setCursor(0, 45);
  oled.println("BLUE / RED : CHANGE");
  oled.display();
}

/**
 * ui_device_info()
 * Shows firmware version, current HID mode, and author credit.
 */
void ui_device_info() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("DEVICE INFO");
  oled.drawLine(0, 13, 127, 13, SSD1306_WHITE);

  oled.setCursor(0, 20); oled.println("GamePad V2");
  oled.setCursor(0, 30); oled.print("Firmware : 1.0");
  oled.setCursor(0, 40); oled.print("Mode : ");
  oled.println(bootMode == BOOT_XBOX ? "XBOX" : "GAMEPAD");
  oled.setCursor(0, 50); oled.println("Author : Kevin Mistry");
  oled.display();
}

/**
 * UI_About()
 * Project credits and build stats.
 */
void UI_About() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(34, 0);  oled.println("GamePad V2");
  oled.drawLine(0, 12, 127, 12, SSD1306_WHITE);
  oled.setCursor(20, 22); oled.println("3rd Prototype");
  oled.setCursor(18, 32); oled.println("6 Months Build");
  oled.setCursor(16, 44); oled.println("ESP32 Controller");
  oled.setCursor(40, 56); oled.println("By Kevin");
  oled.display();
}

/**
 * ui_Developer_mode()
 * Live diagnostic readout: raw ADC joystick values + gyro X/Y.
 * Useful for calibration and hardware debugging.
 * Refreshes every 100 ms.
 */
void ui_Developer_mode() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("DEV MODE");
  oled.drawLine(0, 13, 127, 13, SSD1306_WHITE);

  int rawx1 = analogRead(JOY1_X);
  int rawy1 = analogRead(JOY1_Y);
  int rawx2 = analogRead(JOY2_X);
  int rawy2 = analogRead(JOY2_Y);

  oled.setCursor(0, 18);
  oled.print("J1X:"); oled.print(rawx1);
  oled.print(" J1Y:"); oled.println(rawy1);

  oled.setCursor(0, 28);
  oled.print("J2X:"); oled.print(rawx2);
  oled.print(" J2Y:"); oled.println(rawy2);

  xyzFloat g = gyro.getGyrValues();
  oled.setCursor(0, 40); oled.print("GX:"); oled.print(g.x);
  oled.setCursor(0, 50); oled.print("GY:"); oled.print(g.y);

  oled.display();
  delay(100);
}

// ─────────────────────────────────────────────────────────────
//  Brightness Helpers
// ─────────────────────────────────────────────────────────────

/**
 * brightnessToContrast()
 * Maps the 1–10 user brightness level to the SSD1306 contrast
 * byte range (BRIGHTNESS_MIN … BRIGHTNESS_MAX).
 */
uint8_t brightnessToContrast(uint8_t level) {
  return map(level, 1, 10, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
}

/**
 * apply_brightness()
 * Sends the SSD1306 contrast command immediately.
 * Call this whenever brightnessLevel changes.
 */
void apply_brightness() {
  oled.ssd1306_command(SSD1306_SETCONTRAST);
  oled.ssd1306_command(brightnessToContrast(brightnessLevel));
}

// ─────────────────────────────────────────────────────────────
//  Sleep Helpers
// ─────────────────────────────────────────────────────────────

/**
 * mcu_sleep_animation()
 * Displays a 3-2-1 countdown before entering deep sleep.
 */
void mcu_sleep_animation() {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print("Going to Sleep...");
  oled.drawLine(0, 13, 127, 13, SSD1306_WHITE);
  oled.display();
  delay(1000);

  for (int i = 3; i >= 1; i--) {
    oled.clearDisplay();
    oled.setTextSize(3);
    oled.setCursor(50, 30);
    oled.print(i);
    oled.display();
    delay(1000);
  }
  oled.setTextSize(1); // Restore default text size
}

/**
 * mcu_sleep()
 * Turns off the OLED and enters ESP32 deep sleep.
 * Wake-up source: MODE_BTN pulled LOW.
 */
void mcu_sleep() {
  oled.clearDisplay();
  oled.display();
  delay(500);
  oled.ssd1306_command(SSD1306_DISPLAYOFF);

  // Wake when MODE_BTN is pulled to GND
  esp_sleep_enable_ext0_wakeup((gpio_num_t)MODE_BTN, 0);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  delay(100);
  esp_deep_sleep_start();
}

// ─────────────────────────────────────────────────────────────
//  Mode-Switch Helpers  (save to NVS + reboot)
// ─────────────────────────────────────────────────────────────

/**
 * boot_xbox_mode()
 * Saves BOOT_XBOX to NVS and restarts so the ESP32 comes up
 * in Xbox HID mode with a different BLE MAC address.
 */
void boot_xbox_mode() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.println("Switching To...");
  oled.drawLine(0, 13, 127, 13, SSD1306_WHITE);
  oled.setCursor(0, 20); oled.println("XBOX X MODE");
  oled.display();
  delay(1500);
  my_mcu.putInt("mode", BOOT_XBOX);
  delay(800);
  ESP.restart();
}

/**
 * switch_gamepad_mode()
 * Saves BOOT_GAMEPAD to NVS and restarts into Generic HID mode.
 */
void switch_gamepad_mode() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.println("Switching To...");
  oled.drawLine(0, 15, 127, 15, SSD1306_WHITE);
  oled.setCursor(0, 20); oled.println("GAMEPAD");
  oled.display();
  delay(1500);
  my_mcu.putInt("mode", BOOT_GAMEPAD);
  delay(800);
  ESP.restart();
}

// ═══════════════════════════════════════════════════════════════
//  ██  GENERIC HID GAMEPAD INPUT FUNCTIONS
// ═══════════════════════════════════════════════════════════════

/**
 * ReadButtons()
 * Iterates the face/shoulder/system button list and
 * presses or releases the corresponding HID button ID.
 */
void ReadButtons() {
  int total = sizeof(buttonPins) / sizeof(buttonPins[0]);
  for (int i = 0; i < total; i++) {
    if (digitalRead(buttonPins[i]) == LOW)
      gamepad->press(hidButtonID[i]);
    else
      gamepad->release(hidButtonID[i]);
  }
}

/**
 * ReadDpad()
 * Reads all four D-Pad directions and sets the HID hat switch,
 * including the four diagonal combinations.
 */
void ReadDpad() {
  bool up    = (digitalRead(DPAD_UP)    == LOW);
  bool down  = (digitalRead(DPAD_DOWN)  == LOW);
  bool right = (digitalRead(DPAD_RIGHT) == LOW);
  bool left  = (digitalRead(DPAD_LEFT)  == LOW);

  uint8_t hat = HAT_CENTERED;

  if      (up    && right) hat = HAT_UP_RIGHT;
  else if (right && down)  hat = HAT_DOWN_RIGHT;
  else if (up    && left)  hat = HAT_UP_LEFT;
  else if (left  && down)  hat = HAT_DOWN_LEFT;
  else if (up)             hat = HAT_UP;
  else if (down)           hat = HAT_DOWN;
  else if (right)          hat = HAT_RIGHT;
  else if (left)           hat = HAT_LEFT;

  gamepad->setHat1(hat);
}

/**
 * readJoysticks()
 * Full joystick pipeline for Generic HID mode:
 *   1. Read raw ADC values
 *   2. Subtract mechanical centre offset
 *   3. Apply deadzone (zero small values)
 *   4. Map to signed HID range (-HID_MID … +HID_MID)
 *   5. Apply stick sensitivity multiplier
 *   6. Read gyroscope and optionally add to right stick
 *   7. Constrain and convert to unsigned HID range (0 … HID_MAX)
 *   8. Send to gamepad object
 *
 * Centre offsets should be calibrated for your specific
 * joystick modules — measure raw ADC at rest and adjust.
 */
void readJoysticks() {
  // Empirically measured ADC midpoints (adjust if joysticks drift)
  const int joy1xcenter = 1880;
  const int joy1ycenter = 1830;
  const int joy2center  = 1890;

  // Step 1 — Raw ADC readings (12-bit: 0–4095)
  int rawx1 = analogRead(JOY1_X);
  int rawy1 = analogRead(JOY1_Y);
  int rawx2 = analogRead(JOY2_X);
  int rawy2 = analogRead(JOY2_Y);

  // Step 2 — Centre offset  (Y axis inverted to match convention)
  int lx = rawx1 - joy1xcenter;
  int ly = joy1ycenter - rawy1;
  int rx = rawx2 - joy2center;
  int ry = rawy2 - joy2center;

  // Step 3 — Deadzone (suppress noise near center)
  if (abs(lx) < DEADZONE) lx = 0;
  if (abs(ly) < DEADZONE) ly = 0;
  if (abs(rx) < DEADZONE) rx = 0;
  if (abs(ry) < DEADZONE) ry = 0;

  // Step 4 — Map to signed HID range
  lx = map(lx, -joy1xcenter, joy1xcenter, -HID_MID, HID_MID);
  ly = map(ly, -joy1ycenter, joy1ycenter, -HID_MID, HID_MID);
  rx = map(rx, -joy2center,  joy2center,  -HID_MID, HID_MID);
  ry = map(ry, -joy2center,  joy2center,  -HID_MID, HID_MID);

  // Step 5 — Sensitivity scaling
  lx = applySensitivity(lx);
  ly = applySensitivity(ly);
  rx = applySensitivity(rx);
  ry = applySensitivity(ry);

  // Step 6 — Gyro (only blended into the right stick)
  xyzFloat g = gyro.getGyrValues();
  float gx = g.x, gy = g.y;
  if (abs(gx) < GYRO_DEADZONE) gx = 0;
  if (abs(gy) < GYRO_DEADZONE) gy = 0;

  int gyroRX = (int)(applyGyroSensitivity(gx) * 10);
  int gyroRY = (int)(applyGyroSensitivity(gy) * 10);

  int finalRX = (GyroEnable) ? constrain(rx + gyroRX, -HID_MID, HID_MID) : rx;
  int finalRY = (GyroEnable) ? constrain(ry + gyroRY, -HID_MID, HID_MID) : ry;

  // Step 7 — Shift to unsigned range and clamp
  gamepad->setX  (constrain(lx + HID_MID, HID_MIN, HID_MAX));
  gamepad->setY  (constrain(ly + HID_MID, HID_MIN, HID_MAX));
  gamepad->setRX (constrain(finalRX + HID_MID, HID_MIN, HID_MAX));
  gamepad->setRY (constrain(finalRY + HID_MID, HID_MIN, HID_MAX));
  gamepad->setZ  (constrain(finalRX + HID_MID, HID_MIN, HID_MAX));
  gamepad->setRZ (constrain(finalRY + HID_MID, HID_MIN, HID_MAX));
}

/**
 * gamePad_mode()
 * One complete Generic HID report cycle:
 *   - Read joysticks (+ optional gyro)
 *   - Read face/shoulder/system buttons
 *   - Read D-Pad hat
 *   - Send HID report to host
 */
void gamePad_mode() {
  readJoysticks();
  ReadButtons();
  ReadDpad();
  gamepad->sendGamepadReport();
  delay(20); // ~50 Hz report rate
}

// ═══════════════════════════════════════════════════════════════
//  ██  XBOX HID INPUT FUNCTIONS
// ═══════════════════════════════════════════════════════════════

/**
 * read_Xbox_BTN()
 * Handles Xbox button presses including the special HOME chord
 * (BACK + START held together). All other buttons iterate the
 * pin/ID arrays and press/release accordingly.
 */
void read_Xbox_BTN() {
  bool backPressed  = (digitalRead(BACK)  == LOW);
  bool startPressed = (digitalRead(START) == LOW);

  // HOME chord: BACK + START simultaneously
  if (backPressed && startPressed) {
    if (!Xbox_Home_latch) {
      XBOX_X->press(XBOX_BUTTON_HOME);
      Xbox_Home_latch = true;
    }
    delay(50);
  } else {
    if (Xbox_Home_latch) {
      XBOX_X->release(XBOX_BUTTON_HOME);
      Xbox_Home_latch = false;
    }
  }

  // Regular buttons
  int total = sizeof(XBOX_BTN_PIN) / sizeof(XBOX_BTN_PIN[0]);
  for (int i = 0; i < total; i++) {
    if (digitalRead(XBOX_BTN_PIN[i]) == LOW)
      XBOX_X->press(Xbox_BTN_ID[i]);
    else
      XBOX_X->release(Xbox_BTN_ID[i]);
  }
}

/**
 * Read_xbox_Trigger()
 * Simulates analog triggers using digital buttons.
 * Each press increments the trigger value by RAMP per loop;
 * each release decrements it.  This gives a soft ramp rather
 * than an abrupt 0/max transition.
 */
void Read_xbox_Trigger() {
  trigger_lt += (digitalRead(BTN_LT) == LOW) ?  RAMP : -RAMP;
  trigger_rt += (digitalRead(BTN_RT) == LOW) ?  RAMP : -RAMP;

  trigger_lt = constrain(trigger_lt, 0, 4095);
  trigger_rt = constrain(trigger_rt, 0, 4095);

  XBOX_X->setLeftTrigger(trigger_lt);
  XBOX_X->setRightTrigger(trigger_rt);
}

/**
 * Read_xbox_joystick()
 * Joystick pipeline for Xbox HID mode.
 * Uses signed range (HID_MIN_X … HID_MAX) as required by
 * the Xbox HID descriptor.  Gyro blending is applied to the
 * right thumb stick when GyroEnable is true.
 */
void Read_xbox_joystick() {
  const int joy1xcenter = 1880;
  const int joy1ycenter = 1830;
  const int joy2center  = 1890;

  // Raw ADC
  int rawlx = analogRead(JOY1_X);
  int rawly = analogRead(JOY1_Y);
  int rawrx = analogRead(JOY2_X);
  int rawry = analogRead(JOY2_Y);

  // Centre offset (Y inverted)
  int lx = rawlx - joy1xcenter;
  int ly = joy1ycenter - rawly;
  int rx = rawrx - joy2center;
  int ry = rawry - joy2center;

  // Deadzone
  if (abs(lx) < DEADZONE) lx = 0;
  if (abs(ly) < DEADZONE) ly = 0;
  if (abs(rx) < DEADZONE) rx = 0;
  if (abs(ry) < DEADZONE) ry = 0;

  // Map to Xbox signed range
  lx = map(lx, -joy1xcenter, joy1xcenter, HID_MIN_X, HID_MAX);
  ly = map(ly, -joy1ycenter, joy1ycenter, HID_MIN_X, HID_MAX);
  rx = map(rx, -joy2center,  joy2center,  HID_MIN_X, HID_MAX);
  ry = map(ry, -joy2center,  joy2center,  HID_MIN_X, HID_MAX);

  // Sensitivity scaling
  lx = applySensitivity(lx);
  ly = applySensitivity(ly);
  rx = applySensitivity(rx);
  ry = applySensitivity(ry);

  lx = constrain(lx, HID_MIN_X, HID_MAX);
  ly = constrain(ly, HID_MIN_X, HID_MAX);
  rx = constrain(rx, HID_MIN_X, HID_MAX);
  ry = constrain(ry, HID_MIN_X, HID_MAX);

  // Gyro blend into right stick
  xyzFloat g = gyro.getGyrValues();
  float gx = g.x, gy = g.y;
  if (abs(gx) < GYRO_DEADZONE) gx = 0;
  if (abs(gy) < GYRO_DEADZONE) gy = 0;

  int gyroRX = (int)(applyGyroSensitivity(gx) * 10);
  int gyroRY = (int)(applyGyroSensitivity(gy) * 10);

  XBOX_X->setLeftThumb(lx, ly);

  if (GyroEnable) {
    int finalRX = constrain(rx + gyroRX, HID_MIN_X, HID_MAX);
    int finalRY = constrain(ry + gyroRY, HID_MIN_X, HID_MAX);
    XBOX_X->setRightThumb(finalRX, finalRY);
  } else {
    XBOX_X->setRightThumb(rx, ry);
  }
}

/**
 * read_X_Hats()
 * Maps D-Pad buttons to Xbox D-Pad direction codes including
 * all four diagonal combinations.
 */
void read_X_Hats() {
  bool up    = (digitalRead(DPAD_UP)    == LOW);
  bool down  = (digitalRead(DPAD_DOWN)  == LOW);
  bool left  = (digitalRead(DPAD_LEFT)  == LOW);
  bool right = (digitalRead(DPAD_RIGHT) == LOW);

  uint8_t dpad = XBOX_BUTTON_DPAD_NONE;

  if      (up   && right) dpad = XBOX_BUTTON_DPAD_NORTHEAST;
  else if (up   && left)  dpad = XBOX_BUTTON_DPAD_NORTHWEST;
  else if (down && right) dpad = XBOX_BUTTON_DPAD_SOUTHEAST;
  else if (down && left)  dpad = XBOX_BUTTON_DPAD_SOUTHWEST;
  else if (up)            dpad = XBOX_BUTTON_DPAD_NORTH;
  else if (down)          dpad = XBOX_BUTTON_DPAD_SOUTH;
  else if (left)          dpad = XBOX_BUTTON_DPAD_WEST;
  else if (right)         dpad = XBOX_BUTTON_DPAD_EAST;

  XBOX_X->pressDPadDirection(dpad);
}

/**
 * xbox_mode()
 * One complete Xbox HID report cycle.
 */
void xbox_mode() {
  Read_xbox_joystick();
  read_Xbox_BTN();
  Read_xbox_Trigger();
  read_X_Hats();
  XBOX_X->sendGamepadReport();
  delay(20); // ~50 Hz report rate
}

// ─────────────────────────────────────────────────────────────
//  Gyro Toggle (debounced)
// ─────────────────────────────────────────────────────────────

/**
 * HandleGyroToggle()
 * Toggles GyroEnable once per SELECT_MODE button press.
 * Uses a latch flag so holding the button doesn't spam toggles.
 */
void HandleGyroToggle() {
  bool pressed = (digitalRead(SELECT_MODE) == LOW);

  if (pressed && !gyro_BTN_Latch) {
    GyroEnable = !GyroEnable;
    gyro_BTN_Latch = true;
  }
  if (!pressed) {
    gyro_BTN_Latch = false;
  }
}

// ═══════════════════════════════════════════════════════════════
//  ██  ARDUINO ENTRY POINTS
// ═══════════════════════════════════════════════════════════════

/**
 * setup()
 * Runs once at power-on or after a reset:
 *   1. Init all GPIO pins
 *   2. Start OLED and show welcome / boot screens
 *   3. Init and calibrate MPU-6050
 *   4. Load boot mode from NVS
 *   5. Create and start the appropriate BLE HID device
 *   6. Show connection-wait screen
 */
void setup() {
  BTN_init();

  // ── OLED Init ──────────────────────────────────────────────
  oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  oled.setTextColor(SSD1306_WHITE);

  ui_Welcome();

  // ── Gyro Init ──────────────────────────────────────────────
  gyro.init();
  gyro.setGyrRange(MPU6050_GYRO_RANGE_500); // ±500 °/s range
  delay(1000);
  gyro.autoOffsets(); // Calibrate at rest — keep the controller still

  ui_boot();
  delay(2000);

  // ── Load boot mode from NVS ────────────────────────────────
  my_mcu.begin("boot", false);
  bootMode = my_mcu.getInt("mode", BOOT_GAMEPAD);

  // ── BLE HID Initialisation ─────────────────────────────────
  if (bootMode == BOOT_GAMEPAD) {
    // Use a fixed Bluetooth MAC so the host always recognises us
    uint8_t gpMac[6] = { 0x24, 0x6F, 0x28, 0xAA, 0xAA, 0xA1 };
    esp_base_mac_addr_set(gpMac);

    compositeHID = new BleCompositeHID("GamePad V2 GP", "Kevin", 100);
    GamepadConfiguration GamepadConfig;
    GamepadConfig.setAutoReport(false);   // Manual report timing
    GamepadConfig.setButtonCount(14);
    gamepad = new GamepadDevice(GamepadConfig);
    compositeHID->addDevice(gamepad);
    compositeHID->begin();

    ui_boot_mode();
    uiState = UI_MENU;

  } else {
    // Xbox mode — separate MAC keeps it paired independently
    uint8_t xbMac[6] = { 0x24, 0x6F, 0x28, 0xAA, 0xAA, 0xA2 };
    esp_base_mac_addr_set(xbMac);

    compositeHID = new BleCompositeHID("GamePad V2 XBOX", "Kevin", 100);
    XboxSeriesXControllerDeviceConfiguration* config =
        new XboxSeriesXControllerDeviceConfiguration();
    BLEHostConfiguration hostConfig = config->getIdealHostConfiguration();
    XBOX_X = new XboxGamepadDevice(config);
    compositeHID->addDevice(XBOX_X);
    compositeHID->begin(hostConfig);
    delay(1000);

    // Xbox always boots straight into active mode (no menu needed)
    current_menu = 1;
    uiState = UI_ACTIVE;
    delay(1000);
  }

  ui_connection();
  delay(2500);
}

/**
 * loop()
 * Main event loop — runs continuously after setup().
 * Structure: state machine dispatching on uiState.
 *
 *  ┌─ BLE_Status() ────────────────── detect connect/disconnect
 *  ├─ If not connected ─────────────── bail out early
 *  └─ Switch on uiState ────────────── render UI + handle input
 *      ├─ UI_MENU            Main mode menu
 *      ├─ UI_ACTIVE          Active HID reporting
 *      ├─ UI_DEVELOPER_MODE  Sensor readout
 *      ├─ UI_ABOUT           Credits
 *      ├─ UI_SETTINGS        Settings menu
 *      ├─ UI_BRIGHTNESS      Brightness control
 *      ├─ UI_GYRO_SENSITIVITY Gyro level control
 *      ├─ UI_SENSITIVITY     Stick sensitivity control
 *      └─ UI_DEVICE_INFO     Device info screen
 */
void loop() {

  // Always poll BLE state and update OLED on change
  BLE_Status();

  // Don't process inputs until the host is connected
  if (!compositeHID->isConnected()) return;

  // ── UI_MENU ──────────────────────────────────────────────────
  if (uiState == UI_MENU) {
    draw_menu();

    // Navigate list with D-Pad Up / Down
    if (digitalRead(DPAD_DOWN) == LOW) {
      Selected_menu = (Selected_menu + 1) % Menu_Count;
      delay(150);
    }
    if (digitalRead(DPAD_UP) == LOW) {
      Selected_menu = (Selected_menu - 1 + Menu_Count) % Menu_Count;
      delay(150);
    }

    // Select with A button
    if (digitalRead(BTN_A) == LOW) {
      switch (Selected_menu) {

        case 0: // ── GamePad HID ─────────────────────────────
          if (bootMode == BOOT_XBOX)
            switch_gamepad_mode();   // Reboot into Gamepad mode
          else {
            current_menu = 0;
            uiState = UI_ACTIVE;
          }
          break;

        case 1: // ── Xbox HID ────────────────────────────────
          if (bootMode == BOOT_XBOX) {
            current_menu = 1;
            uiState = UI_ACTIVE;
          } else {
            boot_xbox_mode();        // Reboot into Xbox mode
          }
          break;

        case 2: // ── Developer Mode ──────────────────────────
          current_menu = 2;
          uiState = UI_DEVLOPER_MODE;
          break;

        case 3: // ── Settings ────────────────────────────────
          Selected_setting = 0;
          uiState = UI_SETTINGS;
          break;

        case 4: // ── About ───────────────────────────────────
          current_menu = 4;
          uiState = UI_ABOUT;
          break;
      }
      delay(150);
    }
  }

  // ── UI_ACTIVE  (HID reporting) ───────────────────────────────
  else if (uiState == UI_ACTIVE) {
    ui_active_mode();
    HandleGyroToggle();

    if (current_menu == 0) gamePad_mode();
    if (current_menu == 1) xbox_mode();

    // MODE_BTN exits back to menu
    if (digitalRead(MODE_BTN) == LOW) {
      uiState = UI_MENU;
      delay(50);
    }
  }

  // ── UI_DEVLOPER_MODE ─────────────────────────────────────────
  else if (uiState == UI_DEVLOPER_MODE) {
    ui_Developer_mode();
    if (digitalRead(DPAD_LEFT) == LOW) {
      uiState = UI_MENU;
      delay(150);
    }
  }

  // ── UI_ABOUT ─────────────────────────────────────────────────
  else if (uiState == UI_ABOUT) {
    UI_About();
    if (digitalRead(DPAD_LEFT) == LOW) {
      uiState = UI_MENU;
      delay(200);
    }
  }

  // ── UI_SETTINGS ──────────────────────────────────────────────
  else if (uiState == UI_SETTINGS) {
    Ui_Settings();

    if (digitalRead(DPAD_DOWN) == LOW) {
      Selected_setting = (Selected_setting + 1) % setting_count;
      delay(150);
    }
    if (digitalRead(DPAD_UP) == LOW) {
      Selected_setting = (Selected_setting - 1 + setting_count) % setting_count;
      delay(150);
    }

    if (digitalRead(BTN_A) == LOW) {
      switch (Selected_setting) {
        case 0: uiState = UI_BRIGHTNESS;       break;
        case 1: uiState = UI_GYRO_SENSITIVITY; break;
        case 2: uiState = UI_SENSITIVITY;      break;
        case 3: uiState = UI_DEVIVE_INFO;      break;
        case 4:
          mcu_sleep_animation();
          delay(1000);
          mcu_sleep();
          break;
      }
      delay(150);
    }

    if (digitalRead(DPAD_LEFT) == LOW) {
      uiState = UI_MENU;
      delay(150);
    }
  }

  // ── UI_BRIGHTNESS ─────────────────────────────────────────────
  else if (uiState == UI_BRIGHTNESS) {
    ui_brightness();

    if (digitalRead(BTN_B) == LOW && brightnessLevel < 10) {
      brightnessLevel++;
      apply_brightness();
      delay(100);
    }
    if (digitalRead(BTN_X) == LOW && brightnessLevel > 1) {
      brightnessLevel--;
      apply_brightness();
      delay(100);
    }
    if (digitalRead(DPAD_LEFT) == LOW) {
      uiState = UI_SETTINGS;
      delay(150);
    }
  }

  // ── UI_GYRO_SENSITIVITY ──────────────────────────────────────
  else if (uiState == UI_GYRO_SENSITIVITY) {
    ui_Gyro_Sensitivity();

    if (digitalRead(BTN_B) == LOW && gyroSensitivity < GYRO_HIGH) {
      gyroSensitivity = (GyroSensitivityLevel)(gyroSensitivity + 1);
      delay(150);
    }
    if (digitalRead(BTN_X) == LOW && gyroSensitivity > GYRO_LOW) {
      gyroSensitivity = (GyroSensitivityLevel)(gyroSensitivity - 1);
      delay(150);
    }
    if (digitalRead(DPAD_LEFT) == LOW) {
      uiState = UI_SETTINGS;
      delay(200);
    }
  }

  // ── UI_SENSITIVITY ────────────────────────────────────────────
  else if (uiState == UI_SENSITIVITY) {
    ui_sensitivity();

    if (digitalRead(BTN_B) == LOW && sensitivity < SENS_HIGH) {
      sensitivity = (SensitivityLevel)(sensitivity + 1);
      delay(150);
    }
    if (digitalRead(BTN_X) == LOW && sensitivity > SENS_LOW) {
      sensitivity = (SensitivityLevel)(sensitivity - 1);
      delay(150);
    }
    if (digitalRead(DPAD_LEFT) == LOW) {
      uiState = UI_SETTINGS;
      delay(200);
    }
  }

  // ── UI_DEVIVE_INFO ────────────────────────────────────────────
  else if (uiState == UI_DEVIVE_INFO) {
    ui_device_info();
    if (digitalRead(DPAD_LEFT) == LOW) {
      uiState = UI_SETTINGS;
      delay(150);
    }
  }
}
