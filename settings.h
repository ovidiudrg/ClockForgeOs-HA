#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>

/*
  settings.h
  ==========
  Persistent (EEPROM) user settings for UniClock.

  Purpose:
  - Keep user/UI persisted configuration separate from model/hardware configuration (clocks.h).
  - Provide a single "owner" for settings loaded/saved from EEPROM.

  Versioning:
  - SETTINGS_SCHEMA_VERSION is the EEPROM layout version for this struct.
  - Bump it ONLY when you change the struct layout in an incompatible way.
*/

static constexpr uint16_t SETTINGS_SCHEMA_VERSION = 0x0109; // UniClock settings schema v1.9

// Value used to detect "unset" RGB values when migrating from older EEPROM contents.
static constexpr uint8_t RGB_UNSET = 0xFF;

struct Settings {
  uint16_t schemaVersion;   // must be SETTINGS_SCHEMA_VERSION

  // --- Debug (UI) ---
  bool debugEnabled;        // Enables SSE log streaming when true

  // --- NeoPixel fixed color as true RGB triplet (user-selectable) ---
  // Used when rgbEffect == "fixed color" in firmware logic.
  // If any of these are RGB_UNSET (0xFF) after load, firmware will replace them with defaults.
  uint8_t rgbFixR;
  uint8_t rgbFixG;
  uint8_t rgbFixB;

  // --- UI Customization (persisted across all browsers/devices) ---
  uint16_t uiWidth;         // Page width in pixels (0xFFFF = 100% full width)
  uint8_t uiBgR;            // Background color: Red
  uint8_t uiBgG;            // Background color: Green
  uint8_t uiBgB;            // Background color: Blue
  uint8_t uiPanelR;         // Panel/Card color: Red
  uint8_t uiPanelG;         // Panel/Card color: Green
  uint8_t uiPanelB;         // Panel/Card color: Blue
  uint8_t uiAccentR;        // Accent color: Red
  uint8_t uiAccentG;        // Accent color: Green
  uint8_t uiAccentB;        // Accent color: Blue
  uint8_t uiTextR;          // Text color: Red
  uint8_t uiTextG;          // Text color: Green
  uint8_t uiTextB;          // Text color: Blue

  // --- Tube sensor display toggles ---
  bool enableTempDisplay;   // Show temperature on tubes
  bool enableHumidDisplay;  // Show humidity on tubes
  bool enablePressDisplay;  // Show pressure on tubes
  uint8_t pressureStart;    // Pressure display start second (0..59)
  uint8_t pressureEnd;      // Pressure display end second (0..59)
  uint8_t rgbNightEnabled;  // 0/1: allow RGB effects during automatic night mode

  // --- MQTT inbound topics (runtime configurable from UI) ---
  // Empty string = disabled.
  char mqttInTempTopic[96];
  char mqttInHumidTopic[96];
  char mqttInRadarTopic[96];
  char mqttInLuxTopic[96];
  char mqttInPressureTopic[96];

  // --- RGB AutoDim tuning ---
  uint8_t rgbAutoDimDayMinPercent; // 0..100: minimum RGB level in DAY when auto-dim is active
};

// Global settings instance (defined in a .ino/.cpp)
extern Settings settings;

// Set when something changed and needs persisting
extern volatile bool settingsDirty;

// Defaults / persistence API (implemented in ESP32_UniClock2.ino)
void settingsSetDefaults();
bool settingsLoadFromEEPROM();
void settingsSaveToEEPROM();

// Helper: call when a setting is updated
inline void settingsMarkDirty() { settingsDirty = true; }

#endif // SETTINGS_H
