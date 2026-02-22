# TODO Review Notes

Date: 2026-02-20
Scope: Static code review of current workspace state (no build run in this environment).

## 1) NeoPixel boot hint brightness parameter ignored
- Severity: High
- Files: `z_neopixel.ino:239`, `z_neopixel.ino:245`
- Issue:
  - `showSolidNeopixels(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)` receives `brightness`
  - but calls `applyCurrentLimitAndShow(neoBrightness)` instead of using the function argument.
- Risk:
  - AP/STA boot hint color can be dim or invisible when runtime `neoBrightness` is low/0.

## 2) `doSave` flag is not used (runtime-only settings still saved)
- Severity: High
- Files: `ClockForgeOS.ino:2150`, `ClockForgeOS.ino:2272`, `ClockForgeOS.ino:2671`
- Issue:
  - `doSave` is set to `false` for runtime-only keys, but the final block always calls `requestSaveEEPROM()` when `paramFound`.
- Risk:
  - Unnecessary EEPROM writes and mismatch between intended and actual behavior.

## 3) Touch action `COLOR_PREV` exists but cannot be configured
- Severity: Medium
- Files: `ClockForgeOS.ino:1156`, `ClockForgeOS.ino:2235`, `ClockForgeOS.ino:2243`, `ClockForgeOS.ino:2251`, `ClockForgeOS.ino:5843`
- Issue:
  - Enum/executor supports `TOUCH_ACTION_COLOR_PREV = 5`
  - but config parsing clamps max action to `TOUCH_ACTION_DISPLAY_TOGGLE = 4`.
- Risk:
  - Feature is implemented but unreachable via API/UI.

## 4) Wrong preprocessor guard for `TUBE_POWER_ON`
- Severity: Medium
- File: `clocks.h:2544`
- Issue:
  - Guard uses `#ifndef TUBE_POWER_PIN` before defining `TUBE_POWER_ON`.
  - Expected guard is `#ifndef TUBE_POWER_ON`.
- Risk:
  - Wrong defaulting behavior on profiles that define `TUBE_POWER_PIN` but not `TUBE_POWER_ON`.

## 5) PWM LEDs may stay on while tubes are off (non-CLOCK_54 path)
- Severity: Medium
- Files: `pwmleds.ino:317`, `ClockForgeOS.ino:5938`
- Issue:
  - non-`CLOCK_54` off condition in `doAnimationPWM()` does not include `!tubesPowerState`.
  - `doAnimationPWM()` is called every loop.
- Risk:
  - PWM backlight can remain active when display/tubes are logically OFF.

## Validation gap
- `arduino-cli` is not installed in this environment, so compile/runtime validation was not executed here.
