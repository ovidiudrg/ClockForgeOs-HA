# ClockForgeOS for Home Assistant (HACS)

![ClockForgeOS icon](assets/favicon.svg)

Custom integration for ClockForgeOS devices.

## Release
- Current version: **0.1.26**
- Changelog: `CHANGELOG.md`

## Features
- Auto config flow from UI
- Reads system info from `/getSystemInfo`
- Reads live state from `/getCurrentInfos`
- Exposes dynamic sensors from firmware payloads (including temperature, humidity, pressure, lux, and CPU diagnostics when available)
- Exposes control entities:
  - Switches: `displayPower`, `wakeOnMotionEnabled`, `alarmEnable`, `showTimeDate`, `showTemperature`, `showHumidity`, `showPressure`
  - Numbers: `rgbBrightness`, `rgbAnimationSpeed`, `tubesWakeSeconds`, `alarmTimeHours` (`Alarm Hour`), `alarmTimeMinutes` (`Alarm Minute`) (plus additional numeric firmware keys)
  - Selects: `rgbEffect`, `rgbPalette`, `alarmAmPm` (`Alarm AM/PM`)
  - Binary sensor: `Radar Motion`
- Supports password-protected writes using firmware token auth (`/auth/login` + `X-Auth-Token`)

## Installation (HACS)
1. In HACS, add this repository as a **Custom repository** of type **Integration**.
2. Install **ClockForgeOS**.
3. Restart Home Assistant.
4. Go to **Settings -> Devices & Services -> Add Integration**.
5. Search for **ClockForgeOS** and enter your device host/IP.

## Required firmware endpoints
Your ClockForgeOS firmware should expose:
- `GET /getSystemInfo`
- `GET /getCurrentInfos`
- `POST /saveSetting` (with `key`, `value`)
- `POST /auth/login` (required only if write password is enabled on firmware)

## Notes
- This integration uses HTTP polling.
- If your firmware requires auth for writes, set the device **password** when adding the integration (or in options later).
- `Alarm Hour` and `Alarm Minute` are rendered as numeric input boxes in Home Assistant (12-hour format for hour).
- Use `Alarm AM/PM` to switch alarm meridiem.
- Icon asset used by this repo is `assets/favicon.svg`.
- For the icon to appear in Home Assistant/HACS UI, publish the same icon to Home Assistant brands for domain `clockforgeos`.
