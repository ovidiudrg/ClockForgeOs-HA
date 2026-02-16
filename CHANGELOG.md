# Changelog

## 0.1.7 - 2026-02-16
- Stabilized auth/session behavior to reduce web UI password churn:
  - `get_configuration` now uses passive token behavior and never logs in from polling.
- Improved entity state consistency:
  - switches now use expiring last-known state fallback and clearer `Wake On Motion` label.
  - select effect parsing now normalizes numeric/string values for reliable RGB effect sync.
  - sensors now read `config` (coordinator payload) instead of stale `public_config` key.
- Improved numeric entity robustness:
  - number entities are always created and mark themselves unavailable if unsupported.
  - number reads now include config fallback where appropriate.
- Removed unused/unsafe `button` platform implementation.
- Updated integration metadata to version `0.1.7`.

## 0.1.6 - 2026-02-15
- Fixed integration load errors that caused `Config flow could not be loaded: {"message":"Invalid handler specified"}`.
- Reworked API write authentication to match ClockForgeOS firmware token flow:
  - `POST /auth/login` with configured password
  - `POST /saveSetting` using `X-Auth-Token`
  - automatic token refresh/retry on `401`
- Enabled all supported HA platforms in setup: `sensor`, `switch`, `number`, `select`.
- Added/validated requested control entities:
  - Switches: `displayPower`, `alarmEnable`, `showTimeDate`, `showTemperature`, `showHumidity`, `showPressure`
  - Numbers: `rgbBrightness`, `rgbAnimationSpeed`
  - Select: `rgbEffect`
- Fixed `number.py` key list issues that could break platform setup.
- Updated integration metadata to version `0.1.6`.

## 0.1.5 - 2026-02-15
- Expanded `/saveSetting` compatibility with additional password key variants (`adminPassword`, `pass`, `pwd`).
- Added write fallbacks across request styles (form POST, JSON POST, and GET query fallback).
- Updated integration metadata to version `0.1.5`.

## 0.1.4 - 2026-02-15
- Added explicit `admin_username` and `admin_password` fields in both setup and options flows.
- Updated write authentication to use configured admin username for HTTP basic auth.
- Updated integration metadata to version `0.1.4`.

## 0.1.3 - 2026-02-15
- Improved `/saveSetting` authentication compatibility for devices returning `401 Unauthorized`.
- Added fallback auth strategies for write operations (password fields, query params, and HTTP basic auth with `admin`).
- Updated integration metadata to version `0.1.3`.

## 0.1.2 - 2026-02-15
- Added optional `admin_password` setting in config flow and options flow.
- Added password-aware `/saveSetting` payload to support protected write operations.
- Updated integration metadata to version `0.1.2`.

## 0.1.1 - 2026-02-15
- Fixed `hacs.json` Home Assistant minimum version format for HACS compatibility.
- Updated integration version metadata to `0.1.1`.

## 0.1.0 - 2026-02-15
- Initial HACS release of the ClockForgeOS custom integration.
- Added config flow for host/IP setup and poll interval options.
- Added polling coordinator using `/getSystemInfo` and `/getCurrentInfos`.
- Added diagnostic sensors (OS version, firmware ID, WiFi status/signal, CPU temperature, heap, uptime).
- Added display power switch using `/saveSetting` (`displayPower`).
- Added Home Assistant translation strings and repository metadata.
