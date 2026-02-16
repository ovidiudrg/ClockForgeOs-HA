# Changelog

## 0.1.28 - 2026-02-16
- Hotfix: fixed syntax error in `switch.py` that prevented integration platform import/setup.
- Updated integration metadata to version `0.1.28`.

## 0.1.27 - 2026-02-16
- Fixed config flow compatibility issue causing 500 errors on some Home Assistant builds.
- Replaced password selector widget usage with plain string fields in setup/options flow.
- Updated integration metadata to version `0.1.27`.

## 0.1.26 - 2026-02-16
- Improved setup resilience:
  - integration no longer hard-fails config entry setup when initial refresh fails temporarily.
  - setup continues with empty data and coordinator retries in background.
- Helps avoid `Failed to set up` during transient reboot/auth/network windows.
- Updated integration metadata to version `0.1.26`.

## 0.1.25 - 2026-02-16
- Added periodic re-auth retry when password is configured but token is missing/expired.
- This closes a gap where state could remain stale until a manual write if no 401-triggered re-auth path occurred.
- Updated integration metadata to version `0.1.25`.

## 0.1.24 - 2026-02-16
- Hardened reboot-state protection for switches:
  - increased reboot grace window to 8 minutes.
  - transient `false` values during reboot grace no longer overwrite a previously known `true` state, regardless of source (`config` or `current_info`).
- Updated integration metadata to version `0.1.24`.

## 0.1.23 - 2026-02-16
- Fixed remaining post-reboot auth desync behavior:
  - when `/getConfiguration` returns unauthorized, integration now marks re-auth needed.
  - coordinator performs targeted re-auth with cooldown and retries config fetch in the same poll cycle.
- Eliminates the need to press a switch to force settings sync after reboot/token desync.
- Updated integration metadata to version `0.1.23`.

## 0.1.22 - 2026-02-16
- Fixed automatic switch status recovery regression:
  - configuration switches now prefer persisted config values over transient boot-time `current_info` defaults.
  - keeps `displayPower` reading priority on live/current values.
- Updated integration metadata to version `0.1.22`.

## 0.1.21 - 2026-02-16
- Sensor cleanup and UI polish:
  - removed additional noisy/internal diagnostic sensors from default exposure.
  - added automatic cleanup for already-registered removed sensor entities.
  - improved display labels and icons for key remaining sensors (time/date, WiFi IP/SSID, CPU, LED units).
- Updated integration metadata to version `0.1.21`.

## 0.1.20 - 2026-02-16
- Added automatic auth refresh on reboot detection:
  - detects reboot via `uptimeMinutes` reset/early-boot window.
  - when password is configured and token is not valid, re-authenticates automatically and refreshes config.
- Fixes the remaining case where settings only recovered after pressing a button.
- Updated integration metadata to version `0.1.20`.

## 0.1.19 - 2026-02-16
- Expanded sensor exclusion key coverage for reported variants:
  - added `dayNightIsNight`, `wifiSwitchRollbackSsid`, `wifiSwitchTargetSsid`.
- Added automatic cleanup of already-registered removed sensor entities in HA entity registry.
- Updated integration metadata to version `0.1.19`.

## 0.1.18 - 2026-02-16
- Improved automatic state recovery after clock reconnect:
  - On reconnect, integration performs a one-time auth refresh (when password is set) and pulls config automatically.
  - Config endpoint now keeps last known good payload as fallback during token expiry/unavailable windows.
- This removes the need to press a HA button to force state refresh after reboot.
- Updated integration metadata to version `0.1.18`.

## 0.1.17 - 2026-02-16
- Added reboot grace handling for switch states:
  - During first 3 minutes after device reboot (`uptimeMinutes`), transient `false` values do not overwrite previously known `true` switch states.
  - Prevents post-reboot state flicker/reset in HA until stable payloads return.
- Updated integration metadata to version `0.1.17`.

## 0.1.16 - 2026-02-16
- Improved switch state stability during clock reboot:
  - Switch entities now keep last known state when firmware keys are temporarily missing.
  - Prevents HA controls from briefly reverting to default OFF until a manual action.
- Updated integration metadata to version `0.1.16`.

## 0.1.15 - 2026-02-16
- Removed additional non-actionable/internal fields from sensor entities:
  - `dayNight`, `isNight`, `eepromSize`, `eepromUsed`, `hv5122Pins`,
    `largestFreeBlock`, `totalBytes`, `usedBytes`,
    `wifiSwitchResult`, `wifiSwitchRollbackRunning`,
    `wifiSwitchRollbackSSID`, `wifiSwitchTargetSSID`.
- Added exclusion for common firmware typo variants:
  - `wifiSwithcResult`, `wifiSwtichTargetSSID`.
- Updated integration metadata to version `0.1.15`.

## 0.1.14 - 2026-02-16
- Improved optional secondary sensor handling:
  - `temperature2`, `humidity2`, and `pressure2` are treated as invalid when reported as `255`/`255.0`.
  - These sensors are now exposed only when a valid value exists.
- Updated integration metadata to version `0.1.14`.

## 0.1.13 - 2026-02-16
- Switched `Alarm Minute` back to number-box control to match `Alarm Hour` UI style.
- Kept alarm numeric entities integer-only to avoid trailing decimal formatting like `.0`.
- Added migration cleanup for legacy `alarmMinute` select entity in HA entity registry.
- Updated integration metadata to version `0.1.13`.

## 0.1.12 - 2026-02-16
- Added automatic entity-registry cleanup for legacy alarm minute entity migration:
  - Removes old `number` entity (`alarmTimeMinutes`) on setup so the new two-digit minute select is used.
- Updated integration metadata to version `0.1.12`.

## 0.1.11 - 2026-02-16
- Updated alarm minute control rendering in Home Assistant:
  - `Alarm Minute` is now a select with fixed two-digit options (`00` to `59`).
  - This avoids single-digit display like `0` and keeps consistent `00` formatting.
- Updated integration metadata to version `0.1.11`.

## 0.1.10 - 2026-02-16
- Added 12-hour alarm controls in Home Assistant:
  - `Alarm Hour` now displays and edits as `1-12` (instead of `0-23`).
  - New `Alarm AM/PM` select to control meridiem.
- Kept firmware compatibility by converting HA alarm hour changes to 24-hour values for `/saveSetting`.
- Updated integration metadata to version `0.1.10`.

## 0.1.9 - 2026-02-16
- Updated Home Assistant control rendering:
  - `Alarm Hour` and `Alarm Minute` now use numeric input boxes (not sliders).
- Documentation refresh:
  - README now reflects current entities, including `Radar Motion` binary sensor and RGB palette select.
- Updated integration metadata to version `0.1.9`.

## 0.1.8 - 2026-02-16
- Improved default Home Assistant UI clarity:
  - core number controls stay visible by default (`rgbBrightness`, `rgbAnimationSpeed`, `tubesWakeSeconds`, alarm timing/period).
  - advanced tuning numbers are now disabled by default to reduce clutter/noisy cards.
- Fixed number entity creation to only include keys that are currently available on the device payloads.
- Updated integration metadata to version `0.1.8`.

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
