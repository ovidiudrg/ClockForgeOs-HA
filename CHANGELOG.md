# Changelog

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
