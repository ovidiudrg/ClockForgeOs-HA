# Changelog

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
