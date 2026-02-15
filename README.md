# ClockForgeOS for Home Assistant (HACS)

Custom integration for ClockForgeOS devices.

## Release
- Current version: **0.1.1**
- Changelog: `CHANGELOG.md`

## Features
- Auto config flow from UI
- Reads system info from `/getSystemInfo`
- Reads live state from `/getCurrentInfos`
- Exposes key sensors in Home Assistant
- Exposes a display power switch using `/saveSetting`

## Installation (HACS)
1. In HACS, add this repository as a **Custom repository** of type **Integration**.
2. Install **ClockForgeOS**.
3. Restart Home Assistant.
4. Go to **Settings → Devices & Services → Add Integration**.
5. Search for **ClockForgeOS** and enter your device host/IP.

## Required firmware endpoints
Your ClockForgeOS firmware should expose:
- `GET /getSystemInfo`
- `GET /getCurrentInfos`
- `POST /saveSetting` (with `key`, `value`)

## Notes
- This integration uses HTTP polling.
- If auth is enabled on firmware endpoints, extend `api.py` with headers/token.
