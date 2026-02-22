# ClockForgeOS Mobile HTTP API v1

This document defines a practical API contract for a mobile app, mapped to the existing firmware endpoints.

## 1) Base URL

- Local network URL: `http://<clock-ip>`
- Example: `http://192.168.1.120`

## 2) Authentication

Protected endpoints require token auth.

1. Login:
- `POST /auth/login`
- Content type: `application/x-www-form-urlencoded`
- Body: `password=<WEB_ADMIN_PASSWORD>`

Response:
```json
{
  "ok": true,
  "token": "abc123...",
  "ttlSec": 1800,
  "passwordConfigured": true,
  "passwordWeak": false,
  "mustChangePassword": false
}
```

2. Send token for protected routes:
- Header (recommended): `X-Auth-Token: <token>`
- Fallback body/query key accepted by firmware: `authToken=<token>`

3. On `401 {"error":"unauthorized"}`:
- Re-login and retry once.

4. Optional password rotation (recommended):
- `POST /auth/changePassword`
- Content type: `application/x-www-form-urlencoded`
- Body:
  - `currentPassword=<current_password>`
  - `newPassword=<new_password>`
- Policy:
  - minimum 10 chars
  - must include uppercase, lowercase, digit
  - no spaces

## 3) Endpoint Map For Mobile

### 3.1 Runtime status (polling)
- `GET /getCurrentInfos` (no auth in current firmware)
- Poll every `1000-2000 ms` when app is foreground.

Useful fields:
- `currentDate`, `currentTime`, `timeSource`
- `temperature1`, `temperature2`, `humidity1`, `humidity2`, `pressure`, `lux`
- `tubesPower`, `dayNightMode`, `wakeSecondsLeft`
- `rssi`, `mqttStatus`
- `ledCurrentmA`, `ledAppliedBrightness`, `ledLimitmA`

### 3.2 Full configuration
- `GET /getConfiguration` (auth required)
- Returns current settings and capability flags.

### 3.3 Save one setting
- `POST /saveSetting` (auth required)
- Content type: `application/x-www-form-urlencoded`
- Body:
  - `key=<settingKey>`
  - `value=<settingValue>`

Success:
- HTTP 200 + text body `Ok`

Common keys:
- `displayPower`, `manualDisplayOff`
- `enableTimeDisplay`, `enableTempDisplay`, `enableHumidDisplay`, `enablePressDisplay`
- `rgbEffect`, `rgbBrightness`, `rgbSpeed`, `rgbDir`, `rgbFixR`, `rgbFixG`, `rgbFixB`, `maxLedmA`, `rgbNightEnabled`
- `uiWidth`, `uiBgColor`, `uiPanelColor`, `uiAccentColor`, `uiTextColor`

### 3.4 Set manual time
- `POST /setManualTime` (auth required)
- Option A: `epoch=<unix_seconds>`
- Option B: `date=YYYY-MM-DD` + `time=HH:MM[:SS]`

### 3.5 Diagnostics
- `GET /getSystemInfo` (currently open in firmware)
- `GET /getClockDetails` (HTML block)

### 3.6 WiFi flow
- `GET /scanWifi` (auth required)
- `POST /connectWifi` (auth required), body: `ssid`, `psw`, `save`
- `GET /wifiStatus` (auth required)

### 3.7 Actions
- `POST /reset`
- `POST /factoryreset`
- `POST /firmwareupdate`
- `POST /cathodeProtect`

## 4) Suggested Mobile Service Contract

Use this stable app-side interface:

- `login(password) -> {token, ttlSec}`
- `getStatus() -> CurrentInfos`
- `getConfig() -> Map<String,dynamic>`
- `saveSetting(key, value) -> bool`
- `setManualTimeEpoch(epoch) -> bool`
- `getSystemInfo() -> Map<String,dynamic>`

Internally it maps to existing firmware routes, so firmware can evolve later without breaking app code.

## 5) Polling + reliability

- Status polling interval: `1500 ms` default.
- Request timeout: `3-5 s`.
- Backoff sequence on network fail: `2s, 4s, 8s` then cap `10s`.
- Retry once after re-login when a protected endpoint returns `401`.

## 6) Security notes

- Current transport is plain HTTP on LAN.
- Recommended for production:
  - keep clock in trusted LAN/VLAN
  - use strong admin password
  - avoid exposing clock port to WAN
  - optionally add reverse proxy/TLS gateway later
