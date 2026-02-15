# ClockForgeOS

Advanced universal clock firmware + web UI for ESP32/ESP8266 Nixie/VFD/LED/Numitron style clocks.

Features optional Dallas Thermometer, DS3231 RTC, Neopixel integration, GPS, and comprehensive web-based control interface.

## UI/API Endpoints

All API calls are served by the AsyncWebServer in `ClockForgeOS.ino`.

### 1) Realtime stream

#### `GET /events`
- Type: Server-Sent Events (SSE)
- Event names used:
  - `status` (for example `connected`, `debug_disabled`)
  - `log` (debug log lines)
- Notes:
  - stream is intentionally gated by runtime debug state
  - client should auto-reconnect with backoff

Example (browser/devtools friendly):
```js
const es = new EventSource('/events');
es.addEventListener('status', e => console.log('status', e.data));
es.addEventListener('log', e => console.log('log', e.data));
```

### 2) Configuration + runtime data

#### `GET /getConfiguration`
- Returns: JSON object with persisted configuration values used by UI and runtime.
- Includes (non-exhaustive): time/display settings, RGB settings, WiFi/MQTT settings, wake/sleep controls, UI customization values.

#### `GET /getSystemInfo`
- Returns: JSON object with diagnostics and capability information.
- Display: System Info card in web UI showing OS Version, FirmwareID, Chip model, and more
- Includes (non-exhaustive):
  - OS/firmware info (`osVersion`, `firmwareID`, `tubeDriver`, `chipModel`, `maxBrightness`, `maxDigits`)
  - heap stats (`freeHeap`, `totalHeap`, `heapUsedPercent`, `largestFreeBlock`, `minFreeHeap`, `heapProtection`)
  - CPU/chip info (`cpuTemp`, `cpuFreqMHz`, `chipModel`, `cpuCores`, `uptime`, `uptimeMinutes`)
  - storage (SPIFFS/LittleFS) (`totalBytes`, `usedBytes`, `freeBytes`)
  - EEPROM info (`eepromSize`, `eepromUsed`)
  - network status (`wifiStatus`, `wifiSignal`, `wifiIP`, `wifiSsid`, `macAddress`, `gateway`, `subnet`)
  - WiFi switch state (`wifiSwitchPending`, `wifiSwitchResult`, `wifiSwitchTargetSsid`, `wifiSwitchRollbackRunning`, `wifiSwitchRollbackSsid`)
  - MQTT status (`mqttStatus`)
  - sensor summary (`temperatureSensors`, `humiditySensors`, `pressureSensors`, `installedSensors`, `physicalSensors`, `virtualSensors`, `gestureSensorPresent`)

#### `GET /getCurrentInfos`
- Returns: JSON object with high-frequency current values for dashboard.
- Includes (non-exhaustive): current date/time, sensor values, display state, wake/sleep runtime, LED limiter telemetry.
- Server-side short cache is used to reduce multi-client load.

#### `GET /getClockDetails`
- Returns: `text/html` block used by the details panel.
- Includes firmware ID, tube driver, MAC, sensor counts, used pins, and driver setup strings.

### 3) Settings write endpoints

#### `POST /saveSetting`
- Content type: form-urlencoded (`key`, `value` as POST body params).
- Required fields:
  - `key`
  - `value`
- Success: `200 text/plain` with `Ok`
- Errors:
  - `400` invalid request/missing fields
  - `404` unsupported key

Frequently used keys:
- Display/time: `displayPower`, `manualDisplayOff`, `wakeOnMotionEnabled`, `tubesWakeSeconds`, `enableTimeDisplay`, `manualOverride`, `set12_24`, `enableBlink`, `enableDoubleBlink`
- Sensors schedule: `dateStart`, `dateEnd`, `tempStart`, `tempEnd`, `humidStart`, `humidEnd`, `pressureStart`, `pressureEnd`, `dateRepeatMin`, `tempRepeatMin`
- RGB: `rgbEffect`, `rgbBrightness`, `rgbSpeed`, `rgbDir`, `rgbFixR`, `rgbFixG`, `rgbFixB`, `maxLedmA`
- Service/runtime: `onboardLed`, `animMode`
- WiFi/MQTT: `wifiSsid`, `wifiPsw`, `mqttEnable`, `mqttBrokerAddr`, `mqttBrokerUser`, `mqttBrokerPsw`, `mqttBrokerRefresh`
- UI theme/layout: `uiWidth`, `uiBgColor`, `uiPanelColor`, `uiAccentColor`, `uiTextColor`

Example:
```bash
curl -X POST http://CLOCK_IP/saveSetting \
  -d "key=rgbEffect" \
  -d "value=6"
```

#### `POST /setManualTime`
- Content type: form-urlencoded.
- Accepts either:
  - `epoch=<unix_seconds>`
  - OR `date=YYYY-MM-DD` and `time=HH:MM[:SS]`
- Success: `200 text/plain` with `Ok`
- Errors: `400` for invalid format or missing required params

Examples:
```bash
curl -X POST http://CLOCK_IP/setManualTime -d "epoch=1765812000"
curl -X POST http://CLOCK_IP/setManualTime -d "date=2026-02-13" -d "time=17:45"
```

### 4) WiFi endpoints

#### `GET /scanWifi`
- Starts async scan if idle, or returns in-progress/cached results.
- Typical responses:
  - `{\"status\":\"scanning\"}`
  - `[]` on failure/no entries
  - `[ {\"ssid\":...,\"rssi\":...,\"enc\":...}, ... ]`
  - busy state when WiFi switch/connect is active

#### `GET /connectWifi`
- Query params:
  - `ssid` (required)
  - `psw` (optional)
- Behavior: starts connect attempt without saving credentials.

#### `POST /connectWifi`
- Body params:
  - `ssid` (required)
  - `psw` (optional)
  - `save` (`true`/`false`, optional)
- Behavior: starts connect attempt and optionally persists credentials.

#### `GET /wifiStatus`
- Returns connection + switch state JSON.
- Includes (non-exhaustive):
  - `status`, `statusStr`, `ssid`, `ip`, `rssi`
  - `wifiSwitchPending`, `wifiSwitchResult`, `wifiSwitchTargetSsid`, `wifiSwitchRollbackSsid`

Examples:
```bash
curl "http://CLOCK_IP/connectWifi?ssid=MySSID&psw=MyPass"
curl -X POST http://CLOCK_IP/connectWifi -d "ssid=MySSID" -d "psw=MyPass" -d "save=true"
curl http://CLOCK_IP/wifiStatus
```

### 5) Control / maintenance

#### `POST /reset`
- Reboots the device.

#### `POST /factoryreset`
- Clears settings to defaults and reboots.

#### `POST /firmwareupdate`
- Queues firmware update task.
- Returns JSON confirmation message for modal UI.

#### `POST /cathodeProtect`
- Start/stop toggle endpoint.
- If already running, sends stop request; otherwise starts routine.

Examples:
```bash
curl -X POST http://CLOCK_IP/reset
curl -X POST http://CLOCK_IP/factoryreset
curl -X POST http://CLOCK_IP/firmwareupdate
curl -X POST http://CLOCK_IP/cathodeProtect
```

### 6) Web UI assets
- `GET /` → `index.html`
- `GET /page.js`
- `GET /site.css`
- `GET /jquery.js` (alias)
- `GET /jquery_351.js`
- `GET /jquery_360.js`
- `GET /favicon.ico`

### 7) Captive portal / probe handling
- `GET /generate_204` — captive redirect to local AP page
- `GET /hotspot-detect.html` — probe response
- `GET /connecttest.txt` — probe response

## Implemented In This Session

This section summarizes all functional changes completed in this chat.

### 1) RGB Current Limiter + Telemetry
- Fully wired `maxLedmA` setting end-to-end:
  - save via `/saveSetting`
  - load via `/getConfiguration`
  - factory default set to `350 mA`
- Added runtime telemetry from NeoPixel limiter:
  - applied current (`ledCurrentmA`)
  - applied brightness (`ledAppliedBrightness`)
- Added live dashboard line showing current, limit, and applied brightness.
- Added threshold color styling for the current line using existing badge styles.

### 2) Fixed RGB Rendering + UI Alignment
- Fixed `fixColor()` mapping so LEDs not mapped to tubes are forced black.
- Fixed fixed-color section layout issue in CSS when single child existed in color holder.

### 3) Firmware Update Flow
- Fixed frontend endpoint mismatch (`/firmwareupdate/` -> `/firmwareupdate`).
- Backend firmware update route now returns JSON confirmation immediately.
- Removed accidental update trigger when editing firmware server text field.

### 4) WiFi / Manual Time UI
- Restored missing `AP` label on WiFi AP/Client switch.
- Reworked manual date/time UI to single `datetime-local` input.
- Renamed button text from `Use Browser Time` to `Browser Time`.
- Kept compatibility by splitting datetime into date/time in JS before posting.

### 5) Firmware-Specific UI Rule
- For firmware `FW == "fw64"`, hides/disables animation option `5` (Transition).
- Auto-fallback to mode `0` if mode `5` was selected.

### 6) Pressure Support On Tubes
- Added pressure into tube display scheduler and display priority chains.
- Added dedicated pressure render routine and activation flag.
- Pressure can now rotate on tubes like date/temp/humidity.

### 7) Display Scheduling Controls (UI + Firmware)
- Added independent ON/OFF controls for:
  - Temperature display
  - Humidity display
  - Pressure display
- Added pressure seconds window controls:
  - `pressureStart`
  - `pressureEnd`
- `Date seconds` now has priority in its own window (for example `30-35`) even with overlapping sensor windows.
- Repeat control covers all three: `Temp/H/P Repeat (min)`.

### Recommended Default Schedule
- Date seconds: `30-35`
- Temperature seconds: `35-40`
- Humidity seconds: `40-45`
- Pressure seconds: `50-55`
- Date repeat: `1-3 min` (based on preference)
- Temp/H/P repeat: `1 min` (continuous cycle)

Notes:
- Keep each window `start < end`.
- Keep windows non-overlapping for predictable rotation.
- Date has display priority in its own window.

### 8) Sensor #2 Cleanup
- Tube scheduler suppresses Temp2/Humidity2 display when second sensor values are invalid/near-zero.
- Dashboard auto-hides Temp2/Humidity2 cards when value is `255` or `0`.

### 9) Pressure Numeric Format On Tubes
- Updated 6/8-digit pressure format to display clock-style integer+decimals.
- Corrected ordering so values render like `987.40` (not reversed).

### 10) Cathode Protect Runtime Control
- Cathode protect can be manually stopped after start.
- Same action endpoint now behaves as start/stop toggle when running.
- Added stop checks inside running protect loops for clean exit.
- Button label is dynamic from runtime status:
  - `Cathode Protect` (idle)
  - `Stop Cathode Protect` (running)

### 11) RGB Effects Hardening (All Effects)
- Root cause fixed for random white/blue flashes:
  - removed out-of-bounds `tubePixels` access caused by map/pixel count mismatch
- Added safe mapped-pixel helper and bounds checks.
- Forced all unmapped/extra pixels to black before each `Show()`.
- Reviewed and updated all effect paths (`rainbow`, `effect*`, `kitt`, `heartbeat`, fixed color path).
- Additional anti-flash update:
  - excluded `WHITE_INDEX` from random/palette animation sources (`effect1`, `effect2`, `effect3`, `effect4`)
  - keeps white only where explicitly intended (for example alarm flash), preventing unintended random white bursts.

### 12) Multi-Client Web Stability / Lighter Runtime
- Reduced runtime/log pressure for multi-browser access:
  - removed noisy per-request web route debug prints
  - removed high-frequency `WebRefresh` debug spam
  - reduced in-memory log ring size (`LOG_LINES`) to lower RAM usage
- Added short server-side cache windows for hot API endpoints:
  - `/getCurrentInfos`
  - `/getSystemInfo`
- Live log/SSE now follows debug state:
  - SSE is rejected/closed when debug is disabled
  - frontend starts/stops SSE with debug toggle
  - live log DOM text is capped to avoid long-session memory growth
- System page polling was relaxed to reduce request storms from multiple open clients.

### 13) NeoPixel Sleep/Wake Stability (Validated)
- Ambient LEDs now use a one-shot OFF path while display is sleeping:
  - black frame + brightness `0`
  - no repeated OFF writes every loop while still sleeping
- Effects cleanly resume when display turns ON.
- Additional flash fix in timed wait paths:
  - `Fdelay()` now follows the same one-shot OFF behavior (no periodic OFF re-writes).
- Important ESP32/RMT safety note:
  - do **not** call `strip.Begin()` again after setup to “re-arm” NeoPixel output.
  - repeated `Begin()` can trigger NeoPixelBus RMT install abort (`ESP_ERR_INVALID_STATE`).
  - validated safe pattern is: initialize once in `setupNeopixel()`, then only update via frame writes.

### 14) Hidden Service Page + Navigation Cleanup
- Split maintenance/service controls from regular settings and moved them into a dedicated hidden `Service` page.
- Top menu no longer exposes `Service`; access is provided from the footer/version area link.
- Moved the live debug/log card into the Service page and aligned it with the page layout.

### 15) Mobile Layout Improvements
- Fixed responsive width alignment for cards/rows on smaller screens (notably System section).
- Applied the same full-width stacking behavior to multi-column MQTT rows for consistent mobile rendering.

### 16) Display Power Switch Correctness
- Reworked Display Power toggle to use the standard OFF/ON visual pattern.
- Fixed ON/OFF inversion caused by string truthiness (`"false"` evaluating truthy in JS).
- Added robust boolean parsing in frontend control sync paths.

### 17) Max LED (mA) UI + Runtime Update Path
- Removed badge/chip background styling from current-limit status line; kept text-based state colors only.
- Added dedicated debounced sender for `maxLedmA` updates in frontend.
- Prevented duplicate posts from generic control handlers for `maxLedmA`.

### 18) Onboard ESP32 LED Manual Control
- Added runtime `onboardLed` setting to manually toggle onboard LED from web UI.
- Wired setting through `/saveSetting`, `/getConfiguration`, and `/getCurrentInfos`.
- Added pin/polarity-aware apply helper and safe handling for board variance.
- Startup behavior now defaults onboard LED to OFF.

### 19) New Animation Modes + Random Mapping
- Added `animMode = 8` (**Slot Wave**): dramatic center-out progressive locking.
- Added `animMode = 9` (**Mid to Margin**): mode-3 style shift that propagates from middle toward both edges.
- Updated random mappings:
  - mode `6` random set: `1,2,3,4,5,8,9`
  - mode `7` random set: `1,2,3,4,8,9`
- Mode `6` is hidden in UI (firmware compatibility retained for existing saved configs).
- Tuned mode `8` for a slower, smoother cadence.

### 20) Touch Button on ESP32 GPIO33
- Added capacitive touch support on GPIO33 with runtime calibration and thresholding.
- Added press classification and action mapping for:
  - short press
  - double press
  - long press
- Added live touch debug logging (`raw`, threshold, down/up events, press type).
- Added touch actions:
  - `None`
  - `Alarm OFF`
  - `Color Change`
  - `Display OFF`
  - `Display Toggle`
- Wired touch actions through persistence (`/saveSetting`, `/getConfiguration`) and runtime execution.
- Touch action controls are available on the **Automation** page.
- Fixed UI init restore so touch action dropdowns keep selected value after page refresh.

### 21) AP/Phone Stability + Crash Diagnostics
- Improved AP-mode WiFi scan/connect behavior to keep AP reachable during STA operations (AP+STA preservation flow).
- Reduced risk of first-scan disconnect from mobile clients while in AP mode.
- Added ESP32 startup reset-reason logging (human-readable + numeric) to help diagnose watchdog/panic/brownout resets.

### 22) System/UI Runtime Improvements
- `Uptime` formatting updated to `h:mm` using `uptimeMinutes` (with compatibility fallback).
- Physical sensor summary formatting improved to compact single-line bullet-separated text.
- Network form layout adjusted so scan/connect buttons can be placed below related input fields.

### 23) Factory Reset / Default Baseline (Security + UX)
- AP default identity updated:
  - AP SSID: `NixieClock`
  - AP password: `18273645`
- WiFiManager captive portal now also uses AP password (not open by default).
- Factory-reset defaults updated to:
  - Day brightness: **30%** (instead of max)
  - Date format: **dd/mm/yyyy** (`dateMode = 0`)
  - Double blink: **enabled**
  - Auto dimmer: **enabled**
  - Auto DST: **enabled**
  - Touch short: **Alarm OFF**
  - Touch double: **Color Change**
  - Touch long: **Display Toggle**
  - UI Background RGB: **3, 2, 2**
  - UI Panel Card RGB: **15, 6, 6**

### 24) Factory Reset Reliability + AP Default Enforcement
- Fixed factory-reset persistence path to commit EEPROM immediately before reboot (no deferred save race).
- Ensured factory-reset AP defaults are enforced independent of `clocks.h` profile `AP_NAME` overrides.
- Added explicit factory fallback constants:
  - AP SSID: `NixieClock`
  - AP password: `18273645`
- WiFiManager captive portal now prefers persisted AP credentials (`prm.ApSsid`/`prm.ApPsw`) with fallback to factory defaults.

### 25) WiFi Scan/Connect AP Onboarding Flow (Desktop/Mobile)
- Fixed scan polling logic in frontend so status responses (`scanning`, `busy`, `blocked`) no longer end in false `Scan failed`.
- Added scan debug logs for start/completion/failure states.
- Kept AP onboarding flow available while connected to the clock AP:
  - scan nearby WiFi,
  - select SSID,
  - connect with password,
  - save credentials,
  - rollback path remains if connect fails.
- Standalone/AP mode now starts with `WIFI_AP_STA` to reduce scan/connect disruption while clients are attached.

### 26) RTC Enhancements (PCF8563 + DS3231)
- Added RTC autodetect support:
  - `0x68` -> DS3231
  - `0x51` -> PCF8563
- Implemented PCF8563 read/write/update flow in `rtc.ino` while preserving DS3231 behavior.
- Improved RTC startup diagnostics (including hints for `0x57` EEPROM-only detection scenarios).
- Updated I2C initialization safety for RTC/sensor bus (`INPUT_PULLUP` before `Wire.begin`).

### 27) System Sensors Visibility
- System page `Physical Sensors` summary now includes RTC type when detected:
  - `RTC DS3231` or
  - `RTC PCF8563`.

### 28) AutoDimmer Behavior Refinement
- AutoDimmer now treats manual settings as caps:
  - `Day Brightness` acts as the tube brightness max cap under lux scaling.
  - `RGB Brightness` acts as the RGB max cap under lux scaling.
- RGB auto-dimming now scales over full lux range (`0..MAXIMUM_LUX`) instead of reaching cap around half range.

### 29) Touch Color Cycle Expansion
- Expanded GPIO33 touch double-tap color-change palette from a small set to a wide full-spectrum sequence.
- Includes broader rainbow coverage plus white and warm accent tones.
- Keeps existing behavior: each double tap advances to next color and persists fixed RGB values immediately.

### 30) Optional Gesture Sensor (APDS-9960)
- Added optional APDS-9960 gesture support (`USE_APDS9960_GESTURE`) using I2C.
- New file: `gesture_apds9960.ino` with safe no-op stubs when feature is disabled.
- Gesture actions are mapped to color-palette cycling:
  - `UP` / `RIGHT` -> next color
  - `DOWN` / `LEFT` -> previous color
- Uses debounce to avoid repeated triggers from one hand movement.
- Integration points:
  - setup hook: `setupGestureSensor()`
  - loop hook: `processGestureSensor()`
  - shared color API: `cycleFixedColorPalette(int step)`

Enable checklist:
- Install Arduino library: **SparkFun APDS-9960**.
- In your clock profile (`clocks.h`), define `USE_APDS9960_GESTURE`.
- Wire APDS-9960 to existing I2C bus (`VCC`, `GND`, `SDA`, `SCL`), optional `INT` pin via `APDS9960_INT_PIN`.

## Settings/Schema Notes
- `Settings` schema was incremented for newly added persisted display fields:
  - display toggles
  - pressure start/end window
- After flashing a newer build, one-time settings reset/default reinit can happen.

## Dev Notes
- IntelliSense include path warnings (for example `Arduino.h`, `stddef.h`) are editor configuration warnings and not firmware runtime logic errors.
### Home Assistant Entity Support (2026-02)

The firmware now supports direct Home Assistant integration for the following entity types via the REST API:

**Switches (on/off):**
- `displayPower` — Display power ON/OFF
- `nightMode` — Night mode ON/OFF
- `alarmEnable` — Alarm enabled ON/OFF
- `showTimeDate` — Show time/date ON/OFF
- `showTemperature` — Show temperature ON/OFF
- `showHumidity` — Show humidity ON/OFF
- `showPressure` — Show pressure ON/OFF

**Numbers:**
- `rgbBrightness` — RGB LED brightness (0–255)
- `rgbAnimationSpeed` — RGB animation speed (1–255)

**Selects:**
- `rgbEffect` — RGB animation effect (integer, see UI for available modes)

All of these can be read from `/getConfiguration` and set via `/saveSetting` (POST with `key` and `value`).

This enables robust Home Assistant entity control for display, alarm, and RGB features.
