# UI Refactor Changelog (2026-02-12)

## Summary
- Refactored web UI for better maintainability and stability.
- Preserved existing functionality while cleaning structure and startup flow.

## Implemented
- Fixed RGB section HTML issues (duplicate IDs, malformed block).
- Consolidated frontend startup to a single `$(document).ready(...)` entry.
- Split frontend logic into modules:
  - `data/js/ui-main-extras.js`
  - `data/js/ui-live-log.js`
  - `data/js/ui-debug.js`
  - `data/js/ui-customization.js`
  - `data/js/ui-system.js`
  - `data/js/ui-wifi-mqtt.js`
- Updated `data/index.html` to load modular scripts.
- Replaced inline styles in static and dynamic UI with reusable CSS classes in `data/site.css`.
- Removed stale CSS/JS references (`selectedColor` legacy path, old unused selectors).
- Normalized dynamic menu attribute casing (`pagemenu`).

## Critical Runtime Fix
- Fixed broken page loading after modularization by updating firmware fallback route:
  - `handleNotFound(...)` in `ESP32_UniClock2.ino` now serves existing SPIFFS static files (including `/js/...`) with proper MIME types.

## Validation
- No UI syntax/lint errors reported in edited files.
- Duplicate ID check passed (ignoring commented HTML).
- UI confirmed working after firmware + filesystem upload.

## Deployment Reminder
After UI/route changes, always:
1. Upload firmware.
2. Upload filesystem (`data/` to SPIFFS/LittleFS).
3. Hard refresh browser (`Ctrl+F5`).
