# Bug Report: NeoPixel Freeze on CLOCK_42a (ESP32, IV-11 VFD)

## Summary
On `CLOCK_42a` builds, the NeoPixel strip on `GPIO25` appears frozen after boot/tube-test. Firmware logs continue to show active NeoPixel runtime updates, but physical LEDs do not animate as expected.

## Reported Symptoms
- LEDs stay stuck on last shown colors.
- Sometimes only the last LED changes (including turning white after ~40 seconds).
- Logs show periodic `[NEO] run ...` updates with changing state.
- Issue persists after recent NeoPixel/runtime patches.

## Hardware/Profile Context
- Board/profile: `CLOCK_42a`
- MCU: `ESP32`
- Tube driver: `MAX6921_ESP32`
- NeoPixel pin: `GPIO25`
- Tube power pin: `GPIO27`
- Radar pin: `GPIO34`
- Alarm speaker pin: `GPIO2`

## Representative Boot/Runtime Evidence
- Setup confirms NeoPixel is initialized:
  - `Setup NeoPixel LEDS`
  - `NEOPIXEL_PIN: GPIO25`
  - `Pixel count: 6`
  - `Tube map count/maxDigits: 6/6`
- Runtime logs continue:
  - `[NEO] run e=10 bReq=115 bAp=115 mA=8 maxDigits=6 px=6/6 p0=...`
- Physical behavior still appears blocked/frozen.

## Reproduction Steps
1. Build and flash `CLOCK_42a` firmware with `USE_NEOPIXEL`.
2. Boot device and observe tube self-test and WiFi startup phase.
3. Let clock run with RGB effect enabled.
4. Compare serial `[NEO] run ...` lines vs physical LED motion.

## Expected Behavior
- NeoPixel effects update continuously and visibly across all mapped pixels.

## Actual Behavior
- LED output often remains static while logs imply active animation updates.

## Scope/Impact
- Affects visual ambient RGB behavior on `CLOCK_42a`.
- Confusing mismatch between diagnostics and hardware state.

## Changes Already Attempted
- Switched CLOCK_42/42a NeoPixelBus method to `NeoEsp32Rmt1Ws2812xMethod`.
- Disabled double-show for CLOCK_42/42a (`NEO_DOUBLE_SHOW = false`).
- Kept NeoPixel updates active in `Fdelay()` during startup on CLOCK_42/42a.
- Hardened active-pixel check for CLOCK_42/42a to avoid runtime `maxDigits` dependency.
- Added richer runtime diagnostics (`maxDigits`, active pixel count, pixel0 color).
- Remapped effect `6` and `10` to smooth `effect2()` on CLOCK_42/42a.
- Boot-time EEPROM normalization: remap persisted effect `6/10 -> 5` for CLOCK_42/42a.

## Notes
- Earlier PWM driver changes are likely unrelated to this specific issue path:
  - This profile uses `USE_NEOPIXEL`.
  - `pwmleds.ino` is gated by `#ifdef USE_PWMLEDS`.

## Open Investigation Areas
- Potential pin-level contention on `GPIO25` outside NeoPixel path.
- Timing/contention between NeoPixel RMT output and high-frequency display activity.
- Any board-specific electrical issue causing partial chain latch/stall behavior.

