# Flutter Mobile Starter (HTTP)

This is a minimal starter structure for a ClockForgeOS mobile app using HTTP only.

## Suggested setup

1. Create app:
```bash
flutter create flutter_clockforge_http
```

2. Replace `lib/` with files from this folder.

3. Add dependency in `pubspec.yaml`:
```yaml
dependencies:
  flutter:
    sdk: flutter
  http: ^1.2.2
```

4. Run:
```bash
flutter run
```

## What this starter includes

- Base URL input (`http://<clock-ip>`)
- Login with `/auth/login`
- Live status polling from `/getCurrentInfos`
- Example setting update via `/saveSetting` (`displayPower`)

## Next screens to add

- RGB controls (`rgbEffect`, `rgbBrightness`, `rgbFixR/G/B`)
- Schedule controls (`dateStart`, `tempStart`, `pressureStart`, ...)
- WiFi management (`/scanWifi`, `/connectWifi`, `/wifiStatus`)
- System diagnostics (`/getSystemInfo`)
