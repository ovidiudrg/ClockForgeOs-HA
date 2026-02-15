# Home Assistant Brands submission package

This folder prepares assets for submitting `clockforgeos` to:
- https://github.com/home-assistant/brands
- target path: `custom_integrations/clockforgeos/`

## Files expected by Brands
- Required: `icon.png` (256x256)
- Recommended: `icon@2x.png` (512x512)
- Optional: `logo.png`, `logo@2x.png`, and `dark_*` variants

Because ClockForgeOS uses the same image for icon/logo, icon files are enough.

## Generate PNG files
From this folder:

```bash
pip install cairosvg pillow
python generate_brands_assets.py
```

Output files:
- `custom_integrations/clockforgeos/icon.png`
- `custom_integrations/clockforgeos/icon@2x.png`

## Open a PR in home-assistant/brands
1. Fork `home-assistant/brands`
2. Create folder: `custom_integrations/clockforgeos/`
3. Add generated icon files
4. Open PR title example:
   - `Add ClockForgeOS custom integration brand assets`
5. In PR description include:
   - Domain: `clockforgeos`
   - Custom integration repository: `https://github.com/ovidiudrg/ClockForgeOs-HA`

## Cache note
Brand assets are cached and can take up to 24h+ to appear everywhere.
