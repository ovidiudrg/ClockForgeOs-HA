from pathlib import Path

import cairosvg
from PIL import Image

ROOT = Path(__file__).parent
DOMAIN_DIR = ROOT / "custom_integrations" / "clockforgeos"
SOURCE = DOMAIN_DIR / "SOURCE.svg"


def render_png(size: int, out_path: Path) -> None:
    png_bytes = cairosvg.svg2png(url=str(SOURCE), output_width=size, output_height=size)
    out_path.write_bytes(png_bytes)


def ensure_square(path: Path, size: int) -> None:
    with Image.open(path) as image:
        if image.size != (size, size):
            image = image.resize((size, size), Image.Resampling.LANCZOS)
            image.save(path, format="PNG", optimize=True)


def main() -> None:
    DOMAIN_DIR.mkdir(parents=True, exist_ok=True)

    render_png(256, DOMAIN_DIR / "icon.png")
    render_png(512, DOMAIN_DIR / "icon@2x.png")

    ensure_square(DOMAIN_DIR / "icon.png", 256)
    ensure_square(DOMAIN_DIR / "icon@2x.png", 512)

    # If you use same logo and icon, only icon files are required.
    # logo fallback is automatic in Home Assistant Brands.

    print("Generated:")
    print(DOMAIN_DIR / "icon.png")
    print(DOMAIN_DIR / "icon@2x.png")


if __name__ == "__main__":
    main()
