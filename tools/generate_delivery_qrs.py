"""Generate clearly labelled QR cards for the warehouse-delivery simulator."""

import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

try:
    import qrcode
except ImportError:
    qrcode = None
    import cv2


CARDS = (
    ("01_SMALL_DELIVERY", "SMALL CARGO | TOP-RIGHT DROP", "1 ROBOT", 3.55, 2.55, "SMALL_BOX", 2.0),
    ("02_LONG_DELIVERY", "LONG CARGO | TOP-RIGHT DROP", "2 ROBOTS", 3.55, 2.55, "LONG_BOX", 8.0),
)


def font(size):
    for candidate in (
        "C:/Windows/Fonts/malgun.ttf",
        "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size)
    return ImageFont.load_default()


def create_card(card, output_dir):
    identifier, title, subtitle, x, y, cargo_type, weight = card
    payload = {
        "command": "DELIVERY",
        "cargo_type": cargo_type,
        "weight_kg": weight,
        "dest_x": x,
        "dest_y": y,
    }
    encoded = json.dumps(payload, separators=(",", ":"))
    if qrcode is not None:
        qr = qrcode.make(encoded).convert("RGB")
    else:
        # ROS images already depend on OpenCV.  Use it when the optional
        # qrcode package is unavailable on a Raspberry Pi or Ubuntu host.
        matrix = cv2.QRCodeEncoder_create().encode(encoded)
        qr = Image.fromarray(matrix).convert("RGB")
    qr = qr.resize((720, 720), Image.Resampling.NEAREST)
    image = Image.new("RGB", (800, 1020), "white")
    image.paste(qr, (40, 220))
    draw = ImageDraw.Draw(image)
    title_font, text_font = font(38), font(26)
    draw.text((40, 35), identifier, fill="#174ea6", font=text_font)
    draw.text((40, 80), title, fill="black", font=title_font)
    draw.text((40, 140), subtitle, fill="#555555", font=text_font)
    draw.text((40, 960), f"DESTINATION: ({x:.2f}, {y:.2f}) m", fill="black", font=text_font)
    image.save(output_dir / f"{identifier}.png")
    return encoded


def main():
    output_dir = Path(__file__).resolve().parents[1] / "qr_codes"
    output_dir.mkdir(exist_ok=True)
    payloads = [create_card(card, output_dir) for card in CARDS]
    (output_dir / "payloads.json").write_text(
        json.dumps(payloads, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
