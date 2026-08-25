"""Generate clearly labelled QR cards for the warehouse-delivery simulator."""

import json
from pathlib import Path

import qrcode
from PIL import Image, ImageDraw, ImageFont


CARDS = (
    ("01_SMALL_DELIVERY", "소형 화물 | 공통 목적지", "SMALL / 로봇 1대", 3.20, 2.40, "SMALL_BOX", 2.0),
    ("02_LONG_DELIVERY", "장형 화물 | 공통 목적지", "LONG / 로봇 2대", 3.20, 2.40, "LONG_BOX", 8.0),
)


def font(size):
    for candidate in (
        "C:/Windows/Fonts/malgun.ttf",
        "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
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
    qr = qrcode.make(encoded).convert("RGB").resize((720, 720))
    image = Image.new("RGB", (800, 1020), "white")
    image.paste(qr, (40, 220))
    draw = ImageDraw.Draw(image)
    title_font, text_font = font(38), font(26)
    draw.text((40, 35), identifier, fill="#174ea6", font=text_font)
    draw.text((40, 80), title, fill="black", font=title_font)
    draw.text((40, 140), subtitle, fill="#555555", font=text_font)
    draw.text((40, 960), f"목적지 좌표: ({x:.1f}, {y:.1f}) m", fill="black", font=text_font)
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
