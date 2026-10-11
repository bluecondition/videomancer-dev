#!/usr/bin/env python3
"""Contact sheet: one frame from the middle of each (start,end) shot. Usage: contact.py VIDEO OUT.png s0-e0 s1-e1 ..."""
import sys
import numpy as np
import imageio_ffmpeg
from PIL import Image, ImageDraw

path, out, shots = sys.argv[1], sys.argv[2], [tuple(map(float, a.split("-"))) for a in sys.argv[3:]]
tiles = []
for a, b in shots:
    t = (a + b) / 2
    gen = imageio_ffmpeg.read_frames(path, input_params=["-ss", f"{t:.2f}"], output_params=["-frames:v", "1", "-vf", "scale=320:-2"])
    meta = next(gen)
    w, h = meta["size"] if meta["size"][0] == 320 else (320, int(320 * meta["size"][1] / meta["size"][0]) // 2 * 2)
    buf = next(gen)
    img = Image.fromarray(np.frombuffer(buf, np.uint8).reshape(-1, 320, 3))
    ImageDraw.Draw(img).text((4, 4), f"{a:.0f}-{b:.0f}s", fill=(255, 255, 0))
    tiles.append(img)
cols = 4
th = max(t.height for t in tiles)
sheet = Image.new("RGB", (cols * 320, ((len(tiles) + cols - 1) // cols) * th))
for i, t in enumerate(tiles):
    sheet.paste(t, ((i % cols) * 320, (i // cols) * th))
sheet.save(out)
