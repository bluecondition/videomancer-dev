#!/usr/bin/env python3
"""
RIBBON table generator (v0.3).

Emits the VHDL constants pasted into ribbon.vhd:
  - C_PAL_Y/U/V : 7 candy palettes x 8 stops (dark -> light), SUGARCOAT's
    own encoding (Y = y*1023, chroma deviation x4 -- ~1.15x overdriven, which
    is the vivid candy look on hardware) with ONLY the extremes made legal:
    Y clamped to 64..940 (darks were 53, white stops 1023).
    v0.3.0 tried a strictly legal re-encode (RGB x0.817 into 64..940 / 896):
    hue-exact, but 28% less chroma and 18% less luma -- Ron: "my true rainbow
    palette seems to be gone".  Don't redo it.
  - C_WAVE  : the ripple quarter-wave, 512 x 8 bit, index = zig & position:
    0..255 Smooth (round(255*sin(pi/2*k/255))), 256..511 Zigzag (k-256).
    One 512x8 table = ONE EBR per axis.  (v0.3.0 had 64 levels per quarter:
    at the deep squared Warp each level was a 3-6 px jump -> stair-stepped
    contours that tore and flickered as thin lines while the slider moved.)
  - C_ZOOM  : 512 * 2^(-i/256), i = 0..255 (one octave of the exponential
    Scale mantissa; the octave itself is the barrel-shift pitch).

Standard BT.601 (U = Cb, V = Cr); ribbon.vhd swaps U/V on the way out.
"""
import math


PALETTES = {
    "Penny Candy": [
        (40, 0, 10), (140, 10, 20), (220, 20, 40), (255, 120, 0),
        (255, 210, 0), (120, 210, 30), (90, 40, 160), (255, 255, 255),
    ],
    "Bubblegum": [
        (60, 20, 55), (150, 40, 110), (255, 90, 175), (255, 150, 200),
        (130, 225, 210), (120, 200, 255), (210, 180, 255), (255, 250, 250),
    ],
    "Peppermint": [
        (20, 30, 20), (10, 90, 40), (20, 140, 60), (200, 20, 40),
        (240, 70, 90), (230, 200, 60), (255, 240, 240), (255, 255, 255),
    ],
    "Bakery": [
        (35, 18, 8), (70, 38, 18), (120, 72, 34), (170, 110, 55),
        (205, 150, 85), (230, 190, 120), (245, 225, 170), (255, 248, 225),
    ],
    "Sour": [
        (20, 40, 10), (120, 200, 20), (200, 255, 0), (255, 240, 20),
        (255, 20, 200), (20, 220, 220), (180, 255, 120), (255, 255, 255),
    ],
    "Tropical Taffy": [
        (40, 20, 30), (255, 110, 90), (255, 170, 60), (255, 225, 70),
        (40, 200, 190), (60, 225, 215), (255, 130, 175), (255, 250, 245),
    ],
    "Holo Wrapper": [
        (60, 60, 70), (150, 120, 200), (120, 180, 240), (140, 240, 210),
        (240, 220, 150), (245, 170, 200), (210, 210, 235), (255, 255, 255),
    ],
}


def yuv10(r, g, b):
    # SUGARCOAT's encoding, verbatim: 8-bit BT.601, Y scaled to 10 bits,
    # chroma deviation *4 centred 512 ...
    y8 = 0.299 * r + 0.587 * g + 0.114 * b
    u8 = -0.168736 * r - 0.331264 * g + 0.5 * b        # Cb deviation
    v8 = 0.5 * r - 0.418688 * g - 0.081312 * b         # Cr deviation
    clip = lambda x: max(0, min(1023, x))
    # ... then only the luma extremes made legal
    Y = max(64, min(940, round(y8 * 1023 / 255)))
    return Y, clip(round(512 + u8 * 4)), clip(round(512 + v8 * 4))


def emit():
    out = []
    names = list(PALETTES)
    out.append("    type t_pal56 is array (0 to 55) of unsigned(9 downto 0);")
    for chan, idx in (("Y", 0), ("U", 1), ("V", 2)):
        out.append("    constant C_PAL_%s : t_pal56 := (" % chan)
        for pi, name in enumerate(names):
            vals = [yuv10(*rgb)[idx] for rgb in PALETTES[name]]
            body = ", ".join("to_unsigned(%d, 10)" % v for v in vals)
            comma = "," if pi < len(names) - 1 else ""
            out.append("        %s%s  -- %d: %s" % (body, comma, pi * 8, name))
        out.append("    );")

    wave = [round(255 * math.sin(math.pi / 2 * k / 255)) for k in range(256)]
    wave += list(range(256))
    out.append("    type t_wave512 is array (0 to 511) of unsigned(7 downto 0);")
    out.append("    constant C_WAVE : t_wave512 := (")
    for r in range(0, 512, 16):
        body = ", ".join("to_unsigned(%d, 8)" % v for v in wave[r:r + 16])
        out.append("        %s%s" % (body, "," if r < 496 else ""))
    out.append("    );")

    zoom = [round(512 * 2 ** (-i / 256)) for i in range(256)]
    out.append("    type t_zoom256 is array (0 to 255) of unsigned(9 downto 0);")
    out.append("    constant C_ZOOM : t_zoom256 := (")
    for r in range(0, 256, 16):
        body = ", ".join("to_unsigned(%d, 10)" % v for v in zoom[r:r + 16])
        out.append("        %s%s" % (body, "," if r < 240 else ""))
    out.append("    );")
    return "\n".join(out)


if __name__ == "__main__":
    print(emit())
