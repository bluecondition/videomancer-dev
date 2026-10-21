#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
COCOON - build hook.  Generates cocoon_pkg.vhd (pure Python, no numpy).

  C_TY / C_TU / C_TV  1024 x 8: the cocoon colourizer.  Address =
        bank(1) & F(9:4)(6) & level(3).  bank 1 = the right / wing palette
        (crimson ring, green arc, gray-green corner), bank 0 = the band's left
        palette (yellow -> yellow-green).  level 0 dash, 1 arc+, 2 mid,
        3 arc-, 4 gap (5..7 repeat 2).  Entry = the colour at F = 16 i + 8.
  C_GY / C_GU / C_GV   256 x 8: the diamond glow, entry i = colour at d = 8 i + 4.
  C_G   1024 x 10: the field's x term G(x) = 0.6 soft(1050 - x) + 0.8 soft(x - 1200)
        (soft rounds the corner over 64 px), entry i = G at x = 2 i + 1 (scaled photo x).
  C_XP   256 x 16: the x profile at width-normalised x = 8 i + 4: [15] flank,
        [14:10] window gain 0..16, [9:5] band texture ramp (mirrored), [4:0] the ramp.
  C_VY / C_VU / C_VV    64 x 8: the video colourizer, address = band(1) & luma(7:3):
        bank 0 = the flanks (gray / olive / navy / green), bank 1 = the band
        (green / yellow-green / olive / orange / pink / gray).

Y and chroma are the 10-bit values / 4, in the HARDWARE convention: the U
register carries Cr, the V register Cb.  The knots are fitted to the photo
(WIN_20220829_23_30_37) -- see SUMMARY.md.
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ knots ---
GLOW = [(0, (218, 0, 0)), (560, (216, 0, 0)), (640, (202, 0, 3)), (760, (162, 0, 16)),
        (880, (95, 0, 37)), (1000, (30, 0, 70)), (1100, (2, 0, 98)), (1300, (0, 0, 92))]
FL = [(0.00, (4, 1, 121)), (0.18, (4, 1, 121)), (0.20, (63, 30, 99)), (0.23, (25, 63, 34)),
      (0.27, (25, 63, 34)), (0.29, (100, 72, 9)), (0.38, (100, 72, 9)), (0.40, (117, 113, 104)),
      (0.42, (190, 190, 189)), (1.0, (205, 206, 204))]
BL = [(0.00, (38, 111, 46)), (0.20, (38, 111, 46)), (0.23, (52, 177, 4)), (0.36, (52, 177, 4)),
      (0.38, (128, 188, 7)), (0.50, (128, 188, 7)), (0.52, (130, 125, 10)), (0.58, (130, 125, 10)),
      (0.60, (221, 97, 43)), (0.66, (221, 97, 43)), (0.68, (237, 102, 118)), (0.74, (237, 102, 118)),
      (0.76, (151, 137, 106)), (0.82, (175, 175, 172)), (1.0, (185, 185, 182))]
# Five-level colourizer, fitted per F bin to the photo: dark 30 % / mid / bright 40 % of the luma in
# the band (right of x 1050), the band left of 1050 (left bank) and the wings (m15.py).  Each knot:
# F -> (dash, arc+, mid, arc-, gap).  F < 225 is band-only (crimson dashes on a pink / salmon /
# yellow-orange ground); from 225 the arc levels carry the wing stripes (red / orange / yellow,
# the crimson ring, the green arc, gray-green).
R_KNOTS = [
    (0,    (220, 2, 40),  (241, 32, 55), (244, 55, 58), (247, 80, 62), (246, 96, 58)),
    (100,  (220, 2, 31),  (241, 35, 51), (244, 60, 52), (245, 86, 51), (244, 104, 48)),
    (175,  (224, 15, 27), (243, 62, 40), (244, 80, 40), (244, 95, 42), (242, 112, 40)),
    (225,  (227, 43, 22), (227, 64, 2),  (232, 99, 20), (228, 135, 20), (230, 140, 22)),
    (275,  (220, 68, 26), (219, 73, 1),  (224, 108, 10), (223, 146, 4), (228, 146, 26)),
    (325,  (196, 74, 26), (212, 45, 5),  (224, 100, 5), (226, 150, 10), (215, 140, 40)),
    (362,  (194, 33, 39), (205, 20, 8),  (226, 80, 5),  (215, 170, 20), (210, 96, 50)),
    (387,  (190, 20, 48), (198, 12, 10), (212, 60, 8),  (190, 180, 30), (200, 70, 55)),
    (412,  (178, 11, 51), (185, 8, 12),  (190, 40, 15), (165, 175, 35), (181, 60, 58)),
    (437,  (160, 30, 45), (160, 25, 20), (150, 70, 30), (130, 170, 40), (150, 90, 55)),
    (462,  (122, 78, 29), (122, 78, 29), (104, 123, 37), (106, 157, 39), (106, 157, 39)),
    (487,  (117, 63, 35), (117, 63, 35), (123, 127, 44), (112, 168, 52), (112, 168, 52)),
    (512,  (84, 39, 33),  (84, 39, 33),  (92, 69, 51),  (96, 124, 68), (96, 124, 68)),
    (537,  (71, 98, 42),  (71, 98, 42),  (86, 119, 59), (104, 132, 83), (104, 132, 83)),
    (562,  (61, 80, 44),  (61, 80, 44),  (81, 119, 66), (98, 139, 87), (98, 139, 87)),
    (587,  (53, 54, 52),  (53, 54, 52),  (66, 73, 65),  (88, 96, 85),  (88, 96, 85)),
    (700,  (60, 66, 60),  (60, 66, 60),  (75, 84, 74),  (92, 100, 90), (92, 100, 90)),
    (1023, (60, 66, 60),  (60, 66, 60),  (75, 84, 74),  (92, 100, 90), (92, 100, 90)),
]
L_KNOTS = [
    (0,    (220, 2, 40),  (241, 32, 55), (244, 55, 58), (247, 80, 62), (246, 96, 58)),
    (62,   (219, 2, 16),  (238, 37, 34), (243, 55, 40), (247, 80, 47), (248, 95, 50)),
    (137,  (218, 2, 1),   (237, 37, 15), (240, 60, 22), (246, 84, 31), (245, 100, 30)),
    (187,  (220, 10, 1),  (239, 61, 12), (240, 82, 17), (241, 104, 22), (240, 115, 22)),
    (237,  (224, 29, 1),  (238, 85, 9),  (235, 103, 12), (231, 121, 16), (228, 135, 16)),
    (287,  (220, 67, 2),  (226, 114, 6), (222, 132, 10), (217, 151, 14), (216, 158, 15)),
    (337,  (201, 115, 0), (212, 136, 3), (215, 148, 9), (218, 160, 15), (218, 164, 15)),
    (387,  (201, 103, 1), (207, 142, 3), (212, 153, 8), (217, 165, 12), (217, 168, 14)),
    (412,  (218, 58, 0),  (215, 96, 1),  (218, 122, 5), (220, 149, 10), (222, 150, 10)),
    (462,  (199, 50, 0),  (205, 84, 2),  (207, 108, 7), (209, 132, 12), (209, 132, 12)),
    (487,  (191, 56, 2),  (192, 115, 13), (187, 140, 20), (182, 165, 28), (182, 165, 28)),
    (512,  (163, 25, 5),  (163, 99, 17), (163, 130, 27), (163, 161, 37), (163, 161, 37)),
    (600,  (150, 120, 20), (140, 150, 20), (135, 170, 15), (128, 188, 7), (128, 188, 7)),
    (1023, (150, 120, 20), (140, 150, 20), (135, 170, 15), (128, 188, 7), (128, 188, 7)),
]
LEV_R = [[(k, c[i]) for k, *c in R_KNOTS] for i in range(5)]
LEV_L = [[(k, c[i]) for k, *c in L_KNOTS] for i in range(5)]


def interp(knots, x):
    if x <= knots[0][0]:
        return knots[0][1]
    for (k0, c0), (k1, c1) in zip(knots, knots[1:]):
        if x <= k1:
            t = (x - k0) / float(k1 - k0)
            return tuple(a + (b - a) * t for a, b in zip(c0, c1))
    return knots[-1][1]


def hw8(rgb):
    """rgb 0..255 -> (Y, U = Cr, V = Cb) 10-bit / 4, hardware convention.
    Y is pre-scaled by 16/13: the output conditioning (the HD HDMI bar fix) scales luma by 13/16,
    so the screen shows the intended colour; capped at 940 (-> ~776 on screen)."""
    r, g, b = (c / 255.0 for c in rgb)
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    q = lambda v, lo, hi: max(lo, min(hi, int(round(v / 4.0))))
    return q(64 + 876 * y * 16.0 / 13.0, 16, 235), q(512 + 896 * cr, 16, 240), q(512 + 896 * cb, 16, 240)


def tex_table():
    out = []
    for bank in (0, 1):
        for fi in range(64):
            F = 16 * fi + 8
            for lev in range(8):
                L = lev if lev < 5 else 2
                knots = (LEV_R if bank else LEV_L)[L]
                out.append(hw8(interp(knots, F)))
    return out


def glow_table():
    return [hw8(interp(GLOW, 8 * i + 4)) for i in range(256)]


def vid_table():
    out = []
    for bank in (0, 1):
        for i in range(32):
            out.append(hw8(interp(BL if bank else FL, (i + 0.5) / 32.0)))
    return out


def soft(u):
    return 0.0 if u < -64 else (u if u > 64 else (u + 64) ** 2 / 256.0)


G_Q, G_X0 = 0.0217, 1462         # the rounded right end (ring at x 1490 dy 162, green arc vertical by x ~1560)


def g_table():
    out = []
    for i in range(1024):
        x = 2 * i + 1
        g = 0.6 * soft(1050 - x) + 0.8 * soft(x - 1200) + G_Q * max(0, x - G_X0) ** 2
        out.append(min(1023, int(round(g))))
    return out


def xp_table():
    cl = lambda v, lo, hi: max(lo, min(hi, v))
    out = []
    for i in range(256):
        x = 8 * i + 4

        def tw(xx):
            r1 = cl(((xx - 330) * 218) >> 8, 0, 255)
            r2 = cl(((1580 - xx) * 410) >> 8, 0, 255)
            return min(r1, r2) >> 3
        if x < 358:
            k64, fl = 64 - cl(((x - 306) * 58) >> 6, 0, 64), 1
        elif x > 1567:
            k64, fl = 64 - cl(((1619 - x) * 58) >> 6, 0, 64), 1
        else:
            k64, fl = min(cl(((x - 340) * 37) >> 6, 0, 64), cl(((1585 - x) * 37) >> 6, 0, 64)), 0
        k16 = (k64 + 2) >> 2
        out.append((fl << 15) | (k16 << 10) | (tw(1920 - x) << 5) | tw(x))
    return out


def emit():
    def arr(name, typ, vals):
        body = []
        for i in range(0, len(vals), 16):
            body.append("        " + ", ".join(f'x"{v & 0xFF:02X}"' for v in vals[i:i + 16]))
        return f"    constant {name} : {typ} := (\n" + ",\n".join(body) + "\n    );"

    t, g, v = tex_table(), glow_table(), vid_table()
    def arrb(name, typ, vals, bits):
        body = []
        for i in range(0, len(vals), 16):
            body.append("        " + ", ".join(f'"{v:0{bits}b}"' for v in vals[i:i + 16]))
        return f"    constant {name} : {typ} := (\n" + ",\n".join(body) + "\n    );"

    consts = [arrb("C_G", "t_rom_g", g_table(), 10), arrb("C_XP", "t_rom_xp", xp_table(), 16),
              arr("C_TY", "t_rom1k", [w[0] for w in t]), arr("C_TU", "t_rom1k", [w[1] for w in t]),
              arr("C_TV", "t_rom1k", [w[2] for w in t]),
              arr("C_GY", "t_rom256", [w[0] for w in g]), arr("C_GU", "t_rom256", [w[1] for w in g]),
              arr("C_GV", "t_rom256", [w[2] for w in g]),
              arr("C_VY", "t_rom64", [w[0] for w in v]), arr("C_VU", "t_rom64", [w[1] for w in v]),
              arr("C_VV", "t_rom64", [w[2] for w in v])]
    txt = f"""-- cocoon_pkg.vhd -- GENERATED by cocoon.py; edit the generator, not this file.
--
-- C_TY/TU/TV: cocoon colourizer, address = bank & F(9:4) & level.
-- C_GY/GU/GV: diamond glow, entry i = colour at d = 8 i + 4.
-- C_VY/VU/VV: video colourizer, address = band & luma(7:3).
-- C_G: the field's x term at scaled x = 2 i + 1.  C_XP: the x profile
-- (flank, window gain 0..16, band ramp mirrored / not) at x = 8 i + 4.
-- 10-bit values / 4, hardware convention (U carries Cr).

library ieee;
use ieee.std_logic_1164.all;

package cocoon_pkg is
    type t_rom1k is array(0 to 1023) of std_logic_vector(7 downto 0);
    type t_rom256 is array(0 to 255) of std_logic_vector(7 downto 0);
    type t_rom64 is array(0 to 63) of std_logic_vector(7 downto 0);
    type t_rom_g is array(0 to 1023) of std_logic_vector(9 downto 0);
    type t_rom_xp is array(0 to 255) of std_logic_vector(15 downto 0);
{chr(10).join(consts)}
end package cocoon_pkg;
"""
    with open(os.path.join(HERE, "cocoon_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
