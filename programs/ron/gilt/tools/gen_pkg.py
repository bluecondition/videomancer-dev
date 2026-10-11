"""Generate gilt_pkg.vhd (palette ROM + layout LUTs) from style_constants.json.

style_constants.json is the single source of truth (prompt section 4). Colours are
specified in CIELAB / sRGB there; this script turns them into the 10-bit BT.601 words the
program uses INTERNALLY (u = Cb, v = Cr -- the program swaps chroma at its input latch and
output compose, CLAUDE.md "hardware swaps U/V"). Y = 64 + 876*Y', C = 512 + 896*(...).

Palette ROM: 16 palettes x 64 entries (address = bank(1) & leaf(3) & index(6)).
  bank 0 = KISS, 1 = ADELE/STOCLET.  leaf 0..7 = LEAF knob position (silver .. orange).
  0-31  gold-paint ramp (dark -> light): GILT v0.10's 32-step gold/brown ramp, which Ron
        picked as the colour he liked (2026-10-10), tinted along the LEAF axis
  32 ink  33 umber  34 silver cool  35 silver warm  36 white  37 aura (pale metal)
  38 engraved line (dark metal)  39 highlight (pale metal)  40 shadow (dark metal)
  41 red-orange enamel  42 red-orange dark  43 vermilion  44 flush  45 cream
  46 meadow violet  47 meadow yellow  48-55 jewels  56-59 greens (dark -> light)
  60 cobalt  61 meadow blue  62 meadow white  63 coral

usage: python tools/gen_pkg.py   (giltlab venv: needs scikit-image)
"""
import json
import os

import numpy as np
from skimage import color

HERE = os.path.dirname(os.path.abspath(__file__))
GILT = os.path.dirname(HERE)
SC = json.load(open(os.path.join(GILT, "style_constants.json")))
P = SC["palette"]

Y_CAP = 768          # CLAUDE.md: cap drawn luma ~768 (full-scale Y clips RGB, kills chroma)


def lab_of(e):
    return np.array(e["lab"], float)


def ycc601(lab):
    rgb = np.clip(color.lab2rgb(np.array([[lab]], float))[0, 0], 0, 1)
    r, g, b = rgb
    y = 0.299 * r + 0.587 * g + 0.114 * b
    Y = min(Y_CAP, round(64 + 876 * y))
    Cb = round(512 + 896 * (b - y) / 1.772)
    Cr = round(512 + 896 * (r - y) / 1.402)
    return Y, max(0, min(1023, Cb)), max(0, min(1023, Cr))


def lch_to_lab(L, C, h):
    return np.array([L, C * np.cos(np.radians(h)), C * np.sin(np.radians(h))])


def metal(step):
    """LEAF axis: silver/platinum -> olive brass -> warm gold -> orange gold, 8 steps.
    Returns (chroma C*, hue h) of the metal; lightness comes from the ramps."""
    anchors = [lab_of(e) for e in P["leaf_axis"]]
    t = step / 7 * 3
    i = min(2, int(t))
    f = t - i
    lab = anchors[i] * (1 - f) + anchors[i + 1] * f
    C = float(np.hypot(lab[1], lab[2]))
    h = float(np.degrees(np.arctan2(lab[2], lab[1])) % 360)
    return C, h


# per bank: (L* lo, L* hi, chroma gain, hue offset) for field and robe ramps.
# KISS: dull dark brass field, bright saturated figure gold (STYLE_SPEC 4).
# ADELE: saturated orange field, paler higher-contrast ornament gold.
RAMPS = {
    0: dict(field=(26, 68, 0.70, 0.0), robe=(40, 82, 1.05, 2.0)),
    1: dict(field=(34, 70, 1.05, -6.0), robe=(30, 86, 0.85, -1.0)),
}


def prof(level):
    """Chroma profile along a 16-level ramp: umbers less saturated, highlights paler."""
    p = 0.55 + 0.45 * min(1.0, level / 7)
    if level > 12:
        p *= 1 - 0.15 * (level - 12)
    return p


V010_Y = [40, 94, 135, 184, 228, 273, 318, 361, 402, 440, 476, 511, 546, 579, 612, 644,
          676, 703, 729, 755, 780, 804, 825, 845, 865, 885, 903, 913, 924, 934, 945, 955]
V010_U = [505, 471, 448, 426, 405, 385, 366, 347, 329, 317, 306, 294, 284, 281, 277, 274,
          270, 276, 282, 289, 295, 304, 320, 336, 352, 368, 383, 396, 408, 421, 433, 445]
V010_V = [520, 550, 570, 587, 602, 615, 626, 636, 645, 649, 653, 657, 660, 659, 657, 656,
          655, 650, 644, 638, 632, 626, 617, 609, 600, 591, 583, 577, 571, 564, 558, 552]
LEAF_DEFAULT = 4      # TOML default LEAF 55 % -> step 4 reproduces v0.10 exactly


def ramp_v010(bank, step):
    """v0.10's ramp, chroma rotated/scaled by the LEAF metal relative to the default
    step (so the default IS v0.10), Adele a little more orange and saturated."""
    C0, h0 = metal(LEAF_DEFAULT)
    C, h = metal(step)
    k = C / C0
    dh = np.radians(h - h0 + (-6.0 if bank == 1 else 0.0))
    if bank == 1:
        k *= 1.08
    out = []
    for y, u, v in zip(V010_Y, V010_U, V010_V):
        cb, cr = u - 512, v - 512
        # rotate in the (Cr, Cb) plane: hue grows from +Cr toward -Cb for golds
        cr2 = k * (cr * np.cos(dh) + cb * np.sin(dh))
        cb2 = k * (cb * np.cos(dh) - cr * np.sin(dh))
        out.append((max(64, min(940, y)), int(round(max(0, min(1023, 512 + cb2)))),
                    int(round(max(0, min(1023, 512 + cr2))))))
    return out


def palette(bank, step):
    C, h = metal(step)
    ent = [None] * 64
    fixed = {32: P["ink"], 33: P["umber"], 34: P["silver_cool"], 35: P["silver_warm"],
             36: P["white"], 45: P["cream_stoclet"], 44: P["flush"]}
    for i, e in fixed.items():
        ent[i] = lab_of(e)
    ent[37] = lch_to_lab(86, C * 0.55, h)        # aura
    ent[38] = lch_to_lab(48, C * 0.90, h - 4)    # engraved line
    ent[39] = lch_to_lab(90, C * 0.35, h)        # highlight
    ent[40] = lch_to_lab(22, C * 0.40, h - 8)    # shadow
    oe = P["orange_enamel"]
    ent[41] = lab_of(oe[0]); ent[42] = lab_of(oe[1])
    jw = P["jewels"]
    ent[43] = lab_of(jw[1])                      # vermilion
    mg = P["meadow_greens"]                      # pooled order: sage, olive, leaf, malachite
    ent[46] = np.array([51.1, 17.9, -25.6])      # Kiss meadow violet (measured meadow cluster)
    ent[47] = np.array([68.6, -5.8, 51.1])       # Kiss meadow yellow (measured meadow cluster)
    for k in range(8):
        ent[48 + k] = lab_of(jw[k])
    greens = sorted(mg, key=lambda e: e["lab"][0])
    for k in range(4):
        ent[56 + k] = lab_of(greens[k])
    ent[60] = np.array([30.0, 18.0, -48.0])      # cobalt (Adele bars; from knowledge, darker than lapis)
    ent[61] = lab_of(jw[0])
    ent[62] = lab_of(P["white"])
    ent[63] = lab_of(jw[2])
    out = [None if e is None else ycc601(e) for e in ent]
    out[0:32] = ramp_v010(bank, step)
    return out


def word(y, cb, cr):
    return (y << 20) | (cb << 10) | cr


def main():
    rows = []
    for bank in (0, 1):
        for step in range(8):
            for (y, cb, cr) in palette(bank, step):
                rows.append(word(y, cb, cr))
    assert len(rows) == 1024
    flesh = ycc601(lab_of(P["flesh_kiss"]))
    # tile width / band height multipliers in S/8 units (16-entry, sorted ~log-normal).
    # KISS: narrow tiles, tall bands -> tall cells (bars).  ADELE/STOCLET: wide spread.
    wl = {0: [3, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 11, 12],
          1: [4, 4, 5, 5, 6, 7, 8, 8, 9, 10, 11, 12, 14, 16, 18, 21]}
    hl = {0: [7, 8, 8, 9, 10, 10, 11, 12, 12, 13, 14, 14, 15, 16, 17, 19],
          1: [5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 12, 13, 14, 15, 16, 18]}
    L = []
    L.append("-- gilt_pkg.vhd -- GENERATED by tools/gen_pkg.py from style_constants.json. Do not edit.")
    L.append("library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n")
    L.append("package gilt_pkg is")
    L.append("    -- palette word: Y(29:20) Cb(19:10) Cr(9:0), internal BT.601 (u = Cb, v = Cr)")
    L.append("    type t_pal is array (0 to 1023) of std_logic_vector(31 downto 0);")
    L.append("    constant C_GPAL : t_pal := (")
    for i in range(0, 1024, 4):
        chunk = ", ".join(f'x"{w:08X}"' for w in rows[i:i + 4])
        L.append(f"        {chunk}{',' if i + 4 < 1024 else ''}")
    L.append("    );")
    L.append("    type t_lut16 is array (0 to 15) of unsigned(4 downto 0);")
    for nm, d in (("C_WLUT_K", wl[0]), ("C_WLUT_A", wl[1]), ("C_HLUT_K", hl[0]), ("C_HLUT_A", hl[1])):
        L.append(f"    constant {nm} : t_lut16 := (" + ", ".join(f"to_unsigned({v}, 5)" for v in d) + ");")
    L.append(f"    -- flesh target (Kiss flesh median), BT.601 10-bit")
    L.append(f"    constant C_FL_Y  : integer := {flesh[0]};")
    L.append(f"    constant C_FL_CB : integer := {flesh[1]};")
    L.append(f"    constant C_FL_CR : integer := {flesh[2]};")
    L.append("end package gilt_pkg;")
    open(os.path.join(GILT, "gilt_pkg.vhd"), "w").write("\n".join(L) + "\n")
    # human-readable dump for checking
    with open(os.path.join(HERE, "palette_dump.txt"), "w") as f:
        for bank in (0, 1):
            for step in (0, 4, 7):
                f.write(f"bank {bank} leaf {step}\n")
                for i, (y, cb, cr) in enumerate(palette(bank, step)):
                    f.write(f"  {i:2d}  Y {y:4d}  Cb {cb:4d}  Cr {cr:4d}\n")
    print("wrote gilt_pkg.vhd; flesh", flesh)


if __name__ == "__main__":
    main()
