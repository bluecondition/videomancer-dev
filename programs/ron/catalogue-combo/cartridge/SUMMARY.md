# Cartridge — summary

Catalogue combo #15 (seed 20260914): cartoon / cel shading (ink outlines + flat
luma bands) + retro palette quantization. v0.1.0 (2026-09-10) was flashed and
liked ("cartridge is good", 2026-09-11). v0.2.0 is the first finalizing pass,
built from a review of v0.1.0 on 2026-10-12.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Edge | ink threshold (lower = more ink); the whole travel is live |
| K2 Levels | luma bands: Off / 10 / 8 / 6 / 5 / 4 / 3 / 2, spread over black..white |
| K3 Width | ink weight 1 / 2 / 4 / 8 px → lines 2 / 4 / 8 / 16 px, in both directions |
| K4 Palette | Gameboy, Virtual Boy, CGA, ZX Spectrum, Apple II, C64, EGA, Pico-8 |
| K5 Dither | 4×4 Bayer dither before the quantizer (2-px cells in HD) |
| K6 Range | luma gain into the bands and palette, 0–400% (100% = unity) |
| S7 Ink | Black / Tinted (darkest palette luma carrying the fill's hue) |
| S8 Order | Bands First (bands, then palette) / Palette First (palette, then its luma banded) |
| S9 Edges | Ink / Original colour |
| S10 Luma | from the palette / from the cel bands (palette hue only) |
| S11 Invert | luma inverted (black ↔ white) before the bands and palette |
| P12 Inking | takes the threshold down to ¼ and adds up to 3 weight steps |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- Labelled-default trap: the firmware loads a labelled default as its label
  index, so v0.1 opened on Gameboy / 8 bit / 1 px instead of EGA / 4 bit /
  2 px. Small values are now honoured as an index until each knob first reads
  a real position (infrared's guard).
- The Levels decode didn't match the label zones (5 labels split 1024 evenly,
  the code split it 1/2/2/2/1 of 8ths). Levels now has 8 labels: a
  power-of-2 decode that matches exactly.
- Levels 8 / 6 / 4 bit did nothing in the default order: the quantizer LUT
  only reads Y4 Cr3 Cb3. The posterize also ORed in a half-step bit, which
  raised neutral chroma by +2..+128 codes (grey got a colour cast) and lifted
  black. Levels is now luma bands spread evenly over black..white from a ROM
  (`C_POST`). Chroma is no longer posterized, so greys stay neutral.
- The CGA palette was full range: black at code 0 (below black level), white
  at 1020, chroma about 14% hot. All palettes are now generated in studio
  range by `tools/gen_tables.py`. Gameboy, C64 and EGA come out within ±1 code
  of v0.1.
- The LUT cells were not centred: a neutral grey was looked up as Cr/Cb
  576/576 and black as Y 96, so greys could pick a tinted entry. The cells are
  now centred on 64k (luma) and 128k (chroma), so black and neutral are cell
  centres.
- The picture was 8 px left of its syncs: video latency was 24, syncs 16. The
  last 8 columns showed the source's blanking through the palette. Video and
  syncs now share one latency.
- Ink bars down both frame sides: the edge taps read blanking at the line
  ends. Horizontal edges now need both taps inside the line.

Hardware contracts:
- Generated avid (damier / aurora / infrared parity-locked start, field-latched
  width). Cartridge draws its own palette colour, so the aurora rule applies.
- HD HDMI bar fix (damier v0.4.4): [1 2 1] on Y/U/V plus luma scaled to white
  ~775, before the gate. Ink lines and dither are full-swing pixel edges that
  move with the video.
- Raw video is no longer carried ~20 FFs deep for S9 Original. The picture
  now comes out of the line RAM.
- Controls are latched in vertical blanking (avid absent for 2048 clocks), so a
  knob can't tear the ink counter mid-frame. Nothing is keyed to vsync edges.

Look / controls:
- **Vertical ink**: horizontal lines in the picture now get outlined. v0.1
  only had a horizontal gradient. The picture is shown one line late. A
  2048×32 line RAM holds {row y−1 Y8 Cr6 Cb6, row y−2 Y8, ink counter 4}. The
  vertical gradient |Y(y) − Y(y−2)| marks the two rows straddling an edge. A
  per-column counter extends that ink downward so a horizontal line is as
  thick as a vertical one: 2 / 4 / 8 / 16 rows, with 1 row above the edge and
  the rest below. The first output row is black, because it would show the
  line before the frame.
- K1 Edge: the threshold is now 2..105 (8-bit luma) over the full knob. v0.1
  ran 0..1023 against gradients that top out near 876, so the top half of the
  knob showed no ink. P12 now scales the threshold (down to ¼) instead of
  subtracting from it, so the slider acts the same at any K1. The default
  moved 120 → 280 to keep about the same ink as v0.1's default.
- K4: 8 palettes (was 4). The monochrome handhelds (Gameboy, Virtual Boy) are
  matched on luma alone, with the shades spread evenly. The Virtual Boy's
  brightest red is darker than mid grey, so a nearest-colour match would send
  highlights to black.
- K5: Bayer cells are 2 px × 2 rows in HD (1-px dither at 1080p reads as flat
  colour, and the bar fix's [1 2 1] would erase it). SD keeps 1-px cells,
  keyed on frame rows so the two fields interleave a proper 4×4 pattern.
- K6 Range is displayed 0–400%, so its unity default reads 100% (was 25%).
  Range now comes before the bands, so it also moves S10's cel luma.
- S7 relabelled Black / Tinted (behaviour kept: darkest palette luma with the
  fill's chroma). v0.1 called it "Palette", but the darkest entry is black in
  6 of the 8 palettes, so that reading would make the switch dead.
- S11 Invert maps 16 ↔ 235 (v0.1's bitwise NOT sent black to super-white).

## Build

`./build_combo.sh cartridge` → `out/rev_b/ron/catalogue-combo/cartridge.vmprog`.
Tables: `python3 tools/gen_tables.py` rewrites the block between the GENERATED
TABLES markers in cartridge.vhd.

v0.2.0 build (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 87.0, HD HDMI 93.8, HD Dual 94.3 MHz; SD Analog 101.5, SD HDMI 101.6,
SD Dual 88.0 MHz. About 4,240 LC (55%), 22/32 EBR (16 of them the line RAM; the rest are
the LUT and the palette and band ROMs). Latency 33 clocks in all modes. v0.1.0
was 78–96 MHz HD at 3,589 LC and 1 EBR (its palette ROMs sat in logic).

HW status: v0.2.0 not flashed yet.

Presets: hand-written (Default, Handheld, Comic Strip, Colour Ink, Red
Goggles, Fantasy). The v0.1 three were remapped onto the 8-palette knob
(Handheld = Gameboy, Comic Strip = EGA, Colour Ink = C64). Recapture them with
capture_preset.sh.
