# Matrix Rain

Classic "Matrix" digital rain for Videomancer. A black field is divided into
fixed **16×16 glyph cells**. Every vertical column has an independently-seeded
**falling head** that advances once per video frame (vsync tick) at a
per-column randomized speed. The head glyph is a bright near-white green; the
trailing glyphs fade bright-green → dark-green → black over a configurable tail.
Glyphs **mutate** to new characters at random, staggered times. Pure synthesis —
the incoming video is ignored (genlocked to its sync only).

## Controls

| Ctl | Reg | Name | Effect |
|-----|-----|------|--------|
| K1 | 0x00 | Fall Speed | base fall step per frame |
| K2 | 0x01 | Density | fraction of columns actively raining |
| K3 | 0x02 | Tail Length | number of visible trail rows |
| K4 | 0x03 | Mutation | glyph re-roll speed |
| K5 | 0x04 | Hue Tint | shifts the green hue (centre = pure phosphor green) |
| K6 | 0x05 | Brightness | overall luma scale |
| S7 | 0x06 b0 | Head Glow | 1px horizontal bloom on the head glyph |
| S8 | 0x06 b1 | Mirror | mirror glyphs horizontally (authentic look) |
| P12 | 0x07 | Glyph Size | cell size = 16<<zoom px (16 / 32 / 64 / 128) |

## Implementation notes

- **Grid:** the 16×16 glyph bitmap is drawn into a cell of `16<<zoom` px (zoom
  0..3 from P12 → 16/32/64/128 px). `col = x>>(4+zoom)` and the glyph pixel is
  `(x>>zoom) mod 16` — a nearest-neighbour upscale. All power-of-two bit-slicing:
  no dividers, no multipliers, no zoom reducer (cf. ziffern, which it is forked from).
- **Per-column heads:** a 128-entry **BRAM** holds each column's head position in
  8.6 fixed point. Once per frame, during vblank, an FSM walks the columns and
  does `head += speed[col]`; when the head + tail scroll off the bottom it
  respawns a random few rows above the top (LFSR-seeded gap). `speed[col]` is
  `base + per-column hash jitter` — all shift-only.
- **Randomness:** a free-running 16-bit Fibonacci LFSR (taps 16,14,13,11) plus
  integer hash (`mix16`) for column seeds, speeds, density test, glyph indices
  and mutation phase. No DRAM.
- **Tail fade:** a fixed 16-entry brightness curve indexed by distance-below-head,
  cut off at the tail-length knob; scaled by the brightness knob via a shift-only
  `scale8` LUT. Implemented as a per-row luma multiplier on the green output.
- **Glyph ROM:** generated at build time by `matrix_rain.py` into
  `matrix_rain_glyphs_pkg.vhd` (16×16 1bpp), flattened into a single-port BRAM
  (`addr = glyph*16 + row`, the `*16` is a shift) — one registered read per pixel.
- **Colour:** BT.601 YUV stored U/V-swapped for the Videomancer hardware
  convention (stored U=Cr, V=Cb), matching ziffern / the verified phosphor table.
  Body green Y=831 U=155 V=262; head whitens chroma halfway to neutral with
  luma ≈ max.
- **Pipeline:** 7 register stages (S0 coords → S1 head-BRAM align → S2 distance/
  lit/hash → S3 glyph index/mutation → S4 glyph ROM read → S5 bit+glow → S6
  colour) + output regs. Sync passthrough delayed `C_SYNCD` (=5) to land with S6.

## Glyph set / font

Half-width katakana **U+FF66–U+FF9D** (the authentic Matrix characters), digits
**0–9**, and a few Latin/symbols (`A Z : = * + - .`) — 74 glyphs total.

Font: **GNU Unifont 15.1.05** (`unifont_jp.hex`), a native 16-px bitmap font,
dual-licensed GNU GPL-2.0-or-later (with the font embedding exception) and SIL
OFL 1.1 — both compatible with this program's GPL-3.0. The vendored `.hex` is a
verbatim subset of the upstream data (the glyph rows for the codepoints we use,
in ROM order); `matrix_rain.py` only re-packs it into VHDL. Each half-width glyph
is 8×16 and is centred in its 16×16 cell, so the **Mirror** switch is meaningful.

## Build / verify

```
./build_programs.sh ron matrix_rain          # runs the py hook, synth, packs .vmprog
# -> out/<hardware>/ron/matrix_rain.vmprog

# render a frame and sweep pots:
cd tools/vhdl-image-tester
lzx-vhdl-cli simulate matrix_rain --set 0x00=900 --set 0x02=1023 --set 0x06=0x03
```
