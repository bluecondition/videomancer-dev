# Tremor — summary

Catalogue combo #17 (seed 20260915): value-noise warp + scanline shear +
duotone. v0.1.0 was flashed and liked ("tremor is good", 2026-09-11); the
edge fill v1 was checked on hardware on 2026-09-13 ("fill OK on tremor"),
and v0.1.2 swapped that fill for the address clamp. v0.2.0 (2026-10-12) is
the first finalizing pass, built from a review of v0.1.2.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Scale | noise cell size 4–512 px, log, 64 steps per octave |
| K2 Drift | the noise field scrolls vertically, 0–8 px a frame |
| K3 Skew | scanline shear slope; rows lean by (row in band − P/2) × skew, clamped ±511 px |
| K4 Period | shear band height 2–257 lines |
| K5 Hue | highlight tone hue |
| K6 Spread | hue angle from the highlight tone to the shadow tone, 0–178° |
| S7 Noise | Warp: noise displaces the picture horizontally / Grain: noise is added to luma |
| S8 Shear | sawtooth / triangle |
| S9 Bands | every other shear band swaps the two tones |
| S10 Field | smooth value noise / hard cells |
| S11 Tone | duotone keyed from luma / from the noise field |
| P12 Amount | glided, squared curve. At 0: ±5 px warp or ±30 luma grain (always on). At 70%: about v0.1's maximum. At 100%: ±¼ of the measured width, or ±511 luma |

## v0.2.0 changes (from the v0.1.2 review)

Bugs:
- Drift tore the noise field. The row coordinate wrapped every 4096 lines,
  but the lattice key did not wrap with it. Above 8 px cells, a horizontal
  tear line scrolled through the warp about a quarter of the time. The row
  index now wraps at 1024 cells, and the key's hash input wraps with it.
- At 4 px and 8 px cells, the lattice key `seed + 128·row + column` aliased.
  Cell columns 128 and up repeated the next row. The key is now a per-row
  hash (two passes of the three rounds) plus the column.
- Grain could push luma below black (clamped at 0, not 64).
- The grain in the tone key was 3 pixels behind the grain in the luma.

Hardware contracts:
- Generated avid (parity-locked start, frame-latched width), as in
  damier, aurora and infrared. Tremor draws all of its output chroma: the
  duotone replaces the source's, and the v0.1 U/V line buffers were dead
  code that yosys pruned. So it falls under the aurora rule (thin coloured
  lines from a wandering source avid).
- HD HDMI bar fix (damier v0.4.4): [1 2 1] on Y/U/V plus luma scaled to
  white ~775, before the gate. The duotone swings chroma ±382 across every
  luma edge, and Grain makes moving high-contrast luma. Side effect: the
  picture is about 19% dimmer at white, and highlights keep more of their
  tone instead of clipping to white.
- Latency 28 → 30 clocks, the same in all modes.

Controls:
- Scale is continuous. v0.1 had 8 octave steps (each 128-count zone of the
  knob did nothing) under a linear "4–512 px" display. The lattice is now
  stepped by DDAs (x per pixel, y per active line) at a per-frame step from a
  64-entry 2^(−i/64) ROM plus an octave shift. The display is 0–100%.
- Amount was linear in fixed pixels (±255 px max) and read raw each frame.
  It is now glided (ease by 1/8 a frame) and squared, with a floor, so Scale,
  Drift and Field are visible at any slider position. Warp is width-relative.
- Skew is glided at 10-bit resolution (was 8-bit raw: one count of pot noise
  moved the band ends by 4 px).
- `no_rw_check` on both luma line buffers (collision-free: the bank being
  written is never the one whose read data is used).

Defaults and presets were translated so each keeps its v0.1 cell size and
amplitude at HD: K1 384 → 439 (32 px), P12 300 → 389 (±74 px). Heat Haze:
K1 586, P12 351. Venetian: K1 147, P12 206. Noise Print: K1 732, P12 590
(±200 luma grain).

## Build

`./build_combo.sh tremor` → `out/rev_b/ron/catalogue-combo/tremor.vmprog`.

v0.1.2 (2026-09-13): HD 77.8–82.8 MHz, 4,780 LC (62%), 11/32 EBR.

v0.2.0 build (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 77.6, HD HDMI 78.1, HD Dual 80.1 MHz; SD Analog 75.5, SD HDMI 85.8,
SD Dual 88.3 MHz. About 5,680 LC (74%), 11/32 EBR (10 luma line buffer, 1
sine ROM). yosys `emulate_transparency` count 0 (v0.1.2: 1).

HW status: v0.2.0 not flashed yet.
