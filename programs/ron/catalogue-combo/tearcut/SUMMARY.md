# Tearcut — summary

Catalogue combo #115 (seed 20260926): datamosh / block displacement +
scanline shear / horizontal tearing + edge detect. v0.1.0 (2026-09-12) was
flashed and liked ("tearcut is good": torn/waved edge drawing + moshed blocks
over the untorn picture). v0.2.0 (2026-10-13) is the first finalizing pass,
built from a review of v0.1.0. Backup: `.tearcut_work/v010_backup/` (vhd,
toml, vmprog; also git 3fa438d).

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Gain | edge gain |
| K2 Thresh | edges below this are dropped |
| K3 Block | block size 8 / 16 / 32 / 64 px |
| K4 Density | fraction of blocks moshed, ~5% .. ~98% |
| K5 Tear | tear amplitude, ±8 .. ±518 px |
| K6 Bands | tear band height 2 / 4 / 8 / 16 lines (Random) or wave period 64 / 128 / 256 / 512 lines (Wave) |
| S7 Edges | white / tinted (picture chroma at double saturation, ink luma 768) |
| S8 Tear | random per band / triangle wave down the picture |
| S9 Blocks | plain / moshed blocks go negative (luma 64↔940, chroma mirrored) |
| S10 Mosh | shift / hold (block repeats its first column) |
| S11 Ink | light edges on dark / dark edges on light |
| P12 Blend | edges on paper (0) .. edges over the untorn picture (100) |

## v0.2.0 changes (Tier 1 of the review)

Bugs:
- P12 Blend at 100% crossfaded to the dry picture, so the edges vanished
  (at 100% a white edge over a 200 picture came out at 202). With them went
  K1, K2, K3, K5, K6, S7, S8, S10 and S11. Now the background fades
  paper → picture by Blend, and the edge strength keys the ink on top. At 100%
  the untorn picture has the torn, moshed edge drawing over it, as the
  description said. Gain is scaled ×1.156 so Blend 0 keeps v0.1's ink levels.
- Mosh hash, two faults (checked numerically, as on blockcut):
  - With 8-px blocks in HD, the row key was spaced 128 but there are
    240 columns. The right half of the screen was an exact copy of the left
    half one block-row down (100% match).
  - The roll was added to the key, so every re-roll (~3 s) was the previous
    pattern shifted 7 rows and 117 columns (100% match). The blocks slid
    diagonally instead of re-randomizing.
  - Fix: key = block row & block column (no adder), with a per-roll seed
    XORed in after the first round. Best translated match is now 0.02.
    The tear hash was already fine (adjacent-band correlation −0.05).
- Wave period at K6 = 16 ln: the period index used only 2 of 3 bits, so the
  last position wrapped to the shortest period (32 lines, not 512). Wave
  periods now step 64 / 128 / 256 / 512 lines in knob order.
- Negative used `not y`: black paper came out at 959, above white. It is now
  1004 − y with chroma mirrored, so a moshed block is a true negative at any
  Blend.
- Edge and picture were one pixel apart (the dry ring read one clock late).
  Both are now aligned at stage 10.

Hardware contracts / quality:
- Tinted ink luma 940 with double-saturation chroma clipped RGB and lost the
  tint. Tinted ink is now luma 768 (192 for dark ink). White ink stays 940
  (neutral chroma).
- No edges in the C_EDGE (12 px) side columns: a capture's black border
  columns drew a bright vertical line at each side, which the tear then
  dragged across the picture. No edges on a field's first line (gy compared
  against the previous field's last line).
- `no_rw_check` on all five RAMs (all collision-free: Y reads x while
  writing x − 1; E banks alternate by line; ring w − 8 vs w).
- abs(g) negates at full width, then slices. Hardware was already right
  (synthesis truncates), but GHDL sims folded steps ≥ 512.
- Latency 18 → 19 clocks, the same in all modes; blanking gate kept.

Controls:
- K4 Density floor ~5%, so S9, S10 and K3 do something at any Density
  (the shelf audit flagged S9/S10 as dead at Density 0).
- K5 Tear floor ±8 px, so S8 and K6 do something at any Tear. The TOML
  range now shows the real swing, 8 .. 518 px; v0.1 showed 0 .. 255 while
  the tear actually went to ±510.

## v0.2.3 (2026-10-17): label-index fix + resolution scaling

- K3 Block / K6 Bands: the firmware loads a labelled default as the label
  INDEX (K6 "8 ln" arrived as 2 and gave 2 ln). A value with bits 9..2 = 0
  is now read as the index (folded in from the community v0.2.2 fix).
- Resolution scaling. Blocks (power-of-2 sizes keyed by bit slices), tear
  bands and the tear wave counted pixels and FIELD lines: 1.5x too big at
  720p, and at SD 2.2x too big with bands/blocks twice as tall (no interlace
  compensation). Now a Q.1 virtual x and a Q.8 virtual line (stepped from
  the timing ID, registers_in(8); the field-line step is doubled for
  interlace) feed the block key, the row key, the band key and the wave.
  1080p is unchanged. SD is a left-anchored 1440-unit 4:3 window.
  - Hold reads the first pixel of the virtual block (a latch on the
    block-index change), which is `x and not bmask` at 1080p.
  - The mosh offset (+-511 px at 1080p) and the Tear amount are in virtual
    units: 2/3 at 720p, half at SD. The +-511 clamp became a hash-range test
    (|h - 128| >= 82, exactly the old cut-off) with a per-mode limit.
  - Edges, the C_EDGE inset and the line buffers stay in real pixels.

## Build

v0.2.3, 2026-10-17, `./build_combo.sh tearcut` (router2). All six pass; HD
Analog and HD HDMI missed on seed 1 under heavy machine load and closed on
seed 2 (the builder's retry lands them automatically; v0.2.2 without the
label fix had closed all six on seed 1):

| Config | Fmax (seed) |
|---|---|
| HD Analog | 83.3 MHz (2) |
| HD HDMI | 77.8 MHz (2) |
| HD Dual | 81.2 MHz (1) |
| SD Analog / HDMI / Dual | 84.7 / 82.4 / 82.1 MHz (1) |

~5,240 LC (68%), 20/32 EBR. iceunpack OK on all six.

v0.2.0, 2026-10-13, `./build_combo.sh tearcut` (router2). All six configs
passed on seed 1:

| Config | Fmax (v0.1.0) |
|---|---|
| HD Analog | 82.5 MHz (77.5) |
| HD HDMI | 82.8 MHz (75.0) |
| HD Dual | 81.0 MHz (79.1) |
| SD Analog / HDMI / Dual | 79.3 / 79.9 / 81.0 MHz |

~4,900 LC (64%, v0.1 4,640), 20/32 EBR. yosys `emulate_transparency` = 0.
iceunpack OK on all six. Vmprog: `out/rev_b/ron/catalogue-combo/tearcut.vmprog`.
Not flashed. Not committed: commit at hardware sign-off.

## Hardware checklist (v0.2.0)

- P12 Blend: at 0 it should look like v0.1 (edges on black, same brightness).
  Sweep to 100%: the picture fades in underneath while the edges stay at
  full strength on top (v0.1 lost them above ~80%).
- Re-roll (~every 3 s): blocks re-randomize in place (v0.1 jumped diagonally).
  At Block 8 px the left and right halves no longer match.
- S8 Wave + K6 at 16 ln: the longest zigzag (512 lines), not the shortest.
- S9 Negative: black paper turns white (not super-white), and at high
  Blend the picture inside moshed blocks is a colour negative.
- S7 Tinted: the line colour is clearly visible, including over the picture.
- K4 Density at 0: a few blocks (~5%) are still moshed. K5 Tear at 0: a
  slight ±8 px jitter remains.
- Sides: no bright vertical line at the left and right edges in edge mode.
- HD HDMI: watch for thick bright bars (the damier guard would soften the
  1-px lines, so it is held back until they show up).

## Backlog (Tier 2 / Tier 3, not built)

See the review in the session; summary:
- Tier 2: P12 = Tear with a climax (Blend → K5); Wave scroll + re-roll rate;
  HD-scaled Bands; moshed blocks wrap instead of blanking; HD HDMI guard if
  bars appear; exponential Gain.
- Tier 3: Sobel edges; continuous Block size; per-band mosh; interlace
  re-roll on the top field only; single-bank Y buffer.
