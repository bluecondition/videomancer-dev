# Ridgeline — summary

Catalogue combo #98 (seed 20260924): noise displacement + edge detect.
v0.1.0 (2026-09-11) was flashed and liked ("ridgeline is good",
2026-09-12), with no later hardware notes. It was never touched by the
2026-09-13 edge-fill pass. v0.2.0 (2026-10-12) is the first finalizing pass,
built from a review of v0.1.0. The v0.1.0 sources and vmprog are in
`.ridgeline_work/` (`ridgeline_v010.*`) and git 3fa438d.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Amount | displacement, squared: 0..±511 px (±766 with two octaves). The default (385) is v0.1's 75 px |
| K2 Scale | noise frequency across x, squared, glided: 0..7.5 cycles across 1920 px. 50% ≈ v0.1's top; the default (190) is v0.1's default |
| K3 Speed | drift speed (unchanged) |
| K4 Gain | edge gain: ink alpha = edge × gain / 128 |
| K5 Edges | Roberts / Horizontal / Vertical / Sobel, now labelled; a zone changes only 12 counts inside it |
| K6 Ink | ink luma, black (0) to white (100), continuous. Lines are a lerp toward the ink by the edge alpha |
| S7 Ground | Picture: ink over the displaced picture / Paper: ink on plain paper, white under dark ink and black under light ink (switches at 50%, with hysteresis). On paper the lines carry the picture's colour (chroma × alpha) |
| S8 Order | Warp-Edge: edges between the displaced line above and the input line (the ridges, v0.1's look) / Edge-Warp: the input's own edges, displaced with the picture |
| S9 Chroma | chroma follows luma / on its own noise phase (unchanged) |
| S10 Push | the input gradient adds to the read offset (unchanged) |
| S11 Octaves | one / two (unchanged) |
| P12 Rows | noise frequency down the lines, squared, glided. 0 = pure column waves; 35% ≈ v0.1's top (4 cycles a frame); 100% = 34 cycles a frame, a 32-line ripple that shreds the picture (16-line with two octaves). The default (156) is v0.1's default |

## v0.2.0 changes (from the v0.1.0 review)

Dead or mislabelled controls:
- K6 Ink was a switch. Only its top bit was read, so 0–49% drew black
  lines and 50–100% white ones. It is now a continuous ink luma.
- Edge-Warp disabled K5 and K6. Its write-side edges were a horizontal
  gradient only, always white, over a half-brightness picture. Now the
  luma buffer stores 8-bit luma plus a 4-bit edge alpha per pixel, the
  write-side detector runs all four K5 modes, and both orders share one
  ink compositor. The write side's "line above" is the write bank's own
  old line (2 lines up), read through that bank's idle read port 6 columns
  ahead of the write, so it costs no extra RAM.
- "On black" drew black lines on WHITE whenever Ink was below 50%. S7 is
  now Ground: Picture / Paper, with the paper chosen opposite the ink.
- The "Pushed Lines" preset asked for Sobel with K5 = 3, which decodes
  as Roberts. It is now 896.

Bugs:
- Reads off the line painted black, and the capture's black blanking
  columns showed as a seam. Ridgeline was missed by the 2026-09-13
  edge-fill pass. Reads are now clamped to the line's own edge pixel
  (C_EDGE = 12), the same as the other displacement combos.
- Over mode pushed luma below black (down to 0) and above white (up to
  1023), because v0.1 added or subtracted the edge with a 0..1023 clamp.
  The lerp now always stays between the picture and the ink.
- Pot noise jumped the picture. K2 used 6 bits: a 16-count boundary step
  moved the right side of an HD frame ~47 px at full Amount. P12 used 8
  bits. Both now use all 10 bits, glided by ⅛ of the gap per frame (24-bit
  phases).

Hardware contracts:
- U/V swapped on rows whose hsync→avid parity differs from the row above
  (each row replays the row above from the line buffer; drape v1.3).
- Read and write banks latched per line at the avid rise. The read tail
  runs ~10 clocks past the avid fall, which is past hsync on SD/PAL
  front porches.
- HD HDMI bar conditioning (damier v0.4.4): [1 2 1] on Y/U/V, then luma
  scaled to white ~777. Thin ink lines are near-full-swing edges that
  move every frame, which is the known trigger.
- no_rw_check on all six line-buffer RAMs.
- Latency 19 → 23, the same in all modes.

## Build

`./build_combo.sh ridgeline` → `out/rev_b/ron/catalogue-combo/ridgeline.vmprog`.
v0.1.0: 6/6 seed 1, HD 79–86 MHz, 3,800 LC, 22 EBR.
v0.2.0 (2026-10-12): 6/6 on seed 1 with router2. HD 75.2 / 84.5 / 76.1 MHz
(Analog / HDMI / Dual); SD 75.3 / 81.5 / 79.4 MHz (27 MHz needed). 5,381 LC
(70%), 24 EBR, and no transparency emulation (yosys `emulate_transparency`
count 0). The HD Analog margin is placement noise: the same netlist routes
at 74.4–80.9 MHz over seeds 1–5, and its critical path is the core's SPI
readback (`s_spi_addr` → `din`, 10.7 of 13.3 ns routing), not Ridgeline's
logic. The first v0.2.0 build (74.7 MHz) had the lerp multiplies in one
stage behind one fanned-out alpha register. They are now split hi/lo over
two stages, with separate luma and chroma alpha registers. That build's
vmprog is in `.ridgeline_work/ridgeline_v020_build3.vmprog`.
NOT FLASHED, NOT COMMITTED.

## What to check on hardware

- Default settings should look like v0.1 (same displacement, scale, rows
  and black Roberts lines). The [1 2 1] output smoothing softens the lines
  slightly, and the edge columns now fill instead of going black.
- K6 sweeps continuously, black lines → grey → white.
- S8 Edge-Warp: K5 and K6 now change the picture. Look for clean contour
  lines that move with the picture. If they look stepped (4-bit alpha),
  that's the Tier 2 widening.
- S7 Paper: coloured lines on white paper (Ink < 50%) or on black (Ink > 50%).
- P12 from 0 to 100%: column waves → ripples → shredded 32-line ripple.
- HD HDMI: no thick bars and no U/V-swapped lines over coloured areas.

## Open / next

See the tiered list in the 2026-10-12 session (Tier 2 / Tier 3 below, not
built).

Tier 2:
- K3 Speed bipolar (drift either way) and squared with a faster top. At
  v0.1's top a cycle takes ~2 s. Octave 2 also drifts at Speed 0 (a fixed
  +37 per frame); decide whether 0 should freeze.
- Coloured ink: K6 sweeps black → hues → white, or ink lines take the
  picture's colour on the Picture ground too.
- Push: a signed or smoothed gradient, so pushed edges tear as bands
  instead of 1-px sparkles.
- If the Edge-Warp lines look stepped, widen the stored alpha from 4 to 6
  bits (14-bit words, +2 EBR).
- If Rows doesn't feel like the performance control, swap P12 and K1
  (Amount on the slider).

Tier 3:
- Scale and Rows in screen units, so SD shows the same ripple count as
  HD (SD now shows 3/8 of HD's horizontal cycles).
- True 2-D value noise (hashed lattice) instead of separable sines.
- Full-resolution chroma (+4 EBR). The top row shows the previous frame's
  last line, as in v0.1.
- Presets captured from the device once the look settles (4 of 8 are
  hand-written).
