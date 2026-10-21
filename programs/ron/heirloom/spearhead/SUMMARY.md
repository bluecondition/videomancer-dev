# Spearhead — Heirloom #5 (rebuild v0.5.0)

**Source photo**: `photos/WIN_20230103_01_29_30_Pro.jpg`. Rebuilt 2026-10-21 by the REBUILD.md method. Work files:
`.heirloom_work/rb_spearhead/` (`notes.md`, `proto2.py`, `imodel.py`, `gen_tables.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **Two spearheads**, centres x = 532 and 1472: one horizontal triangle, period 940 px. Each is a nest of pale pink /
  yellow / olive / hot pink / maroon / blue / navy / black outline bands, wide at the top and converging to tips at
  the bottom.
- **Spear field**: F = |x − xc| − φ(y), φ = 114 − 1.65·10⁻⁴·y².
  - The core edge measures 114/102/92/76/64/50/28/10/−10/−30 px at y = 15…925, within about 5 px of the fit.
  - Band edges sit at F = 0/31/58/87/108/133/157 px.
  - The core and the black outside are where the 3-bit quantiser clips at 0 and 7.
- **XOR colouring**: colour = PAL[i XOR j].
  - j is the band index.
  - i is a 3-bit code per 41.15-line row (first boundary at y = 9.4). It comes from a vertical triangle, period 658
    lines, peak at y = 421, clamp(⌊8.3 − 9.6·|2θ−1|⌋, 0, 7). Rows 0–25:
    `0 0 0 0 1 2 4 5 6 7 7 6 5 4 2 1 0 0 0 0 1 2 4 5 6 7`.
  - Verified: j = code XOR i is a clean contour field over the whole frame, and each row's i scores 0.7–0.96 against
    under 0.2 for every alternative.
- **Mosaic shadow**: key = A XOR B.
  - A = (F taken 15 px earlier) < 147.
  - B = the ring 80 ≤ F < 100 on the left flank, < 220 on the right, sampled at block centres. Blocks are 40.9 × 41,
    23 per tile, locked to the tile. Each row is displaced right by D = 70 + 64·cos (period 16.75 rows, RMS 18 px).
  - 96 % of blocks agree.
- **Background**: a row colour from the 4-bit phase of the row oscillator, plus 0–2 green steps of (−35, +30, −23) RGB
  from a triangle of u = |x − 994| + 0.7y, period 1350. Mean error 44 RGB, against 45 for a free colour per row.
- **Texture**: edges soft over 6–10 px; the fringes are blends of neighbouring bands.

## What it does
- **Per field**: the knob curves, the band thresholds × the Bands scale and the row-oscillator steps, all on one
  serial shift-add unit; the tile origin; the Thrust glide; the Speed drifts.
- **Per line** (the engine computes the next line in about 40 clocks):
  - φ = C0 − y² × taper;
  - the row DDA, with the row-code and shadow-wave phases;
  - i, the background phase, D, the block-sample offset and the chevron line term.
- **Per pixel** (16 stages):
  - four Q.1 tile DDAs plus a block counter restarted every tile;
  - F, then seven comparators give j;
  - A, and B latched per block;
  - the chevron level;
  - PAL[i XOR j], or the row colour plus steps;
  - the key mux;
  - the shell's output conditioning and blanking gate.
- The shell is the heirloom v0.2.0 one: generated avid, timing-ID virtual-1080 steps, vblank zone.

## Controls
| Control | Photo element | Effect |
|---|---|---|
| K1 Spears | the two spearheads (tile period) | 1…5 across (log); 2.04 = photo; a single spear is centred |
| K2 Taper | the outline curve (k·y²) | blunt columns … short arrowheads |
| K3 Bands | the nested outline bands | width ×0.4…×3; the key and shadow thresholds follow |
| K4 Speed | row re-colouring, shadow wave, chevrons | bipolar, still within ±8 of centre; default slow forward |
| K5 Mosaic | 41-line rows and 41-px shadow blocks | 12…160 lines (log) |
| K6 Bend | the source | luma about mid-grey bends the band contours and chevrons; default 64 = ±14 px |
| S7 Shadow | the mosaic shadow and holes | On / Off |
| S8 Rows | the row re-colouring | On / Off (plain nested bands) |
| S9 Palette | band and background palettes | Photo / Alt (channels rotated) |
| S10 Point | spear direction | Down / Up |
| S11 Source | — | Video (default) / Osc |
| P12 Thrust | the spear core C0 | 0 % short needles from the top · ~41 % (default 424) = photo · 100 % neighbouring spears' bands interlock; glided 1/8 per field |

## Video
Source luma is held in a 1-line buffer, centred on mid-grey and scaled by K6. It is added to the spear field and the
chevron field: bright parts swell the bands, dark parts pinch them. Osc writes a moving triangle pattern into the same
buffer, so Bend stays live in Osc mode. At the default K6 the camera only bends and tears the contours (the photo
has no camera content); turn K6 up for a stronger effect.

## Status
v0.5.0 built 2026-10-21. router2, all six configs on seed 1; all bitstreams pass iceunpack.
- HD Fmax: Analog 82.10, HDMI 81.63, Dual 79.26 MHz.
- SD Fmax: Analog 80.98, HDMI 79.88, Dual 75.81 MHz.
- About 5.83k / 7,680 LC (76 %), 4 / 32 EBR. Latency 16 in every mode.

Not flashed. The presets are hand-written placeholders: Photo, Needles, Interlock, Plain Bands, Big Blocks.

## Deviations from the photo
- **Background V triangles.** The crisp small V triangles between the spears (tips at y ≈ 300 and 910) aren't
  reproduced. The background uses a broad least-squares chevron, and the colour order down the rows is right. Six
  simpler background models fitted worse; they are logged in `notes.md`.
- **Small details dropped.** The shadow wave is a triangle standing in for the fitted cosine. The 1-line glitch
  streaks and the 9-line top strip are not reproduced.
- **Pale pink is dimmer.** It is capped at about 768 output luma by the HD bar fix; the photo has 820.
- **SD crops the sides.** The 4:3 frame crops about 240 virtual px from each side. Both spears stay visible.
