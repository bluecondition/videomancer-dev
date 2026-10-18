# Afterimage — summary

1-bit dithered 256x128 frame store in EBR (two banks, ping-pong by frame
parity) with stochastic feedback: trails, zoom/drift/spin tunnels, diffusion,
ordered fade, freeze, 8 ink/paper palettes, video keyed through the ink.
Architecture notes: TECHNIQUES.md "Coarse frame store in EBR".

## Status

- **v1.0.0 FINAL except presets** — HW-validated, Ron signed off 2026-10-07
  ("afterimage can be finalized"). Defaults captured from hardware.
- v1.0.2 (2026-10-17): SD/480i grid scaling; built, not flashed.
- v0.1.0 (2026-09-04): first build, timing-closed, never flashed.

## v1.0.2 resolution scaling (2026-10-17)

- SD cells were 2x too big relative to the screen: grid 256x128 was capped to
  128x64 whenever W < 1280 (the cell pipeline wrote the cell index taken 3 px
  back, so cells under ~3 px wrote the wrong address). Now 4:3 SD (timing ID
  0/4/8/12) uses 3/4 of the columns (192x128 / 96x64 / 48x32 / 24x16), so
  cells keep the 1080p shape and size relative to screen height. 480i's
  finest grid is 180x120 (exact 4 px x 2 field-line cells, 6.7% larger):
  128 rows on ~243 field lines mixed 1- and 2-line rows (thin stripes).
- Write address = ended cell latched with the write pulse (wc0) instead of
  the F-chain copy; 1-line rows (short-field sources) get sampled on their
  only line (lookahead, never fires in standard modes). Sub-640 widths still
  cap at 64x32 (decimated sims only).
- 1080p bit-identical to v1.0.1; 720p unchanged (256x128 at 5 px).
- Build: builder default router2, all seed 1 -- HD Analog 89.02, SD Analog
  94.32, HD HDMI 91.95, SD HDMI 90.38, HD Dual 76.90, SD Dual 82.86 MHz;
  4122-4145 LC (54%), 16/32 EBR.

## v1.0.0 changes

- Persist (P12) grain fade compared a 9-bit hash field against the 64..256
  survival value, giving 12.5..50% survival instead of the labelled 25..100%
  (the Bayer fade was already correct). Now compares the low 8 bits; 100% =
  ink never clears (long-exposure / light-painting; Freeze + 100% = frame hold).
- Horizontal [1 2 1]/4 luma low-pass at the output, re-gated on blanking,
  latency 6 -> 8. Pre-emptive: Mono ink/paper is a full-swing 1-px step at
  every cell edge, the trigger class found on Ribbon for the HDMI bars.
- Defaults captured from hardware 2026-10-07 (capture_preset.sh --defaults):
  K3 Exposure 194 (-62%), S10 Ink = Video, P12 Persist 570 (~67%). K1 read
  raw 3 (knob at the bottom = Hold in the VHDL zone decode); the tool wrote
  it as label index 3 "Left", corrected by hand to "Hold".
- Hash spot-checked numerically (h/v/diag and frame-to-frame correlations all
  < 0.02; survival and rounding fields independent).

## Build (2026-10-07, builder default router2, all seed 1)

| Config    | Fmax (MHz) | LC        |
|-----------|-----------:|----------:|
| HD Analog | 84.75      | 3977      |
| HD HDMI   | 91.19      | 3975      |
| HD Dual   | 88.59      | 3968      |
| SD Analog | 89.87      | 3972      |
| SD HDMI   | 89.48      | 3966      |
| SD Dual   | 88.64      | 3984      |

16/32 EBR. No non-1 seeds.

## Open / next

- Presets: 4 hand-written (Camera Trails / Tunnel / Game Boy Smear / Ordered
  Fade), 4 slots free; capture from hardware with capture_preset.sh.
- Tier 2 (polish): grid-independent drift speed and Diffuse jitter via the
  fractional-offset path; optional IIR reset at cell start.
- Tier 3 (headroom, 16 EBR free): per-cell column accumulators for true box
  sampling (2 EBR); previous-SAMPLE bank (8 EBR) for Motion / Reveal laws
  (S8 -> Source Luma/Motion, fade style following K4).
