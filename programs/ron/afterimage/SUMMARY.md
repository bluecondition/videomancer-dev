# Afterimage — summary

1-bit dithered 256x128 frame store in EBR (two banks, ping-pong by frame
parity) with stochastic feedback: trails, zoom/drift/spin tunnels, diffusion,
ordered fade, freeze, 8 ink/paper palettes, video keyed through the ink.
Architecture notes: TECHNIQUES.md "Coarse frame store in EBR".

## Status

- **v1.0.0 FINAL except presets** — HW-validated, Ron signed off 2026-10-07
  ("afterimage can be finalized"). Defaults captured from hardware.
- v0.1.0 (2026-09-04): first build, timing-closed, never flashed.

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
