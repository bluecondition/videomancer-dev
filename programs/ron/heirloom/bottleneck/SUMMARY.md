# Bottleneck — Heirloom #28

**Source photo**: `photos/WIN_20230429_22_57_43_Pro.jpg` (1920×1080 capture, 2023-04-29)

## Breakdown of the photo

- Horizontal bands of blue / green / black, everything smeared sideways (long horizontal streaks).
- Two hot-pink bottle silhouettes (narrow neck, shoulders, wide body) stand in front, with lime-green highlights along their shoulders and edges.

## Design

| Photo element | Program |
|---|---|
| Bands | per-line palette colour (blue / green / black cycle by K1 pitch, rolling with S10), luma averaged with the source |
| Smear | four-lane interleaved leaky integrator along the line (3-clock loop), leak from P12 |
| Bottles | per-line half-width: neck (wmax/4) above row 270, widening to wmax (K2) by row 810; one or two centres (K5 spacing), drifting (K6) |
| Lime rim | \|\|dx\| − hw\| < K3 |

Generated avid; output conditioning; latency 20; 16 EBR line buffer + 2 EBR palette.

## Controls

| Control | Function |
|---|---|
| K1 Bands | band pitch |
| K2 Width | bottle width |
| K3 Rim | rim width |
| K4 Hue | band palette offset |
| K5 Spacing | distance between the bottles |
| K6 Drift | bottles drift sideways, bipolar |
| S7 Bottles | One / Two |
| S8 Rim | Off / On |
| S9 Colours | Pink bottles + lime rim / Lime bottles + pink rim |
| S10 Bands | Still / Rolling |
| S11 Source | Video / Osc |
| **P12 Smear** | none → everything streaks off the right edge |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 80.5–85.7 MHz, 4.6k LC (60 %), 9 EBR reported (expected 18: the 2048×32 line buffer + palette — check on hardware that the picture passes through intact). First builds missed HD (48 / 68 MHz): the per-line width chain and the 11×10 smear multiplier were pipelined (the smear loop is now 4 stages over 4 pixel lanes). **Not yet flashed.**
