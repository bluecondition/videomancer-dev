# Intaglio — the picture, engraved

The frame is redrawn as a copperplate engraving: hatch strokes that follow
the image's form (stroke direction = local luma-gradient direction), stroke
THICKNESS carrying the tone, crosshatch engaging level by level in the
shadows, clean paper in the highlights, contour ink on edges. Fully
streaming and stateless — no coarse field, no frame memory, no warm-up;
every control reacts on the next frame. (Built as the deliberate
anti-thesis of the mercurial/redshift/kudzu field-overlay genre after that
approach was rejected — see feedback_no_field_overlay_genre.)

**P12 "Bite"** (headline): acid depth — barely-touched plate → full tonal
engraving → over-bitten crosshatch swallowing the midtones. The whole tonal
ladder (t1/t2/t3) slides with it.

## Controls

K1 Pitch (stroke spacing, 4 zones ~8/16/33/66 px) · K2 Angle (continuous
plate rotation 0–180°) · K3 Ink (stroke width gain + contour weight) · K4
Tint (near-black ink tint direction: black/sepia/blood/iron-blue) · K5 Hand
(hand tremor: per-line stroke-phase jitter) · K6 Style (stipple / engraving
/ woodcut) · S7 Invert (scratchboard: white strokes on inked ground —
gorgeous) · S8 Wash (half-sat video colour under-print) · S9 Contour ·
S10 Cross 90°/45° · S11 Drift (live plate: phases crawl; tremor pattern
boils at ~10 Hz instead of strobing at field rate).

## Architecture

Streaming pipeline S0..S12, `C_LATENCY = 13`. **Zero per-pixel multiplies,
4 EBR** (two 2048×8 luma line buffers), ~4230 LC (55%).

- **3×3 Sobel** over [1 2 1]-h-blurred luma (prism window, 2 line buffers).
- **Stripe-field accumulators** (the load-bearing trick): the 8 candidate
  stroke families are global stripe fields p_k = x·cos + y·sin, maintained
  as running accumulators (+cos per pixel at S6, +sin per line at hsync) —
  zero multiplies, and every pixel picking family k sees the same coherent
  field, so strokes are continuous across regions. The 16 cos/sin constants
  are rebuilt each frame from borrowed qsin taps → K2 rotates the plate
  continuously. Interlace is detected (field-parity toggle) and the line
  step doubled + parity offset so stroke angles are TRUE in frame space.
- **Direction selection**: |gy| vs four shift-approximated tan() boundaries
  of |gx| → 8 bins over 180°, then ROUNDED to the even 4-direction set
  (22.5° selection granularity shreds soft shading into patchwork), with a
  63-px **hold-last-direction run** so ridge/valley zones (gradient through
  zero) inherit the neighbouring slope's stroke direction instead of
  striping with the default family. Flat areas fall to the plate-angle bin.
- **Tone → ink**: dark = 255−luma; per-level width = base + (dark−t)>>wsh
  (t1 hatch / t2 cross at 90° or 45° / t3 third direction), distance-to-
  stripe-center compare + half-ink antialias band. Stipple swaps the test
  for hash-gated diamond dots (radius capped below the half-cell so dots
  stay separate; density = 4·w). Woodcut is a pure per-frame parameter
  preset (fat strokes, both from K6).

## Timing log (74.25 target)

v0.1: HD missed at 55% LC — a genuine logic-depth path, not congestion:
the fused tone→width chain (`not cl − t` → dynamic `>>wsh` barrel → +base
→ clamp, ×3 levels) in one stage. Split over S6 (subtract) / S7
(shift+clamp) / S8 (pipe) → hd_analog routed 75.3 at seed 1. The width
ladder depends only on tone + per-frame constants, so it starts two stages
before the distances it pairs with.

## Sim notes (GHDL image-tester, still-life test scene)

- Stroke scale bug v0.1: sliced phase bits (10:3) instead of (7:0) — 16×
  too coarse (read as a grid of bars).
- Direction floor 96 → only hard edges steered strokes; 24 lets soft form
  shading (sphere, drapery) steer. Below-floor zones use the hold run.
- Verified: sphere/cylinder/drapery render as engraving with tonal stroke
  widths; opposite fold slopes correctly take opposite stroke directions;
  stipple carries tone; woodcut bold; scratchboard excellent; Drift+Hand
  boil at ~10 Hz (was field-rate strobe: mean |Δframe| 85 → seed the line
  hash from s_dracc(11:5)).

## Status

v0.1.0 — sim-verified; all 6 configs pass ROUTED timing on seed 1, first
try, with real margin: HD Analog 84.7 / HD HDMI 86.1 / HD Dual 82.5 /
SD 80.0–90.8 (displayed accepted-seed numbers; no seed sweeps needed).
Packaged (out/rev_b/ron/intaglio.vmprog); NOT yet hardware-tested. HW check items:
ink/paper tint polarity (BT.601-standard constants), tremor feel on live
video, bucket flicker on noisy camera sources (if strokes shimmer, raise
the direction floor or widen the hold run).
