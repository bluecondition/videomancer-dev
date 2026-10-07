# Intaglio — the picture, engraved

The frame is redrawn as a copperplate engraving: hatch strokes that follow
the image's form (stroke direction = local luma-gradient direction), stroke
THICKNESS carrying the tone, crosshatch engaging level by level in the
shadows, clean paper in the highlights, contour ink on edges. Fully
streaming and stateless — no coarse field, no frame memory, no warm-up;
every control reacts on the next frame. (Built as the deliberate
anti-thesis of the mercurial/redshift/kudzu field-overlay genre after that
approach was rejected — see feedback_no_field_overlay_genre.)

**P12 "Bite"** (headline): acid depth. 0 = contours only; 0–60% walks the
tonal ladder in (hatch → cross → third direction); 60–100% is the
over-bitten climax — strokes swell until shadows go solid, crosshatch floods
the midtones, foul-bite pits (2×2 hash speckle) spread over bare paper,
contours thicken. (v0.1's ladder froze above 59% and showed nothing below
~12%.)

## Controls

K1 Pitch (continuous, exponential 5.7–89 px, 32 steps/octave) · K2 Angle
(continuous plate rotation 0–180°; 0 = strokes run along the contours) ·
K3 Ink (stroke width gain + contour weight) · K4 Palette (8 labelled
ink/paper pairs: Copper, Sepia, Sanguine, Prussian, Banknote, Iron Gall,
Gold-on-black, Blueprint) · K5 Hand
(hand tremor: per-line stroke-phase jitter) · K6 Style (stipple / engraving
/ woodcut) · S7 Invert (scratchboard: white strokes on inked ground —
gorgeous) · S8 Wash (half-sat video colour under-print) · S9 Contour ·
S10 Cross 90°/45° · S11 Drift (live plate: phases crawl; tremor pattern
boils at ~10 Hz instead of strobing at field rate).

## Architecture

Streaming pipeline S0..S12, `C_LATENCY = 13`. **Zero per-pixel multiplies,
4 EBR** (two 2048×8 luma line buffers), ~4230 LC (55%).

- **3×3 Sobel** over [1 2 1]-h-blurred luma (prism window, 2 line buffers).
- **Stripe-field accumulators** (the load-bearing trick): the 4 candidate
  stroke families (45° apart; v0.1 kept 8 but only ever selected the even 4) are global stripe fields p_k = x·cos + y·sin, maintained
  as running accumulators (+cos per pixel at S6, +sin per line at hsync) —
  zero multiplies, and every pixel picking family k sees the same coherent
  field, so strokes are continuous across regions. The 16 cos/sin constants
  are rebuilt each frame from qsin taps, each scaled by the pitch mantissa
  on a serial shift-add multiplier in vblank (8 tap slots × 16 steps) →
  K2 rotates and K1 scales continuously; phase is always the static slice
  p(15:8), so the pixel path lost its three pitch barrel shifters. Interlace is detected (field-parity toggle) and the line
  step doubled + parity offset so stroke angles are TRUE in frame space.
- **Direction selection**: |gy| vs tan 22.5° (≈.406) and tan 67.5° (≈2.406)
  of |gx| → 4 symmetric bins (22.5° selection granularity shreds soft
  shading into patchwork). gy is measured UPWARD but the stripe fields step
  DOWN, so differing signs = the 45° family (v0.1 had the diagonals
  mirrored: strokes ran ACROSS diagonal forms; its rounding was also skewed
  ~11°). With a
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

## v0.2.0 (2026-10-04) — review fixes

Mirrored diagonals fixed + symmetric bins; blanking gate (neutral outside
avid; outputs also reset to neutral); palette stored U/V-swapped (Wash dry
path untouched); paper luma ~760 and Wash luma clamped at 768; 11-bit
active-line counter (stipple froze below line 1000 on 1080p); serrated-vsync
guard on drift/interlace detect/sequencer restart; interlace offset now on
the bottom field (field_n='0'); Cross 45° third level uses +135° (was the
same family as the cross → dead level); P12 remap + overbite climax;
continuous Pitch; K4 palette knob. v0.1 sources in .intaglio_work/.

## Status

v0.1.0 — sim-verified; all 6 configs pass ROUTED timing on seed 1, first
try, with real margin: HD Analog 84.7 / HD HDMI 86.1 / HD Dual 82.5 /
SD 80.0–90.8 (displayed accepted-seed numbers; no seed sweeps needed).
Packaged (out/rev_b/ron/intaglio.vmprog); NOT yet hardware-tested. HW check items:
ink/paper tint polarity (BT.601-standard constants), tremor feel on live
video, bucket flicker on noisy camera sources (if strokes shimmer, raise
the direction floor or widen the hold run).
