# Turpentine — chroma as wet paint

Redshift treated luma as mass-energy; Mercurial treated luma as material.
Turpentine exploits what chroma actually **is**: a 2D vector per pixel.
(U−512, V−512) has a magnitude (saturation) and a direction (hue), so the
picture carries its own force field — read here as pigment: **saturation =
how much wet paint is loaded there (gray = bare paper, never moves), hue =
the direction the paint runs when solvent hits it.**

## The Solvent slider (P12, headline; 0 = exact dry video by construction)

| Zone | Look |
|---|---|
| 0–25% VARNISH | the picture becomes a painting: brush strokes oriented along each region's hue direction, canvas tooth on bare (low-sat) paper |
| 25–70% WASH | solvent soaks in: paint runs — per-pixel hue-directed recursive smear, wetness ∝ saturation; the grain dissolves |
| 70–100% FLOOD | wetness floor rises (even weak colors run), flow + turbulence boost, drips detach from saturated surfaces and streak down carrying their color |

## Controls

K1 Compass (rotates the hue→run-direction map) · K2 Brush (stroke pitch;
zone 0 = off) · K3 Flow (trail persistence ceiling) · K4 Turbulence
(per-line run jitter) · K5 Pigment (chroma boost, shift-add knees) ·
K6 Absorb (saturation threshold to start running) · S7 Medium (Paint =
colors run / Ink = solvent runs, pigment pinned) · S8 Grain · S9 Drips ·
S10 Residue (runs Thin = lighten / Stain = darken) · S11 Creep (compass
auto-orbit).

## Architecture

Single streaming pipeline E1..E15, **C_LATENCY = 15**.

- **Core engine = shearline's recursive line feedback** (dual-bank
  ping-pong buffers, Y 2048 + U/V 2:1 decimated = 22 EBR, write parity
  carried per-pixel), upgraded from global shift + global decay to
  **per-pixel terms from the pixel's own chroma vector**: dx = horizontal
  component of the compass-rotated chroma (4 s8×s8 rotate products from
  per-frame qsin cos/sin + 1 dx-gain mult), t = wetness from saturation.
- **The paint carries its own solvent** (the load-bearing twist): t comes
  from max(live wet source, saturation of the ARRIVING paint read from the
  previous line, ~1.5·max(|cu|,|cv|)). Saturated regions invade dry paper
  below; the (1−t) live inject dilutes the carried pigment until the run
  dries out on its own. Bounded lerp = unconditionally stable.
- **Grain**: flow-octant triangle stripes (w = y / x / x±y so strokes run
  ALONG the flow — zero multiplies) on painted areas, hash canvas tooth on
  paper; display-only, never fed back. Envelope rises in VARNISH, dissolves
  through FLOOD.
- **Drips**: redshift's per-8px-column particle machinery. surf_ram
  {row 10, u3, v3} = topmost saturated row per column + quantized chroma
  (raster min-write + 1 row/field drift re-adapt); part_ram {pos 10, u3,
  v3}; drips fall 2..5 rows/field, respawn AT the surface carrying its
  color, density = flood zone. Vblank 7-state column walk, reads landed
  in FFs.
- **Vblank sequencer** builds all per-frame terms one add per step (zone
  envelopes, folded wetness constants, cos/sin captured two-behind through
  the registered qsin port).

**Multiplies (8/pixel)**: 4 rotate (s8×s8), dx gain (s8×s7), wetness
(u8×u8), 3 lerp (s9×s11). Pigment grade and grain are shift-only.
**EBR 24/32**: feedback 22 + drips 2.

## Timing battle log (74.25 MHz HD; routed paths via manual nextpnr re-run)

1. v0.1 first build: HD ~58 routed. Critical path = the E8 wet-combine
   serial chain (max+min/2 of carried chroma → ink mux → clamp → max with
   live → threshold subtract → clamp): 7 ns logic + 10 ns routing in one
   stage. Fixes, no extra latency: `max(a,b)−thr = max(a−thr, b−thr)` —
   threshold folded into per-frame constants (c_pnt = 8+thr, c_ink =
   182−thr), live source pre-thresholded upstream at E5, carried magnitude
   simplified to 1.5·max(|cu|,|cv|) built at E7. Also: read address
   REGISTERED at E5 (comb adder feeding 22 EBR address pins is
   routing-hostile), grain octant/pitch/tri spread E4→E5→E6, Residue bias
   pre-folded into the live term so the write-back sum stays two-term.
   → HD HDMI 80.4 / Dual 76.9 pass; hd_analog 69.8 routed.
2. Next path: the E11 lerp multiply (11×11, 9.3 ns of routing). Mercurial
   squeeze: t cut to its top 8 bits (s9×s11, same curve) → hd_analog
   73.5–74.05 across seeds — close but no pass, and half the seeds STALL
   router2 (overuse=2 forever; the build script's timeout treats that as
   a crash and reseeds).
3. Final path: the E5 read-address stage itself (tj add → ±15 clamp →
   13-bit subtract → wmax compare, 4 serial carry chains). Dropped the
   aesthetic dx clamp (natural range ±38), left-edge test = subtract's
   sign bit (free), right-edge test = px vs a per-line 40-px margin
   (s_wedge, computed at hsync) in PARALLEL with the subtract; dx-gain
   mult narrowed to s8×s7. → all 6 configs pass ROUTED, seeds 1–2.

## Status

v0.1.0 — all 6 configs routed-closed at full clock (no hd_clock_divisor),
every config exiting the seed sweep early: HD Analog 79.1 (seed 1), HD
HDMI 75.6 (1), HD Dual 81.3 (2), SD 71.6/81.7/80.1 (1). ~6100 LCs (79%),
24 EBR. Built + packaged (out/rev_b/ron/turpentine.vmprog). NOT yet
HW-tested, NOT yet sim-checked. HW check items: hue→direction mapping
sense (sim renders BT.601; if the compass feels mirrored on hardware it's
the U/V convention, rotate with K1), drip color quantization (3+3 bit),
Ink-mode wetness balance (c_ink constant 182), right-edge 40-px dry
margin visibility at max flood.

## Validator gotchas (cost two package cycles)

Preset names max 15 chars; program description max 127 bytes.
