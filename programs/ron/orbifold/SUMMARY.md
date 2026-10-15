# Orbifold — summary

v0.3.0 (2026-10-04) finalization rework. Status: timing-closed, NOT yet flashed.

## Controls
K1 Shape (Circle/Square/Diamond/Octagon/Star/Sparkle/Cross/Mixed), K2 Radius,
K3 Thickness, K4 Depth 1-6, K5 Scale (0-3 octaves, up = bigger cells),
K6 Zoom (bipolar drift, centre deadband = still, right = zoom in),
S7 Corners/Middles, S8 Rings/Discs, S9 Video Off/Weight (luma fattens rings),
S10 Overlap Stack/XOR, S11 Rainbow/Phosphor, P12 Animate (per-level orbit +
radius pulse rippling down the scales; 100% = half-cell swing, rings collapse/swell).

## v0.3 changes vs v0.2
- Zoom: exp2-table mantissa (constant zoom rate, was linear = lurch every octave);
  12-bit mantissa (was 8 -> 3-4 px steps at screen edge); coarsest/finest level
  thin out over the last quarter octave before the drift wrap (was a pop);
  hue keyed on continuous size class + screen-space gradient (no colour jumps).
- K1 Scale + old P12 manual zoom merged into continuous K5 Scale; K5 Hue removed.
- Video background (cheesy, and green on HW: program was `synthesis`) replaced by
  Video Weight halftone; program_type now `processing` + blanking gate.
- Black background Y=64 (was 0); labelled-default index fix on K1/K4;
  serrated-vsync guard on the per-field sequencer.
- Vblank sequencer (one serial 16x10 multiplier, ~700 cycles) computes per-level
  thresholds, orbit offsets, hues, zoom mantissa.

## Build
All 6 configs seed 1, router2: HD Analog 90.6, SD Analog 89.7, HD HDMI 99.4,
SD HDMI 94.4, HD Dual 90.0, SD Dual 83.3 MHz. ~7300/7680 LC (95%), 0 BRAM.
Little LC headroom left -- the per-level pipeline FFs are the bulk.
v0.2 sources backed up in .orbifold_work/.

## Community port — fit for the stock SDK (v0.3.2, 2026-10-15)
- The stock LZX SDK builds 8 configs; its SD Standalone core is ~600 LC larger
  than HD. v0.3.1 packed to 7830/7680 LC there (HD Dual 7534) and crashed
  router1 near the limit.
- v0.3.2 replaces the twelve per-pixel 8-bit squarers (6 levels x 2 axes, each
  a 4-bit-square table + h*l product + add over S11/S12, ~600 LUT + 400 FF) with
  twelve 256 x 16 square ROMs (`C_SQ8`, 12 of the 32 EBRs, all of them idle
  before). Each ROM is addressed by the registered `s10_qa`/`s10_qb`; its output
  register is the S11 stage, and S12 re-registers into fabric before the S13 add,
  so the latency is unchanged.
- Proven bit-exact against v0.3.1: GHDL side-by-side, 40 frames (403 x 201
  raster), knobs and switches changing mid-picture, 0 mismatches in 3.24 M clocks.
- Packed LC (stock SDK): SD Standalone 7830 → 7250, HD Dual 7534 → 7020.
- **Build seed (stock LZX flow): `[firmware] seed = 2`.** Seed 1 hung
  nextpnr's router1 on SD Dual (a rip-up livelock that never ends; LZX's
  build script has no timeout). Seed 2 builds all 8 configs; own HD timing
  (core without the Sync Out generator) 97.7/94.4/96.4 MHz Analog/HDMI/Dual.
