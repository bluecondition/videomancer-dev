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
