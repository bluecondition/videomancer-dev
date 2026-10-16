# Porthole — summary

Catalogue combo #6 (seed 20260912): spherize + value-noise displacement +
warped picture-in-picture. v0.1.0 (2026-09-11, the v2 shared-multiplier
build) was flashed and liked ("porthole is good"). v0.2.0 is the first
finalizing pass, built from a review of v0.1.0 on 2026-10-12. v0.2.1 (same
day, Ron's pick from Tier 3) adds a round porthole on S10, replacing Mirror.

## Controls (v0.2.1)

| Control | Does |
|---|---|
| K1 Curve | lens strength, bipolar: CCW bulges (fisheye), CW pinches; fitted to the window's rows |
| K2 Zoom | 0.25×–4×, log, 256 steps; CW magnifies |
| K3 Noise | value-noise displacement amount (64×32 lattice) |
| K4 Speed | lattice scroll speed, continuous, 0–16 lines a frame |
| K5 Size | window width 10–~98% of the picture; height follows the screen aspect |
| K6 Position | window centre column |
| S7 Outside | original / warped picture outside the window |
| S8 Noise | window only / everywhere |
| S9 Frame | none / black rim (thickness follows the window) |
| S10 Shape | Rect / Round porthole (diameter = Size, aspect-corrected; exact ring rim) |
| S11 Window | Lens: magnifies what is behind it / Fit: the whole picture squeezed in |
| P12 Drift | figure-eight swing; swing (to half the screen) and speed grow with the slider; 100% = edge-to-edge swoop every ~2 s; glided |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- The window vanished at HD when it reached the right side: `wcx + ww/2`
  was an 11-bit add, so it wrapped past 2047 (Position near 100% at the
  default size). Window bounds are now computed signed and clipped.
- Window content slid when the window was clipped by the left edge. The
  window DDA restarted at the clamped edge, not at the window's true edge.
  Both DDAs now run linearly from the line start (u = uc ± (x − cx)·k), so
  there is no reset at the window edge.
- The lens weight was 1 instead of 0 at exactly rho = 1, which gave the
  full-curve scale on the screen's top line and on the window's top and
  bottom rows (`> 1024` changed to `>= 1024`).
- In Fit mode, the kk product overflowed at small windows with zoom-out and
  pinch. It is now saturated.
- Zoom ran backwards: CW zoomed out. CW now magnifies.
- Size topped out at 85% while labelled 100%. It now reaches ~98%.
- The window height used a fixed `ww·H/4096`, so SD windows were a third of
  their proportional height. The height now follows the screen aspect.

Look:
- The lens is fitted to the window. rho is measured from the window's
  centre row over its half-height, so the window shows a real bulging or
  pinching glass. Before, at the default size, only ~19% of the curve varied
  inside the window and the rest acted as a flat zoom. Outside, with S7
  Warped, the screen lens is as before.
- S11 Crop is now Lens: the window magnifies what is behind it (content
  centred on the window's own column), so a drifting window slides over the
  picture like a lens. Fit (the default) is unchanged.
- P12 Drift is a designed arc. v0.1 swung ±127 px at most (6% of HD), with
  a 14–68 s period. The swing is now a figure-eight (x = sin θ, y = sin 2θ):
  up to ±half the width and ±a quarter of the height. Speed is
  32 + P12²/2048 phase steps a frame, from about 34 s a lap down to 2 s at
  100%. Amplitude is linear in the glided slider, so 0 parks the window at
  Position.
- Off-source reads (noise at the edges, zoom-out, the lens) are mirrored
  back in once, with black beyond. This is Ron's edge-fill rule, with the
  sidewinder mirror instead of the black slivers.
- The rim is `ww/128 + 2` px (9 px at the default HD size, 3–17 px) instead
  of a fixed 4 px.
- Speed is continuous (a 1/64-line accumulator) instead of 16 steps. It has
  the same rate at every knob position, minus the steps.
- Curve, Zoom, Size, Position and Drift glide (ease by diff/8 a frame).

Engineering:
- `no_rw_check` on all six line-buffer RAMs (damier rule).
- Knobs and switches are registered locally. Rim and window bounds are
  precomputed per frame, so the pixel stage has no add-then-compare.
- The per-line sequencer interleaves the screen lens and the window lens on
  one shared multiplier. Free-running post-processed product registers hold
  one copy of each clamp. 27 states, the same count as v0.1's single chain.
- Range test and mirror pick are split over stages 4 and 5. The pure delay
  pipe is one stage shorter, so latency is still 12 clocks in all modes.

Presets: the four v0.1 looks are kept. Zoom and Size values are remapped to
the same scale, and Inset Crop is renamed Wide Lens because S11 = 0 is now
Lens. There are two new presets: Magnifier (Lens + zoom-in + bulge) and
Swoop (P12 100%). All are hand-written; recapture them with
capture_preset.sh.

## v0.2.1: round porthole (S10 Shape, replaces Mirror)

- The circle test is exact and per pixel. A running `(x − wcx)²` (one add
  per clock, preset in blanking) is compared with a per-line threshold
  T = R²·(1 − ρ²). That is the same 1 − ρ² that weights the window lens, so
  the lens fades to nothing exactly at the circle's top and bottom. It costs
  one extra product per line on the shared multiplier (R²/64 × w).
- The rim is an exact ring of thickness t: rim where dx² ≥ T − (2Rt − t²).
- R = Size's window width / 2, so the diameter equals the rect window's
  width. Large sizes are clipped by the screen top and bottom.
- Aspect: ρ uses dy in display pixels, dy·kasp, where kasp = W·(9/16 HD,
  3/4 SD)/H comes from a per-frame divide. The circle stays round on 480i
  and 576i fields. The numeric check is within 2.4% of an ideal circle in
  every mode; HD default R = 449 → 448.
- Mirror was the subtlest switch and is gone; the window map is now always
  forward. Presets: Ship Window and Magnifier are Round. Funhouse's
  Mirror = On is dropped (it would now mean Round).

## Build

`./build_combo.sh porthole` → `out/rev_b/ron/catalogue-combo/porthole.vmprog`.

v0.1.0: 5114 LC (67%), 19 EBR, HD 84.6–85.1 MHz, seed 1 everywhere.

v0.2.0 (2026-10-12, router2 default): all 6 configs pass. HD Analog 80.5,
HD HDMI 81.2, HD Dual 80.1 MHz (all seed 1); SD Analog 76.4, SD HDMI 82.6
(seed 1); **SD Dual 85.8 on seed 2** (seed 1 stalled the router; no
timing problem). About 5,870 LC (76%), 19/32 EBR. Standalone yosys:
`emulate_transparency` = 0 (no_rw_check took effect).

v0.2.1 (2026-10-12, router2 default): all 6 configs pass on seed 1.
HD Analog 83.3, HD HDMI 84.1, HD Dual 75.7 MHz (the thinnest margin);
SD Analog 72.6, SD HDMI 77.9, SD Dual 79.2 MHz. About 6,200 LC (81%),
19/32 EBR.

HW status: v0.2.1 not flashed yet (v0.2.0 was superseded before flashing).
