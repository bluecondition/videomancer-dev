# Doom Raycaster — Real-time DDA 3D tech demo (v2)

## What it does
Renders a Doom-style first-person view of a fully enclosed 16×16 walled map. Knobs move the player and rotate the view; toggles control a minimap HUD, side shading, distance fog, and a crosshair. Slider crossfades between input video and the raycast scene.

## How it works
120-column DDA raycaster running as a multi-cycle FSM during vblank (~6k cycles/field). All distance math is consistent 8.8 fixed-point cell units: position = knob<<2 (cell = pos(11:8)), delta_dist = 131072/|dir| via a 128-entry reciprocal LUT, side-dist init = frac×delta>>8 through a shared split 9×16 multiplier (two narrow partial products, summed in the second register stage). Perpendicular distance (15:8) indexes per-resolution wall-height LUTs (1080p / SD field). Results land in a 128-entry column buffer read by a 9-register pixel pipeline: classify (|y_rel| < half), palette+tint, side shade, brightness, 2-bit fog shift, HUD/crosshair overlays, then wet/dry mix decomposed into per-bit shifted terms (m = 0..8, frame-constant). Output is blanking-gated to neutral (Y=64, U=V=512).

v1 showed only gray because the distance→height slice divided by 1024 while one cell of ray travel added ~32 to side_dist — every wall collapsed to zero height, leaving the floor/ceiling gray everywhere. v1 also mis-scaled player position, drove the sin/cos LUT with only a quarter turn, read toggles from the wrong register, and assumed 1280px HD.

## Controls
- K1 Player X — map X position (0..16 cells)
- K2 Player Y — map Y position
- K3 View Angle — full 360°
- K4 Wall Bright — ±256 luma offset on walls
- K5 Fog Density — shifts the 4-level fog curve
- K6 Palette — tint offset (rotates the 15-color palette)
- T7 HUD — 16×16 minimap top-left, player cell in red
- T8 Shading — darken E/W faces 25%
- T9 Fog — fog on walls + floor/ceiling gradient
- T10 Crosshair — center cross
- T11 Bypass — passthrough
- Slider Mix — wet/dry (9 levels)

## Build status (2026-08-02)
All 6 configs close timing at full clock: HD Analog 74.67 (seed 2), HD HDMI 76.44 (seed 2), HD Dual 76.91 MHz (seed 4); SD ≥ 62. ~3720 LCs (48%), 9 BRAMs. Not HW-tested.

Timing lessons: the S4 mega-stage (y_rel+v_center vs wall top/bot compares, fog adds, shift mux in one stage) was the first critical path — classification became |y_rel| < half in its own stage with fog level precomputed. The second was the 11×5 wet/dry mix multiplier + dry add + clamp in one stage — decomposed into per-bit shifted terms (m frame-constant) summed across two stages.
