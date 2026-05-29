# Doom Raycaster — Real-time DDA 3D tech demo

## What it does
Renders a Doom-style first-person view of a small 16×16 walled map. Knobs move the player around the map and rotate the view; toggles control distance shading, fog, a HUD, and a crosshair. The classic 3D corridor look driven from a synthesiser module.

## How it works
80-column DDA raycaster running as a multi-state FSM (DDA_IDLE through DDA_DONE). Per frame, each column casts one ray through the fixed map; reciprocal-distance lookups come from a 128-entry LUT, results are cached in a 128-entry column buffer with wall half-height, side flag, and wall type. Position is 10.4 fixed-point, angle is 8-bit. Wall colours come from a 15-entry palette and an HD wall-height LUT. Optimised for HX4K with a 4-stage output pipeline and reduced map/column count vs the original draft.

## Controls
- K1 Player X — map X position
- K2 Player Y — map Y position
- K3 View Angle — 0..360°
- K4 Wall Bright — wall luma scale
- K5 Fog Density — distance fog strength
- K6 Palette — palette tint offset
- T7 HUD — on-screen HUD overlay
- T8 Shading — distance-based wall shading
- T9 Fog — fog falloff on/off
- T10 Crosshair — centre crosshair on/off
- T11 Bypass — passthrough
- Slider Mix — wet/dry between original and raycast view

## Presets
(none defined in toml)
