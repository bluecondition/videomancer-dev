# Laser Cathedral — Seven stacked parabolic arcs (frown to flat to smile) studded with rainbow origination dots

## What it does
Seven horizontal arcs stack vertically across the screen, with curvature varying linearly from a deep frown at the top, through straight in the middle, to a deep smile at the bottom. Each arc is studded with origination dots at fixed horizontal intervals — small bright markers riding along the arc. The whole stack uses an 8-entry ROYGBIV rainbow palette, with Y-position selecting hue, and optional animation cycles the palette phase. Phase 2 of the design will add a Beam Count knob driving perpendicular fan-beams from each dot.

## How it works
All seven arcs share one parabola family: `y_i = y_base_i + (i - 3) * (k * (x - cx)^2)`, so only one `(x - cx)^2` is evaluated per pixel and the per-arc bend is just shift+add+negate (i ∈ {0..6}, so (i−3) ∈ {−3..+3} → no multiply). One fabric multiply per pixel (the squared distance), per-frame derived constants absorb the rest via a vsync-paced helper pipeline. Rainbow palette is the hardware-verified U/V-swapped ROYGBIV ROM borrowed from Phosphor (3-bit base index + 7-bit fractional interpolation between adjacent entries). 18-stage pipeline (v2 added 5 stages for beam-math).

## Controls
- **K1 Curvature** — parabola steepness (top 3 bits select shift amount)
- **K2 Translucency** — arc thickness / opacity (display label; VHD comment calls it Thickness)
- **K3 Spacing** — vertical pitch between arcs
- **K4 Beam Count** — 1..4 (Phase 2 feature; currently stepped knob)
- **K5 Rotation** — −180..+180° (display label; VHD uses it as Hue Offset)
- **K6 Center X** — parabola apex X position
- **T7 Animation** — enable hue DDS sweep
- **T8 X Mirror** — mirror horizontally
- **T9 Y Mirror** — mirror vertically
- **T10 Invert** — invert luma
- **T11 Saturate** — boost saturation
- **Slider Shift Speed** — hue DDS rate (when Animation is on)

## Presets
- Default
- Deep Bend
- Flat Lines
- Inverted
