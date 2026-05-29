# Tessera — tumbling 3D triangles falling through the frame

## What it does
Two equilateral triangles tumble about the Z-axis and fall (or rise) through
the frame. Each triangle can be drawn as an outline only or filled; the fill
can be a flat colour or the incoming video (cross-faded by the slider). The
outline of the second triangle can take a +90° hue shift for a two-colour
"Multi" look. Size cycles in a 4-step ±R/8 pattern when Size Variation is
on. Hues for outline, fill and background are independent polar knobs.

## How it works
Processing program that overlays generated triangles on incoming video.
Rotation is done by a 7-iteration unrolled CORDIC engine (no sin/cos LUT,
no multiplier for the rotation itself); a quadrant pre-rotation brings the
residual angle into CORDIC's natural [0,90°] range. Vertex pre-scale uses
shift-add chains for fixed `R*K`, `R*sin60*K`, `R*cos60*K` — no multipliers.
Per-pixel rasterisation is a 3-edge DDA (edge-walker): three 29-bit
accumulators advance by `dy_i` per pixel and `-dx_i` per line, so there are
no per-pixel multiplies. The two triangles are produced by an "offset trick":
triangle 1 is triangle 0 translated by 256 internal px, which collapses to a
shift+add and adds only ~840 LC. Fits 7576 / 7680 LC (98%) on the HX4K at
74.25 MHz with Fmax 84 MHz.

## Controls
- **K1 Outline Hue** — outline colour (0..360°)
- **K2 Fill Hue** — fill colour (0..360°)
- **K3 BG Hue** — background colour (0..360°)
- **K4 Size** — triangle radius
- **K5 Density** — wired, currently deferred (single-triangle gate)
- **K6 Rot Axis** — X / Y / Z / Random (only Z currently implemented)
- **T7 Outline Color** — Same / Multi (+90° hue shift on triangle 1 outline)
- **T8 Fill** — Edge-Only / Filled
- **T9 Outline Style** — Solid / Invert
- **T10 Size Variation** — Same / Varied (4-step ±R/8 cycle)
- **T11 Direction** — Fall / Rise
- **Slider Video Mix** — crossfade fill colour with incoming video

## Presets
None defined in the toml.
