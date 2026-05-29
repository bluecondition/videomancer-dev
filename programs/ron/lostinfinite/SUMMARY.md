# Lost Infinite — Single-slice raymarch of the LostInfinite shader: Householder-reflected lattice shell with phase-cycled color

## What it does
A literal-as-possible FPGA transcription of the LZX "LostInfinite" GLSL raymarch shader. Per-pixel rays are reflected through a slowly-wobbling Householder axis, folded into a periodic angular wedge, tiled in XY, then colored by `exp(sin())`-phase-cycled hues through a `1/v` brightness LUT — producing an endless crystalline tunnel / lattice shell that drifts as the axis vector rotates. Five presets cover Shader Default, Slow Drift, Hyper Vortex, Deep Space, and a motion-frozen Frozen Shell.

## How it works
Q5.10 signed 16-bit fixed-point, 15-clock pipeline. The original 80-iteration raymarch loop becomes N=1 (one slice of the scene at a fixed sample depth). To fit HX4K (7680 LC, no DSP) at one pixel/clock: `length()` becomes an octagonal pseudo-metric (`max + mid/2 + min/4`); `atan2` becomes octant decoding plus a linear `atan(t) ≈ t·π/4` approximation; the small post-mod angle (in ±0.2 rad) lets `cos ≈ 1` and `sin ≈ ang` eliminate the sqrt, divider, and trig LUTs. The Householder reflection step preserves 6 real multiplies bit-for-bit, and `exp(sin())/v` is replaced by `col_yuv × brightness_lut(v)`. Color cycles per frame from three phase-offset sin lookups.

## Controls
- **K1 Depth** — sample depth t (Near / Mid / Far / Deepest)
- **K2 Offset** — P.z offset (0..15; shader literal is 14)
- **K3 Wobble** — Householder axis-vector rotation rate
- **K4 Cycle** — `exp(sin())` color cycle rate
- **K5 Glow** — brightness LUT sharpness near the shell (Soft / Medium / Sharp / Razor)
- **T11 Bypass** — pass input through (other switches reserved/unused)
- **Slider Brightness** — master output gain

## Presets
- Shader Default
- Slow Drift
- Hyper Vortex
- Deep Space
- Frozen Shell
