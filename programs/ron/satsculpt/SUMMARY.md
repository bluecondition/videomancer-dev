# Satsculpt — Three-anchor non-linear chroma gain curve

## What it does
A colour processor that reshapes saturation. It measures the chroma magnitude of each pixel and applies a piecewise-linear gain curve defined by three anchor points (Low / Mid / High saturation), letting you boost mid-saturation colours while crushing extremes, gate out near-grey pixels, or invert the curve so only extremes survive. An optional tint can be added on the U or V axis at output.

## How it works
Processing program. Per pixel it computes `du = U-512`, `dv = V-512`, then a saturation metric (Chebyshev `max(|du|,|dv|)` or Manhattan `|du|+|dv|`), optionally biases the sat domain and inverts polarity, looks up which of the three curve segments the value falls into, then uses an `interpolator_u` block to compute the gain. The chroma vector is scaled around 512 via `proc_amp_u` blocks (one per channel) with `brightness = 512` for unity passthrough — a calibration detail noted in the source that was the key bug-fix from v0. A final mix interpolator crossfades dry/wet. Total pipeline latency 21 clocks; Y is just delay-matched and passes through unchanged.

## Controls
- **K1 Low Gain** — gain applied at saturation magnitude 0 (near-grey region).
- **K2 Mid Gain** — gain at the mid anchor (magnitude 256).
- **K3 High Gain** — gain at the high anchor (magnitude 512+).
- **K4 Pivot Bias** — shifts the sat domain so anchors land at different brightness levels.
- **K5 Threshold** — chroma-gate; below this magnitude, output is killed to neutral.
- **K6 Tint** — adds offset on the selected U or V axis.
- **T7 Sat Metric** — Max (Chebyshev) vs Sum (Manhattan) for the magnitude measure.
- **T8 Polarity** — Normal vs Inverted curve.
- **T9 Tint Axis** — V (default) vs U for the tint offset.
- **T10 Channel Lock** — Linked (both channels) vs U Only (V passes through).
- **T11 Bypass** — pass input video straight through.
- **S12 Mix** — dry/wet crossfade.

## Presets
- Mid Punch
- Edge Bloom
- Cinematic Mute
- Chroma Gate
- Warm Mids
- Inverse Punch
