# Drape — Freeze a scanline and drape it downward

## What it does
Processing effect that picks one row of the incoming video, freezes it, and uses that captured line as the source for every row below. The frozen line can be stretched outward from centre (so columns fan out like fabric falling), curved or leaned sideways, blurred, and fades to black further down. Above the split, video passes through untouched.

## How it works
Captures the split row into three inferred 2048×10 single-port BRAMs (one per Y/U/V). Below the split, each row replays the captured line through an "enlarge-from-centre" transform: `source_x = centre + (display_x - centre) × (1 - ε)`, where ε ramps with depth and the Spread knob. Two curve modes — Straight uses an interpolated 64-entry LUT for `eps/(1+eps/256)` (radial lines from a focal point), Curved uses ε directly (hyperbolic fan). Optional 4-tap horizontal box blur. Fade slope from a 32-entry geometric LUT anchored at the Stretch row. 11-stage pipeline; final stages split the multiplies to keep timing.

## Controls
- K1 Split — capture row (0..max_y)
- K2 Stretch — Y where the stretch effect begins (always ≥8 rows below Split)
- K3 Fade — ramp slope to black
- K4 Twist — horizontal lean per row (centred at 512)
- K5 Soft W — Stretch-onset ramp width (used with Soft switch)
- K6 Mix — wet/dry between original and draped output
- T7 Blur — 4-tap horizontal box blur on captured line
- T8 Curve — Straight (LUT-based radial) vs Curved (hyperbolic)
- T9 Soft — gradual Stretch onset over Soft W rows
- T10 Curve+ — Curved-mode style: Hard (fast saturation) vs Soft (smoother)
- T11 Bypass — passthrough
- Slider P12 Spread — horizontal outward stretch amount (headline performance control)

## Presets
- Default
- Long Drape
- Short Tail
- Top Clip
