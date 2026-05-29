# Solitude — glitch cascade on teal

## What it does
A ragged central mass of straight vertical pixel-sort stripes in jewel tones
(magenta / red / blue / green) floats on a flat muted teal background. The
mass's left/right outline zigzags as a lightning-bolt cascade down the screen;
its top is a jagged city-skyline silhouette, its bottom drips long columns
down into the teal. Thin white branching lightning caps the top edge, densest
at the cliff line and fading downward. A Cool palette and a black-background
swap let the same shape read as cool corruption or a void.

## How it works
Pure synthesis (no incoming video used). At each pixel the design tests
whether `x` falls inside a per-row mass window whose centre is shifted by a
triangle wave of `pixel_y` plus per-row hashed noise (the cascade). Slanted
30° y-rotated bands (`y - x/2`) separate four colour passes that each pick
their stripe colour from a per-column hash; stripe width comes from K5. The
top edge is shaped by a step-noise jaggy and the bottom by a per-column drip
length. Lightning is two combined hash thresholds densest near the top.
Pipeline depth is 12 stages with a dedicated stage to register the cascade
multiplier so 74.25 MHz HD timing closes.

## Controls
- **K1 Edge** — top-edge jag amplitude
- **K2 Drip** — drip column frequency / length at the bottom
- **K3 Lit Dns** — lightning density near the top edge
- **K4 Cascade** — horizontal zigzag amplitude of the outline
- **K5 Stripe** — stripe width (top 2 bits select 1..8 px)
- **K6 Lit Depth** — how far down lightning extends
- **T7 Palette** — Jewel / Cool
- **T8 BG** — Teal / Black background
- **T9 Cascade** — Off / On (enable horizontal zigzag of the mass)
- **T10 Lightning** — Off / On (top-edge lightning veins)
- **T11 Bypass** — Off / On (pass input video through)
- **Slider Bright** — overall brightness

## Presets
- Default
- Tight Mass
- Dissolving
- Cool Corruption
- Black Void
