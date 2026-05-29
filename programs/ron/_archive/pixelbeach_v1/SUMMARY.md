# Pixel Beach — Generative 2D pixelated beach scene

## What it does
A purely synthesised seascape: six parallax wave layers crest and roll over each other, with wavy foam bands along their tops, a sky gradient above and warm sand at the bottom-left. The waves drift horizontally at different speeds (back layers slowest), the tide ebbs and flows over a long cycle, and the palette can be shifted toward sunset, night, or deep-ocean moods.

## How it works
Synthesis using a 64-entry signed sine LUT (no multipliers needed for waveform; small 12-bit adders only). Each of six ocean layers has a fixed base Y, amplitude and horizontal scroll shift; per-pixel wave-top height is `base_y + amp * sin(scroll + x>>wshift)` plus an optional second-harmonic term (Choppy mode). A separate sine drives the global tide phase, bobbing all wave baselines together. Foam is rendered as a thin band above each wave crest using a second sine waveform. Layer colours come from a 6-entry YUV palette table shifted by Color/Sunset/Night/Deep Ocean toggles. 7-stage pipeline.

## Controls
- **K1 Wave Spd** — horizontal scroll rate of all layers (parallaxed).
- **K2 Wave Hgt** — amplitude of the per-layer wave displacement.
- **K3 Foam** — thickness/intensity of the white foam band along wave crests.
- **K4 Tide** — depth of the slow ebb-and-flow tide modulation.
- **K5 Color** — global hue shift across the ocean palette.
- **K6 Horizon** — Y position of the horizon line between sky and ocean.
- **T7 Wave Shape** — Smooth (single sine) vs Choppy (with harmonic).
- **T8 Sunset** — warm orange/pink palette tint.
- **T9 Night** — dark blue palette tint.
- **T10 Deep Ocean** — shift palette toward darker, more saturated blues.
- **T11 Bypass** — pass input video straight through.
- **S12 Bright** — overall luma scale.

## Presets
- *(none defined)*
