# Starfield — shader-inspired glowing parallax starfield

## What it does
A shader-style starfield with up to 8 parallax depth layers, each scrolling
sideways at a different speed. Stars are placed on a 16x16 cell grid with an
octagonal glow falloff, drawn in per-star colours from one of four palettes
(Mixed / Cool / Warm / Rainbow), with optional B&W rendering and a Deep Blue
background. A Luma Mode maps the incoming video's brightness to local star
density so video content "lights up" star fields, and a Luma Gate sharpens
that mapping.

## How it works
Pure synthesis with optional video reactivity. Per-cell hashes generate
deterministic star positions and properties; an XOR + multiply + xor-shift
hash plus per-layer seed gives uncorrelated patterns across the 8 layers.
Distance is linear Manhattan-style with shift-only falloff — no per-layer
multipliers, no squaring — so the design stays within iCE40 HX4K resources.
24-bit scroll accumulators per layer give smooth subpixel motion; depth fade
shifts brightness per layer. Pipeline LATENCY = 13 stages, with the density
threshold pre-registered so the gated-luma path closes timing.

## Controls
- **K1 Layers** — 1..8 active parallax layers
- **K2 Density** — manual density (or luma gain in Luma Mode)
- **K3 Speed** — scroll speed (max ~38 px/frame)
- **K4 Palette** — Mixed / Cool / Warm / Rainbow
- **K5 Glow** — glow radius / brightness (10-bit smooth)
- **K6 Depth** — layer parallax spread, 1..8 / Max
- **T7 Direction** — Left / Right scroll
- **T8 Color Mode** — Color / B&W
- **T9 Luma Mode** — Off / On (map video luma to star density)
- **T10 Background** — Black / Deep Blue
- **T11 Luma Gate** — Off / On (sharpen luma-to-density mapping)
- **Slider Brightness** — overall output brightness

## Presets
- Shader Default
- Deep Field
- Hyperspace
- Nebula
- Crystal Night
