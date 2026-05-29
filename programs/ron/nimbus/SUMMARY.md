# Nimbus — Flowfazer-inspired generative plasma. Four pattern modes, 256-entry programmable palette, palette cycling drives all motion

## What it does
A homage to Jordan Mechner's 1989 Flowfazer: a slowly-evolving scalar plasma field is quantized to an 8-bit palette index, and most of the visible motion comes from cycling the palette read offset per-frame rather than recomputing geometry. Four pattern modes cover H+V sin sum (classic flow), diagonal interference (rhombic grid), Manhattan radial rings (pulsing concentric), and twin-source moiré (interfering rings). The 256-entry palette has four curated banks (Aurora, Magma, Moiré, Mono), and Modulate selects whether knob inputs drive phase or palette.

## How it works
9-clock pipeline (1 timing_gen + 1 counters + 6 pipeline + 1 BRAM read). The scalar field `φ(x, y, t)` is built as the sum of three sin terms via a 1024-entry combinational sin LUT. Knob-coordinate interactions are barrel-shifts only — no multipliers anywhere in the plasma core (HX4K has no DSP blocks). The 8-bit quantized index reads three programmable BRAMs (one each for Y, U, V) holding 4 banks × 256 entries of cubic-smoothstep-symmetric palette values, with a per-frame DDS rotating the read offset to animate the colors. Source toggle can swap incoming video luma in as the index for video-reactive playback.

## Controls
- **K1 Freq A** — shift amount on term-A coordinate (8 steps)
- **K2 Freq B** — shift amount on term-B coordinate (8 steps)
- **K3 Warp** — shift amount on term-C / twin-source origin offset (8 steps)
- **K4 Drift** — palette-rotation DDS step per frame (±100%)
- **K5 Palette** — bank select (top 2b) + hue offset (lower 8b)
- **K6 Scale** — pre-shift on sx, sy (overall density; 4 steps)
- **T7 Mode Lo** — pattern-mode bit 0
- **T8 Mode Hi** — pattern-mode bit 1
- **T9 Source** — Synth / Video
- **T10 Modulate** — Phase / Palette
- **T11 Invert** — invert palette index
- **Slider Speed** — master t-increment per frame

## Presets
- Aurora Drift
- Magma Rings
- Moire Cathode
- Mono Mechner
- Video Reactive
