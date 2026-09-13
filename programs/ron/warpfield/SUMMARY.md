# Warpfield — radial starfield: stars fly at you (or away)

## What it does
Starfield's radial sibling. Instead of 8 sideways parallax layers, 6 zoom
*shells* stream stars out of (or into) a vanishing point: each star spawns
far — small and dim — grows through the four glow-kernel levels as its
shell approaches, and sweeps radially outward with speed proportional to
its distance from centre, the classic warp/hyperspace field. Star shapes,
twinkle, the four shimmer palettes, B&W mode, the Deep Space nebula
background and Luma Mode (video brightness → star density and size) are
inherited from starfield. The vanishing point can wander in a slow
Lissajous (K6 Drift), scaled to the measured picture so it never leaves
the frame. Direction reverses the flow: stars recede and vanish into the
deep.

## How it works
Per shell, scaled coordinates S = BIAS + (pixel − centre)·k are pure
accumulators (+k per pixel, +k per active line); k is a 4.12 zoom step
swept one octave (4096 → 2048) per cycle from a 256-entry exp2 EBR table
(linearly interpolated via the shared multiplier — raw 256-step zoom
would judder ~2 px radially at slow warp),
so cells are 16–32 px on screen and the kernel magnifies 1×–2× for free.
Shell phases sit ⅙ apart on a global 24-bit phase accumulator; bits 19:16
of the per-shell sum form a re-seed epoch XOR-folded into the hash seed,
so a wrapping shell respawns a fresh population exactly while its fade
envelope passes through zero. Depth drives the glow level (with per-star
dither between adjacent levels; a brightness ramp per level is baked into
the kernel ROM — all 6 ROMs are identical since depth rotates).
Twinkle × shell-fade form the existing 8×5 envelope multiply; no new
per-pixel multipliers. All frame set-up (quadratic Warp speed curve,
Lissajous parabola-sine, amplitude and half-dimension scaling, centre×k
line origins) runs through ONE shared 13×13 multiplier in a ~256-step
vblank sequencer (power-of-2 slot layout). Raster width/height are
measured cascade-style (avid run / active-line count). Pipeline
LATENCY = 15.

EBR: 12 kernel + 2 shimmer + 8 nebula + 1 exp2 ≈ 23/32. Six shells
(not starfield's 8) is the HX4K fit budget: 8 shells synthesized to 115%.

## Controls
- **K1 Layers** — 1..6 active zoom shells
- **K2 Density** — star density (luma gain in Luma Mode)
- **K3 Brightness** — output brightness (linear multiply)
- **K4 Palette** — Mixed / Cool / Warm / Rainbow, "+" = Lively twinkle/shimmer
- **K5 Size** — adds 0..3 glow levels (dithered, smooth growth)
- **K6 Drift** — vanishing-point Lissajous amplitude (0 = locked centre)
- **T7 Direction** — Toward / Away
- **T8 Color Mode** — Color / B&W
- **T9 Luma Mode** — video luma → star density and size
- **T10 Background** — Off / Deep Space (nebula + gradient)
- **T11 Luma Gate** — sharpen the luma-to-density mapping
- **Slider Warp** — flow speed, quadratic: ~30 s/cycle at 10 %, ~1 s at
  50 %, ~0.3 s hyperspace at full — the headline performance control

## Presets
- Cruise
- Hyperspace
- Star Drift
- Nebula Dive
- Retreat
- Tunnel Weave

## Status
Not HW-tested. Built alongside a review of starfield (2026-08-10).
