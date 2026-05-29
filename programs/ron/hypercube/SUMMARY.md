# Hypercube — 4D tesseract wireframe carving screen into 8 cells, each applying its own chroma rotation

## What it does
A rotating 4D tesseract (16 vertices, 32 edges) is projected to the screen as a wireframe, while the same 4D math partitions the picture into 8 Voronoi cells (one per ±X/±Y/±Z/±W face direction). Each cell applies its own chroma rotation to the live video — swap UV, negate U, or negate V — so the incoming image is sliced into eight color-shifted regions whose borders rotate in lockstep with the wireframe. Cell Tint dry/wets the chroma rotation, Edge Color picks white vs cell-bumped luma, and a Reverse switch flips the spin direction.

## How it works
Per-frame vertex / projection / edge-slope math runs during vblank on a state machine sharing one pipelined `multiplier_s` instance (10-cycle latency, 1/cycle throughput) — ~672 multiply issues per frame, plenty of headroom in HD's ~99k cycle vblank. Six rotation planes (XY, XZ, YZ, XW, YW, ZW) accumulate each frame, W-planes scaled by the 4D Twist knob; W→2D uses a Schlegel projection (`scale = 1/(W_PERSP - w)`). Per-pixel pipeline is 17 stages: parallel banks of 8-cell Manhattan distance + 32-edge sub/abs/compare, OR-reduce trees for edge hits, min trees for cell_id, then chroma rotation, cell-tint crossfade, edge composite, and a master brightness `interpolator_u`. Sized ~3500 LC on HX4K — no DSP used.

## Controls
- **K1 Speed** — base rotation rate
- **K2 4D Twist** — scales XW/YW/ZW planes vs XY/XZ/YZ
- **K3 Size** — projection scale (1×, 1.5×, 2×, 3×)
- **K4 Cell Tint** — dry/wet crossfade between input chroma and cell-rotated chroma
- **K5 Edge Width** — wireframe thickness (1..16 px stepped)
- **K6 Edge Bright** — edge luma intensity
- **T7 Wireframe** — show / hide edges
- **T8 Background** — Black / Video
- **T9 Cell Fill** — enable per-cell chroma rotation
- **T10 Edge Color** — Cell-tinted / White edges
- **T11 Direction** — Forward / Reverse spin
- **Slider Brightness** — master Y gain

## Presets
- Default
- Pure Wireframe
- Color Cells
- Slow 4D Drift
