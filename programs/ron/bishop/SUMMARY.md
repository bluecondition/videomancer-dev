# Bishop — Early-80s wireframe vector head

## What it does
Renders an animated wireframe head reminiscent of early-80s vector graphics demos. The face can blink, talk (mouth opens/closes), tilt its brow, and pseudo-rotate in yaw/pitch. Output is a single-colour line drawing over either black or the incoming video.

## How it works
Pure synthesis program built from an SVG-derived mesh (`bishop_mesh_pkg.vhd`, extracted via `extract_mesh.py`). Each edge gets its own per-process state machine with a per-scanline DDA renderer; edge constants are baked into the synthesised logic rather than fetched from a shared ROM, which is cheaper at this edge count. Edges belong to one of four animation groups (static, brow, eye, mouth) sharing common Y-offsets. Hits are OR-reduced in two stages (N→N/8→1) and the result is multiplied by a fixed 8-entry YUV palette scaled by brightness. 8-stage pipeline.

## Controls
- K1 Size — overall head scale
- K2 Yaw — left/right pseudo-rotation
- K3 Pitch — up/down pseudo-rotation
- K4 Mouth — mouth open amount (Speak switch animates this)
- K5 Brow — brow tilt offset
- K6 Color — 8-entry palette (Green, Amber, Cyan, Red, White, Magenta, Yellow, Blue)
- T7 Speak — animate mouth on/off
- T8 Blink — auto blink on/off
- T9 Mesh — Sparse vs Dense edge set
- T10 Glow — thicker lines (2px vs 4px)
- T11 Background — Black or incoming Video behind the wireframe
- Slider Bright — line brightness scale

## Presets
- Bishop
- Talking Head
- Stern Glance
- Overlay Ghost
