# Tessera — tumbling triangles raining down (sprite engine)

## What it does
1–10 independent equilateral-triangle **outlines** rain down the screen. Each
has a **random in-plane orientation** (so every triangle looks unique), a random
screen-x and vertical stagger, and **tumbles end-over-end at a uniform speed** as
it falls, recycling from the top once it exits the bottom. Outline-only.

## How it works (v7 — line-buffer sprite engine)
Many independent triangles don't fit as parallel per-pixel rasterisers on the
HX4K, so one rasteriser is **time-shared** through a line buffer:

- **Geometry (vblank):** per triangle, rotate the base equilateral verts by a
  random angle φ (shared sin/cos LUT) then foreshorten y by a **triangle-wave**
  factor (linear → constant apparent tumble speed; plain cos would dwell
  face-on and rush through edge-on). Produces three *general* edges (a=dy,
  b=−dx), the edge value at the bounding-box corner, the bbox, and a threshold.
  All multiplies are sequenced through **one shared multiplier**; long arithmetic
  chains (center-y wrap, vertex/bbox) are pipelined across vblank cycles.
- **Sprite FIFO:** per-triangle records live in a **recirculating shift
  register** — the fill engine always reads the head, so there are no
  runtime-indexed register arrays (those blew up LUT usage in an earlier try).
- **Fill (per line):** pop each record; if the line is in its y-band, walk its
  x bounding span with one shared edge tester, marking outline pixels into the
  fill half of a ping-pong line buffer; step the edge row for the next line.
  Drawing is bounded to each span, so a flat triangle draws a short segment,
  never a full-screen line.
- **Scanout:** read the display half of the line buffer (one-line latency; line
  0 blanked), colour mux to outline/background.

Fits ~6700 / 7680 LC (87%) on the HX4K, Fmax ~86 MHz at the 74.25 MHz HD target.

## Two-sided video fill (T11)
With **T11 = Video On**, each triangle is filled like a two-sided card: the face
toward the viewer shows the **incoming video**, and once it tumbles past edge-on
the **back** face shows a **solid colour** (K2 Fill Hue). The side is chosen from
the sign of the tumble factor, so it flips every half-tumble. The line buffer
carries a 2-bit code per pixel (bg / outline / solid-fill / video-fill); the
incoming video is delayed to align with the scanout. T11 Off = outline-only.

## Controls
- **K1 Outline Hue** — outline colour (0..360°)
- **K2 Fill Hue** — solid colour on the back face when Video is on
- **K3 BG Hue** — background colour (0..360°)
- **K4 Fall Speed** — downward speed (shared)
- **K5 Count** — number of triangles (1..12)
- **K6 Tumble Speed** — end-over-end rate (shared)
- **T11 Video** — Off (outline only) / On (front=video, back=solid)
- **Fader Size** — triangle size, 4 steps (~6%–14% of screen height). At the
  largest step the live count auto-caps (to ~10) so the fill engine always
  finishes a scanline — bigger triangles, fewer of them, no glitching.

## Presets
None defined in the toml.
