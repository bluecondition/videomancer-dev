# Tessera — tumbling triangles raining down (sprite engine)

## What it does
1–20 independent equilateral-triangle **outlines** rain down the screen. Each
has a **random in-plane orientation** (so every triangle looks unique), a random
screen-x, **tumbles end-over-end at a uniform speed**, and carries a slow
**constant in-plane spin** as it falls, recycling from the top once it exits the
bottom. Outline-only (optional two-sided video fill on T11).

Vertical placement uses a **golden-ratio (low-discrepancy) sequence** rather than
random offsets, so the triangles stay evenly spread over the screen at any count
without clumping or visible banding (random offsets clump; the golden sequence
does not — and it's still irregular, not a grid). The recycle range is just
`measured_v + 2R`, so a triangle re-enters the top exactly as it exits the bottom
— no blank gap.

**One fader (P12) sets both count and size:** fully down = a single large
triangle (~75% of screen height); turning it up raises the count to 20 while the
triangles shrink (`height = min(0.75, 1.5/count) · screenH`). Because per-line
fill cost ≈ `count · height`, this stays flat (~1.5·H) for count ≥ 2, which is
exactly what keeps the time-shared rasteriser inside its per-line budget at every
setting — big-and-few or small-and-many cost the same.

## How it works (v8 — BRAM sprite store)
Many independent triangles don't fit as parallel per-pixel rasterisers on the
HX4K, so one rasteriser is **time-shared** through a line buffer:

- **Geometry (vblank):** per triangle, rotate the base equilateral verts by a
  random angle φ — the per-triangle hash plus a **shared spin accumulator** that
  advances each frame (slow in-plane spin) — then foreshorten y by a
  **triangle-wave** factor (linear → constant apparent tumble speed; plain cos
  would dwell face-on and rush through edge-on). Produces three *general* edges
  (a=dy, b=−dx), the edge value at the bbox corner, the bbox (xmax clamped to
  screen), and per-edge thresholds. All multiplies are sequenced through **one
  shared multiplier**; long chains (centre-y wrap, vertex/bbox) are pipelined.
- **Sprite store (BRAM):** per-triangle records are packed into a **block RAM**
  (one slot per triangle). Geometry writes every slot in vblank; the fill engine
  reads + writes them back sequentially. Moving the records out of the register
  fabric (an earlier recirculating shift-register FIFO) is what freed the LCs to
  reach 20 triangles — the device sits at ~89% LC / 23 of 32 BRAMs. The store is
  given a full BRAM depth (256) with a **single muxed write port**, or yosys
  leaves it in LUTs/FFs (a 20-deep or dual-write memory won't infer as SB_RAM40).
- **Fill (per line):** for slot 0..count−1, read the record (1-cycle BRAM
  latency), and if the line is in its y-band, walk its x bounding span with one
  shared edge tester, marking outline pixels into the fill half of a ping-pong
  line buffer; step the edge row and write the record back. Drawing is bounded to
  each span, so a flat triangle draws a short segment, never a full-screen line.
- **Scanout:** read the display half of the line buffer (one-line latency; line
  0 blanked), colour mux to outline/background.

Fits ~7000 / 7680 LC (91%), RAMs 23/32. Five of six variants clear the 74.25 MHz
HD target (76–84 MHz); the **HD-HDMI** variant lands ~72 MHz (≈3% under) — the
chip is at capacity with the colour palette added. Shipped at full clock by
choice; HD-HDMI output may be marginal (HD-analog, HD-dual and all SD are fine).

## Two-sided video fill (T11)
With **T11 = Video On**, each triangle is filled like a two-sided card: the face
toward the viewer shows the **incoming video**, and once it tumbles past edge-on
the **back** face shows a **solid colour** (K2 Fill Hue). The side is chosen from
the sign of the tumble factor, so it flips every half-tumble. The line buffer
carries a 2-bit code per pixel (bg / outline / solid-fill / video-fill); the
incoming video is delayed to align with the scanout. T11 Off = outline-only.

## Controls
Each colour knob sweeps a **white → rainbow → black** palette (white, R, O, Y, G,
B, V, black — the rainbow entries are the phosphor program's hardware-confirmed
BT.601 values, U/V swapped for rev_b):
- **K1 Outline Colour** — outline colour
- **K2 Fill Colour** — solid colour on the back face when Video is on
- **K3 BG Colour** — background colour
- **K4 Fall Speed** — downward speed (shared)
- **K5 Spin** — in-plane spin rate (0 = none .. fast)
- **K6 Tumble Speed** — end-over-end rate (shared)
- **T11 Video** — Off (outline only) / On (front=video, back=solid)
- **Fader P12 Count** — sets count + size together: down = 1 large triangle
  (~75% of screen height), up = 20 small ones.

## Presets
None defined in the toml.
