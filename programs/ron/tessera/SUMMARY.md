# Tessera — tumbling triangles raining down (sprite engine)

**Status: v1.0.0 FINAL except presets (signed off by Ron 2026-10-11,
HW-validated: "It looks really good").** v1.0.0 = the v17 engine (TOML version
bump only; bitstreams identical). **Build seeds:** all 6 configs seed 1, router1
(`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron tessera`).

## What it does
1–64 independent equilateral-triangle **outlines** rain down the screen. Each
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

**Count (K5) and Scale (P12), v17:** K5 sets the count, 1–64, on a quadratic
curve (`count = 1 + round(p²/63)`, p = K5[9:4]) so 1–4 big triangles get ~20% of
the travel. Each count has a natural size `0.75/√count · screenH` (1 → 75%, 4 →
37%, 16 → 19%, 64 → 9.4%) that keeps the per-line fill cost roughly flat; the
P12 fader scales it **0.25×–1.5×** on an even exponential sweep (Q10 table in the
ROM; ~1.0× at P12 ≈ 790), capped at the 75% giant. SD progressive (480p/576p)
reads a 0.25×–1.1× table instead — less line time per screen height. Both
controls have a ±6-code deadband (ADC noise at a table-row boundary would flip
the count, or every size, each frame). The two size products (base × k, then ×
screen height) run in turn through the one per-frame multiplier. Up to v16, P12
was count + size together (`0.75/√count`, no separate scale).

**Scale ceiling (Monte Carlo, random orientations/tumble, busiest line per
frame, p99.7 / max):** 1080p at 1.5× — Flat 93% / 98%, Layers 91% / 92%; 1.25× —
77–79%. 576p at 1.0× ≈ 83%, 1.25× ≈ 107% (hence its 1.1× table). A 2 px/clock
fill engine (even/odd line-buffer banks, two edge testers) would allow 2× on HD
(≤ 69%) and ~1.5× on SD — not built (needs LC freed first).

**Why 64 fits (v15):** the old `count · height = 1.5H` law assumed every
triangle could sit on one scanline. They can't — the golden y-spacing keeps only
~4–5 on any line — so the fill engine was ~5× over-provisioned. Worst line under
the √ law (conservative model, every triangle face-on at full width): 1080p 53%
of the line, 720p 51%, 480p 73%, 576p 82%, interlaced SD ~50%. Per-triangle
random values (x, angle offset, tumble phase — lowbias32, spot-checked) and the
P12 count/size table live in one 256×32 **constants ROM** (2 EBR); 64-entry
constant arrays would have been big LUT muxes. The golden step is now 633/1024
(the old 39/64 would have put exactly one triangle per 1/64 slot at count 64 —
a lattice). Geometry pass: ~111 clocks/triangle → ~7.1k clocks for 64, inside
every mode's post-vsync back porch (NTSC ~12.9k). Note: the image-tester's fake
8-line vblank is shorter than that — sims need a wide raster (720p dec 1).

**Depth layers (v16, T10 = Layers):** slot mod 4 picks a layer (0 = near).
Size and fall speed go ×1, 3/4, 1/2, 3/8 front to back, the near layer being
1.25× the P12 size (capped at the 75% giant). Each layer has its own fall
accumulator (Q4, so the far layers still creep at the slowest Fall); the far
layer is drawn first so near triangles paint over it. Size scaling is done on
sin/cos (one adder each: every factor is a sum of two shifts), so it scales
every vertex offset and the flutter at once. Flat moves all four accumulators
as one.

**Quarter-golden y-spacing (v16):** with the plain golden step, a layer's slots
(s ≡ d mod 4) step by 4φ ≈ 0.47 and spread badly (model: 576p 128% of the
line). The step is now (φ+2)/4 = 0.6545: four steps = 2.618 ≡ 0.618, so every
layer is its own golden sequence from the one accumulator (a 4-accumulator ring
did the same for ~120 cells). Flat costs a little evenness (worst line 1080p
53→61%, 576p 83→91%).

**Flutter (v16, always on):** each triangle swings side to side once per tumble
(x −= R·cos τ, layer-scaled), turning round face-on and gliding fastest
edge-on — the falling-card motion. x bounds are now clamped to [0, screen] on
both sides, since a swing can carry a triangle wholly off either edge.

**Fill overrun guard (v16):** if the next line's avid starts while a pass is
still running, the walk in progress ends and the remaining walks are skipped
(edges still step, so nothing desyncs); the next line's pass starts the moment
this one finishes. A rare overrun costs a few missing line segments, not a torn
frame. (A budget-arithmetic version cost ~195 cells; this is two flags.)
Layers worst line (conservative model): 1080p 71%, 720p 63%, 480p 82%, 576p 98%.

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
  reach 20 triangles (v11), and its 256-deep store already holds 64 (v15). The store is
  given a full BRAM depth (256) with a **single muxed write port**, or yosys
  leaves it in LUTs/FFs (a 20-deep or dual-write memory won't infer as SB_RAM40).
- **Fill (per line):** for slot 0..count−1, read the record (1-cycle BRAM
  latency), and if the line is in its y-band, walk its x bounding span with one
  shared edge tester, marking outline pixels into the fill half of a ping-pong
  line buffer; step the edge row and write the record back. Drawing is bounded to
  each span, so a flat triangle draws a short segment, never a full-screen line.
- **Scanout:** read the display half of the line buffer (one-line latency; line
  0 blanked), colour mux to outline/background.

- **Generated avid + blanking gate (v14):** the output avid is the program's
  own (parity-locked start, field-latched width — the cubist/aurora recipe), so
  the encoders' Cb/Cr pairing can't flip on a line whose source avid wanders a
  clock; the line-buffer beam counts that avid. Outside it y/u/v are 64/512/512.
  Latency 7 clocks, all modes.
- **Once-per-field animation pulse (v14):** fall/spin/tumble and the geometry
  pass fire on the first vsync edge after active video (analog vsync serrates).

**Build: router1** (`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron
tessera`). v13 under router2 left HD-HDMI at ~72 MHz; router1 alone closed it
(76 MHz), and `no_rw_check` on the three RAMs (−316 LC: yosys's read-before-
write emulation on the 233-bit record RAM) plus a split size multiply (the old
critical path) gave v14 6/6 on seed 1. **v15: all 6 configs pass on seed 1**
(router1), ~6730 / 7680 LC (88%), RAMs 25/32 — HD Analog 85.39, HD HDMI 91.19,
HD Dual 85.30, SD Analog 88.18, SD HDMI 83.17, SD Dual 83.44 MHz. (A first v15
build had the P12 deadband's add+compare inside the vsync latch enable — HD
Analog 73 MHz on seed 1; pre-registering the sums and the 1-bit "moved" flag
fixed it.)

**v16 build** (router1): ~7170 / 7680 LC (93%), RAMs 25/32 — HD Analog 85.44,
HD HDMI 82.03, HD Dual 84.52, SD Analog 90.76, SD HDMI 84.33 (all seed 1),
**SD Dual 83.93 on seed 2** (seed 1 stalled the router — timeout, not timing).
The first v16 cut sat at 7498 LC (97.6%); a standalone-yosys ablation ranked the
new parts (budget guard ~195 cells, per-layer golden ring ~120, sin/cos scale
~138) and the cheaper equivalents above took back ~340 LC. Splitting the bbox
fold from the new two-sided x clamp (G_VCOMB3/4) took HD Analog 76.6 → 85.4.
Step 4 (8-bit line buffer) will need room: split the record RAM (static vs
per-line e values) and a quarter-wave sin/cos (the SDK LUT is 6 EBR).

**v17 build** (router1, all 6 on seed 1): ~7437 / 7680 LC (**96.8%**), RAMs
25/32 — HD Analog 78.23, HD HDMI 79.20, HD Dual 82.28, SD Analog 81.37, SD HDMI
81.10, SD Dual 83.08 MHz. HD Analog's critical path is the G_STORE threshold
calc (s_dy1 → s_stage, 3.9 ns logic / 8.9 ns routing = congestion). Above the
~94% comfort line: the record-RAM split (−250..300 LC est.) is the next lever
if anything else is added or a seed goes marginal.

## Two-sided video fill (T11)
With **T11 = Video On**, each triangle is filled like a two-sided card: the face
toward the viewer shows the **incoming video**, and once it tumbles past edge-on
the **back** face shows a **solid colour** — since v17 a toned-down version of
the outline colour (K1, from the dim half of the palette), so the bright outline
still reads on a darker card back. The side is chosen from
the sign of the tumble factor, so it flips every half-tumble. The line buffer
carries a 2-bit code per pixel (bg / outline / solid-fill / video-fill); the
incoming video is delayed to align with the scanout. T11 Off = outline-only.

## Controls
Switches 7–9 are not declared (they were labelled but never wired before v14).
The colour knobs sweep a **white → rainbow → black** palette (white Y 940, R, O,
Y, G, B, V, black Y 64 — the rainbow entries are the phosphor program's
hardware-confirmed BT.601 values, U/V swapped for rev_b). The BG knob and the
back faces use a **toned-down copy** (v17: luma 3/8 of the way above black,
chroma half) — the full palette was too bright behind the triangles.
- **K1 Outline Colour** — outline colour (back faces: its toned-down version)
- **K2 Spin** — in-plane spin rate (0 = none .. fast)
- **K3 BG Colour** — background colour (toned-down palette)
- **K4 Fall Speed** — downward speed (layers: x1, 3/4, 1/2, 3/8)
- **K5 Count** — 1–64 triangles (quadratic curve), shown as 0–100%
- **K6 Tumble Speed** — end-over-end rate (shared)
- **T10 Depth** — Flat (one size, one speed) / Layers (4 depth layers; default)
- **T11 Video** — Off (outline only) / On (front = video, back = solid)
- **Fader P12 Scale** — every triangle 0.25×–1.5× of its count's natural size
  (SD progressive 0.25×–1.1×), shown as 0–100%.

## Presets
None defined in the toml.

## Community port — fit for the stock SDK (v1.0.2, 2026-10-15)
- The stock LZX SDK builds 8 configs; its SD Standalone core is ~600 LC larger
  than HD. v1.0.1 packed to 8034/7680 LC there (HD Dual 7678).
- v1.0.2 splits the 233-bit sprite-record RAM in two:
  - The fill engine only ever changes the three edge values (e0..e2, 72 bits)
    when it writes a record back. So the other 161 bits, written only by
    geometry in vblank, live in their own RAM (176-bit padded) with no write
    mux.
  - The edge RAM (80-bit padded) keeps the geometry/fill write mux, now
    72 bits wide.
  - The fill's write-back register shrinks from 233 to 72 bits.
  - EBR 15 → 16 for the record store.
- Proven bit-exact against v1.0.1: GHDL side-by-side, 30 frames, knobs and
  switches changing mid-picture, 0 mismatches in 6.05 M clocks. That run used a
  1003-clock line. On the default 403-clock testbench line, the ~5k-clock geometry
  pass overruns the 10-line top blank and races the first fill passes; there the
  old version's full stale write-back and the new partial one differ in 55
  pixels. Real video has 15k+ (SD) / 90k (HD) clocks before the first active
  line, so geometry always finishes first.
- Packed LC (stock SDK): SD Standalone 8034 → 7633, HD Dual 7678 → 7265.
- **Build seed (stock LZX flow): `[firmware] seed = 7`.** Seeds 1–6 each hung
  nextpnr's router1 in at least one config (a rip-up livelock with no end; this
  netlist sits at 99% LC on SD Standalone). Seed 7 builds all 8; own HD timing
  (core without the Sync Out generator) is 89.6/83.1/82.1 MHz Analog/HDMI/Dual.
