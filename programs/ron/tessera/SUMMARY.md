# Tessera — tumbling triangles raining down (sprite engine)

**Status: v1.0.0 FINAL except presets (signed off by Ron 2026-10-11,
HW-validated: "It looks really good").** v1.0.0 = the v17 engine (TOML version
bump only; bitstreams identical). **Build seeds:** router1
(`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron tessera`); seed 1 on
all 6 configs. Only v1.0.4 needed seed 2 on HD Analog: there, seed 1
livelocked router1. v1.0.5 fixed that.

## v1.0.5 — no duplicate-input cells: router1 livelock fix (2026-10-18)
v1.0.4's stock seed hunt hit real router1 rip-up livelocks: SD Standalone
on seeds 1, 2 and 7, SD HDMI on seed 1, SD Dual on seed 7. The dev HD Analog
build livelocked on seed 1 too.
- **Cause.** dupin found two LUTs with one net on two inputs. Both come from
  the sin/cos depth-layer scaling (`p_lut_reg`): x1 = x/2 + x/2 and
  1/2 = x/4 + x/4 are self-adds. The sign-extended tops of the other two
  factors share nets as well. A net on two pins of one LC is the hypnos
  router1 hang.
- **Fix (bit-exact).** z = x for the near pair of layers, x/2 for the far pair.
  - Even layers (x1, 1/2) are z/2 + z/2, which is z with its LSB cleared:
    no adder.
  - Odd layers (3/4, 3/8) are z/2 + z/4. z is in the LUT's 10-bit range, so
    only bits 8..1 + 9..2 are added (a 9-bit sum, carry included). The two
    top bits are z's sign, so no adder position sees one net twice.
  - Checked exhaustively in Python: 1024 LUT values × 4 layers, 0 differences.
- **dupin:** 2 duplicate-input LUTs → **0 cells** (no LUT or CARRY) on every
  stock netlist (all 8 configs) and on the dev HD Analog / HDMI / Dual netlists.
- **Stock SD Standalone packed LC:** 7479 → **7459** (97.1%), 25 EBR.
- **Stock router1 builds, all completed:**
  - SD Standalone seeds 1, 2 and 7, each routed in 55–58k iterations.
  - SD HDMI seed 1 and SD Dual seed 7.
  - Seed 7 now builds all 8 configs: LC 6896–7459.
  - The stock-core HD Fmax (51–57 MHz) is set by the core's Sync Out
    generator, as before.
- **Bit-exact proof:** the v1.0.4 bench, run against v1.0.3. The same six runs
  (R1 1080p, R2 720p, R3 480i, R4 1080i, R5 576p, S1 short lines), 43 M clocks
  in all, gave **0 output mismatches**, with every edge-RAM write and push
  matching. Layers mode exercises all four layer factors.
- **Dev build** (router1), all 6 configs on seed 1:

  | config | seed | Fmax (MHz) | LC / 7680 | EBR |
  |---|---|---|---|---|
  | HD Analog | 1 | 83.63 | 6882 | 25 |
  | SD Analog | 1 | 85.34 | 6893 | 25 |
  | HD HDMI | 1 | 79.63 | 6864 | 25 |
  | SD HDMI | 1 | 86.63 | 6888 | 25 |
  | HD Dual | 1 | 84.01 | 6889 | 25 |
  | SD Dual | 1 | 84.93 | 6886 | 25 |

## v1.0.4 — fit for the stock SDK SD Standalone (2026-10-18)
After v1.0.3, the stock LZX SDK's SD Standalone config packed to 7767 / 7680 LC
and could not place on any seed. v1.0.4 removes 288 LC with bit-exact
restructuring only. The output is identical at every resolution.
- **One size shift-add set (−92 LC).** v1.0.3 derived R / third / half twice:
  once from the y height and once from the x extent (14 adders).
  - Now one set reads each size as it leaves the per-frame multiplier. It is
    latched into the y set at step 8 and the x set at step 9.
  - third and half share their 1/16 + 1/64 term.
  - The recycle range reuses the latched third (`mv + H + third`).
  - R / third / half now settle one clock earlier, about 15 clocks before
    geometry starts.
- **One outline-threshold unit (−172 LC).** The three per-edge thresholds
  (Manhattan length × 2) were three parallel units in G_STORE. Now one unit
  computes them in turn, on the first wait cycle of products 12 / 14 / 16. At
  that point the multiplier's B operand holds dx_e and its registered copy holds
  dy_e, so the unit needs no input mux. This also removes v17's old HD critical
  path (the G_STORE threshold calc).
- **Threshold LSB left unwritten (−24 LC, −1 EBR).** The thresholds' ×2 LSB
  keeps its '0' init, so yosys still sees a constant lane. Writing that 0 from
  the new unit had cost the static record RAM an 11th EBR. EBR is back to 25.
- **Stock-SDK packed LC:** SD Standalone 7767 → **7479** (97.4%), HD Dual
  7430 → 7145. EBR is unchanged (25 / 26). `emulate_transparency` = 0 on every
  program RAM; HD Dual's one hit is the core's dual sync delay.
- **Bit-exact proof:** GHDL side-by-side against the committed v1.0.3, on the
  same stimulus. Every clock the bench compares data_out and every sprite-store
  write (edge-RAM port, geometry push). Knobs and switches are re-rolled at
  every field start and again mid-picture. Real line and blanking lengths.
  Result: 0 mismatches:

  | run | timing ID | raster | fields | clocks | mismatches |
  |---|---|---|---|---|---|
  | R1 | 13 (1080p) | 1920×1080 in 2200×1125 | 4 | 9.90 M | 0 |
  | R2 | 14 (720p) | 1280×720 in 1650×750 | 6 | 7.43 M | 0 |
  | R3 | 0 (480i) | 720×240 in 858×262 | 30 | 6.74 M | 0 |
  | R4 | 10 (1080i), 64 triangles at start | 1920×540 in 2200×562 | 6 | 7.42 M | 0 |
  | R5 | 12 (576p), P12 max at start | 720×576 in 864×625 | 8 | 4.32 M | 0 |
  | S1 | 3 (1080p), short lines (fill overruns) | 480×270 in 600×300 | 40 | 7.20 M | 0 |

  That is 43 M clocks in total, with every edge-RAM write and push matching.
  The output is never blank: each run draws 21k–65k luma transitions in total, after an
  empty field 0. A +1 threshold perturbation, as a negative control, gave 46
  mismatches and failed every push. Bench:
  `.scaling_work/tessera_fit/equiv/`.

- **Dev build** (router1, `NEXTPNR_ROUTER="--router router1" ./build_programs.sh
  ron tessera`). On HD Analog, seed 1 livelocks router1: 2891 arcs stayed
  stuck for 20 min in a re-check. The builder's retry loop moved on and built
  seed 2. The netlist has no `CARRY I0==I1` cells (dupin). Its only two
  duplicate-input LUTs are the old ones in the sin/cos layer scaling, which
  v1.0.3 also had.

  | config | seed | Fmax (MHz) | LC / 7680 | EBR |
  |---|---|---|---|---|
  | HD Analog | 2 | 83.58 | 6916 | 25 |
  | SD Analog | 1 | 83.19 | 6916 | 25 |
  | HD HDMI | 1 | 79.40 | 6922 | 25 |
  | SD HDMI | 1 | 82.52 | 6908 | 25 |
  | HD Dual | 1 | 83.65 | 6898 | 25 |
  | SD Dual | 1 | 78.39 | 6927 | 25 |

  Dev LC went from 7193–7223 to 6898–6927 (90.2% max). Not flashed. The output
  is bit-exact to the HW-validated v1.0.3.

## v1.0.3 — resolution scaling (2026-10-17)
Triangle sizes were `measured_v × factor` in FIELD lines, and the same size
was used for the x extent in pixels, so interlaced modes drew them ~2× tall
(480i 2.2:1, 1080i 2:1) and SD's non-square pixels were ignored. The x extent
now has its own size, from a per-mode "screen height in pixel widths":
`measured_v` on progressive HD (1080p/720p: same as before), `2 × measured_v`
at 1080i, 540 on every SD mode (720 px = a 4:3 width). It runs through the
per-frame size multiplier right behind the y size (products R·sin, half·cos,
third·sin and the flutter use it). Triangles are now equilateral on a 4:3 SD
screen and at 1080i; 1080p is bit-identical (sim diff 0 px) and 720p is
unchanged.

The "SD progressive" flag (`measured_v` in 400..600) also caught 1080i (540
field lines) and missed 480i/576i; it is now the timing ID, and selects the
0.25×–1.1× scale table on **every** SD mode. Correct-width triangles cost
480i the same line time as 576p always did (Monte Carlo, busiest line p99.7,
worst count: 480i at 1.1× ≈ 576p-old at 1.1×, 55–100%; at 1.5× it would be
~130%, i.e. overruns), so at P12 above ~1.1× SD triangles stop growing while
HD reaches 1.5× (the SD 4:3 sheet shows them ~73% of the HD size at P12
100%). Outline thickness stays at pixel/line scale.

Build (router1, all 6 seed 1): HD Analog 86.74, SD Analog 85.23, HD HDMI
78.24, SD HDMI 82.10, HD Dual 88.35, SD Dual 88.63 MHz; 7193–7223 / 7680 LC
(94.0% max), 25/32 EBR.

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
ROM; ~1.0× at P12 ≈ 790), capped at the 75% giant. Every SD mode (since
v1.0.3; before, only 480p/576p) reads a 0.25×–1.1× table instead — less line
time per screen height. Both
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
  (SD 0.25×–1.1×), shown as 0–100%.

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
