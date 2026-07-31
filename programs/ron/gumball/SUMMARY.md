# Gumball — video as a wall of glossy spheres (v0.1)

The incoming video is re-rendered as a hex-packed field of lit 3D balls —
a pixelation where every "pixel" is a shaded sphere with a specular
highlight. Rows are offset by half a cell; oversized spheres SQUASH flat
against their neighbours along the hex Voronoi boundaries (round balls
with pressed contact facets, up to a full-coverage domed mosaic).

## Controls

K1 Ball Size (smooth 12..128 px) · K2 Light Angle (spins the highlight
around every ball, 360°) · K3 Specular (to deliberate blowout) · K4 Shine
(broad satin → pinpoint gloss) · K5 Squash (separated beads → touching →
pressed-flat domed mosaic) · K6 Candy (chroma gain 0.75–2.7×) ·
S7 Backdrop Black/Video (dimmed live video between balls) · S8 Luma Size
(bright cells grow ±r/2, per-cell rsq stored in the buffer = free per
pixel) · S9 Packing Hex/Square · S10 Chrome (mono ball-bearings) ·
S11 Bypass · **P12 Relief** = flat unlit mosaic → deep-shadow wet gloss
(shading span + specular master ride together; 0 = flat hex pixelation).

## Architecture

Streaming pipeline c1..c16 (LATENCY 15), no divide / no wide multiply on
the pixel path.

- **Hex Voronoi by nearest-of-2**: two half-offset X lattices (A = even
  rows, B = odd rows) run in parallel; per pixel compare d² to the own-row
  and adjacent-row nearest centres — for offset rows that IS the exact hex
  Voronoi. Winner supplies colour buffer word + local (dx, dy). Row
  height H = 7W/8. Top band masks the (nonexistent) row above.
- **Fake-Phong from ONE field**: e² = (dx−ox)² + (dy−oy)² (light offset
  ox,oy = 5r/8·(cos,sin) per frame) drives BOTH diffuse dome and specular
  hotspot. Normalisation without pixel divide: per frame find shift sl
  (priority encoder) and gain k = span·128 / ((range²≪sl)≫8) (serial
  divide), pixel path = 2-stage barrel shift + 8×8 mult. Chroma factor =
  diffuse − spec/2 (one term darkens toward rim AND desaturates the
  hotspot).
- **Cell colours**: point-sampled per cell row into two parity ping-pong
  buffers (46-bit word: Y/U/V/rsq, canonical 1W1R). Row R is sampled on
  the CENTRE line of row R−1 — its spheres first win the Voronoi test
  ~0.16·H later, so the swap is never visible. Row 0 samples on the first
  active line. Sat boost (K6) and luma→radius (S8) applied at WRITE time.
- **Per-line squares incrementally** (the timing fix): dy²/(dy−oy)² for
  both row candidates advance by (d+1)² = d² + 2d + 1 at each h edge —
  adds only. Band-wrap / mid-band-jump reset constants (H/2)², H²,
  (H/2±oy)², (H+oy)² precomputed per frame.
- **Frame FSM**: one shared 12×12 multiplier + 18-bit serial divider,
  ~85 cycles in the vsync cone (knob shaping, r, light vector, ranges,
  2 divides, 5 wrap squares). Sine from SDK sin_cos_full_lut_10x10.

## Timing (routed, all seed 1 unless noted)

| Config | Fmax | Fmin |
|---|---|---|
| HD Analog | 79.79 | 74.25 |
| SD Analog | 77.69 | 27 |
| HD HDMI | 96.96 | 74.25 |
| SD HDMI | 92.23 | 27 |
| HD Dual | 87.81 (seed 2) | 74.25 |
| SD Dual | 91.47 | 27 |

~7,090 LCs (92%), 12 BRAMs. Packaged: `out/rev_b/ron/gumball.vmprog`.

## Lessons

- **Line-rate math is still STA-timed at pixel clock**: v0.1's first build
  shared one 9×9 squarer across 4 per-line ops behind a state mux —
  routed critical path (state → operand mux → multiply → reg) failed HD
  Analog at 69.24 MHz *while the build script printed "✓ 75.11 MHz"*
  (best-effort accept on the last retry seed; the displayed Fmax was the
  pre-route estimate). Re-running nextpnr at the accepted seed and reading
  the Warning line caught it. Fix: incremental squares, no multiplier at
  all.
- Nearest-of-2 with two phase-shifted X lattices renders exact hex
  Voronoi with just two 7-bit squarers + one 16-bit compare per pixel.

## Status

Built + packaged 2026-07-14. Sim-verified (dec 4): default kiss-point
spheres, max-squash hex mosaic facets, separated beads with video
backdrop + Luma Size + rotated light. **Not HW-tested.**

Sims: `sim_001_default.png` (+`_input`), `sim_002_squashed.png`,
`sim_003_beads.png`.
