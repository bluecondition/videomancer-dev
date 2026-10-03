# Gumball — video as a wall of glossy spheres (v1.0)

The incoming video is re-rendered as a hex-packed field of lit 3D balls —
a pixelation where every "pixel" is a shaded sphere with a specular
highlight. Rows are offset by half a cell; oversized spheres SQUASH flat
against their neighbours along the hex Voronoi boundaries (round balls
with pressed contact facets, up to a full-coverage domed mosaic).

## Controls

K1 Ball Size (smooth 12..128 px) · K2 Light Angle (spins the highlight
around every ball, 360°) · K3 Specular (to deliberate blowout) · K4 Shine
(broad satin → pinpoint gloss) · K5 Squash (separated beads → touching →
pressed-flat domed mosaic) · K6 Candy (chroma gain 0.75–2.7×; under Chrome = tint, mono → 0.5×) ·
S7 Backdrop Black/Video (dimmed live video between balls) · S8 Luma Size
(bright cells grow ±r/2, per-cell rsq stored in the buffer = free per
pixel) · S9 Packing Hex/Square · S10 Chrome (ball-bearings, K6 tint, mono backdrop) ·
S11 Bypass · **P12 Relief** = soft low-relief satin → deep-shadow wet gloss
(shading span + specular master ride together; floored at 25% span /
50% spec so K2–K4 read at any slider position — v0.1's 0 = flat
unlit mosaic left three knobs dead).

## Architecture

Streaming pipeline c1..c17 (LATENCY 16), no divide / no wide multiply on
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
- **Cell colours**: point-sampled per cell row. v1.0 buffers (all
  canonical 1W1R, power-of-2 words): rsq15 + sample-line parity in two
  128×16 memories (both candidates read at c1); colour (Y−64 10 b, U/V
  offsets 11 b = 32 b) in ONE 256×32 memory addressed {parity, cell},
  re-read for the Voronoi WINNER at c10 — replaced ~300 FFs of colour
  delay pipes (v0.1 carried a 46-bit 16+16+14 word through 8 stages). Row R is sampled on
  the CENTRE line of row R−1 — its spheres first win the Voronoi test
  ~0.16·H later, so the swap is never visible. Row 0 samples on the first
  active line, which is drawn as backdrop (read-during-write there is
  undefined on EBR). A right-edge partial cell whose centre lies past the
  line end samples the last active pixel on the first blank clock (v0.1
  left it unsampled = zero-init saturated green sliver at most sizes). Sat boost (K6) and luma→radius (S8) applied at WRITE time.
- **Per-line squares incrementally** (the timing fix): dy²/(dy−oy)² for
  both row candidates advance by (d+1)² = d² + 2d + 1 at each h edge —
  adds only. Band-wrap / mid-band-jump reset constants (H/2)², H²,
  (H/2±oy)², (H+oy)² precomputed per frame.
- **Frame FSM**: one shared 12×12 multiplier + 18-bit serial divider,
  ~85 cycles in the vsync cone (knob shaping, r, light vector, ranges,
  2 divides, 5 wrap squares). Sine from SDK sin_cos_full_lut_10x10.

- **v1.0 hardware contracts**: blanking gate at compose (bypass folded in);
  U/V swapped per pixel where the sample line's hsync→avid parity differs
  from the display line's (drape v1.3 technique — a sampled row replays on
  other lines); row anchor re-armed after 2 avid-less lines instead of the
  vsync edge; luma drawn black-referenced ((Y−64)·diffuse + 64) with a
  soft knee (slope ¼ above 704, cap 800); chroma clamped after shading;
  K1/K4/K5 latched through a ±4-code deadband (no grid flicker from ADC
  noise on a size step).

- **v1.0.1**: hex lattice B's cell 0 is centred ON the left edge, so it
  sampled the first active pixel — black on analog sources (black balls
  every other row at the left edge). Now sampled at its last pixel. The
  sample trigger no longer drives the w0 data hold (hold on any avid fall;
  row-level terms registered) — that cone was the HD critical path.

- **v1.1.0**: cell colours are a running average along the sample line
  instead of one pixel (one-pole IIR, tau 4 px below 40 px cells, 16 px
  above, reloaded over each line's first 4 px; sampled tau past the centre
  to cancel the lag). **HW-REJECTED 2026-10-01** ("colors don't look right,
  all the balls changing color") and reverted — v1.0.1 is current. Source
  kept in .gumball_work/gumball_v1.1_averaging.vhd.

## Timing v1.0.1 (routed, all seed 1)

| Config | Fmax |
|---|---|
| HD Analog | 86.42 |
| SD Analog | 83.19 |
| HD HDMI | 86.68 |
| SD HDMI | 87.11 |
| HD Dual | 84.88 |
| SD Dual | 85.90 |

7132–7143 LCs (93%), 10 BRAMs.

## Timing v0.1 (routed, all seed 1 unless noted)

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

v0.1 built 2026-07-14, sim-verified (dec 4). v1.0.0 finalization
2026-09-30 (contracts + control rules + buffer split, see above).
v1.0.1 left-edge fix HW-approved 2026-10-01 — current version (v1.1
averaging rejected on hardware and reverted).

**v1.0.1 FINAL except presets (signed off 2026-10-02).** The four v0.1
presets are untuned against v1.0's control changes (P12 relief floor,
Chrome = K6 tint) — retune on hardware (`./capture_preset.sh`).

Sims: `sim_001_default.png` (+`_input`), `sim_002_squashed.png`,
`sim_003_beads.png`.
