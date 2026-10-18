# HYPNOS — razor-sharp op-art rings (v1.0)

1960s op-art target (Vasarely / Bridget Riley / the *Vertigo* spiral). No input
video. v1.0 is a ground-up rewrite of the renderer for **extreme clarity**:
every ring and spiral edge is a one-pixel anti-aliased line at any ring size,
warp or spiral, and rings finer than ~3 px fade to an even grey instead of
moiré. Four shapes, four palettes, a bipolar speed slider, glide always on.

## Control map

| Phys | Control | Range / detent |
|---|---|---|
| K1 | **Center X** | ±2 screens; detent = frame centre |
| K2 | **Center Y** | ±2 screens; detent = frame centre |
| K3 | **Period** | exponential, 2048 px → 4 px ring period (glided) |
| K4 | **Warp** | detent = flat; CCW tunnel, CW dome (`p = K4/512` gamma, glided) |
| K5 | **Spiral** | whole arms −8…+8, ~64 knob units per arm, centre = plain rings |
| K6 | **Duty** | ink : paper band width 5–95 % |
| S7 | **Form** | Round / Angular |
| S8 | **Six-fold** | Off / On  → Round+Off **circle**, Round+On **flower** (6 petals), Angular+Off **square**, Angular+On **hexagon** |
| S9 | **Ink** | White / Colour |
| S10 | **Paper** | Black / Colour → **B/W**, **Rainbow** (colour ink on black: hue travels outward with the rings), **Duo** (white on blue), **Complementary** (red on turquoise) |
| S11 | **Invert** | swaps ink and paper |
| P12 | **Speed** | bipolar: centre = still, up = rings stream outward, down = inward; exponential, 1 cycle / ~68 s → ¼ cycle per frame (“warp speed” at the top) |

Removed from v0.x: Catch-Up (lives on in Cascade), the Glide switch (always on),
Zoom (K3 Period covers ring scale), Key/Gradient output (sharpness is the point).

## Renderer (streaming, no frame buffer, 22 EBR)

Everything happens in the **log domain** — no CORDIC (a 12-iteration CORDIC
alone was ~2400 LUTs):

- `hi, lo = max/min(|dx|,|dy|)` (13-bit integers). The centre glide eases in but
  never moves slower than 1 px/frame and lands exactly, so the tail of a glide is
  a steady slide, not sporadic one-pixel ticks. (Q4 sub-pixel coords cost +275 LC.)
- `log2 hi`, `log2 lo`: priority encoder + normalise + 256-segment PWL from EBR.
- `u = log2(hi/lo)` indexes two interpolated EBR tables:
  `log2 r = log2 hi + ½·log2(1 + 4^-u)` and the octant angle `atan(2^-u)`
  (~1e-5 accurate; better than the old CORDIC's angle).
- **Shapes**: circle = `log2 r`; square = `log2 hi` (exact L∞); hexagon / flower
  = `log2 r + log2 S(6θ)` from an interpolated EBR table.
- **Warp**: `p·(log2 d − LREF)`; exp2 mantissa (EBR PWL) × frequency mantissa →
  ONE shift gives the phase (Q18).
- **Spiral**: `+ arms·θ` (integer arms = whole cycles per turn = seamless).
- **Anti-aliasing**: a one-pixel **box filter of the band square-wave**,
  `(P(φ+w/2) − P(φ−w/2)) / w` with `P(x) = ⌊x⌋·D + min(frac x, D)`, where the
  filter width `w = |∇φ|` comes **analytically** from the log domain: radial
  `Lf + log2 p + (p−1)(log2 d − LREF)`, angular `log2(arms / 2πr)`, combined as
  `max + ½·log2(1+2^-2|a−b|)`, then rounded UP to a half octave so the divide is
  a shift (× 1/√2 by shift-adds): edges are 1–1.4 px. Above ¼ cycle/px `w` is
  widened so it reaches a full cycle (flat duty grey) at Nyquist. A window
  wholly inside the ink band is forced to exact full coverage.
- **Colour**: per-frame palette constants (ink − paper deltas); Rainbow ink from
  an EBR hue table. Luma lerps at full coverage; chroma switches at the band
  edge (4:2:2 chroma is half-res anyway).
- Long-carried values ride in **EBR delay lines** (an FF shift register costs an
  LC per bit per stage). Centre maths use a **serial** multiplier in vblank.

## Hardware contracts
- Program-generated **avid** (cubist v0.3.3 parity lock) → coloured output never
  U/V-swaps a line; outputs gated to neutral outside it.
- Frame tick = first vsync edge after active video (serration guard).
- Interlace: bottom field (`field_n='0'`) seeds +1 line.
- Line-width hysteresis (>2 px) so analog line wander doesn't shake the centre.
- One fixed latency (45 stages) for every mode.

## Build
`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron hypnos` (router1 is
required). 7415 LC (96.5 %), 22/32 EBR. Routed router1 sweep: HD Analog 79.4 /
82.0, HD HDMI 79.5 / 79.3, HD Dual 90.4 / 71.8 (seeds 1 / 2) — seed 1 passes all.
**Shipped vmprog (2026-10-01), routed-verified:** HD Analog seed 3 92.05, HD HDMI
seed 4 86.24, HD Dual seed 1 90.44 MHz; SD all > 84 MHz. 7392–7453 LC, 23 EBR.
Under machine load router1 can exceed the builder's 360 s timeout on early seeds
(HD Analog/HDMI and SD Dual retried) — that's a stall, not a timing miss.
**Sim-verified (all shapes, palettes, spiral, warp both ways, 1080i), not yet
HW-tested.**

## Fit notes
The first cut (12-iteration CORDIC + everything) was 8478 LUT4 — 3000 over.
The log-domain engine replaced the CORDIC (~2400 LUT). At this pipeline depth
~1700 LCs are register-only (pipeline copies, EBR read registers): long carries
ride EBR delay lines; vblank sequencer work is one op per state (the centre
glide in one state was a 52–62 MHz critical path).

## History
- v0.4 (2026-09-30): integer-arm spiral seam fix, vsync guard, field polarity,
  width hysteresis (CORDIC engine, B/W).
- v0.3: Catch-Up, Invert, solid centre. v0.2: CORDIC true circles + spiral.
  v0.1: octagon metric (hypnos_square, removed 2026-09-27).

## Community port — fit for the stock SDK (v1.0.2, 2026-10-15)
- The stock LZX SDK builds 8 configs; its SD Standalone core is ~600 LC larger
  than HD. v1.0.1 packed to 8299/7680 LC there and 7719 on HD Dual, so it did
  not fit even HD.
- v1.0.2, three bit-exact changes:
  1. **Booth multipliers.** Every per-pixel multiply (`s13_gp/ap`, `vsp`,
     `s20_sp`, `s23_ph/pl`, `s31_ph/pl`, `s43_py`) is a hand-written radix-4
     Booth (`bmul`; `bmul_u` for the unsigned `s31` products, sliced to
     width). Each operand-width pair was checked exhaustively in Python.
  2. **One shift for the phase.** The S33 scale (`<< k` for k >= 0,
     `>> -k` for k < 0) is one left shift by k+17 and a 21-bit window instead
     of two barrel shifters and a mux; k = -17 (the clamp) still gives zero.
  3. **Shared centre glide.** The two centre glides (x at sequencer states
     37..35, y at 21..19) share one datapath. Snapping to the target is
     c + (tg - c) = tg exactly, so it goes through the same adder.
- Proven bit-exact against v1.0.1: GHDL side-by-side, 40 frames (403 x 201
  raster), knobs and switches changing mid-picture, 0 mismatches in 3.24 M clocks.
- Packed LC (stock SDK): SD Standalone 8299 → 7630, HD Dual 7719 → 7071.

## Community port — router1 livelock fix (v1.0.3, 2026-10-15)

- **Cause of the stock-router1 hangs.** Not global promotion. `S32` computed
  `s31_lg + s31_lwp` with `s31_lwp = s30_lg + 128` and `s31_lg = s30_lg`.
  Adding 128 leaves bits 0..6 unchanged, so yosys merged the low 7 FFs of
  `s31_lwp` into `s31_lg`. That left 7 `SB_CARRY` cells with I0 == I1 (one net
  added to itself). nextpnr packs each into an LC whose LUT needs that net on
  both I1 and I2.
- **How router1 hangs on that LC.** On iCE40 one physical LUT pin feeds only
  one logical LUT input: the permutation pips leaving `lutff_N/in_K` share a
  single switch. Router1 routes the arc to `in_1_lut` through the physical pin
  the same net already holds for `in_2_lut`, and that rips up its own other
  arc. Then the other arc does the same, and so on forever.
  - Router debug traces from 5 failing (config, seed) pairs all show this on a
    `s31_lg[k]` LC.
  - The remaining-arcs count freezes, and each 1000-iteration batch is 1000
    ripups.
  - The ripup penalty grows only ~0.16 per round trip, so router1 never
    escapes.
- **Fix (bit-exact).** S31 now registers the flag `s30_lg + 128 > 0`, and S32
  forms `shift_left(s31_lg, 1) + 128`. That equals `lg + (lg + 128)` mod 2^12.
  The netlist now has 0 duplicate-input LCs, in every config.
- **Unchanged.** The same 8 nets are still promoted to globals (clock, the
  141-fanout EBR init-emulation reset, the blanking-gate reset, `sm_go`, the
  avid enables). No reset/CE restructuring was needed.
- **Result with the stock router1 command.**
  - Builds 8/8 on seeds 1, 2, 3, 4 and 5. The reference v1.0.2 built at best
    7/8, on seed 4.
  - Own HD (Sync-Out-free core) is 85–98 MHz on every seed.
- **Packed LC** (stock SDK): SD Standalone 7630 → 7617, SD Dual 7128, HD Dual
  7060.
- **Proof.** Bit-exact against staged v1.0.2:
  - GHDL side by side, 403x201 x 40 frames: 0 mismatches / 3,240,120 clocks;
  - 1003x201 x 16 frames: 0 / 3,225,648;
  - the rewritten S31/S32 stage, checked exhaustively in Python over all 4096
    values of `s30_lg`: 0 mismatches;
  - mutants caught: `+160` for `+128` gave 14 mismatches in 6 frames; a
    threshold of `> 32` for `> 0` gave 41 in 12 frames.
- **HW-validated 2026-10-15** (Ron: "looks good"): the stock-SDK v1.0.3 build, the first time hypnos has been flashed.

## Resolution scaling (v1.0.4, 2026-10-17)
- Baseline: ring period/centre in pixels (720p rings 1.5x, SD much larger and
  2:1 tall -- field_n is tied high on hardware, so the field_n-toggle
  interlace test never fired and every SD field line stepped dy by 1).
- Fix, all per-mode constants (the engine keeps real x pixels, so the 1-px
  box-filter AA is unchanged):
  - steps from the timing ID (`registers_in(8)`): `s_ystp` = field line pitch
    in x-pixel units, Q.8 (1080p/720p 256, 1080i 512, 480i 569, 480p 288,
    576i 480, 576p 240), and a mode class (1080 / 720p / SD);
  - dy accumulates `s_ystp` (Q.8, seeded +0.5 so 1080p integer rows are
    unchanged); the measured height accumulates it too (`s_lacc`), so the
    centre and K2 travel are in the same units;
  - the log distance is made the 1080-line one by `+ log2(s)` (s = x step:
    2 at SD, 1.5 at 720p), folded into the LREF subtract at S16
    (`s15_ld - s_lref`, `s_lref = LREF - log2(s)`, Q14, exact). Ring period,
    tunnel/dome warp (its reference radius included) and spiral follow
    screen height. The radial AA gradient gets `+ log2(s)` (Q6, sequencer
    state 8) so it stays per pixel; the angular one is already per pixel.
  - A first try put `p * log2(s)` on the Q6 frequency log instead: the Q6
    rounding, amplified by 1/p in the warp, left 720p rings ~2.7% large at
    p = 0.28. The Q14 LREF fold is exact (720p/SD radii within 0.5%).
  - `s_ilace` / `s_fpar` and the bottom-field +1 seed are gone (inert on
    hardware; interlace now comes from the timing ID).
- 1080p: 0 differing pixels against the baseline with `s_fpar` initialised
  to '1'. Against the untouched baseline the first frames differ (594k px in
  the 4-frame sim): with `s_fpar := '0'` the old toggle test took the FIRST
  field after power-up for interlace (H doubled, dy step 2), which bends the
  centre-glide start; the steady state is identical.
- Verified with a glide-bypassed sim copy (`.scaling_work/hypnos_tmp/gb_*`,
  sheet `sheets/hypnos_gbv2.png`): 720p and SD @4:3 match 1080p, rings round.
- Build: `MAKE_TIMEOUT=1500 SEED=1 NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron hypnos`,
  all 6 seed 1: HD Analog 83.4, SD Analog 86.0, HD HDMI 92.7, SD HDMI 94.5,
  HD Dual 89.5, SD Dual 94.3 MHz; 6782-6822 LC (~89%), 23/32 EBR.
