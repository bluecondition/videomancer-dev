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
