# Pyre — simulated fire: an advected temperature field (Inferno's successor)

**v1.0.2 (2026-10-07) — the "always moves right" ROOT CAUSE.** Ron (on
v1.0.0): the fire constantly leans/moves right.  Not the wind (the gust
generator emulated exactly is balanced), not the turbulence/entrainment (a
render with K4 = K5 = 0 and the wind forced to zero still shifted), and not the
LFSR grain (a hash-grain variant changed nothing).  It was a CELL-INDEX
mismatch in the three-period phase-B pipeline introduced by the v0.2
re-spacing: the gather in period p is for cell b_x = p-3, but its result is
written at step 5 of period p+1 to a_x = p-1 — every field the whole field
(burner included) moved two cells to the right (~15 px/field at 1080p).  Final
write now addresses c_x = n-4 and the job runs one more period.  Second index
bug fixed at the same time: the write-back of the previous row read new-row
cell n-3, which the late phase had already overwritten with the current row's
result, so the grid also shifted UP one row per field; it now reads cell n-1.
Diagnosis tools: per-row cross-correlation lean metric on GHDL renders,
knob-zeroed renders, forced-wind source variants (`.pyre_work/var_w0`, `var_w4`),
exact LFSR emulation of the gust generator.

**v1.1.0 HW-REJECTED and REVERTED (2026-10-07).** Ron on hardware: glitchy
bars, colours collapsed to orange with no red/yellow range, Luma Mod broken.
Source reverted to v1.0.0 (256x64; saved copies: `.pyre_work/pyre_v1.1.0_rejected.vhd`,
`gen_tables_v1.1.py`).  Not root-caused; suspects for a later post-mortem: the
8-bit S/D table clamp, the FF video-delay pipe (Overlay/Luma path), the grid
bank mux, and the 27-EBR placement (HD Dual needed seed 5).  Do not re-offer
the vertical bump without a hardware A/B plan.

**v1.1.0 (2026-10-06) — vertical bump: 256x80 grid (rejected, see above).** Ron asked whether
the grid could grow again; horizontal is at the display reader's 6-px cell
limit and EBR-bound, but 80 rows (+25% vertical: 17 -> 13.5 px rows at 1080p)
fit by paying 4 EBR: rows 64..79 live in a second 4-EBR bank (both banks read
every clock, outputs muxed by the delayed row bit, write decoded by row), the
S/D smoothstep table shrank to 16 bits (S clamped 255 -> 1 EBR instead of 2),
the 32-bit Overlay video-delay ring (2 EBR) became an 11-deep 30-bit FF pipe,
and the mod-3 slot function is logic (digit fold).  27/32 EBR.  Screen maps
to rows 0..77 (burner 78/79 hidden); cache slot = row mod 3 (per ROW, so the
post-vsync chain length no longer matters); 480i fields (240 lines, 3 lines
per row = 2.6k clocks < the 2.75k half-mode job) use a 64-row WINDOW (rows
16..79, 61 rows displayed) via a per-field row offset added to every job row.
Trap hit on the way: leaving the screen at 61 rows made the post-vsync chain
21 jobs (108k clocks > the 90k 1080p vblank) -> bright band at the top.
Second trap (the build Ron flashed, "fire split in two and vertically
offset"): the vertical DDA's row field was still 6 bits, so rows 64..77 wrapped
onto rows 0..13 (fire mid-screen, the black top repeated below it, and the
unrequested rows made the post-vsync chain overlong -> band at the top).
s_yacc is now 23 bits (7-bit row).  GHDL renders: NTSC (windowed half path)
OK; 1080p full path verified after the 7-bit fix (see below).

**v0.5.1 (2026-10-05)** — the v0.5.0 triangle sway (34 s cycle) was
"unrealistic"; now random GUSTS: every 24..87 fields (0.4..1.5 s) a new wind
target is drawn from a centre-weighted random (sum of two LFSR nibbles - 15,
x3/4: -11..+11 sixteenths of a cell per field, straight most likely); the wind
eases toward it by 1/16 per field (~0.25 s) and both lattice drifts ride it
(wind/4 and wind/4 + wind/8), so the fire leans left, right or stands straight
at random.

**v0.5.0 (2026-10-05)** — Ron: the fire leaned/moved only to the right.
Cause: the octave-1 turbulence lattice drifted right at a constant 1/16 cell
per field (the eddy pattern that shapes the tongues marched with it).  Now a
2048-field triangle wave (34 s at 60 Hz) drives both lattice drifts (+-2/16
cell per field, octave 2 at 1.5x) AND a whole-field wind (+-8/16 cell per field
added to every cell's u), so the fire sways left and right.  Fuel: K1 now spans
0..86% of the raw range (K1 * 0.86 before the Blaze blend; Ron: "0-86% is the
right 0-100").

**v0.4.0 (2026-10-05)** — P12 is now **Blaze**: effective fuel = K1 +
(1023 - K1) * P12 / 1024, so the slider alone blends the fire from the knob
setting to the full-screen climax (cooling base 31..0 — at 0 parcels only lose
T/32 + grain and fill the screen; burner level 32..255; pocket/luma amplitude
0.25x..1.25x) and no longer touches rise speed or turbulence (K4 -> kt = K4/16,
K6 -> vb16 = K6/32 + 5 directly).  The per-field FSM does one serial product
((1023 - K1) x P12) instead of three.  Model: Blaze 100% at default Fuel
engulfs the screen to the top edge.

**v0.3.0 (2026-10-04)** — Ron's HW feedback on v0.2 ("on the right track"):
(1) vertical lines at the bottom instead of fire emitting from the bottom edge
= the two BURNER rows (62/63) were on screen, their per-cell random values
stretched over the bottom ~34 px; the screen now maps onto rows 0..61 (incy =
61*2^16/lines) and the row jobs the raster never reaches (d_last+1 .. 65) run
as a chain after vsync (replaces the fixed 64/65 pair).  (2) Fuel 0% still gave
a third of a screen of fire: only the cooling scaled with Fuel; now the burner
level (24..214), the burner pocket amplitude (0.25x..1.25x via a per-field
`s_samp` multiplier on the phase-B multiplier's free slot) and the cooling base
(32..1) all scale, and the Luma Mod injection uses the same amplitude:
(luma8-96)*amp/128.  Model: mean cell temperature 0.09/15 at Fuel 0, 5.8 at
Fuel 100 (fills ~80% of the screen).  Sim note: the image tester's fake 8-line
vblank is shorter than the 5-job post-vsync chain (14k clocks at 720p half
mode, 27k at 1080p), so sims show a bright first LINE; real vblanks after
vsync are 17k (PAL) .. 90k (1080p) clocks, so hardware is fine.

**v0.2.0 (2026-10-04)** — Ron's HW feedback on v0.1: too soft, and Zoom only
stretched the fire wider.  Grid doubled to 256x64 at 4 bits (same 16 EBR,
hash-dithered stochastic rounding), K2 is now **Width** (horizontal
compression of the turbulence/burner lattices 1x..3x -> dense narrow
tongues), palette low end steepened for crisp flame boundaries, SD/720p run
at half horizontal resolution.

## What it does
Fire that is *simulated*, not drawn: a 128x64 temperature grid lives in EBR
and is re-simulated every field.  Hot parcels rise faster than cool ones
(buoyancy), the flow converges toward hotter columns (entrainment), and two
octaves of curl noise swirl everything — so tongues stretch, pinch off, lick
and dissolve the way a fluid does, instead of noise scrolling past a gradient
(Inferno's tell).  A flickering burner along the bottom feeds it; cool parcels
linger as smoke.  Luma Mod makes every bright video cell a burner (burning
silhouettes with flames pouring off them), Blue Flame negates the chroma into
a gas-flame look, Overlay draws the fire opaque over the video with 50%
smoke, Bypass is a true passthrough.

## How it works
- **Grid**: 256x64 x 4-bit T in one 16-EBR bank (addr = row & x).  Cells are
  7.5x17 px at 1080p; the display DDAs (x per pixel, y per line) come from a
  serial restoring divider on the measured width / active-line count, so any
  mode maps the whole grid to the whole screen.  4-bit storage: cells expand
  x17 for all arithmetic; the new value (8-bit precision after cooling) is
  rounded to 4 bits as (t*15 + dither)/256 with dither = hash(x, row) + a
  per-field phase stepping by 37, so sub-level cooling rates act on average
  without 60 Hz sparkle (each cell's threshold moves slowly).  Deterministic
  rounding would freeze any value whose per-field cooling is under one level.
- **Half mode** (measured width < 1536: SD, 720p): the sim runs 128 cells
  (even columns, odd written as copies), the display DDA runs 128 cells and
  reads even columns.  Needed because 480i gives only 3.2k clocks per grid
  row (256 cells x 20 = 5.1k) and the cache reader needs >= 6 px per cell
  (720p would be 5.0).
- **One bank suffices**: job(d) runs each time the raster enters grid row d —
  prefetch grid row d+2 into a 3-slot row cache, simulate row d-1 into a
  new-row buffer, and write back row d-2 from that buffer.  The sim reads
  rows >= its own (parcels only come from below, v >= 0) which are still last
  field's; the raster only reads the cache, prefetched before the rows it
  shows are overwritten.  Jobs 64/65 after vsync finish row 63, flush, and
  prefetch rows 0/1.  Cache slot = d mod 3 (66 jobs/field is divisible by 3).
- **Cell sequencer**: 20 clocks per cell, three cells in flight (phase A =
  velocity for cell n; phase B = gather for cell n-1 from step 8, finishing
  with cooling/burn/rounding at step 5 of the following period), N+3 periods
  per row: 5.4k clocks at 1080p (37k available), 2.75k in half mode (480i
  has 3.2k).  Both sim multipliers are two-stage (b operand split 5/5 bits,
  latency 3) — the single-stage 11x10 product was a 13 ns path (73.9 MHz).  Phase A: 8 hashes (one 2-stage hash unit), a 128-entry
  smoothstep S/D ROM, one 11x10 multiplier (12 products: per octave
  A = e10<<8 + dA*S(fy), B = e01<<8 + dA*S(fx), dpsi/dx = A8*D(fx),
  dpsi/dy = B8*D(fy); then T0*kb buoyancy, (Tr-Tl)*ke entrainment,
  (dpx1+dpx2)*kt, (dpy1+dpy2)*kt).  u16 = kt*dpsi/dy + entrainment,
  v16 = vbase + T0*kb/256 - kt*dpsi/dx, clamped 0..95 (1/16 cell units).
  Phase B: 4 grid gathers at (x*16-u16, r*16+v16) -> 3 lerps on a second
  multiplier; cooling = cbase + T/32 + LFSR(0..3) above T=40, csm + LFSR
  below (smoke regime); burner psi2 bilinear (3 more products on the phase-B
  multiplier's free slots) on rows 63/62: src = srcl + 1.25*(psi-128) +
  rnd(0..46); Luma Mod: (luma8-96)*2 from the fuel banks; embers: 1/2048
  cells per field set to 200.
- **Curl noise is analytic**: the derivative of smoothstep-bilinear value
  noise is available from the same 4 corner hashes (D(f) = 6f(1-f)), so no
  finite differences; lattices scroll up at 3/4 and 5/4 of the base rise
  (frozen turbulence riding the flow) with slow opposite horizontal drifts.
- **Luma Mod fuel**: per-cell one-pole IIR luma averages (acc += (y-acc)/8)
  in two 128x10 EBR banks by grid-row parity; the display accumulates row d
  into bank d&1 (read-ahead of the next cell at each crossing, commit at the
  crossing / end of line, re-init on the row's first line), the sim reads the
  other bank for row d-1.
- **Display**: free-running alternating cache reads two cells ahead feed
  cur/next/next2 register pairs (6-read preload after each hsync), smoothstep
  weights (C_SS6) on 8-bit fractions, 3 multiplies -> T8 -> 256x30 palette
  ROM (limited range 64..768, U/V swapped, entries < 24 black) with a fabric
  hop; smoke = T in 4..39 as gray 64 + T*K3/64.  Latency 12 in every mode;
  blanking gate at tap C_LATENCY-2; serrated-vsync guard (s_vsedge) on every
  per-field action.
- **Per-field FSM**: serial multiply K1/K4/K6 x Intensify boost, two serial
  divides (incx, incy), derive; BF/BF2 split (clamp->add chain was the
  critical path at 71.8 MHz; split -> 76.7).

## Controls
- **K1 Fuel** — 0..86% of the raw fuel range (burner level, pocket/luma amplitude, cooling base all scale): 0% = almost nothing; Blaze continues above it
- **K2 Width** — lattice x step 16..47/16 per cell: turbulence and burner pockets compressed 1x..3x horizontally (dense narrow tongues); default 1/3
- **K3 Smoke** — cool-regime persistence and smoke brightness
- **K4 Turblnce** — curl-noise gain kt = K4/16 (512 -> 32)
- **K5 Plume** — buoyancy (hot rises up to 4 cells/field faster) + entrainment
- **K6 Rise** — base rise speed K6/32 + 5 sixteenths of a cell per field (512 -> 21 ~ 1.3 cells)
- **T7 Ember** — random hot cells
- **T8 Luma Mod** — bright video cells burn (bottom burner off)
- **T9 Blue Flame** — negated chroma (yellow -> blue, red -> cyan)
- **T10 Overlay** — fire opaque over the video, smoke 50%
- **T11 Bypass** — passthrough with data_in's own syncs
- **P12 Blaze** — blends the effective fuel from K1 toward 100% (full-screen climax: burner 255, cooling base 0); speed and turbulence untouched

## Prototype
`.pyre_work/proto.py` (float look prototype) and `.pyre_work/model.py`
(integer-exact model of the sim core with the VHDL's widths and shifts;
renders contact sheets for any knob set).  Look validated on the model at
default, max and low-fuel/high-smoke settings.

## Status
**v1.0.2 built (two pipeline index fixes), awaiting HW.**  v1.0.0 FINAL except presets (restored 2026-10-07 after the v1.1.0 80-row attempt was HW-rejected).  v1.0.0 was signed off — Ron: "pyre can be finalized for now"
(2026-10-06), after HW iteration v0.1 -> v0.5.1 (realistic fire preferred over
Inferno).  v1.0.0 = v0.5.1 with the version bump, repacked.

History: v0.1.0 flashed 2026-10-04: Ron: "does look good at some settings", but too
low-resolution/soft, and Zoom read as horizontal lines at one end — fire
always too wide; wanted Inferno-style Width.  v0.2.0 addresses both; not yet
flashed.

## Build
### v1.0.2 (2026-10-07)
router2, ~6165 LC (80%), 26 EBR, all genuine post-route passes: HD Analog 78.19
(s1), SD Analog 77.56 (s1), HD HDMI 77.81 (**seed 2**), SD HDMI 78.02 (**seed 2**;
seed 1 stalled), HD Dual 79.43 (s1), SD Dual seed 1) - Fmax: 73.87 MHz.  Packed to
`out/rev_b/ron/pyre.vmprog`.  GHDL 720p render: upright tongues, no offset.
Note: the v0.2..v1.0 builds also rose one extra row per field (write-back bug),
so at a given Fuel the fire now sits a little lower than Ron is used to.

### v1.1.0 (2026-10-06, 7-bit-row fix)
router2, ~6425 LC (84%), 27 EBR, all genuine post-route passes: HD Analog 78.15
(s1), SD Analog 77.24 (s1), HD HDMI 80.62 (s1), SD HDMI 68.84 (s1), HD Dual 78.05
(**seed 5**; 1-4 missed), SD Dual 74.69 (s1).  Packed to `out/rev_b/ron/pyre.vmprog`.
The earlier 6-bit-row build (HD Dual seed 3) is the one that flashed wrong.

### v0.5.1 (2026-10-05)
router2, ~6130 LC (80%), 26 EBR, all genuine post-route passes: HD Analog 80.00
(s1), SD Analog 77.75 (**seed 2**; seed 1 stalled the router), HD HDMI 77.39
(s1), SD HDMI 78.83 (s1), HD Dual 81.83 (s1), SD Dual 77.46 (s1).  Packed to
`out/rev_b/ron/pyre.vmprog`.

### v0.5.0 (2026-10-05)
router2, ~6115 LC (80%), 26 EBR, 6/6 genuine post-route passes on seed 1: HD
Analog 76.30, SD Analog 76.38, HD HDMI 78.19, SD HDMI 78.62, HD Dual 77.95, SD
Dual 76.34 MHz.  Packed to `out/rev_b/ron/pyre.vmprog`.

### v0.4.0 (2026-10-05)
router2, ~6010 LC (78%), 26 EBR, 6/6 genuine post-route passes on seed 1: HD
Analog 78.21, SD Analog 71.23, HD HDMI 75.95, SD HDMI 79.85, HD Dual 75.02, SD
Dual 75.75 MHz.  Packed to `out/rev_b/ron/pyre.vmprog`.

### v0.3.0 (2026-10-04)
router2, ~6100 LC (79%), 26 EBR, all genuine post-route passes: HD Analog 79.18
(s1), SD Analog 74.96 (s1), HD HDMI 77.41 (**seed 3**; 1-2 missed), SD HDMI 73.10
(s1), HD Dual 77.38 (s1), SD Dual 75.02 (s1).  Packed to `out/rev_b/ron/pyre.vmprog`.

### v0.2.0 (2026-10-04)
HD Analog seed 1: 82.01 MHz post-route, 5332 LC (69%), 25 EBR.  Critical paths
fixed on the way: per-line y-DDA add+compare+mod3 in one cycle (73.3) -> two
cycles; single-stage 11x10 sim multiplier (73.9) -> split.  Full build: see below.

### v0.1.0 (2026-10-04), `./build_programs.sh ron pyre` (router2 default), 5930 LC (77%),
26/32 EBR, all genuine post-route passes:

| Config | Seed | Fmax (post-route) |
|---|---|---|
| HD Analog | 1 | 76.68 MHz |
| SD Analog | 1 | 79.38 MHz |
| HD HDMI | 1 | 76.85 MHz |
| SD HDMI | 1 | 73.27 MHz (Fmin 27) |
| HD Dual | 1 | 79.06 MHz |
| SD Dual | 2 | 70.91 MHz (Fmin 27; seed 1 stalled the router) |

Packed to `out/rev_b/ron/pyre.vmprog`.  First pack attempt failed on a 151-byte
description (limit 127 bytes) — shortened.

**v0.2.0 first flash: completely black.** Cause: in the two-cycle vertical
DDA the `s_ystep <= '0'` default was written AFTER the hsync block that sets
it, so the later assignment won, the row pointer never left 0, and the raster
only ever showed rows 0/1 which are never simulated.  Default moved to the top
of the process.  (The bug also let yosys fold away ~600 LC, which is why the
broken build was 69% and the fixed one is 77%.)  GHDL renders at 1080p30 and
720p (half mode) both show the fire after the fix (hue swapped in the sim as
usual).

Full v0.2.0 build (router2, 2026-10-04, after the fix), all genuine post-route
passes, ~5955 LC (77%), 26/32 EBR:

| Config | Seed | Fmax (post-route) |
|---|---|---|
| HD Analog | 1 | 78.67 MHz |
| SD Analog | 1 | 78.47 MHz |
| HD HDMI | 5 | 75.06 MHz (seeds 1-3 missed, 4 stalled) |
| SD HDMI | 1 | 76.23 MHz |
| HD Dual | 1 | 82.71 MHz |
| SD Dual | 1 | 72.07 MHz (Fmin 27) |

Packed to `out/rev_b/ron/pyre.vmprog`.
