# Feedback — summary

A digital camera-at-the-monitor loop. Last frame's composite output is
averaged into a coarse store and read back through a per-pixel zoom /
rotate / drift transform with bilinear interpolation. Live video is keyed,
mixed, added or lightened over the echoes. Each pass through the loop is one
"generation" of processing: gain (folding above unity), hue rotation with
colour bloom, crisp contrast, and negative.

## Status

- v0.1.0 FINAL except presets: signed off by Ron 2026-10-11 ("mark feedback as
  finalized and put in the report as ready"). Built 2026-10-08.
- v0.1.3 (2026-10-17): resolution scaling for SD / 720p / interlace, 1080p
  bit-identical (see "Resolution scaling" below). Build with router1, seed 1.

## Build (2026-10-08, builder default router2)

| Config    | Fmax (MHz) | LC          | Seed |
|-----------|-----------:|------------:|-----:|
| HD Analog | 75.47      | 7089 (92%)  | 3    |
| SD Analog | 75.00      | 7103        | 1    |
| HD HDMI   | 78.86      | 7101        | 3    |
| SD HDMI   | 75.83      | 7094        | 1    |
| HD Dual   | 76.84      | 7097        | 3    |
| SD Dual   | 76.10      | 7106        | 1    |

25/32 EBR. All three HD configs needed seed 3 (seeds 1-2 missed). Remaining
worst path = the vblank sequencer's pc decode -> ALU operand mux (~13 ns,
mostly routing); an extra seed-21 profile of HD Analog gave 77.3 MHz. If a
rebuild misses everywhere: register the pc decode (one-hot step strobes),
or try `NEXTPNR_ROUTER="--router router1"`.

## Architecture

- **Store**: 128x64 luma (4 bit) + 64x32 chroma (U4 V4, signed so zero RAM
  = black + neutral), two banks ping-ponged by field parity. Each grid is
  split into 4 even/odd sub-banks ({c(0), r(0)}), so any 2x2 neighbourhood
  is one read per sub-bank: full-rate bilinear from one port each. 24 EBR + 1
  sine ROM.
- **Read**: affine DDA (u, v in Q18 cells) stepping per input pixel, line
  starts stepping by (dudy, dvdy) at each avid fall. Runs C_LOOK = 8 px AHEAD
  of the raster (idle value = pixel 8 during blanking), so the 11-stage read
  pipe meets live video that is only 3 FFs deep at the mixer. Mirror = fold
  u >= 64 to 128 - u; edges black (out-of-range taps zeroed) or wrap (natural
  address modulo). Blocks = nearest tap.
- **Write**: composite at k=8, one-pole IIR along the line (k = 3 at >= 1024
  wide, 2 below; chroma k+1), captured at each cell end by a cell DDA. Luma
  rows sample the line holding the row middle; chroma rows the line crossing
  an even -> odd luma row (the chroma row centre). Luma chain: gain (8x11
  mult) -> crisp -> wave-fold -> negative -> LFSR-dithered rounding. Chroma
  chain: hue rotation on ONE multiplier over 6 phases (chroma cells are >= 10
  clocks apart), bloom, clamp, negative, rounding.
- **Alignment bookkeeping**: IIR group delay trimmed in the per-frame read
  offset ((2^k - 0.5) kx - n kx / 2: exactly 0 at 1920 wide); +1/32 cell for
  the 4-bit weight floor; read DDA sampled at pixel and line centres
  (C_LOOK + 1/2, dvdy/2). Mixer and all feedback shifts round rather than
  floor (a floor inside the loop pumps a drift every generation).
- **Chroma parity**: stored canonical. Swap U/V at capture when the capture
  row's hsync->avid parity is odd, and at display on odd-parity rows (drape
  pattern).
- **Per-frame**: a program-counter sequencer in vblank (~600 clocks) on one
  serial 18x18 multiplier, one serial divider (2^25/W, 2^24/H; the numerator
  is a single bit generated from the count) and one 30-bit accumulator.
  Products chain from macc straight into the next operand or the
  accumulator.

## Controls

K1 Zoom (bipolar; left = tunnel), K2 Rotate (bipolar, up to ~84 deg/gen),
K3 Hue (bipolar hue rotation/gen + colour bloom from luma), K4 Drift
(bipolar, right = rise), K5 Live (key level / mix / add amount), K6 Blend
Key / Mix / Add / Lighten. S7 Edges Black/Wrap, S8 Mirror, S9 Pixels
Smooth/Blocks, S10 Negative, S11 Crisp. P12 Feedback gain 0.44..1.0 (at 75%
travel, fine near unity) ..1.5 overdrive (folds into rings, not white).
Bipolar knobs have a 16-step centre deadband and a squared response.

## Fit history (lessons)

- First cut: 6765 LUT standalone, 7933 LC in PnR (103%). Hogs: 17 array
  multipliers (~2700 LUT) and a case-per-state sequencer whose every 30-bit
  add/sub was separate hardware (~1700 LUT). Fixed: shift-add smul4/smul5
  with 4/5-bit weights, a serial chroma rotator, one shared accumulator.
- Second cut: 2439 LCs were DFF-only. Cut intermediate registers in the
  sequencer (chain from macc), the fb copies carried for Lighten (now base +
  addend through one adder), the EBR re-register (the next stage is muxes
  only), two merged interpolation stages, and the raw knob latch stage.

## Open / next

- Flash and check first: drift at static settings (K1/K2/K4 centred, P12
  ~95%): any steady creep means a sub-pixel alignment bias; HD HDMI bars;
  colour stability (U/V parity), and the left 8 px smear (pixels before
  C_LOOK reuse pixel 8's sample).
- Ideas to expand: feedback-only displacement by live luma (refraction loop),
  per-generation blur/sharpen kernel, quad mirror / kaleidoscope sectors,
  freeze/hold, strobe (update every Nth field), luma-keyed feedback mask,
  zoom centre offset, colour-cycling palettes.
- Presets are placeholders; capture from hardware.

## Community port — fit for the stock SDK (v0.1.2, 2026-10-15)
- The stock LZX SDK builds 8 configs; its SD Standalone core is ~600 LC larger
  than HD. v0.1.1 packed to 7838/7680 LC there (HD Dual 7410).
- v0.1.2, two bit-exact changes:
  1. **Luma gain table.** The per-pixel 8x11 multiply `l1_y * c_g`, then
     (+512) >> 10, becomes a 256-entry table (`g_tab`, 1 EBR) read at the
     same stage. c_g only changes in the vblank sequence, so `p_gtab` refills
     the table after each `commit` with one add per entry
     (acc = 512 + y*c_g, entry = acc >> 10, 256 clocks). The table starts
     as the identity, which is the power-up gain c_g = 1024.
  2. **Booth hue multiply.** The hue-rotation multiply (9 x 10 signed) is a
     hand-written radix-4 Booth (`booth9x10`): 5 partials of {0, +-b, +-2b},
     with the one's-complement +1s gathered into one correction word. It was
     checked exhaustively against a*b for all 262,144 operand pairs.
- Proven bit-exact against v0.1.1: GHDL side-by-side, 40 frames (403 x 201
  raster), knobs and switches changing mid-picture, 0 mismatches in 3.24 M clocks.
- Packed LC (stock SDK): SD Standalone 7838 → 7620, HD Dual 7410 → 7216.
- Stock build: all 8 configs on seed 1; own HD timing (core without the Sync
  Out generator) 88.5/85.4/80.2 MHz Analog/HDMI/Dual.

## Resolution scaling (v0.1.3, 2026-10-17)
1080p is bit-identical to v0.1.2 (scaling-work GHDL render: 0 differing pixels).
- **Hue at 720p / SD in the scaling sims was a sim artifact, not a program bug.**
  The store is kept in a "canonical" chroma frame (U/V swapped at capture and
  at display on odd hsync->avid parity rows, drape pattern). On hardware the
  decoder and encoder both pair Cb/Cr by hsync parity, so the frame the hue
  rotation runs in is the same in every mode. The scaling-work stimulus feeds
  a fixed-swapped 4:4:4 picture with hsync->avid offsets of 140 (1080p), 185
  (720p) and 69 (SD) clocks: the odd 720p/SD offsets flip the canonical frame,
  so K3's rotation ran backwards there (pink instead of purple) and the bloom
  landed on the other channel. With even offsets (VIT_HB 372 / 136) both modes
  render the 1080p purple. (Standard timings are even: 192 / 260 / 122.)
  The suspected hue-chain overrun is not real: chroma captures are 2 luma cells
  apart (>= 11 px at SD) and the chain needs 8 clocks. No colour change made.
- **Rotation on non-square field lines.** The cross terms now carry
  rho = field-line height / pixel width (timing ID: 480i 2.22, 576i 1.875,
  1080i 2, 480p 1.125, 576p 0.9375, else 1): dudy = -b kx rho,
  dvdx = b ky / rho, two extra serial multiplies in the vblank sequencer
  (steps 13 and 23; later steps renumbered +1/+2). Before, a 480i rotation
  sheared the echoes into a vertical streak.
- **SD capture-phase creep.** 720/128 = 5.625 px cells, so the cell-end
  capture lands on average 7/16 px late (luma) / 3/8 px (chroma, even cells);
  1920 and 1280 have integer cells, so 0. The read offsets now subtract it at
  SD (timing IDs 0/4/8/12); without it the echoes slid left ~0.44 px a field.
- **720p IIR.** g_k3 (W >= 1024) gave 720p the 1080p shift 3 on 2/3-size
  cells, so echoes softened faster. At 720p the write IIR now alternates shift
  3 / 2 per pixel (decay of a 1080p pixel over 1.5 px), with its own trims
  (luma 3.5 kx + 32269, chroma 7.5 kx + 67239, Q18, W = 1280), sized with an
  RTL-faithful 1-D model (.scaling_work/feedback_tmp/mtf3.py).
- Not changed (judgement): the store stays 128 x 64 cells over the whole screen
  in every mode, so Blocks cells are 1/128 of the width at 4:3 too (narrower
  relative to height than at 16:9); interlaced SD shows consecutive generations
  on alternate fields (comb on motion), as a real 60-field loop would.
- Build (2026-10-17): `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron feedback`,
  all 6 configs on seed 1 (router2 missed HD Analog on seeds 3-8, best 65.8 MHz).
  Standalone yosys vs v0.1.2: +55 LUT, +16 FF.

  | Config    | Fmax (MHz) | LC         | EBR   |
  |-----------|-----------:|-----------:|------:|
  | HD Analog | 88.22      | 7050 (92%) | 26/32 |
  | SD Analog | 89.73      | 7050       | 26    |
  | HD HDMI   | 84.70      | 7045       | 26    |
  | SD HDMI   | 82.34      | 7045       | 26    |
  | HD Dual   | 86.48      | 7046       | 26    |
  | SD Dual   | 94.42      | 7050       | 26    |
