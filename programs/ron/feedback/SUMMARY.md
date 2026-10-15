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
