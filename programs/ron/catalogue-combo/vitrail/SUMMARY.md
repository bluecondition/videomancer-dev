# Vitrail — summary

Catalogue combo #20 (seed 20260915): bloom / glow + palette quantization +
mirror / kaleidoscope. v0.1.0 (2026-09-11) was flashed and liked ("vitrail is
good") and listed Ready on the shelf. v0.2.0 (2026-10-12) is the first
finalizing pass, built from a review of v0.1.0. v0.2.1 (same day) fixes v0.2.0's
dead bloom, found on hardware.

## v0.2.1 (2026-10-12) — HW: "P12 doesn't seem to be doing anything"

The v0.2.0 box sums were held at zero while `vsync_n = '0'` (a LEVEL test). On
hardware vsync_n can read low across active video (polarity), so the sums were
zero all frame: no bloom, so P12, K3, K4, S7 and S11 were all dead. Second
instance of the deadchannel v1.1 level-vs-edge trap. Now the sums are re-zeroed
only once the input has had no avid for 512 clocks (vblank; hblank is 138..280).

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Strip | width of the mirrored source strip, 1/32 .. 1/2 of the picture, square-law knob; the top is a two-pane bookmatch |
| K2 Axis | where the strip is taken from; the strip always stays inside the picture (12 px in from each edge) |
| K3 Thresh | bloom threshold, black (64) .. near white (897) |
| K4 Radius | bloom box width 4 .. 128 px, exponential knob |
| K5 Levels | luma levels 2 .. 16; black (64) and white (~940) are always two of them |
| K6 Chroma | 0 = grey; low = a few bold glass colours (3 levels per axis, saturation boosted); high = near-original colour |
| S7 Wash | Colour: the glow is luma only; White: it also bleaches chroma toward neutral |
| S8 Mirror | Kaleido (alternate panes mirrored) / Tile (repeated) |
| S9 Order | Pal Last: palette after the bloom (the glow posterizes into bands); Pal First: palette before (smooth glow over flat poster) |
| S10 Motion | Fixed / Sweep: the strip ping-pongs across the picture, width/512 px per frame, content flowing through fixed mirrors |
| S11 Bloom | Lights glow / Shadows bloom dark |
| P12 Glow | bloom gain 0.25x .. 4x, square law; the top end floods the bright areas to white |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- The bloom trailed RIGHT of every highlight: the running box covered only
  the n pixels to the left of the pixel it was added to (up to 64 px of
  one-sided smear, mirrored into alternating directions by the kaleidoscope).
  Now centred: the base picture is delayed n/2 through a second EBR ring and
  written n/2 columns back; the line's last n/2 columns are written after
  active video (write bank latched per line), and the box sums are held at
  zero while vsync is low (where n can change) instead of at every avid edge.
- Kaleidoscope seams: Axis above ~73% wrapped the strip around the picture
  edge (a hard cut inside every pane), and Sweep wrapped x + slide at the line
  width, which is not a multiple of the pane period (a seam crossing the
  screen, plus a whole-pattern jump each time the slide counter wrapped).
  The strip now always lies inside the picture and the fold is a per-pixel
  counter: no wraps anywhere.
- Chroma posterize had no neutral level (mask | half): greys and whites came
  out tinted (+64 .. +256 on both U and V; at 2 levels every grey became one
  colour). Now a mid-tread round: 512 is always a level.
- Luma posterize never reached black or white (2 levels = two mid-greys at
  256 / 768; 16 levels put black at 96). Levels now land exactly on 64 and ~940.
- P12 Glow at 0 switched the bloom off, so K3, K4, S7 and S11 did nothing at
  the bottom of the slider. Gain now has a 0.25x floor.
- Radius: the top 37% of the knob was dead (all 64 px) and its minimum was 2,
  not the displayed 4. Thresh: the top ~8% was above white (no bloom).
- Bloomed luma clamped at 1023 (superwhite clips RGB and kills chroma on
  hardware). Now 940.
- Panel: S7 and S11 were both "Bloom"; S10 "Axis" duplicated K2. Renamed S7
  Wash (Colour / White) and S10 Motion (Fixed / Sweep).
- K1 (64..512 px), K5 (2..16), K6 (1..8) displayed linear values but were
  4-step knobs. All three are continuous now; K1/K6 display 0..100%.

Upgrades:
- Strip width is continuous (any width, 1/32 .. 1/2 of the measured width).
- The kaleidoscope is centred: the picture centre is a mirror line, so the
  composition is symmetric (v0.1.0 started a pane at the left edge and cut
  one mid-pane at the right).
- Sweep moves the source strip (ping-pong) instead of panning the mirror
  pattern sideways.
- K6 is a designed range: grey, then bold glass colours, then natural colour.
- 2-LSB deadband on K1/K2/K4/K5/K6 and +-1 hysteresis on the measured width,
  so pot noise cannot shimmer the panes now that the geometry is continuous.
- no_rw_check on all 8 RAMs (no read-during-write emulation logic).

Architecture: per-frame constants (strip width, axis, half width, the fold's
start phase = (half - 1) mod / div strip by repeated subtraction, gain,
quantizer tables) come from a vblank sequencer with one shared 11x11
multiplier (split hi/lo, product 3 states after its operands). The palette is
`vitrail_quant.vhd`, instanced twice (before and after the bloom; the idle one
passes through at the same 5-clock latency). Latency 8, all modes.

## Hardware checks for v0.2.0

- Glow is now symmetric (v0.1.0 was a rightward smear — say if that was liked).
- Kaleidoscope centred; K1 sweep from many thin panes to the bookmatch; no
  pot shimmer on K1/K2.
- Greys stay grey at every K6; K6 low end gives bold flat colours.
- K5 at 2: true black and white poster.
- S10 Sweep speed (width/512 px per frame).
- HD HDMI with Sweep on: posterized hard edges in motion are the thick-bar
  trigger profile. If bars show, add the damier output conditioning.
- Presets are hand-translated to the new mappings; re-capture on hardware.

## Build

`./build_combo.sh vitrail` (router2 default). v0.2.0, 2026-10-12, all six configs
seed 1: HD Analog 78.9, HD HDMI 80.4, HD Dual 75.1 (thin margin), SD Analog 82.6,
SD HDMI 82.8, SD Dual 75.9 MHz; ~6,055 LC (79%), 21/32 EBR; `emulate_transparency`
count 0. (v0.1.0: 3,507 LC, HD 83.1 / 88.6 / 79.7.) The two quantizer instances
are ~650 LUT each — merging them into one is the first lever if LC is needed.

v0.2.1, 2026-10-12: HD Analog 82.2, HD HDMI 85.8 (seed 1), HD Dual 78.0 on
**seed 3** (seeds 1-2 missed), SD Analog 82.6, SD HDMI 77.9, SD Dual 81.9 (seed 1);
~6,060 LC, 21 EBR.
