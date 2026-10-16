# Tonecurve — summary

Catalogue combo #34 (seed 20260918): tone curves / S-curve contrast +
picture-in-picture with warping + colour bleed / chroma lag. v0.1.0
(2026-09-11) was flashed and liked ("tonecurve is good", 2026-09-12), with
no later hardware notes. v0.2.0 (2026-10-12) is the first finalizing pass,
built from a review of v0.1.0. It is **not flashed yet**.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Curve | continuous sweep through 8 curves, neighbours blended: gamma 0.5, 0.7, 1.5, 2.2, inverse S, soft S, hard S, crush (display 0.0–7.0; default 5.0 = soft S) |
| K2 Contrast | gain about mid grey before the curve, 0.25–4.2× (unchanged) |
| K3 Size | inset 1/2, 1/4, 1/8 (labelled; default 1/4) |
| K4 Warp | inset wobble, 0..±255 source px (unchanged) |
| K5 Bleed | chroma IIR smear 2ⁿ px, n = 0..7 (unchanged) |
| K6 Lag | chroma behind luma, square law: 0–254 px in HD, 0–127 px in SD |
| S7 Curve In | the curve goes to the main picture / the inset |
| S8 Place | inset in the corner / centred |
| S9 Bleed In | main / inset |
| S10 Inset | picture / negative (luma and chroma, before the curve) |
| S11 Lag In | main / inset |
| P12 Amount | curve strength 25–200%: 0.25× .. 2× the curve's deviation from the original. 100% (the exact curve, v0.1's top) sits at 43%; above it the curve overdrives |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **S10 Negative did nothing at the default routing.** The negation lived
  inside the curve path, so with S7 on Main (the default) the inset never
  took it, and at Amount 0 nothing did. The negative now always applies to
  the inset, before the curve, and it is a full negative: luma about mid
  (64 ↔ 940) and chroma inverted.
- **The curve posterized.** It was a 128-entry ROM indexed by luma / 8, so the
  output had at most 128 levels: steps of up to 59 codes in the shadows on
  gamma 0.5, about 32 levels across the picture at low Contrast, and black
  lifted to 123 on gamma 0.5 (ROM entries were bin centres). Luma now goes
  through a 512-entry transfer LUT that a vblank builder fills. It
  interpolates linearly between curve samples, which are now exact point
  samples, so black stays 64 and gradients are smooth.
- The main picture read 2 px to the left of its own position (the address
  chain's x lagged the inset branch by a stage), so the right 2 columns were
  black.
- The inset read past the line ends: Warp opened black wedges inside the
  inset, and its edges showed the capture's black blanking columns. Inset
  reads are now clamped to the line's real columns (C_EDGE = 12, the
  standard edge fill).
- Categories "Colorizer", "Mixer" and "Retro" are not valid categories. They
  are now Color, Signal and Vision.

Controls (bold, always-on):
- P12 at 0 disabled K1, K2 and S7 (and S10 in practice). Amount now has a
  0.25× floor and runs to a 2× overdrive. Overdrive deepens gamma and crush,
  and folds the S curves over into a solarized midtone reversal (inverse S
  most strongly). This is the slider's climax.
- K1 was an 8-step knob. It is now continuous: the builder blends the two
  neighbouring curves. The order changed so that a sweep runs bright → dark →
  flat → contrasty → crushed: inverse S moved from slot 6 to slot 4.
- K6 Lag topped out at 31 px (1.6% of an HD line). It now reaches 254 px in
  HD and 127 in SD, on a square law, so v0.1's whole range sits in the
  bottom 35% of the knob. Lag is now a chroma read-address offset, not a
  32-deep ring. That frees an EBR, and the lagged edge fills from the first
  real column instead of going grey. In the inset it scales with the
  picture.
- The bleed IIR restarts at the first real column, so it no longer fades in
  from grey at the left edge.
- K3 has value labels, with the labelled-default index guard (the firmware
  loads a labelled default as its index).
- S7/S9/S11 are now "Curve In", "Bleed In" and "Lag In", so they no longer
  share names with K1/K5/K6.

Hardware contracts:
- Per-row U/V parity fix (drape v1.3). Every row replays the row above, so
  rows whose hsync→avid parity differs from that row's get U/V swapped back.
- HD HDMI bar conditioning (damier v0.4.4) before the gate: [1 2 1] on Y/U/V,
  luma (13y + 192)/16 (64 stays, 940 → 775). Crush and hard S at high
  Contrast make full-swing edges in motion, the bar trigger profile. The
  inset frame is drawn at 940 (→ 776, about v0.1's 768) and is 2 px wide, so
  it survives the smoothing.
- ±1 hysteresis on the measured width.
- `no_rw_check` on every RAM. A line-buffer bank is only read while the other
  bank is written. The LUT is written only in vblank (no avid for 512
  clocks), where its reads are gated. Standalone yosys:
  `emulate_transparency` count is 0.
- Latency 18 → 17, the same in all modes. Output data is now aligned to the
  output avid (v0.1 was one pixel off; it was gated consistently, so this
  was harmless).

Architecture: the pixel path lost both multipliers (contrast 12×9 and amount
11×9). Contrast, curve blend and amount are per-entry LUT math in a
27-state builder with one shared 12×10 multiplier, split hi/lo (operands at
state k, product used at k + 3). Each pass latches its parameters at entry 0.
Computing runs any time; only the LUT write waits for the vblank zone. One
pass is 13.8k clocks, about 7 passes per HD vblank and 1 per NTSC vblank.
The curve ROM comes from `tools/gen_curves.py` (`--check` reproduces the
v0.1 table from the same formulas, error 0).

## Hardware checks for v0.2.0

- K1 sweep: smooth through all 8 curves, with no banding in gradients. Black
  should stay black on gamma 0.5 (v0.1 lifted it).
- S10 Negative at the defaults: the inset should go fully negative (it did
  nothing before).
- P12: 0% = a gentle 25% curve; ~43% = v0.1's full curve; 100% = overdrive
  (solarized S curves). Is the climax liked?
- K6 Lag at the top: 254 px of colour ghosting in HD.
- Warp at max: inset edges fill with colour, no black wedges.
- Whites now peak at ~775 (bar conditioning). Say if it reads dim.
- Presets are hand-translated to the new mappings (same curve, lag px and
  strength as v0.1); re-capture on hardware.

## Review backlog (not done)

Tier 2, worth doing after hardware feedback:
- Continuous Size: a per-pixel DDA for the inset step, so Size zooms the inset
  smoothly from 1/8 to 1/2 (or full width) instead of three labels.
- Warp option: wobble the whole inset window, frame included, as a flag wave,
  instead of the content inside a fixed frame.
- Tie the warp's vertical period to the inset height. At 1/8 the inset shows
  a third of a cycle (a lean, not a wobble); at 1/2 it shows 1.3 cycles.
- Contrast: log knob with unity at the centre (unity now sits at 19% of the
  travel).
- Bleed: half-octave steps and a 256-px top for HD (the IIR is a 1-clock
  loop, so a finer coefficient needs care at 74 MHz).
- Inset margin from the corner: scale with the width (8 px fixed now).

Tier 3, optional:
- NTSC dot crawl: the catalogue effect named it, but v0.1 never implemented
  it. A frame-alternating chroma/luma crosstalk checker at chroma edges; no
  free control, so it would ride on Bleed's top end or replace a switch.
- A separate curve or contrast for the inset (now the inset is curved or
  identity).
- The top output line replays the previous frame's last line (1-line replay).
- The inset is a horizontally squeezed band of the same rows, not a scaled
  picture: true vertical scaling needs a frame store the HX4K doesn't have.
- Capture defaults and presets from hardware.

## Build

`./build_combo.sh tonecurve` → `out/rev_b/ron/catalogue-combo/tonecurve.vmprog`.

v0.2.0 (2026-10-12, router2 default): all six configs pass. HD Analog 82.1,
HD HDMI 85.6 on **seed 2** (seed 1 stalled the router, a timeout rather than
a timing miss), HD Dual 84.4, SD Analog 84.4, SD HDMI 81.3, SD Dual 85.9 MHz.
4,322–4,345 LC (56%), 24/32 EBR, all `.bin` files unpack. Standalone yosys:
`emulate_transparency` count is 0. TOML-only edits: repack the cached
bitstreams (regenerate program_config.bin, then vmprog_pack.py).

v0.1.0 (2026-09-11): HD 84.7 / 81.1 / 86.8 MHz (analog / HDMI / dual, seed 1),
3,759 LC, 23 EBR.
