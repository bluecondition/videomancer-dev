# Noisetone — summary

Catalogue combo #137 (seed 20260927): value-noise displacement + gamma /
S-curve tone curves. v0.1.0 was flashed and liked ("noisetone is good",
2026-09-13), with no later hardware notes. v0.2.0 (2026-10-13) is the first
finalizing pass, built from a review of v0.1.0. It is **not flashed yet**.

Rollback: v0.1.0 is in git 3fa438d, and its sources + vmprog are in
`.noisetone_work/v010/`.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Scale | noise cell size 4–512 px, log, continuous (64 steps an octave); the field zooms about the screen centre. Display 0–100% |
| K2 Amount | displacement, square law, up to ±⅛ of the width (±240 px in HD) |
| K3 Drift | vertical scroll, two-way: right of centre the field rises, left it sinks, up to 16 lines a frame (unchanged) |
| K4 Curve | continuous sweep through 8 curves, neighbours blended: gamma 0.5, 0.7, 1.5, 2.2, inverse S, soft S, hard S, crush (display 0.0–7.0; default 5.0 = soft S) |
| K5 Contrast | gain about the pivot before the curve, 25–420% (unchanged) |
| K6 Pivot | grey level the contrast pivots about; it lands on mid grey, so it also sets the exposure (unchanged) |
| S7 Patchy | one exposure everywhere / the noise field shifts the pivot ±256 codes (unchanged) |
| S8 Displace | all channels / luma only (unchanged) |
| S9 Stretch | cells 4× tall / 4× wide (unchanged) |
| S10 Field | smooth value noise / hard cells (unchanged) |
| S11 Chroma | saturation flat / follows the contrast gain (unchanged) |
| P12 Storm | displacement (+⅛ of the width, linear) and contrast (+2×) climb together |

At Amount 0 and Storm 0 there is still a ±5.6 px (HD) floor, so Scale,
Drift, Stretch, Field and Displace always show.

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **Drift tore the noise field.** The row coordinate wrapped every 4096
  lines, but the lattice key did not wrap with it, so a horizontal tear
  line scrolled through the warp about a quarter of the time in HD whenever
  Drift was off centre. The row index now wraps at 1024 cells, and the key's
  hash input wraps with it (Tremor had the same bug).
- **The noise repeated diagonally.** Row keys were a one-round hash, so each
  row of lattice points was an exact copy of the row above shifted ~32
  cells (97% of rows; checked numerically). At small and medium Scale the
  pattern visibly copied itself down the screen on a slant. Row keys now
  take all three hash rounds: visible shifted copies drop to ~3% of rows,
  at random offsets.
- **The curves posterized and lifted black.** The 128-entry ROM was indexed
  by luma / 8 and held bin centres: at most ~110 output levels, steps of up
  to 59 codes in the shadows on gamma 0.5, and black lifted to 123 on
  gamma 0.5 (84 on gamma 0.7, 102 on inverse S). Luma now goes through a
  512-entry transfer LUT that a vblank builder fills, interpolating linearly
  between point samples (Tonecurve v0.2's builder and ROM; the formulas
  reproduce v0.1's table exactly). Black stays 64; gradients are smooth.
- Categories "Distortion" and "Colorizer" are not in the category list:
  now Warp, Noise, Curve. The description promised Storm "boils"; it never
  did (only Drift moves the field). Reworded; a real boil is in Tier 2.

Controls (bold, always-on):
- **Scale** was 8 octave steps (each 128-count zone did nothing) under a
  linear "4–512 px" display that didn't match the steps. It is now x/y
  lattice DDAs at a per-frame step from a 64-entry 2^(−i/64) ROM plus an
  octave shift (Tremor v0.2's engine), so it zooms smoothly, and about the
  screen centre rather than the top-left corner.
- **Curve** was 8 steps; it now blends between neighbouring curves, so every
  position is a different curve. Inverse S moved from slot 6 to slot 4, so a
  sweep runs bright → dark → flat → contrasty → crushed (as in Tonecurve).
- **Amount + Storm** were summed and clamped at ±255 px: above Amount ≈ 50%,
  Storm added no displacement, and the reach was fixed pixels (13% of an HD
  line, 35% of an SD line). Each now adds up to ⅛ of the width (Amount on a
  square law for finer low-end control, Storm linear), with the floor above.

Hardware contracts:
- Per-row U/V parity fix (drape v1.3): each row replays the row above, so
  rows whose hsync→avid parity differs from that row's get U/V swapped back.
- HD HDMI bar conditioning (damier v0.4.4) before the gate: [1 2 1] on
  Y/U/V and luma (13y + 192)/16 (64 stays, 940 → 775). Crush and hard S at
  high Contrast/Storm make full-swing edges that move with the noise — the
  bar-trigger profile.
- `no_rw_check` on every RAM (a line-buffer bank is only read while the
  other is written; the LUT is written only in vblank).
- The read bank is latched per line at the avid rise. v0.1 switched it at
  hsync, which in PAL's 12-clock front porch came before the last 3 pixels
  had read.
- ±1 hysteresis on the measured width (it now sets the amplitude and the
  centring, so analog line-length wander would jitter the field).
- Latency 28 → 30 clocks, the same in all modes.

Defaults and presets were translated to keep v0.1's HD cell size, amplitude
and curve: K1 384 → 439 (32 px), K2 200 → 467 (±50 px), K4 512 → 731 (soft
S). Heat Shimmer K1 293 / K2 296 / K4 0; Patchy Print 732 / 362 / 1023;
Colour Slip 439 / 661 / 876. P12 values are unchanged (Storm's reach is now
⅛ of the width, 240 px in HD vs v0.1's 255).

## Hardware checks for v0.2.0

- Turn on Drift and watch for ~30 s: no horizontal tear line should cross
  the warp (v0.1 tore ~¼ of the time).
- Small Scale (K1 ~20%), Field = Blocky: the cells should look random, with
  no slanted copy of the pattern repeating down the screen.
- K1 sweep: a smooth zoom about the centre (v0.1 jumped in 8 octave steps
  toward the top-left).
- K4 sweep: smooth morph through all 8 curves; gamma 0.5 keeps black black;
  no banding in gradients.
- Amount 0 + Storm 0: a faint ±5 px shimmer remains (intended). Storm at
  100% with Amount at 100%: ±¼ of the width.
- HD HDMI at high Contrast + crush + Storm: no thick bright bars. Whites now
  peak at ~775 — say if it reads dim.
- Presets are hand-translated; re-capture on hardware when dialled in.

## Review backlog (not done)

Tier 2, worth doing after hardware feedback (look changes, Ron's pick):
- **Real Storm boil**: Tremor v0.3-style row slides — each lattice row
  slides sideways at its own random speed, driven by Storm — so P12 at
  100% is a churning field even with Drift at centre (~+150 LC). This is the
  slider-climax rule; today Storm only scales.
- Storm climax: blend the curve toward crush over the top quarter of the
  slider, so 100% is a designed boiling high-contrast scene at any K4.
- Contrast: log knob with unity at the centre (unity now sits at 19% of
  the travel, K5 = 192).
- K6 Pivot: it acts as exposure (at unity contrast it is a brightness knob
  that blows out at both ends). Option: relabel "Exposure", bipolar display.
- S11 Chroma = Contrast reaches 6.2× saturation at full Storm (neon clip).
  Option: cap at ~2.5×, or follow the curve's slope instead.
- Drift is vertical only; a horizontal or diagonal component for flowing
  heat-haze.
- SD: cell size is fixed pixels, so at SD the cells are 2.7× larger relative
  to the picture (displacement is width-relative now). Scale the step by
  the width.

Tier 3, optional:
- A second noise octave (fBm) for a richer texture. Costly: +~1,200 LUT in
  Tremor's trial.
- A square-cell option (Stretch is 4:1 either way).
- The top output line replays the previous frame's last line (1-line
  replay; not visible in practice).
- Capture defaults and presets from hardware (`./capture_preset.sh`).

## Build

`./build_combo.sh noisetone` → `out/rev_b/ron/catalogue-combo/noisetone.vmprog`.

v0.1.0 (2026-09-13): HD 76.5 (seed 2) / 76.0 / 80.1 (seed 2) MHz (analog /
HDMI / dual), 4,865 LC (63%), 21/32 EBR.

v0.2.0 (2026-10-13, router2 default): all six configs pass on seed 1.
HD Analog 77.1, HD HDMI 79.6, HD Dual 83.1 MHz; SD Analog 82.2, SD HDMI
79.7, SD Dual 80.2 MHz. 6,363–6,381 LC (83%), 23/32 EBR (10 luma + 8 chroma
line buffer, 3 curve ROM, 2 LUT); all `.bin` files unpack. Standalone yosys:
`emulate_transparency` 0 (v0.1: 3). The +1,500 LC is mostly ~600 more FFs
(lattice DDAs, sequencers, smoothing window) plus the LUT builder. A copy of
the vmprog is in `.noisetone_work/v020/`.
