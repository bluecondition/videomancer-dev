# Halation — summary

Catalogue combo #5 (seed 20260912): bloom / glow / soft focus + magnetic
wobble. v0.1.x (2026-09-10..13) was flashed and liked ("halation is good",
2026-09-11). v1.0.0 is the finalizing pass, built from a review of v0.1.2,
verified on hardware and signed off by Ron on 2026-10-12. Presets are still
the hand-written ones.

## Controls (v1.0.0)

| Control | Does |
|---|---|
| K1 Glow | bloom gain (0–4× the light above threshold) |
| K2 Threshold | luma the blur must exceed to bloom, black .. ~white (64 + 0.81·K2) |
| K3 Radius | blur width 7–255 px, exponential knob (127 px max in SD) |
| K4 Wobble | per-line sine displacement, 0..±255 px |
| K5 Rate | wobble roll speed, bipolar, centre = frozen |
| K6 Period | vertical wavelength of the wobble |
| S7 Warp | wobble moves picture and glow / the glow only |
| S8 Tint | glow neutral / warm |
| S9 Energy | wobble amplitude scaled by each row's mean luma |
| S10 Blend | glow added / screened |
| S11 Focus | sharp picture / the picture replaced by its blur |
| P12 Halo | red halation: the un-warped glow read at ±d and added on top. Rises as a wider glow (gain full at 25%), then splits into twin red ghosts; d = (P12/4)²·width/2¹⁹, up to 1/8 of the width |

## v1.0.0 changes (from the v0.1.2 review)

Bugs:
- The glow was written at the picture's address, but the box blur lags its
  input, so the glow trailed 14 / 6 / 2 px to the right of each highlight
  (Radius high / mid / low), and Soft shifted luma off its colour. The glow
  now has its own write port at its centre column (newest sample − n − 7,
  exact for every width; measured with a register-level Python model).
- Screen subtracted glow-excess × luma rather than glow × luma, so below ~22%
  Glow it darkened highlights (and still did at Glow 0). It is now a real
  screen: y + light·1.125·(940 − y)/1024.
- Luma clamped at 1023 (full scale kills chroma on hardware); now 940.
- Threshold spanned 0–1023 against a blur that never passes ~940: the top 8%
  of the knob gave no glow. Now 64..~895.
- Energy overflowed in SD for rows averaging above ~728 (bright rows lost their
  wobble). Energy is now the row's mean luma, scaled per mode.

Controls:
- Radius was 2 bits (9 / 17 / 33 px; 25–75% of the knob did nothing). Now two
  cascaded running boxes of width n = 4..128 share one 256×16 EBR ring
  ({luma, box-1 mean} written together, read n back); 1/n is an 8-bit mantissa
  plus shift from a 128-entry case table indexed by K3. The running sums are
  re-zeroed in vblank (avid absent 512–767 clocks), where n is also latched, so
  they never drift. The blur input is padded with the line's own inset edge
  pixels (right edge first, then the left edge as the next line's lead-in), so
  glow reaches the borders.
- P12 was a second Glow amount whenever Wobble was 0. It is now the twin-ghost
  red halo above, read through a quarter-rate copy of the glow buffer whose one
  port alternates between the two ghost addresses every clock. This also
  removed the constant 16-px picture shift (and the ~29 px repeated-column band
  at the left edge) that the old current-line halo needed.

Structure / timing:
- Base Y/U/V are written one clock after the input (was 8); the base read runs
  4 clocks behind the glow read so both meet at stage 8. Latency 14, all modes.
- `no_rw_check` on every RAM (the transparency emulation is gone).
- First build missed HD at 63–66 MHz. Fixes: the Screen headroom clamp (two
  compares + subtract had become one LUT-broken carry chain) is now a subtract
  then a sign test; the per-line and per-frame 10×10 multiplies fed straight from
  SPI registers (K4 × energy, Rate², Period²) take locally registered knobs and
  are split hi/lo across two states.

## Build (2026-10-12)

`./build_combo.sh halation` (router2): all six configs on seed 1.
HD Analog 84.3, HD HDMI 80.6, HD Dual 80.1, SD 81.8 / 76.5 / 84.0 MHz;
5,520–5,540 LC (72%), 26/32 EBR. TOML-only edits: repack the cached bitstreams
(regenerate program_config.bin, then vmprog_pack.py) rather than rebuilding.
