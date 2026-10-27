# Pewter — brushed-metal weave

**v0.1.0 (2026-10-14): built, NOT flashed.** Made from a reference capture: a grey-green
analog-synth frame of brushed-metal tiles woven over lit rods, 6.3 repeats across and 8
different rows. Presets are hand-written placeholders.

## Reading of the reference
Held up in numpy previews (`~/weave_work/proto.py` float, `proto3.py` integer-faithful to the
RTL):
- **Grid.** The frame repeats every **303.2 px** across (6.33 per line) and **584.4 lines** down,
  in **8 rows of 73 lines**. Folding the frame by both periods gives one clean tile
  (`tile.npy`).
- **Rows are fixed waveforms.** Each row is ONE luma waveform repeating along the line. It has
  flat tiles that approach their level with a lag-like sheen, and two transition zones of thin
  bright and dark edges (these read as the "rods" between tiles). Rows 2, 3 and 7 carry a fine
  8 px ripple on some tiles.
  - A resonant-filter fit to 4 segments failed (10 % RMS error). The edge clusters are several
    close steps, not ringing.
  - So the program **stores the measured waveforms**: a per-row median over the 57 core lines
    and all ~12 repeats, resampled to 256 samples.
- **Streaks.** Line-detail residual std is about 5/255 in the mid-tones and 1.6 in the blacks.
  Horizontal correlation is 0.58 at 16 px and 0.39 at 32 px; vertical correlation is about 0
  past one line. So the streaks are noise smeared ~25–30 px along the line, new on every line,
  weaker in the blacks.
- **Edges are stable.** Rod edges do not move from line to line (0 px jitter), so the program
  adds no line jitter.
- **Colour.** The tint is fixed and grey-green, measured as a function of luma: Cb −2 → −14 and
  Cr 0 → −10 from black to white.

## Controls
| Control | Function |
|---|---|
| K1 Rows | Rows per horizontal period, 1–16 (square law). The reference is 4.15 (at 47 %) |
| K2 Sheen | Contrast about mid-grey, ×0.4 (matte) to ×2.6 (chrome, clipped). Unity at 27 % |
| K3 Brush | Scratches, 0 to 3× the reference (1× at 33 %) |
| K4 Drift | Bipolar sideways speed, up to ~1.7 periods/s. Still within ±24 of centre; positive moves right |
| K5 Roll | Bipolar vertical speed, up to ~1.7 rows/s. Positive rolls down |
| K6 Metal | Pewter (measured) / Steel / Silver / Gold / Brass / Copper / Bronze / Verdigris |
| S7 Shuttle | Odd rows sit half a period over and drift the other way (the loom) |
| S8 Mirror | Odd rows play their waveform backwards |
| S9 Shuffle | Hashed row order: no 8-row repeat |
| S10 Twill | Each row 1/8 period further on: diagonal twill |
| S11 Negative | Inverts the luma |
| P12 Zoom | About the screen centre, which is the reference's brightest plate. 16 periods across (fine metal mesh) at 0, the reference at 37 %, 1.3 periods (two huge brushed plates with their rods) at 100 % |

**Why Zoom is the headline:** numpy trials of resonance, wavefold, lag, chevron/herringbone and
twill all failed as a P12 climax.
- Resonance read as noise, or as harsh self-oscillating bars.
- Fold washed the picture out.
- Lag degenerated to grey.
- The offsets barely read, because all 8 row waveforms differ.

Zoom has two designed ends (mesh and macro), and the scratches scale with it.

## Engine
- **Wavetable.** 8 rows × 256 samples (`pewter.py`), in even/odd 1024×8 banks (4 EBR). Samples
  n and n+1 are read on one clock and interpolated by a 4-bit fraction to a 12-bit level.
- **Coordinates.**
  - Horizontal: `phi = phi_c + (x − W/2)·inc` with `inc = 2^24/W · 16 · 2^−x`. `x` is Zoom in
    octaves; `2^−frac` comes from a 256×16 ROM (1 EBR).
  - Vertical: `v = v_c + (y − H/2)·vinc` with `vinc = inc·rows`.
  - The per-field sequencer computes these, plus the centring products `inc·W/2` and `vf·H/2`,
    on one serial multiplier.
- **Per active line** (~19 states after the generated avid's fall), the line engine computes:
  - the row number;
  - the waveform (`k mod 8`, or a hash under Shuffle);
  - the start phase and step (Shuttle, Twill, Mirror);
  - the line's noise seed.
  The hashes run on a small **op-code mix ALU**: the states issue only op codes.
- **Scratches.** Value noise along the UNWRAPPED phase:
  - Knots every 1/16 period. Two parallel hash pipes (knots j and j+1) compute
    `(seed xor j·rot) + j → mix_b → mix_c`, then the knots are lerped by the knot fraction.
  - The seed hashes the row coordinate in 1/128 rows, so scratches ride the weave, scale with
    Zoom and roll with the rows.
  - The hash was spot-checked: neighbour correlations are below 0.005 and the FFT is flat.
  - The noise is weighted by level (black stays black, the brights a little calmer) and
    multiplied by Brush.
- **Colour.** Sheen gain (split into nibbles), clamp, Negative, then luma `64 + 0.859·y`
  (white 940). The metal palette is 8 × 64 luma steps → U/V (2 EBR, stored U/V-swapped).
- **Contracts.**
  - damier's generated avid.
  - 23-stage latency in every mode.
  - [1 2 1] Y/U/V smoothing + 13/16 luma bar fix.
  - Blanking gate.
  - Glides on Rows, Sheen, Brush and Zoom (Zoom eases by 1/4).
  - Per-field work on the first vsync edge after active video.
  - Bottom field one frame line down.

## Build notes
- **v0.1.0 (2026-10-14), router2. Build seeds: HD Analog = seed 2** (seed 1 stalled the router;
  a standalone seed-1 trial earlier had routed at 82.3 MHz). The other five are seed 1.
  - Fmax: HD Analog 78.6, HD HDMI 86.2, HD Dual 82.8, SD Analog 81.0, SD HDMI 81.2,
    SD Dual 81.9 MHz.
  - ~5990/7680 LC (78 %), 7/32 EBR (wavetable 4, palette 2, zoom curve 1).
  - Every bitstream passed iceunpack. Output: `out/rev_b/ron/pewter.vmprog`.
- Headroom: ~1700 LC and 25 EBR are free, with ~5–15 % timing margin. Room for a second
  wavetable set, or for more metals.

## Working files
In `~/weave_work/`:
- `ref.jpg`: the reference.
- `tile.npy` / `tile.png`: the folded period.
- `profiles.png`: the 8 row waveforms.
- `fit.py`: the failed filter fit.
- `noise_stats.py`, `measure.py`: streak statistics.
- `hashcheck*.py`: the hash spot-checks.
- `proto.py`, `proto2.py`, `proto3.py`: previews (`proto3.py` is the integer model).
- `sheet*.png`: control contact sheets.
