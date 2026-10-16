# Impasto — summary

Catalogue combo #23 (seed 20260916): value-noise displacement + Kuwahara-lite
oil-paint smoothing. v0.1.x was flashed and liked ("impasto is good",
2026-09-11); the v2 edge clamp (v0.1.3, 2026-09-13) has no hardware
confirmation on record. v0.2.0 is the first finalizing pass, built from a
review of v0.1.3; Ron flashed it and said "looks good" (2026-10-12), then
asked for the colour brush and the vertical brush (v0.3.0).

## Controls (v0.3.0)

| Control | Does |
|---|---|
| K1 Scale | noise cell size 4–512 px, octave steps |
| K2 Drift | noise field scrolls vertically |
| K3 Stroke | noise cells stretched horizontally ×1–×8 |
| K4 Brush | window 4–64 px each side, exponential (0% 4, 50% 16, 75% 33, 100% 64); the vertical brush drags ~N/2 lines |
| K5 Edge | 0 = plain box blur of both windows … 100 = hard nearest-mean pick; also the step that stops the vertical brush |
| K6 Relief | brush-stroke relief (left mean − right mean), up to 2× |
| S7 Noise | Warp: noise displaces the picture / Steer: noise flips the pick |
| S8 Field | smooth value noise / hard cells |
| S9 Pick | nearest mean / farthest mean (haloed oil) |
| S10 Relief | light from the left / right |
| S11 Knife | luma posterized to 8 steps |
| P12 Amount | Warp: displacement up to ±255 px. Steer: share of the picture flipped, none at 0 up to half at 100 |

## v0.3.0 changes (Ron: "do 10 and 11")

- **Colour brush.** U and V get running box sums beside the luma ones. The window
  whose luma mean is nearer the centre (or farther, or the steered one) gives
  its chroma too, mixed toward the two-window average by Edge like the luma.
  Before, only luma was painted and the chroma stayed sharp. Chroma is signed
  (neutral 0) inside the brush, so the floored 1/N mantissa pulls toward grey,
  not toward green. It runs as ONE channel at pixel rate, U on odd columns and
  V on even ones, because the line buffer keeps one U/V pair per two columns
  anyway; the pair is written at the odd column.
- **Vertical brush.** A per-column IIR, state = last line's brushed value (a
  2048×10 luma RAM and a 2048×8 chroma RAM, read-modify-write one line apart):
  state += a·(brushed − state), a ≈ 2/N in half-octave steps (Brush 50% ≈ 8
  lines, 100% ≈ 32), with stochastic rounding from an LFSR so slow steps never
  stick. A step bigger than T = 48 + 3·(255 − Edge) luma codes (T/4 for chroma)
  restarts the column at the new value, so it fills regions with paint dragged
  down and stops at real edges; at Edge 0 it is a plain downward smear. The
  first line of each frame restarts every column. Relief and Knife are applied
  after it, so they stay crisp.
- **Single-bank line buffer** (aurora technique) to make room: column c is
  written ~82 clocks after the read that displays it, so one bank serves both.
  A read displaced ≥ 65 px left sees the current line instead of the previous
  one (invisible); the one read per collision that lands on the column being
  written takes the written data (explicit bypass; the EBR returns undefined
  data there).
- Ring sums are stored pre-shifted by the 1/N shift (shared by both windows):
  each sum ring is 32 bits, 2 EBR.
- Latency 30 → 21 in every mode: the nine fetch-delay stages were padding.
  The avid and steer delays to the centre pixel ride in the centre ring's
  spare bits; the noise is evaluated 13 px ahead so its steer bit is ready when
  the pixel enters the ring.

## v0.2.0 changes (from the v0.1.3 review)

Bugs:
- Relief wrapped on hard edges. The relief term (window difference × Relief / 128)
  reaches ±2038 but was held in 11 bits; at high Relief the strongest edges
  (difference > ~515 at 100%) truncated, which flips a bright relief to dark.
  Now 12 bits.
- Luma left the legal range: relief shadows went below black (codes 0–63),
  highlights up to 1023, and Knife's top step was 960. Now clamped 64–940;
  Knife's top step is 940.
- Scrolling tear in the noise. The noise row coordinate wraps every 4096 lines
  (line + drift phase), but the row after the last one was keyed as a new row,
  not row 0. For Scale ≥ 16 px (default 32 px) that gave a horizontal seam where
  the warp jumped, scrolling up the picture with Drift and on screen about a
  quarter of the time at 1080p. The next-row key now wraps with the same mask.
- Edge stripes. A window reaching past either end of the line summed blanking
  (left: the previous line's held last luma; right: the held value) or the
  capture's black columns, so the outer N px smoothed toward the wrong value
  and Relief drew a stripe down both sides. A window that reaches past the
  12-px inset is now replaced by the other window, so the edges smooth
  one-sided with no relief there. Blanking now feeds zeros (the hold and its
  hold×N reset are gone).

Controls:
- P12 was dead in Steer (displacement zeroed, nothing else). It now sets the
  Steer coverage: the pick flips where the noise is below P12/2, so 0 = plain
  S9 behaviour and 100 = half the picture flipped, in noise-shaped patches. The
  steer bit is delayed 57 px so the mask lines up with the centre pixel (in
  v0.1 it was ~24 px off, and the last ~24 px of each line got a flat mask).
- Brush is continuous: 64 sizes, 4–64 px (was 4 / 8 / 16 / 32). 1/N is an 8-bit
  floored mantissa + shift from a 64-entry table (≤ 0.6% low); the sums are
  pre-shifted so the product is 12 × 8, split on the mantissa nibbles. The old
  step values sit at 0 / 25 / 50% of the knob, so the default is unchanged
  (16 px). 64 px max is sized for 1080p.

Plumbing:
- Centre delay 35 → 67 px (rings 64 → 128 deep, same EBR) for the 64 px right
  window; the write port moves to input stage 13. Output latency is still 30 in
  every mode (the write port is ahead of the line buffer).
- `no_rw_check` on all 10 RAMs (rings read ≥ 1 behind the write; a line-buffer
  bank is only read for the picture while the other one is written).

## Build

`./build_combo.sh impasto` → `out/rev_b/ron/catalogue-combo/impasto.vmprog`.
The v0.1.3 vmprog is kept for A/B at `.impasto_work/impasto_v013.vmprog`.

v0.2.0 build (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 87.5, HD HDMI 78.6, HD Dual 91.5 MHz; SD Analog 88.3, SD HDMI 83.0,
SD Dual 88.5 MHz. About 5,600 LC (73%), 23/32 EBR. v0.1.3 was 81–92 MHz HD
(HD HDMI 78.1) at ~5,000 LC. Standalone yosys: emulate_transparency 5 → 0.

v0.3.0 build (2026-10-12): all 6 configs pass on seed 1. HD Analog 81.3,
HD HDMI 88.5, HD Dual 87.9 MHz; SD Analog 89.7, SD HDMI 91.6, SD Dual 86.0 MHz.
6,127 LC (80%), 25/32 EBR. The v0.2.0 vmprog is kept at
`.impasto_work/impasto_v020.vmprog`.

HW status: v0.2.0 flashed, "looks good" (Ron, 2026-10-12). v0.3.0 flashed: "Looks really good on hardware" (Ron, 2026-10-12) — HW-validated.

Presets: hand-written (Default, Wet Canvas, Palette Knife, Halo Oil). Wet Canvas
now gets the 64 px brush (K4 1023). Palette Knife's P12 1023 is now meaningful
(Steer at half coverage). Recapture with capture_preset.sh when the look settles.
