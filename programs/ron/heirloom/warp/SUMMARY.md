# Warp — a perspective stripe plane warped into checkers, moiré and loops

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Heirloom #48. Source photo:
`photos/WIN_20230909_23_29_18_Pro.jpg` (1920×1080 capture, 2023-09-09). This replaces v0.1.0 (an r²-bent
stripe field with block shear), which Ron judged too abstract. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_warp/` (`notes.md` has every step).
- **Carrier.** The stripes are isolines of a product field, the phase of an hsync-reset oscillator whose
  frequency is the vertical ramp:
  - above the horizon: θ = (x + 94)(690 − y)/60000, hyperbolic arcs with a 30–110 px period;
  - below it: θ = (x + 94)(y − 305)/96000, a broad '/' fan.
  - Fitted from the structure-tensor orientation map (cos 2Δ = 0.92 top / 0.93 bottom) and the stripe
    period along the normal (`orfit.py`, `nper.py`). The oscillator's phase starts 94 px before the line.
- **Horizon seam.** A strong full-width seam at y 693–699, where the two laws meet. Weaker seams at 403,
  465/473, 585 and 933. Row-to-row shifts are only ±0.3 px, so there is no line tear.
- **Colourizer (channel split).**
  - R is the triangle of the phase. Its histogram is flat (18/17/16/17/18/14 %).
  - Each ~39-px cycle is double-humped: green → orange peak → dip (R ≈ 85) → lavender bump (R ≈ 145,
    B ≈ 167) → green.
  - Relative to the carrier, lavender sits on the R-rising side 57–64 % of the time (falling side
    35–51 %). B is explained 0.47–0.52 (R²) by a symmetric bump of R, and 0.54–0.62 with a 3-px lag.
    The rest is a second, finer grating (~17 px at the top right): the moiré.
  - Full-cycle palette, measured as the most common colour per phase bin (top):
    green (12,166,14) → teal → lavender (101,136,178) → orange (221,126,1) → olive (161,130,44) →
    lavender (97,137,174) → teal → green.
  - Below the horizon the orange is redder and there is violet (122,131,194).
- **Haze.** G falls down the frame: 146 (top) → 140 (540) → 104 (760) → 84 (bottom).
- **Warp.** The checker of steep seam bands (top left), moiré lenses (top right) and the band of hooks and
  loops with dark-green '/' lozenges (y 760–900) are a phase map D(x, y) added to the carrier.
- **Margin.** Black for x < 50, then a green ramp to G 112 at x 200. The stripe edge is x 212 down to
  y 520, then a staircase to x 35 at y 700 and ~20 at the bottom.
- **Dark green.** The (3,43,2) k-means cluster (7.8 %) is mostly the margin. Right of x 260 there is
  1.0 % dark green (the lozenges).

## What it does
1. **Carrier.** P = (x + 94)·f(y), with f = K·|y_h − 5 − y| above the horizon and K'·(y − y_h + 390)
   below it. F per line is computed on a serial multiplier; a 24-bit per-pixel DDA does the rest.
2. **Warp.** + n·D(x, y): a 120 × 136 knot map (16 virtual px × 8 lines, 4 bits, 1/16 cycle),
   interpolated wrap-aware vertically per line and horizontally per pixel. n = 0, 1 or 2 by distance from
   the horizon against P12's two fronts.
3. **Bend.** + the camera luma (Video) or an Osc lattice, × K4.
4. **Colour.** A 64-step full-cycle palette over frac(P), one above and one below the horizon. Then the
   green haze (offsets on Y, U, V per line, × K5), then the margin key.
5. The output stage, and the blanking gate on the generated avid.

**Fit** (`dfit11.py`):
- The map is seeded by coherent demodulation of R against the carrier. This is sign-consistent and
  cannot flatten.
- It is then refined on R, G and B against the full-cycle palette, anchored (0.03) to the demodulated
  phase, with wrap-aware smoothing (2).
- The palettes are re-measured each round as the most common colour per phase bin, never the mean. Means
  turned the earlier fits grey.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Warp** | The warp field: checker, lenses, loop band. 0 % = the clean perspective plane (arcs over a fan); the warp grows out from the horizon; 50 % = the photo; 100 % = a second front doubles the warp into a dense loop field. Glided. |
| K1 Stripes | Stripe density of the plane, ×0.5..×2 (centre = the photo). Glided. |
| K2 Horizon | The horizon seam row, ±256 lines (centre = y 695). Moves where the arcs and the fan meet, and the palette split. Glided. |
| K3 Flow | Stripe drift, bipolar, still at the centre ±8, square law. Default 640 = one period in ~6 s. |
| K4 Bend | How far the camera (or Osc) luma bends the stripe phase: 0..±1 cycle at full luma. Default 100 ≈ ±0.09. |
| K5 Haze | The green haze down the frame: none / the photo (centre) / double. |
| K6 Palette | Warp (the photo) / Ember / Ice / Toxic / Candy / Mono / Neon / Rust: transforms of the fitted palettes, so each keeps the photo's band structure. |
| S7 Seam | On: the photo's horizon seam (two laws) / Off: one through-zero law. |
| S8 Margin | The black/green left margin with its staircase edge. |
| S9 Mirror | The right half mirrors the left (the DDA reverses at the centre). |
| S10 Zigzag | A rolling per-line zig-zag on the stripe phase (24-line period, ±0.15 cycle). |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** camera luma, 2·(Y − 512), × K4 is added to the stripe phase. The stripes bend around the
  picture while the photo's composition stays readable at the default.
- **Osc:** a drifting triangle lattice (640 virtual px × 541 lines) feeds the same path. K4 stays live.

## Engine
- **Per field** (vblank zone: no avid for 1024 clocks):
  - glides (1/8 of the gap per field, deadband 4) on P12, K1, K2;
  - K3 squared on a serial shift-add;
  - carrier gains from 64-entry tables (kd = 2^((K1 − 512)/512)), the horizon, and P12's two fronts
    (1.41·P12 and 1.41·(P12 − 512));
  - the flow, Osc and zigzag phases;
  - the labelled Palette default is read as its index (v < 8).
- **Per line** (kicked at the source avid rise, computing the NEXT line):
  - F on a serial multiplier, then the line-start phase S0 = F·94 (334 at SD) + flow + zigzag;
  - the per-pixel step (×1 / ×1.5 / ×2 for 1080 / 720 / SD);
  - the warp count, the palette half, the haze offsets, the margin edge;
  - the map fill: 120 column values interpolated from two knot rows (even/odd-row ROMs) into a ping-pong
    256 × 16 column-pair RAM (`no_rw_check`).
- **Pixel path** (24 stages): carrier DDA (mirror reversal), column-pair read, horizontal wrap-aware
  interpolation, bend product, P = carrier + n·D + bend, palette ROMs, haze, margin key, then output
  conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma ×13/16) and the blanking gate.
- **Memories** (23 EBR): map 2 × 8192 × 4 (16), palettes 3 × 1024 × 8 (6), column RAM (1).
- **Colours:** in the hardware convention (U = Cr). Luma pre-boosted 16/13; Cb pre-emphasised ×1.08
  against the mandatory chroma filter (measured on the finest stripes at 1080).
- **Contracts:** generated avid (parity-locked); one 24-clock latency in every mode; virtual-1080
  geometry from the timing ID; HD HDMI bar fix before the gate.
- **Verification:**
  - The GHDL 720p render matches the integer model (`imodel.py`) plus output stage to a mean of
    0.59/255 (100 % within ±12).
  - Motion: frame 6 matches the model advanced 5 fields (0.60 against 3.21 un-advanced).
  - Against the photo:

    | | Region means (4×4, max/mean of 255) | \|Cr\| p50/p90 | \|Cb\| p50/p90 | Contrast |
    |---|---|---|---|---|
    | photo | 0 / 0 | 115 / 219 | 113 / 250 | 1.00 |
    | 1080 (model + output stage) | 12.2 / 3.1 | 125 / 211 | 107 / 264 | 0.97 |
    | 720p RTL | 12.3 / 2.9 | 122 / 208 | 101 / 264 | 0.97 |
    | v0.5.0 first cut (proto) | 16.8 / 6.3 | 87 / 192 | 126 / 251 | 0.67 |

## Status
- **v0.5.0 built 2026-10-27, NOT flashed.** router2 (`./build_heirloom.sh warp`).
- **Build seeds:** HD HDMI = seed 4 (seeds 1–3 missed); the other five are seed 1.
- **Fmax (post-route):** HD Analog 79.92, HD HDMI 75.71, HD Dual 81.51, SD Analog 79.59, SD HDMI 79.72,
  SD Dual 79.81 MHz.
- 5440–5468 / 7680 LC (71 %), 23/32 EBR. All bitstreams pass iceunpack.
- `out/rev_b/ron/heirloom/warp.vmprog`, 433,992 B.
- HD HDMI has the thinnest margin; if a rebuild misses there, try router1 before restructuring.

## Deviations from the photo, and why
- **Fine moiré.** The top-right second grating (near twice the carrier frequency, not phase-locked to it)
  is not reproduced. The program has one phase, and that grating could not be fitted as a slow law.
  The top right reads as clean fine stripes without the lens beats.
- **Top-left checker.** Blockier and less regular than the photo's steep seam bands.
- **Knot notches.** Small rectangular steps where the photo's phase changes by more than half a cycle
  between 16 × 8 knots.
- **Colour order.** Lavender vs olive follows one fixed full-cycle sequence; the photo mixes them about
  60/40 per side (its finer B grating).
- **Static warp.** The warp map is a fixed realisation of the photo. Motion comes from Flow, Zigzag,
  Bend and P12.
- **SD (4:3).** SD shows the central 1440 virtual px, so most of the left margin and its staircase edge
  are cropped. 720p and 1080 show the same picture.
- **Chroma.** Chroma uses the 9-tap filter in every mode. At 720p the median |Cb| is 11 % under the photo
  (13.5 virtual px of filter); at 1080 it is within 5 %.
- **Interlace.** No bottom-field offset (field_n is tied high on hardware).
