# Revenant — "Ghost in the Signal"

Analog luma-contour glitch. Re-draws a live source (a portrait on near-black
ground) as a degrading analog signal: bright parts of the figure are redrawn by
swarms of thin electric contour lines that hug the volume, dark parts fall into
true black, the eyes blow out into a solid icy fill + halo, and the whole image
frays / fringes / jitters under the P12 master "signal decay" fader.

Forked from the triangulator engine. **Streaming pixel pipeline, no frame buffer
and no BRAM** — every effect is computed per pixel in the coordinate + luma
domain.

## How the redraw works
The contour lines are iso-lines of a height field
`field = px + (luma >> dispShift) + ripple + rowJitter`. **px dominates, so the
strokes run VERTICALLY** and bow left/right as the luma changes down each column —
thick vertical strokes that follow the form's curvature, with the y-dependent wave
+ row jitter adding chaotic wobble. A stroke is drawn where `field mod pitch <
thickness` (thickness ≈ 3/8 of pitch → fat strokes) **and** `luma > key`, so the
silhouette frays into black exactly where the body ends. Curvature is always on
(Warp/P12 only scale it). Brightest cores (eyes) exceed a high key and flip to a
solid icy fill with a luma-falloff halo.

K2 Line Density sets the vertical pitch (32..8 px); Warp + P12 push the
luma-curvature bend and chaos.

## Motion model
The line pattern is **static** (the multi-octave wobble is a function of `py`
only, and the per-row texture jitter is a deterministic `py` hash) so the strokes
stand still — no constant scroll/shimmer. All frame-to-frame motion comes from a
**glitch random-walk**: only on occasional frames (probability set by P12) does a
saturating horizontal drift accumulate per row, tearing the lines; calm frames let
it settle back toward rest. So at low P12 it's nearly frozen, and P12 up = more
frequent/violent tears.

## Removed / off by default
- **Tracking band removed** (was a full-width horizontal brightness band).
- **Window cascade defaults OFF** (S10) — when on, the outer boxes' offsets ran
  their right edges off-screen, drawing horizontal stroke segments from mid-frame
  to the right edge that chroma-fringed/flashed. Still available on the S10 toggle
  (would want win-edge clamping + draw-after-chroma-sep before re-enabling).

## Controls (P1–P12 mapped to real hardware: 6 pots + 5 toggles + fader)
- K1 Luma Key — where line-fill begins (high = sparse, more black)
- K2 Line Density — contour pitch (power-of-two, denser where brighter)
- K3 Contrast — black-point crush (fixed x1.5 gain)
- K4 Hue Rotate — 90° spins of the violet/cyan/white tritone
- K5 Saturation — shift-add chroma scale (x0..x4)
- K6 Eye Key — top-luma threshold for solid icy fill + halo
- S7 Warp · S8 Chroma Sep · S9 Text (reserved) · S10 Windows · S11 Bypass
- **P12 Signal Decay (HERO fader)** — scales warp + ripple + row jitter + chroma
  fringe + tracking-band activity + line bloom together. 0 = nearly clean stable
  portrait; 1 = fully possessed fraying ghost.

## Pipeline (11 datapath stages + 8 chroma-delay taps, LATENCY 19)
luma crush → contour-displacement → iso-line decision + key gate + eye/halo flags
→ palette LUT + shift-add saturation → hue rotate → colour select → analog
degradation (scanline/tracking/dither) → window membership → window mux + chroma
separation taps → output.

## Status
Built + timing-closed on all 6 configs, **full clock, no divisor**.
Fmax 80.6–101.4 MHz (HD Fmin 74.25), LCs 4878/7680 (63%), 0 BRAM, 0 DSP.
**Not yet HW-tested. No image sim run.**

### Timing-closure notes (HX4K, no DSP)
1. First cut used 4 multipliers (crush, displacement, ×2 saturation) → 44 MHz.
   Replacing all with shift / shift-add got to ~55 MHz (matches the project's
   known "multiplies kill Fmax" lesson).
2. Non-power-of-2 constant divisions in the vsync latch (`/300`, `/146`) synthesize
   as combinational dividers and are STA-timed even though they update once a frame.
   Switched to `/256`, `/128`.
3. **The real wall was the window-cascade overlay** — a wide comparator tree (5
   nested rects × bbox+stroke compares) feeding one register = ~16 ns. Fixed by
   factoring the rectangle test into independent X-only / Y-only membership bits
   computed in one stage and AND/OR-combined the next: 55 → 81–101 MHz.

## Phase 2 (deferred)
S9 Text overlay — scattered ASCII / system-log glyphs via a Python glyph-ROM hook
(like matrix_rain). Toggle + RTL not yet built.
