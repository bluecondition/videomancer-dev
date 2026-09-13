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

**v0.3: the pitch is continuous.** A per-pixel phase DDA (`xacc += s_step` each
active pixel, reset at hsync) replaces the power-of-two `field mod 2^n` mask, so
K2 Line Density sweeps the stroke pitch smoothly from **48 px down to 4.9 px**
(`pitch = 65536 / step`, `step = 1365 + 12*K2`). Stroke width comes free as a
constant 3/8-of-pitch compare (`v_ph < 24576`) — no thickness arithmetic at all.

Everything that offsets the pattern (luma bend, wobble, jitter, glitch walk,
drift) is now expressed in **phase** rather than pixels, so its amplitude is
measured in stroke periods and stays visually constant as the density changes.
The luma bend is `lc << 6..9` = 1..8 periods of bow per full luma swing (Warp +
P12), and the 16-bit wrap *is* the contour modulo.

K2 is glide-filtered rather than deadbanded (`k2_f`, ease by `diff>>4`): a DDA
step is enormously noise-sensitive — 1 LSB of pot noise is 12 step units, which
is a third of a stroke pitch of accumulated phase error by the right-hand edge —
and the `>>4` truncation gives an inherent 15-code dead zone when parked while
still easing smoothly when the knob moves.

Warp + P12 push the luma-curvature bend and chaos.

## Motion model (v0.2 — smoothed)
The line pattern is **static in space** (the multi-octave wobble is a function of
`py` only; the per-row texture jitter is a deterministic `py` hash), and all
frame-to-frame movement is a **constant one-way scroll**: one phase increment
added to the whole contour field per frame (`drift <= drift + s_drinc`).
Deterministic and continuous, so the strokes flow steadily in one direction
rather than bouncing. **K5 alone owns it** — P12 no longer scales drift, so the
two knobs don't eat each other's travel.

The scroll runs in **both** motion modes, so K5 is never dead: S9 Glitch only
*adds* the tearing walk on top of it.

**S9 Motion = Glitch** adds the old model on top: occasional frames (probability
set by P12) accumulate a saturating per-row random walk that tears the lines,
then settle back. That per-frame lottery + snap-back was the "jumps every few seconds"
the smooth model replaces; it is now opt-in.

Two other jump sources fixed alongside:
- **Quantised knob decodes had no deadband.** Hue (`K4(9..8)`), saturation step
  (`K5(9..7)`), line pitch (`K2(9..7)`) and every chaos-zone shift (`/256`) flip
  on 1-LSB pot ADC noise whenever the pot sits on a step boundary — each flip
  re-draws the whole picture. All of them now read a deadbanded capture
  (`db_upd`, 24-code hysteresis, refreshed once per line). This also pulls the
  SPI carry chains out of the vsync decision cone.
- **Serrated vsync.** Analog vsync has many edges per field, so the once-per-frame
  block (size latch, glitch lottery, drift step) fired several times per field.
  Now armed by active video and fired once (`vs_armed`).

## Horizontal-bar bug (fixed, sim-diagnosed)
The "horizontal bars on the keyed image" were solid fills on bright regions striped
by the scanline dimmer. Root causes, all removed/changed:
- **Scanline dimming** (dim every other line) striped any solid region into
  horizontal bars — removed.
- **Halo solid-fill** (fired at eyeKey−80) flooded merely-bright areas solid — removed.
- **Denser-when-bright** (halved pitch, kept thickness) pushed line duty toward
  100% (solid) — removed; pitch is now uniform.
- **Eye solid-key** was tested on crushed luma Lc, which the contrast crush clamps
  to full-scale across whole lit areas (indistinguishable from true highlights) →
  huge solid fills. Now keyed on **RAW luma** (`pipe(3).y`) so only genuine
  specular highlights flip to solid.
Also **reduced the wobble amplitude** (`wave >> 3`) so lines read as mostly-vertical
following the luma, not steep diagonals. Verified by GHDL sim (see below).

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
- **K5 Drift Speed** — 0 = frozen, then a **constant one-way scroll** of the
  contour phase on a pseudo-exponential ramp: one stroke width every ~2 min just
  off zero, ~4 s at mid travel, ~7 strokes/second wide open. The field is periodic
  in phase, so the 16-bit wrap is exactly one stroke pitch and the scroll is
  seamless. (Was Saturation; chroma now sits at the palette's native level.)
- K6 Eye Key — top-luma threshold for solid icy fill + halo
- S7 Warp · S8 Chroma Sep · **S9 Motion (Smooth / Glitch)** · S10 Windows · S11 Bypass
- **P12 Signal Decay (HERO fader)** — scales warp + ripple + row jitter + chroma
  fringe + tracking-band activity + line bloom together. 0 = nearly clean stable
  portrait; 1 = fully possessed fraying ghost.

## Pipeline (11 datapath stages + 8 chroma-delay taps, LATENCY 19)
luma crush → contour-displacement → iso-line decision + key gate + eye/halo flags
→ palette LUT + shift-add saturation → hue rotate → colour select → analog
degradation (scanline/tracking/dither) → window membership → window mux + chroma
separation taps → output.

## Status
v0.4 built + timing-closed on all 6 configs, seed 1, **full clock, no divisor**.
Fmax 83.6–93.9 MHz (HD Fmin 74.25), LCs 5319/7680 (69%), 0 BRAM, 0 DSP.
**HW-iterating** — v0.1 was on hardware and read as jumpy/glitchy every few
seconds; v0.2 is the smoothing pass (above) awaiting a flash.

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

## Simulating (headless GHDL)
From `videomancer-sdk/tools/vhdl-image-tester`, source the oss-cad-suite env, then
the CLI runs the VHDL directly (no bitstream build — fast iteration):
```
./.venv/bin/python run.py simulate revenant --image PORTRAIT.png \
  --programs-dir /home/ron/videomancer-dev/programs/ron --preset "Possessed" \
  --output out.png
```
GHDL elaboration needs `libz.so`; only `libz.so.1` exists, so symlink one and
`export LIBRARY_PATH=<dir-with-libz.so>:$LIBRARY_PATH`. NOTE: the sim uses the
opposite U/V convention from hardware, so palette colours render swapped (electric
violet/cyan shows as green in sim) — judge structure, not hue, in sim.

## Phase 2 (deferred)
S9 Text overlay — scattered ASCII / system-log glyphs via a Python glyph-ROM hook
(like matrix_rain). Toggle + RTL not yet built.
