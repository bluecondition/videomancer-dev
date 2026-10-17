# Revenant — "Ghost in the Signal" (v1.0.0)

**STATUS: FINAL except presets — signed off by Ron 2026-10-10** ("let's finalize").
v1.0.0 = the v0.6 bitstreams repacked with the version bump (bitstreams
byte-identical, md5-checked). Remaining: the three presets are hand-typed
placeholders. Capture real ones from the device with `./capture_preset.sh`
(`--defaults` first), then rebuild.

Analog luma-contour glitch. Re-draws a live source (a portrait on near-black
ground) as a degrading analog signal: bright parts of the figure are redrawn by
swarms of electric contour strokes that hug the volume, dark parts fall into
true black, the eyes blow out into a solid icy fill, and the whole image
frays / fringes / jitters under the P12 master "signal decay" fader.

Forked from the triangulator engine. **Streaming pixel pipeline, no frame buffer
and no BRAM** — every effect is computed per pixel in the coordinate + luma
domain.

## v0.6 (2026-10-10) — Tier 3: signal quality
v0.5 was HW-checked by Ron ("all things check out good"); v0.6 is the look-polish tier.
- **Thin-stroke hue (4:2:2 pair averaging).** The core's packer point-samples U
  on even and V on odd avid pixels, so a 1–2 px stroke on black (K2 above ~55%
  makes strokes < 3 px) reached the encoder with one chroma axis (bishop's
  red/yellow-on-blue). The chroma delay lines now hold pair averages: `uf(k)` =
  U averaged with the NEXT pixel, `vb(k)` = V averaged with the PREVIOUS one.
  Whichever pixel the packer samples carries its pair's mean — no parity
  tracking and no latency change (the chroma delay line gives the lookahead
  for free). Luma stays per pixel.
- **Luma pre-filter.** A 4-tap horizontal box on the source luma before the
  crush (×1.5) and bend (up to 510 phase/code) that amplify analog noise into
  crawling key edges and bends. The eye key, halo key and source-avid content
  gate are re-aligned to the box centre (3 taps); the picture shifts 1.5 px,
  which is invisible. LATENCY unchanged at 27.
- **xorshift32 grain** with disjoint bit fields (grain 7..2, walk 18..12,
  lottery 31..22). The old Galois LFSR shifted one bit per clock, so neighbouring
  pixels got shifted copies of the same bits (diagonal grain).
- **Halo is back, as STROKES.** Raw luma within 64 codes below the K6 eye key
  recolours strokes an icy cyan (Y 740), so the solid eye fill gets a ring of
  icy strokes. It is never a fill (a halo fill flooded lit areas in v0.1).
- K3 Contrast left as is. With P12 and S7 now driving the bend continuously, a
  K3 "Bend" knob would fight P12 for the same parameter.
- (The row-jitter hash fix listed in Tier 3 already landed in v0.5.)

## v0.5 (2026-10-10) — finalization pass: hardware shell + controls
From the finalization review (Tier 1 + Tier 2). v0.2–v0.4 were never flashed,
so v0.5 is the first hardware look since v0.1.

**Hardware contracts (back-ported from glitchscape / rollthebones, both HW-validated):**
- **Blanking gate** — output is 64/512/512 outside a **generated, hsync-parity-locked
  avid** (Revenant draws its own colour; aurora showed thin red lines on HD HDMI
  with the source avid). Content is also gated by the source avid before the
  chroma-sep taps, so the fringe pulls neutral at the line ends.
- **No raw video carried deep.** The sync pipe holds syncs only; the eye key
  compares raw luma at s1. **S11 Bypass = SDK 1-cycle passthrough** (the
  19-deep raw path was the HD-HDMI green trap).
- **Luma contract** — black 64 (was 24), coloured strokes capped at 768, the
  near-white top zone at 900, eye fill 880 (was 990); bloom brightens up to the
  cap instead of clipping. Grain is floored at black.
- **Windows removed** (S10). It was dead on hardware — `max_py` was never
  written, so the frame height latched 0 and every window's top edge wrapped
  past row 3796 — and it was ~40% of the logic (692 LUT / 637 carry / 555 FF,
  standalone yosys ablation).

**Controls:**
- **K6 Eye Glow** (was Eye Key): 0 = off, then the raw-luma threshold falls
  960 → 642 over the whole knob. The old default 950 sat above legal white (940)
  so the eye fill never fired.
- **Deadband rail snap** — within 24 codes of either end the K4/K5 captures read
  exactly 0 / 1023, so K5 = 0 really freezes (a slow sweep used to park 1–24
  codes short and creep).
- **P12 is continuous** (was 4 coarse `/256` zones = 3 visible jumps), glided
  `diff>>3` per field. Bend is a per-pixel `lc * g8` (two 10×4 halves); ripple,
  row jitter and glitch walk are per-line products in a free-running chain
  latched at hsync.
- **S7 Warp** — Off = base bend (1 period per full luma swing); On = 3–8
  periods with P12 riding it. Was a no-op below P12 25%.
- **S9 Glitch** — 1 in 32 frames at P12 = 0, ~28% at full (was dead at 0).
- **S8 Chroma Sep** — 3–15 px at HD, 2–8 px at SD (was 1–8 px everywhere;
  1 px is invisible at 1920 with 4:2:2 chroma). CHR_MAX 16.
- **P12 = 100% climax** — row jitter capped at ±¼ stroke period (was ±1 full
  period = contours dissolved into per-row noise). The row hash is now a real
  mix (glitchscape's, 11-bit); the old `py xor` hash was a regular sawtooth.
- **S10 Invert** (new): strokes fill the DARK (`lc < key`) with the palette
  ramp flipped, so the figure becomes a black silhouette cut out of a bright
  line field; eyes still glow inside it.

## How the redraw works
The contour lines are iso-lines of a height field
`field = xPhase + luma*bend + ripple + rowJitter`. **px dominates, so the
strokes run VERTICALLY** and bow left/right as the luma changes down each column —
thick vertical strokes that follow the form's curvature, with the y-dependent wave
+ row jitter adding chaotic wobble. A stroke is drawn where `field mod pitch <
thickness` (thickness ≈ 3/8 of pitch → fat strokes) **and** `luma > key`, so the
silhouette frays into black exactly where the body ends (S10 Invert flips the
key). Curvature is always on (Warp/P12 only scale it). Brightest raw-luma cores
(eyes) exceed the K6 threshold and flip to a solid icy fill (the halo branch is
plumbed but unused).

**v0.3: the pitch is continuous.** A per-pixel phase DDA (`xacc += s_step` each
active pixel, reset at hsync) replaces the power-of-two `field mod 2^n` mask, so
K2 Line Density sweeps the stroke pitch smoothly from **48 px down to 4.9 px**
(`pitch = 65536 / step`, `step = 1365 + 12*K2`). Stroke width comes free as a
constant 3/8-of-pitch compare (`v_ph < 24576`) — no thickness arithmetic at all.

Everything that offsets the pattern (luma bend, wobble, jitter, glitch walk,
drift) is now expressed in **phase** rather than pixels, so its amplitude is
measured in stroke periods and stays visually constant as the density changes.
The luma bend is `lc * g8 * 2` (v0.5; was `lc << 6..9`) = 1 period of bow per
full luma swing with Warp off, 3..8 with Warp on (P12 riding it), and the 16-bit
wrap *is* the contour modulo.

K2 is glide-filtered rather than deadbanded (`k2_f`, ease by `diff>>4`): a DDA
step is enormously noise-sensitive — 1 LSB of pot noise is 12 step units, which
is a third of a stroke pitch of accumulated phase error by the right-hand edge —
and the `>>4` truncation gives an inherent 15-code dead zone when parked while
still easing smoothly when the knob moves.

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
  huge solid fills. Now keyed on **RAW luma** (s1, v0.5) so only genuine
  specular highlights flip to solid.
Also **reduced the wobble amplitude** (`wave >> 3`) so lines read as mostly-vertical
following the luma, not steep diagonals. Verified by GHDL sim (see below).

## Removed
- **Tracking band** (was a full-width horizontal brightness band).
- **Window cascade** (v0.5) — dead on hardware and ~40% of the logic; S10 is Invert now.

## Controls (6 pots + 5 toggles + fader)
- K1 Luma Key — crushed luma above which strokes appear (high = sparse, more black)
- K2 Line Density — continuous stroke pitch, 48 px → 4.9 px (glide-filtered)
- K3 Contrast — black-point crush (fixed ×1.5 gain)
- K4 Hue Rotate — 90° spins of the violet/cyan/white tritone
- K5 Drift Speed — 0 = frozen, then a constant one-way scroll of the contour
  phase on a pseudo-exponential ramp: one stroke every ~2 min just off zero,
  ~4 s at mid travel, ~7 strokes/second wide open. Seamless (the 16-bit phase
  wrap is one stroke pitch).
- K6 Eye Glow — 0 = off; up = more of the brightest raw highlights flip to a
  solid icy fill (raw-luma threshold 960 → 642), ringed by icy strokes in the
  64 codes below it
- S7 Warp · S8 Chroma Sep · S9 Motion (Smooth / Glitch) · **S10 Invert** · S11 Bypass
- **P12 Signal Decay (HERO fader, continuous)** — warp bend (with S7), ripple,
  row jitter, glitch walk + rate, chroma fringe, grain and stroke bloom together.
  0 = nearly clean stable portrait; 100% = fully possessed fraying ghost.

## Pipeline (11 datapath stages + 16 chroma-delay taps, LATENCY 27, all modes)
s1 black-point sub + raw-luma eye key → s2 crush ×1.5 + line offset into the DDA
→ s3 crushed luma → s4 bend product halves → s5 bend phase + key test + palette
zone → s6 stroke decision → s7 palette → s8 hue rotate + bloom → s9 colour select
(capped) → s10 grain → s11 source-avid content gate → chroma-sep taps → output
(blanking-gated on the generated avid). v0.6 adds an s0 luma box filter ahead
of s1 (no latency change) and pair-averages the chroma lines at s11.

## Status
v1.0.0 FINAL except presets (signed off 2026-10-10). Bitstreams built as v0.6,
**6/6 seed 1 (router2), full clock, no divisor**: HD Analog 87.9 · HD HDMI 87.5 ·
HD Dual 87.7 · SD Analog 79.3 · SD HDMI 79.1 · SD Dual 87.4 MHz.
LC 4885–4904/7680 (64%), 0 BRAM, 0 DSP. **Build seeds:** all configs seed 1.
HW history: v0.1 flashed (jumpy); v0.2–v0.4 never flashed; v0.5 HW-checked OK;
v0.6 signed off.

### Timing-closure notes (HX4K, no DSP)
0. (v0.6) The first v0.6 build put HD HDMI at 74.8 MHz (seed spread 74.8–83.8).
   The critical path was the K4/K5 deadband compare feeding the hsync-gated
   enable (3.5 ns logic, 9.9 ns routing); the rail snap had deepened it. Fix:
   compute `db_upd` free-running into `db_k*_n` (one subtract for both
   directions) and only latch at hsync. HD HDMI seed spread is now 81.3–89.2.
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

## Dropped
The Phase 2 text overlay is dropped: there is no free switch for it, and
overlays are the rejected genre.

## Build seeds — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

v1.0.2 (2026-10-17): **seed 2**, set as `[firmware] seed = 2` in revenant.toml. LZX's stock build uses one seed for all 8 configs.
- **Why:** v1.0.2 adds the labelled-default fix for K4 Hue Rotate. With it, seed 1 stalls router1 on HD Standalone (overuse stuck at 2576).
- **Sweep:** seeds 2, 3 and 4 all build 8/8.
- **Own HD timing** (scratch core without LZX's Sync Out generator): seed 2 gives 98.7 / 97.6 / 102.0 MHz (Analog / HDMI / Dual).
- The dev flow (router2) is unchanged.
