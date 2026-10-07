# Roll the Bones — v1.0

Video re-rendered as a full-screen tabletop of lit six-sided dice. Every cell is
a die: a rounded-corner face carrying 1..6 drilled pips, where the pip count
tracks the local picture (luma or chroma) so the field reads as a halftone
portrait. The dice are lit like cast objects — a pillow-domed top, a glossy
corner reflection, recessed pip dimples with rim glints — and the **Tumble**
fader spills the neat grid into a scattered, rotated, shadow-casting pile.

## What it does

- **Halftone of dice.** Each cell samples the video once (point sample, during
  the previous cell row, into a ping-pong BRAM). The sampled luma — or chroma
  saturation (S7) — is quantised against five thresholds into a die face 1..6.
  For light-bodied inks more (dark) pips fall in the shadows; for dark-bodied
  inks more (white) pips fall in the highlights. Either way the pile reads as a
  positive portrait.
- **Realistic lighting (no per-pixel divide).** The flat die top is domed by a
  single bevel dot `u*lx + v*ly` (bright toward the movable light K4, dark on
  the far side); the extreme lit rounded corner throws a crisp specular; each
  recessed pip gets a directional dimple (near rim shadowed, far inner wall lit)
  plus a rim glint — all from adds, no extra multiplies.
- **Eight inks (K2).** Casino White, Blood Red, Onyx, Ivory, Jade, Gold, plus
  per-die **Rainbow** (each die a wheel hue from its own hash) and **Spectrum**
  (die hue driven by the sampled video chroma). Pip colour auto-contrasts. K3
  Hue rotates the wheel; **Ink=Video** (S10) tints the die bodies with the live
  picture.
- **Zoom (K1)** sets die/grid size (~18..150 px). **Luma Size (S8)** grows
  bright cells' dice and shrinks dark ones. **Table (S9)** is felt green or
  dimmed live video peeking between the dice. **Gloss (K5)** rides matte→hard.
- **Tumble (P12)** — the headline fader. 0 = a clean aligned grid; up = the dice
  shrink to open gaps of table, rotate by a stable per-cell random angle, jitter
  off centre, and cast bottom-right drop shadows — a rolled, spilled pile.

## Controls

```
K1 Zoom      K2 Palette   K3 Hue       K4 Light     K5 Gloss    K6 Contrast
S7 Source(Luma/Chroma)  S8 Luma Size  S9 Table(Felt/Video)  S10 Ink(Painted/Video)  S11 Bypass
P12 Tumble
```

## Architecture

Forked conceptually from **gumball** (per-cell sample → ping-pong BRAM, frame
FSM on one shared multiplier, LATENCY-aligned sync pipe). Square lattice (no hex
offset). 100-bit cell word, ping-pong by row parity:
`{face, sin, jx, jy, R, Rin, P, pr, rc, bY, bU, bV, pol}`.

- **Pip lattice** is a 3×3 quantise (nearest of {-P,0,+P}²) with a 6-entry
  face→9-bit-mask ROM — O(1) per pixel, no "nearest of six pips" search.
- **Octagonal distance** `max(|a|,|b|) + min(|a|,|b|)/2` for both pips and the
  rounded corners — a near-circular metric using only adds/shifts, so the pixel
  path carries **no distance-squared multiply** and stores radii (`pr`,`rc`),
  not squares. Round enough to be indistinguishable at pip sizes.
- Only **four pixel multiplies** survive: 2 small-angle rotation (8×9) + 2 edge
  bevel `u*lx`,`v*ly` (8×8, operands clamped since the die fits its cell).
- **Stable per-cell randomness** from an LFSR reset at vsync and advanced once
  per sample, so each die's angle/jitter/hue is fixed frame-to-frame.
- No pixel-path divide.

### Timing playbook (HX4K, no DSP — 54 → 78 MHz)

The first build closed only ~54 MHz at 94% LC. What moved the needle, in order:
1. **Kill barrel shifters on the pixel path.** Gloss was a variable shift of the
   bevel; folding gloss into the `s_span` *clamp* (fixed shift) removed it. (The
   alternative — folding gloss into `lx` via a mult — *added* ~250 LC and blew
   past 7680; don't.)
2. **Replace distance² with octagonal distance.** Removed 4 pixel + 2 sample
   multiplies at once; biggest single win for both LC and Fmax.
3. **Shrink multiplier operands to what the range needs.** Bevel `u,v` clamp to
   ±127 (die fits cell) → 8×8; rotation `dxj` fits 8-bit, `sin` 9-bit → 8×9.
4. **Booleanise values used only as flags.** `edge proximity` was a signed
   subtract but only ever tested `>0`; a 1-bit `inband` flag removed the
   subtract and a 10-bit×3-stage pipe.
5. **Split the deepest stages, and don't chain flags after clamps.** The `c8`
   colour-assembly cone (40 levels) split into `c7b`/`c8a`/`c8b`; the specular
   flag reads the pre-clamp shade in parallel, not after it. Adding stages here
   *lowered* LC (narrower pipes) while shortening paths.
6. **Narrow sample-pipe arithmetic** (luma-size radius was a 13-bit add+clamp →
   10-bit).

Gotcha logged: a leftover duplicate `c8a_by <= v_by` (stale variable) from a
half-finished refactor silently painted every die green — geometry was fine, so
only a pixel-value check (not the shape) caught it.

Colour convention: constant inks stored **U/V-swapped** (Cb/Cr) to match the
Videomancer hardware output convention (mondrian/CGA/C64 rule); live video passes
through native. Headless BT.601 sims show the inks U/V-swapped — verify geometry
+ luma in sim, trust colour from the palette rule.

## Timing (all 6 configs, HX4K, ~83% LC / 18 EBR) — v0.2

| Config    | Target   | Fmax     | Seed |
|-----------|----------|----------|------|
| HD Analog | 74.25    | 76.75    | 1    |
| SD Analog | 27       | 77.33    | 1    |
| HD HDMI   | 74.25    | 76.56    | 4    |
| SD HDMI   | 27       | 77.70    | 1    |
| HD Dual   | 74.25    | 79.05    | 1    |
| SD Dual   | 27       | 75.73    | 1    |

~6393–6428/7680 LC. All at full clock, no divisor. Packaged:
`out/rev_b/ron/rollthebones.vmprog`. (The v0.2 denoise first regressed HD HDMI
to ~66 MHz by adding `s_dh±7` arithmetic to the timing-critical `p_position`
counter process; precomputing the window bounds in the frame FSM recovered it.)

## v0.2/0.3 fixes (2026-08-24/26)

- **Bypass green-out (HW HDMI, bypass-only).** S11 passthrough had a green cast;
  the dice looked correct. First mis-diagnosed as a chroma-convention issue and
  "fixed" with a U/V swap — WRONG (a true bypass changes nothing; the swap only
  made it weirder). Correct fix (matching SDK `passthru`, which the user cited as
  the reference): bypass is now a **dedicated 1-cycle raw forward of `data_in`**
  (`s_bpass <= data_in`) muxed at the output — byte- and timing-identical to
  `passthru` — instead of tapping the deep dice pipeline (which apparently
  greened out on the HDMI encoder). The dice path is untouched. **Pending HW
  re-confirmation.** Lesson: replicate the known-good reference exactly rather
  than theorise about the encoder.
- **Latency alignment.** Also set `LATENCY=12` so the dice video (c9) and the
  sync tap land at the same delay (they were 1 apart), per prism's same-latency
  rule.
- **Dice flicker / noise.** Point-sampling one pixel per cell flickers when the
  video is noisy (the face flips as luma crosses a threshold). Now the sample is
  an **8-px horizontal area-average** around each cell centre on the sample line
  (memoryless, no BRAM — the same denoise that fixed C64). Steadier faces.
  (The window arithmetic must be precomputed in the frame FSM, not `p_position`,
  or it regresses HD HDMI timing — see below.)

## v0.4 finalization pass (2026-10-05)

From the 2026-10-05 review (A contracts, B controls, C1 roll, C2 sample line,
D word slimming). Not flashed yet.

**Hardware contracts**
- **Blanking gate** at c9: y=64, u=v=512 whenever latency-aligned avid is low
  (felt/dice colours used to run through blanking).
- **Bypass at the same latency.** S11 now muxes raw `pipe(10)` video into c9, so
  every mode presents the one 12-cycle latency (v0.3's 1-cycle bypass was a
  mode-dependent tap). This is the passthru-identical bypass *at a fixed
  latency*; if the old HDMI green cast returns with S11 on, that is new info.
- **Luma in range:** final clamp [64, 800]; bodies White 680 / Ivory 700 /
  Rainbow+Spectrum 600; light pips 760, dark pips 64; shadow = 32 + table/2.
- **Per-die randomness keyed to (row, column):** LFSR reseeded at every hsync
  from a per-row Weyl seed (ACE1 + r*9E37) and stepped 16 bits per die.
  Analog line-length jitter adding/dropping the right-edge partial die no
  longer reshuffles every die below it. Spot-checked: h/v/diag correlations
  < 0.02.
- **First active line shown as table:** row 0's buffer is written on that line
  (iCE40 EBR read-during-write is undefined).

**Controls**
- **K3 Hue** rotates the felt and painted inks (unity at centre, 64 steps) via
  a per-frame rotation on the shared multiplier; on White/Onyx/Ivory it also
  tints the pips (saturation = |K3 - centre|; just off centre = red pips).
  Rainbow/Spectrum keep the old 16-step wheel offset.
- **Auto-level:** the face thresholds sit at a centre that tracks the frame's
  median (up/down census, 12.5% deadband, 2 or 8 codes/frame, ties ignored so
  flat fields settle). Separate luma/chroma centres, so S7 flips instantly.
  Chroma mode was near-dead before (thresholds at 512 vs typical sat 50-250).
  Luma Size also measures from the centre.
- **K6 Contrast** inverted: up = narrower face spacing (sp = 4 + (1023-K6)/8);
  the top end collapses to binary 1/6 dice.
- **Shadows fall away from the light** (quadrant of -L follows K4).
- **Deadband** (+/-4 codes) on K1 Zoom and P12 Tumble.

**Look**
- **C1 Rolling.** From P12 ~63% each die rolls for a per-die window of
  `(P12-640)/2` frames out of every 128 (phase = per-die offset + frame
  count): it spins (1/3/5/7 x 5.6 deg per frame, random direction), shows a
  random face every 4 frames, flies at a fixed size, then lands back on its
  picture face. From ~88% the whole table rolls.
  - Exact rotation without a cosine: pixel path does u = x + y*t, v = y - x*t
    (t = tan, |t| <= 1); the per-die radius is pre-scaled by sec(atan t) in
    the sample pipe as two selected shifts of R (<=2.5% error). A 10-bit
    angle (9 bits/quarter + rot90) spins through 360; rot90 transposes faces
    2, 3, 6.
  - The bevel now uses screen-space offsets, so the light stays put while the
    dice spin.
- **C2:** dice sample the LAST line of the row above (was its centre), halving
  the vertical offset of the portrait (one die -> half a die).

**D: cell word 100 -> 64 bits** (face, rot90, t, jx, jy, R', bU, bV, 7 spare).
Rin/P/pr/rc are slices or one add of R in the pixel path; body Y and pip
polarity are per-frame. EBR 18 -> 14.

**Fit/timing notes from this pass**
- First cut was 99% LC (7637). Ablation priced the new features (hue ~366
  LUT, auto-level ~278, roll ~238, sec 8x7 multiply ~173). Trims: sec as two
  selected shifts; spin as `(phase*speed) mod 32` (only 5 product bits matter,
  phase inversion = free direction flip); rolling radius per frame; sample pipe
  7 -> 5 stages; auto-level as one up/down counter + abs + one centre adder;
  bypass via the c9 mux; 10x11 shared multiplier. Result ~94% LC.
- Critical paths found and fixed: the auto-level update in one vblank state
  (split to one op per state); the luma clamp against 64/800 right after the
  body+shade add (now body+300 precomputed per frame, clamp moved to c9); the
  sin/cos ROM register absorbed into EBR feeding the bevel multiply (second
  fabric register added); the Luma Size radius add+clamp+roll mux (sum moved
  a stage earlier).

**Timing v0.4 (2026-10-05, router2, full clock, 14 EBR):**

| Config    | Fmax  | Seed | LC        |
|-----------|-------|------|-----------|
| HD Analog | 75.13 | 1    | 7265 (95%)|
| SD Analog | 74.59 | 1    | 7285      |
| HD HDMI   | 75.60 | **2**| 7270      |
| SD HDMI   | 81.15 | 1    | 7279      |
| HD Dual   | 77.59 | 1    | 7248      |
| SD Dual   | 84.33 | **2** (seed 1 stalled router2) | 7286 |

HD margins are thin (~1 MHz); if a rebuild misses, seed-hunt or try
`NEXTPNR_ROUTER="--router router1"` before restructuring.
Packaged: `out/rev_b/ron/rollthebones.vmprog` (v0.4.0).

## v0.5 (2026-10-06) — HW feedback on v0.4

Ron flashed v0.4: **Gloss did nothing**, and **not a fan of the rolling on
the slider**.

- **Why Gloss was dead (since v0.1):** the bevel shade only reached ~+/-90 at
  default zoom, but the Gloss clamp ran 70..325, so above ~10% it never bound;
  and the specular needed shade >= clamp-6 while only enabled at K5 >= 60% —
  it could never fire.
- **Gloss rework:** bevel normalised to die size (per-frame shift 3..6 from
  log2 R, so +/-~250 at the edge at any zoom); K5 sets the dark-side depth
  24..279 (lit side = half, for luma headroom); from 25% a white,
  half-desaturated glint (Y = 800) covers the lit-facing edge band where shade
  >= 300 - K5/4, so it grows from a corner spot to a lacquer rim. White/Ivory
  bodies 680/700 -> 600/620 so lit sides and glints separate under the cap.
- **Rolling removed.** P12 Tumble is the static scatter again, but the
  per-die tilt (exact tan+sec rotation kept) now grows as (P12/8)^2/128:
  ~1 deg at the 18% default (as v0.3), ~14 deg at 50%, +/-45 deg at 100%.
- Timing (6/6, ~93% LC, 14 EBR): HDA 80.21 s1, SDA 80.12 **s2** (s1 router
  stall), HDH 76.31 s1, SDH 77.98 s1, HDD 78.21 **s2**, SDD 76.64 s1.

## v0.6 (2026-10-06) — HD HDMI green/noisy input video

Ron, at sign-off: S11 Bypass on HDMI "is not a true bypass" — green cast +
noisy look, and the same look on incoming video in the dice modes (generated
dice fine).

- Ruled out: program logic (bypass was a plain 12-register raw tap with
  matched syncs); core input capture (re-routed hd_hdmi seed 1 reproduces
  76.31 MHz; ADV7611 D/DE pin->FF delays 0.6-1.7 ns vs passthru 0.9-2.8);
  RAM inference of the delay chain (none); the core uses the program's own
  syncs and blanks on its avid, so a uniform delay should be invisible.
- Pattern across HW cases: raw input video carried ~12-15 deep on HD HDMI
  goes green + noisy (Cascade v1.4 at 15 -> fixed at 6; rollthebones v0.2 at
  13; v0.5 at 12), generated content at the same depth is fine. Mechanism
  still unknown.
- Fix/test: S11 = 1-cycle raw forward with its own syncs (== SDK passthru,
  as v0.3); Table=Video backdrop from pipe(4) (6 px ahead of the dice,
  black where the tap has left active video). Ink=Video never used the delay
  chain (samples data_in directly) -> discriminator on HW.
- Timing 6/6 seed 1: HDA 80.08, SDA 76.07, HDH 85.74, SDH 82.20, HDD 80.98,
  SDD 63.61 (27 target); 6940 LC (90%).
- Version 0.6.0; Onyx Grid preset fixed (K2=300). Bump to 1.0.0 once HW-clean.

## v1.0.0 (2026-10-07) — Ink=Video saturation + faded gloss

Ron HW-approved v0.6 ("looks really good now"), then asked for:
- **K3 = saturation when S10 Ink=Video** (16 levels, gain k/8: 0% grey, 50%
  exactly unchanged, 100% ~1.9x; saturates at +/-511). Hue's felt rotation
  and pip tint are neutral in that mode.
- **Gloss edges faded:** the glint is a 16-step strength a = min(ramp past
  the Gloss threshold over 64 shade units, ramp into the rim band normalised
  to the band width). Luma += a*K with K = (800 - body)/16 per frame; chroma
  moves toward neutral by 0, 1/8, 1/4, 3/8 (shifts only). One extra pixel
  stage: LATENCY 12 -> 13 (all modes; video taps unchanged: table pipe(4),
  bypass 1-cycle passthru).
- **Fit:** both first cut = 102% LC (general multipliers). Fix: ONE shared
  per-die multiplier in the sample pipe (tilt, jitter x, jitter y, U gain,
  V gain; one die in flight, strobes >= 18 clks apart) with REGISTERED
  operands (the strobe->mux->mult cone was the critical path), consumers two
  stages after the pick; pack/write at W7. Back to v0.6's LUT count, 92% LC.
- Timing: HDA 79.39 **s2**, SDA 65.60 s1, HDH 77.42 s1, SDH 80.77 s1,
  HDD 81.64 **s3 (hand-installed, solo route)**, SDD 79.87 **s2
  (hand-installed, solo route)**. Builder run died at HD Dual (router2
  stall); vmprog repacked with vmprog_pack.py.

## v1.0.1 (2026-10-07) — faded glint reverted

Ron: remove the heavier gloss changes so it builds without seed hunting.
The faded glint (rim-band ramp, threshold ramp, c8c blend stage, LATENCY 13)
is reverted to v0.6's pixel pipeline (binary white glint, LATENCY 12); K3
saturation under Ink=Video and the shared sample multiplier stay.
**6/6 seed 1, 88% LC:** HDA 83.58, SDA 77.51, HDH 81.77, SDH 79.76,
HDD 82.39, SDD 81.54. (Supersedes the v1.0.0 seed/hand-install notes above.)
The fade itself is in `.rollthebones_work/rollthebones_v100.vhd` if wanted
later (it fit at 92% but needed seeds 2/3 + solo routes).

## Status

v1.0.1 packaged 2026-10-07, 6/6 seed 1. v0.6 look HW-approved; v1.0.1 adds
only Ink=Video saturation on K3 — quick HW look, then FINAL. Presets to
capture from HW.

## Sims

- `sim_008_opt_default.png` … `sim_010_fixed_tumble.png` — default grid, big-die
  detail (pillow dome, gloss sweep, recessed pips), full Tumble scatter with
  shadows. `sim_005_rainbow.png` — per-die Rainbow hues.
