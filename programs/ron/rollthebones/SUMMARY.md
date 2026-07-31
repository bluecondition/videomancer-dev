# Roll the Bones — v0.1

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

## Timing (all 6 configs, HX4K, ~82% LC / 18 EBR)

| Config    | Target   | Fmax     | Seed |
|-----------|----------|----------|------|
| HD Analog | 74.25    | 74.35    | 2    |
| SD Analog | 27       | 71.10    | 1    |
| HD HDMI   | 74.25    | 79.63    | 1    |
| SD HDMI   | 27       | 80.06    | 1    |
| HD Dual   | 74.25    | 83.22    | 1    |
| SD Dual   | 27       | 81.39    | 2    |

~6300–6321/7680 LC. HD Analog's 74.35 is just seed variance (the design's real
Fmax is ~80; other HD configs sit 79–83). All at full clock, no divisor.
Packaged: `out/rev_b/ron/rollthebones.vmprog`.

## Status

Built (all 6 configs close, packaged) + sim-verified (default grid, big-die
detail, full Tumble pile, Rainbow). NOT yet hardware-tested. On HW watch: the
100-bit ping-pong BRAM (wide word — pad if corruption appears, cf. inferno vdly
split) and the U/V ink convention.

## Sims

- `sim_008_opt_default.png` … `sim_010_fixed_tumble.png` — default grid, big-die
  detail (pillow dome, gloss sweep, recessed pips), full Tumble scatter with
  shadows. `sim_005_rainbow.png` — per-die Rainbow hues.
