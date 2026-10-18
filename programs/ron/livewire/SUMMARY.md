# LIVEWIRE (v1.1.1 — edges as a living material)

**STATUS: FINAL except presets — signed off by Ron 2026-10-09** ("this is
fine I think we can finalize livewire"), HW-validated on v1.1.0/v1.1.1. Ship
build = v1.1.1 (SD HDMI seed 3, all others seed 1). Remaining: the four
presets are hand-written; capture real ones from the device with
`./capture_preset.sh` (Kirlian and Ink Bleed have none yet), then rebuild
per the note below. Five modes on K1: Neon, Kirlian, Spectral, Cel, Ink Bleed.

K1 shows 0–4 on the device, not the mode names. The vmprog carries the labels,
encoded exactly like LZX's stic and Edge Lab. The converter sends 3+-label
knobs as a plain range ("treated as linear for now"), so this is most likely
a firmware display limitation, not a program bug. Ron accepted it as is. Full design and build
history: [DESIGN.md](DESIGN.md) (§17 = v1.0.0 review fixes, §18 = v1.1.x).

## Build (v1.1.1, 2026-10-09) — shipped vmprog

| config | seed | post-route Fmax |
|---|---|---|
| HD Analog | 1 | 81.00 MHz |
| HD HDMI | 1 | 77.71 MHz |
| HD Dual | 1 | 85.85 MHz |
| SD Analog | 1 | 79.98 MHz |
| SD HDMI | **3** (seeds 1–2 stalled the router: timeouts, not timing misses) | 82.01 MHz |
| SD Dual | 1 | 82.97 MHz |

~6835 / 7680 LC (89%), 23/32 EBR. For TOML-only edits (labels, presets),
don't rerun the builder: rebuild a single config with `make ... SEED=<seed
above> "ROUTER=--router router2"` and repack with `vmprog_pack.py --no-sign`.
Earlier: v1.1.0 HD HDMI seed 2; v1.0.0 HD HDMI seed 2; build12 HD Dual seed 2.

## v1.1.4 (2026-10-18): resolution scaling

The halo/trail field is a grid of cells, and its cell width came from a
measured-width switch (`s_meas_w > 1024`): 16 px at 720p AND 1080p, 8 px at
SD. 720p halos and trails were therefore 1.5x wider relative to the picture.
Three more resolution bugs showed up in debug renders of the field alone:

- **Cells in virtual units.** The field engine's cell clock counts 16 virtual
  units, using the timing ID's x step (Q.1: 2 / 3 / 4 per pixel). 1080p keeps
  16-px cells and SD keeps 8-px cells. 720p gets 10.67-px cells (11/11/10
  clocks), so it has 120 columns, the same as 1080p. Cell rows were already
  height-based (17 / 12 / 4 lines).
- **Horizontal interpolation, 720p and SD.** The DDA stepped one corner
  difference per pixel, which assumes 16-px cells. SD's 8-px cells ramped
  only half way and then jumped at every cell edge, a sawtooth halo. The step
  is now the difference times the x step: x2 at SD, x1.5 at 720p. The 720p
  step is precomputed in the cell's spare engine states from two registers,
  because x + x/2 on one signal is a router1 self-add. Its half rounds toward
  zero: a floored −1/2 overshot falling ramps below 0, which drew bright 1-px
  columns at the halo edge.
- **720p vertical lerp overflow.** 720p cell rows are 12 lines, and the weight
  took the raw line index (0..11), so `8 − w` wrapped on the last three lines
  of every row. That drew horizontal bands through the halo. Rows of 10..12
  lines now take 3/4 of the index.
- **Kirlian/Ink filaments** are projected in virtual x/y (Q.1 / Q.8
  accumulators). Their pitch follows the screen height, and the diagonal
  filaments stay at 45° at SD. Before, SD's vertical pitch was in field lines
  (4.4x too coarse).
- Unchanged, at pixel scale: the Sobel window, the K2 pyramid (its coarse box
  is still 16 px at 720p), grain, the EMERGE dither and the 28-px / 3-line
  edge guard.
- 1080p is bit-identical (pixel diff 0, Neon and Kirlian). Standalone +118
  LUT4. dupin 0. `emulate_transparency` 5, the same as before.

Build (v1.1.4, dev flow router2): HD Analog 80.10 MHz (**seed 2**: seed 1
missed timing under machine load), HD HDMI 83.62 (seed 1), HD Dual 76.28
(seed 1); SD Analog 81.58 (**seed 2**: seed 1 stalled the router), SD HDMI 85.80,
SD Dual 73.10 MHz (seed 1). ~7,030 / 7,680 LC (91.5%, was 89%), 23/32 EBR.
HD Dual's margin is thinner than v1.1.1's (85.85). The community port
(router1, `[firmware] seed = 2`) needs re-checking when this is synced.

## v1.1.x changes

- **Kirlian crackle:** on random frames the filaments re-deal their lengths
  and hop 0–3 px, so the corona flickers like a discharge.
- **Neon buzz:** about once a second on average (1 frame in 64) the tube
  stutters 1–4 times: the white-hot core drops out and the glow halves for a
  frame.
- v1.1.0's K1 dip-through-video transition was HW-rejected (bright
  horizontal bars; clean cuts felt better) and removed in v1.1.1. K1 is a
  clean cut with zone hysteresis.

## v1.0.0 changes (from the 2026-10-09 release review)

- K5 renamed "Levels" -> **"Character"**. It now acts in every mode: Neon
  halo hue relative to the tube (50% = the old quarter-turn), Spectral swirl
  speed (still at 0), Kirlian/Ink filament scale, Cel level count, plus S9's
  lit side.
- S10 renamed "Spectral" -> **"Rainbow"**. In Spectral mode it now flips back
  to the flat K6 palette (it used to do nothing there).
- **Invert is a true negative.** Previously the output U/V swap was dropped
  under Invert.
- The hue wheel reads its BRAM again (it had been orphaned and synthesized
  as a LUT mux). It is now 256 steps, so K6 and the swirl turn it smoothly.
- Hysteresis on K1 mode zones and K5 steps, so a knob parked on a boundary
  can't flicker the frame.

## Hardware checklist (used for the v1.1.x sign-off)

1. No field wrap: bright edges and haze at the bottom must not reappear at
   the top (09-20 partial-bottom-row fix).
2. Cel mode (never flashed): posterised flats, black outline, K5 level count.
3. Material (K4) 0% = bare line with no fog, in every mode.
4. P12 sweep 0 -> 100 in each mode: 0 = clean video, 100 = a designed climax.
5. S11 Trails.
6. New in v1.0.0: K5 in Neon (halo hue sweeps around the tube's) and Spectral (swirl
   speed); S10 in Spectral (flat palette); S7 at P12 0 (true negative: skin
   turns cyan-ish).
7. New in v1.1.x: Kirlian crackle (lively or too strobey?), Neon buzz
   (charming or annoying? it can be gated or made rarer), K1 clean cuts with
   no bars.
8. Watch for thin coloured lines across Neon/Spectral colour on HD HDMI. If
   they appear, add the generated-avid fix (cubist/aurora, ~25 LC).

## Remaining after the HW pass

- Presets 4 -> 8, captured with `./capture_preset.sh`. Kirlian and Ink Bleed
  have none yet; also an Invert look and Rainbow over Neon.

## Build seeds — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

v1.1.3 (2026-10-16): **seed 2**, set as `[firmware] seed = 2` in livewire.toml. LZX's stock build uses one seed for all 8 configs.
- **Why:** v1.1.3 adds the labelled-default fix for K1 Mode. With it, seed 1 stalls router1 on HD Standalone (overuse stuck at 13322).
- **Sweep:** seeds 2, 3 and 4 all build 8/8.
- **Own HD timing** (scratch core without LZX's Sync Out generator): seed 2 gives 89.9 / 89.5 / 89.7 MHz (Analog / HDMI / Dual), seed 3 gives 87.5 / 87.7 / 87.5, seed 4 gives 88.8 / 93.6 / 84.1.
- The dev flow (router2) is unchanged.
