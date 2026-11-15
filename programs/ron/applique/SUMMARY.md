# APPLIQUE (v0.1.1 — flat mattes cut from geometric patterned cloth)

**STATUS: built 2026-11-14, timing-closed on all six configs, NOT flashed.** The RTL fill was
checked against the integer model (below) and rendered on photos; nothing here has been seen
on hardware. Design notes: [DESIGN.md](DESIGN.md). Controls: [CONTROLS.md](CONTROLS.md).

Ron's brief (2026-11-14): a variation of VENEER (whose textures all came out organic) with
the geometric textured looks QUILT has — "not necessarily the same ones, but that style".

## What it is

VENEER's matte pipeline (flat regions by luma level and chroma class), with every region cut
from a sheet of patterned cloth: 16 patterns x 4 variants (stripes, ridges, checks, plaids,
houndstooth, dots, targets, diamonds, studs, triangles, basketweave, zigzags, bricks, scales,
crosses), drawn as two tones with an embossed edge, in eight sets of eight (Quilt, Weave,
Stripe, Check, Dots, Tile, Deco, Hatch). Scale is continuous, the cloth can drift, regions can
have black seams, and one switch turns every pattern 45 degrees.

`applique.vhd` is a fork of `veneer.vhd` as of 2026-11-14 00:36; the matte code is kept line
for line so VENEER's later fixes can be ported (DESIGN.md section 1).

## Build

`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=1500 ./build_programs.sh ron applique`

v0.1.1 (2026-11-14, router1, **seed 1 on all 6 configs**), 6363–6395/7680 LC (83%),
21/32 EBR:

| Config | Fmax (MHz) | Target |
|---|---|---|
| HD Analog | 89.62 | 74.25 |
| HD HDMI | 86.42 | 74.25 |
| HD Dual | 88.65 | 74.25 |
| SD Analog | 83.57 | 27 |
| SD HDMI | 86.35 | 27 |
| SD Dual | 82.20 | 27 |

v0.1.1 = v0.1.0 with the seam footprint changed from Medium (4 px x 2 lines) to Thin (2 px x
1 line) — Ron's one change after the preview page. v0.1.0 (same flow, seed 1 everywhere):
6405–6435 LC, 22 EBR, HD Analog 86.16, HD HDMI 84.33, HD Dual 88.65; `emulate_transparency`
0 in a standalone synthesis.

router2 (the builder's default) stalled on seed 1 for HD Analog and HD Dual (6 minutes each
before the retry) and gave 75.9 MHz on HD HDMI; router1 places and routes each config in about
35 seconds. Use router1.

Standalone yosys (program + core): 3480 LUT4 + 1035 carry + 2299 FF (VENEER's texture build:
6991 in the same count; the pattern generator is smaller than the noise engine it replaces,
and the seams came in with the difference).

## Verification done

- **RTL against `model.py`**: an 8-band grey card (Fixed thresholds, 8 Levels, Palette, seams
  off) puts all eight entries of a set on screen at once. GHDL renders at 960 x 540 of the
  Quilt, Check and Tile sets (24 recipes) are **identical to the model** inside every band,
  relief included, once the model is shifted 2 pixels right and 1 line up (the first two
  pixels of a line repeat pattern position 0: a constant offset of the whole cloth). The one
  exception is benign: 3 pixels on every fourth row of one diagonal edge differ by one step
  of the colour ramp, because the RTL's chroma smoother carries its rounding residue from
  line to line and the model restarts it at 0.
- **Photos** (re-rendered for v0.1.1): 38 GHDL renders at 960 x 540 and 3 at 1920 x 1080 (bear, portrait, synth
  gradients, test card; hardware-order colours) on the APPLIQUE Previews page,
  https://claude.ai/artifact/QSqqMe6jLjRnXNiSR9XPr9 (runner and page generator in
  `.applique_work/previews/`).
- Not simulated: interlaced timings, 720p, SD (the steps for those come from the timing ID
  table in `p_ctl`; the arithmetic is in `model.steps`).

## First hardware checks

1. Default look on HD HDMI (Quilt, Palette, 5 Levels, seams, embossed). Colours should match
   the previews (red / navy pinwheels on the darkest level).
2. **Thick bright bars** on HD HDMI: this program draws crisp, high-contrast edges and Drift
   moves them. The output has the [1 2 1] conditioning on Y/U/V and chroma never switches at
   pixel rate, but if bars show (try Stripe, Depth 100%, Drift up), widen the chroma filter
   first (CLAUDE.md, wrinkle).
3. K5 Scale sweep: smooth glide, no shimmer when the knob rests, no tearing mid-frame.
4. K6 Drift: steady slide at every speed, still at 0.
5. S10 Relief Embossed against Flat: lit edges on the upper-left side of the light tone, on
   horizontal as well as vertical edges. The first 12 pixels of each line have no relief.
6. S7 Seams in Palette colour: lines only where the cloth changes.
7. Region-edge boil on live video (inherited from VENEER: temporal vote off, 1-code
   hysteresis) — it shows more here because a region that flips level swaps its whole pattern.
8. 720p / SD source: cloth the same size relative to the screen, checks square (DESIGN.md
   section 4 lists what will differ).

## Open choices (made without asking; easy to change)

- **Name** "Applique".
- **K6 is Drift and Relief is a switch** (VENEER has Relief on K6 with four strengths and a
  Light switch). Light is fixed upper-left here.
- **No Ink fill** (VENEER's S7): Depth 0 gives flat value.
- Seam width: **Thin, chosen by Ron** (2026-11-14).
- Presets are hand-written placeholders.
