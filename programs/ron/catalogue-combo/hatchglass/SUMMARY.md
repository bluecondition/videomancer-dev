# Hatchglass — summary

Catalogue combo #93 (seed 20260924): mirror / kaleidoscope + crosshatch +
luma / saturation key. v0.1.0 (2026-09-12, derived from lattice) was flashed
and liked ("hatchglass is good"). v0.2.0 (2026-10-12) was the first
finalizing pass. v0.3.0 (2026-10-13) adds five Tier 3 picks from Ron's
review. Neither has been flashed yet.

v0.1.0 is in git 3fa438d. For A/B the vmprogs are kept at
`.hatchglass_work/hatchglass_v010.vmprog` and `hatchglass_v020.vmprog`.

## Controls (v0.3.0)

| Control | Does |
|---|---|
| K1 Ink | stroke colour: black ink at 0%; at 100% the picture's own colour (half its brightness above black, 1.5x its saturation) |
| K2 Slide | the mirror lattice slides sideways, bipolar; squared speed curve with a centre detent (to 15 px a frame) |
| K3 Weight | stroke weight in the darkest tones, 9–69% of the pitch |
| K4 Pitch | stroke spacing 4–48 px (32 exponential steps) |
| K5 Depth | the panes multiply: 2 at 0% (a mirror at the screen centre) to 128 at 100%; exponential, glided |
| K6 Soft | key edge softness, 16 half-octave steps |
| S7 Key | luma / saturation |
| S8 Invert | Engrave takes the darks (or greys) first instead of the lights |
| S9 Space | Screen / Glass. Glass mirrors the strokes and the key with the panes (chevrons at every seam, a symmetric key); Screen keeps both on the real picture, so the subject's silhouette cuts through |
| S10 Paper | hatch on white paper / hatch over the mirrored picture |
| S11 Fold | mirror (alternate panes reversed) / repeat (tiles) |
| P12 Engrave | the key level sweeps the picture into hatch: none at 0%, everything at 100%, whatever Soft and Invert are |

## v0.3.0 changes (Ron's Tier 3 picks, 2026-10-13)

- P12 is Engrave, a key-level sweep ("picture dissolving into hatch"). The
  level runs from 8 codes past the light end of the key range (luma
  64..940, saturation 0..640) to 8 codes plus the Soft ramp past the dark
  end, so 0% has no hatch and 100% has it all at any Soft. Invert reverses
  the order. At 0% the hatch controls (Weight, Pitch, Ink, Paper) have
  nothing to act on; Ron accepted that trade.
- Depth moved to K5, where Level was. Same range and glide as v0.2.
- Glass key: a second key chain runs on the mirrored pixel. S9 (renamed
  Space) switches the strokes and the key together. Ron chose sharing S9
  over giving up Fold or Invert.
- Ink (K1, replacing Strip): stroke = black + Ink · ½(y − 64) luma and
  Ink · 1.5 · chroma, from the mirrored picture. Panes now always start at
  the picture centre.
- SD interlace: strokes use the true frame line. field_n toggling between
  fields marks interlace (HD inputs tie it high, so HD is unchanged). The
  line phase steps by 2 per field line, and the bottom field (field_n = 0)
  sits one line down. Diagonals are now 45° on SD and horizontal strokes
  are their real thickness, but 1-px horizontal strokes now show 30 Hz
  interlace flicker (accepted).
- Top row: the field's first row has no row above to replay, and used to
  show the previous field's stale line. It is now drawn from black.

## v0.2.0 changes (from the v0.1.0 review)

(Controls named here are v0.2.0's: K1 Strip, K5 Level, S9 Strokes, P12 Depth.)

Bugs:
- P12 Depth did the same thing as K1 Segment. Nested halving folds collapse
  to one triangle fold of width W/2^depth, so the two controls were 5 and 4
  steps of one parameter (pane width 1024..8 px in powers of two).
- When Depth disabled a fold stage, that stage's register was skipped. The
  picture shifted 0–3 px with each Depth step and the avid gate sat up to
  3 px off.
- Repeat read a strip twice the pane width, starting left of centre. At the
  widest segment that ran off the picture: 576 black columns in every
  2048-px tile on HD, and black at any segment ≥ 512 in SD. Mirror at the
  widest segment also ran off an SD picture (black side bars).
- S8 Invert and S9 Keyed did the same thing (both swap which region gets the
  hatch; with a soft key they differ only by where the ramp sits).
- The tone ladder put all four stroke directions only below luma 128 (near
  black) and none above 768. Most of a picture sat at one or two
  directions, with hard contours between levels.
- Weight was in pixels while Pitch was 4/8/16/32, so at pitch 4 the top half
  of Weight was solid black.
- Saturation key used (|u| + |v|)/2, so the top half of Level was dead in Sat
  mode. Level defaulted to 0, which keyed everything: Key, Invert and Soft
  did nothing at power-on.
- A full key reached only 255/256 of the hatch.

Hardware contracts:
- HD HDMI bar conditioning: highlight knee (600 + (y − 600)/2, capped at 784;
  the paper is ~770, was 940) and [1 2 1]/4 on Y/U/V before the gate. 1-px
  black strokes on white paper, moving with the panes, are the known trigger.
- U/V swapped on rows whose hsync→avid parity differs from the row above.
  Each row replays the row above (drape v1.3 / flutter / benday).
- Read bank latched per line at the avid rise; no_rw_check on all six
  line-buffer RAMs.
- Width = widest line of the previous field; per-field work runs on "no
  avid for 512 clocks", not on vsync.
- One latency (23) in all modes; blanking-gated before and after the
  smoothing.

Controls:
- Mirrors rebuilt as a two-mirror counter (reflector's engine): any pane
  width, centred with a seam at the screen centre. P12 Depth runs from 2 panes
  (the right half mirrored onto the left) to 128 panes (~15 px, reeded glass)
  over six octaves. It is deadbanded and glided, so zooming is smooth and pot
  noise can't shimmer the outer panes.
- K1 is now Strip (Segment moved to P12). It picks the source region and
  reflects at the picture edges.
- S9 is now Strokes: Glass takes the stroke x phase from the mirror counter,
  so diagonals flip at every seam.
- Hatch is tonal engraving. Darkness picks how many directions show (/, \,
  −, |, spread evenly over black..white) and the newest direction thickens
  from 0 to the full weight, so tone is continuous. Pitch is continuous
  (modular counters, any pitch), and Weight is a share of the pitch.
- Slide uses a squared curve with a ±8-count detent. The default (560 → 598)
  keeps v0.1's speed. The top is 15 px a frame (was 4).
- Soft has 16 half-octave steps (was 8 octave steps). Even steps match v0.1's
  gains, so old Soft values look the same.
- Luma Level spans black..white (64 + 0.875·K5). Sat Level is compared with
  |u| + |v|.
- Removed the dead logic inherited from lattice (tint, skew, unused dry ring).

Presets were hand-mapped from v0.1: pane widths, pitch and key kept, and the
Keyed=Picture preset became Invert. Two new presets: Leaded Glass (Glass
strokes, everything engraved) and Reeded Glass (128 panes, strokes over the
picture).

## Build

Build with `./build_combo.sh hatchglass` (router2 default).

v0.3.0 (2026-10-13): all 6 configs pass on seed 1. HD Analog / HD HDMI / HD Dual
82.8 / 85.5 / 81.2 MHz; SD 83.2 / 79.3 / 85.4. ~6,390 LC (83%), 18/32 EBR.
Standalone synth: 0 `emulate_transparency`.
(v0.2.0: ~5,440 LC, HD 85.9 / 87.3 / 81.9 MHz, all seed 1. v0.1.0: 4,153 LC,
HD 83.7 / 77.7 / 74.8 MHz, all seed 1.)

## Open (for Ron)

- Hardware check, especially:
  - P12 Engrave end to end, in both Invert settings and both Key modes
    (0% should be pure kaleidoscope, 100% fully engraved);
  - Space = Glass, now with the symmetric key;
  - Ink at full (the 50% brightness / 1.5x saturation may want tuning);
  - Depth on K5;
  - the black first row;
  - SD diagonals, and how much the 1-px horizontal strokes flicker;
  - from v0.2: tonal hatching, the dimmer paper, the [1 2 1] softening, and
    HD HDMI with a fine pitch + Slide.
- Presets: capture defaults and looks from the hardware.

## Build seeds — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

2026-10-14: **seed 2**, set as `[firmware] seed = 2` in the staged community TOML (the dev TOML is unchanged). Why: seed 1 crashed nextpnr's router1 (assertion) on SD Analog. Sweep 2-6: 2, 3, 5 build 8/8; 4 and 6 hang one or two configs. Own HD logic with seed 2 (scratch core without the Sync Out generator): 90.03 / 91.25 / 94.73 MHz (Analog / HDMI / Dual).
