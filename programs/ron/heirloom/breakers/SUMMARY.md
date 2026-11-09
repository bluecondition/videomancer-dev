# Breakers — wavy blue ribbons on cyan, green surf eating them, a lagoon with a sinking eye

**v0.1.1 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20230214_23_42_08` at 0.5 s
(`photos/vid/WIN_20230214_23_42_08_t0p5.png`). Two wavy vertical ribbons (deep blue core, pink and white edges)
swing left and right down a saturated cyan sea. From the left, green "surf" eats them: fine dark-teal, green, mint
and lime stripes that follow the same waves, torn into streaks. The bottom third is a flat cyan lagoon with blue
dashes beside the surf, and an eye of ribbon colours swells on the right edge just above it. Cyan dropout rows and
slipped slabs tear the ribbons. In the clip the waves roll down once a second, the lagoon edge sinks and snaps back
at the same rate, and a new eye is born at the top right each cycle and sinks into the lagoon. Presets are
hand-written placeholders.

## v0.1.1 (2026-11-09, review fixes)
- **K5 Grain made obvious.** It was near-dead on the sheet (0 / 512 / 1023 looked alike).
  - Grain now scales the surf's jitter depth linearly: 0 at Grain 0, the clip's at 512, 12× at 1023. The per-field
    base and cap of the depth ramp are multiplied by 0 .. 1 .. 12.
  - In the bottom quarter (Grain < 256) the shore's noise jitter is off.
  - Above 512 a growing share of slabs (up to every slab at 1023) slips sideways by up to ±0.5 band.
  - Grain still sets the streak length and slab height (8× shorter .. 8× longer).
  - Result: 0 = clean, smooth surf stripes; 512 = the still; 700 .. 1023 = deep shredding, long smears, slabs
    offset sideways across the left half.
- **Camera bend softened.** Default Etch 80 (was 200): on selfie.jpg the ribbons bend smoothly instead of fraying.
  The Osc multiplier went from 4× to 10× the camera's, so the default Osc bend is unchanged.
- **Default frames bit-exact with v0.1.0.** The RTL's raw 10-bit output with a black input and with Source = Osc
  matches v0.1.0 on all 921,600 lines (720p60 dec 1 sims), and the integer model agrees (0 differing pixels).
- Single-row check: 0 runs of 24 px or more at 1080p at the default, Grain 0 / 700 / 1023, Grain 1023 with P12 at
  100 % and Osc. At 720p it is 0 except Grain 1023 with P12 at 100 % (4 runs of at most 26 px in one field).
- 7003–7035 LC (v0.1.0: 6940–6968).

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_breakers/` (`notes.md`, `palband.py`, `fit2.py`, `motion1.py`..`motion3.py`,
`proto.py`, `imodel.py`, `sheet.py`, `thincheck.py`).
- **Palette** (k = 14, whole still): cyan (6,253,253) 32 %; greens (48,247,168) 9.7 %, (51,244,101) 7.8 %,
  (124,240,134) 7.5 %; teal (18,187,143) 5.4 %; light cyan (66,248,245) 5.2 %; dark green (24,175,84) 5.0 %;
  pale cyan (134,247,239) 4.9 %; lavender (183,185,235) 4.4 %; blues (0,72,244) 4.1 % and (2,125,238) 4.1 %;
  white (212,249,243) 3.8 %; dark teal (7,109,81) 3.3 %.
- **Colouriser.** One field read through a fixed colour cycle (an RGB colouriser): G is a wide soft trough (the
  blue core), R has narrow peaks (pink at ±0.16 of a band from the core, white at ±0.37), B is saturated. One band
  period was measured along the fitted field in 1/128 steps (`BAND` in `breakers.py`): blue core (0,59,245) → pink
  (214,185,237) → cyan → white (228,255,240) → cyan → white → pink → blue.
- **Ribbons.** Both bands' G troughs (582 points, rows 30..716) fit one chirped sine, rms 6.7 px:
  x_k(y) = M(y) + A(y) sin(2π (y/420 + 0.0005 y²/1080) + 0.916) + k S(y), with M = 1062.8 − 109.3 yn − 469.2 yn²,
  A = −152.6 (1 − 0.70 yn), S = 440.2 − 113.5 yn + 427.5 yn² (yn = y/1080). The local period is 446 px at the top
  and 341 px at row 650, the amplitude 153 → 83 px, the spacing 433 → 528 px. Band 0's core sits at x 1136 on
  row 230 (the wave's right extreme) and x 839 on row 430.
- **Open water.** Flat cyan past f = 1.5 band, with one faint darker contour at f ≈ 1.8 (the "(" arcs at the right).
- **Surf.** Wavy stripes parallel to the ribbons, spaced ~110 px near the shore and ~60 px at the far left
  (surf phase 3.2 d + 0.45 d² cycles, d = shore − f), broken into streaks: horizontal autocorrelation 0.91 / 0.71
  / 0.33 at 4 / 8 / 16 px, vertical 0.93 / 0.83 / 0.71 / 0.60 at 1 / 2 / 4 / 8 rows. Mean RGB (55,217,125), sd
  (48,45,42). The shore runs about 0.3 band left of band 0.
- **Lagoon.** Edge at row ~748 in the still; blue dashes 25..50 px long on a 64..68 px pitch (horizontal
  autocorrelation peak 0.6), thinning out by x ~1100.
- **Eye.** Right edge, rows 560..720, core (1880, 645): white ring, pink, a blue / pink core.
- **Tears.** Two-row pure-cyan dropout rows (rows 350, 564, 578, 602, 686, 728 ...) from the surf edge (x 850..1020)
  to the right edge, a few shifted rows next to them, and a +47 px comb over rows 22..49.

## Motion in the clip
Measured on every unique frame of the 11.1 s clip (29.3 fps):
- **The waves roll down** one wave per 1.00 s: the fitted wave phase falls ~12° per frame (−364° over 30 frames)
  while the band centre stays at x 1456 ± 4. The program's K4 default (700) gives exactly one wave per second,
  downward (1093 / 65536 cycle per field at 60 fields/s).
- **The lagoon edge** sinks 758 → 894 (136 lines) in 1.00 s and snaps back (snaps at frames 4, 33, 63, 92). The
  program puts it at 734 + 136 · frac(τ), on the same master phase τ as the roll.
- **The eye** is born as a faint white blob at the top right late in each cycle, grows (rx 250 → 450, ry 65 → 115)
  and gains a pink then a blue core while sinking (row 585 → 730), then flattens into a blue arc on the lagoon edge.
  The program's eye rides the same τ: row 545 + 380 · frac(τ + 0.15), height 50 + 260 · frac, peak lift
  1.8 · frac − 0.12 band.
- **Surf, dashes and tears** are re-randomised every frame; the program redraws them every field.
- **Ron's knobs:** none moved in the 11 s. The sibling clips are the same colouriser on other fields:
  `WIN_20230214_23_38_38` is a mirrored "crab" of black, green, orange, red and blue rings (→ S10 Crab, S7 Mirror)
  and `WIN_20230214_23_40_17` is mirrored pink / blue lens bands beside a blue noise field.

## What it does
1. **Field.** f = (x − X0(y)) / S(y) in band units (band 0 at f = 0), computed per line by the line engine from the
   fitted M, A, S and chirped phase (the quadratic terms freeze below row 756, where the fit's data ends), and
   stepped across the line by a DDA. Ribbons (K1) scales S about band 0, Swell (K2) scales A, Ripple (K3) scales
   the wave's phase rate.
2. **Curl.** Above the lagoon edge the field bends by (64 − d)² · 31 / 2^20 band (d = rows to the edge).
3. **Surf.** Two streak-noise generators (xorshift16, random holds, ¼ slew) reseeded at every line from a slab seed
   (slabs of 6..23 virtual rows) and a row seed (rows of 4 virtual rows): slab streaks of 48..172 virtual px and
   row streaks of 12..43, mixed 1 : ¼ (all lengths × 8 .. ÷ 8 by Grain). The noise jitters f by a depth that grows
   with the distance into the surf; Grain multiplies the depth by 0 .. 1 .. 12. Left of the (noise-jittered) shore
   the colour comes from the surf bank, indexed by shore − f: broad dark-teal stripes on the chirped surf phase with
   random fine green / mint / lime / tan stripes between. The surf reads the field held at each noise row's start
   (plus its slab's sideways slip above Grain 512), so its stripes break in whole 4-row blocks.
4. **Lagoon.** Below the edge and right of the surf: cyan, with blue dashes where a 64 px triangle ripple (phase per
   noise row) is high and the row noise beats a threshold that falls off with the distance from the shore.
5. **Eye.** From band 1's white edge (f = 1.3) on, the field (capped at the open water's 1.5) is lifted by
   ea (1 − t²), t = |y − ey| / ry, times a ramp over half a band, and read through the ribbon colours: rings of
   white, pink and blue on the right edge.
6. **Tears.** Per noise row a draw (0.7 % .. 5.9 % by P12) starts a cyan dropout row; 60 % of them then slip the
   next 1..4 noise rows by up to ±112 px.
7. **Palette.** 2048 × 32 ROM: Sea and Crab sets × clean and surf halves × 512 entries (`breakers.py`).
8. **Video.** The camera's luma bends the field (K6 Etch); Osc feeds a drifting triangle lattice instead.

## Controls
| Control | Element | Function |
|---|---|---|
| K1 Ribbons | the ribbons' spacing | ×2.8 wide (0) .. the clip (512) .. ×0.35 fine (1023); band 0 stays put, the surf and the open water re-space with it |
| K2 Swell | the waves' amplitude | 0 = straight vertical ribbons .. the clip (512) .. 3× (1023): deep chevrons |
| K3 Ripple | waves down the frame | ×0.33 (one long sway) .. the clip (~2.7 waves) .. ×3 (tight zigzags) |
| K4 Speed | the roll, the lagoon and the eye (one LFO) | bipolar, still in the centre ±8; 700 = the clip (one wave per second, down); up to 7.8 waves/s either way |
| K5 Grain | the surf | 0 = clean, smooth surf stripes (no jitter, a smooth shore) .. the clip (512) .. 1023 = 12× deeper shredding, 8× longer smears and taller slabs, every slab slipped sideways by up to ±0.5 band |
| K6 Etch | camera / Osc into the field | how far the camera's luma (or the Osc lattice, at 10× the camera's gain) bends the ribbons; 80 = a smooth bend on a face |
| S7 Mirror | the whole sea | the right half reflected onto the left: ribbons meeting in the middle, the eye twinned |
| S8 Lagoon | the lagoon and the curl | On (the clip) / Off: the ribbons run all the way down |
| S9 Fold | the open water | the colouriser repeats past band 1: endless ribbons fill the cyan; the lagoon shows the bands slipped half a band |
| S10 Palette | the colours | Sea (the clip) / Crab (the sibling clip's black, green, orange, red and blue rings, dark surf) |
| S11 Source | video in | Video (default) / Osc |
| P12 Break | the surf's reach and the tears | 0 = calm water: clean ribbons, a strip of surf at the left, few tears; 410 = the clip; 100 % = a storm: the surf swallows all but band 1, shreds deeper, tears 8× as often |

Glides (1/8 per field): K1, K2, K3, P12. K5 and K6 are read per field; K4 has a ±8 deadband.

## Video
- **Video (default).** The camera's luma ((Y >> 2) − 16, every other pixel, a one-line buffer) is added to the field
  with gain K6, so the ribbons, the surf stripes and the shore bend around the picture, and the eye and the lagoon
  dashes follow. At the default (Etch 80) the ribbons bend smoothly on a face (`rtlG_cam.png`); at 1023 the
  camera's shapes take over.
- **Osc.** A drifting 2-D triangle lattice (640 × 540 virtual px, one step per field), written signed about its mean
  (a bipolar bend) at 10× the camera's gain: at the default it folds the ribbons into diamond chevrons, at 1023 into
  a lattice of cyan diamonds outlined in ribbon colours. Etch stays live in both modes.

## Engine
- **Sequencer** (vblank zone, "no avid for 1024 clocks"): one serial shift-add multiplier (19 slots), one serial
  divider (2^16 / ry), one EXP2 ROM with a shared shifter (the K1, K3, K5 exponentials), a shared glide over K1,
  K2, K3 and P12. Per field: the master phase τ (+= speed² · 2212 / 2^16), the lagoon row, the eye's row, height and
  lift, the spacing and phase-rate factors, the swell and its taper, the grain step, the slab step, the shore, the
  Grain depth factor (k5 · 8, or 4096 + (k5 − 512) · 88) and the surf depths it scales, the slab-slip chance, the
  tear rate, the etch gain.
- **Line engine** (kicked at each g_start, computes the NEXT line in ~315 clocks, so it fits SD's 858-clock line):
  xorshift32 draws (new noise row / slab, seeds, the slab's sideways slip, tears), P = Y (639132 + 124 Y) / 4096,
  θ = vf P + φ, the sine (256 × 16 quarter ROM), A(y) sin, M(y), S(y), Sh2 = S · sp, G = 2^22 / Sh2 (serial
  divider), F = G (xms − 2 X0 − slip) as a signed shift-add multiply, the curl, the eye's t, t² and lift, the surf's
  held field and shore offset.
- **Beam:** one 27-bit field DDA (reversed left of the centre under Mirror).
- **Pixel path (20 stages):** field and shore distances, noise sum and depth, noise × depth, etch (signed luma ×
  gain), etched field, eye ramp and cap, surf index and key, eye lift (8 × 8), compose (eye / lagoon / dashes / Fold
  / dropouts), palette address, palette ROM, a fabric copy, compose / blank.
- **Output:** U and V through [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums, the 8-back samples from a 256 × 20
  block-RAM ring with `no_rw_check`), luma [1 2 1]/4 and 13/16, then the blanking gate. The running sums snap back
  to neutral after 8 blank samples (the family's resync).
- **Contracts:** generated avid (parity-locked start), one 20-clock latency in every mode, the blanking gate, virtual
  1080-line coordinates (SD shows the centre 4:3), colours in the hardware convention (U = Cr), luma pre-scaled
  16/13 and capped at 940 (drawn whites ~777).
- **ROMs / RAMs (21 EBR):** palette 2048 × 32 (16 EBR), sine and EXP2 (256 × 16 each), the line buffer (1024 × 8),
  the chroma ring (256 × 20).
- **Fit history.** The first cut (a 24-stage path with a sample-and-hold, four noise generators, an elliptical eye
  with its own DDA, a 9-tap luma box) packed to 9230 LC (120 %). Cut to fit: the eye became a lift of the open
  water's field (no x DDA, no square ROM); the surf's held field became a per-line offset; the sample-and-hold and
  two generators went; the chroma box history moved into block RAM; the luma box became [1 2 1]; the F multiply went
  signed (no abs / negate). v0.1.0: 6940–6968 LC. v0.1.1 added Grain's depth scaling, the slab slips and the shore
  gating (a per-pixel ramp-slope mux and a camera pair-average were tried and dropped to stay under budget).

## Status
- **v0.1.1 (2026-11-09), ROUTER1.** Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900
  .heirloom_work/vid/build_queue.sh breakers`.
  - All six configs on seed 1. Fmax (builder, post-route): HD Analog 90.76, HD HDMI 90.59, HD Dual 87.86,
    SD Analog 84.51, SD HDMI 88.67, SD Dual 95.56 MHz.
  - 7003–7035 / 7680 LC (≤ 91.6 %), 21 / 32 EBR. Every bitstream passed iceunpack.
- `dupin.py`: 0 `CARRY I0==I1`. In v0.1.0 one self-add in the tear slip (16v + 8v + 4v) stalled a router1 run; it is
  written as 32v − 4v.
- v0.1.0 (2026-11-09, router1, all seed 1): HD Analog 90.39, HD HDMI 89.93, HD Dual 95.30, SD Analog 91.64,
  SD HDMI 94.45, SD Dual 94.23 MHz; 6940–6968 LC.
- Output: `out/rev_b/ron/heirloom/breakers.vmprog`. **Not flashed.**

## Deviations
- **Whites are pale.** The ribbons' white edges come out at the shell's ~777 luma cap (and 13/16 scale), so they
  read pale cyan-grey beside the still's blown-out white.
- **Surf texture.** The still's surf has finer row-to-row streaking; the program's surf breaks in 4-virtual-row
  blocks (no single-row hard lines, by the hardware rule) and its colours are a little darker and more teal.
- **Eye.** The eye is a lift of the open water's field rather than a free ellipse, so its rings follow the open
  water's contours (round on the right edge, cut flat where the ramp saturates at a narrow Ribbons setting).
- **Tears.** Dropouts are whole 4-virtual-row cyan rows (the still's are 2 rows, one field line); the +47 px comb at
  the top of the still is not modelled.
- **Cut for budget:** the surf's sample-and-hold smear, a separate slow noise layer, and P12's control of the dash
  density (the dashes follow the shore only). Grain's depth ramp keeps a fixed slope (only its base and cap scale).
- **Osc at high Etch.** With Osc at 10× the camera's gain (to keep the default flip), Etch 1023 in Osc mode is a
  lattice of cyan diamonds outlined in ribbon colours, stronger than v0.1.0's.
- **Resolutions.** 720p shows the same picture. SD shows the centre 4:3 (virtual x 240..1680): the eye on the right
  edge and the leftmost surf are cropped. At 720p a 4-row noise unit is 2 or 3 lines, so a tear boundary can
  occasionally leave a one-line chroma step (~37 px) at a ribbon edge; at 1080p the single-row check is 0.
