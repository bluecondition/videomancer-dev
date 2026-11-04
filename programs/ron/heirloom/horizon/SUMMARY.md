# Horizon — a dusk horizon whose dark masses dissolve into yellow-rimmed ribs

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20230703_01_29_07_t10p0.png` (clip WIN_20230703_01_29_07 at 10.0 s, 12 s long). An RGB-colouriser
patch, mirrored about the centre: a cyan sky with pale lavender lobes hanging from the top, a magenta sun (a diamond
with its apex on the horizon) between two soft black mountains, black streaks on the horizon line, water striped by
a chirped horizontal oscillator, and two dark striped masses under a convex horizon arc whose lower edges break up
into a comb of ribs: dark fingers in the centre, narrow gaps opening from below in the masses, a yellow rim on every
dark-to-light edge. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_horizon/` (`notes.md`; `proto.py` = the fit iterations, `proto2.py` = the float
proto with the controls, `imodel.py` = the integer model, `sheet_im.py` = the control sheet).
- **Three colourisers.** The channels split cleanly: G is one smooth folded field, R carries the sky key and the
  stripes, B is flat except for the key. The channels have different axes: G 971, R/B 1002 (an analog channel delay
  of ~30 px), the rib comb peaks at 961.
- **G** = |c(y) + h(|x - 973|)|, one V fold for the whole frame: separable least squares in the sky gives
  c = 172 - 0.528 y, h = 174 (1 - exp(-u / 550)) (rms 5.8 levels); below y = 600 the slope is 0.28.
- **R in the sky** = a ramp 0.69 y + 0.00032 |u| (87.5 - y), keyed by
  - the **lobes**: a comb about x = 978 with a 192 px period (gaps at |u| = 305 / 500 / 699 / 880), 96 px wide,
    narrowing to 75 / 52 / 33 px at y = 210 / 250 / 270 and ending round at y ~ 285;
  - the **sun**: half-widths 339 / 196 / 124 / 53 at y = 300 / 380 / 420 / 460, so the apex is at (1000, 489) and
    the edges fall 0.556 line/px, inside a column |u| < 400 (the heart's shoulders).
- **Mountains**: black on all three channels (RGB ~17). A band about 95 lines thick whose top edge rises 0.36-0.43
  line/px toward peaks at (773, 370) and (1232, 395), cut by an inner edge y > 505 - 0.63 |u| and by the horizon at
  y ~ 510. The thin black streaks below are the band where the first stripe trough crosses it.
- **Stripes**: a triangle wave with flat-clipped troughs (10 %), peaks at y = 519, 579, 637 ... 1063, so
  phase = -8.0513 + 0.013967 y + 2.882e-6 y^2 (the period chirps from 59 to 51 lines; +-3 lines residual). The
  envelope is 240 (1 - exp(-(y - 495) / 130)). The same signal is R in the water and B in the masses.
- **Horizon arc** (mass tops and finger tips, |u| from 1000): 880 at the axis, 818 / 784 / 728 / 682 / 642 / 610 /
  588 / 573 / 566 at |u| = 120 ... 900.
- **Ribs**: a rib comb with a 156 px period and peak at x = 961. The background gaps grow as 2.6 sqrt(depth) px below
  a gap-tip curve: 975 (|u| = 867), 958 (725), 928 (573), 868 (420), 815 (350). The curve rises above the arc for
  |u| < 270, so the centre shows separate fingers with rounded tips on the arc (790 / 875 / 812). The centre gaps
  are 44-60 px wide, wider near the axis.
- **Rims**: inside the key, R = G ramps from ~20 to ~206 over ~24 px toward the falling edge (yellow where the stripe
  makes B low, whitish where it makes B high: the beaded zigzag). The rising edge has an olive glow of ~110 decaying
  over ~20 px, with a core of ~12. The left mass's arc carries a yellow rim.
- **Fit**: float proto 16.0 mean |RGB| vs the still (`cmp_proto.png`); the integer model at 1080p is 17.7.

## Motion in the clip
- **The stripes translate down.** Every stripe peak moves by the same amount: +12/+13 px (960 scale) from m0 to the
  still, +15 from the still to m1. That is ~25-30 lines per 0.5 s, about 55 lines/s or one stripe period per second.
  The program rolls the stripes' phase by 1083/65536 cycle per field (K4 = 672): 53.5 lines/s. The model's stripes
  sit 28 lines lower at +0.5 s and 57 at +1.0 s (`cmp_motion.png`).
- **The rib comb is hsync-locked.** Finger centres in m1/m2/m3 stay within 3 px (960 scale), so the program keeps the
  comb still.
- **The rest is Ron's knobs**, each mapped to a control:
  - 8.5 s: two smooth hills meeting in a V (P12 = 0).
  - 9.0-9.5 s: stripes over the whole frame, with black slivers in the sky (K3 = 0).
  - 10.0 s: the still.
  - 10.5-11.5 s: the masses dissolve into a ribcage of tall yellow-rimmed ribs (P12 = 100 %), the stripes recede and
    vanish (K3 = 1023), and the lobes go (K5 = 0).
- **Sibling stills.** t 6.5 shows ribs broken into rows of yellow beads (S8 Beads). 01_34_12 and 01_35_00 are the
  same geometry in grey (S10 Mono).

## What it does
1. **Per line** (the line engine, computing the next line):
   - the stripe value S (phase, triangle, envelope);
   - the horizon and streak gates;
   - cG;
   - the R ramp's offset and |u| slope;
   - the sun's line term;
   - the lobe half-width at this y (the rounded end);
   - the comb rate (with Fan) and its start phase;
   - the bead terms.
2. **Per pixel**:
   - **ROM lookups.** |x - 1000| addresses two ROMs: the arc and gap-tip curve, and the mountain slope and the
     central gap cap. |x - 973| addresses the hG / lobe-distance ROM.
   - **Rib key**:
     - t = y - (565 + Bowl arc - camera);
     - gap = min(cap, 2.13 sqrt(y - yg)) below the tip (a ROM), -0.75 (yg - y) above it;
     - hw = min(0.5 - gap, (0.5 - gap) circle(t / 40));
     - the bead term is added on the falling edge;
     - key = t > 0 and q < hw, where q is the comb's distance from its peak.
   - **Shades**: the yellow ramp (dY - dist) x 4 on the falling edge, the linear glow on the rising edge, the arc rim
     on the left mass.
   - **Sky**:
     - R = ramp x max(lobe key, min(sun diamond, column));
     - G = |cG + hG - camera|;
     - B = 255;
     - the water: R = max(R, S).
   - **Mountains**: min of the band's top ramp, its bottom ramp, the inner cut and the horizon/streak gate; then a
     black key on R, G and B.
   - **Compose** and convert RGB -> hardware YUV (U = Cr).

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Ribs | the rib comb's rate: 6 .. 25 ribs across (exponential; 512 = the still's 156 px period, 12.3 across), centred on the middle rib |
| K2 Bowl | the horizon arc's depth, x0 .. x2: 0 = a flat horizon at y 565 with tall central ribs; 512 = the still (565 -> 880 on the axis); 1023 = a deep bowl dipping off the bottom |
| K3 Tide | where the stripes begin: 0 = the whole frame striped, black slivers in the sky (the clip at 9-9.5 s); 512 = from the horizon (y 494, the still); 1023 = no stripes: black ribs with yellow rims over clean cyan water (the clip at 11 s) |
| K4 Speed | bipolar (+-8 deadband, square law): the stripes roll down (right) or up (left); 672 = the clip's ~55 lines/s; ends ~590 lines/s |
| K5 Lobes | the sky lobes' length: 0 = none (a plain gradient sky); 512 = the still (they end at y 286); 1023 = down to y ~640, over the mountains |
| K6 Etch | how far the camera (or the Osc) bends the picture (square law): per full luma the horizon arc and rib tips rise ~130 lines at the default (560), ~430 at 1023; the gaps open, the sky's green dims, the mountains lift |
| S7 Fan | Off / On: the comb's rate falls toward the top (perspective), so the ribs and gaps radiate from below like a fan |
| S8 Beads | Off / On: the stripes patched hard into the rib key (+-1/8 cycle on both edges): the ribs break into stacks of beads (the clip's t 6.5 look) |
| S9 Dawn | Off / On: the red ramp inverted: a pink-white sky at the top and a violet sun |
| S10 Palette | Dusk (the still) / Mono (the grey look of the sibling clips 01_34_12 and 01_35_00) |
| S11 Source | Video / Osc |
| P12 Ribcage | the gap-tip curve (1.42 lines per step): 0 % = two solid hills meeting in a V (the clip at 8.5 s); 460 (45 %) = the still; 100 % = every gap open and the yellow rims ~50 % wider: a full ribcage across the horizon (the clip at 10.5-11.5 s) |

Glides (1/8 of the gap per field, +-2 deadband) on all six knobs and P12.
**Presets:**
- Photo
- Hills (P12 0, Tide 300)
- Ribcage (P12 1023, Tide 760)
- Flood (Tide 0, Lobes 760)
- Bead Curtain (Beads, P12 800, Ribs 650)
- Fan Dawn (Fan, Dawn, P12 700)
- Grey Comb (Mono, Ribs 300, P12 900)
- Deep Bowl (Bowl 900, Speed 900)

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) - 16 from a one-line buffer, times Etch, gives cL in lines.
  The camera is woven in through four modulation inputs:
  - the horizon arc and the rib tips rise by cL, so the masses and ribs grow up into bright parts of the picture;
  - the gap-tip curve rises by 9/8 cL, so the gaps open there;
  - the sky's G fold dims by 5/8 cL, so bright areas tint the sky magenta and blue;
  - the mountain band rises by cL/2.

  With no input, Video mode is exactly the still: the black-input RTL render is bit-exact with the integer model.
  At the default Etch the composition still reads with a camera (`cmp_final.png`, the selfie collage).
- **Osc mode.** The shell's drifting 2-D triangle lattice, here at 320 x 270 virtual px (twice the usual density),
  feeds the same path. At the default Etch the horizon becomes a row of peaks, the sky tints in a lattice and the rib
  tips ripple.

## Engine
- **Vblank-zone sequencer.**
  - The shared 7-channel glide.
  - The knob curves on one serial 26 x 10 multiplier, one product every 11 clocks (zc 40-121):
    - kf0 = 105 exp2 / 2^13;
    - the stripe start, 1.453 k3 - 250;
    - the lobe end, 0.70 k5 - 72;
    - the gap-tip offset, 652 - 1.418 p;
    - the rim width;
    - the drift step dd^2 3 / 64;
    - the etch gain k6^2 / 2^14.
  - The stripe phase drift accumulates once per field.
- **Line engine.** Kicked at each g_start (and at zc 1300 for line 0), it computes the NEXT line in ~200 clocks
  (fits SD's 858) on one serial 32-bit multiplier:
  - the stripe phase, triangle, clip and envelope;
  - cG;
  - the R ramp terms (normal or Dawn);
  - the sun's line term;
  - the lobe end (69 CIR, as x 8 + 128);
  - the Fan rate kf0 - kf0 (1080 - y) / 2048;
  - the comb start phase.
  The values are double-buffered (n_* -> l_* at g_start).
- **Beam.** x in half-px and the rib comb phase accumulator. A second copy of the phase runs 13 clocks behind
  (stage 14), so q and the slope side need no delay line.
- **Pixel path (22 stages, then the shell's boxes; C_LAT 28).**
  - 2-3: axis offsets and |u|.
  - 4: ROM addresses. 5: EBR. 6: fabric copies.
  - 7: Bowl x arc, the camera x etch terms, the R ramp product.
  - 8: t, dy, the key ramps.
  - 9: the sqrt ROM address, CIR, G, the R key.
  - 10-11: sqrt EBR and its copy, R x key, the mountain mins.
  - 12: gap.
  - 13: hwB, fz, the rim gate.
  - 14: hwB x circle.
  - 15: tip, q / side from the delayed phase.
  - 16: hw = min, bead - q.
  - 17: distance, and the background x mountain key products.
  - 18: key and shades.
  - 19: compose. 20-21: matrix. 22: offsets, cap, Mono, black outside the avid. 23: copy.
  - Then box A 24-25, box B 26-27, luma 28, gate 29.
- **Block RAM (18 EBR).**
  - arc / gap-tip ROM 512 x 16 (2)
  - mountain / gap-cap ROM 512 x 16 (2, sharing the arc ROM's address)
  - hG / lobe ROM 512 x 16 (2)
  - sqrt ROM 512 x 8 (1)
  - stripe envelope ROM 512 x 8 (1)
  - exp2 256 x 16 (1)
  - line buffer 2048 x 8 (4)
  - the output-box 8-back rings 4 x 256 x 16 (4)
  - the sync ring 256 x 4 (1)

  The rings carry `no_rw_check`.
- **Contracts.**
  - The generated avid (parity-locked).
  - One latency (28) in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - [1 2 2 2 2 2 2 2 1]/16 Y/U/V boxes and 13/16 luma before the blanking gate.
  - The running sums snap to neutral off the avid (resync).
  - Colours in the hardware convention (U = Cr).
- **Verified.**
  - GHDL 720p60, decimation 1, default knobs, black input: bit-exact with `imodel.py` at fields 2 and 4 (0 difference).
    The second capture's stripes are one 720p line lower.
  - Osc: 0.47 mean |dYUV| (the model's lattice phase is approximate).
  - Camera (selfie_small.jpg): 3.9 mean |dYUV| (the tester's input conversion).
- **Fit history.**
  - The first RTL built at 7,586 LC (98.8 %). It met timing (HD 78.75-85.95 MHz) but was over budget.
  - Cuts that brought it to 7,030-7,058:
    - every knob curve moved onto the zone multiplier (about -300 LC);
    - the q / slope delay lines replaced by the stage-14 phase copy;
    - t carried as 7 bits;
    - the background carried as R / G / key;
    - the bead term added after the tip;
    - narrower multipliers;
    - the mountain slope and gap cap moved into a ROM;
    - shift-only ramps.
  - Two visible details were cut: the mountains' left/right tilt, and the exponential glow (now linear).
  - The motion capture found the stripe drift running upward; its sign was fixed before the final build.

## Status
- **v0.1.0 built 2026-11-04 through the shared queue with ROUTER1 (`NEXTPNR_ROUTER="--router router1"`), all six
  configs on seed 1.** NOT flashed.
- Post-route Fmax: HD Analog 97.68, HD HDMI 83.47, HD Dual 97.65, SD Analog 92.50, SD HDMI 95.34, SD Dual 93.65 MHz.
- 7,030-7,058 / 7,680 LC (91.6-91.9 %), 18 / 32 EBR. iceunpack OK x6; the packed config carries the new labels.
- Output: `out/rev_b/ron/heirloom/horizon.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh horizon`.
- An earlier router2 build of a near-identical netlist stalled on HD Analog seed 1 and passed on seed 2 (78.08 MHz).
  Stay on router1. dupin.py: 0 `CARRY I0==I1`.

## Deviations
- **Lobes and mountains.**
  - The lobes are vertical and all the same length. In the still they lean slightly, and the shoulder lobes beside
    the sun merge into the heart; a column |u| < 400 approximates that merge.
  - The mountains are symmetric: the still's right mountain sits ~20 lines lower (a measured tilt, cut for LC).
  - The horizon streaks are thinner than the still's.
- **Ribs.**
  - The rib tips are rounded by a fixed 40-line circle.
  - The still's fingers vary in width (93 / 66 / 80 px); the program narrows only the central one, through the gap
    cap.
  - The rim's beads are a +-5 px stripe modulation of the falling edge, softer than the still's diamond beads.
  - The rising-edge glow is a linear 110 -> 12 ramp, where the still's decays exponentially.
  - The comb is uniform. In the still its leftmost period is compressed (~143 px against 156).
- **Shell artefacts.** Each line fades in from black over ~8 px (the box warm-up). Luma is scaled 13/16, so whites
  come out at ~777.
- **Motion.** By default only the stripes move. The clip's other changes were Ron's knobs, now K3 / P12 / K5.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (1440 virtual px), so the outermost gaps and lobes at the sides are cut.
  - Every size is in virtual px and lines, so shapes keep their proportions.
