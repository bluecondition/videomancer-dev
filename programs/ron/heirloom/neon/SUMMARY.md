# Neon — glowing white neon tubes around violet and magenta interiors, swaying in S-curves over teal

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20230919_00_32_29` at 3.0 s
(`photos/vid/WIN_20230919_00_32_29_t3p0.png`). Three neon bands sweep down the frame on a teal ground. They bend into
S-curves that lean further the further right they are. Each band is:
- a white tube in a red-brown glow;
- a red gap and a lavender tube;
- a violet interior with magenta stripes;
- a red-magenta rim and a second white tube in its glow.

Inside, the magenta stripes run in ")(" pairs with sharp cusps on fixed rows. There are lens-shaped eyes where the
cusps meet, and dark cores near the middle rows. In the clip the S-curve rolls down the frame while the interior
pattern stays put: the bands are moving windows onto a pattern fixed to the frame. Presets are hand-written
placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_neon/` (`notes.md`, `track.py`, `fitP.py`, `fitP2.py`, `fitwave.py`,
`prof2.py`, `recon.py`, `sfit.py`, `proto.py`, `imodel.py`, `sheet.py`).
- **The bands are one H-synced oscillator, FM'd by a slow vertical sine.**
  - The left and right outer tubes of the three bands were tracked on every row of 7 frames: the still, its four
    m-frames and the two sibling stills.
  - They fit x = c + (k + a + s w) P(y), rms 1.8 px:
    - **c = -184 px**: the ramp starts 184 px before active video;
    - a = 0.547 band;
    - tube-to-tube width w = 0.349 band.
  - The band frequency 1/P(y) is a sine of y: **f = F0 (1 + 0.146 cos(2 pi (y - yp) / 2240))**, F0 = 1/647.7 px.
    Its relative rms is 0.34 %; a sine in P fits worse (0.50 %).
  - So the phase is U = (x + 184) f(y): the further right a band, the more it swings.
  - The band spacing runs 632..757 px down the still (747 at y = 540).
- **Band profile** (row-aligned median of three frames, `prof_aligned.npy`, in band-phase fractions u):
  - teal (0,66,71) for u 0.14..0.29;
  - a red glow, R ramping 0 -> 124 over u 0.29..0.52 (~150 px);
  - a white tube at u = 0.548 (229,228,229), FWHM 0.03 (~21 px);
  - a red gap (179,88,112) at 0.578;
  - a thin lavender tube (217,168,215) at 0.59;
  - the interior, 0.60..0.845;
  - a red-magenta rim (155,40,180) at 0.85;
  - a white tube at 0.900, FWHM 0.04 (~28 px);
  - a red glow decaying to teal by u = 1.14.
- **The interior is fixed to the frame.** The white chevron lines are identical in all five m-frames
  (`fixed_wh.png`). The whole pattern was rebuilt from 14 frames by keeping the deepest-inside pixel of each
  (`recon.png`):
  - mirrored about x = 992 (|x - 992|), with dark-column axes at x = 220, 992 and 1762;
  - around each axis, a ")(" pair of magenta stripes with white edges, with arms at ax = A0 + D(y) +- W(y):
    - A0 = 87;
    - D = 30 (1 - |cos(pi (y - 515) / 520)|) gives the cusps, on rows 255 and 775;
    - W = 29 cos(pi (y - 515) / 1040) gives the stripe width; the stripe merges to a single line at y ~ 0 and ~1035;
    - units are the middle region's px; the side regions are 1.08-1.24x wider;
  - lens-shaped eyes on the axes at the cusp rows: inner white tips at +-48 px, +-15 rows; outer at +-96 px,
    +-60 rows; i.e. contours of ax + 12.4 sqrt(dy);
  - dark cores (25,13,42), ~35 px wide and ~360 rows tall, at (436, 480) and (1164, 500).
- **Palette** (k = 14):
  - teal 31 %, teal-glow mix 10 %;
  - violet (125,9,249) 9.6 %, white (228,215,233) 8.4 %;
  - red glow (110,61,73) 7.2 %, magenta (210,14,254) 2.9 %;
  - dark blue (78,10,177) 2.8 %, black cores 0.8 %.
- **Texture.**
  - The tube edges carry a fine zipper: a ~4.6-line serration of 1-2 px, plus a 9-10-line wobble of 0.5-1 px.
    The edge residual std is 0.5-1.0 px.
  - There are two small interlace-like comb patches: at x 270, y 390-560, and at x 1730, y 250-330.

## Motion in the clip
- **The FM sine rolls down the frame at 313-316 lines/s**, steadily through the whole 7 s clip.
  - The sine's peak sits at y = 1375 / 1533 / 1688 / 1848 / 2007 at t = 2.5 / 3.0 / 3.5 / 4.0 / 4.5 s, i.e.
    +155..160 lines per 0.5 s.
  - The sibling stills at 0.5 s and 5.5 s fall on the same line. One cycle of 2240 lines takes 7.1 s.
  - The depth stays at 14.5-15.4 %, and the band phase a at 0.545-0.546, in every frame: no horizontal drift and no
    knob turned.
  - The clip's three "looks" are one patch at three phases of the roll.
- **The interior does not move.** The bands slide over it, so the stars and eyes appear and vanish as the windows
  pass (`cmp_motion.png`: the proto at -0.5 / +0.5 / +1.0 s against the m-frames).
- **The program:**
  - At the default Drift (700) the FM phase steps 39,363 / 2^24 cycle per field. That is 5.27 lines per field:
    316 lines/s at 60 Hz (263 at 50 Hz), downward, as in the clip.
  - The interior is fixed to the frame.
  - Checked in the RTL: two captures two fields apart at full Drift match the integer model bit-exactly at t = 2 and
    t = 4.

## What it does
1. **Bands.**
   - Per line, f = F0 (1 + A w), where w = cos(2 pi theta) comes from a 4096-step quarter-wave cosine (Square: its
     sign).
   - The band phase is U = (992 + 184 + J) f + (x - 992) f, where J is the zipper: two cosines of 2.0 and 1.2 px.
   - Mirror uses |x - 992|.
2. **Profile.** v = frac(U) - 0.7225, plus the etch, is shifted outward by the Gap offset on each side of the centre.
   It indexes a 512-entry profile ROM holding:
   - teal, glow, tubes, gap and rim;
   - a glow flag for Bare;
   - the interior weight 0 / 1/2 / 1.
3. **Interior.**
   - P = |x - 992| * 1024/770 (zoomed by Facets) is folded to the nearest multiple of 1024 q, giving ax.
   - Per line: D and W (cosine ROM), the eye term 827 sqrt(dphi) (sqrt ROM) and the dark weight.
   - S = min(|ax - 127 - D| - W, |ax + eye - 105| - 35) + etch. The eyes sit on the peak axes, and are faded in on
     the outer side of the trough axes.
   - S indexes a 512-entry interior palette: magenta core, white line, violet, dimming.
   - The dark cores blend toward (25,13,42).
4. **Compose**: the profile colour or the interior colour, by the weight. Argon rotates the chroma (Cr' = -Cb,
   Cb' = Cr).
5. **Output**: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 at the same centre, luma 13/16, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bands | the bands' count: x1/3 .. x3 (exponential, 512 = the clip's 2.6 across): 0 = one wide band, 1023 = ~7 thin bands |
| K2 Gap | the interior's width between the tube pairs: x0.25 (the tubes nearly touching, wide teal) .. the clip (512) .. x2 (wide interiors, the teal gone) |
| K3 Drift | bipolar (+-8 deadband, square law): the S-curve rolls down (+) or up (-); 700 = the clip's 316 lines/s; full = ~1.2 cycles/s |
| K4 Waves | the vertical sine's frequency: 0.15 .. 5 cycles per frame (exponential, 512 = the clip's 0.48): one gentle arc .. a tight serpentine |
| K5 Facets | the interior pattern's scale (stripes, cusp rows and eyes together): x0.4 (one broad stripe per band) .. x3.2 (dense stripes and many eyes) |
| K6 Etch | how far the camera (or the Osc) bends the tubes and the interior contours: square law, 0 .. 0.47 band and 477 q at full luma; 400 = 0.036 band and 72 q |
| S7 Wave | the vertical sine: Sine (the clip) / Square: the bands become straight and break into stepped segments |
| S8 Mirror | the bands mirrored about the centre: Off (the clip) / On: ")(" bands bulging both ways |
| S9 Glow | the red-brown halo: Halo (the clip) / Bare: crisp tubes straight on teal |
| S10 Palette | Neon (the clip: magenta/violet/white on teal) / Argon: cyan and blue tubes and interiors on a forest-green ground |
| S11 Source | Video / Osc |
| P12 Sway | the vertical sine's depth 4 % .. 75 % (square law, 395 = the clip's 14.6 %): 0 % = nearly straight upright tubes, 100 % = the right bands folded into hairpins |

Glides (1/8 of the gap per field, +-2 deadband) on K1, K2, K4, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) - 16 from a one-line buffer, is added at Etch to two places:
  - the band phase: the tubes bend and fray around the picture;
  - the interior field: bright areas push the magenta stripes and eyes aside.

  At the default Etch the three-band composition reads clearly. With no input, Video mode is the still.
- **Osc mode.** A drifting 640 x 540 virtual-px triangle lattice feeds the same path, at twice the gain and centred
  on its mean. The tubes wobble and the interior stripes swell into large magenta lenses. Etch scales it.

## Engine
- **Per-field sequencer** (vblank zone).
  - One serial shift-add multiplier with 20 slots; the slot is decoded two clocks ahead and the operands are
    pre-muxed.
  - A shared glide, and a 256-entry exp2 ROM for the exponential controls.
  - Per field it computes:
    - F0, the sine's frequency and its per-line step;
    - the interior's x and y scales and steps, and the interior beam start;
    - the depth A, the drift step and the FM phase;
    - the etch gain, the zipper steps, the Gap offset and the Osc mean offsets.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in ~76 clocks (it fits SD's
  858-clock line):
  - five cosine lookups: the FM sine, two zipper cosines, W and D;
  - the sqrt-ROM eye term and the dark weight;
  - on one serial multiplier: A |w|, F0 A w, Uc and Us0.
- **Beam**: the band phase (signed, 29 bits) and the interior phase (signed, Q16 q), stepped per pixel.
- **Pixel path (21 stages)**:
  1. abs of both phases (ones' complement);
  2. the band phase fraction;
  3. the fold to the axis;
  4. the etch products;
  5. the arm and eye distances;
  6. Gap;
  7. S = min + etch, and the palette address;
  8. the palette ROM and its fabric copy;
  9. the dark-core products;
  10. the profile ROM and its fabric copy;
  11. Bare and the interior weight;
  12. compose and Argon;
  13. the filter input;
  14. the chroma 8-sums and the luma taps;
  15. the filters, then luma 13/16, a pipeline stage and the gate.
- **Stores** (21 EBR):

  | Store | Shape | EBR |
  |---|---|---|
  | profile | 512 x 32 | 4 |
  | interior palette | 512 x 32 | 4 |
  | quarter-wave cosine | 1024 x 16 | 4 |
  | sqrt | 256 x 16 | 1 |
  | exp2 | 256 x 16 | 1 |
  | line buffer | 2048 x 8 | 4 |
  | chroma filter's 8-back ring (`no_rw_check`) | 256 x 20 | 2 |
  | C_LAT sync ring (`no_rw_check`) | 256 x 3 | 1 |
- **Contracts**:
  - the generated avid (parity-locked start, field-latched width);
  - one latency, C_LAT 20, in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr, V = Cb);
  - the chroma filter's running sums snap back to teal after 8 blank samples (resync);
  - luma pre-scaled 16/13 and capped at 940 in the ROMs (drawn whites ~777).
- **Verified**:
  - GHDL at 720p60, decimation 1: the default Video frame with a black input is **bit-exact with `imodel.py`**
    (100 % of pixels, at the model's t = 2 fields).
  - The Osc frame matches 71 % exactly, 0.39 mean absolute difference (the lattice's one-line buffer lag).
  - The camera frame (selfie.jpg) shows the woven etch.
  - The motion pair (decimation 2, full Drift, two fields apart) is bit-exact with the model at t = 2 and t = 4.
  - The control sheet `sheet.png` is rendered from `imodel.py`.
- **Fit history**:
  - The first cut placed 7,977 LC and overflowed. It had two 9-tap boxes on Y, U and V as FF arrays, a 32-bit beam
    and a full alpha multiplier.
  - These changes took it to 6,744-6,768 LC (88 %):
    - one chroma box with a block-RAM 8-back ring, and luma [1 2 1];
    - the syncs in a block-RAM ring;
    - a Q16 band-phase pipe and a 3-level interior weight;
    - trimmed line-engine scratch.
  - A bug found on the way: a narrowing `resize` of a signed sum folded the band-phase constant X4. It is fixed with a
    slice.

## Status
- **v0.1.0 built 2026-11-09 with ROUTER1; all six configs on seed 1. Not flashed.**
- Post-route Fmax (MHz):

  | Config | Fmax |
  |---|---|
  | HD Analog | 92.31 |
  | SD Analog | 90.19 |
  | HD HDMI | 93.31 |
  | SD HDMI | 88.46 |
  | HD Dual | 89.37 |
  | SD Dual | 90.99 |

- 6,744-6,768 / 7,680 LC (88 %), 21 / 32 EBR. iceunpack OK x6. The packed config carries the labels and presets.
- Output: `out/rev_b/ron/heirloom/neon.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh neon`, or
  `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh neon`.
- router2 stalled HD Analog seed 1 (the 15-minute timeout) and reached 80.84 MHz on seed 2. router1 routed the same
  netlist at 92.31 MHz on seed 1. Stay on router1.
- The critical path (router1, HD Analog) is the sequencer's slot decode (z_sel / z_pre).
- Presets are hand-written. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **Stars at the walls.** The interior is the clip's own pattern, fixed to the frame, so the stars and eyes come and
  go as in the clip. But the still's upper star in the middle band (white lines converging on the arm's cusp) is
  drawn as the eye's right half with the arm's cusp beside it.
- **One interior scale.** The clip's trough regions are 1.08-1.24x wider than the middle (peak) region. The program
  uses one scale (1.1x the middle region's), so the left band's stripe is ~10 px narrower and its dark core ~20 px
  further left than in the still.
  - Eyes sit on the peak axes and, faded in over 12 q, on the outer side of the trough axes (the clip's trough "eye
    tips").
  - The inner side of the trough axes has none, as in the clip.
- **Tube whites** come out at about 777 output luma (RGB ~212 against the still's 229), the shell's white cap.
- **Softness.**
  - The output conditioning (chroma 9-tap, luma [1 2 1]) stands in for the still's camera blur. The measured profile
    already carries the photo's tube softness.
  - The interior weight at the walls is 3-level (0, 1/2, 1), smoothed by the chroma filter.
- **Zipper** is a deterministic two-cosine serration (the still's teeth are irregular camera moire). The two comb
  patches are not modelled, and the capture border is not drawn.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (virtual x 240..1680), so the left band's outer glow and the right band's outer tube fall
    off the edges. The interior mirror stays at x = 992.
