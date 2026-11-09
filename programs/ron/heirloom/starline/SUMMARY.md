# Starline — a neon star of nested tubes, its colours sweeping sideways

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), rebuilt from the clip
`photos/vid/WIN_20231218_21_31_11` at 3.0 s. Presets are hand-written placeholders.

## Reading of the photo
Working files are in `.heirloom_work/rb_starline/` (`notes.md` has the measurements).
- **The drawing.** A neon "star" drawn as bundles of 2–4 thin bright tubes, on black and on a magenta wash.
  - A left point at (313, 775) and a right point at (1810, 775). Both are sharp cusps on one row, the kink row.
  - A ")" arc runs from the left point up to an apex near (830, 600), then off the left edge at y ≈ 320. A
    mirrored "(" arc has its apex near (1300–1360, 590) and leaves the right edge at y ≈ 340. Each arc has three
    nested tubes.
  - A V of 2–3 tubes runs from both points down to a rounded bottom at (1062, 888–913).
  - A band of 3–4 tubes crosses the full width. It dips to a rounded V at (1071, 374) and rises to y ≈ 255 at
    the edges; the tubes are 10 px apart at the dip and 60 px apart at the edges.
- **Field.** Every contour is symmetric about x = 1071: T = |x − 1071|, and the two halves overlay on a T–y plot.
  - Arcs: the outer tube is b(y) − T = 0, with b(y) = 395 + q·(y − 605)². q is 0.0075 above the apex and
    0.0118 below it, down to the kink row (b = 736). Below the kink a straight fall of 5.36 px per line makes
    the V.
  - On the arcs the nested tubes are geometric, at ratios 1, 0.85 and 0.71 (0.24 octave apart). Below the kink
    they run parallel.
  - Band: rays from the dip, with slopes of 0.11, 0.16, 0.21 and 0.27 px per unit of T (geometric, 0.45 octave
    apart).
  - Tube width is constant in pixels: about 10 px tall on the V and 15 px wide on the arcs.
- **Colour.** One 1-D hue ramp in x, read through a colouriser.
  - Tube colour by x: pink-white (x < 700), then white, yellow (850–1000), green (1050–1500), yellow
    (1600–1700), pink at the right point and magenta (x > 1800).
  - Wash: magenta over the first quarter of the ramp, violet at the left edge, maroon at the bottom left, and
    black under the green part. A soft green glow sits above the band's dip.
  - A thin bright vertical line at x = 312, full height, marks the ramp's reset.
  - In the right third the tubes are beaded: sample-and-hold blocks of about 40 px. A top-right "ladder" is cut
    into vertical stripes about 35 px wide.
- **Texture.** Soft everywhere: a camera/CRT blur of about 6 px.
- **Fit at the defaults.** Integer model vs the still at 1920 × 1080:
  - mean |RGB| error 23.3;
  - 60 px block luma std 15.5, against the still's 15.4;
  - 14 of the 16 cells of a 4 × 4 region-mean grid are within 22 levels. The other two are the top-right ladder
    (77 vs 35) and the bottom-centre wash (103 vs 63).

## Motion in the clip
- **The star holds still.** Through the 12 s clip the points, kink row and apexes stay within ±20 px. No knob is
  turned.
- **The colours sweep sideways on a slow LFO,** carrying the hue ramp, the reset line and the bead zone.
  - The reset line sits at 1275, 725, 308, 100, 132, 428 and 868 px, for t − 1.0 to t + 2.0 s in 0.5 s steps.
  - Fit: x_v(t) = 1913 − 1826·cos(2π(t − 3.7 s)/8.7 s). The peak speed is about 1100–1400 px/s, and the line turns
    round at about 150 px.
  - The ramp is line-locked, with a period of about 2200 px (the 1080p line including blanking). Only that
    period reproduces the dark right edge at t + 0.5 s and t + 1.0 s.
  - For about 4 s of each cycle the reset line is hidden past the right edge.

## What it does
1. **Distance.** T = |x − 1071| in virtual px, rounded near the centre (128 + dx²/512 h for dx < 128 px). Etch
   adds camera (or Osc) luma to T.
2. **Star comb.** Tubes sit where log2 B(y) − log2(T + c(y)) = i/4 octave.
   - Above the kink row, B = 395 + q·(y − 605)²: the arcs.
   - Below it, B holds the kink value and c = 5·(y − y_k): the V's parallel lines.
   - At the points, a parabolic fillet over ±4 rows removes the crease that a hard kink leaves on its row.
3. **Band comb.** Tubes sit where log2 a(y) − log2 T = j/2 octave, with a = 8.7·(374 − y): rays from the dip.
4. **Tubes.**
   - The nearest tube is found by rounding in the log domain, and its position t_i comes back through an exp2 ROM.
   - The distance |T − t_i| is scaled by 1/sqrt(1 + slope²) per line (a fixed 29/256 for the band), so the tubes
     keep a constant pixel width.
   - A profile ROM turns the distance into {tube, glow}. P12 fades the last tube in by quarters.
5. **Colour.**
   - k = the wash level (per line) + the tube/glow intensity, attenuated under the wash.
   - u = (x − x_v)/2200 is the hue ramp, and x_v rides the LFO.
   - The 2048-entry LUT {u7, k4} is dithered with a period-8 pattern that the output box cancels.
   - The reset line is drawn where |u| is small.
6. **Beads.** Where the ramp passes green (u 0.45–0.80), x is sampled and held in 8 px steps and hashed 32 px
   blocks are darkened. This chains the tubes into beads.
7. **Output.**
   - Y, U and V go through one [1 2 2 2 2 2 2 2 1]/16 box. It gives the softness and is the HD HDMI bar fix, and
     it resyncs to neutral after 8 blank samples.
   - Luma is scaled 13/16 before the blanking gate. Palette luma is pre-scaled 16/13 and capped at 940.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Arcs | the arcs' curvature, ×1/4 .. ×4 (exponential): near-straight sides (0) .. the clip (512) .. tight hooks curling back (1023) |
| K2 Waist | the arcs' apex distance from the centre: 0 = the arcs touch in a lens at the centre, 449 = the clip (395 px), 1023 = pushed out to the edges (900 px) |
| K3 Points | the row of the two points, where the V starts: high (line 560) .. the clip (774) .. low (line 1000) |
| K4 Sweep | the colour LFO's period, exponential: 62 s (0) .. the clip's 8.7 s (512) .. 1.2 s (1023) |
| K5 Bands | the hue ramp's period: two lines' worth, broad colour fields (0) .. one line, the clip (512) .. 137 px rainbow stripes (1023) |
| K6 Etch | how far the camera (or the Osc) bends the tubes, square law: 0 .. 74 px per full luma at 420 .. 436 px at 1023 |
| S7 Mirror | Off (the clip) / On: the star mirrored about its point row. The arcs curl below as well and the V is gone, giving a four-armed star |
| S8 Beads | Zone (the clip: beads where the ramp passes green) / All: a 32 × 16 px mosaic over the whole picture |
| S9 Wash | On (the clip) / Off: tubes, glow and the reset line on black |
| S10 Colour | Neon (the clip) / Ember: Cb inverted, giving an orange-red wash with cyan-blue tubes |
| S11 Source | Video / Osc |
| P12 Neon | the tube count, 1 .. 12 (glided): 0 % = one tube per family, a bare outline; 187 = three, the clip; 100 % = a dozen nested tubes that fill the star into a glowing tunnel |

**Presets:**
- Photo
- Tunnel (P12 100 %)
- Four Arms (Mirror, P12 400)
- Ember Hooks (Ember, K1 900)
- Rainbow Bars (K5 900, K4 700)
- Mosaic (Beads All, P12 300)
- Bare Tubes (Wash Off, K2 250)
- Lens (K2 60, K1 300)

## Video
- **Video mode (the default).** Camera luma − 64 (10 bits, from a one-line buffer at every other pixel) is added
  to T as Etch × luma. Bright areas pull the tubes in toward the centre line.
  - At the default Etch (74 px per full luma) the selfie bends and frays the tubes, and the star still reads.
  - With no input, Video mode shows the still.
- **Osc mode.** A drifting 2-D triangle lattice replaces the camera.
  - Its cells are about 320 × 270 virtual px. It drifts about 38 px/s right and 24 lines/s down, and it is
    sharpened (3v − 510, clipped).
  - It enters the same path at 4× the camera gain. At the default Etch the tubes zigzag into polygon outlines up
    to about 257 px out: an unmistakable flip.

## Engine
- **Per-field sequencer (vblank zone).** One serial right-shift multiplier (16 steps per slot, 18 slots), a shared
  glide, and log2 borrowed from the pixel path's ROM. It computes:
  - the arc curvatures (exp2 of K1), the apex distance and the kink row, b at the kink and its log2;
  - the upper arm's slope at the kink, and its 1/sqrt(1 + slope²);
  - the tube counts and the glow weight, from P12;
  - the hue step (exp2 of K5), the LFO rate (exp2 of K4), the LFO phase and its smoothstep-cosine, x_v and the
    line-start hue;
  - the etch gain (K6²).
- **Per-line engine.** It is kicked at each line start and computes the NEXT line on one serial multiplier, well
  inside SD's 858 clocks:
  - b(y), and the slope's 1/sqrt(1 + slope²) from a 64-entry table;
  - a(y), c(y) and the fillet;
  - the wash level and its attenuation code;
  - two log2 values, borrowed from the pixel path's log2 ROM in hblank.
- **Beam.**
  - A DDA carries x and the hue phase.
  - A late copy, aligned to pixel stage 24, carries the bead hash, the hue phase for the bead zone and the reset
    line, and the colour index's u and dither.
  - The colour index takes u and the dither 5 px ahead of its own pixel. This is invisible on a 2200 px ramp.
- **Pixel path (40 stages).** In order:
  - S&H hold, |x − centre|, the rounded-centre ROM and etch;
  - the log2 ROM, two combs and the nearest tube;
  - two exp2 ROMs, the distances × g and two profile ROMs;
  - tube/glow max, beads, intensity and wash attenuation;
  - the colour LUT (3 × 2048 × 8), compose, the box, 13/16 and the gate.
- **Memory.** 23 EBR:
  - the colour LUT (12) and the line buffer (3);
  - one log2 ROM, two exp2 ROMs, the rounded-centre ROM and two profile ROMs;
  - a T delay ring and a sync delay ring, both `no_rw_check`.

  The sync ring replaces 120 shift-register FFs. A testbench (`tbs/tb_sync.vhd`) shows that it delays hsync,
  vsync and field exactly like the old shift register (C_LAT − 1 = 39 clocks), with avid unchanged.
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 40 (= processing_delay_clks), in every mode.
  - Virtual 1080-line coordinates. SD shows the centre 4:3.
  - Colours in the hardware convention (U = Cr, V = Cb).
- **Verified (GHDL, 720p60, decimation 1, real timing ID).**
  - The default frame with a black input is 99.76 % bit-exact against the integer model `imodel.py`. The rest is
    within 16 codes.
  - Osc and selfie-camera frames are in `cmp_final.png`.
  - `thincheck.py` finds no single-row horizontal run of 24 px or more at the defaults, at P12 0 % and 100 %, or
    with any switch on. Only K1 = 0 leaves one 27 px run.
  - Not checked by sim: SD, interlace, and motion over time. These are reasoned from the code and the model;
    `cmp_motion.png` shows the model against the clip over time.

## Status
- **v0.1.0 built 2026-11-09 with router1, NOT flashed.**
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh starline`
    (or `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh starline`).
  - All six configs passed on seed 1. Post-route Fmax:

    | Config | Fmax |
    |---|---|
    | HD Analog | 92.85 MHz |
    | HD HDMI | 87.11 MHz |
    | HD Dual | 93.31 MHz |
    | SD Analog | 90.97 MHz |
    | SD HDMI | 86.48 MHz |
    | SD Dual | 88.05 MHz |
  - 7,078–7,119 / 7,680 LC (92 %), 23 / 32 EBR. Every bitstream passed iceunpack.
  - Output: `out/rev_b/ron/heirloom/starline.vmprog`.
  - Standalone synth: 3776 LUT, 2713 FF, 1346 carry. dupin shows 0 `CARRY I0==I1`.
- **Timing history.**
  - An earlier netlist missed timing on a fused intensity / attenuation stage at 59 MHz; that stage was split.
  - The build before the final one (all six on seed 1, 89.8–99.0 MHz) sat at 7,209–7,242 LC. It was trimmed
    under budget by moving the sync shift registers into an EBR and dropping the hue-index carry pipe.
- **Watch on hardware.** No pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config.

## Deviations
- **Resolutions.**
  - 720p shows the same picture as 1080p.
  - SD shows the centre 4:3 (virtual x 240–1680), so the right point and the tip of the right arc fall off the
    right edge. The left point stays on screen.
  - Tube widths are in virtual px, so they look the same at every resolution. SD is softer, because the box is
    per output pixel.
- **Not drawn.** The faint seam along the kink row left of the left point, and the faint whitish arcs in the
  top-left magenta glow.
- **Ladder and wash.** The top-right ladder is drawn as beaded band tubes. Its dark gaps and the blue patch at the
  right edge are not reproduced, so that region is brighter than the still (region mean 77 vs 35). The
  bottom-centre wash is also brighter (103 vs 63).
- **Spacing.** Tube spacing is quantised to 1/4 octave on the star (still 0.24) and 1/2 octave on the band (still
  0.45), so the band's rays spread slightly wider. The V falls 5 px per line, against a fitted 5.36.
- **Beads.** The beads are hashed 32 px blocks darkened in the zone, an approximation of the clip's dot chains.
- **Hue ramp.** The 2200 px line-locked period is inferred from the motion; shift-matching on one frame gave
  1875–1925 px. The LFO is a smoothstep approximation of the fitted cosine.
- **Softness.** One output box, not two. Part of the still's softness comes from the tube and glow profiles
  instead.
- **Cut for budget.** The colour index reads its u and dither 5 px ahead of its pixel (invisible), and the sync
  delay moved into an EBR (no visible change).
- **K1 extreme.** K1 = 0 leaves one 27 px single-row bright run at the point row. The defaults and P12 0 % and
  100 % are clean.
