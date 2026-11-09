# Effigy — a yellow idol in red and black flames

**v0.1.0, built 2026-11-09. Not flashed.** This is a Heirloom program from the video clips (phase 3). It was rebuilt
from the clip still `photos/vid/WIN_20231229_00_46_59_t6p0.png` (clip WIN_20231229_00_46_59 at 6.0 s).

The still shows a yellow idol standing in the middle of the frame: a column of hexagonal blobs and a heart, mirrored
left to right. Black horizontal bands cut the column into windows. Their halos glow dark red to bright red toward the
right, with orange teeth along the edges. Around the idol, its own outline is echoed outward as nested zigzag contour
lines, with orange shards. Orange streaks are held at the left edge, with a yellow ghost blob against them. In the
clip, the idol's profile scrolls upward through the fixed bands. The presets are hand-written placeholders.

## Reading of the photo (measured)
The working files are in `.heirloom_work/rb_effigy/` (`notes.md` lists every script).
- **Idol.**
  - Yellow (250, 215, 0), mirrored about x = 1002. The centre of each row wanders between 992 and 1013.
  - It is visible in four windows between the bands. The half-width in each:

    | Rows | Shape | Half-width |
    |---|---|---|
    | 100–220 | hexagon | 113 → 72 |
    | 305–440 | hexagon | 133 → 177 → 163 |
    | 640–850 | heart | flat 220-wide top, tapering to a point at 850 |
    | 980–1080 | rising | 35 → 185 |

  - An orange rim runs outside the yellow: about 17 px wide on the left and about 32 px on the right (a horizontal
    lag).
- **Bands** (where they cross the column at x = 1002):
  - **T:** rows 8–104.
  - **B:** rows 224–310. Thin at x ≈ 1200, then a red lozenge with a black hole on the right (rows ≈ 100–300).
  - **C:** a lens, rows 434–640, with tips at x ≈ 430 and ≈ 1620 (y ≈ 535).
  - **D:** the bottom arc, centred on row 937. It sinks to 1025 at the left edge and 985 at the right, and widens
    past x 1750.
  - The cores are pure black. The halos ramp from dark red up to bright red (233, 6, 2) toward each band's edge, and
    they get brighter toward the right. The outer edge cuts sharply to the background black, which is faintly
    red-tinted.
  - Measured as 16 knots per band, 128 px apart: centre, half-height and halo gain (`bandmodel.py`).
- **Teeth.** Orange (243, 118, 1) right triangles, about 20–30 px tall and about 55 px apart, along the red halo
  edges.
- **Contour lines.**
  - Nested zigzag outlines of the idol.
  - Spacing: about 62 px on the left and about 140 px on the right (autocorrelation peaks at 70 and at 142–162).
  - Colour: dim lines are dark green (3, 20, 1) in the upper left (x < 820, y < 580), dark red elsewhere, and bright
    red on the right. Some segments are orange shards.
  - A per-row phase fit of the lines against |x − 1002| is only 0.22 coherent. The spacing grows outward and the
    lines are irregular, so they are modelled as nested outlines of the idol profile.
- **Left edge.**
  - Horizontal orange or dark-red streaks about every 45 rows, 60–170 px long.
  - A yellow ghost blob at rows 305–400, x 0–175, with a rounded right end.
- **Palette** (k-means, 12 clusters): black 56 %, yellow 7.7 %, red ramp 25 %, orange 2.7 %, dark green 1.7 %. The
  colouriser is a fire ramp: R rises first, then G.
- **Integer model vs the still, class shares** (black / dark red / bright red / orange / yellow / green), by quadrant:
  - top left: model 77 / 4 / 1 / 4 / 8 / 8, photo 80 / 4 / 1 / 3 / 6 / 7;
  - top right: model 48 / 27 / 15 / 2 / 9 / 0, photo 54 / 29 / 6 / 3 / 8 / 0;
  - bottom left: model 66 / 21 / 2 / 4 / 7 / 0, photo 69 / 19 / 1 / 4 / 6 / 1;
  - bottom right: model 52 / 15 / 18 / 3 / 12 / 0, photo 54 / 15 / 18 / 3 / 10 / 0.

## Motion in the clip
These numbers come from the four m-frames (t − 0.5 s to + 1.5 s) and the strip.
- **The idol scrolls up.** The bands and windows stay put while the idol's profile scrolls upward through them.
  - The best shift between successive m-frames is 42, 62, 58 and 54 rows per 0.5 s, so about **108 rows/s**.
  - What is left after the shift (22–36 px rms) is the profile morphing as its peaks grow.
  - The heart turns into an hourglass by + 0.5 s to + 1.0 s, and the ghost blob rides up with the profile.
- **The bands drift slowly.** The bottom arc rises 8–10 px per 0.5 s; the lens moves 2–8 px.
- **What the program does with it.**
  - K3 at its default of 680 scrolls the measured profile table up at exactly 108 rows/s (1.707 table entries per
    field at 60 Hz; it runs 5/6 as fast at 50 Hz). The contours, the ghost and the streaks scroll with it.
  - The bands bob ±22 rows on an 8 s triangle.
  - A GHDL capture with K3 at 900 after 10 frames shows the heart turned into an hourglass. It matches the integer
    model at field 9 on 99.3 % of pixels.
- **Ron's other takes of the same patch.**
  - 45_38: one big yellow lens crossed by two sine bands.
  - 48_09 and 49_21: one tall vase or heart idol, green bands, and fat orange zigzag stripes everywhere.
  - 48_54: red dominated.
  - These set the ranges of K2 (vase vs stacked blobs), S8 (green bands), S10 (crossing bands) and P12 (the stripes
    lighting up).

## What it does
1. **Idol key.** F = w(y) − |x − 1002| + etch.
   - The pixel is yellow where F > 0.
   - Below the edge there is an orange rim: level 90 − 1.5u on the right and 90 − 2.5u on the left, with u = −F.
   - w(y) comes from a measured 1024-entry table. Where the still's bands hide the idol, the table uses the still's
     profile filled in from the m-frames. It is scrolled by K3, stretched by K2 and scaled by K1.
2. **Contours.**
   - e = (w_line(y) − |x − 1002| + etch) / λ, using λ = 62 px on the left and 140 px on the right, times K4.
   - w_line is the same profile bridged across the band gaps, so the outlines run on unbroken.
   - Each period carries a trapezoid line.
   - Each segment (contour index × 48-row block) draws a class from a random table: off, dim, bright red or orange,
     with different odds on the left and right. That class gives its level and width.
   - The dim end of the palette is green in the upper left.
3. **Bands.**
   - Four bands, each from 16 knots. Per line, the engine computes the depth e = h − |y − yc| at every knot.
   - For each 128-px segment it keeps the band with the larger depth sum at the segment's two knots.
   - Per pixel, it interpolates the depth E and the halo gain G across the segment.
   - E ≥ 0 is the band: the halo level is G · (1 − E/64), so the edge glows and the core is black.
   - Just outside the edge, a 64-px sawtooth with hashed heights, scaled by the halo gain, draws the orange teeth.
4. **Left edge.**
   - A held streak every 45 rows, each with its own level and length from a table, scrolled with the profile.
   - The ghost: a 96-row blob (175 − 0.04 v² px wide) against the left edge, one per frame height, scrolling with the
     idol.
5. **Priority.** Band > idol and ghost > tooth > max(line, rim, ghost rim, streak).
   - The result goes through a 512-entry palette: fire, fire with a green dim end, the Ember halo or the Jade halo.
   - Then come the shell's output conditioning and the 13/16 luma scale, so drawn yellow is about 765.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Idol | Idol width ×0.25 .. ×4 (exponential; 512 = the still). The blobs, the ghost and the contour outlines scale together. 0 = a thin yellow spine; 1023 = the idol fills the frame between the bands. |
| K2 Stretch | The idol table's vertical rate ×0.4 .. ×2.5 (512 = the still). 0 = one tall vase, like the clip's other takes; 1023 = a stack of many small blobs. |
| K3 Speed | Bipolar (±8 deadband, square law). The profile, with its contours, the ghost and the streaks, scrolls up (+) or down (−). 680 = the clip's 108 rows/s; full = about 1070 rows/s; the centre stops the scroll (the bands keep bobbing). |
| K4 Contours | Contour spacing ×0.25 .. ×4 (62 / 140 px left / right at 512): dense fine outlines .. a few broad ones. |
| K5 Bands | Band half-height −128 .. +80 px (512 = the still). 0 = no bands: one unbroken idol column wrapped in contours. 1023 = thick black bands with glowing halos swallow most of the idol. |
| K6 Etch | How far the camera (Video) or the lattice (Osc) bends the idol and its contours: 0 .. 600 px per full luma (square law). 450 = 116 px. |
| S7 Mirror | Off (the still: a dark left side and red flames on the right) / On: the right side's halos, teeth and wide contours mirrored onto the left. |
| S8 Palette | Ember (red halos) / Jade (green halos and green dim contours, the clip's other takes). |
| S9 Twin | One idol / two idols side by side, each half as wide, with the contours echoing both. |
| S10 Wave | Off / On: the four bands undulate with a travelling sine (±90 px), crossing the idol like the 45_38 take. |
| S11 Source | Video / Osc. |
| P12 Burn | The flames. 0 = embers: the idol alone in black, faint outlines, dim halos, no teeth. 410 = the still. 100 % = inferno: every contour segment lit into fat orange and yellow stripes, halos ×2.2, teeth ×3. |

## Video
- **Video mode (the default).**
  - The camera feeds the line buffer at every other source pixel, as Y/8 − 8.
  - A per-field etch table turns each sample into an offset, and the offset is added to both the idol key and the
    contour field.
  - Bright parts of the picture swell the idol into yellow, and the contours bend around them like a contour map.
    At the default Etch, white adds 116 px.
  - With no input, Video mode is the still.
- **Osc mode.** A 160 × 135 virtual-px triangle lattice, drifting, signed ±64, takes the camera's place.
  - At the default Etch it is ±66 px: the contours reshape into a zigzag lattice and the idol's edges scallop.
  - Etch scales it. All the video-path controls stay live.

## Engine
- **Per-field sequencer** (vblank zone):
  - A shared glide for K1, K2, K4, K5, K6 and P12.
  - One serial shift-add multiplier with 22 slots, and a 512 × 16 ROM: exp2, reciprocals, the streak table and the
    ghost table.
  - Per field it computes:
    - the width gain and the table rate;
    - 1/λ for each side, as a 7-bit mantissa and a shift;
    - the band offset and the etch gain;
    - the Burn scalings: the line levels and slopes for the four classes, the halo and the teeth;
    - the scroll and the bob.
  - It then fills two tables: the line-level table (4 classes × 128 distances) and the etch table (256 samples).
- **Per-line engine.**
  - It is kicked at each line start and computes the NEXT line in about 250 clocks, which fits SD's 858-clock line.
  - The work: the table index (y × rate + scroll), the idol and contour half-widths × gain, the ghost width, the
    streak level and length, and the segment-class offset.
  - Then the band knots: 128 knot reads, two per segment and band. The winning band's knot pair goes into a
    ping-pong knot buffer (32 × 36).
- **Pixel path, 21 stages.**
  1. |x − axis|.
  2. Twin fold, knot address, etch table.
  3. F, the contour field and the ghost field.
  4. The contour distance × 1/λ (a 13 × 7 mantissa product, then a 0–5 shift), and the knot lerps.
  5. Contour index plus the per-line offset into the random class table (EBR); halo product; tooth length.
  6. Class into the line-level table (EBR).
  7. Compose, palette (EBR), and the expansion into Y, U and V.
- **Block RAM, 23 / 32 EBR.**
  - Idol table 1024 × 18 (5), knots (2), palette 512 × 24 (3), sequencer ROM (2), class table (1).
  - Knot buffer (3), etch table (1), line-level table (1), line buffer 1024 × 8 (2).
  - Chroma rings (2) and the sync ring (1).
  - Every collision-free RAM carries `no_rw_check`. The only transparency emulation left is the line buffer's, as in
    the shell.
- **Contracts.**
  - The generated avid (parity-locked).
  - One 28-clock latency in every mode (`processing_delay_clks` = 28).
  - Output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums that resync in blanking, and luma
    13/16.
  - The blanking gate.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr, V = Cb).
- **Verified (GHDL, 720p60, decimation 1, real timing ID).**
  - Default frames match the integer model `imodel.py` on 99.3 % of pixels, once it is shifted 4 px to the generated
    avid's start and blurred like the output filter. This holds for Video with a black input, Osc, and the K3 900
    motion capture at field 9.
  - The check found and fixed four bugs:
    - two narrowing `resize(signed)` wraps, on |x − axis| and on the contour distance;
    - a `shift_left` width wrap in the line-level fill;
    - four signed `x + (x >> n)` adds that put the same net on two LUT pins (router1 hung on these until they were
      rewritten in unsigned form; dupin is now clean).

## Status
- **v0.1.0, built 2026-11-09, ROUTER1. All six configs passed on seed 1.**
- Rebuild with `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh effigy`, or
  `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh effigy`.
- Post-route Fmax and LC per config:

  | Config | Fmax | LC |
  |---|---|---|
  | HD Analog | 86.56 MHz | 7118 |
  | HD HDMI | 85.60 MHz | 7082 |
  | HD Dual | 84.06 MHz | 7114 |
  | SD Analog | 79.08 MHz | 7114 |
  | SD HDMI | 86.72 MHz | 7116 |
  | SD Dual | 87.02 MHz | 7118 |

- That is 7082–7118 / 7680 LC (92.2–92.7 %, at the edge of the ~7100 budget) and 23 / 32 EBR.
- Every bitstream passed iceunpack. The output is `out/rev_b/ron/heirloom/effigy.vmprog`. **Not flashed.**
- **Do not use router2 for this netlist.** On an earlier 95 % netlist it stalled with overused = 2 on HD Analog seed 1.
- **Fit history.**
  - The first cut came to 9020 LC (117 %).
  - The second cut, 7320 LC, made these changes:
    - smaller multipliers;
    - a ROM for the segment hash;
    - a two-read band loop;
    - the knot ROM moved into EBR;
    - a 28-clock latency.
  - The third cut reached 7118 LC with these changes:
    - a half-resolution etch line buffer, with a lookup table in place of the etch multiplier;
    - the line-level table in place of the distance × slope multiplier;
    - a coarser G lerp;
    - the streak and ghost tables moved into the sequencer ROM.

## Deviations
- **Contour lines.** They are clean nested outlines of the idol profile, with a fixed spacing on each side. The
  still's lines are more irregular, and their spacing grows outward. Which segments are dim, bright or orange is a
  different random realisation; the orange shards on the left are not at the photo's exact places.
- **Bands.**
  - Built from 16 knots each, so the top-right red lozenge is a smooth lens with a dark core where the still's is
    ragged.
  - The thin gap inside the lens band, where the m-frames show a small yellow lens, is not modelled.
  - The teeth are a regular 64-px sawtooth with four hashed heights. The still's teeth are more irregular.
- **Idol.** The axis is fixed at x = 1002, without the per-row wander. The profile only scrolls; the clip's peaks
  also grow and shrink (22–36 px rms), and that morph is not modelled. Use K1 and K2 for shape changes.
- **Bands' motion.** The bands only bob vertically (±22 rows over 8 s). The clip's slight sideways sway is not
  modelled.
- **Left edge.** The streaks and the ghost blob are hashed and table-driven stand-ins. The blob is a 96-row parabolic
  lens, one per frame height.
- **Resolutions.** 720p shows the same picture as 1080p. SD shows the centre 4:3 (virtual x 240–1680), so on SD the
  left-edge streaks, the ghost blob and the outer end of the top-right lozenge fall off screen. SD is also softer.
- **Etch.** It samples every other source pixel (half horizontal resolution of the camera's luma).
- **Burn at 100 %.** It deliberately turns every contour into fat orange and yellow stripes, a busy "inferno" scene.
- **Budget.** LC sits at 92.7 %, about 18 LC over the ~7100 guide. Timing closed with wide margin (84–87 MHz on HD
  at seed 1).
