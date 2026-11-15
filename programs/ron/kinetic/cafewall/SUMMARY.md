# Cafewall (Kinetic)

**v0.1.1 built 2026-11-15 (K6 camera pass), not flashed.** Flash `out/rev_b/ron/kinetic/cafewall.vmprog`.

## What it draws

The cafe wall illusion (Gregory and Heard, 1979; Munsterberg's shifted chessboard before it): rows of
alternating dark and light tiles, each row shifted against the next, with thin mid-tone mortar lines between
the rows. The mortar lines are parallel and straight but look like long wedges. The default frame (Ink
palette) is about twelve rows of black and off-white tiles, 1.6 times as wide as high, with thin grey mortar;
the rows' shifts follow a sine down the wall, so the tile edges trace a sinuous column and the mortar lines
seem to tilt one way where the shift grows and the other way where it falls. The wave travels down the wall
(about 7 s per cycle at the default Rate, 60 fields per second), so the illusory tilts ripple up and down.

## Element and state

Element: a row of tiles (height H, tile width W = H x aspect). State: its horizontal shift, in tiles.

    theta_j = Spread * index_j - phase       index: Linear = row number from the centre row; Radial = |row number|
    shift_j = SHIFT * wave(theta_j)          0 .. 2 tiles. Odd rows start one tile over, so shift 0 is a
                                             checkerboard; Rows = Alternate negates the shift on odd rows
    tile(x) = floor((x - centre) / W + shift_j)        Radial: |x - centre|, so the row is mirrored about the
                                                       screen centre and the tiles flow out of / into the axis
    mortar  = the first M virtual lines of each row, centred on the row boundary

Row 0 is centred on the screen centre, so Scale zooms about the centre.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | row height 24 .. 288 virtual lines, exponential, glided (default 93: about twelve rows) |
| K2 | Shape | tile aspect 1 .. 4: squares .. long bricks (default 1.6) |
| K3 | Mortar | thickness 2.5 % .. 37 % of the row height (square law; never under 2 field lines) and tone 1/4 .. 3/4 of the way from the ground tile to the A tile (default: 7 % and a little under the mid tone) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Colour | the 16 shared palettes with hysteresis: ground and A = the two tiles, mortar between them, B = the tint the picture gives the A tiles, accent = Reveal's mortar (K6 before v0.1.1) |
| K6 | Camera | the picture's tone strength (0 .. 100 %, default 50 %), glided. 0 = none (the plain wall), 25 % = v0.1.0's tones, 50 % = the picture's contrast doubled, 100 % = x 4 (a near two-tone mosaic), always inside v0.1.0's tone range so the two tile sets never meet. Scales Source = Osc too. (Spread, K5 before v0.1.1, is fixed at 420: one slow S down the wall) |
| S7 | Wave | Smooth (sine) / Snap (rows dwell at the two ends and flip quickly: at SHIFT 50 % a brick bond with fault lines that travel) |
| S8 | Field | Linear (the wave runs down the wall) / Radial (rows above and below the centre move alike and every row is mirrored about the centre, so the wall is four-fold symmetric and the tiles stream out of the centre axis) |
| S9 | Rows | Same / Alternate (every other row moves the opposite way: the Munsterberg zigzag) |
| S10 | Reveal | Off / On: every tile tone is pulled 3/4 of the way to the mid tone (two close tones) and the mortar takes the accent colour, so the lines are plainly straight |
| S11 | Source | Video / Osc |
| P12 | SHIFT | the headline: wave depth, square law, glided. 0 % = a plain checkerboard with straight lines, 50 % (default) = the classic half-tile wall, 100 % = rows sliding two whole tiles each way |

Depth is the headline, so it sits on P12 and K3 became Mortar (the spec's rule for this program).
Presets: Cafe Wall (defaults), Munsterberg, Brick Flip, Pond Tiles, Straight Lines, Mosaic, Landslide.

## Video

`program_type = "processing"`. The picture's luma is the second modulator and drives a second property of the
element, its tiles' TONE: luma x 1.5 -> 1/16 one-pole filter along x (restarted every line) -> sampled at each
tile's last pixel on the row's first line (which is always all mortar) into a block RAM indexed by tile number
-> held for the rest of the row, so every tile is one flat tone. 16 steps: ground tiles go toward A (0.70 of
the way at full luma), A tiles go toward B (0.23). Since v0.1.1 the step goes through the K6 Camera law first
(see "K6 camera pass" below). The picture shows as a mosaic in the wall while the two
tile sets stay apart, so the illusion holds; it reads as a picture at small Scale. With a black input both
steps are zero and the wall is the plain two-tone one. Source = Osc substitutes the shell's drifting
640 x 540 triangle lattice at twice the gain (clipped), so diamonds of tone drift through the wall.
Reveal divides the picture's contrast by four along with the tiles'.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): glides on Scale and SHIFT (1/8 of the gap per field),
  one 24-step serial multiplier (row height through a 2^(i/256) ROM, tile width, mortar, SHIFT^2, Spread^2,
  rate), the wave phase step, the palette, the tile table (2 x 16 tones by accumulators) and the mortar colour,
  and two adder loops for the origins (row 0 on the centre line; the tile counter's value at pixel 0).
- **Per line**: a row counter (position in the row, row number, Spread x number) steps at every line start;
  then a 45-clock engine on a second small serial multiplier works out the NEXT line's values: theta -> quarter-
  wave sine ROM (or Snap) -> SHIFT x wave -> mod 2 tiles -> x tile width -> the tile counter's start value and
  parity; and the line's mortar coverage in eighths (the mortar is anti-aliased vertically).
- **Per pixel** (15 stages, no multiplier): a tile counter in 1/16 virtual px (Radial: counting down to the
  screen centre, then up) -> the pixel that holds a tile edge gets the next tile's share of it in sixteenths ->
  held luma step (RAM) -> tile colour (table RAM) -> shift-add blend with the next pixel's colour -> shift-add
  mortar blend -> output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums, luma [1 2 1]/4
  delayed to the same group delay, luma 13/16 toward black) -> blanking gate.
- Generated avid (the shell's parity-locked start), one latency (22) in every mode, virtual 1080-line
  coordinates from the timing ID (SD shows the centre 4:3). Luma 10 bits, chroma 8 bits until the output
  filter. Colours in the hardware convention (U register = Cr). Drawn white comes out at Y = 733.
- Chassis blocks (lifted from louver) are tagged [A] .. [F] in cafewall.vhd.

## Status

- v0.1.0 built 2026-11-14 through the shared queue with **router1**
  (`NEXTPNR_ROUTER="--router router1"`), every config on seed 1. Not flashed.

| Config | Router | Seed | Post-route Fmax | LC |
|---|---|---|---|---|
| HD Analog | router1 | 1 | 92.31 MHz | 5822 |
| SD Analog | router1 | 1 | 101.13 MHz | 5818 |
| HD HDMI | router1 | 1 | 87.17 MHz | 5827 |
| SD HDMI | router1 | 1 | 89.81 MHz | 5823 |
| HD Dual | router1 | 1 | 97.87 MHz | 5818 |
| SD Dual | router1 | 1 | 97.58 MHz | 5833 |

- 5,833 / 7,680 LC (76 %), 11 / 32 block RAMs. Six bitstreams pass iceunpack. HD needs 74.25 MHz, SD 27.
- The first build used router2 (the builder default): HD Analog 81.63, SD Analog 92.43, HD HDMI 76.56, SD HDMI
  79.11 MHz on seed 1, then **HD Dual stalled the router** on seed 1 (15-minute timeout) and seed 2 was still
  routing after 3 minutes when that build was stopped. Rebuild with router1:
  `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic cafewall`.
- dupin on the synthesised netlist: no carry self-adds (3 duplicate-input LUTs on sign-extension bits only).
  Standalone yosys: no `emulate_transparency`.
- Model = RTL (GHDL): pixel diff 0 on every render made:
  - 720p60, decimation 1, the whole frame (918,720 px): defaults with a black input; Source = Osc; Video with
    selfie.jpg (the model fed the tester's own input image): 0 each;
  - 720p60, decimation 2 (the top-left 640 x 360): Snap + Radial + Alternate + Reveal with Scale 300, Shape 600,
    Mortar 800, Rate 200, Spread 700, Colour 10, SHIFT 800 (glides bypassed in a sim-only copy): 0 of 228,960;
    Osc and selfie again: 0;
  - NTSC, decimation 1, Radial: 0 of 173,988 on each of the two fields (720 x 243).
  The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the left (generated avid).
  Not rendered: 1080i, 1080p; Radial's right half (past the mirror) was rendered on NTSC only. The first
  three renders used the default Rate 640; the TOML default became 740 before the final build (a TOML value
  only, the VHDL is the simulated one).
- Renders: `.kinetic_work/rb_cafewall/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion); both
  sheets are from the integer model.

## K6 camera pass (2026-11-15)

- **K6 = Camera**, 0 .. 100 %, default 50 % (512), glided (the shared glide, now three channels; +-4 deadband).
  The filtered sample s (8 bits; Osc keeps its x 2, clipped) is multiplied by the glided value k in its own
  stage, then the tone step is **l = clamp((s k - 120 (k - 256)) >> 12, 0, 15)**, with the offset 0 below
  k = 256. So up to 25 % the step is k / 256 of v0.1.0's (0 % = the plain wall, exactly the black-input
  picture; 25 % = v0.1.0's step bit for bit); above 25 % the picture's contrast is stretched g = k / 256 times
  about the middle of the 16 steps (50 % = 2 x, 100 % = 4 x, a near two-tone mosaic), clamped to v0.1.0's tone
  range. The tile table (16 steps) is unchanged.
- Why not 2 x in amplitude: v0.1.0 already takes the ground tiles 0.70 of the way to A at full luma, so doubling
  the deflection crosses the two tile sets and the wall washes to one light grey on a bright picture (tried
  first: last panel of the sheet). Inside the v0.1.0 range the two tile sets never meet, so the illusion holds
  at every K6 setting; at 100 % on a bright picture most ground tiles sit at the top step (grey next to the
  off-white A tiles), on a dark one they split black / grey.
- GHDL renders (720p60, decimation 2, selfie.jpg, glide started at the K6 value in a sim-only copy):
  `.kinetic_work/rb_cafewall/k6sheet.png`.
- **K5 Spread is fixed at 420** (its default: one slow S down the wall); K6's old function, **Colour (all 16
  palettes), moved to K5** (with its hysteresis). The presets' Spread values (180 .. 520) are gone; their
  palettes moved to K5.
- The hold sample now sits 3 pixels further along the line (the multiply and the stretch stages), inside the
  16-pixel filter.
- Build v0.1.1 (`.kinetic_work/tools/k6build.sh`): **router1**. HD Analog 98.48 MHz on **seed 2** (seed 1
  stalled the router, 600 s timeout), HD HDMI 97.74 (seed 1), HD Dual 97.54 on **seed 3** (seeds 1 and 2
  stalled), SD Analog 97.01, SD HDMI 101.71, SD Dual 95.49 (seed 1). 6,125 .. 6,144 / 7,680 LC (80 %), 11 / 32
  EBR.

## Cut / deviations

- No float prototype was kept: the integer model (`imodel.py`, RTL widths) was the prototype (three look
  rounds). Sheet and strip are from it.
- Radial also mirrors each row about the screen centre (not in the spec): with rows as the elements, |row
  number| alone was too close to Linear on the control sheet.
- K3 moves thickness and tone together (one knob): thin + darker at the bottom, thick + lighter at the top.
- The mortar is never thinner than 2 field lines, so on SD (4.4 virtual lines per field line) the thinnest
  mortar is about 9 virtual lines where HD draws 2: at small Scale the SD wall has proportionally thicker
  lines. A 2-line mortar is drawn as one full line and soft partial lines.
- The picture reaches the tiles as 16 tone steps, and only at small Scale does it read as a picture (twelve
  rows of thirteen tiles cannot show a face). The top row of the field uses the hold sampled one field
  earlier (it has no mortar line above it on screen to sample on).
- The luma filter is 16 real pixels at every resolution (twice as wide in picture terms on SD).
- The left 4 px of the picture blend to the mortar colour in chroma (the 9-tap filter meets the line start).
- `field_n` is delayed with the syncs as in the shell; the core ties it high on every input, so interlaced
  fields draw the same lines (SD and 1080i are line-doubled, as in every program).
