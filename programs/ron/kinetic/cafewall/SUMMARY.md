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

## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. Ron's rule after the K6 = 100 % renders: bend / element programs get a 25 % stronger camera at the
top of K6; picture programs keep their logic. The K6 default is now 100 % (initial 1023).

- **Classification: element.** K6 sets how strongly the camera's luma recolours the tiles (the tone step of
  each tile in the drawn wall). The wall itself is never the camera image. It is close to a picture mosaic,
  but the set rule counts every Kinetic program as element.
- **Change:** the per-field gain is k + k / 4, latched at zone clock 24 (`c_kg`, 11 bits, 0 .. 1278), so
  g = 1.25 k / 256. That puts v0.1.0's step at 20 %, 2.5 x at 50 % and **5 x at 100 %** (was 4 x). The
  stretch offset 120 (k' - 256) follows (17 bits, at most 122,640), and the result is still clamped to the
  v0.1.0 tone range. The pixel-path multiply is now 8 x 11 (was 8 x 10), with a 19-bit product and a 20-bit
  offset subtract. Latency unchanged.
- **Look check:** `.kinetic_work/rb_cafewall/k6b/k6b_cmp.png` (GHDL, 720p60 / 4, bear.png, glides snapped
  in both copies), old against new at K6 = 100 %. It is the same wall with the tile tones a little closer
  to two-tone (9.5 % of pixels changed). No wrap. The 4 x top was already near the clamp, so the gain is mild.
- **Build** (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic cafewall`): router1, every config on seed 1
  (no router stalls this time). Post-route Fmax HD Analog 101.42, HD HDMI 88.83, HD Dual 104.36 MHz (worst HD
  88.83); SD Analog 101.28, SD HDMI 100.46, SD Dual 97.48. 6,148 .. 6,165 / 7,680 LC (80 %), 11 / 32 EBR.
  vmprog: out/rev_b/ron/kinetic/cafewall.vmprog (v0.1.2). Not flashed.

## v0.1.3: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`0 - v` as `+v`); the simulator is
right, so every render above showed the intended picture, but the v0.1.2 bitstreams did not. Fixed (same operands
and width, the value the simulator always computed):
- line 570 `c_nxq <= -v_xq` (was `0 - v_xq`): the tile counter's step while it counts DOWN from the left edge to
  the screen centre in S8 Field = Radial (a_dec = '1');
- line 921 `v_fn := unsigned(to_signed(0, 6) - v_d(5 downto 0))`: Radial's left half, the edge pixel's share of the
  next tile in the down-counting direction (the anti-aliasing blend); it was masked by line 570 and would have
  blended edges the wrong way once 570 alone was fixed.
On hardware v0.1.2 (never flashed) Radial would have counted UP in the left half and never crossed a tile edge:
each row's left half one solid tile colour, then a strip of ~1-px stripes at the centre and scrambled per-row
tile phase in the right half. v0.1.3 draws the designed Radial wall, each row mirrored about the centre with tiles
streaming out of the centre axis and anti-aliased edges on both halves. Linear mode is unchanged. No TOML remaps
(group 1). Nothing left as-is (no other audited site in this program). `scan.py` lists no site. Backups (v0.1.2
vhd / toml / vmprog): .heirloom_work/rb_cafewall/.

Build (2026-11-21, shared queue, router1, every config seed 1): HD Analog 91.95, SD Analog 86.29, HD HDMI 90.16,
SD HDMI 95.43, HD Dual 89.41, SD Dual 88.25 MHz; 6,188-6,206 / 7,680 LC (81 %), 11 / 32 EBR; all six pass iceunpack.

HD HDMI skew: the builder's HD HDMI router1 s1 was +0.06 / <= -0.88 (HSYNC below the window). Screened HD HDMI
router1 + router2 seeds 1-40 (68 routed, 12 timeouts under load, 64 timing-clean, 8 in the window). Installed
**HD HDMI router2 s23 (+1.90 / <= -1.08, 83.89 MHz)**. HD Analog router1 s1 had data <= -0.33: installed **HD Analog
router1 s9 (-0.92 / <= -0.74, 91.95 MHz)**. HD Dual router1 s1 had data <= +0.67: installed **HD Dual router1 s10
(-2.21 / <= -1.19, 89.59 MHz)**. So the shipped seeds are HD Analog r1 s9, HD HDMI r2 s23, HD Dual r1 s10, SD
configs r1 s1. Spare: HD HDMI router2 s20 (+1.31 / <= -1.27, 84.94 MHz),
.heirloom_work/rb_cafewall/skew/cafewall_v013_spare_hd_hdmi_router2_s20.vmprog. Replaced bins kept in
.heirloom_work/rb_cafewall/skew/. Shipped vmprog: out/rev_b/ron/kinetic/cafewall.vmprog (v0.1.3). Not flashed.
