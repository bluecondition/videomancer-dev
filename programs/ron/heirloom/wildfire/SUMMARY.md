# Wildfire — black flame pits on gold, an egg column and a fan of fins, flowing down

**v0.1.1 built 2026-11-09, NOT flashed** (v0.1.0 built 2026-11-04). Heirloom phase 3 (video clips), from clip WIN_20230509_23_58_21 at 0.5 s
(`photos/vid/WIN_20230509_23_58_21_t0p5.png`). The clip shows a modular patch of two oscillators summed into a
high-gain RGB colouriser: black pits on a yellow ground, a column of eggs with thin olive "eyes", a fan of long fins
with green fringes that ends in a zig-zag edge, tall tilted bars on the left, a fine diagonal lace with a blue key in
the lower third, and a per-line strip down the left edge. Everything flows down; the whole clip is a 9.5 s cycle.
Presets are hand-written placeholders.

## v0.1.1 (2026-11-09): the two biggest regions brought to the still
Review of v0.1.0: the egg column, the eyes, the right fan, the motion and the controls were accepted; the top-left
blob and the lower third were not yet the still's.
- **Top-left black blob.** The V oscillator's knots left of x 330 are now low-frequency (0.0044-0.0065 rad per line,
  phase centred on pi), so x ~120-330 reads as one tall black mass from the top to ~line 400, with dark lobes below
  at x 120-250 that turn navy where the blue key starts (~line 560). The tilted bars stay at x 330-800.
- **Lower third in hot pink and red.** The lake's R/G split is stronger (-1.25 F below line 650), the lace now rides
  the split (D + aw cos w: its stripes alternate pink, red, green and pale), and the bottom ~120 lines are a solid
  hot-pink band (split -2 F, dimmed 1.6 F). Measured over lines 600-1075 (720p RTL render): pink 38 %, red 14 %,
  blue 15 %, pale 12 %, yellow 10 %, green 3 %, black 2 % (the still: 27 / 14.5 / 8 / 20 / 11 / 9 / 7 %).
- **Navy patches.** The blue key is a per-knot table again (the still's measured key line: from ~540 on the left,
  up under the egg column to ~570, down to 870 on the right). Key values above 60 (where the blue is already full)
  darken: 4 x (K - 60) is subtracted from the colouriser input. A per-knot depth (x 576-1216) times a line window
  (lines 520-690) puts a navy patch under the egg column; the left lobes give the other.
- **S8 Deep** is bolder: the saw on the H oscillator is 5/8 of a cycle (was 5/16), so every edge becomes a stack of
  teeth.
- The default frame no longer matches v0.1.0 below line ~450 and left of x 330; above and right of that it is the same.

## Reading of the photo
Working files: `.heirloom_work/rb_wildfire/` (notes.md, palfit, blobs.npy, vknots.py, proto.py / proto5.py,
knots.py, imodel.py).
- **Palette** (k-means, whole frame): yellow (235,238,5) 22 %, black (20,41,12) 18 %, pale yellow (223,208,104) 11 %,
  hot pink (243,91,117) 9.5 %, orange (238,170,12) 8 %, red (220,62,16) 4 %, blue (31,51,128) 4 %, cyan-green
  (67,212,111) 4 %. The top half has B ~ 0: a two-channel (R, G) colouriser.
- **Channels.** R and G carry the same pits with different thresholds that vary over the frame (no spatial offset,
  <= 2-8 px by cross-correlation): G turns on first on the right fan (green fringes on both sides of every yellow
  stripe), R first around the eggs and the left blobs (orange / brown fringes). B is a binary key (0 / ~120): black
  becomes blue, red becomes hot pink, yellow pale yellow, green cyan-green.
- **Pits.** Egg column at x 955-1005, centres y 63 / 141 / 221 / 318 / 450 (spacing 78, 80, 97, 132), eyes (local
  maxima, thin olive outlines) at x ~870 and ~1100 between them. Right fan x 1180-1700, fins at x 1400:
  y 26 / 94 / 159 / 224 / 296 / 371 / 446, slopes -0.25 .. -0.58 (steeper lower). Left: tall blob x 120-330,
  y 0-390, saddle at (190, 410), tilted bars x 330-800.
- **Vertical period** (autocorrelation): 76-79 lines on the fan and the column at the top, 58-67 at x 1500,
  19-25 in the lace below line ~600.
- **Field fit.** Gradient fits of cos U + cos V to the luma collapse to grey (tried three bases), so the V
  oscillator's phase was built from measured pit centres: 64-px knot columns, phi_v = c2 Y^2 + c1 Y + c0 per knot
  (rms <= 0.6 rad on the column and the fan), with Y a soft-clipped V ramp (slope 1 to line 450, easing to 0.12).
  The H oscillator is a slewed staircase (pi .. 5 pi between x 790 and 1230, flat over the fins, back to 4 pi past
  the zig-zag edge, the edge tilting in below line 250).
- **Lace and key.** A fine oscillator, 22 px, rising 20 deg to the right, VCA'd in from line ~520. Blue key under
  a line from (0, 600) to (1920, 880), edged by the lace.
- **Left strip** x 0-110: a per-line colour (green y 20-200, orange 240-360, pale / cyan below), i.e. the
  colouriser fed by a slow vertical oscillator while the H ramp is clamped.

## Motion in the clip
- Every region flows **down 8.8 lines per camera frame** (30.08 fps; corr 0.99): **4.4 lines per 60 Hz field**.
- It is not a rigid roll: correlation with the still at the best shift falls 0.85 (0.13 s), 0.61 (0.5 s), 0.45 (1 s),
  0.2 (4 s). The pits swell, the lace recedes downward, black floods in around 4-5 s, then it all returns.
- **The clip is periodic: the frame at 10.06 s matches the still (0.5 s) with corr 0.986, a 9.5 s cycle.** No knob
  is turned in it; it is free-running oscillators beating. The same patch's other clips (23_58_04: red-orange ground
  with zig-zag columns; 23_52_12: chirped horizontal bands) are other settings of the colouriser and frequencies.
- **What the program does:** at the default Speed the V phase flows the fins and eggs down ~4.2 lines per field
  (model: 2 rows of 540 per field), the lace scrolls with it, the H oscillator drifts one cycle per 9.5 s, and a
  breathing LFO with the same 9.5 s period (0 at the photo) slows the flames (x 0.4), sinks the lake (+500 lines)
  and raises the black threshold, then returns. Speed scales all of it (bipolar).

## What it does
1. Per field (vblank sequencer): glides, the Flames / Eggs gains (exp2 ROM), Fringe, Lace, Black, Etch, the speed
   steps, the breathing LFO, then a 256-word loop that scales the 31 knots' constants into a coefficient RAM.
2. Per line (engine kicked at each line start, computing the next line): the V ramp Y and Y^2, the lace amplitude,
   the lake dimming and its bottom band, the fringe ramp, the key ramp and the navy window, the strip values, the saw,
   then 31 knots x {V, H, D|K} into double-banked knot RAMs.
3. Per pixel: four DDAs interpolate the knot tables (V, H, D and the key K) exactly from each segment's first pixel;
   the lace phase is a linear DDA; the camera (or the Osc) adds to the V phase; three cosine ROMs; F = cos h + cos v +
   aw cos w; u0 = F - lake - threshold - 4 max(K - 60, 0); d' = d + aw cos w; R and G ROMs at u0 - 2d' and
   u0 + 5/8 d'; a B ROM by the key + the lace edge; Y / Cr / Cb summed from the three channel ROMs.
4. Output: one [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V (8-back samples from a block-RAM ring, resync to black),
   luma 13/16, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Eggs | the H oscillator (egg column, eye gaps, fin ends): density x0.25 .. x4 (512 = the photo). 0 = no segmentation, continuous fins and bars; 1023 = a grid of eggs |
| K2 Fringe | the R/G threshold split: 0 = plain black on yellow with a blue lake; 341 = the photo's green fan / orange eggs / hot-pink and red lake; 1023 = wide green and red bands |
| K3 Lace | the lower "lake": lace amplitude 0 .. 1.6 and how high the lace, the blue key, the navy patch, the dimming and the pink band reach (-384 .. +384 lines); 0 = no lace, the lake sinks off the bottom; 1023 = lace and blue over most of the frame |
| K4 Speed | bipolar flow (+-8 deadband); 700 = the clip's ~4.4 lines per field down and its 9.5 s breathing; full = 2.8x; left of centre flows up |
| K5 Black | colouriser threshold: thin black lines .. the photo .. black dominating |
| K6 Etch | how far the camera (or the Osc lattice) bends the V oscillator: 0 .. 3.7 cycles per full luma (160 = ~0.6) |
| S7 Fold | the oscillator sum folded before the colouriser: pits and peaks both go black, the eggs become ringed |
| S8 Zigzag | Fine (the photo's small saw teeth on the H oscillator) / Deep (10x, 5/8 of a cycle: the whole picture becomes stacked saw teeth) |
| S9 Palette | Wildfire (yellow ground) / Ember (red-orange ground, green only at the peaks; the same patch's 23_58_04 look) |
| S10 Invert | the oscillator sum inverted: peaks black, pits gold |
| S11 Source | Video / Osc |
| P12 Flames | V oscillator density x0.35 .. x3 (500 = the photo): 0 % = a few huge slow flames; 100 % = a fine fire lace of dense fins |

## Video
- **Video (default):** camera luma, (Y >> 2) - 16 from a one-line buffer, times Etch, is added to the V oscillator's
  phase, so the fins and eggs fray and bend around the picture while the composition stays. With no input it is the
  photo.
- **Osc:** a 640 x 540 virtual-px triangle lattice (half amplitude), drifting, feeds the same path: a gentle wobble at
  the default Etch, diamond lattices at full Etch.

## Engine
- **Shared multiplier** 16 x 13, 3-clock latency: the operands come from a one-hot AND-OR whose selects are
  registered one clock ahead (`p_opm`, no sequencer decode on the input), then three partials built from 2-bit
  digits ({0, a, 2a, 3a}, with 3a registered beside a), then one sum.
- **Vblank zone** (no avid for 1024 clocks): glide (6 channels), per-field scalars at zone clocks 25-48, coefficient
  loop 200-463 (KNOT ROM word x gain -> coefficient RAM; word 6 the key line, word 7 the navy depth), line-0 kick at
  600, line-0 prefetch at 900.
- **Line engine** (~280 clocks; fits SD's 858): 14 per-line products, then an 8-clock-per-knot loop (a Y2, b Y4, hs y,
  dxk m, tz win) writing the V, H and D|K knot RAMs (bank = next line).
- **Pixel path**: beam 5 clocks behind the generated avid; DDAs reload exactly at every 64-px knot (the next knot is
  read back during the segment; k0 / k0+1 prefetched in hblank). Etch computed in parallel (luma read 2 pixels ahead).
  D and K ride a 256 x 16 block-RAM ring to stage 9. One latency, C_LAT 24, in every mode.
- **Memories** (23 EBR): line buffer 4, cos x3, R / G colour ROMs 3 + 3 (512 x 24), KNOT ROM, EXP2 ROM, coefficient
  RAM, V / H / D knot RAMs, D|K ring, output-box ring 2, sync ring.
- **Contracts:** generated avid (parity-locked), blanking gate, the HD HDMI conditioning (one 9-tap box on Y/U/V,
  luma 13/16; drawn whites ~777), resync of the running sums, virtual 1080-line coordinates (SD shows the centre 4:3
  from virtual x 256), colours in the hardware convention (U = Cr), syncs delayed in a block-RAM ring.

## Status
- **v0.1.1 built 2026-11-09 with ROUTER1, NOT flashed.** `out/rev_b/ron/heirloom/wildfire.vmprog`.
  - Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=300 .heirloom_work/vid/build_queue.sh wildfire`
    (or `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=300 ./build_heirloom.sh wildfire`). Keep the short timeout:
    router1 stalled once on SD Analog seed 1 (v0.1.0) and the builder then moves on to the next seed.
  - **All six configs on seed 1.** Post-route Fmax: HD Analog 95.40, HD HDMI 91.04, HD Dual 94.72, SD Analog 93.99,
    SD HDMI 94.51, SD Dual 95.60 MHz.
  - 7035-7072 / 7680 LC (92 %), 23 / 32 EBR. iceunpack OK x6; the packed config carries the labels and presets.
  - router2 stalls on this netlist (overused = 2 for thousands of iterations); router1 routes each config in ~20 s.
  - v0.1.0 (2026-11-04): router1, SD Analog seed 2, the rest seed 1; HD 90.1-100.1 MHz; 6675-6689 LC.
- **Checks.**
  - GHDL analyse / elaborate / synth front end clean; TOML pre-check clean; dupin: no CARRY I0 == I1 (safe for router1).
  - **GHDL 720p (real timing ID, decimation 1), black input: 100 % of Y, U and V samples equal the integer model
    `imodel.py`** at field 3 (v0.1.1, `out_black11.txt`). In v0.1.0 a second capture three fields later also matched
    the model at field 6, with the fins moving 4.5 virtual lines per field.
  - `cmp_final.png`: still / proto (the integer model) / RTL Video with a black input / RTL Osc / RTL Video (selfie).
    `sheet.png` (from the bit-exact model).
  - Presets are hand-written placeholders; capture Ron's looks with `./capture_preset.sh`.

## Deviations from the photo
- **The lower third** now reads hot pink and red with navy patches, but it is still a fine lattice: the still's big
  dark diagonal strokes are not placed like the photo. A pale-yellow column runs below the egg column (the H
  oscillator's eye gap continues down), where the still is mixed pink / green / blue. Blue is a little high and pale and
  green low against the still. The navy patch under the egg column is squarer than the still's diagonal stroke.
- **The top-left blob** is one tall dark mass without the still's saddle at (190, 410); its lower lobes turn navy
  below ~line 560 and stay darkish to the bottom on the left.
- **The left bars** (x 330-800) are more regular than the still's tilted ellipse and staggered blobs.
- The egg column, the eyes and the right fan sit where the still has them. The fan's zig-zag edge is softer and its saw
  teeth smaller than the photo's (S8 Deep exaggerates them).
- The R/G split is a smooth x/y field (green right, orange centre-left, red and pink below line ~450). In the still it
  also varies with each blob.
- **Motion:** the local flow matches (4.5 vs 4.4 lines per field). The 9.5 s cycle is a smooth breathing LFO, so the
  pits swell and the lace sinks more slowly in the first second than in the clip, and the mid-cycle frames have no
  blue/cyan bands.
- The clip's rolling horizontal seams / glitch lines are not drawn.
- **Resolutions:** 720p shows the same picture as 1080p. SD shows the centre 4:3 (virtual x 256-1696), so the left
  strip and the right edge of the fan are off-screen there.
- Drawn whites are ~777 (the shell's 13/16 luma cap). One 9-tap box replaces the photo's slightly softer blur.
