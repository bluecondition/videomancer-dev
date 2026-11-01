# Cutout — a collage head of torn magazine scraps on a yellow ground (Heirloom #55)

**v0.6.0 (2026-11-01, phase 2): every scrap is its own module (crop, flip, scale, drift); built, NOT
flashed.** v0.5.0 (2026-10-28) rebuilt it from the photo. Source: `photos/tezcatlipoca.jpg`.
v0.1.0 posterized the video inside an oval head shape, with no relation to the collage's pieces. This
version rebuilds the collage: the measured head cut-out, its 53 traced torn scraps, the soup-cup eye,
the plate of pastries, the stone teeth, the "Love." label and the black jaw. Every scrap prints the
camera picture. Osc mode substitutes a generated magazine photo.
Working files: `.heirloom_work/rb_cutout/` (`notes.md`, `pieces.py`, `features.py`, `mapbuild2.py`,
`fams.py`, `gendata.py`, `proto.py`, `imodel.py`, `sheet.png`, `cmp_final.png`, `cmp_motion.png`); phase 2 in
`.heirloom_work/rb_cutout/p6/` (`imodel6.py`, the v0.6 integer model; `sheet6.png`; `rtl_vs_model.png`).

## Phase 2 (v0.6.0)
Ron 2026-11-01, before flashing v0.5, picked these from a proposed map:
- **K2 Zoom -> Crops.** 0 % = every scrap prints one camera frame in register, zoomed so the frame spans the
  head (centred). 50 % = the photo's crops. Up from there each scrap's crop slides off on its own (the crop
  offsets scale x1 .. x4), and the scraps flip over and change scale one at a time. Below 50 % they lose
  their flips and scale one at a time as they come into register. The global zoom is gone.
- **K6 Speed** now drives each scrap's drift on its own line, rate and direction (ping-pong, reversing at
  the end of its travel; the old common pan is gone). It also crawls the "Love." text along its label.
- **S8 Fringe -> Inks: Photo / Swap.** Swap trades the ink ramps: the blues and blacks go to warm inks of
  similar luma (maroon, red, cream, gold, olive, pink) and the cup and plate go to blues. The white fringe
  moved onto **K1 Rag**: none at clean scissors, the photo's at 50 %, thick and on every scrap when shredded.
- **S10 Eye: Soup / Mirror.** Mirror is now Tezcatlipoca's mirror showing the live camera, mirrored, in a
  grey ramp, centred in the soup (it was a glossy black soup).
- **P12 unchanged.** **K3 Ink** range is now 0 = flat paper .. 50 % the photo .. 100 % hard (x1.9 the photo).
- The default frame matches v0.5 (the scrap crops differ by under 0.25 store px at power-up; the tear and
  brush noise run on 12-bit phases, the same look).

## Reading of the photo (measured)
The photo is a portrait paper collage, 4776 × 5987 (0.80:1). Measurements below are in page px of a
1276 × 1600 working copy (H = page height).
- **Ground.** Flat cadmium-yellow acrylic, (255, 254, 37), hardware 10-bit Y 852–856, U 573–576,
  V 120–170. The brush texture is faint: Y sd 2–5 codes, V sd 5–11 codes. Near the head the strokes
  run parallel to the cut edge.
- **Head.** One cut-out silhouette in profile, facing left.
  - Rows 147–1433 (0.804 H tall, top at 0.092 H). Widest 893 px (0.558 H) at y 827, x 245–1137.
  - The cranium dome is at the top; a yellow notch marks the upper lip (x 270–330, y 1050–1110); an
    ear bump sits at the back (x 1080–1160, y 790–1000).
  - The edge is a clean painted line, one span per row.
- **Torn scraps.** 53 polygons were traced (`poly_over_*.png`):
  - top: glossy black night, a grey beach with tiny upside-down people, a deep-blue "eye" swirl, a
    khaki aerial-landscape strip down the back of the head, dark aerial forest;
  - a blue surf strip (y 335–440), a teal "VS." box, a royal-blue "12 HRS / AVAILABLE NOW" strip;
  - the black band across the mid-face (y 730–1010): crumpled black pieces with white scratches, a
    pink triangle, maroon, dark olive, slate;
  - the lower face: a night road with light trails, a grey ornament building, navy denim, teal foam,
    sky with clouds, grey-blue and light-blue papers;
  - the jaw: black and navy, with blue knit, brown hair, teal and a denim zip strip.
- **Scrap palette.** The head is 30 % near-black (Y 177–228). Grouped into 16 scrap families plus 15
  feature families, each ink ramp is 16 luma-quantile colours of that family's photo pixels.
  - Local texture contrast (luma sd in 15-px windows) is 27–87 codes: black 37, sea 64, teal 78,
    stone 87.
  - Torn edges are ragged at 5–9 px rms about a line, with fine fibres of 1.4–5 px; the
    autocorrelation is ~0 at 20 px. About a third of the edges show a white paper-core fringe.
- **Cup (the eye).** Tilted 15°, the top leaning right.
  - Soup: ellipse at (353, 622), 47 × 78, smooth red-brown.
  - Cream rim: 59 × 90, wider on the left.
  - Green body, offset lower-right: (371, 641), 62 × 92.
  - Dark saucer: (408, 592), 56 × 72.
  - Garnish: r 21 at (367, 613), with an orange core of r 12.
- **Plate (the cheek).**
  - The plate: (730, 605), 180 × 160, blue-teal, with a teal rim band over the outer 26 %.
  - Glass bowl ring: (682, 615), 97 × 136.
  - Gravy: dish (700, 632) 62 × 108, pool (676, 634) 45 × 104.
  - Pastries: (815, 540) 104 × 84 tilted and (800, 716) 80 × 60, plus two crescents on the left.
  - Two forks and a spoon handle.
- **Teeth.** 10 upper and 7 lower rounded grey stones on black fabric (x 268–725, y 1065–1210). One is
  an upside-down face photo, one is a pair of glasses.
- **Label.** "Love." in blue (60, 150, 210) on pale paper (770–940, 1190–1282), tilted −8.5°.

## 16:9 mapping
- The frame height equals the page height. The page is centred and the yellow ground continues to the
  frame edges.
- Geometry is in map units (0.75 virtual px of a 1080-line virtual frame), stepped from the timing
  ID. 720p is identical to 1080p.
- 4:3 SD shows the whole head with narrower yellow margins; the cup and plate stay round.

## What it does
1. **Piece map.** A 54 × 74 cell map of square 12-virtual-px cells holds, per cell, the scrap id
   (5 bits) and the distance to the nearest other id (3 bits). The ids are 1–22 torn scraps, then
   saucer, cup, garnish, plate, plate rim, gravy dish, gravy pool, pastries and label paper.
   - The four cells round a pixel come from four bank ROMs. Their signed distance is interpolated
     bilinearly (3-bit fractions), so the edges come back at sub-cell precision.
   - A tear noise is added to the distance: two triangle plane waves plus a positional hash, scaled
     by K1. It is ×1/4 on the painted head edge and ×1/8 on the scissor-cut features.
   - The sign selects the scrap. The distance gives the rings: the cup-body edge, the saucer rim, the
     plate's inner line, the garnish core, and the shading of the gravy and pastries.
2. **Camera print.** Every scrap reads a 128 × 64 × 4-bit store through its own slot word from the
   crop RAM: crop (Wu, Wv), x/y flip and scale (2 or 4 virtual px per store pixel at the photo's zoom). The
   field sequencer recomputes all 33 slot words every field (the teeth have slot 0, the camera mirror slot
   32) from K2 Crops and K6 Speed.
   - The read is nearest-neighbour.
   - The level passes the K3 contrast plus a shade, a surface texture (grain, or specks on the
     blacks) and dither.
   - A 512-entry ink ROM (family × 16 levels) gives the colour.
3. **Features.**
   - Soup and cream rim: chord quadratics `(x − xc(y))² + k·dy²`, one squarer per pixel.
   - Teeth: a 3-virtual-px detail ROM with rim / stone / highlight codes.
   - "Love.": a 64 × 20 glyph in the same ROM, sheared 5/32.
   - Forks: one x-range per line.
4. **Ground.** The yellow ink ramp with a brush wave and paint rings round the head edge, plus the
   K5 paint offset.
5. **Fringe.** White paper core on the near side of a third of the scrap edges (hash-gapped, 7 × the
   K1 rag gain + 0..15 units wide: 35–50 at the photo; on every scrap edge from K1 ≈ 75 %).
6. **Tear and jaw.** Eight strips separate along the measured horizontal tears, at page y 335, 500,
   742, 930, 1005, 1150 and 1300.
   - The boundary is ragged by ±4 map units.
   - Each strip's x offset is P12 × amplitude × sway, applied to both the map and the store
     coordinates.
   - The jaw shifts every line below the mouth line down by J; the gap shows the yellow ground.
7. **Output.** Shell output conditioning, then the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Rag | the torn edges, the paper core | 0 = clean scissor cuts, no fringe; 50 % = the photo's tears and fringe; 100 % = shredded, thick fringe on every scrap (32-step gain table) |
| K2 Crops | the pictures inside the scraps | 0 = all scraps in register on one camera frame zoomed to span the head; 50 % = the photo; 100 % = crops slid off x4, every scrap flipped and rescaled (one at a time along the travel) |
| K3 Ink | the scraps' texture contrast | 0 = flat paper colours → 50 % = the photo → 100 % = hard (×1.9) |
| K4 Jaw | the jaw | 0 = closed (the photo) → the jaw dropped by 0.10 H, yellow in the gap |
| K5 Paint | the yellow ground | 8 positions: yellow (the photo), orange, red, magenta, violet, blue, cyan, green |
| K6 Speed | (motion) the scraps' pictures, the label, the Osc photo, the strips | bipolar, still in the centre (±16). Each scrap's picture drifts back and forth on its own line (periods 4–17 s at the default 640, amplitude ×Crops); "Love." crawls along its label (8.5 s a lap at the default); the Osc photo drifts ~15 px/s; the strips sway with a 4.3 s period when P12 > 0 |
| S7 Facing | the profile | Left (the photo) / Right (mirrored) |
| S8 Inks | the scrap colours | Photo / Swap (blues and blacks → warm inks, the cup and plate → blues) |
| S9 Chatter | the jaw | Off (the photo) / On: the jaw chatters at 2 Hz, at least 35 % open |
| S10 Eye | the soup cup | Soup (the photo) / Mirror (the live camera, mirrored, grey, in the soup) |
| S11 Source | the pictures in the scraps | Video (default) / Osc |
| **P12 Tear** | the horizontal tears through the head | 0 = the intact collage (the photo); up: 8 strips slide apart alternately and sway; 100 % = flung up to ±560 virtual px across the frame |

- P12 glides (1/8 of the gap per field, ±4 deadband, the tail snaps). K2 follows the knob outside a ±3
  deadband, once per field.
- K1, K3 and K5 use 32-step indices with hysteresis, so ADC noise does not flicker the tables.
- The control sheet is `.heirloom_work/rb_cutout/p6/sheet6.png` (from the v0.6 integer model).

## Video
- The camera picture is written (box-filtered) into the 128 × 64 store over the whole frame.
- Each scrap prints a different crop, at its own scale and flip, through its own measured ink ramp:
  blacks stay black, sky pieces stay sky blue. The head reads as a collage cut from the live
  picture.
- The cup, saucer, plate, gravy, pastries and label paper print the camera through their own ramps
  too. The teeth print it in stone grey.
- Osc writes a generated "magazine photo" into the store instead (two broad triangle-wave gradients
  that drift).
- Every control acts in both modes.

## Engine
- **Field sequencer** (vblank, from zone clock 64): one serial multiplier in 24-clock slots (select and
  crop-RAM read, operand prep, load, 16 multiply clocks, write), with registered one-hot operand selects.
  - Phase A, 30 ops: the chatter jaw, the zoom products (store x step and start, store y step and start,
    jaw store offset), then per strip the sway amplitude, the map offset and the store offset. Results go
    to a strip RAM (one EBR). Its spare words hold the zoom (now K2 Crops' zoom), rag-gain and paint
    tables, read at zone clocks 40–47.
  - Phase B, 4 ops for each of 33 crop slots (~3.2k clocks; it ends ~4.0k clocks into vblank, inside
    every mode's vblank):
    `Du = (tri8((Tph << e) + 0x4000) xor su − 128) · cu`; `Wu = 2A · ((ou − 40) · 256 + Du) >> 10, + 40 · 256`;
    the same for v. A (Q.9) = g below the K2 centre and 3g − 1024 above (0 .. 1 .. 4). The W ops take D
    from the product register, untouched since the D op. Flip and scale step against per-slot
    thresholds t1 / t2 / t3. Each W op writes its half of the slot word straight from the product
    register in its last clock.
  - The slot constants (e, su, sv, cu, cv; ou', ov', flip, k, t1, t2, t3, f2) are generated by `cutout.py`
    (`_crop_consts`, seeded) into the crop RAM's spare words; `scrap_words()` there is the bit-exact
    reference the model uses.
- **Line prep** (hblank, 20 clocks after the generated avid falls; line 0 at zone clock 850). It
  writes the line's live registers directly:
  - the map-y, store-y, noise-phase and brush DDAs;
  - jaw and gap;
  - strip and its offsets (strip RAM reads);
  - map row bases (×27) and row fraction;
  - the cup chord centre, and k·dy² through the pixel path's squarer;
  - teeth row base (×105), glyph v, fork range, the row-hash seed.
- **Pixel path (32 stages):**
  - S0: noise DDAs. S1–S3: rag, strip side, torn map x.
  - S4–S8: map ROM addresses and data, cup quadratic, detail ROM, hash.
  - S8–S13: SDF corners, reference id, lerps, + noise, scrap id.
  - S14–S16: parameters, rings, features, texture and offset; S14 sends the slot (scrap id, teeth 0,
    camera mirror 32) to the crop RAM, whose word is fabric-registered at S16; S16 the Inks swap.
  - S15–S17: the store-x DDA, the strip offset, then scale, flips and crop from the slot word, store address.
  - S18–S19: store. S20–S21: contrast and level. S22–S23: ink ROM. S24: compose.
  - S25–S32: conditioning and gate.
- **27 EBR:** store 8; box-sum accumulator 1; map banks 8; ink ROM 3; detail ROM 2; strip RAM 1;
  sync/avid delay 1; crop RAM 2 (two 16-bit halves, 256 deep); the output chroma filter's 8-back tap 1
  (a 256 × 16 ring read 6 behind the write, which replaced 112 delay flip-flops). The collision-free RAMs
  carry `no_rw_check`.
- **Contracts:**
  - generated avid (parity-locked, nominal width);
  - virtual 1080-line geometry from the timing ID;
  - one 32-clock latency in every mode (`processing_delay_clks` = 32);
  - output conditioning (luma [1 2 1] and scaled toward ~777; chroma [1 2 2 2 2 2 2 2 1]/16) before
    the blanking gate;
  - colours in the hardware convention (u = Cr), from `cutout.py` → `cutout_pkg.vhd`.

## Status
- **v0.6.0 built 2026-11-01, ROUTER1** (`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900
  ./build_heirloom.sh cutout`), **builder 6/6 on seed 1; HD seeds then re-picked by a pin-skew screen. NOT
  flashed.**
  - Fit: 7490–7511 / 7680 LC (97.5–97.8 %), 27/32 EBR. All six bitstreams pass iceunpack.
  - The first build overflowed (7728 LC). Fixed with no feature lost:
    - the crop RAM split into two 16-bit halves written straight from the product register;
    - the D product reused from the product register;
    - the drift rate as a shift;
    - K2 a deadband latch instead of a second glide;
    - Ink without its range adder;
    - 12-bit tear-noise and brush phases;
    - the output chroma filter's 112-FF delay lines moved into a block-RAM ring.
  - Builder post-route Fmax (seed 1): HD Analog 74.97, SD Analog 77.91, HD HDMI 77.56, SD HDMI 77.30,
    HD Dual 75.54, SD Dual 76.09 MHz.
  - **Shipped HD bitstreams, hand-installed from a router1 pin-skew screen and repacked** (screen:
    `.heirloom_work/rb_cutout/p6/skew/screen*.txt`; HD HDMI s1–24, HD Analog s1–30 plus router2 s1–24, HD
    Dual s1–12). The builder's seed 1 failed all three: HD HDMI HSYNC +0.38; HD Dual data up to +0.57; HD
    Analog +0.70 / ≤ −0.61.
    - HD HDMI **router1 s17**: 81.39 MHz, HSYNC +2.01 / data ≤ −0.72 ns.
    - HD Analog **router1 s7**: 79.47 MHz, HDMI HSYNC +0.70 / data ≤ −1.16, analog −2.21 / ≤ −1.57. No
      HD Analog seed reached +1.0..+2.2 with data ≤ −0.6 (positive HSYNC always came with a data bit
      across the clock; router2 mostly misses timing), so this one sits at the edge of the danger zone.
      Re-screen it first if the HD Analog config's HDMI output shows lines.
    - HD Dual **router1 s7**: 83.17 MHz, HDMI −1.40 / ≤ −1.34.
    - SD configs: the builder's seed 1.
  - To reproduce: `skew/synth.sh <config>` then `skew/route.sh <config> <seed>` (router1; seed 1 re-routes
    the builder's bins byte-exact). The builder's own vmprog is `skew/cutout_v060_builder.vmprog`.
  - `out/rev_b/ron/heirloom/cutout.vmprog` (503,148 bytes). Presets are placeholders (Collage, Torn
    Strips, Chatterbox, Smoking Mirror, Shredded, One Picture, Inverse, Scattered).
  - **GHDL renders** (720p60, decimation 1, Video fed `cam169.png`) match the v0.6 integer model:
    Speed 1023 with Swap and Mirror (sequencer drift words, swap inks, the mirror slot), and Crops 0 (the
    zoomed in-register frame). The residue is the RTL's output smoothing at edges (`p6/rtl_vs_model.png`,
    `p6/rtlA2_cmp.png`). The sequencer needs ~4k clocks of vblank: fine in every hardware mode, but the
    tester's default 480 × 270 raster has too short a fake vblank (sim at 720p60, decimation 1).
- **v0.5.0 built 2026-10-28, ROUTER1** (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh
  cutout`), **6/6 on seed 1, not flashed.**
  - Post-route Fmax: HD Analog 76.24, HD HDMI 85.39, HD Dual 77.87, SD Analog 80.87, SD HDMI 78.67,
    SD Dual 70.95 MHz (SD target 27).
  - Fit: 7432–7451 / 7680 LC (97 %), 24/32 EBR. All six bitstreams pass iceunpack.
- **router2 does not close this netlist**: 73.3 MHz on HD HDMI seed 1. Rebuild with router1.
- **The fit is over the 92 % budget.** The first version was 157 %: a generic engine, double-buffered
  line registers, a bilinear store and long pipeline carries. If timing tightens, the remaining large
  pieces are the field sequencer's operand muxes, the cb/ft pipeline carries and the noise DDAs (16-bit
  phases could become 12-bit).
- **GHDL renders** (720p60, decimation 1, defaults, exact hardware hues via raw2hw), in both Osc and
  Video (fed a 16:9 crop of selfie_small.jpg), match the proto and the integer model.
  - 4 × 4 region means over the head: within 12 RGB levels per channel of the photo.
- **Motion check** (480p, captures 5 fields apart): the strips slide and sway, and the jaw chatters.
- Interlaced modes were not simulated.

## Deviations from the photo
- **The scraps' content.** The scraps print the camera, or a generated photo, through measured
  per-family ink ramps. There is no stored magazine imagery (sea, people, text, landscapes, fabric), so
  the textures are generic and softer than the photo's (local sd ~40 vs 53).
- **The pictures in the scraps.** The store is sampled nearest-neighbour at 2 virtual px per store
  pixel, giving a fine pixel grain. At Crops 0 the one-frame view is a coarse mosaic (128 × 64 store
  pixels across the head, ~13 virtual px each, 16 levels) through each scrap's own ink.
- **Map resolution.** With 12-virtual-px cells, corners round off by about one cell, scraps narrower
  than a cell (the navy stick, slivers) are lost or thinned, and 3-scrap junctions are approximate.
- **Cup and plate.**
  - The cream rim is concentric with the soup (the photo's is wider on the left). The handle is
    merged into the body.
  - The glass bowl's rim is approximated by the plate's inner line. Fork and spoon are plain bars.
  - The pastries' chocolate and nuts come only from the store texture. The red dart on the light-blue
    paper is omitted.
- **Teeth and label.** The teeth are 3-px ROM cells in three tones (rim, stone, highlight). The
  "Love." glyph is a hand-cleaned 64 × 20 bitmap, tilted by a y-shear.
- **Whites and yellow.** Luma is capped at ~777 by the shell's output scale (the HD HDMI bar fix), so
  the yellow is about (233, 232, 12) against the photo's (255, 254, 37). Chroma is softened by the
  9-tap filter.
- **Surface textures.** Streak and hair textures were dropped for fit; grain and specks remain. The
  pixel hash has a mild horizontal lag-2 correlation (−0.15); other lags are ≤ 0.05.
- **Motion.** The photo is still. At the default Speed each scrap's picture drifts slowly on its own
  line and "Love." crawls along its label; the tear and jaw are at rest.
- **SD 4:3.** The whole head fits, but at high Tear the outer strips swing past the 4:3 frame edge.
  Interlaced modes draw the same lines in both fields.
