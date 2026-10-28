# Cutout — a collage head of torn magazine scraps on a yellow ground (Heirloom #55)

**v0.5.0 (2026-10-28): rebuilt from the photo, built, NOT flashed.** Source: `photos/tezcatlipoca.jpg`.
v0.1.0 posterized the video inside an oval head shape, with no relation to the collage's pieces. This
version rebuilds the collage: the measured head cut-out, its 53 traced torn scraps, the soup-cup eye,
the plate of pastries, the stone teeth, the "Love." label and the black jaw. Every scrap prints the
camera picture. Osc mode substitutes a generated magazine photo.
Working files: `.heirloom_work/rb_cutout/` (`notes.md`, `pieces.py`, `features.py`, `mapbuild2.py`,
`fams.py`, `gendata.py`, `proto.py`, `imodel.py`, `sheet.png`, `cmp_final.png`, `cmp_motion.png`).

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
2. **Camera print.** Every scrap reads a 128 × 64 × 4-bit store at its own scale (2 or 4 virtual px
   per store pixel, ×K2 zoom), x/y flip, crop offset and drift direction.
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
5. **Fringe.** White paper core on the near side of a third of the scrap edges (hash-gapped, 35–50
   units wide).
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
| K1 Rag | the torn edges | 0 = clean scissor cuts (rounded); 50 % = the photo's tears; 100 % = shredded (32-step gain table) |
| K2 Zoom | the pictures inside the scraps | 3 octaves in 32 steps: small whole pictures ↔ big close-ups (blocky at max) |
| K3 Ink | the scraps' texture contrast | 0.25× (flat paper colours) → 1× (the photo) → 1.7× |
| K4 Jaw | the jaw | 0 = closed (the photo) → the jaw dropped by 0.10 H, yellow in the gap |
| K5 Paint | the yellow ground | 8 positions: yellow (the photo), orange, red, magenta, violet, blue, cyan, green |
| K6 Speed | (motion) the crops, the Osc photo, the strips | bipolar, still in the centre (±16). At the default (640) the crops pan ~7.5 virtual px/s, the Osc photo drifts ~15 px/s, and the strips sway with a 4.3 s period when P12 > 0 |
| S7 Facing | the profile | Left (the photo) / Right (mirrored) |
| S8 Fringe | the white torn paper core | On (the photo) / Off |
| S9 Chatter | the jaw | Off (the photo) / On: the jaw chatters at 2 Hz, at least 35 % open |
| S10 Eye | the soup cup | Soup (the photo) / Mirror (Tezcatlipoca's obsidian mirror: glossy black) |
| S11 Source | the pictures in the scraps | Video (default) / Osc |
| **P12 Tear** | the horizontal tears through the head | 0 = the intact collage (the photo); up: 8 strips slide apart alternately and sway; 100 % = flung up to ±560 virtual px across the frame |

- P12 glides (1/8 of the gap per field, ±4 deadband, the tail snaps).
- K1, K2, K3 and K5 use 32-step indices with hysteresis, so ADC noise does not flicker the tables.
- The control sheet is `.heirloom_work/rb_cutout/sheet.png` (from the proto); every control changes
  the picture.

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
- **Field sequencer** (vblank, from zone clock 64): one serial multiplier, 30 ops in 24-clock slots,
  with registered one-hot operand selects.
  - Ops: the chatter jaw, the zoom products (store x step and start, store y step and start, jaw
    store offset), then per strip the sway amplitude, the map offset and the store offset.
  - Results go to a strip RAM (one EBR). Its spare words hold the zoom, rag-gain and paint tables,
    read at zone clocks 40–47.
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
  - S14–S16: parameters, rings, features, texture and offset.
  - S15–S17: the store-x DDA, scale, flip/offset/drift, store address.
  - S18–S19: store. S20–S21: contrast and level. S22–S23: ink ROM. S24: compose.
  - S25–S32: conditioning and gate.
- **24 EBR:** store 8; box-sum accumulator 1; map banks 8; ink ROM 3; detail ROM 2; strip RAM 1;
  sync/avid delay 1. The collision-free RAMs carry `no_rw_check`.
- **Contracts:**
  - generated avid (parity-locked, nominal width);
  - virtual 1080-line geometry from the timing ID;
  - one 32-clock latency in every mode (`processing_delay_clks` = 32);
  - output conditioning (luma [1 2 1] and scaled toward ~777; chroma [1 2 2 2 2 2 2 2 1]/16) before
    the blanking gate;
  - colours in the hardware convention (u = Cr), from `cutout.py` → `cutout_pkg.vhd`.

## Status
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
  pixel, giving a fine pixel grain, blocky at maximum Zoom.
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
- **Motion.** The photo is still. The default Speed gives a slow pan of the crops; the tear and jaw
  are at rest.
- **SD 4:3.** The whole head fits, but at high Tear the outer strips swing past the 4:3 frame edge.
  Interlaced modes draw the same lines in both fields.
