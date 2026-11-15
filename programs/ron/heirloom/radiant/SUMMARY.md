# Radiant — pink paper rays burst from a dark halo behind a bust with a flag lens and a cup of coffee on his head (Heirloom #52)

**v0.5.2 (2026-11-15): K6 camera pass (K6 = Camera, Pulse on K3, Stripe fixed), built, NOT flashed; see that section.** **v0.5.0 (2026-10-28): rebuilt from the photo, built, NOT flashed.** Source: `photos/duci.jpg`.
v0.1.0 drew 2–8 generated rays and a halo ring around the video, without the collage's figures. This version
rebuilds the collage itself:
- the measured layout and cut-outs (hands, bust, woman, soldier) and the four paper rays;
- the halo disc, the cup on the head, the glasses with the flag lens, the DUCI label and the tissue ground.

The live camera is printed through the cut-outs. Osc mode substitutes generated shading.
Working files: `.heirloom_work/rb_radiant/` (`notes.md`, `proto.py`, `mprog.py`, `sheet.png`, `cmp_final.png`).

## K6 camera pass (2026-11-15), v0.5.2
- **K6 is now Camera** (0..100 %, default 50 %).
- **Class: the camera IS the picture.** The cut-outs print the camera, so K6 fades that print against the program's own
  flat paper, rather than adding a gain on top of a drawing.
- **Law.** shade = 128 + (src − 128) · G, built as G · src + 128 (1 − G).
  - src is the camera luma, (Y >> 2 − 16) · 17/16, or the Osc blobs.
  - The result then goes through the usual grain and dither into the zone's 16-step ramp.
  - G takes four steps, floor((k + 128) / 256): 0, ½, 1 and 2 (2 from k = 640).
  - It is a 4-way shift-select because LC is tight here. The first build, a 9 × 9 multiply with a continuous
    G = k / 512, came to 7190 LC (93.6 %) and failed placement. An 8-step shift-select, at 7030 LC, placed on
    only a few seeds per config.
  - 0 %: G 0. Flat paper: every figure zone sits on its middle step (the collage's median colour for that zone),
    with grain.
  - 25 %: G ½, half contrast.
  - 50 % (the default): G 1. Exactly the v0.5 print; the camera is tapped one pixel earlier.
  - 100 %: G 2, ×2 contrast about the middle step. Zones reach their darkest and lightest quantiles, but stay
    in their own colours, so nothing washes out. A dark camera (the selfie) reads darker and more mottled.
- **Sims** (NTSC, selfie, `.heirloom_work/rb_radiant/k6/k6sheet.png`, rendered with the shipped build):
  - 0: flat cut-outs.
  - 25 %: a soft print.
  - 50 %: the v0.5 print.
  - 100 %: a deeper, higher-contrast print.
  - Osc keeps its blotches, scaled the same way.
- **How.**
  - A 2-bit step register is updated one clock after the vblank-zone start, outside the register machine. Each
    field it moves at most one step toward the knob, and only when the knob is 4 codes past the step boundary.
    That gives a glide and a ±4 deadband without a glide register.
  - Stage S4 forms src, with the camera tapped there, one pixel earlier than v0.5. The source's own 0..255 clamp is
    gone; S6 still clamps the final sum.
  - Stage S5 is the shift-select G · src (0, src / 2, src, 2 src).
  - Stage S6 is unchanged in shape: one three-operand add of G · src + grain + (128 (1 − G) + dither). The per-field
    constant 128 (1 − G) (128, 64, 0, −128) fills the dither nibble's free upper bits, so no adder is needed for it.
  - The latency is unchanged (16).
- **Frozen: K3 Stripe at 512** (the photo's white / pink / line mix).
  - It was the weakest knob: in `sheet.png` its whole travel only re-colours the inside of the four thin rays.
- **K6's old function, Pulse, moved to K3** with its range and default (25 %). It is the program's only motion.
  Bare Halo's and Blaze's Pulse values moved with it; Blaze lost its Stripe 800.
- **Build (v0.5.2, 2026-11-15):** router1 (`SEED=3 .kinetic_work/tools/k6build.sh heirloom radiant`).
  - Five configs on seed 3. SD Dual is on seed 4: seed 3 stalled router1.
  - Post-route Fmax: HD Analog 91.56, HD HDMI 89.52, HD Dual 88.95 (worst HD), SD Analog 95.81, SD HDMI 96.65,
    SD Dual 89.48 MHz.
  - Fit: 6951–6966 / 7680 LC (91 %), 16 / 32 EBR.
  - Seeds 1 and 2 fail placement on some configs. A placement-only probe of all six netlists placed them all at
    seed 3.
  - Output: `out/rev_b/ron/heirloom/radiant.vmprog`. Not flashed.

## Reading of the photo (measured)
The photo is a portrait paper collage, 3196 × 4176 (0.765:1). Measurements below are in page heights H
(1600-line working copy). Horizontal positions are measured from the page's left edge.
- **Ground.**
  - Dark maroon paint, rgb (58, 38, 36) / (69, 38, 37), under torn mauve tissue (111, 72, 79).
  - The tissue is light (165, 123, 120) where layers overlap. There are dark stains.
  - Shares: paint 34 %, tissue 39 %, light tissue 22 %, stains 5.5 %.
  - The shards have straight cut edges with ragged tears, plus fine mottling.
- **Rays.** Four paper strips (not six), each a constant 0.0525 H wide.
  - Across the strip: white 22 | pink 13 | dark line 9 | pink 14 | white 24. The pink band spans 0.44 of the
    half width and the line 0.11.
  - Colours: white (245, 236, 240), pink (246, 112, 162), centre line dark with pink scratches.
  - Outward angles 137.3°, 112.2°, 70.3° and 48.9°: two pairs, symmetric about about 92°.
  - Perpendicular offsets from the halo centre: 0.013 / 0.023 / 0.013 / 0.056 H.
  - They run under the halo disc and off the page.
- **Halo disc.**
  - Centre 0.397 H across, 0.553 H down; radius 0.238 H. It is round (ellipse fit rx 380, ry 376).
  - Fill dark red-brown (61, 28, 22), with a thin black hand-drawn outline of about 0.004 H.
- **Cup on the head.**
  - Rim ellipse: centre 0.401 / 0.4375 H, rx 0.156 H, ry 0.084 H.
  - A straight wall 0.0375 H tall down to a base ellipse (centre 0.475 H, ry 0.0625 H).
  - Coffee ellipse: rx 0.140 H, ry 0.063 H.
  - Colours: rim white (226, 226, 232); coffee tan (190, 160, 100), darkening to (80, 55, 35) in the right-hand
    shadow.
- **Bust** (0.18–0.65 H across, top at 0.43 H).
  - Grey hair, a skin-coloured face, a blue suit, a pale yellow shirt with a V-shaped collar down to the knot,
    and an olive tie.
  - Glasses: a thick black frame. The left lens (0.369–0.476 × 0.570–0.644 H) is the Dominica flag:
    - a green field and a yellow / black / white cross;
    - a red disc of radius 0.023 H with green stars and a parrot.
  - The right lens is clear and shows the eye. There is also a bridge, an arm and an end piece.
- **Woman** (left, from the page edge to 0.19 H, top at 0.366 H; cut by the page edge).
  - A cream-peach headscarf, a face in three-quarter profile and a black collar.
  - A cream satin gown with folds.
  - She stands in front of the bust.
- **Soldier** (right, 0.63 H to the page edge, top at 0.54 H; cut by the page edge).
  - An olive beret, a dark face and a yellow T-shirt.
  - Camouflage in five tones (dark 36, 31, 28 up to light 142, 130, 105).
  - He stands in front of the bust.
- **Hands** (top, 0.20–0.63 H across, down to 0.31 H; cut by the page top).
  - Khaki sleeves, two cupped orange hands, and a heap of black coal with teal-grey facets.
  - The DUCI label: a navy (45, 83, 102) block, 0.146 × 0.079 H, with cream (215, 201, 171) block letters.
- **Layers, back to front:** ground, rays, halo disc, hands, bust, glasses / flag, cup, woman, soldier.

## 16:9 mapping
- The frame height equals the page height. The page (0.766 H wide) sits centred, and the maroon ground
  continues to the frame edges.
- K5 Flank can slide the woman and the soldier out into that space.
- Geometry is in virtual 1080-line units from the timing ID, so 720p renders the same picture and the disc and
  cup stay round at SD.

## What it does
1. **Cut-outs.**
   - Each figure is a 64-row slot table: six edges and five zone labels per row, interpolated between rows so
     the outlines have no stair steps.
   - Three slots per line: the woman, the centre (the hands, or the bust), and the soldier. The side figures are
     drawn in front.
   - Each pixel's zone picks one of 16 measured colour ramps. Each ramp is the zone's 16 luma quantiles on the
     collage:
     - hands: sleeve, coal, skin;
     - bust: tie, shirt, suit, hair, skin;
     - woman: face / hand, collar, scarf, gown;
     - soldier: beret, skin / hand, shirt, camo.
   - The step is shade + grain + dither. Video takes the shade from the camera's luma; Osc generates it.
   - The coal gets hashed glitter, re-seeded every 8 fields.
2. **Rays.**
   - The four measured strips are drawn as per-line crossings.
   - Each pixel tests only the nearest ray, chosen by the midpoints between crossings. |d| is compared against
     that ray's white, pink and line half widths.
   - The rays are drawn only above the halo centre, under the disc and the figures.
3. **Halo disc and cup.**
   - A squared-distance DDA (dx² by second differences) is compared against per-line thresholds for the disc and
     its rim, and for the cup's rim, base and coffee ellipses.
   - The cup wall is shaded across its width. The coffee runs through a 12-step ramp that darkens to the right.
4. **Glasses and flag.**
   - A lens ROM gives the outer and inner half widths per row for each lens. The bridge, end piece and a stepped
     arm come from per-line row flags.
   - The flag shows the green field, the yellow / black / white stripes and the red disc (from a half-chord
     table).
   - The right lens shows the bust behind it, with an eye.
5. **DUCI label.** A 60 × 24 bitmap: cream on a navy box over the coal.
6. **Ground.**
   - Two integer-angle lattices of hashed shards, with edges jittered by noise so they look torn.
   - Each lattice cell is paint or tissue; the K4 threshold sets the share. Where two tissue layers overlap they
     are light, and some cells are dark stains.
   - The paint varies between two tones.
7. **Output.** Shell output conditioning, then the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Halo | the dark red-brown halo disc | radius 50–150 % of the photo's 0.238 H (100 % at 50 %); the thin black rim follows |
| K2 Lift | the cup of coffee on his head | 0 = sitting on the head (the photo); up to 0.19 H higher, floating up through the halo |
| K3 Pulse | (motion) the rays | (K6 before v0.5.2) the rays breathe in width. Default 25 %: ±12.5 %, 9.3 s period. 100 %: ±50 %, 2.7 s. 0 = still |
| K4 Tissue | the torn mauve tissue on the maroon paint | 0 = bare two-tone paint; 100 % = dense tissue with light overlaps and stains |
| K5 Flank | the woman and the soldier at the page edges | 0 = the photo's positions; 100 % = slid out to the frame edges (about 0.49 H each way at 16:9, 0.27 H at 4:3) |
| K6 Camera | the camera printed through the cut-outs | how strongly the camera (or Osc) shades the figures, in four steps: 0 = flat paper (each zone's middle step), 25 % = half contrast, 50 % = the v0.5 print (default), 100 % = ×2 contrast. See "K6 camera pass" |
| S7 Lens | the Dominica-flag lens | Flag (the photo) / Clear (the left lens shows an eye like the right) |
| S8 Cup | the cup | On / Off (the bare head with its grey hair) |
| S9 Halo | the disc's paper | Dark (the photo's red-brown) / Gold |
| S10 Label | the DUCI label | DUCI / Coal (bare coal) |
| S11 Source | what fills the cut-outs | Video (default) / Osc |
| **P12 Radiant** | the paper rays | ray width: 0 = no rays (the bare halo); 50 % = the photo's four strips; 100 % = 2.2× wide (blazing) |

- P12 glides (1/8 of the gap per field).
- Presets: Duci (the photo), Bare Halo, Blaze, Gold Icon, Wide Stage.
- Since v0.5.2 the rays' stripe mix (K3 Stripe) is fixed at 50 % (the photo's mix).
- The control sheet is `.heirloom_work/rb_radiant/sheet.png` (from the proto). Every control changes the
  picture.

## Video
- The camera's luma, at each pixel's own position, picks the step of that figure-zone's measured colour ramp.
  - The camera picture therefore shows through every cut-out in the figures' own colours: blue suit, cream gown,
    camo, orange hands.
  - The camera's colour is not used.
  - The picture is not framed per figure: one live picture lies under all the cut-outs.
- Rays, disc, cup, glasses, flag, label and ground are drawn the same in both modes.
- Osc substitutes a generated shade: two crossed triangle waves, giving soft blotches of about 60 lines that drift
  diagonally (one wave period in about 2 s).
- Every control acts in both modes.

## Engine
- **Per-field register machine** (vblank = "no avid for 1024 clocks").
  - Triplicate's machine, extended:
    - 65 microcode states in block RAM;
    - a register file in one EBR;
    - one serial shift-add multiplier (signed A × unsigned B, arithmetic result shift) with negation;
    - add and max-0 post-ops;
    - two 4-iteration loops (figures, rays).
  - It computes:
    - the page origin, halo centre and label start;
    - the figure positions and row steps, with the K5 offset;
    - the disc and rim r² (K1), and the cup lift offsets and steps (K2);
    - the squared-distance DDA start and step;
    - the lens and label row steps and the label column step;
    - the tissue threshold (K4) and the P12 glide;
    - the ray width scale: P12 × the K6 triangle LFO;
    - the stripe fractions (K3);
    - per ray: the half widths, the line steps and the absolute crossings.
- **Per-line engine in hblank** (76 clocks, from 8 clocks after the generated avid falls).
  - It steps the ray crossings and ellipse offsets (rotating chains fed from the register file), the four figure
    rows, and the lens and label rows.
  - It reads 3 slots × 7 slot-table words and interpolates 18 edges (a three-clock lerp).
  - It squares four line offsets on a serial squarer (disc, cup rim, base, coffee) to give the per-line
    thresholds, reads the lens ROM, and sets the row flags.
- **Pixel path:**
  1. beam and dx² DDA;
  2. 18 edge compares, nearest-ray pick and page offsets;
  3. per-slot zone labels, |d| for the ray, disc and ellipse tests, glasses and flag geometry, ground lattices,
     Osc waves;
  4. classes, front-most figure and its zone, ground hashes;
  5. overlay code, shade and grain;
  6. palette and overlay ROM addresses;
  7. ROMs and compose;
  8. output conditioning and the gate.
- **16 EBR:**

  | Use | EBR |
  |---|---|
  | slot tables | 4 |
  | microcode | 4 |
  | zone palette | 2 |
  | overlay colours | 2 |
  | lens ROM | 1 |
  | register file | 1 |
  | chroma ring | 1 |
  | sync delay | 1 |

  All written RAMs carry `no_rw_check`.
- **Contracts:**
  - generated avid;
  - virtual 1080-line coordinates from the timing ID;
  - one 16-clock latency in every mode (`processing_delay_clks` = 16);
  - output conditioning before the blanking gate: luma [1 2 1] and scaled toward ~777, chroma
    [1 2 2 2 2 2 2 2 1]/16;
  - colours in the hardware convention (u = Cr).

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). A small g_avid shift register was added for the gate. Post-route Fmax: HD Analog s4 80.10, SD Analog s3 72.36, HD HDMI s4 81.77, SD HDMI s3 80.23, HD Dual s4 75.69, SD Dual s3 77.43 MHz; 6911-6936 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-28, router2, 6/6 on seed 3 (base `SEED=3`), not flashed.**
  - Post-route Fmax: HD Analog 82.01, HD HDMI 82.04, HD Dual 76.42, SD Analog 82.11, SD HDMI 80.44,
    SD Dual 74.55 MHz.
  - Fit: 6899–6922 / 7680 LC (90 %), 16/32 EBR. All six bitstreams pass iceunpack.
- **Rebuild with `SEED=3 ./build_heirloom.sh radiant`.**
  - At seeds 1 and 2, HD HDMI fails placement ("Unable to find legal placement"; both routers, since the placer
    is the same). The builder does not retry placement errors.
  - Seeds 3–8 placed and passed on HD HDMI (77.6–86.6 MHz).
  - router1 also closes (HD Dual 89.9 / 90.7 MHz on seeds 1 / 2 in probes).
- **Timing history.** The first full build of this design ran at 40–47 MHz on HD. The deep cones were split
  across pipeline stages and the line engine's clocks; details in `notes.md`:
  - the zone/winner choice and the ray test;
  - the edge lerp, the squarer input and the grain hash;
  - the overlay dither, the flag-row maths and the machine's load and sign steps.
- **GHDL renders** (720p60, decimation 1, defaults, exact hardware hues): Osc, and Video fed the selfie.
  - Halo top at 0.315 H (photo 0.315 H).
  - Disc radius 0.235 H, measured at the outline (photo 0.238 H).
  - The layout matches the float proto.
- Motion across frames, the SD modes and interlace were not simulated.

## Deviations from the photo
- **The camera, not the photographs.**
  - The cut-outs print the live camera's luma through each figure's measured colour ramps. The figures keep
    their colours and silhouettes, but faces, folds and camouflage come from whatever the camera sees.
  - No likeness is stored.
  - There is no frame store, so the picture is not framed or scaled per figure: one picture lies under all of
    them.
- **Osc.** Soft drifting blotches through the same ramps. The faces are featureless.
- **Rays.**
  - The photo's four strips, straight and with a constant width.
  - No sway; the motion is K6's width pulse.
  - The pink scratches on the centre line are not drawn.
- **Glasses and flag.**
  - The arm is a 3-step line.
  - The flag's ten stars and the parrot are dropped: a green field, the yellow / black / white cross and the red
    disc remain.
- **Ground.** Generated shards with jittered edges, not the photo's own tears. The default tissue share is lower
  than the photo's.
- **Cup.** The rim's shading and the coffee's dark specks are simplified to a ramp plus a 1-bit dither.
- **Whites and chroma.**
  - Whites are capped at about 777 by the shell's output scale (the HD HDMI bar fix), so the paper whites are
    slightly dim.
  - Chroma is softened by the 9-tap filter (invisible at HD).
- **SD 4:3.** The same picture with less ground at the sides (0.28 H instead of 0.51 H). K5 Flank's travel is
  shorter to match. 720p is identical to 1080p.
- **Interlace.** Interlaced modes draw both fields the same (line-doubled), because the hardware delivers no
  bottom field.
