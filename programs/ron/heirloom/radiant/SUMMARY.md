# Radiant — pink paper rays burst from a dark halo behind a bust with a flag lens and a cup of coffee on his head (Heirloom #52)

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/radiant.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.1 (2026-11-15): K6 camera pass on the diag (K6 = Camera, Pulse on K2, Lift fixed), built; see that section.** **v0.6.0 (2026-11-03, phase 2): diag preview built, awaiting Ron's preview** (`Radiant Diag`,
`out/rev_b/ron/_diag/radiant_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`). Production v0.5.1 is
untouched. Phase-2 working files: `.heirloom_work/rb_radiant/p2/` (mprog_d.py machine program, gen_d.py splice,
sweep.py, check.sh, psim.sh, renders diag2_osc.png / diag2_x1.png, the cut sort version in mprog_d_v1sort.py /
radiant_v1sort.vhd).

**v0.5.0 (2026-10-28): rebuilt from the photo, built, NOT flashed.** Source: `photos/duci.jpg`.
v0.1.0 drew 2–8 generated rays and a halo ring around the video, without the collage's figures. v0.5 rebuilds
the collage itself:
- the measured layout and cut-outs (hands, bust, woman, soldier) and the four paper rays;
- the halo disc, the cup on the head, the glasses with the flag lens, the DUCI label and the tissue ground.

The live camera is printed through the cut-outs. Osc mode substitutes generated shading.
Working files: `.heirloom_work/rb_radiant/` (`notes.md`, `proto.py`, `mprog.py`, `sheet.png`, `cmp_final.png`).

## K6 camera pass (2026-11-15), v0.6.1
- **K6 is now Camera** (0..100 %, default 50 %), with the same design as production v0.5.2.
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
- **S10 Ground = Camera** is scaled too. The ground's luma multiplier gm (5 bits) becomes
  16 + (gm − 16) · G, the same four steps, clamped to 3..31.
  - 0 %: the flat paint / tissue tone.
  - 50 %: v0.6.0.
  - 100 %: ×2 contrast.
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
- **Frozen: K2 Lift at 0** (the cup sits on the head; it still rides with the bust under K4 Rise).
  - Lift was the weakest knob here: it moves one small element, while K1 Halo, K3 Fan, K4 Rise and K5 Flank re-lay
    the whole collage.
- **K6's old function, Pulse, moved to K2** with its default (25 %). Ascension and Gold Icon lost their Lift values;
  Blaze's Pulse 700 moved to K2.
- **Sims:** `.heirloom_work/rb_radiant/k6diag/k6sheet.png` (S10 Ground Camera at K6 0 / 512 / 1023). Rendered with
  the 8-step build; its G at those points (0, 1, 2) is the same.
- **Build (v0.6.1, 2026-11-15):** router1, all six configs on seed 3 (`SEED=3 .kinetic_work/tools/k6build.sh diag
  radiant`). Seeds 1 and 2 still fail placement on some configs; a placement-only probe placed all six netlists at
  seeds 3 and 4.
  - Post-route Fmax: HD Analog 92.32, HD HDMI 93.25, HD Dual 91.93 (worst HD), SD Analog 86.61, SD HDMI 79.19,
    SD Dual 83.57 MHz.
  - Fit: 7011–7024 / 7680 LC (91 %), 17 / 32 EBR.
  - Output: `out/rev_b/ron/_diag/radiant_diag_v0.6.1.vmprog`. Not flashed.

## Phase 2 (v0.6.0)
Ron left no notes for radiant (free hand, 2026-11-03 batch). The map follows the phase-2 brief for a collage:
per-cut-out position instead of fixed figures, the rays as one module with their own spread, bolder switches, about
twice the range each side of the photo. The defaults still give the photo: the default frame is raw-identical to
v0.5.1.

| Control | v0.5 | v0.6.0 | Full sweep at 1080p |
|---|---|---|---|
| K1 Halo | radius 50–150 % | radius 25 % .. 100 % (at 50 %) .. 194 %, two slopes | disc 128 px → ~1000 px across |
| K2 Lift | cup rises 0–0.19 H | cup rises 0–0.43 H, and rides on the bust's head when K4 moves the bust | 0 → 459 px |
| K3 **Fan** (was Stripe) | the rays' stripe mix | the four rays' slopes ×0.3 .. ×1 (50 %) .. ×3, pivoting on the halo-centre line; paper width kept constant | outer tips at the top edge from about ±600 px in to about ±200 px; at 100 % they leave through the sides ~280 px down |
| K4 **Rise** (was Tissue) | tissue share | bipolar, photo at 50 %: each cut-out slides up (right) / down (left) by its own travel: soldier ±0.38 H, hands ±0.33 H, bust ±0.30 H, woman ±0.22 H; glasses and cup ride with the bust, the DUCI label with the hands | right: cut edges float above the frame bottom and the hands leave the top; left: the hands come down over the halo, only the bust's head and the soldier's beret show |
| K5 Flank | 0 = photo .. 100 % = slid out | bipolar, photo at 50 % (new default): left slides the woman and the soldier inward until they pass each other in front of the bust; right = v0.5's slide out | ±0.49 H (527 px) each; 0.27 H at 4:3 |
| K6 Pulse | rays breathe in width | unchanged | |
| S7 **Rays** (was Lens) | Flag / Clear lens | Under / Over: the paper strips lie over the cut-outs (hands, head, label), under the cup and glasses | |
| S8 Cup | On / Off | unchanged | |
| S9 **Palette** (was Halo) | Dark / Gold disc | Photo / Gold: gold disc plus pink strips with a wide dark line (v0.5's Stripe at 100 %) | |
| S10 **Ground** (was Label) | DUCI / Coal | Paper / Camera: the camera luma printed into the maroon paint and tissue (Osc: the blob wave) | |
| S11 Source | Video / Osc | unchanged | |
| P12 Radiant | ray width 0..1..2.2× | ray width 0..1..3× | ~3× the photo's 57 px strip |

Dropped, fixed at the photo: the continuous stripe mix (its pink end is S9 Gold), the tissue share (v0.5's
default), the Clear lens, the label-off option. A Burst switch (rays continuing below the halo) was built first.
It needs a per-line sort of the four ray crossings (~270 LC), which took HD Analog to 7309 LC (95 %), and placement
failed. It was cut. Without the sort, Fan's low end stops at ×0.3, so the one ray crossing above the centre
(TL / TR, 17/F px up) stays inside the smallest disc.

How it works:
- **Fan:** the machine scales each ray's slope (DXF = DXL0 F) and re-pivots its crossing on the halo-centre line
  (+ (DXL0 − DXF) 597.1 / 16, added in whole half-pixels so F = 1 is exact). Width: HW = HW0 + HP (AMB(kF) − AMB(k)),
  with AMB an alpha-max-beta-min |(1, k)|, exact HW0 at F = 1. The midpoints now sum at 15 bits (a ×3 fan wrapped
  the 14-bit sum at the top-right).
- **Rise:** in the figure loop the machine computes RR = (K4 − 512) RISE / 512 per figure and pushes RR × KV14 into
  the line engine's row chain (new output O_RV); the field-start run adds them to the collage's rows. The lens-row
  and label-row DDAs get RR × 102.75 / RR × 71.75 at the field start (r_lno / r_lro), and the cup lift adds the
  bust's RR.
- **Rays Over:** a ray overlay sets a bit that blocks the figure / label in the compose.
- **Ground Camera:** the camera's luma (5 bits, floor 3) is the ground's x/16 luma multiplier.
- **Fit:** the line engine reads each figure's x from the register file at slot stage C (r_xf registers and the
  machine's XF state gone); the output filter's 4-deep luma copy chain is a block-RAM ring (+1 EBR).

Presets: Photo (defaults), Bundle (Fan 0, Over, wide rays), Wide Fan, Ascension (Rise up, cup lifted, big halo),
Low Tide (Rise down, flanks crossed), Gold Icon, Camera Ground, Blaze.

**Build (2026-11-03):** router2, base SEED=3, 6/6. Post-route Fmax: HD Analog s3 81.23, HD HDMI s5 80.48 (s3, s4
missed), HD Dual s3 77.39, SD Analog s3 83.71, SD HDMI s3 74.45, SD Dual s3 79.63 MHz. 6950–6965 / 7680 LC
(90.7 %), 17/32 EBR; all six .bin pass iceunpack. Rebuild with `SEED=3` (HD HDMI placement fails at seeds 1–2).

**Checks:**
- GHDL analyse / elaborate / synth exit 0 (`p2/check.sh`).
- RTL-width machine sweep (`p2/sweep.py`): 90,720 runs over all 7 timing modes × knob grids × P12 × S9 × pulse
  phase. No RF word over 16 bits, no 28-bit overflow, every output fits its register, and the defaults equal v0.5.1
  (outputs, RF tables, XF; Rise offsets 0).
- Default frame (GHDL 720p60 dec 1, Osc) raw-identical to v0.5.1 (0 px).
- One extreme render (Video selfie: Fan 100 %, Over, Rise 15 %, Flank 10 %, Halo 0, Ground Camera, P12 88 %): clean
  ray convergence, the hands, label, bust, cup and glasses move together, the flanks crossed, the camera in the
  ground.
- Not verified: per-field motion, interlace, SD renders.

The sections below are the production (v0.5.1) SUMMARY, unchanged.

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
| K3 Stripe | the rays' stripes | 0 = white strips with a thin pink core and a hairline; 50 % = the photo's mix (pink 0.44, line 0.10 of the half width); 100 % = pink strips with a wide dark line |
| K4 Tissue | the torn mauve tissue on the maroon paint | 0 = bare two-tone paint; 100 % = dense tissue with light overlaps and stains |
| K5 Flank | the woman and the soldier at the page edges | 0 = the photo's positions; 100 % = slid out to the frame edges (about 0.49 H each way at 16:9, 0.27 H at 4:3) |
| K6 Pulse | (motion) the rays | the rays breathe in width. Default 25 %: ±12.5 %, 9.3 s period. 100 %: ±50 %, 2.7 s. 0 = still |
| S7 Lens | the Dominica-flag lens | Flag (the photo) / Clear (the left lens shows an eye like the right) |
| S8 Cup | the cup | On / Off (the bare head with its grey hair) |
| S9 Halo | the disc's paper | Dark (the photo's red-brown) / Gold |
| S10 Label | the DUCI label | DUCI / Coal (bare coal) |
| S11 Source | what fills the cut-outs | Video (default) / Osc |
| **P12 Radiant** | the paper rays | ray width: 0 = no rays (the bare halo); 50 % = the photo's four strips; 100 % = 2.2× wide (blazing) |

- P12 glides (1/8 of the gap per field).
- Presets: Duci (the photo), Bare Halo, Blaze, Gold Icon, Wide Stage.
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

## Open questions for Ron
1. Stripe and Tissue are now fixed. One back on a knob (trade K6 Pulse or K1 Halo)?
2. Rise moves every cut-out the same way, each at its own rate. Prefer a seesaw (side figures against the bust and
   hands) or per-figure drift LFOs?
3. Burst (rays continuing below the halo) needs ~270 LC for a ray sort. Worth trading the camera ground / Rise
   glasses-and-label tracking for it?
4. S10 Ground Camera vs the old DUCI / Coal and Flag / Clear options, or a "Dense tissue" ground?

## Hardware watch list
- Renamed controls: K3 Stripe → Fan, K4 Tissue → Rise, S7 Lens → Rays, S9 Halo → Palette, S10 Label → Ground.
  K5 Flank's default moved from 0 % to 50 %. K1's display is now 0–100 % (was 50–150 %).
- The ray order is fixed (no sort). With K1 near 0, K3 near 0 and P12 high, a small sliver near the halo centre could
  draw a ray wrong (mostly behind the bust and cup).
- If figures sit at the wrong x, suspect the figure-x read from the register file in hblank.
- Ground Camera with no / black input gives a dark ground (floored at 3/16 of the paint tone).
- Cut for fit: S7 Burst (ray sort).

## Next step
On approval: `./heirloom_diag.sh merge radiant`, then repack the previewed diag bitstreams under the production TOML
instead of re-routing:
- copy `build/programs/ron/radiant_diag/rev_b/bitstreams/*.bin` plus the production `program_config.bin` into a
  scratch dir;
- `vmprog_pack.py --no-sign --hardware rev_b --toml-path programs/ron/heirloom/radiant/radiant.toml <dir>
  out/rev_b/ron/heirloom/radiant.vmprog`.

Then update the README / PHASE2 rows and commit only radiant's files and rows.
