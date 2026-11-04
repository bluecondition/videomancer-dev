# Aureole — a cyan bloom in nested bands, a warm horned cap scrolling through it

**v0.1.1 built 2026-11-04, NOT flashed.** Heirloom from the video clips: clip `WIN_20230314_23_42_54` at 7.5 s
(`photos/vid/WIN_20230314_23_42_54_t7p5.png`). A soft cyan bloom sits in nested vertical bands: blue, dark, a
thin teal line, olive, brown, yellow-green, then the cyan plateau. A dark-red floor with a horned wave runs along
the bottom. A warm cap (white core, cream, orange, red, pink lobes) scrolls down through the picture every ~14 s,
with sharp horns on its lower edge. Presets are hand-written placeholders.

v0.1.1 (review fix): S9 Gold is a full-frame gold halo on every line. In v0.1.0 it recoloured only the afterglow
below the warm cap, which is off screen at the default phase. S9 Blue (the default) is unchanged.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_aureole/`: `notes.md`, `proto_final.py`, `imodel.py`, `beamcheck.py`,
`cmp_final.png`, `sheet.png`, and `clip_sheet.png` (the whole clip at 0.5 s steps).
- **Field.** Every band edge is a contour of ONE parabola in x: `F = 511 − k (x − xc)²`.
  - xc ≈ 1000 px; k = 4.9e-4 F/px² on the left and 5.6e-4 on the right (the right side's bands are ~15 % tighter).
  - A triangle doesn't fit: between y = 260 and 900 the contours move in by 53 px (blue), 67 px (teal), 113 px
    (core edge) and 151 px (plateau edge), so the slope grows with |x − xc|.
  - The plateau is 645 px wide at y = 260 (x 689..1334), 594 px at y = 20 and 316 px at y = 900, and closes at
    y ≈ 965. Its centre wobbles ±12 px down the frame.
- **Palette** (left half, row 540):
  - cyan plateau, x 720..1300 (G and B saturated);
  - yellow-green fringe, x 640..700, (82, 133, 16);
  - brown (79, 58, 0), x 560..600;
  - olive-green ramp, x 360..520;
  - a thin teal line (34, 142, 90), ~30 px wide, at x = 320;
  - dark (30, 20, 4), x 160..240;
  - blue (9, 66, 103), x 40..120;
  - a dark capture edge.

  The 512-entry cool palette is read straight off rows 560..640 as colour against F, with the plateau held from
  entry 478.
- **Vertical shape.** The bands are widest at y ≈ 260 and squeeze toward the bottom. Two terms make this:
  - a bulge of ~25 F, peaking ~300 px below the warm→cool floor;
  - a quadratic drop of ~110 F over 400 px above the cool→warm floor.

  The drop walks every band down the palette:
  - olive becomes the bright teal entry (so the photo's thin bright floor line is the palette's teal entry being
    swept);
  - the core turns green, then yellow (161, 171, 60);
  - blue goes dark.

  Within ~100 px of the floor the cool layer gates off (B first, then G, then R), and the warm layer is a dim red
  (R 90..110, G and B < 10).
- **Warm cap** (from the same clip at 11.0 s). The same bands in a warm palette: white core (255, 255, 248),
  cream, orange (232, 115, 47), red, salmon, a whitish line where the teal line is, and pink (183, 90, 136) at the
  blue band. It rises from the floor over ~500 px, R first (R ≈ 0.55 of full at the floor), then G and B.
- **Afterglow.** Below the warm→cool floor the cool bands are lifted toward blue-white. The blue band is
  (10, 93, 154) at y = 220 against (11, 58, 100) mid-frame, and the core's R is 137 at y = 30 against 104. The lift
  fades by ~700 px.
- **Horns.** The floor contour is a sawtooth in x:
  - period 350..372 px (peaks at x ≈ 420, 780, 1140, 1540 in the 11 s frames);
  - a slow curved side (the contour drops 25, 79, 151, 233, 326 px at 40-px steps from a peak, between linear
    and parabolic) and a sharp rise (< 60 px);
  - ±60..100 px high in the still. Ron turned it up to ±140 px by 11 s, and had it reversed (slow rise, sharp fall)
    at 0.5 s.
- No noise, tearing or comb. Horizontal edges take ~9 px from 10 to 90 %.

## Motion in the clip
- **Scroll.** The whole warm/floor structure scrolls DOWN rigidly at **165 px/s** at 7.5 s. At x = 50 the floor
  is at y = 540 at 4.0 s and y = 1020 at 7.0 s. The rate was 200 px/s at 0..1.5 s and 145 px/s at 9..13.5 s, so
  Ron was turning a rate knob.
- **Period.** The vertical period is ~2200 px: the 0.0 s and 13.5 s frames match. At 7.5 s the cool band is
  ~1300 px and the warm band ~900 px.
- **Horns.** The horn wave is locked in x: its peaks stay put while the floor scrolls 330 px through the 11 s
  frames. It is a line-rate oscillator with no tilt; its height and skew are knobs.
- **What the program does with it.** A 2200-px vertical cycle scrolls down 2.68 virtual px per field (161 px/s at
  60 Hz) at the default Speed.
  - The warm→cool floor sits 200 px above the frame at the default phase, so the horns enter at the top about
    1.2 s later, as in the clip's +1.0 / +1.5 s frames.
  - The floor at the bottom leaves the frame in the first ~0.5 s.
  - GHDL check: the floor and teeth move down 7 rows at 720p between the warm-up-2 and warm-up-6 frames (10.5
    virtual px; the model predicts 10.7).

## What it does
1. **Field.** `F = 511 − k d²`, plus the line's bulge term, minus the displaced drop, plus the video bend.
   - An exact second-order DDA in h (Q20 F per h²). The arch centre (2004 h) lands on the beam exactly in every
     mode, so the centre crossing reloads A2 = 0.
   - Twin: two arches with one k, ×4. At the mid point (1920 h) the normal step lands on k·960², and D is mirrored.
   - Well inverts the parabola (`F = k d²`).
2. **Vertical cycle.** Per line, `pos = (y − floorA) mod 2200`. Per pixel, `pos' = pos + horns(x) + etch`
   (mod 2200), which addresses a 1024-entry table (4 px per entry) refilled every field from Bloom's band lengths:
   - Z = −dist/2 in the cool band, +dist/4 in the warm band, where dist is the distance to the nearest floor;
   - the field drop `110 (1 − d/400)²`, with d = min(1.5 dA, dB) in the cool band.
3. **Horns.** A per-pixel phase accumulator (Q16; Teeth sets the step), restarted at each line start.
   - The slow side `A (1 − u − u²)` is a second-order DDA; the rise `A (1 + (u − 1) S/256)` is a ramp. Both reset
     at the wrap.
   - The output is the larger of the two, clipped to ±A, and negated for a negative Skew.
4. **Colour.** `RGB = PC(F) · gc(Z) + PW(F) · gw(Z) + glow(line)`, with per-channel gates:
   - gcR / gcG / gcB reach full at 80 / 110 / 150 px into the cool band;
   - gwR runs from −200 px (0) through the floor (0.44) to +200 px (1); gwG over 0..200 px; gwB over 10..190 px of
     the warm band.

   The glow term:
   - Blue: (28, 48, 64)·g/256 below the warm cap, where g is the glow shape.
   - Gold: (112, 72, 8)·(160 + g/4 + g/16)/256 on every line.

   Then RGB → YUV:
   - Y = 64 + Y8 · 1086/256, capped at 940 (pre-scaled 16/13 for the output's 13/16), where
     Y8 = (77R + 150G + 29B)/256;
   - U (Cr) = 512 + 641 (R − Y8)/256;
   - V (Cb) = 512 + 508 (B − Y8)/256 (hardware convention).
5. **Output.** Chroma goes through one [1 2 2 2 2 2 2 2 1]/16 box: two adjacent 8-sums, with the 8-back tap from a
   256 × 16 block-RAM ring, and a resync to neutral after 8 blank samples. Luma goes through [1 2 1] and 13/16, then
   the blanking gate. This is the HD HDMI bar fix, and it also gives the clip's softness.

## Controls
| Control | Element it drives |
|---|---|
| K1 Width | the bands' spread: the parabola ×0.25 (the core fills the frame) .. ×4 (a narrow core with ten bands); 512 = the clip |
| K2 Horns | the horn wave's height on both floors: 0 (flat floors) .. ±399 px; 205 = ±80 px, the clip |
| K3 Skew | the horn wave's shape, bipolar: left = slow rise / sharp fall (the clip at 0.5 s), centre = near-triangle teeth, right = sharp rise / slow fall; 820 = the clip at 7.5 s |
| K4 Speed | bipolar (±8 deadband, square law): the warm cap scrolls down (+) or up (−), 0 .. 12 virtual px per field; 761 = the clip's ~165 px/s |
| K5 Teeth | horns per screen width, 2 .. 12, linear; 357 = 5.5 (the clip's 350-px period) |
| K6 Etch | how far the picture (or the Osc lattice) pushes the floor and horns (0 .. ±476 px) and bends the bands (0 .. 119 F); 400 = ±187 px / 47 F |
| S7 Palette | Cyan (the clip) / Gold: the cool palette's chroma rotated 240° (gold core, violet and indigo bands); the warm cap is unchanged |
| S8 Twin | one arch / two arches side by side (half width each) |
| S9 Glow | Blue (the clip's blue-white afterglow below the warm cap) / Gold: a gold halo over the whole frame on every line, +70 / +45 / +5 RGB rising to +104 / +67 / +7 just below the cap. Golden bands, a pale cyan core and an orange floor, at any scroll phase |
| S10 Shape | Arch (the clip) / Well: the parabola inverted, a dark blue slot between cyan walls |
| S11 Source | Video / Osc |
| P12 Bloom | the warm cap's share of the 2200-px cycle: 0 % = all cool, with a thin dark seam (where the horns still bite) scrolling through; 489 = the clip (900 px); 100 % = the white bloom fills the screen with one ~360-px cyan slot |

## Video
- **Video mode (the default).** The camera luma, as (Y >> 2) − 16 from a one-line buffer, is multiplied by Etch.
  - The product pushes the vertical cycle: bright parts of the picture pull the floor, the horns and the warm cap
    up into them.
  - It also bends the bands, so the contours wrap around the picture.
  - With no input, Video mode is the clip. The GHDL default frame with black input is bit-exact with the model.
- **Osc mode.** The shell's 640 × 540 virtual-px triangle lattice, drifting ~75 px/s, feeds the same path, so the
  floor and the band edges wobble with the lattice.
- Etch, the only video-path control, is live in both modes.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial shift-add multiplier with 12 slots (zc 32 + 24 k, operands pre-muxed a clock ahead). Per field it
    computes:
    - kL and kR (Width mantissa from a 256-entry exp2 ROM, ±2 octaves);
    - the horn step (Teeth), the speed step (m² · 3105 ≫ 18) and Bloom's band length;
    - the wave seeds (A S, (A S ≫ 7) · inc, A inc, A inc²);
    - the parabola's start values (d0² kL, |D0|).
  - A shared glide on K1, K2, K3, K5, K6 and P12.
- **Table fill** (zc 600..~1630). 1024 entries through four pipelined stages. The drop comes from a 1024 × 8 shape
  ROM (bump, glow and drop curves, 4 px per entry) that the line engine shares.
- **Per-line engine.** Kicked at zc 1700 and at each line start, it computes the NEXT line:
  - the line's position (Q8, mod 2200 · 256);
  - the bulge term 25 (bump − 1);
  - the Gold halo factor (160 + g/4 + g/16);
  - the three glow adds, read from the shape ROM.
- **Beam.**
  - The parabola DDA (A2 32 bits, D 28 bits, exact). It reloads at the centre and mirrors D at the Twin mid point.
    `beamcheck.py` emulates the registers clock by clock and matches the closed form exactly in all three x-steps,
    Twin on and off, with K1 at 0 / 512 / 1023.
  - The wave accumulators (u 16, ws 27, wd 22, wr 24 bits), with the event flag registered a clock ahead.
- **Pixel path** (17 stages to the compose):
  1. Field quotient and wave max.
  2. Arch or well; the wave clip.
  3. Luma and the skew sign.
  4. Etch product.
  5. Displaced position and the bent field.
  6. mod 2200, to the table address.
  7. Table data.
  8. Fabric copy.
  9. Field index, to the palette addresses.
  10. ROM data.
  11. Fabric copies and the six gates.
  12. Six 8 × 8 products.
  13. Mix plus glow, clipped.
  14. Luma partials.
  15. Y8.
  16. Colour differences and the luma scale.
  17. Compose (black outside the generated avid).

  Then the output box, [1 2 1], 13/16, padding and the gate. C_LAT is 24 in every mode (= processing_delay_clks).
- **Block RAM (21 / 32 EBR).**
  - exp2 256 × 16 (1);
  - shapes 1024 × 8 (2);
  - cool palettes 2 × 512 × 24 (6);
  - warm palette 512 × 24 (3);
  - the vertical table 1024 × 16 (4, `no_rw_check`);
  - the line buffer 2048 × 8 (4);
  - the chroma ring 256 × 16 (1, `no_rw_check`).
- **Contracts.**
  - The generated avid.
  - One latency (24) in every mode.
  - Black outside the generated avid, and the box resync.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr) from the RGB → YUV matrix.
- **Verified (GHDL, 720p60, decimation 1, on the v0.1.0 RTL).**
  - Default knobs, black input, warm-up 6: **100 % bit-exact** with `imodel.py` field 6 in Y, U and V over columns
    16..1263 (99.84 % of the full frame; columns 2..3 are the filter's line-start warm-up). Fields 5 and 7 differ,
    so the RTL advances exactly one model field per frame.
  - Twin (S8 On): **100 % bit-exact** over the same columns.
  - Osc and Video renders at the defaults are in `cmp_final.png`.
  - v0.1.1 changes only the S9 Gold branch of the per-line engine; S9 Blue's code path is untouched. The model's
    default frame still matches the v0.1.0 RTL dump 100 %. S9 Gold has been checked in the model (sheet and
    `s9_try.png`), not by GHDL.

## Status
- **v0.1.1 built 2026-11-04, NOT flashed.** `out/rev_b/ron/heirloom/aureole.vmprog` (476,871 bytes). All six
  configs pass on the LAST post-route line. iceunpack OK ×6.
- **ROUTER1, all six on seed 1.**

  | Config | Seed | Post-route Fmax | LC | EBR |
  |---|---|---|---|---|
  | HD Analog | 1 | 76.35 MHz | 7,202 / 7,680 | 21 / 32 |
  | SD Analog | 1 | 75.59 MHz | 7,208 | 21 |
  | HD HDMI | 1 | 75.26 MHz | 7,206 | 21 |
  | SD HDMI | 1 | 75.79 MHz | 7,206 | 21 |
  | HD Dual | 1 | 76.72 MHz | 7,220 | 21 |
  | SD Dual | 1 | 68.22 MHz | 7,215 | 21 |

  LC is 93.8–94.0 %, at the top of the ~94 % routing guideline. The S9 halo cost ~30 LC and the v0.1.0 Twin fix
  ~95 LC.
- **Rebuild:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh aureole`.
  - Use router1. On the v0.1.1 netlist it closed HD HDMI and HD Dual on seeds 1 and 2 (75.26 / 76.88 and
    76.72 / 75.02 MHz).
  - router2 needed HD HDMI seed 4 and HD Dual seed 5 on v0.1.0, with a 15-minute router stall on each.
  - dupin finds no CARRY I0 == I1 self-adds, so router1 won't hang.
- **History.**
  - v0.1.0 (router2): HD Analog s1 77.77, SD Analog s1 74.32, HD HDMI s4 78.57, SD HDMI s1 75.98, HD Dual s5
    75.73, SD Dual s1 75.98 MHz; 7,174–7,185 LC.
  - The first build was 9,854 LC (128 %). The fit came from:
    - one chroma box on a block-RAM ring instead of two 3-channel boxes in FFs;
    - 8-bit chroma through the output stage;
    - the wave restarting at the line start (no seeded start phase: −5 multiplier slots);
    - the arch centre placed where every mode's beam lands exactly (no residual reload tables).
  - Before shipping v0.1.0, `beamcheck.py` caught the Twin mid-point reload reusing the line-start values (start
    distance 944 / 936 / 448 h instead of 960). That gave a 15 / 22 F seam at 1080p / 720p and a wrong right arch
    at SD. It now mirrors D. `aureole_prev_twinbug.vmprog` in the work dir is that pre-fix build; don't flash it.
  - v0.1.1: S9 Gold became the full-frame halo (review: Gold was invisible at the default phase).
- No pin-skew screen was run. If HD HDMI shows hsync lines, re-seed that config.

## Deviations
- **Floor.** The still's floor is one long drape (one tooth's slow side across the bottom right). The program
  shows the full 5.5-tooth wave at ±80 px, so at the default phase the bottom edge reads as a row of teeth. The
  clip's later frames (horns entering at the top) match better than the still itself. The bottom-centre-right
  region is the largest error (37 levels mean RGB).
- **Top glow.** The top ~250 px is 30–60 levels short in G and B. For example, the teal line at y = 20 is
  (66, 146, 104) against the still's (66, 210, 193). The fitted afterglow is one additive RGB term per line, while
  the clip's glow brightens the teal and blue entries more (a gain). Default model frame against the still: mean
  |RGB| 20.6 (median 9.5); 4 × 4 region means 1–28 levels, apart from that bottom cell.
- **Right edge.** It takes the left half's palette beyond the teal line (teal → dark → blue). The still's right
  edge is teal → blue-cyan → dark → dark green.
- **Centre wobble.** The core's ±12 px S-wobble down the frame is not modelled.
- **Horn colours.** The horn tongues at +1.0 / +1.5 s are dark red with the teal line, as on the warm cap's lower
  edge in the 11 s frames. At 8.5–9.0 s the clip had them pink and bright, with almost no dark band.
- **Chroma lag.** Chroma trails luma by 4 output pixels (the box's centre against the [1 2 1]'s). It is invisible
  on these ≥ 60-px gradients; the clip's own camera smears Cb about as far.
- **Edges.** The left edge fades in over ~8 px (filter warm-up), and the last ~6 px of each line fall behind the gate.
- **Whites.** Whites cap at ~777 output luma (the shell's luma cap). The plateau cyan is exact.
- **Osc and Video at the defaults.** The Etch default (400) lets the Osc lattice and a bright camera lift the
  floor by up to ~187 px and roughen the band edges. The arch, the bands and the floor still read; Etch 0 gives the
  pure clip in either mode.
- **Resolutions.**
  - 720p matches 1080p.
  - SD shows the centre 4:3 (virtual x 240..1680): the blue band and the left teal line fall outside, and the
    right teal line sits at the edge.
  - The horn wave restarts at each mode's line start, so SD's teeth sit at a different phase (start at virtual
    x 256 instead of 8).
- **Presets** are hand-written placeholders. Capture real ones with `./capture_preset.sh`.
