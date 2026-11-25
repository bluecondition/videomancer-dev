# Marquetry: the Syntonie shape rack

**v0.1.1 (2026-11-25): v0.1.0 with the HD HDMI bitstream re-placed (router1 seed 6); awaiting hardware.** Onevolt 8. Three ready-made Rampes figures (a diamond, a rectangle and a
cross), curved together by Courbes, shifted with wrap and cut into each other by Seuils' logic, posterised with
swapped bits by Isohelie and coloured by Matrice: a dense, symmetric inlay of flat colour. Presets are
hand-written placeholders.

## The rack

- **Syntonie Rampes**: the mirrored H and V ramps (1 V at the centre, 0 V at the edges) and the figures made from
  them: DIAMOND (average), RECTANGLE (minimum), CROSS (maximum).
- **Syntonie Courbes**: one curve on both ramps (P12), exponential (t^2) .. linear .. logarithmic (2t - t^2):
  four-pointed stars .. diamonds .. ellipses.
- **Syntonie Seuils**: three ramp shifters. SHIFT knobs K1-K3, the camera on the CV inputs (K6), the logic
  switches S7-S9 (comparator / boolean of the channels).
- **Syntonie Isohelie**: three 3-bit posterisers with the bit-swap multiplexer (K4 BITSWAP, the V ramp on its CV);
  S10 takes it out of the path.
- **Syntonie Matrice**: the colour matrix (K5 sweeps eight crosspoint / invert settings, blended).
- **Syntonie Animate**: the quadrature LFO (0 / 90 / 180 degrees, 23 s) on the Seuils CVs; two slower LFOs (13 s
  on the bitswap offset, 37 s leaning the curve).
- **Syntonie Sortie**: the encoder (clips; never wraps).
- The camera arrives through the shell's decoder (luma, slewed) on Seuils' CV jacks; S11 puts the shell's drifting
  triangle-lattice oscillator on the same jack.

## The patch

    Rampes mirrored H -> Courbes A ; Rampes mirrored V -> Courbes B          (one curve: P12 + 37 s lean)
    Courbes A, B -> average -> Seuils IN A   (DIAMOND)
    Courbes A, B -> minimum -> Seuils IN B   (RECTANGLE)
    Courbes A, B -> maximum -> Seuils IN C   (CROSS)
    camera luma (8-px cells, vertical slew, two horizontal slews) -> Seuils CV A, normalled to B and C;
        CV attenuverters +1 / -1 / +3/4, common depth K6           (S11: the Osc lattice on the same jack, 2x)
    Animate 0 / 90 / 180 deg -> Seuils CV A / B / C (summed in)
    Seuils SHIFT A, B, C -> Isohelie IN A, B, C                     (S10 Out: straight on to Matrice)
    Rampes V ramp -> Isohelie CV A (normalled to B, C); BITSWAP K4 + Animate 13 s
        (channel B's and C's BITSWAP knobs sit three and five zones on)
    Isohelie OUT A, B, C -> Matrice IN 1, 2, 3 (K5) -> Sortie

Each Seuils channel: L = (in > thr) or the boolean on its switch; out = clip(in - thr + (L ? 0 : 1)). With its own
comparator that is fract(in - thr), a wrap; with a boolean, wherever L disagrees with the channel's own
comparator the offset lands on the wrong side and the output clips flat to 0 (L = 1) or 1 (L = 0), in regions
shaped by the other channels' figures.

## What it draws

At the defaults: three symmetric figures on top of each other, the diamond in red, the rectangle in green and the
cross in blue, each a gradient posterised into eight flat steps and shifted with wrap, so each carries one hard
seam in its own shape where the steps jump from full back to black. Where they overlap they make a wall-to-wall
inlay of saturated flat colour (yellow, magenta, cyan, white, black and the 512 mixtures between): nested
diamond facets, rectangular frames, cross bars and stepped corners around a green diamond eye. The bit swap
re-orders the steps differently in three horizontal zones (the seams between them are the only full-width
horizontal edges), so a staircase becomes dark-light-dark-light in one zone and a folded double ramp in another.

Motion at the defaults: Animate's three phases creep the three band systems in and out against each other (the
diamond's steps shrink toward the centre while the rectangle's grow), the zone seams breathe up and down every
13 s, and the curve leans every 37 s, so the diamond's sides bow slightly in and out.

## Controls

| Control | Module | What it does and its range |
|---|---|---|
| K1 Shift A | Seuils SHIFT A | slides the diamond's eight steps and its seam through the figure, with wrap. One full cycle per turn, so 0 % and 100 % are the same picture (the spec: "the two ends meet"). It also moves the A comparator, so it reshapes the S7 / S8 / S9 cuts. Default 420 |
| K2 Shift B | Seuils SHIFT B | the same for the rectangle. Default 620 |
| K3 Shift C | Seuils SHIFT C | the same for the cross. Default 860 |
| K4 Bitswap | Isohelie BITSWAP (+ the V ramp CV) | slides the three horizontal zones up the screen continuously: one zone per 1/8 turn, a full cycle per turn. Each zone has its own bit order: a staircase, swapped steps (dark, light, dark, light), or a ramp folded into two repeats. Default 300 |
| K5 Colour | Matrice crosspoints and inverts | eight colourings over the travel, blended continuously: RGB hot (0, the default), Primaries, CMY, Neon (inverted cross), Ice, Mono, Woods, Fire (100 %) |
| K6 Camera | Seuils CV attenuverters (common depth) | camera luma into the three thresholds (+1 / -1 / +3/4), 0 .. 0.74 cycle per full luma. 0 % = the black-input picture exactly; 100 % (default) = the subject as a contour relief |
| S7 Seuils A | COMP A / A&B | Comp A (default): the diamond wraps. A&B: where the diamond's comparator is on and the rectangle's off, channel A clips flat full, so a big flat red-orange ring (diamond minus rectangle) frames the inner rectangle |
| S8 Seuils B | COMP B / A\|B | Comp B (default). A\|B: in that same ring channel B clips flat black, so green leaves it and a red / blue / magenta picture stays |
| S9 Seuils C | COMP C / A^B | Comp C (default). A^B: channel C clips to 0 or 1 wherever the cross comparator disagrees with A xor B, so blue floods or vanishes in cross-shaped regions and a green / yellow hexagon appears |
| S10 Isohelie | In / Out | In (default): eight hard, bit-swapped steps per channel. Out (the module taken out of the path): smooth wrapped gradients, soft shaded figures with only the seams hard |
| S11 Source | Video / Osc | the camera, or the shell's drifting lattice oscillator on the same Seuils CV jack at twice the camera's level |
| P12 Curve | Courbes curve on both ramps | -100: fully exponential. The diamond is a four-pointed star with concave circular-arc sides; the rectangle's and cross's steps crowd toward the centre. 0 (default): linear, straight-sided diamond. +100: fully logarithmic, the climax. The diamond swells into an ellipse; the rectangle's and cross's steps go wide and pillowed at the centre and crowd at the edges |

## Video

- **Video (default).** Camera luma, (Y >> 2) - 16, averaged over 8-pixel cells, slewed down the screen (one pole,
  1/32 per line; one 12-bit state per cell in a block RAM) and across (two one-pole filters, 1/16 per pixel), is
  patched into Seuils' CV inputs: +1 into A, -1 into B, +3/4 into C, depth K6 (0.74 cycle per full luma at
  100 %). Every seam and every step edge is displaced by the picture, so the subject appears as a contour relief
  in three colours (the bear's head, eyes and snout read at K6 100 %; selfie edges stay clean curves). The
  logic-cut regions follow the picture too, since the comparators see the same thresholds. K6 0 % is exactly the
  black-input picture.
- **Osc.** The shell's drifting 640 x 540 px triangle lattice on the same jack at twice the camera's level: the
  figures dissolve into a lattice of warped, wrapped contour cells drifting across the screen.

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks").**
  - A glide on K1-K6 and P12 (1/8 per field, +-2 codes deadband).
  - Three LFO phases (23 / 13 / 37 s).
  - 19 slots on ONE serial shift-add multiplier, in 32-clock frames from zone clock 40: Animate's five parabolic
    sines t (2 - |t|), the curve c, the camera depth, and the K5 blend. The blend is 12 values (offset and nine
    table steps), each read from a 256 x 16 colouring ROM (value + difference per colouring pair) and multiplied
    by the blend fraction.
  - The shift sums of signed values run in offset-binary form (no sign net on two LUT inputs).
  - Then two table writers at zone clock 680: the Courbes table f(t) by second differences (Q32, 1024 x 16 block
    RAM, no multiplier) and the three Matrice tables (256 x 32 each; entry k = k x step, for Y / 2, U and V).
  - The zone engine needs about 1,730 clocks after the 1,024-clock idle test; every real timing has 14,000 or more.
- **Per line (computed for the next line in hblank).**
  - The V ramp accumulator (2^24 over the field height, from the timing ID).
  - The fold, and a curve-table read on the time-shared port, giving fy.
  - The row-to-row change |fy - fy_prev|; its bit length sets the coverage scale.
  - The bitswap zone.
  - All of these are latched by the next line's start.
- **Per pixel (18 stages to the composed pixel).**
  1. The H ramp accumulator (2^24 over the active line).
  2. The fold and the curve-table address.
  3. The table (EBR).
  4. Fabric copy (fx).
  5. fx + fy and the compares.
  6. DIAMOND / RECTANGLE / CROSS, and the thresholds (base + camera term).
  7. The 16-bit wrapping subtract and the three comparators.
  8. The logic switches and the clip.
  9. Isohelie: the code, the bit order by zone (a case function), the nearer neighbour's code and the distance
     to that step edge.
  10. Coverage: 16 - d >> (e - 5) thirty-seconds, a 1-2 row vertical blend toward the neighbouring step, only
      where the edge has a vertical component.
  11. The 4 x 5 blend product.
  12. The Matrice table addresses.
  13. Three EBR reads.
  14. Fabric copies.
  15, 16. The sums with the offset.
  17. The encoder's clip.
  18. Black off the generated avid.
- **Output.** The shell's conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line
  resync, luma 13/16, a vertical [1 1]/2 with the line above, and the blanking gate.
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 24 (= processing_delay_clks), in every mode.
  - The V ramp spans the field at every timing (virtual 1080-line steps).
  - The H ramp spans the active line (Rampes is screen-locked).
  - Colours in the hardware convention (U = Cr, V = Cb); drawn whites ~777.
- **Block RAM (20).** Courbes table 4, Matrice tables 6, colouring ROM 1, luma cells 1, box delay rings 4, line
  store 4. Every written RAM carries `no_rw_check`.

## Status

- **v0.1.1 (2026-11-25), same RTL, HD HDMI re-placed.** Ron flashed v0.1.0 on HD HDMI and saw thin yellow / cyan
  lines across the whole screen: the encoder pin-skew fault (per-line U / V swap), not the program. The v0.1.0
  HD HDMI placement (router2 seed 1) had HSYNC at -0.63 ns against the forwarded clock (data <= -0.66). v0.1.1
  ships **HD HDMI router1 seed 6: HSYNC +1.45 ns, data <= -1.21 ns, 96.72 MHz** (the window is +1.0 .. +2.3 with
  data <= -0.6), installed with `.heirloom_work/skew_tools/install.sh marquetry router1 6 hd_hdmi`. The other
  five bitstreams are the v0.1.0 ones. A plain `./build_set.sh onevolt marquetry` rebuild gives the router2
  seed-1 placement again: re-run the install afterwards. Screen data: `.heirloom_work/rb_marquetry/skew/`.

- **v0.1.0 built 2026-11-25 with router2 (the build default), all six configs on seed 1, not flashed.**
  Rebuild: `./build_set.sh onevolt marquetry` -> `out/rev_b/ron/onevolt/marquetry.vmprog` (425,822 bytes).
- Post-route Fmax (the builder's last "Max frequency" line per config, all PASS on the first seed):

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 83.77 | 6,489 |
  | SD Analog | 1 | 78.04 | 6,479 |
  | HD HDMI | 1 | 87.83 | 6,487 |
  | SD HDMI | 1 | 84.93 | 6,482 |
  | HD Dual | 1 | 76.88 | 6,500 |
  | SD Dual | 1 | 81.34 | 6,496 |

  6,479 - 6,500 / 7,680 LC (84.6 %), 20 / 32 EBR. iceunpack OK x6. The packed config carries the new labels and
  presets.
  - HD Dual's margin is 2.6 MHz. If a rebuild misses, let the seed retry run, or use router1:
    `NEXTPNR_ROUTER="--router router1" ./build_set.sh onevolt marquetry`. dupin on the synth json reports 0
    CARRY I0==I1 and 0 duplicate LUT inputs.
  - No pin-skew screening.
- **First build attempt.** HD Analog and SD Analog closed (83.75 / 81.47 MHz). HD HDMI's router2 run then stalled
  on seed 1 (more than 11 minutes against about 60 s for the others), and I stopped the build. dupin had found
  three LUTs with duplicate inputs: sums of arithmetic shifts of one signed value in the sequencer. Those were
  rewritten in offset-binary form (exactly equal results) and the rebuild closed all six configs on seed 1.
- **Checks** (working files in `.onevolt_work/rb_marquetry/`):
  - `check.sh` clean.
  - `scan.py`: # 0 files, 0 sites.
  - TOML pre-check passes: 12 parameters, 8 presets, description 116 bytes.
  - `thincheck.py` on the model (720p and 480i; 17 settings x 3 times): 0 single-row lines of 24 px or more.
  - GHDL 720p60, decimation 1, black input: **bit-exact with the integer model `imodel.py`** (0 of 917,444
    compared pixels, rows 1 .. 719; the tester's capture sits 4 px left of the generated avid, as in every shell
    program). That holds for each of these frames:
    - the default (`rtl_default.png`, from the final sources, every control at its TOML default);
    - S7, S8, S9 and S10 each flipped alone;
    - P12 at 0 and at 1023;
    - S7 + S8 + S9 + S10 with K5 439 and P12 200;
    - K5 1023 with K4 0;
    - a later default frame (zone 10; 34 % of pixels changed against the zone-3 frame).
    Non-default frames come from a sim-only copy whose knob glides load in one field (`simprog/`, `mksimcopy.sh`).
  - Both 480i fields (720 x 243, `--video-mode ntsc`): **bit-exact** with the model.
  - Both 1080i fields (1920 x 540, `--video-mode 1080i5994`): **bit-exact** with the model (rows 1 on): the same
    picture as 720p / 1080p.
  - Camera (selfie) and Osc frames match the model's renders by eye (`cmp_selfie.png`, `cmp_osc.png`) but not bit
    for bit: the model's camera path does not reproduce the RTL's cell / slew alignment exactly (separatrix's
    path, same limitation). The differences are band-phase offsets of a few pixels.
  - Standalone yosys (`ysynth.sh`): 3,535 LUT4, 20 SB_RAM40_4K, 0 transparency emulations.
  - `sheet.png`, `strip.png` and `camera.png` are rendered from the integer model.

## Cut / deviations

- **K1-K4 are cycles.** A full knob turn is one full cycle (the spec), so each knob's 0 % and 100 % are the
  same picture. The travel between them is continuous.
- **The curve reshapes only the diamond's outline.** A common monotonic curve commutes with min and max, so the
  RECTANGLE and CROSS keep their outlines under P12. P12 changes the spacing of their steps (pillowed: wide at the
  centre and crowded at the edges in log; the reverse in expo). The diamond carries the shape change (star ..
  diamond .. ellipse).
- **Seuils' threshold is taken modulo one cycle** (a 16-bit wrap), so the knob ends meet and the camera CV can
  never push a channel into the region the real module clips past 1 V.
- **Isohelie's eight bit orders** are not listed in the guide. They are the six permutations of three bits plus two
  multiplexer settings that send one input bit to two outputs. The INV switches are not used. Channel B's and C's
  BITSWAP knobs are fixed three and five zones on (one panel knob, K4, moves all three).
- **Matrice is one designed affine map per colouring** (crosspoints + inverts), eight of them blended on K5. The
  hero (RGB hot) runs the crosspoints above unity (1.25-1.4) with a -0.25 offset, the encoder's contrast pushed.
  The encoder clip is hard (it never wraps) rather than a soft knee.
- **No path-delay misregistration between colours**: the shell's 9-px horizontal conditioning would hide 1-3 px.
- **Coverage is vertical only** (the horizontal conditioning softens near-vertical edges). The logic-cut edges get
  none: they are the other figures' comparator edges, hard as the real comparator makes them.
- **Resolutions.**
  - The H ramp spans the active line at every timing (Rampes is screen-locked), so on 4:3 SD the figures fill the
    4:3 screen (narrower diamonds) instead of being cropped.
  - The V ramp and the zones follow the virtual 1080 lines, so 720p and 1080p draw the same picture.
  - The camera slews run per real line and pixel, so the relief is a little softer at SD.
- The bitswap zone seams are the only full-width horizontal edges (3 zones, 360 virtual rows each), as the spec
  allows.
