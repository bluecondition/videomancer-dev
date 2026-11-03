# Heirloom (started 2026-10-20)

One Videomancer program per still photo of Ron's old modular video synthesizer. Each photo is examined and broken
down (what the patch is doing: its shapes, colours, motion cues, textures, the signal behaviour it implies), and the
program is designed from that breakdown and named after what the picture shows.

Flash from `out/rev_b/ron/heirloom/<name>.vmprog`; rebuild one with `./build_heirloom.sh <name>`.

## Status

- **Phase 1 is complete (2026-10-28).** All 55 programs are rebuilt to v0.5.x from their photos with the method in
  [REBUILD.md](REBUILD.md), built in all 6 configs and committed. None has been flashed yet.
- **Phase 2 is next:** one program per tab, turning each replica into a playable modular patch with broader ranges,
  per-element parameters and bolder switches, while the defaults still give the photo. Ron's intentions, the
  per-tab workflow, the known issues and each program's LC/EBR headroom and rebuild command are in
  [PHASE2.md](PHASE2.md). Read it before starting any program.
- **Family-wide fix: output filters now resync every line (2026-11-01).**
  - **Symptom.** On hardware the fern_diag colours came out wrapped: maroon ground, olive maze, green fronds. One
    constant chroma offset, wrapping mod 1024, reproduces the capture.
  - **Cause.** The shared output conditioning is a running 8-sum (`box + new − tap(8 back)`) that never resyncs, so
    a single wrong delayed sample (block-RAM start-up, a clock glitch on a source re-lock) offsets the colours for
    good.
  - **Fix.** Every program now forces the box input neutral off the generated avid and snaps the sum back to N ×
    neutral once N blank samples are in.
  - **Rebuilt.** 49 programs, patch bump, all 6 configs timing-clean. HW-confirmed on fern_diag and facet_diag v0.6.1 ("look correct"). Earlier hand-installed HD seeds are gone; any
    non-builder seed is in that program's SUMMARY.
  - **Already did it:** argus, crossbeam (once a field), curtain, shield, streak, tangram.
  - **chartreuse** is edited but not built; its tab has work in progress.
  - **Rule for new work:** never write a running-sum or integrator filter without a periodic resync.

## Layout

```
programs/ron/heirloom/
  README.md          this file: rules + the photo -> program index
  photos/            drop zone for source photos (JPG/PNG; git-ignored, stays local)
  <name>/            one folder per program
    <name>.vhd
    <name>.toml
    SUMMARY.md       source photo, the breakdown, design mapping, controls, build/HW status
```

Windows path to the drop zone: `\\wsl$\Ubuntu\home\ron\videomancer-dev\programs\ron\heirloom\photos`

## Per-photo workflow

(The original v0.1 workflow, kept for the record. The v0.5 method is [REBUILD.md](REBUILD.md), and phase-2 work
follows [PHASE2.md](PHASE2.md).)

1. **Breakdown**: view the photo and write down what it shows (composition, dominant shapes and how they repeat,
   palette, edge character, implied motion, which classic modular-video process made it, such as a colouriser,
   keyer, ramp/oscillator mixing, feedback or a raster scan). This goes at the top of the program's SUMMARY.md.
2. **Name**: taken from what the picture shows, one word if possible. It must not collide with any existing
   `programs/ron/<name>` or `programs/ron/catalogue-combo/<name>`, because the build links the folder in by name.
3. **Design**: decide what the photo's look becomes as a program, and what each of the 12 controls does.
4. **Build**: `./build_heirloom.sh <name>`, then report post-route Fmax/LC per config. Ron flashes and judges.
5. **Index**: add a row to the table below.

## Rules for every Heirloom program

- **All 12 controls are used**: K1-K6 (`registers_in(0..5)`), S7-S11 (bits 0-4 of `registers_in(6)`),
  P12 (`registers_in(7)`). No unused or placeholder control.
- **P12 slider = the most prominent effect**: the headline performance control. Its full travel must stay
  visibly alive, and 100% must be a designed climax, not math degenerating.
- Every control visibly changes the picture at any setting of the others (none gated behind another's mode);
  switches are instant look-flips.
- Reproduce the photo's LOOK, not its pixels. The program should be able to land on the photographed scene and
  then perform around it. Re-render structurally (shapes, lines, geometry), never a coarse overlay.
- HD-first (1080p @ 74.25 MHz, no clock divisor), the same picture at SD/720p/1080p (virtual 1080-line
  coordinates), and every hardware contract in the repo-root CLAUDE.md (blanking gate, one latency, generated
  avid for drawn colour, luma cap ~768, chroma neutral 512, U/V swap).
- `program_type = "processing"` if `data_in` is read in any mode; `synthesis` only for pure generators.

## Index

All 55 were rebuilt to v0.5 from their photos between 2026-10-21 and 2026-10-28, following [REBUILD.md](REBUILD.md):
measure → float proto → controls → integer model → RTL → GHDL render vs photo → build. Each SUMMARY.md has the
measured reading, the control map, the deviations and the exact rebuild command. Seeds other than 1 and ROUTER1
are listed below; rebuild those with `NEXTPNR_ROUTER="--router router1"` and/or `SEED=<n>`.

| Photo | Program | What it shows | Status |
|-------|---------|---------------|--------|
| WIN_20220629_23_49_20 | [escalator](escalator/) | picture tiled ~7 across, pastel rows with a vivid pink band, green stepped diagonals marching down the grid | v0.6.0 phase 2 (K1 Tiles = grid only under fixed-pitch stairs, bipolar Bands count + height, column bob with Speed, S8 Ramp, wider Blob / Bend), built 2026-11-01, router2, HD seeds pin-skew screened (HD HDMI s45, HD Analog s19, HD Dual s10); not flashed |
| WIN_20220829_23_30_37 | [cocoon](cocoon/) | a woven cocoon in a diamond glow, torn video banded across it | v0.6.0 phase 2 (arcs modulate the weave, K4 fades the red box, P12 band from nothing, S9 Slats), built 2026-10-31, router2, HD seeds pin-skew screened (HD HDMI s11, HD Analog s8, HD Dual s2); not flashed |
| WIN_20221122_23_37_36 | [starburst](starburst/) | 4-fold mirror kaleidoscope of saddle contour bands (gray X's, CMY chevrons), serrated edges | v0.5.0 rebuilt from the photo, built 2026-10-21, all configs seed 3; not flashed |
| WIN_20221124_22_54_05 | [rorschach](rorschach/) | posterized pastel picture over wavy stripes, mirrored left/right, torn sideways in 8-line bands | v0.5.0 rebuilt from the photo, built 2026-10-21, HD Dual seed 3; not flashed |
| WIN_20230103_01_29_30 | [spearhead](spearhead/) | two nested-outline spearheads (black/magenta/yellow/blue/white) over a pastel mosaic of diagonal colour bands | v0.5.0 rebuilt from the photo, built 2026-10-21, 6/6 seed 1; not flashed |
| WIN_20230128_01_06_08 | [mandala](mandala/) | smooth rainbow contour rings lobing out of several centres, mirrored left/right | **v0.6.0 phase 2 FINAL** (diag approved 2026-11-03: face elements as patch points with per-element offsets, K3 Eyes, K6 Breathe drives five element LFOs, K2 Width 0.12..8, P12 x0.125..x3.6, S9 Glow Ramp / Bands), router2 (HD Analog seed 2); 96 % LC |
| WIN_20230129_00_40_34 | [streak](streak/) | smeared colour streaks over a frozen mosaic | v0.5.0 rebuilt from the photo, built 2026-10-21, 6/6 seed 1; not flashed |
| WIN_20230207_23_50_54 | [argyle](argyle/) | diamond lattice of nested rhombus rings, magenta cores, fine blue/green/pink rings | v0.6.0 phase 2 (K1 Bloom / K2 Scatter swell or shrink every diamond in its own cell, replacing the Columns / Rows zoom), built 2026-10-29, router2, HD Analog / HD Dual seed 3, HD HDMI seed 9 hand-installed (pin-skew screen), 94 % LC, 25 EBR; not flashed |
| WIN_20230222_00_06_47 | [tangram](tangram/) | three stripe fields (diagonals, verticals, counter-diagonals, rainbow) keyed into each other, seams torn sideways in bands | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1, HD Dual seed 5; not flashed |
| WIN_20230223_01_46_41 | [crenel](crenel/) | hot-pink crenels and camera domes, tiled by a bent oscillator | v0.6.0 phase 2 (K2 Dome radius, K4 per-band drift, S7 Swap), built 2026-11-01, router1, HD seeds pin-skew screened (HD HDMI r2 s17, HD Analog s3, HD Dual s7); not flashed |
| WIN_20230305_23_38_39 | [terrace](terrace/) | a colour-bar staircase through two quantizers | v0.5.0 rebuilt from the photo, built 2026-10-21, HD HDMI seed 2; not flashed |
| WIN_20230305_23_51_38 | [medallion](medallion/) | warm stripes folded into two chevron systems, a central circle showing the video as a hard CMY plaid | v0.5.0 rebuilt from the photo, built 2026-10-21, HD Analog/HDMI seed 2; not flashed |
| WIN_20230307_01_04_17 | [vesica](vesica/) | an orange eye in a pastel lattice | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1 6/6 seed 1, 97% LC, ROUTER1 6/6 seed 1, 97% LC; not flashed |
| WIN_20230307_23_36_05 | [marmalade](marmalade/) | a sample-and-hold mosaic through a nine-zone orange colourizer | **v0.6.0 phase 2 FINAL** (diag approved 2026-11-03: K1 Cells / K2 Rows per row (detune, cascade), K5 ringing .. smear, S7 Drift, S9 Tilt Slant, S10 Mirror, P12 1..36 bands + band width), ROUTER1 6/6 seed 1; 89 % LC |
| WIN_20230309_00_28_35 | [riptide](riptide/) | a torn, mirrored swirl band | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1, HD Analog seed 2, 99% LC, ROUTER1, HD Analog seed 2, 99% LC; not flashed |
| WIN_20230315_00_28_42 | [jukebox](jukebox/) | a wall of rainbow capsules | **v0.6.1 phase 2 FINAL** (diag approved by Ron after flashing it, merged 2026-11-03: K1 Widths Lens / Fan folding past L = 1, K2 Heights Cascade x10 / Alternate x20, K3 Nose 0-200 %, K5 Detune, S8 Fold Tri / Saw, E interpolation; the thin rainbow lines (one bad fitted table entry) fixed), built 2026-11-03, router2 6/6 seed 1 (HD 83.1 / 86.6 / 85.5 MHz), 95.5 % LC, 21 EBR; bitstreams byte-identical to the approved diag; not yet flashed as production |
| WIN_20230315_00_30_57 | [cusp](cusp/) | horizontal rainbow stripes bent into right-pointing cusps by a triangle of x, navy inside the deep cusps | v0.6.0 phase 2 (K1 black-core rows only, K2 Split, rows slide apart, stripes scroll / curl, rainbow swing, S8 Opposed, S9 Rainbow core), built 2026-11-01, router2, HD seeds pin-skew screened (HD HDMI s138, HD Analog s48, HD Dual s11); not flashed |
| WIN_20230315_00_53_56 | [sinew](sinew/) | ropes and sinews | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1 6/6 seed 1; not flashed |
| WIN_20230315_00_58_54 | [plume](plume/) | mirrored central plume of red/magenta/yellow chevron ripples out of green and blue flow | v0.5.0 rebuilt from the photo, built 2026-10-27, HD HDMI + SD HDMI seed 2; not flashed |
| WIN_20230319_01_27_56 | [ticker](ticker/) | camera video through a sample-and-hold mosaic | v0.5.0 rebuilt from the photo, built 2026-10-21, HD HDMI + SD Analog seed 2; not flashed |
| WIN_20230325_11_04_55 | [argus](argus/) | 4-fold mirrored stripes bent into arrow chevrons, magenta centre band, dark eyes | v0.6.0 phase 2 (S7 Mirror Fold / Off: Off replaces Repeat, the silhouettes run unmirrored over the folded colour field), built 2026-10-29, router2, 6/6 seed 1, 94 % LC, 22 EBR; v0.5.0 flashed OK apart from Repeat, v0.6.0 not flashed |
| WIN_20230406_23_49_00 | [blossom](blossom/) | nested red/green/magenta chevron petals opening across flat magenta, pink lobes | v0.6.0 phase 2 (cascading Rows, Sway, heartbeat Breathe, Twist, Bloom +-3), built 2026-10-30; HD HDMI s12 / HD Analog s5 hand-installed; not flashed |
| WIN_20230407_23_48_05 | [cathode](cathode/) | a cycling S&H colour mosaic with three-tube purple flames | v0.6.1 phase 2 (writhing flames: per-tube travelling waves, licking tips; the camera bends them), built 2026-10-31, router2, HD seeds pin-skew screened (HD HDMI s32, HD Analog s6, HD Dual s1); not flashed |
| WIN_20230410_01_10_37 | [lozenge](lozenge/) | nested black diamonds with road-marking edges, cut by squeezing stripes | **v0.6.0 phase 2 FINAL** (K1 Stripes x1/8..x8, K2 Chirp to 3x the photo's squeeze, K3 Aspect -> Bulge: 4-point stars / quatrefoil .. diamonds .. round lens rings, K5 Black 0..3x), approved by Ron 2026-11-03 as previewed (diag bitstreams repacked), ROUTER1 6/6 seed 1, 93 % LC, 19 EBR; presets are placeholders |
| WIN_20230410_01_13_24 | [tartan](tartan/) | diagonal stripe field sliced into offset, recoloured bands with black bars across | v0.5.0 rebuilt from the photo, built 2026-10-27, HD HDMI seed 2; not flashed |
| WIN_20230424_23_53_49 | [beacon](beacon/) | a candle flame and a hyperbolic bell keyed into blue bars | v0.6.1 phase 2 round 2 (blue flame and beam against amber bars, the camera warped into the flame; round 1: wider Bars / Columns / Beam, Dusk and Halo close in on the tower, flame centred, Speed swells the bands), built 2026-10-30, all six bitstreams hand-routed (HD HDMI router2 s23, HD Analog / Dual router1), 96 % LC, 29 EBR; v0.6.0 flashed, v0.6.1 not flashed |
| WIN_20230424_23_59_47 | [vase](vase/) | a glossy lathe vase striped by a rolling colour cycle | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1, HD Analog seed 5; not flashed |
| WIN_20230429_22_57_43 | [bottleneck](bottleneck/) | two glass bottles on glowing stripes | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1, SD HDMI seed 2; not flashed |
| WIN_20230515_00_46_28 | [magma](magma/) | streaked cyan/magenta/blue field with yellow cores, flat red where it saturates, mirrored | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1 6/6 seed 1; not flashed |
| WIN_20230515_23_08_59 | [ricochet](ricochet/) | nested cyan / lavender chevrons pointing right on orange, mirrored top and bottom, flat beyond the last | v0.5.0 rebuilt from the photo, built 2026-10-21, ROUTER1 6/6 seed 1; not flashed |
| WIN_20230515_23_14_37 | [idol](idol/) | a camera idol folded in a frame-store mirror, colourized on flat yellow | **v0.6.1 phase 2 FINAL** (diag approved by Ron after flashing it, merged 2026-11-03: K1 Feather streaks + slips, K3 Axis 13..91 % with the idol riding the line (stacked idols low), K4 Speed swell + ripple, K6 Ground wider, S9 Palette Photo / Rainbow, P12 Bands 0.25..4x), built 2026-11-03, router2 6/6 seed 1, 94 % LC |
| WIN_20230515_23_17_11 | [kiln](kiln/) | camera fired in a frame-store kiln | **v0.6.1 phase 2 FINAL** (Ron flashed the v0.6 diag: "pretty much the perfect program"; K1 Tiles 0.5..6.4 exp, K2 tear to 3.9x, K3 Blocks 0.25..4x, K5 Rims -1..2.9x + Video rims, S9 Strips, S10 Tile Lock: whole tile counts gliding into place), 2026-11-03, router2 (HD Analog s4, HD HDMI s2, HD Dual s6); presets are placeholders |
| WIN_20230515_23_23_36 | [lotus](lotus/) | twin contour eyes over pink cushions, torn by a stuttering sync | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1, HD Dual seed 2; not flashed |
| WIN_20230521_01_34_12 | [curtain](curtain/) | red drapes, swept folds and pleat cuts | v0.6.1 phase 2 (Seam wipe, Weave, Swap / Ribbons / Fire; pleats still at the default Speed), built 2026-11-01, router2, HD Analog s5, HD HDMI s11 hand-installed, HD Dual s6; not flashed |
| WIN_20230521_23_37_25 | [bunting](bunting/) | bow-tie pennants under a grid of keyed glyph marks | v0.6.0 phase 2 (pennant Band, mirrored Flare, glyph slide, morphing glyphs, H-then-V Spread), built 2026-10-30, ROUTER1, HD seeds s5 / s7 / s2 for pin skew; not flashed |
| WIN_20230523_01_26_43 | [archway](archway/) | a pointed arch of stepped mosaic, black arcs and cyan lines | v0.6.0 phase 2 (Steps 5.5–194 px, cubic Etch, Speed splits lines / grid by side, Chevron gone .. outer arc, smooth Palette morph, S9 Flip), built 2026-10-28, router2, HD Analog / HDMI / Dual seed 2, SD Analog seed 3, 96 % LC, 20 EBR; v0.5.0 flashed, v0.6.0 not flashed |
| WIN_20230527_01_43 | [crossbeam](crossbeam/) | a glowing cross beam keyed over torn colour bars | v0.6.0 phase 2 (Scatter / Lens rows and columns, Warp, Swing, push, Slip), built 2026-11-01, ROUTER1 6/6 seed 1; Ron 11-01: "meh", parked |
| WIN_20230529_22_56_41 | [shield](shield/) | twin striped shields, plaid in nested U bands, blue pillars | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1 6/6 seed 1; not flashed |
| WIN_20230530_22_27 | [ribboncandy](ribboncandy/) | twisted ribbons over a sample-and-hold mosaic | v0.5.0 rebuilt from the photo, built 2026-10-27, router2, HD Analog seed 2, HD HDMI/Dual seed 3; not flashed |
| WIN_20230602_01_26 | [facet](facet/) | mirrored green eyes, red cone, white-edged wedge and floor | v0.6.2 phase 2 (K5 Spread per-eye-pair heights, K4 rate + depth with eye ripple and wedge/floor slide, S7 Floor V / Peak, S8 Eyes Lens / Diamond; K2 sweeps to flat, K3 slope rides the height, wedge/floor overlap XOR with the video), approved 2026-11-02 after two diag rounds, ROUTER1 (HD Analog seed 2), 97 % LC, 26 EBR; v0.6.1 diag flashed (colours correct) |
| WIN_20230602_01_30 | [cathedral](cathedral/) | teal egg-crate arches around a pink diamond | v0.6.0 (phase 2) K2 Arch to x4, K3 Diamond 10..569 lines with its halo, built 2026-10-30, router2 (HD HDMI s61 + HD Analog s3 hand-installed); not flashed |
| WIN_20230605_01_03_50 | [abacus](abacus/) | rows of glossy beads on their rods | v0.6.3 phase 2 (Width / Height resize every bead in its slot; Slide / Roll pulse each bead / row in X and Y; wider Rims, Flip glide, deeper Swell; S11 Theme Photo / Gold; S9 V Mirror), v0.6.3 built 2026-10-28, router2, HD Analog / HD Dual / SD HDMI seed 2, 96 % LC, 27 EBR; v0.6.1 flashed OK, v0.6.3 not flashed |
| WIN_20230605_01_14_56 | [lava](lava/) | flame lobes dripping into blue-violet pods, striped magenta curtain at the left | **v0.6.0 phase 2 FINAL** (K1 Columns x0.25..x4, K2 Bend -1..3 with a valley side, K3 per-band row heights, K4 Speed x2, K5 Ripple -> Drip per-drop breathing tongues / arches, K6 Melt x2, S8 Ground Stripes / Camera), approved by Ron 2026-11-03 as previewed (diag bitstreams repacked), router2 (HD HDMI s2, SD Dual s2), 94.6 % LC, 26 EBR |
| WIN_20230610_00_10_54 | [seismic](seismic/) | a chirp across five ribbons, torn into staircase strips | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1 6/6 seed 1; not flashed |
| WIN_20230610_00_51 | [waterfall](waterfall/) | a curtain of chirped lines falling through four bands | v0.5.0 rebuilt from the photo, built 2026-10-27, ROUTER1 6/6 seed 1; not flashed |
| WIN_20230713 | [fern](fern/) | nested pink / orange / yellow parabola fronds, mirrored, over a teal ripple field with a blue zone | **v0.6.2 phase 2 FINAL** (Ron picked the v0.6 diag: K1 Fan, K2 Lean, Speed waves, flames follow Arrow, S9 Echo; plus wide Lean / Flesh / Arrow 1/16x..3x), built 2026-11-02, ROUTER1, HD Analog s7 + HD Dual s10 hand re-routes; not flashed |
| WIN_20230801_00_19_00 | [halo](halo/) | glowing rings in red crescents between blue hourglass columns | v0.6.0 phase 2 (K1 Rings x0.19..x2.25, K2 Cells 0.5..8, K3 Pinch -100..220 %, K5 Tear -100..300 %, K6 Warp -15..185 %, S9 Ember Crescent / Core), built 2026-11-02, router2 (HD HDMI seed 3, rest seed 1); not flashed |
| WIN_20230909_23_29_18 | [warp](warp/) | a perspective stripe plane warped into checkers, moiré and loops | v0.5.0 rebuilt from the photo, built 2026-10-27, router2, HD HDMI seed 4; not flashed |
| WIN_20230916_01_08_00 | [goblet](goblet/) | a glass goblet over RGB stripe oscillators | v0.6.0 (phase 2): wider K1 Stripes (x0.25..x4), K2 Width (x0.4..x3) and K5 Melt (x2.9, bolder Osc); built 2026-11-02, router2 (HD HDMI seed 3, HD Dual seed 2); not flashed |
| WIN_20231003_23_01_50 | [ovals](ovals/) | chains of rainbow-roped vesica ovals on a striped pink ground | v0.5.1 rebuilt from the photo, built 2026-10-28, router2, HD Dual seed 2, 99.6% LC; not flashed |
| WIN_20231211_00_38_58 | [chartreuse](chartreuse/) | a glossy lime surface folded into a sphere and crescents | v0.6.1 (phase 2): crescent spacing, arm swing, sphere cone, bigger breathing, S8 Mirror; built 2026-11-02, ROUTER1 (HD Dual seed 3, rest seed 1); not flashed |
| duci.jpg (collage) | [radiant](radiant/) | pink paper rays burst from a dark halo behind a bust with a flag lens and a cup of coffee on his head | v0.5.0 rebuilt from the collage, built 2026-10-28, router2, base SEED=3 6/6; not flashed |
| selfie.jpg (collage) | [triplicate](triplicate/) | the picture photocopied in red, green and blue, cascading over a wall of prints | v0.5.0 rebuilt from the collage, built 2026-10-21, 6/6 seed 1, thin HD margins; not flashed |
| television.jpg (collage) | [greenroom](greenroom/) | the TV in the green room: a painted collage with the camera on its screen | v0.5.0 rebuilt from the collage, built 2026-10-28, ROUTER1 6/6 seed 1, 95% LC; not flashed |
| tezcatlipoca.jpg (collage) | [cutout](cutout/) | a collage head of torn magazine scraps on a yellow ground | v0.6.0 phase 2 (K2 Crops, per-scrap drift, S8 Inks swap, S10 camera mirror), built 2026-11-01, ROUTER1, HD seeds pin-skew screened, 98% LC, 27 EBR; not flashed |
