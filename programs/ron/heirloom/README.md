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
| tezcatlipoca.jpg (collage) | [cutout](cutout/) | a collage head of torn magazine scraps on a yellow ground | v0.6.0 phase 2 (K2 Crops, per-scrap drift, S8 Inks swap, S10 camera mirror), built 2026-11-01, ROUTER1, HD seeds pin-skew screened, 98% LC, 27 EBR; not flashed |

## From the video clips (phase 3, started 2026-11-04)

Stills taken out of the 2023–2024 Camera Roll mp4 clips (481 clips, 164 capture sessions, 1,099 distinct looks), capped at
**50 programs**, one per patch (clips shot back to back are one patch). The brief is [VIDEO.md](VIDEO.md); the picks,
grades and reserve list are in `.heirloom_work/vid/` (`queue.txt`, `triage.txt`). Stills: `photos/vid/<clip>_t<sec>.png`
with a motion strip and four motion frames beside each. Programs are built to the phase-2 standard from the start,
versions from v0.1.0.

| # | Still (clip, time) | Program | What it shows | Status |
|---|---|---|---|---|
| 1 | 20230127_01_48_55 t25.5 | tongues | twin red/yellow flame tongues mirrored, green ground at right, black | v0.1.0 built 2026-11-04 (ROUTER1; SD HDMI seed 2, others seed 1; 6957–6992 LC, 16 EBR); not flashed |
| 2 | 20230203_00_19_56 t6.0 | tapers | magenta/cyan/blue candle-flame columns on black | v0.1.0 built 2026-11-04 (router2; HD Analog seed 2; 6296–6308 LC, 9 EBR); not flashed |
| 3 | 20230205_00_16_05 t0.5 | zigzag | green/rainbow zigzag V ribbons with white/blue highlights | v0.1.0 built 2026-11-04 (ROUTER1 6/6 seed 1; 5863–5882 LC, 21 EBR); not flashed |
| 4 | 20230206_00_35_06 t0.5 | frogspawn | green/black cellular contour lattice | v0.1.1 built 2026-11-04 (router2; HD Analog seed 2, HD Dual seed 3; HD HDMI and HD Dual thin, +0.3/+0.5 MHz; 6730–6752 LC, 17 EBR); not flashed |
| 5 | 20230210_01_00_21 t3.0 | peaks | pink/orange/violet sunset peaks with teeth | v0.1.0 built 2026-11-04 (ROUTER1 6/6 seed 1; 7033–7048 LC, 17 EBR); not flashed |
| 6 | 20230216_01_27_34 t2.5 | grommets | purple/blue ring lattice with black holes | v0.1.0 built 2026-11-04 (router2; HD HDMI seed 3, HD Dual seed 2; 6650–6675 LC, 23 EBR); not flashed |
| 7 | 20230221_00_58_22 t0.5 | placard | hard-edged yellow/magenta/cyan/black graphic with stripe hatching in a frame | v0.1.0 built 2026-11-04 (ROUTER1 6/6 seed 1; 7150–7158 LC, 15 EBR); not flashed |
| 8 | 20230301_23_17_23 t0.5 | undulate | rainbow sine ripple field | v0.1.1 built 2026-11-04 (router2 6/6 seed 1; HD HDMI thin, 74.31 MHz; 7054–7072 LC, 21 EBR); not flashed |
| 9 | 20230305_00_05_43 t0.5 | chalice | mirrored chalice figure with rainbow contours on black | v0.1.0 built 2026-11-04 (router2 6/6 seed 1; 6923–6952 LC, 17 EBR); not flashed |
| 10 | 20230314_23_42_54 t7.5 | aureole | soft glowing white/cyan/orange bloom figure | v0.1.1 built 2026-11-04 (ROUTER1 6/6 seed 1; 7202–7220 LC, 94 %, 21 EBR); not flashed |
| 11 | 20230401_00_20_03 t0.5 | lattice | magenta/green/blue diamond trellis with blue diamonds | v0.1.0 built 2026-11-04 (ROUTER1 6/6 seed 1; 6552–6566 LC, 15 EBR); not flashed |
| 12 | 20230404_23_59_18 t9.5 | zebra | black/white zebra stripe swirl with red/blue fringes | v0.1.0 built 2026-11-04 (router2; HD Analog seed 3, SD Dual seed 2; HD margins thin; 6917–6940 LC, 19 EBR); not flashed |
| 13 | 20230428_23_30_39 t0.5 | vinyl | fine blue/red vinyl-groove moire rings | v0.1.0 built 2026-11-04 (router2 6/6 seed 1; 7140–7177 LC, 93 %, 18 EBR); not flashed |
| 14 | 20230501_01_34_42 t0.5 | lentil | bright pink curved lens bands around blue diamonds | v0.1.0 built 2026-11-04 (router2 6/6 seed 1; 6910–6948 LC, 17 EBR); not flashed |
| 15 | 20230509_23_58_21 t0.5 | wildfire | green/yellow/blue rainbow flame contours | v0.1.1 built 2026-11-09 (ROUTER1 6/6 seed 1; 7035–7072 LC, 23 EBR); not flashed |
| 16 | 20230518_23_23_28 t0.5 | crucible | orange/blue flame cross on magenta with mirrored blue shapes | v0.1.1 built 2026-11-04 (ROUTER1 6/6 seed 1; 6994–7010 LC, 21 EBR); not flashed |
| 17 | 20230529_01_27_21 t0.5 | serpent | red/green/yellow serpent S-ribbon on black | v0.1.0 built 2026-11-04 (ROUTER1 6/6 seed 1; HD Dual thin, 74.46 MHz; 7086–7107 LC, 18 EBR); not flashed |
| 18 | 20230618_20_23_10 t0.5 | hourglass | rainbow hourglass lens pair on black | v0.1.0 built 2026-11-04 (router2; HD HDMI seed 2; 5648–5661 LC, 74 %, 21 EBR); not flashed |
| 19 | 20230620_00_50_54 t0.5 | saltire | magenta/yellow/green/blue hard-edged neon X with wavy edges | v0.1.0 built 2026-11-04 (ROUTER1 6/6 seed 1; HD Dual thin, 75.27 MHz; 7179–7200 LC, 24 EBR); not flashed |
| 20 | 20230703_01_29_07 t10.0 | horizon | cyan/blue/yellow planet horizon with ribs | v0.1.0 built 2026-11-04 (ROUTER1 6/6 seed 1; 7030–7058 LC, 18 EBR); not flashed |
| 21 | 20230710_00_15_14 t1.5 | plumage | orange/magenta/green feather fan curves | v0.1.0 built 2026-11-04 (router2; HD HDMI seed 2; HD Analog thin, 75.05 MHz; 7140–7173 LC, 23 EBR); not flashed |
| 22 | 20230712_00_46_37 t2.5 | scales | cyan/orange/pink crescent fish scales | v0.1.0 built 2026-11-04 (ROUTER1; SD Analog seed 2; 6922–6933 LC, 20 EBR); not flashed |
| 23 | 20230717_23_51_09 t7.5 | ironwork | magenta/green/black lace contour ironwork | v0.1.1 built 2026-11-04 (ROUTER1 6/6 seed 1; 6802–6827 LC, 22 EBR); not flashed |
| 24 | 20230722_01_18_41 t5.5 | carpet | red/white/orange kaleidoscopic carpet | v0.1.0 built 2026-11-04 (ROUTER1 6/6 seed 1; 6777–6794 LC, 14 EBR); not flashed |
| 25 | 20230731_01_19_22 t0.5 | pebble | red/orange concentric dotted ripple rings | v0.1.0 built 2026-11-04 (router2 6/6 seed 1; 6696–6766 LC, 21 EBR); not flashed |
| 26 | 20230816_00_32_31 t5.0 | temple | orange/yellow/blue symmetric sunset temple with blue flames | v0.1.0 built 2026-11-04 (ROUTER1; HD Dual seed 2; 6410–6443 LC, 23 EBR); not flashed |
| 27 | 20230820_01_33_09 t0.5 | brushfire | orange/green/white burning-grass streaks | v0.1.0 built 2026-11-09 (ROUTER1; HD Dual seed 2; 7087–7113 LC, 23 EBR); not flashed |
| 28 | 20230825_00_00_20 t6.5 | arcade | orange/blue ring arches on blue stripes | v0.1.0 built 2026-11-09 (router2; SD Dual seed 2; 6866–6891 LC, 19 EBR); not flashed |
| 29 | 20230905_00_44_28 t0.5 | coral | red hexagon core with cyan coral rings on light blue | v0.1.1 built 2026-11-09 (ROUTER1 6/6 seed 1; 6020–6035 LC, 23 EBR); not flashed |
| 30 | 20230906_00_33_30 t11.0 | spine | magenta/purple/black torn diagonal spine | v0.1.0 built 2026-11-09 (ROUTER1 6/6 seed 1; 6930–6944 LC, 22 EBR); not flashed |
| 31 | 20230910_00_11_50 t7.0 | quill | blue/white feather plumes on black | v0.1.0 built 2026-11-09 (router2 6/6 seed 1; 7018–7057 LC, 16 EBR); not flashed |
| 32 | 20230914_01_59_34 t4.0 | marble | cream/orange/red marble swirl arcs with blue dots | v0.1.0 built 2026-11-09 (router2 6/6 seed 1; 6891–6902 LC, 15 EBR); not flashed |
| 33 | 20230919_00_32_29 t3.0 | neon | magenta/white glowing S-curve neon tubes on teal | v0.1.0 built 2026-11-09 (router1; all six seed 1; 6,744-6,768 LC, 21 EBR); not flashed |
| 34 | 20230930_01_37_53 t0.5 | brazier | blue/purple nested diamond with pink flames on a red/black ground | v0.1.0 built 2026-11-09 (ROUTER1 6/6 seed 1; 6724–6754 LC, 18 EBR); not flashed |
| 35 | 20231002_01_20_50 t0.5 | lantern | magenta/yellow/black nested petal contours around a pink disc | v0.1.0 built 2026-11-09 (router2; SD Dual seed 2; 5114–5133 LC, 18 EBR); not flashed |
| 36 | 20231009_00_32_55 t0.5 | cumulus | blue/yellow/magenta/cyan scalloped cloud waves with drips | v0.1.0 built 2026-11-09 (router2 6/6 seed 1; 6990–7005 LC, 16 EBR); not flashed |
| 37 | 20231104_16_39_25 t0.5 | streamers | bold rainbow wavy vertical ribbons | v0.1.1 built 2026-11-09 (router1; HD HDMI seed 2, others seed 1; 6,946-6,959 LC, 24 EBR); not flashed |
| 38 | 20231116_23_41_37 t3.5 | hawser | yellow/black/red wavy rope strands with red glints | v0.1.0 built 2026-11-09 (ROUTER1 6/6 seed 1; 6824–6840 LC, 22 EBR); not flashed |
| 39 | 20231119_00_47_47 t0.5 | herringbone | blue/white zigzag herringbone stripes | v0.1.0 built 2026-11-09 (router2 6/6 seed 1; 5974–5986 LC, 12 EBR); not flashed |
| 40 | 20231123_23_27_50 t5.5 | confetti | magenta petal cards over a striped grey ground with red/blue stripes | v0.1.0 built 2026-11-09 (router2; HD HDMI seed 5, others seed 1; 6,890-6,914 LC, 22 EBR); not flashed |
| 41 | 20231124_00_45_44 t5.0 | pyramid | teal/pink pyramid with a striped core on a dark vignette | v0.1.0 built 2026-11-09 (router2; all six seed 1; 6,724-6,731 LC, 20 EBR); not flashed |
| 42 | 20231204_21_01_30 t0.5 | platter | pink dotted vinyl disc on dark with curved magenta/green strands | v0.1.1 built 2026-11-09 (router1; all six seed 1; 7,108-7,116 LC, 24 EBR); not flashed |
| 43 | 20231204_23_13_02 t5.5 | plaid | red/yellow/black woven plaid with wavy curves | v0.1.0 built 2026-11-09 (router1; all six seed 1; 7,019-7,046 LC, 22 EBR); not flashed |
| 44 | 20231205_23_09_56 t0.5 | bunsen | orange/blue flame on blue; blue with yellow flame blobs | v0.1.0 built 2026-11-09 (router1; all six seed 1; 6,917-6,945 LC, 23 EBR); not flashed |
| 45 | 20231227_22_52_47 t3.5 | kachina | red/magenta/blue/yellow mirrored totem mask | v0.1.1 built 2026-11-09 (router1; all six seed 1; 6,831-6,849 LC, 24 EBR); not flashed |
| 46 | 20231229_00_46_59 t6.0 | effigy | yellow idol in red/black flames, mirrored | v0.1.0 built 2026-11-09 (router1; all six seed 1; 7,082-7,118 LC, 23 EBR); not flashed |
| 47 | 20240102_00_30_49 t1.0 | cateye | pink/yellow chevron eye | v0.1.1 built 2026-11-09 (router1; all six seed 1; 7,147-7,177 LC, 21 EBR); not flashed |
| 48 | 20240104_00_30_04 t3.0 | grotto | blue/white/black cavern eye with cyan edges | v0.1.0 built 2026-11-09 (router2; HD Dual seed 2, others seed 1; 6,870-6,884 LC, 23 EBR); not flashed |
| 49 | 20240105_20_46_57 t0.5 | bubbles | bubble lenses in cyan/green/red wavy bands | v0.1.0 built 2026-11-09 (router1; all six seed 1; 6,995-7,014 LC, 17 EBR); not flashed |
| 50 | 20240216_23_50_17 t0.5 | velvet | magenta/white/yellow vertical drapes with a reflection | v0.1.1 built 2026-11-09 (router1; all six seed 1; 7,135-7,143 LC, 20 EBR); not flashed |

## From the video clips, part 2 (phase 3b, started 2026-11-09)

Stills taken out of the same clips: 30 more patches, picked after a visual de-dupe of every remaining A/B-graded session
against all 101 existing heirloom stills (`.heirloom_work/vid/dedupe.py`, sheets in `.heirloom_work/vid/dd/`). Same brief and standard.

| # | Still (clip, time) | Program | What it shows | Status |
|---|---|---|---|---|
| 51 | 20230129_23_23_56 t0.5 | gable | red/cyan pointed chevron fan on dark, cyan/magenta streak alt | v0.1.0 built 2026-11-09 (router2; HD Dual seed 2, others seed 1; 6,836-6,870 LC, 19 EBR); not flashed |
| 52 | 20230214_23_42_08 t0.5 | breakers | cyan/blue wavy ribbon bands; green/red/blue contour crab alt | v0.1.1 built 2026-11-09 (router1; all six seed 1; 7,003-7,035 LC, 21 EBR); not flashed |
| 53 | 20230318_12_24_18 t4.0 | tilework | skewed colour-block grid; cyan/green mesh checker alt | v0.1.0 built 2026-11-09 (router1; all six seed 1; 6,917-6,937 LC, 7 EBR); not flashed |
| 54 | 20230318_14_13_38 t0.5 | staircase | cyan diagonal pixel staircase; magenta/cyan random block mosaic alt | v0.1.0 built 2026-11-09 (router2; all six seed 1; 6,453-6,481 LC, 6 EBR); not flashed |
| 55 | 20230328_00_11_14 t0.5 | pods | green/yellow cell eyes lattice in olive; grey chevron stacks with rainbow fringes alt | v0.1.1 built 2026-11-09 (router1; all six seed 1; 6,437-6,455 LC, 19 EBR); not flashed |
| 56 | 20230402_23_03_06 t0.5 | seaweed | green/magenta wavy vertical curtain bands | in progress (wave 4) |
| 57 | 20230408_23_11_38 t3.5 | pupil | blue/yellow moire lens eyes on flat green | v0.1.0 built 2026-11-10 (router1; all six seed 1; 6,546-6,564 LC, 13 EBR); not flashed |
| 58 | 20230424_23_47_33 t8.0 | tatter | white torn tear over orange/magenta diamond zigzags (beacon/vase session, different patch) | in progress (wave 4) |
| 59 | 20230627_23_37_56 t7.0 | nesting | blue/magenta/red mirrored nested squares | in progress (wave 4) |
| 60 | 20230723_00_52_24 t0.5 | tunnel | black/white concentric diamond moire eye | v0.1.0 built 2026-11-09 (router1; all six seed 1; 6,755-6,764 LC, 13 EBR); not flashed |
| 61 | 20230804_22_27_52 t6.0 | mummer | green/red/blue/magenta contour mask faces | in progress (wave 4) |
| 62 | 20230817_00_06_49 t0.5 | gothic | green/blue/red arch stripes | in progress (wave 4) |
| 63 | 20230818_01_17_40 t3.0 | owlet | blue/cyan contour eyes on a red ground; purple/red blobs alt | v0.1.0 built 2026-11-09 (router1; all six seed 1; 7,088-7,119 LC, 19 EBR); not flashed |
| 64 | 20230904_01_56_58 t0.5 | specter | green ghost flame figure on black with blue floor | in progress (wave 4) |
| 65 | 20230906_22_43_27 t2.5 | mantis | green/purple/white kaleidoscope insect | in progress (wave 4) |
| 66 | 20230907_00_18_59 t5.5 | cupola | red/orange/green sunset bands with blue domes | v0.1.1 built 2026-11-09 (router1; all six seed 1; 6,647-6,667 LC, 22 EBR); not flashed |
| 67 | 20230927_23_58_07 t2.5 | batwing | magenta/yellow/blue mirrored bat-wing kaleido; green/yellow zigzag alt | v0.1.0 built 2026-11-09 (router1; SD HDMI seed 2, others seed 1; 6,865-6,881 LC, 21 EBR); not flashed |
| 68 | 20231003_00_48_14 t1.5 | marionette | magenta/green/blue curtain of hanging figures | v0.1.0 built 2026-11-09 (router1; all six seed 1; 7,127-7,165 LC, 21 EBR); not flashed |
| 69 | 20231008_20_41_21 t0.5 | butterfly | magenta/blue/black kaleido butterfly | in progress (wave 4) |
| 70 | 20231022_00_05_49 t0.5 | oilslick | orange/blue oil-slick swirl; green/magenta contour blobs alt | in progress (wave 4) |
| 71 | 20231026_23_13_39 t0.5 | vine | green/purple/black diagonal flame streaks | in progress (wave 4) |
| 72 | 20231121_21_36_42 t0.5 | portal | cyan/magenta/green mirrored arcs with stripes | in progress (wave 4) |
| 73 | 20231208_23_03_08 t0.5 | tiger | orange/blue/green wavy diagonal tiger bands | in progress (wave 4) |
| 74 | 20231218_00_10_10 t0.5 | harlequin | orange/purple/green wavy stripes with orange diamonds | in progress (wave 4) |
| 75 | 20231218_21_31_11 t3.0 | starline | magenta/green neon chevron star lines on black; rainbow stripes with white dots alt; B/W silhouettes alt | v0.1.0 built 2026-11-09 (router1; all six seed 1; 7,078-7,119 LC, 23 EBR); not flashed |
| 76 | 20231227_00_20_47 t6.0 | ocelot | orange/black/blue wavy zebra stripes with blue dots | v0.1.0 built 2026-11-09 (router2; all six seed 1; 5,677-5,710 LC, 19 EBR); not flashed |
| 77 | 20240111_01_01_24 t0.5 | yolk | yellow/red/white flames and eggs mirrored; magenta/blue triangles alt | v0.1.0 built 2026-11-09 (router2; HD Analog seed 2, HD HDMI seed 2, others seed 1; 6,871-6,878 LC, 23 EBR); not flashed |
| 78 | 20240117_22_55_32 t0.5 | coastline | blue field with a red/yellow/green wavy ridge | v0.1.0 built 2026-11-09 (router2; HD HDMI seed 2, others seed 1; 6,844-6,864 LC, 15 EBR); not flashed |
| 79 | 20240207_23_34_28 t0.5 | hornet | yellow/black/red horizontal bands with eyes; red/cyan vertical gloss alt | in progress (wave 4) |
| 80 | 20240213_23_38_52 t3.0 | flamecrest | magenta/red/yellow flame fringe on black; masks alt | v0.1.1 built 2026-11-09 (router1; all six seed 1; 7,186-7,203 LC, 23 EBR); not flashed |
