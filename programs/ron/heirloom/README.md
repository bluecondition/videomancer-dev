# Heirloom (started 2026-10-20)

One Videomancer program per still photo of Ron's old modular video synthesizer. Each photo is examined and broken
down (what the patch is doing: its shapes, colours, motion cues, textures, the signal behaviour it implies), and the
program is designed from that breakdown and named after what the picture shows.

Flash from `out/rev_b/ron/heirloom/<name>.vmprog`; rebuild one with `./build_heirloom.sh <name>`.

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

| Photo | Program | What it shows | Status |
|-------|---------|---------------|--------|
| WIN_20220629_23_49_20 | [escalator](escalator/) | picture tiled ~7 across, pastel rows with a vivid pink band, green stepped diagonals marching down the grid | v0.2.0 (Source switch) built 2026-10-20, 6/6 (HD Dual seed 2), HD 80–87 MHz, 5.0k LC, 15 EBR; v0.1.0 previewed |
| WIN_20220829_23_30_37 | [cocoon](cocoon/) | stretched hexagon of woven chevrons in a red→blue radial glow, posterized video keyed in and torn at the flanks | v0.2.0 (Source switch) built 2026-10-20, 6/6 (HD Dual seed 2), HD 75–80 MHz, 4.45k LC, 19 EBR; v0.1.0 previewed |
| WIN_20221122_23_37_36 | [starburst](starburst/) | 4-fold mirror kaleidoscope of saddle contour bands (gray X's, CMY chevrons), serrated edges | v0.2.0 (Source switch) built 2026-10-20, 6/6 seed 1, HD 79–80 MHz, 4.6k LC, 17 EBR; v0.1.0 previewed |
| WIN_20221124_22_54_05 | [rorschach](rorschach/) | posterized pastel picture over wavy stripes, mirrored left/right, torn sideways in 8-line bands | v0.1.0 built 2026-10-20, 6/6 (HD HDMI seed 3), HD 78 MHz, 4.1k LC, 15 EBR; not flashed |
| WIN_20230103_01_29_30 | [spearhead](spearhead/) | two nested-outline spearheads (black/magenta/yellow/blue/white) over a pastel mosaic of diagonal colour bands | v0.1.0 built 2026-10-20, 6/6 (SD HDMI seed 2), HD 75–83 MHz, 4.87k LC, 15 EBR; not flashed |
| WIN_20230128_01_06_08 | [mandala](mandala/) | smooth rainbow contour rings lobing out of several centres, mirrored left/right | v0.1.0 built 2026-10-20, 6/6 (HD Dual seed 4), HD 75.0–78.9 MHz, 5.9k LC, 9 EBR; not flashed |
| WIN_20230129_00_40_34 | [streak](streak/) | horizontal rainbow smear, hue turning band by band, barcode blocks below a diagonal | v0.1.0 built 2026-10-20, 6/6 (HD Dual seed 2), HD 74.7–80.6 MHz, 5.0k LC, 15 EBR; not flashed |
| WIN_20230207_23_50_54 | [argyle](argyle/) | diamond lattice of nested rhombus rings, magenta cores, fine blue/green/pink rings | v0.1.0 built 2026-10-20, 6/6 (HD Dual seed 2), HD 78.6–88.0 MHz, 4.1k LC, 9 EBR; not flashed |
| WIN_20230222_00_06_47 | [tangram](tangram/) | three stripe fields (diagonals, verticals, counter-diagonals, rainbow) keyed into each other, seams torn sideways in bands | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 82.9–84.8 MHz, 3.9k LC, 6 EBR; not flashed |
| WIN_20230223_01_46_41 | [crenel](crenel/) | navy field, hot-pink crenellation combs, blue chrome relief video torn in bands, pink V's | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 81.8–89.9 MHz, 3.7k LC (48 %), 5 EBR; not flashed |
| WIN_20230305_23_38_39 | [terrace](terrace/) | three panels of a cell grid dropping one cell per column: diagonal staircases of yellow / magenta / cyan / white / grays / black | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 77–80 MHz, 4.1k LC, 15 EBR; not flashed |
| WIN_20230305_23_51_38 | [medallion](medallion/) | warm stripes folded into two chevron systems, a central circle showing the video as a hard CMY plaid | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 80–87 MHz, 4.1k LC, 9 EBR; not flashed |
| WIN_20230307_01_04_17 | [vesica](vesica/) | orange vesica eye of nested rings with crescents, on a pastel diamond lattice over pale green | v0.1.0 built 2026-10-20, 6/6 (HD Analog seed 3, SD HDMI seed 2), HD 76–88 MHz, 4.46k LC, 12 EBR; not flashed |
| WIN_20230307_23_36_05 | [marmalade](marmalade/) | Mondrian orange / magenta / yellow blocks in row bands slid sideways, the picture block-torn under them | v0.1.0 built 2026-10-20, 6/6 (HD Analog / SD Analog / HD HDMI seed 2), HD 77–79 MHz, 4.44k LC, 15 EBR; not flashed |
| WIN_20230309_00_28_35 | [riptide](riptide/) | saturated smeared rainbow band across the middle, blue/red speckle streaks above and below | v0.1.0 built 2026-10-20, 6/6 (SD Dual seed 2), HD 78–80 MHz, 4.5k LC, 15 EBR; not flashed |
| WIN_20230315_00_28_42 | [jukebox](jukebox/) | ten tall panels of rainbow rounded-square rings mirrored top/bottom | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 78.7–80.1 MHz, 3.7k LC, 13 EBR; not flashed |
| WIN_20230315_00_30_57 | [cusp](cusp/) | horizontal rainbow stripes bent into right-pointing cusps by a triangle of x, navy inside the deep cusps | v0.1.0 built 2026-10-20, 6/6 (SD Analog seed 2), HD 85.1–95.8 MHz, 3.4k LC, 6 EBR; not flashed |
| WIN_20230315_00_53_56 | [sinew](sinew/) | vertical warm bars wrung into S-curves by a vertical sine inside a centre band, row ripple | v0.1.0 built 2026-10-20, 6/6 (HD Dual seed 3), HD 74.4–75.8 MHz, 4.7k LC, 15 EBR; not flashed |
| WIN_20230315_00_58_54 | [plume](plume/) | mirrored central plume of red/magenta/yellow chevron ripples out of green and blue flow | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 82.5–97.1 MHz, 3.1k LC (41 %), 8 EBR; not flashed |
| WIN_20230319_01_27_56 | [ticker](ticker/) | monochrome bars scrambled sideways per row band, cyan dashes: a readout gone wrong | v0.1.0 built 2026-10-20, 6/6 (HD Analog seed 3, HD HDMI / HD Dual seed 2), HD 76–79 MHz, 3.4k LC, 5 EBR; not flashed |
| WIN_20230325_11_04_55 | [argus](argus/) | 4-fold mirrored stripes bent into arrow chevrons, magenta centre band, dark eyes | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 84.9–96.5 MHz, 3.4k LC (45 %), 8 EBR; not flashed |
| WIN_20230406_23_49_00 | [blossom](blossom/) | nested red/green/magenta chevron petals opening across flat magenta, pink lobes | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 86.7–91.5 MHz, 3.4k LC (44 %), 5 EBR; not flashed |
| WIN_20230407_23_48_05 | [cathode](cathode/) | torn magenta / green / yellow pixel mosaic with two purple parabolic tubes sweeping down | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 77–88 MHz, 4.3k LC, 15 EBR; not flashed |
| WIN_20230410_01_10_37 | [lozenge](lozenge/) | nested black / pink / olive diamonds cut by colour-swapping stripes, one-sided highlight lines | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 74.4–88 MHz (HD Dual marginal), 3.65k LC, 5 EBR; not flashed |
| WIN_20230410_01_13_24 | [tartan](tartan/) | diagonal stripe field sliced into offset, recoloured bands with black bars across | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 75.0–82.2 MHz, 3.3k LC, 5 EBR; not flashed |
| WIN_20230424_23_53_49 | [beacon](beacon/) | white flame (glow + cone) over blue/black blocks torn into dashes | v0.1.0 built 2026-10-20, 6/6 (HD Analog seed 2), HD 80–84 MHz, 5.0k LC, 15 EBR; not flashed |
| WIN_20230424_23_59_47 | [vase](vase/) | tall vase silhouette on black filled with glossy rainbow stripes, row-torn | v0.1.0 built 2026-10-20, 6/6 (HD HDMI seed 2), HD 77–83 MHz, 4.6k LC, 19 EBR; not flashed |
| WIN_20230429_22_57_43 | [bottleneck](bottleneck/) | two pink bottle silhouettes with lime rims over smeared blue/green/black bands | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 80–86 MHz, 4.6k LC, 9 EBR reported; not flashed |
| WIN_20230515_00_46_28 | [magma](magma/) | streaked cyan/magenta/blue field with yellow cores, flat red where it saturates, mirrored | v0.1.0 built 2026-10-20, 6/6 (SD HDMI seed 2), HD 84–97 MHz, 3.2k LC, 9 EBR; not flashed |
| WIN_20230515_23_08_59 | [ricochet](ricochet/) | nested cyan / lavender chevrons pointing right on orange, mirrored top and bottom, flat beyond the last | v0.1.0 built 2026-10-20, 6/6 (HD HDMI seed 3, SD HDMI seed 2), HD 74.7–80.5 MHz, 4.3k LC, 5 EBR; not flashed |
| WIN_20230515_23_14_37 | [idol](idol/) | mirrored relief-shaded column of the picture in blue/orange chrome on flat yellow | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 76.4–89.1 MHz, 3.7k LC (48 %), 15 EBR; not flashed |
| WIN_20230515_23_17_11 | [kiln](kiln/) | the picture through a fire ramp (violet shadows, red, yellow), glowing vertical bands, streaked | v0.1.0 built 2026-10-20, 6/6 (HD Dual seed 2), HD 80–94 MHz, 3.2k LC, 9 EBR; not flashed |
| WIN_20230515_23_23_36 | [lotus](lotus/) | orange / green nested diamonds above a seam, pink-white lobes on acid green below, two across | v0.1.0 built 2026-10-20, 6/6 (HD Analog seed 2, HD HDMI seed 6), HD 74.6–76.3 MHz, 4.42k LC, 5 EBR; not flashed |
| WIN_20230521_01_34_12 | [curtain](curtain/) | magenta drapes wrung into S-curves on red, blue pools, lime piping on the crests, torn rows | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 79–83 MHz, 3.45k LC, 4 EBR; not flashed |
| WIN_20230521_23_37_25 | [bunting](bunting/) | yellow and cyan wedge fan over white, rows of stepped red / blue glyph blocks | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 78–81 MHz, 4.55k LC, 15 EBR; not flashed |
| WIN_20230523_01_26_43 | [archway](archway/) | tall arch of cyan/blue parabola rays, stepped pink/yellow/navy blocks on the flanks, pink chevron | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 82–86 MHz, 4.0k LC, 13 EBR; not flashed |
| WIN_20230527_01_43 | [crossbeam](crossbeam/) | soft cyan X over torn Mondrian colour blocks, red ripples in the dark ones | v0.1.0 built 2026-10-20, 6/6 (HD Dual seed 2), HD 77–81 MHz, 4.2k LC, 2 EBR; not flashed |
| WIN_20230529_22_56_41 | [shield](shield/) | heraldic shields: orange striped arches over blue/red/black V chevrons, 2 across, mirrored | v0.1.0 built 2026-10-20, 6/6 (HD Analog seed 2), HD 80–83 MHz, 4.9k LC, 15 EBR; not flashed |
| WIN_20230530_22_27 | [ribboncandy](ribboncandy/) | vertical S-curved two-colour ribbons wobbling over a black-and-white block field with CMY dashes | v0.1.0 built 2026-10-20, 6/6 (HD HDMI seed 3), HD 77–84 MHz, 5.4k LC, 4 EBR; not flashed |
| WIN_20230602_01_26 | [facet](facet/) | flat-shaded geometric facets cut by three rotating wedge fields and a circle, mirrored, the picture as relief | v0.1.0 built 2026-10-20, 6/6 (HD Analog seed 2), HD 75–87 MHz, 5.1k LC, 4 EBR; not flashed |
| WIN_20230602_01_30 | [cathedral](cathedral/) | teal gothic parabola arches, mirrored, on a rainbow-banded ground with a pink diamond | v0.1.0 built 2026-10-20, 6/6 (HD Analog + HD Dual seed 2), HD 75–77 MHz, 4.4k LC, 3 EBR; not flashed |
| WIN_20230605_01_03_50 | [abacus](abacus/) | rows of glossy red beads lit from above with chrome rims, blue/green blob rows between | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 77.6–84.4 MHz, 4.2k LC, 5 EBR; not flashed |
| WIN_20230605_01_14_56 | [lava](lava/) | flame lobes dripping into blue-violet pods, striped magenta curtain at the left | v0.1.0 built 2026-10-20, 6/6 (HD HDMI seed 2, rest seed 1), HD 85.0–89.8 MHz, 3.7k LC (48 %), 20 EBR; not flashed |
| WIN_20230610_00_10_54 | [seismic](seismic/) | rainbow smear rows alternating with rows of glowing scalloped lenses on black | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 77.1–85.4 MHz, 3.9k LC, 8 EBR; not flashed |
| WIN_20230610_00_51 | [waterfall](waterfall/) | a curtain of thin green lines bending below a fold into an orange stripe fan, over pastel blocks | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 77–84 MHz, 3.5k LC, 2 EBR; not flashed |
| WIN_20230713 | [fern](fern/) | nested pink / orange / yellow parabola fronds, mirrored, over a teal ripple field with a blue zone | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 76–81 MHz, 3.75k LC, 5 EBR; not flashed |
| WIN_20230801_00_19_00 | [halo](halo/) | glowing yellow-green rings on dark olive, blue / violet chevrons between, mirrored cells | v0.1.0 built 2026-10-20, 6/6 (HD HDMI seed 2), HD 74.4–81.4 MHz, 4.57k LC, 6 EBR; not flashed |
| WIN_20230909_23_29_18 | [warp](warp/) | orange / green / violet stripes bent into arcs about the centre and sheared in blocks, black at the left | v0.1.0 built 2026-10-20, 6/6 (HD HDMI seed 3), HD 74.9–81.0 MHz, 5.1k LC, 6 EBR; not flashed |
| WIN_20230916_01_08_00 | [goblet](goblet/) | cyan glass goblet outline with chrome highlights over mirrored rainbow stripes | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 78–87 MHz, 4.3k LC, 19 EBR; not flashed |
| WIN_20231003_23_01_50 | [ovals](ovals/) | lattice of rimmed lenses with white/orange cores on a streaked hot-pink ground | v0.1.0 built 2026-10-20, 6/6 (HD HDMI seed 3, HD Dual seed 2), HD 78.1–80.6 MHz, 4.6k LC, 19 EBR; not flashed |
| WIN_20231211_00_38_58 | [chartreuse](chartreuse/) | glossy lime-green folds with chrome highlight lines and a soft orb | v0.1.0 built 2026-10-20, 6/6 (HD Dual seed 2), HD 78–84 MHz, 3.9k LC, 4 EBR; not flashed |
| duci.jpg (collage) | [radiant](radiant/) | a sunburst of pink/white rays round a dark halo on a maroon ground, the video cut in at the centre | v0.1.0 built 2026-10-20 (router1), 6/6 seed 1, HD 77.9–81.5 MHz, 5.5k LC, 16 EBR; not flashed |
| selfie.jpg (collage) | [triplicate](triplicate/) | RGB-tinted copies of the keyed picture cascading behind the original over a gray tiled wall of itself | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 74.9–83.7 MHz, 4.83k LC, 18 EBR; not flashed |
| television.jpg (collage) | [greenroom](greenroom/) | green/teal painterly wash with orange splatters, a rounded television window showing the video | v0.1.0 built 2026-10-20, 6/6 (SD Analog seed 2), HD 88.6–91.2 MHz, 4.18k LC, 19 EBR; not flashed |
| tezcatlipoca.jpg (collage) | [cutout](cutout/) | a head-profile cutout on a flat yellow ground filled with the video torn into blue/teal paper pieces | v0.1.0 built 2026-10-20, 6/6 seed 1, HD 76.1–82.9 MHz, 4.43k LC, 15 EBR; not flashed |
