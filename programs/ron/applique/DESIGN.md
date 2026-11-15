# APPLIQUE — design notes

Status and build record: [SUMMARY.md](SUMMARY.md). Control cards: [CONTROLS.md](CONTROLS.md).

APPLIQUE is the geometric sibling of VENEER (`programs/ron/veneer/`). Ron's brief (2026-11-14):
VENEER came out with all-organic textures; he wants a second program with the geometric,
fabric-like textures QUILT has ("not necessarily the same ones, but that style"). The picture
is flattened into matte regions exactly as in VENEER, and every region is cut from a sheet of
patterned cloth: quilt blocks, weaves, stripes, checks, dots, tiles.

## 1. Relation to VENEER

`applique.vhd` is a fork of `veneer.vhd` (the texture build) as it stood on 2026-11-14 00:36
(md5 `330ebe9b…`), made once by `.applique_work/fork.py`. Shared, taken over unchanged:

- the whole matte pipeline (front end, smoother, chroma class, quantizer, mode filters,
  thresholds sequencer), the set ROM lookup, the colour stages, the output stage and its
  contracts (blanking gate, parity-corrected avid, [1 2 1] conditioning, one latency of 84).
  Read VENEER's DESIGN.md sections 2 and 7 for these.

Different:

| | VENEER | APPLIQUE |
|---|---|---|
| Fill value `t` | two noise octaves (node engine, 3 EBR) + 8 shapes | pattern generator: 16 patterns x 4 variants, no noise |
| Relief | `t` against 3 pixels left | `t` against 2..3 pixels left **and 2 lines up** (1 EBR line buffer) |
| Scale | none (fixed 64 / 16 px nodes) | continuous, x1..x16, in virtual 1080-line units |
| Motion | octave 2 drifts | the whole cloth drifts |
| Seams | none in the texture build | region outline (VENEER INK's Outline layer, Thin footprint) on S7 |
| Fill switch | Ink / Texture | always pattern (Depth 0 = flat value, colour still alternates) |

The matte code is kept line for line (including the unused VENEER INK generate blocks) so
that fixes VENEER gets after its first hardware session can be carried over with a three-way
diff against the fork point. The two deliberate edits inside shared code are the seam mask
(below) and the recipe register (`r33_w`).

**If VENEER's matte pipeline changes, port it here.** VENEER has not been flashed yet either;
its first-hardware list (region-edge boil with the temporal vote off, 1-code hysteresis)
applies to APPLIQUE unchanged.

## 2. Pattern generator (F domain, full pixel rate)

The cloth is one fixed sheet seen through the regions (patterns are screen-anchored, as in
VENEER). Per pixel:

- **Coordinates** (`p_geo`). Two accumulators in Q11.10 *pattern positions* (64 positions per
  cell at size 0): x steps every pixel, y every active line. T0 registers x, y and the bias
  axes x + y and x − y. The step is `4 / zoom` positions per virtual pixel, times the timing's
  pixel size (section 4); the accumulators start at the Drift offset.
- **T1 axes.** The recipe picks straight (x, y) or bias (x + y, x − y) axes, turned 90° or
  not. S11 flips the bias bit of every recipe.
- **T2 size.** An 8-bit field per axis (2 cell-index bits + 6 position bits) taken at one of
  four fixed slices (cell = 16 / 32 / 64 / 128 virtual px at Scale 0%): a case of slices, not
  a barrel (the QUILT lesson). Stagger: every other row of cells (or half row) slides half a
  cell, by flipping one bit. Folds: distance of each position from its cell's centre line.
- **T3 features.** Every carry chain of the generator: a > b, a + b, a − b, the square /
  diamond / round distances from the cell centre (the squares come from a 32-entry table, not
  a multiplier), the chevron and scallop phases, the goose triangle.
- **T4 pattern.** 16 patterns x 4 variants as bit tests on the features, giving `t` in 16
  levels (0 = tone A, 15 = tone B):

  | Pattern | Variants |
  |---|---|
  | plain | flat, canvas, fine rib, fine twill |
  | stripe | one pair per cell, two pairs, pinstripe, regimental |
  | ridge (ramps) | saw x2, rounded rib, saw x1, triangle x2 |
  | check | checkerboard, gingham, windowpane, fine checkerboard |
  | plaid | tartan with light and dark lines, double light lines, wide / narrow sett, madras |
  | houndstooth | 2/2 twill of 4 + 4 threads, fine twill, mixed with a small check, three-tone |
  | dot | medium, small, large, ring |
  | target | nested squares (log cabin) wide / narrow, rings, nested diamonds |
  | diamond | harlequin, argyle, small diamonds, snowball octagons |
  | stud (ramps) | pyramid, waffle, dome, stepped diamond |
  | triangles | half-square, pinwheel, hourglass, flying geese |
  | basket | ramp slats, slats with gaps, big weave, parquet |
  | zigzag | chevron, herringbone, shaded chevron, fine chevron |
  | brick | running bond, square tiles, tall tiles, shaded bricks |
  | scale | scallop outlines, shaded shells, two-tone waves, fans |
  | cross | plus signs, diagonal lattice, 4 x 4 screen, stars |

- **T5 relief.** The top two bits of `t` against the sample 2..3 pixels to the left and two
  lines up (one field line on interlaced timings), from a 1024 x 4 line buffer that holds the
  two previous lines at half horizontal resolution. Light comes from the upper left, so
  horizontal, vertical and both diagonal edges all get a lit and a shaded side. The
  difference (−3..3) x 21 x 2 is added to `t − 128` when S10 is Embossed; at a region
  boundary the sample belongs to the neighbour's pattern, which bevels the cut edge too.
- **T5..T8 colour and value**: VENEER's, unchanged. Chroma = A + m/4 x (B − A) with m from a
  smoothed copy of `t` (chroma never swings at pixel rate: the HD HDMI bar trigger); luma =
  the region's bin centroid + e x Depth / 8 (VENEER: / 16). **Value always comes from the
  picture.**

Recipe word (set ROM word 0): `[3:0]` pattern, `[5:4]` variant, `[7:6]` size, `[8]` bias,
`[9]` turn, `[10]` stagger, `[11]` invert, `[12]` half stagger. Word 1 is VENEER's colour
word. `tables.py` holds the eight sets and rewrites the constant block of the VHDL;
`model.py` is the integer model of the fill (it mirrors the RTL operation for operation) plus
VENEER's approximate matte model, and draws contact sheets:

    python3 model.py --sheet out.png          # all 16 patterns x 4 variants
    python3 model.py --sets out.png           # the eight sets, slot by slot
    python3 model.py IMAGE out.png --set 3    # a fill on a picture at true 1080p

## 3. Seams

VENEER INK's Outline layer with the Thin footprint (one half-resolution sample: 2 px x 1 line
at 1080p), black, on S7 (Ron's pick, 2026-11-14; v0.1.0 had Medium, 4 px x 2 lines).
One change: a boundary counts only where the **fill** changes. In Palette colour the fill
follows the luma level alone, so the chroma-class bits of the label are masked out of the
three label compares (`c_lmask`); otherwise a seam would be drawn through the middle of a
uniform patch wherever the picture's hue class changes.

## 4. Resolution consistency (warnings for Ron)

- The **patterns** are in virtual 1080-line units: the steps follow the timing ID
  (x: 1 / 1.5 / 2 per pixel at 1080 / 720p / SD; y: 1 / 2 / 1.5 / 4.5 / 3.75 / 2.25 / 1.875
  per line at 1080p / 1080i / 720p / 480i / 576i / 480p / 576p). A check is the same fraction
  of the screen height at every resolution and square on 4:3 SD.
- The **mattes** are VENEER's pixel kernels, so they are **not** the same: region shapes are
  1.5x coarser relative to the screen at 720p and about 2.7x at SD, seams are thicker, and SD
  is line-doubled (no field parity reaches a program). So at SD the same cloth shows through
  blockier holes.
- At SD the small sizes (cells of 16–32 virtual px at low Scale) are only 4–7 field lines
  per cell: stripes survive, dots and diagonals alias. Turn Scale up on SD.
- The relief sample is 2..3 pixels and 2 lines away at every resolution (1 field line when
  interlaced), so the bevel is relatively wider at SD.

## 5. Things to know

- **Scale is anchored at the top-left corner**, not the screen centre (the centre would need
  a per-frame multiply). Turning K5 slides the cloth out from that corner.
- **K5 has a 4-count deadband and a quarter-per-frame glide.** One count of scale moves the
  far edge of the screen by several pattern positions, so raw pot noise would make the whole
  sheet shimmer.
- **Drift is in pattern positions per frame**, so at a larger Scale the cloth moves faster in
  pixels (it always takes the same time to cross one cell). Direction: 2 right-to-left for
  every 1 bottom-to-top, so straight, turned and bias patterns all move.
- **Levels and slots.** As in VENEER a region's slot is its luma level, so with 2 Levels only
  the first two entries of a set are on screen; Patchwork spreads regions over all eight.
- `C_SIM_FAST` (sim copies only) makes the adaptive thresholds and the Scale glide settle in
  one frame.
- Yellow is cream only where the picture is bright: a "cream" tone B in a dark region reads
  as olive. The sets keep yellows for the light slots.
