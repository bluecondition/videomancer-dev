# SUGARCOAT v2 — design notes

Status and build record: [SUMMARY.md](SUMMARY.md). Control cards: [CONTROLS.md](CONTROLS.md).

Ron's brief (2026-11-14): take what was learned in APPLIQUE and use it to improve Sugarcoat;
make the candy and cookie textures more realistic and varied ("I want it to look like real
candy and cookies"), with frosting and anything else that adds variety; include APPLIQUE's
thin line / just matte options.

v1 (the signed-off v1.2.2) classified every pixel on its own and painted 16 flat two-colour
textures. v2 keeps the idea (the picture decides where each candy goes, nothing moves on its
own) and replaces both halves: the regions are APPLIQUE's clean mattes, and the fills are
drawn candy.

## 1. Relation to APPLIQUE and VENEER

`sugarcoat.vhd` is a fork of `programs/ron/applique/applique.vhd` v0.1.1 (md5 `21f63c7a…`,
2026-11-14), made once by `.sugarcoat_work/fork.py`. Taken over unchanged: the whole matte
pipeline (front end, smoother, chroma class, quantizer, mode filters, threshold sequencer),
the line (seam) stages, the output stage and its contracts (blanking gate, parity-corrected
avid, [1 2 1] conditioning with luma toward black, one latency of 84 clocks in all modes).
Read VENEER's DESIGN.md sections 2 and 7 for those.

Edits inside the shared code (all marked `SUGARCOAT` in comments):

- **K5 Level**: the smoothed luma gets an offset before the quantizer; the histogram keeps the
  luma as captured (`ys17r` / `ys18r`), so Adaptive thresholds do not cancel the knob.
- **Class instead of slot**: the cleaned label of stage 29 goes through `f_cls` (a few LUTs)
  straight into the theme ROM's address, so the candy id is ready at stage 30 and the recipe at
  31, two H stages (four clocks) earlier than APPLIQUE's. That bought the generator four more
  pipeline stages at the same latency.
- **Lines** compare classes, so a line is drawn only where the candy changes.
- **K4 Shade** is computed in the H domain (stages 33..36) from the bin centroid.
- **Zone latching at full scale**: APPLIQUE's stepped knobs refuse to latch inside 8 counts of
  a zone edge, which includes the top 8 counts of the last zone. A knob or the slider resting at
  full scale from power-up would never select its last zone. Fixed here for K1 and P12; APPLIQUE
  and VENEER still have it.

**If VENEER's or APPLIQUE's matte pipeline changes after hardware, port it here.** Their
first-hardware list (region-edge boil with the temporal vote off, 1-code hysteresis) applies
unchanged, and it shows more here: a region that flips level swaps its whole candy.

## 2. From label to candy

- **Class** (16): 0..7 are luma slots, 8..15 hue sectors. In the Luma map every region uses its
  luma level; in the Chroma map a region with any saturation uses its hue sector instead. The
  levels in force (P12, 2..8) are spread over the 8 slots (2 levels use slots 0 and 7, 4 use
  0 / 2 / 5 / 7), so the darkest region always gets a theme's darkest candy and the lightest its
  lightest; the hue sectors merge into 2 / 4 / 8 groups with the same slider.
- **Theme ROM** (256 x 8): bank, theme, class → candy id. 16 themes; each lists 8 candies for
  the luma slots, re-sorted by measured mean luma when the tables are stamped, and 8 for the hue
  sectors (violet, pink, red, orange / yellow, yellow-green, green, teal, blue), picked by
  colour as far as the theme allows.
- **Candy ROMs** (two, 256 x 16, two words per candy): geometry recipe and colours. 96 candies
  are defined, 128 fit. A candy is a shape, a variant, a size, layout flags and three palette
  colours, so a new candy is a line in `tables.py`, not logic.
- **Palette ROM** (64 colours): Y / 16, U / 32, V / 32 so that chroma has an exact neutral.
  Blocks of eight (brights, pastels, fruit jellies, bakery browns) let an assorted candy pick
  2, 4 or 8 neighbours by the cell hash.

## 3. Candy generator (F domain, full pixel rate)

Twelve stages, C1..C12, beside H stages 31..37. The first five are APPLIQUE's recipe with a
hash and a scatter added:

- **Coordinates**: APPLIQUE's two accumulators (Scale is the step; virtual 1080-line units),
  widened to Q14.10 so a cell index has 5 bits per axis at every size. No drift.
- **C1 axes** straight or bias, turned or not. **C2 size**: an 11-bit field per axis (5 cell
  bits + 6 position bits) at one of four slices; `wide` halves the row height; stagger is a real
  add of half a cell so the cell index follows.
- **C3 hash**: 16 bits per cell from a 4-round substitution / permutation network (4-bit
  S-boxes, PRESENT's): four LUT levels, 64 LUTs, no carry chain. Shift-add hashes with one or
  two adds left neighbour correlations of 0.3–0.7 on some bits; this one measures like a random
  table (`.sugarcoat_work/spntest.py`).
- **C4 scatter**: the piece is moved inside its cell by sign-extended slices of the hash
  (jitter 0 / ±4 / ±8 / ±16 positions; the recipe keeps radius + jitter ≤ 32 so a piece never
  wraps), folded about the cell centre, and optionally given a quarter turn by a hash bit.
- **C5 features**: every carry chain: r² (from the 32-entry square table), the 2:1 ellipse,
  diamond / square / octagon distances, sums and differences, chevron and scallop phases, the
  chip compares, and the **light term**.
- **C6 shape**: 22 shapes x 4 variants as bit tests. Each gives the pixel a region (ground A,
  piece B, detail C, white), an assortment index, a relief height and a shading select.
  Constant compares are written as gates (`f_lt`): as `<` operators they were 320 carry cells.
- **C7 shade**, **C8 palette index**, **C9..C12** colour.

What makes it read as candy rather than as pattern:

| | How |
|---|---|
| Volume | The light term is the dot product of the piece's own (a, b) with a light from the upper left, taken in sign-magnitude so there is no negation: −(a + b) on straight axes, −2 x the axis facing the light on bias axes. Domes use it in 2-D, sticks and piped lines across their width, and ramp shapes (chocolate bevels, shells, rosettes, gem facets) use a 4-bit height instead. |
| Gloss | A specular crescent: the light term above a threshold, inside an inner circle. It replaces the colour with white. Four shading models per candy: matte, soft, glossy, jelly (the absolute value of the light term: bright where the light enters and where it leaves). |
| Edge | APPLIQUE's relief line buffer on a 2-bit height: a lit rim on the upper left of every piece and a contact shadow on the ground at its lower right. |
| Scatter | Position, size, silhouette, colour and turn per cell from the hash. |
| Surface | Grain from a second hash on 4-position micro cells: crumbs (take the candy's detail colour), sugar (bright specks, some of them white), nonpareils (eight colours). |
| Sprinkles | A second small generator on straight axes: one rod per 64-position cell, four directions, eight colours, present when the cell's hash byte is below the density. It overrides colour and height, so sprinkles get rims and shadows too. |

Shapes: plain surface (flat, knife waves, pillows, glaze sheen), disc, ring / filled disc,
bean, chip, rounded square (plain or layered), wedge disc (starlight, citrus slice, quarters),
target (rings, fluted cup, sandwich cookie), heart and gems, triangles (candy corn, bar
peaks, melon slices), stripes, sticks (plain, barber pole, twist, filled), checks, bar
segments, wafer lattice, piped lines, zigzag (piped, drizzle, ribbon), scallops (icing
outline, shells, waves), basketweave and pie lattice, quilting with pearls, rosettes, holes.

Recipe words and palette: the header of `tables.py`. `model.py` mirrors the RTL operation for
operation and draws contact sheets and pictures:

    python3 model.py --sheet out.png           # every candy
    python3 model.py --themes out.png          # the 16 themes, class by class
    python3 model.py --stats                   # mean luma / hue; writes candy_luma.json
    python3 model.py IMAGE out.png --theme 3   # a picture at true 1080p (approximate mattes)
    python3 tables.py                          # restamp the ROMs in sugarcoat.vhd

After changing a candy's colours or shape run `--stats`, then `tables.py`: the themes' luma
lists are sorted from the file `--stats` writes.

## 4. Resolution consistency (warnings for Ron)

- The **candy** is in virtual 1080-line units (APPLIQUE's steps per timing ID), so a jelly bean
  is the same fraction of the screen height at every resolution and round on 4:3 SD.
- The **mattes** are VENEER's pixel kernels, so region shapes are 1.5x coarser relative to the
  screen at 720p and about 2.7x at SD, lines are thicker, and SD is line-doubled.
- At SD the small candies (cells of 16–32 virtual px at low Scale) are 4–7 field lines per
  cell: sticks and stripes survive, small round pieces and the grain alias. Turn Scale up on SD.
- **Grain scales with Scale.** At Scale 0% a grain dot is under one pixel at 1080p: it turns
  into fine noise. From about 25% up it is 2 px and reads as specks.

## 5. Things to know

- **Scale is anchored at the top-left corner**, with a 4-count deadband and a quarter-per-frame
  glide (APPLIQUE's).
- **Value comes from the candy, not the picture.** K4 Shade adds the region's level back; in
  the Chroma map a dark coloured region otherwise gets a candy as bright as its hue's candy is.
- **Whites are about 81% on screen**: the output stage scales luma toward black (the HD HDMI
  bar conditioning, kept as built in APPLIQUE).
- `C_SIM_FAST` (sim copies only) makes the adaptive thresholds and the Scale glide settle in
  one frame.
- In sims set P12 and K1 explicitly; any value works now, including 1023.
