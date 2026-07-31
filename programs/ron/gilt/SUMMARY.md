# GILT — Klimt "Golden Phase" re-render

Live video re-rendered as a Gustav Klimt *Golden Phase* painting (the world of
*Adele Bloch-Bauer I* and *The Kiss*). Flesh stays painted and human; everything
else is **gilded** — flattened into gold leaf, mosaic tesserae and hand-placed
ornament. The signature is the hard **collision** between the two worlds and
ornament that is *placed, never tiled*.

Fully streaming, no frame buffer. `C_LATENCY = 13`. Four EBR (one contour
line buffer, 2048×8). Three multiplies (final dry/wet lerp only); everything
else is shift/add, a 32-entry squared-distance LUT and a 32-entry gold-ramp
logic ROM.

## Pipeline (per pixel)

| Stage | Work |
|---|---|
| E1 | input latch, position, dry SR, prev-pixel luma, line-buffer read |
| E2 | flesh chroma distance · region-hash A · luma-gradient edge |
| E3 | region-hash B → mood / fine-subdivide / shear / cell-size |
| E4 | sheared cell coords · cell-hash A · tessera seam · grain-hash A |
| E5 | cell-hash B → motif id/accent · dx,dy → \|dx\|,\|dy\| · grain-hash B · ramp index |
| E6 | squared-distance / max · gold-ramp ROM read · gild key |
| E7 | motif band compares → layer select · patina chroma adjust |
| E8 | compose ground (+grain +tessera +meadow) with ornament on top |
| E9 | contour ink + halo |
| E10 | flesh restore (hard-edged warmed original) |
| E11–E13 | GILD dry/wet mix (P12 staged), 3 inline multiplies |

The geometry (dx→|dx|→r²) and the gold-ramp (index→ROM→patina→clamp) each
started as a single overloaded stage and were the two successive critical
paths on HD HDMI; splitting each across two/three stages lifted all six
configs from ~67–73 MHz to a full-clock close (see trades below).

## The "not a grid" ornament engine

Irregularity is built in *structurally*, not sprinkled on afterward. (v0.1's
single fixed cell size per region read as a grid — v0.2 replaces it with a real
quadtree.)

- **Quadtree cell sizes** — every pixel resolves its cell by testing three
  independent coarse-block hashes at 64/32/16 px. A block that "subdivides"
  resolves finer while its neighbour may not, so **big plain cells sit beside
  clusters of tiny ones and cell edges never fall on one lattice**. `P4 SCALE`
  sets the subdivide probability (bold poster blocks → fine jeweller detail);
  the multi-scale *mix* survives at every setting. The three hashes evaluate in
  parallel (not a serial descent) to stay timing-shallow.
- **Per-region shear + mood** — a coarse 128 px hash shears the cell lattice so
  neighbouring regions don't align, and sets a **mood** that only nudges each
  region's *plainness* (calm vs. busy), never forces one motif.
- **Per-cell motif** — each cell's own hash picks its motif, so a region is a
  *mix* of motifs and plain gold, never a repeated tile. In-cell coordinates are
  normalized to a 0–63 space so motifs are size-independent. Plain gold stays
  the common outcome (`P3` plain-gate, capped).
- **Tessera follows the cells** — the mosaic crackle is a thin dark seam on each
  irregular cell edge (not a fixed 16 px lattice), so the gold fractures along
  the varied cell boundaries like Byzantine leaf. The seam is wobbled ±1 px by
  the grain hash so the crack wanders like hand-fractured leaf.
- **Hand-placed, hand-cut** (v0.5) — every motif centre is jittered ±4 by its
  cell hash; glyph radii/squares are grain-dithered so band edges are slightly
  torn; concentric rings carry a per-cell phase so no two cells' rings align;
  bar edges are fuzzy. Nothing sits exactly on-grid or ruler-straight.
- **Drape** — the ground colour is the *input luma* run through the gold ramp,
  so the leaf follows the form (bright cheek → pale gold, fold → umber) for
  free, per-pixel, with no cell sampling.

## Motif vocabulary (per-pixel, buffer-free)

Bank A "Adele": checker (gold/ink/bone 2×2), concentric squares, almond eye
(triangle + oval), squared spiral, disc/rings, speckle. Bank B "Kiss": vertical
black bars (robe), concentric colour discs/rings (dark centre), spiral,
rosettes (meadow). Shared: tessera crackle (thin dark seams on the irregular
quadtree cell edges) and gold-leaf grain speckle. Each cell picks its own motif
independently, so motifs never tile.

## Palette

Gold ground is a 9-stop ramp (ink brown-black → deep umber → bronze → ochre →
brass → pale gold → bone highlight) interpolated to 32 entries, indexed by
input luma. Values are 10-bit YUV, BT.601 full-range, authored in standard
`U=Cb / V=Cr` (gold pulls U down and V up — warm metal, never a flat yellow
tint). Jewel accents (crimson, lapis, viridian, violet, ivory, dead black) are
used only as small incidents, gated by `P5`.

**Hardware chroma swap.** This Videomancer hardware carries chroma with U and V
swapped relative to standard BT.601 (same as cga / mondrian / phosphor in the
library). Authoring the palette in standard convention would render gold as
*aqua* and would also mis-target the flesh key. Rather than swap every constant,
GILT swaps chroma once at the input latch and once at the output, so all
internal logic stays in plain standard convention while the hardware boundary is
corrected; video passthrough remains bit-identical.

## Control map

| | | |
|---|---|---|
| P1 | **Flesh** | width of the skin chroma-key; faces surface out of the metal |
| P2 | **Patina** | walks the ramp: smoky brown-black → brass → cool silver-gold + warmth |
| P3 | **Ornament** | coverage / plain-gate (capped so plain gold always survives) |
| P4 | **Scale** | base cell size 8–64 px (multi-scale irregularity preserved at every setting) |
| P5 | **Jewel** | frequency + saturation of accent colours |
| P6 | **Drift** | animation rate; 0 is truly frozen (only flesh moves) |
| S7 | **Mosaic** | tessera crackle on/off |
| S8 | **Outline** | contour ink on/off. In Luma mode, repurposed as the overlay EDGE switch: On = clean per-pixel cut at the luma boundary, Off = whole cells (quilt-style full glyphs of every size pop in/out intact) |
| S9 | **Luma** | luma-mod overlay: the base render stays untouched; where input luma exceeds the P1 threshold, a SECOND independent gilt world (own quadtree cells/shear/motifs, pale-gold ground + dark ornament + jewels) is revealed on top |
| S10 | **Bank** | Adele / Kiss motif bank (instant dramatic switch); also auto-enables the Meadow flower field at the bottom of frame |
| S11 | **Halo** | radiant gold aura ringing the flesh key |
| P12 | **Gild** | the performance fader, staged (below) |

**P12 staging** (all monotonic in P12 → the pull-back reverses identically):
0–15% barely-warmed video · 15–35% gold creeps up **from the shadows** (a rising
luma ceiling gilds dark tones first) · 35–60% mids flatten, contour ink appears ·
60–85% ornament blooms · 85–100% flesh key tightens to face+hands, jewels and
halo arrive.

## Intent-vs-timing / resource trades

- **Per-pixel luma drape instead of per-cell BRAM sampling.** Turpentine
  samples each cell's centre pixel into a BRAM and paints the whole cell from
  it. GILT instead colours the ground from the *current* pixel's luma through
  the gold ramp. This is cheaper (no capture BRAM, no prefetch engine) and
  gives smooth intra-cell tonal drape, at the cost that a motif's *fill*
  decision uses the cell hash while its *brightness* tracks live luma — motifs
  can therefore show a faint internal gradient. In practice this reads as the
  ornament being lit by the form, which is desirable. The alternative (BRAM
  cell sampling) would give perfectly flat motifs but costs an EBR + a
  prefetch FSM and was judged not worth it here.
- **Shear-per-region, not true rotation.** Non-orthogonal cells are
  approximated by a per-region horizontal+vertical shear of an axis-aligned
  lattice rather than a full rotation (which needs a per-region sin/cos and a
  multiply per pixel). Neighbouring regions still visibly break alignment; a
  literal rotate would be the upgrade if LC budget allows.
- **Tessera crackle is a staggered dual-lattice**, not a true 2-nearest
  Voronoi. A real Voronoi seam needs a 3×3 jittered-centre scan (9 distance
  compares/pixel); the staggered brick-offset lattice gives irregular fragment
  boundaries at a fraction of the logic. Fragments are quad-ish rather than
  arbitrary polygons — the visible difference is small under grain + gold.
- **Contour is a 2-tap gradient** (prev-line via the one line buffer + prev
  pixel), not a full Sobel. Enough to ink strong flesh/ornament borders; a 3×3
  Sobel would need a second line buffer.
- **Halo is the soft key-transition band**, not a distance transform. The ring
  just outside the flesh key is brightened toward gold; true radiant arcs/rays
  would need the flesh centroid (a per-frame reduction) — deferred.

## Luma-mod overlay (v0.6)

S9 keeps the base render untouched and reveals a **second, fully independent
gilt world** wherever the input is brighter than the P1 threshold. The
overlay has its own quadtree (different hash salts + shear, so its cell
boundaries never align with the base), its own moods and motif bank (bold
tall bars, checker, concentric squares, big rings, jewel cells, speckle) and
its own pale-gold per-cell ground with tessera seams — dark-on-pale, the
inverse of the base emphasis, so a revealed patch reads as a different
surface. S8 picks the edge: clean per-pixel cut, or whole-cell inclusion
(each overlay cell samples the 16×16-tile luma at its own ORIGIN via a
4-stripe ring buffer — 64 lines, exactly the tallest cell — so cells of any
size are included whole, like quilt's luma mod). Contour/halo/flesh-restore
are gated off in Luma mode; P12 fades base+overlay together.

## Status

v0.6.0: all six rev_b configs close timing at full clock post-route
(HD 74.25 MHz Fmin, SD 27 MHz): Fmax 83.2–88.7 MHz, five configs on seed 1
(hd_analog seed 5). ~5925 / 7680 LC (77%), 6 EBR (contour line buffer +
4-stripe luma-tile ring), C_LATENCY 13. Chroma is swapped at the I/O
boundary for this hardware (see Palette). Timing lessons from the luma-mod
reworks: start EBR-read consumer chains one op per stage (EBR clk-to-q is
2.15 ns); keep per-pixel compares/adders (`s_aline > 420`, grain adds) out
of the E8 compose mux chain by registering flags / folding into E7.
**Not yet hardware-tested.** Defaults on load put a face pointed at the
camera straight into a Klimt portrait with no knob-turning; presets
"Luma Figure" (clean cut) and "Luma Cells" (whole cells) demo the overlay.
