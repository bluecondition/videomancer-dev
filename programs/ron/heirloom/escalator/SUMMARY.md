# Escalator — Heirloom #1, rebuilt from the photo

**v0.6.0 (2026-11-01, phase 2 round 1): built, NOT flashed** (see [Phase 2](#phase-2)).
**v0.5.0 (2026-10-21): built, NOT flashed.** Rebuilt from `photos/WIN_20220629_23_49_20_Pro.jpg`
after Ron judged v0.2.0 too abstract. It now redraws the photo itself: a pastel tile grid of
lavender halves, violet blobs and pink bands with yellow/orange blobs, crossed by green
stair-stepped slabs that ride down from column to column. Work files: `.heirloom_work/rb_escalator/`
(`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`).

## Reading of the photo (measured)
- **Tiles:** 269.0 px period (boundaries at x = 215 + 269k, 7.14 per line). The line starts
  0.2 tile in, so the first column is a partial "sync tile".
- **Bands:** pink bands from y = 116.5 + 260.3k, 71.5 lines tall (27.5 %), 4.15 rows per frame.
  Pale 2–3-line lines on the band edges.
- **Inside a tile** (u in px):
  - u < 128: lavender (169,171,206), or pink (208,166,169) in a band.
  - From the blob edge b(y) to the tile end: a violet blob (188,146,247) fading pale to the right,
    strongly at the top of the frame. In a band: yellow-green -> yellow -> orange (249,140,0); the top
    band runs on into magenta/pink.
  - Between them: grey (177,179,177).
  - b(y) = 127.3 + 70.5 yy - 27.9 yy^2 px; +17 px on band rows; rounded corners r 14/22 px;
    per-tile wander sd 9 px.
- **Slabs:**
  - Bottom edge fit (1554 samples, rms 7.8 lines): 396.2 + 161.33 c + 0.594 min(u,124) - treads,
    with a chain period of 485 lines (1/3 period per column).
  - About 4.8 treads of ~26 px with ~8-line drops (light top edge, dark corner).
  - Thickness 102, 103, 112, 134, 139, 183, 224 lines over columns 0–6, fit 100 - cx + 3.4 cx^2.
    The sync tile has a ~40-line wedge.
- **Slab colours:**
  - Green (0,176,0) over the left half.
  - Dark green -> teal -> blue over the blob: blue (0,80,245) on band rows, bright green near the top.
  - Blue glow within 12–32 lines of the bottom edge, a cyan (72,225,223) rim, and a 14-px cyan lead
    strip into the next column.
- **Sync tile, lower half:** magenta blobs with chevron band blobs.
- **Mechanism:**
  - A line-locked tile ramp and a frame-locked band ramp.
  - Keys on the tile ramp feed a colourizer.
  - A window comparator on a slow V ramp minus an H staircase (column steps + clipped ramp + tread
    saw) switches the colourizer bank. Its phase rides, so the slabs move down the stairs.

## What it does
- A 24-bit tile phase sets the columns, the regions and the blob distance.
- Per line: a 16-bit band phase gives the band, corners and drift.
- A 16-bit slab phase (65536 = 485 lines) builds the staircase: column steps, slant DDA, tread
  saw.
- Thickness comes from a 2nd-order DDA across x, latched per column.
- The colourizer ROM pair (2 banks x 2 band states x 8 levels x 32 d bins) is fitted to the
  photo's medians. Even/odd levels are interpolated in quarter steps.
- Then the glow, rims, treads, lead strip and pale band lines.
- Luma is pre-boosted 16/13 so the output luma scale lands the photo's levels.

## Controls (v0.6.0)
| Control | Photo element it drives |
|---|---|
| K1 Tiles | the pastel grid's columns only, 2–20 (square law; 7.14 = photo at 547). The stairs keep the photo's pitch on their own phase, so the grid detunes under them and the slab colours (green over the lavender half, teal/blue over the blob) follow the tiles each slab crosses. Locked to the stairs at 547 |
| K2 Bands | bipolar, centre = photo (4.15 bands, 27.5 % tall). Right: more, thinner bands, to 11.9 at 6.8 % (hairlines with their pale edges). Left: fewer, taller bands, to 1.57 at 84 % (the rows flood pink, lavender left as thin strips) |
| K3 Blob | blob left edge, bipolar: left = wide blobs (no grey gap), -1/4 tile; right = narrower blobs that vanish, +1/2 tile; centre = photo |
| K4 Flood | slab thickness, thin threads .. green flood (x0.12..x2.5, square law) |
| K5 Speed | ride speed down/up the stairs, still at centre (+-8), up to ~24 lines/frame (square law; default 623 = ~1 line/frame down). Each column's slab also bobs: a unipolar triangle wave 3.5 columns long travels across the columns, depth 0..236 lines peak-to-peak (square law with speed: ~9 lines at the default), phase rate speed/1024 (33 s a cycle at the default, 1.35 s at full) |
| K6 Bend | source bend of the blob edges and, an eighth as much, the slab edges; cubic law: the default 492 = v0.5's photo bend, 100 % = 4.5x v0.5's maximum (blob edges swing ~1 tile on full-swing video) |
| S7 Mirror | every other column mirrored: chevron stairs (tile grid and stairs each on their own column parity) |
| S8 Ramp | Stairs / Ramp: the slant runs the whole column with no step, so the slabs become continuous diagonal saw ribbons with ~10 treads; with S7, every other ribbon flips its slant (chevrons) |
| S9 Invert | key invert: green/blue bank outside the slabs, the pastel grid inside |
| S10 Vivid | every blob in the band's yellow/orange/magenta palette |
| S11 Source | Video (default) / Osc |
| P12 Rise | escalator step (drop + slant + treads): 0 = level green bars, 66 % = photo, 100 % = half-period zig-zag cascade |

## Video
Camera luma is added to the blob comparator, so the blob edges bend around the picture. A quarter of
that bend goes into the slab window, so the stairs ripple. The grid, bands and stairs stay, so the
composition reads at the default Bend. Osc mode: two slow drifting triangle waves into the same path
(the photo's +-9 px organic wander, moving). Bend is live in both modes. The video tap is luma only,
into geometry, never to the output.

## Engine
- **Per frame:**
  - 5 glides (K1–K4, P12; 1/8 per field, +-4 deadband), pipelined over 20 zone clocks.
  - One 20x18 serial shift-add multiplier, 33 slots of 32 clocks (11-bit zone counter), strobes
    registered from the zone counter. Slots 1–24: tile and slab increments, slant, tread, flood,
    thickness DDA, sync wedge, speed, blob offset, bend (K6 cubic), K1 square, K2 band count and
    height. Chained slots (3->4, 9->10, 18->19, 20->21) load straight from the product register.
    Slots 25–33 write the 9-entry column table (one EBR): entry k = (k - 1) R + bob(k).
  - The ride, column-bob and Osc phases. The line-0 precompute starts at zone clock 1100, after the
    last slot.
- **Per line:** a 13-state engine computes the next line: band/slab phases, drift and corners from
  one 128x16 table EBR, blob edge, sync edge, palette level and fraction, pale flag.
- **Beam (v0.6):** two 24-bit phases, each with its wrap carry precomputed one pixel ahead: the tile
  phase (K1, the grid's regions, mirror parity, sync tile) and the slab phase (fixed 7.14 columns:
  staircase, slant, treads, thickness latch, lead strip, sync wedge). At each slab wrap the column
  base loads from the prefetched table entry (address 0 through the blanking, next column during
  the line).
- **Pixel path:** 24 stages, 24-clock latency in all modes. Then the conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma x13/16) and
  the gate on the generated avid.
- **Shell:** v0.2.0 heirloom.
- **ROMs / RAM:** 6 x 512x8 colourizer, 1 x 128x16 line table, 1 x column table (16 x 16 in one EBR, `no_rw_check`).

## Status
**v0.6.0 built 2026-11-01, NOT flashed.** 7226–7252/7680 LC (94 %), 8/32 EBR, router2. Fmax post-route:
| Config | Seed | Fmax | HDMI HSYNC − clk / closest data bit |
|---|---|---|---|
| HD Analog | s19 (hand-installed) | 81.82 MHz | +1.64 / ≤ −0.57 ns |
| HD HDMI | s45 (hand-installed) | 77.76 MHz | +2.30 / ≤ −0.91 ns |
| HD Dual | s10 (hand-installed) | 88.07 MHz | −2.28 / ≤ −1.27 ns |
| SD Analog | s1 (builder) | 77.92 MHz | |
| SD HDMI | s1 (builder) | 75.82 MHz | |
| SD Dual | s2 (builder) | 84.13 MHz | |

The builder's HD seeds were unsafe for HDMI pin skew: HD HDMI s1 +0.47, HD Analog s1 +0.07. A router2 screen
(HD HDMI s2–48, HD Analog / Dual s1–24; `.heirloom_work/rb_escalator/p2/skew/screen.txt`) picked the seeds
above. The most positive HD HDMI HSYNC with data ≤ −0.6 won (beacon's HW-confirmed fix sat at +2.31).
Fallback HD HDMI bins are in `skew/out/`: s18 (+1.69 / ≤ −0.99, 81.35 MHz) and s8 (+1.46 / ≤ −1.21, 86.17 MHz).
The builder's own vmprog is `skew/escalator_v060_builder.vmprog`. A plain `./build_heirloom.sh escalator` re-picks seed 1
for HD, so after a rebuild re-install the three HD bins with
`cp skew/out/<cfg>_router2_s<N>.bin build/programs/ron/escalator/rev_b/bitstreams/<cfg>.bin` and repack
(`vmprog_pack.py --no-sign --hardware rev_b --toml-path escalator.toml build/programs/ron/escalator/rev_b out/...`).
iceunpack OK on all six. Presets are placeholders.

v0.5.0 built 2026-10-21, router2, all configs seed 1. Fmax post-route: HD Analog 83.25, HD HDMI 79.37,
HD Dual 86.42, SD Analog 85.17, SD HDMI 73.90, SD Dual 82.20 MHz. 6916–6927/7680 LC (90 %), 7/32 EBR.
iceunpack OK. NOT flashed; presets are placeholders.

## Deviations from the photo, and why
- Blob-edge wander is statistical (Osc), not the photo's exact bulges.
- The photo's per-column band-1 hue jumps (one fully magenta band blob) are not reproduced.
- The sync tile's lower half is simplified (straight onset at line 530).
- The slab's internal blue is a band term plus edge glows, not the photo's traced soft ellipses.
- Not reproduced: the cyan wedges where slabs are cut by the frame top, ringing above moving slab
  tops, composite dot-crawl.
- Thickness follows a smooth quadratic (photo scatter +-15 lines).
- At SD (4:3) the 7.14 columns span a narrower picture: tiles 25 % narrower, slants steeper. Pale
  band lines are 1 line at SD/1080i.
- v0.6: at SD and 720p the tile and slab counts are the same as at 1080 (count-based), so the
  4:3 SD crop of v0.5 still applies; K2's hairline bands are 1 line thinner at SD/1080i. The column
  bob and band heights are in virtual 1080-line units, so they look the same in every mode.

## Phase 2

### Round 1 — v0.6.0 (2026-11-01)
Ron left no notes on v0.5.0, which has never been flashed. He picked a proposed map (all four
recommended options):
- **K1 Columns -> Tiles (grid only).** v0.5's K1 scaled the grid and the stairs together, a global
  X scale. Now the tile ramp (pastel grid) and the slab staircase run on separate phases. K1 sets
  only the grid (2–20, square law), and the stairs stay at the photo's 7.14 columns. Both phases
  start at the same point and lock at K1 = 547, which is the photo. Away from it the grid detunes
  under the stairs. Each slab switches the colourizer bank for whatever tiles it crosses, so its
  green / teal / blue follows the grid, as the patch's bank switch would.
- **K2 Bands -> count + height (bipolar).** The band count and the band height move together:
  1.57 bands at 84 % .. 4.15 at 27.5 % (photo, centre, deadband) .. 11.9 at 6.8 %.
- **K5 Speed -> unison ride + column bob.** Top speed goes from 10 to ~24 lines/frame; the default
  is now 623 (still ~1 line/frame down). Each column's slab also bobs on a unipolar triangle wave,
  3.5 columns long, that travels across the columns. Its depth rises with the square of speed
  (~9 lines peak-to-peak at the default, 236 at full) and its phase moves at speed/1024, so it is
  still at the centre and runs in the ride's direction.
- **S8 Climb -> Ramp.** Stairs / Ramp: the slant runs the whole column (no UE clip, a mirrored
  column starts at R) with ~10 treads, so the slabs become continuous diagonal saw ribbons.
  The staircase no longer climbs.
- Also widened, as proposed:
  - **K3 Blob** goes to +1/2 tile on the narrow side (the blobs vanish); the wide side stays at
    -1/4 tile.
  - **K6 Bend** uses a cubic law. The default moves 480 -> 492 = v0.5's photo bend; 100 % =
    4.5x v0.5's maximum.
- **Default frame:** a GHDL render at 720p (Osc) matches the v0.5 baseline `rtl720_osc.png`, with
  the slabs a few lines lower (ride phase + bob). See `.heirloom_work/rb_escalator/p2/cmp_def.png`.
- **Fit:**
  - The first version packed at 7548 LC (98 %).
  - It came down to 7237 with no visible loss:
    - chained sequencer slots load straight from the product register (no intermediate
      registers);
    - the bob is unipolar on an 8-bit column phase, stepping with the ride;
    - the bend multiply is back to 8x10;
    - the lead strip reads the next column's base straight from the prefetch register;
    - the band-bottom distance needs no complement register.
