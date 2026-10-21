# Escalator — Heirloom #1, rebuilt from the photo

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

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Columns | tile columns across, 2–14 (7.14 = photo) |
| K2 Bands | pink band rows, 2–10 (4.15); band = 27.5 % of a row |
| K3 Blob | blob left edge, bipolar: wide blobs (no grey gap) <-> narrow blobs; centre = photo |
| K4 Flood | slab thickness, thin threads .. green flood (x0.12..x2.5, square law) |
| K5 Speed | ride speed down/up the stairs, still at centre (+-8), up to 10 lines/frame; default ~1 down |
| K6 Bend | source bend of the blob edges (+-32 px) and, more gently, the slab edges |
| S7 Mirror | every other column mirrored: chevron stairs |
| S8 Climb | the staircase climbs instead of descending |
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
  - One 20x18 serial shift-add multiplier, 18 slots, strobes registered from the zone counter.
    The slots compute increments, slant, tread, flood, thickness DDA, sync wedge, speed, blob
    offset and bend.
  - The ride and Osc phases.
- **Per line:** a 13-state engine computes the next line: band/slab phases, drift and corners from
  one 128x16 table EBR, blob edge, sync edge, palette level and fraction, pale flag.
- **Pixel path:** 24 stages, 24-clock latency in all modes; the tile-wrap carry is precomputed one
  pixel ahead. Then the conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma x13/16) and
  the gate on the generated avid.
- **Shell:** v0.2.0 heirloom.
- **ROMs:** 6 x 512x8 colourizer, 1 x 128x16 line table.

## Status
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
