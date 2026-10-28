# Abacus — rows of glossy beads on their rods

**v0.5.0 (2026-10-27): built, NOT flashed.** Rebuilt from the photo WIN_20230605_01_03_50 (Heirloom #42):
rows of red beads between wavy blue/green bands, one bead every 188 px, the rows mirrored about the bead
rows. The beads change across the frame: dark maroon with green lenses and navy "eyes" on the left, bright
red ovals in the middle, pink-cored hourglass beads on the right. The band pinches shut into a red dot and a
yellow "Y" spark in the middle. Every bead carries cream/white glints on its rims. Presets are hand-written
placeholders.

## Reading of the photo
Measured on the frame and on a tile averaged over its 4 half-period copies (they agree to 6.1 RGB levels).
- **Vertical.** Flip-correlation puts a mirror at y = 554 (mean |diff| 7.8) and shift-correlation gives a
  period of 495 lines (8.5). So bead rows sit on lines 59 / 554 / 1049 and band centres on 306.5 / 801.5
  (2.18 periods per frame). d = distance from the bead row, 0 .. 247.5.
- **Horizontal.** There is no global repeat or mirror (residual 31–35).
  - The band-centre green is a clean cosine (rms 4.3; a triangle fits at 5.4). It peaks at x = 161, 348, 533,
    723, 913, 1100, 1289, 1473, 1667, 1853: period 188.0, 10.2 beads across. It rides a slow bowl, with
    G = 38.6 cos + 180 − 481u + 658u² − 163u³ (minimum 82 at u ≈ 0.45).
  - Bead centres sit on the cosine's troughs (254 + 188k), the gaps on its peaks.
  - In the band, blue is a window on green: B = 255 for G in 73..110, falling to 0 by G = 155.
- **Across the frame (trough columns k = 0..8).**
  - The bead-centre red goes 99, 190, 211, 248, 254, 248, …, pink 189 (+G/B 42), 97: past the red peak on
    the right, which makes the hourglass and X beads.
  - Bead depth goes 65 → 150 lines.
  - The red band-edge line sits at d = 170–205 (left), 120–240 (the pinch at x ≈ 800: red dot, Y spark) and
    165–180 (right).
  - Gap colour runs pinkish-brown, maroon, salmon (e27e66 at x ≈ 912–1100), maroon, olive-pink.
- **Left third.** Between bead and band lie an extra green layer (green diamonds at the gaps, d ≈ 100), navy
  eyes at the troughs (d ≈ 165; orange→grey arc, maroon interior) and red/pink bow-ties.
- **Gloss.** Glints are bright warm/neutral pixels: cream arcs on the bead tops and bottoms, cream streaks
  between the middle beads, orange and yellow sparks, grey band-edge glints.
  - They make up 4.9 % of the frame, 1–5 px wide.
  - The outlines change colour along a contour (orange on the left side, grey/violet on the right).
- **No single analog patch fits.** Additive and product separability fails (residual variance 0.42 of
  0.61, where a synthetic additive field gives 0.03). Palette purity of every amplitude-modulated
  vertical × 188-px oscillator family stays poor, per window, jointly, per row, per column and per
  channel. So the field is measured rather than modelled: one scalar field s(x, d) through one palette (the
  mandala/blossom knot-grid mechanism), plus a measured gloss layer.

## What it does
1. **Field.** s(x, d) is a bilinear knot grid of 32 rows (every 8 lines) × 129 columns (every 15 virtual px),
   fitted to a per-pixel solution of the photo.
   - Method: k-means(48) of the tile, spectral seriation of the clusters into a 1-D colour order, a smoothed
     per-pixel solve, then an alternating knot least-squares fit and palette re-learn.
   - The knot field departs from the free field by rms 0.035 (of 1).
   - The horizontal knot column comes from a triangle of the horizontal phase: the photo, then its mirror
     image, repeating.
2. **Controls on the field.** s' = 512 + Rims·(s − 512) + Swell + Etch·(luma − 110). The index is s'/2 into a
   512-entry palette: bright red … maroon … pink/cream … olive … navy … red line … violet … green … blue.
   The band's green diamonds are lifted to the photo's saturation.
3. **Gloss layer** (measured column by column, every 7.5 virtual px, from one tile copy of the photo).
   - Glint and fill pixels are classed into cream, pink, orange, yellow, grey, navy, green and amber where they
     differ from the base colour. Navy, green and amber are allowed only in the left third.
   - Runs are tracked by continuity into 2 slots per column: centre, half-width class (1.5 … 40 lines),
     amplitude (4 levels), colour, and a jump bit.
   - Per pixel, the centre and amplitude are lerped between neighbouring columns (the centre is held at a
     jump). Alpha = amplitude × clamp((w − |d − centre|) / 2 lines). The stronger slot wins, and its colour is
     blended over the base in Y/U/V.
   - Gating: glints only on beads and rims (s' < 768), fills where s' < 960. So at Swell 0 the blue/green
     sea has no glints.
4. **Output.** Palette Swap after the blend, then output conditioning and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Beads | beads across: horizontal zoom 0.5–2.5× (centre = the photo's 10.2); past the measured picture it continues mirrored |
| K2 Rows | bead rows per frame height: vertical zoom 0.5–2.5× (centre = the photo's 2.18 periods) |
| K3 Rims | the dark rims round the beads and the band-edge lines: field gain 0.5–2.5× about the rim level; low = fat soft rims and grey bands, high = thin hard rims, flat red beads and flat blue bands |
| K4 Slide | the beads slide along their rods: bipolar pan, still within ±8 of centre; default 640 ≈ 0.8 px/field (≈ 49 px/s) leftward; maximum 14.5 virtual px/field |
| K5 Roll | the rows roll up/down: bipolar vertical drift, still at the default centre |
| K6 Etch | the picture's luma pushed into the field: bright areas push the contours towards the band colours, dark areas swell the beads (160 = default) |
| S7 Mirror | left/right mirror about the centre: the left half (lenses, eyes, dark beads into bright beads) on both sides |
| S8 Palette | Photo / Swap (U and V exchanged: blue beads in red/magenta bands) |
| S9 Stack | Mirror (the photo's mirrored rows) / Saw (rows repeat and are cut flat at each band centre) |
| S10 Flip | beads and bands exchange places (s → 1 − s): red bands, blue beads |
| S11 Source | Video / Osc |
| **P12 Swell** | bead size (palette offset ±0.25): 0 % = small pips in a blue/green sea, 50 % = the photo, 100 % = the beads merge into red bars |

Glides (1/8 of the gap per field) act on K1, K2, K3, K6 and P12.

## Video
- **How the camera enters.** Its luma (line buffer, (y >> 2) − 16, clipped) minus 110, scaled by Etch, is
  added to the field before the palette. The contours (bead outlines, band edges, glint gating) bend around
  the picture while the rows, beads and palette stay. At the default Etch the selfie reads in the band and on
  the beads, and the composition holds (cmp_final.png, bottom).
- **Osc** substitutes the shell oscillator, slow diagonal triangles drifting per field, through the same
  path. Every control stays live.

## Engine
- **Per-field sequencer** (vblank zone: no avid for 1024 clocks). It has one serial multiplier with fixed
  slots and one shared glide. It sets the zoom steps, the start phases, the pan / roll accumulators, the Rims
  gain, the Swell offset and the Etch gain.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line's 129-entry field row: two knot
  rows blended by the line's fraction, written into the other bank of a 2 × 128 row RAM. d is the |triangle|
  of a 495-line oscillator, or a saw for Stack.
- **Pixel path** (latency 24 clocks in every mode).
  - Triangle fold of the horizontal phase gives the knot column and fraction (6 bits), plus the gloss column
    (8 units = 7.5 virtual px) and its fraction.
  - Row RAM, fabric copy, R1 − R0, lerp product, s − 512, Rims product, + Swell, + Etch, clip, Flip, palette
    ROM (512 × 32), fabric copy.
  - In parallel: gloss ROM (256 × 64: 2 slots × 2 columns), decode, the centre/amplitude fraction products,
    d − centre, the soft edge, alpha, slot select, the s' gate, (gloss − base) × alpha / 16.
  - Blend, Palette Swap, padding, output conditioning.
- **EBR (22).** Knot ROM 4096 × 8 (8), palette 512 × 32 (4), gloss table 256 × 64 (4), line buffer 2048 × 8
  (4), row RAM 256 × 32 (2).
- **Contracts.**
  - damier's generated avid (parity-locked).
  - Output conditioning before the blanking gate: chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums, luma [1 2 1],
    both centred on the same pixel, then luma 13/16 (drawn whites ≤ ~775).
  - Colours in the hardware convention (U register = Cr, V = Cb).
- **Resolution.** Geometry is in virtual 1080-line units from the timing ID, so 720p shows the same picture.
  **SD (4:3) shows the central 1440 virtual px: 7.7 beads of the photo's 10.2, the same round shapes.**
  Interlaced modes get no field offset (field_n is tied high on hardware).
- **Build hook.** abacus.py holds the measured tables as integers (knots, palette, gloss runs, gloss colours)
  and writes abacus_pkg.vhd.

## Status
- **v0.5.0 (gloss) built 2026-10-27, router2 (the builder default), NOT flashed.**
- **Seeds:** HD Analog = seed 5, HD HDMI = seed 2, HD Dual = seed 6; the three SD configs are seed 1.
- **Fmax (post-route, last line):** HD Analog 75.93, HD HDMI 74.54 (thin), HD Dual 75.85, SD Analog 74.39,
  SD HDMI 73.92, SD Dual 72.17 MHz (SD Fmin 27).
- **Resources:** 6369–6406 / 7680 LC (83 %), 22 / 32 EBR. Every bitstream passed iceunpack.
  out/rev_b/ron/heirloom/abacus.vmprog (454 301 B).
- **If HD timing regresses:** rebuild with `NEXTPNR_ROUTER="--router router1"` before restructuring.
- **Verification** (GHDL 720p, decimation 1):
  - The default render matches the integer model imodel.py at mean |ΔRGB| 0.46, 86 % of pixels within ±1,
    at a constant 2-px beam offset.
  - K4 = 1023, fields 3 → 6: pure translation, 29 px (720p) to the left, residual 1.8; the gloss moves with
    the beads.
- **Metrics vs the photo** (4 × 4 region means; local contrast = rms of luma minus its 15 × 15 mean):

  | | region \|ΔRGB\| | contrast ratio | glint |
  |---|---|---|---|
  | v0.5.0 without gloss | 5.4 | 1.22 | 0 % |
  | with gloss | 4.8 | 1.38 | 2.4 % |
  | photo | — | — | 4.9 % |

## Deviations from the photo, and why
- **Glints are stepped and thinner on the ground.** The gloss layer is measured per 7.5 px column with 2 runs
  per column, so arcs come out in small vertical steps and short bars rather than smooth curves. They cover
  2.4 % of the frame where the photo has 4.9 %. One-pixel cream arcs become soft 2–4-line lines. A third slot
  fitted better but overflowed the chip (8106 LC).
- **Left-third eyes.** They read as dark maroon ovals with red rims, amber flecks on the arc and a navy
  surround, not the photo's clean navy eyes with an orange→grey arc. The green lenses are small green blobs.
  The left column carries 0.2 % glints where the photo has 3.4 %.
- **Y spark.** At the pinch (x ≈ 800) it reads as a double red ring with a small yellow centre.
- **Crisper than the photo.** Local contrast is 1.38× the photo's: the knot field's palette steps and the
  gloss edges are harder than the analog original's soft horizontal bandwidth.
- **One tile.** The top and middle bead rows differ slightly in the photo (mirror residual 7.8); both are
  drawn from the averaged tile.
- **Motion is designed.** The photo is a still and no oscillator patch reproduces it. Slide and Roll move the
  measured picture, and its left→right evolution travels with it, continuing in mirror image past the edge.
- **Capture borders.** The capture's dark top/bottom rows and left edge are not drawn.

## Working files
`.heirloom_work/rb_abacus/`:
- `notes.md`: measurement log, including every fit that failed.
- `T.npy`: the averaged tile.
- `kalt8.py`, `ka_K_h15.npy`, `ka_P_h15.npy`: knot-grid field and palette.
- `gcol4.py`, `gc4_cfg4.json`, `gc4d.npz`: gloss runs (2 slots, as built).
- `protogloss.sh`: renders `proto_final.png`.
- `imodel.py`: integer model of the RTL.
- `gen_hook.py`, `hook_template.py`: write `abacus.py`.
- `metrics.py`, `metrics_after.txt`: before/after metrics.
- `sheet.png`, `crops_gloss.png`, `cmp_proto.png`, `cmp_final.png`.
- `v1_renders/`: v0.5.0 without gloss.
