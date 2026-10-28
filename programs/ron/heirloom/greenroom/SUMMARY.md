# Greenroom — the TV in the green room: a painted collage with the camera on its screen (Heirloom #54)

**v0.5.0 (2026-10-28): rebuilt from the photo, built, NOT flashed.** Source: `photos/television.jpg`, one of Ron's painted
paper collages (4786 × 5993). v0.1.0 was a triangle-octave paint field with a rounded TV window. It read as abstract.
This version rebuilds the collage itself:
- a fitted layout map of the painting, with a texture measured from the photo;
- the TV, the clock and the caption at their measured places;
- the camera on the TV screen.

P12 Wash scrapes the paint off to show the camera underneath, or floods it.
Working files: `.heirloom_work/rb_greenroom/` (`notes.md`, `proto.py`, `imodel.py`, `buildmap.py`, `mkhook.py`,
`sheet.png`, `cmp_final.png`, `cmp_wash.png`, `motion_crop.png`).

## Reading of the photo (measured)
Positions are in page heights. The page is 0.80 wide.
- **The wash.** Green and teal acrylic, washed and scraped over magazine cut-outs.
  - k-means (16 colours, whole frame) is dominated by dark greens: rgb(28,42,35) 14.2 %, (14,62,48) 12.5 %,
    (39,62,44) 11.9 %, (15,86,47) 10.1 %.
  - Also: teal (13,79,78) 6.9 %, olive and ochre (133,113,66) 5.6 %, orange (241,103,59) 2.4 %, cream
    (236,215,175) 0.7 %.
  - Texture (luma 0..255): band-pass std hf (<5 px) 9.5, mf (5–17 px) 9.9, lf (17–49 px) 10.5. Chroma: Cb 3.6 / 3.6,
    Cr 5.5 / 6.0.
  - Strokes run near-vertical, leaning "\" (structure-tensor peak at 75–90° from +x, y down; coherence 0.45).
  - The local std is about 0.18 × the colour's luma, so the texture is multiplicative.
- **The TV**, the one clean picture:
  - rounded screen x 0.282–0.480, y 0.392–0.531, corner radius 0.018;
  - pale-green control panel x 0.498–0.539, y 0.375–0.550, with two dials (x 0.518, y 0.401 / 0.517, r 0.014), each
    a dark ring, pale centre and diagonal bar;
  - grille x 0.539–0.591, lavender-grey in its lower right part;
  - dark cabinet with two light lines at y 0.552 and 0.564.
  - The screen shows a woman's face cut-out (blonde curls, white collar) on teal.
- **The caption** "…cture yourself on this pa…": dark sans letters 0.017 tall on a pale yellow-green strip,
  x 0.295–0.550, y 0.612–0.641.
- **The clock**, top left: centre (0.152, 0.281), Roman-numeral ring r 0.036–0.056, a small face photo in the centre
  (r 0.031), dark teal rim.
- **Around it:**
  - an orange blob with a cream top edge (0–0.19 wide, 0.09–0.28 down);
  - the onion vase (ellipse centre (0.400, 0.209), 0.122 × 0.081) under a dark funnel with pink scraped streaks, light
    green fans on either side;
  - pink hands over the TV;
  - a navy wedge and a paper strip, top right;
  - the suit with clasped hands and a red label, right;
  - the blue scraped mountain with orange flecks, left;
  - the child's face with yellow spirals, bottom left;
  - orange splatters, bottom centre;
  - dark teal scraped over cream flecks, bottom right.

## 16:9 mapping
- Page height equals frame height, and the page (864 virtual px wide) is centred.
- The sides continue as a quiet dark wash: the darkest four colours of the neighbouring tile, chosen by the stroke
  hash. The tile seams are jittered.
- Geometry is in virtual 1080-line units from the timing ID, so 720p renders the same picture.
- On SD 4:3 the page is the same and the sides are narrower (288 virtual px instead of 528). The clock and dials stay
  round.

## What it does
1. **Paint.**
   - The layout map holds 96 × 120 cells of 9 virtual px (12 EBR).
   - Each cell is an index into 16 colours fitted per tile (4 × 3 tiles) on the photo's cell means. The palette is
     Y6 U5 V5, with luma pre-boosted 16/13 and a knee so the shell's 13/16 output scale returns the photo's tones.
   - The map is read through a warp: three triangle waves per axis (four 12-bit phase accumulators, plus the sums
     f1 + 2f2 and f3 + 2f4), plus a stroke-hash jitter. This gives organic, crisp, scraped edges.
2. **Texture.**
   - T = grain + 7/8 S1 + 7/8 S2 + (SX + SY)/4, where:
     - the grain is a per-pixel hash;
     - S1 and S2 are stroke streaks hashed over 4 × 16 and 8 × 32 skewed cells, leaning 3/16;
     - SX + SY is a smooth blotch.
   - It is applied multiplicatively to luma: Y + (Y − 64) · T · gain. Chroma follows the luma change (Cr + dY/8,
     Cb − 3dY/16).
   - Check against the photo (integer model, 4 × 4 grid): region means |dY| 1.7, |dCb| 0.7, |dCr| 0.8 (8-bit);
     band-pass std ratios hf 0.91, mf 1.01.
3. **Elements on top:**
   - clock numerals: 12 radial marks in the 39–60 px ring;
   - the TV: screen with rim, panel with three-radius dials and a bar, grille with pixel lines, cabinet lines;
   - the caption: a 256 × 16 one-bit glyph strip of the fitted font, magnified to 278 × 18.
4. **Camera.**
   - The camera's central 68 % × 85 % is written into a 128 × 64 store (4 banks, 4-bit dithered luma, 2ⁿ-pixel box
     columns, one line per row) with 32 × 16 U4V4 chroma.
   - The store is shown bilinear on the TV screen, and teal-tinted in the clock centre.
5. **Wash key** (P12): key = colour depth (from palette luma) + noise − threshold.
   - Scrape: the live camera shows through, with a cream edge band at the scrape line.
   - Flood: deep wash green covers the paint.
6. **Output.** Shell conditioning (luma [1 2 1] and toward ~777, chroma [1 2 2 2 2 2 2 2 1]/16), then the blanking
   gate on the generated avid.

## Controls
| Control | Photo element | Function |
|---|---|---|
| **P12 Wash** | the green wash | 0 = scraped: the paint is gone (lightest first, cream scrape edges) and the live camera fills the frame under the TV and caption. 50 % = the photo. 100 % = flooded deep green; only the TV and caption float. Glided |
| K1 Orange | the orange paint (blob, splatters) | 0 = the orange dissolves into the wash (noise key); 50 % = the photo; 100 % = orange drips over up to 24 % of the stroke cells |
| K2 Brush | the scraped paint edges | warp 0 (map cells as a mosaic) .. 7/4 cells (the photo ≈ 1 cell plus stroke jitter). Glided |
| K3 Grain | canvas grain, brush streaks, blotches | texture 0 (flat paint) .. about 2× the photo. Glided |
| K4 Tuning | the TV picture | 0 = locked (the photo); up: vertical roll ∝ k² (to about 4 store rows per field) and snow replacing up to 15/16 of the picture |
| K5 Speed | the paint flow | bipolar, still within ±16 of centre. The four waves drift at +1, −2, −1, +2 × Speed/16 (12-bit phase per field) |
| K6 Tint | the wash hue | bipolar, ±8 deadband: yellow-green .. the photo .. teal (not on the orange) |
| S7 Crawl | the caption | Off / On: the strip scrolls one glyph column per field |
| S8 Screen | the TV picture | Colour / Mono |
| S9 Clock | the photo in the clock centre | Camera (teal-tinted) / Paint |
| S10 Lines | the TV picture | Off / On: scanlines |
| S11 Source | what is on the TV | Video (default) / Osc |

Defaults are K1–K3, K6 and P12 at 512, K4 at 0, K5 at 614 (slow flow), switches off. The control sheet is
`.heirloom_work/rb_greenroom/sheet.png` (from the proto). Every control visibly changes the picture.

## Video
- **Camera content.** The camera is printed where the photo has camera-like content:
  - clean and bright on the TV screen, where the woman's face is;
  - teal-tinted in the clock centre, where the small face photo is;
  - full-frame and live under the paint when P12 scrapes it off.
- **Osc** substitutes a generated TV portrait in the same paths (store, clock and reveal):
  - a teal screen with a rolling hum bar;
  - blonde octagon hair with stripes and a skin-coloured octagon face;
  - eyes that blink, a red mouth, a white collar;
  - a head sway of about 8 s.
- Every control acts in both modes.

## Engine
- **Per field.** The field starts at the vblank zone (no avid for 1024 clocks).
  - Glides run (P12, K2, K3: 1/8 per field), switches and K1/K6 are latched, and derived values settle.
  - DDA starts and steps come from per-mode constant tables, indexed by a class from the timing ID.
- **Per line.** State advances at the generated avid fall: vy (Q.8), the map row and the store and glyph rows
  (Q.12), the wave line phases (Q.4), and the hash row and column Weyl starts.
  - The update is pipelined over three clocks, and the row values over four registers.
  - One square table is sequenced for the per-line terms: screen-corner ey², dial dy², clock dy².
- **Pixel path, 28 stages.**
  - 1: triangles and hash sums.
  - 2–4: warp sums and products, three hash rounds.
  - 5: jx and jy, texture terms.
  - 6: warped cell coordinates.
  - 7: cells and tile.
  - 8: map address and texture factor.
  - 9: map EBR.
  - 10: palette address.
  - 11: palette EBR, with its fabric copy.
  - 13: palette expand and key partials.
  - 14: keys.
  - 15: colour index to a colour ROM.
  - 16–17: texture.
  - 18: camera override.
  - 19–27: conditioning.
  - 28: gate.
- **Element beam and store reader.** These start at stage 13.
  - Element shapes come from 5- and 6-bit square tables.
  - Constant compares are built from gates, so they map to LUTs rather than carry chains.
- **Source side.** The Osc generator and the store writer.
- **EBR (25):** map 12, palette 1, glyphs 1, store luma 8, store chroma 1, sync/avid ring 1, chroma-filter ring 1.
- **Contracts:** generated avid; one 28-clock latency in every mode (`processing_delay_clks` = 28); colours in the
  hardware convention (u = Cr).

## Status
- **v0.5.0 built 2026-10-28, ROUTER1 (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh greenroom`), 6/6 on
  seed 1, not flashed.**
  - Post-route Fmax: HD Analog 81.35, HD HDMI 81.35, HD Dual 82.24, SD Analog 75.37, SD HDMI 83.15, SD Dual
    81.40 MHz.
  - Fit: 7284–7298 / 7680 LC (95 %, over the 92 % guideline but with about 7 MHz of HD margin), 25/32 EBR.
  - All six bitstreams pass iceunpack.
- **Router.** Rebuild with router1. A router2 probe gave HD Analog only 74.6 MHz, and one router2 run stalled.
- **Fit history.**
  - The first RTL used a microcoded engine (per-field DDA starts, per-line square-root half-widths), side maps and an
    accumulator store. Its GHDL render matched the integer model, but it needed 10,816 LC.
  - Removing the engine, building constant compares from gates, adding EBR delay rings, single-line store sampling
    and a cheaper Osc brought it to 7.3 k.
- **GHDL renders** (720p60, decimation 1, raw2hw, Osc and Video):
  - They match the integer model: mean difference about 6.5 after a uniform 4-px shift.
  - Scrape and flood renders behave as designed.
  - A later frame (Speed at maximum) shows the paint edges flowing and the Osc head swaying and blinking.
- SD and interlaced modes were not simulated.

## Deviations from the photo
- **Soft cut-outs.** With 9-px cells and 16 colours per tile, the cut-outs (hands, suit, child, onion, funnel) become
  painted tones and silhouettes with warped edges. No photographic likenesses are stored. Crisp outlines exist only
  for the TV, the clock numerals and the caption.
- **Not drawn** (budget): the onion's thin outline, the child's yellow spirals, the clock's numeral glyphs (12 radial
  marks instead), the "TAN" strip text, the red label and the bottom letters.
- **The camera picture** is a 128 × 64 store with 4-bit dithered luma and single-line vertical sampling (fine
  horizontal detail can alias). Chroma is one sample per 32 × 16 cell. It is softer than the photo's crisp magazine
  face.
- **The Osc portrait** is a stylised octagon face, not a likeness.
- **The revealed camera** is offset about 25 px to the left by the pipeline.
- **The sides** are an invented, static dark-wash continuation of a portrait collage.
- **Whites** are capped at about 777 by the shell's output scale (the HD HDMI bar fix).
- **Motion.** The photo is still. The paint flows slowly and the TV picture is live.
- **S10.** The proto design had S10 Flip (the collage mirrored); it was replaced by Lines to fit.
