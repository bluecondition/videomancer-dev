# Crossbeam — a glowing cross beam keyed over torn colour bars

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Photo: `photos/WIN_20230527_01_43_26_Pro.jpg`.
The v0.1 version, "soft cyan X over hashed Mondrian blocks", was judged too abstract.
The picture is three layers:
- a grid of colour bars;
- a mirrored contour field (red/black zebra stripes and green→red ramps) seen through the bars' "transparent" cells;
- a glowing cyan cross beam, luma-keyed over both.
Brighter bars push into the beam (the bullet-shaped side wedges, the yellow sliver in the core) and a dark keyer outline runs along every cut.
Presets are hand-written placeholders.

## Reading of the photo
Measured in `.heirloom_work/rb_crossbeam/` (`m1`–`m14.py`, `st.py`, `integ2.py`, `cmaps.py`, `fitx5.py`, `cellkeys.py`; `notes.md`).
- **Palette** (k-means, 14 colours): black 17 %, maroon 11 %, olive 10 %, cyan core (0,219,222) 9 %, dark teal 9 %, teal 8 %, red (196,20,12) 7 %, magenta (249,8,207) 7 %, yellow 5 %, white 4 %, green 2 %, blue 2 %.
- **Colour bars.** The bars are translational. The mirror-symmetric parts (beam and zebra) sit on top of them.
  - Rows: boundaries at y = 2.9 + 39.04 k (residual < 1 line), 28 rows per frame.
  - Columns: the translation correlation peaks at 612 px. Each period is three columns A / B / C = 305 / 230 / 77 px, with A starting at x = 117 + 612 k.
  - Torn rows: rows 6–9 and 22–25 are shifted +376 px (comb-phase fit 493–504 against 117).
  - Colours: the cell colours (magenta, yellow, white, blue, navy, olive, maroon, black, cyan; photo medians) form a near-16-row cycle with exceptions. They are stored as a measured 28 × 9 table (3 periods). The table also marks ramp cells, zebra cells and one "ramp over white" cell type: the white strip in row 11.
- **Beam.** The beam is cyan (G ≈ B, R ≈ 0), peaking at (0,240,247) and fading to dark teal.
  - Profile: fitted as G = A(|dx|) · (1 − |t|)^p with t = (dy² − s² dx²) / c². That is a hyperbolic X with A0 250.8, a 882.5, falloff exponent 2.17, centre (984.7, 466.8), s 0.232, c 191.4, p 0.62 between the arms and 0.68 outside.
  - Repeat: the beam repeats vertically every 731 lines, which is why the next cross's arms show at the bottom edge.
  - Keying: the beam wins where G exceeds a key level for the colour underneath. YEL 221 > MAG 158 > OLV 132 > NAV 102 > BLU 80 > MAR 72 > BLK 38; white never loses. Where a cut crosses a cell, the key is refined per cell. The fit classifies 96.3 % of pixels correctly.
- **Zebra.** Red stripes (R up to 245) on black, pitch 16–45 px (median 25).
  - They are a wavefolder on a smooth field, mirror-symmetric about x = 984.5 (the left half overlays the flipped right half).
  - Near the beam the contours run parallel to the arm edges. In the corners they wrap round blob-like plateaus with rounded right-angle bends. They are torn row by row.
  - The field was reconstructed from the stripes' local orientation and spacing: structure tensor, sign flood, zero gradient on the plateaus, weighted least-squares integration.
- **Ramp cells.** Smooth green → ochre → red with R and G varying with position and mirror-consistent: x = 700 and x = 1270 at y 120–160 are both (148,103,0).
- **Mixing.**
  - Zebra and beam meet in a per-channel max: salmon lines on teal (the photo's "pink" cluster (153,88,95)).
  - Ramps and bars are keyed with a sharp cut and a dark outline.
  - Horizontal edges are soft (~4 px); row edges are sharp.

## What it does
1. **Bars.**
   - A row accumulator in virtual 1080-line units, with K5 roll, gives row r and the half-row flag.
   - A block-x DDA (Q.6, K6 scale, wrapped at 3 periods) gives column j.
   - The cell ROM at r·16 + j holds the colour code and key code.
   - Rows 6–9 and 22–25 start 376 block px later when Tear is on.
2. **Contour field.** Its coordinate is the folded, torn u = |x − 984.5| + tear(r) + jitter.
   - Zebra phase: a 64 × 32 table (16 px × 34 lines, 16 levels per stripe). The camera adds up to 1 stripe. Then × K3 gain (mantissa 16–31 × 2^e), + K4 flow, and the stripe profile clip(5 |f − 128| − 192), × the amplitude^0.7.
   - Amplitude and ramp red/green: a 32 × 32 table (32 px × 34 lines).
3. **Beam.** In the log domain: log A(|dx|) + log F(t) + log P12 + ridge + camera bloom, then an exp ROM.
   - t comes from exact squares: a 2nd-order DDA of (s·dx)² per pixel against a serial square of dy per line.
   - A ridge boost along the arms (+6 % within 45 lines) makes the yellow sliver taper.
4. **Composite.**
   - Zebra cells: the beam colour plus the zebra red, a per-channel max.
   - Ramp cells and bars: the beam where G3 > key, otherwise the ramp or bar colour.
   - A dark outline (× 7/16) wherever the key flag changes within ±2 px or from the line above.
5. **Colour.** In YUV per source: a beam colour ROM, shift-add red and green parts, and a YUV palette. Luma is pre-boosted 16/13 and kneed above 724.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Angle | the beam's arms | arm slope s 0.05–0.55; 0.232 = the photo (default 372) |
| K2 Rows | the colour-bar rows | 8–64 rows per frame; 27.7 = the photo (default 359) |
| K3 Contours | the zebra stripes | stripe density 0.25–4×, exponential; 1× = the photo (default 512) |
| K4 Flow | the zebra contours | bipolar, still in the centre ±8, square law, up to ±4 stripes/s. Positive streams the contours one way. Default 680 ≈ 0.4 stripes/s |
| K5 Roll | the colour bars | bipolar, still in the centre ±8, square law, up to ±4 rows/s. Positive rolls the bars up. Default 600 ≈ 0.1 rows/s |
| K6 Columns | the colour-bar columns | 1.5–8 three-column periods across; 3.14 = the photo (default 258) |
| S7 Bars | the colour bars | On (default) / Off: the beam over the zebra field alone |
| S8 Tear | the torn rows | On (default): the 376 px row shifts, the per-row zebra tears and per-field jitter (±1–2 px) |
| S9 Stack | the second cross | Off (the photo: period 731 lines) / On: half the period, three crosses stacked |
| S10 Palette | the colours | Photo / Swap (Cr and Cb exchanged: amber beam, blue zebra, cyan bars) |
| S11 Source | the zebra field and the beam | Video (default) / Osc |
| P12 Beam | the cross beam | log fade: 0 % = off (the bars and zebra alone), 50 % = the photo, 100 % = 2× (the beam swallows everything but white, yellow and the zebra peaks) |

K1, K2, K3, K6 and P12 glide (1/8 of the gap per field, ±4 deadband).

## Video
The photo is synthetic, so the camera is woven into the patch rather than replacing a layer:
- **Contours.** The camera's luma, slewed horizontally like an analog low-pass (s += y − s/8), is added to the zebra phase, up to one stripe. The zebra contours bend around the picture.
- **Bloom.** The same luma blooms the beam (log + up to ×1.37). The cross spreads into the bars where the picture is bright.
- **Black camera.** Video mode with a black camera is exactly the photo.
- **Osc mode** uses the stored contour field (the photo's own) with no camera term. Every control stays live in both modes.

## Engine
- **Per-field sequencer.** One serial shift-add multiplier (24 × 20), 12 slots of 32 clocks in the vblank zone. It computes:
  - the beam's per-pixel s·dx step and start;
  - the row step (per-timing constants), the block step and start;
  - Flow and Roll speeds (square law);
  - SDX0², SDX0·kS and kS² for the square DDA.
  - The K6 scale is a ROM. The K3 mantissa, the P12 log gain and the knob curves are shift-adds.
- **Per-line scalar engine.** Advanced after each source avid rise, it computes the next line:
  - row accumulator (mod 28), beam dy (wrapped at the period), table row, tear (+ LFSR jitter), block start;
  - a 17-step serial square of dy.
  - Line 0 is prepared in the vblank zone.
- **Table engine.** It lerps the next line between two table rows into ping-pong line RAMs: zebra phase as 64 {value, slope} words; amplitude and ramp as 32 entries in even/odd banks. It takes 10 clocks per entry with a serial radix-4 product, 640 clocks per line, which fits SD's 858.
- **Pixel path, 17 stages, then output conditioning.**
  - **Beam:** beam DDAs (virtual x, s·dx, block x, the (s·dx)² DDA); |dy² − (s·dx)²| → t; log-profile and log-falloff ROMs; the log-sum; the exp ROM → G3.
  - **Cells and zebra:** cell ROM; zebra lerp, gain and fold; amplitude and ramp lerps (3-bit fraction).
  - **Keying and colour:** key compare; beam colour ROM; YUV composite; outline with a key-flag line buffer; Cr/Cb swap.
  - **Output stage:** the luma knee is spread over two of the output stage's luma delay registers. Then chroma [1 2 2 2 2 2 2 2 1]/16 from an EBR ring (re-neutralised in vblank), luma [1 2 1], luma 13/16, and the blanking gate on the generated avid.
  - Latency is 24 clocks in every mode. Syncs are delayed in an EBR ring.
- **ROMs** (generated by `crossbeam.py` into `crossbeam_pkg.vhd`):
  - phase 2048 × 8;
  - amplitude/ramp 1024 × 16;
  - cells 512 × 8;
  - log profile 512 × 11;
  - log falloff 256 × 11;
  - exp 1024 × 8;
  - beam colour 256 × 30;
  - K6 scale 256 × 14;
  - small LUTs: amplitude gamma, bloom log, gain mantissa, tears, row shifts, YUV palette.
- **Contracts:**
  - generated avid (parity-locked start);
  - virtual 1080-line geometry from the timing ID;
  - engines started per field and per line;
  - every RAM collision-free with `no_rw_check` (0 emulated transparency);
  - no self-add carries (checked with dupin).

## Status
- **v0.5.0 built 2026-10-27, not flashed. ROUTER1** (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh crossbeam`); every config is seed 1.
  - Fmax (post-route): HD Analog 82.05, HD HDMI 89.79, HD Dual 85.37, SD Analog 76.24, SD HDMI 91.40, SD Dual 85.70 MHz.
  - 6999–7024 / 7680 LC (91 %), 23 / 32 EBR. All bitstreams pass iceunpack. `out/rev_b/ron/heirloom/crossbeam.vmprog` is 488,336 bytes.
- **Fit history.**
  - The first RTL (RGB colour path, per-pixel multipliers) came to 8299 LC.
  - Ablations found the beam multipliers at 840 LUT, RGB → YUV at 580, the zebra multipliers at 540, the d1·d2 product at 400, the lerps at 390 and the engine multipliers at 390.
  - Fixes: a log-domain beam, YUV per source, an exact square DDA, a serial engine, narrower lerps, a sync EBR ring.
  - The luma knee was the critical path (45 → 67 MHz) until it was spread over the luma delay stages.
  - router1 stalled on four self-add carries (3d = d + 2d in the engine), rewritten as 4d − d.
- **Verification.**
  - GHDL 720p60 (decimation 1), Osc, defaults, matches the integer model `imodel7.py` to a mean |diff| of 0.80 (Y) / 0.17 / 0.14, with 0.00 % of pixels > 16 after the 4 px generated-avid offset.
  - Motion test (K4 = K5 = 1023): the frame matches the moving model (flow 0.2 stripe, roll 0.2 rows at field 4) to 0.00 % of pixels > 16, against 28 % for the still model.
  - The Video sim (selfie) shows the beam blooming toward the face and the zebra bent by the picture, with the layout intact.
  - Region-mean error against the photo (4 × 4 grid): proto 6.4, RTL 6.7.
  - SD was not simulated.

## Deviations from the photo
- **Zebra contours.** They are a reconstruction: the stripe orientation, spacing, plateaus and bundle placement match, but individual contours differ.
  - The field is stored at 16 px × 34 lines, so detail finer than about 16 px is smoothed.
  - The per-row zebra tears are a designed table of similar size (±10–40 px) plus jitter, not the photo's exact offsets.
- **White level.** Whites and yellows are compressed to the HD HDMI bar-fix level: output luma 719 / 689 against the photo's ~832 / ~773. Greys, cyan and the mid colours match.
- **Yellow sliver.** It tapers correctly but is thinner: about 18 lines against 33 at its thick end.
- **Wedge outlines.** The cuts are stepped per row as in the photo, but the noses sit within about ±10–20 px of the photo's. The beam is an analytic hyperbolic X with per-cell keys, at 96 % fit accuracy.
- **Outline.** It is one-sided vertically (the line above only).
- **Amplitude and ramp resolution.** The zebra amplitude envelope and the ramp colours are tabled at 32 px × 34 lines (5–6 bits).
- **Palette Swap.** It exchanges Cr and Cb, a hue reflection, rather than a designed palette.
- **Cross-resolution.** Geometry is in virtual 1080-line units, so 720p and SD frame the same picture. The outline width and the output softness are in output pixels, so they are relatively thicker at SD. SD has not been simulated or flashed.

## Working files
`.heirloom_work/rb_crossbeam/`:
- `ref.jpg` and `notes.md`.
- Measurement and fit scripts:
  - `m1`–`m14.py`;
  - `st.py`, `integ2.py` (zebra field), `cmaps.py` (amplitude and ramp maps);
  - `fitx2`/`fitx5.py` (beam), `cellkeys.py`, `tables.py` and `geom.py` (the cell table).
- `proto.py` → `pfinal.py` (the float proto with controls), `proto_final.png`, `cmp_proto.png`, `sheet.py` → `sheet.png`.
- `hwtables.py` and `hwtables2.py` (the quantised tables); `imodel4`–`imodel7.py` (the integer models; `imodel7` is the shipped RTL); `mkhook.py` (writes `crossbeam.py`).
- `parts/` (the RTL source pieces), `abl/` (ablations), `prof/` (critical-path profiles), `rdiff.py` (RTL against model).
- Renders: `rtl720_osc7.png`, `rtl720_vid9.png`, `rtl720_mot9.png`, `cmp_final.png`.
