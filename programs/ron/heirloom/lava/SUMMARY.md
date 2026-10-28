# Lava — Heirloom #43

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230605_01_14_56_Pro.jpg` (1920×1080 capture, 2023-06-05). Replaces v0.1.0 (lobes, pods and a stripe
curtain), which Ron judged too abstract. Presets are hand-written placeholders.
Working files: `.heirloom_work/rb_lava/` (`notes.md` has every number; `treg.py` warp registration, `mkdata3.py`
tables, `proto2.py` float proto, `imodel.py` integer model, `sheet.png` / `sheet_im.png` control sheets,
`cmp_final.png`).

## Reading of the photo (measured)
- **One oscillator.** The whole picture is ONE period of a horizontal oscillator, period 166.9 px (stripe B peaks
  at x = 114, 271, 435, 603, 770; best right-half fold 169.75; right-half autocorrelation peak 167–173 px on every
  60-line band). Seen through a warped row coordinate t(x, y), every column of the photo is a column of that tile.
  - Registering every column to one shared tile (per-pixel t search, the tile = the mean colour per (phase, t)
    bin) reproduces the photo to MAE 6.3.
  - With the warp held on a 42 × 24-px knot grid it is 11.0.
- **The warp is L-shaped.**
  - On the right (x > 1000) t ≈ y: the rows run flat.
  - Towards the top-left the rows bend up along the diagonal. The highlight row passes (400, 90), (600, 400),
    (700, 630) and (900, 810), then runs flat at y ≈ 860.
  - The rows stretch vertically on the left, so each column becomes a hanging drape whose tip steps down to the right.
  - The lava edge (drape tips) passes (170, 0), (260, 400), (340, 560), (460, 660), (600, 760), (800, 940),
    (940, 1020), then y ≈ 1050 to the right edge.
- **Tile rows, top to bottom (t = right-side y):**
  - worms with peach/white cores at x = 1070, 1234, 1398, 1567, 1741 (wobble ±10 px), t 0–250;
  - a red zigzag rim, t ≈ 300–370;
  - olive diamonds (54, 60, 8) with pink/lavender hearts and violet saddles, t ≈ 430–620;
  - slanted red rims;
  - the white web with an orange highlight row (223, 102, 15), t ≈ 760–880;
  - rimmed red blobs with dark cells, t ≈ 880–990;
  - green / navy / violet / pink scallops, t ≈ 990–1076;
  - then the background.
- **Background stripes.** The same oscillator: B is a triangle 0..223, peaking half a period after the troughs
  (x = 40, 189, 351, 517, 683, 851). R (39..189) lags B by 0.08 period (≈ 14 px), and G stays at 2..12. Identical
  from y = 560 to 1000.
- **Palette** (k = 14 shares):
  - reds (177, 7, 4) 13 %, (146, 9, 3) 13 %, (114, 13, 3) 11 %, (76, 12, 8) 9 %;
  - near-black (39, 11, 14) 12 %;
  - olive (54, 60, 8) 8 %;
  - plum (112, 24, 85) 6 %;
  - violet (160, 9, 213) 6 %;
  - bright red (206, 17, 8) 6 %;
  - rose 4 %, orange 4 %, navy-violet 3 %, mauve 3 %, pink-white (205, 141, 165) 3 %.
  - The colours lie on a near-1-D chain: a cyclic 96-node map fits to median 8.5, against 7.5 for k-means with 96
    nodes. In order: black, green, olive, maroon, dark red, red, bright red, orange, peach, pink, lavender, violet,
    magenta, plum, navy, black.
- **Texture.**
  - There is no line tearing: row-to-row cross-correlation is 0 px on 98 % of lines.
  - The serrated edges on the near-vertical rims are a vertical comb of period 8.5 lines (vertical spectrum peak
    7.8–9.1 lines in every region), ±5–8 px.

## What it does
1. **Phase.** φ = frac((x − 960)·k/166.9 + (960 − 114)/166.9 + drift), on the virtual 1920-px line (Mirror
   reverses it at the centre).
2. **Warp.** A fitted 48 × 42 knot grid (42 virtual px × 27 virtual lines apart) holds (t − y)/8. The per-line
   engine interpolates the line's knot row; the pixel path interpolates across the line, giving t.
3. **Controls on t.** t'' = 80 + Rows·(y − 80) − Flow + Rows·Bend·(t − y) + Melt·source.
4. **Comb.** On the lava only (t < 1060), φ += Ripple · tri(line / 8.5 + crawl).
5. **Tile.**
   - Luma: 24 phase knots × 164 t knots (6.4 lines apart), bilinear.
   - Chroma: 12 × 82 knots, the nearest t knot, linear along φ.
   - Fitted from the registered tile and sharpened ×0.6; luma pre-boosted 16/13.
   - Below the lava edge, the tile's stripe rows draw the violet background (Ground = Black blanks them).
6. **Output.** The shell's conditioning: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16, blanking gate.
   Output luma peaks at 730.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Flow** | The lava sheet's reach: t offset ±416 lines. 0 % = the lava drawn back into a bowl at the top right over the stripes; 50 % = the photo; 100 % = it floods down over the stripes, molten columns at the top. Glided. A 12 s breath (±20 lines) always moves the edge. |
| K1 Columns | The drops and stripes across: oscillator frequency 0.5×–2× (the photo: 11.5 periods across). Glided. |
| K2 Bend | The drapes: how far the rows bend up on the left (0 = a flat lattice of rows across the whole width, 1 = the photo, 1.6 = deep drapes). Glided. |
| K3 Rows | Row density about the top of the sheet, 0.6×–1.6× (worms, diamonds, web and blobs squeeze or stretch). Glided. |
| K4 Speed | Bipolar, still in the centre ±8: the drops and stripes slide sideways (+ = left), up to one drop per 2 s; default +17 % ≈ one drop per 12.6 s. |
| K5 Ripple | The serrated 8.5-line comb on the lava's edges, 0–8 px (default 2.5). Glided. |
| K6 Melt | How far the camera (or Osc) bends the rows: up to ±256 lines from the camera luma (default 20 % ≈ ±48). Glided. |
| S7 Mirror | The left half reflected: twin curtains meeting in the middle. |
| S8 Ground | Stripes (the photo) / Black: the background under the lava. |
| S9 Flip | The sheet rises from the bottom. |
| S10 Palette | Lava / Ice (U and V exchanged: blue lava on green). |
| S11 Source | Video (default) / Osc. |

Glides: 1/8 of the gap per field, ±4 hold.

## Video
- The photo is fully synthetic, so the camera is woven in the analog way: the camera luma (2-px average,
  (Y − 512)/2) × Melt is added to the row coordinate. The rows bend around the picture, and bright areas push the
  lava down. At the defaults the composition, rows, drapes and stripes still read.
- **Osc:** two slow drifting triangle waves (x and y) at a quarter of the video's swing go through the same path,
  so Melt stays live and the rows wobble gently.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks), on one serial shift-add multiplier, 6 slots:
  - phase per virtual px (K0 · Columns);
  - phase per pixel;
  - start phase (drift − kv · xm);
  - drift step;
  - Rows · Bend;
  - comb gain (Ripple · kv / 2¹³).
- **Glides:** a shared bank (K1, K2, K3, K5, K6, P12).
- **Per line** (kicked at the source avid rise of line n, computing line n + 1; about 130 clocks):
  - flip, knot row and fraction, comb triangle;
  - a second serial shift-add multiplier for (y − 80)·Rows and tri · comb gain;
  - 48 knots × 2 rows read from the warp ROM, blended a + (b − a)·fy, written as t into a ping-pong line table
    (even/odd banks).
- **Pixel path, 36 stages:**
  - warp cell (Q16) and line-table read; fabric copy; knot select; (w1 − w0)·fx split; t;
  - d = t − y; melt product; d × Rows·Bend split; t''; clamp and background flag;
  - tile rows; phase from a late copy of the beam started by g_start + 13 (no phase delay line), + comb;
  - knot indices, then bank addresses (luma 4 banks by i/j parity, chroma 2 banks);
  - ROMs + fabric register; corner reorder and differences; φ products; t product;
  - compose (Ice, Ground); chroma box ring, luma [1 2 1], 13/16, gate.
- **Memories (21 EBR):** warp ROM 2048 × 8 (4), luma tile 4 × 1024 × 8 (8), chroma tile 2 × 512 × 16 (4), line
  table 2 × 256 × 16 (2), chroma ring (2), sync ring (1). RAMs carry `no_rw_check`.
- **Contracts:**
  - Generated avid (`p_gavid` / `g_start`).
  - `C_LAT` = `processing_delay_clks` = 36 in every mode; syncs ride a 256 × 3 EBR ring.
  - Virtual-1080 geometry from the timing ID: 4:3 SD shows the central 1440 virtual px.
  - Colours in the hardware convention (U = Cr, V = Cb).
  - Tables come from the `lava.py` build hook (pure Python) into `lava_pkg.vhd`.
- **Fit history:**
  - The first RTL had 6-bit bilinear in luma and chroma, parallel per-pixel Bend and Rows multiplies, and a
    19-stage phase delay line: 9306 LC (121 %), failed placement.
  - Standalone-yosys ablations (LUTs saved): luma bilinear 553, chroma bilinear 309, warp interpolation 314,
    Bend × Rows 315, melt 157, glides 138.
  - v0.5.0 folds Rows × Bend into one per-field gain, moves the per-line products onto a serial multiplier, drops
    the phase delay, and uses 4-bit tile fractions, nearest-row chroma and a 5-bit melt: LUT4 3878 standalone.
- **Verification** (GHDL 720p60, decimation 1):
  - Default Osc frame: the raw 10-bit output matches `imodel.py` to 0.33 code mean |diff| (1.1 % of pixels > 8
    codes: the Osc tap is a few px early).
  - Speed-max frame 3 frames later: matches the model's drift (0.34): the drops move left.
  - 4 × 4 region-mean |ΔRGB| against the photo: 4.0. Integer model against the photo: MAE 16.1 with Melt 0.

## Status
- **v0.5.0 built 2026-10-27, NOT flashed.** router2. Post-route Fmax:
  - HD Analog 84.78 MHz on **seed 2** (seed 1 stalled the router under machine load).
  - HD HDMI 86.63, HD Dual 78.72 MHz (seed 1).
  - SD Analog 87.03, SD HDMI 84.60, SD Dual 77.68 MHz (seed 1).
- 6673–6701 / 7680 LC (87 %), 21/32 EBR. All six bitstreams pass iceunpack.
- Presets are hand-written placeholders: Lava, Receding, Flood, Twin Curtain, Ice Sheet, Rising.

## Deviations from the photo, and why
- **Drape zone (x ≈ 150–800).** It renders as narrower vertical tongues with streaky detail instead of the photo's
  broad drapes with nested rims; local contrast there is 2–3.5× the photo's. The warp is a smooth 42 × 27 knot grid,
  which cannot hold the photo's per-drape staircase. A per-pixel warp fits it (MAE 6.3) but needs far more memory.
  The overall bend, the tip line and the colours are right.
- **White web.** It is fainter than the photo's. The tile is sharpened (×0.6) to restore the rim and web contrast
  on the right; more sharpening raised the error elsewhere.
- **Video mode.** It is busier: fine camera texture at the default Melt (20 %) gives ragged row edges.
- **Ground = Black.** The leftmost ~150 px keep some violet: those columns map to the tile's bottom (scallop) rows,
  not to the background.
- **Bilinear precision.** The tile uses 4-bit fractions, and chroma is taken at the nearest t knot (12.8 lines).
  This was a budget cut and is not visible at a glance.
