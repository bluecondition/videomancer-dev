# Lava — Heirloom #43

**v0.6.0 FINAL (phase 2, approved by Ron 2026-11-03: "I like it the way it is"), merged.** The diag preview
(`Lava Diag`, `out/rev_b/ron/_diag/lava_diag_v0.6.0.vmprog`) went into production unchanged;
`out/rev_b/ron/heirloom/lava.vmprog` holds its six bitstreams. See [Phase 2](#phase-2-v060) and [Status](#status).
v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed. Source photo:
`photos/WIN_20230605_01_14_56_Pro.jpg` (1920×1080 capture, 2023-06-05). Replaces v0.1.0 (lobes, pods and a stripe
curtain), which Ron judged too abstract.
Working files: `.heirloom_work/rb_lava/` (`notes.md` has every number; `treg.py` warp registration, `mkdata3.py`
tables, `proto2.py` float proto, `imodel.py` integer model, `sheet.png` / `sheet_im.png` control sheets,
`cmp_final.png`); phase 2 in `.heirloom_work/rb_lava/p2/`.

## Phase 2 (v0.6.0)
Built with a free hand while Ron was away (no notes from him for lava). He previewed the diag and approved it
as it is on 2026-11-03. The aim, from PHASE2.md: turn the global
X / Y controls into element controls, widen every knob, make each switch a real look-flip, and keep the photo at
the defaults. Lava is a measured field (one fitted oscillator period seen through a fitted warp), so the new
parameters act on the table's coordinates: the tile's row bands get their own heights, each drop gets its own
length, and Bend gets a negative side.

| Control | Was (v0.5) | Now (v0.6.0) |
|---|---|---|
| K1 Columns | drops across ×0.5 .. ×2, linear, photo at 341 | **×0.25 .. ×4**, exponential (4 octaves, piecewise-linear exp2), photo at **50 %** (512, kv bit-exact). The display is now 0–100 % |
| K2 Bend | the drapes 0 .. 1.6, photo 1.0 at 650 | **−1 .. 3**, linear, photo 1.0 at **50 %** (512). Below 0 the rows bow DOWN on the left (the drapes turn into a valley and the sheet rises up the left side); above 1.6 the drapes pull up and away to the right. Bend is now applied per warp knot in the line engine (so Drip isn't scaled by it); exact at 1.0 |
| K3 Rows | global row density ×0.6 .. ×1.6 about the top of the sheet (moved the sheet's outline) | **per-band heights, bipolar; the outline stays.** Left: the top bands (worms, zigzag rim) swell, up to ×20 at the top, while the lower bands squeeze toward the edge. Right: the lower bands (web, blobs, scallops) swell, the top ones squeeze. Centre ±8 = the photo (remap bypassed, bit-exact). A per-field remap RAM t → t + c·t(1034 − t)/1034, c = ±0.98, written in vblank by forward differences |
| K4 Speed | ±1 drop / 2 s, photo drift at 600 | **±1 drop / s** (twice the rate per step); the photo's drift (bit-exact) at the new default **560** |
| K5 Ripple → **Drip** (renamed) | the 8.5-line edge comb, 0 .. 8 px (near-dead: the sheet barely changed) | **bipolar per-drop tongues.** Right: every drop hangs its own tongue down from the sheet (to ~380 lines); left: every drop's centre pulls up into an arch. Each drop has its own length (0.25 .. 1 of the depth, 32 shuffled phases by drop number) and it breathes on a ~5.7 s cycle, so the tongues grow and shrink out of step. The comb serration rides along: 2.5 px at the centre (the photo) .. 10 px at the ends. Centre = the photo exactly. The tongues are drawn on the 48 warp knots (per-field table, added to each knot's t), so they are piecewise-linear: ~4 segments per drop at the default Columns, coarser at high Columns |
| K6 Melt | camera → rows 0 .. ±256 lines (K6 / 32), photo at 200 | **0 .. ±504 lines** (K6 / 16, twice the gain per step); the photo's ±48 at the new default **100** |
| S8 Ground | Stripes / Black (Black left violet columns at the far left) | **Stripes / Camera**: under the lava, the camera's luma in violet (a duotone in the stripes' family); in Osc mode the Osc waves as violet clouds |

Unchanged: S7 Mirror, S9 Flip, S10 Palette, S11 Source, P12 Flow (still the headline: 0 % drawn back into the top
right, 50 % the photo, 100 % flooding down). Default positions moved for K1 / K2 / K3 / K5 (to 512), K4 (560) and K6
(100); the defaults draw the v0.5 picture exactly (see Checks).

Presets: **Photo** (the defaults), Receding (Flow 120), **Dripping** (Drip 900, Flow 600), Twin Curtain (Mirror,
Bend 630 = v0.5's 1.28), **Giant Worms** (Rows 60, Columns 300), **Valley** (Bend 80, Drip 200 = arches),
**Over Camera** (Ground Camera, Drip 820, Flow 380), **Ice Rising** (Ice, Flip, Speed 466, Columns 760). All
hand-written placeholders.

Built and dropped for fit: the ground stripes on their own oscillator at the photo's pitch, so that Columns would
change only the drops and they'd detune against the stripes (~130 LC). Also tried first and replaced: a per-pixel
Drip (two multiplies and an early phase copy, +630 LUT) and a full-colour camera ground (+2 EBR).

**Fit.** The first full design was ~98 % LC. Fitted with no feature lost by: Drip moved from the pixel path to a
per-field knot table (serial multiplier) added in the line engine; Bend moved into the engine too (one multiply per
knot instead of per pixel); cheaper knob curves (exponential Columns as a constant-shifted multiplicand, linear
Bend / Melt / Speed with the default knob moved so the photo stays exact); the remap writer without a clamp or
staging; and the four pure output copies dropped (latency **36 → 32**, the same in every mode; an even change, so
the generated avid's U/V parity is unchanged). The Mirror turn compare (an 11-bit subtract feeding the phase and
warp registers) became a flag registered a clock early: HD Analog went 70.9 → 80.4 MHz.

**Build** (router2, 2026-11-02, `.heirloom_work/diag_build.sh lava`): HD Analog s1 80.39, SD Analog s1 79.31,
HD HDMI s2 77.61 (s1 missed), SD HDMI s1 80.30, HD Dual s1 77.24, SD Dual s2 73.17 MHz (Fmin 27; s1 stalled the
router); 7,242–7,265 / 7,680 LC (**94.6 %**, v0.5.1: 87 %), **26 / 32 EBR** (v0.5.1: 21). No pin-skew screen (the
predictor was found blind on 11-01); builder seeds kept.

**Checks.**
- GHDL analyse / elaborate / synth front end: clean.
- Python sweep of every new knob curve at the RTL widths (`p2/sweep.py`, `p2/sweep2.py`): no wraps; the Rows remap
  is monotone and inside 0..1034 at all 1024 settings; the tongues reach −379 .. +378 lines; the comb's modulo
  slice matches v0.5 at the default.
- **Default frame bit-exact with v0.5.1**: GHDL, 720p59.94 decimation 2, Osc, 3rd frame (drift included); the raw
  10-bit output of production v0.5.1 and the diag are identical on every pixel (`p2/sim/prod_osc*`, `diag_osc*`).
  This also confirms the latency-32 alignment.
- One extreme render (sim-only copy with the glides powered up at Rows 150, Drip 950, Ground Camera, Osc):
  worms swell into tall columns, tongues hang off the edge, the violet clouds show under the lava, no seams or
  garbage (`p2/sim/cmp_x.png`; at 720p decimation 2 the tester shows the left half of the virtual frame).
- Not verified: hue on hardware; 1080p / SD renders; motion of the tongues (breathing, drifting with Speed);
  Video-mode Camera ground; Flip / Mirror with Drip (reasoned from the code: the knot table mirrors with the warp).

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
3. **Controls on t.** t'' = y − Flow + Bend·(t − y) + Drip(x) + Melt·source (Bend and the Drip tongues are applied
   per warp knot in the line engine); then the Rows remap t → t + c·t(1034 − t)/1034 on the tile row.
4. **Comb.** On the lava only (t < 1060), φ += Ripple · tri(line / 8.5 + crawl).
5. **Tile.**
   - Luma: 24 phase knots × 164 t knots (6.4 lines apart), bilinear.
   - Chroma: 12 × 82 knots, the nearest t knot, linear along φ.
   - Fitted from the registered tile and sharpened ×0.6; luma pre-boosted 16/13.
   - Below the lava edge, the tile's stripe rows draw the violet background (Ground = Black blanks them).
6. **Output.** The shell's conditioning: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16, blanking gate.
   Output luma peaks at 730.

## Controls (v0.6.0)
| Control | Photo element it drives |
|---|---|
| **P12 Flow** | The lava sheet's reach: t offset ±416 lines. 0 % = the lava drawn back into a bowl at the top right over the stripes; 50 % = the photo; 100 % = it floods down over the stripes, molten columns at the top. Glided. A 12 s breath (±20 lines) always moves the edge. |
| K1 Columns | The drops and stripes across: oscillator frequency ×0.25–×4, exponential (the photo: 11.5 periods across, at 50 %). Glided. |
| K2 Bend | The drapes: −1 (the rows bow down on the left: a valley) .. 0 (a flat lattice) .. 1 (the photo, 50 %) .. 3 (the drapes pull up and away). Glided. |
| K3 Rows | The row bands' own heights, bipolar, outline fixed: left = the worms / zigzag rim swell and the lower bands squeeze; right = the web / blobs / scallops swell. Centre = the photo. Glided. |
| K4 Speed | Bipolar, still in the centre ±8: the drops and stripes slide sideways (+ = left), up to one drop per second; default 560 = the photo's drift (one drop per 12.6 s). |
| K5 Drip | Bipolar per-drop tongues: right = each drop hangs its own tongue down (to ~380 lines), left = arches; every drop its own length, breathing (~5.7 s); the edge serration deepens with it (2.5 → 10 px). Centre = the photo. Glided. |
| K6 Melt | How far the camera (or Osc) bends the rows: up to ±504 lines from the camera luma (default 100 = ±48, the photo setting). Glided. |
| S7 Mirror | The left half reflected: twin curtains meeting in the middle. |
| S8 Ground | Stripes (the photo) / Camera (the camera's luma in violet under the lava; Osc clouds in Osc mode). |
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
  - kv · 960 (the drops' phase at virtual x 0, with the drop index above it);
  - comb gain (comb amplitude · kv / 2¹⁴);
  - kv · 42 (the phase step per warp knot).
  Then, from zone clock 2, the Rows remap RAM (1024 × 11, forward differences, one entry per clock), and from zone
  clock 256 the Drip knot table (48 knots × 32 clocks on a serial shift-add: window × the drop's breathing length ×
  depth), written into the bank of the field parity; the line engine reads the other bank.
- **Glides:** a shared bank (K1, K2, K3, K5, K6, P12). Speed, Rows (c) and the switches are field-latched.
- **Per line** (kicked at the source avid rise of line n, computing line n + 1; about 130 clocks):
  - flip, knot row and fraction, comb triangle;
  - a second serial shift-add multiplier for (y − 80)·Rows and tri · comb gain;
  - 48 knots × 2 rows read from the warp ROM, blended w = a + (b − a)·fy, then t = ye + Bend·w + Drip (Bend split
    6 | 4 over two stages), written into a ping-pong line table (even/odd banks).
- **Pixel path, 32 stages** (v0.5: 36):
  - warp cell (Q16) and line-table read; fabric copy; knot select; (w1 − w0)·fx split; t;
  - d = t − y; melt product; two copy stages (Bend moved to the engine); t''; clamp and background flag, Rows RAM
    address; Rows RAM + fabric copy; the remapped row (bypassed at the Rows centre and on the ground);
  - tile rows; phase from a late copy of the beam started by g_start + 13 (no phase delay line), + comb;
  - knot indices, then bank addresses (luma 4 banks by i/j parity, chroma 2 banks);
  - ROMs + fabric register; corner reorder and differences; φ products; t product;
  - compose (Ice, Ground Camera from an 8-bit camera ring); chroma box ring, luma [1 2 1], 13/16, gate.
- **Memories (26 EBR):** warp ROM 2048 × 8 (4), luma tile 4 × 1024 × 8 (8), chroma tile 2 × 512 × 16 (4), line
  table 2 × 256 × 16 (2), chroma ring (2), sync ring (1), Rows remap 1024 × 11 (3), Drip knot table (1), camera
  ring 256 × 8 (1). RAMs carry `no_rw_check`.
- **Contracts:**
  - Generated avid (`p_gavid` / `g_start`).
  - `C_LAT` = `processing_delay_clks` = 32 in every mode (v0.5: 36); syncs ride a 256 × 3 EBR ring.
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
- **v0.6.0 FINAL, approved by Ron 2026-11-03 after previewing the diag ("I like it the way it is"); merged.**
  - `out/rev_b/ron/heirloom/lava.vmprog` holds the approved diag's six bitstreams, repacked with the production
    TOML (id `com.czartroniks.lava`, name "Lava").
  - The merged sources match the diag's byte for byte, apart from the `lava.vhd` header comment.
  - Seeds (router2): HD HDMI s2, SD Dual s2, the rest s1. See the Phase 2 section for Fmax.
  - To rebuild from source, run `./build_heirloom.sh lava`. router2 is nondeterministic, so a rebuild won't
    reproduce the approved routes exactly.
  - Presets are still the hand-written placeholders. Capture Ron's looks with `./capture_preset.sh`.
- v0.6.0 diag built 2026-11-02 (phase 2; see the Phase 2 section).
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). The once-a-field zone reseed is replaced by the per-line resync. Post-route Fmax: HD Analog s1 77.28, SD Analog s1 85.47, HD HDMI s1 82.93, SD HDMI s1 83.77, HD Dual s1 82.94, SD Dual s1 87.32 MHz; 6696-6718 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
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

## Open questions (2026-11-02 batch): closed

Ron approved v0.6.0 as it is (2026-11-03), so both questions are settled the way the diag has them:
1. The ground stays Stripes / Camera. The separate ground-stripe oscillator (cut for fit) is not coming back.
2. The edge comb keeps deepening with Drip.

If HD HDMI ever shows lines, re-seed that config. No pin-skew screen was run.
