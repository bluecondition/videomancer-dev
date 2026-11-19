# Riptide — a torn, mirrored swirl band, Heirloom #15

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/riptide.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-03, phase 2): diag preview built, awaiting Ron's preview** (`Riptide Diag`,
`out/rev_b/ron/_diag/riptide_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`). Production v0.5.1
(`programs/ron/heirloom/riptide/`) is untouched. Phase-2 working files: `.heirloom_work/rb_riptide/p2/`.
v0.5.0 (2026-10-27): rebuilt from the photo `photos/WIN_20230309_00_28_35_Pro.jpg` (1920×1080 capture,
2023-03-09), built, NOT flashed.

## Phase 2 (v0.6.0)
Ron left no notes for riptide (free hand, 2026-11-03 batch). This round follows the phase-2 brief:
- groups of shapes get their own parameters;
- the near-dead K6 is re-assigned and the redundant S9 replaced;
- nothing new moves at the defaults.

v0.5.1 was 99 % LC. The new features are paid for by moving per-field arithmetic and pixel delay chains into
existing machinery, and the diag ends ~150 LC smaller than production. The defaults draw the v0.5 picture.

| Was (v0.5) | Now (v0.6.0) | Full sweep in screen terms |
|---|---|---|
| K4 Speed: bipolar, still at the centre: hue drift, ridge crawl, column drift. The strips slid only when P12 was above 50 % | **+ past its default the strips slide apart by themselves.** Each of the seven mirrored strips swings on its own triangle oscillator (v0.5's), at its own rate and phase, with amplitude \|speed\| − 128 px. This adds to Rip's slide | No slide anywhere in 376..648 (default 640 = the photo). 52 px at 700, 152 at 800, 252 at 900, ±376 px (±20 % of the width) at either end. Strip periods are 0.2..0.45 s at full speed and 0.5..1.25 s at 700. With P12 at 100 %, up to ±675 px |
| K2 Ridges: ridge spacing 0.37..2.4× in 16 steps (0 % and 50 % looked almost the same) | **K6 Columns: the period of the texture's crimson-beaded columns.** Exponential, 4× wider .. ¼, smooth (0.4 % steps, glided), scaled about the screen centre. 50 % = the photo's 375 px, bit-exact. The ridges stay at 10.8 lines | 0 % = 1.3 columns (1,500 px), 25 % = 2.6, 50 % = 5.1, 75 % = 10.2, 100 % = 20.4 (94 px pinstripes). Covers the whole texture, 64 % of the screen |
| S9 Palette: Photo / Opposite (hue + 180°, the same as K3 at 50 %) | **S9 Palette: Photo / Prism.** Strip k of the band, and the glitch zone below it, turns k/7 round the colour wheel (0, 52, 103, 155, 205, 257, 308°). Mirror pairs match. The texture stays blue | Seven distinct colour strips, rainbow-stepped from each edge to the axis. K3 still turns everything. Torn-edge lines take the hue of the band row they reflect |

Unchanged: K1 Band, K2 Etch, K3 Hue, K5 Gain, S7 Mirror, S8 Quad, S10 Twin, S11 Source, P12 Rip (its strip slide
now adds to K4's). Renamed: **K6 Ridges → Columns**, and S9's second label **Opposite → Prism**.

Presets:
- **Photo** (the defaults; was "Riptide").
- Calm Water, Kaleidoscope, Twin Rivers: unchanged.
- Shredded: K4 900 now also slides the strips ±252 px.
- **Prism Slide** (new): S9 Prism, K4 860, K1 760.
- **Pinstripes** (new): K6 1000 ≈ 19 columns, K5 680, K4 380 (slow reverse).
- **Broad Tide** (new): K6 40 ≈ 1.4 columns, K4 1000, P12 600.

Fit (no feature lost, defaults bit-exact):
- Per-field shift-add chains moved onto the serial multiplier (9 → 14 slots): the band column of the first pixel
  (4369 × xsu / 16), the stripe phase (337 × xcol), the gain (134 × k5q) and Rip's slide (300 × h).
- K2, K3 and K5 are read directly; K1, K6 and P12 keep the glide.
- The gain's leading-one float is computed per field.
- Latency is 28 → 26: the output pads are removed (an even change, so U/V pairing is kept). The sync ring follows
  `C_LAT`, and the TOML `processing_delay_clks` is 26.
- One 256 × 32 block-RAM ring at delay 4 (read 2 behind the chroma ring's write pointer) carries the luma to the
  box centre, the column fraction, the band flags and the value. The value pipeline is re-timed one stage later.
- Result: 7,615 → 7,462 LC (hd_hdmi pack) with all three features.

Build (ROUTER1, 2026-11-03, `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh riptide`):
- HD Analog s1 88.51, SD Analog s1 82.54, HD HDMI s2 92.77 (router stall on seed 1), SD HDMI s1 78.03, HD Dual s1
  86.15, SD Dual s1 87.47 MHz.
- 7,457–7,472 / 7,680 LC (97.2 %), 29 / 32 EBR.
- dupin: 0 self-add carries. All six bitstreams unpack cleanly. No pin-skew screen; the builder's seeds are kept.

Checks (work files in `.heirloom_work/rb_riptide/p2/`):
- The GHDL front end is clean (`check.sh`).
- `sweep.py` swept the new or moved arithmetic at RTL widths: 0 failures (moved products bit-exact with v0.5, K6
  default exact, centre column fixed, slide ≤ 675 px, |e_off| ≤ 803 px, band column stays inside the mirrored field).
- The default frame is **bit-exact** with production v0.5.1: GHDL 720p60, decimation 1, Osc, 2 warm-up frames, 0 of
  921,600 pixels (`r_prod_def.npy` vs `r_diag_def.npy`).
- One extreme render (`r_diag_ext.png`: Prism, K6 1023, K4 1023) shows Prism strips, sliding and ~20 columns, with
  no wraps.
- Not simulated: Video mode, SD and 1080p, motion over time, Twin / Repeat / Quad with Prism.

The sections below are the production (v0.5.1) SUMMARY, unchanged.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_riptide/` (`notes.md` has every number).
- **Composition.**
  - A band of psychedelic swirl fills rows 327–712 (half-height 193). It is red / blue / cyan / green / olive /
    orange / black blobs with smooth gradients and hard edges.
  - The band is MIRRORED top to bottom about y = 519.5. The flip-correlation error is 20.6 at that axis
    against ~100 elsewhere, and the left/right flip gives 76 (no mirror). Shapes on the axis are symmetric:
    "O", "D" and "C" rings at x ≈ 110, 850, 940.
- **Strips and tears.**
  - The band is a stack of smooth strips separated by glitch zones of torn lines. In rows from the band
    edge (0 = edge, 192 = axis) the zones are 18–27, 44–56, 98–107, 119–128, 148–166 and 168–177, mirrored.
  - Inside a zone every line is jittered sideways, with σ ≈ 3 px (zone line-to-line difference 13.3).
  - Both band edges are torn over ~22 lines (y 304–326 and 713–735): lines of band colour alternate with
    texture lines.
- **Colour mechanism.** Every colour sits on the RGB-cube faces (band median minimum channel 5): HSV at full
  saturation, so one hue/value colourizer explains the picture. Fitted as a 256 × 32 knot grid of (hue 6 bits,
  value 4 bits) over the top half-band, the mean |ΔRGB| is 12.55.
- **Texture (above and below, the same statistics).** A blue "fingerprint", mean RGB (35, 70, 139).
  - **Ridges.** Ridges every 10.8 lines (9.5–11.2 per 256-px block). They tilt −1.75 °/px of phase but
    wander 1.6–2.0 rad around that, so they read as wavy horizontal lines.
  - **Columns.** One every 374.9 px (5.12 per width). Crimson-beaded columns sit at x ≈ 88 + 375k, with a
    weaker one at 280 + 375k; clean blue/teal columns lie between.
  - **Hue.** The base is 210–226° (sd 21–31°) with crimson beads at 324–338°. Beads are up to 27 % of the
    pixels at a red column's centre (zone R mean 70) and none in the clean columns. The hue histogram is
    bimodal: 43 % of pixels at 210–240° plus the crimson mode.
  - **Hue/value correlation.** −0.6 in the red columns, +0.8 in the clean ones.
  - **Floor.** The minimum channel is about 22.
- **Fit quality.**
  - Integer model against the photo at 1080p: 4×4 region-mean difference 6.8 of 255. The red-column R
    profile peaks at 66 against 70, and the secondary red column is 36 against 34.
  - RTL (GHDL) against the model: region-mean difference 1.4 (Osc) and 1.6 (Video).

## What it does
1. **Per field** (vblank, on one 9-slot serial shift-add multiplier):
   - band half-height from K1²;
   - Rip² and Gain²;
   - zone phase;
   - lattice start rows and shears;
   - ridge rate from the K6 table.
   These give the torn-edge depth (Rip² · 64 / 256 lines, 23 at the default), the jitter gains, the strip
   slide (300 px · (2·Rip − 1) above 50 %), the value gain (0.41..2.5) and the hue offset (Hue + drift +
   Opposite).
2. **Per line** (a micro-coded engine on one two-phase 22-bit ALU, kicked at each line start, computes the
   NEXT line):
   - a xorshift32 draw;
   - the band / torn-edge / texture decision, with Twin's second band and the torn edge's "near" lines
     (235 of 256 lines);
   - Fold or Repeat, then the knot row and 4-bit fraction;
   - the strip from the glitch-zone table;
   - the strip's slide (a per-strip triangle oscillator) and the per-line jitter;
   - the column DDA start.
3. **Band fill.** Two knot rows (20 EBR) are lerped vertically, hue on the shortest path, into a two-bank row
   RAM of (H8, V8) per knot column.
4. **Texture fill.**
   - **Noise.** A pipelined 24-bit hash (four add/xor rounds) drives two sheared lattices: fine 8 px ×
     2.39 lines, coarse 16 px × 8 lines. Each is lerped vertically in quarters, then combined as
     n = (2C + 1.25F) / 8 into the two noises p and q.
   - **Zone.** 32 zones per 374.9 px give muH, AH, muV, AV, the bead threshold, and whether the hue noise
     is q, q + p or q − p (the zone's hue/value correlation).
   - **Ridge.** A triangle floored at −0.39, so the ridges are one-sided teal lines, with its phase
     wandered by q / 8 cycle.
   - **Hue.** H = muH + AH·noise + KS·ridge, or for a bead (q above the zone's threshold, set from the
     photo's red fraction) the crimson HB.
   - **Value.** V = muV + AV·p (+ VB for a bead). One (H8, V8) is written per 8-px column.
5. **Pixel path.**
   - Column DDA, field-edge and Quad mirrors, row-RAM read.
   - Hue / value lerp, + hue offset + Etch (source luma × K2, up to ±176°).
   - Value × gain (float).
   - A 256-entry hue ROM (Y, Cr, Cb at full value) × value: three radix-4 products, + the floor.
6. **Output.** Generated avid. Conditioning before the blanking gate: chroma 9-tap box via an EBR ring,
   luma [1 2 1] and 13/16. Latency 28 clocks in every mode. Geometry is in virtual 1080-line units; SD shows
   the centre 1440 of the 1920 virtual columns (a 4:3 crop of the same picture).

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Rip** | The glitch zones and torn edges. 0 = a clean mirrored band. Default 310 = the photo: ~3 px jitter in the six zones and 22-line torn edges. Edge depth grows as Rip² (255 lines at 100 %). Above 50 % the strips slide sideways (up to ±300 px), every band line jitters and the texture lines tear. |
| K1 Band | Band half-height 24–541 lines (square law). Default 585 = 193 lines = the photo. Past that the knot field refolds. |
| K6 Etch | The source picture etched into everything as hue contours: luma (Video) or the oscillator (Osc) rotates the hue of band and texture, up to ±176°. Default 300. |
| K3 Hue | Colourizer phase, a full turn. 0 = the photo's palette. |
| K4 Speed | Bipolar, still at the centre (±8). Hue drift, the strips' slide oscillation, ridge crawl and column drift. Default 640. |
| K5 Gain | Value gain 0.41–2.5× (square law), from dark ink to posterized plateaus. Default 547 (53 %) = 1.0× = the photo. |
| K2 Ridges | Fingerprint ridge spacing 0.37–2.4× in 16 steps. Centre (50 %) = 10.8 lines = the photo. |
| S7 Mirror | Fold (the photo's top/bottom reflection) / Repeat (strips stacked without reflection). |
| S8 Quad | The band is also mirrored left/right. |
| S9 Palette | Photo / Opposite (hue + 180°: an orange fingerprint around a complementary band). |
| S10 Twin | One band / two bands. |
| S11 Source | Video / Osc. |

Presets (placeholders): Riptide (defaults), Calm Water (Rip 0, slower), Kaleidoscope (Quad, full band),
Shredded (Rip 100 %, fast), Twin Rivers (Twin, narrower bands, hue 340).

## Video
The camera enters only through Etch. Its luma, centred, times K2 rotates the hue of every band and texture
pixel, so faces and edges show up as hue contours flowing through the swirl and the fingerprint. The structure
(band, strips, texture) is always generated. Osc replaces the camera with two crossed triangle ramps
(3x + t, 4y − t) that drift each field.

## Engine
- **Vblank zone.**
  - Width and switch capture.
  - A 6-channel glide datapath (K1, K2, K3, K5, K6, P12), with ease by diff >> 3 and a snap at warm-up.
  - Mode geometry.
  - Then the 9-slot serial multiplier (16 steps a slot) and its derived terms.
- **Line engine.**
  - One 22-bit ALU computing a + (b xor inv) + cin. Operands are registered in phase 0 and executed in
    phase 1, which closed 28.9 → 80+ MHz.
  - It runs from a micro-program per line type, with its serial products on the same ALU.
  - Fills are written by write-column counters.
- **Memory.** 27/32 EBR: knot ROMs 20, the row RAM, the colour ROM and the sync / chroma delay rings.
- **History.** The first Cartesian (u, v) design was 9743–10327 LC. The polar redesign took it to 7552 LC.
  Red beads, the correlation select, ridge floor and wander, and removing the zone dither and the BH product
  ended at 7553–7586 LC.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). s_gav_sr was extended to index 18 for the gate. Post-route Fmax: HD Analog s1 79.50, SD Analog s1 79.97, HD HDMI s1 84.16, SD HDMI s1 92.16, HD Dual s1 88.37, SD Dual s1 76.95 MHz; 7609-7640 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- v0.5.0 (2026-10-27): built 6/6 with **ROUTER1** (`NEXTPNR_ROUTER="--router router1"`), all HD PASS at
  74.25 MHz on the last post-route line:

  | Config | Seed | Fmax | LC |
  |---|---|---|---|
  | HD Analog | **2** | 79.26 MHz | 7553 |
  | SD Analog | 1 | 88.90 MHz | 7586 |
  | HD HDMI | 1 | 87.54 MHz | 7573 |
  | SD HDMI | 1 | 77.50 MHz | 7566 |
  | HD Dual | 1 | 86.17 MHz | 7576 |
  | SD Dual | 1 | 81.35 MHz | 7572 |

  - HD Analog seed 1 stalled the router, so that config is on seed 2.
  - 98–99 % LC (very full), 27/32 EBR, no self-add carries (dupin 0).
- GHDL renders (720p60 Osc and Video, 480p motion) match the integer model. **Not flashed.**

## Deviations from the photo
- The band's 1–3 px detail (fine tendrils, dashed glitch lines) is softened by the 256 × 32 knot grid at
  6 + 4 bits. The fit error is 12.55 against ~10 for a Cartesian (u, v) grid that did not fit the chip.
- The texture is a statistical match, not a pixel replica (the noise differs).
  - It is slightly lighter and greyer: floor 40 against ~22 (the tuner pinned it at its bound).
  - The clean columns are a little greener (G 88 against 74), and the red columns darker in G/B.
  - The streaks are a little taller than the photo's 1–2-line dashes.
- The bottom half is an exact mirror (the photo's is shifted 0–6 px in places).
- The torn edges keep only the "near" lines (the band's edge rows reflected and jittered). The photo's
  longer streak lines reaching into the texture were cut to fit.
- K6 changes only the ridge spacing (the lattice scale is fixed), and Speed does not roll the texture.
- Hue is quantized to 256 steps (1.4°) by the colour ROM.
- The capture borders are not drawn.
- At SD the picture is the centre 4:3 crop of the 16:9 composition (same scale, about 3.8 of the 5.1 columns
  visible); 720p and 1080p are identical.

## Open questions for Ron
1. K4's slide gets wider AND faster together, and is frantic at the ends. Should the slide amplitude get its own
   knob instead of K3 Hue?
2. Should the ridges come back on K6, scaled together with the columns, or stay fixed?
3. For Prism, would you prefer every other strip at +180°, or a Prism that also turns the texture?
4. Should the strips slide a little at the default speed (now 0, which keeps the photo)?

## Hardware watch list
- **Renamed controls:** K6 Ridges → Columns, and S9's second label Opposite → Prism.
- **Latency 28 → 26.** If the diag shows U/V-swapped colour that production doesn't, suspect this first.
- **The new block-RAM ring** (29 EBR). Suspect it for any flicker that production doesn't have.
- **K4 at the ends:** the strips whip ±376 px several times a second.
- **K6 at 100 %:** 94 px columns made of 8 px cells; check they still read as columns.
- Timing is comfortable everywhere: every HD config routes at ≥ 86 MHz.
- Nothing was cut for fit.

## Next step
On approval:
1. Run `./heirloom_diag.sh merge riptide`.
2. Repack the previewed diag bitstreams under the production TOML instead of re-routing: copy
   `build/programs/ron/riptide_diag/rev_b/bitstreams/*.bin` into a scratch dir together with the production
   `program_config.bin`, then run `vmprog_pack.py --no-sign --hardware rev_b --toml-path
   programs/ron/heirloom/riptide/riptide.toml <dir> out/rev_b/ron/heirloom/riptide.vmprog`.
3. Update the README and PHASE2 rows, and commit riptide's files only.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. Here the camera strength was on K2 (Etch), so K2 and K6 swapped: **K6 = Etch** (the camera strength, default now 512 = 50 %), **K2 = Columns** (was K6). Nothing else changed: the VHDL swaps the two knob inputs (the processing is identical), the TOML blocks and preset keys moved with them (the Controls rows above are renumbered). v0.6.1, rebuilt with router1: SD HDMI seed 2; HD worst 84.73 MHz (HD Analog 84.76, HD HDMI 84.73, HD Dual 84.92), LC LC. Rebuild: `NEXTPNR_ROUTER="--router router1" ./heirloom_diag.sh build riptide`. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/.

## K6 top +25 % and default 100 % (2026-11-15)

**Element type**: K6 Etch rotates the hue by the camera. The change is the same as in the main program: s_eh = (K6 / 16 + K6 / 64) / 2 (0 .. 39), applied >> 1 through the slice p9_he(12:1). The top swing is +-219 degrees (was +-176). The K6 default is now 100 % (orchestrator).
- Not rendered; checked by GHDL analysis and the build.
- Build: v0.6.2, router1; SD Dual seed 2 (seed 1 stalled router1 at the 600 s timeout under machine load), seed 1 elsewhere; worst HD 80.60 MHz (HD Analog 80.60, HD HDMI 86.04, HD Dual 86.19); 7,498 / 7,680 LC. `.kinetic_work/k6_builds.txt`: diag riptide OK router1 worst-HD 80.6 LC 7498.
