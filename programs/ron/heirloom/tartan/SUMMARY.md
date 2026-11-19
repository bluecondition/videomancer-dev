# Tartan — Heirloom #25: a neon A-frame cut by squeezed dark bands

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/tartan.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Tartan Diag`, `out/rev_b/ron/_diag/tartan_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/tartan/`) is untouched.
v0.5.0 (2026-10-27): rebuilt from the photo WIN_20230410_01_13_24, built, NOT flashed.
The v0.1 (generic diagonal stripes sliced into offset bands) was too abstract. This version is measured
from the photo. A big A-frame of nested diagonal stripes is cut by horizontal bands. The bands alternate
between a bright colourizer (cyan, green, orange, magenta) and a dark one (black with cyan and green
stripes, edged in yellow). The bands are thin in the middle of the frame and wide at the top and lower
down.

## Phase 2 (v0.6.0)
No notes from Ron for tartan (free hand, 2026-11-09 batch). The v0.5 knobs were already bold (Stripes, Slope,
Rows, Squeeze and Shade each redraw the picture), but every one of them was a global oscillator setting, and three
switches barely showed: S7 Fringe (the yellow edges), S8 Mirror (the left barcode on both sides) and S9 Tear (the
dotted glitch). The round:
- makes the **apex column** an element: every band now carries its own apex offset, computed per line, so the
  A-frame can slide, tear and repeat band by band (the beam start A0 = g·d0 moved from a per-field constant into
  the line engine);
- patches the **band type into the stripe direction** (S7 Twill: the dark bands draw the A upside down);
- re-uses the apex machinery for a **row of peaks** (S8) and **shredded bands** (S9);
- gives K5 a second job past the photo's speed (the bands slide sideways, each on its own oscillator);
- re-curves K1 to an exponential ×0.25 .. ×4.

Every new effect is exactly zero at the default knob value and in the default switch positions; the default frame is
byte-identical to v0.5.1 (checks below). Phase-2 working files: `.heirloom_work/rb_tartan/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| K1 Stripes: colourizer gain ×0.35 .. ×3, square law, photo at 507 | **×0.25 .. ×4, exponential** (an octave per quarter turn, linear within it), photo at 507 (exact: g = 256) | the A-frames' stripe period ×4 wider (the photo's ~80 px inner ripple becomes ~320 px bands, three or four stripes per leg) .. ×¼ (hairline A's, ~20 px ripple) |
| K5 Speed: the A-frames climb, the bands roll, the squeeze drifts | same (bit-exact), **plus past the photo's speed** (\|v\| > 188: K5 below 324 or above 700; exactly 0 in 324 .. 700, default 700) **every band's apex slides sideways** on its own triangle oscillator (four rates 1 : 1.25 : 1.5 : 1.75 by band, phases staggered band to band) | amplitude 3 half px per step: ±486 px (a quarter of the width each way) at either end; one slide cycle ~17 s just past the threshold, ~4.8 s (fastest band ~2.8 s) at the ends. The bands' apexes snake against each other; the seam, the hooks and the curl ride with each band's apex |
| S7 Fringe: the yellow fringes into black, On / Off (subtle) | **S7 Weave: Plain / Twill** (renamed). Twill: the dark bands draw the A-frame **upside down** (a V: u = ±g(x − xc) + g·s·(y − 1080)), the bright bands keep the A | about half the frame (every dark band) flips to "\ /" stripes, so the bright A's and dark V's interlock into a herringbone / harlequin lattice; with K5 moving, the V's travel against the A's. The fringe is always on (the photo) |
| S8 Mirror: the left family on both sides (subtle) | **S8 Peaks: One / Four** (renamed). Four: the A-frame repeats every 480 px (apexes at x ≈ 25, 505, 985, 1465), each peak mirrored (v0.5's Mirror per peak: the left barcode both sides, seamless at the apex and the valley) | from one big A to a row of four nested A's: a zigzag mountain range over the full width; slide and shred move the whole row per band |
| S9 Tear: the dotted sync glitch and the slipped band tops, On / Off (subtle) | **S9 Tear: Photo / Shred**. Photo = v0.5's On (the dots, one dark band in four slipped by ±32 px). Shred adds **every band torn sideways** by its own random offset, re-rolled every 128 fields (~2 s) | each band's apex jumps up to ±384 px (±20 % of the width), so the A-frame breaks into a staircase of offset bands; adds to the K5 slide |

Unchanged: K2 Slope, K3 Rows, K4 Squeeze, K6 Bend, S10 Palette, S11 Source (Video / Osc, default Video), P12 Shade.
The K1 display stays 0–100 %.

**Presets** (preset 0 = the defaults): **Photo**; **Bright Tartan** (P12 0); **Dark Tartan** (P12 100 %);
**Herringbone** (S7 Twill, even bands K4 0, more rows); **Mountains** (S8 Four, steeper legs, broader stripes);
**Shredded** (S9 Shred, P12 41 %); **Slipstream** (K5 990: fast slide, deeper squeeze); **Argyle Night** (Twill +
Four + Night, even bands, finer stripes). Dropped: Even Bands, Folded Night, Camera Weave.

**Engine (new).**
- **K1 curve.** g8 = 2^((k + 5 − 512)/256) · 256, piecewise linear within each octave (64 + f/4, 128 + f/2,
  256 + f, 512 + 2f; f = the low 8 bits), at zone clock 25. It replaces the two square-law serial products
  (slots 32 / 56). c_gs widened to 12 bits (g up to 1022 × s up to 639).
- **Per-line apex offset** (line engine clocks 59–92, after the existing 58): the band index (mod 4) and a band
  phase offset (+1443 per band) step at each band start; the band's phase = (1 + m/4)·T + offset (m = band index mod 4; T is 18 bits so the quarter rates stay continuous across its wrap); its
  triangle × the slide amplitude (serial multiply) = the slide; plus the Shred draw (its own LFSR, so the photo's
  slip sequence is untouched) × 3/8 = ±768 half px. d0 = x0 − xc + offset (+2 periods under Peaks, folded into
  [−480, 480) half px), then A0 = g8 · d0 on the serial multiplier. The beam loads (A0, d0) at each line start:
  at the defaults A0 is exactly v0.5's per-field c_a0.
- **Beam.** A second counter d (13 bits) runs beside A. Peaks: when d ≥ 480 − xs both fold back one period
  (d += xs − 960, A += g8·xs − g8·960, per-field constants). The side split (55 / 75 px past the apex) and the curl
  read d instead of the raw x, so they follow each band's apex.
- **Twill.** The dark bands' line term is scroll + V·g − V·g(1080) (one more per-field product, slot 152, freed by
  the old A0 constant), in place of scroll − V·g.
- **Slide.** K5 is now glided (zone clocks 22–24, the shared glide registers), only for the slide amplitude and
  rate; the climb / roll / drift rates still read the raw K5 exactly as v0.5.
- Removed: the Fringe and Mirror switch logic (Fringe is always on; Mirror lives on inside Peaks).
- No pipeline stages added or removed: latency 32 in every mode, U/V parity unchanged.

**Build** (router2, `.heirloom_work/diag_build.sh tartan`, 2026-11-09): HD Analog s1 79.08, SD Analog s1 78.58,
HD HDMI s1 84.68, SD HDMI s1 76.98, HD Dual s1 78.86, SD Dual s1 72.14 MHz (all seed 1). 6,652–6,680 / 7,680 LC
(87 %, +~930 over v0.5.1), 7 / 32 EBR (unchanged). iceunpack OK ×6; the packed config carries the new labels and
presets. No pin-skew screen.

**Checks** (working files in `.heirloom_work/rb_tartan/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py` at RTL widths: the K1 curve over all 1024 positions (65 .. 1022, monotonic, exactly 256 at 507);
  c_gs / c_ast / c_gp / c_astw over K1 × K2; the slide over every K5 (exactly 0 at the default 700); d0, the Peaks
  fold, A0 = g8·d0 and the whole beam line in all three rasters, both Peaks positions and the Shred extremes: no
  overflow, A = g8·d at every pixel, the fold always lands in [−480, 480). The shortest barcode run is 8 entries,
  so K1's ×4 stretch has no 1-entry spike to expose.
- Default frame (GHDL 720p60 decimation 1, Osc, 2 warm-up + 1 capture, every switch set explicitly; production
  v0.5.1 simulated the same way, `simpair.sh`): raw 10-bit output **byte-identical** (0 of 921,600 px; Y mean 434,
  22 % bright, not a blank frame), on the first source and again on the final one (`simdiag2.sh`).
- One extreme frame (`diag_ext.png`, Twill + Four + Shred + K5 990, Osc; `cmp_def_ext.png` = default over
  extreme): four peaks, each band's row of peaks offset by its own amount, the dark bands' V's interlocking. Rendered
  before the slide-rate fix (it changes only the phase rate).
- Not verified: the K5 slide in motion (amplitude and rates reasoned from the code), Video mode with a real camera,
  the new features at 1080 and SD (720p only), K1 at its ends on hardware.

## Reading of the photo (measured; .heirloom_work/rb_tartan/notes.txt)
- **Palette (k-means, 10 clusters).**
  - Black (16,21,10) 28 %, cyan (2,217,218) 25 %, green (4,245,137) 20 %, orange (251,96,7) 13.5 %,
    magenta (253,18,174) 3 %.
  - The yellow fringe peaks at (205,222,93) on cyan → black and (73,241,28) on green → black. It is about
    7 px wide and sits only on edges INTO black, moving right; the edges out of black are clean.
- **Two colourizers.**
  - In the bright bands only cyan–green, green–orange and orange–magenta ever touch, so the bright bank is
    a 1-D colourizer, C < G < O < M.
  - The dark bank shows black, cyan and green.
- **Bands.** The boundaries fit phase(y) = 0.2284 + 0.011381·y + 0.924·sin(2π(y − 463.2)/729.2), bright
  where frac(phase) < 0.4986.
  - That is a vertical square wave whose frequency is modulated by a slow vertical sine: 10.8 bands per
    frame, 1.48 modulator cycles.
  - Band heights: 26–28 lines in the middle (y 400–550), 92–135 at the top and at y 700–910.
  - Fit: within ±3 lines, 96.7 % of rows typed correctly.
- **A-frame.** u = |x − 984.5| − 0.9 y. A general T(x) + V(y) fit by contour alignment gives T linear
  (slope 1.01 on both sides, flattening only at x < 55) and V linear (0.89).
  - In u-space every stripe edge is a vertical line through all bands. The colour is therefore a measured
    barcode of u, one per band type and side, accurate to 91.5 % (bright) and 92.6 % (dark) of pixels.
  - The inner ripple (period ~80 px) is symmetric about the apex. The outer parts differ left and right.
- **Junction.** The left family's stripes run 55 px (bright bands) or 75 px (dark bands) past the apex
  and curl into hooks.
- **Corner.** The far-left stripes (u > 438) are orange/magenta in the top bands and cyan/green below:
  the same edges two colourizer levels apart.
- **Glitches.**
  - A dotted yellow line (32 px period, 11 px dashes) on the first 1–2 lines of each dark band, x < 520.
  - A horizontal slip of the first ~18 lines of the top dark band.
- **Edges.** Soft horizontally (10–90 % over 9–12 px); band edges sharp (1–2 lines).

## What it does
1. **u (Q9).** g·(±(x − xc)) − g·s·y + scroll + curl + Bend·luma, wrapping mod 2^21.
   - The left family (−) holds to xc + 55 px in bright bands and xc + 75 px in dark bands.
   - In dark bands the left family adds a curl, 0.004·(x − xc)² px, past the apex: the hooks.
   - Mirror uses |x − xc| with the left table on both sides.
2. **Barcode.** Index = u/512 + 1024 (mod 4096), folded back at the table ends, into a 4096 × 8 ROM.
   - Address: side (1 bit) and index (11 bits).
   - Data: the bright code, the dark code and the corner flag.
3. **Band.** Per line, phase = roll + b·y + c·sin(2π(y − y0)/729) from a quarter-sine ROM. Bright where
   frac(phase) < duty (P12).
4. **Corner.** Bright bands that start above line 280 keep the warm top-left corner. Below that, orange
   becomes cyan and magenta becomes green there, so the corner edge always lies on a band boundary.
5. **Fringe.** A 7-px look-ahead window (5 at 720p, 4 at SD): a non-black dark-band pixel with black
   within the next F px turns pale yellow (on cyan) or yellow-green (on green).
6. **Tear.** The dotted line on the first 2 virtual lines of every dark band (x < 520). One dark band in
   four, picked by an LFSR, slips lines 2–19 by ±32 px; the slips re-roll every 32 fields.
7. **Palette and output.**
   - Two 8-entry palettes, luma pre-boosted 16/13 (drawn luma ≤ 759 after the output's 13/16).
   - [1 2 … 2 1]/32 on Y/U/V at 1080, /16 at 720p and SD (the photo's soft horizontal edges).
   - Luma 13/16, then the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Stripes | the nested A-frames | colourizer gain 0.35–3× (square law; 1.0 = the photo at 507) |
| K2 Slope | the A-frame's legs | leg slope 0.2–2.5 lines per px (square law; 0.9 = the photo) |
| K3 Rows | the horizontal bands | row-oscillator frequency, 3–40 bands per frame (square law; 12.3 = the photo) |
| K4 Squeeze | the bands thinning in the middle | FM depth 0–1.9 cycles: even bands at 0, the photo at 0.92, bands folding back near max |
| K5 Speed | stripes, bands, squeeze | bipolar, still in the centre ±8 (square law). The A-frames climb (6 px/field at full), the bands roll (0.02 cycle/field), the squeeze point drifts (4 lines/field). At the default one band rolls by in ~6.6 s |
| K6 Bend | the stripes bent by the picture | Video (or the Osc lattice) luma added to u, 0–360 px per full luma (square law; ~32 at the default) |
| S7 Fringe | the yellow fringes into black | On (the photo) / Off |
| S8 Mirror | the A's apex | the left family on both sides: a symmetric, seamless A |
| S9 Tear | the dotted sync glitch and slipped band tops | On (the photo) / Off |
| S10 Palette | the colours | Photo / Night (navy, blue, violet, amber, pink) |
| S11 Source | Video / Osc | Video (default) / Osc |
| P12 Shade | the dark bands | dark-band duty: 0 = the all-bright tartan, 50 % = the photo, 100 % = the all-dark tartan with yellow-edged black stripes. Glided 1/8 per field |

## Video
- The camera enters the patch the way an analog artist would. Its luma (unipolar, 8-px box) is added to
  the A-frame coordinate, so the stripes bend around the picture. Black adds nothing, so with no camera
  Video mode shows the photo.
- At the default Bend the photo's composition reads clearly, with the picture worked into the stripes.
  At 100 % the stripes shred around the image.
- In Osc mode a drifting 2-D triangle lattice (341 × 205 virtual px, 0..127, AC-coupled) feeds the same
  Bend path. At the default it gives a gentle ±11 px wobble.

## Engine
- **Vblank-zone sequencer** (zone = no avid for 1024 clocks), one small operation per zone clock:
  - a shared glide over K1, K2, K3, K4, K6 and P12;
  - one serial multiplier for 16 products: the knob squares × constants, g, s, g·s, the beam start
    g·(xc − x0), the beam step g·xs, b, c, the Bend gain and the three speed rates;
  - the motion accumulators (scroll, roll, modulator offset), duty and the LFSR seed.
- **Per-line engine** (kicked at each line start, computes the NEXT line, ~58 clocks), on one serial
  multiplier:
  - V·g;
  - b·y;
  - 90·y − θ into the quarter-sine ROM, then c·sin;
  - the phase (Q20) and the band type;
  - band starts: the LFSR draw, the slip and dot flags, the corner latch;
  - the line term.
- **Beam:** A = g8·(x − xc) by DDA (Q9).
- **Pixel path (32 stages, latency 32 in every mode):**
  - S1–S5: side, u, curl (8×8 product), Bend (two nibble partials).
  - S6: fold and ROM address.
  - S7–S8: EBR and fabric copy.
  - S9: code and corner.
  - S10–S16: fringe window.
  - S17: fringe and dots.
  - S18: palette, neutral outside the line.
  - S19–S30: conditioning and the gate.
- **ROMs:** barcode 4096 × 8, quarter sine 256 × 16, palette 16 × 30 (7 EBR in all). tartan.py (pure
  Python, the build hook) holds the run-length barcodes and generates tartan_pkg.vhd.
- **Contracts:**
  - virtual 1080-line coordinates (x in half virtual px, steps 2/3/4; SD shows the centre 1440 virtual
    px);
  - generated avid (parity-locked);
  - one 32-clock latency in every mode;
  - colours in the hardware convention (U = Cr, V = Cb).
- **Verification:**
  - imodel.py is bit-exact against the GHDL render at 720p60 with Speed still: mean |diff| 0.02, no pixel
    off by more than 24 after the 4-px generated-avid offset.
  - The default (Osc, moving) render matches the model to within 0.43, and the motion frames 6 fields
    apart at full speed to within 0.42.
  - The P12 glide matches.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Not touched: the source-side bend-tap running sum (v_box), which steers geometry, not colour. Post-route Fmax: HD Analog s2 76.79, SD Analog s1 74.11, HD HDMI s1 80.03, SD HDMI s1 77.73, HD Dual s1 84.27, SD Dual s1 87.11 MHz; 5717-5743 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-27, NOT flashed.** router2 (`MAKE_TIMEOUT=900 ./build_heirloom.sh tartan`).
- **Build seeds:** HD HDMI = seed 2 (seed 1 missed); the other five are seed 1.
- **Post-route Fmax:** HD Analog 83.21, HD HDMI 76.25, HD Dual 83.98, SD Analog 78.79, SD HDMI 78.43,
  SD Dual 71.97 MHz (SD Fmin 27).
- **Fit:** 5710–5727 / 7680 LC (74.5 %), 7 / 32 EBR. iceunpack OK × 6. tartan.vmprog 368,161 bytes.
- **If a later version gets tight:** HD HDMI is the thinnest (76.25); try
  `NEXTPNR_ROUTER="--router router1"` or another seed first.
- Presets are hand-written placeholders: Photo, Bright Tartan, Dark Tartan, Even Bands, Folded Night,
  Camera Weave.

## Deviations from the photo, and why
- **Centre junction.** The two stripe families meet at a vertical seam (xc + 55/75 px), with a curl
  standing in for the photo's hooks. The following are not drawn: the concentric "(" arcs (dark band at
  y 644–695 right of centre, and the top-left dark band), the magenta arch, and the solid orange and
  black blocks right of centre in the two widest bands.
  - The centre-right 4×4 cell is 22 RGB short of red.
  - Tried and rejected: a rounded apex, an additive two-family field, a second-triangle decomposition,
    slanted seams and a ripple envelope near the centre. None beat the barcode model by eye.
- **Top edge.** The fitted band oscillator puts a 13-line dark sliver at the very top. The photo's bright
  band starts at line 4.
- **Tear.** The slip is random per band (one in four); the photo shows one slipped band.
- **Edges and luma.** The yellow fringe is the brightest drawn colour (759 after the 13/16 output
  scale); nothing is capped.
- **Osc default.** In Osc mode the lattice gives a gentle ±11 px wobble that the still photo does not
  have: Bend stays live in Osc, as the method requires.
- **SD.** SD shows the centre 4:3 of the picture (the A-frame stays centred); 720p is identical to 1080p.
- **Motion.** The photo is a still, so the motion is designed: the A-frames climb, the bands roll and the
  squeeze drifts.

## Working files
`.heirloom_work/rb_tartan/`:
- `ref.jpg`, crops `c_*.png` / `z_*.png`, label maps `lab_*.png`, `chan.png`, `uview.png`, `bscat.png`.
- `rows.py` / `rowfit.py`: the band fit.
- `trans*.py`: colourizer order.
- `mkbar.py` / `common.py`: the barcodes.
- `tvfit.py`: the T/V fit.
- `split.py`, `hsplit.py`, `linesplit.py`, `dsplit2.py`, `bendfit.py`, `envtest.py`: the junction.
- `decomp*.py`, `addfit.py`, `softabs.py`: rejected models.
- `model.py`: the float proto. `proto_final.png`, `cmp_proto.png`.
- `sheet.py`: `sheet.png`.
- `imodel.py`: the integer model.
- `rtl_*.png`; `cmp_final.png`, `cmp_motion.png`, `cmp_p12.png`, `cmp_res.png`.
- `build3.log`, `notes.txt`.
- `old_v010/`: the v0.1 files.

## Open questions for Ron (2026-11-09 batch)
1. The Fringe on/off switch became Weave (the yellow fringe is now always on). Do you miss turning it off?
2. Peaks repeats the A four times with the mirrored (left) barcode. Would you rather have the peak count on a knob
   (2..6), or the unmirrored left / right barcodes with a seam at every valley?
3. The band slide rides on K5 past the photo's speed. Should it have its own knob instead, so the bands can slide
   while the climb stays slow (e.g. taking over K4 Squeeze's upper half)?
4. Shred re-rolls every ~2 s. Faster (a jittery tear), slower, or frozen until the switch flips?

## Hardware watch list
- Renamed: S7 Fringe → **Weave** (Plain / Twill); S8 Mirror → **Peaks** (One / Four; v0.5's Mirror lives on inside
  Four); S9 Tear Off / On → **Photo / Shred** (Off is gone; Photo = the old On). Presets: Even Bands, Folded Night and
  Camera Weave dropped; Herringbone, Mountains, Shredded, Slipstream and Argyle Night are new.
- K1's curve is now exponential: the same picture at the default, but positions away from it differ from v0.5.
- HD timing (router2, seed 1) has 6–14 % margin (see the build line), but router2 is nondeterministic: the first
  build of this design (before the slide-rate fix) gave HD Analog 75.45 and HD HDMI 75.10. If a rebuild misses, try
  another seed or router1 before restructuring.
- The K5 slide reads a glided K5 (1/8 of the gap per field), so it settles a moment after the knob stops; the climb,
  roll and drift still read the raw K5 exactly as v0.5. On the slow side of the photo speed (K5 324..700) the bands
  never slide.
- Cuts for fit: none.

## Next step
Wait for Ron's preview notes. Rebuild the diag with `.heirloom_work/diag_build.sh tartan`. On approval:
`./heirloom_diag.sh merge tartan`, then repack the previewed diag bitstreams under the production TOML (copy
`build/programs/ron/tartan_diag/rev_b/bitstreams/*.bin` into the production build dir, regenerate
`program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`) instead of re-routing;
update the production SUMMARY / README / PHASE2 row and commit only tartan's files.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: a slightly stronger camera effect at the top of K6, default K6 = 100 %. **Bend type**: K6 Bend adds camera (or Osc lattice) luma × gain to the A-frame coordinate u, so the stripes bend; the picture itself is never shown.
- The identical change to production tartan: the per-field Bend gain (K6² × 842 >> 20) is now g + g / 4 (c_cb 10 → 11 bits, 0 .. 1,050), so every K6 position is ×1.25 (0 .. 450 px per full luma, was 360); bend partials 15 bits, bend sum 19 bits (255 × 1,050 < 2^19). Latency unchanged.
- The TOML default (K6 initial 1023) and version were set by the orchestrator (v0.6.2).
- Build (router1, 2026-11-15): all six seed 1; HD worst 77.85 MHz (HD Analog 80.12, HD HDMI 87.02, HD Dual 77.85); 6,697 / 7,680 LC max. `out/rev_b/ron/_diag/tartan_diag_v0.6.2.vmprog`.
