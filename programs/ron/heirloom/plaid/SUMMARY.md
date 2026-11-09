# Plaid — yellow-to-red colour cells woven between black weft bars over bands of hot-pink stripes

**v0.1.0 built 2026-11-09, not flashed.** Heirloom phase 3 (video clips), from the clip `WIN_20231204_23_13_02`
at 5.5 s (`photos/vid/WIN_20231204_23_13_02_t5p5.png`). The still is a woven plaid. A smooth colour field
(yellow and white on the left through orange to red on the right; warm at the top, a cool white / cyan / teal zone
low in the middle) shows through a grid of cells. The cells sit between black weft bars and warp gaps. Wherever a
cell is closed, a back layer shows: bands of crimson-to-hot-pink sawtooth stripes, curved in S-waves, each band
starting on a dead-straight left edge, with black between the bands. In the clip the weft bars slide down, the
colour field scrolls down with them, the warp gaps pulse, and the magenta bands drift sideways and breathe.
Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_plaid/` (`notes.md`, `masks.py`, `gapw.py`, `backkey.py`, `backpal.py`,
`gfit.py`, `gbil.py`, `hough.py`, `orfit.py`, `proto.py`, `imodel.py`).
- **Palette** (palfit k = 14):
  - black 22 %;
  - red (241, 21, 2) 13 %;
  - crimson (234, 3, 81) 8 %;
  - gold (234, 154, 13) 8 %;
  - dark crimson (145, 3, 50) 7 %;
  - hot pink (254, 4, 134) 5 %;
  - cream (217, 220, 170) 5 %;
  - teal (61, 156, 142) 3 %.

  By pixel class the still is 25 % black, 24 % magenta and 50 % colour field.
- **Three layers, keyed like a weave.** front = (open row) AND (open column); else the back layer (magenta bands,
  or black).
- **Warp (columns).** The gaps are fixed in x at 35, 300, 579, 858, 1135, 1424 and 1700 (period 277.5 px, 6.9
  across). Each gap is 60–110 px wide (mean about 80 px, 29 % of the period). The centres stay put while the
  widths vary down the frame: a PWM'd pulse.
- **Weft (rows).** The still's bars are at 36–80, 120–158, 366–414 and 834–998.
- **Magenta bands = a hard-synced oscillator under a chirped master.**
  - The band left edges (the syncs) fit φ(x) = 1.4355e-3 x + 3.36e-7 x² (syncs at 85, 676, 1163 and 1587; the
    period runs from 697 px down to 369 px across the frame).
  - The band ends are at 350, 891, 1410 and 1892, so the duty rises from 0.42 to 0.80 left to right.
  - Inside a band, rows 850, 900 and 950 show one fixed stripe sequence scaled 255, 245 and 228 px. So the stripes
    are a palette over a ramp started at the sync: bump, dip, pink peak, deep trough, peak, shallow trough, a long
    ramp up to hot pink (255, 0, 170), a sharp fall, then black (`backpal.py`, median of 48 rows).
  - The S-waves come from a per-line phase offset.
- **Colour field.**
  - It has a vertical period of about 1100 lines (fitted on palette-bin variance).
  - It has vertical seams at x = 652 and 1556, 25–30 px left of the band syncs. The master's ramp is added into the
    colour index there, which tilts the colour bands into diagonal "rays".
  - It is fitted as a 32 × 32 (X, W) grid read bilinearly: rms 11.4 / 8.2 / 5.3 RGB on the visible cells
    (`gbil.py`, least squares with a smoothness term that fills the W rows hidden behind the bars).
- **Edges.** The cell edges are soft (about 6 px). Every cell has a dark-green fringe (0, ~30, 0), 8–15 px wide,
  on both vertical sides where it meets black (luma wider than chroma). Horizontal edges have none.

## Motion in the clip
The whole clip was re-extracted at 960 × 540 (30.07 fps) for kymographs (`kymo_*.png`).
- **The weft slides down.**
  - A Hough transform on 2693 bar edges finds two velocity families: slow at 152–180 lines/s (median 165,
    constant-width bars of 36–64 lines) and fast at 290–334 lines/s.
  - Bars with one slow and one fast edge grow or shrink; bars appear as slivers mid-screen and split.
  - Fitting two rigidly scrolling row tables combined by OR matches 85.6 % of the clip's row key (165 and
    334 lines/s); periodic pulse waves only reach 74 %.
- **The colour field scrolls down.** The top of the cool zone moves 480 → 560 → 700 → 780 → 840 lines over
  t − 0.5 … t + 1.5 s, about 160 lines/s around the still. That is the slow weft's speed.
- **The warp gaps** stay fixed in x. Their width pulses with a period of about 0.38 s.
- **The magenta bands drift.**
  - Their edges move at 40–90 px/s and reverse every 2–4 s, with neighbouring edges moving in opposite directions.
  - The fit at the still gives dc/dt = −0.18 cycles/s for the master phase and db/dt = 1.8e-7/s for the chirp.
- **Other looks of the patch** (t = 0.5, 3.5, 7.5 s) are the same patch at other scroll phases: the cool zone high,
  many bars, the cool zone wrapped to the top and bottom.
- **What the program does with it:**
  - At the default K3 the slow weft and the colour field scroll at 165 lines/s and the fast weft at 330 lines/s,
    starting exactly from the still's frame.
  - The gap ripple scrolls up with a 0.76 s period.
  - The band phase rides a 7.2 s triangle LFO (0.18 cycles/s), and the stripe S-waves drift with a 3.1 s period.
  - The RTL-exact model at +0.5 / +1.0 / +1.5 s moves like the m-frames: the bars descend and the cool zone reaches
    the bottom by +1.5 s (`cmp_motion.png`).

## What it does
1. **Weft.**
   - The bar test is (slow table[y·dens − o1(t)] + bias > 0) OR (fast table[y·dens − o2(t)] + bias > 0).
   - The tables are signed-distance ROMs (lines): the slow one has 2 lines per entry over a 2048-line loop, the fast
     one 4 lines per entry over a 4096-line loop.
   - They hold the still's bars at t = 0 and the clip's next bars after it (`plaid.py`).
2. **Warp.**
   - The gap test is |frac((x − 35)/277.5·dens + ½) − ½| < hw(y, t).
   - hw = hw0 (1 ± 0.11 sin), a sine per line that scrolls up.
   - The fringe is the gap's outer 10 px: dark green, drawn where the back is black.
   - Twill adds one virtual px of shift per line.
3. **Master.**
   - φ = a·x + b·x² + c(t), as a second-order DDA per pixel.
   - A band is lit while frac(φ) < duty(x), where duty is a saturating ramp from 0.29 to 0.70.
   - The colour seam is frac(φ + 0.05)/16, added into the colour index.
4. **Slave.**
   - r restarts at every wrap of φ (hard sync), loaded with δ(y): ±0.22 cycles over 760 lines (Waves) or ±2 cycles
     over 1080 lines (Arcs).
   - Otherwise r += k per pixel.
   - The colour is a 256-entry stripe palette over r: 230 stripe entries, then black.
5. **Colour field.**
   - W = y/1100 + seam + ψ(t) + dye·luma.
   - Luma is bilinear on a 32 × 32 grid (four parity-split ROMs).
   - Chroma is linear in W, read at an X dithered with period 8, which the output's 8-sums cancel exactly.
6. **Compose.** front = NOT bar AND NOT gap (Layer inverts it). Otherwise the stripes where the band is lit, else
   the fringe, else black. Orchid rotates the chroma 90°.
7. **Output.** U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes, Y through [1 2 1] aligned to them, then luma
   13/16 and the gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Warp | Warp gap width (the black / striped columns between the cells). 0 = no gaps: the cells merge into horizontal colour bands. 512 = the still (about 80 px of every 277.5, 29 %). 1023 = 89 %: the cells shrink to tapered slivers in a magenta curtain |
| K2 Weft | Weft bar height, ±60 lines. 0 = no bars, only the warp columns. 512 = the still. 1023 = the bars swallow most of the cells |
| K3 Shuttle | Bipolar, ±8 deadband at 512. The weft slides down (+) or up (−). The fast weft moves at twice the rate, and the colour field scrolls with the slow weft. 820 = the clip (165 lines/s); 1023 = 2.8× |
| K4 Bands | Magenta band rate (the master's frequency, with its chirp scaled to match). 0 = ×¼: one or two wide bands. 512 = the still (about 3.7 bands, period 697 → 369 px). 1023 = ×4: a dozen narrow bands |
| K5 Stripes | Magenta stripe density (the slave's rate). 0 = ×0.35: fat stripes, part of one cycle per band. 512 = the still (one cycle per 237 px). 1023 = ×4: fine stripes |
| K6 Dye | How far the camera (Video) or the Osc lattice shifts the colour field's index: 0 .. 1.8 colour cycles per white (square law; 512 = 0.45) |
| S7 Layer | Cells (the still) / Bands: the weave inverted, with stripes in the cells and the colour field in the bars and gaps |
| S8 Arcs | Waves (the still's S-curved stripes) / Arcs (a ±2-cycle offset across the frame: steep sweeping arcs) |
| S9 Colours | Ember (the still) / Orchid: every chroma turned 90°. Yellow → pink, red → magenta, teal → green, the stripes → violet-blue |
| S10 Twill | Plain (the still) / Twill: the warp leans 45° (one virtual px per line), giving parallelogram cells |
| S11 Source | Video / Osc |
| P12 Weave | Plaid density, warp and weft together (glided). 0 = loose (2.3 columns, bars 3× taller). 410 = the still. 1023 = fine tartan (×4: about 28 columns, thin weft) |

`sheet.png` (from the float proto with the final control map) shows every control at min / default / max and P12
at 0 / 50 / 100 %. All five switches change the whole frame with every other control at its default. Every knob
stays visible at any P12.

## Video
- **Video mode (the default).**
  - The camera's luma is (Y >> 2) − 16, smoothed 1/4 per pixel, through a one-line buffer at half horizontal
    resolution.
  - It is added to the colour field's index at Dye's gain, as a patch cord into the colouriser would be.
  - The camera then appears inside the cells, recoloured through the Ember grid (warm to cool). The grid, the bars
    and the magenta bands are untouched, so the plaid reads clearly (`cmp_final.png`, bottom panel).
  - With no input, Video mode is the still: the RTL is bit-exact with the integer model.
- **Osc mode.** A drifting triangle lattice (320 × 270 virtual px, the shell's oscillator) feeds the same path.
  The cells fill with chevron ripples of the colour field. Dye stays live.

## Engine
- **Per-field sequencer** (vblank zone = no avid for 1024 clocks).
  - It has a shared glide (K1, K2, K4, K5, K6, P12) and an exp2 ROM.
  - One serial shift-add multiplier runs 18 slots. The operands are pre-muxed; mode constants are concurrent and
    static.
  - Per field it computes:
    - the weave density, band rate and stripe rate;
    - the column step and line-start phase;
    - the fringe width;
    - a·bnd, bnd², the chirp b·bnd², f0, cm and φ0 (with SD's x0 = 240 terms on the multiplier);
    - the slave rate;
    - the dye gain;
    - the speed² → weft step;
    - the weft step per line;
    - the twill step.
  - The time phases update per field: the weft offsets, the colour scroll, the gap-ripple phase, the stripe-wave
    phase and the master LFO (a 24-bit bounce).
- **Line engine.** It is kicked when the generated avid falls (and once in the zone for line 0) and computes the
  next line in about 27 clocks of horizontal blanking, which fits SD's 138 and the tester's fake 64. It reads two
  sines from a 512-entry quarter-wave ROM and runs two products on an 8-bit serial multiplier, then writes the line
  registers directly: the gap half width and fringe edge, the stripe-phase offset, the weft table offsets, the
  colour W and the twill column phase.
- **Beam DDAs.** Warp phase (24-bit), master phase (33-bit) and its step (30-bit), duty (20-bit, saturating), the
  slave ramp (24-bit, reset on the master's wrap), colour X (22-bit) and the luma read address.
- **Pixel path** (C_LAT = 20, 21 stages):
  1. Beam.
  2. Warp distance, master high (from the previous pixel's phase, so the band edge meets the slave reset), seam,
     colour X and its dither, luma.
  3. Gap and fringe tests, W = line + seam, dye partial products, weft fabric copies.
  4. Weft bar, dye sum.
  5. W + dye.
  6. Six grid ROM addresses.
  7. EBR.
  8. W-parity swap.
  9. W differences.
  10. × aw.
  11. Add.
  12. X-parity swap and difference.
  13. × ax.
  14. Colour (10 bit); stripe palette.
  15. Compose.
  16–19. Two chroma boxes; the 8-back samples come from two 256-deep block-RAM rings (`no_rw_check`, read 6
      behind the write, plus a fabric register), with the running-sum resync on a blank history.
  17. Luma [1 2 1].
  20. 13/16.
  21. Gate.
- **ROMs / RAMs (22 EBR):**
  - luma grid 4 × 256 × 8 and chroma grid 2 × 512 × 16;
  - stripe palette 256 × 24;
  - weft tables 2 × 1024 × 8;
  - exp2 256 × 16;
  - quarter sine 512 × 8;
  - line buffer 1024 × 8;
  - delay rings 256 × 30 and 256 × 20.

  `plaid.py` (pure python, the tables embedded) writes `plaid_pkg.vhd`.
- **Contracts:**
  - generated avid (parity-locked start);
  - one latency (20) in every mode;
  - blanking gate on the latency-aligned generated avid;
  - output conditioning (chroma 9-tap boxes, luma [1 2 1], luma 13/16) before the gate;
  - virtual 1080-line coordinates (SD shows the centre 4:3 via x0 = 240);
  - colours in the hardware convention (U = Cr, V = Cb);
  - box resync.

  dupin: 0 `CARRY I0==I1`.
- **Verified:**
  - check (with the pkg) is clean.
  - GHDL at 720p60, decimation 1, black input, default knobs: **100.00 % bit-exact** with `imodel.py`.
  - A later frame (720p60, decimation 2, nine fields in): 100.00 % bit-exact, so the per-field scroll / LFO / phase
    logic is checked too.
  - That sim caught a real bug before the final build: the box history registers took the avid one stage late,
    dropping each line's first pixel from the running sums and leaving a constant chroma tint.
- **Fit history.**
  - The first cut was 10,490 LC (136 %): a bilinear 32 × 64 grid, multi-operand sequencer sums, Y in the boxes and
    a per-line slave taper.
  - Ablations measured the costs: bilinear grid 1590 LC, sequencer sums 1130, boxes 900, line engine 555.
  - The fixes were: luma bilinear with chroma dithered in X; sums moved onto multiplier slots; chroma-only boxes on
    block-RAM rings; the line engine writing straight into the line registers; the slave taper, rate cos and two
    LFOs dropped. Result: 7035 LC.

## Status
- **v0.1.0 built 2026-11-09, not flashed.** ROUTER1, **all six configs on seed 1**. Post-route Fmax:

  | Config | Fmax |
  |---|---|
  | HD Analog | 80.66 MHz |
  | SD Analog | 88.39 MHz |
  | HD HDMI | 84.53 MHz |
  | SD HDMI | 74.32 MHz |
  | HD Dual | 92.82 MHz |
  | SD Dual | 84.37 MHz |
- 7019–7046 / 7680 LC (92 %), 22 / 32 EBR. iceunpack OK ×6, and the packed config carries the labels.
- Output: `out/rev_b/ron/heirloom/plaid.vmprog` (484,892 bytes).
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh plaid`.
- Presets are hand-written. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **Left edge.** The still's x = 0–36 strip, the tail of a band synced off-screen, is black here. The program's
  first band starts at the x = 85 sync.
- **Stripe detail.** One stripe palette (measured on bands 676 and 1163) serves every band. In the still each band's
  inner stripes differ slightly; band 1587 shows a tail and then a second cycle. The S-waves are one sine per line;
  the still's top-left arcs are steeper.
- **Weft sequence.** The still's frame is exact at t = 0. After it the weft follows the OR-of-two-tables fit, which
  matches 85.6 % of the clip's row key, then loops (12.4 s for the slow table, 12.3 s for the fast one).
- **Motion simplified.** The chirp b is constant: the clip's band breathing is carried by the c LFO alone, so the
  bands slide rather than spread. The slave-rate LFO, its per-line cos and the per-pixel slave taper were cut for
  area. The stripes have one width across a band, and the bands still widen to the right through the duty ramp
  and the chirp.
- **Colours.**
  - Whites come out at about Y 777 (the shell's cap before 13/16) against the still's saturated camera whites.
  - Chroma is read at a dithered X, invisible after the boxes. Luma is fully bilinear.
  - The grid smooths hidden rows by interpolation, so the cool → warm turn under the big bottom bar is inferred.
- **Edges.** Cell edges are clean: the still's ±2 px per-line raggedness is not drawn. Luma uses [1 2 1], so it is
  crisper than the still's roughly 6 px softness; chroma has the 9-tap softness.
- **Offset.** The output lands about 12 output px to the right of the beam coordinate (the causal boxes), as in the
  other phase-2/3 shells.
- **Additions beyond the clip:** Layer, Arcs, Orchid, Twill and the Weave density.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (x 240–1680 virtual px) with the same rows, columns and bands. It is softer, because
    the boxes work per output pixel.
  - P12 near 100 % (fine tartan) aliases at SD.
