# Tilework — a skewed grid of colour blocks: a staircase of rows, an inverted centre column, rainbow seams

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20230318_12_24_18` at 4.0 s
(`photos/vid/WIN_20230318_12_24_18_t4p0.png`). A grid of flat, saturated colour blocks: seven rows climb a staircase
(blue, orange, pink, lime, yellow, cyan) into a long green band, and a centre column carries every row inverted
(cyan, yellow, lime, pink, orange, blue, magenta). The column edges lean right going down and each one is a thin
rainbow. In the clip Ron turns one knob that slows both oscillators: the grid grows out of the top-left corner from a
fine green/magenta mesh (2.0 s) through the still (4.0 s) to one solid colour (4.4 s). Presets are hand-written
placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_tilework/`: `notes.md`, `measure.py`, `runs.py`, `vtrack.py`, `vtrack2.py`,
`htrack.py`, `edges.py`, `edgefit.py`, `edgefit2.py`, `hfit.py`, `proto.py`, `imodel.py`, `isheet.py`, `thincheck.py`.
- **Palette** (band means; k = 12 shares in brackets). The camera clipped every colour to a rail:
  blue (23,15,255) [9.8 %], magenta (215,5,255) [14.3 %], pink (255,5,151) [10.1 %], orange (255,74,0) [10.0 %],
  yellow (255,228,0) [10.1 %], lime (74,255,0) [10.4 %], green (4,238,103) [23.9 %], cyan (7,201,190) [10.1 %];
  the fringes are the remaining ~1.2 %. Flat areas have a std of ~1 RGB level (no noise to model).
- **The colouriser is a hue wheel.** In hue order the eight colours are blue, magenta, pink, orange, yellow, lime,
  green, cyan, about 45° apart. The side columns show wheel position W = L xor 1 for a 3-bit level L; the centre
  column shows 7 − W (all three bits inverted).
- **Rows = a quantized triangle.** Row boundaries at y 76.7, 184.6, 292.4, 402.1, 509.8, 619.7, 729.2 (step
  108.75 lines, sd 1.0). Tracked over the zoom (the alt t0p5, the 3.5 s frame, 40 frames of the clip), the vertical
  signal is a triangle through a quantizer that clips at level 7: interior trough bands are 2.00 steps tall (trough
  exactly at v = 1.0) and the green peak bands 4.1 steps (3.93–4.25 over 8 frames), so v = 1 + 8.05 · tri,
  L = min(7, floor v).
- **The reset.** The level-1 band after the reset is always ONE step tall and starts at y_s = step − 28..32 lines
  (15 frames, step 40 to 148; the still: 108.75 − 76.7 = 32). Above it the frame shows the wave carried over from the
  previous field's bottom: the top band has the bottom's level in 18 of 20 checked frames (the still: green).
- **Columns = a square wave, reset per line, with a keystone.** 16 edge fits (two edges x 8 rows) give
  phi = (x + 97)(1 − 0.024 y / 1080) / 711.1 − 0.184 half-periods, rms 1.74 px: the first edge at 1.184 h − 97
  (745 px), then every 711 px; the H frequency falls 2.4 % top to bottom (the right edge leans 30 px, the left 18 px).
  The first-edge law x1 = 1.18 h − 97 also holds over 9 frames of the zoom.
- **Fringes = the wheel sweeping the LONG way round.** Classified to the palette, all 14 column edges pass through
  5 or 7 wheel steps (d = 7 − 2W; when |d| ≤ 3 the sweep goes d ∓ 8). Each fringe dwells half way at the complement
  colour (blue→cyan shows a wide yellow, cyan→blue orange, orange→yellow cyan, yellow→orange blue): the half turn
  switches first. Fitted as a two-stage sweep over 32 px (the half turn over the first 30 %, a dwell, the rest over
  the last 20 %): rms 15.1 RGB over the edge windows, against 12–38 for a linear sweep.
- **Texture.** Row edges are hard (1–2 px); column edges are the 25–30 px rainbow plus the camera's blur. A 2–3 px
  dark capture border (not drawn).

## Motion in the clip
Measured on 181 frames (2–8 s) with `vtrack.py` / `htrack.py`:
- **Ron turns one knob that slows both oscillators.** The step grows 6 → 12 (2.27 s) → 25 (2.8 s) → 48 (3.33 s) →
  84 (3.87 s) → 106 (4.0 s, the still) → 190 (4.27 s) → one solid colour at 4.4–5.3 s (both oscillators stopped),
  then back to the fine mesh by 6.4 s. ln(step) rises ~1.3/s (doubling every ~0.55 s). Both resets anchor the growth
  at the top-left corner.
- **The rows zoom faster than the columns:** ln(step) against ln(h) has slope 1.56 over 8 frames; the alt t0p5
  (step 6.2, h 148) gives 1.82.
- **Untouched, the patch is static** (6.4–8.0 s: identical frames; both oscillators locked).
- The lean changes with the knob (the still leans right; 5.5 s leans left, twice as much).
- From 8 s on the clip is a different patch: stepped blue/cyan triangles, an H staircase compared against the V one,
  through the same wheel. S8 Stairs borrows its stepped edges.
- **The program:** P12 is that knob (log zoom, the rows at 1x, the columns at 0.55x); K4 Sweep adds Ron's hand as a
  triangle LFO on the zoom, zooming OUT first. At the default (400: ±0.46 octave, 7.8 s period) the integer model and
  two RTL captures 0.5 s apart show the 720p rows moving from 52/125/198/271/344 to 58/137/216/295 lines (+8 %, out
  of the top-left corner): the m-frames' direction, more gently than Ron's hand. K4 at full reaches ±3 octaves at
  0.28 Hz.

## What it does
1. **V staircase** (per line): r = (y − y_s) / step as a DDA in Q20 levels, wrapped at 16.1 levels; v = o + tri(r);
   above the reset (r < 0) the line holds the previous field's last value. The reset rides the zoom (y_s = step − 32).
2. **Camera / Osc** (per pixel): v += Bend · luma (Osc: the lattice centred on its mean, at 3x the camera's gain);
   L = clamp(floor v, 0, 7).
3. **H square wave** (per pixel): a phase accumulator phi (Q24 half-periods) from a per-line start phase and step;
   the keystone moves both per line. Stairs adds 0.12 half-period per level to the line's start.
4. **Wheel position**: W = L xor 1 (Plaid: xor a 4-step H staircase), 7 − W in odd half-periods. Within half a
   fringe width of an edge the wheel sweeps the long way (a 64-entry offset table, 32 steps across the edge).
5. **Colour**: a 256-entry hue-wheel palette (32 per step, RGB-linear between the 8 colours). Colours turns the
   wheel 3 steps.
6. **Output**: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (with the running-sum resync), luma 13/16, the
   blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Rows | the row height (the V oscillator), ±2 octaves: 3 giant rows (0) .. the still (512) .. ~25 thin rows (1023) |
| K2 Columns | the column width (the H oscillator): −0.8 octave, one edge at two-thirds of the width (0) .. the still (512) .. +2.5 octaves, ~13 narrow columns (1023) |
| K3 Level | the staircase's offset, ±3.5 levels: the trough clips into a long magenta/green band and the rows drop (0) .. the still (512) .. only yellow/cyan rows over a huge clipped green/magenta (1023) |
| K4 Sweep | Ron's hand on the zoom: a triangle LFO on P12, zooming out first; 0 = still, 400 = ±0.46 octave every 7.8 s (the default), 1023 = ±3 octaves at 0.28 Hz |
| K5 Skew | the keystone lean of the column edges: fanned left, curving edges (0) .. the still's slight right lean (512) .. fanned right, an edge sweeping into a curve (1023) |
| K6 Bend | how far the camera (or the Osc) bends the rows: 0 .. 4 levels per full luma (square law); 300 = 0.34 level |
| S7 Plaid | Off (the still) / On: each column becomes a 4-step staircase too, xor'd into the rows: a tartan |
| S8 Stairs | Straight (the still) / Stairs: each row shifts the column edges by its level: stepped chevrons (the clip's later patch) |
| S9 Prism | Thin (the still) / Wide: the rainbow fringes 8x wider: broad rainbow pillars |
| S10 Colours | Wheel (the still) / Turned: the hue wheel turned 135° (magenta/yellow tops, green/blue rows) |
| S11 Source | Video / Osc |
| P12 Zoom | both oscillators (the knob Ron turns in the clip), log: giant tiles, four rows and one column edge (0) .. the still (246) .. the clip's fine green/magenta mesh, step 6.4 lines and 149 px columns, as in its t0p5 (1023) |

Glides (1/8 of the gap per field, ±2 deadband) on K1–K6 and P12. The bipolar knobs (K1, K2, K3, K5) have a ±8
deadband at 512, which is exactly the still.

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 through the line buffer, is added to the staircase
  value before the quantizer, like patching the camera into the quantizer's input: the rows bend around the picture
  (bright areas step up the wheel). At the default Bend (300) the grid reads clearly on selfie.jpg (`cmp_final.png`,
  bottom); at full the camera is posterized through the wheel inside the grid. With no input, Video mode is the
  still.
- **Osc mode.** The shell's drifting 640 x 540 virtual-px triangle lattice feeds the same path, centred on its mean,
  at 3x the camera's gain: at the default Bend every row becomes a zigzag (±0.5 level), the troughs dipping into
  magenta wedges.

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier (17 slots; the slot number decoded two
  clocks ahead, operands pre-muxed), the shared glide over 7 channels, a 256 x 16 exp2 table. Per field: the sweep
  LFO and its depth; the zoom exponents e_v = e_P12 + sweep + K1 (capped so a row is never under 2 real lines) and
  e_h = 0.55 (e_P12 + sweep) + K2; the V step (levels per line) and the reset; the H step, the fringe gain, the
  keystone step and the start-phase step; the Level offset; the bend gain and the Osc centring.
- **Per-line engine** (8 clocks, kicked at each line start, computes the NEXT line): the V ramp (DDA, wrap), the
  triangle, the held top band, the staircase value (less the Osc centring); the keystone (H step and start-phase
  DDAs); Stairs.
- **Pixel path** (20 stages): beam phase; edge distance and the plaid quarter; d·G partials (6/6-bit split) and the
  bend nibble products; t and the bend sum; the line value + bend; the quantizer; wheel positions; the long-way
  offset table; wheel index; Colours; palette address, ROM, fabric copy; compose; two box filters; luma 13/16; gate.
- **Stores** (7 EBR): line buffer 2048 x 8 (4), palette 256 x 32 (2), exp2 256 x 16 (1).
- **Contracts:** damier's generated avid; one latency, C_LAT 20, in every mode; virtual 1080-line coordinates (SD
  shows the centre 4:3); colours in the hardware convention (U = Cr; `tilework.py` writes the palette); luma
  pre-scaled 16/13 and capped at 940; the box running sums snap back to black/neutral after 8 blank samples.
- **Verified (GHDL, 720p60 decimation 1, against `imodel.py` with the glide modelled), on the shipped sources:**
  - default frame, black input: bit-exact (100 % of pixels);
  - testA (Rows 700, Columns 300, Level 650, Sweep 1023, Skew 800, Plaid, Stairs): bit-exact;
  - testB (Zoom 1023 while gliding, Level 400, Skew 200, Prism, Colours): bit-exact (one revision earlier, logically
    the same datapath);
  - two captures 0.5 s apart (720p decimation 2) show the rows moving 52/125/198/271/344 → 58/137/216/295, matching
    the model's fields 3 and 33.
  - The first RTL render caught a narrowing-resize fold in the triangle (a 4-line blue line at the peak); fixed with
    a slice.
  - Integer model against the still at 1080p (field 0): mean abs RGB 3.0 (float proto 2.6); flat areas 1.8.
  - `thincheck.py`: no single-row line >= 24 px at the defaults, P12 0 / 100, every knob extreme, every switch, Osc,
    at 720p, 1080p and NTSC, at 4 sweep phases.

## Status
- **v0.1.0 built 2026-11-09 (ROUTER1), not flashed.** `out/rev_b/ron/heirloom/tilework.vmprog`.
  - Build: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh tilework`.
  - All six configs on seed 1. Fmax (post-route): HD Analog 94.48, SD Analog 90.38, HD HDMI 89.61, SD HDMI 89.29,
    HD Dual 87.61, SD Dual 87.53 MHz. 6917–6937 / 7680 LC (90 %), 7 / 32 EBR. Every bitstream passed iceunpack.
- **Timing history.**
  - The first router2 build missed HD HDMI on all six seeds (best 65 MHz) and needed seed 2 on HD Analog. The
    culprit was the per-field reset clamp: a negate and a 28-bit compare on c_r0's clock enable (15 ns). It is now
    three zone steps, and the Osc centring moved from the pixel path to the line engine. A private HD HDMI run then
    gave router2 81 / 77 / 83 MHz and router1 89 / 97 MHz on seeds 1–3.
  - router1 then stalled (remaining arcs frozen at ~10 k) on a LUT that saw e_incl[28] twice (a sign-extended
    x·3); that operand is now zero-extended (dupin: 0), and router1 closed HD HDMI on seeds 1–3 (89.6 / 92.2 /
    93.8 MHz). Stay on router1.

## Deviations from the photo
- The fringe colours follow a two-stage long-way sweep; the photo's exact dwell widths differ per edge (edge-window
  rms 15 RGB).
- The capture border is not drawn. The left 12 px fade in from black (the output boxes' warm-up); the column edges
  are pre-shifted 12 px left to compensate, so they land where the still has them.
- The top band holds the previous field's last value instead of carrying the wave on through the blanking
  (the clip agrees in 18 of 20 frames), so it never shows a 1-line sliver.
- The camera's soft ~13 px blur is replaced by the shell's 9-tap boxes on Y, U and V.
- The default motion (Sweep 400) is far gentler than Ron's hand in the clip, which zoomed all the way out in 0.4 s;
  Sweep at full comes close.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3: the column edges fall at 35 % and
  85 % of the SD width instead of 39 % and 76 %, so the right side column is narrower. At SD the zoom is capped at
  3.6 octaves (rows >= 2 field lines), so P12 above ~85 % no longer gets finer there, and the fine-mesh rows are
  coarser than at HD.
