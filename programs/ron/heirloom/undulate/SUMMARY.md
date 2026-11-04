# Undulate — green sine ripples bent into S-curves over a blue fold, red flecks crawling up their flanks

**v0.1.1 built 2026-11-04, not flashed.** Heirloom (video phase), from the clip `WIN_20230301_23_17_23` at 0.5 s
(`photos/vid/WIN_20230301_23_17_23_t0p5.png`). A field of green sine ripples sweeps from vertical stripes on the
left into S-curves, a saddle and fine horizontal lines at the top right, over a blue 45-degree fold that is black
in the bottom-left corner; red and pink flecks sit on one flank of every ripple, and the ripple brightness
breathes down the screen. Presets are hand-written placeholders.
v0.1.1: S10 Wave = Square is now a hard look-flip. The ripples become hard full-level green bars (48 % duty) and
the flecks hard red bars (28 % duty) in the gaps. Only the pulse ROMs changed, plus one ROM address bit.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_undulate/` (`notes.md`, `phasefit.py` .. `tabsolve2.py`, `bfit.py`,
`genv2.py`, `redamp.py`, `proto.py`, `imodel.py`).
- **The ripple is one phase field.** The local wavevector was measured on a 32-px grid (structure tensor +
  profile FFT, neighbour-consistent periods), and the phase was solved by weighted least squares on the local
  wrapped phases into a 60 x 34 node table (32-px cells).
  - Plane fit: phi = -2.581 + 0.009395 x - 0.01180 y cycles (x, y in px). The residual D runs -7.7 .. +9.2
    cycles; the ripple period runs from 23 px (top right) to 110 px (bottom left), ~30 cycles across the frame.
  - cos(table phase) correlates 0.73 with the still's high-passed green (0.84 for the per-pixel refined
    phase). Analog generators were tried first: sums, products and quadrature tensors of sines, and polynomial
    phases, reached only 0.07–0.40; a rank-3 separable phase leaves 0.17 cycle rms. So the bend is a measured
    table, scaled and modulated like an oscillator (the plane is the patch's ramp, D its undulation).
- **Green** is ((1 + cos phi) / 2)^2 (the phase-aligned profile), times a slow envelope (table, G peak 25 .. 244)
  times a vertical sine: the Y oscillator, period 222.5–225 lines, phase 168, depth 0.30.
- **Blue** is a folded 45-degree ramp: B = clip(36 + 131.8 |v| + 28.8 v^2 + 77.2 u), v = (X + Y) / sqrt2,
  u = (X - Y) / sqrt2, X and Y in -1 .. 1 (rms 7.7 per 32-px cell). Blue dips under the green peaks by ~25 % of G.
- **Red** has the green's period (median ratio 1.006 over 47 cells) and sits on its flank, offset from the green
  peak by 250 + 142 X - 12.5 Y degrees (the XY term fits to 2 degrees), within +-40 degrees of that centre.
  - Coverage 8.1 % (R > 128): 4 % on the left to 15 % on the right, peaking on the envelope maxima.
  - Per-line phase std 11 degrees, correlated over ~7–10 lines (the wiggly magenta lines at the top left).
  - Blobs: area p50 28 / p90 353 px, width 8 / 52, height 4 / 20; blobs over 200 px hold 77 % of the red.
- **Palette** (k = 12): blue (10,19,148) 15 %, black (11,19,18) 14.5 %, deep blue (12,18,83) 12.4 %, bright blue
  (5,26,212) 10.7 %, teal (8,77,113) 9.3 %, green (6,83,28) 8.1 %, cyan-green (10,115,180) 7.9 %, green
  (4,145,97) 6.6 %, pink (157,66,148) 5.7 %, bright green (10,189,23) 4.4 %, red (212,33,43) 3.5 %, yellow
  (141,128,17) 1.8 %. The picture is RGB-native: G = ripple, B = fold, R = flecks.
- Capture border: columns 0–2 and row 0 dark.

## Motion in the clip
- **The ripple field is static.** Cross-correlating the green+blue of the m-frames (t - 0.5 .. t + 1.5 s) in 12
  regions gives dx = dy = 0 (frame differences 8–17 levels, noise).
- **The red flecks crawl up the ripples**: the red mask moves dy -32 .. -52 px per 0.5 s (64–104 px/s) in the
  top regions, drifting 40–60 px along the stripes in the centre; the red fraction stays at 7.9–8.4 %.
- The clip's sibling stills are the same patch with Ron's knobs elsewhere: `23_15_53` (large red horizontal sine
  ripples with green rings) and `23_18_49` (vertical rainbow bands wobbled by a ~237-line sine), i.e. the X / Y
  oscillator frequencies and the shaper mixes were being turned.
- **The program:** at the default Flow the ripples stand still; the flecks scroll up 1.19 virtual lines per field
  (71 px/s at 60 Hz, 59 at 50 Hz) and their jitter re-targets per 8-line group as they go. Checked in the RTL:
  8 lines per 10 frames at 720p (7.9 expected), stripes static (correlation 0.999).

## What it does
1. **Phase.** phi = N plane + N U D(x, y) + Etch luma + flow (+ Wobble sin theta_y).
   - The plane is a Q20 DDA, and D comes from the 60 x 34 table, interpolated bilinearly.
   - N = Ripples, U = Undulate.
2. **Green** = E(x, y) (1 + depth cos theta_y) wave(phi). wave is ((1 + cos) / 2)^2; with Wave = Square it is
   a hard full-level bar of 48 % duty.
3. **Blue** = clip(36 + 144 |v| + 80 u) Blue - green / 4. The fold is a Q14 DDA along the line.
4. **Red** is a comparator: A(x, y) pulse(phi + delta) + noise > threshold.
   - pulse is ((1 + cos) / 2)^6; with Wave = Square it is a hard bar of 28 % duty.
   - delta = 0.335 + 0.394 X - 0.035 Y cycles, plus a per-line jitter of up to +-0.04 cycle.
   - The noise is two LFSR streams seeded per 8-line group, blended down the group and smoothed along the line.
   - Everything scrolls up 1.19 lines per field.
   - A is the fitted fleck-amplitude table (the still's coverage per 64-px cell).
5. **RGB to YUV** (luma 80 / 144 / 32, BT.601 within 4 %), then one [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V,
   luma 13/16, and the gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Ripples | the ripples' density (N on the whole phase): x0.4 .. x2.5, exponential; 512 = the clip |
| K2 Bands | the breathing brightness bands (the Y oscillator): 1.6 .. 14 bands per screen (exponential, 512 = 4.8), depth 30 % at the centre rising to 100 % at both ends (0 = one dark lower half, 1023 = a dotted lattice) |
| K3 Blue | the blue fold's level: none (black ground, the ripples alone) .. the clip (512) .. x4 (cyan-washed); square law |
| K4 Flow | bipolar (+-8 deadband, square law): the ripples stream through the field, up to ~1 cycle/s at the ends; centre = still, as in the clip |
| K5 Flecks | red fleck density (the comparator threshold 0.85 .. 0.55 .. 0.13): bare ripples (0) .. the clip (512) .. red flooding every flank (1023) |
| K6 Etch | how far the camera (or the Osc) bends the ripples: 0 .. 3 cycles per full luma, square law; 320 = 0.29 |
| S7 Flip | the ripple field (phase, envelope and fleck tables) upside down; the blue fold stays |
| S8 Palette | Neon (the clip) / Ember: the RGB cables swapped (red ripples, green fold, blue flecks) |
| S9 Wobble | the band oscillator (K2) also bends the ripples +-0.5 cycle: wavy stripes, toward the sibling still 23_18_49 |
| S10 Wave | the ripple and fleck waveforms: Sine (the clip's glowing ripples) / Square (hard-keyed neon: full-level green bars of 48 % duty with sharp edges, and solid red bars of 28 % duty in the gaps) |
| S11 Source | Video / Osc |
| P12 Undulate | the undulation's depth, x0 .. x2.2 (465 = the clip): 0 % = the bare ramp (plain parallel diagonal stripes), 100 % = the S-curves and saddle overdriven into whorls |

Glides (1/8 of the gap per field, +-2 deadband) on K1, K2, K3, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) - 16 from a one-line buffer, is added to the ripple
  phase at Etch: the ripples bend around the picture, while the clip's composition reads clearly at the default
  Etch. With no input, Video mode is the still.
- **Osc mode.** A drifting 2-D triangle lattice (640 x 540 virtual px) feeds the same etch path: the ripples
  wobble gently, and Etch scales it.

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier, 20 slots, the slot decoded two clocks
  ahead.
  - The exp2 ROM gives N (Ripples) and the band frequency.
  - It computes U, N U, the plane's x / y steps and base (Q20, with N), the band depth (square law), Blue,
    Flow, the fleck threshold and Etch.
  - Per mode: the plane / fold / red-offset start values and line steps.
  - It shares the glide with the knobs.
- **Per-line engine**: it computes the NEXT line on a 24-bit serial multiplier, in ~135 clocks (it fits SD's
  858-clock line).
  - The table rows and line fraction, and the plane start (c_base + c_syl y, plus flow and wobble).
  - The band oscillator: a cos / sin ROM, giving the band modulation (Q4) and the wobble.
  - Two hashed noise seeds (the 8-line group and the next) and the jitter target, eased per line.
  - The red-offset start. Line accumulators carry the fold (v, u - v) and the red offset.
- **Prefetch unit**: per 32-px cell, in 13 clocks, into lane P.
  - It reads the two table rows of a node (D 12-bit Q7, E and A 4-bit each) and interpolates vertically: D on
    5 bits of the line fraction, E and A in 8-line steps.
  - It scales D by N U on a radix-8 serial multiplier, E and A by the band modulation, and computes the slope
    to lane B, and x xs.
  - In hblank it fetches the line's first two nodes, and the lanes shift twice.
- **Beam** (forward DDAs): table coordinate, plane (Q20), red offset (Q20), fold v (Q14). The D lane restarts
  at every cell entry at lane B with lane P's slope. That is exact at a cell's first h, which is where the HD
  and SD lookup starts; at 720p a cell entry lands 0–2 h in.
- **Pixel path** (15 stages):
  - E / A horizontal interpolation (2-bit fraction).
  - Phase = plane + D, plus etch.
  - Pulse ROM addresses (Wave picks both tables' half).
  - Fold sum, then fold x Blue.
  - Envelope x pulse, amplitude x pulse, and the fleck noise.
  - Comparator, palette, then RGB to YUV.
  - The envelope / amplitude pair rides a 256 x 16 block-RAM ring from stage 3 to 8.
- **Output**: one [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V. The 8-back tap is a block-RAM ring, and the running
  sums snap back to black on 8 blank samples (resync). Then luma 13/16 and the blanking gate on the generated
  avid.
- **ROMs** (`undulate.py` writes `undulate_pkg.vhd`):
  - phase / envelope table 2048 x 16 (8 EBR) and fleck amplitude 2048 x 4 (2);
  - pulses: green 512 x 8 (sine | square) and red 512 x 8 (sine^6 | square);
  - cos 256 x 8 and exp2 256 x 16;
  - line buffer 2048 x 8 (4) and three rings (3). 21 EBR in all.
- **Contracts**:
  - the generated avid (parity-locked start, field-latched width);
  - one latency, C_LAT 20, in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr, V = Cb).
- **Verified**:
  - GHDL 720p, decimation 1 (v0.1.0; defaults are unchanged in v0.1.1): Osc and Video renders match the integer
    model `imodel.py`. The 3 x 5 region means agree within 1–2 levels, red coverage is 8.1 % against 8.2 %, and
    the remaining difference is the noise realisation.
  - Motion pair at decimation 2: see above.
  - The control sheet `sheet.png` is rendered from `imodel.py` (the RTL arithmetic); the S10 row is re-rendered
    for v0.1.1.

## Status
- **v0.1.1 built 2026-11-04 with router2 (the builder's default); all six configs on seed 1. Not flashed.**
- Post-route Fmax:

  | Config | Fmax (MHz) |
  |---|---|
  | HD Analog | 81.96 |
  | HD HDMI | 74.31 |
  | HD Dual | 90.33 |
  | SD Analog | 76.64 |
  | SD HDMI | 84.93 |
  | SD Dual | 80.35 |

- 7,054–7,072 / 7,680 LC (92 %), 21 / 32 EBR. iceunpack OK x6. The packed config is byte-identical to the
  v0.1.1 TOML.
- Output: `out/rev_b/ron/heirloom/undulate.vmprog`.
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh undulate` (or `./build_heirloom.sh undulate`).
- **HD HDMI's margin is thin (74.31 MHz).** It routed under a machine load of ~31; v0.1.0's near-identical
  netlist routed this config at 87.27. If a rebuild misses, re-route HD HDMI alone with seed 2–3 and repack with
  `vmprog_pack.py`, or rebuild under less load.
- v0.1.0 (2026-11-04, superseded): seed 1 everywhere; HD Analog 84.30, HD HDMI 87.27, HD Dual 85.06,
  SD 78.51–87.82 MHz.
- **Fit history**:
  - The first RTL multiplied N U D per pixel and had a Mirror switch (bidirectional DDAs): 8,574 LC (111 %).
  - Now D is pre-scaled per node in the prefetch and the beam is forward-only (Mirror became Wave).
  - The plane, offset and fold DDAs are narrowed, and the output is one box: the documented contract, where
    lozenge used two.
  - The YUV and fold constants use fewer signed digits, and the noise is blended at the IIR's input.
  - Standalone-yosys ablation guided the cuts (`notes.md`).
- No pin-skew screen was run. If HD HDMI shows lines, re-seed that config.

## Deviations from the photo
- **The fringe-birth band** (x 1250–1550, top half), where the still's stripes split as the period drops from
  40 to 23 px, can't be one smooth phase. The table smooths it, so the top right has ~20 stripes against the
  still's 25–29 along x = 1500–1700, and is coarser.
- **Fleck shapes** are blobs from a smoothed noise comparator: coverage and run lengths match (h 10 / 40, v 6 / 14
  against 9 / 30, 4 / 11). The still's flecks are more ragged and stringy along the stripes (camera and chroma
  artifacts).
- **Blue** is the fitted fold with simplified coefficients (144 |v| + 80 u, no v^2 term); cyan tints under the
  green come from that fold.
- **Softness and luma.** Horizontal softness is one 9-tap box (sigma ~2.4 px) against the still's ~1.6 px.
  Luma uses 80 / 144 / 32 weights, and drawn whites are ~777 after the 13/16 scale.
- **Small constant shears**: the phase is read 2 px late and the red offset 4 px late.
- **Not drawn**: the capture border, and the bright streaks in the still's left 40 px at the top.
- **Resolutions**: 720p shows the same geometry as 1080p (sub-pixel D shifts at cell entries). SD shows the
  centre 4:3 (x 264–1704 virtual px), so the left and right sixths are off screen.
- **Cut for budget**: the planned Mirror switch (replaced by Wave) and the red offset's XY term (2 degrees).
