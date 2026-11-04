# Tapers — candle-flame columns of neon mosaic rows rising through blue and maroon dashes

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom (video phase), from the clip `WIN_20230203_00_19_56` at
6.0 s (`photos/vid/WIN_20230203_00_19_56_t6p0.png`). A row of seven candle-flame columns ("tapers") rises from
the bottom of the frame, filled with horizontal mosaic rows of hot pink, lavender, sky, teal and green; above
their tips the columns are flat rectangles, and every 154 lines each column widens and steps back. The
background is a dark field of blue, maroon, black and navy mosaic dashes shaped by the camera scene (a solid
blue bar and a maroon bar at the left, a plant silhouette, a pink/yellow strip, a "lamp" of white and yellow
blocks at the top). Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_tapers/` (`notes.md`, `widths.py`, `fitT.py`, `periods.py`, `motion2.py`).
- **Tapers.** Centres at x = 48, 327, 622, 915, 1199, 1494, 1785: a cosine of period 289.5 px (6.63 per screen)
  with a peak at x = 48. Rounded tips and rounded wedge bottoms (a sine, not a triangle). Edges are soft over
  6–8 px.
- **Tip heights.** The 936/1202 tapers are the highest, then 621, 1494, 327, 1800, 48. Fitted as an additive
  horizontal V (a slow triangle oscillator summed with the carrier, not an amplitude envelope: a multiplicative
  envelope predicts a 204 px width where 116 is measured): D(x) = 0.0017·|x − 1000| − 0.5, so
  H(x) = cos(2π(x − 48)/289.5) − D(x). Offsets relative to the 621 taper: 1.10, 0.53, 0, −0.44, −0.20, 0.30, 0.87
  (V residuals ≤ 0.14).
- **Vertical threshold.** The width step-backs sit at y = 185, 337, 495, 649, 803, 955 (period 154.3 = 1080/7,
  the same rows in every frame of the clip: a frame-locked 7-per-frame sawtooth). Above the apex the columns are
  flat (130 then 88 px for the 621 taper); below it each band widens ~5.3 px per 10 rows and steps back ~25 px
  at every boundary. Fit (rms 0.04 in cosine units over 14 band ends):
  T(y) = 0.75·sin(2π(y − 56)/1380) + 0.35·(½ − frac((y − 185)/154.3)). The sine's average rising slope
  (0.33 per band) equals the saw's (0.35 per band), which is what turns the rising side into a staircase of
  rectangles. The key is H(x) > T(y).
- **Interior colour.** B is always high; R and G cross-fade with a vertical sine of period 206 lines (pink
  (238,52,255) at y = 220 + 206k, green (0,255,78) at 117 + 206k, lavender / sky / teal between), sampled per
  22-line mosaic row. There is no tilt (the 2-D autocorrelation peaks at dx = 0 for every dy) and no horizontal
  component: every well-lit block column sits at the same phase (±0.07 cycle); the 621 taper's odd rows and the
  right taper's quarter-cycle offset are the camera's mid level shifting the hue.
- **Background.** Blue (19,27,198), maroon (105,25,50), black (22,32,9) and navy (22,31,48) on the mosaic grid.
  Some columns are constant (the camera's bars and silhouettes), others cycle blue → maroon → black → navy at
  43.5 lines per step (period 174): two square-wave bits, blue = (1,0), maroon = (1,1), black = (0,1),
  navy = (0,0). At x = 440 both bits are slow (44-line blocks); at x = 300 and 1640 one bit toggles with the
  11-line half row (interlace), giving the fine blue/black and maroon/navy dashes; at x = 1720 the second bit runs
  the other way.
- **Mosaic.** 44.4 px × 22 lines (x edges at 149 + 44.4k, y edges at 87 + 22k).
- **Camera.** A static scene. Posterised colours: black (22,32,9) (a lifted black), navy, maroon, blue, pink
  (232,17,165), white (183,173,151), yellow (244,212,55). Bright blocks (the lamp) show through the tapers.

## Motion in the clip
- The threshold sine drifts **up 440 lines/s** (the fit gives ys = 256 at t − 0.5 s and 56 at t): the tips rise,
  the tapers fill the frame in about 2 s, and a new row of tips appears at the bottom (strip frames 2–3, m3).
  The 7-per-frame sawtooth does not move.
- The interior hue rows scroll **up 128 lines/s** (64 lines per 0.5 s mod 206, correlation 0.99).
- The background cycle scrolls **up 160 lines/s** (80 per 0.5 s mod 174, correlation 0.94–0.99).
- The camera and the taper x positions are static.
- The program's defaults reproduce these three rates at their ratios (K4 Rise drives all three; 1080p runs the
  per-field drift at 2× to keep the rate per second).
- The clip's other still (t = 0.5 s) is the same patch at another phase of the slow sine (tall tapers), not a
  knob change; the 00_04_08 clip is a different patch from the same session (the same colouriser family).

## What it does
1. **Key.** Per pixel H = cos(φ) − D with φ a phase accumulator (K1) and D a triangle DDA centred on
   x = 1000 (K2); per line T = 0.75·sin(ψ) + Steps·(½ − saw) + Height, with ψ and the saw phase accumulators
   (ψ drifting per field by Rise). The key is m = H − T, blended over ±1/16 (about 6 px) in five steps.
2. **Inside.** A 32-entry LUT (pink → lavender → sky → teal → green → … → pink) indexed by the hue phase
   accumulator (K5), sampled on the first line of each mosaic row (S10 Rows) and held; the camera's level 2 adds
   a quarter cycle and level 5 a half; levels 6 and 7 show the camera colour instead.
3. **Outside.** The camera luma (or the Osc) in 8 levels, sampled at every block start (44 virtual px, K6) on
   the first line of each block row and held in a 256-entry row memory for the other lines: black, navy, the
   dashed cycle (one bit = the half-row parity), the block cycle (two square waves from the background phase
   accumulator), maroon, blue, pink, yellow.
4. **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the HD HDMI bar fix and the clip's
   softness), luma 13/16 (palette luma pre-scaled 16/13, capped at 940), then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Tapers | the number of columns across the screen: 2.3 (big bells) .. 6.63 (512, the clip) .. 19 (thin spikes), exponential, anchored at the centre |
| K2 Bowl | how much the outer tapers are shortened (the slow horizontal triangle): 0 = all the same height, 341 = the clip, 1023 = 3×, only the middle ones rise |
| K3 Steps | the 7-per-frame sawtooth level: 0 = smooth bells, 358 = the clip's flat rectangles and step-backs, 1023 = deep stair steps, every band restarting from a tip |
| K4 Rise | bipolar (±8 deadband, square law): the tips rise (right) or sink (left) and the colour rows and the background cycle scroll with them at the clip's ratios; 776 = the clip (440 lines/s), full = 1500 lines/s |
| K5 Hue cycle | the interior colour cycle's period: 582 lines (0, long sweeps) .. 206 (512, the clip) .. 73 lines (1023, fine stripes); the background cycle follows at its 174/206 ratio |
| K6 Mosaic | the block size: 9×5 (0) .. 44×22 (512, the clip) .. 211×106 virtual px (1023); it sets the camera blocks and the colour rows together |
| S7 Flip | Rise (the clip) / Hang: the tapers hang from the top and the whole motion mirrors |
| S8 Invert | swap the palettes: dark tapers on the striped field |
| S9 Palette | Neon (pink / green, the clip) / Amber (amber / cyan with a navy-blue / brick background) |
| S10 Rows | Mosaic (colour rows sampled per block row, the clip) / Smooth (continuous gradients) |
| S11 Source | Video / Osc |
| P12 Height | the threshold offset: 0 % = a row of small flames at the apex sweeping up the screen, 500 = the clip, 100 % = a solid striped wall with dark notches hanging between the merged tapers |

## Video
- **Video mode (the default).** The camera luma is quantised into 8 levels, (Y − 64)·9/8 in steps of 114
  (the top level from Y = 860), and mosaic'd by the block S&H. Outside the tapers it chooses the background
  colour: dark areas are black and navy, mid-dark areas get the dashed and block cycles, brighter areas maroon
  and blue, the brightest pink and yellow — the clip's posteriser. Inside the tapers the mid level (2) shifts the
  hue a quarter cycle (the clip's sky bands), level 5 a half cycle, and levels 6–7 show through (the clip's lamp
  blocks); so a bright busy scene weaves into the rows instead of burying the tapers. With no input
  the background is black with the hue rows in the tapers.
- **Osc mode.** A generator with the camera's statistics replaces the luma: two horizontal triangles (610 and
  1330 virtual px) and a weak vertical one (900 lines), quantised to the same 8 levels — vertical bars of every
  level, mostly dark, drifting slowly. Every video-path control (K6, the palettes, Invert) stays live.

## Engine
- **Per-field sequencer (vblank zone).** One serial shift-add multiplier, 12 slots (carrier step, phase at x0,
  bowl slope and start, rise², drift, hue step per virtual line and per line, background step, sine step, band
  step, block width), a 256-entry exp2 ROM for K1 / K5 / K6, a shared glide (K1, K2, K3, K5, K6, P12). The
  field-start phases of the threshold sine, the hue wave and the background cycle start at the clip's and
  accumulate the drift (±, ×1 / ×2 / ×3) every field.
- **Per-line engine.** At each line start the current line takes the next-line registers, then the four phase
  accumulators advance (backwards under Flip), the row counter wraps at the block height (the sampling line and
  the half-row flag), and the sawtooth level is multiplied on its own serial multiplier. The threshold sine is
  one read of the shared sine ROM 5 clocks after the generated avid falls (in the zone for line 0).
- **Beam.** Per pixel: the carrier phase (24 bits, Q4), the bowl DDA (±slope, direction from x < 2000 h), the
  block counter (Q4 h), the Osc phases, and the level S&H (the level of the input pixel at every block start;
  written to the row memory on sampling lines).
- **Pixel path (21 stages).** 2 ROM/RAM addresses, 3 data, 4 fabric copies and the level mux, 5 cos·1024 − D,
  6 − T, 7 soft weight and palette addresses, 8 palette ROMs, 9 fabric copies, 10 camera override and Invert,
  11 inside − outside, 12 × weight (0..4), 13 blend, 14 compose (palette black off the generated avid),
  16–19 the two boxes, 20 luma 13/16, 21 gate (the beam registers a pixel one clock after its g_avid clock, so
  the shell's avid indices sit one stage earlier than in lozenge).
- **ROMs / RAM.** Sine 1024×12 (3 EBR, shared), exp2 256×16 (1), inside and outside palettes 256×32 (2 + 2),
  the camera row memory 256×4 (1, `no_rw_check`: the sampling line reads the x-hold, never the memory).
- **Contracts.** The generated avid; one latency (C_LAT 20) in every mode; virtual 1080-line coordinates (SD
  shows the centre 4:3); the running-sum resync on both boxes; colours in the hardware convention (U = Cr).
- **Verified (GHDL 720p60, decimation 1, real timing ID).**
  - Synthetic camera (the still's mosaic blocks encoded as 8 grey levels), Rise still: the RTL frame against
    `imodel.py` has a mean |RGB diff| of 0.62, 99.5 % of pixels within 8 levels and 100 % within 16, once the
    model is rolled 3 px right (the tester captures at a fixed 20-clock delay from the source avid while the
    generated avid starts 3 clocks later; every heirloom program sits there).
  - Osc mode: mean |diff| 5.2, 89 % within 8 (the generator's level is sampled 3 clocks later than the model's).
  - Real photo (`photos/selfie.jpg`): the tapers keep their composition with the photo woven in.
  - Motion: with Rise at its default (776) the RTL frame captured after two fields equals the model's frame 2
    exactly (mean |diff| 0.62, key / hue / background correlations 1.000): the tips 8.6 physical lines higher at
    720p (4.3 per field = 440 virtual lines/s), the hue rows 2.6 and the background cycle 4.0 lines, all upward.
  - Knob sweep at the RTL's widths (34,992 settings over all timing modes): no sequencer or line value out of range.

## Status
- **v0.1.0 built 2026-11-04, NOT flashed.** `out/rev_b/ron/heirloom/tapers.vmprog` (423,043 bytes).
  - Build: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh tapers` (router2, the builder's default).
  - **Seeds: HD Analog = seed 2** (seed 1 missed timing); the other five on seed 1.
  - Fmax (post-route): HD Analog 86.84, SD Analog 87.85, HD HDMI 84.42, SD HDMI 85.90, HD Dual 91.27,
    SD Dual 80.92 MHz. 6296–6308 / 7680 LC (82 %), 9 / 32 EBR. iceunpack OK on all six.
  - Standalone yosys: 3308 LUT4, 2688 FF, 1321 carry, 0 transparency emulation (the camera row memory carries
    `no_rw_check`).
  - To rebuild from source: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh tapers` (or
    `./build_heirloom.sh tapers` from the repo root). Do not edit the sources while a build runs: the builder
    reads them per config.
  - Presets are hand-written placeholders. Capture Ron's looks with `./capture_preset.sh`.

## Deviations from the clip
- The camera enters only through the 44 × 22 mosaic: the clip's smooth-edged silhouettes (the blue bar's
  rounded bottom, the plant, the pink/yellow strip's curved edge) become block-edged. The scene itself is a
  live camera on hardware, so only the texture of the edges differs.
- The clip's background columns show three cycle behaviours (44-line blocks, 11-line interlace dashes, the
  reversed order); the program gives the dashed and the block cycle as two camera levels and drops the
  reversed order.
- Inside the tapers only the brightest camera levels (6, 7: pink, yellow) show through; the clip's lamp also
  carries a few black and blue blocks inside the 936 taper's edge, which here stay in the hue rows.
- The lamp's white (183,173,151) is rendered as the yellow level (the 8-level posteriser has one top colour).
- The threshold sine's rising side is a staircase here because the saw's slope equals the sine's average rising
  slope at the default Steps; the clip's rectangles are flat to ±2 px, the model's to ±0.1 of the saw step
  (a shallow bow of ~6 px over a band), as the clip's own band 1 shows.
- The first 8 px of each line are the output boxes' warm-up from the palette black (the clip's capture border
  is ~3 px). The white cores are capped at ~777 output luma by the shell.
- **Resolutions.** 720p shows the same geometry as 1080p (verified). SD shows the centre 4:3 (virtual x 240 ..
  1680): the x = 48 and x = 1785 tapers fall outside, and the per-field drift runs at the 60 Hz rate (1080p at
  2× per frame, so the rates per second match). 1080i treats each field as a frame of 540 lines (field_n is
  tied high on every input); the block rows are 11 field lines there.

## Working files
`.heirloom_work/rb_tapers/`: `notes.md` (the reading and every measurement), `widths.py` / `fitT.py` /
`periods.py` / `motion2.py` / `tilt.py` (measurements), `proto.py` + `proto_final.png` + `cmp_proto.png`
(the float replica), `sheet.py` + `sheet.png` (every control at min / default / max), `imodel.py` (the integer
model), `mkcam.py` + `cam_synth_720.png` (the synthetic camera), `rtl_*.png` (GHDL renders),
`cmp_final.png` (still | proto | RTL Osc | RTL Video), `build_v0.1.0.log`.
