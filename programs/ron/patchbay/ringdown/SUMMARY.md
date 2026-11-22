# Ringdown — a square wave through eight resonant filters

**v0.1.0 built 2026-11-22, not flashed.** Patchbay wave 2. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay ringdown`.
Presets are hand-written placeholders.

## The patch

    H-rate square-wave LFO (K1 Period; K3 Speed drift; S8 Drive Square / Pulse)
      -> bank of 8 resonant filters, tuned alike (K2 Ring; a slow triangle LFO breathes the tuning),
         dampings stepped heavy -> light (K4 Spread), all scaled by P12 RESONANCE (the Q CV, multed to the
         bank's detune input so the traces weave at high Q)
      -> 8 comparators against the V ramp
           V ramp -> |V - centre| folder (S7 Mirror: two squeezed copies, the lower one inverted)
           + camera luma, 8-px cells slewed down and across (K6 Camera; S11 Source Video / Osc lattice)
      -> resistor ladder: the soft comparator outputs summed = how many traces lie below the beam
      -> colouriser: 9 bands with pale fringes over a 16-colour wheel (K5 Colour rotation; S10 Spectrum / Neon)
         S9 Look: Bands (the count staircase) / Traces (each comparator edge as a glowing scope line)

## What it draws

A scope photograph of step responses, drawn solid. Every edge of the square wave kicks the eight filters: the
heavily damped one eases over, the lightly damped ones overshoot and ring, all settle toward the new level, and
the screen is coloured by how many traces lie below each point: flat saturated bands between successive traces
(crimson below everything, deep blue above, a violet..cyan rainbow between), woven where the detuned traces cross,
calming toward the end of each half-period. At the defaults: about 2.8 edges across the screen, rainbow flames
over crimson mountains under a deep-blue sky, drifting right while the ringing breathes (11 s triangle LFO).

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Period | square-wave LFO | cycles across the screen, 0.5 .. 5.7 (exponential); default 1.4 |
| K2 Ring | filter bank tuning | ring cycles per half-period, 1.5 .. 9 (exponential, plus the LFO); default 2.4 |
| K3 Speed | square-wave LFO | bipolar drift along the line, +-8 deadband, square law; right = rightward; default slow right |
| K4 Spread | filter bank | how far apart the eight dampings (x4.6 per side at full) and tunings are: 0 = all alike, one heavy rainbow line; full = from dead-beat to barely damped, strongly detuned |
| K5 Colour | colouriser | 16-step rotation of the bands over the wheel (centre = the designed colours; each step moves every band one colour along) |
| K6 Camera | V-ramp mixer (camera / Osc VCA) | camera luma added to the V ramp: 0 = the black-input picture .. 25 % = 55 rows per full luma .. 50 % = x2 .. 100 % = x5 (default) |
| S7 Mirror | V-ramp folder | Off / On: the ramp folded at the centre and squeezed, a butterfly of two full copies |
| S8 Drive | square-wave LFO | Square / Pulse: alternating kicks (impulse responses, sin onset) about the centre line |
| S9 Look | comparator outputs | Bands / Traces: eight glowing lines on black (white-hot cores, colour by band, dim glow) |
| S10 Colours | colouriser | Spectrum / Neon wheel |
| S11 Source | V-ramp mixer | Video / Osc (a drifting 640 x 540 triangle lattice at 2x the camera level) |
| P12 Resonance | filter bank Q | 0 % = near critical damping: a rounded square wave with nested S-curves fanning into colour fringes; 100 % = barely damped and detuned: a dense woven coloured lace (the climax); default 78 % |

## Video

The camera's luma, (Y >> 2) - 16, is averaged over 8-pixel cells, slewed down the screen (one pole, 1/16 per line,
state in a cell store), scaled by K6 at cell rate and slewed across (one pole, 1/16 per pixel), then added to the
V ramp in eighth-rows: the bands bend round bright areas like a scan processor (up to 272 rows for full white at
100 %). With a black input the patch is unchanged. Osc substitutes the shell's drifting triangle lattice at twice
the camera's level: the bands break into a lattice of displaced blocks.

## Engine

- Every line draws the same eight traces, so they are computed once per field, in the vblank zone:
  - per field (zone sequencer, one 21 x 19 serial multiplier, the exp2 ROM): the square-wave rate, the ring rate
    with the LFO, P12^2 / P12^4, the centre damping and spread, the detune, the drift phase, the camera gain;
  - per trace: two exp2 look-ups (zeta^2, Sigma = decay octaves per half-period, >= 1) and five serial products
    (Omega, the per-sample phase and decay steps, the phases at the first sample);
  - the fill engine: one term per clock, four terms per sample (the periodic steady state truncated at four edges):
    log|cos| ROM, the exponent, the exp2 ROM with 4-bit interpolation, a shift, the signed sum with the level;
    a slope encoder writes one 8-bit code per two output pixels (fine: signed 7-bit quarter-rows; coarse: 8x)
    with error feedback, after a 16-entry preamble that rebuilds the line-start value; ~23k clocks at 720p,
    ~33k at 1080p, ~13k at NTSC (all inside the real vblank).
- Per line: the V ramp at the line centre in eighth-rows (Mirror folds it); after each line the 16-entry preamble
  (32 clocks) reloads the eight accumulators.
- Per pixel (stages 1-13): table read, decode, accumulate, minus the beam, soft compare (8 rows, x8 wider on steep
  runs); the ladder sum and the traces brightness; the palette ROM (512 x 24); traces shading; compose.
- Shell (heirloom/lozenge): virtual 1080-line coordinates, vblank zone, generated avid, C_LAT 20 in every mode,
  output conditioning (two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16; the chroma 8-back
  taps ride two block-RAM rings at 8 bits), blanking gate. Colours in the hardware convention (U = Cr).
- Block RAM (24): 8 trace tables (2 each), palette 3, log|cos| 1, exp2 1, luma cell store 1, chroma rings 2.

## Status

- **v0.1.0 built 2026-11-22 with ROUTER1, all six configs on seed 1 (first attempt), not flashed.**
  Post-route Fmax: HD Analog 83.54, SD Analog 82.13, HD HDMI 86.84, SD HDMI 74.25 (Fmin 27), HD Dual 83.82,
  SD Dual 81.02 MHz. 7,201 .. 7,233 / 7,680 LC (94 %), 24 / 32 EBR. iceunpack OK x6; the packed config carries
  the new labels. `out/rev_b/ron/patchbay/ringdown.vmprog` (third build of the day; the first two shipped the
  Traces-look bugs listed below).
  Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay ringdown` (router2 was not tried;
  `dupin.py`: 0 duplicate-input cells). `scan.py`: 0 integer-minus-signed sites.
- **Checks** (working files in `.patchbay_work/rb_ringdown/`): GHDL raw 10-bit output against the integer model
  `imodel.py`, black input. The tester's fake vblank (8 lines) is shorter than the fill, so the renders use a
  sim-only copy (`mksim.sh`) whose single change is that the table writes are not gated by the vblank zone: the
  fill then finishes in the first 8-12 lines of the next frame, and those lines mix the previous and current
  field's tables (on hardware the fill completes inside the real vblank).
  - 720p60, decimation 1, defaults: rows 9-719 **0 differing pixels**; rows 0-8 1,615 (the fill tail).
  - 640 x 360 (720p, decimation 2): defaults, Mirror, Pulse, Neon, Traces: 0 differing pixels from row 12 down;
    a later frame (9 warm-up frames) matches the model six fields on (the drift and the LFO).
  - 320 x 180: K5 at 0 and at 1023 (the stepped knob's two ends, glide preset in the copy): 0 differing pixels.
  - NTSC decimation 1 (both fields) and 1080i5994 decimation 1 (both fields): 0 differing pixels from row 12 /
    row 8 down; the two fields of each match the model at consecutive field counts (the motion per field).
  - Osc and camera (selfie.jpg) frames at 720p60: checked by eye (the model's camera path is approximate).
  - The model: `thincheck.py` finds no single-row hard lines (runs >= 24 px) at 720p or NTSC over 12 cases.
- Bugs the sims found and fixed before the final build: `to_integer` of a signed slice in two `case`
  selectors (the decay rate of the lightly damped traces came out x128; the square-wave rate at K1 < 29 %),
  a narrowing `resize` of a signed value in the Traces shading (U/V folded: green lines), and a 10-bit overflow
  of the Traces white core before its clamp. After the last two fixes the Traces look (640 x 360) is
  bit-exact from row 12 down; the other checks above were run before them and do not touch that path.

## Cut / deviations

- The spec's engine (E_k -= E_k >> s per trace, per-pixel multiplies) was replaced by a per-field fill into
  tables (same formula family, exact periodic steady state truncated at four edges, decay >= 1 octave per
  half-period); the per-pixel work is decode / accumulate / compare.
- The step response uses the spec's cos-only form with the physical damped-frequency shift (1 - zeta^2/2);
  the small sin term was dropped (invisible in the prototype).
- K5 is a 16-step rotation over a curated wheel rather than a continuous hue rotation (the per-field palette
  engine that did that cost ~800 LC).
- The slope-aware soft edge has two levels (normal / x8) instead of a continuous law.
- SD uses the same soft widths in virtual rows as HD (softer-looking fringes are not added at SD).
