# Whorl (Vellum)

**v0.1.0 built 2026-11-16, not flashed.** Flash `out/rev_b/ron/vellum/whorl.vmprog`.

## What it draws

One figure filling the screen: a nest of polygons whose shape changes as it goes out. A sharp, slightly
pinched N-gon at the centre rounds into a circle mid-screen and opens into a soft N-lobed flower at the
edges, each band turned a little more than the one inside (a twist), so the nest whirls. Art reference:
nautilus whorls, rose windows, Vasarely's later polygon spirals ("spirals of different shapes that evolve as
they go out from the center"). The default frame is a triangle nest: a three-armed pinched star at the
centre, circles through the middle of the screen, a soft trefoil at the edges, in four grey steps (black,
dark, light, white), streaming slowly outward.

    L     = log2(r / 120 virtual px)                          the Hypnos log-polar engine (log2 r, theta)
    psi   = theta + twist * L                                 twist = 7e/64 turn per octave (e = P12 / 50 %)
    P_N   = log2 cos(pi/N) - log2 cos(a)    (<= 0)            the N-gon's polar factor; a = the angle from
                                                              the middle of a side (ROM, 64 steps per side)
    alpha = clamp(1 - 6 * twist * L, -0.44 e, 1 + e/4)        > 1 pinched, 1 the polygon, 0 a circle,
                                                              < 0 a flower
    phase = m (L - alpha P_N) + arms * theta / 2pi + camera - drift          (bands; m = bands per octave)
    tone  = Grey: the band's step (k mod 4) ; Ink: black / white          (Fine: lines 1/6 of a band)

At r = 120 virtual px alpha is 1 (the exact polygon) for every P12, and the twist pivots there, so the
centre figure keeps its orientation while P12 reshapes and turns the rest. The circle (alpha = 0) sits at
about 330 px at the default, 200 px at P12 100 %.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the centre, horizontal: detent (+-14 counts) = screen centre, +-2 screens at the ends (Hypnos) |
| K2 | Y | the centre, vertical, the same |
| K3 | Scale | bands per octave of radius, 2 .. 22.6, exponential (default 420 = 5.6) |
| K4 | Sides | N = 3, 4, 5, 6, 8 (five zones, 8-count hysteresis with the two end exceptions; default 3) |
| K5 | Drift | bipolar speed (Hypnos): +-24 counts of centre = still; up = the bands stream out of the centre, down = into it, 2^-12 .. 2^-2 band per field; default 720 = 0.0033 band per field (0.2 band per second at 60 Hz) |
| K6 | Camera | how hard the camera bends the bands: 0 = the clean figure, 25 % = 0.64 band at white, 50 % = 1.28, 100 % (default) = 2.55 |
| S7 | Glide | On (default): X, Y, Scale, Camera and Evolve ease 1/16 of the gap per field, Drift's speed 1/8 (never the phase); X / Y glide at 1/16 count = half a pixel at 1080, so the centre slides and lands. Off: no easing, the value latched at the field start is used that field. A +-1-count hold against pot jitter either way |
| S8 | Arms | Rings (default) / Spiral: 8 arms in Grey (two tone cycles per turn, seamless), 4 in Ink |
| S9 | Tone | Grey (default): four grey steps by band / Ink: black and white bands (each half a band) |
| S10 | Weight | Bold (default): full bands / Fine: lines 1/6 of a band, never under 2 px (they widen, then fade); in Grey the lines step dark, light, white and the black step drops out (groups of three) |
| S11 | Source | Video (default) / Osc: a static triangle lattice (512 virtual px) at 2x the camera gain (2.3x its swing) |
| P12 | Evolve | 0 = the same sharp N-gon at every radius, no twist (a plain polygon nest); 50 % (default) = pinched polygon -> circle -> flower with a gentle twist; 100 % = a steep metamorphosis inside the inner half of the screen, twist doubled (a whirlpool) |

## Motion

The band phase drifts: one per-field accumulator (Q20 bands), advanced once per field on the guarded vsync
edge, wrapping modulo four bands (one Grey tone cycle; Ink repeats every band), so the stream never jumps.
Because the polygons twist with radius, a band that streams outward turns as it goes: a whirlpool. Speed
range 2^-12 .. 2^-2 band per field (Hypnos's decode, top kept: a quarter band per field is fast but does not
strobe with four tones). The Osc lattice is static, so nothing moves with Drift at its centre.

## Camera

The camera is never shown. Its luma ((Y >> 2) - 16, black = 0) goes through a horizontal box (128 px at
1080, 64 px otherwise, edge-replicated at the line start), a vertical one-pole filter in a half-resolution
line store (1/32 per line at 1080p, 1/16 at 1080i / 720p / 480p / 576p, 1/8 at 480i / 576i), times K6,
and is added to the band phase: bright areas push the bands outward, so the subject appears as bends and
islands in the nest; every edge stays an anti-aliased curve. Gain at white: 0.64 band (25 %), 1.28 (50 %),
2.55 (100 %). Osc substitutes two triangle waves at the same point at twice the gain.

## Engine

Hypnos v1.1.0 chassis (timing-ID virtual coordinates, the log-domain polar engine, Hypnos's speed decode,
half-octave box anti-aliasing widths); the source field from Kinetic riley via Hardline blaze.

- Per field (vblank sequencer from the first vsync edge after active video, one small op per state): six
  knobs through one hold + glide datapath; sides; m from an exp2 ROM; Drift's speed and phase; twist and the
  alpha clamps from Evolve; camera gain; the centre (one serial multiplier).
- Per pixel, 41 stages, latency 42 clocks in every mode (= processing_delay_clks):
  S1-15 |dx|, |dy| -> log2 r, theta (Hypnos tables); S16-18 L, Y = twist * L; S19-20 psi -> N psi -> the
  sector fold -> ROM address, alpha = clamp(1 - 6Y); S21-24 polygon ROM (value + slope + tan a),
  interpolation, alpha tan a; S25-27 alpha * P, Phi, |R| -> G = max + min / 2; S28-30 m * Phi + arms theta +
  camera - drift; log2 G -> the half-octave filter width and the fade; S31-36 signed distance -> coverage ->
  tone = X + Y * coverage (X, Y from the fade ROM); S37-41 luma [1 2 1] / 4, 11/16 toward black (white 768),
  blanking gate. U = V = 512 on every pixel.
- Anti-aliasing from the ANALYTIC gradient |grad| = m / (r ln2) * |(R, B)|, R = 1 + e (g P - alpha tan a),
  B = alpha tan a (validated against finite differences of the model's phase: within -0.25 .. +0.18 octave,
  1st .. 99th percentile); bands under 4 px fade to the flat grey, gone at 2 px (the dense centre).
- Multipliers per pixel: twist * L, alpha * P, m * Phi (each split in two), the ROM interpolation,
  alpha tan a, e R', the tone, the camera gain, and Hypnos's two table interpolations. No divider, no root.
- Block RAM: Hypnos log / polar tables 8, polygon ROM 4, fade ROM 2, exp2 1, delay lines 6 (no_rw_check),
  box 1, IIR line store 4 = 26.

## Status

- Built 2026-11-16 through the shared queue with **router1** (the Hypnos chassis's router), **seed 1 on all
  six configs** (no retries). Not flashed.
- Post-route Fmax (the builder's last "Max frequency" line per config): HD Analog 88.29 MHz, HD HDMI 86.17,
  HD Dual 79.35, SD Analog 78.93, SD HDMI 78.90, SD Dual 76.83. HD needs 74.25, SD 27. All six bitstreams
  pass iceunpack.
- 7,026 .. 7,058 / 7,680 LC (91.5 .. 91.9 %), 26 / 32 block RAMs. No `emulate_transparency` (every
  read-and-written RAM carries `no_rw_check`); dupin.py: no duplicate-input cells. Latency 42 clocks =
  `processing_delay_clks`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum whorl`.
- Model = RTL (GHDL image tester against `imodel.py` in the work dir, which mirrors the RTL stage by stage):
  - 720p60 decimation 1 (1280 x 720), defaults, black input: 0 differing pixels of 921,600 (the capture is
    aligned with the model: no generated-avid column offset);
  - 720p60 decimation 2 (640 x 360): defaults 0; camera = bear.png at K6 100 % 0 (against the tester's
    actual input image; the camera stream sits 21 columns ahead of the pixel pipeline); S11 Osc 0;
  - NTSC decimation 1: 0 of 174,960 in each field, and shown at 4:3 it is the same picture as the centre 4:3
    of the 720p frame (round stays round);
  - motion: at the default Drift, a capture after 1 field (Glide Off) and one after 4 fields (Glide On at the
    power-up defaults) each match the model at exactly that travel (0 differing; one field either side
    differs by ~21,000 px), i.e. the bands stream outward at 0.0033 band per field as designed;
  - 720p60 decimation 1, S11 Osc: 0 of 921,600; camera = bear.png at K6 100 %: 0 of 921,600.
  Not simulated: 1080p / 1080i rasters, a glide in progress, knob changes mid-run.
- Renders (work dir `.vellum_work/rb_whorl/`): `cmp_final.png`, `sheet.png` (control sheet, model),
  `strip.png` (P12 travel), `motion.png` (default Drift 120 fields apart, both ends), `camera.png` (bear at
  25 / 50 / 100 %), `motion_rtl.png` (RTL travel against the model), `presets.png`, `ntsc_vs_720p.png`.

## Cut / deviations

- **A calmer default shape range than the sketch.** The spec's alpha 1.6 (centre) .. -0.9 (edge) with a
  0.9 rad/octave twist rendered busy and wavy at the edges; the default is 1.25 .. -0.45 with 0.69
  rad/octave, and the clamps grow with Evolve, so P12 100 % reaches the sketch's 1.5 .. -0.9 (the climax).
- **alpha is tied to the twist** (alpha = 1 - 6 * twist * L): one per-pixel multiply serves both, and the
  pivot (alpha = 1, no rotation) is fixed at 120 virtual px.
- **Spiral arms: 8 in Grey, 4 in Ink** (the spec suggested 4 / 1-2): fewer arms were barely visible at the
  default density, and the switch must be an unmistakable flip. Both are whole tone cycles per turn.
- **The Drift phase wraps at four bands in every mode** (the Grey cycle; Ink and Fine repeat every band).
- **No generated avid.** Hypnos makes its own avid so the encoders cannot swap Cb / Cr; Whorl outputs
  U = V = 512 on every pixel, which no pairing can swap, so the source avid is used (delayed by the latency)
  and the ~100 LC went to the fit.
- **X / Y glide in the knob domain**, at 1/16 count (half a pixel at 1080), through the shared knob glide,
  instead of Hypnos's separate pixel-domain centre glide (430 LUT). With the +-1-count jitter hold a glide
  settles within 2 counts of the knob (up to 15 px at 1080), then stops cleanly.
- **Anti-aliasing approximations** (all model = RTL): e to 1/4 in the gradient, |(R, B)| ~ max + min / 2, the
  spiral's own angular term left out (< 10 % on the width), the polygon ROM's 64 steps per side with a 6-bit
  interpolation fraction, Hypnos's u-table interpolation at a 5-bit fraction (Hypnos: 7). The last can put
  sub-pixel kinks (up to ~0.3 px at 1,000 px from the centre) where an edge crosses a ray of constant
  |dy| / |dx|; not seen in the renders.
- **The fade to flat grey has 16 steps** (folded with the tone into a ROM); the camera gain is 8 bits.
- **The Osc lattice is static** (only Drift moves the picture) and, as in Blaze, it is 768 virtual px at
  720p against 512 at 1080 and SD.
- **Camera box 128 px at 1080, 64 px otherwise**: 128 virtual px at 1080 and SD, 96 at 720p (720p bends a
  little more sharply). The half-resolution IIR store is held, not interpolated (2-px steps in a field that
  is smooth over 100+ px).
- **Near the centre** a few 2-3 px spots by the polygon corners keep half contrast inside the flat grey disc:
  they are real places where the bands are locally wide (the analytic gradient agrees with finite
  differences there), so they change tone as the bands stream.
- **Fine Grey's flat grey is darker** (64: half the duty of the line tones), so the dense centre of a Fine
  Grey picture fades to a dark grey glow rather than mid-grey.
- Output scaling 11/16 in one step (white 768), as Blaze.
- Fit: the first RTL packed to 8,699 LC (113 %); see notes.md in the work dir for the five rounds of cuts.
