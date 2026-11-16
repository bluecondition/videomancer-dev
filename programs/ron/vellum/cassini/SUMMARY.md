# Cassini (Vellum)

**v0.1.0 built 2026-11-16, not flashed.** Flash `out/rev_b/ron/vellum/cassini.vmprog`.

## What it draws

Cassini ovals and the lemniscate of Bernoulli, after the bipolar-coordinate plates and field diagrams of old
physics books: one figure filling the screen, bands of constant r1 x r2 about two foci. Small ovals ring each
focus, the figure-eight lemniscate sits between them, and outside it come peanuts, ovals and near-circles. The
bands stream out of the foci. Black, white and greys only.

    Cassini   phi = (B/2)(log2 R1 + log2 R2) - (B/2)(36.28 + la/64) - 4 drift + camera      (bands, mod 4)
              Grey: a four-step staircase by band (black, 1/3, 2/3, white); Ink: black / white bands
              Fine: lines on the band edges (Grey: white and dark-grey lines alternate)
    Targets   phi1 = (B/2)(log2 R1 - 18.14) - 4 drift + camera     one log-spaced ring target per focus,
              phi2 = (B/2)(log2 R2 - 18.14) + 3 drift + camera     two translucent films
              Grey: target 1 x 1/3 + target 2 x 2/3 (four tones); Ink: XOR, the bipolar checkerboard
              (its diagonals are the Cassini ovals and the Apollonian circles)
    R1, R2 = squared distances to the foci; the lemniscate is the phi = 0 edge at every Spread up to 50 %

At the default the lemniscate is the band edge through the midpoint, the foci are 536 virtual px apart (half
the screen height) and the picture is the grey staircase with the bands flowing slowly outward.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the midpoint of the foci, about one screen width either way |
| K2 | Y | the same, vertically, about one screen height either way |
| K3 | Scale | bands per octave of r1 r2, 0.7 .. 11.3, exponential (default 2.8) |
| K4 | Angle | the axis through the foci, -90 .. +90 degrees, 8-count centre detent = horizontal |
| K5 | Drift | bipolar speed (Hypnos decode), +-24 counts still, 2^-12 .. 2^-4 cycle (4 bands) per field; > 512 streams the bands out of the foci, < 512 into them; default 660 (+29 %) |
| K6 | Camera | luma added to the band phase (default 100 %): the bands bend around the picture |
| S7 | Glide | Off / On (default On): eases X, Y, Scale, Angle, Camera and Spread (1/16 of the gap per field, at least half a count, landing exactly, behind a +-1-count hold) and the Drift speed (1/8). Off = this field's knob values |
| S8 | Family | Cassini (ovals about both foci) / Targets (two ring targets, one per focus, drifting against each other) |
| S9 | Tone | Grey (four-step staircase; Targets: films adding) / Ink (black and white; Targets: XOR checkerboard) |
| S10 | Weight | Bold (filled bands) / Fine (lines on the band edges, 4 virtual px wide; at least 1.5 pixels) |
| S11 | Source | Video / Osc |
| P12 | SPREAD | the foci separation: 0 = one point (concentric rings), 50 % = 536 virtual px (default), 100 % = 1,737 virtual px (foci near the screen edges, the reference follows the separation so a huge figure-eight spans the screen) |

## Motion

One accumulator (Q18 cycles of 4 bands, wrapping exactly) advances once per field by the Hypnos speed
2^(ls/64 - 12) cycles, ls = u + u/16 (u = |K5 - 512| - 24, at most 512): from 1/1024 band per field just past
the dead zone to 1/4 band per field at the ends (capped there: a quarter cycle per field would strobe the
four-tone staircase). The default is about 0.24 bands per second, outward. Cassini: every band moves out of
the foci, so the lemniscate edge peels off the foci and passes outward every few seconds. Targets: target 1
streams outward at 1x while target 2 streams inward at 3/4, so the checkerboard shears and churns. Drift at
centre (+-24 counts) is the only still state.

## Camera

The camera bends the bands: luma (or Osc) -> 8-pixel means -> a gain table (refilled per field from K6) ->
horizontal one-pole (1/4 per mean, reloaded at each line start) -> vertical one-pole (1/64 per line at
1080p, 1/32 at 720p / 1080i / SD progressive, 1/16 at 480i / 576i) in a 1/8-resolution line store ->
linear interpolation -> added to the band phase. White adds 0.75 bands at K6 25 %, 1.5 at 50 %, 3 at
100 %; black adds nothing, so a black input (or K6 0) is exactly the clean figure. Osc substitutes the
shell's drifting 640 x 540 virtual-px triangle lattice at 2.5x gain (up to about 7.6 bands): the ovals
buckle into a drifting quilt.

## Engine

- **Per field** (vblank zone, started at every zone start): a microcoded engine after Hardline flute
  (cassini.py PROGRAM, 492 micro-ops, 62 registers in one block-RAM register file initialised to the TOML
  defaults; 37-bit accumulator, one adder, a serial 18 x 18 signed multiplier and a serial shift; about 2,830
  clocks): knob holds and glides, the Drift speed and accumulator, Scale (2^(i/256) table), Angle (an
  interpolated quarter sine), Spread, the exact x / y / axis DDA constants and line-0 start values (Q8 h^2),
  the anti-aliasing constant and the camera gain step. Its outputs start the fills: the T table (two copies,
  (B/2) log2(1 + i/256) mod 4 with its slope, 257 entries by a radix-4 serial multiplier, 10 clocks each),
  the E table (64 entries per octave sum, (B/2) n - reference - 4 drift; a second copy + 4 drift for Targets)
  and the camera gain table.
- **Per line**: three exact second-order DDAs (Zy, its step, the axis term) advance at each line start.
- **Per pixel** (stages 2 .. 28): ZA and Lx DDAs -> R1 = ZA - Lx, R2 = ZA + Lx, |z|^2 -> leading-one
  normalise (16-bit mantissas) -> T RAM + 4-bit slope interpolation, E RAM by exponent -> phase(s) + camera ->
  analytic anti-aliasing in Q6 log2 (KB + (log|z|^2 - log R1 - log R2) / 2, the window rounded up to a half
  octave, at most 1/4 period per pixel) -> one coverage unit across the nearest band edge -> tone (staircase,
  ink, Fine line or the two Targets films) -> past 1/4 period per pixel (only right at the foci) a four-step
  fade to flat grey -> luma [1 2 1]/4 -> 13/16 toward black (white 777) -> blanking gate. U = V = 512.
- One latency (28) in every mode; generated avid with the shell's parity-locked start; virtual 1080-line
  coordinates (SD shows the centre 4:3, round things round); the Fine line is 4 virtual px wide (at least 1.5 pixels).
  `field_n` passes through one register.
- Chassis: the Kinetic riley shell ([A] timing, [D] source field, [F] output conditioning).

## Status

Built 2026-11-16 with the default router (router2), all six configs on seed 1:

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | 1 | 81.53 MHz |
| HD HDMI | 1 | 85.29 MHz |
| HD Dual | 1 | 78.52 MHz |
| SD Analog | 1 | 79.74 MHz |
| SD HDMI | 1 | 87.05 MHz |
| SD Dual | 1 | 73.43 MHz (Fmin 27) |

6,214 .. 6,245 LC (81 %), 20 / 32 EBR. Rebuild: `./build_set.sh vellum cassini`.

Verification: the integer model (.vellum_work/rb_cassini/imodel.py) matches the GHDL render pixel for pixel
(0 mismatches) on full 720p frames (black input, Osc, bear.png at K6 100 %) and on NTSC; two captures at the
default Drift 6 fields apart equal the model at fields 3 and 9 (the edges move outward about 0.25 px per
field near focus 1), and a Glide On frame at the power-up defaults equals the model's defaults. The engine's
emulator equals the model's per-field arithmetic on 3,000 random knob / switch / mode sets. dupin: 0
`CARRY I0==I1` (router1-safe); no emulated RAM transparency.

## Cut / deviations

- S8's second family is **Targets** instead of Confocal (ellipses + hyperbolas). Confocal needs r1 and r2
  themselves and its anti-aliasing softened 3 - 5 px at the folds; it did not fit beside the camera. Targets
  keeps the brief's two translucent films (Grey = four tones, Ink = the XOR curved checkerboard) and its
  checkerboard diagonals are themselves Cassini ovals and Apollonian circles.
- Right at each focus the rings go sub-pixel; they fade to flat grey there (by design, the brief's "bound it
  near the foci").
- The leftmost 7 pixels of each line take the camera value of the line above (the coarse store's first
  column is not yet written); invisible.
