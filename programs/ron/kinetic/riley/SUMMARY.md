# Riley (Kinetic pilot)

**v0.1.1 (K6 camera pass) built 2026-11-15, not flashed.** Flash `out/rev_b/ron/kinetic/riley.vmprog`.

## What it draws

Bridget Riley's black-and-white wave paintings: "Current" (1964), "Fall" (1963), "Cataract 3" (1967) and
her zigzags. Parallel bands of even weight, all displaced by one shared wave whose wavelength tightens
along the band, so the slopes crowd at one end and the eye sees shimmer and depth there. The default frame
is a "Current"-like field: 40 off-white bands on near-black (Ink palette), about six wave cycles across,
long and lazy at the left, tight at the right, the crests slanting a little from band to band. The wave
travels along the bands (about 7 s per cycle at the default Rate).

## Element and state

Element: a band. State: its displacement across the band axis, continuous along the band.

    PHI(u, v) = v / pitch + Fold * wave(theta)          band = floor(PHI); it fills |frac(PHI) - 1/2| < weight / 2
    theta     = f0 (u + c u^2 / 2U) + Spread * v / pitch + phase                 Field = Linear
              = f0 (1 + 3c/4) r^2 / 2R + (Spread / 4) * v / pitch + phase        Field = Radial
    weight    = K3 + min(1, 2 luma^2) * (0.95 - K3)                              the second modulator
    coverage  = clamp(1/2 + (weight / 2 - |frac(PHI) - 1/2|) / |grad PHI|)       17 steps per edge

u runs along the band, v across it, both from the screen centre (so Scale zooms about the centre), r is the
distance from the centre. f0 is the wave's base frequency: wavelength 181 virtual px at pitch 8, growing as
the square root of the pitch (333 px at the default pitch 27, 1,145 px at pitch 160), so every Scale shows
several cycles. c is the chirp, +-0.94: the frequency ramps linearly from f0 (1 - c) to f0 (1 + c) across
the picture.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | band pitch, 8 .. 160 virtual lines, exponential, glided (default 27 = 40 bands); the wavelength follows as its square root |
| K2 | Colour | the 16 shared Kinetic palettes (ground + A; Quad adds B and the accent), with hysteresis (K6 until v0.1.0; Chirp is now fixed at its old default 760, a gentle tightening toward the right) |
| K3 | Weight | band duty at black, 10 % .. 90 % (default 40 %); the source adds to it |
| K4 | Rate | wave travel, bipolar, centre = still; exponential, 60 s per cycle just off the deadband to 2 cycles per second at either end |
| K5 | Spread | crest slant: phase step per band, square law, 0 .. 1/2 cycle. Low = slanting crests ("Cataract 3"); high = neighbouring bands in antiphase, a woven standing figure. Radial: slides the pond's centre along the band axis |
| K6 | Camera | camera (and Osc) strength, 0 .. 100 %, default 50 %: 0 % = no camera (the black-input picture), 25 % = the v0.1.0 weight curve, 50 % = twice it, 100 % = three times (see "K6 camera pass") |
| S7 | Wave | Smooth (sine) / Zigzag (triangle: Riley's chevrons). **Zigzag replaces the standard Snap**: a dwell-and-flip wave only reads on discrete elements, a triangle is the Riley figure |
| S8 | Field | Linear (the wave runs along the bands) / Radial (a pond: a calm centre, rings that crowd outwards) |
| S9 | Bands | Duo (every band is colour A) / Quad (the bands step through A, B, accent, ground: three coloured lines and a rest) |
| S10 | Axis | Across (horizontal bands displaced vertically) / Down (vertical bands, as in "Fall") |
| S11 | Source | Video / Osc |
| P12 | FOLD | the headline: wave depth on a square law, 0 .. 6 bands. 0 % = ruled straight lines, about 38 % (default, 0.85 band) = "Current", past about 60 % the bands turn over on themselves, 100 % = folded moire: eyes and islands where the bands fold, grey where they outrun the raster |

Depth is the headline, so it sits on P12 and K3 became Weight (the brief's rule for this program).

## Video

`program_type = "processing"`. The picture's luma is the second modulator and drives a second property of
the element, its WEIGHT: luma -> 32-pixel box (edge-replicated at the line start) -> vertical 1/8 IIR in a
line store -> squared -> scaled into the head-room between K3 and 95 %. Dark areas keep the K3 line weight,
bright areas swell the bands until only hairline gaps are left, so the camera appears as an engraving while
the wave still displaces the lines. With a black input the weight is uniform and the wave alone runs.
Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice at the same point, at full swing
(the camera rarely reaches it), so big diamonds of swelling lines drift across: an unmistakable flip.
The weight map lags the picture by about 12 pixels to the right (the box and the 9-tap output filter).

## Engine

The whole phase field is one bi-quadratic, theta = J1 dx + J2 dx^2 + K1 dy + K2 dy^2 + phase, and the band
coordinate is linear, pv = jPV dx + kPV dy. Field and Axis only choose the six coefficients, so all four
combinations run on the same hardware and the pixel path has no multiplier.

- **Per field** (vblank zone, "no avid for 1024 clocks"): the knob glides (K1, K2, K3, K5, P12: 1/8 of the gap
  per field), one serial 16-step multiplier in 17 slots (pitch and wavelength through a 2^(i/256) ROM, chirp,
  spread, fold, rate, the beam DDAs' start values), the wave phase step, three table refills by accumulators
  (Fold x magnitude, 64 entries; the weight curve, 256 entries, by second differences), the palette
  (ground + three differences from it), then the line-0 kick. About 600 zone clocks.
- **Per line**: an engine kicked at every line start computes the NEXT line's start values on a second serial
  multiplier (dy^2, K2 dy^2, K1 |dy|, kPV |dy|; 71 clocks): theta and the band coordinate at the pre-pixel.
- **Per pixel** (29 stages): two free-running beam DDAs (theta second-order = the chirp, 36 bits; the band
  coordinate, 28 bits) -> quarter-wave sine ROM (4,096 points per cycle) or triangle -> Fold table (two
  6-bit lookups) -> PHI (Q4.12) -> gradient by finite differences against the previous pixel and the line
  above (a 2,048 x 16 line store), per REAL pixel, so the anti-aliasing is right in every video mode ->
  coverage by a 4-step divider (17 levels); past 1/4 band per pixel the transition widens 5x so bands that
  outrun the raster fade to their mean instead of aliasing; band and gap are each held to at least one pixel
  of gradient (2 pixels of line) -> shift-add blend ground + (colour - ground) * coverage -> output
  conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums, luma [1 2 1]/4 delayed to the same
  group delay, luma 13/16 toward black) -> blanking gate.
- Pixels 0 .. W + 7 are rendered and output column j is the filter over pixels j .. j + 8, so both picture
  edges are filtered from real content. Generated avid (the shell's parity-locked start), one latency (36)
  in every mode, virtual 1080-line coordinates from the timing ID (SD shows the centre 4:3).
- Colours in the hardware convention (U register = Cr). Drawn white comes out at Y = 732.

Chassis sections in `riley.vhd` (for the programs that start from this one; every block carries its
letter in a comment):
- [A] timing and virtual coordinates: 81-153 (step functions, zone / width / generated-avid signals), 395-402
  (the beam's pre-pixel by video mode), 433-461 and 475-496 (p_ctrl: steps, zone, width, per-field latches),
  681-690 (sync, avid and start shift registers), 695-737 (p_gavid);
- [B] controls: 154-171, 462-474 (rate deadband), 475-496 (switch latches, palette hysteresis), 497-533 (glide);
- [C] wave engine: 172-207 and 213-256 (declarations), 404-414 (the six coefficients by Field / Axis), 535-656
  (per-field sequencer), 740-793 (p_line), 796-815 (p_beam), 818-883 (p_pix, with the Fold-table fill);
- [D] source field: 257-293, 886-962 (p_src: Osc, box, IIR store, weight-table fill);
- [E] palette + AA blend: 208-212, 294-312, 658-679 (palette load), 965-1066 (p_blend);
- [F] output conditioning: 313-326, 1069-1112 (p_out).
The RAM / ROM processes are 331-392. `imodel.py` (work dir) mirrors each section.

## Status

- v0.1.0 built 2026-11-14 through the shared queue, router2 (the builder default). Not flashed.
- Post-route Fmax: HD Analog 75.98 MHz (seed 1), HD HDMI 79.80 (seed 1), **HD Dual 81.29 (seed 2**; seed 1
  missed), SD Analog 77.30 (seed 1), SD HDMI 73.09 (seed 1), SD Dual 75.24 (seed 1). HD needs 74.25, SD 27.
- 6,673 .. 6,695 / 7,680 LC (87 %), 22 / 32 EBR. Six bitstreams pass iceunpack. No `emulate_transparency`,
  no carry self-adds (dupin: 0).
- Rebuild: `./build_set.sh kinetic riley`.
- Model = RTL (GHDL, 720p60, decimation 1): pixel diff 0 of 918,720 on four renders: the defaults; Radial +
  Quad + reverse Rate + Vega; a sim-only copy with other knob values (Scale 600, Chirp 200, Weight 700,
  Spread 600, Fold 700, Zigzag, Down, Citrus); both fields of an NTSC render (720 x 243) and both fields of
  a 1080i render (1920 x 540). Osc and camera renders differ from the model only in the weight's lattice /
  filter phase (3.7 % of pixels, under 1 code on average). The GHDL frame is the
  model's shifted 4 columns right with 4 blank columns at the left: the generated avid starts 4 clocks after
  the delayed source avid, as in the heirloom shell; the geometry is offset to match, so the picture centre
  sits on the screen centre.
- Renders: `.kinetic_work/rb_riley/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion).

## Cut / deviations

- **Radial is a quadratic pond, not chirped(r).** A true distance needs a per-pixel square root; the first
  version had one (16-stage pipelined root, pixel diff 0 in GHDL) and came to 8,263 LC, 107 % of the chip.
  theta = q r^2 is exact with the two DDAs already there, so Radial now draws a pond with a calm centre whose
  rings crowd outwards. Lost: equal-spaced rings, and "tight at the centre" (in Radial, Chirp sets how fast
  the rings crowd, 0.3x .. 1.7x). Spread in Radial slides the pond's centre along the band axis.
- Zigzag in place of Snap (see S7).
- The camera adds to the weight through luma^2 into the head-room above K3, not as a plain sum: a plain sum
  clipped every highlight once K3 passed 50 %. At K3 = 90 % there are only 5 points of head-room left, so the
  picture is faint there.
- Fold tops out at 6 bands, not 8: at 8 more than half of a 720p frame was grey.
- The wavelength follows the square root of the pitch, so Scale is not a pure zoom.
- `field_n` is passed through one register instead of the 36-clock delay (the core ties it high on every
  input).
- SD: the same picture (the centre 4:3, same band count top to bottom), but 480i has 243 lines per field, so
  below about Scale 25 % (pitch under 16 virtual lines = under 4 field lines) the bands begin to fade toward
  grey, and at Scale 0 % the field is flat grey with faint moire. 720p and 1080p hold every Scale.

## K6 camera pass (2026-11-15), v0.1.1

- **K6 = Camera**, 0 .. 100 %, default 50 % (initial 512): the strength of the camera's (and the Osc
  lattice's) effect on the band weight. It is the gain g of the weight curve,
  W(l) = K3 + min(1, 2 g l^2) x head-room, with g = K6 / 256 up to 50 % and half that slope above
  (g = 2 + (K6 - 512) / 512):
  - 0 % (g 0): the table is flat at K3, exactly the black-input picture (the wave alone);
  - 25 % (g 1): the v0.1.0 curve, bit for bit (a subtle engraving);
  - 50 % (g 2, default): twice v0.1.0; mid-greys swell the bands to hairline gaps, darker areas keep their
    gaps; clean, no ragged edges (the weight only changes duty, never position);
  - 100 % (g 3): most of a mid-key picture is hairline gaps, only the darker areas still show as thick
    dark bands. A straight g = 4 (K6 / 256 all the way) washed the selfie out to nearly uniform hairlines,
    so the slope halves past 50 %.
  - Source = Osc goes through the same table, so it keeps its full-swing lead over the camera.
- Where it is applied: per field, in the weight-table fill (no pixel-path change). K6 is glided in the
  existing shared glide (channel 1, which was Chirp's; deadband +-4 codes, 1/8 of the gap per field), one
  new slot (17) on the per-field serial multiplier forms span x K6 / 64, and the table's second-difference
  DDA steps by that (28-bit accumulator, Q17). The fill moved from zone clock 224 to 464 (after slot 17;
  done by 720, well inside every mode's vblank).
- **K2 = Colour** (the 16 palettes, was K6); presets keep their palettes on K2.
- **Chirp is fixed** at its old default 760 (c = +0.45: the wave tightens gently toward the right; in Radial
  the rings crowd 1.34x outwards). It was the weakest knob: its sheet panels at 0 / 760 / 1023 barely
  differ. The presets lost their Chirp values (Fall 900, Cataract 512, Chevrons 620, Pond 300, Moire 700).
- Build (router1 through the queue, 2026-11-15): all six configs on seed 1. Post-route Fmax HD Analog
  90.10 MHz, SD Analog 95.14, HD HDMI 93.70, SD HDMI 94.57, HD Dual 93.49, SD Dual 91.11 (worst HD 90.10).
  6,560 / 7,680 LC (85 %), 22 / 32 EBR. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic
  riley`.
- Look check: `.kinetic_work/rb_riley/k6sheet.png` (GHDL, selfie, 720p60 / 2, glides bypassed in a sim-only
  copy; K6 = 0 / 256 / 512 / 1023 and Osc at 512; Colour fell to Moss in those renders because the
  labelled K2 sims at 512).

