# Doppler: wavefronts from a moving source

**v0.1.0 built 2026-11-22, not flashed.** Patchbay (wave 2). A source travels through still air, seen from above:
twelve circular wavefronts, each centred where the source was when it was emitted, bunch up ahead of it and
stretch out behind; at Mach 1 they pile into a wall, above it they overtake each other inside a Mach cone.
Presets are hand-written placeholders.

## The patch

    H ramp, V ramp, minus the source CV S --> two squarers --> sum           |q|^2              (P DDA, 2nd order)
    source velocity CV d = Mach x wavelength at the heading . (H, V ramps)    d.q                (H DDA)
    ring oscillator (K3 Rate) --> phase f; the centre CV passes down an analog shift register clocked by it,
      so ring n (age m = n + f) is centred at S - m d with radius m L
    12 comparators on F_m = |q|^2 + 2 m d.q + m^2 (|d|^2 - L^2)              (adder chain F_{n+1} = F_n + J_n)
      window outputs |F_m| < T_m (line weight, K4) --> line mixer --> fringe lines / ring brightness
      sign outputs F_m < 0 --> counter --> colouriser staircase (K5 rotation)  (band identity = count + wraps)
    |cross| cos a - along sin a comparator, sin a = 1 / Mach                  the Mach cone (envelope)
    Mach x (dir . q) / 4L --> hue (S10 Shift: red behind, magenta abeam, blue ahead)
    camera luma --> 8-px cells --> vertical slew 1/32 --> 2 horizontal slews 1/16 --> local speed of sound (K6)
    Osc lattice (S11) at 2.5x the camera's level, same point
    LFO 1 (23 s) --> heading sway +-14 deg;   LFO 2 (9.7 s) --> Mach +-5 %
    output: two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16, blanking gate (the shell's conditioning)

## What it draws

At the defaults (Mach 0.79): the source sits right of centre, heading up-right. Its youngest fronts are small
discs around it; older fronts are larger circles centred further and further behind, so the bands are thin and
packed against the right edge and wide on the left. Bands run black / vivid hue / near-black / hue + 60 deg, each
edge drawn as a pale fringe line; the hue follows the doppler shift: orange-red and chartreuse behind, magenta
across, violet-blue in the packed stack ahead. Beyond the outermost front: black.

Motion at the defaults: the fronts expand (a new front is born at the source about every 2.5 s, every band keeps
its colour as it grows and the oldest fades out), the heading sways over 23 s and the Mach number breathes over
9.7 s, so the bunching ahead tightens and relaxes; the strip never loops.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Wavelength | ring oscillator / speed of sound | L = 40 .. 320 virtual px (exponential); 380 = 87 px |
| K2 Heading | velocity CV | direction of travel 0 .. 360 deg, plus the slow LFO sway; 960 = 337 deg (up-right) |
| K3 Rate | ring oscillator | bipolar (+-8 deadband, square law): fronts expand (right) or contract back into the source (left), full = 3 fronts/s; 704 = 0.40 /s |
| K4 Line | window comparators | ring line half-width 1 .. 16 px (exponential); hairline fringes .. fat glowing rings |
| K5 Colour | colouriser | hue rotation of the whole palette (0 .. 360 deg) |
| K6 Camera | speed of sound (camera / Osc) | box-filtered luma dents every front by up to 80 px (Osc 2.5x), linear; default 100 % |
| S7 Look | colouriser / line mixer | Bands (the ring count as colour bands, lines as fringes) / Rings (bright lines on black, line width doubled) |
| S8 Pulse | counter / line mixer | Continuous / Burst: three fronts in four drop out of the count and dim to a quarter: wave packets |
| S9 Fade | colouriser | Off / On: older bands and rings fade toward the edge of the pattern |
| S10 Colours | colouriser | Shift (doppler hue) / Phosphor (one scope green, black / green / near-black / green bands) |
| S11 Source | camera input | Video / Osc |
| P12 Mach | velocity CV | 0 = at rest (a concentric target), 40 % = Mach 0.5, 65 % = Mach 1 (all fronts touch at the source: the wall), 100 % = Mach 2.2: the cone, its two envelope lines drawn bright; the scene zooms out above Mach 1 to keep it framed |

## Video

- **Video (default).** Camera luma ((Y >> 2) - 16) is averaged over 8-pixel cells, slewed down the screen (1/32 per
  line, state in a block RAM) and across (two one-poles, 1/16 per pixel), then added to each front's J term, i.e.
  a local change in the speed of sound: every front dents by the same distance (up to 80 px for full white at
  100 %) round the bright parts of the picture. The slews keep every front a clean curve. With a black input the
  patch gives its full picture.
- **Osc.** The shell's drifting triangle lattice takes the camera's place at 2.5x the level: the fronts wobble in
  broad lobes.

## Engine

- **Microcoded engine** (doppler_engine in doppler.vhd; program assembled by doppler.py, emulated bit-exactly by
  imodel.py): 128 x 32 register file and 512 x 48 program in block RAM, one serial multiply-accumulate
  (sign-magnitude, stops at the last set bit), single-cycle ADD / SUB, MIN / MAX, table reads (exp2, sin, sqrt),
  IN / OUT, WAIT / JMP. Restarted at every field start (the vblank zone). The field program (~4.6 k clocks) computes
  Mach, heading, wavelength, zoom, line width and the chain scale 2^-e (T1 = 2 L w in [16, 32) after the shift),
  the source position, every DDA start / step, the cone and hue coefficients, the phase f and band identity;
  then it runs one line ahead of the beam (~0.5 k clocks a line), loading the line DDAs in hblank.
- **Pixel path.** P and H DDAs (stage 1), a 2^-e barrel, the camera term, the 12-stage ring chain (F 25 bits,
  J 22 bits; per ring |F|, T_n - |F|, a soft tent with a fixed per-ring ramp, a burst / age cap), the count and
  line sums, hue (128-entry table, K5 rotation, 4-phase horizontal dither), the Mach-cone distance, the
  1024 x 24 colouriser ROM, the band-fade / brightness multiplier, then the shell's output conditioning and the
  blanking gate. Per field: everything in the engine; per line: P, H, hue and cone starts; per pixel: the rest.
- **Contracts.** Generated avid; one latency (C_LAT 36) in every mode; virtual 1080-line coordinates (SD shows the
  centre 4:3); colours in the hardware convention (U = Cr); drawn whites about 777; syncs delayed in a block-RAM
  ring.
- **Block RAM (24).** Program 6, register file 2, tables 3, colouriser 6, camera cells 2, box rings 4, sync ring 1.

## Verification

- **Engine.** RTL engine vs the Python emulator (`tb/tb_engine.vhd`, `cmp_eng.py`): 76 / 76 port writes identical
  at six settings. Field program ~4.6 k clocks, a line ~0.5 k (fits the NTSC sim's fake 8-line vblank, so SD sims
  are valid too).
- **GHDL image sims vs the integer model `imodel.py` (bit-exact model of the RTL arithmetic):**
  - 720p60 dec 1, black input, defaults (`rtl_default.png`): **0 differing pixels** of 921,600 (capture shift 4).
  - 720p60, 5 fields later (`rtl_motion.png`): 0 differing pixels; 302,969 pixels changed against the first
    capture, so the motion follows the model.
  - 720p60, Rings + P12 100 % (sim-only copy with the P12 glide bypassed, `rtl_climax.png`): 0 differing pixels
    (the Mach cone and its envelope lines verified).
  - NTSC 480i dec 1 (`rtl_ntsc.png`): both fields 0 differing pixels. SD shows the centre 4:3 of the 16:9 frame.
  - Osc (`rtl_osc.png`) and selfie.jpg (`rtl_selfie.png`): same structure as the model (the model's lattice and
    camera-cell emulation is approximate, so these are not compared bit for bit); every front stays a clean curve.
  - 1080i59.94 dec 1 (`rtl_1080i.png`): both 540-line fields 0 differing pixels at the 1080 virtual scale.
- Small-raster program_top testbench (`tb/tb_top2.vhd`, 256 x 12 active): output identical to the model for the
  defaults and for Rings + Fade + P12 900 (with the model's knob-glide emulation).
- Single-row line check (`thincheck.py`): none at the defaults; a few short runs only at K4 = 0 (hairline).

## Status

- **v0.1.0 built 2026-11-22 (ROUTER1, all six configs on seed 1), not flashed.** Rebuild:
  `NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay doppler` (an earlier router2 build missed HD Analog
  on seeds 1-5; the final netlist was only built with router1).
- Post-route Fmax: HD Analog 78.97 MHz, SD Analog 76.16, HD HDMI 82.99, SD HDMI 75.63, HD Dual 83.25,
  SD Dual 80.63. 7,201 - 7,220 / 7,680 LC (94 %), 24 / 32 EBR. iceunpack OK x 6.
  `out/rev_b/ron/patchbay/doppler.vmprog` (501,650 bytes).
- Timing note: the engine's FSM decode into the multiply-accumulate was the critical path (72 MHz, router1 seed 1)
  until the datapath got its own process with plain state compares; HD Analog then went from 73.5 to 79.0 MHz.
- Presets are hand-written placeholders (Doppler, At Rest, Sound Wall, Mach Cone, Wave Packets, Scope).

## Cut / deviations

- Twelve rings, not sixteen (the spec's allowed cut): sixteen did not fit (8,409 LC). The default wavelength keeps
  the frame covered; the outermost front is 12 L from its centre.
- The ring tent uses a fixed per-ring ramp shift with the threshold scaled into [16, 32): the line half-width is
  exact on every ring, the soft edge's width varies up to 2x between rings / settings.
- Only the oldest ring has an age cap; Fade On in Rings mode uses the band-age fade (the two halves of a line can
  differ by one band step, ~13 %).
- LC is 7,201 - 7,220 (94 %), about 120 over the brief's ~7,100 (92 %) target; timing closes on seed 1 everywhere.
- S7's default is Bands (the bold tone areas the set aims for); Rings is the switch position. The spec listed Rings
  first.
- The Mach-cone half-angle is asin(1 / Mach) by construction (sin a = 1 / M in the cone comparator); the climax
  render shows the envelope tangent to every ring.
- hd_clock_divisor untouched.
- The shell's line buffer is replaced by chladni's camera cell path (8-px cells, slews), as the brief's camera rule
  asks.
