# Guilloche (Hardline)

**v0.1.0 built 2026-11-22, not flashed.** Flash `out/rev_b/ron/hardline/guilloche.vmprog`.

## What it draws

Guilloche: the engine-turned lace of banknotes, share certificates and watch dials. A rose engine cuts many
copies of one wave, each shifted a step in phase, and the copies overlap into a woven band. Black and white
only; U = V = 512 on every pixel.

The default frame is a stack of banknote borders: three and a half horizontal lace bands, each framed by a pair
of thin ruled lines. Each band is three laces XORed: an outer lace of 4 sine copies (big scalloped white
lozenges along both edges, thinning to points at the crests) with two longer, narrower laces inside it (2
copies at twice and at four times the wavelength), which weave a braid of medallions down the band's middle.
At power-up the laces slide gently: the outer lace one way, the inner laces at -3/4 and +2/3 of its speed, and
alternate bands the opposite way, so the medallions keep re-forming.

    s = (c - y) / A                  the band coordinate (up = +), per line
    h(s)                             half-width of the cam above level s (triangle / sine / cycloid, blended)
    P = M (x - X) / lambda + drift + camera,   H = M h(s)
    pixel = stripe(P + H) xor stripe(P - H)    (the parity of the copies above the pixel; per lace, then xor)

Closed form, verified against a brute-force drawing of the M curves (float prototype, 0 mismatches).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | slides every lace along its band (a pan), +-1024 virtual px. Default 512 = centred |
| K2 | Y | moves the stack of bands up and down, +-512 virtual px (the bands wrap). Default 512 |
| K3 | Period | the wavelength, 72 .. 1440 virtual px, exponential. Default 560 (371 px, about five waves across) |
| K4 | Lines | M, the copies in the outer lace: 2, 3, 4 (default), 6, 8, 12, 16, 24 (the inner laces carry 2, 2, 2, 4, 4, 6, 8, 12). Labelled steps; a new step is taken 8 counts inside it, with both ends of the travel excepted |
| K5 | Drift | bipolar speed, centre +-24 counts = still; exponential 2^-12 .. 2^-2 copy period per field (one copy period every 68 s just off centre, a quarter per field at the ends). Up = the outer lace of the centre band slides right. Inner laces at -3/4 and +2/3 of the outer lace's speed in px, alternate bands the opposite way. Default 680 (+33 %): about 10 px per second at 1080p |
| K6 | Camera | how far the picture's luma pushes the laces along their bands; 1280 virtual px at full white at 100 % (default). 0 = the black-input picture exactly |
| S7 | Glide | Off: the knob values read at this field start are used as they are. On (default): X, Y, Period, Camera and Cam ease 1/16 of the gap per field (never slower than one count, landing exactly), Drift's speed 1/8; +-1-count hold against pot jitter. Glide eases the speed, never the phase |
| S8 | Form | Bands (default): the stacked borders / Sheet: one lace as tall as the screen |
| S9 | Layers | Single: the outer lace alone (a clean harlequin of lozenges) / Triple (default): the three laces XORed |
| S10 | Weight | Bold (default): the parity fill / Fine: the curves themselves as 2-virtual-px lines (the engraving) |
| S11 | Source | Video (default) / Osc |
| P12 | Cam | the wave shape: 0 % triangle (straight edges, a lattice of sharp diamonds), 50 % sine (default, the classic guilloche), 100 % cycloid with the cusps up (Gothic lace); a continuous morph between |

## Camera

`program_type = "processing"`. The camera is never shown. Its luma (6 bits) x the gain -> a box over 96
virtual px (96 / 64 / 48 pixels at 1080 / 720p / SD) -> a vertical one-pole IIR (about 32 virtual lines) in a
half-resolution line store. The result c0 is a displacement in cycles of the outer lace and is added to every
lace's phase scaled by its own copy density, so all three laces move by the same distance: the band warps as
one engraving. The displacement is fixed in pixels (1280 virtual px at full white at 100 %, 640 at 50 %, 320
at 25 %), so the warp is the same at every Period; below a wavelength of 82.6 px it is capped at 15.5 cycles.

On bear.png (`camera.png`, integer model): 25 % is a gentle swerve of the lozenges; 100 % turns the head into
contour swirls (eyes and muzzle visible as concentric whorls in the bands), every edge a clean curve. The
camera's own gradient enters the anti-aliasing footprint, so squeezed lace stays 1 px soft.

Source = Osc substitutes the shell's drifting 642 x 541 triangle lattice at the same point, at a quarter of the
full-white camera level: the lace is compressed and stretched in a slowly drifting diamond lattice.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): holds and glides (6 channels), the Drift decode
  (exponent ROM), then one serial 16-step multiplier in 16 slots: lace 1 / 2 drift steps, Period through a
  2^(i/256) ROM -> 1 / lambda, per lace the phase per virtual px, per pixel and at the pre-pixel (pan), the
  camera gain, the footprint base. The drift accumulators step once per field (copy phase, modulo one copy
  period, so the motion never jumps).
- **Per line**: a line engine with its own serial multiplier computes the NEXT line during the current one
  (about 260 clocks, fits the SD line): the band coordinate (one adder, wrapping by the pitch), the rule's
  coverage of the line, and per lace s = -v / A -> the sine / cycloid ROMs (1024 x 16 each) -> the P12 blend as
  two accumulated products -> H = M h (mod 2) and the per-line gradient |dh|.
- **Per pixel** (latency 36 in every mode): three beam DDAs (Q1.20, modulo 2 copies) + the camera phase ->
  P +- H -> per lace the distance to the nearer stripe edge and the parity; the footprint |gx0 + dc0/dx| +
  |dc0/dy| + gy_i normalised -> a 1 / footprint ROM -> a log shifter and one 6 x 5 product = the distance in px.
  All six stripes: the parity, the nearest edge, the strongest fade -> one coverage (17 levels, edges 1 px);
  stripes under 4 px fade to grey by 2 px. Band frame and rule -> luma 64 .. 940 -> [1 2 1] / 4 -> 13/16 toward
  black (white 775) -> blanking gate. Chroma constant 512.
- Chassis: the Kinetic riley shell via hardline/tweed (timing-ID virtual-1080 steps, generated avid with the
  parity-locked start, source field with the dazzle line-0 Osc fix, luma conditioning). `field_n` passes
  through one register.

## Status

- Built 2026-11-22 through the shared queue with **router1**, every config on seed 1. Not flashed.
- Post-route Fmax (builder, routed): HD Analog 87.02 MHz, HD HDMI 82.12, HD Dual 88.71 (HD needs 74.25);
  SD Analog 89.57, SD HDMI 86.18, SD Dual 83.47. Six bitstreams pass iceunpack.
- 7,015 .. 7,034 / 7,680 LC (91.6 %), 14 / 32 EBR. Standalone yosys: 3,980 LUT + 2,769 FF, no
  `emulate_transparency`; dupin: 0 duplicate-input cells. scan.py: 0 sites.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline guilloche`. Router2 was not tried.
- Model = RTL (GHDL captures diffed against `imodel.py`, which mirrors the RTL's widths, ROMs, line engine and
  source field), pixel diff 0 on every render compared:
  - 720p60 decimation 1 (921,600 pixels): the power-up defaults with Glide On and the default Drift
    (= `rtl_default.png`); Source = Osc; bear.png at K6 100 % (the model fed the tester's own stimulus luma);
  - NTSC decimation 1, both fields (720 x 243 each; the second field one drift step later): the same picture as
    the centre 4:3 of 720p;
  - 720p60 / 2 (230,400 pixels): Drift at centre; Drift +100 % and -100 %, each at fields 3 and 5; Lines at
    0 and at 1023 from power-up (both stepped ends select their step); Fine + Sheet + cycloid blend (P12 800) +
    Lines 12 + X / Y off centre + Period 41 % + Drift -61 %; Single + Lines 3 + triangle blend (P12 200) +
    Period 0 + Drift +100 %; Fine + Lines 24 + Period 15 % + P12 100 % (the fade to grey); bear.png; Osc;
  - Glide On, 720p60 / 2: 3 fields after power-up with X 700, P12 900, Drift 1023 set: equal to the model with
    the glide accumulators stepped as the RTL does (X and P12 1/16 per field, the Drift speed 1/8, the drift
    integrating the eased speed).
- The drift steps once per field (the model needed exactly one more step per field, at 720p and on both NTSC
  fields). Direction (model, equal to the RTL): Drift up slides the centre band's outer lace right and the band
  above it left.
- Renders in `.hardline_work/rb_guilloche/`: `rtl_default.png`, `cmp_final.png`, `sheet.png`, `strip.png`,
  `motion.png` (model: default Drift 30 fields apart, and both ends), `motion_rtl.png` (GHDL, +-100 %, fields 3
  and 5), `camera.png`, `lines.png` (every Lines step).

## Cut / deviations

- **Nearest-edge anti-aliasing.** Each lace's coverage comes from the nearer of its two stripe edges, and the
  three laces from the nearest of the six, with the XOR of all six parities. The first RTL computed every
  stripe's coverage and soft-XORed them (148 % of the chip); the model renders of the two differ by 0.1 codes
  on average. X-junctions get single-edge AA (corners very slightly rounded).
- **Fine lines are 2 virtual px wide everywhere**, not 1/6 of the copy spacing: the duty-based width needed a
  second barrel per lace and only widened the lines by up to a quarter.
- **The camera displacement is fixed in pixels** (1280 virtual px at 100 %), not "2 wavelengths": in cycles the
  warp folded the lace into smears at long periods. At the default Period it is 3.5 wavelengths.
- **Osc is at a quarter of the full-white camera level**, not 2-4 x the camera: at half it already folds the
  lace into horizontal smears. It is still an unmistakable flip (a regular drifting compression lattice).
- **Drift wraps one copy period** (lambda / M, the lace pattern's own period), which is exactly seamless; the
  top speed is a quarter copy period per field so it never strobes into standing still.
- **The inner laces' copy counts are even** (2, 2, 2, 4, 4, 6, 8, 12): an odd count inverts the band below its
  envelope with a hard horizontal edge. With Lines 3 the outer lace is odd, so the band's lower half is light
  with dark points (a deliberate one-sided lace on that step).
- **Fine lines are clipped at their lace's envelope**: the bottom of an inner lace's trough loses the outer half
  of its line (a flat-bottomed curve over about one pixel).
- **SD**: the same picture (the centre 4:3), but a 480i field line is 4.4 virtual px, so the ruled lines (8 virtual
  px) are about two field lines and the finest laces (Period low, Lines 16 / 24) fade toward grey sooner. 720p
  fades 1.5 times sooner than 1080p.
- Not verified on hardware: anything.
