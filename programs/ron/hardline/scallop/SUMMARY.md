# Scallop (Hardline)

**v0.1.0 built 2026-11-15, not flashed.** Flash `out/rev_b/ron/hardline/scallop.vmprog`.

## What it draws

Seigaiha, the Japanese "blue sea wave" pattern of overlapping concentric-arc fans, and the Art Deco scale /
fan ornament that descends from it, in black and white. Rows of round scales lie over one another like roof
tiles: every other row is offset by half a pitch and the lower rows are in front. Each scale is inked with
concentric arcs. The default frame is the classic pattern: scales 300 virtual px wide (6.4 across the
screen), four white arcs on black, rows stepping down by half a radius.

    lattice   centres 2 R apart along a row; rows s = Overlap x R apart; every other row offset by R
    scale     radius rho = Fan x R; a pixel belongs to the lowest row whose scale contains it
    phi       = n (1 - d / rho) + camera        d = distance to the scale's centre (Round) or |dx| + |dy| (Pointed)
    keyline   a rigid black line at every rim (phi without the camera under 1/2 band; 1/8 band in Fine and Shade)
    ink       Bold: white for the second half of each band.  Fine: a white line 1/6 band wide.
              Shade: a sawtooth across each band, white at its outer edge, in 4 flat steps or a smooth ramp

Nothing moves by itself: with no hand on a control and a black input the picture is still.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | pans the lattice, +-960 virtual px (default centred: a scale sits on the screen centre) |
| K2 | Y | pans the lattice, +-960 virtual px |
| K3 | Period | R, the lattice radius, 20 .. 300 virtual px, exponential (default 150); zooms about the panned centre |
| K4 | Rings | arcs per scale, 1 .. 8, stepped with hysteresis (default 4). Fewer are drawn while a band would fall under 3 pixels (smallest Period x smallest Fan) |
| K5 | Overlap | row step from R / 4 (deep, flattened fans) through R / 2 (default, the classic) to R (shallow scales: whole discs packed like coins), exponential |
| K6 | Camera | how far the camera's luma pushes the arcs: 0 .. 3.4 bands at white, default 100 % |
| S7 | Glide | On (default): X, Y, Period, Overlap, Camera and Fan ease 1/16 of the gap per field, never slower than half a count per field. Off: no easing, the knob acts the same field. A knob within 2 counts of its value is held (pot jitter) in both positions |
| S8 | Form | Round (circular scales) / Pointed (diamond scales and chevrons: the distance is |dx| + |dy|) |
| S9 | Shade | Flat (two-tone) / Shade (each arc band ramps from dark at its inner edge to white at its outer edge, a shell; a thin black keyline separates the scales) |
| S10 | Weight | Flat: Bold (equal black and white bands) / Fine (thin white arcs on black). Shade: Bold = 4 flat steps / Fine = a smooth 64-level ramp |
| S11 | Source | Video (default) / Osc |
| P12 | Fan | the headline: the drawn radius of every scale, R / 4 .. 2 R, exponential. 0 % = small ringed buds on a black ground; they grow, touch at 67 % (default, the classic pattern) and overlap deeper until at 100 % only bold wave crests are left |

## Camera

The camera is never shown. Its luma is multiplied by K6, box-filtered (128 pixels at 1080, 64 at 720p and SD)
and smoothed down the field by a one-pole (1/32 per line at 1080p and 720p, 1/16 at 1080i and 480p, 1/8 at
480i / 576i), then added to the arc phase, so the arcs inside each scale swerve around the picture while the
scale rims (the keylines) stay where they are. A black input adds nothing: K6 = 0 and a black input give the
same picture, bit for bit.

- 25 %: 0.85 band at white (0.4 at mid grey): a gentle swerve;
- 50 %: 1.7 bands at white;
- 100 % (default): 3.4 bands at white (1.7 at mid grey): a bold bend, the subject readable as distorted
  linework, every edge a clean curve (`camera.png`).

The anti-aliasing follows the bend: the edge width is taken from the steeper of the rings' own phase slope
and the camera's (its horizontal step plus its vertical step), so bent edges stay one pixel wide and a bend
too steep for the raster fades to grey instead of aliasing.

Source = Osc feeds the shell's drifting 640 x 540 triangle lattice at the same point at twice the level (up to
6.9 bands at 100 %), so big diamonds of bent arcs drift across. The luma lags the picture by about 50 pixels
to the right at 1080 (the box's centre): invisible, since the camera is not shown. The spec's optional
radius swell is not built (see Cut).

## Engine

All geometry runs in rho-units (the drawn radius = 1.0), so the hit test is "sum < 1", the ring phase is
n (1 - d) and nothing in the pixel path multiplies by a knob.

- **Per field** (vblank zone, "no avid for 1024 clocks", about 560 zone clocks): the glides; one 32-bit serial
  multiplier in 15 slots. Radius, fan and overlap are exponentials from a 256-point 2^(i/256) ROM, shifted by
  a one-hot product: rho-units per pixel and per field line, the half cell 1 / Fan, the row step
  Overlap / Fan, the beam's start value in its cell (pan and zoom about the centre), line 0's row and row
  remainder. Then the pixel size rounded up to 2^k or 2^k x 32/23, the ring count, the keyline width and the
  thresholds.
- **Per line** (62 clocks after each line end, and once in the zone for line 0): the row remainder advances
  (a wrap = the next row, parity flips); for the four candidate rows, |dy| and its square (four 13-step
  products on the field multiplier), stored as dy^2 minus the hit threshold.
- **Per pixel** (25 stages): a wrapping accumulator along the line (cell width 2 / Fan) -> |dx| to the nearer
  centre for each row parity -> two squarers (512 x 16 ROM for the high 8 bits + an 8 x 5 product) -> four
  sums dx^2 + dy^2 - threshold -> the front scale (lowest row hit) and the one behind it -> square root
  (512-point ROM with 5-bit interpolation, three input ranges) -> n (1 - d) + camera -> coverage of the nearest
  tone edge and of the keyline (17 steps each; the division is a shift by a divisor rounded up to 2^k or
  2^k x 32/23, so every edge is 1 .. 1.45 pixels wide) -> tone (0 .. 64) -> luma ROM.
  Pointed uses the same path: the squarers return |dx|, the line engine |dy|, the root is bypassed.
- **Rims**: the pixel just outside a front scale's rim belongs to the scale behind it, multiplied by the
  front rim's own coverage, so the rim is anti-aliased against whatever lies behind.
- **Output**: luma [1 2 1] / 4, then 13/16 toward black (white = 776), then the blanking gate. U = V = 512 on
  every pixel. Generated avid (the shell's parity-locked start), one latency (32) in every mode, virtual
  1080-line coordinates from the timing ID (SD shows the centre 4:3).

## Status

- v0.1.0 built 2026-11-15 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`), every
  config on seed 1. Not flashed.
- Post-route Fmax: HD Analog 84.59 MHz, HD HDMI 82.23, HD Dual 83.89 (HD needs 74.25); SD Analog 84.96,
  SD HDMI 83.23, SD Dual 82.37 (SD needs 27).
- 6,995 .. 7,013 / 7,680 LC (91 %), 14 / 32 EBR. Six bitstreams pass iceunpack. No `emulate_transparency`, no
  carry self-adds (dupin: 0).
- The same sources also built with router2 (the builder default), every config on seed 1, but with less
  margin: HD Analog 78.68, HD HDMI 76.82, HD Dual 74.93 MHz. The shipped vmprog is the router1 build.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline scallop`.

- Model = RTL (GHDL): pixel diff 0 on every render made. 720p60 decimation 1 (918,720 pixels): the default
  frame with a black input, with Osc, and with bear.png at K6 100 %. 720p60 decimation 2: the default; Pointed +
  Fine with X, Y, Period, 7 rings, Overlap and Fan all moved; Shade (4 steps) with 2 rings, shallow overlap,
  small fan; Osc; bear.png; Shade (smooth) with bear.png at K6 59 %; Glide On three fields into a glide of X,
  Period, Overlap and Fan. NTSC decimation 1: both fields. Not rendered in RTL: 1080p and 1080i (the model
  covers them; `camera.png` is the model at 1080p).
- Renders (work dir `.hardline_work/rb_scallop/`): `cmp_final.png`, `sheet.png` (control sheet), `strip.png`
  (8 frames across Fan), `camera.png` (bear.png at K6 25 / 50 / 100 %), `ntsc_check.png`.
- `imodel.py` (work dir) mirrors the RTL section by section.

## Cut / deviations

- **Rings follow the drawn radius** (n arcs per scale at any Fan), not the lattice radius: the spec's formula
  could be read either way. Buds are small n-ring targets and the 100 % frame is bold; with constant ring
  spacing the top of the slider ended in dense grey linework.
- **No radius swell from the camera.** The camera only shifts the arc phase; the scale rims are rigid black
  keylines. A plain phase add without them made rims vanish wherever white met white, and the picture shredded
  at 100 %.
- **Keyline.** Bold has the classic half-band black rim; Fine and Shade get a thin (1/8 band, at least 2
  pixels) black rim line that the spec does not mention. It is never more than half (Bold) or a quarter of the
  row step, so the 100 % frame keeps its white at 1 or 2 rings.
- **Camera smoothing** is wider than the shell's (128-px box and a 1/32 one-pole instead of 32 px and 1/8): at
  the shell's values the 100 % bend had ragged edges. The one-pole only smooths downward.
- **Edges are 1 .. 1.45 pixels wide**, not exactly 1: the coverage divisions are shifts (the dividers did not
  fit). The front rim's edge sits half a pixel outside the scale's radius.
- **Square root** by ROM + interpolation: within 1 LSB (rho / 4096) down to rho / 16; a few LSB at the very
  centre of a scale, where no edge can be seen.
- **X / Y pan +-960 virtual px** on both axes (Y is not scaled to the screen height).
- **Under 2 px** the bands are not flat grey: the ring count drops instead (3 pixels per band at least), and a
  camera bend steeper than 1/4 band per pixel fades to grey.
- **SD and 1080i**: the same picture (SD = the centre 4:3), but the anti-aliasing pixel is the field line
  (4.4 virtual lines at 480i, 2 at 1080i), so edges are about two pixels wide horizontally there and rings are
  dropped sooner; at the smallest Period and Fan the buds are one or two field lines tall and nearly vanish.
- `field_n` is passed through one register (the core ties it high on every input).
