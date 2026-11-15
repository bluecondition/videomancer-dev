# Pinwheel (Kinetic)

v0.1.0 built 2026-11-14, all six configs, not flashed. Flash `out/rev_b/ron/kinetic/pinwheel.vmprog`.

## What it draws

A field of rotors, one per square cell: pinwheel quilts, op-art fans, a wall of small turbines. Each rotor is a
disc of N blade pairs drawn as alternately coloured sectors (palette A and B) with a hub dot in the accent
colour; one of its A blades is drawn in the accent colour too (the marker), so a rotor's angle reads like a
clock hand and the travelling wave shows in a still. Twist bends the blades: the sector boundaries sit at
angle + Twist x radius, so straight fans wind into spirals. At the defaults (Citrus palette) 9 x 5 orange and
grey pinwheels with one red blade each sit on navy, their blades bent a quarter turn, the red blades fanning
round as a wave of rotation crosses the field on the diagonal.

## Element and state

- Element: the pinwheel of one cell of pitch P. Cell (0, 0) is centred on the screen centre, so Scale zooms
  about the centre.
- State: its rotation angle alpha (13 bits per turn).
- theta = Spread x index - phase. Linear: index = i + j (the wave runs down the diagonal). Radial: index =
  max + min / 2 of (|i|, |j|), an octagonal ripple from the centre cell.
- alpha = Depth x wave(theta) + source. wave is a sine, or Snap: the first sixteenth of the sine stretched over
  the quarter cycle (75 % dwell, eased flips).
- Blades: sp = N x (angle - alpha - Twist x r) mod one turn; A where sp < 1/2, B elsewhere; blade pair 0's
  A blade is the marker.
- Hand = Alternate mirrors every other cell (checkerboard), so neighbours wind and turn opposite ways.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | cell pitch 64 .. 477 virtual lines, exponential, glided (default 215: 9 x 5 rotors) |
| K2 | Shape | blade pairs: 2, 3, 4, 5, 6, 8, 10, 12 (eight steps with hysteresis; default 4) |
| K3 | Depth | rotation swing 0 .. 1.25 turns, square law (default a quarter turn; near the top the rotors spin right round and back) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Colour | the 16 shared palettes with hysteresis: ground, A and B = blades, accent = hub and marker blade (was K6 until v0.1.1) |
| K6 | Camera | camera strength 0 .. 100 %, default 50 %: gain k / 256 on the held source (0 = as with a black input, 25 % = the v0.1.0 strength, 50 % = twice it, 100 % = four times); scales Osc too. Spread (K5 until v0.1.1) is fixed at 409 (0.08 cycle per cell) |
| S7 | Wave | Smooth / Snap (rotors dwell and flip) |
| S8 | Field | Linear (diagonal wave) / Radial (octagonal ripple from the centre) |
| S9 | Hand | Alternate (neighbours are mirror images) / Same (every rotor winds and turns the same way) |
| S10 | Tile | Disc (round rotors with ground between) / Square (the sectors fill the whole cell: quilt blocks) |
| S11 | Source | Video / Osc |
| P12 | Twist | 0 % straight-bladed fans .. 100 % two turns of bend at the cell edge (square law, glided): tight spirals |

Presets: Pinwheels (defaults), Quilt, Op Fans, Whirlpools, Flip Dots, Spiral Field.

## Video

- Source = Video: the luma (>> 2, less 16) is smeared along x by a 1/64 one-pole filter and sampled ONCE per
  cell (at the cell's last column, on the row just above the cell row) into a 256-entry RAM, then held for the
  whole cell, so every rotor stays rigid. It does two things: it adds to the rotation (alpha += 10 x luma: white
  = 0.27 turn) and it swells the disc (radius = 75 % + 25 % x luma of the full 0.46 P). A bright area shows as
  bigger, further-turned rotors.
- Black input: the offset is zero, every disc is at 75 % and the wave alone runs.
- Source = Osc: a two-axis triangle lattice (640 x 540 virtual px) drifting diagonally replaces the luma at
  twice the camera's gain on both the rotation and the size (the size clamps at 100 %), so diamonds of big
  rotors drift across the field.
- In Tile = Square there is no disc edge, so the source only turns the rotors.

## Engine

Chassis sections lifted from louver.vhd, marked [A] .. [F] in pinwheel.vhd.

- Per field (vblank zone = no avid for 1024 clocks): knob glides (Scale, Twist), one 24-step serial multiplier
  (pitch exponent, depth^2, twist^2, twist / pitch, spread^2, rate), the palette, three adder-only fills (the
  blend table, 2 x 9 x 9 entries; the disc radius by source value, 256 entries; the count that finds the radius
  where the twist has turned one radian), and the cell origin (step back from the centre along x, then y).
- Per line: the y offset inside the cell row, |dy| and its sign, the row's parity and Spread x j
  (wrap candidates pre-registered).
- Per pixel (29 stages + 4 output stages + 4 px of filter delay; one latency, C_LAT = 36, in every mode):
  x offset in the cell, cell parity, Spread x i -> theta -> quarter-wave sine ROM -> Depth x wave -> alpha;
  |dx|, |dy| -> max, min -> reciprocal ROM (512 x 16: 11-bit mantissa + exponent) -> ratio = min x mantissa
  (gated-sum multiply) >> exponent -> atan / sec ROM (1024 x 16) -> angle = ratio + D, radius = max x sec ->
  unfold by octant (mirrored in odd cells) -> minus alpha, minus Twist x radius -> x N -> distance to the
  nearest blade edge x radius (3-bit mantissas and a shift) -> coverage in eighths; disc edge and hub coverage
  from the radius -> ONE read of the blend table -> output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
  resynced 8-sums, luma [1 2 1]/4 delayed to the same group delay, luma 13/16 toward black) -> blanking gate.
- No CORDIC, no square root, no divider. Every multiply is a sum of gated operands (9 x 11, 9 x 8, 10 x 10,
  13 x 4, 10 x 8), each over its own stages; block-RAM data gets a fabric register before arithmetic; no RAM
  ever reads the address it is writing (no_rw_check on all of them).
- Angle resolution 1 / 8192 turn (0.2 px at a 256 px radius). The blade-edge AA caps the radius where the
  twist's radial gradient takes over, so tight spirals stay soft instead of aliasing.
- Generated avid (parity-locked start), virtual 1080-line coordinates from the timing ID, colours in the
  hardware convention (U register = Cr), luma 10 bits and chroma 8 bits up to the output filter.

## K6 camera pass (2026-11-15)

v0.1.1. K6 is now **Camera**, the camera strength (Ron's rule for the Heirloom / Kinetic / Patchbay sets).

- What it does: K6 scales the per-cell source value (smeared luma, or the Osc lattice) before it is held, so it
  scales both things the source drives: the extra rotation (10 x source, 13 bits per turn) and the disc swell
  (75 % + 25 % x source, clamped at the full disc). Source = Osc keeps its 2x on top.
- Gain law: linear, gain = k / 256 (k = the knob, 0 .. 1023). Decoded per field in the vblank sequencer (zone
  clocks 96 .. 99): the knob is latched with a +-4 deadband (the ends snap to 0 / 1023), then the gain eases 1/8
  of the gap per field. Applied with one 8 x 10 gated-sum multiply over three stages on the smeared sample just
  before it is written into the hold RAM (now 10-bit values; the write lands 3 px later, invisible against the
  64 px smear). The rotation term wraps modulo a turn; the radius index clamps at 255.
  - 0 % = gain 0: the picture is bit-identical to a black input (checked in GHDL: 0 differing pixels);
  - 25 % (256) = 1x, the v0.1.0 strength;
  - 50 % (512, default) = 2x: mid-grey and brighter cells reach the full disc, the rotation offset doubles
    (white = 0.53 turn); still clean, rigid rotors;
  - 100 % (1023) = 4x: nearly every lit cell is a full disc and the markers fan through up to 1.1 turns;
    the camera shows as rotation. No washout (the palette is untouched).
- Frozen: K5 Spread at 409 (its default: 0.08 cycle per cell). Spread was the weakest knob on the control
  sheet (min / default / max look alike in a still; Rate, the only speed control, was not touchable).
- K6's old function (Colour, all 16 palettes) moved to K5 with its hysteresis. Presets: Colour values moved
  to K5; the Spread values of Quilt (0), Flip Dots (600) and Spiral Field (250) are gone, so those presets
  now run at Spread 0.08 (Quilt's square blocks no longer move in unison).
- Renders: `.kinetic_work/rb_pinwheel/k6sheet.png` (selfie, 720p60 at decimation 2: K6 0 / 256 / 512 / 1023
  and Source = Osc at 512).
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic pinwheel`): router1, every config on seed 1.
  Post-route Fmax HD Analog 90.84, HD HDMI 84.29 (worst HD), HD Dual 84.60, SD Analog 84.20, SD HDMI 83.51,
  SD Dual 86.66 MHz; 6,948 / 7,680 LC (90.5 %, was 6,616), 23 / 32 EBR. vmprog:
  out/rev_b/ron/kinetic/pinwheel.vmprog (v0.1.1; this supersedes the v0.1.0 builds under Status). Not flashed.

## Status

**Shipped vmprog: the ROUTER1 rebuild of 2026-11-14** (same sources). Post-route Fmax: HD Analog 83.91 MHz; SD Analog 86.51 MHz; HD HDMI 80.59 MHz; SD HDMI 81.75 MHz; HD Dual 89.14 MHz; SD Dual 94.13 MHz. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic pinwheel`. The router2 figures below are the first build, kept for the record; it was timing-marginal.


- v0.1.0 built 2026-11-14 through the shared queue with router2 (the builder default). Not flashed.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 81.37 MHz | 6607 |
| SD Analog | 2 | 81.74 MHz | 6605 |
| HD HDMI | 4 | 78.08 MHz | 6614 |
| SD HDMI | 1 | 74.46 MHz | 6613 |
| HD Dual | 1 | 79.69 MHz | 6616 |
| SD Dual | 1 | 85.40 MHz | 6612 |

- 6,616 LC (86 %), 23 of 32 block RAMs. HD needs 74.25 MHz, SD 27. Six bitstreams pass iceunpack.
- Seeds other than 1: SD Analog seed 2 (seed 1 stalled router2 on two overused wires), HD HDMI seed 4 (seeds 1
  and 2 stalled the same way, seed 3 routed but missed timing). The stalls were cut short by hand (SIGALRM to
  the builder's timeout wrapper, which takes its own "stalled, next seed" path). router2 is stall-prone on this
  netlist: if a rebuild stalls again, use NEXTPNR_ROUTER="--router router1" (dupin.py on the synth json: no
  carry self-adds). router1 was not tried.
- Rebuild: `./build_set.sh kinetic pinwheel`.
- GHDL render = integer model (`.kinetic_work/rb_pinwheel/imodel.py`):
  - defaults, black input, Rate running, full 720p60 frame (decimation 1): 0 mismatching pixels of 918,720
    (the RTL frame is the model's frame 2, 4 columns to the right);
  - Source = Osc, Rate still, 640 x 360 window of 720p60: 0 of 226,080 (model windowed to the sim raster);
  - camera (selfie.jpg), 640 x 360 window: 6 % of pixels differ from the model, mean 0.4 code. Not traced:
    the likely cause is that the tester and the model resample the photo differently, which moves a few cells'
    held luma by a step. Every rotor in the render is a clean rigid disc;
  - motion: a second capture 7 frames later (640 x 360 window) = the model's frame 9: 0 of 226,080;
  - NTSC (720 x 243, decimation 1, Rate still), both fields: 0 of 172,044 each. The SD frame shows the centre
    4:3 of the HD picture: the same five rows, 6.7 of the 9 columns;
  - 1080i (960 x 270 window per field, Rate still), both fields: 0 of 255,960 each. The two fields draw the
    same lines (no half-line offset between fields, as in the pilots).
- Knob values other than the defaults were checked in the model only (control sheet), not in GHDL.
- The model's output has no single-row hard lines (thincheck.py: 0 runs of 24 px in 11 settings, 720p and 1080p).
- The standalone yosys run of the first RTL had 0 `emulate_transparency`; the radius table added after it
  carries the same `no_rw_check` attribute but its final netlist was not re-checked.
- Renders: `.kinetic_work/rb_pinwheel/cmp_final.png`, `sheet.png` (control sheet, from the model), `strip.png`
  (eight phases of one wave period, from the model).
- The picture sits 4 real pixels right of the screen centre (the generated avid starts 4 clocks after the
  source's; the geometry is not offset to compensate), and the GHDL capture shows 4 blank columns at the left,
  as in the pilots.

## Cut / deviations

- Added to the spec: the MARKER blade (one A blade per rotor in the accent colour). Without it the rotation of
  a 4-fold rotor is invisible in a still and Depth, Spread, Snap, Radial and Osc all looked alike on the sheet.
- Added to the spec: the source also swells the disc (75 % .. 100 %), because rotation alone made Source = Osc
  and the camera too weak. With a black input every disc is therefore at 75 % of the full size.
- Shape steps through 2, 3, 4, 5, 6, 8, 10, 12 blade pairs (7, 9 and 11 are skipped).
- Depth tops out at a 1.25-turn swing (back and forth), not unbounded continuous spinning.
- Tile = Square: neighbouring blades meet in one continuous field only where the neighbours' states are equal
  (Spread 0 or Depth 0, with Hand = Alternate); otherwise each cell is a square quilt block with straight seams.
- The blade-edge AA gain is quantised (3-bit mantissas), so edge softness varies by up to about +-12 %. At 720p
  the edge ramps are about 2/3 px wide; on SD the pixel is not square in virtual coordinates and the AA uses
  the x size only.
- The marker blade's rim pixels blend A (not the accent) with the ground: a sub-pixel fringe.
- The source smear is 64 real pixels at every resolution; the sample point is the cell's right edge on the row
  above the cell row, so a rotor shows the picture just above-right of its own centre. Row 0 of the frame uses
  the previous field's last sample for the top cell row.
- Scale's pitch is quantised to half a virtual pixel, so a slow Scale glide moves the outer cells in sub-pixel
  to 2-px steps.
- `field_n` is delayed with the syncs; nothing in the picture depends on it.
