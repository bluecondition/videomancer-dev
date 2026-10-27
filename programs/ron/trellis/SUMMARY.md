# Trellis — summary

**What**: three-oscillator RGB synth reproducing a reference analog-synth still
(pink FM-bent bands, green tapered columns, blue lens lattice with
grating-filled holes).  Synthesis program, no input video.

**Status**: v0.1.0 built 2026-10-16, 6/6 seed 1 (HD Analog 85.2, HD HDMI 83.9,
HD Dual 92.2 MHz; ~5980 LC = 78%; 13 EBR), not flashed.  Presets placeholders.

**Design notes**
- Fields in signed 10-bit units (+-511); threshold t = 384 - Width*0.625
  (+384 hairlines .. -255 fat); soft knee spans 512 units, hard spans 64.
- Warp (P12, glided g): PM depth = g*3 (phase units, +-1.5 cycles at full),
  lens pinch a = g*3/8 /256 (0 .. 1.5), taper amplitude A = g/2 (ramp from
  -A at the top row to +A at the bottom).
- Speed: deadband +-14 about 512, velocity = spd^2/16 phase units per field;
  bands x2, modulator x1, columns -x1, lattice x -x1.5 / y x0.5, stripes x4.
- Hue: K6 top 8 bits index the sine ROM (sin, cos at +64), 8-bit cos/sin
  split into 4-bit hi/lo partials; U' = (U c - V s) >> 7.
- Colour ROMs: BT.601 at 0.8 gain, per gun (Y, Cb, Cr)/256 coefficients
  R (210, -121, 360), G (413, -238, -301), B (80, 360, -58); white Y = 764.
- Hardware convention: data_out.u carries Cr, data_out.v carries Cb.
- Line constants (band/lattice row phase, taper) latch on the generated
  avid's rise, not hsync: SD's front porch is shorter than the 21-stage
  pipeline.
