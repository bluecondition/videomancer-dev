# Trellis

A three-oscillator colour synth, built from a reference still of a classic
analog-synth pattern: pink horizontal bands bent into arcs, green columns that
taper down the screen, and a blue lens lattice whose eye-shaped holes are
filled with a fine vertical grating.

Three independent 2-D wave fields are drawn straight into the red, green and
blue guns and summed additively, so every overlap makes a new colour
(R+B pink, R+G yellow, G+B cyan, all three white).

| Gun   | Field                                              | Look                         |
|-------|----------------------------------------------------|------------------------------|
| Red   | sin(wy·y + m·sin(wx·x))                            | FM-bent horizontal bands     |
| Green | cos(wx·x) + taper(y)                               | columns narrow at the top, wide at the bottom |
| Blue  | sin(wy·y + m/2·sin(wx·x)) + a·cos(wx·x), holes × grating | lens lattice, stripes under glass |

Each field is thresholded (Width) with a soft or hard knee, so the picture is
made of shapes, not gradients.

## Controls

| Control | Name     | Function |
|---------|----------|----------|
| K1      | Bands    | band rows per screen height, 1 .. 8 |
| K2      | Columns  | columns per screen width, 1 .. 8 (also the bend and lattice rate) |
| K3      | Stripes  | grating lines per screen width, 8 .. 72 |
| K4      | Width    | shape threshold: hairlines .. fat; centre = half duty |
| K5      | Speed    | motion, bipolar, centre = still: bands scroll, bends travel, the lattice counter-scrolls, stripes slide |
| K6      | Hue      | rotates every colour around the wheel |
| S7      | Mirror   | the right half reflects the left |
| S8      | Negative | every gun inverted |
| S9      | Grating  | Holes: stripes only inside the lattice holes / Screen: a line screen cut into every shape, width-modulated by the field |
| S10     | Edges    | Soft / Hard |
| S11     | Stripes  | Vertical / Diagonal |
| P12     | **Warp** | bend + lens pinch + column taper: straight at 0, fully folded at 100 |

All knobs glide (one pole, ~8 frames); Speed has a centre deadband.

## Engine

No framebuffer; one pixel per clock at the full pixel clock.

- Four 20-bit phase accumulators step per pixel (modulator, column, lattice,
  grating) from a per-line start phase; two step per active line (band and
  lattice rows) plus a taper ramp.  Mirror retraces them from the centre
  column.
- Every increment is cycles / measured width (or height) from a 28-bit
  serial divider in the vblank sequencer, so the pattern tiles the screen
  the same at SD and HD, progressive or interlaced.
- Six 256 x 10 sine ROMs (one EBR each) supply the waves; one more serves
  the hue angle.  Three 256 x 32 colour ROMs turn each 8-bit gun into its
  (Y, Cb, Cr) contribution, so RGB -> YUV is three adds; a (Cb, Cr) rotation
  (four split multipliers) follows for Hue.
- Output conditioning for HD HDMI: [1 2 1]/4 on Y/U/V, luma capped at 764.
- Clean generated avid (cubist recipe), blanked to neutral outside it.
  Latency 21 clocks in every mode.  13 EBR.

## Status

v0.1.0 — built 2026-10-16, 6/6 seed 1 (HD 84-92 MHz, 78% LC), not yet flashed.  Presets are placeholders until captured
from the hardware with `capture_preset.sh`.
