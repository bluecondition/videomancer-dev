# Gothic — nested gothic arches of blue leaves and wavy red lines over rolling green stripes

**v0.1.1 built 2026-11-10, NOT flashed.** v0.1.0 was built the same day.
v0.1.1: the camera luma is smoothed before it bends the field (8 px box plus a vertical IIR), the default camera bend
is lowered (Etch 120), and the Osc has its own x32 multiplier. This fixes the hairy per-line jaggies on cyan lines and
ribbon edges with a live camera.

Heirloom phase 3 (video clips), from clip WIN_20230817_00_06_49 at 0.5 s (`photos/vid/WIN_20230817_00_06_49_t0p5.png`).
Nested pointed arches of ribbons stand over soft green/black horizontal stripes. The ribbons carry blue leaf-shaped cores
with dark green flanks, patches of wavy red lines on black, and a thick cyan line along the edge of each gap. In the clip
the arches sink, the stripes roll down and the red patches rise. Presets are hand-written placeholders.

## v0.1.1 (2026-11-10): smoothed camera etch
The review of v0.1.0 found that at the default Etch the selfie turned the cyan lines and ribbon edges into hairy per-line
jaggies, which read as tearing on hardware.
- **Smoothing.** The camera luma, or the Osc lattice, is smoothed before it enters the arch field:
  - pair sums at half horizontal resolution;
  - an 8 px running box (the last 4 pairs, restarted every line);
  - a vertical IIR f = (prev + new) / 2 in a second half-resolution line buffer.
- **Line buffers.** The pixel path reads its own copy of the smoothed line 8 px ahead of the write. Both 1024 x 8 RAMs
  are collision-free and carry `no_rw_check`. EBR is unchanged at 23 (two half-resolution buffers replace the old
  full-width one), and the old buffer's transparency emulation is gone.
- **Default camera bend:** Etch 120 = 0.032 cycles per full luma, about 0.21x v0.1.0's 0.15. K6 at max still etches
  strongly (2.3 cycles). The etch gain slice is 2x finer.
- **Osc:** its own x32 shift of the etch product, 0.95 cycles at the default (v0.1.0: 0.9). The flip is unchanged.
- **Checks.**
  - On photos/selfie.jpg the cyan lines and ribbon edges stay clean curves (cmp_final.png, c_cam_cmp.png).
  - The GHDL 720p dec-2 selfie frame equals `imodel.py` on 100.0 % of interior pixels.
  - The black-input frame is unchanged.
- **Cost:** +~130 LC after trims (16-bit wobble line phase, the band test without a register, one smoothing pipe
  stage).

## Reading of the photo
Working files are in `.heirloom_work/rb_gothic/`: notes.md, fit1 / fit2 / fit5 (field fits), fitq2 (key fit), proto.py,
imodel.py, thincheck.py, isheet.py.
- **Arch field.** Every ribbon edge is a contour of F = -PH tri(u) + CX x + warp(x) + g(y).
  - Arch phase u = (x - XP) / PH, with PH 965.7 px and apex XP 1020.75. Horizontal ramp CX 0.2096: falling sides move
    1.6x more than rising ones in the measured row-to-row shift field.
  - g(y) = 3263 sin(2 pi (y + 2150) / 11950) - 3129 (rms 6.4 against the fitted cubic). Sides lean ~0.75 px/line at the
    top and run vertical below y ~840.
  - warp(x) is a screen-locked bend: Fourier terms k = 1..3 of up to +-34 F-px.
- **Colouriser.** It wraps every 333.8 F-px.
  - The gap band f in [0.435, 0.759) is keyed transparent, so the stripes show through. The gap-mask fit on
    bright-stripe rows is 87.9 % accurate (baseline 71.8 %, no warp 84.2 %).
  - Then a ~16 px cyan line, dark green flanks and a blue core (f 0.94-0.12).
- **Red vs blue.** This is not a function of f. It follows the arch phase and y: Q = 0.658 * 2 tri + 0.321 u + 0.905 Y
  - 1.187 Y^2 explains 0.27 of 0.69 nats. Red where frac(Q) is in [0.68, 0.17) or [0.33, 0.47).
- **Red lines.** Near-vertical wavy lines ~35 px apart, wobbling +-25-30 px with a ~117-line period, with loops where
  the wobble is deep. Patch edges carry ~8 px / 18-line teeth.
- **Stripes.** Period 173 lines, green peak at y = 152, level clip(0.596 cos + 0.556); green (0, 240, 4).
- **Palette (k-means, 12 clusters).**
  - black (9, 15, 11) 26 %
  - stripe green (0, 240, 4) 12 %
  - dark green (7, 75, 14) 10 %
  - blue (6, 16, 181) 10 %
  - dark blue (7, 33, 91) 8 %
  - dark reds (44, 17, 6) / (80, 27, 7), red (124, 21, 4) / (182, 21, 10)
  - cyan (57, 198, 191) 2.4 %
- **Class shares (still / RTL at defaults).** Black 36.8 / 39.0, gap green 18.5 / 16.4, dark green 12.5 / 12.9,
  blue 15.6 / 12.2, red 14.0 / 16.6, cyan 2.6 / 2.8 %. Gap agreement 89.2 %; 4x4 luma |diff| 7.3.

## Motion in the clip
- **Stripes** scroll down ~106 lines/s.
- **Arch field offset** -36 F-px/s, no horizontal drift: the apexes sink and the sides close in (the central arch's
  right side at y 600 goes 1159 -> 1064 px over 2 s). One colour cycle takes ~9 s.
- **Red patches** rise ~100 px/s.
- **The same patch's other stills:** straight W chevrons and a V-hold slip at 07_58 t5.0; wavy ribbons and cyan-cored
  stripes at 07_58 t0.5. These are the ends of K2, S9 and S7.
- **What the program does with it.** At the default Speed (700): field flow 36.3 F-px/s, stripe roll ~107 lines/s, key
  rise ~103 lines/s, and the red lines ripple. The GHDL frame at Speed 1023 equals the model after 2 fields.

## What it does
1. **x terms as DDAs.** All of them are per field x 1/P: R = CX x / P, and T = PH tri(u) / P as a folded DDA modulo one
   arch period. The warp table is in Q16 cycles at P0.
2. **Line term.** An interpolated sine (1024 x 16, 10-bit fraction) times Ag, plus GOFF / P, minus flow. In the tear
   bands it gets +0.37 cycle; Flip negates the sine part.
3. **Field and frame word.** F = R - T + warp + line term + Etch * smoothed luma (mod 1). f8 reads the frame word: mode,
   envelope, leaf, dark green, red body, wobble.
4. **Key.** Q = 0.656 * 2 tri + 0.320 u + 0.2405 cos(2 pi (Y - 0.381 + rise) / 2) - 0.068 plus 18-line teeth. Region
   tests give red or blue, and a taper at the key edges points the leaves.
5. **Red lines.** Phase x N / 324 plus wob(f) * sin(2 pi (y / 117 + x / 1800 + t)), through a sin^3 profile.
6. **Composition by minimum.** Stripe level / cyan / red waves / blue leaf / dark green, then a 512 x 32 palette ROM in
   hardware convention, luma pre-scaled 16/13.
7. **Output.** A [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V (block-RAM ring, running-sum resync), luma 13/16, blanking
   gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Arches | the arches' horizontal frequency, x0.35 .. x2.8. 512 = the still's ~2 arches across. The amplitude is fixed, so more arches means steeper lancets |
| K2 Curve | the vertical sine's frequency at a fixed zero crossing, +-3 octaves: straight W chevrons (0) .. the still's gothic bow (512) .. wavy sway and lens shapes (1023) |
| K3 Ribbons | colouriser density, nested ribbons per arch, x0.35 .. x2.9. 512 = 333.8 F-px per cycle |
| K4 Speed | bipolar (+-8 deadband, curve ^1.5): the arches sink, the stripes roll down, the red patches rise, the lines ripple. 700 = the clip; 1023 ~4.7x; left of centre = reverse |
| K5 Waves | red line spacing: 13 px fine lines (0) .. 36 px, the still (512) .. 100 px broad looping swirls (1023) |
| K6 Etch | how far the smoothed camera (or the Osc) bends the arch field, square law. 120 = 0.032 cycles per full luma (a gentle bend); 1023 = 2.3 cycles (strong etch) |
| S7 Stripes | Green (the still) / Cyan: white-cyan cored stripes, as later in the same patch |
| S8 Swap | red waves <-> blue leaves |
| S9 Tear | rolling V-hold slips: alternate 512-line bands, rolling with Speed, where the arches jump 0.37 cycle, the ground goes black and red / blue swap |
| S10 Flip | the vertical term inverted: hanging arches and lens shapes |
| S11 Source | Video / Osc |
| P12 Vault | the vertical slope: straight pillars (0, 0.08x) .. the still (400) .. 3x, tight arches cascading down (1023) |

## Video
- **Video mode (the default).**
  - Camera luma, as (Y >> 2) - 16, is smoothed: pair sums at half horizontal resolution, an 8 px running box, and a
    vertical IIR f = (prev + new) / 2 in a second line buffer.
  - It is then added to the arch field times the Etch gain.
  - At the default (Etch 120) the arches bend gently around the picture while the cyan lines and ribbon edges stay clean
    curves. K6 at max etches the picture strongly into the field.
  - With no input, Video mode is the still.
- **Osc mode.**
  - A 640 x 540 virtual-px triangle lattice, drifting one step per field, takes the same smoothing.
  - It rides its own x32 shift of the etch product: 0.95 cycles at the default.
  - The arches fold into lattice loops, an obvious flip that keeps the palette, stripes and ribbons. K6 stays live.

## Engine
- **Shared serial multiplier.** The vblank-zone sequencer and the line engine share one serial shift-add multiplier
  (42-bit accumulator, b up to 24 bits).
  - Zone: 18 slots at zc 64 + 32 k. The exp2 mantissa is read just in time and shifted on the b operand; the x step is
    folded into the constants.
  - Per field: du, u0, 1/P, omega, Ag, the phase step and origin, C0, flow, the wave step, the etch gain, Kt, dv, v0
    and the ramp step.
  - Shared glide on K1, K2, K3, K5, K6 and P12; signed motion steps.
- **Line engine.** Kicked at each line start, it computes the NEXT line in ~66 clocks: the tear band, the interpolated
  sine and Ag product, the stripe level, the key term and teeth.
- **Camera smoothing** on the source side: pair sums, an 8 px box (resets each line), the IIR read-modify-write in the
  `ir` RAM, and a copy into `lb` for the pixel path.
- **Pixel path (20 stages, C_LAT 20 in every mode).** Lagged beam reads instead of carry chains, etch nibble products
  (x32 shift for the Osc), frame ROM, wobble 5 x 7, profile table, palette ROM, box filter, 13/16, gate.
- **EBR (23).** Smoothed-luma buffers 2 x 1024 x 8 (4); sine 1024 x 16 (4); wobble sine (1); warp 512 x 16 (2);
  frame 256 x 32 (2); palette 512 x 32 (4); exp2 (1); P12 vault (1); speed curve (1); box ring (2); sync ring (1).
- **Contracts.** Generated avid; one latency; virtual 1080-line coordinates (SD shows the centre 4:3); hardware colour
  convention (U = Cr); running-sum resyncs (output box, smoothing box).
- **Verification (GHDL).**
  - 720p dec 1 black: 100.00 % equal to `imodel.py`. 720p dec 2: 100.00 %. Speed 1023: 100 % after 2 fields.
  - v0.1.1 selfie at dec 2: 100.0 % with the tester's exact luma.
  - Thin-line check: 0 single-row runs >= 24 px over the default, P12, K3, K2, Tear, Flip, K1 and K5 extremes.

## Status
- **v0.1.1 built 2026-11-10 with ROUTER1, all six configs on seed 1. NOT flashed.**
  - Post-route Fmax: HD Analog 86.10, SD Analog 86.42, HD HDMI 82.72, SD HDMI 80.76, HD Dual 88.57, SD Dual 79.85 MHz.
  - 7198-7225 / 7680 LC (94 %), 23 / 32 EBR. iceunpack OK x6.
  - Output: `out/rev_b/ron/heirloom/gothic.vmprog`.
- **Rebuild:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh gothic`.
- **Stay on router1.** router2 stalled (overused = 2) on this design family. dupin: 0 duplicate-input cells.
- **v0.1.0 (same day):** router1, all seed 1 (HD Analog 81.69, HD HDMI 84.43, HD Dual 82.62 MHz), 7073-7083 LC.
- **Fit history.** The first RTL (an X * 1/P pixel multiplier, 28 slots) packed to ~8,100 LC. It came down through
  multiplier-free x DDAs, the shared multiplier, lagged beam reads, the just-in-time exp2, narrower etch / wobble / wave
  widths and tear bands.
- Presets are hand-written placeholders. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **Camera is softened** (v0.1.1): half horizontal resolution, an 8 px box and a 2-line vertical blend, so fine camera
  detail no longer shows in the etch. There is a ~4-8 px horizontal offset and a 1-line lag, and the first 8 px of each
  line bend slightly while the box fills.
- **Red / blue layout** is a smooth fitted key, so the still's fragmented red patches and organic looped red lines come
  out more regular.
- **Small details not drawn:** the cyan line's white/pink fringe, the light-blue leaf glints, and the black capture
  glitch at the top left.
- **Arch geometry.** Gap mask 87.9 %: some ribbon segments sit 20-40 px off. The screen-locked bend does not scale with
  K3.
- **Osc gain is x32 the camera gain**, so its flip stays strong while the camera default is gentle.
- **Tear** is alternating 512-line bands, not one rolling band (budget).
- **Whites** are capped by the shell's 13/16 luma (~777).
- **Resolutions.** 720p matches 1080p. SD shows the centre 4:3: the outer arch sides fall off-screen.
