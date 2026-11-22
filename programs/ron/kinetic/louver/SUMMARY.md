# Louver (Kinetic #6, pilot)

v0.1.1 (K6 camera pass) 2026-11-15, not flashed (v0.1.0 built 2026-11-14).

## What it draws

Rows of flat two-sided slats, each turning about its long axis, lit from one side so the turn reads as 3-D.
The art it comes from: kinetic louvre walls and tri-vision billboards, venetian blinds turning in a wave, and
twisted-ribbon sculpture. At the defaults (Bauhaus palette) about twelve cream-backed rows of red / blue
ribbons with gold edges fill the screen, a slow wave rolling down through them, each ribbon already twisted a
little along its length.

A slat is drawn as a thin SLAB, not a sheet: width Blade x pitch, thickness pitch/16 + 1.5 px. Its big faces
are the front (palette A, or the incoming picture) and the back (palette B), shaded 0.42 + 0.58 |cos(angle -
light)| with a glint in the accent colour where the face catches the light. Its thin face is the accent
colour, so an edge-on slat is a gold line at least 2.5 px thick and a twisted ribbon pinches to a clean waist
where the gold edge swaps sides.

## Element and state

- Element: a slat of pitch P; slat 0 is centred on the screen centre, so Scale zooms about the centre.
- State: its angle alpha about its long axis (14 bits per turn).
- theta = Spread x index + Twist x u / P - phase, where u is the distance along the slat from the centre.
  - Linear: index = slat number.
  - Radial: index = max + min/2 of (|slat number|, |u| / P), and the twist term uses |u|, so the picture is
    mirror-symmetric about the centre.
- alpha = Depth x wave(theta) + source. wave is a sine, or Snap: the first sixteenth of the sine stretched
  over the quarter cycle (75 % dwell, eased flips).
- Projected half-height H = (Blade P / 2) |cos alpha| + (thickness / 2) |sin alpha|, never more than P/2 - 1
  (closed shutters keep a 2-px ground seam; slats never overlap, so one slat test per pixel is enough).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | slat pitch 16 .. 270 virtual lines, exponential, glided (default 87: twelve slats) |
| K2 | Blade | blade width 20 % .. 100 % of the pitch (thin blades and wide gaps .. closed shutters) |
| K3 | Depth | swing 0 .. 1.25 turns, square law (near the top the slats spin right round) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Colour | the 16 shared palettes with hysteresis: ground = gap, A = front, B = back, accent = glint and thin face (K6 until v0.1.0; Spread is now fixed at its old default 354, a phase step of 0.06 cycle per slat) |
| K6 | Camera | camera (and Osc) strength, 0 .. 100 %, default 50 %: 0 % = no camera (the black-input picture), 25 % = the v0.1.0 gain (white opens a slat 0.27 turn), 50 % = twice it, 100 % = four times (see "K6 camera pass") |
| S7 | Wave | Smooth / Snap (slats dwell and flip quickly) |
| S8 | Field | Linear (the wave rolls through the rows) / Radial (an octagonal ripple from the centre) |
| S9 | Axis | Across (horizontal slats) / Down (vertical slats) |
| S10 | Face | Palette / Video (the front face carries the picture, shaded; with Osc, the lattice's colours) |
| S11 | Source | Video / Osc |
| P12 | Twist | 0 % rigid slats .. 100 % half a wave cycle per pitch length along the slat (square law, glided): chains of beads |

Presets: Ribbons (defaults), Tri-Vision, Ripple Blinds, Helix Field, Bead Curtain, Picture Blinds.

## Video

- Source = Video: the luma (>>2, less 16) is smeared along x by a 1/64 one-pole filter and HELD per slat, so
  the picture bends the ribbons without shredding an edge. alpha += 20 x luma: white opens a slat 0.27 turn
  (just past edge-on), so the picture shows as slat width (a venetian-blind halftone).
  - Across: the smeared row just above each slat is kept in a line buffer (every second pixel) and read 56 px
    ahead to cancel the filter's lag.
  - Down: one sample per slat per line, filtered 1/16 along y in a 256-entry RAM.
- Black input: the offset is zero and the wave alone runs.
- Source = Osc: a two-axis triangle lattice (640 x 540 virtual px) drifting diagonally replaces the luma at
  twice the camera's gain (full scale = 0.54 turn) and supplies its own colours for Face = Video.
- Face = Video: the front face shows the picture in place (block-RAM delay ring), times the shade.

## Engine

Chassis sections in louver.vhd, marked [A] .. [F]:

- [A] Timing ID -> virtual-1080 steps; vblank zone (no avid for 1024 clocks); source width; glides on Scale and
  Twist; a per-field sequencer with one 24-step serial multiplier (10 products, 32 clocks apart), a serial
  shifter for the 2^n scalings, the palette read, and three adder-only loops: the blend table (153 entries),
  the across-axis origin, and the along-slat start value (step accumulated over the pixels / lines to the
  centre).
- [B] Generated avid (parity-locked start 3 / 4 clocks after the source's, field-latched width).
- [C] Source field: Osc lattice, smear, the two per-element holds, the video delay ring.
- [D] Wave engine: every quantity is "start value per line + step per pixel" (28-bit accumulators); the same
  accumulators serve Across and Down because only the steps change. theta (14 bits) -> quarter-wave sine ROM
  -> Depth x wave (gated-sum multiply) -> alpha.
- [E] The look: |cos|, |sin| ROM -> H -> signed distances -> coverage in eighths over a ramp of (pixel size +
  |dH| per pixel along the slat) rounded up to a power of two (1 .. 16 px), so steep pinches stay
  anti-aliased; a face narrower than 2 .. 4 px fades into its neighbour (no 1-px hard lines). Colour:
  out = T(coverage, face weight) + face colour x shade x face weight, where T = ground + (accent - ground) x
  coverage/8 - accent x weight/16 is a 512-word table refilled every field. Luma is 10 bits, chroma 8 bits
  until the output filter.
- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums with the per-line resync, luma [1 2 1]/4
  delayed to the chroma's group delay, luma 13/16 toward black, blanking gate.

One latency in every mode: C_LAT = 36 = processing_delay_clks; 29 pixel stages + 4 output stages + 4 px of
filter delay. Per field: all knob-derived constants, the blend table, the phase step. Per line: the across
offset / slat number / along-slat start values (wrap candidates pre-registered). Per pixel: everything else.
Every multiply is a sum of gated operands, one adder per two operand bits per stage; block-RAM data gets a
fabric register before arithmetic; no RAM ever reads the address it is writing (no_rw_check on all of them).

## Status

- Built with router2 through `./build_set.sh kinetic louver` (rebuild command), 2026-11-14.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 79.18 MHz | 7021 |
| SD Analog | 1 | 82.26 MHz | 7016 |
| HD HDMI | 1 | 86.45 MHz | 7017 |
| SD HDMI | 1 | 76.27 MHz | 7011 |
| HD Dual | 1 | 77.77 MHz | 6996 |
| SD Dual | 1 | 79.43 MHz | 7006 |

- 7,021 LC (91.4 %), 24 of 32 block RAMs. Never flashed. Every config closed on seed 1 (the build before the
  last source-smear change needed seed 2 on HD Analog, so the seed spread is wide: 74 .. 86 MHz).
- GHDL render = integer model: 0 mismatching pixels with a black input in Across / Down, Linear / Radial,
  Smooth / Snap, several knob settings, 720p, NTSC and 1080i. With Osc + Face = Video in Across the only
  difference is row 0 (see below).

## Cut / deviations

- The slat is a slab with an accent-coloured thin face instead of the spec's flat sheet whose silhouette
  pinches to a point: the pinch is a 2.5-px-or-thicker gold waist. This is what makes the ribbon read as
  solid, and it keeps every horizontal feature at least 2 virtual rows tall.
- The thin face is the plain accent colour (its own shading was cut for logic), and the glint mixes the face
  toward the accent in sixteenths.
- Chroma is carried at 8 bits through the blend (10 bits again after the 9-tap output filter); the face's
  chroma is shaded in 5 bits, its luma in 7.
- Depth is quantised to 1/128 turn, the wave ROM to 10 bits, |cos| to 9 bits over 512 steps per quarter turn.
- Only Scale and Twist are glided; Blade, Depth and Spread read the knob directly.
- The source smear is 64 real pixels at every resolution, so it is twice as wide in picture terms on SD.
- Across, Source on: the top row of the frame (row 0 only) uses the previous field's last held line.
- Down, Source on: the per-slat filter runs on through the field boundary, so the top ~30 lines lean toward
  the bottom of the picture.
- The first design (three-colour blend with per-pixel multipliers, thin-face shading, 10-bit chroma, 32-bit
  accumulators) was 10,685 LC and did not fit; the blend table and the gated-sum multiplies brought it to 7,021.

## K6 camera pass (2026-11-15), v0.1.1

- **K6 = Camera**, 0 .. 100 %, default 50 % (initial 512): the gain of the source term that turns the slats.
  alpha += src x c, c = 20 g (Osc: 40 g), g = K6 / 256 (linear), alpha in 2^14 per turn, src = smeared luma
  >> 2 less 16 (0 .. 239):
  - 0 % (g 0): no source term, exactly the black-input picture (the wave alone);
  - 25 % (g 1, c = 20): the v0.1.0 gain, bit for bit: white opens a slat 0.27 turn (a venetian-blind halftone);
  - 50 % (g 2, default): white turns a slat 0.54 turn, past edge-on to its back face, so bright areas show
    the back colour; the slat outlines pick up the picture's texture as gentle lumps, still clean;
  - 100 % (g 4): white turns a slat 1.09 turns: the ribbons break into red / blue segments with pinches and
    gold waists wherever the picture's brightness crosses a quarter turn. Bold, lumpier outlines, no washout
    (the angle wraps, it never clips).
  - Osc keeps its x2 over the camera (1.09 turns of lattice at the default).
  - Face = Video is NOT scaled by K6 (the front face still carries the picture at K6 = 0 %; that switch is
    the deliberate "picture on the slats" mode, and scaling three channels would need three more multiplies).
- How: K6 is glided as a third channel of the shared glide (deadband +-4 codes, 1/8 of the gap per field);
  c = 5 K6 / 64 (0 .. 79) is formed once per field (zone clock 40, one add). Pixel path: stage 11 = two gated
  partial sums src x c(3..0), src x c(6..4) (was src x 20 by shifts), stage 12 = their sum (Osc x2, modulo a
  turn) and +-Depth x wave, stage 13 = alpha, folded into the |cos| ROM-address stage (the old stage-12 add).
  The alpha delay ring now writes at rc + 7 (alpha is a stage later), so the shade address still lands at 22.
  Latency unchanged (36).
- **K5 = Colour** (the 16 palettes, was K6); presets keep their palettes on K5.
- **Spread is fixed** at its old default 354 (0.06 cycle per slat: a gentle diagonal roll through the rows).
  Rate is the only speed control and stays; Scale, Blade, Depth all change the frame far more than Spread.
  Presets lost their Spread values (Tri-Vision 300, Ripple Blinds 420).
- Look check: `.kinetic_work/rb_louver/k6sheet.png` (GHDL, selfie, 720p60 / 2 = the top-left quarter of the
  frame, glides snapped; K6 = 0 / 256 / 512 / 1023 and Osc at 512).
- Build (router1 through the queue, 2026-11-15): post-route Fmax HD Analog 91.84 MHz (seed 1), SD Analog
  88.84 (**seed 2**), HD HDMI 95.52 (**seed 2**), SD HDMI 78.14 (seed 1), HD Dual 91.60 (seed 1), SD Dual
  90.73 (seed 1); worst HD 91.60. 7,217 / 7,680 LC (94.0 %, from 91.4 %), 24 / 32 EBR. On SD Analog and HD
  HDMI router1's seed 1 stalled (the builder's 600 s timeout) and seed 2 closed. The netlist has no carry
  self-adds (dupin: 0 CARRY I0==I1) but two LUTs with a duplicated input in the Osc face-colour path
  (v_iu / v_iv = 6 x lattice + 512, unchanged since v0.1.0, which was only ever routed with router2): the
  likely cause of the stalls. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic louver`
  (expect a seed-2 retry on two configs), or router2.


## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. Ron's rule after the K6 = 100 % renders: bend / element programs get a 25 % stronger camera at the
top of K6; picture programs keep their logic. The K6 default is now 100 % (initial 1023).

- **Classification: element.** K6 is the gain of the source term that turns the slats (the slat angle).
  S10 Face = Video, which puts the picture on the front face, is separate and unchanged.
- **Change:** the camera coefficient gets c = c + c / 4 at a new zone clock 41, after the existing
  c = 5 K6 / 64 at clock 40. It is still 7 bits (0 .. 98, was 0 .. 79), so g = 1.25 K6 / 256:
  - 20 % = v0.1.0;
  - 50 % = 2.5 x;
  - **100 % = 4.9 x** (white turns a slat about 1.35 turns, was 1.09).

  The pixel path is unchanged: the partial sums already hold src x 127, and alpha wraps mod a turn as
  before. Osc keeps its x 2 on top.
- **Look check:** `.kinetic_work/rb_louver/k6b/k6b_cmp.png` (GHDL, 720p60 / 4, bear.png, glides snapped).
  The same ribbons with more pinches and segment changes, and clean outlines.
- **Build** (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic louver`): router1. Post-route Fmax HD Analog
  83.91 (seed 1), HD HDMI 83.96 (seed 1), **HD Dual 85.70 on seed 2** (worst HD 83.91); **SD Analog 96.30 on
  seed 2**, SD HDMI 85.30, SD Dual 90.16 (seed 1). On HD Dual and SD Analog, router1's seed 1 stalled (the
  600 s timeout); the duplicated-input LUTs in the Osc face path are still the likely cause. 7,200 .. 7,225 /
  7,680 LC (94.1 %), 24 / 32 EBR. vmprog: out/rev_b/ron/kinetic/louver.vmprog (v0.1.2). Not flashed.

## v0.1.3: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`N - x` as `x - N`) while the GHDL
simulator computes it correctly, so every render showed the intended picture and the hardware did not (CLAUDE.md
VHDL traps; audit in `.heirloom_work/ghdl_sub_check/`). Rewritten as `to_signed(N, w) - x` / `-x + N`:

- **line 1147** (major): 8 - thin-face coverage, in eighths: nce = clamp(4 - e, 0, 8), e = signed distance to the
  thin-face boundary (> 0 inside the thin face). Big-face coverage = min(slab coverage, nce): ribbons in palette A/B
  with a gold … On hardware it was: nce = clamp(e - 4, 0, 8), inverted. With ct = 8 (thin face >= 1 px, the usual
  case) the big face is drawn where the thin face should be and bc = 0 over the big-face region, so the accent
  colour fills it. The default red/blue ribbons with gold edges render as gold ribbons with red/blue edge strips,
  and narrow faces …

**Build** v0.1.3, router2: HD Analog 85.00, SD Analog 77.64, HD HDMI 81.24 (s2), SD HDMI 85.09, HD Dual 82.45, SD Dual
85.36. 7204-7231 / 7680 LC, 24 / 32 EBR; seeds other than 1: HD HDMI. iceunpack OK. Not simulated, not flashed.
**HD skew**: HD HDMI builder seed kept router2 s2 (+1.17 / <= -1.10, 81.24 MHz) | hd_analog n/a -> router1 s7 (-1.49 /
<= -1.64, 87.50 MHz) | hd_dual n/a -> router1 s10 (-1.91 / <= -1.90, 85.01 MHz) | spare HD HDMI router2 s27 (+2.11 /
<= -1.01, 79.46 MHz):
/home/ron/videomancer-dev/.heirloom_work/rb_louver/skew/louver_v013_spare_hd_hdmi_router2_s27.vmprog. A full builder
rebuild re-routes seed 1: re-install the seeds with `.heirloom_work/skew_tools/install.sh` afterwards.
