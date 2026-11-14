# Louver (Kinetic #6, pilot)

v0.1.0 built 2026-11-14, all six configs, not flashed.

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
| K5 | Spread | phase step per slat 0 .. 1/2 cycle, square law |
| K6 | Colour | the 16 shared palettes with hysteresis: ground = gap, A = front, B = back, accent = glint and thin face |
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
