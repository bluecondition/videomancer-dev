# Wallpaper (Kinetic)

**v0.1.0 built 2026-11-14, not flashed.** Flash `out/rev_b/ron/kinetic/wallpaper.vmprog`.

## What it draws

The wallpaper groups: one small asymmetric motif repeated under every combination of translations, turns,
mirrors and glides that a square lattice allows (the textbook diagrams of the 17 plane groups, of which a
square / rectangular lattice carries these 12). The motif is a white bar, a grey wedge and a red disc (Ink palette). The default frame is
p4: about 8 x 5 motifs, four to a cell, each cell a pinwheel; a wave of rotation crosses the screen
diagonally, so every pinwheel winds and unwinds in turn (about 7 s per cycle at the default Rate).

The twelve groups, in rising symmetry on P12: p1, p2, pm, pg, cm, pmm, pmg, pgg, cmm, p4, p4g, p4m. In p4g
and p4m the motif is also folded across a mirror through its own centre, so it becomes a kaleidoscope figure
that changes shape as it turns.

## Element and state

Element: a lattice cell (2 x 2 quadrants; one motif copy sits at the centre of every quadrant, so all twelve
groups have the same density). State: the motif's rotation inside its quadrant, the same for the four copies
of a cell; the group's flips and turns carry it to the partners, so mirror partners turn the opposite way.

    psi_cell   = Depth * wave(phase + Spread * index)       index = i + j (Linear) or |i| + |j| (Radial)
    (x, y)     = rot(-psi_cell) * fold_group(a, b) * g      (a, b) = position inside the quadrant
    g          = 1 / scale of this quadrant's motif, from the source (the second modulator)
    d          = min(bar, wedge, disc distances at (x, y)) + erosion;   coverage = clamp(1/2 - d / pixel)

The second modulator (source) drives a second property, the motif's SCALE, per quadrant.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | cell pitch 144 .. 576 virtual px (motif pitch half of that), exponential, glided; default 455 |
| K2 | Shape | the motif, interpolated through five figures: Pin, Flag (default), Sail, Dot, Slab (the bar, wedge and disc move and resize) |
| K3 | Depth | swing of the rotation wave, square law, 0 .. half a turn each way (default about +-40 degrees) |
| K4 | Rate | wave travel, bipolar, centre = still; 60 s per cycle just off the deadband to 2 cycles per second |
| K5 | Spread | phase step per cell, square law, 0 .. 1/2 cycle (default about a tenth: one wave across the screen) |
| K6 | Colour | the 16 shared Kinetic palettes (ground; bar = A, wedge = B, disc and mesh = accent), with hysteresis |
| S7 | Wave | Smooth (sine) / Snap (dwell at the two extremes, quick eased flips) |
| S8 | Field | Linear (the wave crosses on the diagonal) / Radial (rings from the screen centre) |
| S9 | Spin | Off / On: the whole lattice turns (one turn in 68 s) and breathes (+-25 %) |
| S10 | Mesh | Off / On: the quadrant boundaries, and the fold diagonals of p4g / p4m, as 2.5-pixel accent lines |
| S11 | Source | Video / Osc |
| P12 | SYMMETRY | the 12 groups, p1 at 0 % to p4m at 100 % (default p4), with hysteresis. A change is handed over: the motif erodes away over 8 fields, the group changes, the motif grows back |

## Video

`program_type = "processing"`. Luma -> 32-pixel box -> a 4-bit level, sampled when the beam crosses a
quadrant's centre and held for that quadrant in a block RAM (two banks by field: the picture reads the level
sampled on the field before), so every motif is rigid. The level sets the motif's scale: 0.67 of the quadrant
at black (so the wave alone runs with nothing plugged in) to the full quadrant at white: the camera appears
as a halftone of motif sizes. Source = Osc substitutes the shell's drifting triangle lattice at the same point
with a wider range (0.52 .. 1.0), so big diamonds of swelling motifs drift across.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): knob glides (K1, K2, K3, K5); one serial multiplier
  in 16 slots (pitch through a 2^(i/256) ROM, the spin's cos / sin / breathing, the lattice coordinate of line
  0 and its steps per pixel and per line, spread, depth, rate); the AA code (half-octaves of pixels per unit);
  the group and hand-over step; then a second sequencer on the same multiplier: the ten figure parameters
  interpolated between two neighbouring figures, R^2, and the 64-entry wave table (wave -> swing -> cos, sin,
  20 clocks an entry); the palette and a pre-blended colour table (3 colours x 17 coverage steps).
  About 1,900 zone clocks.
- **Per line**: the lattice coordinate's start value steps once (one add each for U and V).
- **Per pixel** (stages 2 .. 28): two beam DDAs (U, V; Q6.26 cells) -> quadrant-local (a, b) with the group's
  flips / swap (bitwise) -> diagonal fold for p4g / p4m -> wave-table and hold-RAM reads -> (a, b) * g (two
  10 x 5 multipliers) -> rotation (four 11 x 8 multipliers) -> bar / wedge / disc signed distances (the disc's
  squares from a ROM) -> nearest primitive -> erosion -> distance to 1/16 pixel by a half-octave shift ->
  coverage 0 .. 16, mesh coverage on top -> pre-blended table -> output conditioning (chroma
  [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums, luma [1 2 1]/4 at the same group delay, luma 13/16 toward
  black) -> blanking gate.
- Generated avid (the shell's parity-locked start), one latency (35) in every mode, virtual 1080-line
  coordinates from the timing ID (SD shows the centre 4:3). Colours in the hardware convention (U register =
  Cr). Chroma is carried at 8 bits from the blend table to the output filter (512 stays exactly neutral).

## Status

- v0.1.0 built 2026-11-14 through the shared queue, router2 (the builder default). Not flashed.
- Post-route Fmax: HD Analog 81.45 MHz (seed 1), HD HDMI 82.24 (seed 1), **HD Dual 77.39 (seed 2**; seed 1
  missed), SD Analog 77.55 (seed 1), SD HDMI 70.08 (seed 1), SD Dual 77.68 (seed 1). HD needs 74.25, SD 27.
- **7,371 .. 7,383 / 7,680 LC (96 %)**, 20 / 32 EBR. This is over the set's 7,100 LC budget: the first RTL was
  8,199 LC (106 %, did not fit) and the restructuring listed below brought it to 96 %; it fits and closes
  timing, but there is little room left for additions. No `emulate_transparency` (standalone yosys log).
- Rebuild: `./build_set.sh kinetic wallpaper`.
- Model = RTL (GHDL): pixel diff **0 of 918,720** on the full 720p60 frame (decimation 1) at the defaults
  (p4, Ink, black input, third field), and 0 of 228,960 on the same settings at decimation 2 (the top-left
  quarter). The first RTL (before the restructuring) was also 0 of 228,960 in pmg with the Moss palette.
  The generated avid starts 4 clocks after the delayed source avid, so the GHDL frame is the model's shifted
  4 columns right with 4 blank columns at the left (as in the pilots).
- More GHDL renders of the built RTL, all 720p60 at decimation 2 (640 x 360, the top-left quarter):
  Source = Osc at the defaults: **0 of 228,960**. Video = selfie.jpg at the defaults: 2,192 pixels differ
  (1.0 %), all inside two or three motifs whose held level is one step off. A sixth-field render with p4m +
  Mesh + Spin + Radial + Snap + Osc + Vega: 2,308 pixels differ (1.0 %), again whole motifs one level off;
  moving the model's hold sample by two source pixels changes that count between 457 and 4,700, so the
  residue is the phase of the source sample (and, for the photo, the tester's own image scaling), not the
  geometry: the fold, the mesh, the spin, the radial index and Snap all match. That render is at the sixth
  field, so the wave phase and the spin angle step as the model's do.
- NOT rendered in RTL: the groups other than p4, p4m (and pmg on the first RTL), the hand-over, Shape away
  from the default, SD and 1080i. The integer model covers them (sheet.png, groups.png); an NTSC model frame
  shows the same picture as the centre 4:3 of the 720p one.
- Renders: `.kinetic_work/rb_wallpaper/cmp_final.png`, `sheet.png` (control sheet, integer model),
  `strip.png` (motion, integer model), `groups.png` (the 12 groups at Depth 0 with Mesh, integer model).
- What went to block RAM to make it fit: the per-line multiplier engine became a per-line DDA; the shift-add
  colour blend became a per-field pre-blended table (2 EBR); the two serial multipliers became one; chroma is
  carried at 8 bits and its 8-pixel delay, and the mesh-coverage delay, are 16-deep block-RAM rings.

## Cut / deviations

- The element is the cell, not the single motif: with one wave phase per motif the symmetry inside a cell was
  lost in the first renders. The four copies of a cell now turn together.
- The motif's scale is driven by the source only (per quadrant); the wave drives the rotation only.
- The hand-over shrinks the motif by eroding its outline (a distance offset), not by scaling it.
- p4g and p4m fold the motif across a mirror through its own centre (a kaleidoscope figure) instead of
  placing eight smaller whole copies per cell. In p4g at Depth 0 the disc falls on the folded-away side and
  only appears as the motif turns.
- Mesh draws every quadrant boundary (each quadrant is one copy's domain) plus the two fold diagonals; it does
  not distinguish mirror lines from glide lines.
- Disc edge: distance = (r^2 - R^2) / 2R with 2R rounded to 181 / 256 / 362, so the disc's AA ramp is up to
  20 % wider or narrower than one pixel. All AA ramps are quantised to half-octaves (1 .. 1.5 pixels wide).
- Spin's rate is fixed (68 s a turn); it restarts from the same angle at power-up.
- Shape interpolates linearly between figures; each primitive keeps to its own sector so they never touch.
- Hold RAM is indexed by 5 + 5 bits of the quadrant number: with Spin on at the smallest Scale the far
  corners can share a level with the opposite corner (a wrong motif size there, nothing else).
- The source level lags the picture by about 12 pixels to the right (the box filter).
