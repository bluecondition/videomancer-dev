# Euclid — a Euclidean step sequencer drawn as a piano roll, through a colouriser

**v0.1.0 built 2026-11-14, not flashed.** Patchbay. Up to 12 rhythm tracks are rows, steps are columns and hits
are pads. A playhead sweeps the grid; every pad it strikes flashes and decays by an envelope that depends only on
how many steps ago it was struck. Presets are hand-written placeholders.

Working files: `.patchbay_work/rb_euclid/` (`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`, `sheet.png`,
`strip.png`).

## The patch

```
clock (Tempo) ──> step counter per track (loop n; Poly: n, n-1, n-2, n-3)
step counter  ──> Euclidean pattern generator (k hits over n steps, rotated per track) ──> gate
gate          ──> envelope generator (a function of the steps since the strike; Tails) ──> VCA level
H ramp x Steps  ──> step phase ──> pad window (Gate) ──> triangle shaper ─┐
V ramp x Tracks ──> row phase  ──────────────────────> triangle shaper ─┴─> sum, clipped = the pad shape
pad shape x (rest level + envelope + camera S&H gated by the envelope) ──> slew limiter along the scan
slew limiter + accent bus (the row glows after a strike) + playhead pulse ──> colouriser
colouriser hue <── track number x Colour
breath LFO ──> rest level of each track (phase-shifted per track);  hum bar ──> level
camera ──> 8-px box ──> sample-and-hold per pad (clocked by the step grid, held down the row) ──> VCA
(Osc: a drifting triangle lattice in the camera's place, not held, at about twice the camera's level)
```

## What it draws

- **The grid.** Tracks are rows that fill the screen height; steps are columns that fill the width. A hit is a
  pad; an empty step is a faint ghost dot in the track's darkest colour.
- **Pads through a colouriser.** A pad is a horizontal triangle plus a vertical triangle, clipped: a chamfered
  lozenge whose level rises from its edge to a plateau. The level goes through a 64-level colouriser of flat
  bands with thin fringes: black, deep, body, a dark ring, bright, a hot fringe, a hot tint, white, then an upper
  register (a second hue, a dark line, a third hue, white). A resting pad shows the deep and body colours, and
  sometimes a dark eye. A struck pad shows every band as nested contours with a hot core, and cools band by band.
- **Streaks.** A slew limiter along the scan line leaves a tapering streak to the right of each pad. The hotter
  the pad, the longer the streak, so struck pads become comets.
- **The playhead.** A soft white vertical line at each track's clock position.
- **Motion at the defaults.** The playhead crosses the grid at about 3 steps a second, lighting pads that then
  decay over several steps. The breath LFO opens and closes the dark eyes of the resting pads, track after
  track, on a 5 s cycle. A hum bar drifts through on a 7 s cycle and shifts the band edges by a line or two.
- **Rhythm.** Each track has its own hit count, so the rows read as different Euclidean patterns (4, 5, 7, 9,
  10, 11, 13 of 16 at the defaults), staggered by Rotate.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Steps | H ramp / step counters | steps per loop, 4 .. 32 (16 at the default): four huge pads per row .. 32 slivers |
| K2 Gate | pad window | pad width within its step, 30 % .. 100 %: narrow, dimmer pads (a short gate does not reach full level) .. pads that touch |
| K3 Tracks | V ramp | 2 .. 12 rows (8 at the default) |
| K4 Tempo | clock | bipolar, square law, ±8 deadband: the playhead runs left or right, up to about 11 steps a second. Centre = stopped clock; the breath LFO and hum bar keep moving |
| K5 Rotate | pattern generators | pattern offset per track, 0 .. 4 steps per track: the rows slide against each other and the hits line up into diagonals |
| K6 Colour | colouriser | hue step from one track to the next, 0 .. 2.75 of 16 hues: all tracks hot pink (0) .. pink to orange (default) .. twice round the wheel (max) |
| S7 Layout | H ramp | Sweep: the playhead crosses a fixed grid / Roll: the grid, at twice the size, scrolls under a playhead fixed at the centre |
| S8 Tails | envelope generator, slew limiter | Long: a slow decay (6 steps) and long streaks / Short: a harder strike that reaches the white and second-hue bands, gone in about 3 steps, with stub streaks |
| S9 Accent | accent bus | Off / On: every strike lights its whole row, which then decays on the long envelope |
| S10 Metre | step counters | Same: every track n steps / Poly: n, n-1, n-2, n-3, n, ... Each row is one loop, so the columns no longer line up and the playheads drift apart |
| S11 Source | camera input | Video / Osc |
| P12 FILL | pattern generators | hit count per track: 0 % = four-on-the-floor on every track (diagonals under Rotate); the tracks then fill in one after another; 100 % = every step a hit, the playhead dragging a cascade of flashes |

## Video

- **Video (the default).** Camera luma goes through an 8-px box and a sample-and-hold per pad. It is sampled
  on the first line of each row, in the second quarter of each step, and held down the row. The held value is
  added to the pad's level, gated by the envelope: a pad that was just struck opens onto the camera, and the
  brighter the picture is there, the further up the colouriser the pad goes. Unlit pads stay dark. The camera
  is therefore a mosaic at the grid's resolution (up to 32 x 12), played by the rhythm, and every contour stays
  clean. With a black input the patch gives its full picture.
- **Osc.** A 640 x 540 virtual-px triangle lattice, drifting, takes the camera's place at about twice its level.
  It is not held, so the lit pads fill with moving diagonal bands in the upper-register colours.

## Engine

- **Per field (vblank zone).**
  - A shared glide for K1, K2, K3, K5, K6 and P12 (1/8 of the gap per field).
  - One serial shift-add multiplier: six global slots (steps, tracks, gate, tempo squared, hue step, row step).
  - A track sequencer, started at zone clock 200 every field, that loops over the 12 tracks. Per track it works
    out the loop length, the hit count (loop x density), the clock position (read-modify-write of the track
    RAM, wrapped at the loop), the phase step (loop x 2^25 / width), the Roll offset, the hit mask (an
    accumulator: add k, subtract n on overflow, hit when the value is below k; run ahead by the rotation first),
    the steps since the track's last strike, the accent level (envelope ROM), the hue and the rest level.
    It writes seven 16-bit words per track into a 128 x 16 track RAM. About 150 clocks a track.
- **Per line.** An engine kicked at each line start computes the NEXT line: the row number from a row-phase
  accumulator, the vertical triangle, the hum bar, the row's record (seven RAM reads) and accent x triangle.
- **Per pixel (15 stages, then the output conditioning; latency 20 in every mode).**
  1. Step phase accumulator (25 fraction bits) and a step counter that wraps at the loop length.
  2. Hit bit; step distance to the clock.
  3. Wrap; pad window; source level from the line buffer.
  4. Age; past-the-playhead flag; horizontal triangle.
  5. Envelope ROM address; pad shape; playhead one-shot.
  6. to 7. Envelope ROM and its fabric copy; held or tracking source level.
  8. Window gain x source level (5 x 7 bits).
  9. Level.
  10. Level x shape (7 x 6 bits).
  11. Slew limiter.
  12. Accent, playhead, colouriser address.
  13. to 15. Colouriser ROM, fabric copy, compose.
- **Memories (18 EBR).** Colouriser 1024 x 32 (16 hues x 64 levels), envelope ROM 1024 x 16 (envelope, window
  gain and accent level by tails, age and sixteenths of a step), line buffer 2048 x 8, track RAM, hold RAM.
- **Contracts.** Generated avid; one latency; blanking gate; output conditioning (Y, U and V through two
  [1 2 2 2 2 2 2 2 1]/16 boxes with a per-line resync, luma 13/16); the camera's 8-px running sum resyncs every
  line; colours in the hardware convention (U = Cr). Rows fill the 1080 virtual lines from the timing ID.
  Steps are a count across the width, so SD shows the same grid with pads narrower in proportion.

## Status

- **v0.1.0 built 2026-11-14 with router2 (the build default), all six configs on seed 1. Not flashed.**
  - Post-route Fmax: HD Analog 87.98, HD HDMI 85.65, HD Dual 91.37, SD Analog 91.73, SD HDMI 81.14,
    SD Dual 89.82 MHz (HD needs 74.25).
  - 5,533 .. 5,560 / 7,680 LC (72 %), 18 / 32 EBR. All six bitstreams pass iceunpack; the packed config carries
    the labels and the six presets.
  - Output: `out/rev_b/ron/patchbay/euclid.vmprog`.
  - An earlier build of the same design on an overloaded machine stalled router2 once on SD Dual seed 1 and
    passed on seed 2 (89.74 MHz). If that happens again, rerun or use `SEED=2`.

- **Checks (GHDL, 720p60, decimation 1, against the integer model `imodel.py`).**
  - Default frame, Video with a black input: 100 % of pixels bit-exact.
  - Motion: a frame four fields later is also bit-exact.
  - All four pattern / layout switches on (Roll, Short, Accent, Poly): bit-exact.
  - Steps 32, Tracks 12, Gate, Rotate, Colour and Fill off their defaults, Poly (glide bypassed in a sim-only
    copy): bit-exact.
  - Video with `selfie.jpg`: bit-exact when the model is fed the tester's own stimulus luma (`cmp_vid3.py`).
    This check found a real fault in the first draft: the camera's 8-px running sum dropped the first sample of
    each line and stayed offset for the rest of it. Fixed before the final build.
  - Osc: 99.99 % of pixels bit-exact.
  - SD (ntsc, 2 fields) and 1080i (2 fields): every field bit-exact with the model, same picture.
  - The compared area leaves out a 16-px border at the sides and 4 lines top and bottom. The picture sits 4 px
    right of the source timing (the shell's generated avid).
- `thincheck.py`: no single-row band line of 24 px or more at the defaults, 720p or 1080p (11 cases; two
  25 .. 60 px runs in two corner cases at 1080p).
- Rebuild: `./build_set.sh patchbay euclid`.

## Cut / deviations

- **Camera windows are a sample-and-hold mosaic, not a see-through picture.** The spec asked for lit pads as
  windows onto the camera. Per-pixel camera inside the pads shredded the contours and streaks (`p3_cmp.png`,
  `p4_cmp.png`), so the camera is held per pad. You see it as pad heat at the grid's resolution, not as an image
  inside each pad.
- **Roll also doubles the step size**, so that a still frame flips unmistakably; the spec only asked for the
  scroll.
- **Tails: Long is the first label** (the default with the switch down). Short also strikes harder.
- **Poly is n, n-1, n-2, n-3 repeating** (never below 2), and each row is stretched to the full width.
- **FILL fills the tracks one after another** rather than all together, so the middle of the slider gives a mix
  of Euclidean patterns.
- **Accent decays on the long envelope whatever Tails is**, at a modest level, so FILL 100 % with Accent gives a
  gentle ripple instead of a field-rate strobe.
- **Rotate is the subtlest knob in a still.** It slides the rows; with mixed hit counts the diagonals only read
  clearly at low FILL.
- **Steps, Tracks and Rotate glide**, so a fast turn passes through the counts in between.
- **SD** shows the same grid over the full width (pads narrower in proportion) and is softer, because the
  output blur is per pixel.
- The first field after loading draws ghosts only (the track RAM is empty until the first vblank).

**Watch on hardware:** nothing here has been flashed. The track RAM and hold RAM carry `no_rw_check`. No
pin-skew seed screening was run.
