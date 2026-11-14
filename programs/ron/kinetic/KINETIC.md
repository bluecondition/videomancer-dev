# Kinetic: the brief (started 2026-11-14)

A run of 20 generative programs for Ron's Videomancer: **clean geometry set in motion by a modulation
engine**. Op-art, kinetic sculpture, tilings, interference, illusions. Each program is one geometric idea
drawn crisply and anti-aliased in a curated palette, and what makes it a *Kinetic* program is that every
element of the geometry (each band, slat, cell, ring, source) carries a **state** that a travelling wave
drives. The same wave engine and the same six knob functions are on all 20, so the set is learnable as a set.

Flash from `out/rev_b/ron/kinetic/<name>.vmprog`; rebuild one with `./build_set.sh kinetic <name>`.
The slate and the status of each program are in [README.md](README.md).

## What every Kinetic program has

### 1. Elements with a state

Decide what the *elements* are (bands, slats, grid cells, rings, wave sources, layers) and what one element's
**state** is (an angle, a size, a displacement, a shape morph, a colour step). The picture is a pure function
of position, the element states and the knobs. Never a coarse overlay on something else, never a filter.

### 2. The wave engine (identical behaviour on all 20)

    theta_e = phase + Spread * index_e          (mod one cycle)
    state_e = Depth * wave(theta_e) + source_e

- **phase** advances once per field by Rate. Rate is bipolar with a deadband at the centre: centre = still,
  either side = direction; exponential, about 60 s per cycle just outside the deadband to about 2 cycles per
  second at either end.
- **index_e** is the element's position in the **Field**:
  - *Linear*: the index runs along the travel axis (for a 2-D grid, `i + j` or a chosen diagonal), so the
    wave sweeps across the picture.
  - *Radial*: the index is the element's distance from the screen centre in elements (octagonal metric
    `max + min/2`, or `|i| + |j|`), so the wave radiates as rings.
- **Spread** is the phase step per element, 0 to about half a cycle per element on a curve that gives fine
  control at the low end. At 0 everything moves in unison; a little gives one slow wave across the screen;
  a lot gives many cycles and, near the top, aliases into new standing figures (the "A Million Times"
  effect). Spread must never show a seam where an index wraps or where the Field changes sign.
- **wave** has two shapes on a switch: *Smooth* (sine) and *Snap* (dwell at the extremes with quick eased
  flips, like a mechanical flip; about 75 % of the cycle is dwell). Programs where a zigzag or step reads
  better than Snap may substitute it and say so.
- **Depth** sets how far the wave moves the state, on a square law (fine control low, bold at the top).
- **source_e** is the second modulator: the incoming video's luma at the element (box-filtered at least
  8 px, held per element where the element must stay rigid), or with Source = Osc a slow internal field
  (two-axis triangle lattice drifting diagonally, as in the heirloom shell, fed at 2-4x the camera level so
  the flip is bold). It adds to the same state, or drives a second property of the element (weight, size,
  colour step); say which in the SUMMARY. With a black input in Video mode the offset is zero and the wave
  alone runs, so the program is never dead with nothing plugged in.

### 3. The control layout

The six standard functions exist on every program, on these knobs unless stated:

| Control | Function |
|---|---|
| K1 Scale | element size / pitch / density, exponential, glided |
| K2 Shape | the program's own static shape morph |
| K3 Depth | wave depth |
| K4 Rate | wave speed, bipolar, centre = still |
| K5 Spread | phase step per element |
| K6 Colour | palette, 16 steps with hysteresis (the shared table below) |
| S7 Wave | Smooth / Snap |
| S8 Field | Linear / Radial |
| S9, S10 | the program's own look-flips |
| S11 Source | Video / Osc (default Video) |
| P12 | the program's headline, named in the slate |

When a program's headline IS one of the standard functions (Depth on riley, Spread on horologe), that
function moves to P12 and its knob gets a program-specific control instead. Where a geometry makes a
standard function meaningless, substitute the closest equivalent and say so. Every control must visibly
change the picture at any setting of the others, every switch must be an unmistakable look-flip at the
default state of everything else, and P12 at 100 % is a designed climax scene.

### 4. Clean rendering

- **Anti-aliased edges.** Every drawn edge comes from a signed distance in sub-pixel units and blends the
  two colours through at least 4 coverage steps (shift-add blends, no multipliers). No stair-steps, no
  single-pixel sparkle, no seams at cell or wrap boundaries. Diagonal and curved edges need it most.
- **No single-row hard lines**: any horizontal feature is at least 2 virtual rows tall or soft (a 1-px-tall
  run of 24+ px reads as a sync glitch on hardware). Minimum line weight 2 virtual pixels at every Scale.
- **Virtual 1080-line coordinates**: the same picture at SD, 720p and 1080p, round things round (SD shows
  the centre 4:3 of the 1920-wide canvas).
- **Palette**: the 16 shared four-colour palettes in `.kinetic_work/tools/kpalette.py` (ground, A, B,
  accent), stored in the hardware convention (U register = Cr). Copy the table into the program's own
  `<name>.py` build hook so the folder is self-contained. A program may shade a colour (toward black or
  toward the accent) but does not invent its own palette list.
- **Output conditioning for a sharp picture** (this differs from the heirloom shell, which blurs luma):
  luma `[1 2 1]/4`, chroma `[1 2 2 2 2 2 2 2 1]/16` (two 8-sums with the per-line resync), luma delayed to
  the chroma's group delay, then luma `y - (d>>3) - (d>>4)` with `d = y - 64`, then the blanking gate. One
  latency in every mode.

### 5. Video

`program_type = "processing"`. Source on S11, default Video. Besides entering as `source_e`, the picture
may also colour the elements on one of the program's own switches (an element face showing the camera,
shaded), which is encouraged wherever it fits. Osc substitutes at the same point so every video-path
control stays live. At the default knobs with a camera, the geometry must stay clean: the picture bends
or weights the elements, it never shreds an edge into per-line jaggies.

## The bar

A Kinetic program is accepted on five tests, judged on renders at 1280x720 or larger:

1. **Poster**: any default frame could hang on a wall as a print. Deliberate composition, generous scale,
   clean edges at 4x zoom, a palette that looks chosen.
2. **Motion**: a strip of 8 frames across one wave period shows coherent travelling motion that a viewer
   can follow, and the motion is the point of the piece, not a wobble added to it.
3. **Range**: on the control sheet every control at min / default / max gives three clearly different
   pictures, none washed out, none degenerate, none aliasing into noise.
4. **Climax**: P12 at 0 % and 100 % are both designed scenes; 100 % is the wildest frame the program makes
   and still reads as this program.
5. **Model = RTL**: the GHDL render matches the integer model (pixel diff 0, or every difference explained).

## Method

1. **Design in numpy first.** `proto.py` (float) until the look, the motion and the ranges are right; view
   every render, list what is wrong, fix, repeat. Then `imodel.py` at the RTL's widths. Sheets and strips
   come from the model.
2. **RTL from the heirloom phase-2 shell** (`programs/ron/heirloom/lozenge/` or `kiln/` or `fern/`), which
   carries the timing-ID virtual-1080 steps, the vblank zone and per-field latches, the generated avid, the
   Source oscillator, the block-RAM delay rings, the resynced output filters and the blanking gate. Replace
   its look, and change its luma conditioning as in "Clean rendering". After the pilots are flashed, later
   programs start from `programs/ron/kinetic/louver/` or `riley/` instead.
3. **Check, render, build** with the set tools (all take the set name first):
   - `.kinetic_work/tools/check.sh kinetic <name>` (runs the build hook, analyses the pkg too)
   - `.kinetic_work/tools/sim.sh kinetic <name> OUT.png [--video IMG] [run.py options]`
   - `.kinetic_work/tools/build_queue.sh kinetic <name>` (shared 5-slot queue; router1 with
     `NEXTPNR_ROUTER="--router router1"`)
   - Python with numpy and PIL: `.heirloom_work/tools/py`; colour helpers `.heirloom_work/tools/hwcolor.py`;
     labelled stacks `.heirloom_work/tools/cmp.py`. Work dir `.kinetic_work/rb_<name>/`. Never /tmp.
4. **Budget**: at most about 7,100 LC (92 %) and 24 EBR; HD timing closed at the full 74.25 MHz; never touch
   `hd_clock_divisor`. Every rule in the repo-root `CLAUDE.md` applies.

Each program folder holds `<name>.vhd`, `<name>.toml`, optional `<name>.py` + `<name>_pkg.vhd`, and
`SUMMARY.md` (what it draws and the art it comes from, the element and its state, the 12 controls, how
video enters, the engine, build status with seeds / Fmax / LC / EBR, anything cut).
