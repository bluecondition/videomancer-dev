# Patchbay: the brief (started 2026-11-14)

Heirloom turned photos and clips of Ron's old modular video synthesizer into programs. **Patchbay is the
patches that were never built**: each program is a patch that *could* have been made on that kind of rack
(ramps, oscillators with sync and FM, wavefolders, multipliers, sample-and-holds, comparators and keyers,
logic, sequencers, colourizers, a camera input), designed from the signal flow outward and given the same
analog character and the same playable controls as a phase-2 Heirloom program. There is no photo to match;
the program's own "photo" is designed first, as a prototype frame, and then treated exactly like one.

Flash from `out/rev_b/ron/patchbay/<name>.vmprog`; rebuild one with `./build_set.sh patchbay <name>`.
The index of patches is in [README.md](README.md). The set is open-ended: new patches get a row there.

## What every Patchbay program is

- **A patch, written down.** SUMMARY.md opens with the signal flow as a patch sheet: which module feeds
  which (for example `H ramp -> wavefolder -> VCO 2 FM in`, `VCO 1 x VCO 2 -> window comparator ->
  colourizer`). Every visible element of the picture traces back to a line of that sheet, and every control
  is a knob or switch on one of those modules, named after it.
- **Analog character**, calibrated against the real rack. Before designing, view three Heirloom comparisons
  (`.heirloom_work/rb_lozenge/cmp_final.png`, `rb_kiln/cmp_final.png`, `rb_velvet/cmp_final.png`; the top
  panel of each is a real photo) and two or three photos in `programs/ron/heirloom/photos/`. Note what a
  real patch looks like: saturated colourizer bands with thin fringe colours between them, slightly soft
  edges, flat black where a key closes, small imperfections (a band that is not quite even, a faint ripple).
  A Patchbay frame should pass for a capture from the same rack.
- **Playable to the phase-2 standard**: all 12 controls used; each one named after a patch element and
  changing that element; broad ranges; every control visibly alive at any setting of the others; switches
  are unmistakable look-flips at the default state of everything else; P12 is the headline with a designed
  0 % and a designed 100 % climax.
- **Patched motion.** At the defaults the picture moves the way the patch would: at least two parameters
  are driven by internal modulators (LFOs, drifting oscillator phases, a clock), at different rates so the
  motion does not simply loop. A Speed-type control, bipolar with a deadband where direction matters. No
  switch freezes all motion.
- **Video on S11**: `Source = Video / Osc`, default Video. The camera is patched in the way an analog
  artist would patch it (luma into a field, an FM input, a key, a sample-and-hold, a tint). With a black
  input the patch still gives its full picture. Osc substitutes a generator at the same point, fed boldly
  enough that the flip is unmistakable. `program_type = "processing"`.

- **K6 is the camera strength** (Ron, 2026-11-15, the same rule as every Heirloom and Kinetic program): 0 % = no camera
  (or Osc) effect, 25 % = the strength the patch was designed with, **50 % = the default** (about twice that, or the strongest
  clean level), 100 % = a bold climax that does not wash out. A patch whose own colour or mode control would have lived on K6
  puts it on its weakest other control.

## The bar

1. **Rack photo**: the default frame would pass for a capture from Ron's rack, and is striking enough that
   he would have photographed it. Not "basic shapes put together".
2. **Patch logic**: every element traces to the patch sheet, and turning a control does what turning that
   module's knob would do.
3. **Range and climax**: the control sheet shows three clearly different pictures per control, none washed
   out or degenerate; P12 at 0 % and 100 % are both designed.
4. **Motion**: an 8-frame strip shows the patch moving coherently at the defaults.
5. **Model = RTL**: the GHDL render matches the prototype in structure, palette and texture.

## Method

The Heirloom method ([../heirloom/REBUILD.md](../heirloom/REBUILD.md)) from step 3 on, with the designed
prototype frame standing in for the photo: float prototype iterated by eye, controls designed on it,
integer model, RTL on the phase-2 Heirloom shell (`../heirloom/lozenge/`, `kiln/` or `fern/`, kept as it is:
generated avid, one latency, blanking gate, resynced output conditioning, virtual-1080 coordinates,
Source oscillator), GHDL render against the prototype, build.

Tools (all take the set name first): `.kinetic_work/tools/check.sh patchbay <name>`,
`.kinetic_work/tools/sim.sh patchbay <name> OUT.png ...`, `.kinetic_work/tools/build_queue.sh patchbay <name>`.
Python with numpy and PIL: `.heirloom_work/tools/py`. Work dir `.patchbay_work/rb_<name>/`. Never /tmp.

Budget: at most about 7,100 LC (92 %) and 24 EBR, HD timing closed at the full 74.25 MHz, never touch
`hd_clock_divisor`. Every rule in the repo-root `CLAUDE.md` applies.
