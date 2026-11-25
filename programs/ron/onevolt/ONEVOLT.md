# Onevolt: the brief (started 2026-11-25)

Heirloom rebuilt patches Ron photographed on his modular video synthesizer. Patchbay invented patches from an
idea of a picture. **Onevolt starts from the modules.** Each program is one small rack of real analog video
modules (LZX Industries, Syntonie and a few other makers), cabled into one patch, and the program behaves the
way that rack would: the knobs are the panel knobs, and **each switch unplugs one module and plugs another into
the same cables.** There is no reference image. The name is the patchable signal standard those modules share:
0 to 1 volt, DC-coupled, every input fast enough for video.

Flash from `out/rev_b/ron/onevolt/<name>.vmprog`; rebuild one with `./build_set.sh onevolt <name>`.
The index of racks is in [README.md](README.md).

The research this set is built on is in the repo, with a source URL beside every claim:
`.onevolt_work/research/lzx_modules.txt` (about 100 LZX modules, Gen3 back to Visionary, Cadet and Castle),
`.onevolt_work/research/other_makers.txt` (Syntonie's whole line from the maker's user guides, Reverse Landfill,
Foxing Hour, BSO, Visible Signals, and the historic machines: Sandin, Paik/Abe, Beck, Rutt/Etra, Spectron) and
`.onevolt_work/research/patching_practice.txt` (what analog video synthesis is, how patches are chained, 35
documented patches, what makes the result look analog, what performers ride). Read the module blocks your rack
names before you design anything. Do not rely on memory for what a module does.

## What analog video synthesis is (the short form of the research)

- A patch is a function of two voltages, the horizontal and the vertical ramp, evaluated along the scan line with
  no stored frame. Every block is an add, a multiply, a rectifier, a min or max, a comparator or a clip of 0-1 V
  signals. Only frame stores, rescan and feedback need memory.
- The order people chain things in: ramps -> shift / rotate -> mirror -> curve -> combine H with V -> fold ->
  key -> layer -> colour matrix -> encoder (which clips to 0-1 V). Modulating early (the ramps) bends or moves
  space; modulating late (a threshold, a matrix gain) changes edges and palette.
- Oscillators are locked by hard reset. A horizontal-range oscillator restarts at the start of every line, so it
  draws vertical bars anchored at the left edge with a partial last cycle at the right; a vertical-range one draws
  horizontal bars. Free vertical-range oscillators roll smoothly. Smooth horizontal motion does not come from
  oscillators (that is what a motion ramp generator like Scrolls is for).
- "What makes the patch isn't the content of the signals themselves; it's what they're patched into" (Lars
  Larsen, LZX). One cable or one module changes the look: which combiner output you take (average, minimum,
  maximum, absolute difference), hard key or soft key or outline, the order of the three colour cables, a ramp
  swapped for a camera, what feeds an oscillator's sync.

## What every Onevolt program is

- **A rack, written down.** The SUMMARY opens with the modules (maker, name, job here) and the patch sheet, cable
  by cable. Every visible element traces to a line of that sheet. Nothing in the picture comes from a module that
  is not in the rack.
- **Knobs are panel knobs.** Each of K1-K5 and P12 is a named control on a named module and does what the real
  control does, over at least its real range. They are **performance controls**: a position, a phase, a
  threshold, a frequency, a gain. No knob is an animation speed (Ron, repeatedly, on the Heirloom reviews:
  "make K4 a performance knob rather than animation").
- **Switches are module swaps.** S7-S10 each replace one module in the chain with another that takes the same
  cables (or move one cable to the next jack of the same module). Both positions are full, designed pictures; the
  flip is unmistakable at the default state of every other control; and the four switches multiply, so the 16
  combinations are 16 different looks and the knobs mean something different in each. This is how the switches
  broaden what is on the knobs.
- **S11 is the input module**: Source = Video (the camera, through a decoder) / Osc (an oscillator patched to
  the same jack, fed boldly enough that the flip is unmistakable). Default Video. `program_type = "processing"`.
- **K6 is the camera strength**, default 100 % (1023). 0 % is exactly the black-input picture. At 100 % the
  camera is bold and the subject is recognisable (test with bear.png and selfie.jpg), without shredding edges
  into per-line jaggies. The camera is patched in where an analog artist would patch it; each rack uses a
  different jack (a fold input, a rotation CV, an FM input, a sync input, a filter input, an ADC, the luma of a
  colour-space loop, a threshold CV, a scan input, a keyed layer).
- **P12 is the headline**, the control a performer would ride, with a designed 0 % and a designed 100 % climax
  that still reads as this rack.
- **Patched motion.** At the defaults the picture moves the way the rack would: two or three internal slow
  modulators (LFOs, a motion ramp generator, a free-running vertical oscillator) at unrelated, designed rates,
  patched at modest depth, added on top of the knob positions. The knobs stay direct. No switch freezes all
  motion.
- **Smooth and wide.** Continuous knobs change the picture continuously over the whole travel, with no dead
  zones and no jump at any point (Ron: "why do the controls seem so stepped / choppy"; "doesn't visually flip in
  a jarring way at 50 %"). Ranges are broad: each knob's two ends are clearly different pictures, neither washed
  out nor aliased into noise. Every control is visibly alive at any setting of the others, P12 at 0 % included.

## The analog character to build in (all of it is in the research, section 5)

1. **Continuous in X.** No pixel grid: edges fall at sub-pixel positions and are drawn with coverage, bands have
   no pixel quantisation ("surreal and silky"). The only blocks are the ones a clocked module makes on purpose.
2. **Everything happens along the line.** Filters act horizontally only and smear to the right. A vertical edge
   lights up; a horizontal one does not.
3. **Path delay.** Each module in a path delays it by 16-32 ns, about 1-2 pixels at 1080p. Colour channels that
   pass through different numbers of modules misregister by 1-3 virtual pixels to the right, a faint fringe on
   vertical edges. It is free in the pipeline: take the three colours from different delay taps.
4. **Soft clipping** at 0 and 1 V in the encoder: highlights compress, they never wrap.
5. **Hard-reset oscillators**: bars anchored at the left edge (and the top), fans that hinge there under FM.
6. **Slewed knobs, unslewed video-rate CVs.**

Do NOT emulate pixel-level noise, hum bars, torn or zippered key edges, tearing or sync loss. On this hardware
they read as faults (Ron's notes call them "glitchy"), and sharp pixel-rate swings in motion trigger the HD HDMI
bars. Keep drawn luma at or below about 768.

Calibration, before designing: view `.heirloom_work/rb_lozenge/cmp_final.png`, `rb_kiln/cmp_final.png` and
`rb_velvet/cmp_final.png` (the top panel of each is a real photo of Ron's rack) and three or four photos in
`programs/ron/heirloom/photos/`. Note what his patches look like: saturated colouriser colour, mirror symmetry
from folded ramps, stair steps, hard keys over gradients, dense layered structure across the whole screen,
flat black where a key closes.

## The bar

1. **Rack photo.** The default frame would pass for a capture from a real rack, and is striking enough to have
   been photographed: one idea with real structure covering the screen in bold, deliberate tone areas. Not
   "basic shapes put together", not thin motifs on an empty ground.
2. **Module logic.** Turning a control does what that module's control does; flipping a switch does what
   swapping that module does. Someone who owns the module would recognise it.
3. **Sixteen looks.** The 16 switch combinations are 16 different pictures, none dull.
4. **Range and climax.** Each knob at 0 / 25 / 50 / 75 / 100 % gives a smooth series with clearly different
   ends; P12 at 0 % and at 100 % are both designed.
5. **Camera.** At K6 100 % the subject is recognisable and edges stay clean; at 0 % the picture is exactly the
   black-input one; Osc is an unmistakable flip.
6. **Motion.** An 8-frame strip shows the patch moving coherently at the defaults.
7. **Model = RTL.** The GHDL render matches the integer model.

## Method and budget

The Heirloom method ([../heirloom/REBUILD.md](../heirloom/REBUILD.md)) from step 3 on, with the designed
prototype frame standing in for the photo; RTL on the phase-2 Heirloom shell (`../heirloom/lozenge/`, kept as it
is: generated avid, one latency, blanking gate, resynced output conditioning, virtual-1080 coordinates, Source
oscillator, vblank sequencer). The full agent brief is `.onevolt_work/agent_prompt.txt`; each rack's spec is
`.onevolt_work/prompts/<name>.txt`.

Tools (all take the set name first): `.kinetic_work/tools/check.sh onevolt <name>`,
`.kinetic_work/tools/sim.sh onevolt <name> OUT.png ...`, `.kinetic_work/tools/build_queue.sh onevolt <name>`.
Python with numpy and PIL: `.heirloom_work/tools/py`. Work dir `.onevolt_work/rb_<name>/`. Never /tmp.

Budget: at most about 6,750 LC (88 %) and 22 EBR, HD timing closed at the full 74.25 MHz, never touch
`hd_clock_divisor`. The budget is lower than the earlier sets' on purpose: the HD HDMI bitstream is re-placed
after the build to keep the encoder's pin skew in its safe window, and that needs free cells. A patch is a short
chain; a program over budget has too many modules in it. Every rule in the repo-root `CLAUDE.md` applies.
