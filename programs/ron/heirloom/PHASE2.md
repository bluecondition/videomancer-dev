# Heirloom phase 2: from replica to playable modular patch

This is the brief for the second pass over the 55 Heirloom programs, one program per tab. It closes out phase 1,
records what Ron wants from phase 2, and gives each tab what it needs to start: how to work a program, the
hardware headroom left in each one, and the known issues to check.

## Phase 1: closed 2026-10-28

- **All 55 programs were rebuilt to v0.5.x from their source pictures** (2026-10-21 → 10-28). They used the method in
  [REBUILD.md](REBUILD.md): look → measure → float prototype iterated against the photo → controls → integer model →
  RTL → GHDL render compared with the photo → build. Every program is built for all 6 configs and committed, with
  its `.vmprog` in `out/rev_b/ron/heirloom/`. **None has been flashed yet.**
- **Each `<name>/SUMMARY.md` holds:**
  - the measured reading of the photo;
  - what the program does;
  - the control → photo-element map;
  - the engine;
  - build seeds and router;
  - the deviations from the photo.
- **Working files** are in `.heirloom_work/rb_<name>/`. They include `notes.md` (every measurement and iteration),
  `proto*.py` (the float prototype), `imodel.py` (a bit-accurate model of the RTL), `cmp_final.png` (photo / proto /
  RTL Osc / RTL Video) and `sheet.png` (control sheet).
- **Every program was reviewed against its photo before acceptance.** Four were sent back and fixed:
  - **warp:** washed-out palette. The colour fit couldn't tell the two sides of a stripe apart and averaged them to
    grey.
  - **abacus:** missing bead gloss.
  - **ovals:** hard vertical seams and flat interiors.
  - **chartreuse:** grey highlights and broken fins, then an over-darkened tone curve.
- **Weaker matches** (start here if comparing against the photos):
  - **abacus:** glints are stepped, the colour steps in visible bands where the photo grades smoothly, and the
    left-third "eyes" are off.
  - **lava:** the left drapes are streaky tongues.
  - **warp:** the fine top-right moiré is missing.
  - **greenroom and radiant:** the collage cut-outs are soft silhouettes.
- **Known issues to check on first flash:**
  - **Right-edge cut-off.** The shared shell may drop the last ~4 px of each line (gate/avid ahead of the output
    filter centre). The tangram agent found and fixed it locally; others probably share it.
  - **Whites read dimmer than the photos.** This is the HD HDMI bar-fix luma cap of ~777, by design.
  - **SD usually crops the sides** of the 16:9 picture. Each SUMMARY says what SD loses.
  - **curtain:** its sources were tidied after its build, so rebuild before release.
  - **radiant:** must be built with `SEED=3`. HD HDMI fails placement at seeds 1–2, and the builder does not retry
    placement errors.
  - **goblet:** the "Hourglass" preset also turns the stripes off (unintended).
  - **Presets are hand-written placeholders everywhere.** Capture real ones from the hardware with
    `./capture_preset.sh`.

## Ron's intention for phase 2

In Ron's words (2026-10-28):

> These are captures from my modular synthesizer. They were created with many different modules, with patch cords
> connecting different points to create the shapes, colors, movement, etc. You've broken down the image into
> different elements. You've done well at making the patch look good. Now I would like to expand each of these
> programs to be more like having an actual modular video synth. I would like each control to have a broader range
> (I will note particular ones that need it) and there may be some cases where a particular control needs a
> smaller range or different range for visual improvement. I would like as many individual shapes/subshapes as well
> as groups of the same shape to be tied to parameters as possible so they can be modulated. Always prefer that
> over simply scaling the entire X/Y, which is also ok in some circumstances. More visual range or options in the
> switches, less controls that do almost nothing. I would like the program to be a little "looser" for lack of a
> better word and more range in visuals, but still able to produce something close to the original image at the
> right or default settings.

### What that means for each program

1. **The photo stays reachable.** The defaults (and preset 0, "Photo") must still reproduce the v0.5 look: close to
   the original picture. Keep the v0.5 `cmp_final.png` as the baseline, and after changes check that the default
   frame still matches it. Everything else is range on top of that anchor.
2. **Think in modules and patch points, not in a picture.**
   - The SUMMARY's reading breaks each photo into elements: oscillators, folds, keys, bands, rings, strips, rays,
     cut-outs, textures.
   - Give each individual shape or sub-shape, and each group of repeated shapes, its own parameters wherever possible:
     its frequency or count, phase or position, width or size, slope, level, colour step, edge softness, and its
     rate of motion. Then a knob behaves like a knob on one module, or a patch cord into one input.
   - **Prefer element-level parameters to a global X/Y scale or zoom.** A global scale is acceptable only as one
     option among several.
3. **Broader ranges.** Each control's full travel should produce clearly different pictures, not 10 % around the
   photo.
   - Ron will name the specific controls that need more range.
   - Some may need a narrower or re-centred range to look better: follow his notes.
   - Keep the photo's value reachable, normally at the default position.
4. **Switches with real options.** Each switch should be an obvious look-flip: mirror, a second copy, a different
   waveform, a different routing, a palette family, a mode. Replace switches whose effect is subtle.
5. **No controls that do almost nothing.** If a control's full travel barely changes the picture, re-assign it to an
   element parameter with visible range.
6. **Looser.** Let the patch leave the photo. Examples:
   - wider oscillator rates and free-running drift;
   - elements that can slide, detune or invert relative to each other;
   - one element's signal modulating another's (the patch-cord idea), within the hardware budget;
   - less locking of every element to the measured fit.
   The photo is one setting of the instrument, not its limit.
7. **Ron drives the priorities per program.** He will flash, play and note which controls need more, less or
   different range. Work from his notes; propose new control maps to him before rewiring a program wholesale.

### Constraints to respect (from the repo-root CLAUDE.md and project memory)

- **Twelve physical controls:** K1–K6, S7–S11, P12.
  - P12 stays the headline performance control, with a designed 0 % and 100 %.
  - S11 stays Source: Video / Osc, defaulting to Video. Incoming video is used wherever possible.
  - "As many shapes tied to parameters as possible" therefore means choosing the most expressive element parameters
    for the 12 controls. One control may drive several related element parameters in a designed relationship, like
    one CV multed to several modules through attenuators.
- **Never gate one control's effect behind another control's mode.** Every control must stay visibly live at any
  setting of the others.
- **Never multiply a knob that already spans its full range by a second control.** It eats the first knob's travel.
  Never give a motion program a switch that halts all motion.
- **Hardware contracts stay:**
  - generated avid;
  - one latency in every mode;
  - the blanking gate;
  - the HD HDMI output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16;
  - virtual 1080-line coordinates;
  - colours in the hardware convention (U = Cr).
- **HD timing at the full 74.25 MHz.** Never change `hd_clock_divisor` without asking.

## How to work a program (per tab)

1. **Claim the program** in [Phase 2 progress](#phase-2-progress) (Status `in progress`). Then **read** `<name>/SUMMARY.md`, then `.heirloom_work/rb_<name>/notes.md`, and view `cmp_final.png` and `sheet.png`.
   The prototype (`proto*.py`) and the integer model (`imodel.py`) are there to reuse for quick looks at new
   parameter ranges.
2. **List the elements and their current drivers.** Write down each shape, sub-shape and repeated group from the
   reading and what (if anything) controls it now. That list is where the new parameters come from.
3. **Pick the implementation lever,** cheapest first:
   - a per-field constant computed in the vblank sequencer or microcode engine. This is usually one more slot, nearly
     free in LC.
   - a per-line value in the line engine.
   - pixel-path logic. This is the costly option: every multiply needs its own stage.

   Tables (palettes, knot grids, glyphs, slot maps) are generated by the build hook `<name>.py` into
   `<name>_pkg.vhd`. Edit the generator, not the package.
4. **Measured-field programs** (abacus, goblet, warp, chartreuse, lava, waterfall, fern, mandala, starburst, crossbeam;
   see the table) carry some shapes as fitted tables, not oscillators, so those shapes don't move by themselves.
   - To make them more modular, expose the table's coordinates and levels: per-region offsets and rates, warps of the
     lookup, phase or palette rotation, and blends of two copies.
   - Or replace part of a table with an analog-style generator that approximates it.
   - The collages (triplicate, radiant, greenroom, cutout) have layout maps. Their natural parameters are per-cut-out
     position, scale, slide and tint, and per-scrap camera crop.
5. **Mind the budget** (table below). Programs marked **FULL** (≥ 95 % LC) have no headroom: adding a parameter means
   removing or cheapening something else, judged by standalone-yosys ablation (CLAUDE.md "Fit & high utilization").
   Keep LC ≤ ~92–94 % and ≤ ~24 EBR when possible.
6. **Build and hand over.** Use `./build_heirloom.sh <name>`, with the router and seed noted in the table.
   - Judge timing from the last post-route "Max frequency" line per config.
   - Ron flashes and reports. Hardware eyes are the primary check.
   - Simulate only when it's genuinely needed (CLAUDE.md "Default to building, not simulating"), for example a new
     artifact that can't be reasoned out from the code. A quick check that the default frame still matches the photo
     is reasonable after a large change.
7. **Finish each round:**
   - bump the version (phase 2 starts at v0.6.0);
   - update SUMMARY.md: controls, status, seeds, and what changed;
   - update the program's row in [README.md](README.md) and in [Phase 2 progress](#phase-2-progress);
   - commit only that program's files.
   - When Ron settles on a look, capture it with `./capture_preset.sh`.

## Phase 2 progress

Each phase-2 tab works the first program not yet started, unless Ron names one.
- **When you start:** set its Status to `in progress`.
- **After each build:** set it to `built vX.Y.Z, awaiting HW`.
- **When Ron is happy with it:** set it to `done vX.Y.Z`.
- **Ron's notes:** record his notes (controls to widen, narrow or reassign; looks he wants) in the Notes column, briefly,
  with the detail in the program's SUMMARY.md.

Commit the row along with the program's own files.

| Program | Status | Phase-2 version | Notes |
|---|---|---|---|
| abacus | v0.6.3 HD seeds re-picked after HD HDMI lines (pin skew), awaiting HW | v0.6.3 | Ron 10-28: no X/Y zoom (resize individual beads instead); widen Rims; Slide + Roll also wave individual bead sizes; Etch a bit stronger on video; Flip glides; Swell deeper at 0 %. v0.6.0: K1 Width / K2 Height per-bead warps, all six asks in. Round 2 (sheet): Slide / Roll waves scale beads in X and Y, prominent even slow, each bead distinct -> v0.6.1 per-bead / per-row swell pulses; 95 % LC, 24 EBR, router2 (HD HDMI + SD Analog seed 2). Round 3: S11 Source -> Theme Photo / Gold (Ron: K6 = 0 already turns the video off; exception to "S11 stays Source"). Round 4: S9 Stack -> V Mirror (rows always mirrored) |
| archway | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-28 (on v0.5.0): K1 Steps wider (esp. low end); K3 Etch more top range; K4 Speed: left = lines fast / blocks slow, right = the reverse; K5 Chevron 0 = gone .. 1023 = out to the arch edge; K6 smooth palette morph; S9 -> Y flip (K5 now hides the chevron). v0.6.0: all seven in (K6 = box-mixed dither, 32 steps); 96 % LC, 20 EBR, router2 (HD seeds 2, SD Analog 3) |
| argus | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-29 (on v0.5.0): only note: dislikes S7 Mirror's Repeat option. v0.6.0: S7 = Fold / Off (Off: the silhouettes run unmirrored down the frame, colour field still folded); 94 % LC, 22 EBR, router2, 6/6 seed 1 |
| argyle | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-29 (on v0.5.0): likes almost everything; K1/K2 should change individual diamonds (patterns, per-diamond sizes) rather than scale the whole screen. v0.6.0: K1 Bloom (bipolar: centre diamonds swell / tunnel) and K2 Scatter (each diamond its own random size, breathing with Drift) swell each diamond in its own cell; lattice fixed at the photo's; 94 % LC, 25 EBR, router2 (HD Analog / Dual seed 3, HD HDMI s9 hand-installed after a pin-skew screen), latency 27 |
| beacon | built v0.6.1, awaiting HW | v0.6.1 | Ron 10-29 (on v0.5.0): K1 Bars / K2 Columns +50-100 % range both ways; K3 Dusk lower end squeezes closer to the tower; K5 Beam +50 % both ways; K6 Halo 100 % much closer to the beacon; flame centred on the tower (was offset left); video more prominent; K4 Speed also swells the bands in sequence (sine, amplitude rising with speed). v0.6.0: all eight in (Bars x0.25..x4, Columns 1106..138 px, Dusk squeeze radius onto the tower outline, Beam x0.35..x2.83, Halo to +723 px re-centred on the tower, flame on the bell axis + symmetric, camera through the dark outside the bell, travelling band swell); 93 % LC, 25 EBR, router2, 6/6 seed 1. 10-30 flash: hsync red lines on HD HDMI s2 (+1.29) -> HD HDMI s42 (+2.31), HD Analog s14 hand-installed (v0.5.0's clean HD HDMI was +3.09) 10-30: lines gone (s42 HW-clean). Round 2: Blue palette -> blue flame/beam + amber bars (Ember OK); camera warped into the flame. v0.6.1: both in (64x64 thumbnail texture-mapped to the flame's width/height per line); 95.6 % LC, 29 EBR; all six bitstreams hand-routed (HD HDMI router2 s23 +1.94 ns, HD Analog r1 s12, HD Dual r1 s1) |
| blossom | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-30 (on v0.5.0): Rows -> separate elements scaling differently (not a plain vertical density); Speed -> something other than scrolling; Breathe -> more dynamic or video-modulated; S10 Wave: keep Fold, replace Saw creatively; Bloom +100 % range both ends; Mirror + palettes good. v0.6.0: K1 Rows bipolar (right: more rows cascading x5.2 small at top -> x0.75 at bottom, hearts sized with their row; left: fewer rows, hearts swell tall / lobes squeeze, x1.4 wide); K4 Sway replaces the roll (travelling sideways wave, amplitude + rate rise from a still centre); K6 Breathe = heartbeat travelling down the rows (each row grows x1.41 and blooms; video scales the bloom); S10 Fold / Twist (per-line X scale cos 2 pi u: rows turn about the centre line); P12 Bloom +-3 units. 95 % LC, 24 EBR, router2; HD HDMI s12 (+1.68) + HD Analog s5 (-2.09) hand-installed after a pin-skew screen, HD Dual s2, SD s1/s1/s2 |
| bottleneck | not started | | |
| bunting | not started | | |
| cathedral | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-30 (on v0.5.0): loves the colours, shapes and blending; only range tweaks. K2 Arch: keep going ~50 % past the current 100 %; K3 Diamond: 0-70 % barely changes the size -> 0 % = almost gone, 100 % = ~25 % bigger than the current max. v0.6.0: K2 x0.5..x4 exponential (photo at 33 %, old max at 67 %); K3 radius 10..569 lines quadratic (photo at 50 %), and the diamond's warm halo now shrinks with it (it was filling the lens, hiding the size change); default bit-exact with v0.5; 91 % LC, 20 EBR, router2; HD HDMI s61 + HD Analog s3 hand-installed after the pin-skew screen |
| cathode | not started | | |
| chartreuse | not started | | |
| cocoon | not started | | |
| crenel | not started | | |
| crossbeam | not started | | |
| curtain | not started | | |
| cusp | not started | | |
| cutout | not started | | |
| escalator | not started | | |
| facet | not started | | |
| fern | not started | | |
| goblet | not started | | |
| greenroom | not started | | |
| halo | not started | | |
| idol | not started | | |
| jukebox | not started | | |
| kiln | not started | | |
| lava | not started | | |
| lotus | not started | | |
| lozenge | not started | | |
| magma | not started | | |
| mandala | not started | | |
| marmalade | not started | | |
| medallion | not started | | |
| ovals | not started | | |
| plume | not started | | |
| radiant | not started | | |
| ribboncandy | not started | | |
| ricochet | not started | | |
| riptide | not started | | |
| rorschach | not started | | |
| seismic | not started | | |
| shield | not started | | |
| sinew | not started | | |
| spearhead | not started | | |
| starburst | not started | | |
| streak | not started | | |
| tangram | not started | | |
| tartan | not started | | |
| terrace | not started | | |
| ticker | not started | | |
| triplicate | not started | | |
| vase | not started | | |
| vesica | not started | | |
| warp | not started | | |
| waterfall | not started | | |

## Per-program reference

- **LC:** the highest of the 6 configs at v0.5.
  - **room:** below 92 %.
  - **tight:** 92–94 %.
  - **FULL:** 95 % and above.
- **Router / seeds:** as built. Seeds the builder found on its own retry are listed, so a plain rebuild normally finds
  them again, except radiant (`SEED=3`). Prefix router1 builds with `NEXTPNR_ROUTER="--router router1"`.
- **Type:**
  - **oscillator/field:** analytic generators and fitted fields.
  - **measured field:** major shapes stored as fitted tables.
  - **collage:** a layout map, with the camera printed through cut-outs.

| Program | Version | LC | EBR / 32 | Router | Seeds (non-1 noted) | Type |
|---|---|---|---|---|---|---|
| [abacus](abacus/SUMMARY.md) | v0.5.0 | 83 % (room) | 22 | router2 | HD Analog seed 5, HD HDMI seed 2, HD Dual seed 6 | measured field |
| [archway](archway/SUMMARY.md) | v0.6.0 | 96 % (**FULL**) | 20 | router2 | HD Analog / HDMI / Dual seed 2, SD Analog seed 3 | oscillator/field |
| [argus](argus/SUMMARY.md) | v0.6.0 | 94 % (tight) | 22 | router2 | 6/6 seed 1 | oscillator/field |
| [argyle](argyle/SUMMARY.md) | v0.6.0 | 94 % (tight) | 25 | router2 | HD Analog / HD Dual seed 3; HD HDMI seed 9 hand-installed | oscillator/field |
| [beacon](beacon/SUMMARY.md) | v0.6.1 | 96 % (**FULL**) | 29 | router1 + router2 | hand-routed: HD HDMI r2 s23, HD Analog r1 s12, HD Dual r1 s1, SD Dual s2 | oscillator/field |
| [blossom](blossom/SUMMARY.md) | v0.6.0 | 95 % (**FULL**) | 24 | router2 | HD HDMI s12 + HD Analog s5 hand-installed; HD Dual / SD Dual seed 2 | oscillator/field |
| [bottleneck](bottleneck/SUMMARY.md) | v0.5.0 | 96 % (**FULL**) | 17 | router1 | SD HDMI seed 2 | oscillator/field |
| [bunting](bunting/SUMMARY.md) | v0.5.0 | 83 % (room) | 4 | router1 | 6/6 seed 1 | oscillator/field |
| [cathedral](cathedral/SUMMARY.md) | v0.6.0 | 91 % (room) | 20 | router2 | SD Analog seed 3; HD HDMI s61 + HD Analog s3 hand-installed | oscillator/field |
| [cathode](cathode/SUMMARY.md) | v0.5.0 | 92 % (tight) | 2 | router2 | 6/6 seed 1 | oscillator/field |
| [chartreuse](chartreuse/SUMMARY.md) | v0.5.2 | 91 % (room) | 23 | router1 | 6/6 seed 1 | measured field |
| [cocoon](cocoon/SUMMARY.md) | v0.5.0 | 94 % (tight) | 19 | router2 | HD HDMI seed 4, SD Dual seed 2 | oscillator/field |
| [crenel](crenel/SUMMARY.md) | v0.5.0 | 92 % (tight) | 23 | router1 | 6/6 seed 1 | oscillator/field |
| [crossbeam](crossbeam/SUMMARY.md) | v0.5.0 | 92 % (tight) | 23 | router1 | 6/6 seed 1 | measured field |
| [curtain](curtain/SUMMARY.md) | v0.5.0 | 94 % (tight) | 23 | router2 | HD HDMI + SD Dual seed 2 | oscillator/field |
| [cusp](cusp/SUMMARY.md) | v0.5.0 | 94 % (tight) | 11 | router2 | 6/6 seed 1 | oscillator/field |
| [cutout](cutout/SUMMARY.md) | v0.5.0 | 97 % (**FULL**) | 24 | router1 | 6/6 seed 1 | collage |
| [escalator](escalator/SUMMARY.md) | v0.5.0 | 90 % (room) | 7 | router2 | 6/6 seed 1 | oscillator/field |
| [facet](facet/SUMMARY.md) | v0.5.0 | 94 % (tight) | 20 | router2 | HD Dual seed 2 | oscillator/field |
| [fern](fern/SUMMARY.md) | v0.5.0 | 93 % (tight) | 24 | router1 | HD HDMI seed 2 | measured field |
| [goblet](goblet/SUMMARY.md) | v0.5.0 | 93 % (tight) | 25 | router2 | HD HDMI/Dual seed 2 | measured field |
| [greenroom](greenroom/SUMMARY.md) | v0.5.0 | 95 % (**FULL**) | 25 | router1 | 6/6 seed 1 | collage |
| [halo](halo/SUMMARY.md) | v0.5.0 | 89 % (room) | 23 | router2 | 6/6 seed 1 | oscillator/field |
| [idol](idol/SUMMARY.md) | v0.5.0 | 89 % (room) | 25 | router2 | HD Dual seed 3 | oscillator/field |
| [jukebox](jukebox/SUMMARY.md) | v0.5.0 | 84 % (room) | 18 | router2 | HD Dual seed 4 | oscillator/field |
| [kiln](kiln/SUMMARY.md) | v0.5.0 | 93 % (tight) | 24 | router2 | HD Analog seed 3, HD Dual seed 2 | oscillator/field |
| [lava](lava/SUMMARY.md) | v0.5.0 | 87 % (room) | 21 | router2 | HD Analog seed 2 | measured field |
| [lotus](lotus/SUMMARY.md) | v0.5.0 | 93 % (tight) | 22 | router1 | HD Dual seed 2 | oscillator/field |
| [lozenge](lozenge/SUMMARY.md) | v0.5.0 | 84 % (room) | 15 | router1 | 6/6 seed 1 | oscillator/field |
| [magma](magma/SUMMARY.md) | v0.5.0 | 92 % (tight) | 22 | router1 | 6/6 seed 1 | oscillator/field |
| [mandala](mandala/SUMMARY.md) | v0.5.0 | 92 % (tight) | 21 | router2 | 6/6 seed 1 | measured field |
| [marmalade](marmalade/SUMMARY.md) | v0.5.0 | 76 % (room) | 8 | router1 | 6/6 seed 1 | oscillator/field |
| [medallion](medallion/SUMMARY.md) | v0.5.0 | 82 % (room) | 14 | router2 | HD Analog/HDMI seed 2 | oscillator/field |
| [ovals](ovals/SUMMARY.md) | v0.5.1 | 99.6 % (**FULL**) | 15 | router2 | HD Dual seed 2 | oscillator/field |
| [plume](plume/SUMMARY.md) | v0.5.0 | 91 % (room) | 23 | router2 | HD HDMI + SD HDMI seed 2 | oscillator/field |
| [radiant](radiant/SUMMARY.md) | v0.5.0 | 90 % (room) | 16 | router2 | base SEED=3 6/6 | collage |
| [ribboncandy](ribboncandy/SUMMARY.md) | v0.5.0 | 89 % (room) | 8 | router2 | HD Analog seed 2, HD HDMI/Dual seed 3 | oscillator/field |
| [ricochet](ricochet/SUMMARY.md) | v0.5.0 | 89 % (room) | 13 | router1 | 6/6 seed 1 | oscillator/field |
| [riptide](riptide/SUMMARY.md) | v0.5.0 | 99 % (**FULL**) | 27 | router1 | HD Analog seed 2 | oscillator/field |
| [rorschach](rorschach/SUMMARY.md) | v0.5.0 | 90 % (room) | 24 | router2 | HD Dual seed 3 | oscillator/field |
| [seismic](seismic/SUMMARY.md) | v0.5.0 | 92 % (tight) | 11 | router1 | 6/6 seed 1 | oscillator/field |
| [shield](shield/SUMMARY.md) | v0.5.0 | 87 % (room) | 12 | router1 | 6/6 seed 1 | oscillator/field |
| [sinew](sinew/SUMMARY.md) | v0.5.0 | 90 % (room) | 22 | router1 | 6/6 seed 1 | oscillator/field |
| [spearhead](spearhead/SUMMARY.md) | v0.5.0 | 76 % (room) | 4 | router2 | 6/6 seed 1 | oscillator/field |
| [starburst](starburst/SUMMARY.md) | v0.5.0 | 89 % (room) | 13 | router2 | all configs seed 3 | measured field |
| [streak](streak/SUMMARY.md) | v0.5.0 | 95 % (**FULL**) | 20 | router2 | 6/6 seed 1 | oscillator/field |
| [tangram](tangram/SUMMARY.md) | v0.5.0 | 93 % (tight) | 24 | router1 | HD Dual seed 5 | oscillator/field |
| [tartan](tartan/SUMMARY.md) | v0.5.0 | 75 % (room) | 7 | router2 | HD HDMI seed 2 | oscillator/field |
| [terrace](terrace/SUMMARY.md) | v0.5.0 | 89 % (room) | 7 | router2 | HD HDMI seed 2 | oscillator/field |
| [ticker](ticker/SUMMARY.md) | v0.5.0 | 90 % (room) | 2 | router2 | HD HDMI + SD Analog seed 2 | oscillator/field |
| [triplicate](triplicate/SUMMARY.md) | v0.5.0 | 92 % (tight) | 24 | router2 | 6/6 seed 1, thin HD margins | collage |
| [vase](vase/SUMMARY.md) | v0.5.0 | 91 % (room) | 24 | router1 | HD Analog seed 5 | oscillator/field |
| [vesica](vesica/SUMMARY.md) | v0.5.0 | 97 % (**FULL**) | 22 | router1 | 6/6 seed 1 | oscillator/field |
| [warp](warp/SUMMARY.md) | v0.5.0 | 71 % (room) | 23 | router2 | HD HDMI seed 4 | measured field |
| [waterfall](waterfall/SUMMARY.md) | v0.5.0 | 94 % (tight) | 24 | router1 | 6/6 seed 1 | measured field |
