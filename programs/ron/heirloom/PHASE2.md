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
  - **curtain:** its sources were tidied after its build, so rebuild before release (done: rebuilt at v0.6.0, 2026-11-01).
  - **radiant:** must be built with `SEED=3`. HD HDMI fails placement at seeds 1–2, and the builder does not retry
    placement errors.
  - **goblet:** the "Hourglass" preset also turns the stripes off (unintended; fixed in v0.6.0, 2026-11-02).
  - **Fitted tables can hide a single-entry spike** (jukebox, found on its first flash 2026-11-02). A photo fit that
    interpolated across a phase unwrap left one far-off entry in a band-offset table: thin full-height rainbow
    lines at a fixed x in every column. If a measured-field program shows x-locked thin lines in image colours,
    scan each table per kind for an isolated second-difference spike (wrap phase-like fields mod their period).
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
   - Cheap room in every program (from cutout v0.6.0): the shell's output chroma filter keeps two 9-stage 8-bit
     delay lines (`s_csu` / `s_csv`) but reads only the 8-back tap. A 256 × 16 block-RAM ring (read pointer =
     write − 6, then one fabric register) replaces them for ~−80 LC and +1 EBR. FF-to-FF copies (result copies,
     carries) cost a whole LC per bit, while registers on LUT outputs pack almost free, so cut copies first.
6. **Preview first, in a diag copy** (Ron, 2026-11-01). Propose a control map, then build the first round of changes
   in a diag copy, not in `programs/ron/heirloom/<name>/`. Leave the original intact until Ron previews and approves.
   - `./heirloom_diag.sh new <name>` copies the program to `.heirloom_work/rb_<name>/diag/<name>_diag/`. The copy gets
     its own program_id (`…<name>_diag`) and name ("<Name> Diag"), so it loads beside the original.
   - `./heirloom_diag.sh build <name>` writes `out/rev_b/ron/_diag/<name>_diag_v<version>.vmprog`.
   - Row status: `diag vX.Y.Z built, awaiting preview`.
   - On approval, run `./heirloom_diag.sh merge <name>`, then do step 7 on the production copy.
7. **Build and hand over.** Use `./build_heirloom.sh <name>`, with the router and seed noted in the table.
   - Judge timing from the last post-route "Max frequency" line per config.
   - Ron flashes and reports. Hardware eyes are the primary check.
   - Simulate only when it's genuinely needed (CLAUDE.md "Default to building, not simulating"), for example a new
     artifact that can't be reasoned out from the code. A quick check that the default frame still matches the photo
     is reasonable after a large change.
8. **Finish each round:**
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
| bottleneck | deferred | | Ron 10-30: "weird", skip for now and come back to it later |
| bunting | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-30 (on v0.5.0): K1 Flags +100 % range both ways; K2 Band bipolar (flip which way the shape faces); K3 Flare bipolar flip, equal both sides; K4 Speed glides the glyphs side to side, wide travel; K5 Glyphs randomized glyphs that morph as you zoom; P12 Spread scales horizontally, then vertically, with a smooth handover. v0.6.0: all six in (Flags 0.375..12 X's; Band = pennant taper; Flare mirrored, photo 768 / mirror 255; Speed keeps the X drift and glides the glyph grid -1200..+846 px from home at 600; per-cell glyph re-roll away from K5 621; Spread sh 0.25..1 then sv 1..1.8); first build 99 % LC, fixed by moving the zone adders into the multiplier; 88 % LC, 5 EBR, router1 6/6 seed 1, HD seeds re-picked for pin skew (HD Analog s5, HD HDMI s7 at -0.86 next to v0.5's -0.94, HD Dual s2; spare HD HDMI router2 s9 vmprog in the work dir) |
| cathedral | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-30 (on v0.5.0): loves the colours, shapes and blending; only range tweaks. K2 Arch: keep going ~50 % past the current 100 %; K3 Diamond: 0-70 % barely changes the size -> 0 % = almost gone, 100 % = ~25 % bigger than the current max. v0.6.0: K2 x0.5..x4 exponential (photo at 33 %, old max at 67 %); K3 radius 10..569 lines quadratic (photo at 50 %), and the diamond's warm halo now shrinks with it (it was filling the lens, hiding the size change); default bit-exact with v0.5; 91 % LC, 20 EBR, router2; HD HDMI s61 + HD Analog s3 hand-installed after the pin-skew screen |
| cathode | built v0.6.1, awaiting HW | v0.6.1 | Ron 10-31 (on v0.5.0): biggest complaint: the flames only sway lightly; they need more dynamic shape and movement. v0.6.0: the sway is replaced by a wave running down each tube (base to tip, growing toward the tip), the three tubes at their own rates (0.75 / 1 / 0.625 of K4) so they writhe out of step and cross; each tube's length licks +-32 on sy8 (triangle LFO) and thins as it stretches; P12 also sets the writhe (~10 / 73 / 135 px at 0 / 50 / 100 %); K4 left runs the waves tip to base; no control re-assigned. Ron 10-31: flames more affected by the video -> v0.6.1: in Video mode the camera's low-passed luma minus its mean displaces the flame coordinate by g/4 px per level (g 1..3 from P12, 2 at the default), so the tube outlines trace the picture; Osc unchanged. 96 % LC, 2 EBR, router2 builder 6/6 seed 1; HD seeds from a pin-skew screen: HD HDMI s32 (+1.62 / <= -0.79), HD Analog s6, HD Dual s1 |
| chartreuse | built v0.6.1, awaiting HW | v0.6.1 | Ron 10-31 (on v0.5.2): K1 Crescents / K2 Arms split into components that scale and move differently, not a global X/Y zoom, more range both ways; K3 Sphere subtle, widen both ways; breathing much more prominent; S8 Sway redundant (Speed turns it off), repurpose; Speed range = the old -40..+40 %. v0.6.0: K1 = the crescents' own spacing x0.35..x2.2 (sphere keeps its size); K2 = bipolar arm swing (spacing changes with distance from the axis: bars .. photo .. chevron fan); K3 = round cone on the sphere, radius 0..254 (crater .. lit ring / target, +-1.5 F); K4 = old +-40 %, still at centre, depth rises with |K4|: bands breathe +-0.19 F, crescents pulse +-37 %, sway +-48 px; S8 Mirror (two spheres side by side; four with S7); first build 102 % LC, fitted by slot-computed knob curves, radius-x24 dome, glides on K1/K2/P12 only, bit-decoded fold-line windows, RAM-register window. v0.6.1 = + output-filter resync (96c4a7c). 94 % LC, 23 EBR, router1 (HD Dual seed 3, rest seed 1; HD HDMI 74.58 thin) |
| cocoon | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-31 (on v0.5.0): more weave modulation by the arcs, its strength on K4 Glow with the glow; S9 Band -> something more interesting, band permanent; P12 0 % = no band at all .. 100 % = the current 100 % (band fills the screen); the red box (glow window) fades out as K4 goes down, in as it goes up, with the diamond glow. v0.6.0: all four in (arc triangle fed back through a k x tri ROM into the row phase + dash width, k = K4 cube, +-34 at the photo; box: camera below 25 %, black, glow in by 50 %; band 0..247 over P12 0..50 %; S9 Solid / Slats, per-pixel band from the row fold); 96 % LC, 21 EBR, router2; HD seeds from a pin-skew screen: HD HDMI s11 (+1.22 / <= -0.61), HD Analog s8, HD Dual s2; SD Analog s2 |
| crenel | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-31 (on v0.5.0, before flashing it): K2 Tiles and K4 Drift -> granular element controls; S7 Tear -> something else; give the dome, V, arch and dash elements their own controls. v0.6.0: K2 Dome (R 24..2068 px, a true circle, the photo at 471); K4 Drift per band (each row its own rate and direction, x-3..x3; still in the centre); S7 Swap (pink shapes print the camera, camera areas turn pink); tears always on. V's / arch / dashes get their own drift rates but no knob (all 12 controls taken). 94 % LC, 23 EBR, router1 builder; HD seeds from a pin-skew screen: HD HDMI router2 s17 (+2.00 / <= -1.63), HD Analog r1 s3, HD Dual r1 s7 |
| crossbeam | meh (v0.6.0) | v0.6.0 | Ron 11-01: "meh", parked. Ron 10-31 (on v0.5.0, sight unseen, free hand): "needs more". K2 / K6 no longer global grid scales; beam more dynamic and adjustable; warp the table lookup and let the beam push the contours; S8 Tear -> something more interesting. v0.6.0: K2 Rows / K6 Columns bipolar per-row height / block scale (left Scatter, right Lens around the beam line, columns bulge); K3 Warp (density dropped) ripples the zebra / ramp lookup; K4 also rolls the bars; K5 Swing (right scissor, left the cross travels up); P12 also pushes the contours (signed, ghost X at 0 %); S8 Tear Still / Slip (rows slide at their own speeds); K1 0.02..0.77. New per-line engine (serial multiplier + line ROM); 91 % LC, 23 EBR, router1 6/6 seed 1 |
| curtain | built v0.6.1, awaiting HW | v0.6.1 | Ron 10-31 (on v0.5.0, sight unseen): approved the proposed map, plus something with the seam height or a blend that removes the seam (K1 traded for it, his pick). v0.6.0: K1 Seam (wipe along the pleat sawtooth, stripe-roughened, half-height K1^2/951 lines; each half's table mirrored past the seam; both halves' rows on every line); K3 Weave (stripe amplitude + bite + chevron depth + drape slant, 0 = flat fields); K4 also drifts the pleats right; S7 Swap (drapes up), S8 Arcs / Ribbons (2.5x, ungated), S9 Teeth / Fire (3x flames, big domes); defaults = v0.5. Fit from 102 % pack: unsigned melt/weave multiplies, luma-difference RGB->YUV, chroma filter delay in RAM. Ron 11-01 (flashed v0.6.0): "liked the previous one more", the default no longer looked like v0.5.0 -> the K4 pleat drift moved the cuts at the default Speed and the x-locked fits left the photo within seconds. v0.6.1: the pleats stand at the photo position at K4 600 and drift with K4 - 600; 95 % LC, 27 EBR, router2; HD Analog s5, HD HDMI s11 hand-installed (+1.88 / <= -0.67), HD Dual s6 |
| cusp | built v0.6.0, awaiting HW | v0.6.0 | Ron 10-31 (on v0.5.0, sight unseen, picked from a proposed map): K1 Cusps = black-core rows only (orange rows keep theirs, rows detune); K2 Rows -> Split (orange-row height, seams .. dominant, row count fixed); K4 also slides the row types apart, scrolls the stripes, cycles the rainbow; K6 Bend wider (~1 cycle), bolder Osc, and the key bends the fine stripes; K3 Fish wider; S8 -> Alternate / Opposed (orange rows point left); S9 -> Core Black / Rainbow. v0.6.0: all in. Orange rows the other way at the same rate (one drift accumulator also drives the stripe scroll and rainbow LFO); the stripes breathe 2 periods into the cores and back (a wrapping scroll ticked every rim); curl only outside the cores; S9 core = yellow-to-pink glow from the L layer; K3 Fish 0..4x, K6 Bend to ~1.1 cycle. GHDL = model bit-exact. 97 % LC, 11 EBR, router2; HD seeds from a pin-skew screen: HD HDMI s138 (+0.96 / <= -0.78; nothing >= +1.0 with data <= -0.6 in 144 router2 seeds, router1 all negative), HD Analog s48, HD Dual s11 |
| cutout | built v0.6.0, awaiting HW | v0.6.0 | Ron 11-01 (on v0.5.0, before flashing it; picked from a proposed map): K2 Zoom -> Crops (0 = all scraps in register, 50 % = photo, 100 % = crops slide apart, scraps flip / rescale one by one); K6 per-scrap drift + "Love." crawl; S8 Fringe -> Inks Photo / Swap, fringe onto K1 Rag; P12 unchanged; S10 -> Soup / Camera mirror. v0.6.0: all in (33 per-scrap crop words {crop, flip, scale} recomputed per field by the sequencer into a crop RAM; Crops 0 = one camera frame zoomed across the head; drift ping-pongs on each scrap's own line; Swap = warm head / cool cup and plate; Mirror = the live camera, mirrored, grey, in the soup; K3 Ink 0 = flat .. x1.9). First build 7728 LC, fitted with no feature lost (chroma filter delay into a block-RAM ring, 12-bit noise phases, leaner sequencer); 97.8 % LC, 27 EBR, router1 builder 6/6 seed 1; HD seeds from a pin-skew screen: HD HDMI s17 (+2.01 / <= -0.72), HD Analog s7 (+0.70 / <= -1.16, edge of the window, none better in 54 seeds), HD Dual s7 |
| escalator | built v0.6.0, awaiting HW | v0.6.0 | Ron 11-01 (on v0.5.0, before flashing it; picked from a proposed map): K1 Columns -> grid only; K2 Bands -> count + height; K5 Speed -> unison ride + column bob; S8 Climb -> Ramp; wider K3 Blob / K6 Bend. v0.6.0: all in (tile and slab phases split, stairs fixed at the photo's 7.14 columns, K1 Tiles 2..20 square law, locked to the stairs at 547; K2 bipolar 1.57 bands at 84 % .. photo .. 11.9 at 6.8 %; K5 to ~24 lines/frame + a unipolar bob wave travelling across the columns (9-entry column table in one EBR, depth 0..236 lines with speed^2); S8 Stairs / Ramp (diagonal saw ribbons); K3 to +1/2 tile (blobs vanish); K6 cubic, 4.5x v0.5's max, default = the photo). First build 98 % LC, fitted by chaining sequencer products and a leaner bob; 94 % LC, 8 EBR, router2; HD seeds from a pin-skew screen: HD HDMI s45 (+2.30 / <= -0.91), HD Analog s19 (+1.64 / <= -0.57), HD Dual s10 |
| facet | done v0.6.2 | v0.6.2 | Ron 11-01 (on v0.5.0, before flashing it; picked from a proposed map): K5 Halo -> Spread (per-eye-pair heights, bipolar; halo width onto P12); K4 Speed = rate + depth (eye-pair ripple wave, wedge/floor slide); S7 Lines -> Floor V / Peak; S8 Glow -> Eyes Lens / Diamond. v0.6.0 diag (production v0.5.0 untouched): all four in, plus K4 square law (v0.5 motion at the new default 50 %, x4 at 100 %), eye-pair ripple and wedge/floor slide to +-253; 96 % LC, 26 EBR, router1 (SD Dual s2; HD HDMI s10 hand-installed after a pin-skew screen, +2.03 / <= -0.55); preview out/rev_b/ron/_diag/facet_diag_v0.6.0.vmprog. v0.6.1 diag (resync fix) HW: colours correct. Ron 11-02 (on v0.6.1): Wedge sweep further; Floor angle changes as it rises; wedge/floor overlap XOR-style mix with the ground, using the video. v0.6.2 diag: K2 cubic kA 0.02 .. 5 (photo at 38 %), K3 slope kB 0.21 .. 0.373 (photo) .. 1.2 riding the height, overlap = purple/ground (red/light in eyes) bands flipped by the source luma contours; 97 % LC, 26 EBR, router1 (HD Analog s2); preview out/rev_b/ron/_diag/facet_diag_v0.6.2.vmprog. Ron 11-02: approved; merged (production = the approved diag bitstreams) |
| fern | not started | | |
| goblet | built v0.6.0, awaiting HW | v0.6.0 | Ron 11-02 (on v0.5.1, before flashing it): likes it a lot; just widen the knob ranges where it makes sense. v0.6.0: K1 Stripes x0.25..x4 (was x0.5..x2); K2 Width x0.4..x3 (was x0.6..x1.6; narrow repeats the goblet ~3x, wide zooms into the bowl); K5 Melt square law, default unchanged, top x2.9 (video bends to +-176 px); Osc oscillator 4x on a cubic Melt curve (Osc default still the photo, full Melt zigzags the stripes); K3 / K4 / K6 / P12 unchanged; Hourglass preset no longer turns the stripes off. Default frame bit-exact with v0.5 (model). 94.7 % LC, 25 EBR, router2; HD HDMI seed 3, HD Dual seed 2, rest seed 1. Ron 11-02: approved the diag preview; merged and rebuilt with the same seeds |
| greenroom | not started | | |
| halo | built v0.6.0, awaiting HW | v0.6.0 | Ron 11-02 (on v0.5.1): K1, K2, K3, K5, K6 +100 % range both ways; S9 (written "P9") a more prominent change. v0.6.0 (diag approved 11-02, merged): each side of the photo setting doubled: K1 Rings x0.19..x2.25 exp (photo at 68 %), K2 Cells 0.5..8 exp (photo at 50 %), K3 Pinch -100..220 % (negative = lens columns), K5 Tear -100..300 % (negative tears the other way), K6 Warp -15..185 %; S9 Ember Crescent / Core (red-hot inside every ring, brown surround). 92 % LC, 23 EBR, router2 (HD HDMI s3, rest s1) |
| idol | **v0.6.1 FINAL** (diag approved by Ron, merged 2026-11-03) | v0.6.1 | **Merged 2026-11-03**, production built router2 6/6 seed 1 (same Fmax as the diag), `out/rev_b/ron/heirloom/idol.vmprog`; presets still hand-written. **v0.6.1 (Ron's flash of v0.6.0: "not seeing much from P3 Axis"):** the idol now rides on the mirror line: a K3-scaled store-row DDA folded about the photo's line; 13 .. 91 %; below the photo the fold shrinks into stacked idols (4 at 0 %), above it the top half stretches down to the line; live with Mirror off too; default unchanged; 94 % LC, 27 EBR, router2 all seed 1 (HD HDMI 74.79 / HD Analog 75.13 thin); out/rev_b/ron/_diag/idol_diag_v0.6.1.vmprog. No Ron notes (free hand, 2026-11-02 batch). v0.6.0 diag (production v0.5.1 untouched): K1 Feather = streaks 0..4x + slips 0..24x (calm areas at half rate, 0 = clean idol); K3 Axis 12..92 % (stacked mirror copies low); K4 Speed also swells the colour bands in sequence and ripples the rows sideways (+-129 px at 1080p), depth past the default speed, 0 at the default; K6 Ground to 0.44 cycle each side; S9 Slips -> Palette Photo / Rainbow (black ground); P12 Bands 0.25..4x exponential; default frame bit-identical to v0.5.1 (GHDL Osc). Latency 28 -> 30 (two swell stages, parity kept). First build 96 % LC, fitted (4-bit swell depth, single-add ripple, chroma-filter delay into a block-RAM ring); 93 % LC, 27 EBR, router2 (HD HDMI s2, rest s1; HD Analog 74.29 thin); preview out/rev_b/ron/_diag/idol_diag_v0.6.0.vmprog |
| jukebox | **FINAL v0.6.1** (diag merged into production 2026-11-03), built, awaiting HW | v0.6.1 | Ron 11-03: "jukebox approved" (after flashing diag v0.6.1). Production build 2026-11-03: router2 6/6 seed 1 (HD 83.1 / 86.6 / 85.5 MHz), 95.5 % LC, 21 EBR; bitstreams byte-identical to the approved diag; not yet flashed as production, out/rev_b/ron/heirloom/jukebox.vmprog. **v0.6.1 diag (2026-11-02, Ron's first-flash notes on v0.6.0):** thin rainbow lines = one bad fitted E-table entry (A pos 250, interpolated across a 4-fold unwrap; 5 lines at 1080p, 5 px before each A -> B cut), fixed; K1 Lens/Fan both to L = 1.5 (fold into mirrored columns past 1); K2 Cascade x10 top (~10 rows), Alternate x20 / x1.5; K3 Nose 0-200 % readout, photo at 100 %, 21x at the top; E interpolated between table positions (no stepping when magnified); 3 block-RAM delay rings pay for it. 7,303-7,333 LC (95.5 %), 21 EBR, router2 6/6 seed 1 (HD 83.1 / 86.6 / 85.5 MHz); preview out/rev_b/ron/_diag/jukebox_diag_v0.6.1.vmprog. Earlier: no Ron notes (free hand, 2026-11-02 batch). v0.6.0 diag (production v0.5.1 untouched): grid fixed at the photo's 9.15 x 2.08; K1 Columns -> Widths (bipolar: Lens, centre columns swell to ~600 px / edges ~100 px, L capped at 0.81 / Fan, columns 630 -> 80 px left to right with the centre held; step from a second-order DDA); K2 Rows -> Heights (bipolar: Cascade, rows small at the top growing down, ~5.4 rows / Alternate, ~414-line capsules between ~49-line strips); K3 Nose long side x10.7 (lower half bit-exact); K5 Tint -> Detune (column kinds' bands x2.84 / x0.36 against each other); K6 Etch 0..6 folds (photo now at 326); S8 Pairs -> Fold Tri / Saw. Default frame bit-exact with v0.5 in GHDL (x >= 8; x 4-7 = the v0.5.1 resync). First build 7,891 LC, fitted with no feature lost (DDA in place of the per-pixel slope multipliers, shift-add knob curves, chained sequencer products, shared and pipelined knob decoder, registered slot decode); 95.3 % LC, 18 EBR, router2 6/6 seed 1 (HD 85.7 / 83.7 / 83.9 MHz); preview out/rev_b/ron/_diag/jukebox_diag_v0.6.0.vmprog |
| kiln | **done v0.6.1** (diag merged into production 2026-11-03) | v0.6.1 | Ron 11-03: "finalize kiln"; merged, production vmprog = the v0.6.1 diag bitstreams repacked (open questions settled as built: K4 wave stays cut, Video rims stay). Ron flashed v0.6.0 (2026-11-02): "pretty much the perfect program"; one change: replace Quench with a K1 Tiles mode that glides and locks into whole tile counts. v0.6.1 diag: S10 Tile Lock Free / Lock (Free = v0.6.0 K1; Lock = 1..6 whole copies in six equal K1 bands, the X scale gliding 1/16 per field and landing exactly; Quench removed). 96 % LC, router2: HD Analog s4 76.41, HD HDMI s2 77.75, HD Dual s6 76.58, SD s1; preview out/rev_b/ron/_diag/kiln_diag_v0.6.1.vmprog. Earlier: no Ron notes (free hand, 2026-11-02 batch). v0.6.0 diag (production v0.5.1 untouched): K1 Tiles 0.5..6.4 copies exp (photo at 54 %, bit-exact); K2 Fibres colour noise as before + its own tear factor 1x..3.9x above the photo; K3 Blocks 0.25..4x exp; K5 Rims -1x..2.9x and live in Video (edge map of the camera store; was dead in Video, so the default Video picture now has faint pale rims; Osc default unchanged); S9 Weave (Smear near-invisible) -> Strips (8 mirrored bands per half, quarter-copy offsets, half stand while the rest scroll); drift wraps at 2 copies (Kaleido pop fix). K4 sideways wave built then cut for fit (97.5 % LC, HD Analog 65-73 MHz on 5 seeds; code kept in rb_kiln/p2/kiln_p2d_wave.vhd); x17 tear cut to 3.9x. Fit: chroma-filter + sync delays into 3 block-RAM rings, one shared hash pipeline, Glaze/clamp split across stages 21/22. Default Osc frame bit-exact with v0.5 (model + GHDL). 94 % LC, 27 EBR, router2 (HD Dual s6, rest s1); preview out/rev_b/ron/_diag/kiln_diag_v0.6.0.vmprog |
| lava | **done v0.6.0** (diag merged into production 2026-11-03) | v0.6.0 | Ron 11-03 (on the v0.6.0 diag): "I like it the way it is"; merged unchanged, production vmprog = the approved diag bitstreams repacked (both open questions settled as built: Camera ground stays, the comb deepens with Drip). Earlier: no Ron notes (free hand, 2026-11-02 batch). v0.6.0 diag (production v0.5.1 untouched): K1 Columns x0.25..x4 exp (photo 50 %); K2 Bend -1..3 (negative = valley; Bend now per warp knot in the line engine); K3 Rows -> per-band heights, bipolar cascade with the outline fixed (per-field remap RAM t + c*t(1034 - t)/1034, c +-0.98; left = worms swell, right = web / blobs / scallops swell); K4 Speed x2 per step (default 560 = photo drift); K5 Ripple -> Drip (per-drop tongues down / arches up, each drop its own breathing length, on a per-field 48-knot table added in the line engine; the comb rides 2.5 -> 10 px); K6 Melt x2 per step (default 100 = photo); S8 Ground Black -> Camera (camera luma in violet / Osc clouds). Default frame bit-exact with v0.5.1 (GHDL raw diff 0). First design ~98 % LC; fitted by moving Drip and Bend off the pixel path, cheaper curves, latency 36 -> 32 (output copies, even change keeps U/V parity) and a pre-registered Mirror turn; the ground-stripe oscillator was dropped for fit. 94.6 % LC, 26 EBR, router2 (HD HDMI s2, SD Dual s2, rest s1); preview out/rev_b/ron/_diag/lava_diag_v0.6.0.vmprog |
| lotus | not started | | |
| lozenge | **done v0.6.0** (diag merged into production 2026-11-03) | v0.6.0 | Ron 11-03: "lozenge diag, approved"; merged unchanged, production vmprog = the approved diag bitstreams repacked (both open questions settled as built: Bulge replaces Aspect, the star side stays uncapped with the quatrefoil at 0). Earlier: no Ron notes (free hand, 2026-11-02 batch). v0.6.0 diag (production v0.5.1 untouched): K3 Aspect -> Bulge (quadratic shaper on both triangles, c*(tx^2+ty^2), square law: left = 4-point stars whose rings widen outward and fold back, 0 = quatrefoil; 512 = the photo, bit-exact; right = round lens / oval rings crowding outward; c tracks the ring gain so P12 keeps the shape; per-field 1024-entry c*r^2 table filled by a DDA in vblank, read by \|x\| in the pixel path and by \|y\| in hblank, 4 EBR, no multipliers); K1 Stripes x1/8..x8; K2 Chirp to 3x the photo's squeeze (two-slope, photo at 772); K5 Black 0 (no black, navy rings) .. 3x; presets re-aimed + Quatrefoil / Rounds / No Black / Star Twins. GHDL: default frame byte-identical to v0.5, Bulge frame bit-exact with the model. 93 % LC, 19 EBR, router1 6/6 seed 1 (HD 82.9 / 83.1 / 81.8 MHz); preview out/rev_b/ron/_diag/lozenge_diag_v0.6.0.vmprog |
| magma | not started | | |
| mandala | done v0.6.0 | v0.6.0 | No Ron notes (free hand, 2026-11-02 batch). v0.6.0 (diag previewed, then merged): the face's elements become patch points: a region ROM tags the 43x46 control points (eyes / mouth / cheeks + brow / forehead + nose bridge / outer lobes, soft 1/4..1 weights) and the line engine adds a per-element offset before the blend; K3 Blue -> Eyes (+-4 stripes: shut .. black pupils); K6 Breathe above its default deepens and drives five detuned element LFOs (9.7-27 s); K2 Width 16 banks 0.12..8 (default 608); P12 x0.125..x3.6; K4 Red +-3; K5 Melt x1.9 at the top; S9 Edge -> Glow Ramp / Bands. Defaults bit-exact with v0.5 (model, three LFO phases); 3 GHDL renders match the model at 98.5-98.7 %. First build 98.5 % LC, slimmed with no feature lost to 96 % LC, 27 EBR, router2 (HD Analog s2, rest s1; HD HDMI 76.92 MHz). Ron 11-03: approved as built ("finalize"); merged into production. The production vmprog repacks the previewed diag bitstreams. Open, unanswered: the Blue level back on a knob? per-element breathing at K6's default? |
| marmalade | done v0.6.0 | v0.6.0 | No Ron notes (free hand, 2026-11-02 batch). v0.6.0 (diag previewed, then merged): K1 Cells / K2 Rows are per-row modules instead of global grid scales: K1 detunes each row's S&H clock by its own share of +-1.5 oct (finer x1.7..x4.4 / coarser x0.6..x0.22; line engine computes each line's cell clock via an exp2 ROM, centred start phase and drift), K2 a bipolar row cascade from the top (rows shrink to ~8 lines or swell to ~900 at the bottom); both defaults moved to 512 (= the photo); K3 stripe bend x2 (+-1.2 cycle), above 25 % the source also bends the cell edges (+-4.5 cells); K4 top x3.25; K5 bipolar: ringing rainbow fringes .. hard .. photo .. smears (signed 9-bit slew kernel); S7 Stepped / Drift (rows slide at their own rates with K4 plus a free run); S9 Spacing -> Tilt Level / Slant (45 deg stripes; bunching always on); S10 Clocks -> Mirror; P12 1..6..36 bands plus band width (25 % .. 50 % .. 87 %: the stripes take over the blocks); fixed: the first line of each shifted band uses its own shift state. Default GHDL 1080p Osc render matches v0.5 (98.1 % px identical; rest = 1-px tilted-stripe edges, line-end resync, the two shift-toggle lines). 89 % LC, 10 EBR, router1 6/6 seed 1 (HD 93.01 / 92.16 / 94.98 MHz). Ron 11-03: approved as built ("finalize"); merged into production. The production vmprog repacks the previewed diag bitstreams. Open, unanswered: a plain global cell/row count back? restore S7 Straight, S9 Even or the second row clock? |
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
| [bunting](bunting/SUMMARY.md) | v0.6.0 | 88 % (room) | 5 | router1 | 6/6 seed 1; HD Analog s5, HD HDMI s7, HD Dual s2 hand-installed | oscillator/field |
| [cathedral](cathedral/SUMMARY.md) | v0.6.0 | 91 % (room) | 20 | router2 | SD Analog seed 3; HD HDMI s61 + HD Analog s3 hand-installed | oscillator/field |
| [cathode](cathode/SUMMARY.md) | v0.5.0 | 92 % (tight) | 2 | router2 | 6/6 seed 1 | oscillator/field |
| [chartreuse](chartreuse/SUMMARY.md) | v0.5.2 | 91 % (room) | 23 | router1 | 6/6 seed 1 | measured field |
| [cocoon](cocoon/SUMMARY.md) | v0.6.0 | 96 % (**FULL**) | 21 | router2 | HD HDMI s11 + HD Analog s8 + HD Dual s2 hand-installed; SD Analog seed 2 | oscillator/field |
| [crenel](crenel/SUMMARY.md) | v0.6.0 | 94 % (tight) | 23 | router1 + router2 | HD HDMI r2 s17, HD Analog r1 s3, HD Dual r1 s7 hand-installed | oscillator/field |
| [crossbeam](crossbeam/SUMMARY.md) | v0.6.0 | 91 % (room) | 23 | router1 | 6/6 seed 1 | measured field |
| [curtain](curtain/SUMMARY.md) | v0.6.1 | 96 % (**FULL**) | 27 | router2 | HD Analog s5, HD Dual s6; HD HDMI s11 hand-installed | oscillator/field |
| [cusp](cusp/SUMMARY.md) | v0.6.0 | 97 % (**FULL**) | 11 | router2 | hand-installed HD HDMI s138, HD Analog s48, HD Dual s11; SD seed 1 | oscillator/field |
| [cutout](cutout/SUMMARY.md) | v0.6.0 | 98 % (**FULL**) | 27 | router1 | builder 6/6 seed 1; HD HDMI s17, HD Analog s7, HD Dual s7 hand-installed (pin-skew screen) | collage |
| [escalator](escalator/SUMMARY.md) | v0.6.0 | 94 % (tight) | 8 | router2 | HD HDMI s45, HD Analog s19, HD Dual s10 hand-installed; SD Dual seed 2 | oscillator/field |
| [facet](facet/SUMMARY.md) | v0.6.2 | 97 % (**FULL**) | 26 | router1 | HD Analog seed 2 | oscillator/field |
| [fern](fern/SUMMARY.md) | v0.5.0 | 93 % (tight) | 24 | router1 | HD HDMI seed 2 | measured field |
| [goblet](goblet/SUMMARY.md) | v0.6.0 | 94.7 % (tight) | 25 | router2 | HD HDMI seed 3, HD Dual seed 2 | measured field |
| [greenroom](greenroom/SUMMARY.md) | v0.5.0 | 95 % (**FULL**) | 25 | router1 | 6/6 seed 1 | collage |
| [halo](halo/SUMMARY.md) | v0.6.0 | 92 % (tight) | 23 | router2 | HD HDMI seed 3 | oscillator/field |
| [idol](idol/SUMMARY.md) | v0.6.1 | 94 % | 27 | router2 | 6/6 seed 1 (HD HDMI 74.79, HD Analog 75.13 thin) | oscillator/field |
| [jukebox](jukebox/SUMMARY.md) | v0.5.0 | 84 % (room) | 18 | router2 | HD Dual seed 4 | oscillator/field |
| [kiln](kiln/SUMMARY.md) | v0.5.0 | 93 % (tight) | 24 | router2 | HD Analog seed 3, HD Dual seed 2 | oscillator/field |
| [lava](lava/SUMMARY.md) | v0.5.0 | 87 % (room) | 21 | router2 | HD Analog seed 2 | measured field |
| [lotus](lotus/SUMMARY.md) | v0.5.0 | 93 % (tight) | 22 | router1 | HD Dual seed 2 | oscillator/field |
| [lozenge](lozenge/SUMMARY.md) | v0.5.0 | 84 % (room) | 15 | router1 | 6/6 seed 1 | oscillator/field |
| [magma](magma/SUMMARY.md) | v0.5.0 | 92 % (tight) | 22 | router1 | 6/6 seed 1 | oscillator/field |
| [mandala](mandala/SUMMARY.md) | v0.6.0 | 96 % (**FULL**) | 27 | router2 | HD Analog seed 2 | measured field |
| [marmalade](marmalade/SUMMARY.md) | v0.6.0 | 89 % (room) | 10 | router1 | 6/6 seed 1 | oscillator/field |
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
