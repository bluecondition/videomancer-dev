# Heirloom rebuild: method and acceptance (2026-10-21)

## Why this exists

Ron flashed the v0.1/v0.2 heirloom programs and wrote:

> "most of these are disappointingly bad. They just seem very low effort, most have no apparent
> relationship to the picture, at least not a strong enough one, or they are very abstract. When I did
> this exercise before, the few programs that I made almost looked like EXACT copies of the still
> images, but with logical moving parts. This just looks like basic shapes put together. [...] Please
> work into all of them with more detail towards making it look like the actual picture. examples that
> we did before are wraith, isobar, marrow, pewter, loupe. these all had details that looked exactly
> like the still and actually mapped controls to each element correctly."

The v0.1 programs went from a one-line sketch straight to VHDL. The good programs
(`programs/ron/{wraith,isobar,marrow,pewter,loupe}`) went: **measure the reference numerically →
float prototype rendered next to the reference, iterated until it is nearly a replica → integer
model → RTL**. Every heirloom program is now rebuilt that way. Read at least `loupe/SUMMARY.md`
and `wraith/SUMMARY.md` before starting, and look at `~/loupe_work/fit.py`, `~/loupe_work/proto.py`
and `~/loupe_work/mcmp.png` (top = reference, bottom = the proto): that comparison is the bar.

## The bar

At the default knob settings, a frame from the program must be recognisable as the photo at a glance.
Someone holding the photo next to the screen sees the same picture:
- the same composition, with every element in the same place, at the same size and with the same
  repeat count;
- the same palette (hue, saturation, relative brightness, including the thin fringe colours between
  bands);
- the same edge character (hard or soft edges, outlines, glows, highlights);
- the same textures (line tearing, interlace comb, mosaic blocks, smear, dither, noise).

Then it moves the way the original analog patch would have moved. "Basic shapes put together" fails.
So does a generic stand-in such as "a ring field" for what is really a specific nested
rounded-diamond with an orange core and a 3-px yellow fringe.

## Inputs and tools

- Photo: `programs/ron/heirloom/photos/<file>` (1920×1080 captures of Ron's old modular video synth;
  four are paper collages: duci, selfie, television, tezcatlipoca).
- Work dir: `.heirloom_work/rb_<name>/`. Never use /tmp: this WSL wipes it mid-run.
- Python: **`.heirloom_work/tools/py`** is the only interpreter with numpy and PIL. scipy is not
  installed, so write FFT, autocorrelation, k-means and fits in numpy.
- `.heirloom_work/tools/palfit.py IMG K OUTPREFIX [x0 y0 x1 y1]`: k-means palette with pixel
  shares, hardware 10-bit YUV and a label image.
- `.heirloom_work/tools/hwcolor.py`: `rgb_to_hw10(rgb)` gives (Y, U, V) **in the hardware register
  convention (U register = Cr, V register = Cb)**, `hw10_to_rgb` goes back, and `swap_uv` converts
  between the two. Store generated colours exactly as `rgb_to_hw10` returns them.
- `.heirloom_work/tools/cmp.py OUT.png "label=img" "label=img" ...`: labelled vertical stack.
- `.heirloom_work/tools/sim.sh <name> OUT.png [--video IMG] [run.py options]`: GHDL image sim.
  Writes `OUT.png` in **exact hardware hues**, converted straight from the tester's raw 10-bit
  `output.txt` (u = Cr, v = Cb) by `tools/raw2hw.py`, and `OUT_raw.png` (the tester's own render, which
  shows generated colours U/V-swapped and clips near-gamut colours before any swap — never judge hue from
  it). `--video IMG` feeds IMG as the source, pre-swapped, so video
  and generated colours both preview as on hardware. Example:
  `tools/sim.sh escalator rb_escalator/rtl.png --set toggle_switch_11=1 --decimation 2 --warmup-frames 3`.
  If it fails with "obsoleted by package", delete `.heirloom_work/simbuild/<name>` and rerun.
- `.heirloom_work/check.sh <name>`: GHDL analysis, elaboration and synth front-end in about 10 s.
  It catches width mismatches that otherwise only fail the 4-minute build.
- Build: `MAKE_TIMEOUT=900 ./build_heirloom.sh <name> > .heirloom_work/rb_<name>/build.log 2>&1`
  (from the repo root). For router1: prefix `NEXTPNR_ROUTER="--router router1"`.

## Workflow (every step is mandatory)

### 1. Look

View the whole photo, then at least 6 crops at 2–4× zoom: centre, corners, every distinct element,
and every boundary between elements. Write `notes.md` in the work dir as you go. List every element
you can see (shapes, bands, fringes, textures, glitches) with its position in pixels. For each one,
decide which analog mechanism produced it. The candidates:
- ramp, triangle or sine oscillators, and their sums;
- |x| folds, products (multipliers), wavefolders, hard sync;
- distance fields, keyers and colourizers (a 1-D palette over a scalar field);
- sample-and-hold or pixel-clock mosaics, slew or low-pass smears;
- sync instability (line tearing, V-hold slips), interlace combs;
- camera video processed by any of these.

The mechanism gives you the field function, the natural motion and the natural controls.

### 2. Measure (numbers, not adjectives)

- **Palette.** Run `palfit.py` on the whole frame and on crops (k = 8–16). Paint the label image and
  view it. Find the ordered sequence of colours across each band system, including thin fringes, and
  measure their widths.
- **Geometry.**
  - Make masks per colour or label.
  - Take profiles along chosen rows and columns.
  - Find periods by autocorrelation or FFT, and mirror axes by flip-correlation.
  - Locate contour positions.
  - **Fit the field function**: posit the mechanism, then grid-search its parameters. Score by
    palette-label purity per field bin (loupe `fit.py`), or by least squares on measured contour
    positions. Record the fitted constants.
- **Textures.**
  - Per-line horizontal offsets: cross-correlate each row with the row above, giving the tear
    distribution, run lengths and burst rate.
  - Mosaic block sizes, smear length (autocorrelation along a row), comb amplitude, noise level.

### 3. Float prototype, iterated to a near-replica

`proto.py` renders 1920×1080 from the fitted model. Run `cmp.py cmp_protoN.png photo=... protoN=...`,
**view it**, write down every visible difference, fix them, and repeat. Expect 5–10 rounds. Stop only
when the remaining differences are ones you chose: a different noise realisation, or a detail the
hardware cannot afford, which you must note. Keep `proto_final.png` and `cmp_proto.png`.

### 4. Controls, designed on the proto

- **Every control is named after an element of the photo and changes that element**: its count,
  size, position, shape, colour or texture. Ron should be able to point at a part of the photo and
  find its knob. Use all 12: K1–K6 rotaries, S7–S11 switches, P12 slider.
- **Defaults reproduce the photo.**
- **P12 is the headline**: the most prominent element or motion, with a designed 0 % and a designed
  100 % climax scene (not math degenerating). Glide P12 (1/8 of the gap per field).
- **Logical moving parts.** At defaults the picture moves the way the analog patch would: an
  oscillator drifts, bands scroll, a tear rolls, a key breathes. Keep it moderate and readable. Use a
  Speed-type control, bipolar with a deadband if direction matters. No switch may freeze all motion.
- **Switches are instant look-flips** tied to photo elements: a mirror, a second copy, a palette
  alternative, tear on/off, outline on/off and so on.
- **Video (Ron's rule): incoming video always sits on a switch, `Source = Video / Osc`**, so the
  program can be fully generative. He wants video used at every chance, so the default is **Video**.
  - Where the photo shows camera content (a mosaic of a scene, a posterized picture, collage
    material), Video mode puts the camera there through exactly the processing the photo shows. Osc
    mode substitutes a generator whose statistics match the photo's content (block sizes, level
    distribution, colour).
  - Where the photo is fully synthetic, Video mode feeds the camera into the patch the way an analog
    artist would: luma added to the field (contours bend around the picture), keying a layer,
    modulating an oscillator's phase or a band's width, or tinting the palette. At default knobs the
    photo's composition must still read clearly. The video is woven in; it does not replace the
    picture.
  - Every video-path control stays live in Osc mode (the oscillator feeds the same path).
- Render a **control sheet** from the proto (`sheet.png`: each control at min / default / max, plus
  P12 at 0 / 50 / 100 %) and view it. Every control must visibly change the picture at any P12
  position.
- Knobs whose zero would kill a feature get a floor. Bipolar knobs get a ±8 deadband. Glide any knob
  that shows ADC noise.

### 5. Integer model

`imodel.py` reproduces the RTL arithmetic: widths, ROM sizes and contents, DDA increments, the
per-line engine's quantised values and the palette indexing. Compare it to the proto. Precision bugs
show up here as streaks or stair edges; wraith needed a 1024×16 sine. You can skip this only if the
datapath has no multiplies, accumulators or ROM-indexed fields, and you must say so in `notes.md`.

### 6. RTL

- Keep the program name, directory and `program_id`. Set `version = "0.5.0"`. Keep
  `program_type = "processing"`. `description` ≤ 127 bytes; the packer fails on longer ones only
  AFTER all 6 bitstreams have built. Preset names ≤ 15 characters, categories from the fixed list,
  labelled params use `initial_value_label`.
- **Shell.** Reuse the heirloom v0.2.0 shell from `escalator/escalator.vhd`, `cocoon/cocoon.vhd` or
  `starburst/starburst.vhd`. That shell, flashed, works on hardware. It includes:
  - timing-ID virtual-1080 steps (`f_vstep_x/y`, round shapes at SD, 720p and 1080p);
  - a vblank zone (no avid for 1024 clocks) with per-field latches;
  - source-width measurement;
  - a generated avid (`p_gavid` + `g_start`, with the beam counters initialised on the `g_start`
    edge);
  - a line buffer for the video (1-line delay, read-before-write);
  - the Source oscillator block (`s_ox/s_oy/s_ot`, `f_tri8`) muxed at the line-buffer write;
  - output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-box sums, luma [1 2 1], luma scaled
    to ~777. This is required against the HD HDMI bars, and it means drawn whites come out ≈ 777;
  - a blanking gate on the latency-aligned generated avid;
  - `C_LAT` a multiple of 4, equal to `processing_delay_clks`, with the syncs delayed by `C_LAT`.

  The look itself must be rebuilt from the proto; do not keep the v0.1 look.
- **Engine patterns** from the reference programs:
  - a sequencer with ONE serial shift-add multiplier, registering op code and one-hot operand
    selects from the state number (wraith's fix);
  - a serial divider;
  - a per-line engine kicked at the line start that computes the NEXT line (marrow/loupe);
  - a 2nd-order DDA for squares (`A += D; D += 2k²`) and square root via normalize → mantissa ROM
    (loupe);
  - palettes as ROMs generated by a `<name>.py` build hook into `<name>_pkg.vhd`, with Y/U/V stored
    in hardware convention (wraith.py, loupe.py);
  - field latches for anything that feeds a multi-line structure.
- **Budget**: ≤ ~7,100 LC (92 %) and ≤ 24 EBR; the reference programs ran at 6–7.2 k LC. HD timing
  must close at the full 74.25 MHz; never touch `hd_clock_divisor`.
- **Rules.** Obey `CLAUDE.md` (repo root; read all of it). The traps that bite most often:
  - `to_unsigned(N, W)` wraps;
  - `shift_left` keeps the operand width;
  - narrowing `resize(signed)` folds the value;
  - `&` returns an ascending range;
  - no slice of a function result;
  - `unsigned & literal` is ambiguous inside `signed(...)`;
  - chroma code 0 is GREEN;
  - cap drawn luma at ~768;
  - per-line arithmetic is STA-timed at the pixel clock, so one multiply per stage;
  - EBR outputs get a second fabric register;
  - one write statement per inferred RAM;
  - vsync serrates (use edges plus a saw-active guard);
  - one latency for all modes;
  - no self-adds (`x + x`) under router1.
- Run `.heirloom_work/check.sh <name>` until it is clean, then build.

### 7. Verify the RTL render (required for this rebuild)

Render-correctness is the deliverable, and the first version already failed on hardware, so sims
are justified here (CLAUDE.md's sim rule).
- Run `tools/sim.sh <name> rb_<name>/rtl_osc.png --set <Source switch>=<Osc> --decimation 2
  --warmup-frames 3` (960×540) at default knobs. If a vblank engine needs more than the tester's fake
  8-line vblank, use `--video-mode 720p60 --decimation 1` instead.
- Build `cmp_final.png` with `cmp.py`: photo | proto_final | RTL (Osc) | RTL (Video, fed
  `--video photos/selfie.jpg` or another photo).
- The RTL must match the proto in structure, palette and texture. Debug the RTL until it does.
  Never declare done on a mismatch.
- Check motion with a second capture a few frames later, or `--capture-frames 2`. It must move, and
  move the right way.

### 8. Build

All six configs must build. Every HD config must PASS 74.25 MHz on the LAST post-route
"Max frequency" line. Treat the builder's final-seed "✓" as suspect. If router2 misses everywhere,
rebuild with router1 before restructuring. Record any seed other than 1. **Never** run
`build_programs.sh` with no args, never `rm -rf out/`, and never use a global `pkill` (use
`pgrep -f "[n]ame"` patterns, and kill only your own PIDs).

### 9. Deliver

- `programs/ron/heirloom/<name>/`: `<name>.vhd`, `<name>.toml`, the optional `<name>.py` and
  `<name>_pkg.vhd`. Delete files of the old version that are no longer used. The shipped sources must
  be exactly the ones the `.vmprog` was built from: do any cleanup BEFORE the final build, never after.
- `SUMMARY.md` (rewritten): subagents can't write `.md` files, so put its full text at the END of
  your final message in one fenced block; the orchestrator writes it. It follows the shape of
  wraith/loupe:
  - Reading of the photo, with the MEASURED numbers;
  - What it does;
  - a Controls table where every row says which photo element the control drives;
  - Video (how the camera enters, and what Osc substitutes);
  - Engine;
  - Status: v0.5.0 built, date, seeds, HD/SD Fmax, LC, EBR, not flashed;
  - Deviations from the photo, and why.
- Do NOT edit README.md, do NOT commit (the orchestrator does both), and do not touch other programs.
- Final message to the orchestrator:
  - the path of `cmp_final.png`;
  - a one-paragraph reading of the photo;
  - the control → element map;
  - the build numbers;
  - the known deviations from the photo.
