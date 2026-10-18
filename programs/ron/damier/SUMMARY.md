# DAMIER (v0.4.8 — op-art checkerboard engine)

**STATUS: FINAL except presets (signed off by Ron on hardware, 2026-10-11:
"OK I think we can mark damier as completed in the report").** Marked
Completed in the Program Shelf report. Remaining: capture real presets with
`./capture_preset.sh`. The HD seeds are hand-installed, so for a
preset-only TOML change, regenerate `program_config.bin` and repack; never
run a full rebuild.

**v0.4.8 (2026-10-17): resolution scaling, not yet flashed.** The RTL
changed, so the v0.4.5 HD picks are void; the new HD placements were
pin-skew screened again and hand-installed (see Build). Needs a hardware
look at 1080 (should be unchanged), 720p and SD.

DAMIER (French for checkerboard; Vasarely's word for his grids) is a
synthesis program loosely descended from GILT's Kiss robe. A clean
checkerboard is redrawn square by square from a vocabulary of shapes, folded
by up to three mirrors, bent by travelling waves, and filled with concentric
rings in each square's own shape that tighten until they alias into moiré.

## Controls

| Control | What it does |
|---|---|
| K1 Layout | 16 arrangements: Classic, Vega, Albers, Harlequin, Pinwheel, Chevron, Truchet, Weave, Bullseye, Fractal, Quartered, Mosaic, Rings, Zebra, Robe, Shimmer |
| K2 Scale | square size 8 to 256 px at 1080 (the same share of the screen height at 720p and SD), exponential |
| K3 Wave | wave amplitude; a second wave crosses in past 50% |
| K4 Angle | lattice rotation 0 to 360°; the waves ride the lattice axes |
| K5 Speed | everything that moves: waves, drift, spin, size pulses (square law) |
| K6 Palette | 16 schemes: Op, Riley, Vega, Kiss, Bauhaus, Memphis, Neon, Pastel, Seventies, Mondrian, Delft, Ruby, Lagoon, Sunset, Acid, Prism |
| S7 Mirror, S8 Flip, S9 Kaleido | fold switches (bilateral, quad, 8-fold); with any fold on, the lattice spins |
| S10 Shape | wave shape: Sine or Zigzag (chevrons) |
| S11 Motion | Bend (displace across the travel) or Squeeze (along it) |
| P12 Resonance | 0 = untouched; ring boundaries appear at the cell corners and multiply inward (each square a target in its layout's shape: square rings on the plain checker, circles on Vega, arcs on Truchet …); the rings stand still and change only with the slider; their period tightens to ~17 px at 100% (v0.4.3 range: the old lower half). The period is set in pixels, so the look holds at any Scale |

## Architecture (see the header of damier.vhd)

1. **Folded coordinates and lattice accumulators.** Each pixel step adds one
   of {a, −a, b, −b}. Line seeds come from a serial multiplier that runs
   during the previous line.
2. **Waves (every 4 pixels).** Two wave tables are read and interpolated
   per 4-pixel block. The result is interpolated across the block while the
   coordinate waits in an EBR ring.
3. **Layout and shape engine.** Per cell: shape type, solid or concentric,
   size, orientation and colour zone. The palette ROM is indexed by
   {scheme, zone, parity, ink}.
4. **Resonance (stages 33–35).** The shape metric m is multiplied by the
   ring frequency F16 (cycles per half cell × 16, split 6+5-bit operand).
   The ring bit is bit 7 of (m·F16/16 − Q), XORed into the ink. The
   microcode sets F16 = cell_px·(2^(s/128) − 1)/32 (a pixel-fixed period;
   cell_px = 65536/zoom by Newton), floored by a per-cell ramp so small
   squares respond from ~6%. The ring phase Q is fixed at 0 (v0.4.2): the
   rings move only with the slider.
5. **Microcoded engine** (cubist's design at 16 bits). It runs the
   per-field sequencer (glides, time, zoom, rotation, drift, the ring
   frequency and phase) and rebuilds the double-buffered wave tables. Source:
   `damier_ucode.py`, assembled by `damier_uasm.py`, emulated by
   `damier_uemu.py`; the `damier.py` build hook emits `damier_pkg.vhd`.
   It needs ~21k clocks per field.

Hardware contracts: generated avid (parity-locked); one latency in all
modes; outputs gated to neutral outside avid; colours stored U/V-swapped;
per-field work on the first vsync edge after active video; interlace bottom
field starts one frame line down.

## Build

**v0.4.8 (2026-10-17).** `./build_programs.sh ron damier` (router2, base
seed 1): 6/6 closed, HD HDMI on seed 2 (seed 1 stalled the router). The
three HD configs were then re-screened with `tools/hdmi_pin_skew.py`
(router2 seeds, json re-synthesised into a scratch BUILD_ROOT, which
reproduces the builder's bins byte-exact), the picks icepacked into
`build/programs/ron/damier/rev_b/bitstreams/` and repacked with
`vmprog_pack.py` (all 6 iceunpack OK). Builder bins:
`.scaling_work/damier_tmp/v048_builder/`, installed set:
`.scaling_work/damier_tmp/v048_installed/`.

| Config | Router / seed | Fmax (MHz) | HDMI hs-clk / data (ns) |
|---|---|---|---|
| HD HDMI | router2 **s33** (of 36) | 88.27 | +1.19 / −3.89..−0.78 |
| HD Analog | router2 **s3** (of 12) | 86.59 | +1.05 / −2.17..−0.78 |
| HD Dual | router2 s1 (builder) | 85.51 | −2.13 / −2.49..−0.85 |
| SD Analog | router2 s1 | 90.05 | |
| SD HDMI | router2 s1 | 90.05 | |
| SD Dual | router2 s1 | 91.55 | |

~5,905–5,933 LC (77%), 15/32 EBR (+~100 LC over v0.4.5). The builder's
HD HDMI s2 had HSYNC at +0.22 ns (danger zone), so do not ship builder HD
bins for this program.

Earlier builds:

v0.4.1 (2026-10-11; identical placement to v0.4.0, since only the microcode ROM contents changed): `./build_programs.sh ron damier` (6/6 on seed 1),
then all three HD configs re-picked by pin-skew screening
(`tools/hdmi_pin_skew.py`). HD HDMI took 13 router2 seeds and 6 router1
seeds. The picks were installed and repacked with `vmprog_pack.py`
(6 bitstreams, validated, all iceunpack OK).

| Config | Router / seed | Fmax (MHz) | HDMI hs-clk / data (ns) |
|---|---|---|---|
| HD Analog | router2 **seed 5** | 86.25 | +1.14 / −4.07..−1.65 |
| HD HDMI | router2 **seed 25** | 84.52 | +1.92 / −2.95..−0.91 |
| HD Dual | router2 **seed 6** | 92.78 | −2.44 / −2.80..−1.30 |
| SD Analog | router2 seed 1 | 84.39 | |
| SD HDMI | router2 seed 1 | 82.24 | |
| SD Dual | router2 seed 1 | 75.39 | |

~5,650 LC (74%), 15/32 EBR.

The builder's HD HDMI seed 1 had HSYNC at +0.29 ns, inside the danger zone,
and its HD Analog seed 1 straddled. Those bins are kept in
`.damier_work/v040_builder/`.

**HD HDMI, v0.4.1b (2026-10-11).** The v0.4.0/v0.4.1 pick, router2 seed 6
(HSYNC +1.78, top data bit −0.20), showed "lines over everything" and
glitching on hardware. Re-screened 26 more placements requiring HSYNC +1.2
to +2.0 and every data bit ≤ −0.6, and picked router2 seed 25 (+1.92 /
−0.91). The backup, router1 seed 8 (+1.07 / −0.68, 95.5 MHz), is packed in
`.damier_work/v041b_alt/damier.vmprog`. On
this netlist router1 times faster (85–97 MHz) but landed HSYNC within
0.6 ns on all 6 seeds. A rebuild does not reproduce the picks: re-route
the solo seeds and repack.

## Status

v0.4.5: output conditioning always on. A [1 2 1]/4 horizontal smoothing on
Y/U/V, plus luma 64 + 13/16 (y − 64) (white about 775), sits before the
blanking gate (+2 stages over v0.4.3). S10 is Zigzag again. ~5,810 LC, all 6
on seed 1 in the build; HD placements pin-skew screened and hand-installed:

| Config | Seed | Fmax (MHz) | HDMI hs-clk / data (ns) |
|---|---|---|---|
| HD HDMI | router2 **s3** | 89.45 | +1.14 / −3.48..−1.26 |
| HD Analog | router2 **s4** | 88.58 | +0.80 / −3.86..−1.19 |
| HD Dual | router2 s1 (builder) | 82.6 | −2.50 / −2.86..−1.38 |

HD HDMI s3 was the best of 17 seeds, matching v0.2.3's HW-clean +1.16 with
twice the data margin. The builder bins are in `.damier_work/v045_builder/`.
A rebuild does not reproduce the picks: re-route the solo seeds and repack.

## History

- **v0.4.8** (2026-10-17). Resolution scaling (the virtual 1080-line
  canvas of `.scaling_work/RECIPE.md`).
  - **Was:** everything was set in pixels (squares 8–256 px, the ~17 px
    ring period, the waves ride the lattice), so 720p drew 1.5× bigger and
    SD 2.2× bigger against the screen height.
  - **Fix:** the microcode scales the lattice step per pixel by the
    pixel's width in virtual units (s ×1.5 at 720p, ×2 at SD; the ring
    frequency is per cell, so the ring period follows). Y runs in the same
    pixel-width units, Q.8: 1 per line at 1080p and 720p, 2.22 per field
    line at 480i, 1.875 at 576i; the engine computes the field's top Y as
    −H2·R9 (new CTL R9, port Y0). The line seeder's target is Q.4 into a
    16-bit serial multiplier (18 clocks a product). The pixel path's
    diagonal (Kaleido) threshold is the rounded Y, which is exact at 1080
    and 720p and within half a pixel at SD. Interlace now comes from the
    video timing ID (480i, 576i, 1080i), not the sync-phase detector.
  - **Result:** 1080 is bit-identical: full-res 1080p sims of v0.4.7 and
    v0.4.8 differ in 0 pixels at warmup 5 (steady state), and at warmup 3
    with the old interlace source; 720p is the 1080
    picture scaled by 2/3; SD is the centre 1440-unit 4:3 window at the
    same scale, round things round.
  - **Sim note:** the old `is_interlaced` source (the SDK sync-phase field
    detector) reports interlaced for one field after start-up. In a
    4-frame sim that field is the captured one, so the old baselines at
    1080p/720p render 2:1 squashed (the "hexagonal" kaleidoscope). With
    warmup 6 the old code renders square; hardware steady state is
    unaffected. The knob glides also need ~12 fields to settle, so compare
    modes with a glide-snapped sim copy.

- **v0.4.5** (2026-10-11). Ron: "The bars stop with guard." The guard is
  now permanent (the on/off mux removed) and S10 Zigzag is restored. The
  bar fix is recorded for every program in CLAUDE.md (hardware contracts),
  TECHNIQUES.md and memory avid-dropout-uv-bars.

- **v0.4.4-diag** (2026-10-11). Bar-guard A/B on S10. The prior evidence
  (memory avid-dropout-uv-bars) is that the bars need sharp, high-contrast
  edges in motion. DAMIER also exceeded the ~768 drawn-luma cap (palettes
  reach 874–906). There are no extreme codes (Y 95–906, U 158–855,
  V 155–764). Placement screening needed 28 seeds to find s24 (the 28-way
  parallel batch was killed for memory; batches of 12 ran fine).

- **v0.4.3** (2026-10-11). Ron: "I think it works well now". His last
  change: the old 0–50% of the slider becomes the full 0–100%, because
  further up the rings got too fine. The ring maths now uses half the
  glided slider (the deadband and glide still run on the full input), so
  100% now gives a ~17 px ring period. Microcode only; the same screened HD
  seeds were re-installed.

- **v0.4.2** (2026-10-11). Ron: "why are the concentric circles always
  moving, i'd like them to be stationary and only move when the slider
  moves". The outward ring flow (phase Q) is removed; the phase is fixed at
  0. This is microcode only, so placement is identical and the screened HD
  seeds were re-installed (HD HDMI router2 s25, HD Analog s5, HD Dual s6).
  The rings still ride their cells when the board itself moves (waves,
  drift, spin).

- **v0.4.1** (2026-10-10). Ron on v0.4.0: "very glitchy as if the slider is
  jumping in values".
  - **Cause.** The ring position is m·F, so at a square's edge (m = 511) a
    change ΔF moves the rings by 511·ΔF/16 r-units. One count of slider
    noise moved them about 1/8 ring near the top, and one count of Scale
    (through cell_px) about 1/4 ring. The 10.4 glide smoothed the noise
    but still passed ±1-count flicker, and moiré magnifies it.
  - **Fix (microcode only).** Every glided knob (Scale, Wave, Angle, the
    slider) gets a deadband hold: it follows the pot only on 5+ counts,
    the bajaweave lesson. The ring frequency, the zoom mantissa (256 steps
    an octave) and the angle (1/16384 turn) are computed from the glide's
    10.4 value, so a deliberate move changes them smoothly. The flow cap
    went from 40 to 24/256 cycle a field.
  - **Emulator check.** Under ±2-count noise on three knobs, F16 and the
    lattice are bit-constant; a slow sweep changes F16 by ≤1 a field.

- **v0.4.0** (2026-10-10): P12 Resonance replaces Slide, at Ron's request.
  - **RTL:** Slide's 4 stages and the mortar came out. The Resonance
    stages (2 of them) went in between the shape metric and the ink
    decision. The plain checker (and the sub-checker) now carry the square
    metric, so their rings are square; this changes nothing at P12 = 0.
  - **Phase design:** a ring phase Q is a whole-cell flip when the ring
    frequency is near 0. So Q flows only once a cell holds ~2 rings, and
    settles to 0 below that.
  - **Size:** ~5,650 LC (74%), 15 EBR, 6/6 on seed 1.
  - **Presets:** "Cafe Wall" became "Targets" (P12 420).

- **v0.3.0** (2026-10-10): P12 Slide replaces Vertigo.
  - **Removed with the lens:** r² tracking, the lens table and its
    multiplier, the orbit, and the block-float path, ~1,200 LC.
  - **Added:** the Slide stages (8 per-field row offsets selected by row
    class and parity; column ±offsets) and the mortar. About 6,000 LC (78%)
    and 15 EBR; every config closes on seed 1 at 84–90 MHz.
  - **Climax fixed in sim:** the first cut put only row mortar on the
    sliding columns. The grout broke into dashes and same-coloured squares
    merged into blobs. Vertical grout now fades in as the columns move.
  - **Presets** were re-tuned by hand. "Vortex" became "Cafe Wall", and
    "Vega Sphere" became "Vega Dots".

- **v0.2.3** (2026-10-10). Ron on v0.2.2: still there, "it grows in one
  place as I increase vertigo and then jumps to a broader circle".
  - **Diagnosis.** A full-1080p RTL sim at his settings reproduced it: a
    thin band of scrambled pixels at the lens table's END (index 511,
    r = sqrt(511·2^lsh)). It stays put while the lens grows and doubles
    when lsh steps, exactly as he described.
  - **Cause.** Since v0.2.0's serial table read, B5/B6 took the outside
    flag and the interpolation fraction from the NEXT block's sample. So
    entry 511's wrapped i+1 (entry 0, the lens centre) leaked through, and
    every block interpolated with its neighbour's fraction.
  - **Fix.** Capture the fraction and flag with entry i.
  - **Verified.** RTL sims show it gone.
  - **The earlier fixes.** v0.2.1 (shading removed, rounding, interpolated
    sine) and v0.2.2 (profile) were not the reported ring. They stay as
    improvements.

- **v0.2.2** (2026-10-10). Ron on v0.2.1: "the outer circle is still there."
  - **Diagnosed then as the profile (wrong, see v0.2.3).** The old
    (1−u)² bulge did have a 1.5× crush band at 0.7R; reshaping it was
    still worthwhile.
  - **Profile.** The new core profile is φ = (1−u)²/(1+10u). It has a
    heavy tail (max squeeze 1.18×), the twist shears in the core, and
    value and slope are 0 at the rim.
  - **Table.** The table radius is 2.2× the old one; the core sits at about
    the old visible size, and the rim is off-screen at high Vertigo.
  - **Precision.** Lens values went from Q10 to Q12 (14-bit c, d, and a
    13×14 multiplier), because the long tail would have drawn Q10 seams.
    1/(1+10u) is one Newton step per entry, and the engine's worst case
    stays ~184k clocks per field. lsh 14 was added for the bigger table.
  - **Verified.** A GHDL render at Vertigo 300 and 700 shows no ring.

- **v0.2.1** (2026-10-10). Ron on v0.2.0: "a jagged thick circle around
  the entire edge of what is going to get vertigo; it would be better if
  that outer part didn't show." Three fixes:
  - **The lens shading is gone.** It was a 4u(1−u) darkening ring, 6-bit
    and held per 4-px block. It outlined the lens even at low Vertigo, ended
    sharply at the rim and stair-stepped. The warp itself fades out as
    (1−u)², so with no shading the rim is invisible.
  - **Rounded lens products.** Truncating m·cos 0 (32767/32768) had left
    c = −1 inside the lens: a ~1 px warp step at the rim.
  - **Interpolated sine in microcode.** The bare 512-entry quarter-sine ROM
    steps 1/2048 turn. That staircased the twist by 2–3 px at large radii
    and made the lattice spin and the lens orbit jitter. It is now used for
    the twist, rotation and orbit; the ripple and wave tables keep the bare
    ROM.

  The engine worst case is now ~183k clocks per field (NTSC 225k). The
  shade ring's EBR and pipeline are freed.

- **v0.1** (design + microcode): never fitted. It came in at 9,900 LC, then
  8,674 (112%) at v0.1.3.
- **v0.2.0** (2026-10-09): fit pass from ~8,150 estimated to 7,138 LC.
  - `no_rw_check` on all 9 RAMs: −380 LC. This is a general lesson,
    recorded in CLAUDE.md.
  - Drift and wave phases moved into microcode.
  - The geometry knobs are read straight off the registers.
  - Incremental block interpolation.
  - The wave sample is now taken per pixel from the final coordinate, so it
    is lensed too.
  - The walker was replaced by an exact serial line seeder.
  - r² is tracked with 2 trackers and a registered increment.
  - The lens multiply uses 4 products instead of Gauss's 3.
  - The lens table is read twice per block, with no even/odd split.

  Timing went from 55 to 62.8 MHz on seed 1, and up to 77.5 MHz on other
  seeds:
  - the step decision is registered one pixel ahead;
  - g_go is registered and duplicated;
  - the zoom octave has one copy per consumer;
  - MRD shifts in two registered steps.
