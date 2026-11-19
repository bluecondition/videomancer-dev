# Plume — Heirloom #19

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/plume.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-03, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Plume Diag`, `out/rev_b/ron/_diag/plume_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/plume/`) is untouched.
v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed. Source photo:
`photos/WIN_20230315_00_58_54_Pro.jpg` (1920×1080 capture, 2023-03-15). Replaces v0.1.0 (a mirrored column of
chevron ripples), which Ron judged too abstract.

## Phase 2 (v0.6.0)
No notes from Ron for plume (free hand, 2026-11-03 batch). The v0.5 controls were all live, but Seams, Stripes and
Reach spanned only about ×0.5..×2, K6 Tear and S8 Wedge barely showed, and S9 Ribbons Filled was subtle. The round:
- widens the oscillator and shape knobs;
- gives the chevron's **knee** (the apex row of the red fall) its own parameter, on the headline P12 with the
  hooks, so the slider changes the plume's *shape* and not only its texture;
- re-patches two subtle switches: the seam saw into the ground stripes (S8 Hooks), the camera into the ground
  (S9 Ground);
- turns K6 into a whole-picture "unsteadiness" control: the stripes' wiggle and the line tear together.

Every new effect is exactly zero at the default knob value; the default frame is byte-identical to v0.5.1
(checks below). Phase-2 working files: `.heirloom_work/rb_plume/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| P12 Wake: hooks 0 .. ×1 (50 %) .. ×3; fall teeth with it | **P12 Wake**: hooks 0 .. ×1 .. **×4**, teeth to ±217 px, **plus the knee**: +320 px (blunt) at 0 % .. 0 at 50 % .. −320 px (swept back) at 100 % | 0 % = straight stripes through a blunt, shallow-V plume (knee rows at x ≈ 970 instead of 650). 100 % = 4× hooks, big teeth, the knee pulled behind the red rise: the plume pinches shut around each knee row and splits into an inner arrow and the outer bands. ±17 % of the width |
| K1 Seams: ×0.5 .. ×2 | **×0.25 .. ×4**, exponential (an octave per quarter turn), photo at 50 % (exact) | inner seam period 108 px → 432 px (a few broad hooks) .. 27 px (fine comb; SD aliases near the top) |
| K2 Reach: ×0.375 .. ×1.62 | **×0.125 .. ×1 .. ×1.87**, photo at 50 % (exact); readout now 0–100 % | the arrow tip at the axis from x ≈ 620 (a thin red sliver on the rise) through the photo's 1436 to off-screen (from ×1.53 the plume fills the right side) |
| K3 Stripes: ×0.5 .. ×2 | **×0.25 .. ×4**, exponential, photo at 50 % (exact) | axis stripe period 36 lines → 144 (broad bands) .. 9 (hairlines), anchored at the fold |
| K4 Flow: bipolar square law, top 7906 Q16/field | same curve up to fm 256 (default 450/field, **bit-exact**), steeper above: top **×2** (15672) | ≈ 1700 px/s at the ends (about a screen width a second); the default ~45 px/s drift is unchanged |
| K6 Bend: video (or Osc) bends the seams (±2 cycles) and stripes (±4) | same, **plus in Video mode the camera pushes the whole plume** (rise, fall, window, wedge) | ±47 px at the default, ±246 px at 100 %; mid-grey = no push; Osc keeps the outline still |
| K5 Tear: line tear in the measured top/bottom bands | **K6 Wobble** (renamed): the stripes' x wiggle **×0 .. ×1 .. ×2.9**; the tear **×0 .. ×1 .. ×9**, its zone floor rising to the whole frame above the default | 0 % = dead-straight stripes, no tear. 100 % = the stripes swing about ±1 cycle and every line of the picture tears up to ±354 px (triangle-distributed, typically ±120 px) |
| S8 Wedge: pink wedge On / Off (subtle) | **S8 Hooks: Plume / All** (renamed): All patches the seam-sheared stripes into the ground | the ground (the left quarter and the right band, about half the frame) takes the plume's hooks, its blue corner stripes with it. The wedge is always on (the photo). Needs some Wake: at P12 0 % only the outer band changes |
| S9 Ribbons: Dark / Filled (subtle) | **S9 Ground: Stripes / Camera** (renamed): the camera luma replaces the ground's green level | the camera in green over the blue gradient, seen through the stripes (Stripes / Wobble / Hooks stay live); the black ribbons become camera windows. Osc mode: the Osc plaid |

Unchanged: S7 Twin, S10 Palette, S11 Source (Video / Osc, default Video).

**Presets** (preset 0 = the defaults): **Photo** (renamed from "Plume"); **Calm Water** (P12 0, slow); **Split Wake**
(P12 100 %, faster, Wobble 68 %); **Twin Plumes** (as before); **Hook Field** (S8 All, more seams, P12 68 %);
**Blinds** (S9 Camera, denser stripes, more Bend); **Blue Storm** (Swap, Wobble near max, fast); **Sliver**
(Reach 0, Seams max, broad stripes). Dropped: Deep Wake (→ Split Wake), Blue Plume, Clean Lines.

**Engine (new).**
- **Knee.** A 19th serial-MAC slot: xF' += κ·tri'(y') / 256, tri' = 5/8 × (256 − |y' − 256|) (0 at the axis and
  the frame edge, 160 at the photo's knee row), κ = 511 − P12 (a bit inversion; +511 .. −512). The slot's
  pre-load is the constant 255 and the product is added to xF' at the store, so κ = −1 at the default adds
  exactly 0. The engine ends at clock ~390, inside SD's 858-clock line.
- **Seams / Stripes curve.** One exponential (one octave per quarter turn, linear within it), evaluated once in
  the serial glide datapath when the K1 or K3 glide writes.
- **Wobble.** The line's measured wiggle amplitude (1..4) × a Q4 gain (16 = the photo, exact) at engine clock
  8 into a 4-bit next-line register; the stage-4 wiggle product is 10 × 5 bits.
- **Tear.** Gain 9 bits (32 = the photo, 32 + (k − 512)/2 above), zone = the measured zone OR a floor
  0..255 from K6 above the default.
- **Shear.** The per-line shear register widened to 7 bits (×4 Wake).
- **Flow.** The speed slot multiplies fm by fm + 2·fm(7:0) when fm ≥ 256.
- **Hooks All.** The ground's cos register takes the plume's sheared-stripe ROM output (one pixel later:
  invisible).
- **Ground Camera.** Level = 128 + source / 2 (the source's 8-bit slice with its MSB flipped: camera luma
  (Y − 512)/2, or the Osc), registered at stage 1 and read at stage 10 (a 9 px skew), in place of the measured
  envelope; the stripe term still subtracts from it.
- **Outline push.** Video only: the bend product / 32 is subtracted from the stage-3 x copy.
- **Delay ring.** The red-key inputs (stage 5 → 9) and the blend weight (7 → 11) ride one 256 × 16 block RAM
  (`no_rw_check`, read = write − 1, plus a fabric register = 4 clocks), replacing 52 FF of copies.
- No pipeline stages added or removed: latency 24 in every mode, U/V parity unchanged.

**Build** (ROUTER1: `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh plume`, 2026-11-03):
HD Analog s1 81.82, SD Analog s2 80.17 (s1 stalled router1), HD HDMI s1 80.17, SD HDMI s1 81.40, HD Dual s1
84.38, SD Dual s1 87.11 MHz. 7,189–7,213 / 7,680 LC (93.9 %), 24 / 32 EBR (+1: the delay ring); no RAM
transparency emulation; iceunpack OK ×6; the packed config carries the new labels and presets. No pin-skew
screen. Router2 on the same netlist: HD HDMI 66.7 / 73.5 / 66.2 / 75.0 MHz (seeds 1–4), so stay on router1.
**Fit history.** The first build (router2) reached 7,498 LC with HD Analog at 60 MHz: the Ground Camera tap
read the stage-0 source register at stage 9, dragging its net across the die (7.9 ns first hop), and the crawl
added two 42-bit pre-load adders to the MAC. Ranked by standalone-yosys ablation and fitted: the wiggle crawl was
cut; the camera level, the curves, the outline push, the tear and the knee were made cheaper; the delay ring was
added (`p2/abl/`).

**Checks** (working files in `.heirloom_work/rb_plume/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py` plus an inline check: every new knob path at RTL widths over all knob extremes and every line: no
  overflow; the defaults are exact (curves 256, knee 0, tear 32, wobble 16, v0.5 speed law below fm 256).
- `spikes.py`: no isolated second-difference spike in the per-line tables the new controls magnify (kx, xL,
  pg, dlt, sg, tz); dlt steps 252 per 2 lines (the ROM's 2-line granularity, negligible at ×4).
- Default frame (GHDL 720p60 dec 1, Osc, 2 warm-up + 1 capture, production v0.5.1 simulated the same way): raw
  10-bit output **byte-identical**, before fitting and again on the final source.
- One extreme frame (pre-fit knee, the same maths): P12 100 % + Hooks All, Osc (`r06_p12max_hall.png`): the
  plume pinches at both knee rows into an arrow and outer bands; hooks fill the ground.
- Not verified: Ground Camera with a real camera, the Video outline push, Wobble at its extremes, K4 top speed.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_plume/` (`notes.md` has every number).
- **Fold.** The frame is mirrored top/bottom about y = 519.5: flip-correlation of the red channel gives a mean
  absolute error of 8.0 there, 13 five lines either side. Below, y' = |y − 519.5|.
- **The red plume.** A red key points right (`meas1.npz`).
  - It rises softly at xL ≈ 400–520, over about 170 px.
  - It falls at a chevron-shaped edge xF: 1447 at the axis, 1269 at y' 40, 1031 at 100, 929 at 150, 786 at 200,
    640 at the apex row y' 260, then back out to 1074 at 390, 1222 at 430 and 1680 at the frame edge.
  - The fall is a soft ramp 125–220 px wide (`rfall.py`), with teeth locked to the seam phase.
  - A second red ramp, the pink wedge, rises about 820 px right of the fall (y' 100–400).
- **Ground (left and right of the plume).**
  - **G:** horizontal stripes, a soft raised cosine with peak ≈ 225 and trough ≈ 50. The period is 36 lines at the
    axis, 44–52 at y' 130, broad 90-line bands at y' 245–300, and 42 from y' 390. They wobble in x as the sum of
    two sines (250 / 410 px, ±0.35 cycle, less in the middle band).
  - **Black ribbons:** G < 10 at y' 145–195 and 340–370.
  - **B:** a vertical gradient with no x dependence (σ 2–8 along a row): 254 at the axis, 18 at y' 160, 128 at
    y' 260, 17 at y' 360, 128 at y' 450.
  - **Corners:** blue stripes in anti-phase with the green.
- **The seams.** Inside the plume a chirped x oscillator cuts everything into vertical seams.
  - Its period is 105–113 px in the inner zone. In the outer zone it runs 150 px → 45–61 px, saturating over
    200–450 px.
  - A per-row phase draws the hooks (`rowphase2.npy`, `chirpfit.npy`).
  - **B** is a function of the seam phase: a narrow pulse (about 25 % duty, 25 → 230) in the ribbon zone, and a
    weak cosine (amplitude 16–60) elsewhere.
  - **G** stripes are sheared by the seam's sawtooth: rise fraction 0.22, power 1.4, depth about 42 lines. This
    makes the yellow/pink chevron ripples.
  - The outer band carries its own fine "<" chevrons (G about 25 lines, B about 28 lines, sheared about 0.5 cycle).
- **Palette.** k-means 14 shares: navy 12.2 %, orange 9.6 %, teal 8.8 %, steel 8.5 %, pink 7.5 %, green 7.4 %,
  lilac 7.2 %, dark green 7.2 %, red 7.0 %, blue 6.5 %.
- **Texture.**
  - Line tearing: row-to-row shift is 0 at y' 60–240, with ±2–7 px bursts at y' 250–290 and beyond 400.
  - Horizontally soft (a box of about 5 px).
- **Mechanism.** A voltage-controlled x ramp oscillator gives the seams. Its sawtooth shears a vertical stripe
  oscillator (G) and a slow vertical ramp (B). Red keys on the distances to xL and xF window the plume. Everything
  is mirror-folded about the screen centre and colourised per channel.

## What it does
1. **Per line** (folded y'): one 256 × 240 line ROM (`plume.py`) holds, per two lines:
   - the quadratic seam phase θ = A0 + A1 s + A2 s², with s = (x − 1000)/1024;
   - A0 for both lines, and the x where the quadratic continues linearly;
   - the B and G channel laws;
   - xL, xF − xL, the ground blue and its stripe gain, the tear zone, the stripe phase, the ribbon envelope terms,
     the wiggle amplitude, the plume stripe offset and shear, and the red level;
   - odd-line deltas for the fast-changing values.
2. **Seam phase.**
   - A second-order forward-difference DDA (Q32), scaled by Seams and offset by Flow.
   - Video (or Osc) luma × Bend is added to the phase.
   - It addresses a raised-cosine pulse ROM (8 widths), a sawtooth/cos ROM and the stripe cos ROM.
3. **Colour.**
   - **B (plume):** mB + aB · pulse.
   - **G (plume):** gO + gK · cos(stripe phase + offset − shear · saw).
   - **Ground:** G is the envelope-gated stripe; B is the gradient minus the anti-phase blue stripes.
   - The plume blends into the ground over the window ramps (x − xL, xF′ + 330 − x, and the wedge).
4. **Red.** min(rise ramp, max(fall ramp with teeth, wedge ramp), red level).
5. **Output.** RGB → 10-bit YUV in the hardware convention (U = Cr, V = Cb), luma pre-compensated 16/13 and
   capped at 940.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Wake** | The chevron hooks: the depth of the stripe shear and the fall teeth. 0 = the stripes run straight through the plume, 50 % = the photo, 100 % = 3× deep hooks. Glided. |
| K1 Seams | The number of seams across the plume, ×0.5..×2 (centre = the photo). Glided. |
| K2 Reach | How far the red chevron reaches right, xF′ = xL + (xF − xL) × 0.375..1.62 (centre = the photo). Glided. |
| K3 Stripes | Green stripe density, ×0.5..×2 (centre = the photo). Glided. |
| K4 Flow | Bipolar, still at the centre ±8. Positive streams the seams right along the plume, square-law speed; the stripes drift at a quarter of it. Default 640 ≈ 45 virtual px/s. |
| K6 Bend | How far the video (or Osc) bends the seams (up to ±2 cycles) and the stripes (±4 cycles). Default 200. Glided. |
| K5 Tear | Line tearing of the top and bottom bands (triangle jitter plus a 3-line hold, gated by the measured tear zone). 0 = clean. |
| S7 Twin | A second plume mirrored from the right edge. |
| S8 Wedge | The pink wedge on the right: On (the photo) / Off. |
| S9 Ribbons | The black ribbons: Dark (the photo) / Filled with stripes. |
| S10 Palette | Photo / Swap (red and blue exchanged: a blue plume over red stripes). |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** the camera luma (2-px average, (Y − 512)/2) × Bend is added to the seam phase and to the stripe
  phase. The seams and stripes bend around the picture while the plume, ribbons and stripes still read at
  default knobs (`cmp_final.png`, fed `photos/selfie.jpg`).
- **Osc:** the sum of an x triangle (about 370 virtual px) and a y triangle (about 270 lines), drifting per field,
  feeds the same path, so Bend stays live.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks):
  - flow and stripe-drift phases;
  - Osc phases;
  - knob curves;
  - glides on K1, K2, K3, K5 and P12 (±4 deadband, 1/8 ease).
- **Per line:** kicked one clock after the line's generated-avid start, computing the NEXT line.
  - Clocks 1–7: y′ (folded at 520) and the line-ROM address; xorshift32 for the tear; the odd-line deltas.
  - Clocks 48–372: 18 slots of one serial multiply-accumulate (P = A × B, with chained accumulation):
    - the tear;
    - Seams × A1/A2;
    - the DDA start phase θ0 and its two differences D1, D2;
    - the plume reach xF′;
    - the stripe phase and offset × Stripes;
    - the shear × Wake;
    - the wiggle-phase accumulator starts;
    - the flow speed.
  - Fits SD's 858-clock line.
  - The line ROM's output register holds the next line's constants; they are copied to the line registers at
    the line start.
- **Pixel path (17 stages, latency 24 in every mode):**
  - the DDA; bend; parabolic wiggle sines from two per-pixel accumulators;
  - the pulse / saw-cos ROMs (+ fabric register);
  - two shared distances (x − xL, xF′ − x) for every ramp;
  - ground stripe phase → cos ROM; sheared plume phase → cos ROM;
  - the channel laws (narrow products); the red key;
  - the plume/ground blend (4-bit weight); palette swap; RGB → YUV.
- **Output:** chroma [1 2 2 2 2 2 2 2 1]/16 (two 8-box sums on an EBR ring), luma box-4 + [1 2 1], luma to
  about 777, blanking gate on the generated avid. Syncs ride a second EBR ring.
- **Memories (23 EBR):** line ROM 15, pulse ROM 2, saw/cos ROM 1, cos ROMs 2, box ring 2, sync ring 1.
- **Shell:** heirloom v0.2.0 / mandala — timing-ID virtual-1080 steps, generated avid (`p_gavid` / `g_start`),
  source-width measurement.
- **Fit history:**
  - The first build overflowed at 11,300 LC.
  - In turn (each step is in `notes.md`):
    - the knot table became a direct line ROM;
    - the cubic per-row seam phase became quadratic;
    - the third-order DDA became second-order;
    - the pixel path was rescheduled to 24 stages;
    - the seam-locked G term, the per-line fall width and the separate rise/wedge ramps were dropped.
  - Result: 6,960–6,990 LC.
- **Verification** (GHDL 720p60 dec 1 against the RTL-exact `imodel.py`, static frame):
  - Y is identical and U/V are within ±1; 100 % of pixels are within ±4 after the tester's 4-px capture offset.
  - Motion: at K4 max the seams move +8 px per frame to the right, matching the per-field phase step.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). The once-a-field zone reseed is replaced by the per-line resync; off-avid luma is now 64 (was 0). Post-route Fmax: HD Analog s2 81.02, SD Analog s1 75.29, HD HDMI s5 77.59, SD HDMI s1 74.62, HD Dual s2 81.44, SD Dual s1 66.65 MHz; 6969-6995 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-27, NOT flashed.** router2. Post-route Fmax:

  | Config | Seed | Fmax (MHz) |
  |---|---|---|
  | HD Analog | 1 | 78.67 |
  | SD Analog | 1 | 73.57 |
  | HD HDMI | **2** (seed 1 missed) | 77.33 |
  | SD HDMI | **2** (seed 1 stalled the router) | 74.24 |
  | HD Dual | 1 | 78.74 |
  | SD Dual | 1 | 78.65 |

- 6959–6988 / 7680 LC (90.6–91.0 %), 23/32 EBR. All bitstreams pass iceunpack.
  `out/rev_b/ron/heirloom/plume.vmprog` (502,207 B).
- Presets are hand-written placeholders: Plume, Calm Stripes, Deep Wake, Twin Plumes, Blue Plume, Clean Lines.
- The TOML default for Bend is 200; the RTL's power-up glide value is 120, which is harmless (the firmware
  pushes the default and the glide converges in under a second).

## Deviations from the photo, and why
- **Outer-band chevrons:** the fine "<" texture in the top and bottom bands matches the photo's means and
  contrast but not its exact phases. The per-row seam phase is quadratic (median residual 0.23 cycle against 0.06
  for a cubic); the cubic did not fit on the chip.
- **Ribbon cells:** the seam-locked G term (about ±35 in y' 130–200) was dropped for LC. A model ablation showed
  the red/yellow cells unchanged or closer to the photo.
- **Red fall:** a single 170-px ramp centred on xF, where the photo varies 150–200 px.
- **Colour by region (4×4 grid):** within about 10 per channel, except the left corners' green (about 10 low)
  and the bottom-right red (about 28 low: the wedge is narrower than the photo's).
- **Texture realisation:** the tear and the wiggle shapes are one fixed noise realisation, with parabolic rather
  than exact sines.
- **Whites:** about 777 (the HD HDMI bar fix scales luma by 13/16).
- **SD (4:3):** shows the central 1440 virtual px, so the plume's base and the wedge are partly cropped. 720p and
  1080 are the same picture.
- **Interlace:** no bottom-field half-line offset (field_n is tied high on hardware, so this is moot).

## Open questions for Ron (2026-11-03 batch)
1. The Wobble crawl (the stripes' wiggle sliding sideways above the default) was cut for fit. Bring it back in
   place of something else, e.g. K4's faster top speed?
2. Is the P12 100 % split (the plume torn into an arrow and the outer bands) the right climax, or should the
   knee stop just short so the plume stays whole?
3. S9 Camera shows the camera in green over the blue gradient. Would you prefer the full-colour camera picture?
4. The Wedge on/off switch is gone (always on). Do you miss it?

## Hardware watch list
- Renamed: K6 Tear → Wobble, S8 Wedge → Hooks (Plume / All), S9 Ribbons → Ground (Stripes / Camera); preset
  "Plume" → "Photo", Deep Wake → Split Wake.
- The Video-mode default differs from v0.5: the plume follows the camera by ±47 px at the default Bend (Osc
  default unchanged). With black input in Video mode, the plume sits ~42 px left.
- Hooks All needs some Wake: at P12 0 % only the outer band changes.
- Seams near max: the fine comb aliases on SD.
- Cut for fit: the Wobble crawl. Reach tops out at ×1.87 (was planned ×2.5).
- ROUTER1 now (v0.5 was router2). No pin-skew screen; if HD HDMI shows lines, re-seed that config with router1.

## Next step
Wait for Ron's preview notes. Rebuild the diag with
`NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh plume`. On approval: `./heirloom_diag.sh merge plume`,
then repack the previewed diag bitstreams under the production TOML (copy
`build/programs/ron/plume_diag/rev_b/bitstreams/*.bin` into the production build dir, regenerate
`program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`) instead of
re-routing; update the production SUMMARY / README / PHASE2 row and commit only plume's files.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. Here the camera strength was on K5 (Bend), so K5 and K6 swapped: **K6 = Bend** (the camera strength, default now 512 = 50 %), **K5 = Wobble** (was K6). Nothing else changed: the VHDL swaps the two knob inputs (the processing is identical), the TOML blocks and preset keys moved with them (the Controls rows above are renumbered). v0.6.1, rebuilt with router1: all six seed 1; HD worst 79.41 MHz (HD Analog 79.41, HD HDMI 83.72, HD Dual 82.53), LC LC. Rebuild: `NEXTPNR_ROUTER="--router router1" ./heirloom_diag.sh build plume`. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/.


## K6 top +25 % and default 100 % (2026-11-15)

Ron (2026-11-15): the camera effect at K6 = 100 % 25 % stronger; K6 default 100 % (initial 1023, v0.6.2).
- **Classification: bend** (Bend adds camera luma to the seam and stripe phases and, in this diag, pushes the
  plume outline by the same product / 32).
- **Change.** The bend gain ×1.25 at every K6 position: `f_b5 = (g/64 + g/256) / 4` (6 bits, 0 .. 39, was
  0 .. 31), so the outline push grows with it (±312 px at most, was ±248). The bend product still fits 15 bits.
  The same change as production plume (no render for the diag).
- **Build:** router1; HD Analog s1 79.76, SD Analog s1 83.40, HD HDMI s1 78.15, SD HDMI s1 77.96, HD Dual s1 82.62, SD Dual s1 87.15 MHz; worst HD 78.15 MHz; 7235-7254 / 7680 LC; all seed 1. vmprog: out/rev_b/ron/_diag/plume_diag_v0.6.2.vmprog.
