# Plume — Heirloom #19

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230315_00_58_54_Pro.jpg` (1920×1080 capture, 2023-03-15). Replaces v0.1.0 (a mirrored column of
chevron ripples), which Ron judged too abstract.

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
| K5 Bend | How far the video (or Osc) bends the seams (up to ±2 cycles) and the stripes (±4 cycles). Default 200. Glided. |
| K6 Tear | Line tearing of the top and bottom bands (triangle jitter plus a 3-line hold, gated by the measured tear zone). 0 = clean. |
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
