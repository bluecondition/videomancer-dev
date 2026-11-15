# Monodala: one colour, topographic rings

**v0.2.0 (2026-11-04): built, NOT flashed.** Not from a photo: the effect Ron found on
[Mandala](../mandala/SUMMARY.md) (Diag v0.6.0) with its red key at −100, on its own and over the whole screen.

## Brief
Ron, 2026-11-03: Mandala with "red at −100, so the whole screen is blue with the light green topographical marks
over the image, showing the incoming video". He wanted that effect with the colour on a knob and the depth marks
in a different accent colour.

Ron, 2026-11-04, on v0.1.0, which kept Mandala's fitted face: "I don't want to see the same face as mandala has, I
want this effect to cover the entire screen and only be the solid color (deep blue by default) with the green
topographical circles modulated by the incoming video."

Kept from his hand-dialled Mandala Diag settings (read with `./capture_preset.sh --defaults --dry-run`):

| Setting | Value | Becomes |
|---|---|---|
| Width | 1023 | K3 default |
| Melt | 824 | K4 default |
| Flow | 329 | K5 default |
| Rings | 494 | P12 default |
| Breathe | 276 | fixed LFO depth (69) |

## What it does
1. **Field.** F is the exact distance from the screen centre, in rings, from a pipelined 8-step CORDIC (round at
   SD, 720p and 1080, within 0.5 virtual px). Then:
   - **S10 Two** folds x at ±480 virtual px: two ring systems, mirrored.
   - F += Melt × the input luma (or the Osc) + a slow breathe LFO.
   - The rings bend around the picture like contour lines on a map.
2. **Lines.** p = Mandala's pulse ROM[width][frac(F · Rings + Flow)]: 16 width banks; the rings are 40 virtual px
   apart at Rings ×1.
3. **Ground.** One solid colour at a level L:
   - **Solid:** L = K6 Level.
   - **Video:** L = the aligned input luma, darkened by Level.

   GT[L] = L × ground colour is a 256-entry table, rebuilt by accumulation in every vblank zone.
4. **Colour.** out = G + p · (C − G). C is the line colour from a 512-entry hue ROM (ink & hue), so lines keep
   their own colour over any ground.
5. **Output.** Mandala's conditioning: box-8 (box-4 at 720p / SD), luma [1 2 1], the chroma box pair, luma to
   ~777, then the blanking gate.

## Controls
| Control | What it does |
|---|---|
| **P12 Rings** | Ring density: 0 = ×0.125 (a few broad rings, 320 px apart), 50 % = 40 px, 100 % = ×3.6 moiré (gain 0.125 + 3.5p²). Glided. Default 494. |
| K1 Hue | The ground colour, the full wheel (256 hues: 0 = red, 171 = blue). Default 684 = blue. Glided. |
| K2 Accent | The line colour as a hue offset from the ground. Turning K1 recolours everything and keeps the contrast. Default 686 = −120°, green on blue. Both ends (0 / 360°) are tone on tone. |
| K3 Width | Line width: washed-out bands → hair-thin lines. Default 1023 = the thinnest. |
| K6 Melt | How far the video (or Osc) bends the rings: Mandala's curve, up to ±3.8 rings. Default 824 (±2.9 rings). |
| K5 Flow | Bipolar ring drift, still at the centre ±8; + streams outward, square law. Default 329 (inward). |
| K4 Level | The ground's brightness: 0 = black (lines on black) → 100 % = the full colour. Default 800 = deep blue (0, 0, 200). In Video ground: the picture's brightness. |
| S7 Lines | One / Spectrum: every ring takes its own hue, 45° steps from the accent; the colours ride with Flow. |
| S8 Ink | Bright / Dark: Dark lines are the accent at 3/8 level. |
| S9 Ground | Solid / Video: the ground becomes the incoming picture tinted in the ground colour, with the rings over it. |
| S10 Centres | One / Two: two ring systems side by side (a fold at ±480 virtual px). |
| S11 Source | Video / Osc: what bends the rings. The Video ground always shows the input. |

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks), on Mandala's serial shift-add multiplier: the Rings gain,
  the line start −(W/2) × step, the Flow speed, the LFO |sin| and the breathe offset. Glides (±4 deadband, 1/8
  ease per field) run on K1, K2, K3, K4, K6 and P12.
- **Ground table:** in the zone, the hue ROM is addressed with the ground hue. It is latched at zone clock 240,
  then GT[L] = (L × colour) >> 8 is written at zone clocks 256..511, by accumulation (no multiplier).
- **Coordinates:** x per pixel and y per line from the screen centre, in Q8 "ring units". 7.7729 per virtual px
  is chosen so that the CORDIC output (gain 1.6467) is the distance in Q9 rings with no scaling.
  - Per-mode steps come from the timing ID (virtual 1080-line coordinates).
  - y is reset to −540 virtual px each field and advanced at each source avid rise.
- **Pixel path, 28 stages:**
  - 1: |x|.
  - 2: the Centres fold; |y|.
  - 3–10: CORDIC.
  - 8–10, in parallel: the bend source (aligned luma or Osc) × Melt + breathe.
  - 11: F.
  - 12: ×Rings split; Video luma ×19.
  - 13: Fg; the Video level.
  - 14: phase + flow, in Q9 mod 256 rings with a ½-ring start, so the ring index changes between lines. Pulse ROM
    address; GT address.
  - 15: the hue ROM address (+ index × 45° in Spectrum).
  - 16–17: fabric copies.
  - 18: C − G.
  - 19: p × (C − G), split 5 | 4.
  - 20–21: sums.
  - 22–28: the output conditioning.

  Colour lands at stage 21 and the output at 28, Mandala's depth, so the hsync-to-avid parity (U / V pairing on
  hardware) is unchanged.
- **Video:** a 256 × 8 block-RAM ring of the input luma, read 4 + 3 clocks back. The bend reads it aligned at
  stage 7; the Video ground reads it 4 FFs later, at stage 11. No raw video rides in long FF pipes.
- **Memories (14 EBR):** pulse ROM 4, hue ROM 4, GT 2, box ring 2, sync ring 1, video ring 1. GT and the rings
  carry `no_rw_check`.
- **Latency** 28 in every mode (= `processing_delay_clks`). Generated avid; blanking gate; colours in the
  hardware convention (U = Cr), from Mandala's RGB → YUV coefficients.

## Build
v0.2.0, 2026-11-04, router2, all six configs on seed 1. Post-route Fmax:

| Config | Fmax (MHz) |
|---|---|
| HD Analog | 89.34 |
| HD HDMI | 88.51 |
| HD Dual | 79.67 |
| SD Analog | 82.09 |
| SD HDMI | 87.10 |
| SD Dual | 80.12 |

- 6,119–6,144 / 7,680 LC (80 %); 14 / 32 EBR.
- All six bitstreams pass iceunpack.
- vmprog: `out/rev_b/ron/heirloom/monodala.vmprog` (421,646 bytes).

## Resolution notes
- The rings are sized to screen height (virtual 1080-line coordinates) and round at every resolution.
- SD (4:3) shows less width, so fewer rings reach the sides.
- 720p and SD use a box-4 output filter instead of box-8, so their lines are slightly crisper.

## Status
- **v0.2.0 built 2026-11-04, NOT flashed.** The face is gone: rings over the whole screen; K6 Shade → **Level**;
  S10 Faces → **Centres**. No image simulation; an integer check of the CORDIC as written showed ≤ 0.46 virtual px
  error over the 1080 frame and no overflow.
- v0.1.0 (2026-11-03, not flashed) kept Mandala's fitted face. Ron rejected it: the effect alone, over the whole
  screen.
- Presets are hand-written placeholders: Blue Topo (the defaults), Spectrum, Amber Video, Dark Ink, Night Rings,
  Moire Climax, Twin Centres. Capture real ones with `./capture_preset.sh`.
- Rebuild: `.heirloom_work/rb_monodala/build.sh` (the shared slot queue + scoped clean), or
  `./build_heirloom.sh monodala`.

## Watch on first flash
- Defaults: a solid deep-blue screen, thin green rings round the centre, bent by the video and drifting inward.
- If "deep blue" should be darker or brighter, turn K6 Level and I'll move the default.
- **S9 Video** shows the input tinted. If it looks shifted horizontally against the bends, the video ring delay
  (`rv_ra <= s_wp - 4`) is off.
- HD HDMI: thin saturated lines on a contrasting ground are pixel-rate chroma swings. Mandala's chroma
  [1 2 2 2 2 2 2 2 1]/16 pair is in place; if thick bars appear, widen the chroma first.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. Here the camera strength was on K4 (Melt), so K4 and K6 swapped: **K6 = Melt** (the camera strength, default now 512 = 50 %), **K4 = Level** (was K6). Nothing else changed: the VHDL swaps the two knob inputs (the processing is identical), the TOML blocks and preset keys moved with them (the Controls rows above are renumbered). v0.2.1, rebuilt with router1: all six seed 1; HD worst 91.69 MHz (HD Analog 95.05, HD HDMI 93.74, HD Dual 91.69), LC LC. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh monodala`. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/.
