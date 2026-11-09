# Bubbles — rainbow bubble lenses ringed by dark halos, swelling as a slow swell rolls up through them

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20240105_20_46_57_t0p5.png` (clip `WIN_20240105_20_46_57` at 0.5 s). The still shows three columns
of glassy bubble lenses on a mint-to-cyan ground. Each lens is filled with a red / yellow / pink / magenta rainbow and
ringed by a soft dark-green halo. A small lens hangs under each big top lens, joined to it in cusped "beaks", and the
clip's wavy rainbow curtain is just entering the top-right and bottom-right corners. Presets are hand-written
placeholders.
Working files: `.heirloom_work/rb_bubbles/` (`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **One analog field draws everything.** The halo rings' half-widths per row (centre column, 31 rows) fit
  S(y) = 1.98 + 1.52 cos(2π(y − 109)/1050) − 2.535e-5·u², with u = ((y − 242) mod 536) − 268, at rms 0.04.
  - The ring sits at S = dx²/50000 and the lens edge at S − dx²/50000 = 1.
  - The ring-to-edge gap hw_halo² − hw_lens² is 44–60 k px² on four rows. That is constant, so the horizontal
    term is a parabola.
  - So: a horizontal parabola per column, plus a screen-fixed cusped vertical parabola wave (period 536 lines,
    seams at y 242 and 778, which give the beaks), plus a slow vertical cosine (period 1050).
- **Columns.** Axes at x ≈ 50, 987 and 1924 (period 937 px), found from the symmetric halo pairs.
- **Halo.** A near-black core (G ≈ 37) with linear sides about 45 px each way on the top rows (±0.7 field units),
  wider where the field is shallow (about 80 px under the middle lens). It sits about 80–120 px outside the lens
  edge.
- **Ground.** Between the columns only the red moves, and it follows the slow cosine: R 188 → 21 → 173 at y 180 /
  600 / 1060; G 224–253, B 182–236. Fit: R = 112 + 65 Vs, G = 246 − 6 Vs, B = 214 − 20 Vs.
- **Lens fill = the RGB negative of a G/B colour cycle.** The lens colours are the inverses of the curtain's bands
  (magenta ↔ green, yellow ↔ blue, red ↔ cyan, white ↔ black); clip `20_45_50` confirms it.
  - The cycle is screen-fixed with a 512 px period: the white→yellow edges sit at x 299, 813 and 1844, and the
    red→magenta edges at 99, 1124 and 1639.
  - It is bent per line by a third vertical cosine: τ = 134 − 150 cos(2π(y − 10)/1050) px. The bands slant
    +0.4–0.56 px/line in the top lenses and run about vertical in the bottom ones.
  - A dark line in the cycle (phase ≈ 0.93) gives the dark arcs inside the lenses.
- **Inversion key.** Its edges are the non-inverted crescent under the top lens (y 195–252 on the axis) and the
  mint lid on the middle lens (y 255–305).
  - Fit: Vf + 1.12 − dx²/125000 > 0, with C2 between 1.07 and 1.18.
  - At the Vf peaks it is 374 px wide, which also places the curtain's inversion edge at x ≈ 1550.
- **Curtain.** The edge points (1404, 5) … (1670, 165) and (1374, 1060) … (1596, 920) fit
  x = 1995 − 591 cos(2π(y − 10)/1050) to a few px. It is the same screen-fixed cosine as τ.
  - Inside are bands parallel to the edge: green and black (non-inverted), then red, magenta, pink and
    yellow-green where the inversion key is on.

## Motion in the clip
- **The slow swell rolls up; the slots stay put.** The clip was extracted at 10 fps.
  - The ground's red gradient rolls up about 125 px/s: its minimum goes from y ≈ 650 at t 0 to ≈ 50 at t 4.8 s.
  - The beak seam (y ≈ 250) and the curtain tips (y ≈ 20 and ≈ 1045) do not move for the first 2 s.
  - The lens edges climb about 82 px/s. The left top lens's bottom goes 234 → 24 over 1.0–3.6 s, and the bottom
    lens's top goes 950 → 802 over 0–1.8 s.
  - The lenses morph as they climb: the middle lens shrinks to a dark bell (+0.5 s) and a pillar (+1.0 s), then
    comes back as a small lens with a pale rim (+1.5 s), and the bottom lens rises into an "8" stack.
- **Fit.** The float proto was fitted to the four m-frames (`motfit.py`): best Vf speed 0, Vs speed 130–140 px/s up.
  The mean abs error is 157 over the four frames, against 210+ for a static field. The "rising bubbles" are the
  swell passing through fixed slots, and the proto reproduces the bell, the pillar and the return (`cmp_motion.png`).
- **Ron's knob.** From about 2.4 s the curtain sweeps in from the right and fills the frame with wavy vertical
  rainbow bands. The other clip (`20_45_50`) is that full curtain with lenses riding in it. That became P12.
- **In the program.**
  - The default Rise (756) rolls the swell 2.19 virtual lines per field up (132 px/s at 59.94 Hz).
  - The RTL advances exactly one model step per field: GHDL frames at 2 and 3 warm-up fields match the integer
    model at steps 2 and 3, with a mean difference of 2.6.

## What it does
1. **Field per line** (line engine, the next line):
   - Vf = −0.1038·u² on the 536-line saw, Vs = 1.52 cos(rolling phase), Vq = cos(screen-fixed phase).
   - S1 = 1.98 + Vs + Vf − bubble level, S2 = Vf + 1.05, Sh = S1 − halo offset.
   - The ground colour from Vs, the rainbow bend τ and the curtain edge from Vq (plus Vf kinks at the P12 climax),
     and the two phase starts.
2. **Per pixel.**
   - a = c·dx² comes from a second-order DDA (A += D, D += 2c) that wraps at each column edge.
   - F1 = S1 − a, F2 = S2 − 0.375 a, d = Sh − a. The camera term goes into a (and F2) and into the rainbow phase.
   - Lens where F1 > 0. Inverted where F2 > 0 (Lens Flip swaps that). Curtain where x > the edge.
   - Lens: the rainbow ROM at the screen-fixed phase, RGB-inverted, with rim shading: up to ¾ toward light cyan on
     the right rim and toward white-pink on the left.
   - Non-inverted: 255 − colour. Curtain: the rainbow at its own phase, parallel to the edge.
   - Ground: clamp(ground − dip(|d|)). The dip is a V of ±0.7 units to a green-black core; Night makes it a glow.
   - A dark outline in the band just outside the lens edge: ¼ brightness on the right side, ½ elsewhere.
3. **Colour.** RGB → hardware YUV by three 256-entry contribution ROMs (U = Cr), luma pre-scaled 16/13 and capped
   at 930. Then the chroma box with resync, luma [1 2 1], 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bubbles | the lens level: no lenses, only halo rings (0) .. the still (512) .. huge merged lenses swallowing the ground (1023); the halo follows the lens |
| K2 Halo | the halo ring's distance from the lens: hugging the rim (0) .. the still (512) .. far out, closing into diamond rings between the columns (1023) |
| K3 Columns | the column spacing: ×2.5, one column (0) .. 937 px (512) .. ×0.4, five columns with merged lens bands (1023); exponential |
| K4 Rise | bipolar swell roll, ±8 deadband, square law, up to 10 lines per field. 756 = the clip's 2.2 lines per field up (the lenses swell, shrink to a dark bell and return); 512 = still; left = the swell sinks |
| K5 Rainbow | the colour-cycle width inside the lenses: ×4, one or two colours per lens (0) .. 512 px (512) .. ×¼, candy stripes (1023); the curtain's bands follow |
| K6 Camera | how far the source swells the lenses and halos (white = +1.85 field units at 1023) and bends the rainbow (up to 0.47 cycle). 256 is the default |
| S7 Lens | Normal (the still) / Flip: the inversion key flipped, so the lenses turn cyan / blue / green / black and the crescents and lids red |
| S8 Rings | Off (the still) / On: the halo repeats every 2 field units into nested contour rings over the whole ground |
| S9 Mirror | Off (the still) / On: the whole field read upside down, so the small lenses hang under the centre, the halos drip upward and the swell sinks |
| S10 Ground | Mint (the still) / Night: a dark navy ground with cyan glowing halos |
| S11 Source | Video / Osc |
| P12 Wave | the rainbow curtain: none, pure lenses (0) .. the still's corners (196) .. a full wavy curtain with the lenses riding in it as inverted rainbow discs (1023). Above the still its swing narrows from 591 to 261 px and Vf kinks up to 160 px appear, so the bands stand up and kink at the seams like the other clip; glided |

## Video
- **Video mode (the default).**
  - The camera's luma, (Y >> 2) − 16 from a one-line buffer, is patched into the field. White adds up to +1.85
    units to F1 and the halo distance, and half that to the inversion key.
  - The same luma bends the rainbow phase by up to 0.47 cycle.
  - Bright parts of the picture swell the lenses and push the halos out, so the picture shows as ragged lens and
    halo contours with the rainbow bent inside them.
  - At the default Camera (256) a busy photo (`selfie.jpg`) bends the edges and rings visibly while the composition
    still reads. With no camera (black input) the frame is the still.
- **Osc mode.**
  - A drifting 2-D triangle lattice (214 × 180 virtual px) feeds the same path.
  - At the default gain it gives the lenses and halos a lumpy, wobbling outline and kinks the rainbow stripes.
  - Camera stays live in both modes.

## Engine
- **Per-field sequencer (vblank zone).**
  - A shared glide over K1, K2, K3, K5 and P12.
  - One serial multiplier runs 16 slots:
    - the column period (exp2 ROM) and its /3 for 720p;
    - the DDA start and wrap constants: c·m0², c·(2 m0 + 1), c·mw², c·(2 mw + 1), with m0 reduced into the
      column;
    - the rainbow and curtain phase steps (exp2 ROM);
    - the curtain edge, swing and kinks;
    - the cosine step per line;
    - Rise².
  - The swell phase rolls once per field.
- **Per-line engine.** Kicked at each line start (and once in the zone for line 0), it computes the next line in
  ~141 clocks, so it fits SD's 858-clock line.
  - One serial multiplier: u², ×425, |cos|·6226, |cq|·300, |cq|·swing, |Vf|·kinks, kr·|tx|, kw·|tw|.
  - A quarter-wave 256 × 16 cosine ROM.
- **Beam and pixel path (latency 20).**
  - The beam holds the dx² DDA with the column wrap and x. The rainbow and curtain phase accumulators run late,
    holding the stage-5 pixel's phases.
  - The camera luma is read 2 px ahead so it lands at stage 2.
  - Stage 3 is the gain product, stage 4 the camera folded into a, stage 5 the three fields, and stage 6 the keys,
    the outline band, the rim weight, the halo index and the rainbow address.
  - Stages 7–8: rainbow and halo ROMs plus a fabric copy. Stage 9: the rim delta and the halo clamp. Stage 10: the
    rim mix. Stage 11: select + outline into the matrix ROM address.
  - Stages 12–17: matrix, sums and compose. Stages 18–21: output conditioning.
- **Memories (17 EBR).** Line buffer (4), rainbow 3 × 512 × 8 (3), cosine (1), exp2 (1), halo dips 128 × 32 (2),
  matrix (6).
- **Contracts.**
  - The parity-locked generated avid, and one latency of 20 in every mode.
  - Virtual 1080-line coordinates: SD shows the centre 4:3.
  - Colours in the hardware convention (U = Cr), and the running-sum resync on the chroma box.
  - No narrowing resize() of signed values (all slices), and the ground colour proven in range.
- **Verified (GHDL, 720p60, decimation 1).**
  - The default Video frame with a black input matches `imodel.py`: mean |diff| 8.5, with 4 % of pixels > 24, all on
    edges softened by the output filter. The RTL output sits 8 px right of the model: the generated-avid start
    plus the filter centre.
  - The Osc and camera frames bend as the model does.
  - The motion advances one model step per field.
  - `dupin.py`: 0 `CARRY I0==I1`, 0 duplicate LUT inputs.

## Status
- **v0.1.0 built 2026-11-09 with ROUTER1, all six configs on seed 1. NOT flashed.**
  - Post-route Fmax (builder): HD Analog 101.68, SD Analog 101.03, HD HDMI 91.63, SD HDMI 90.07, HD Dual 102.36,
    SD Dual 100.96 MHz.
  - 6995–7014 / 7680 LC (91.3 %), 17 / 32 EBR. iceunpack OK ×6.
  - Output: `out/rev_b/ron/heirloom/bubbles.vmprog` (471,744 bytes, unsigned).
- **Rebuild:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh bubbles`.
  The first router2 build passed HD Analog (83.2) and SD Analog (85.9), then stalled on HD HDMI seed 1 for more
  than 14 min. Router1 closed HD HDMI at 91.6 MHz on seed 1 in 90 s, so stay on router1.
- `hd_clock_divisor` untouched. No pin-skew seed screen was run; if HD HDMI shows lines, re-seed that config.
- **Fit history.**
  - The first netlist packed to 8143 LC (106 %). It had three halo multipliers, multiply-based rim shading,
    saturating line values, shift-add ground colours, a 24-stage pipeline, a 6-channel glide and a Stagger switch
    with a second set of line values.
  - Cut to 7001 LC:
    - additive halo dips;
    - quarter-step rim shading;
    - line values proven in range (no saturation);
    - ground colours from single shifts;
    - late phase accumulators and a camera read-ahead (latency 24 → 20);
    - K6 unglided with a 6-bit gain;
    - 0.375 a for the inversion key (C2 → 1.05);
    - no band densification at the P12 climax;
    - S9 Stagger replaced by Mirror.

## Deviations
- **Lens colours.**
  - The middle lens is pinker than the still's white and yellow.
  - The crescent under the top lens reads cyan / blue where the still's is green / teal.
  - The rainbow bend is one cosine for the whole frame, so the left column's stripes slant as much as the
    centre's. The still's slant grows from left (0.12 px/line) to right (0.56).
- **Rim shading.** Its three zones (left / middle / right of each column) can show a faint vertical seam inside
  small lenses.
- **Curtain.** The top-right curtain shows a few more bands than the still. Its non-inverted part uses a 0.53-cycle
  phase offset so that it reads green then black next to the inverted red, as in the still.
- **Not drawn:**
  - the faint teal darkening down the column midlines (G 253 → 219 near y 700–800);
  - the halo rims flashing pale or white in the clip (a level wobble);
  - the camera softness and the lens-outline line jitter.
- **Outlines.** Hard-keyed with the shell's output blur; the left 8 px fade in from black (filter warm-up), the same
  on every row.
- **Cut for budget.**
  - S9 Stagger (every other column half a slot down, a honeycomb) became Mirror.
  - The P12 climax no longer makes the curtain bands denser (only straighter and kinked).
  - The camera luma is not low-passed, so very fine detail in the picture makes the lens edges ragged.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3 (virtual x 240–1680), so the
  left and right columns are cut in half and only the inner tips of the curtain show.
