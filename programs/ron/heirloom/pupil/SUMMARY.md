# Pupil — almond eyes of cream moire rings round a blue-violet pupil, snapping open on every beat

**v0.1.0 built 2026-11-10, not flashed.** Heirloom (video phase), from the clip `WIN_20230408_23_11_38` at 3.5 s
(`photos/vid/WIN_20230408_23_11_38_t3p5.png`, frame 105 of 177; the same patch at 0.5 s and 4.0 s). A column of three
almond "eyes" on flat green. Each eye is cream / pale-green / teal ring arcs on its left half, cut into brick-like
dashes, with a soft dark-green halo outside. In the middle is a blue core holding a violet ball with a navy rim and a
dark crescent. The core has flat top and bottom edges. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_pupil/` (`notes.md` has every number).
- **Ground.** Flat green (0, 255, 0) covers 54.6 % of the frame (hardware 10-bit Y 578, Cr 137, Cb 216).
- **Eye field.** Every edge and ring is a level of one field, the eye depth
  `e = R − |x − cx| − k·dy²`. That is an H triangle plus a parabola tiled vertically (a squared triangle). dy is the
  distance to the nearest eye row.
  - Fitted on the left edges of 10 frames (`fitgeo.py`): period 520–522 lines (522), eye rows at 520–522 (521),
    k = 0.00627–0.00641 px per line² (0.00635, so Kv = k·522² = 1730 px), cx = 1072–1079 (1077).
  - The cusps where neighbouring eyes meet come from the parabola tile's sharp peaks. The noses on the left are
    rounded.
- **Rings ride the edge.** The ring peaks sit at e = 25, 70, 105, 138, 200 and 270 px in all 19 measured frames,
  whether the edge is at x = 176 or 764. So the colouriser reads the depth from the edge, not a fixed field.
- **Core.** The core appears only on rows where the eye is deep enough at its centre column: R − k·dy² ≥ W, with
  W = 351–373 over 8 frames (362). That is why its top and bottom are flat.
- **Left / right.**
  - The rings show on the left half and fade into the green on the right.
  - On the rest frames, rings on the right show only deeper than e ≈ 120–150 px, so the fade follows the eye's
    unfolded right edge.
  - In core rows the violet gives way to the blue at about x = cx + 40..150.
- **Palette** (palfit of the still, k = 12):
  - green 54.6 %, blue (17, 22, 247) 7.6 %, violet (98, 13, 253) 4.8 %;
  - halo greens (5, 181, 3), (3, 219, 5) and (24, 140, 21);
  - pale greens (80, 179, 59) and (145, 208, 104), cream (218, 239, 140) 3.0 %;
  - navy (22, 32, 165) and (15, 25, 85), dark teal (18, 81, 39).

  The tables over e were taken as medians over 17 frames (`palx.py`) and sharpened (`pal.py`):
  - left: halo → rings → navy rim → violet ball (peak at e ≈ 400) → dark crescents;
  - right, core rows: green edge → teal → blue, with a dark rim at e ≈ 380;
  - right, other rows: green.
- **Moire.** The vertical autocorrelation of the ring band peaks at 20–22 lines in 5 frames. Bands of about 10 rows
  are alternately dilated by about half a ring pitch, which gives the bricks and the 8–15 px teeth on the edges and on
  the core's right side.
- **Streaks.** Teal key-dropout streaks, 4–6 rows tall, cross the blue core (y 422, 460, 600, 637).
- **Mid-attack.** The still catches the eye mid-attack: R ≈ 400 at the top, 459 at the middle eye and 670 at the
  bottom (R rises about 0.42 px per line below line 400).

## Motion in the clip
All 177 frames were extracted (29.98 fps, 5.9 s; `breath.txt`, `palx.py` per-eye R).
- **A percussive envelope.** The eye snaps open in about one field, sweeping down the frame (frames 58 and 104 show
  the top eye still small and the bottom one already open). It closes in about 0.23 s.
  - R per 30 fps frame: 919, 790, 667, 558, 461, 393, 354, then rest at 310–330. The drops are 129, 123, 109, 97,
    68 and 39 px.
  - During the decay R also falls down the frame (580 / 558 / 542 over the three eyes), so the decay is continuous
    along the scan.
- **A rhythm.** Hits fall at 0.37, 0.57, 1.33, 1.93, 3.10, 3.50, 3.70, 4.50 and 5.10 s: steps 0, 1, 5, 8, 14, 16,
  17, 21 and 24 of a 0.2 s grid.
- **The core.** It appears whenever R > 362.
- **The other looks** are the same patch:
  - 4.0 s is the rest state (small cream eyes, no core);
  - in 0.5 s (and frames 12–38) Ron's colour knob turned the rings lime / dark green.
- **What stays still.** The eye column doesn't drift. The moire bands sit at different rows in every frame.
- **What the program does with it.**
  - It replays Ron's 32-step hit pattern at 0.2 s per step (12 fields).
  - Each hit attacks at 0.42 px per virtual line, reaching the 440 px depth in one field and sweeping down the
    screen as in the still.
  - The decay runs per line at about ΔR/4 + 2 px per field, capped at 60 px per field at the default depth. The eye
    is back in about 0.25 s, and the eyes lower on the screen are a little smaller during the fall, as in the clip.
  - The moire bands crawl 0.67 virtual lines per field.

## What it does
1. **Eye depth.** `e = R + z − Kv·t² − |x − cx| + Bend·luma`, in half virtual px.
   - `t = frac((y − 521)/P + 0.5) − 0.5`, from a Q20 phase accumulator.
   - `R` = Dilate + the beat envelope, per line.
   - `z` = the moire: 16..31 px on the first 10 virtual lines of every 20, random per band.
2. **Core rows.** A row is a core row when `(R − Kv·t²)·s ≥ 362`. The moire is left out of this test, so the window
   has clean edges.
3. **Palettes.** Two palette ROMs are read at the ring-scaled depth `e·s`:
   - the left table (halo, rings, navy rim, violet ball, crescents; the deepest quarter repeats for the dilated
     pupil);
   - the right table (core-row blue or other-row green).
4. **Blend.** `out = R + (L − R)·A`, with `A = clamp((K − x)/128 px, 0, 1)`.
   - In other rows `K = ev + cx − 80`: rings show on the right only deeper than ≈ 130 px.
   - In core rows `K = cx + 60`.
5. **Dropouts.** In one moire band in 8, the first 5 virtual lines read the green table instead of the core
   blue: the streaks.
6. **Slip** (S7). Bands of 32–152 virtual lines are shifted ±0–315 px, re-rolled every 8 fields.
7. **Output.** Y, U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes (the clip's softness and the HD HDMI bar fix,
   with the running-sum resync), then luma 13/16 and the blanking gate. Palette luma is pre-scaled 16/13 and capped
   at 940.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Eyes | the column of eyes: 0.73 per frame height (one giant eye) .. 2.07 (512, the clip) .. 5.8, exponential; the almond shape is kept |
| K2 Squint | the eye's vertical curvature Kv, ×1/4 .. ×4: eyes merged into a wavy blue column (0) .. the clip (512) .. flat slits with wide green gaps (1023) |
| K3 Rings | the ring scale, ×0.35 .. ×2.8, with the core threshold scaled with it: fat cream rings filling the eye (the core shows only on beats) .. the clip .. fine rings round a big blue core with dark crescents |
| K4 Beat | the rhythm's step, 0.8 s .. 0.2 s (512, the clip) .. 0.05 s (a drum roll), exponential; Ron's 32-step hit pattern; never stops |
| K5 Pulse | how far each beat throws the eye open: 0 (a still eye) .. 440 px (512, the clip's peak R ≈ 920) .. 880 px (each beat floods the screen with the pupil) |
| K6 Bend | how far the camera (or the Osc) bends the eye: 0 .. 2 px per luma step, square law; 354 = 0.24 px (48 px at white) |
| S7 Slip | Off (the clip) / On: sync slips, whole bands of the picture thrown sideways |
| S8 Core | Blue (the clip) / Rings: no core, the ring moire repeats through the whole eye (the clip's rest look) |
| S9 Twin | one column / two mirrored columns gazing at each other (eyes at 600 px either side of the centre) |
| S10 Colours | Green (the clip) / Magenta: complementary chroma (magenta ground, lilac rings, olive / amber core) |
| S11 Source | Video / Osc |
| P12 Dilate | the pupil's opening R: 0 % = a column of small cream seeds (150 px); 343 = the still (476 px); 100 % = the eye swallows the screen, violet crescents and blue edge to edge (1306 px) |

**Presets:**
- Still (the defaults);
- Rest Seeds (the clip's rest);
- Wide Open;
- Twin Gaze;
- Ring Moire;
- Slip Drums;
- Magenta Slits;
- Osc Crystal.

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 from a one-line buffer, times Bend, is added to the
  eye depth.
  - Bright areas dilate the eye locally, so the rings and the core edge bend around the picture.
  - The violet ball and the crescents emboss it.
  - At the default Bend the selfie bends the drawing and the eyes stay readable. 1023 tears them into the picture.
  - With no input, Video mode is the still.
- **Osc mode.** A 160 × 135 virtual-px triangle lattice, drifting about 1 px per field, feeds the same path at twice
  the camera's gain.
  - At the defaults the eyes turn crystalline and wavy: an unmistakable flip.
  - Bend stays live.
  - ×3 was tried first and left single-row ring fragments, so ×2 was kept.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial shift-add multiplier, 9 slots. The slot number is decoded two clocks ahead and the operands are
    pre-muxed.
  - Five exp2-ROM lookups: K1, K2, K3, 1/K3 (the depth bias 256/s, so the index needs no add) and K4.
  - A shared glide (1/8 per field) on K1, K2, K3, K5, K6 and P12.
  - It computes:
    - the eye phase step and the phase at line 0;
    - Kv and the ring mantissa / shift;
    - the core threshold;
    - the dilation (two slopes), the pulse depth and the decay cap;
    - the attack step, the bend gain and the beat step;
    - the moire roll and the slip seed.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in about 108 clocks (it fits SD's
  858-clock line), using 6 serial products:
  - t², then Kv·t²;
  - ΔR·ys for the decay;
  - two band hashes (`n·40503`, xor-folded);
  - the core test.

  It also runs the beat accumulator and step, the attack / decay envelope (Q12 px), the moire band and dropout, and
  the slip band and offset.
- **Beam.** x in half virtual px from the line's slip offset, plus a reflecting DDA for |x − 1920| (the Twin fold).
- **Pixel path (15 stages).**
  1. Fold.
  2. Distance to centre, fade ramp and luma.
  3. |d|, A and the luma × gain nibbles.
  4. Biased depth and bend sum.
  5. Clamp.
  6. e × mantissa (3 + 3 bits).
  7. Sum.
  8. Shift and the two addresses (Rings wrap, deep-pupil repeat).
  9. ROMs.
  10. Fabric copy.
  11. L − R.
  12. × A partials.
  13. Out.
  14. Compose (complementary chroma; black off the generated avid).
  15. Output boxes, 13/16 and the gate, at C_LAT 20 in every mode.
- **ROMs (13 EBR):**
  - left palette 512 × 32 (4);
  - right palette 512 × 32 (4);
  - exp2 256 × 16 (1);
  - line buffer 2048 × 8 (4).
- **Contracts:**
  - the generated avid;
  - one latency;
  - the blanking gate;
  - output conditioning with the running-sum resync;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr), generated by `pupil.py`.
- **Verified** (GHDL, 720p60, decimation 1, against the integer model `imodel.py`, after the tester's 4-px capture
  offset):
  - defaults, black input: 100 % of pixels within 6 levels (mean |d| 0.00);
  - Beat 1023, frame 6 (eyes thrown open): 100 %, so the beat, the envelope and the line engine are bit-exact;
  - Osc: 90.5 %; selfie video: 91.8 % (the line-buffer and lattice lag);
  - `thincheck.py`: 0 single-row dark or bright runs ≥ 24 px at the defaults, P12 0 / 100, Slip, Rings, Twin, Osc and
    K1 / K3 max (fields 3, 15, 40);
  - proto against the still (with the still's attack ramp): mean |RGB| 19.7; 4 × 4 grid median cell difference 9
    levels; green 59.5 % vs 61.1 %, blue 13.6 % vs 14.8 %.

## Status
- **v0.1.0, 2026-11-10, ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh pupil`). All six configs on
  seed 1.
- Post-route Fmax (builder-reported): HD Analog 98.69, SD Analog 101.35, HD HDMI 105.04, SD HDMI 99.76,
  HD Dual 100.04, SD Dual 93.83 MHz.
- 6546–6564 / 7680 LC (85 %), 13 / 32 EBR. iceunpack OK × 6.
- `out/rev_b/ron/heirloom/pupil.vmprog`. **Not flashed.**
- The first build of this netlist was killed by a machine reboot. The rebuild above is the one packed.

## Deviations
- **Default frame.** The still is caught mid-attack: its top eye is small and its bottom eye already open. At the
  defaults the program rests at the middle eye's opening (R 476), so all three eyes match the middle one. The still's
  top-to-bottom gradient appears during each beat's one-field attack.
- **The clip's rest.** The small cream eyes with no core are not the default; they are P12 ≈ 18 % (preset Rest
  Seeds). At the defaults the beat throws the eye from the still's size to the clip's peak (~920 px).
- **Brightness.** The cream ring peaks come out at about 777 output luma (the shell's white cap).
- **Palette.** The ring tables are cross-frame medians, sharpened 1.4×. The still's individual bricks are crisper and
  more irregular. The moire is a clean square wave (period 20 virtual lines, 50 % duty, 16–31 px per band), and the
  core's right-edge spikes are shorter than the still's longest ones.
- **Streaks.** The dropout streaks are pure green (the non-core table) where the still's are teal. The thin bright
  line along the core window's top edge is not drawn.
- **Colour knob.** The 0.4–1.3 s lime-ring state is not reproduced. S10 gives a complementary palette instead (a bold
  flip).
- **Pinch.** The still's top pinch meets at x ≈ 980; the model's eyes meet at cx (1077) when they just touch.
- **Left edge.** It fades in from black over 8 px (filter warm-up).
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3, with the eye column a little right
  of centre as in HD. The moire and slip bands are counted in virtual lines, so they look the same at every
  resolution.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. K6 here already was the camera strength, so only its default changed: initial value 354 -> 512 (v0.1.1). The vmprog was repacked from the same bitstreams (verified: only the 150-byte settings block differs), so timing, seeds and LC are as above. Presets keep their own K6 values. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/heirloom/.

## K6 top +25 % and default 100 % (2026-11-15)

Ron, after looking at fern and kiln renders with K6 at 100 %: in programs where K6 bends the drawing, the camera effect at the top of the knob is 25 % stronger. The K6 default is now 1023 (100 %), v0.1.2.
- **Classification: bend.** K6 Bend adds the camera luma to the eye depth, so the rings and the core edge bend around the picture.
- **Change.** Per field (step 315), the camera's applied gain is c_g6e = g + g/4 (×1.25 at every knob position; max 255 -> 318, inside the 10-bit register; nibble products ≤ 4770 of 14 bits, the bend sum ≤ 76 k of 18 bits, e clamped as before). That is up to ~2.5 px per luma step instead of 2. The Osc keeps 2 g, so Osc mode is unchanged (now 1.6× the camera rather than 2×).
- Look: `.heirloom_work/rb_pupil/k6b/k6b_cmp.png` (bear, 720p, K6 100 %). The same eye, with more of the camera's outlines bent into it and the blue core showing where bright areas dilate past it.
- Build: router1. HD Dual closed on seed 2 (seed 1 stalled the router); the other configs on seed 1. HD worst 85.51 MHz (HD Analog 85.51, HD HDMI 94.26, HD Dual 98.18); 6,566 LC.

## v0.2.0: Ron's preview notes (2026-11-19)

Ron (Heirloom Preview Queue, rework on v0.1.0): "I do not like the beat effect but everything else looks good"

- **Removed: the beat.** Ron's 32-step hit pattern (C_PAT), the beat accumulator and step, and the per-line attack /
  decay envelope added to R (with its decay product, cap and attack constants, and the K4 exp2 lookup). R is now
  Dilate (P12) + the Tilt below, so the eyes no longer snap open. The moire crawl (0.67 virtual lines per field) and
  Slip's re-rolls are the only motion left; nothing else Ron liked changed (default frame = v0.1.x between beats).
- **K4 Beat -> Bricks** (it had nothing left to do): the moire depth, the clip's brick bands. 0 % = clean concentric
  rings (no bricks, no teeth on the edges), 512 = the clip (16-31 px, unchanged default), 100 % = x4 (64-124 px): heavy
  brick bands breaking every ring and the core edge. Gain k/4 below 512, 128 + 3(k-512)/4 above (Q7), applied per
  moire band in the line engine (one 6 x 9 product in its own state). K4 joins the shared glide (7 channels).
- **K5 Pulse -> Tilt** (likewise): the still's own top-to-bottom opening, held still instead of flashed by a beat.
  R(y) = Dilate + slope * (y - 540) virtual lines, slope = (k - 512) / 512 px per virtual line with a ±8 deadband:
  0 = the top eyes open and the bottom ones shut (±540 px at the edges), 512 = flat (the default, as v0.1.x at rest),
  1023 = the bottom eyes flooding, the top shut. **~727 matches the still's mid-attack gradient (0.42 px / line).**
  A per-line accumulator (e_tl, Q12 px) on active lines only, like the eye phase; R clamps at 0.
- **Presets:** "Slip Drums" (K4 820 = a fast beat) is now "Slip Bricks" (K4 820 = moire x2.8, with Slip); "Wide
  Open" dropped its K5 700 (the old pulse depth), so it keeps its look without a tilt. The others are unchanged.
- v0.1.2's vhd / toml / vmprog are in `.heirloom_work/rb_pupil/` (`*_v012.*`).
- **Build** v0.2.0, router1, all six configs on seed 1 (v0.1.2 needed seed 2 on HD Dual). Post-route Fmax: HD Analog
  95.08, HD HDMI 96.68, HD Dual 103.97 MHz (worst HD 95.08); SD Analog 95.11, SD HDMI 103.70, SD Dual 98.05 MHz.
  6382-6394 / 7680 LC (83 %, ~180 LC fewer than v0.1.2: the beat's logic is gone), 13 / 32 EBR. iceunpack OK x 6.
  `out/rev_b/ron/heirloom/pupil.vmprog`. Not simulated (build-first; Ron checks on hardware). **Not flashed.**

## v0.2.1: HD HDMI re-seed for the hsync red lines + HW-captured defaults (2026-11-20)

- **Ron (HW, v0.2.0):** "pupil has those red horizontal lines associated with hsync" (his name for the HD HDMI
  placement artifact, memory `hdmi_tx_placement_skew`). The shipped HD HDMI bitstream (router1 s1, re-routed
  byte-identical) put the ADV7513 HSYNC at **−2.35 ns** (data ≤ −3.27) against the forwarded TX clock: a negative
  HSYNC, like abacus v0.6.3 (−1.45, red and blue lines).
- **Screen:** hd_hdmi router1 + router2 seeds 1–60 (94 timing-clean), hd_analog router1 s1–16, hd_dual router1 s1–12
  (`.heirloom_work/rb_pupil/skew/`: synth.sh, route.sh, pick.py, screen1/2.txt). In the +1.0..+2.3 window with every
  data bit ≤ −0.6: router1 s29 (+1.47 / −0.73), router2 s50 (+1.29 / −0.62), router1 s23 (+1.23 / −0.60), router1
  s45 (+1.07 / −0.97).
- **Shipped:** HD HDMI = **router1 seed 29** (+1.47 / ≤ −0.73, 94.94 MHz), installed by hand into
  build/.../rev_b/bitstreams/ and repacked (`vmprog_pack.py --no-sign`). The other five bitstreams are v0.2.0's seed-1
  builds: HD Analog s1 (HDMI −0.86 / ≤ −1.21) and HD Dual s1 (−2.50 / ≤ −1.65) meet the rules used for those configs.
  **Spare** for an A/B if lines remain: `.heirloom_work/rb_pupil/skew/pupil_v021_spare_hdhdmi_router2_s29.vmprog`, the
  same except HD HDMI = router2 s29 (+3.35 / ≤ −0.57, 81.20 MHz: near beacon's HW-clean +3.09 / −0.69). The old
  hd_hdmi.bin is kept as `skew/hd_hdmi_shipped_r1s1.bin`, the v0.2.0 vmprog as `rb_pupil/pupil_v020.vmprog`.
  A full `build_programs.sh` rebuild would re-route HD HDMI on seed 1 and bring the lines back: rebuild with
  `SEED=29` for hd_hdmi, or re-install the bitstream.
- **Defaults** (Ron dialled in on v0.2.0, `./capture_preset.sh --defaults`): K1 Eyes 759, K2 Squint 267, K3 Rings 1023,
  K4 Bricks 0 (clean rings), K5 Tilt 340, P12 Dilate 451 (K6 Bend and S7-S11 unchanged). Presets unchanged.

## v0.2.2: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`1920 - x` became `x - 1920`); the simulator
is correct, so every render showed the intended picture while the hardware did not.

- **Fixed: line 772** (pixel stage 2), `p2_xf <= 1920 - signed(resize(b_am, 14))` -> `to_signed(1920, 14) - ...`:
  the S9 Twin folded x.
- **On hardware before / after.** With S9 Twin On the fold was negated (`|x - 1920| - 1920`), so the distance to the
  eye centre was ≥ 1200 half-pixels everywhere: the eye columns sat off-screen beyond the left and right edges, and at
  most the outer moire rings showed near the edges. Now Twin shows what the sims always did: two mirrored eye columns
  gazing at each other, centred 600 px either side of the screen centre. Twin Off (the default) is unchanged.
- **Remaps:** none (a switch; the default is Off).
- **Left as is:** no other audited site in this program.
- **Build** v0.2.2, router1:
  - HD Analog 88.59 (**seed 2**), SD Analog 99.21, HD HDMI 85.82 (**seed 2**), SD HDMI 88.26, HD Dual 97.21, SD Dual
    95.68 MHz (**seed 2**); worst HD 85.82. Seed 1 stalled router1 into the 900 s timeout on those three, under a
    machine load of ~70. dupin finds 5 pre-existing sign-bit duplicate-input LUTs (output filter p12_du / dv / dy)
    and no CARRY I0 == I1.
  - 6392-6415 / 7680 LC (83 %), 13 / 32 EBR. iceunpack OK x6.
  - Backups of v0.2.1 (vhd, toml, vmprog): `.heirloom_work/rb_pupil/pupil_v021.*`.
- **Skew** (`QUICK=1 .heirloom_work/skew_tools/screen.sh pupil`, r1 + r2 s1-40, extended to s41-60: 98 HD HDMI
  timing-clean, 24 route timeouts, 1 in the window):
  - The builder's HD HDMI router1 s2 was **+0.12 / ≤ −1.03** (positive but sub-window, the danger zone). The v0.2.1
    pick, router1 s29, lands at +0.16 / ≤ −1.53 on the new netlist, so it could not be kept.
  - Installed HD HDMI **router2 s35 (+1.13 / ≤ −0.61, 77.46 MHz)**, the only placement in the window (thin on both
    margins).
  - HD Analog router1 s2 (−0.50 / ≤ −1.12) and HD Dual router1 s1 (−1.37 / ≤ −0.83) kept.
  - Spare: HD HDMI router1 s35 (+1.33 / ≤ −0.51, 89.81 MHz; data just outside the rule), at
    `.heirloom_work/rb_pupil/skew/pupil_v022_spare_hd_hdmi_router1_s35.vmprog`.
  - The v0.2.1 screen and its spare moved to `.heirloom_work/rb_pupil/skew_v021/`. The replaced builder bin is in
    `.heirloom_work/rb_pupil/skew/hd_hdmi_replaced_*.bin`.
  - **A full rebuild re-routes the builder seeds.** After a rebuild of the same netlist run
    `.heirloom_work/skew_tools/install.sh pupil router2 35` (router2 may not reproduce: re-screen if unsure).
- Not flashed.
