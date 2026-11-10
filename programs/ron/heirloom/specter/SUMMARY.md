# Specter — a green ghost dripping through a haze of hanging curtains onto a blue floor

**v0.1.0 built 2026-11-10, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20230904_01_56_58_t0p5.png` (clip WIN_20230904_01_56_58, 1.0 s, frame at 0.5 s; the same patch in
clip WIN_20230904_01_55_47 at 0.5 s and 4.5 s). A green ghost figure stands in the middle of the frame: a striped
head, outstretched arms with a raised left hand, and legs that stream down to the bottom edge. Around it hangs a
haze of green curtains: an S-shaped arc and parallel streaks on the left, a slanted shelf on the right. Under the
haze is a dark band, and a blue floor glows in the bottom corners. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_specter/` (`notes.md`, `meas1..8.py`, `fit.py`, `fit2.py`, `fitfig.py`,
`fit_mask.py`, `proto.py`, `imodel.py`, `isheet.py`, `thincheck.py`, `score.py`, `syn/`).
- **Palette.** Two pure ramps over a faint green black (k = 12 on the whole frame). Black (6,15,6) covers 23 %,
  (5,26,4) 19 %. Then comes green (0, G, 0) up to G 245; R and B fall to 0 as G rises. Blue (5,12,B) reaches B 133
  here; the other clip's moments saturate it at (4,10,250).
- **The ghost** is the camera subject, Ron in front of the lens:
  - head x 870–1150, y 190–420, peak level 147 at (1000, 280);
  - neck y 420–500, with torn bright horizontal dashes;
  - arms band x 740–1300, y 530–760, with a sharp top edge at y 555, level up to 222, a dark chest spot at
    (1010, 640) and the left hand raised at x 760–830;
  - legs x 880–1120, streaming from y 760 to the bottom; the level falls from 190 to 80 (0.3 per line) inside a
    soft V-shaped aura (x 760–1260 at y 820, x 880–1150 at y 1000).
- **The comb.** Vertical stripes on the head and arms:
  - period 39–40 px (autocorrelation peaks at 40 / 76 / 115 px), amplitude ±7–15 % of the level;
  - slanted −0.11 px per line, phase 25 px at y 0;
  - screen-locked: the frames at 0.0 s and 0.5 s show the same comb phase on the same rows while the figure moved
    110 lines. So it is an unlocked horizontal oscillator multiplying the camera key;
  - absent on the legs (their streaks are 63–69 px).
- **The haze.**
  - A dim layer (level 15–20) over y 40–300, with a bright S-curve at x 240–440 and parallel arcs.
  - Left curtains: x 0–700, y 350–800, level 60–100, streaks at 82 / 160 px.
  - Right shelf: x 1250–1920, with a slanted top edge from (1240, 590) up to (1920, 480), level 80–100, streaks
    65–108 px.
  - Torn diagonal streaks in the top right.
  - The bottom edge sits at y ≈ 805 on average (std 66 px). Where the curtains are bright it hangs in pointed tips,
    which reach y 880–900 at x 380–700.
- **The dark band** spans y 840–990 (G 11–14), except for the legs and their aura.
- **The blue floor** covers only the last ~90 lines, at x < 620 and x > 1480. B rises from 5 to 75 between y 985
  and y 1060 (~1.2 per line), with flame bumps 50–160 px wide. The ghost's sub-key aura cuts it out in the middle.
- **Tears.** Rows 340–480 (the lower head and the neck) are shifted ±1–5 px against their neighbours.
- **Mechanism.** The camera goes through a heavy vertical smear and a luma key coloured green, times the comb
  oscillator. A second, inverted key with a vertical ramp gives the blue at the bottom, and a vertical ramp keys the
  haze off at y ≈ 805. Across the clips Ron's knob moves the green / blue split.
- **Fits.**
  - The envelope (G boxed over 39 px, minus 13) is split in two. The haze texture is inpainted under the figure,
    window-normalised, mirrored below y 745 and wrapped cyclically; it is sampled at 32 × 32 px cell centres
    (64 × 32 cells) and pre-sharpened in x.
  - The ghost is the residual over the rendered haze inside a feathered mask: 32 × 32 cells of 19 × 22 px over
    x 704–1312, y 152–856. Its last row continues down as the legs.
  - The bottom key is 1 − ss((y − yb + 145 − 1.22 Tv) · 13/16 / 64), with yb = 815 at the default.
  - Mean |ΔG| against the still (15-px blurred G): float proto 5.3, integer model 6.1, GHDL render 6.2.

## Motion in the clip
- The clip is only 1.0 s long. The strip's frames 0–2 are clamped to 0.0 s, frame 3 is the still, and frames 4–7
  are past the end.
- From 0.0 s to 0.5 s (cross-correlation of crops at 960 × 540) everything slides **down**:
  - head (+12, +55) and arms (−6, +57): the ghost moves ~110 lines;
  - left curtains (+2, +28) and right shelf (−10, +46): the haze moves 56–92 lines;
  - the haze's bottom edge moves 40 lines (Ron's knob).
- The same patch in clip 01_55_47 (12 s):
  - the curtains drift down 16–32 px and left 8–12 px per 0.5 s;
  - the green band's bottom moves 546 → 600 and the blue top 864 → 910 over 1.5 s (the split moving);
  - the bright camera figure sweeps 300 px per 0.5 s.
- **In the program** (K4 at its default 768 = "1.0"):
  - the ghost bobs ±110 lines over 3.2 s (216 lines / s at the zero crossing, 93 lines in the first 0.5 s);
  - the haze texture rolls down 96 lines / s and drifts left 20 px / s;
  - the floor flames drift;
  - the comb stays screen-locked;
  - Calm tears re-draw every 4 fields;
  - P12 is Ron's split knob.
  - The integer model moves the ghost 84 lines and the curtains 46 lines in 0.5 s. In the GHDL renders (720p),
    fields 3 → 9 moved the ghost 14 lines (21 virtual) and the curtains 6 (9 virtual), as the rates predict.

## What it does
1. **Coordinates.** x in half virtual px (2 / 3 / 4 per pixel at 1080 / 720p / SD), virtual 1080 lines. The beam
   starts 8 pixels early for the output filter. A per-line tear offset shifts the whole line's x: Calm = 8-line
   bands ±8 px in the neck zone (260–120 ghost lines above its centre); Torn = 32-line bands across the frame,
   60 % of them slipping ±128 px.
2. **Haze.** The line engine interpolates two map rows (rolling: row = y − 72 − roll) times the Haze gain into a
   64-entry profile of pairs {v, clip(dv)}. The pixel reads the pair at (x + drift − 16 px + bend) / 32 px and lerps
   it to Tv. The bottom key's index is a13 − 1.22 Tv, where a13 = (y − yb + 145) · k13 / 16 per line, and the brighter
   curtains hang lower. w = min(top ramp, 255 − ss(index)), and haze = Tv · w.
3. **Ghost.** Two map rows at (y − fcy) / scale, the legs' fade folded into the row weights, go into a 32-entry
   profile. The pixel phase is u = (x − 1008 px) / (19 · scale) + 15.5 cells (Twin: |x − 960| − 430, running down
   to the centre and then up), plus the bend · 2048. Lerp gives F, and F · (256 + depth · tri(comb phase) / 16) / 256
   is the combed ghost, plus the camera glow.
4. **Colour.** green = haze + ghost (Shadow: 160 − that). blue = 1.25 (y − yf) + flames − aura − 2 · ghost, where the
   aura is 150 − |u − 16.9 cells| · 0.094 and follows the ghost's scale and Twin. Both feed palette ROMs, summed in
   YUV.
5. **Output.** U and V through a [1 2 2 2 2 2 2 2 1]/16 box: 8-bit chroma, an 8-sample block-RAM ring, snapping back
   to the palette black after 8 blank samples. Y goes through [1 2 1]/4 at the same group delay. Then the blanking
   gate. The shell's luma 13/16 is folded into the palette: its brightest green is Y 558, under the ~777 cap.

## Controls
| Control | Element it drives |
|---|---|
| K1 Ghost | the ghost's size: 0.55× (0) .. the still (512) .. 1.9× (1023), linear in 1/scale; the tear zone, aura and comb fade scale with it |
| K2 Comb | the ghost's vertical stripes, V-shaped: wide deep bars, 110 px at ±87 % (0) .. the still's faint 39 px comb at ±9 % (512) .. fine deep bars, 15 px at ±87 % (1023); slant −0.11 px / line kept |
| K3 Haze | the curtains' level: none, a lone ghost on black (0) .. the still (512) .. 2.2×, brighter and hanging lower (1023) |
| K4 Drift | bipolar motion speed, still in the centre ±8: the ghost's bob, the haze roll and drift, the floor flames; 768 = the clip, 1023 = 4× faster, left = reversed (the ghost rises, the haze rolls up) |
| K5 Drip | the legs and the curtains' tips: no legs, a floating head and arms (0) .. the still (512) .. full legs to the floor while the curtains drip far down, key slope 13/16 → 3/16 (1023) |
| K6 Etch | how far the camera (or the Osc) bends the haze and the ghost (unipolar, rightward: 56 px at white at the default, 283 px at full) and glows through them (luma above 140); 200 = a gentle bend |
| S7 Twin | one ghost / a mirrored pair at 430 px either side of the centre, each with its own aura in the floor |
| S8 Tear | Calm (the still's torn bands at the neck) / Torn (32-line bands slip up to ±128 px across the whole frame, re-drawn every 2 fields) |
| S9 Ghost | Glow (the still) / Shadow: the key inverted (160 − level): the dark floods mid-green and the ghost turns into a black cut-out |
| S10 Palette | Spectre (green / blue, the still) / Ember (amber haze and ghost, violet floor) |
| S11 Source | Video / Osc |
| P12 Tide | the haze's bottom and the blue floor together (Ron's knob in the clips): haze down to the floor and no blue (0) .. the still, haze edge 815, blue from 1010 (512) .. a blue sea from y ~585 with tall flames, the green confined to a top band, the ghost's legs and aura cutting a dark pool (100 %) |

## Video
- **Video mode (default).** The camera's luma, (Y >> 2) − 16, goes into a half-resolution line buffer, read 5 pixels
  ahead so its effect meets the beam with its own pixel. It does two things:
  - it **bends** the haze and ghost lookups rightward by luma × Etch (default up to 56 px at white);
  - it **glows**: luma above 140 adds green to the ghost layer (default up to ~40 levels).
  A black input changes nothing, so with no camera Video mode is the still. With `photos/selfie.jpg` at the default
  the ghost, curtains and floor stay readable, bent around faint outlines of the photo. At full Etch the photo takes
  over in green (not pale).
- **Osc mode.** The shell's drifting triangle lattice (640 × 540 virtual px) goes into the same path at 2.5× the
  camera's gain (bend up to ~137 px at the default). The curtains and ghost bend into zig-zags and the lattice's
  diamonds glow green. Every video-path control stays live.

## Engine
- **Vblank sequencer.**
  - The shared glide covers K1, K2, K3, K5, K6 and P12.
  - 15 slots (k = 0–15, k = 5 unused) on a 32-clock grid. Decode at zc(4..0) = 30, store at zc(4..0) = 18, so no
    wide zc compares. They feed the serial multiplier, which is shared with the line engine:
    - speed²;
    - the bob phase, the haze roll and drift, and the flame phase;
    - the bob (sine ROM);
    - ku;
    - the comb slant;
    - the comb depth;
    - the haze gain;
    - the tide yb and yf;
    - the legs' fade rate and start;
    - the bend gain.
  - Single-clock steps compute the linear 1/scale, the linear comb step (298 .. 840 .. 2184), the glow threshold,
    the Drip key slope and the tear seed.
- **Line engine.** It is kicked at each line start (and at zc 900 for line 0) and computes the NEXT line:
  - 7 serial products: yr, the ghost row, the haze row weights, the legs fade, the ghost row weights, u0, and the
    key term;
  - shift-add terms: the floor ramp, the top ramp, the comb depth (halving every 30 ghost lines below the
    shoulders), the tear offset (from yr) and the comb phase;
  - then it builds both profiles: 192 ROM reads with one 4×9 + 4×9 multiplier, done by ~300 clocks, so it fits
    SD's 858-clock line.
- **Pixel path (17 stages).**
  - 1: beam.
  - 2: profile addresses, comb triangle, aura distance, glow.
  - 3: EBR, comb product.
  - 4: fabric copy, comb multiplier.
  - 5: lerp products, aura.
  - 6: Tv, F.
  - 7: key index, combed ghost.
  - 8: window, ghost + glow.
  - 9: haze.
  - 10: green, floor.
  - 11: palette addresses.
  - 12: ROM.
  - 13: fabric copy.
  - 14: YUV sum and black outside avid.
  - 15–16: chroma box and luma [1 2 1].
  - 17: gate.
- **Block RAM (19).** Haze map 2048 × 8 (4), ghost map 1024 × 8 (2), green palette 512 × 32 (4), blue palette
  256 × 32 (2), sine 256 × 8 (1), camera line buffer 1024 × 8 (2), profiles 128 × 16 + 64 × 16 (2), chroma ring
  (1), sync ring (1). Every collision-free RAM carries `no_rw_check`.
- **Contracts.**
  - generated avid;
  - one latency (C_LAT 16) in every mode; the syncs go through a block-RAM ring of exactly C_LAT;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr);
  - the chroma box resyncs to black after 8 blank samples;
  - EBR outputs get a fabric copy before arithmetic;
  - every multiply has a stage of its own;
  - dupin on the pre-build netlist: the two duplicate-input LUTs it found were removed; no carry cell had the same
    signal on both inputs. The build's own netlist was not re-checked.
- **Verified.**
  - `imodel.py` mirrors the arithmetic, and the GHDL renders (720p60, decimation 1: black input, Osc, selfie, and
    a second capture for motion) match it.
  - `thincheck.py`: no single-row luma lines of 24 px or more at 1080p (|ΔY| > 40 on both sides) for the defaults,
    P12 0 / 100, Torn, Twin, Shadow, Drip 0 / 1023, Ghost 0 / 1023, Comb 1023, Haze 1023 and Osc at three bob phases.
- **Fit history** (the SDK core alone packs to 1,145 LC). The first netlist was 8,569 LC full (111 %). It came
  down to ~7,190 through these changes:
  - the camera look-ahead, so no deep u / x delay lines;
  - a single load site for the multiplier, and then one multiplier shared by the sequencer and the line engine;
  - a single 8-bit chroma box in a block-RAM ring, and luma [1 2 1];
  - the trail buffer removed;
  - the slots re-gridded;
  - the sync ring;
  - linear knob laws in place of exp2;
  - the tear zone and comb fade taken from the ghost row;
  - one flame accumulator.

## Status
- **v0.1.0 built 2026-11-10 through the shared queue with ROUTER1; all six configs on seed 1. Not flashed.**
  - HD Analog 78.15 MHz (7,192 LC), SD Analog 86.01 (7,204), HD HDMI 84.03 (7,187), SD HDMI 84.98 (7,192),
    HD Dual 84.38 (7,177), SD Dual 81.82 MHz (7,202); 19 / 32 EBR. The builder's figure is the last post-route
    report; seed 1 was accepted with no retries. iceunpack OK ×6.
  - Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh specter`.
  - Output: `out/rev_b/ron/heirloom/specter.vmprog`.
  - LC is 93.4–93.8 %, over the nominal 92 % budget but under the 94 % closure guideline. Timing has margin
    (worst HD 78.15 MHz).

## Deviations
- **The ghost is a fixed silhouette.** The clip's camera figure is stored as a map. It bobs but does not change
  pose, and its stripes are the comb.
- **Map resolution.** Haze cells are 32 px and ghost cells 19 × 22 px. The top-right diagonal tear streaks, the
  neck's bright dashes and the S-curve's sharpness are softened. The mean |ΔG| is ~6 on a 15-px blurred G.
- **No trail buffer** (cut for budget). The clip's feedback smear on the camera is not reproduced: the camera bends
  and glows but does not streak down. The legs do not taper.
- **Floor flames** are two phase-locked triangles (233 and 93 px), more regular than the clip's bumps.
- **Comb.** 32 depth levels; its origin phase is exact only at the default period. Below the shoulders it fades in
  30-line halvings instead of a smooth ramp.
- **The bend is unipolar** (rightward), so that a black input leaves the still untouched. A bright camera shifts
  the whole drawing a little to the right.
- **Camera look-ahead.** The first 5 pixels of each line use the previous line's camera tail. The left ~8 px are
  dark (filter warm-up), as in the shell.
- **Line width** comes from the timing ID (1920 / 1280 / 720), not measured.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3: the outer curtains and part of
  the blue corners fall outside (virtual x 240–1680).
