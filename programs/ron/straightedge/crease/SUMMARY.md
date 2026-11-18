# Crease (Straightedge)

**v0.1.1 built 2026-11-17, not flashed.** Flash `out/rev_b/ron/straightedge/crease.vmprog`.

## What it draws

A striped sheet of paper folded flat along straight creases: Bridget Riley's zigzag paintings ("Descending",
1965), Frank Stella's notched and concentric stripe canvases, herringbone parquet. Straight parallel stripes in
flat grey steps (black, dark, light, white, then black again: a sawtooth, so every stripe has a direction) lie on
a sheet that has been folded up to three times. Each fold reflects the half of the sheet beyond its crease, so the
stripes kink into sharp chevrons at every crease and stay continuous across it. Luma only (U = V = 512 on every
pixel); every edge on the screen is a straight line.

    fold k:   p' = p - 2 n_k max(0, n_k . p - c_k)        k = 1 .. Folds, fold 1 first, p measured from the pivot
    phase  =  d(Angle) . p_folded / (4 bands) + Drift      one cycle = four bands

With the fold decisions (s1 s2 s3) as a region code the folded point is M p + t, so the phase of each of the eight
regions is an affine function of the screen and each crease test is a half plane with a constant unit normal.
The crease itself is drawn as nothing: the stripes just turn there, exactly. Crease normals -25, 50 and 110
degrees, passing -80, 120 and 200 virtual px from the pivot.

At the default (three folds, Angle 50 %) the stripes close into nested seven-sided polygons round the folds, a
little left of centre, with open chevrons running off to the right.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the pivot the creases are arranged round, +-704 virtual px from the centre (always on screen); the whole figure moves with it. Glided |
| K2 | Y | the same, +-480 virtual px. Glided |
| K3 | Scale | band width 12 .. 300 virtual px, exponential (default 60). Creases: the stripes scale about the pivot, the creases stay. Pleat: the whole pattern zooms. Glided |
| K4 | Folds | 1 / 2 / 3 creases (default 3), stepped with hysteresis. Pleat: 3 / 2 / 1 stripe cycles per strip (wide, medium, tight pleats) |
| K5 | Drift | the stripes stream along the sheet and round every bend; the creases stand still. Bipolar, centre +-24 counts = still, 2^-12 .. 2^-4 cycle per field (default 680: about one band in 3 s) |
| K6 | Camera | how far the camera (or Osc) shifts the band weight of each 32 x 32 px block: 4 flat levels, the light bands widen and the dark ones thin, never past the cap (every cycle keeps all its tones). Default 100 %. Glided |
| S7 | Glide | Off = the knob value of this field, no easing at all / On = 1/16 of the gap per field, never slower than one count, landing exactly; the Drift speed 1/8 (default On) |
| S8 | Fold | Creases (the free folds) / Pleat (an endless accordion of horizontal creases: regular zigzag chevrons) |
| S9 | Tone | Grey (four steps) / Ink (black and white bands) |
| S10 | Weight | Bold (equal bands) / Fine (a thin line at the trailing edge of each band, 1/6 of the band and never under 2 pixels; Grey: lines of dark, light, white, light) |
| S11 | Source | Video / Osc |
| P12 | Angle | the headline: the direction of the stripes on the sheet, a half turn. 0 % = chevrons radiating from the folds like a star; 50 % = the stripes closed into nested polygons (default); 100 % = the star again with the tones and the stream reversed. In between every kink opens and closes at its own stage, like a jaw. Pleat: right-angle herringbone at 0, 50 and 100 %, straight vertical stripes near 26 %, mirrored horizontal seams near 76 %. Glided |

Every glided knob has a +-1-count hold against pot jitter (the two ends always get through).

## Motion

One phase accumulator (22 bits = one cycle of four bands) advances once per field in the vblank zone and is
added to the seed of every region, so the stripes stream along the folded sheet: outward or inward through the
nested polygons, along the chevron arms and round every kink, while the creases stand still. It wraps exactly one
cycle, so it never jumps. Speed: +-24 counts about the centre = still, then exponential from 2^-12 to 2^-4 cycle
per field (Hypnos's decode; the top is capped at 2^-4 because 2^-2 would be one whole band per field, which
strobes). Only Drift at centre stops it. Glide On eases the speed (1/8 of the gap per field), never the phase.

## Camera

The camera is never shown. Its luma (minus black) goes through a per-field gain table (K6), a horizontal box as
wide as a block (32 px at 1080, 16 px at 720p and SD), is sampled at the last pixel of every 32 x 32 virtual px
block and runs through a vertical one-pole per block column; the result is quantised to 4 levels,
level = floor(6 * luma * K6), and written to one bank of a 64 x 64 x 2-bit grid on every line (the last line of
a block row stays). The next field reads the other bank at the pixel's own block, so the level is constant across
the block and one field old. It drives the band weight (the spec's second choice), capped so that no luma level
erases a stripe:

- Grey Bold: the black, dark and light bands are each 1, 5/6, 2/3 or 1/2 band wide (level 0 .. 3) and white
  takes the rest of the four-band cycle (1 .. 2.5 bands). Every cycle keeps all four tones; the thinnest band
  is half a band = 1/8 of the period (6 virtual px at the smallest Scale).
- Ink Bold: the white band is 1, 1.25, 1.5 or 1.75 bands of the two; the black line is never under 1/4 band
  (1/8 of the period, 3 virtual px at the smallest Scale).
- Fine: the line is 1/6, 1/3, 1/2 or 3/4 of its band; the gap is never under 1/4 band.

Black input, or K6 = 0, is exactly the generated picture (level 0 is the clean figure, so the excursion is
referenced to black, not to mid grey). 25 %: only the highlights (luma above 2/3) reach level 1. 50 %: levels
at 1/3, 2/3 and white. 100 % (default): levels at 1/6, 1/3 and 1/2. The stripes and every kink stay on the
whole frame; the bright parts of the picture show wide white bands with thin dark lines, the dark subject equal
bands (bear.png: the bear is the heavy-striped patch, clearest with Fine). Source = Osc puts the shell's
drifting 640 x 540 triangle lattice at the same point at twice the camera's gain (level 3 from a quarter of the
lattice's peak): the whole figure goes light with stepped patches of heavier stripes at the lattice's troughs.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): the hold + glide of five knobs (five clocks each), the
  Drift speed decode and phase step, then a sequencer of 130 slots of 20 clocks on ONE serial 16-step multiplier
  (22 x 16 bits): the stripe scale (2^(i/256) ROM), the pivot as a pixel / line index, the angle; for each of the
  eight regions its direction (fold constant +- Angle), sine and cosine (1024-point quarter-wave ROM, linearly
  interpolated), the phase step per pixel and per line, the fold offset and the seed; for each of the seven
  crease tests its line step, distance and seed. Every DDA gets (pixel step, line wrap, field seed) in a small
  table. The gain table is refilled by an accumulator. About 3,700 zone clocks.
- **Per line**: no multiplier. 15 DDAs (8 phases, 22 bits, cycles Q22; 7 crease tests, 25 bits, pixel widths Q12),
  each with its own increment register: it adds the pixel step on every clock of the line window (W + 8 steps),
  then once more with the wrap (line step - steps x pixel step) parked in that register from the table.
- **Per pixel** (13 stages, no multiplier): the three fold decisions from the sign bits (fold 2 picks one of two
  tests, fold 3 one of four); the region's phase (8 : 1); the region's footprint |dphase/dx| + |dphase/dy| and the
  block's camera level from block RAM; the band / line engine (nearest edge, this side's tone, the tone across);
  coverage = (8 + min(8, 16 d / w)) / 16 by a 4-step divider; the blend in sixteenths of a tone step; luma.
- **Pleat** uses the same hardware: crease test 0 becomes a strip counter running on (phase 0 - phase 1) / 2N,
  and its half-pair bit picks region 0 or 1. Because a strip is a whole number N of stripe cycles, the two
  regions need no per-strip offset and the stripes meet exactly at every crease.
- **Output**: luma [1 2 1]/4, luma 13/16 toward black (white = 777), blanking gate. Generated avid (the shell's
  parity-locked start), one latency (24) in every mode, virtual 1080-line coordinates from the timing ID.

Chassis: Kinetic riley by way of Hardline dazzle: [A] timing / zone / width / generated avid, the [B] hold +
glide, the serial multiplier and DDA loader of [C] / [E], the [D] Osc lattice (with dazzle's restart fix), gain
table and box, and the luma half of [F]. The wave engine, palettes and every chroma path are gone.

## Status

- v0.1.1 built 2026-11-17 through the shared queue, **router2** (the builder default). Not flashed.
- Post-route Fmax: HD Analog 79.25 MHz (seed 1), HD HDMI 76.45 (seed 1), HD Dual 79.10 (**seed 3**; seeds 1 and 2
  missed timing) (HD needs 74.25); SD Analog 81.71, SD HDMI 71.62, SD Dual 73.17 (all seed 1; SD needs 27).
- 6,684 .. 6,716 / 7,680 LC (87 %), 15 / 32 EBR. Six bitstreams pass iceunpack. No `emulate_transparency`, no
  carry self-adds (dupin: 0 on the v0.1.0 netlist; the v0.1.1 change adds no sum of a signal with itself).
- Rebuild: `MAKE_TIMEOUT=900 ./build_set.sh straightedge crease`. The seed spread is several MHz (v0.1.0 routed
  HD HDMI at 75.15 and 76.90, HD Dual at 75.64 and 81.33 on two builds), and router2 once stalled on SD HDMI
  seed 1 (stuck at overused = 2 until the timeout, then seed 2 routed).
- v0.1.0 (same day, superseded): the camera filled the dark bands by whole tone steps until the block was white,
  which erased the stripes wherever the picture was bright. v0.1.1 changes the camera law only (and the Osc gain).
- The first RTL packed 7,155 LC (93 %) and stalled router2 on HD Analog. The diet: phase DDAs 24 -> 22 bits, crease
  tests 28 -> 25 bits (Q12), a 38-bit multiplier, the fold constants read straight from their ROM instead of four
  latches, the Pleat strip counter taken from one region's line step, a 12-bit one-pole.
- **Model = RTL**: pixel diff 0 on every frame rendered; the list is under "Verification".

## Verification

GHDL renders against the integer model `imodel.py` (Glide Off and Drift still unless noted). The GHDL frame is the
model's shifted 4 columns right with 4 blank columns at the left (the shell's generated-avid start).

On the v0.1.1 sources (the built ones):
- Full 720p60 frame (918,720 px compared): 0 different at the defaults with a black input, 0 with bear.png as
  the camera at K6 100 %, 0 with Source = Osc.
- NTSC (720 x 243 per field): 0 of 173,988 on each field; the same picture (the centre 4:3 of the 16:9 one).
- 720p quarter rasters (228,960 px), 0 different on each: default; bear; Osc; Fine + bear at K6 59 %; Ink + bear;
  Ink + Fine + bear with 2 folds and Scale 29 %; Pleat + bear with Scale and Angle moved; Pleat + Osc at K6 39 %.
- Glide On at the power-up defaults: the fourth field matches the model with the Drift speed easing in
  1/8 steps (phase 4320), 0 different.

On the v0.1.0 sources (same geometry, sequencer and DDAs; v0.1.1 changed the tone stages 4 .. 6 and the Osc
gain only):
- 720p quarter rasters, 0 different on each: X, Y, Scale and Angle moved with 2 folds; 1 fold + Ink; Scale at
  both ends; Angle 100 %; Pleat at each Folds setting; Pleat + Fine; Pleat + Ink + Fine.
- Motion: at the default Drift the captures after 4 and after 8 fields match the model with the phase advanced
  4 x 5184 and 8 x 5184 (2^22 = one cycle), 0 different: the travel and its direction are the model's.
- Not rendered: the 1080 rasters, 1080i, PAL, 480p / 576p.
- Renders: `.straightedge_work/rb_crease/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (P12 travel),
  `motion.png` (Drift), `camera.png` (K6 = 25 / 50 / 100 %), `zoom.png` (edges at 4x), all from the integer model
  except the RTL panels of cmp_final.

## Cut / deviations

- **Camera target: band weight in four flat levels, not a phase slip**, referenced to black and capped. Tried on
  bear.png in the prototype: the spec's per-block phase slip, weight in quarter-band steps, an octave split of
  the stripes and a per-block turn to another fold region all left the bear unreadable at the default Scale (the
  stripes are larger than its features). v0.1.0 filled the dark bands until the block was white, which read as
  a bear but washed the figure out; v0.1.1 keeps every tone in every cycle, so the subject reads only as heavier
  and lighter stripes (weakly in Grey Bold at the default Scale, better with Fine or a smaller Scale).
- **Pleat pitch is counted in stripe cycles** (3 / 2 / 1 per strip on K4), not set in pixels. It therefore follows
  Scale and opens up as Angle nears "straight through" (strip width = N cycles / (2 |cos|)); in return every
  chevron is identical, the stripes meet exactly at every crease, and Pleat costs no extra hardware. K4 is
  stepped in both modes.
- **Drift tops out at 2^-4 cycle per field**, not 2^-2 (a band per field strobes).
- **X / Y reach +-704 / +-480 virtual px**, so the pivot never leaves the screen (SD shows the centre 1440).
- The camera blocks start at the left edge of the raster in every mode, so at SD the block grid sits half a
  block from where it lies at HD. Block borders fall on whole pixels (21 / 22 px wide at 720p).
- At 480i / 576i with Scale at minimum the Fine lines (widened to two field lines) fill their bands.
- The crease directions are rounded to Q12 (the crease can sit up to a quarter pixel from where two regions'
  stripes meet, at the far end of a line).
- `field_n` is passed through one register (the core ties it high on every input).

## Not sure of on hardware

- Nothing on this chassis has been flashed yet. The generated avid, zone and width logic are the heirloom
  shell's, which is hardware-validated.
- The picture is sharp, full-swing luma edges in motion: the [1 2 1] + 13/16 conditioning is built in, but if
  HD HDMI bars show, Scale at minimum with Ink is the first place to look.
- Camera noise near a level threshold will make single blocks flicker between two levels (there is no hysteresis
  in time; the box and the one-pole average about a thousand pixels per block). With the weight law a level
  step moves the band edges inside the block, so such flicker shows as edges jumping by 1/6 band.
- The Folds knob's hysteresis band (24 counts) and the +-1-count hold are untested against real pot noise.
