# Temple — an orange sunset temple: cyan flames, maroon shapes, orange spindles in a pink band

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20230816_00_32_31_t5p0.png` (clip WIN_20230816_00_32_31 at 5.0 s; 12.0 s clip). A mirror-symmetric
orange/yellow/blue "temple": an orange and yellow sky with four soft cyan flame columns, maroon shapes with black
rims (a ring at the top, a crooked centre block, two small blobs, two lens ellipses and a band), a salmon-pink band
holding four orange spindles with yellow cores, arrow wings and torn white rims, a stack of wavy purple / pink /
red / yellow bands with blue rims, a jagged white rim, and a cyan floor with darker columns and a yellow half-dome.
Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_temple/` (`notes.md`; `fit_*.py` = the field fits; `proto.py` = the float proto;
`gen_tables.py` = every table; `imodel.py` = the integer model; `sheet.py` = the control sheet).
- **Mirror** about x ≈ 960: flip-correlation axis 954.5 (rows 0-360), 969.5 (360-720), 954.5 (720-1070), error
  24-31 mean |RGB| (an analog fold, not a perfect mirror). The spindles sit at x 538 / 768 (left) and 1397 / 1175
  (right), i.e. 430 / 203 px from an axis at 968.
- **Channels.** R is ~255 almost everywhere (dark only in the soft cyan flames and the maroon shapes); G is the
  smooth "heat" field (yellow 214 .. red 25); B is a key: 0 in the orange family, ~112 in the pink/purple family,
  ~250 in the cyan family. Pink (250,101,112) = orange (250,100,0) + B 112; purple (124,11,111) = maroon + B.
  k-means (k = 16): pink 16.4 %, cyan (7,204,251) 11.3 %, orange (249,100,3) 10.1 %, red-orange (251,47,2) 10.0 %,
  yellow-orange (248,163,5) 9.0 %, sky (12,167,248) 7.5 %, maroon (100,17,7) 4.7 %, white (223,205,197) 4.4 %,
  purple (141,9,114) 3.7 %, black (24,13,10) 2.2 %, blue rim (34,57,156) 1.8 %.
- **Sky** (rows 0-388): orange/yellow ground (corners yellow-orange, the centre column red-orange) with four cyan
  flame columns (x 430-620 and 660-830, mirrored) tapering to V tips at y ≈ 370, horizontal edges soft over
  20-40 px. Fitted as a colouriser over S = a(y) + g(y) T(|x - 960|) with a heat-ordered palette
  (maroon - red - orange - yellow - white - cyan - blue): 16.9 mean |RGB| on the sky's own pixels.
- **Dark shapes** (maroon brightening inward, black rim 3-4 px): the ring x 916-1027 y 2-106 (teardrop), the centre
  block x 840-1112 y 232-376 (an asymmetric polygon), small blobs x 623-707 / 1238-1318 y 226-358 (stem + body),
  lens ellipses x 0-439 / 1504-1899 y 310-358 (pointed ends), and the band y 386-440 (black top rim, maroon
  80 -> 187 downward, a soft 6 px edge into pink, bulging to 466 at the centre over a half-width of 135).
- **Pink band** y 440-645, flat (250,100,112) with faint per-line streaks.
- **Spindles**: lens bodies, half-width 38 (y 400) .. 62 (500) .. 46 (600) .. 0 (655), with arrow wings at y 548
  (half-width 100, pointed tips); the inner pair is 0.70 x the outer pair. Interior G = peak(y) - k |dx|, linear at
  1.6-2.2 G per px, peak G 74 (y 392) .. 214 (548) .. 57 (640). A soft white rim (5-9 px) on the pink side, torn by
  per-line jitter.
- **Stack** y 645-830: the outer column reads pink | purple | blue rim | red | blue rim | purple | pink | purple |
  blue rim | red | orange | yellow | orange | red | dark | white rim; the bands pinch at the spindle columns and
  stretch into purple domes at the centre, with white (y ≈ 686) and cyan (≈ 756) sparkles at the columns. Modelled
  as a vertical ramp through a VCA whose gain is an H waveform (1.5 at the columns, 0.12 at the centre), into a
  colouriser that is the outer column's band profile.
- **White jagged rim** y 774-878: a gentle curve 809-832 outside the columns, sawtooth teeth over them.
- **Floor** y 830-1080: cyan with darker blue columns, an orange inverted V at the centre, a yellow half-dome
  x 870-1100 y 960-1080 (radius 124 about (985, 1088)), pale corners. Fitted like the sky: 26.2 mean |RGB|.
- **Fit**: integer model 25.3 mean |RGB| against the still (720p); GHDL RTL 26.6 (720p, black input); RTL vs
  integer model 0.83 after a 4-px alignment.
- Automated fits with free palettes (rank-1 additive, rank-2, 8-zone) kept collapsing into plaid; heat-ordered
  palettes plus explicit keyed layers are what reads as the photo.

## Motion in the clip
- Measured on the m-frames (t - 0.5 .. + 1.5 s) and the strip (t - 1.5 .. + 2.0 s). One knob-like sweep moves the
  colouriser through its palette, while the pink band and the dark shapes hold still:
  - top-left corner (x 50-350, y 30-150): (251,140,0) -> (247,168,3) -> (242,157,18) -> (218,135,53) ->
    (187,167,147): orange -> yellow -> pale, toward cyan;
  - bottom-left: (101,235,242) -> (22,213,249) -> (6,191,252) -> (52,153,224) -> (118,80,67): white -> cyan ->
    blue -> dark -> red;
  - the outer spindle's left wing tip at row 548: x 476 -> 436 -> 406 -> 364, then (t + 1.5 s) a needle runs to the
    screen edge: about 40 px per 0.5 s;
  - the top ring's upper edge: row 4 -> 4 -> 10 -> 72 -> 126 (sinking ~120 rows/s once it starts).
- The clip's other stills are other positions of the same sweep: t 0.5 s low (no pink band, small spindles),
  t 7.5 s high (needles to the edges, a red floor), t 10.5 s a yellow sky.
- **What the program does**: P12 Sun sets the sweep position and K4 Drift runs a triangle LFO on it, ±32 palette
  units over 8 s at the default (16 units/s against the clip's ~18). The sweep rotates the sky and floor palettes
  (the clip's colour cycling), grows the spindle wings by 4.5 px per unit on the centre wing rows (the needle),
  sinks the ring by 4 rows per unit and scrolls the stack's centre line by half a row per unit. The GHDL frames 3
  and 15 (Drift 1023) move the right way: the sky corner goes orange -> pale, the flame cores deepen to blue.

## What it does
1. **Columns.** xc = |x - 960| k1 from a per-pixel DDA (the beam's start is a per-line product, the step k1 xs);
   one 512 x 32 column ROM gives, per xc/2: the sky waveform T, the floor waveform T, the stack VCA gains above and
   below the stack centre line, and the white rim's row.
2. **Rows.** A 1024 x 16 row ROM (two words per 2.1 lines): the zone (sky / middle / lower), and per zone the
   colouriser offset a and VCA gain g (sky, floor), the ellipse half-width, the spindle half-width, peak and slope,
   the band row and shade, the wing weight, the pink row, the sparkle / dome kind and half-width.
3. **Sky / floor**: S = a + (g k3 T) >> 9 + camera, clamped to 0..95, + the sweep, mod 128, into a 128-entry
   palette (96 fitted entries + a 32-entry wrap: blue - indigo - black - maroon).
4. **Stack**: S = y + (e (M - 1)) >> 4 + camera / 2, e = (y - centre) k5, into the 256-entry band profile.
5. **Middle**: the band (black rim, shade, soft 3-step blend into pink), pink (four per-line variants), the
   spindles (d = min(|xc - 424|, 1.4375 |xc - 196|) + tear; body where d < hw k2 + wings, colour index
   gp - kk d; a white rim 0..7 px outside on the pink side); below pb(x) the stack.
6. **Lower**: stack above the rim, the floor below, the white rim line (3 shades), sparkles (two levels), the dome.
7. **Dark shapes** over everything (priority centre block > ring > small blobs > ellipse), depth-shaded maroon
   (index = depth / 2, a black rim at depth < 4). Off with S9.
8. A 1024-entry palette (Y, Cr, Cb), then the shell's output conditioning.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Columns | the column spacing of every repeated element (flames, spindles, stack pinches, floor columns, small blobs, ellipses): x0.5 (two spindles, wide flames) .. x1 (512, the photo) .. x2 (the pattern crowded into the middle, the outer sky stretched) |
| K2 Spindles | the spindles' size: x0.3 slivers .. the photo (512) .. x2 fat crosses touching |
| K3 Flames | the depth of the H waveform in the sky and the floor: 0 = plain gradients (no flames, no floor columns), 512 = the photo, 1023 = blazing blue flame columns and deep floor columns |
| K4 Drift | the sweep LFO's rate: 0 = still, 512 = the clip (8 s triangle, ±32 units), 1023 = 4 s |
| K5 Stack | the stack's pinch: 0 = flat horizontal bands, 512 = the photo's lenses and domes, 1023 = x4, the bands torn into deep lenses |
| K6 Camera | how far the camera's light (or the Osc lattice) bends the sky, floor and stack: 0 .. 0.43 palette units per luma step (200 = 0.1) |
| S7 Flip | the temple upright (the photo) / upside down (floor and stack on top, sky below) |
| S8 Palette | Sunset (the photo) / Ice: Cr and Cb swapped (blue sky with orange flames, a lavender band, blue spindles) |
| S9 Shadows | On (the photo) / Off: the band, the ring, the centre block, the small blobs and the ellipses disappear, the pink band reaches the sky |
| S10 Tear | Off: the still's faint line jitter (±1-4.5 px on 1 line in 8, which tears the spindle rims) / On: sync-tear bursts (1 start in 8 lines, 8-39 lines long, ±40-199 px) |
| S11 Source | Video / Osc |
| P12 Sun | the sweep position (glided): 0 = dusk (a maroon/red sky with dark flames, an orange floor, spindles without wings), 512 = the photo, 1023 = night-blue sky with black flame cores, wings stretched into needles across the band, a dark floor with red columns |

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) - 16 from a half-resolution line buffer, times K6, is added
  to the sky and floor colouriser input and (halved) to the stack row: bright parts of the picture push the sky
  toward white and cyan and bend the stack bands; black adds nothing, so with no input Video mode is the still.
  The dark shapes, the pink band and the spindles stay put.
- **Osc mode.** The shell's drifting 640 x 540 virtual-px triangle lattice (~75 px/s) feeds the same path: at the
  default Camera the sky gets soft pale/blue patches and the stack bands wave; K6 scales it.

## Engine
- **Vblank-zone sequencer**: a shared 7-channel glide (K1-K6, P12; 1/8 of the gap per field, ±2 deadband), then one
  small operation per zone clock: the knob curves (shift-adds, no ROM), the LFO phase, the sweep dc, the stack
  centre line (688 - dc/2), the ring offset (4 dc) and the DDA step (k1 xs).
- **Line engine** (kicked at each line start and at zone clock 1300 for line 0; computes the NEXT line in ~80
  clocks, so it fits SD's 858-clock line): the row (Flip), two row-ROM words, the dark-shape row and the ring row
  (dark ROM), the tear / jitter LFSR (crucible's), and four serial products: g k3, (y - centre) k5, hw k2 (plus
  the wing term and a 5/8 floor) and the beam's DDA start (xb - 1920) k1.
- **Pixel path (14 stages)**: 2 x, the folded xc, camera luma; 3 column-ROM address, distances, span edges, the
  camera product; 4 the scaled inner-spindle distance, shape depths, bulge; 5 the column word (fabric copy), the
  spindle distance, sparkle flags, the dark shape by priority, the dome; 6 the two products (g T or kk d; e (M - 1));
  7 S, the stack row, the spindle G, the region tests; 8 clamps and the sweep rotation; 9 the palette address;
  10 palette ROMs; 11 fabric copy; 12-14 pads; 15 Ice swap and black outside the generated avid. Then the two
  [1 2 2 2 2 2 2 2 1]/16 boxes (16-19), luma 13/16 (20) and the gate (21).
- **Block RAM (23 EBR)**: column ROM 512 x 32 (4), row ROM 1024 x 16 (4), dark-shape ROM 256 x 32 (2), palette
  3 x 1024 x 8 (6), camera line buffer 1024 x 8 (2), output-box rings 4 x 256 x 16 (4), sync ring 256 x 4 (1).
  The rings carry `no_rw_check`; the only transparency emulation is the line buffer's.
- **Contracts**: the generated avid; one latency, C_LAT 20, in every mode; virtual 1080-line coordinates (SD shows
  the centre 4:3); Y/U/V boxes and luma 13/16 before the gate with the running sums snapping to neutral off the avid
  (resync); palette luma pre-scaled 16/13 and capped at 940; colours in the hardware convention (U = Cr).
- **Verified** (GHDL 720p60, decimation 1): black input (0.83 mean |RGB| from `imodel.py`), Osc, Video (selfie.jpg);
  frames 3 and 15 at Drift 1023 (decimation 2) for the motion. Not checked by sim: SD, Flip / Ice / Tear / Shadows
  (from the integer-model sheet and the code).

## Status
- **v0.1.0 built 2026-11-04 through the shared queue with ROUTER1.** NOT flashed.
  - HD Analog seed 1: 85.93 MHz; HD HDMI seed 1: 91.14 MHz; **HD Dual seed 2: 87.35 MHz** (seed 1 stalled router1,
    timeout); SD Analog s1 80.97, SD HDMI s1 79.96, SD Dual s1 86.16 MHz (SD Fmin 27).
  - 6,410-6,443 / 7,680 LC (83.9 %), 23 / 32 EBR. iceunpack OK x6; the packed config carries the labels and presets.
  - dupin.py: 0 `CARRY I0==I1` (standalone synth).
- Output: `out/rev_b/ron/heirloom/temple.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh temple`.

## Deviations from the photo
- **The stack is the weakest region.** Its bands come from one VCA-ramp model: the outer bands, the central purple
  dome and the pinches read, but the photo's lens shapes between the columns and the lower pink "smile" lens are
  only approximated, and the white/cyan sparkles are flat two-level ellipses where the photo has soft glows.
- **The floor's columns are blockier** than the photo's (the fitted waveform has plateaus), and its pale corner
  glows sit as a band near the bottom.
- **The dark shapes are fixed tables** (static in the clip too); the centre block keeps its asymmetric outline,
  while the small blobs (measured on the left one) and the ellipses (measured on the right one) are mirror copies.
  The band has no gaps of its own: the spindle bodies are drawn over it.
- **The spindles** are lens bodies with straight-ish shoulders where the photo's tops narrow into the flame tips;
  the white rim is a 3-px peak with a short glow (the photo's is wider and softer).
- **One mirror axis at 960** for everything (the photo's axis wanders 954-970), so the spindles sit ±8 px off.
- **Output and edges**: drawn whites come out at about 777 output luma (the shell's cap); the left edge fades in
  over 8 px (box warm-up); the capture's dark border is not drawn.
- **Resolutions**: 720p shows the same geometry as 1080p (the tables are per virtual row / column). SD shows the
  centre 4:3 (1440 virtual px), so the outer ellipses are cut and the corner glows lost; the row tables have 2.1-line
  resolution, so steep band edges step by 2 lines at 1080p. The LFO is per field, so 50 Hz modes drift 17 % slower.
- **Nothing was cut for budget.**
