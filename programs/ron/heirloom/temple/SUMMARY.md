# Temple — an orange sunset temple: cyan flames, maroon shapes, orange spindles in a pink band

**Current: v0.3.0 (2026-11-21), Ron's second notes: taller / wider flames, a growing sun with a glow, detail all over (see the last section); v0.2.0 was flashed by Ron ("We are on the right track").** v0.1.0 built 2026-11-04. Heirloom from the video clips (phase 3), made from the still
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

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. K6 here already was the camera strength, so only its default changed: initial value 200 -> 512 (v0.1.1). The vmprog was repacked from the same bitstreams (verified: only the 150-byte settings block differs), so timing, seeds and LC are as above. Presets keep their own K6 values. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/heirloom/.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 loads at 100 % (initial_value 1023), and bend / element programs get a 25 % stronger camera at the top of the knob.
- **Element.** K6 Camera adds the camera luma to the sky / floor colouriser input and (halved) to the stack row: it recolours the drawing and bends the stack bands.
- **Change.** `c_k6` = (K6 + K6 / 4) / 8 in Q8 (was K6 / 4 in Q9), and the camera term takes p3_cam(15..8) instead of (15..9). The camera term peaks at 158 (was 127); every register keeps its width (the 8 × 8 product is unchanged). Latency unchanged.
  - Check: the new code at K6 818 reproduces the old code at K6 1023 (0.4 % of pixels differ by > 8; `.heirloom_work/rb_temple/k6b/k6b_eqcmp.png`).
- **Render.** `.heirloom_work/rb_temple/k6b/k6b_cmp.png` (bear, 720p60 / 4, K6 100 %, old against new, glide bypassed in the scratch copies). The same frame structure: the bright areas move further along the colouriser, so on the bear the ears and eyes pass cyan back toward blue and the bright chest clamps at the end of the ramp (less orange). That is the palette's cycle, not a wrap (see the K6 818 check).
- **Build** v0.1.2, router1, all six configs seed 1 (v0.1.0's HD Dual needed seed 2). HD Analog 88.05, HD HDMI 95.62, HD Dual 89.06 MHz (worst HD 88.05). 6426–6446 / 7680 LC, 23 / 32 EBR. Not flashed.

## v0.2.0: Ron's preview notes (2026-11-21)

Ron (hardware, v0.1.2): "I would like more control of the individual elements with knobs - the height of the overall pink middle should be adjustable. the height of the orange flames should be separate. make the controls more dynamic. there should be a setting for white edges around some of the internal geometry. incoming video in the pink section should do something."

The "orange flames" are the four spindles in the pink band (orange bodies, yellow cores, arrow wings). No fault, five feature requests. v0.1.2 backups (vhd, toml, py, vmprog): `.heirloom_work/rb_temple/temple_v012.*`.

### Control map, old -> new
| Slot | v0.1.2 | v0.2.0 |
|---|---|---|
| K1 | Columns x0.5 .. x2 | **Columns** x0.375 .. x1 .. x3 (wider) |
| K2 | Spindles x0.3 .. x2 | **Flame Width**: the same job under Ron's word, x0.125 .. x1 .. x3 |
| K3 | Flames (sky / floor waveform depth) | **Flame Height** (new) x0.15 .. x1 .. x2.5 |
| K4 | Drift (sweep LFO rate) | **Pink Height** (new) 0.2x .. 1x .. 3.7x |
| K5 | Stack (stack pinch) | **Waves**: old K3 + old K5 merged |
| K6 | Camera | **Camera** (same curve, default 100 %), now also in the pink band and the flames |
| S7 / S8 / S9 / S11 | Flip / Palette / Shadows / Source | unchanged |
| S10 | Tear (sync-tear bursts) | **Edges** (new): Off = the still, On = white edges |
| P12 | Sun | Sun (unchanged); the drift now always runs at the clip's rate |

- **Retired:** the Drift rate knob and the Tear bursts.
  - The sweep LFO always runs at the old default (the clip's 8 s triangle, +-32 palette units around the Sun position), so the default picture moves as before. It can no longer be stopped or sped up.
  - The still's faint per-line jitter (the old Tear Off) stays.
- **Merged:** the sky / floor flame depth and the stack pinch were both VCA depths of the same column waveform. Waves drives both with their old curves; the sky's top end widened from x2 to x2.5.

### Controls (512 = the still)
| Control | Element | 0 .. 1023 |
|---|---|---|
| K1 Columns | the column spacing of every repeated element | x0.375 (outer flames off-screen, two huge inner ones) .. x3 (the pattern crowded into the centre) |
| K2 Flame Width | the orange flames' width and wings | x0.125 slivers .. x3, the four flames merge into a wall of fire |
| K3 Flame Height | the orange flames' height, anchored where they meet the stack | x0.15 (40-row flames on the stack) .. x2.5 (they rise through the maroon band into the sky between the cyan flame columns, to the top of the screen) |
| K4 Pink Height | the pink band's height; the sky and the lower part share the rest of the screen | about 70 visible pink rows .. about 620 rows, with the sky and the stack/floor squeezed to half height |
| K5 Waves | the column waveform depth | plain sky gradient, flat floor and flat stack bands .. blazing blue flame columns, deep floor columns, the stack torn into lenses (x4) |
| K6 Camera | the camera's light (or the Osc lattice) | 0 = the still .. 100 % (default): bends the sky / floor colouriser and the stack as before, lights the pink band (a 32-step ramp from the still's pink to near white, index = camera / 4) and heats the flames (+ camera / 2 on the flame ramp: yellow cores where the picture is bright) |
| S7 Flip | upside down | |
| S8 Palette | Sunset / Ice (Cr and Cb swapped) | |
| S9 Shadows | the band and the dark shapes off | |
| S10 Edges | On: the dark shapes' black rims turn white (depth < 8 px, so they survive the output box blur); the band's top rim and lower edge white; a white rim all round the orange flames (through the band and in the sky too, where the still has it on the pink side only); the stack's three blue band rims white | |
| S11 Source | Video / Osc | |
| P12 Sun | the sweep position (palette rotation, wings into needles, the ring sinking, the stack scrolling) plus the drift | dusk .. night blue |

Every knob and the slider ride the shared glide (1/8 of the gap per field, +-2 deadband).

### How
- **Pink Height remap.** The line engine maps the screen's virtual row v to a table row r through three linear segments: above the pink, table rows 468..628, below.
  - The slopes come from a serial divider in the vblank zone: 160 x 256 / P and 920 x 256 / (1080 - P).
  - The segment starts are A = (1080 - P) 521 / 1024 and B = A + P.
  - Everything except the flames uses r: zones, band, stack, white rim, dark shapes, the ring.
- **Flame Height remap.** The orange flames read their own table row, 645 + (v - v645) x 65536 / F, so their height is independent of Pink Height. Their table words (half-width, peak, slope, wings) count only where that row lands in the middle zone.
- **Checked.** At the defaults both maps are exactly the identity for every row. `.heirloom_work/rb_temple/v020/remap_check.py` (an integer model) also checks monotonicity, continuity and the product widths over the knob grid.
- **Flames in the sky.** The flame colour product (slope x distance) has its own 6 x 7 multiplier, so the flames can be drawn in the sky zone (under the dark shapes) as well as in the middle zone.
- **Palette** (temple.py):
  - 680..687: the Edges flame rim (white);
  - 688..719: the pink camera ramp;
  - 768..1023: a copy of the stack profile with its blue rims white (Edges selects it with one address bit).
- **Router1 self-add.** Router1 hung on HD Analog seed 1 of the first v0.2.0 build (remaining arcs frozen at 3,823). dupin found two LUTs with the same net on two pins, from the sign-extended self-adds in the Sun curve 3(p - 512) and the wing term 3 dc (code unchanged since v0.1.0, whose HD Dual seed 1 also stalled).
  - Both are now computed in offset binary: 3p - 1536, and the wing terms 3u - 384, their half, and 4u + u/2 - 576 with u = dc + 128.
  - The wing terms are per-field constants, selected per line and added once.
  - Bit-identical over the full ranges; dupin now reports nothing.
- **Unchanged:** the latency (C_LAT 20 in all modes), the generated avid, the output boxes, luma 13/16, the blanking gate and the running-sum resync.

### Judgement calls
- The Drift knob and the Tear switch were retired to make room (see above). The drift itself was kept at the clip's rate.
- **Pink Height stretches table rows 468..628, not the whole pink (440..645).** The maroon band's bulge (440..467) and the stack's top dome (630..645) keep their shapes; the visible pink grows around them.
- **Flame Height is in screen rows, independent of Pink Height ("separate").** With a short pink band, the default-height flames rise past the band into the sky. With a tall one they stand at its bottom with bare pink above. K3 sets them.
- **The camera lights the pink geometry but does not move it.** Black or no input leaves the still.

### Resolution
- The remaps work in virtual 1080-line rows, so SD, 720p and 1080p show the same picture (SD still shows the centre 4:3, as before).
- The row tables step every 2.1 rows. Where Flame Height stretches the flames 2.5x, their outlines step every ~5 lines at 1080p (~3.5 at 720p, ~2.5 at SD); the per-line jitter already tears those edges.
- Pink Height stretches only flat pink, so it shows no steps.

### Build
- **v0.2.0 built 2026-11-21 through the shared queue with ROUTER1, all six configs on seed 1.** NOT flashed.
  - HD Analog 92.62, HD HDMI 80.59, HD Dual 80.19 MHz (worst HD 80.19; v0.1.2 had 88.05); SD Analog 78.69, SD HDMI 80.90, SD Dual 91.43 MHz (SD Fmin 27).
  - 7,060-7,086 / 7,680 LC (92.3 %, was 83.9 %), 23 / 32 EBR (unchanged). Standalone synth: LUT 3,531 -> 3,981, CARRY 1,399 -> 1,605.
  - iceunpack OK x6; the packed config carries the new labels and presets. dupin: nothing (no CARRY I0==I1, no duplicate LUT inputs).
  - The first v0.2.0 build (before the self-add fix) hung router1 on HD Analog seed 1 and passed on seed 2 at 78.09 MHz; that build was stopped and is not shipped (log `.heirloom_work/rb_temple/v020/build_v0.2.0_selfadd_attempt.log`).
- Output: `out/rev_b/ron/heirloom/temple.vmprog`. Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh temple`.
- **HD HDMI re-seeded (the ADV7513 HSYNC skew, Ron's "hsync lines").**
  - The builder's HD HDMI router1 s1 sat at HSYNC **+0.16 ns** / data <= -0.61: positive, but inside the sub-window danger zone where wildfire showed lines at +0.65.
  - The screen (`.heirloom_work/skew_tools/screen.sh temple`: hd_hdmi router1 + router2 s1-60, 109 timing-clean, 33 in the window) put router2 s18 most positive in the window.
  - **Installed: HD HDMI = router2 s18 (+2.28 / <= -0.71, 79.79 MHz).** Spare: router2 s3 (+2.18 / <= -0.78, 84.48 MHz) in `.heirloom_work/rb_temple/skew_v020/temple_v020_spare_hd_hdmi_router2_s3.vmprog` (moved there for v0.3.0).
  - HD Analog router1 s1 (+0.76 / <= -0.66) and HD Dual router1 s1 (-1.78 / <= -1.72) kept.
  - The replaced s1 bitstream is in `.heirloom_work/rb_temple/skew/`. The shipped HD HDMI is thus a **non-1 seed (router2 s18)**.
  - **A full builder rebuild re-routes HD HDMI router1 s1 (+0.16).** Re-install afterwards with `.heirloom_work/skew_tools/install.sh temple router2 18` (same netlist), or re-screen if the RTL changed.
- Presets (hand-written, re-aimed): Photo, Dusk (P12 120), Blaze (P12 940), **Pink Hall** (Pink Height 1000, Flame Height 360; replaces Still Temple, which used Drift), Wide Columns (K1 180, Flame Width 760), Ice Flip, **Neon Lenses** (Edges + Waves 1000; was Torn Lenses: Tear + Stack 1000), **Rising Fire** (Shadows off, Flame Height 1000, Waves 900; replaces Bare Sky: Shadows off + Flames 900).

## v0.3.0: Ron's second notes (2026-11-21)

Ron (hardware, v0.2.0, "We are on the right track"): "flame width and height seem underutilized, the orange flames should be much taller at their peak and wider at their peak width. the Sun control should cause the yellow sun at the bottom to grow and shrink and bleed into the pink/orange area above. we want visually stunning complexity inside the pink area and all over, more detail".

Backups of v0.2.0 (vhd, toml, py, vmprog): `.heirloom_work/rb_temple/temple_v020.*`. The control map is unchanged; three controls do more.

### What changed
- **K2 Flame Width** reaches **x6** (was x3). The lower half is unchanged (x0.125 .. x1); the upper half is linear from x1 to x6, `128 + 5 (k - 512)/4` in Q7.
  - At the top, the two flame pairs merge into one wall of fire across the pink.
  - The flame colour ramp is not scaled with the width: wide flames keep the photo's ~+-120 px yellow-orange core inside a red-orange body. Width-scaling it cost 77 LC.
- **K3 Flame Height** reaches **x5** (was x2.5). The lower half is unchanged (x0.15 .. x1); the upper half is linear, `256 + 2 (k - 512)` in Q8.
  - At x5 the wing row sits near screen row 160 and the narrow tops run off the top of the screen.
  - The flames still rise through the band into the sky and stay under the dark shapes.
- **P12 Sun** keeps its sweep job (palette rotation, wings, the ring, the stack scroll and the clip's drift).
  - **It now also sets the yellow sun's radius: 12 (0) .. 124 (512, the photo's dome) .. 604 (1023).**
  - The sun is an analytic disc on row 1088, x 985, in virtual coordinates, so it stays round at every resolution.
  - Each line computes the half-width sqrt(R^2 - dy^2): one serial product and a 10-step serial square root in the line engine.
  - It keeps the old depth shading (pale rim, yellow inside).
  - It is drawn over the floor, rim, stack and pink, under the dark shapes and the flames, so the flames stand silhouetted against a big sun.
  - **The glow:** a yellow bloom outside the disc. g = (distance into the glow) / 16, max 15; the bloom adds luma + 4g and Cb - 4g (clamped).
    - The glow width is R/4 + 3 (p - 512)/8 above 512 (up to 341 px), and R/4 below.
    - Above the disc it rises as a cone, -(dy - R)/2, so at the top of the slider the yellow bleeds up through the stack and the pink into the sky.
    - At the default it adds at most +4 luma (invisible). The disc's own colours are not bloomed, so the default dome is the photo's.
- **Detail (always on):**
  - **flame echoes**: nested outlines 8..71 px outside every flame, every 16 px (orange fading toward pink), rippling outward 1 px per field. Columns scales their spacing. Edges turns them white.
  - **pink contour lines**: iso-lines of the stack field (every 16 rows of the stack ramp), drawn in deep rose (214, 52, 112) across the pink band. They curve like the stack's lenses and domes; Waves bends them, the camera warps them and the Sun scrolls them.
  - **sky / floor contour lines**: iso-lines of the colouriser, nested outlines of the cyan flames and the floor columns, drawn in the opposite palette colour. They crawl as the Sun / drift rotates the palette.
  - **rings inside the dark shapes**: every 8 px of depth, in red.
  - **contour lines inside the flames**: every 8th colour-ramp step alternates.
- **Output (sharper).**
  - U and V go through one [1 2 2 2 2 2 2 2 1]/16 box (was two). That is the 9-tap chroma wrinkle confirmed as the HD HDMI bar fix.
  - Luma goes through [1 2 1]/4 at the same centre (was the two 17-tap boxes), taken from the freed block-RAM ring.
  - The 13/16 luma stage is gone: the palette is no longer pre-scaled and is capped at 763 (the old 940 x 13/16, the bar fix's dim white), so the output levels are the same.
  - The picture is crisper than v0.2.0 (luma no longer smeared over 17 px), so the detail survives.
  - Latency **C_LAT 20 -> 17** in every mode (processing_delay_clks 17).
- **Fit work (no visual change):**
  - the Osc's per-clock x121 multiply became a per-mode constant;
  - k1 x xs became a 3-way case;
  - the camera line buffer got `no_rw_check` (the beam reads 3-4 px behind the source write; emulate_transparency is now 0).
- **Palette** (temple.py): 720..724 the echo ramp, 728 the pink contour colour.

### Controls (512 = the still)
| Control | Element | 0 .. 1023 |
|---|---|---|
| K1 Columns | the column spacing of every repeated element (also the echo spacing) | x0.375 .. x3 |
| K2 Flame Width | the orange flames' width and wings | x0.125 slivers .. x6, one wall of fire |
| K3 Flame Height | the orange flames' height, anchored at the stack | x0.15 (40 rows) .. x5 (past the top of the screen) |
| K4 Pink Height | the pink band's height | ~70 .. ~620 visible rows |
| K5 Waves | sky / floor flame depth and stack pinch; it also bends the pink contour lines | flat .. blazing / torn |
| K6 Camera | bends the sky, floor, stack and pink contours, lights the pink band, heats the flames | 0 = the still .. 100 % (default) |
| S7 Flip | upside down (the sun goes to the top) | |
| S8 Palette | Sunset / Ice (the sun and its glow turn blue) | |
| S9 Shadows | the band and the dark shapes off | |
| S10 Edges | white edges (dark shapes, band, flame rims, stack rims) and white flame echoes | |
| S11 Source | Video / Osc | |
| P12 Sun | the sweep (palette, wings, ring, stack) + the drift; the sun's radius 12 .. 124 .. 604 and its rising glow | dusk with a pin-point sun .. blaze with a giant sun bleeding yellow up the screen |

### Judgement calls
- **Width-scaled flame core dropped** to stay at 93-94 % LC (see K2).
- **The detail is always on, in colours drawn from the picture.** The photo stays recognisable at the defaults (same geometry and colours) with fine lines added. There is no knob to turn it off; Edges turns the echoes white.
- **The sun follows the glided slider only**; it does not breathe with the drift.
- **Sharper luma and narrower chroma** change the look slightly from v0.2.0's soft picture (the photo's analog softness). Both are the recipes confirmed on HD HDMI hardware (damier [1 2 1] luma + dim white, wrinkle 9-tap chroma), but this exact combination is new here.

### Resolution
- Same picture at SD, 720p and 1080p: the sun, the echoes and the contours are in virtual 1080-line coordinates.
- **SD shows the centre 4:3 (virtual x 240..1680), so a sun with R above ~500 is cut at the sides on SD.**
- The echo and contour lines are 1-2 px at 1080p and 1 px at SD.

### Build
- **v0.3.0 built 2026-11-21 through the shared queue with ROUTER1, all six configs on seed 1.** NOT flashed.
  - HD Analog 87.24, HD HDMI 92.87 (builder's route, see the re-seed below), HD Dual 84.46 MHz; SD Analog 87.42, SD HDMI 79.35, SD Dual 84.36 MHz.
  - 7,160-7,185 / 7,680 LC (93.6 % max; v0.2.0 had 7,060-7,086). 22 / 32 EBR (was 23: the second output box's rings are gone).
  - iceunpack OK x6; the packed config carries the labels and presets. dupin: nothing.
- **Fit.** The first v0.3.0 draft packed to 7,328 LC (95.4 %). Parallel standalone ablations ranked the costs (bloom chain 131 LUT, the 8x9 flame product 72, detail 65, sqrt 55, the curves' third segments 55). Cuts and simplifications:
  - a fixed glow shift, and a shift-only bloom;
  - two-segment flame curves;
  - no sun branch in the sky zone;
  - white Edges variants for the echoes only;
  - no 13/16 stage;
  - the Osc / c_dk multiplies replaced by cases;
  - lb `no_rw_check`;
  - and, last, the width-scaled flame core (77 LC).
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh temple`.
- **HD re-seed (the ADV7513 HSYNC skew, Ron's "hsync lines").** The screen covered hd_hdmi router1 + router2 s1-60 (104 timing-clean, 11 in the window).
  - The builder's HD HDMI router1 s1 sat at HSYNC **+0.25 / data <= -0.05**: unsafe on both counts.
  - **Installed HD HDMI = router2 s38 (+1.75 / <= -1.26, 80.81 MHz).**
  - **HD Dual: router1 s1 (-0.30 / <= -0.37) -> router1 s7 (-2.27 / <= -1.90, 82.51 MHz).**
  - HD Analog router1 s1 (+0.49 / <= -0.63) kept.
  - Spare HD HDMI router2 s19 (+1.52 / <= -0.79), with the same HD Dual: `.heirloom_work/rb_temple/skew/temple_v030_spare_hd_hdmi_router2_s19.vmprog`.
  - The replaced s1 bitstreams are in `.heirloom_work/rb_temple/skew/`. The v0.2.0 screen moved to `skew_v020/`.
  - The shipped HD HDMI and HD Dual are thus **non-1 seeds** (router2 s38, router1 s7).
  - **A full builder rebuild re-routes seed 1.** Afterwards re-install with `.heirloom_work/skew_tools/install.sh temple router1 7 hd_dual` and `... temple router2 38` (same netlist), or re-screen if the RTL changed.
