# Portcullis: three-bit video in clocked columns (an LZX Castle rack)

**v0.1.0 built 2026-11-25, not flashed.** Onevolt 6. A folded diamond is digitised to three bits, held column by
column by D flip-flops on a hard-synced Clock VCO, two of the bit planes run late through shift registers, and the
bits go straight to R, G, B or through a DAC and a matrix mixer. Presets are hand-written placeholders.

## The rack

| Maker | Module | Job here |
|---|---|---|
| LZX | DSG3 Dual Shape Generator | internal H and V ramps, Fold 1 on both: the Average jack (diamond) or the Maximum jack (nested windows) |
| LZX | Stairs | the fold chain; Steps (K2) is its input gain, its Phase CV carries the bias, the LFO, the ripple and the camera |
| LZX | DWO3 | one free V-range sine (the slow ripple that rolls down) and two LFOs (11 s on the bias, 27 s on the clock FM) |
| LZX | Castle 000 ADC | the folded signal to D0 (MSB) D1 D2: eight levels |
| LZX | Castle 010 Clock VCO | the pixel clock: Frequency (P12), VC Level from the V ramp (K1), hard sync at every line start |
| LZX | Castle 111 D Flip Flops | each bit sampled at the clock edge and held for one clock period: the columns |
| LZX | Castle 110 Counter | S9: its D3 D2 D1 replace the ADC's bits (clock = VCO, reset = line sync and every ADC D0 edge, Up / Down = ADC D0) |
| LZX | Castle 011 Shift Register (x2) | D1 delayed e clock periods, D2 delayed 2e (K4), in sub-clock steps |
| LZX | Castle 100 Multi Gate | S7: each bit XOR its own copy a little later (outlines) |
| LZX | Castle 001 DAC | S10: the three bits to an 8-level voltage |
| LZX | SMX3 | S10: the DAC level to a designed colour family (K5) |
| LZX | ESG3 | the encoder: bits straight to R, G, B at 75 % (K5 = cable order / polarity) |
| -- | camera (decoder luma) | S11 Video, into Stairs' Phase CV through K6 (Osc: a drifting triangle lattice at twice the gain) |

## The patch

    DSG3 Shape 1 (H, V ramps, Fold 1)  Average (S8 Diamond) | Maximum (S8 Windows)  -> Stairs G jack
    Stairs Steps <- K2 Bands                                         (gain ahead of the fold chain, 1 .. 8)
    Stairs Phase CV <- K3 Bias + DWO3 LFO (11 s) + DWO3 V-range sine (free, rolls, 23 s) + camera luma x K6
    Stairs out -> Castle ADC -> D0 D1 D2
    Clock VCO: Frequency <- P12 ; VC in <- V ramp through VC Level (K1) + DWO3 LFO (27 s) ; Sync <- line sync
    Clock VCO -> D Flip Flops clock ; ADC D0 D1 D2 -> FF D0 D1 D2   (S9: Counter D3 D2 D1 -> FF D0 D1 D2)
    Clock VCO -> Counter clock ; line sync + ADC D0 edges -> Counter reset ; ADC D0 -> Counter Up / Down
    FF D1 -> Shift Register A (K4: e clocks) ; FF D2 -> Shift Register B (2e clocks)
    S7 Multi Gate: Thru | XOR (each plane with its own copy w = max(0.75 + 0.75 e clocks, 12 virtual px) later)
    S10: FF D0, SR A, SR B -> ESG3 R G B (K5 = cable order and polarity) | -> DAC -> SMX3 (K5) -> ESG3

## What it draws

At the defaults: a large screen-shaped diamond of eight flat bands in the eight pure colours (black, blue, green,
cyan, red, magenta, yellow, white in binary order, folding back at white and at black), about 60 columns across.
Every band edge is cut into steps one column wide while down the screen the treads stay smooth; the columns fan
(narrower at the top, wider at the bottom, hinged at the left edge where the clock restarts), so the risers on the
right lean into saw-teeth. The green and blue planes arrive about one and two clock periods late, so each step
carries thin displaced fringes of other colours to its right. A gentle ripple rolls down through the edges.

Motion at the defaults: an 11 s LFO on the bias makes the bands flow out of and back into the centre (about two
bands a second at its fastest), the V-range sine rolls down (23 s), and a 27 s LFO leans the clock FM so the fan
breathes.

## Controls

| Control | Module | What it does and its range |
|---|---|---|
| K1 Clock FM | Clock VCO VC Level (V ramp), bipolar, +-8 deadband | centre = straight columns (plus the LFO's lean); right = columns narrow at the top, wide at the bottom (5.7:1 at the end, 9:1 with the LFO); left = the reverse; 760 = the default fan |
| K2 Bands | Stairs Steps | 1 .. 8 half-folds (eight bands each) from the centre to the corners, exponential: 0 = one giant diamond of wide bands, 1023 = a dense nest |
| K3 Bias | Stairs Phase (DC) | slides the band sequence 0 .. 1.5 half-folds (12 bands) through the figure: the bands travel in or out |
| K4 Echo | Shift Registers A / B | 0 (pure colours, no fringe) .. 3 clock periods on D1 and 6 on D2, in 1/256-clock steps; in XOR it also widens the outlines |
| K5 Colour | ESG3 cables (RGB) / SMX3 (DAC) | nine keyframes, pure on a +-19 % plateau each, blended between. RGB: nine cable orders and polarities (default = straight). DAC: phosphor, ember, ice, rose-lime, dusk (default), teal-rust, violet-jade, grey, cobalt-scarlet |
| K6 Camera | Stairs Phase CV depth | 0 = none (exactly the black-input picture) .. 100 % (default) = one half-fold per full luma: the subject is posterised into the figure's bands |
| S7 Gate | Multi Gate | Thru (the bands) / XOR: each plane XOR its own later copy: offset red, green and blue outlines on black |
| S8 Shape | DSG3 output jack | Diamond (Average) / Windows (Maximum: nested rectangles whose vertical sides snap to the columns, colour-echoed) |
| S9 Bits | ADC / Counter | 000 ADC (bands follow the figure) / 110 Counter: 8-step colour staircases two clocks a step, restarting at every ring of the figure, up in the bright rings and down in the dark |
| S10 Output | ESG3 direct / DAC + SMX3 | RGB (eight pure colours) / DAC: an 8-step ramp in one colour family |
| S11 Source | camera / Osc | Video / Osc |
| P12 Clock | Clock VCO Frequency (headline) | 0 % = 384 columns (5 virtual px): silky, un-clocked bands; 50 % = 60 columns; 100 % = 6 columns: monumental slabs with wide colour echoes |

## Video

Camera luma, (Y >> 2) - 16, averaged over 8-pixel cells, slewed down the screen (one pole, 1/32 per line), then
across (two one-poles, 1/8 per pixel), times K6, enters Stairs' Phase CV: one half-fold (eight bands) per full
luma at 100 %. The subject is posterised into eight levels together with the figure, column-sampled and
colour-echoed; the slews keep its edges clean curves (no per-line jitter). With a black input the patch gives its
full picture. Osc: the shell's drifting 640 x 540 triangle lattice at twice the camera's gain bends the diamond into
a lattice of cells.

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks").** Glide on all seven continuous controls (1/8 per field);
  knob decodes; nine slots on one 40-bit serial multiplier (the clock exponent, the x and y gains of the figure,
  the clock increment, the XOR floor, the FM product and its line step, the camera gain, the colour blend), exp2 and
  sine ROMs, then a 24-step loop on the same multiplier that blends the two K5 keyframe palettes into an 8-entry
  palette RAM. The line engine is loaded at zone clock 900.
- **Per line.** The V term R |y - 540| (an accumulator), the ripple (sine ROM), the FM'd clock increment (an
  accumulator, clamped at 0.2 of the base) and its 3/4 threshold, latched at each line start.
- **Per pixel (13 stages to the colour).** 1 beam: the signed x term, the clock phase (Q20) and its wrap. 2 |x|,
  the clock edge's quarter-pixel position, the five tap fractions. 3-4 the DSG3 combiner (Average or Maximum).
  4-5 the fold argument s = figure + Phase CV (mod one fold period). 6-7 the field at the clock edge (linear between
  the two pixels, 2-bit position). 8 fold + ADC; the flip-flops / Counter and a 16-deep column shift register,
  clocked on the wrap. 9 five taps (whole clocks back, one more when the phase is short of the tap's fraction).
  10 the Multi Gate. 11-13 the palette RAM and its fabric copy.
- **Output.** As the shell: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma
  13/16, then a vertical [1 2 1]/4 with the two lines above (two line stores), the blanking gate.
- **Contracts.** Generated avid; one latency (C_LAT 20) in every mode; virtual 1080-line coordinates (the column
  count keeps at SD); colours in the hardware convention (U = Cr); drawn whites ~750.
- **Block RAM (19).** Palette keyframes 2, palette RAM 2, sine 1, exp2 1, luma cells 1, box delay rings 4, line
  stores 8. Every written RAM carries `no_rw_check` (0 transparency emulations).

## Status

- **Pin-skew placement (orchestrator, after the build).** HD HDMI re-placed for the encoder pin skew: router2 seed 29 (HSYNC +2.30 ns, data <= -0.91 ns, 83.56 MHz); the builder's router2 seed 1 was outside the +1.0..+2.3 ns window. A plain rebuild gives the builder's placement again: re-run .heirloom_work/skew_tools/install.sh portcullis router2 29 hd_hdmi. Screen data: .heirloom_work/rb_portcullis/skew/.

- **v0.1.0 built 2026-11-25 with router2 (the build default), all six configs on seed 1, not flashed.**
  Rebuild: `./build_set.sh onevolt portcullis` -> `out/rev_b/ron/onevolt/portcullis.vmprog` (440,503 bytes).
- Post-route Fmax (the builder's last report per config, PASS on the first seed):

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 83.44 | 6,574 |
  | SD Analog | 1 | 82.81 | 6,578 |
  | HD HDMI | 1 | 85.46 | 6,581 |
  | SD HDMI | 1 | 82.30 | 6,582 |
  | HD Dual | 1 | 78.03 | 6,580 |
  | SD Dual | 1 | 79.52 | 6,579 |

  6,574 - 6,582 / 7,680 LC (86 %), 19 / 32 EBR. iceunpack OK x6; the packed config carries the labels and presets.
  The first build came out at 6,867 LC (over the 6,750 budget); removing about 300 redundant flip-flops (per-line
  threshold copies, zone temporaries, a palette-loop copy, one pixel stage) brought it to 6,574 with the same
  arithmetic. dupin on the final synth netlist: 0 `CARRY I0==I1`, 0 duplicate LUT inputs (router1 safe);
  `scan.py`: 0 integer-minus-signed sites; standalone yosys: 0 transparency emulations. No pin-skew screening.
- **Checks** (working files in `.onevolt_work/rb_portcullis/`):
  - `check.sh` clean. TOML pre-check clean (description 99 bytes, presets <= 15 chars).
  - GHDL 720p60, decimation 1, black input, **bit-exact with the integer model `imodel.py`** (0 of 916,168 compared
    pixels differ, rows 2 .. 719; the tester's capture sits 4 px left of the generated avid, as in every shell
    program): the program as built at its defaults (field 7, `rtl_default.png`), and, on a sim-only copy whose
    glides load the knob target at once (`simprog/`, the only change), the defaults, S7 XOR, S8 Windows,
    S9 Counter, S10 DAC, P12 0 %, P12 100 %, and two combinations (XOR + Counter + DAC with K1 100, K4 900,
    K5 300, P12 300; Windows with P12 1023, K2 900, K3 200, K6 700). `rtl_switches.png`.
  - Motion: the defaults at field 2 and field 7 differ in 53 % of the pixels and both match the model exactly.
  - SD (`--video-mode ntsc`): both 480i fields (720 x 243) **bit-exact** with the model; SD shows the centre 4:3
    of the same diamond (`rtl_ntsc_view.png`, one field line-doubled).
  - 1080i (`--video-mode 1080i5994`): both fields (1920 x 540) **bit-exact** with the model; the same picture as
    720p / 1080p (`rtl_1080i_view.png`, one field line-doubled).
  - Osc (`rtl_s11.png`) matches the model's render in structure (40 % of pixels differ: the model's camera-path
    filter does not reproduce the RTL's cell / slew start-up or the lattice phase exactly, as in separatrix).
    Camera (`rtl_camera.png`: `rtl_bear.png`, `rtl_selfie.png`): the diamond bends around the subject with clean,
    column-stepped contours and no per-line jitter; with bear.png the head and both ears read as a dark posterised
    silhouette with the bands running through it (a silhouette more than a portrait).
  - `thin.py` on the model (720p, 38 settings x 4 times each: the defaults, P12 0 / 100 %, all 16 S7-S10
    combinations, each switch alone at P12 0 and 100 %, K1 and K2 extremes): 5 lines of 24 px or more in the
    152 frames, none at the defaults: XOR (32 px, one time), XOR + Windows + Counter (2 x 36 px), XOR at
    P12 100 % (82 px), Windows at P12 100 % (36 px), all soft 3-row lines after the vertical [1 2 1] (tread coincidences between a column and its
    echoed neighbour, or the Maximum combiner's corner).
  - `sheet.png`, `combos.png`, `strip.png`, `camera.png`, `presets.png`: from the integer model; `*_proto.png` from
    the float prototype.
- **Watch on hardware:** crisp moving colour edges at HD HDMI (the shell's conditioning is in). HD Dual has the
  smallest margin (3.8 MHz).

## Cut / deviations

- **S8 is the DSG3 output jack, not the Clock VCO's Sync switch.** A free-running clock only changes how the stair
  risers line up: a lean of 0.5-1 px per line looks like the default, and a strong lean turns every edge into
  saw-teeth and, where it runs along a band edge, long horizontal tears (prototype renders). Neither is a
  look-flip, and the strong one reads as a fault. The clock stays hard-synced (columns anchored at the left edge).
  The spec's "in S8 Off the columns crawl" motion is therefore gone; the bias LFO, the rolling ripple and the FM
  lean remain.
- **K3 Bias works on the fold's phase (Stairs' Phase CV), and the camera enters there too.** An ADC bias after a fold
  only pushes the top bands into white (a washed-out end); on the fold's phase the bands travel, as the spec asks.
- **The Counter also resets at every ADC D0 edge and its outputs are D3 D2 D1.** With line-sync reset only, the
  up / down count carries along the whole line, so the rest of every line shifts each time the D0 edge moves a
  column: long horizontal tears. D1..D3 makes each step two clocks wide so the staircases read.
- **P12 0 % is 384 columns (5 virtual px), not 1-2 px.** Counter staircases at 1-3 px columns blur to grey under
  the output boxes.
- **Vertical [1 2 1]/4 output mix (two line stores) instead of the shell's [1 1]/2**, and a sub-pixel clock edge:
  together they remove the 1-row lines that column sampling under FM and tread coincidences produce. The integer
  model's thin-line check is clean at the defaults, at P12 0 / 100 % and for every switch alone except a few short
  soft lines listed in Status.
- No 1-px coverage blend at the column edges: the output boxes already soften every column edge over ~9 px.
- K5 is a blend between nine keyframes (pure on plateaus) rather than nine free SMX3 gains.
- **Resolutions.** 720p and 1080p draw the same picture. SD shows the centre 4:3 of the diamond and keeps the
  column COUNT (as a Clock VCO's frequency knob would), so its columns are 25 % narrower relative to the figure.
  The clock is capped below one column per pixel (only reachable at SD with P12 near 0 % and strong FM).
