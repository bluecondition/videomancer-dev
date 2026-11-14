# Limelight: soft-keyed beams of gelled light

v0.1.0 built 2026-11-14, not flashed. Patchbay #9. Presets are hand-written.

## The patch

    H ramp, V ramp -> four ramp rotators, one per lamp (lamps on a rail 110 lines above the top edge,
                      at x = 315, 745, 1175, 1605 of 1920)
    LFO 1 (K4 rate) -> each rotator's angle, depth K3, phase offset per lamp K5
    LFO 2 (slower, its own drift + 1/4 of LFO 1's rate) -> every rotator's angle, 7 degrees, a quarter
                      cycle apart per lamp (a slow lean that keeps the sweep from looping)
    rotator -> two rotated ramps hA, hB: half-planes through the lamp at theta - w and theta + w (w from P12)
    hA, hB -> soft window comparator: min(hA, hB) clamped; gain = 1 / K2 (a linear penumbra, centred on the
                      geometric edge) = a cone of light
    V ramp -> full-wave rectifier at the floor line -> every rotator: below the floor line the cones mirror
    V ramp -> level ramp (K1): 0.78 at the top edge .. 0.40 at the floor; 0.28 fading to 0 below it
    level ramp + bias (P12) + haze texture + source -> the level; each window is clipped to it (min)
    the window again, minus a threshold that follows an ellipse profile across the floor line -> the pool
    key = clipped window + pool, per lamp -> gel (a Y tap of 1/4, 1/2 or 1 and U, V taps of 0 or +-full)
    four gelled keys -> additive Y / U / V mixer        (S9 Key: only the first lit lamp passes, doubled)
    mixer luma above full -> narrows the chroma limiter (white where enough light adds up)
    haze texture, S8 Smooth: two triangle oscillators of H and V (298- and 128-line strata, tilted opposite
                      ways, drifting 0.10 and -0.15 Hz), mixed 3 : 1
    haze texture, S8 Rays: five ratio comparators on hA : hB of the front lamp (1/4, 1/2, 1, 2, 4): shafts
                      that radiate from the lamp and turn with it
    source, S11 Video: camera luma (minus a black threshold, x 1.25) -> the level
    source, S11 Osc: a drifting 320 x 270 diamond lattice, thresholded to bright diamonds on a darkened level

## What it draws

Four cones of coloured light from above, crossing: magenta, red, green and azure at the defaults. Where
two overlap the colours add (red + green = orange, green + azure = cyan), where all four meet the sum is
neutral and reads white. Every beam has a soft linear penumbra, fades toward the floor, lands in a bright
lozenge-shaped pool on the floor line, and continues below it as a dim mirror image that fades out (a glossy
stage floor). Faint tilted strata of haze drift through the beams. Outside the beams the frame is flat black.
The beams swing on LFO 1 with a phase step from lamp to lamp (a wave travelling along the rail) while LFO 2
leans the whole rig slowly at an unrelated rate, so the crossings never repeat exactly; at the default Speed a
sweep takes about 9 s.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Reach | level ramp / rectifier | the floor line, 500 .. 1030 lines below the top (exponential), and the fade scaled to it: 0 = short beams over a deep mirror floor, 1023 = the beams run to the bottom edge. Default 800 |
| K2 Soft | window comparators | penumbra width 12 .. 384 px (5 octaves, glided): crisp cones .. a wash of colour. Default 600 = 91 px |
| K3 Sweep | LFO 1 depth | swing 3 .. 45 degrees (never 0, so the rig cannot freeze). Default 600 = 28 degrees |
| K4 Speed | LFO 1 rate | bipolar, square law, +-8 deadband: +-0.9 sweeps / s; the sign reverses the travelling wave. LFO 2 keeps its own 0.04 Hz plus a quarter of this. Default 700 = 0.11 Hz |
| K5 Spread | LFO 1 phase taps | phase step per lamp, 0 (all four in unison: a chorus line) .. half a cycle (neighbours opposed: scissoring). Default 300 |
| K6 Colour | gels | 8 gel sets: Primaries (magenta, red, green, azure), Stage (rose, amber, cyan, violet), Warm, Cold, Rack (hot pink / lime), CMY, Police (rose red / azure), Limelight (white, yellow, ice, lilac) |
| S7 Lamps | rail position | Top / Foot: the rail goes below the bottom edge, the beams rise and fan outward, the "floor" becomes a ceiling line near the top |
| S8 Haze | haze texture | Smooth (drifting strata) / Rays (the front lamp's cone split into six shafts) |
| S9 Mix | mixer / keyer | Add (light adds) / Key (opaque layers, lamp 1 on top; each layer's soft edge falls to black before the next shows: the dark seam of an analog keyer) |
| S10 Pools | pool key | On / Off |
| S11 Source | level input | Video (the camera picture is lit by the beams) / Osc (diamond gobo) |
| P12 Blaze | window width + level bias | half-angle 1.3 .. 20 degrees (exponential, glided) and the level bias -0.23 .. +0.23: 0 % = four dim pencils with pinpoint pools, 100 % = wide cones flooding the frame, burnt to white where they overlap, the pools one merged slab |

Presets: Limelight (defaults), Pencils, Chorus Line, Cut Paper, Footlights, White Out.

## Video

Video (default): camera luma is written to the shell's line buffer as (Y >> 2) - 16; the pixel path takes what
is above a black threshold (44), times 1.25, and adds it to the level every window is clipped to. So the
picture appears only inside the beams, as brightness on top of the haze, in the gel colours; outside them the
frame stays black. With a black input the level is the haze alone: the full picture. The level reaches the
windows four pixels late, so the picture sits 4 px to the right inside the beams.
Osc: the shell's triangle lattice (320 x 270 virtual px, drifting about one cell per 8 s) is thresholded at
0.6: bright diamonds add 0.27 to the level and the rest of the beam is dimmed by 0.37, about three times the
swing a camera gives, so the flip is unmistakable.

## Engine

- Per field (vblank zone, started at the zone start): one serial shift-add multiplier, 72 slots of 24 clocks.
  16 global slots: Speed squared, five exponentials through a 256-entry exp2 ROM (floor distance, level slope,
  pool height, penumbra gain, half-angle), gain x pixel pitch, the bias, the sweep depth, the edge offset.
  Then 14 slots per lamp: two LFO sines -> theta; per edge cos x gain -> the per-pixel step, sin x gain ->
  the per-line step, step x width -> the line wrap, and the seed (the ramp's value at the first pixel of
  line 0). A 2048 x 16 quarter-wave sine ROM gives 8192 angle units per turn. One 25-bit adder forms every
  signed result and is the write data of a 64-word block RAM (step, wrap, + and - line step, seed per edge).
- Per line (line engine, kicked at each line start, computes the NEXT line in about 40 clocks): distance to the
  floor line, the level (one product), the pool band position (one product) through a 256-entry ellipse
  profile ROM -> the pool threshold.
- In hblank a bus sequencer plays wrap, line step and pixel step of each edge from the RAM (32 clocks). Each of
  the eight 25-bit pixel accumulators is `acc += op`; its operand register is loaded from the bus, so there
  is no multiplexer in front of the adder. The line step is skipped on the line that crosses the floor and its
  sign flips below it: that is the mirror. The pixel count per line is fixed (own counter).
- Pixel path, 10 stages: accumulators; clamp to 8 bits; min of the two ramps; clip to the level, pool; key;
  keyer select; gel taps and pair sums; sums; luma cap and knee; Y / U / V out. The level path (haze triangles
  or ray comparators, source, bias) runs alongside and is four stages longer.
- Shell (heirloom lozenge): timing-ID virtual 1080-line steps, vblank zone, width measurement, generated avid,
  line buffer + Source oscillator, shared knob glide (K1, K2, K3, K5, K6, P12), output conditioning (Y, U, V
  through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16), blanking gate, one latency
  (C_LAT 16 = processing_delay_clks) in every mode. Colours in the hardware convention (U = Cr, V = Cb).
- 16 block RAMs: sine 8, line buffer 4, coefficient RAM 2, exp2 1, ellipse profile 1.

## Status

- v0.1.0, built 2026-11-14 with ROUTER1 (`NEXTPNR_ROUTER="--router router1"`). Not flashed.
- Post-route Fmax (the last "Max frequency" line of each config):
  - HD Analog 80.53 MHz, seed 1
  - HD HDMI 78.67 MHz, seed 3 (seeds 1 and 2 missed)
  - HD Dual 78.45 MHz, seed 1
  - SD Analog 77.22 MHz, SD HDMI 75.91 MHz, SD Dual 78.67 MHz, all seed 1
- A first build with router2 was marginal (HD Analog 79.50 seed 2, HD HDMI 76.27 seed 3, HD Dual 74.25 seed 2 with
  zero margin, SD HDMI seed 1 stalled the router); the shipped vmprog is the router1 build.
- 6,857 .. 6,895 / 7,680 LC (90 %), 16 / 32 block RAMs. hd_clock_divisor untouched.
- Output: out/rev_b/ron/patchbay/limelight.vmprog.
  Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay limelight
- The HD critical path is in the vblank sequencer (the one adder, z_acc, with its add / subtract select). If a
  rebuild misses on HD, register that select one phase earlier.
- Verified in GHDL against the integer model (.patchbay_work/rb_limelight/imodel.py), 0 differing pixels in
  each case:
  - 720p60, defaults, Source Video with a black input
  - 720p60, Source Osc
  - NTSC (two fields), defaults
  - NTSC (two fields) with S7 Foot, S8 Rays, S9 Key and S10 Pools Off together
  - NTSC (two fields) with K1 200, K2 900, K3 1000, K4 900, K5 800, K6 600, P12 850 and Rays, caught mid-glide
    (fields 17 and 18): the glides and the motion match the model field for field
  - 1080i (two fields), defaults
  - With selfie.jpg as the camera, 7.6 % of pixels differ from the model by at most 6 codes (the tester's
    image scaling against mine); the picture reads cleanly inside the beams.
- The sim PNGs sit 4 px to the right with 4 black columns at the left: the tester crops on the source avid
  and the generated avid starts 4 clocks later. On hardware the output avid is the generated one.
- Not simulated: 1080p (same steps as 1080i with half the line step), PAL, 480p / 576p.
- Work files: .patchbay_work/rb_limelight/ (cmp_final.png, sheet.png, strip.png, proto.py, imodel.py, notes.md).

## Cut / deviations

- Pools are lozenges, not ellipses. The spec's pool was to be its own shape on the floor line; a second set of
  windows cost about 400 LC. The pool is now the beam's own window above a threshold that follows an ellipse
  profile across the floor line: bright and rounded at the ends when the beam is soft, a flat slab with
  slanted ends when K2 is hard, one merged slab at P12 100 %.
- The camera is added to a level the beams are clipped to; it is not a multiply and it does not darken. With a
  black input the picture is complete. A typical picture shows as texture inside the beams, strongest toward
  the floor where the level ramp leaves headroom.
- The first RTL (RGB mixer, a multiplier, an RGB to YUV matrix) packed to 8,531 LC standalone and was
  rebuilt: the mixer now adds in Y / U / V with gels as taps, so gel colours are limited to those taps (no
  pale tints except in the Limelight set), and "white where beams cross" comes from complementary gels
  cancelling plus a knee that narrows the chroma limit once the luma sum passes full.
- Rays belong to one lamp per pixel (the lowest-numbered lamp whose cone holds the pixel), six shafts each.
- K2 Soft starts at 12 px, not hard-edged: the output softening hides anything sharper.
- K6 is a numeric 1 .. 8 knob, not labelled, so presets and the sim take plain values.
- Added beyond the spec: the mirror image below the floor line (free: the line step changes sign), and LFO 2.
- The level reaches the windows four pixels late, so haze, rays and the camera picture sit 4 px to the right
  of where the beams cut them.
- Resolutions: 720p and 1080i show the same picture as 1080p. SD shows the centre 4:3 (1440 of 1920 virtual
  px): the outer lamps are 75 px from the frame edge instead of 315, so wide sweeps leave the frame sooner. The
  output softening and the 4-px shift are per output pixel, so SD is softer. Interlaced modes repeat the same
  field (the core gives no field flag).
- Not checked on hardware: everything. Watch first for the gel colours (the chroma taps are full-scale at low
  luma and rely on the encoder's clipping; the previews are computed, not measured), the HD Dual config (zero
  timing margin), and how visible the camera is at the default Blaze.
