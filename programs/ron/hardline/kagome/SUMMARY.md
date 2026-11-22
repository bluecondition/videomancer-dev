# Kagome (Hardline)

**v0.1.0 built 2026-11-22, not flashed.** Flash `out/rev_b/ron/hardline/kagome.vmprog`.

## What it draws

Kagome, the Japanese triaxial bamboo basket weave, and chair caning. White strips in three directions 60
degrees apart, truly interlaced over and under on black; each strip carries a black edge line inside its
edges, so wherever a strip dives under another it is cut cleanly by the upper strip's edge. Between the strips:
black hexagonal eyes ringed by small triangles (the star-of-David lattice). Black and white only, U = V = 512
on every pixel. At power-up the basket turns slowly (about 44 s per 60-degree repeat) and glides along one
strip direction.

    c_k   = (p - pivot) . n_k / Period + o_k        n_k at th, th + 120, th + 240 degrees (they sum to zero)
    a_k   = distance from the nearest strip centre line (0 .. 1/2 cycle);  hw = half-width (Weave + camera)
    weave = 0 over 1, 1 over 2, 2 over 0 (cyclic: along any strip the crossings alternate, so every strip goes
            over, under, over ...). One signed-distance field, positive = white:
    white_k = min(hw - L - a_k [, a_k - L/2 in Double],  a_(k-1) - min(hw, max(a_(k+1), a_k + L)))
    W       = max_k white_k     (Fine: min(max_k e_k, -W), only the outlines)

Where all three strips overlap (wide strips, the camera, or Shift toward the knots) the least-covered strip
drops out and the cyclic rule picks between the other two: a pinwheel tuck, continuous across every region
boundary, its seams outlined. The spec's fixed 0 > 1 > 2 order inside the triple region was tried in the float
prototype and left seams and broken edge lines (proto.py mode=fixed), so the pinwheel replaced it.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | pans the weave: the pivot (also the turn's centre), +-1024 virtual px. Glided |
| K2 | Y | the same vertically, +-1024 virtual px. Glided |
| K3 | Period | strip pitch 60 .. 480 virtual px, exponential (default 440: 146 px, 7.4 strips per family per screen height). Glided |
| K4 | Shift | slides the third family across the other two: 0 = the true kagome (default), through lopsided eyes, to 100 % = all three families crossing at single points (a triangular lattice of six-way pinwheel knots). Glided |
| K5 | Drift | bipolar, centre = still (+-24 counts): the weave turns 2^-16 .. 2^-8 turn per field (about 18 min to 4 s per full turn) and glides along strip 0 by 8 x the turn in cycles, both wrapping exactly. Default 660 (+29 %): 0.023 degrees per field. Glide On eases the speed 1/8 per field, never the angle |
| K6 | Camera | box-filtered luma widens the strips (bright closes the eyes), default 100 %. Glided |
| S7 | Glide | Off = the value read at this field start, no easing. On (default): X, Y, Period, Shift, Camera, Weave ease 1/16 of the gap per field, never slower than 1/8 count, landing exactly; the Drift speed 1/8 |
| S8 | Strand | Single (default) / Double: a black centre line splits every strip into two canes (chair caning) |
| S9 | Shade | Flat (default) / Shade: each cane shaded as a rounded rod (a triangle ramp of the distance to its nearest black line; the cane also darkens where it dives under the next strip) |
| S10 | Weight | Bold (default) = white strips / Fine = only the strips' outlines, a wire weave. With Shade on: Bold = 4 flat steps (black, dark grey, light grey, white), Fine = a smooth 6-bit ramp |
| S11 | Source | Video (default) / Osc |
| P12 | WEAVE | strip width / pitch: 0 % = 0.05 (hairline strips, a thin star lattice), 45 % (default) = 0.23 the classic basket, 50 % = 0.25, 100 % = 0.75: a tight woven surface of pinwheel-tucked canes and outlines with pin-hole eyes (the climax). Glided |

Every knob has a +-1-count hold against pot jitter (the two ends always get through). No stepped knobs.
The edge line is 1/40 of the pitch, held to 3 .. 8 virtual px, never under 2 virtual px and never over 0.39 of
the strip half-width; the strip itself never narrows under 6 virtual px, so Weave 0 % stays a visible thin
basket even at the smallest Period.

## Camera

`program_type = "processing"`. The camera is never shown. Its luma ((Y >> 2) - 16) goes through a per-field
gain table T(l) = max(0, l - 72)^2 x K6 >> 13 (a square law above a knee, in cycles of half-width), a 32-px
box (64 px on the 1920-wide rasters), edge-replicated at each line start, and a vertical one-pole (1/32 per
line, 1/64 at 1080p, 1/16 interlaced) in a half-resolution line store, and is ADDED TO THE STRIP HALF-WIDTH.
Dark areas keep the open basket; bright areas swell the canes until the eyes close and the triple overlaps
tuck into pinwheels: the picture woven into the basket. Every edge stays a clean curve (the width only moves
edges along their normals).

- 25 %: up to +0.08 cycle of half-width at full white; a gentle swelling of the brightest areas.
- 50 %: up to +0.16 cycle.
- 100 % (default): up to +0.33 cycle (eyes close where the subject is bright). On bear.png (camera.png) the
  dark bear reads as open basket on a closed, bright ground, the light muzzle as a closed patch.
- K6 0 or a black input is exactly the clean weave.
- Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice at the same point through the same
  table at twice the gain: drifting flowers of closed, pinwheel-tucked canes on the open basket.

A linear gain was tried first: the bear's bright background saturated and the dark fur swelled too, so the
whole frame closed and the bear vanished; the knee and square law fixed that.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): the hold + glide of seven channels (five clocks each),
  then a sequencer of 10 global + 2 x 8 family slots of 24 clocks on ONE serial 16-step multiplier (about 650
  zone clocks): the pitch through a 2^(i/256) ROM (1 / Period, cycles per pixel and per line, pixels per cycle
  as a 4-bit mantissa + shift), the pivot's column and row, the half-width hw0, the edge line L, the cane-ramp
  gain (a 64-entry table by Weave), the Drift decode and the two accumulators (turn, glide), the Shift sum; then
  per family its angle, sine and cosine (1024-point quarter-wave ROM, interpolated), the pixel step, line
  step, line wrap and the field seed, which is written straight into the family's DDA. The gain table is
  refilled by second differences.
- **Per line**: no arithmetic beyond the DDAs' wrap add.
- **Per pixel** (stages 5 .. 21, latency 32 in every mode, no divider, no square root): two family DDAs in
  cycles (Q26) -> the third family as their sum's complement -> a_k by a one's-complement fold (Q14) -> the
  weave field above in 15-bit signed arithmetic (one pipeline stage per operation) -> max over the strips ->
  clamp, x pixels per cycle (one 12 x 5 multiply), shift -> coverage 0 .. 16 (analytic, 1 px; 2 px when
  interlaced) -> the cane level (one 13 x 4 multiply) -> luma 64 + ((cover x level) >> 4) x 14 -> [1 2 1]/4 ->
  13/16 toward black (white 781) -> blanking gate.
- Chassis: the Kinetic riley shell as stripped by hardline/dazzle ([A] timing-ID virtual-1080 steps, generated
  avid with the parity-locked start, sync delay; [B] hold + glide; the serial multiplier and sine / exp ROMs;
  [D] source field with the Osc line-0 fix; the luma half of the output conditioning). `field_n` passes
  through one register (the core ties it high on every input).

## Status

- Built 2026-11-22 through the shared queue with **router1**, **every config on seed 1**. Not flashed.
- Post-route Fmax (the last report per clock): HD Analog 91.72 MHz, HD HDMI 85.25, HD Dual 81.55 (HD needs
  74.25); SD Analog 79.08, SD HDMI 87.08, SD Dual 84.15. 6,977 .. 6,988 / 7,680 LC (91 %), 15 / 32 EBR. All six
  bitstreams pass iceunpack. No carry self-adds (dupin: 0); no `emulate_transparency` in a standalone yosys run.
- **router2 does not close this program**: on the same netlist (before a small zone-only fix) router2 gave
  HD Analog 79.30 MHz (seed 1) but SD Analog stalled on seed 1 (passed on seed 2) and HD HDMI missed 74.25 on
  seeds 1 and 2; that build was stopped and rebuilt with router1.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline kagome`.
- **Model = RTL** (GHDL captures diffed against `imodel.py`, pixel for pixel; the GHDL frame is the model's
  shifted 4 columns right with 4 blank columns, the shell's generated-avid start):
  - 720p60 decimation 1, every control at its TOML default incl. Glide On (`rtl_default.png`): 0 of 918,720;
    Source = Osc 0 of 918,720; bear.png at K6 100 % 0 of 918,720 (the model fed the tester's own
    stimulus luma).
  - NTSC decimation 1: field 1 and field 2 each 0 of 173,988 (the drift steps once per field: 5 and 6 field
    steps); the same picture as the centre 4:3 of 720p (`ntsc_vs_720.png`).
  - 720p / 2 (0 of 228,960 each): every knob at 0 (minimum Period with Weave 0 %: the corner that exposed the
    half-width wrap, fixed) and every knob at 1023 with Osc, both on the built sources; and, on the sources
    before that zone-only fix (it does not change their per-field constants): Double + Shade + Fine + Shift
    59 % + Period 29 % + off-centre pivot + reverse Drift + Weave 63 %; Double + Shade + Fine alone; reverse
    Drift alone; Osc; bear.png at K6 100 %; Drift +86 % and -80 % after 7 field steps; Glide On in flight (K1,
    K3, K5, P12 moved, 3 fields of easing: the speed eased 1/8, the rest 1/16).
  - The 720p / 1 frames and NTSC above are from the built sources.
- Renders in `.hardline_work/rb_kagome/`: `cmp_final.png`, `rtl_default.png` (GHDL); `sheet.png`, `strip.png`,
  `motion.png`, `camera.png` (integer model); `zoom_eye.png` (one eye at three turns: every crossing alternates).

## Cut / deviations

- **Triple overlaps use a pinwheel rule, not the fixed 0 > 1 > 2 order** (the fixed order left seams).
- **P12 100 % is 0.75 of the pitch, not 0.64.** With three families at 60 degrees the hexagonal eyes only close
  as the width approaches the pitch (eye inradius = (pitch - width) / 2); at 0.64 they were still large. The
  small triangles close at 1/3 of the pitch, so from about 60 % of the slider the triple overlaps appear and
  tuck into pinwheels; at 100 % the surface is nearly all cane and outline, with pin-hole eyes.
- **The pinwheel seams are thinner than the edge lines** (L / sqrt 3, about 2 virtual px at the default Period,
  1.4 px at 720p): the seam scale is 1 so the seams meet the edge lines without a bump and the weave field
  needs one stage less; the seam anti-aliasing ramp is about 0.6 px.
- **Shade is a ramp of the weave field itself** (the distance to the nearest black line), so canes also darken
  where they dive under the next strip; one ramp instead of three per-strip ramps (fit).
- **The turn wraps at a full turn (2^24 steps), not 60 degrees.** A full-turn accumulator is seamless for every
  Shift (60 degrees is a symmetry of the true kagome only); the glide along strip 0 wraps at one lattice cell.
- **Drift tops out at 2^-8 turn per field** (1.4 degrees, 88 degrees per second), not a quarter cycle: faster
  strobes a lattice. Drift is a one's-complement fold around 512, so the slow end below centre is one count
  wider than above (both 24-count deadbands).
- **Default Period 146 px (K3 440), not 50 %**: the bear reads better in the finer weave, and the basket still
  fills the screen in bold strips.
- **X / Y pan +-1024 virtual px**, not two screens: a lattice has no centre to lose; they also move the turn's
  centre.
- With a black input X and Y only shift a repeating lattice, so their sheet panels look alike.
- **The camera's half-width is capped at 0.625 cycle** (gain table saturated at 40960), so the half-width
  never overflows its register; past about 0.5 cycle plus an edge line the picture is saturated anyway.
- **SD**: the same picture (the centre 4:3, same strip count top to bottom); field lines are 4.4 virtual px, so
  edges get a 2-px ramp and below Period about 15 % the hairline strips of Weave 0 % fade toward grey. 720p
  pixels are 1.5 virtual px: thin seams and the Weave-0 % hairlines are 1 .. 2 px there.
