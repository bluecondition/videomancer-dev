# KINEGRAM: a barrier-grid animation cranked by hand (Hardline)

**v0.1.0 built 2026-11-15, not flashed.** Polar chassis (hypnos v1.1.0 engine), `program_type = "processing"`.

## What it draws

Art reference: barrier-grid animation (Scanimation, kinegrams, "magic moving pictures"). A picture made of N
interleaved slivers of N animation frames is seen through a sheet of black bars with one clear slit per N
slivers; sliding the sheet one sliver shows the next frame.

- **Lattice**: vertical slivers of width w (K5 Pitch), anchored on the screen centre. Sliver index
  k = floor(x / w) mod N (K4 Frames).
- **Figure**: Rings `frac(r / Period - k / N) < 1/2` (the rings grow frame by frame), or Spokes
  `frac(arms theta / 2 pi - k / N) < 1/2` (a wheel that turns). Centre on K1 / K2.
- **Barrier**: period N w, one slit a sliver wide. Output = figure AND slit, so 1 / N of the picture is lit
  (it is dark by nature) and the slits are full white (Y 776).
- **Slide (P12)** moves the barrier: at rest the slit sits exactly on sliver 0 and the picture is one clean
  frame; one sliver of travel is the next frame, and between frames two frames share each slit (the shimmer).
- Every slit edge is a one-pixel box filter (the barrier runs in sub-pixel steps), every ring / spoke edge
  is the hypnos analytic one-pixel filter. A pixel half covered by the slit shows the sliver its lit part
  lies in, so the rest frame has no stray lines from the neighbouring frame.

## Controls

| Control | Name | Function |
|---|---|---|
| K1 | X | figure centre, +-2 screens, detent at the middle |
| K2 | Y | figure centre, +-2 screens, detent at the middle |
| K3 | Period | Rings: ring period 896 .. 56 virtual px (exponential, 4 octaves; default 160). Spokes: 1 .. 16 arms (stepped; default 10) |
| K4 | Frames | N = 2 .. 8 (stepped with hysteresis; default 4) |
| K5 | Pitch | sliver width 4 .. 32 virtual px (exponential; default 8), so the barrier period is N x that |
| K6 | Camera | how far the picture pushes the barrier: 0 .. about one period at white (default 100 %) |
| S7 | Glide | On: Period, Pitch, Camera and Slide ease 1/16 of the gap per field, the centre eases (never slower than 1 px per field, lands exactly). Off: no easing, values used the same field |
| S8 | Figure | Rings / Spokes |
| S9 | Barrier | Slits / Inverse (the sheet inverted: N - 1 of N slivers show, a dense moire that still animates) |
| S10 | Weight | Bold = slit one sliver wide, Fine = half a sliver (crisper frames, darker). In Inverse it thins the bar |
| S11 | Source | Video / Osc |
| P12 | Slide | barrier position, bipolar: centre = rest (+-8 counts of deadband), +-4 barrier periods at the ends (four loops of the animation each way; one frame = 32 counts at N = 4). Past half travel the sheet also LEANS, linearly up to two periods top to bottom at the ends: the frames stack down the screen and the target shears into an egg. 0 % and 100 % lean opposite ways |

Knob channels hold a change of one count or less (pot jitter), never ease it.

## Camera

The camera is never shown. Its luma (above black, 10 bits) goes through a 32-px box and a vertical 1/8
one-pole (quarter horizontal resolution, linearly interpolated), and shifts the BARRIER sideways:
shift = luma x K6, so each region of the picture shows the animation frame its brightness selects and the
slits bend around the subject. Gain: 25 % = 0.24 of a period at white (a wobble of about one frame at N = 4),
50 % = 0.48, 100 % = 0.96 of a period (the whole loop from black to white). Black input = no shift = the
clean figure. S11 Osc substitutes the shell's drifting triangle lattice (640 x 540 virtual px) at 4x the
camera level; it is the only thing that moves by itself.

## Engine

- **Per field** (vblank sequencer started on the vsync edge after active video, 34 states, one serial
  shift-right multiplier): six knob channels (Q6, one shared datapath), the centre target and glide,
  log2 frequency -> mantissa, sliver width w = 2^(...) from a second copy of the exp2 table (real pixels,
  Q10), P = N w, slide = P x frac, lean and lean per line, camera gain, the camera table
  t(L) = L x gain mod P (1024 entries, one EBR set, two clocks per entry), and the lattice seed: the two
  pixel DDAs are run from zero for W / 2 clocks in vblank and what they reach is mirrored into pixel 0.
- **Per line**: the line seed leans one line further (one modular add / wrap on a single adder).
- **Per pixel** (39 clocks, the same in every mode): hypnos's log-domain polar chain (log2 hi / lo ->
  log2 r, theta from interpolated EBR tables; exp2 mantissa x frequency mantissa and one shift = ring phase;
  arms x theta = spoke phase; analytic filter width, half-octave reciprocal) -> minus k / N of a cycle ->
  one-pixel box filter of the band; in parallel a lattice DDA (sliver index) and a barrier DDA minus the
  camera table -> two clamps = slit coverage; min(figure, slit) -> Y = 64 + 712 c -> [1 2 1] / 4 ->
  blanking gate. U = V = 512 on every pixel. No per-pixel multiplier was added to hypnos's (the camera
  shift is a table, the product of the two coverages is a min).
- Virtual 1080-line geometry as hypnos v1.0.4 (timing-ID steps); the lattice is in real pixels scaled by
  the x step, anchored on the measured centre, so SD shows the centre 4:3 of the same picture.
- Generated avid (parity-locked), frame tick on the vsync edge with the saw-active guard, engine restarted
  every field.

## Status

- Router1, seed 1 on all six configs (2026-11-15), post-route Fmax: HD Analog 87.43 MHz, HD HDMI 80.30 MHz,
  HD Dual 82.36 MHz; SD Analog 84.43, SD HDMI 82.55, SD Dual 74.27 MHz (SD needs 27).
- 7085 .. 7111 LC of 7680 (92 %), 24 of 32 EBR. No `hd_clock_divisor`. 0 duplicate-input carries (dupin.py).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline kinegram`
  (router2 was not tried). vmprog: `out/rev_b/ron/hardline/kinegram.vmprog`.
- GHDL render vs the integer model (`.hardline_work/rb_kinegram/imodel.py`), pixel diff after aligning the
  4-column capture offset: 0 of 918,720 at 720p60 decimation 1 (default frame, black input); 0 at 640 x 360
  for Spokes + Fine + 6 frames + off-centre, for Inverse + 3 frames + P12 100 % (the lean), for the camera
  (bear.png, K6 100 %) and for Osc; 0 on an NTSC field (720 x 243, decimation 1: the centre 4:3 of the same
  picture, rings round). Glide was off in every sim: the eased paths (S7 On) are not simulated.
- Not flashed. First things to look at on hardware: the vblank lattice-seed run (the pixel DDAs are started
  from the sequencer in vblank), HD HDMI bars (hard vertical full-swing edges every few pixels; the
  [1 2 1] and the 776 white are in), and the power-up glide of the centre from the top-left corner (as hypnos).

## Cut / deviations

- **The lean on P12 is an addition to the spec.** A pure +-4.0-period slide is cyclic: 0 %, 50 % and 100 %
  would be the same picture. The sheet now turns as it nears either end of travel (as a hand-slid sheet
  does), which gives 100 % a climax and leaves two clean loops each way around the centre.
- Pitch is 4 .. 32 virtual px, not 2 .. 32: at 2 px the Fine slit would be 1 virtual px.
- Ring period 896 .. 56 virtual px: finer rings only alias against the barrier.
- Output white is 776: the set's `y - (d >> 3) - (d >> 4)` is folded into the luma scale (64 + 712 c)
  ahead of the [1 2 1] filter, which is the same arithmetic in a different order.
- Figure AND slit is min(coverage, coverage), not their product (they differ only on the few pixels where
  a ring edge crosses a slit edge).
- Pitch has 1/256-octave steps with no interpolation (cut for fit): at the screen edge a slow Pitch glide
  ticks about 2.6 px per step at 1080p. Period keeps hypnos's 1/64-octave mantissa.
- Osc is 4x the camera level (a shift, not 3x).
- The slit edges assume the camera shift changes slowly across a pixel: where it changes fast (strong
  edges at large Pitch x Frames) the edge is sharper or softer than one pixel.
- Camera luma is not pipeline-aligned (the box sits a few pixels left of the pixel it bends).
- SD: at the minimum Pitch a Fine slit is one real pixel wide (dim after the [1 2 1] filter).
