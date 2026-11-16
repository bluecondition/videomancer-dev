# KINEGRAM: a barrier-grid animation that plays itself (Hardline)

**v0.2.0 built 2026-11-16, not flashed** (v0.1.0 2026-11-15: P12 was a hand Slide). Polar chassis (hypnos v1.1.0 engine), `program_type = "processing"`.

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
- **Speed (P12)** slides the barrier sheet continuously, as hypnos's Speed moves its rings: one sliver of
  travel is the next frame, so the frames play forward (rings grow outward, the wheel turns) or backward;
  between frames two frames share each slit (the shimmer). At the centre it stops, landing on one clean frame.
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
| S7 | Glide | On: Period, Pitch and Camera ease 1/16 of the gap per field, the Speed 1/8 (never the position), the centre eases (never slower than 1 px per field, lands exactly). Off: no easing, values used the same field. Glide never stops the motion |
| S8 | Figure | Rings / Spokes |
| S9 | Barrier | Slits / Inverse (the sheet inverted: N - 1 of N slivers show, a dense moire that still animates) |
| S10 | Weight | Bold = slit one sliver wide, Fine = half a sliver (crisper frames, darker). In Inverse it thins the bar |
| S11 | Source | Video / Osc |
| P12 | Speed | bipolar, as hypnos: centre +-24 counts = stopped (the sheet lands on the nearest clean frame at 1/32 sliver per field); up = forward, down = backward; exponential over 10 octaves, 1/2048 .. 1/2 sliver per field (at 60 fields/s: one frame per 34 s .. 30 frames per second, i.e. 7.5 loops a second at N = 4). Default 862 (68 %) = 0.05 sliver per field, 3 frames per second. The position wraps modulo one barrier period (N slivers), so it runs forever with no seam |

Knob channels hold a change of one count or less (pot jitter), never ease it.

## Camera

The camera is never shown. Its luma (above black, 10 bits) goes through a 32-px box and a vertical 1/8
one-pole (quarter horizontal resolution, linearly interpolated), and shifts the BARRIER sideways:
shift = luma x K6, so each region of the picture shows the animation frame its brightness selects and the
slits bend around the subject. Gain: 25 % = 0.24 of a period at white (a wobble of about one frame at N = 4),
50 % = 0.48, 100 % = 0.96 of a period (the whole loop from black to white). Black input = no shift = the
clean figure. S11 Osc substitutes the shell's drifting triangle lattice (640 x 540 virtual px) at 4x the
camera level. The camera's shift adds to the moving barrier, so the subject stays a time-displaced zone of
the playing animation.

## Engine

- **Per field** (vblank sequencer started on the vsync edge after active video, 35 states, one serial
  shift-right multiplier): six knob channels (Q6, one shared datapath), the centre target and glide,
  log2 frequency -> mantissa, sliver width w = 2^(...) from a second copy of the exp2 table (real pixels,
  Q10), P = N w, the Speed (mantissa from the exp2 table, shifted by the octave one bit per clock), the
  position accumulator (3 whole slivers mod N + a 20-bit fraction; one add, one wrap per field), the slide
  = w x position, camera gain, the camera table
  t(L) = L x gain mod P (1024 entries, one EBR set, two clocks per entry), and the lattice seed: the two
  pixel DDAs are run from zero for W / 2 clocks in vblank and what they reach is mirrored into pixel 0.
- **Per line**: nothing (v0.1.0's per-line lean is gone).
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

- v0.2.0: router1, seed 1 on all six configs (2026-11-16), post-route Fmax: HD Analog 83.84 MHz,
  HD HDMI 83.70 MHz, HD Dual 84.55 MHz; SD Analog 83.89, SD HDMI 91.96, SD Dual 85.72 MHz (SD needs 27).
- 6990 .. 7046 LC of 7680 (91-92 %), 24 of 32 EBR (v0.1.0: 7085 .. 7111). No `hd_clock_divisor`.
  0 duplicate-input carries (dupin.py).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline kinegram`
  (router2 was not tried). vmprog: `out/rev_b/ron/hardline/kinegram.vmprog`.
- GHDL render vs the integer model (`.hardline_work/rb_kinegram/imodel.py`, now with the Speed
  accumulator), pixel diff after aligning the 4-column capture offset:
  - v0.2.0 still (Speed at the centre, 720p60 decimation 1, black input): 0 of 918,720 against the model, and
    0 pixels different from the v0.1.0 default-frame capture.
  - v0.2.0 motion (720p60 decimation 2, Glide off): Speed +91 % and -90 % (0.25 sliver per field), each
    captured after 1 and after 4 fields: all four 0, the model's accumulator at the same field count. Up
    plays forward (rings move outward), down backward (`motion.png`).
  - v0.1.0 (same engine): 0 for Spokes + Fine + 6 frames + off-centre, Inverse, the camera (bear.png, K6
    100 %), Osc, and an NTSC field (the centre 4:3 of the same picture, rings round).
  - v0.2.0 Glide On at the power-up defaults (S7 On, Speed 68 %): renders clean and moving; the speed
    glide starts at the TOML default so there is no speed transient; the centre slides in from the
    top-left corner over the first second (as hypnos and v0.1.0). Not compared to the model.
- Not flashed. First things to look at on hardware: the vblank lattice-seed run (the pixel DDAs are started
  from the sequencer in vblank), HD HDMI bars (hard vertical full-swing edges every few pixels, now moving;
  the [1 2 1] and the 776 white are in), and the power-up glide of the centre from the top-left corner (as
  hypnos).

## Cut / deviations

- v0.2.0: the v0.1.0 lean is dropped. It existed only because a pure hand slide made the slider's ends
  look like its centre; with a Speed the ends are the fastest animation, the climax.
- v0.2.0 top speed is capped at 1/2 sliver per field (30 frames per second at 60 fields/s), two octaves
  below hypnos's 1/4 cycle: past about one sliver per field the frames strobe or read backwards. The speed
  is in slivers (frames) per field, so the frame rate does not change with Frames or Pitch.
- v0.2.0: stopped at the centre, the sheet lands on the nearest clean frame (1/32 sliver per field, at
  most 16 fields) rather than freezing between two frames. This landing is not affected by Glide.
- The deadband edge on the down side is one count wider (the magnitude is a one's complement: 487 is the
  slowest backward setting, 536 the slowest forward one).
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
