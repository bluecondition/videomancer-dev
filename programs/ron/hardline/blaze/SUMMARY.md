# Blaze (Hardline)

**v0.2.0 built 2026-11-16, not flashed.** Flash `out/rev_b/ron/hardline/blaze.vmprog`.

v0.2.0 (Ron, 2026-11-16) adds Hypnos-style self-motion: K5 is now **Speed**, and the zigzag rings stream out
of (or into) the grey centre. **Skew was removed for Speed**: the zigzag apex is fixed at 1/2 (even chevrons,
the v0.1.0 default). Everything else is as v0.1.0.

## What it draws

Bridget Riley's "Blaze 1" (1962) and "Blaze 4" (1964): a black-and-white sunburst whose rays change
direction at every ring, so the flat disc reads as a stack of twisted cones. The picture is log-polar, so it
is self-similar all the way to the centre: every ring is the same figure, smaller. The default frame is 24
rays zigzagging through about 1.5 rings per octave of radius with even chevrons, the centre on the screen
centre, the innermost 30 pixels fading to flat grey. At the power-up Speed the chevrons flow outward along
the rays, about 1.2 rings per second, endlessly: the picture is self-similar, so the travel never jumps.

    v    = m * log2(r / 512 virtual px) - travel        m = rings per octave (K3); travel from K5 Speed
    t    = frac(v)                                       position inside the ring
    zig  = 2t below 1/2, 2(1 - t) above it               (v0.1.0: the apex moved on K5 Skew)
    PHI  = Rays * theta / 2pi + Twist * zig + camera                 S8 Zigzag
         = Rays * theta / 2pi + Twist * v0  + camera                 S8 Checker: a log spiral (v0 = v without
                                                                     the travel), then XOR the ring band t < 1/2
    ink  = frac(PHI) < duty                              S9 Flat (S10: duty 1/2, or a thin line)
         = frac(PHI) as a sawtooth ramp                  S9 Shade (S10: 4 flat steps, or smooth)

Both terms of the gradient of PHI fall as 1 / r, so the anti-aliasing filter width is one per-field constant
per zigzag segment over r, and the centre fades to grey by itself.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | centre, horizontal: detent = screen centre, the ends are 2 screens off-frame (the rays become near-parallel chevron stripes) |
| K2 | Y | centre, vertical, same range |
| K3 | Period | rings per octave of radius, 0.5 .. 8, exponential (default 1.5). The rings breathe about the 512-pixel radius |
| K4 | Rays | 16 steps: 4 5 6 7 8 10 12 14 16 20 24 28 32 40 48 64 whole rays (default 24), with hysteresis |
| K5 | Speed | ring travel (v0.2.0, replaced Skew), bipolar: a 48-count dead centre = still, up = the rings stream OUT of the centre, down = in. Exponential, 1 ring per 68 s just off the dead zone to 1/4 ring per field at either end (15 rings per second at 60 Hz). Default 844 (+65 %): about 1.2 rings per second outward. Zigzag: the chevrons flow along the rays; Checker: the ring family flows through the fixed spiral. Seamless: the ring phase wraps modulo one ring |
| K6 | Camera | how far the camera (or Osc) bends the rays: 0 % none, 25 % half a ray period at white, 50 % one, 100 % (default) two |
| S7 | Glide | On: X / Y ease 1/16 of the gap per field (never slower than 1 px per field, landing exactly); Period, Speed, Camera and Twist ease 1/16 per field (the speed eases, never the travelled phase, and nothing stops the travel but a speed of zero). Off: no easing, the value read at this field start is used this field. Either way a knob is followed only once it has moved 2 counts (pot jitter is held, not eased) |
| S8 | Form | Zigzag / Checker: rays XOR rings, a radial checkerboard tunnel that Twist shears into a spiral checkerboard |
| S9 | Shade | Flat / Shade: a sawtooth ramp across each ray band (folded-paper pleats) |
| S10 | Weight | Flat: Bold (equal bands) / Fine (white lines 1/6 of the period on black, never under 2 px). Shade: Bold = 4 flat steps (black, two greys, white) / Fine = a smooth ramp |
| S11 | Source | Video / Osc |
| P12 | TWIST | the headline, bipolar. Centre (a 16-count dead zone) = perfectly straight rays, a Siemens star. Either way the zigzag deepens to 2.46 ray periods per ring, mirrored: 100 % is Blaze at full violence. Default +1.6 periods (82 %) |

At the exact centre of P12 the rays are straight and K3 / K5 have nothing to shape (in Zigzag): a still
Siemens star, the spec's centre. In Checker the rings still flow there. Everywhere else every control
changes the picture, and all of them keep working while it moves; the camera bends the moving lines.

## Camera

The camera is never shown. Its luma (10 bits -> (Y >> 2) - 16, so black is exactly 0) goes through a
horizontal box (64 px at 1080, 32 px in the other modes), a vertical one-pole filter in a half-resolution
line store (1/32 per line at 1080p, 1/16 at 1080i / 720p / 480p / 576p, 1/8 at 480i / 576i, so the blur
is about 32 to 64 virtual pixels in every mode), a linear interpolation back to full resolution, times K6,
and is added to PHI. Bright areas push the rays round the centre: the subject appears as bent linework, and
every edge stays a clean curve because the bend is smooth over tens of pixels. Black input = zero = the
clean figure. Gain at white: 25 % = 0.42 period, 50 % = 0.85, 100 % = 1.7 (2 periods at luma code 256).
The field is tapped shallow and not delayed: the bend sits 18 pixels less the filter's centre (about 14
pixels at 1080, 2 at SD) beside the subject. Osc substitutes two triangle waves (512 virtual px, drifting one
period in 256 fields, the only thing that moves by itself) at the same point, at 1.5 x the camera gain and
full swing: up to 3 periods of bend.

## Engine

Hypnos v1.1.0 chassis: timing-ID virtual coordinates, generated avid (parity-locked to hsync), centre
glide, log-domain polar engine, analytic one-pixel box-filter anti-aliasing. Hypnos's shapes, warp, exp2,
spiral, gradient combine and palettes are gone; the source field is lifted from Kinetic riley.

- **Per field** (first vsync edge after active video, run in blanking, 121 states, one small operation
  each): six knobs through a 2-count hold and the glide; rays; m from an exp2 ROM; Twist; the centre (one
  serial multiplier, the Hypnos glide); the slopes A = B = 2 Twist, Tm = Twist * m; log2 of each (a serial
  normaliser) for the filter-width constants. SPEED, as Hypnos: |K5 - 512| < 24 is still, else
  2^(-12 + u / 48.8) ring per field (exp2 ROM mantissa shifted one bit per clock); a 20-bit ring-phase
  accumulator (one ring, wraps seamlessly) is stepped once per field, divided by m (the serial divider,
  23 clocks) and folded into the log2-radius reference the ring coordinate uses, so the pixel path gains one
  parallel subtract and no depth. The Checker's spiral keeps the unshifted coordinate (a whole ring of travel
  would otherwise turn it by Twist ray periods: a seam).
- **Per pixel** (40 clocks, the same in every mode):
  S1 .. S15 |dx|, |dy| -> log2 r and theta (Hypnos, block-RAM tables);
  S16 .. S18 L = log2(r / 512) - travel / m (and L0 without it, for the spiral), Rays * theta as two gated
  adds and a shift (rays are (4 + a) << b), v = m * L;
  S19 .. S22 the zigzag fold and ONE multiply X * Y (t or 1 - t times the slope, or L0 times
  Tm in Checker), PHI = Rays * theta +- z + camera (Q16 cycles);
  S20 .. S25 filter width log2 w = LG - log2 r, widened to a full cycle at Nyquist (flat grey),
  rounded up to a half octave so the divide is a shift;
  S26 .. S31 the box filter (unit A). Shade reuses it: the sawtooth's mean is lo + w/2 minus the part of the
  window past the reset; the 4-step ramp runs the unit on 4 * PHI (the quarter boundaries);
  S28 .. S31 unit B, the Checker's rings: signed distance to the nearer ring edge times 1 / w, 17 levels;
  S32 .. S34 tone, XOR as tone + (256 - 2 tone) * ring by shift-adds;
  S35 .. S39 luma [1 2 1] / 4 with replicated picture edges, 11/16 toward black (white = 768), blanking
  gate. U = V = 512 on every pixel.
- Long carries ride in seven block-RAM delay lines (never reading the address being written, `no_rw_check`).
- Multipliers per pixel: m * L (18 x 10), X * Y (18 x 14), the camera gain (11 x 7), and the two small
  Hypnos table interpolations. No divider, no square root.

## Status

- **v0.2.0** built 2026-11-16 through the shared queue with **router1**, **seed 1 on all six configs**. Not
  flashed. Post-route Fmax (the builder's last "Max frequency" line per run): HD Analog 77.15 MHz, HD HDMI
  81.10, HD Dual 79.15, SD Analog 78.97, SD HDMI 78.05, SD Dual 79.61. HD needs 74.25, SD 27.
  7,023 .. 7,086 / 7,680 LC (91 .. 92 %), 23 / 32 block RAMs. No `emulate_transparency`, no carry self-adds
  (dupin: 0). Six bitstreams pass iceunpack. Latency 40 clocks = `processing_delay_clks`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline blaze`.
- v0.2.0 verification (GHDL against `imodel.py`, which now models the speed decode, the accumulator and the
  divide by m), all with 0 differing pixels:
  - still: Speed at centre (K5 512), 720p60 decimation 1, Glide Off: 0 of 918,720 against the model, and the
    output is byte-identical to the v0.1.0 RTL default frame;
  - motion, 720p60 decimation 2, Glide Off, K5 950 (+85 %) and K5 74 (-85 %), each captured after 1 and 3
    fields of travel: each frame equals the model with the ring phase advanced by that many steps of the
    decoded speed (0.0864 ring per field). Frame B is frame A zoomed by 2^+0.115 about the centre at +85 %
    (rings out) and 2^-0.115 at -85 % (rings in); expected 0.117 octave. `motion.png`;
  - Glide On at the power-up defaults (K5 844): the travel runs at the default speed from the first field
    (model: 2 fields of travel + 2 fields of the centre glide from the power-up position).
  The Checker's ring flow, the camera with motion and the 1080 rasters were not simulated for v0.2.0.
- v0.1.0 (2026-11-15): router1 seed 1 on all six; HD Analog 81.23, HD HDMI 81.56, HD Dual 76.25, SD Analog
  75.41, SD HDMI 78.19, SD Dual 79.41 MHz; 6,985 .. 7,043 LC. Its model = RTL record (all 0 differing pixels):
  - 720p60 decimation 1 (1280 x 720), defaults, black input: 0 of 918,720;
  - 720p60 decimation 1, camera = bear.png at K6 100 %: 0 of 918,720;
  - 720p60 decimation 1, S11 Osc: 0 of 918,720;
  - NTSC decimation 1 (720 x 243 per field), defaults: 0 of 173,988 in both fields, and the field shown at
    4:3 is the same picture as the centre 4:3 of the 720p frame (`ntsc_vs_720p.png`);
  - 640 x 360 (720p60 decimation 2): defaults; Shade 4 steps; Checker; Checker + Shade 4 steps with a negative
    Twist, 8 rays, Skew 300, Period 600 and the centre at the left edge; Shade smooth + Fine + Osc with 64
    rays, Skew 800, Period 200; camera with Fine.
  The GHDL frame is the model's shifted 4 columns right with 4 blank columns (the generated avid starts 4
  clocks after the source's, as on Hypnos). 1080 rasters were not simulated.
- Renders (work dir `.hardline_work/rb_blaze/`): `motion.png` (v0.2.0), and from v0.1.0 (Skew, no motion):
  `cmp_final.png`, `sheet.png`, `strip.png`, `camera.png`, `presets.png`.
- The first v0.1.0 build missed timing on HD Analog on seeds 1 to 4 (70.9 MHz on seed 1 run alone): the knob
  glide's compare, select and add sat in one sequencer state. Split over two states it closed at 81 MHz.

## Cut / deviations

- **m defaults to 1.5, not 2.7.** The concept sketch's 1.9 is per natural-log unit, which is 1.3 rings per
  octave; 2.7 rendered much busier than the sketch. K3 still spans 0.5 .. 8.
- **Rays are 16 labelled steps**, not every whole number from 4 to 64 (the TOML allows 16 labels, and
  (4 + a) << b needs no multiplier).
- **v0.2.0: Skew was removed for Speed** (Ron asked for self-motion; P12 Twist stays the headline). The apex
  is fixed at 1/2, the v0.1.0 default, so the "saw blade" looks are gone, and the Checker's rings are always
  half duty. Removing it also dropped the per-field divider for the two slopes and one zigzag segment's
  constants, which paid for most of the motion (+67 LC in the standalone count).
- **Speed glide is 1/16 per field** like the other glided knobs (Hypnos uses 1/8 for its speed).
- **The top speed stays at Hypnos's 1/4 ring per field.** Under the Nyquist half-cycle, so the direction
  reads correctly; at 60 Hz it is a fast strobing pour.
- **Power-up centre** is now the 1080 screen centre (was the top-left corner), so at 1080 with Glide On the
  figure no longer slides in from the corner after power-up; at 720p / SD it slides the short way.
- **Twist is not centred on the ray**: the zigzag runs 0 .. Twist, so the ring boundaries stay put and the
  apexes swing. (Centring it cost an adder for no visible gain.)
- The presets were updated for Speed: "Saw Blades" (it needed Skew) became "Warp Out" (fast outward);
  "Spiral Checker" pours inward; "Paper Pleats" drifts slowly out.
- **Camera gain is 2 periods at luma code 256 (1.7 at video white), linear in K6.** 2.5 periods shredded
  bright texture into islands on the test image.
- **Output scaling is 11/16 (white 768)** in one step after the [1 2 1], instead of 13/16 of 64 .. 940
  (776): the same thing within 1 %, one adder less, and at CLAUDE.md's luma cap.
- **The Checker's ring edges** use a one-pixel linear ramp whose width is taken from the ray filter's
  (rounded to the sharper octave: 0.6 .. 1.2 px), not a second full box filter; rings finer than about 3 px
  go to the flat duty grey. A second full unit cost 470 LUT.
- **Filter-width constants are approximate** (log2 with a linear mantissa, the two gradient terms combined as
  max + 1/4 octave): the width is rounded up to a half octave anyway.
- A camera-gradient term in the anti-aliasing width was modelled and dropped (no visible difference at this
  gain).
- Fit: the first RTL was about 8,500 LC; see `.hardline_work/rb_blaze/notes.md` for what was cut.
