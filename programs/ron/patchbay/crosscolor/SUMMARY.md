# Crosscolor — a zone plate through a fake composite decoder: rainbow ghost rings where its pitch meets the subcarrier

**v0.1.0 built 2026-11-14, not flashed.** Patchbay (a patch that was never built). A luma-only pattern oscillator
is fed to a fake composite decoder; the decoder mistakes luma detail near its colour subcarrier for chroma, so the
fine rings turn into rainbow moire (cross-colour), and the false chroma can be sent back into luma as crawling
dots. Working files: `.patchbay_work/rb_crosscolor/` (`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`,
`sheet.png`, `strip.png`).

## The patch
```
pattern VCO (quadratic phase: zone plate, or a sheared chirped grating) -> soft clip (x1.6) -> band-limit VCA
camera luma (or the Osc) ----------------------------+
                                                     v
pattern luma -----------------------> non-additive mixer (max)  =  "composite" ----------------> luma out
                                                     |                                              ^
                                                     v                                              |
      half-subcarrier-period comb  Y(x) - Y(x - P/2)       (the decoder's chroma band-pass)         |
                                                     v                                              |
      x square subcarrier: 0 deg -> U, 90 deg -> V         (demodulators: add or subtract, no multipliers)
                                                     v                                              |
      running box, 0.5 .. 8 subcarrier cycles              (chroma low-pass; Bandwidth)             |
                                                     v                                              |
      limit at +-1024 -> recursive line comb x1.5 -> limiter (Saturation) -> U, V out               |
                                                     |                                              |
                                                     +--> second encoder (its own crawling line phase) --+  (Dot Crawl)

subcarrier VCO: phase = x / P + quadrant(line) + burst phase (Hue).  The quadrant steps once per "line", a strip
                0.75 P tall (180 deg flips) or 0.5 P tall (90 deg steps), and the strips crawl every field.
modulators:     pattern phase drift (Speed) | subcarrier field advance | the plate's centre on a slow Lissajous
                (23 s and 31 s triangles) | the gratings' shear (17 s triangle) | the second encoder's crawl
```

## What it draws
- **Default frame.** A black-and-white zone plate (soft-clipped sine rings, 1/24 px local frequency 480 px from the
  centre, contrast falling off toward the fine outer rings) with four round rainbow ghost ring systems where the
  ring pitch equals the subcarrier's: (+-480, +-320) virtual px from the plate's centre. Each ghost is a saturated
  flat core (the limiter) wrapped in rainbow rings that get finer outward; small third-harmonic ghosts sit at a
  third of the radius, and faint lattice ghosts further out.
- **Why there.** With phase kappa r^2 the plate's local frequency is 2 kappa r. Chroma = the beat between the
  plate and the subcarrier, so a ghost is another zone plate centred where the two frequencies match:
  x0 = 1 / (2 kappa P) horizontally, y0 = 1 / (2 kappa Pv) vertically (Pv = 1.5 P for line flips, 2 P for 90 deg
  steps). Line flips are a standing wave, so all four diagonals show a ghost; 90 deg steps are a travelling wave,
  so only one diagonal does.
- **Motion** (defaults): the rings stream out of the centre (0.37 cycles/s); every ghost's rainbow rings pulse
  through the hue wheel (the plate's drift minus or plus the subcarrier's crawl, so the upper and lower ghosts turn
  at different rates); the plate's centre wanders on a slow Lissajous (+-128 px, +-64 lines), carrying the ghosts.
- **P12 BLEED** drives the luma into the demodulator: 0 % is the clean monochrome plate; ghosts fade in from their
  cores; at 100 % every box side lobe and harmonic ghost is limited flat: rainbows nearly everywhere.

## Controls
| Control | Module | What it does |
|---|---|---|
| K1 Pattern | pattern VCO | the zone plate's chirp / the gratings' pitch, x0.35 .. x2 (exponential; 512 = default). Low: broad rings, the ghosts leave the screen and only the third-harmonic ghosts remain. High: fine rings, ghosts close to the centre, the outer rings fade out before they can alias |
| K2 Subcarrier | subcarrier VCO | its period, 48 .. 14.3 virtual px (exponential; 585 = 24 px): where on the pattern the rainbows land, and the size of the strips and dots. Left: ghosts crowd the centre in tall colour bars; right: ghosts at the screen's corners |
| K3 Saturation | limiter | the limiter's level = the chroma's peak, 60 .. 389 (10-bit chroma units): pale pastel .. vivid |
| K4 Speed | pattern VCO phase | bipolar (+-8 deadband, square law): the rings stream out (right) or in (left), up to 3.6 cycles/s; 680 = the default drift. The subcarrier's crawl and the Lissajous never stop |
| K5 Bandwidth | decoder low-pass (box) | box length 0.5 .. 8 subcarrier cycles: wide band = large ghosts with fine rainbow fringes; narrow band = broad flat pastel smears. Above 2 cycles the line comb lengthens too |
| K6 Hue | subcarrier VCO (burst phase) | rotates every decoded colour, one full turn over the knob |
| S7 Pattern | pattern VCO | Zone Plate / Gratings: a chirped grating sheared 19 .. 40 deg, swaying. The rainbow becomes long diagonal bands that open wide when the grating's tilt matches the subcarrier's line phase (striped-shirt moire) |
| S8 Comb | line comb | On: a recursive vertical average of the decoded chroma: compact smooth ghosts / Off: the raw decoder, every strip its own hue (Hanover bars) in full-height bands |
| S9 Dot Crawl | second encoder | Off / On: the decoded chroma is remodulated on a second subcarrier and added to luma: a crawling checker of dots wherever there is colour |
| S10 Standard | subcarrier VCO (line phase) | 180 Flip: four ghosts / 90 Steps: two ghosts on one diagonal, strips in four hues |
| S11 Source | mixer input | Video / Osc |
| P12 BLEED | demodulator drive | pre-limiter gain 96 (p/1023)^3 (glided): clean mono .. four ghosts (49 %) .. rainbows everywhere |

Presets (hand-written): Zone Plate, Clean Mono, Rainbow Flood, Hanover Bars, Striped Shirt, Test Card Osc,
Pastel Smear.

## Video
- **Video (default).** Camera luma (one line buffer, 8 bits) goes into a non-additive mixer with the pattern luma
  (output = the brighter of the two), ahead of the decoder. The plate's dark rings become windows onto the camera,
  bright camera areas cover the plate, and the camera's own fine detail (text, keys, fabric, sharp edges) decodes
  to rainbow like any composite picture. The rings are never bent or shredded. With a black input the camera term
  sits below every pattern level, so the picture is exactly the patch alone.
- **Osc.** The same input gets the shell's slow 640 x 540 px triangle lattice plus a fine tilted triangle grating
  (26 px, near the default subcarrier, drifting). Its pitch beats with the subcarrier, so broad diagonal rainbow
  bands sweep the whole screen between the ghosts; Subcarrier tunes the bands (wide at zero beat).

## Engine
- **Per field (vblank zone).** One serial multiplier (38 x 17 bits, mod 2^38 because phase only matters mod one
  cycle), 37 slots of 32 clocks. The multiplicand can be the previous product, the exp2 ROM is addressed per slot,
  and the exponent's integer part is a pre-shift of the multiplicand. One accumulator sums the products.
  It computes: the chirp (exp2), shear products, the line-0 phase A X0^2 + B X0 Y0 + C Y0^2 - drift, the first
  difference at the line start, the line-to-line step and its step, the second difference; the subcarrier step,
  its SD window offset and the strip step; the box length, the half period, the limiter level and the gain
  (bleed^3 x level / box length, Q12). Four knobs glide (K1, K2, K5, P12; 1/8 of the gap per field).
  Modulators are free-running 16-bit counters folded to triangles.
- **Per line.** Five adds after each line start: phase += step; step += step2; first difference += its line step;
  strip phase += strip step; then the next line's start quadrant and the dot encoder's quadrant offset.
- **Per pixel (24 stages).** Phase Q28 by second differences (Q += D; D += DD) -> 512 x 8 half-sine ROM ->
  amplitude from the local frequency (1 -> 1/2 by a quarter cycle per pixel or line, then to 0 before Nyquist) ->
  max with the camera -> minus the ring tap half a period back -> x gain -> +- by the subcarrier quadrant -> two
  running box sums (a 256 x 32 ring holds the products; resynced to zero once the box is full of blank samples)
  -> limit -> comb (state per two pixels in a 1024 x 32 RAM, state += (in - state) / 2^n, n from the strip height
  and the line step) -> limiter -> dots -> output conditioning.
- **Latency 48** in every mode. The chroma stream runs 25 pixels ahead of the luma it is laid over (the luma waits
  in a ring), which cancels the box's lag at the default settings; syncs ride a block-RAM ring.
- **Contracts.** Generated avid; one latency; chroma [1 2 2 2 2 2 2 2 1]/16 (resynced 8-sums), luma [1 2 1] and
  13/16 (white ~756) before the blanking gate; virtual 1080-line coordinates (SD shows the centre 4:3); colours
  in the hardware convention (U = Cr, V = Cb); every running sum has a resync.
- **Memory.** 21 EBR: line buffer 4, comb store 8, product ring 2, chroma 8-sum ring 2, luma rings 2, sine 1,
  exp2 1, sync ring 1.

## Status
- **v0.1.0 built 2026-11-14 with router2, NOT flashed.** `out/rev_b/ron/patchbay/crosscolor.vmprog`.
  Rebuild: `./build_set.sh patchbay crosscolor`.
- Post-route Fmax: HD Analog 79.50 (seed 1), HD HDMI 79.88 (seed 1), HD Dual 80.78 (seed 1), SD Analog 71.72
  (seed 1), SD HDMI 78.69 (seed 1), SD Dual 77.33 MHz (**seed 2**: seed 1 stalled the router).
  6,911 - 6,937 / 7,680 LC (90 %), 21 / 32 EBR.
- First synthesis was 8,633 LC (112 %): the sequencer was rebuilt around the multiply-accumulate slots, pipeline
  delays moved into the rings' read offsets, and the amplitude law lost its two multipliers.
- **Verified (GHDL against the integer model `imodel.py`, every frame compared on all of Y, U, V; the sim
  picture starts 4 px right of the model's, the generated avid's start):**
  - 720p60, decimation 1, defaults, black input: **0 of 918,720 pixels differ**;
  - 720p60 (320 x 180 corner) four fields later: 0 differ (the motion is the model's);
  - Gratings + 90 Steps + Dot Crawl: 0 differ; Comb Off + Dot Crawl + K3 900 + K6 200 + K4 300: 0 differ;
  - K1 800, K2 200, K5 800, P12 900 while the knobs glide: 0 differ;
  - NTSC (720 x 243 fields, decimation 1), both fields: 0 differ; 1080i (960 x 270 corner), both fields: 0 differ.
  - Source Osc and the camera (selfie.jpg) at 720p60: `cmp_final.png`. The Osc frame matches the model in
    structure and hue but not bit for bit (the model's Osc generator is not cycle-exact).
  - `cmp_modes.png`: the same field at 1080p, 720p, 1080i (model) and NTSC (GHDL): one picture.
- Not checked on hardware: everything. Watch for: HD HDMI bars (the luma is a sharp moving ring pattern behind a
  [1 2 1] only; the roll-off keeps the finest rings at half contrast), and Comb Off (see below).

## Cut / deviations
- **K3 Saturation** is the limiter's level, not a second gain: the spec's "decoder chroma gain" is P12's job, and
  two gains on one signal would eat each other's travel.
- **S8 Comb** is a recursive vertical average of the decoded chroma (one line store), not a two-line comb: it is
  what makes the ghosts compact. It does not stop the hue pulsing (the field advance still turns the ghosts);
  with it off the strips themselves roll.
- **Gratings** are sheared, not rotated (exact integer arithmetic, no trig), and there is one grating, not several.
- **Dot crawl** needs S9 on, also at P12 = 100 %.
- **The amplitude roll-off is in real pixels and lines**, so the fine outer rings fade earlier at 720p and SD than
  at 1080p (SD's top and bottom rings are at about 65 % of the 1080p contrast). That is the fine pitch SD gives up:
  K1 and K2 at their fine ends alias first there, and the roll-off turns that into a fade. The strips (0.75 P
  tall) are about four SD lines at the default and under two at K2 = 100 %, where SD shows coarse hue bars.
- **The comb's length is a power of two of field lines**: 32 lines at 1080p and 720p (32 and 48 virtual lines),
  16 at 1080i, 8 at SD (36 virtual lines), so the ghosts' vertical softness differs a little between modes (at
  1080p a little more of the strip texture survives between the ghosts than at 720p).
- **Comb Off has thin chroma lines.** At the defaults (Comb On) the model has no single-row lines at all. With the
  comb off, the raw decoder's hue turns inside every strip and the limiter squares it, so where a hue crossing
  meets a strip edge a one-row chroma dash appears (about 170 dashes of 24 - 70 px in a 720p frame; chroma only,
  the luma has none). That is the raw look; a longer smoothing would turn Comb Off back into Comb On.
- The chroma box is causal: at other Bandwidth / Subcarrier settings than the default the colour sits a few
  pixels left or right of its luma (a real decoder's chroma delay).
- In the sim the picture starts 4 px right (the generated avid's start, as in every program on this shell).
- The centre's Lissajous is +-128 px by +-64 lines (powers of two) instead of the prototype's +-150 by +-90.
