# BRILLIANT (Straightedge)

v0.1.0, built 2026-11-22, not flashed. Flash out/rev_b/ron/straightedge/brilliant.vmprog.

## What it draws

Art reference: the gemmologist's facet diagram of the round brilliant cut seen from above, and the way a turning
stone scintillates (facets flash white one after another as the light goes round).

One great cut stone nearly the height of the screen, on black. The whole disc is divided into flat facets with exact
straight edges:
- an octagonal table, with the pavilion showing through it as a sixteen-sector star (alternate sectors);
- eight star facets, eight kites and sixteen upper girdle facets round a 16-gon girdle.

Each facet is one flat tone (black, dark, light, white) from a light that circles the stone. The white flashes run
round the stone as a wave while the stone itself turns slowly the other way.

Pave (S8) tiles the screen with a square lattice of smaller stones. Each stone's light is offset by its cell index,
so the flashes travel across the wall as a glitter.

How it is drawn:
- The pixel is folded into one 22.5-degree wedge: |X|, |Y|, swap, then a reflection in the 22.5-degree line, which is
  exact on the fold line, so the fold leaves no seam.
- Four affine edges inside the wedge (table, star/kite, kite/girdle, girdle) give every facet boundary.
- Every edge is anti-aliased over about one pixel (straight edges only, no gradients, luma only: U = V = 512).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | The stone's centre (Pave: the lattice's pan), +-640 virtual px. Glided. |
| K2 | Y | Vertical centre, +-360 virtual px, up = up. Glided. |
| K3 | Scale | Exponential. Stone girdle radius 225 .. 520 .. 1200 virtual px (the top runs the girdle off the screen into a few giant facets). Pave cell 130 .. 266 .. 546 virtual px, stone radius 0.48 cell. Glided. |
| K4 | Fire | Flash sharpness. 0 = broad soft modelling, most facets mid-grey. 1023 = a dark stone with rare hard white flashes. Default 450. Glided. |
| K5 | Drift | Bipolar (Hypnos law, +-24 dead at centre). The light circles at 2^-11 .. 2^-5 turn per field, and the stone turns the other way at 1/4 of that rate. Default 680 = about 10 s per turn of the light; full scale about 0.5 s. |
| K6 | Camera | Camera strength, default 100 %. 0 = exactly the black-input picture (see Camera). Glided. |
| S7 | Glide | On: knobs ease 1/16 of the gap per field (minimum 1/2 count, landing exactly) and the Drift speed eases 1/8 per field. Off: no easing at all. Default On. |
| S8 | Form | Stone (one great stone) / Pave (a wall of stones, each with its own light phase). |
| S9 | Tone | Grey (four tones) / Ink (pure black and white facets, thresholded flash). |
| S10 | Weight | Bold (filled facets) / Fine (the facet diagram as thin white lines, with the brightest facets filled grey; Ink + Fine = lines XOR the white facets). |
| S11 | Source | Video / Osc (the shell's triangle lattice, x3 about its middle, drifting). |
| P12 | Cut | The proportions t (table) and s (star points) together, from a 65-point table. 0 % (0.80, 0.885) = an old rose cut, huge table, thin border. 50 % (0.55, 0.75) = the classic brilliant. 100 % (0.16, 0.42) = a tiny table and long needle kites: a starburst. The edge order is valid across the whole travel. Glided. |

Presets:
- Brilliant: the defaults.
- Rose Cut: Fire 250, Cut 0.
- Starburst: Fire 800, Drift 720, Cut 1023.
- Diagram: Fire 900, Fine.
- Pave: Scale 380, Pave.
- Black Ice: Fire 1023, Drift 640, Ink + Fine.

## Motion

The light's azimuth is a phase accumulator, with one full turn = 2^26. It advances once per field, on the engine's
field start (the vblank zone), not on a vsync edge.
- Speed = 2^(-11 + ls/64) turn per field, where ls = 0.78 (|v| - 24) and v = Drift - 512. The sign of v picks the
  direction, and +-24 around the centre is dead (still).
- At the default 680 that is 2^-9.25 turn per field, about 10 s per turn at 60 fields/s. Just outside the deadband it
  is about 34 s per turn; at full scale it is 2^-5, about 0.5 s per turn.
- The stone's own angle is -1/4 of the light's phase (Q26 - th/4). It turns the other way four times slower, and its
  rotation wraps exactly with the accumulator.
- Glide On eases the speed (1/8 per field), not the phase, so motion never stops because of the switch.

A facet's tone = its base tone (a three-colouring of the diagram, so neighbours differ) + the flash count. The flash
count is the number of Fire thresholds under cos(facet azimuth + stone angle - light angle), clamped to 0 .. 3. The
pavilion sectors in the table use the opposite azimuth (their reflection).

## Camera

The source luma, raw with no gain, is held in a 32 x 32 virtual-px block grid. This is windrose's grid: block sums
plus a per-column one-pole, two banks. K6 sets the engine's thresholds, so the strength is applied once per field.

Stone:
- Facet steps: each facet takes a tone step of +-2 from the luma at its anchor point relative to the picture's mean,
  9 g (L - mean) / 2^15 with g = K6, clamped to +-2. The facets stay clean and flat; the stone reads as lit by the
  picture.
- Glitter: around the stone, a halftone of lozenge dots, one per 32-virtual-px cell. Dot size is the cell's luma in
  5 levels, radius = level x 41/8 virtual px, with a twinkle running diagonally through the dot field.
- Effect on the bear: the bear reads as a dot-screen silhouette (ears, head, the bright background) around the stone.
  - 25 %: only the brightest cells get small dots.
  - 50 %: the head and ears read clearly.
  - 100 %: the dense screen reads, and the facet tones follow the picture.
- Black input or K6 = 0: no dots and no steps, exactly the plain picture.

Pave: each stone's girdle shrinks with its cell's luma below the picture's mean, a halftone of gems. Below the mean,
stones shrink in four steps to nothing; dark areas of the subject become empty black.

Osc substitutes the shell's drifting triangle lattice (x3 about its middle) for the luma, so the camera features
(dots, steps, sizes) show without a source.

I tested on bear.png in the integer model before writing RTL, per the addendum. The spec's per-32-px-block tone step
inside the stone cut every facet with block borders and the bear was not readable, so I chose the facet-anchor
steps + glitter halftone instead.

## Engine

- **Per field:** Hardline flute's microcoded engine (via windrose / lowpoly).
  - Hardware: a 2048 x 16 microcode ROM, a 256 x 32 register file, a 1024 x 16 table ROM (exp, sin, cut tables
    t / s / k2 / k3 at 65 points of P12, an alpha ROM), a four-deep link stack, and a grid mailbox port.
  - It is started at every field start, so it restarts from PC 0 each field and needs no separate watchdog.
  - Each field it writes a two-bank 64-word table: the DDA steps and wraps for the turned frame, the lattice
    counters, the edge constants (gamma1..4, kappa2/3 and 3 kappa, alpha snaps), the 16 per-sector tone words, the
    ramp, the camera thresholds and the glitter offsets.
  - Knobs act one field late.
  - Worst case about 147k clocks, which fits NTSC (225k) and PAL.
- **Per line:** the DDA pair (X, Y in the stone's turned frame) reloads from the table, and the lattice counters
  step.
- **Per pixel:**
  - fold and sector code;
  - five edges, where kappa x b is radix-4 with a per-field 3 kappa (the only per-pixel products);
  - coverages in 9 levels;
  - the sector word, read from the table once per pixel;
  - two levels of selected blends, then Fine lines or Ink XOR;
  - the glitter dots;
  - luma at S23;
  - output conditioning: [1 2 1]/4 horizontal smoothing, luma 13/16 toward black (white about 776), and the
    blanking gate.
- **Latency:** one latency of 36 clocks in every mode (processing_delay_clks 36). Syncs are delayed through a
  64-deep block RAM.
- **Shared rules:** virtual 1080-line coordinates from the timing ID; generated avid; interlace from the timing ID.

## Status

- Router: **router1** on every config, all on seed 1. router2 kept stalling on HD HDMI at overused = 2 for the
  full 15-minute make timeout, on two seeds of the final netlist and once before.
- Fmax per config, post-route (the builder's last "Max frequency" line), target 74.25 MHz HD / 27 MHz SD:
  - HD Analog: seed 1, 78.28 MHz
  - SD Analog: seed 1, 79.37 MHz
  - HD HDMI: seed 1, 83.49 MHz
  - SD HDMI: seed 1, 82.20 MHz
  - HD Dual: seed 1, 87.37 MHz
  - SD Dual: seed 1, 85.70 MHz
- LC 6919 .. 6947 / 7680 (90.5 %, SD Dual highest), EBR 26 / 32. All six .bin pass iceunpack.
- Build history (all in .straightedge_work/rb_brilliant/build_v0.1.0*.log):
  - Try 1: router2 stalled on HD HDMI. dupin found two LUTs with a duplicated input: the alpha scale's 2x + x and
    4x + 2x on a sign-extended operand (the kelp pattern). 3x is now built from the magnitude bits
    (3 xu - 1536 xs), with the same values, and dupin reports 0.
  - Try 2: router2 closed every config (HD Dual on seed 3).
  - Try 3: the first-cell copy fix, router2. HD HDMI stalled at overused = 2 on seeds 1 and 2, so I rebuilt with
    router1.
    - The bug: the full-raster sims showed one column of mismatches at the left edge. The per-cell values
      (glitter level, twinkle; Pave girdle size, light offset) were copied down a 15-clock chain started at the
      line's first pixel. When the first lattice cell is shorter than that, as with the 4-pixel first cell at 720p
      and NTSC, it took the second cell's values.
    - The fix: the first cell's copies now run in the blanking, 24 clocks after the line end.
- Verification, GHDL RTL against the integer model (imodel.py, bit-exact by design; RTL column c + 4 = model
  column c):
  - On the final netlist:
    - rtl_default, 720p dec 1, every control at its power-up default (Glide On, Drift 680): 0 of 918,720 pixels
      differ (the model at t = 3 fields). This is also the Glide On check.
    - NTSC dec 1 with the bear (Drift still, Glide off): 0 of 173,988 on both fields.
    - Motion, 720p / 2 at the default Drift, Glide off: 3 and 7 warm-up frames match the model at t = 3 and t = 7,
      0 of 228,960 each. The drift advances one model step per field in the right direction.
    - 720p dec 1, Drift still, Glide off: Osc 0 of 918,720 and bear at K6 100 % 0 of 918,720. These are the two
      frames that showed the first-cell bug before the fix.
    - Pave at 720p / 2: 0 of 228,960.
  - Before the first-cell fix, which changed only the first lattice cell of each line:
    - 720p / 2, 0 of 228,960 each: default; Ink + Fine + Fire 900 + Cut 900 + X 300 / Y 700; bear; Pave + bear at
      Scale 300; Osc + Fine + Cut 100 + Scale 700.
    - 720p dec 1: default 0 of 918,720. Osc and bear showed 9 and 45 pixels differing in output column 0: the bug,
      since fixed.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge brilliant`

## Cut / deviations

- The culet star is removed: no second pavilion edge, so the pavilion seen through the table is a 16-sector star of
  alternate tones.
- Camera targets changed from the spec, after the bear test:
  - Stone = per-facet tone steps + a glitter halftone around the stone (the spec asked for per-32-px-block steps
    inside the stone);
  - Pave = girdle size per stone (the spec asked for a tone step per stone).
- The girdle edge slope is 53/256 (11.7 degrees; 16-gon exact would be 0.199). It comes from the same product as the
  table edge (106/256).
- The Pave light phase offset uses a linear cell index, (col + 3 row), not a hash. The flashes travel across the wall
  diagonally instead of blinking at random.
- alpha (edge AA scale) is snapped to 5 values (8, 6, 4, 3, 2 / 8). The AA width of the star / kite
  edges therefore varies by up to about 20 % with Cut.
- The glitter dots have 5 sizes.
- Drift's top speed is 2^-5 turn per field.
- The per-field advance happens on the engine's field-start pulse (the vblank zone), not on a vsync edge.
- Knobs act one field late (two-bank table).
- Pave cells are a whole number of pixels: the cell size rounds to a pixel count per mode, so the lattice never
  drifts along a line.
- Fine draws only the facet edges: there are no lines along the table's 0-degree axes inside the pavilion star.
- There are no stepped knobs, so the stepped-knob end-hysteresis check does not apply.
