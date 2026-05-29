# Oracle — Deep Image Analysis & Design Spec

> A glitched triptych portal effect.  A divinatory "eye" opens in a static-torn
> window, revealing a rainbow spectrum behind it. Above the window lie
> high-frequency vertical chroma bars; below it, slow horizontal rainbow bands.

## 1. Visual decomposition of the reference image

The source image is one square frame that can be read as **three stacked
horizontal zones** plus **one inset rectangular window** containing a **central
ellipse** that acts as a "portal" revealing a different layer.

Using normalized vertical position `vy = (y - y_top) / (y_bot - y_top)`:

| Zone  | Vertical extent      | Primary content                                            |
|-------|----------------------|------------------------------------------------------------|
| **A** | `vy ∈ [0.00, 0.20]`  | Top: **vertical high-frequency rainbow bars** with vertical noise  |
| **B** | `vy ∈ [0.20, 0.80]`  | Middle: **inset window** with noisy interior + central **portal**  |
| **C** | `vy ∈ [0.80, 1.00]`  | Bottom: **horizontal smooth rainbow bands** (ROYGBIV)        |

### 1.1 Zone A — vertical rainbow chroma bars (top strip)

- Each column repeats a full ROYGBIV spectrum along X at very high frequency
  (perceived as ~50–120 thin vertical bars across the width).
- There is a secondary modulation running *vertically* that looks like
  scanline / field-line jitter — the column color shifts slightly as y moves,
  making the bars twinkle rather than sit still.
- Synthesis recipe: `hue_idx = (x * k_scale) + per_row_hash(y)` then expand
  `hue_idx` into a smooth 0→1024 hue ramp and look up the rainbow.

### 1.2 Zone C — horizontal rainbow bands (bottom strip)

- Thick horizontal bands stacked top-to-bottom: violet → indigo → blue →
  cyan → green → yellow → orange → red → violet (wrap). The gradient is
  *soft* at band boundaries, not hard — the palette is interpolated.
- Band period is about `height/14`: ~14 full cycles across Zone C.
- The bands very slightly wobble along X (a hint of `+small_jitter(y)`),
  but are essentially horizontal.
- Synthesis recipe: `hue_idx = y * k_scale`, blend adjacent palette entries
  with the fractional part.

### 1.3 Zone B — the window, the noise, and the portal

Zone B is structurally the most complex region, and it is itself composed
of three sub-layers:

**Frame surround (Zone B outside the inset):** a narrow gutter around the
inset rectangle leaks the same rainbow gradient that Zone C produces (soft
horizontal bands) — this is why the top & bottom of Zone B edges look like
a continuation of the gradient behind the window.

**Noise interior (Zone B, inside the inset, outside the portal):**
- A chaotic monochrome-ish cellular texture: mostly broken streaks and
  dabs of near-white, gray, brown, and black with occasional **rainbow
  speckles** (pixels that broke through to a saturated color).
- The noise is *horizontally-stretched*: it reads as short horizontal
  streaks rather than isotropic grit — as if an LFSR were updated once
  per pixel in X but only occasionally per row in Y.
- Origin: a multiplicative hash of `(x >> s, y >> t)` with
  `hash_out(top_bits)` driving luma and `hash_out(mid_bits)` occasionally
  swapping the pixel for a saturated rainbow sample.

**Portal (Zone B, inside the central ellipse):**
- A tall, narrow vertical ellipse roughly centered horizontally, spanning
  most of the inset's vertical extent. It looks like a "0" or a
  mandorla/vesica.
- The **interior of the portal displays the Zone C rainbow continuum** —
  i.e. the horizontal rainbow bands continue *inside the portal*, as
  though the noise window has a hole punched through to the underlying
  spectrum.
- The portal's *boundary* has a ragged, torn quality — one-pixel-thick
  rainbow-ish halo bleeding outward.

**Tear edges at inset borders:** where Zone A meets Zone B and where Zone
B meets Zone C, the edge is jittered left/right by a per-row hash by a
few pixels, producing a "datamoshed" ragged boundary instead of a clean
horizontal cut.

## 2. Chromatic analysis

The image uses a **full-saturation rainbow palette** throughout. Measured
against the project's existing Phosphor ROYGBIV values (already stored in
`phosphor_rainbow_palette.md`), the palette here is essentially identical:
10-bit YUV values with U/V swapped per the Videomancer convention.

### 2.1 Rainbow palette (8 entries, YUV 10-bit, U/V-swapped)

Taken directly from the calibrated Phosphor palette — proven to look
correct on Videomancer hardware:

| Idx | Color      |   Y   |   U   |   V   |
|-----|------------|-------|-------|-------|
| 0   | Red        |  328  |  960  |  360  |
| 1   | Orange     |  584  |  772  |  212  |
| 2   | Yellow     |  840  |  584  |   64  |
| 3   | Green      |  580  |  136  |  216  |
| 4   | Blue       |  164  |  440  |  960  |
| 5   | Indigo     |  192  |  608  |  696  |
| 6   | Violet     |  349  |  756  |  852  |
| 7   | Red (wrap) |  328  |  960  |  360  |

Between adjacent entries we **linear-interpolate** in all three channels
to get a smooth gradient. `hue_idx(9 downto 7)` picks the base entry;
`hue_idx(6 downto 0)` is the 7-bit fractional blend weight.

### 2.2 Noise luma palette (grayscale with rainbow pokethrough)

- Base grayscale: `y = 200..900`, uniform in that band, U = V = 512.
- Saturation spikes: when `hash_mid(5 downto 0) = 0`, substitute the
  rainbow palette entry selected by `hash_top(2 downto 0)`. Probability
  ≈ 1/64, matching the sparsity of the colored speckles in the source.

## 3. Spatial / geometric parameters

### 3.1 Zone boundaries (signal-proportional, not pixel-absolute)

Because the core runs at both HD and SD resolutions, we express zone
boundaries as fractions of the currently-detected active-video height.
In hardware we count hsyncs between vsync assertions into `max_y_r` and
compare `pixel_y` against scaled thresholds:

- Zone A top cut: `y < max_y >> 3`       (top 12.5%)
- Zone C top cut: `y > max_y * 7 >> 3`   (bottom 12.5%)
- Inset (window) border: `max_y/8 + 32 ≤ y ≤ 7*max_y/8 - 32`

The image shows a slightly wider Zone A / Zone C (closer to 20%) but
this is controllable via a knob; we'll use 1/8 as the default and let
the user widen with "Zone Width" knob.

### 3.2 Portal ellipse

- Vertical ellipse, aspect `a : b ≈ 1 : 3` (narrow and tall).
- Center defaults to `(max_x/2, max_y/2)`, user-adjustable via knobs.
- Radius defaults to `max_x / 8` horizontally, `max_y / 3` vertically.

### 3.3 Rainbow band frequency

- Zone A vertical bars: `hue = (x * stripe_k) + row_hash(y)`,
  with `stripe_k` = 16..128 (knob).
- Zone C horizontal bands: `hue = y * band_k`, with `band_k` = 2..32.
- Portal interior: same as Zone C (horizontal bands passing through).

## 4. Procedural decomposition & effect name

"Oracle" captures the image: a divinatory eye opening in a wall of cosmic
static, with ordered spectra above and below. The central ellipse-portal
is the pupil; the ROYGBIV surrounds are the aura; the noise is the static
between signal and revelation.

## 5. Pipeline design (targeting 12-stage latency @ 74.25 MHz on HX4K)

The HX4K has no hardware DSP multipliers, so every `unsigned * unsigned`
above ~5×5 bits is expensive and must be pipelined. We break the path
deliberately across many registered stages.

```
S0  Input register: data_in, params, position (pixel_x, pixel_y, frame_count)
S1  Zone classify: zone = {A, B-gutter, B-inset, C}; portal dx/dy absolute diffs
S1m Multiply stage 1: dx*dx  (pipelined multiply, 12x12 -> 24-bit)
S2  Multiply stage 2: dy*dy*SCALE (dy squared and multiplied by aspect scale)
S2b Add: dx² + (dy*scale)²  compare -> portal_hit bit
S3  Noise hash A: xor mix of (x>>shift, y>>shift, frame_count, seed)
S3b Noise hash B: 16x16 hash multiply, top bits give noise_val
S4  Rainbow hue-index derivation (three candidate indices: Zone A, Zone C, noise-pokethrough)
S5  Palette lookup A: fetch base & next palette entries (Y, U, V)
S6  Palette blend:    lerp(entry_n, entry_n+1, frac) for Y/U/V
S7  Zone color select: mux final (Y,U,V) per zone and portal_hit
S8  Edge tear / jitter modulation at zone seams
S9  Brightness multiply (mix knob): prod = y*brt, register
S10 Bypass mux: data_in vs effect
S11 Output register: data_out, assign pipelined sync signals
```

**Total `LATENCY = 12` pixel clocks = 161.6 ns**, well within any practical
downstream budget. All multiplies are split so no register-to-register
path contains more than one multiplier input.

## 6. Register map (t_spi_ram, each `std_logic_vector(9 downto 0)`)

| Register            | Parameter         | Usage                                               |
|---------------------|-------------------|-----------------------------------------------------|
| `registers_in(0)`   | Portal Size       | 0..1023 → ellipse radius scale                      |
| `registers_in(1)`   | Portal X          | 0..1023 → center X position (linear across width)   |
| `registers_in(2)`   | Portal Y          | 0..1023 → center Y position (linear across height)  |
| `registers_in(3)`   | Noise Density     | 0..1023 → noise scale (larger = bigger chunks)       |
| `registers_in(4)`   | Hue Cycle Speed   | 0..1023 → DDS-driven hue phase drift per frame       |
| `registers_in(5)`   | Band Frequency    | 0..1023 → horizontal band count (2..32)              |
| `registers_in(6)`   | Switches (bits)   | b0 Top Zone b1 Bot Zone b2 Portal b3 Tear b4 Bypass |
| `registers_in(7)`   | Brightness        | 0..1023 → final Y/chroma attenuation                 |

## 7. Resource budget targets

- **LCs:** 4000–5500 of 7680 available on HX4K. Close to starfield/solitude
  budget but leaves room for P&R spread.
- **BRAM:** 0 blocks. All palettes as constants; no line buffers needed.
- **Multipliers:** 4 major (dx², dy²·scale, brightness·y, chroma lerp),
  each split across at least two pipeline stages. No DSPs available — all
  are LC-based, so each contributes ~300 LCs to the count.

## 8. TOML parameter plan

8 parameters total, all rev_b-compatible and linear with display labels.

- Rotary 1–6: Portal Size / Portal X / Portal Y / Noise / Hue Speed / Band Freq
- Toggle 7–11: Top Zone / Bot Zone / Portal / Tear Edges / Bypass
- Linear 12: Brightness

Presets: "Default Oracle", "Quiet Portal", "Maximal Chaos", "Stopped Clock",
"Pure Bars" — showcasing different combinations.

## 9. Build & test plan

1. Build with `build_programs.sh oracle` targeting HD (74.25 MHz).
2. Inspect `nextpnr-ice40` timing report: require `Fmax ≥ 74.25 MHz`.
3. If any path fails, do NOT change seeds. Instead split the offending
   path by inserting a registered signal between the critical multiply
   and its consumer — add a new `sNm_...` pipeline stage and bump
   `LATENCY`.
4. Confirm no combinational loops; every `sN_*` signal is only assigned
   inside one `if rising_edge(clk)` process.
