# Hueflutter — summary

Catalogue combo #111 (seed 20260925): hue rotation / saturation + magnetic
warp (wobble) + ordered dithering. v0.1.0 was flashed and liked
("hueflutter is good", 2026-09-12). v0.1.2/v0.1.3 (2026-09-13) closed HD
timing and swapped the edge fill for the address clamp, and were never
re-checked on hardware. v0.2.0 (2026-10-13) is the first finalizing pass,
built from a review of v0.1.3 while Ron was away. It is **not flashed yet**.
v0.1.3 sources: git 3fa438d, and `.hueflutter_work/v013/`. Its vmprog is
`.hueflutter_work/hueflutter_v013.vmprog`.

Build: `./build_combo.sh hueflutter` → `out/rev_b/ron/catalogue-combo/hueflutter.vmprog`.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Wobble | displacement amplitude, 0–255 px (default 450 ≈ 112 px) |
| K2 Waves | vertical wavelength: line-phase step. Glided |
| K3 Hue | base hue rotation, 0–360°. 0 = unrotated (it was −90° in v0.1) |
| K4 Sat | saturation 0–400%, 100% = unity |
| K5 Luma | luma levels, 1–8 bits |
| K6 Chroma | chroma levels, 1–8 bits (1 bit = −/grey/+ per channel) |
| S7 Bands | Phase: band k of 8 is k/16 of a cycle ahead / Amp: band k swings (k+1)/8 of K1 (dark bands calm, bright bands swing) |
| S8 Order | Dither In: chroma posterized before the hue rotation (the colour lattice spins), dither pattern travels with the picture / Dither Out: posterized after it (a fixed palette the colours snap between), dither pattern fixed to the screen |
| S9 Chroma | With / Quarter: chroma reads a quarter wave apart (fringes) |
| S10 Hue | Static / Ripple: the hue follows the wobble phase (rainbow ripples) |
| S11 Quant | Plain / Dither: 2×2 ordered dither (cells 2 px wide) |
| P12 Spin | hue spin speed, bipolar, squared. Centre (±8 codes) = still; the ends ≈ 0.55 s a turn |

## v0.2.0 changes (Tier 1 of the review)

Bugs:
- **Hue 0° was a −90° rotation.** The sine ROM holds −cos, and v0.1 read
  sin at the angle and cos a quarter turn on, so at Hue 0 every colour was
  turned −90°: (U, V) → (V, −U). Same bug as negspin/gouache. The reads are
  now sin = ROM(a + 64), cos = ROM(a xor 128). With Spin running (the
  default) the absolute hue cycles anyway; this matters with P12 centred.
- **Greys went pink.** The chroma posterizer was mid-rise: grey (512)
  became U = V = 576 at the default 3 bits, 640 at 2 bits (the Posterwave
  preset), 768 at 1 bit. In Dither In mode the cast was rotated by the spin,
  so greys cycled through colours. It is now a centred mid-tread quantizer
  (flutter v0.2.0): levels at 512 ± k × step, clamped 64..960.
- **The luma posterizer lifted black.** Black 64 → 96 at the default
  4 bits; 1 bit gave two greys (256 / 768). Luma is now normalized to the
  video range, cut, bit-replicated and scaled back (flutter v0.2.0), so black
  and white are always levels.
- **Dither brightened the picture.** It added 0, ½, ¾, ¼ of a step before a
  floor quantizer: +⅜ step on average. It is now centred (luma ±⅛, ±⅜ step;
  chroma rounds to nearest with a ⅛..⅞ offset).
- **The output dither cell came from the input-side counters.** It was
  keyed on the x and line count of the pixel entering the program, ~22
  clocks earlier, so over the last ~22 px of every line the x count read 0
  (the pattern collapsed to stripes) and the line parity flipped early. It
  now has its own counters at the output.
- **K1 read −255 px at zero wobble.** The TOML showed −255..255, but the
  amplitude was always 0..255. Display is now 0..255 px.
- **S7's default (Amp) used a third of K1.** The band gains were
  0.125..0.34 (flutter's Spread knob is absent here, so its constant 64 was
  left in), so K1 at full gave at most 88 px. "Same" wasn't the same either:
  the bands were up to 79° apart in phase. Now: Amp gain (k+1)/8, so the
  top band gets all of K1; Phase spread k/16 of a cycle (up to 158°, as
  flutter's default Spread). Labels Same/Split → Phase/Amp. Default K1
  700 → 450, so the default wobble (14..112 px) is a little bolder than
  v0.1's (22..60 px) rather than 3×.
- Categories → Color / Warp / Pixel ("Colorizer" and "Retro" are not in
  the schema list).

Hardware contracts:
- U/V are swapped on rows whose hsync→avid parity differs from the row
  above. Each row replays the row above from the line buffer, the same case
  as drape v1.3 and flutter v0.2.0.
- Read and write banks are latched per line at the avid rise (they switched
  at hsync).
- HD HDMI bar conditioning: highlight knee 600 + (y − 600)/2 (white ~770,
  midtones untouched — negspin's variant), then [1 2 1] on Y/U/V before the
  blanking gate. Low-bit posterized/dithered luma is full-swing 1-px edges
  moving every frame, the known trigger. **The dither cells are now 2 px
  wide**: a [1 2 1] flattens a 1-px alternation completely (it would have
  turned the checker into horizontal line stripes); at 2 px the pattern
  survives at half contrast.
- no_rw_check on all six line-buffer RAMs (one bank is read while the
  other is written; the write bank's read data is discarded).
- Latency 24 → 29, the same in all modes.

Controls:
- **P12 Spin is squared** with a ±8-code centre deadband (gouache/negspin
  curve). The ends are ~8× faster than v0.1's top (0.55 s a turn instead of
  4.3 s). Default and presets re-solved for their v0.1 speeds: 600 → 596,
  Rainbow Jelly 800 → 656, Posterwave 200 → 361.
- **K2 Waves** uses all 10 bits and glides by ⅛ of the gap per frame. One
  step of the old 8-bit value moved the bottom of a 1080-line frame by
  ~0.07 of a cycle (pot noise jumped the picture).
- **K5/K6** change depth only 12 counts inside a new zone (hysteresis).
  Preset depths that sat exactly on a zone boundary (128, 256) moved to the
  zone middle (192, 320), same bit depths.
- S8 now acts on luma only through the dither pattern (screen-fixed or
  travelling), which is all it ever did for luma: a pixel shift commutes
  with a per-pixel quantizer. Its chroma effect (lattice spins vs fixed
  palette) is unchanged. Luma is posterized once, after the fetch.
- Dead flutter leftovers removed (Speed/Spread/Twist/Mirror signals and the
  mirror branch of the edge clamp).

## Review backlog

### Tier 2: needs Ron's call (these change the look)
- **A. The wobble has no speed control.** It drifts at one fixed rate (a
  cycle every ~2.7 s at 60 fields/s). Options:
  1. P12 also drives the drift: base speed + |Spin|, so the slider is one
     "flutter harder" performance control (never stops);
  2. S9 becomes Drift Slow / Fast (losing the Quarter fringes);
  3. leave it.
- **B. Bands follow K5** (flutter v0.2.0): the band is the posterized luma
  level, so each flat poster region slides as one piece. At 1–2 bits, light
  and dark split into big halves. Default 4 bits would give 16 bands.
- **C. Sat knob.** Unity sits at 25% of travel. A proc-amp curve with unity
  at centre (0..100% left half, 100..400% right half) would be symmetric.
  Presets would need translating.
- **D. K1 range.** 255 px is 13% of an HD line. A squared curve to 511 px
  keeps fine control low and gives a bigger top.
- **E. Possible thin wrong-hue lines on HD HDMI.** If the source avid's
  hsync parity wanders, the encoders mis-pair U/V on that line, and for a
  hue rotation that shows the opposite rotation (swap · R(θ) · swap =
  R(−θ)). Nobody has reported it on v0.1. Cure if seen: generated avid
  (~25 LC, cubist/aurora).
- **F. Smoothing vs crisp dither.** If HD HDMI shows no bars even at 1-bit
  dithered luma with motion, the [1 2 1] could go and the dither cells go
  back to 1 px (the crisper v0.1 texture). The standing rule says build the
  fix in for this kind of picture, so v0.2.0 has it.
- **G. Presets.** Capture them from hardware once the look settles
  (`./capture_preset.sh`). 4 slots are free. The 4 existing presets are
  hand-written; in Amp mode their K1 now swings bright bands ~3× more.

### Tier 3: polish (small)
- **a.** Bands come from raw luma bits 9..7: band 0 (64..127) is half
  width, band 7 (896+) is nearly empty. Normalize before banding.
- **b.** The 8-bit hue angle steps 1.4° at a time; very slow spins tick.
- **c.** The K5/K6 displays don't line up with the depth zones: "2 b"
  shows ~65 counts before the depth changes, "8 b" ~40 counts after.
- **d.** The top row shows the previous frame's last line (stale bank).
- **e.** Chroma is stored at 2-px, 8-bit resolution (full res: +4 EBR).
- **f.** Interlace: both fields use the same per-field-line phase (a
  half-step vertical twitter at high Waves).

## Hardware check list (v0.2.0)
1. Power-on default: wobble bands (dark calm, bright swinging up to
   ~112 px), 4-bit dithered luma, 3-bit chroma, hue spinning slowly with
   rainbow ripples. Greys should be grey (v0.1 had a pink cast).
2. P12 centred: still hue; K3 Hue 0 = the source's own colours. Ends of P12
   ≈ half a second a turn; slow creep just off centre.
3. K5 at 1 bit: black and white (v0.1: two greys). S11 Dither: a soft 2×1
   checker, no brightness jump when toggling it.
4. S8 with S11 on: Dither Out = screen-fixed checker + fixed palette;
   Dither In = checker rides the wobble + spinning colour lattice.
5. S7 Phase vs Amp: Phase = all bands swing fully, offset in time; Amp =
   darks calm, brights wild. K1 should reach a big swing in both.
6. K2 Waves: smooth (no jumps) while resting or turning slowly.
7. HD HDMI: thick bars at 1-bit dithered luma with motion? Thin wrong-hue
   lines over colour (Tier 2 E)? Left/right edge fill clean.

## Build results
v0.2.0 (2026-10-13), router2:

| Config | Fmax (MHz) | Seed |
|---|---|---|
| HD Analog | 85.19 | 1 |
| HD HDMI | 88.54 | 1 |
| HD Dual | 91.07 | 1 |
| SD Analog | 86.15 | 2 (seed 1 stalled the router, not a timing miss) |
| SD HDMI | 80.04 | 1 |
| SD Dual | 91.02 | 1 |

v0.1.3 needed seed retries on two configs (77.8 MHz on seed 3; a stall then
86.9 on seed 2). LC 5,368–5,397 of 7,680 (~70%, was ~4,880), 22 EBR.
Standalone synth: 2,790 LUT / 825 carry (v0.1.3: 2,533 / 672), and
`emulate_transparency` 0 (v0.1.3: 3). All six bitstreams pass iceunpack.
NOT FLASHED, NOT COMMITTED.

## Build seeds — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

2026-10-14: **seed 3**, set as `[firmware] seed = 3` in the staged community TOML (the dev TOML is unchanged). Why: seed 1 stalled nextpnr's router1 on SD Dual. Sweep 2-6: 2, 3, 4 build 8/8; 5 and 6 hang one config. Own HD logic with seed 3 (scratch core without the Sync Out generator): 104.81 / 101.17 / 103.97 MHz (Analog / HDMI / Dual).
