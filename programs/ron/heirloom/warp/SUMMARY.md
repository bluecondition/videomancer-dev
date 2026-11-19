# Warp — a perspective stripe plane warped into checkers, moiré and loops

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/warp.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Warp Diag`, `out/rev_b/ron/_diag/warp_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/warp/`) is untouched.
v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed. Heirloom #48. Source photo:
`photos/WIN_20230909_23_29_18_Pro.jpg` (1920×1080 capture, 2023-09-09). This replaces v0.1.0 (an r²-bent
stripe field with block shear), which Ron judged too abstract.

## Phase 2 (v0.6.0)
No notes from Ron for warp (free hand, 2026-11-09 batch). In v0.5, P12 Warp, K2 Horizon, K6 Palette and Mirror were
bold, but:
- K1 Stripes was one global zoom over ×0.5..×2 (a 64-step table, so a glide stepped);
- K4 Bend reached only ±1 cycle;
- K5 Haze and S8 Margin were a colour nudge and a 10 %-of-the-width strip;
- the fitted warp map never moved: only the stripes drifted under it.

The round treats the patch as its modules: a VCO whose frequency is a ramp (density, horizon, vanishing point, ramp shape),
a warp signal patched into its phase (amount on P12, now also sliding), a camera patched into its phase (Bend), and a
colourizer. The haze and the margin are fixed at the photo. Every new effect is exactly zero at its default knob value;
the default frame is raw-identical to v0.5.1 (checks below). Phase-2 working files: `.heirloom_work/rb_warp/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| P12 Warp: 0 clean plane / 50 % photo / 100 % doubled warp | unchanged | 0 % = clean arcs over a fan; 100 % = a second front doubles the warp into a dense loop field |
| K1 Stripes: ×0.5 .. ×2, 64-step table | **×0.25 .. ×4**, exponential (one octave per quarter turn, 256 steps per octave, computed per field), photo at 50 % (exact) | top-right stripe period 30 px → 120 px (a few broad bands) .. 7.5 px (hairlines); the fan below from 3–4 broad bands to one, or to ~14. In Rings, the ring density by the same factor |
| K2 Horizon: ±256 lines | **±512 lines** (y 183 .. 1206), photo at the centre (exact) | from a high horizon (the red/violet ground fan fills 83 % of the frame) to one below the bottom edge (the arcs fill the frame) |
| K3 Flow: stripe drift, bipolar square law | the same drift **plus a warp-field slide**: past the photo's speed (K3 below 376 or above 648; the default 640 is exactly at the threshold) the fitted warp map slides sideways the stripes' way, up to 12 virtual px a field | at the ends the checker, the lenses and the loop band cross the screen in ~2.7 s (≈ 720 px/s) through the flowing stripes. The map ping-pongs (reflects at its edges), so it never shows a seam |
| K6 Bend: camera (or Osc) → phase, 0 .. ±1 cycle | **0 .. ±4 cycles**: v0.5's law up to the centre (default 100 = ±0.09 cycle, exact), steeper above | at 100 % the camera picture is drawn as ~8 contour bands through the stripes (a face becomes a topographic map); Osc: the lattice folds the stripes into a quilt |
| K5 Haze: green haze none / photo / double | **K5 Vanish** (renamed): the plane's vanishing point x, 4 virtual px per step, glided; photo at the centre (exact; x0 = −94) | left: x0 to −2142, so the arcs flatten into near-horizontal receding stripes. Right: x0 to +1950, so the vanishing point enters the frame: both branches of the hyperbolas, steep lines converging on a vertical spine (≈ 76 %, x0 = 960), then the arcs bend the other way (a mirror of the photo). ±2048 px = ±107 % of the width. In Rings, it moves the ring centre. The haze is fixed at the photo |
| K4 Palette | unchanged | |
| S7 Seam: two laws / one law | unchanged (in Rings: two ring systems split at the horizon / one) | |
| S8 Margin: on / off | **S8 Form: Plane / Rings** (renamed): Rings makes the VCO's ramp x² + y², a zone plate | the whole frame becomes concentric rings centred on the horizon (x 960 at the default Vanish; below the horizon a second system centred 385 lines higher, so the seam shows), warped by the same map, bent by Bend and coloured by the palettes. Ring period 160 px at r 100, 40 px at r 400, 16 px at r 1000, then moiré. The margin is always on (the photo) |
| S9 Mirror | unchanged | |
| S10 Zigzag: ±0.15 cycle | **±0.3 cycle** | a bolder rolling zig-zag |

Unchanged: P12 Warp, K6 Palette, S7 Seam, S9 Mirror, S11 Source (Video / Osc, default Video).

**Presets** (preset 0 = the defaults): **Photo**; **Clean Plane** (P12 0); **Deep Warp** (P12 100 %, faster); **Mirror
Ice** (as before); **Zone Plate** (Rings, less warp, denser); **Spine** (Vanish at the screen centre, one law: a full
four-quadrant vanishing point); **Slide** (fast Flow: the warp field crossing the screen); **Low Sky** (high horizon,
strong Bend, Neon).

**Engine (new).**
- **Per-field serial multiplier.** One shift-add multiplier in the vblank zone runs seven 16-clock slots:
  - the flow speed (m², which replaces v0.5's own m² multiplier);
  - the carrier gains ka and kb, and the ring gain A, each a constant × (256 + f) >> (10 − e) from K1 (e = the
    quarter, f = the position in it);
  - |x0|², A·x0² and A·|x0| for the rings.

  It then forms the ring steps for the mode's pixel pitch s (1 / 1.5 / 2 virtual px): inc0 = A(2·x0·s + s²) and
  c2 = 2·A·s². The C_KA / C_KB tables are no longer read.
- **Vanish.** vp = t_vp − 4·(K5 − 512), signed. The line engine's S0 = F·vp multiply takes |vp| (12 bits now) and the
  sign of F·vp.
- **Rings.** Two more line-engine multiplies (dy², then A·dy²): S0 = A·(x0² + dy²) + flow + zigzag. The engine ends at
  clock 55, inside SD's 858-clock line.
- **Beam.**
  - A quadratic DDA: acc += inc, inc += c2. At the mirror the first reversed pixel holds while inc steps back, then
    acc −= inc and inc −= c2, an exact retrace.
  - In Plane c2 = 0, so the beam is v0.5's.
- **Slide.**
  - A per-field offset (16 bits: knot × 256 + fraction, wrapping at 240 knots).
  - The map fill writes 122 columns. Its knot index ping-pongs over the 120 knots, starting from the offset.
  - The pixel path reads the column buffer at map x = beam x + fraction, from a second beam counter; mirrored, it is
    `f_mc − b_mx`, so there is no extra adder in the address path.
- **Map.** The unfitted flat knots under the left margin, which the slide brings into view, are refilled in the hook
  with a reflection of their neighbours. Only knots that reach no pixel outside the opaque margin are touched, so the
  defaults are unchanged.
- **Haze and margin.** The haze multiply is gone: h = the table value, as at v0.5's K5 centre. The margin key is
  always on.
- No pipeline stages added or removed: latency 24 in every mode, U/V parity unchanged.

**Build** (router2, `.heirloom_work/diag_build.sh warp`, 2026-11-09):
- Post-route Fmax: HD Analog s1 82.69, SD Analog s1 74.22, **HD HDMI s2** 87.72 (seed 1 missed), SD HDMI s1 74.23,
  HD Dual s1 81.43, SD Dual s1 67.65 MHz.
- 6522–6539 / 7680 LC (85 %), 23 / 32 EBR (unchanged). iceunpack OK ×6; the packed config carries the new labels and
  presets. No pin-skew screen.
- v0.5.1's HD HDMI seed was 4; this is a new netlist.

**Fit history.** The first build reached HD Analog 71.3 MHz on its best seed. The critical path (nextpnr profile)
was the line engine's state 2:
- it computed yh − 5 − y and its absolute value, three carry chains, then loaded the multiplier;
- the law choice (`e_vyi < f_yh`, another compare chain) fanned out across the 32-bit multiplicand mux.

Restructured to one small operation per state:
- per-field yh − 5, yh − 390 and ±r1 / ±r2;
- both laws' dy computed in parallel;
- the law picked by the sign of y − yh in a new state 3, with the abs there;
- the line S0 partial sums made in idle multiplier-setup states;
- the slide wrap and the vanishing point split over zone clocks;
- the mirror test against a precomputed 15360 − sx.

HD Analog then routed at 82.7 MHz, with the critical path in the core's SPI block. Nothing was cut.

**Checks** (working files in `.heirloom_work/rb_warp/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: every new path at RTL widths over all knob extremes and every line: no overflow. The defaults are
  exact (ka 71583, kb 44739, A 524, vp 94, bend gain 3). The ring DDA is within 0.025 cycle of the exact x² + y²
  across the line for s = 1 / 1.5 / 2, and the mirrored line is exactly symmetric.
- Default frame: GHDL 720p60 dec 1, Osc, 2 warm-up + 1 capture, with production v0.5.1 simulated the same way. The
  raw 10-bit output is **byte-identical** (0 of 921,600 px), on the final source after the timing restructure.
  - A first cut differed in 0.26 % of px. The refilled margin knots reached the margin's soft edge, and a hblank
    sample of the never-written column 120 leaked into the first pixel.
  - Both were fixed: only fully hidden knots are refilled, and column 120 is written only with a slide fraction.
- One extreme frame (`warpdiag_rings.png`: Rings, K2 400, K5 600, P12 300, K3 max): a zone plate split at the horizon
  seam and warped by the map. Its centre sits where the 3-field glide puts it.
- Not verified on screen: the slide in motion, Vanish in Plane mode, Bend at max on a camera, Zigzag ×2.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_warp/` (`notes.md` has every step).
- **Carrier.** The stripes are isolines of a product field, the phase of an hsync-reset oscillator whose
  frequency is the vertical ramp:
  - above the horizon: θ = (x + 94)(690 − y)/60000, hyperbolic arcs with a 30–110 px period;
  - below it: θ = (x + 94)(y − 305)/96000, a broad '/' fan.
  - Fitted from the structure-tensor orientation map (cos 2Δ = 0.92 top / 0.93 bottom) and the stripe
    period along the normal (`orfit.py`, `nper.py`). The oscillator's phase starts 94 px before the line.
- **Horizon seam.** A strong full-width seam at y 693–699, where the two laws meet. Weaker seams at 403,
  465/473, 585 and 933. Row-to-row shifts are only ±0.3 px, so there is no line tear.
- **Colourizer (channel split).**
  - R is the triangle of the phase. Its histogram is flat (18/17/16/17/18/14 %).
  - Each ~39-px cycle is double-humped: green → orange peak → dip (R ≈ 85) → lavender bump (R ≈ 145,
    B ≈ 167) → green.
  - Relative to the carrier, lavender sits on the R-rising side 57–64 % of the time (falling side
    35–51 %). B is explained 0.47–0.52 (R²) by a symmetric bump of R, and 0.54–0.62 with a 3-px lag.
    The rest is a second, finer grating (~17 px at the top right): the moiré.
  - Full-cycle palette, measured as the most common colour per phase bin (top):
    green (12,166,14) → teal → lavender (101,136,178) → orange (221,126,1) → olive (161,130,44) →
    lavender (97,137,174) → teal → green.
  - Below the horizon the orange is redder and there is violet (122,131,194).
- **Haze.** G falls down the frame: 146 (top) → 140 (540) → 104 (760) → 84 (bottom).
- **Warp.** The checker of steep seam bands (top left), moiré lenses (top right) and the band of hooks and
  loops with dark-green '/' lozenges (y 760–900) are a phase map D(x, y) added to the carrier.
- **Margin.** Black for x < 50, then a green ramp to G 112 at x 200. The stripe edge is x 212 down to
  y 520, then a staircase to x 35 at y 700 and ~20 at the bottom.
- **Dark green.** The (3,43,2) k-means cluster (7.8 %) is mostly the margin. Right of x 260 there is
  1.0 % dark green (the lozenges).

## What it does
1. **Carrier.** P = (x + 94)·f(y), with f = K·|y_h − 5 − y| above the horizon and K'·(y − y_h + 390)
   below it. F per line is computed on a serial multiplier; a 24-bit per-pixel DDA does the rest.
2. **Warp.** + n·D(x, y): a 120 × 136 knot map (16 virtual px × 8 lines, 4 bits, 1/16 cycle),
   interpolated wrap-aware vertically per line and horizontally per pixel. n = 0, 1 or 2 by distance from
   the horizon against P12's two fronts.
3. **Bend.** + the camera luma (Video) or an Osc lattice, × K4.
4. **Colour.** A 64-step full-cycle palette over frac(P), one above and one below the horizon. Then the
   green haze (offsets on Y, U, V per line, × K5), then the margin key.
5. The output stage, and the blanking gate on the generated avid.

**Fit** (`dfit11.py`):
- The map is seeded by coherent demodulation of R against the carrier. This is sign-consistent and
  cannot flatten.
- It is then refined on R, G and B against the full-cycle palette, anchored (0.03) to the demodulated
  phase, with wrap-aware smoothing (2).
- The palettes are re-measured each round as the most common colour per phase bin, never the mean. Means
  turned the earlier fits grey.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Warp** | The warp field: checker, lenses, loop band. 0 % = the clean perspective plane (arcs over a fan); the warp grows out from the horizon; 50 % = the photo; 100 % = a second front doubles the warp into a dense loop field. Glided. |
| K1 Stripes | Stripe density of the plane, ×0.5..×2 (centre = the photo). Glided. |
| K2 Horizon | The horizon seam row, ±256 lines (centre = y 695). Moves where the arcs and the fan meet, and the palette split. Glided. |
| K3 Flow | Stripe drift, bipolar, still at the centre ±8, square law. Default 640 = one period in ~6 s. |
| K6 Bend | How far the camera (or Osc) luma bends the stripe phase: 0..±1 cycle at full luma. Default 100 ≈ ±0.09. |
| K5 Haze | The green haze down the frame: none / the photo (centre) / double. |
| K4 Palette | Warp (the photo) / Ember / Ice / Toxic / Candy / Mono / Neon / Rust: transforms of the fitted palettes, so each keeps the photo's band structure. |
| S7 Seam | On: the photo's horizon seam (two laws) / Off: one through-zero law. |
| S8 Margin | The black/green left margin with its staircase edge. |
| S9 Mirror | The right half mirrors the left (the DDA reverses at the centre). |
| S10 Zigzag | A rolling per-line zig-zag on the stripe phase (24-line period, ±0.15 cycle). |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** camera luma, 2·(Y − 512), × K4 is added to the stripe phase. The stripes bend around the
  picture while the photo's composition stays readable at the default.
- **Osc:** a drifting triangle lattice (640 virtual px × 541 lines) feeds the same path. K4 stays live.

## Engine
- **Per field** (vblank zone: no avid for 1024 clocks):
  - glides (1/8 of the gap per field, deadband 4) on P12, K1, K2;
  - K3 squared on a serial shift-add;
  - carrier gains from 64-entry tables (kd = 2^((K1 − 512)/512)), the horizon, and P12's two fronts
    (1.41·P12 and 1.41·(P12 − 512));
  - the flow, Osc and zigzag phases;
  - the labelled Palette default is read as its index (v < 8).
- **Per line** (kicked at the source avid rise, computing the NEXT line):
  - F on a serial multiplier, then the line-start phase S0 = F·94 (334 at SD) + flow + zigzag;
  - the per-pixel step (×1 / ×1.5 / ×2 for 1080 / 720 / SD);
  - the warp count, the palette half, the haze offsets, the margin edge;
  - the map fill: 120 column values interpolated from two knot rows (even/odd-row ROMs) into a ping-pong
    256 × 16 column-pair RAM (`no_rw_check`).
- **Pixel path** (24 stages): carrier DDA (mirror reversal), column-pair read, horizontal wrap-aware
  interpolation, bend product, P = carrier + n·D + bend, palette ROMs, haze, margin key, then output
  conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma ×13/16) and the blanking gate.
- **Memories** (23 EBR): map 2 × 8192 × 4 (16), palettes 3 × 1024 × 8 (6), column RAM (1).
- **Colours:** in the hardware convention (U = Cr). Luma pre-boosted 16/13; Cb pre-emphasised ×1.08
  against the mandatory chroma filter (measured on the finest stripes at 1080).
- **Contracts:** generated avid (parity-locked); one 24-clock latency in every mode; virtual-1080
  geometry from the timing ID; HD HDMI bar fix before the gate.
- **Verification:**
  - The GHDL 720p render matches the integer model (`imodel.py`) plus output stage to a mean of
    0.59/255 (100 % within ±12).
  - Motion: frame 6 matches the model advanced 5 fields (0.60 against 3.21 un-advanced).
  - Against the photo:

    | | Region means (4×4, max/mean of 255) | \|Cr\| p50/p90 | \|Cb\| p50/p90 | Contrast |
    |---|---|---|---|---|
    | photo | 0 / 0 | 115 / 219 | 113 / 250 | 1.00 |
    | 1080 (model + output stage) | 12.2 / 3.1 | 125 / 211 | 107 / 264 | 0.97 |
    | 720p RTL | 12.3 / 2.9 | 122 / 208 | 101 / 264 | 0.97 |
    | v0.5.0 first cut (proto) | 16.8 / 6.3 | 87 / 192 | 126 / 251 | 0.67 |

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 75.51, SD Analog s2 78.70, HD HDMI s1 76.03, SD HDMI s1 84.91, HD Dual s1 75.36, SD Dual s1 72.31 MHz; 5449-5473 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-27, NOT flashed.** router2 (`./build_heirloom.sh warp`).
- **Build seeds:** HD HDMI = seed 4 (seeds 1–3 missed); the other five are seed 1.
- **Fmax (post-route):** HD Analog 79.92, HD HDMI 75.71, HD Dual 81.51, SD Analog 79.59, SD HDMI 79.72,
  SD Dual 79.81 MHz.
- 5440–5468 / 7680 LC (71 %), 23/32 EBR. All bitstreams pass iceunpack.
- `out/rev_b/ron/heirloom/warp.vmprog`, 433,992 B.
- HD HDMI has the thinnest margin; if a rebuild misses there, try router1 before restructuring.

## Deviations from the photo, and why
- **Fine moiré.** The top-right second grating (near twice the carrier frequency, not phase-locked to it)
  is not reproduced. The program has one phase, and that grating could not be fitted as a slow law.
  The top right reads as clean fine stripes without the lens beats.
- **Top-left checker.** Blockier and less regular than the photo's steep seam bands.
- **Knot notches.** Small rectangular steps where the photo's phase changes by more than half a cycle
  between 16 × 8 knots.
- **Colour order.** Lavender vs olive follows one fixed full-cycle sequence; the photo mixes them about
  60/40 per side (its finer B grating).
- **Static warp.** The warp map is a fixed realisation of the photo. Motion comes from Flow, Zigzag,
  Bend and P12.
- **SD (4:3).** SD shows the central 1440 virtual px, so most of the left margin and its staircase edge
  are cropped. 720p and 1080 show the same picture.
- **Chroma.** Chroma uses the 9-tap filter in every mode. At 720p the median |Cb| is 11 % under the photo
  (13.5 virtual px of filter); at 1080 it is within 5 %.
- **Interlace.** No bottom-field offset (field_n is tied high on hardware).

## Open questions for Ron (2026-11-09 batch)
1. **Haze and Margin** lost their controls; they are fixed at the photo, so K5 could carry the vanishing point and S8
   the Rings form. Do you miss either? The haze could come back as a third S8 look, or as part of the palettes.
2. **The warp slide rides on K3 Flow** (past the photo's speed). Would you rather have it on its own knob (e.g. K5,
   with Vanish moved elsewhere), or slide it vertically as well?
3. **Rings centre.** The rings sit on the horizon at the screen centre by default (Vanish moves them sideways,
   Horizon up and down). Should they rather start where the photo's vanishing point is (off the left edge, quarter
   rings)?
4. **K1 Stripes** now spans ×0.25..×4: the hairline end (7.5 px stripes) aliases into moiré at SD. Keep, or stop at ×3?

## Hardware watch list
- **Renamed:**
  - K5 Haze → **Vanish**;
  - S8 Margin → **Form** (Plane / Rings).

  The haze and the left margin are fixed at the photo.
- **K3 Flow past the default** slides the warp field: at the ends it crosses the screen in ~2.7 s. The map reflects at
  its edges (ping-pong), so a mirror line passes through. In the upper rows a sliver of up to 3 unfitted knots (≤ 48 px)
  can show as unwarped stripes.
- **K4 Bend near max** folds the stripes up to ±4 cycles. On a noisy camera picture this may read as sparkle.
- **Rings near the edges and K1 high** reach moiré (a zone plate): intended, but watch for HD HDMI bars there. The
  output conditioning is unchanged.
- **Pre-existing, kept as in v0.5** (the default frame is raw-identical): a faint luma dot in the first pixel of each
  line over the black margin. The luma filter window there includes one hblank sample. It's worth a shell fix later
  (gate the luma filter input outside avid).
- Build: router2. HD HDMI is on seed 2 (seed 1 missed); the other five are on seed 1. HD margins are 81–88 MHz.

## Next step
Wait for Ron's preview notes. Rebuild the diag with `.heirloom_work/diag_build.sh warp`. On approval:
`./heirloom_diag.sh merge warp`, then repack the previewed diag bitstreams under the production TOML instead of
re-routing:
1. copy `build/programs/ron/warp_diag/rev_b/bitstreams/*.bin` into a scratch dir;
2. add the production TOML's `program_config.bin`;
3. run `vmprog_pack.py --no-sign --hardware rev_b --toml-path programs/ron/heirloom/warp/warp.toml <dir>
   out/rev_b/ron/heirloom/warp.vmprog`.

Then update the production SUMMARY / README / PHASE2 row and commit only warp's files.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. Here the camera strength was on K4 (Bend), so K4 and K6 swapped: **K6 = Bend** (the camera strength, default now 512 = 50 %), **K4 = Palette** (was K6). Nothing else changed: the VHDL swaps the two knob inputs (the processing is identical), the TOML blocks and preset keys moved with them (the Controls rows above are renumbered). v0.6.1, rebuilt with router1: all six seed 1; HD worst 92.25 MHz (HD Analog 94.24, HD HDMI 96.98, HD Dual 92.25), LC LC. Rebuild: `NEXTPNR_ROUTER="--router router1" ./heirloom_diag.sh build warp`. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/.

## K6 top +25 % and default 100 % (2026-11-15)

**Bend type**: K6 Bend adds the camera (or Osc) to the stripe phase through the two-slope gain f_bg (0 .. 127). The K6 default is now 100 % (orchestrator).
- A free-running register f_bg5 = f_bg + f_bg >> 2 (0 .. 158) now feeds the pixel product, which widens to 12 x 9 bits. Full luma bends up to +-5 cycles (was +-4).
- Not rendered; checked by GHDL analysis and the build.
- Build: v0.6.2, router1; HD Dual seed 2 (seed 1 stalled router1 at the 600 s timeout under machine load), seed 1 elsewhere; worst HD 92.95 MHz (HD Analog 97.80, HD HDMI 92.95, HD Dual 94.14); 6,573 / 7,680 LC. `.kinetic_work/k6_builds.txt`: diag warp OK router1 worst-HD 92.95 LC 6573.
