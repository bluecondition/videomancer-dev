# Ricochet — Heirloom #30

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/ricochet.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-03, phase 2): diag preview built, awaiting Ron's preview** (`Ricochet Diag`,
`out/rev_b/ron/_diag/ricochet_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`). Production v0.5.1
(`programs/ron/heirloom/ricochet/`) is untouched. Phase-2 working files: `.heirloom_work/rb_ricochet/p2/`
(sweep.py, check.sh, psim.sh, renders, pre-trim ricochet_v060a_7480lc.vhd / .vmprog).
v0.5.0 (2026-10-21): rebuilt from the photo `photos/WIN_20230515_23_08_59_Pro.jpg`, built, NOT flashed.
Work files: `.heirloom_work/rb_ricochet/` (`notes.md` has the full measurement log; `cmp_final.png` = photo |
float proto | RTL Osc | RTL Video).

## Phase 2 (v0.6.0)
No notes from Ron (free hand, 2026-11-03 batch). The v0.5 knobs were all live and bold, so this round makes the
chevron systems separate modules and widens the two knobs that stopped short. The line engine counts the
vertical triangle's phase wraps since the top, so each line knows its system (e_sys); each system gets its own
DDA mode (S7 Alternate) and its own sideways LFO (the K5 ricochet).

| Was (v0.5) | Now (v0.6.0) | Notes |
|---|---|---|
| K1 Chevrons: 1 .. 6 systems per frame height (1 + 5k^2) | **0.47 .. 7.95** (1911 + (k^2 >> 4) * 15 >> 5, Q12), bit-exact at 473 | from one system taller than the frame to ~8 thin rows |
| K2 Core: P0 20 .. 260 u (linear) | **P0 0 .. 498 u**, two slopes about the photo (4664 / 9464 Q10 per step), bit-exact at 353 | 0 = no core; top = red covers most of the frame (the reach end rides along) |
| K5 Speed: bipolar band flow | band flow unchanged, **plus the ricochet** past the photo's speed | depth = (\|v\| - 136) * 8 hw (0 for K5 376 .. 648, the default 640 included), max +-3008 hw = the vertex sweeping +-~700 px at 1080p, the cores swelling / shrinking; even systems on tau, odd on tau2 (rates 3/4 (\|v\| - 8) and 1.375x per field, periods 12 .. 2.9 s / 8.9 .. 2.1 s), each a quarter cycle on from the last; triangle LFO |
| S7 Point: Right / Left (dead while Twin was on) | **Point: Right / Alternate** (option relabelled) | every second system points left; with S8 Twin, bowtie and **Burst** (Twin fired from the centre: red diamond core, chevrons out to both edges) systems alternate |

Unchanged: K3 Bands, K4 Seams, K6 Etch, S8 Twin, S9 Ground, S10 Palette, S11 Source, P12 Reach.

**Presets:** Photo; Full Reach; Ricochet (Alternate, K5 960); Bow and Burst (Alternate, Twin, Azure, P12 620);
Herringbone (Alternate, K1 1023, K3 700); Echoes; Red Giants (K2 900, K5 1023, P12 800); Osc Weave.

**Engine (new).**
- The DDA table has 12 entries (class * 4 + mode; mode 3 = Burst, x' = 1920 - 2n xs, reversed at the centre by
  the existing Twin hold). The constants are loaded per line at e_lc 3 for the next line (after the beam has
  taken this line's starts; c_dd depends only on class and Twin, so it never changes inside a field).
- Ricochet: e_lc 51 .. 71, phase -> triangle -> depth x |tri| on the line multiplier -> next line's
  P0 / Pe = s_p0 / s_pe - offset, latched at g_start and used at pixel stage 6. Nothing added to the pixel path;
  latency 28 unchanged.
- e_lc is 7 bits; e_ph carries into the 4-bit system counter.

**Build** (router1, `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh ricochet`, 2026-11-03):
6/6 seed 1. HD Analog 84.55, SD Analog 78.99, HD HDMI 82.34, SD HDMI 81.54, HD Dual 82.79, SD Dual 78.17 MHz.
7,183 - 7,196 / 7,680 LC (93.7 %, +~330 over v0.5.1), 13 / 32 EBR. iceunpack OK x6, packed config carries the new
labels and presets. First build 97.4 % LC (a 7th glide for the depth, per-system rates (8 + k) / 8 by a per-line
shift-add chain, a 4-arm K1 curve); trimmed with no feature lost. No pin-skew screen.

**Checks.** GHDL front end clean. p2/sweep.py at RTL widths: K1 / K2 bit-exact at their defaults and monotonic,
no overflow (s_tmp, s_p0, s_pe, n_p0l, n_pel), depth 0 on 376 .. 648, LFO phase continuous. GHDL 720p dec 2 Osc
defaults: diag vs production v0.5.1 0 of 230,400 px differ (pre-trim source; the trims are bit-exact at the
default). 720p dec 1 Alternate + Twin + K1 760: bowtie / burst alternation with clean seams. Not rendered:
the ricochet motion itself.

The sections below are the production (v0.5.1) SUMMARY, unchanged.

## Reading of the photo
One scalar field, coloured by a 1-D colouriser. Every number below was measured and fitted (`fit*.py`,
`hfit_hw3.py`, `fitj2.py`); the fitted constants live in `ricochet.py`.
- **Colour path.** Every pixel lies on one path (k-means, 12 centres, median 4 RGB off it): red (224,14,3) →
  orange (255,75,1) → salmon (244,120,109) → pink (232,130,145) → lavender (182,147,204) → periwinkle
  (150,157,224) → cyan (24,196,237). The scalar *s* is the arc position along it.
- **Two chevron systems.** Cores on lines 256.0 and 779.0 (flip-correlation), seams on 517.5 and −5.5: a vertical
  triangle with a period of 523 lines (2.065 per frame).
- **Field.** Fitted first by palette purity, then by joint s-error (photo units):
  - `phiA = softmax(−56.3 + 0.411 x − 4.32e−5 x², −35.1) + Y (1.347 − 3.73e−4 x)`. This is a horizontal ramp
    with a soft start (the bands bend horizontal over the first ~80 px) and a slightly falling slope, plus the
    triangle `Y = |y − core|` through a gain that falls across the line (an x·Y product term).
  - `phiB = 332.2 − 0.102 x + 3.977 (261.5 − Y)`: the same triangle, inverted and 4× steeper.
  - `phi = min(phiA, phiB)`. The chevron arms kink into the seam stripes along the crease where the two meet, and
    the flat ground is darkest exactly on that crease, which is what identifies the min.
  - Vertex rounding is only ~4 lines (measured by tracking one contour), so the tips are sharp V's. A
    purity-only fit wanted 22, which renders U-shaped tips.
- **Colouriser** (fitted to the measured LUT, error 0.00033):
  - 8.34 bands between the core (phi 103) and the reach (397 beyond it). A 0.28 chirp takes the band period
    from 60 to 42.
  - The bands ride a base that rises from orange to cyan (0.318 + 0.599 w), amplitude 0.198.
  - The red core is a steep wall with a red → orange shoulder `0.06 + 0.005 (d + 27)`.
  - At the reach a steep cut (0.029 per phi unit) drops to the flat ground (s = 0.205), which then darkens
    slowly (−0.00046 per phi unit).
- **Bottom flyback.** From line 1032 the triangle keeps going at 1.2× instead of folding at the bottom seam. That
  gives the red strip at the bottom right and the fine stripes at the bottom left (error 0.113 → 0.006 on
  those rows).
- **No textures.** No tearing, comb or noise. The 9-line seam stripes keep full amplitude (vertical blur under
  1.5 lines); the softness is the sinusoidal colouriser itself.

## What it does
1. **Per field.** A vblank-zone sequencer (one serial multiplier on fixed slots, one serial divider, one shared
   glide; one small operation per zone clock) computes, from the glided knobs:
   - the vertical phase step and the flyback step;
   - the core level P0, the reach D, 2^21/D and the reach end;
   - the band frequency, the integer band count for the chirp, the seam gain c and the etch gain;
   - the band-flow step.
2. **Per line.** An engine kicked at each line start computes the next line on one signed serial multiplier:
   - the triangle Y from a 20-bit phase, plus the bottom flyback;
   - the DDA starts `Y·gA0`, `Y·gA1·xs` and `gB0 + c·(4096 − Y)`.
3. **Per pixel:**
   - **Fields.** The F ramp is a 2nd-order DDA with a quadratic knee. The Y-gain and seam-field DDAs run 2 and 3
     pixels late, so they meet the pipe at the stage that uses them. Twin holds and reverses all three at the
     centre. The field is `phi = min(F + YA, B) + etch·luma`.
   - **Band phase.** `(phi − P0)·Nb/Ddef + chirp ROM·Nb − t`, as 3-bit row partial products summed over two
     stages, into a full-wave cosine ROM.
   - **Colour.** `s = clip(max(min(base − a0·cos, limit), shoulder))` into the palette ROM along the photo's
     path. The side values {base, limit, shoulder} ride a 2-EBR ring.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Chevrons | the two chevron systems | Vertical triangle frequency at fixed amplitude: 1–6 systems per frame height (2.07 at the default 473). More systems give flatter arms. |
| K2 Core | the red teardrops | Core level P0 = 20–260 (103 = the photo): small cores ↔ big red cores pushing the bands right. |
| K3 Bands | the bands in each chevron | 2–24 bands across the photo's reach (8.3 = the photo). |
| K4 Seams | the stripe bands at the seams | Seam slope c from 13.7 (thin dense seams) to 0.75 (seams take over the ground); 3.98 = the photo at 512. |
| K5 Speed | the bands | Bipolar band flow, ±8 still. Right streams the rings out of the cores toward the vertex (default 640 ≈ 0.42 bands/s). |
| K6 Etch | the whole band field | Picture luma pushed into the field (up to ~19 bands); moderate at the default 200. |
| S7 Point | chevron direction | Right (photo) / Left. |
| S8 Twin | the chevrons | Off / On: chevrons fired from both sides, meeting in a bowtie of flat ground. |
| S9 Ground | the flat orange ground | Flat (photo) / Echo: the chevron block repeats past the reach. |
| S10 Palette | the colours | Photo / Azure: chroma mirrored (azure ground, teal cores, orange outer chevrons). |
| S11 Source | — | Video / Osc. |
| **P12 Reach** | the outermost chevron / where the ground begins | 0 % = red cores with one or two rings on flat orange. Default 490 = the photo (vertex at 83 % of the width). 100 % = no ground: chevrons and seam stripes over the whole frame, the outer area glowing cyan. Glided 1/8 per field. |

## Video
The photo is fully synthetic, so the camera is woven into the patch rather than shown. Its luma (stored as
`(y >> 2) − 16` in an 8-bit line buffer) is added to the field before the colouriser:
- the band contours bend around the picture;
- bright areas push the reach outward, so silhouettes appear along the ground boundary.

At the default Etch a full-white area shifts the bands by about half a band, and the composition stays intact.
Osc mode feeds the shell's drifting triangle lattice through the same path (the bands wobble gently), so Etch
stays live.

## Engine
- **ROMs.** `ricochet.py` → `ricochet_pkg.vhd`:
  - chirp, 512×12;
  - full-wave cosine, 1024×9;
  - knee square, 128×8 (LUT logic);
  - palette, 256×32. Y is pre-scaled 16/13 for the output luma scale; Azure is a chroma mirror applied at the
    output.
  - DDA constants: a 9-entry table, 3 width classes × Right / Left / Twin.
- **Units.** 4096 hw = the triangle amplitude (261.5 photo units). The vertical phase is Q20; the DDAs are Q12,
  with F's differences in Q20.
- **Geometry.** Virtual 1920×1080 from the timing ID: an x step per class (1 / 1.5 / 2.667), the y step from the
  shell table.
- **EBR (13).** 4 luma line buffer, 2 side-value ring (`no_rw_check`; read 1 behind write), 3 cosine,
  2 chirp/table, 2 palette.
- **Contracts.**
  - generated avid (parity-locked);
  - one 28-clock latency in every mode;
  - chroma 9-tap, luma [1 2 1] and luma 13/16 before the blanking gate on the latency-aligned generated avid;
  - the line engine is restarted from the vblank zone every field;
  - vsync is used only through the shell's idle-zone detector.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 82.34, SD Analog s1 84.08, HD HDMI s1 84.72, SD HDMI s1 84.13, HD Dual s1 85.80, SD Dual s1 85.33 MHz; 6842-6875 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- v0.1.0 (2026-10-20): abstract four-colour chevrons, judged not like the photo.
- v0.5.0 (2026-10-21): rebuilt by the REBUILD.md method. **Built with ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh ricochet`); all six configs on seed 1, post-route:

  | Config | Fmax |
  |---|---|
  | HD Analog | 86.33 MHz |
  | HD HDMI | 84.43 MHz |
  | HD Dual | 83.81 MHz |
  | SD Analog | 78.78 MHz |
  | SD HDMI | 82.77 MHz |
  | SD Dual | 86.56 MHz |

  - 6840–6866 / 7680 LC (89 %), 13 / 32 EBR, iceunpack OK ×6.
  - router2 does not close this netlist (69.5 MHz on HD Analog).
  - **Not flashed.**
- RTL verification (720p60, decimation 1): every per-line value is bit-exact with the integer model (`imodel.py`).
  Osc mode at default knobs matches it to a mean of 4.3 RGB. The bands flow outward between captures 6 fields
  apart.

## Deviations from the photo
- The capture borders (black top rows, dark left edge) are not drawn.
- The bands sit within a few px of the photo's, not exactly on them (mean abs RGB error 10.7 for the float proto,
  11.1 for the integer model). The residual is sub-band phase.
- The colouriser is one smooth function, so the photo's slight band-to-band colour unevenness is not reproduced.
- Three small simplifications, all invisible at a glance:
  - vertex tips are drawn sharp (the ~4-line rounding is folded into the ramp offset);
  - the band amplitude is constant (the photo's falls by ~0.04 across the reach);
  - the chirp is scaled by the integer band count.
- Knobs start at the TOML defaults and glide, so after power-up the picture eases to the real knob positions
  within about a second.
- **SD:** x is mapped to the full width, so on 4:3 the chevrons are 25 % narrower (steeper arms). The seam stripes
  (~11 lines at 1080) are only ~2.5 lines at 480i and will alias. 720p matches 1080.

## Open questions for Ron
1. Bring the plain Left mirror back (e.g. as a third state via Twin), or keep Alternate?
2. Give the ricochet its own knob instead of the outer part of K5's travel (it would cost Core or another knob)?
3. Slide each system as a rigid block (a true x-shift, more LC) instead of the current level shift, which also
   swells the cores?
4. Cap K2 Core's top at ~400 u instead of 498 u?

## Hardware watch list
- Renamed option: S7 Left -> Alternate (the plain mirror is gone).
- Alternate shows nothing with K1 below ~1 system (only one system on screen).
- Systems offset against each other leave a horizontal crease at each seam centre (by design).
- Ricochet depth uses the raw K5 (no glide): up to ~3 px pot-noise jitter while bouncing; no bounce within
  8 counts of 640.
- SD: seam stripes alias more at high K1. Timing has 8 - 10 MHz margin on all HD configs.

## Next step
On approval: `./heirloom_diag.sh merge ricochet`, then repack the previewed diag bitstreams
(build/programs/ron/ricochet_diag/rev_b/bitstreams/) under the production TOML's program_config.bin with
`vmprog_pack.py --no-sign --hardware rev_b` into out/rev_b/ron/heirloom/ricochet.vmprog instead of re-routing;
update SUMMARY / README / PHASE2 and commit only ricochet's files and rows.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 now loads at 100 % (initial_value 1023, v0.6.2) and its top end is 25 % stronger. **Bend type**: K6 Etch
pushes the camera luma into the band field (the bands bend round the picture, bright areas push the reach). Same
change as production ricochet: the etch scale constant in the per-field serial multiply 687 -> 859 (x1.25 at every
knob position; s_eq <= 857 of 10 bits, ~19 -> ~24 bands at full white). The Osc rides the same gain. Widths as in
production (nibble products <= 12.9k of 14 bits, etch sum <= 218k of 18 bits, the field gains at most 13.7k in 19
bits signed). Production render: `.heirloom_work/rb_ricochet/k6b/k6b_cmp.png`.
**Build:** router1, seed 1 on all six; HD Analog 86.79, HD HDMI 78.68, HD Dual 86.03 MHz (worst HD 78.68); SD 84.11 /
82.96 / 85.06 MHz; 7181-7221 / 7680 LC (94.0 %), 13/32 EBR. Preview `out/rev_b/ron/_diag/ricochet_diag_v0.6.2.vmprog`.
NOT flashed.
