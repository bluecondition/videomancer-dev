# FLATLINE — summary

Proof-of-concept image-flattening lab (cel matte / cut paper / traced ink): five build
variants from one source, `programs/ron/flatline/flatline.vhd` (master; `variants.py` stamps
the siblings and generates every TOML from `_template.toml`). Design and deviations:
[DESIGN.md](DESIGN.md). Control card: [CONTROLS.md](CONTROLS.md).

Status: **v0.1.1: all five builds timing-closed and packed (16:05-16:45, 2026-11-09), none flashed.**
**v0.1.1 fixes:** the first GHDL preview renders found four defects that no build report
shows: the dry (raw) path read its chroma back from the delay ring crossed (the raw half of the
wipe would have been hue-swapped on hardware), the representative-colour table had sectors 2/3
and 6/7 crossed (red drew as yellow-orange, blue as teal), the XDoG threshold inked the whole
picture, and the Canny gain was 4x too low to draw anything on a photo. All four are fixed in the
source (DESIGN.md section 4, "Fixed after the first renders") and in the vmprogs in out/. Brief: Ron's FLATLINE prompt of 2026-11-09; Ron's
standing note: if the mode filter and the bilateral conflict for budget, the mode filter stays
(it does; every build has it).

## What is in which build

| | A `flatline` | B `flatline_ink` | C `flatline_canny` | D `flatline_kuwa` | E `flatline_finish` |
|---|---|---|---|---|---|
| K1 smoother | Off / Bilateral (separable 5x5, octave sigma) | Off / Median 3x3 | Off / Median 3x3 | Off / Kuwahara 5x5 (range) | Off / Median 3x3 |
| K4 lines | None / Edge / Edge White / Edge Bold | ... / XDoG | ... / Canny-lite | ... / Edge Bold | ... / Edge Bold |
| Quantization | hard, fixed / adaptive (all builds) | | | | hard / **soft** (S7) |
| Cleanup | horizontal mode always on, vertical mode on S9 (all builds) | | | | |
| S10 | Temporal vote | Temporal vote | Canny chroma gradient | Temporal vote | Temporal vote |
| K6 drop shadow | not built | not built | not built | not built | **built** |
| S11 view | Final / Lines (all builds) | | | | |
| Not built anywhere | open/close, median+bilateral two-pass, 2x2 split view, designer palette, XDoG/Canny white ink (DESIGN.md section 4) | | | | |

Why five narrow builds: the shared pipeline (half-res analysis, chroma classes, label
quantizer with hysteresis, separable 5x5 mode filter, percentile thresholds, boundary ink,
output conditioning, dry-video wipe) is ~73% of the HX4K on its own after four rounds of
trimming (the first full version synthesized to ~2x the chip). One smoother plus one line
algorithm per build is what fits at ~85-92%.

## Cost table (standalone yosys, program only; LUT4 + carry + FF; routed LC ~0.85x + 1137 core)

Measured on the final source (`~/flatline_work/synth_*.log`, `ablate.py` configurations).
"Base" = everything shared: F front end + dry ring, half-rate luma/chroma windows, label
quantizer with hysteresis and temporal vote, both mode passes, boundary ink, percentile
thresholds, compose + conditioning + wipe.

| Piece | LUT4 + carry + FF | ~routed LC | % of HX4K | Notes |
|---|---|---|---|---|
| SDK core (passthru, routed) | — | 1137 | 15% | measured |
| Base pipeline (with temporal vote) | 5402 | 4590 | 60% | 24 EBR |
| Median 3x3 | +866 | +740 | +10% | sorted-column network, 19 CE |
| Bilateral 5x5 separable, octave sigma | +1340 | +1140 | +15% | 8 taps; a full 24-term 5x5 was ~2200 LC |
| Kuwahara 5x5 (range) | +2046 | +1740 | +23% | 5-column stat register is 260 FF |
| XDoG (3x3 vs 5x5 binomial) | +804 | +680 | +9% | +1 EBR (ink RAM widens) |
| Canny-lite (NMS + hysteresis) | +1012 | +860 | +11% | +3 EBR |
| Soft quantization (count blend) | +483 | +410 | +5% | |
| Drop shadow (16-line ring) | +350 | +300 | +4% | +2 EBR |
| Temporal vote | ~+60 | ~+50 | +1% | +1 EBR |
| Region-boundary ink | in base | — | — | ~2 EBR (cleaned row + bit history) |
| Removed: 2x2 split + palette | ~150 LC | | | lines-only view kept |
| Removed: k-means engine | ~1000 LC | | | percentile placer ~350 LC instead |

Build totals (standalone sum / estimated routed LC): A 6742 / 6870 (89%), B 7072 / 7150 (93%),
C 7280 / 7325 (95%), D 7448 / 7470 (97%), E 7095 / 7170 (93%). Real routed LC has come in
about 5 points under these estimates (see the build record).

## Build record (v0.1.1, 2026-11-09 evening; judge the LAST post-route Fmax per config)

v0.1.1 = v0.1.0 plus the four preview-render fixes (output mux wiring, a constant table, the
DoG threshold arithmetic and a 5-bit gain): same LC within a few cells, same EBR. The v0.1.0
numbers are kept in the line under each row where they differ in a seed.

Builder default = router2 with up to 6 seed retries; A was built with
`NEXTPNR_ROUTER="--router router1"` (router2 missed every seed on the un-split bilateral and
router1 was the one that closed its diagnostic route). Seeds other than 1 are noted per config
(a non-1 seed means that seed must be reused for a repack: `SEED=<n>`); configs in order
HD Analog / SD Analog / HD HDMI / SD HDMI / HD Dual / SD Dual. SD configs are judged against
the 27 MHz SD clock, so their Fmax numbers are informational.

| Build | LC | EBR | HD Analog | SD Analog | HD HDMI | SD HDMI | HD Dual | SD Dual | Result |
|---|---|---|---|---|---|---|---|---|---|
| B flatline_ink (median + DoG ink) | 6661 (87%) | 29 | 84.63 s2 | 72.38 | 83.49 s4 | 79.26 s3 | 82.26 | 77.07 | **all pass** (v0.1.0: 74.76 / 78.74 s2 / 79.23 s2 / 80.56 / 83.10 / 70.68). router2 stalled on HD Analog s1, HD HDMI s1+s2, SD HDMI s1+s2 |
| C flatline_canny (median + Canny) | 6848 (89%) | 29 | 77.63 s2 | 79.75 s2 | 80.44 | 80.08 | 74.86 s2 | 75.68 | **all pass** (HD Dual margin 0.6 MHz; v0.1.0: 85.19 / 81.70 / 82.32 s2 / 76.31 / 82.32 / 80.78 s2) |
| D flatline_kuwa (Kuwahara) | 6859 (89%) | 28 | 76.41 | 70.02 | 79.55 | 79.38 | 79.47 s3 | 78.02 | **all pass** (v0.1.0: 78.16 / 77.87 / 74.45 / 81.41 / 74.85 s4 / 74.63) |
| A flatline (bilateral), router1 | 6587 (86%) | 28 | 82.18 | 81.10 | 86.82 | 82.08 | 78.13 | 81.80 | **all pass**, all seed 1 (v0.1.0: 82.18 / 85.41 / 84.36 / 80.69 / 78.73 / 80.74, also all seed 1; before the bilateral stage re-split every HD config sat at 69.6-70.9 on router1 seeds 1-6) |
| E flatline_finish (median + soft + shadow) | 6777 (88%) | 28 | 74.55 s5 | 68.38 | 75.61 | 71.85 s2 | 79.56 s6 | 73.48 s2 | **all pass** (HD Analog margin 0.3 MHz, HD HDMI 1.4; v0.1.0: 82.14 / 75.10 s2 / 82.24 s2 / 70.80 s3 / 77.50 s3 / 70.81). router2 stalled on 9 of its 17 seed attempts; before the shadow popcount pipeline HD Analog sat at 52-58 MHz on both routers |

vmprogs: `out/rev_b/ron/flatline.vmprog`, `flatline_ink.vmprog`, `flatline_canny.vmprog`, `flatline_kuwa.vmprog`, `flatline_finish.vmprog`.
The marginal HD configs (E HD Analog 74.55, C HD Dual 74.86) are real passes but thin; if a
mode looks wrong only on that input, suspect timing before the algorithm and re-route that
config with another seed (`make ... CONFIG=<cfg> SEED=<n>` + repack). A has 4-12 MHz of margin
on every HD config. E's HD Dual closed on the last retry (seed 6) at 79.56, a real pass.
SD configs are judged against the 27 MHz SD clock.

## Previews (GHDL render matrix, 2026-11-09)

https://claude.ai/artifact/XH5t8MW9oYVUY33XAcPBP2 -- 99 renders at 960x540 (1080p, decimation
2): every build at 12 shared settings plus its own extras on a photo, and portrait / synth /
test-card sources, colour-corrected to hardware order (stimulus Cb/Cr pre-swapped, render
post-swapped). Made with `~/flatline_work/previews/run_previews.py` + `gen_page.py`. Kernels
appear 2x coarser than on HD hardware; timing, noise and motion effects are not represented.

## Sim check (GHDL image tester, 480x270, build A)

Two renders only, to catch gross pipeline errors (the standing rule is build + hardware eyes):
`~/flatline_work/sim/a_bars4.png` (test card, wipe 50%) and `a_photo2.png` (gallery bear, wipe
100%). Mattes, adaptive levels, edge ink, the wipe seam and the blanking all behave; the
first render caught a real bug (a `resize(signed, 10)` in the compose stage folded every
value >= 512: the processed half came out saturated green). Sim traps worth knowing:

- The tester drives labelled knobs at exactly 512, which sits on a zone boundary, so the +-8
  hysteresis never resolves and K1/K3/K4 stay at their reset zones (Off / 4 levels / None).
  On hardware the firmware loads the label index. In sim always `--set` K1/K3/K4 explicitly.
- Two sims of the same program at once share `/tmp/lzx_vit/<prog>` and one of them dies with
  "Output file is empty". Run them one after the other.
- At decimation 4 the fixed pixel kernels are 4x coarser relative to the picture (a 30-px bar
  is 3.75 half-res samples, so a 2-sample ink footprint eats most of it). Judge geometry on
  hardware or at decimation 1.
- Colours appear hue-swapped in the sim, on the raw half as well as the processed half: the
  tester feeds the program hardware-order chroma and renders the output as standard BT.601
  (a dry render of the test card shows its red stripes blue). One Cb/Cr swap on the rendered
  PNG gives what the hardware will show (`~/flatline_work/previews/uvswap.py`); the preview
  page was made that way.

## Recommendation (what to look at first)

Per resource, from the cost table (the look itself still needs Ron's eyes; every build is
"built, not flashed"):

1. **Stage 1 = Median 3x3, Stage 3 = horizontal + vertical mode (S9 on), Stage 4 = region
   boundary (K4 "Edge" / "Edge Bold")** -- that is B, C or E with K4 in its first three zones.
   ~5330 LC (69%) all in, 26 EBR, and it leaves ~2300 LC for a finished program's own
   features. The median only has to kill single-pixel noise before quantization; the mode
   passes do the flattening, and the boundary ink is free (it reads the labels the mode
   filter already produced). Expect: regions with crisp 1- or 2-px borders, straight ink
   along every region change, no ink inside a flat region.
2. **Bilateral (A)** costs +400 LC over the median for a softer gradient-to-matte transition
   (K2 low keeps texture, high blurs it flat). Worth it only if the median+mode result shows
   mottled region borders on gradients (sky, skin). It was also the only build that needed
   router1 and a stage re-split to close at HD: the 5x5 window + range test is the timing
   hog, not the LC.
3. **Kuwahara (D)** is the most expensive smoother (+1000 LC over the median) and the most
   "painterly"; try it last, and only if A's gradients still band.
4. **XDoG (B) and Canny-lite (C)** are the line alternatives. XDoG gives drawn-looking strokes
   that follow luma detail inside regions (texture lines, not just borders); Canny-lite gives
   thin single-pixel contours including chroma edges (S10 "Chroma Grad"). Either costs ~700-
   860 LC and 1-3 EBR more than the boundary ink. For the cut-paper / cel look the boundary
   ink is the right one; XDoG is for the ink-trace look.
5. **Soft quantization and the drop shadow (E)** are finish-stage items (+400 / +300 LC,
   +2 EBR for the shadow). Soft only shows at low level counts (K3 2..4); the shadow needs
   the black background (bin 0) to read as a cut-out.

What to check on hardware first, in order: the B build, K1 Median, K3 "4", S9 on, K4
"Edge", P12 half way: the processed half should be flat mattes with black ink along the
region borders and a clean seam at the wipe. Then S10 (temporal) on/off on a static scene:
borders should stop flickering. Then K3 up through "8 Coarse" (chroma collapses to 4 hues x
2 saturations at the top).
