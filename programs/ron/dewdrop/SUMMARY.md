# DEWDROP (v0.3.0 — glossy lens-drops over a waving rainbow band field)

**STATUS: v0.3.2 timing-closed on all 6 configs and packed 2026-10-16
(`out/rev_b/ron/dewdrop.vmprog`), NOT yet flashed.** Builder (router2):
HD Analog seed 1 75.31 MHz; HD Dual seed 6 76.18 MHz; SD all pass. HD HDMI
missed router2 seeds 1–9 except seed 7 (74.99); **hand-installed with
router1 seed 1 = 80.30 MHz** (`tools/` notes below: rebuild that config
alone, then repack — the builder's retry loop deletes hand-installed
bitstreams). 7030–7046 LC (92%), 13 EBR. Description trimmed to 117 bytes
(the first pack failed at 135). Designed from a
reference still (glossy bubble/lens shapes with dark rainbow-fringed rims,
saturated rainbow bands bending in S-curves inside them, over a mint/cyan
pastel ground). Two earlier cuts never fit (v0.1: 173% LC, v0.2: 107%);
see "Fit history" for what each restructure bought.

## The look

- A rainbow band field: a 256-entry wrapping palette (yellow, red, magenta,
  dark, blue, cyan, green, black, back to yellow) indexed by
  `((x' + A·sin(y'·fw + ph)) · bs) >> 8`, so the vertical bands bend
  sideways in a sine down the screen (Wave, 1.5 cycles per screen) and
  scroll/phase-animate (Flow).
- Eight circles (four big, four at 0.61×) orbit on Lissajous paths. Inside
  a circle the field is sampled with the coordinate pulled toward the
  circle's centre (Refract = magnification), plus a dispersion term that
  rotates the hue toward the rim. Overlaps go to the lowest-index circle
  (big circles win over their small partners).
- The union of the drops wears a dark rim (0 ≤ m < D) lit from the
  top-left (white highlight, black shadow, from `−(dx+dy)` scaled per
  frame) with the outer quarter of the ring a full-spectrum fringe.
- Outside: the same field as pastel (`Y/2 + 368`, chroma halved), or dark,
  or a flat mint. Chrome greys the field but keeps the fringe.
- P12 Swell: radius W/32 → W/2; at the top the discs cover the screen and
  their rim arcs cross the whole picture.

## Engine

- **Per circle, per pixel: two adders.** `m_j = (x−cx)² + (y−cy)² − R²`
  runs along the line by second differences; the increment `2(x−cx)+1` is
  a register `2·s_bcol + (1−2cx)` computed from the column counter one
  cycle ahead, so the accumulator has no load/increment mux (the DDA is
  aligned with `s_bcol_d`, one cycle behind the generated avid).
- **No argmin tree.** Inside = the sign bit of m_j (free); rim = one
  compare per circle; the owner is a priority encoder over the inside
  flags (else over the rim flags) and one 8:1 mux of m. The tree version
  cost ~130 LUT + 50 carry per node (7 nodes) for a nearest-circle seam
  nobody would notice.
- **Line-start values from a circle-table EBR.** A line FSM computes
  `M0_j + dy²` for the NEXT line during the current active line (one
  serial 16×16 square per circle, dy in Q4 horizontal-pixel units from a
  16/18/15 step so circles are round in every raster; interlace seeds the
  bottom field one frame line later) and loads the eight accumulators at
  the following hsync. No y-DDA state at all.
- **cx/cy per pixel from two lookup EBRs** addressed by the owner index
  (written by the sequencer alongside the table); dx, dy are subtracted
  one stage later.
- **Vblank sequencer**: two knob slews (Wave, Swell; the other knobs use
  their top bits), nine products on one 24×12 serial multiplier (R, R²,
  Rs², w, (R+w)², (Rs+w)², A, Flow², per circle Wh·|sin θ|, Hh·|sin φ|,
  cx²), shift amounts from a shared `ilog2`, orbit phases θ/φ kept in the
  table and advanced by per-circle rates (Drift), sine ROM time-shared.
  ~700 cycles.
- **Pixel path**: 22 stages + [1 2 1]/4 Y/U/V conditioning + blanking gate
  + 1 pad = latency 28 (all modes). Multiplies: dx·k, dy·k (6-bit k split
  3/3), b·fw (4-bit), sin·A (8-bit), p·mantissa (5-bit) with a 2-bit
  exponent slice. Barrels are explicit slice-select cases.
- **11 EBR**: circle table, cx/cy lookups, sine, 3 palette, 4 delay lines
  (x, a coordinate, fringe/disp/flags bundle, shade/flags bundle) — delay
  FFs were ~800 LC in v0.1.
- Generated avid (cubist recipe, verbatim from aurora).
- Palette authored at 0.8125 intensity (white Y ≈ 775) in the HW-verified
  convention (u register = BT.601 Cr) — sim previews show mirrored hues.

## Controls

| Control | Function |
|---|---|
| K1 Bands | band width: one palette cycle over 264 .. 4096 px (mantissa/exponent, monotonic) |
| K2 Wave | sideways bend amplitude, 0 .. W/2 (slewed) |
| K3 Refract | magnification inside the drops: 1× .. 64× |
| K4 Rim | rim thickness, 2 px .. R/4 |
| K5 Drift | orbit speed (never still) |
| K6 Flow | wave phase + band scroll, bipolar, centre still (square law) |
| S7 Ground | Pastel / Dark |
| S8 Drops | Pairs (small drop hangs 0.875R under each big one) / Solo |
| S9 Bands | Vertical / Horizontal |
| S10 Backdrop | Field / Mint (flat pale cyan; deep teal when Dark) |
| S11 Colour | Rainbow / Chrome |
| P12 Swell | drop radius W/32 .. W/2 (slewed) |

## Fit history (standalone yosys, program only: LUT4 / carry / FF)

| version | LUT4 | carry | FF | placed LC (with core) |
|---|---|---|---|---|
| v0.1 (10 circles, parallel FF arrays, FF delay chains) | — | — | — | 13336 (173%) |
| v0.2 (table EBR, line FSM, union rims, Lissajous) | 5384 | 1830 | 3051 | 8280 (107%) |
| v0.2 + narrower mults, cx/cy EBR, EBR delays | 4701 | 1730 | 2722 | — |
| + 2 slews, slice barrels, mantissa bands | 4574 | 1560 | 2626 | — |
| v0.3.0 flags + priority owner (no tree) | 4307 | 1628 | 2708 | 7358 (95%), HD Analog 63.6 MHz |
| v0.3.1 owner m from live DDAs, EBR sync/x delays, split sequencer cones | 4478 | 1626 | 2482 | 7228 (94%), HD Analog ≤69.6 MHz |
| v0.3.2 handshaked serial multiplier, square run flag, no vertical lens | 4265 | 1558 | 2440 | 7030–7046 (92%), see Status |

Ablation deltas (LUT4) on the 4701 build: tree 2 of 3 levels −398,
three barrels −258, lens mults −245, wave mults −223, five slews −205
(−134 carry), band mult −128, four DDAs −140 (−142 carry), band-exponent
case with per-arm adders −84, serial-multiply adder −15. Shift-add
replacements for serial products COST +100: the shared multiplier is
nearly free, its operand mux is what counts.

## Timing history

- v0.3.0: critical path = the sequencer's orbit-position store (f_fr8 ×
  3 adders + add + clamp + mux in one state, 15.7 ns; 4.4 logic / 11.3
  routing at 95% LC). Split per op.
- v0.3.1: critical path = the sequencer's control cone (state decode →
  per-bit multiplier enables → a constant's enable, 3.7 logic / 11 ns
  routing). Fixed by giving the serial multiplier its own process with a
  start/done handshake (the state register no longer drives the datapath)
  and the line FSM's square its own run flag.
- v0.3.2: HD Analog 75.3 (seed 1), HD Dual 76.2 (seed 6), HD HDMI best
  69.6 on router2 seeds 1–6; spread: router2 s7 75.0, s8 63.1, s9 68.7;
  router1 s1 80.3, s2 72.6 → shipped router1 seed 1.

## Build

`./build_programs.sh ron dewdrop` → `out/rev_b/ron/dewdrop.vmprog`, but
HD HDMI needs the hand-installed router1 seed 1 afterwards:
```
B=build/programs/ron/dewdrop/rev_b; rm -f $B/bitstreams/hd_hdmi.{asc,bin}
make -C videomancer-sdk/fpga VIDEOMANCER_SDK_ROOT=$PWD/videomancer-sdk PROJECT_ROOT=$PWD/programs/ron/dewdrop/ \
  BUILD_ROOT=$PWD/$B/ PROGRAM=dewdrop CONFIG=hd_hdmi DEVICE=hx4k PACKAGE=tq144 FREQUENCY=74.25 \
  HARDWARE=rev_b CORE=yuv444_30b PLATFORM=ice40 SEED=1 "ROUTER=--router router1"
python3 videomancer-sdk/tools/toml-converter/toml_to_config_binary.py programs/ron/dewdrop/dewdrop.toml $B/program_config.bin
python3 videomancer-sdk/tools/vmprog-packer/vmprog_pack.py --no-sign --hardware rev_b --toml-path programs/ron/dewdrop/dewdrop.toml $B out/rev_b/ron/dewdrop.vmprog
```
`.dewdrop_work/prof.sh <seed> <tag>` runs an independent nextpnr profile on
the HD Analog netlist with a kept log; `tools/critpath.sh <log>` summarises
its critical path.
`tools/gen_tables.py` (palette + sine) → `tools/tables.vhd.txt`; the .vhd
is `tools/part1.vhd + tables + tools/part3.vhd` concatenated.
`tools/synth_standalone.sh [file] [tag]` = 15 s program-only size check;
`tools/attrib.py` is a (failed) source-line histogram — the GHDL plugin
keeps no `src` on cells, use ablation copies instead.

## History

- v0.1.0 (2026-10-16): first cut, 10 circles, overflowed 173%.
- v0.2.0: circle table EBR + line FSM, 8 circles, union rims — 107%.
- v0.3.0: tree removed, EBR delay lines, narrower multipliers — 95%, 63.6 MHz.
- v0.3.1: owner m from the live accumulators (no copies), syncs/x in EBR
  delay lines, deep sequencer states split — 94%, ≤69.6 MHz (control cone).
- v0.3.2: multiplier handshake, square run flag, horizontal-only lens —
  92%; HD Analog and HD Dual close, HD HDMI 69.6.
