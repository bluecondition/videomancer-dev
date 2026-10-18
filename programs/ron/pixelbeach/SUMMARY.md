# PIXEL BEACH (v3.24.0 — "Shorebreak": side-on 8-bit beach on real wave physics)

**STATUS: FINAL except presets — signed off by Ron 2026-10-10** ("this looks
good", "let's not wait for presets"). Ship build = v3.24.0 with the
hand-picked HD seeds below. Every feature of the current look has been
HW-confirmed by Ron. Remaining: no presets yet. Capture them from the device
with `./capture_preset.sh --preset N --name "Name"`, then rebuild and repack
with the same HD seeds.

A side-on cross-section of a beach. Sky, sun or moon, gulls and stars above;
a flat ocean strip at the horizon; the water in the middle is a real 1D
shallow-water simulation, so waves steepen over the shoaling bottom, crash,
foam, run up and drain back. Every 3.4 s a wash sheet comes down over the
sand crest, and the sand stays dark where it touched and dries back. A red
crab (S11) wanders the sand and runs from the wash. Every visible cell is one
solid colour (16 or 8 px), with no dithering.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Rhythm | Tempo of the waves and of the slow surge cycle |
| K2 | Tide | Water level ±4 rows; walks the shoreline up and down the sand, and widens/narrows the wash |
| K3 | Foam | Foam injection strength (crest, whitecaps, spray, droplets) |
| K4 | Chop | Secondary short wave + per-column crest jitter |
| K5 | Swell | Sea energy: wave height and surge depth; also deepens the wash |
| K6 | Time | Physics speed: 1–4 physics ticks per field |
| S7 | Gulls | Three low-res gulls (with reflections); they roost when it's dark |
| S8 | Pixels | 16 px / 8 px cells |
| S9 | Storm | +50% wave energy, hotter foam |
| S10 | Sky | Day / Night (moon, night palette, full star field) |
| S11 | Crab | On: the crab wanders the sand. Off: it sinks into a 1-row hole. On again: a hole opens somewhere new and it climbs out top row first. Holes fade like wet sand. |
| P12 | Sun | Sun (or moon) height; in Day the palette follows it Day → Golden → Sunset, the sky fades to dark, and stars come out once the sun has set |

Automatic (no control): stars appear as the sun sets (geometry-relative, so
the same at every resolution and pixel size); wet sand darkens the moment the
wash leaves a cell and dries over about 3 s.

## Architecture (see the header of pixelbeach.vhd)

- **Physics (`p_fsm`)**: 1D shallow-water per cell column: eta, u, foam in
  3 BRAMs; a V pass (gravity, friction) and a conservative F pass (flux,
  diffusion, foam advection), forced swell at the right edge. A large FSM on
  a half-rate clock enable, `fsm_en` (registered: `not fsm_ce and not m_stall`),
  with `fsm_encoding none` (yosys once mis-synthesized it). It runs 1–4 ticks
  per field after vsync.
- **Serial multiplier (`p_mult`)**: a Robertson shift-add, 13-bit add per
  clock, 10 clocks per product. It replaced a 12×10 LUT-array multiplier
  (294 LUTs). It starts when ma/mb change; the FSM stalls while
  `m_stall = (ma,mb) /= (op_a,op_b)`; op_a/op_b are released only on an edge
  right after an FSM CE edge, so the stall has the FSM's 2-clock budget.
- **Display**: live state is copied into display snapshot banks after each
  vsync. `p_disp` prefetches the next cell mid-cell (eta, foam, wetness, star
  hash, wash edge, crab-hole column). The cell-boundary update (`pr_cell`,
  a registered strobe) is the FIRST branch of its if-chain. As the last arm
  of the `x_loc` chain, its ~200-register fan-out waited on the x_loc
  compares (22–28 ns).
- **Wash**: a global animated reach (wph phase), per-column sine edge, capped
  near the end: 16px tapers over ~4 columns, 8px over ~18 (a long soft tail
  to w_edge+15).
- **Wet sand**: `sw_mem` 4096×4 (column × 16 rows under the beach line). It
  is set to 15 while the wash covers the cell and drops 1 every 12 fields,
  shaded SANDW/SANDM/SAND2.
- **Stars**: per-cell S-box hash (sbox → +rotl7 → sbox → +rotl11; plain ARX
  hashes streaked). Density 0–48/4096, rising as the sun sets.
- **Crab**: behaviour in p_vseq steps 7–15 (WALK/PAUSE/BURROW/HIDDEN/EMERGE/OFF,
  free-running LFSR). The sprite is plain logic (as a case table yosys
  inferred a ROM in an EBR). Its hole is its own 1-row overlay.
- **Palette**: 4 time-of-day palettes × 32 slots (5-bit index), stored U/V
  swapped for hardware. `genpal.py` is the source of truth. `make_reference.py`
  renders `color_reference.png` (168 numbered swatches; #153–160 stars,
  #161–168 crab).

Resources: 7458–7496 / 7680 LC (97.2%), 25/32 EBR.

## Resolution scaling (v3.24.2, 2026-10-18)

- Before: cells were a fixed 16/8 PIXELS and rows 16/8 field LINES (halved
  only when field_n toggled, which never happens on hardware), so 720p cells
  and sun were 1.5x too big and SD 2x wide / 4x tall (2:1), with only ~16 rows.
- Now the lattice steps through virtual 1080-line units: `x_a` (Q.1, step
  1 / 1.5 / 2 per pixel) decides the cell boundary while `x_loc` still counts
  CLOCKS into the cell for the prefetch sequence; `y_a` (Q.8 per field line)
  decides the row boundary; `pr_cell` = "next pixel is the cell's last"
  (precomputed thresholds `s_cl1 = csz - xs`, `s_cl2 = csz - 2xs`).  Every
  cell-based layout (n_rows, sun, gulls, beach, crab) follows.
- 16 px mode: exact at every resolution (720p cells 10-11 px, SD 8 px x
  3.6 field lines, square on 4:3).
- 8 px mode: the display prefetch needs >= 8 clocks per cell, so it is
  clamped to 8 px: 720p 12 units (1.5x the 1080p 8 px look), SD 16 units
  (= the 16 px look; S8 then only changes the beach slope / wash tail at SD).
- field_n interlace detect removed (the interlaced y steps replace it; 1080i
  rows were 2x tall on hardware before).  1080p bit-identical (0 px).

## Build

### v3.24.2 (2026-10-18, resolution scaling) — packed, not flashed

6833–6866 LC (89%), 16 EBR (dev SDK).  SD from the builder
(`NO_TIMING_RETRY=1`, router2 seed 1): SD Analog 67.52, SD HDMI 65.96, SD Dual
70.37 MHz (Fmin 27).  HD hand-picked from router2 screens (`--detailed-timing-report`,
`tools/hdmi_pin_skew.py`, `icetime -c 37.125`; hd_hdmi 1–14, hd_analog 1–32, hd_dual
1–14), copied into `build/programs/ron/pixelbeach/rev_b/bitstreams/` and repacked:

| Config | Router / seed | Fmax (MHz) | HDMI hs-clk / data (ns) | analog hs-clk / data | icetime @37.125 |
|---|---|---|---|---|---|
| HD HDMI | router2 **seed 13** | 63.08 | +1.49 / −1.99..−0.62 | +0.23 / −0.98..+0.63 | PASS 15.19 ns |
| HD Analog | router2 **seed 28** | 62.67 | +1.50 / −2.13..−0.74 | −1.57 / −2.94..−1.05 | PASS 15.10 ns |
| HD Dual | router2 **seed 12** | 70.72 | −2.28 / −2.96..−1.21 | +3.01 / +0.00..+1.13 | PASS 14.08 ns |

All three worst paths are FSM (2-clock) paths.  Rejected on the way: hd_hdmi s1/s5
(worst path a 1-clock DISPLAY path x_idx -> surf_q at 70.8/66.4 MHz), hd_analog
1–23 (no seed had HSYNC in +1.0..+2.0 with every data bit <= -0.6).  The screens ran
6 in parallel on a loaded machine, so a re-route may not reproduce byte-exact: the
picked .asc/.bin/reports are kept in `.scaling_work/pixelbeach_tmp/picked/`.

**Do not trust a plain `./build_programs.sh ron pixelbeach` for HD.** It
installs seed-1 HD bitstreams, which here are slow and unscreened. The
shipped HD bitstreams were routed solo and picked by hand:

| Config | Router / seed | Fmax (MHz) | HDMI hs-clk / data (ns) | icetime @37.125 |
|---|---|---|---|---|
| HD HDMI | router2 **seed 16** | 53.86 | +1.59 / −3.01..−0.14 | PASS 16.81 ns |
| HD Analog | router2 **seed 6** | 56.11 | +0.91 / −2.43..−0.22 | PASS 17.37 ns |
| HD Dual | router2 **seed 8** | 56.83 | −2.23 / −2.60..−0.54 | PASS 16.96 ns |
| SD Analog | router2 seed 1 (builder) | 43.53 | | |
| SD HDMI | router2 seed 1 (builder) | 35.65 | | |
| SD Dual | router2 seed 1 (builder) | 33.87 | | |

To rebuild (unloaded nextpnr reproduces a seed byte-exact):
1. `NO_TIMING_RETRY=1 ./build_programs.sh ron pixelbeach` (SD bitstreams + config binary).
2. Per HD config:
   `make -C videomancer-sdk/fpga PROGRAM=pixelbeach PROJECT_ROOT=$PWD/programs/ron/pixelbeach BUILD_ROOT=<dir> CONFIG=<cfg> <dir>/bitstreams/<cfg>.json`,
   then `nextpnr-ice40 --hx4k --package tq144 --pcf videomancer-sdk/fpga/hardware/rev_b/constraints/pin_map.pcf --freq 74.25 --json <json> --asc <asc> --seed <N> --router router2 --timing-allow-fail --report <r.json> --detailed-timing-report`,
   then `icepack`.
3. Copy the three HD `.bin` files into `build/programs/ron/pixelbeach/rev_b/bitstreams/` and repack:
   `videomancer-sdk/tools/vmprog-packer/vmprog_pack.py --no-sign --hardware rev_b --toml-path programs/ron/pixelbeach/pixelbeach.toml build/programs/ron/pixelbeach/rev_b out/rev_b/ron/pixelbeach.vmprog`
   (check "Found bitstream" ×6).

**After any RTL change, re-screen HD seeds** (typically 8–24 per config):
- **(a)** the reported worst path is in the FSM, and its source/sink are
  FSM registers;
- **(b)** Fmax ≥ ~54 MHz (worst ≤ ~18.6 ns, like the HW-clean
  3.17–3.19 builds; never below 37.1 MHz = the 2-clock FSM budget);
- **(c)** `tools/hdmi_pin_skew.py`: HD HDMI/Analog hs-clk ≈ +1.0..+2.0 with
  all data bits before the edge (≤ −0.3). HD Dual sits at −1.5..−3. A +0.48
  build gave thin brown per-line U/V-swap lines on HW (3.17.1);
- **(d)** `icetime -c 37.125` PASS.

**Timing notes.** HD never meets full-rate STA: the FSM is a deliberate
2-clock path. nextpnr reports only the single worst path, and its detailed
report covers only ~40% of nets, so hidden 1-clock paths are bounded only by
that number. The known structural ones are `fsm_en` → FSM CENs through the
state decode (no global buffer, 8/8 used) and `ma/mb` → `m_stall` →
`fsm_en`/`m_run`. At 97% LC the paths are routing-dominated (seed spread
26–60 MHz). If HD ever shows sparkles or flicker, free LC first. Next lever:
the sky-colour engine (sga/sgb/ga/gb and the lerp outputs, ~300 FF computed
once a frame).

## History (this finalization round; earlier v3.x in Claude memory)

- **3.17.0**: `no_rw_check` on all RAMs (−95 LUT −214 FF), dead S11 Bypass
  removed (synthesis programs get no input video). LC 99.7% → 93%.
- **3.17.1**: 1080 8px Tide-edge triangle fixed (the wash cap
  `(w_edge−x)<<7` wrapped at 240 columns). HW-confirmed.
- **3.17.2**: the thin brown lines were HDMI TX pin skew (HSYNC +0.48 ns from
  the forwarded clock). Re-seeded with the new pin-skew tool. HW-confirmed.
- **3.18.0**: per-cell wet-sand RAM (the old band edge darkened by position,
  vanished at each new wash, and slid with the sine). Wash-edge cone split.
  HW: "looks perfect".
- **3.19.0**: twinkling stars. HW: "look fine".
- **3.20 → 3.24**: the S11 crab.
  - Walking the beach line was rejected → it now walks the sand area and
    flees the wash.
  - Burrow fix: a 2-bit `2 − sink` wrapped at sink 3, so its whole body
    reappeared one row lower.
  - 1-row hole; whole-sand roaming.
  - The serial multiplier, to fit.
  - 8→16px position rescale.
  - The pop-up check mixed the new candidate's depth with the old
    candidate's ground (~9% of 16px pop-ups landed off-screen).
  - The promote-first chain (−90 LC).
  - HW: "looks good".

## Community port (stock LZX SDK 0a65d53), 2026-10-14

LZX's stock build (router1, one seed for all 8 configs, 8 configs including HD/SD
Standalone) overflowed: SD Standalone needed 8250 of 7680 LC, because LZX's core
is ~600 LC bigger in that config. The program itself was the same size in every
config. Three look-identical optimizations, each proven bit-exact in GHDL against
v3.24 (`~/czartroniks_port/pb/eq/run_equiv.sh`: old and new side by side, 3.2M
clocks over 40 frames with a P12 sweep, Night, Storm, 8/16 px and crab off/on;
syncs every clock, colour while avid):

1. **One read site per sine/sun ROM.** Each extra read site of a constant array
   is its own copy of the table. (-37..-46 LC)
2. **Serial sunset engine on one ROM port.** SKYG_ROM had six read sites, so six
   copies (12 EBRs!) plus nine parallel lerps. Now SKY_CH (SKYG_ROM split per
   channel at elaboration, FSM sine in its unused corner) has one port, and one
   lerp runs a channel at a time; band RAMs are per channel. (-600 LC, EBR 25 -> 16)
3. **Bit-test row compares.** P1 thresholds are whole rows, so they are bit tests
   on one subtract per boundary; the sky line shares the streak's subtract; the
   mirror/streak flags come from st_d. (-64..-126 LC)

Result (packed LC): SD Standalone 8250 -> 7525 (98%), HD 7625 -> 6855-7000
(89-91%). Split-domain timing (FSM on its own clock net, scratch core without
the Sync Out generator): HD display paths 89.7-90.5 MHz (were 82.8-85.5), FSM
74-81 MHz (budget ~37).

Measured feature costs (scratch ablations, for future trims): crab 619, gulls 216,
wet sand 185, sun reflection/streak 170, stars 130, droplets+spray 76; 8px-only
saves 2, 16px-only 68.

**Build seeds -- community port:** `[firmware] seed = 6` (one seed, all 8
configs). Reason: HDMI pin skew, not timing. tools/hdmi_pin_skew.py over seeds
1-16 on this netlist: seed 1 HD HDMI hs-clk +0.50 (the +0.48 brown-line zone);
seed 6: HD HDMI -1.84 (data -5.36..-3.33), HD Analog HDMI +0.71 / analog -1.87,
HD Dual HDMI -2.28 / analog +1.75. No seed in 1-16 was clear on every pin; +0.71
on HD Analog's HDMI pin is untested on HW (known good +0.91, known bad +0.48).
Any RTL or SDK change reshuffles this: re-screen. Processing delay: declared 0
(synthesis); true latency 5 clocks.
