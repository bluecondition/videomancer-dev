# Triangulator — triangle pixelation (v1.0.0)

**STATUS: v1.0.0 FINAL — signed off by Ron 2026-10-11, HW-validated** ("it
looks good now, let's finalize with this version"). v1.0.0 is the v0.2.0 RTL
and bitstreams (the screened HD seeds below) repacked with the version bump;
the bitstream files are md5-identical to the build Ron approved. Defaults are
HW-captured. The 4 presets are still hand-remapped placeholders, so capture
real ones with `./capture_preset.sh --preset N --name ...` and repack (do NOT
run `./build_programs.sh`: it would revert the HD seeds to seed 1).

The picture is rebuilt from a lattice of flat-coloured upright and inverted
~equilateral triangles (cell Wc wide x Wc*7/4 tall, one diagonal per cell,
orientation flipped per column and per row). Each triangle's colour is an 8-px
average of the source taken on the line just above its row, held down the row.
There's no frame buffer and no per-pixel multiply in the geometry.

## Controls (v0.2)

| Control | Function |
|---|---|
| K1 Grid Colour | Black / White / Red / Orange / Yellow / Green / Blue / Violet (scope-verified ROYGBIV values) |
| K2 Contrast | Fill contrast about mid, always on (50% = unity) |
| K3 Luma Level | S8 off: darker fills are lifted to this level. S8 on: triangles below it are hidden. 0 = no-op |
| K4 Colorize | Original / Duotone / Posterize / Hue Spin |
| K5 Colorize Param | Original: saturation (50% = bit-exact unity). Duotone: colour pair (4 zones). Posterize: rainbow shift (7 zones) + grey (top zone). Hue Spin: 20..340 deg, never identity |
| K6 Line Width | 1 / 2 / 3 / 4 px |
| S7 Grid Lines | Triangle edges in K1 colour |
| S8 Luma Gate | Show only triangles at or above K3 |
| S9 Fill Mode | Faceted (per half-cell) / Flat (one colour per triangle) |
| S10 Wireframe | Edges only. With the gate off, edges take the live video colour |
| S11 Bypass | 1-cycle raw passthrough (identical to SDK passthru) |
| P12 Triangle Size | 8..256 px, with hysteresis |

## v0.2.0 (2026-10-10) — finalization pass

**Hardware contracts:**
- **Blanking gate.** Output is 64/512/512 whenever the aligned avid is low.
  v0.1 drove effect pixels through blanking.
- **No raw video carried deep.** The P12 dry/wet Mix is gone, so there's no
  dry path. Bypass is the SDK 1-cycle passthrough (rollthebones v0.6 fix), and
  wireframe live-video edges read a short tap (`pipe(3)`, 8 px ahead).
  v0.1 carried raw video 10-12 FFs deep, the HD-HDMI green trap.
- **Size hysteresis.** The lattice is anchored at the left edge, so one step of
  cell width moves column k by k px. Pot noise could shake the right side of
  the frame by up to ~240 px. P12 now only moves when it changes by 4 or more
  counts (10-bit).

**Sampling:**
- **8-px area average** per triangle (the C64/rollthebones denoise) replaces
  the single-pixel point sample, which flickers with analog noise.
- **Flat mode samples on the triangle axis** (the cell's right edge). v0.1
  sampled the cell centre, which for ▲ triangles sat inside the neighbouring
  ▽, half a cell left of the ▲.
- **Row-parity banks** (rollthebones scheme). The last line of each row samples
  the next row into the other bank, so a row never reads the bank being
  written. v0.1 showed a 1-px seam of the previous row's colours on every
  row's top line with the grid off. The frame's first active line (row 0's
  sample line) is drawn black. The buffer is one 512 x 32 RAM (Y/U/V packed,
  padded) with `no_rw_check`.
- **Partial right-edge cell.** If the line ends before a cell's window closes,
  the last active pixel is written instead, so the right-edge strip is never
  stale.

**Controls:**
- P12 Mix dropped. **P12 = Triangle Size** (was K1).
- K2 Contrast is always on (it was gated by S8). K3 is a lift with the gate
  off and a threshold with it on (it was gate-only).
- K5 is split from the grid colour, which moved to K1. In v0.1 the duotone
  pair and the grid colour shared K5's top bits, Hue Spin's bottom quarter was
  the identity, and K5 did nothing in Original.
- Hue Spin is a continuous rotation (64 angles) through a 3-multiply complex
  product. Original mode uses the same datapath for saturation. Both reuse the
  three Duotone multipliers via a per-mode operand select.
- Black is 64 (it was 0 / 48).
- The presets were hand-remapped to the new layout and need re-capturing.

**Pipeline:** LATENCY 12 -> 11 (one latency in every mode except S11's
passthrough). The effect is now exactly aligned with the sync (v0.1 was 2 px
early). Every multiply has its own stage. The three colorize products split
operand B hi/lo across two stages (St4a/St4b): as a single-stage 11x12 multiply
it was the HD HDMI critical path (13.7 ns, 9.6 ns of it routing; seeds 1-2
missed).

## Build (v0.2.0, 2026-10-10, router2 via `./build_programs.sh ron triangulator`)

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 83.13 MHz | 4572 / 7680 (60%) |
| SD Analog | 1 | 86.41 MHz | 4563 |
| HD HDMI | 1 | 82.01 MHz | 4572 |
| SD HDMI | 1 | 82.39 MHz (Fmin 27) | 4581 |
| HD Dual | 1 | 80.93 MHz | 4585 |
| SD Dual | 1 | 81.02 MHz | 4579 |

(Rebuilt after the defaults capture. The build just before it needed seed 2
on HD Analog: router2 seed spread, not a real deficit.)

4 EBR (one 512x32 buffer); 0 `emulate_transparency` in yosys, so
`no_rw_check` took effect. v0.1 was ~4900 LC with HD Analog closing only on
seed 10 at 74.60 MHz.

## HD encoder pin-skew re-seed (2026-10-11)

Ron's first look at the seed-1 build: "colours weird, horizontal lines
flashing all over". `tools/hdmi_pin_skew.py` put two shipped HD placements in
the danger zone:

| Config | Seed 1 (shipped) | Re-seeded to |
|---|---|---|
| HD HDMI | HSYNC +1.19 ns, but closest data bit **+0.11 ns** | **seed 9**: HSYNC +1.34, data <= -0.72, 81.99 MHz |
| HD Analog | HDMI HSYNC **-0.08 ns** (on the clock edge) | **seed 20**: HDMI HSYNC -1.19, data <= -1.22; analog encoder also clean; 76.38 MHz |
| HD Dual | HSYNC -1.37, closest data bit -0.29 (marginal) | **seed 6**: HSYNC -2.13, data <= -1.11, 87.36 MHz |

Damier's "lines over everything" build had its closest data bit at -0.20 ns.
No HD Analog seed in 1-22 gave a positive HSYNC margin with clean data, so
seed 20 is the negative-side pick. These were hand-installed (icepack of
the screened .asc + `vmprog_pack.py --no-sign`). **`./build_programs.sh`
reverts them to seed 1**, so re-screen after any rebuild. SD configs stay on
seed 1. If the lines persist on the re-seeded build, the next suspect is the
drape row-parity swap: each row replays its sample line's U/V on rows whose
hsync->avid parity may differ.
**HW result (2026-10-11): the re-seeded build looks good.** The lines were
placement skew; no parity fix was needed.

## Defaults (HW-captured 2026-10-10)

Captured from the device with `./capture_preset.sh --defaults`: K2 Contrast
1023, K3 Luma Level 126, K5 1023 (Original = max saturation), K6 Bold, S9
Flat, P12 95 (~31 px). The rest are unchanged (K1 Black, K4 Original, S7 Grid
on). K6's default label is index 2, and the firmware loads labelled defaults
as the raw INDEX, so the K6 decode treats raw < 4 as the label index (the
edgelab fix).

## Watch on first flash

- Lattice alignment, grid thickness (H vs diagonal), and the U/V-swapped hues
  of the posterize/duotone palettes (not HW-verified; the grid palette is).
- Thin coloured grid lines on HD: per-line U/V swap lines (aurora; fix =
  generated avid) or one-axis chroma on 1-2 px strokes (revenant/bishop 4:2:2
  pair averaging).
- Triangles are ~2x too tall on interlaced sources (the row counter counts
  field lines). This is the SD/1080i follow-up.
