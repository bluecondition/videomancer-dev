# Cubist

A glossy Rubik's cube product shot that turns its own layers at random, with live-video stickers or backdrop, auto-spin on each axis and a turn-speed slider.

**P12 (Turn Speed):** How fast the random layer turns happen, from slow, eased quarter-turns to rapid-fire.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Yaw | 0–360deg | Turns the whole cube left/right (yaw angle) |
| K2 | Pitch | 0–360deg | Tilts the cube forward/back (pitch angle) |
| K3 | Roll | 0–360deg | Rolls the cube clockwise/anticlockwise |
| K5 | Video | Off / White / Yellow / Red / Orange / Green / Blue / All | Video stickers: off, one sticker colour, or all stickers show the live video |
| K6 | Size | 50–90% | Cube size on screen, 50–90% |
| S7 | Run | Solved / On | Solved: cube holds still and solved; On: layers turn at random |
| S8 | Auto Yaw | Off / On | On: cube spins continuously around yaw |
| S9 | Auto Pitch | Off / On | On: cube spins continuously around pitch |
| S10 | Auto Roll | Off / On | On: cube spins continuously around roll |
| S11 | Background | Grey / Video | Backdrop: grey sweep or the live video |
| P12 | Turn Speed | 0–100% | How fast the layers turn, from slow to rapid-fire |

## Presets

Hero, Corner.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.

## Community port — fit for the stock SDK (v0.5.5, 2026-10-15)

- The stock LZX SDK builds 8 configs; its SD Standalone core is ~600 LC larger
  than HD. The staged v0.5.4 packed to 8082/7680 LC there (SD Dual 7609,
  HD Dual 7585), so it did not fit SD Standalone. Its `x6_dlin` adder also
  put one net on two inputs of an LC (`x5_c19[17]`), which livelocks router1.
- Eight bit-exact changes (packed LC, SD Standalone, stock SDK, after each step):
  1. **Booth multipliers** (the earlier `pb/cand/cubist` candidate). The
     per-pixel products `x6_p0/p1/p2`, `x10_py`, `x14_pn` use the radix-4
     Booth `bmul`; `x12_pm` uses `bmul_u`. 8082 → 7925 (−157).
  2. **`no_rw_check` on all eight RAMs** (`rf_a`, `rf_b`, `wram`, `sd_h1`,
     `sd_h2`, `sd_lf`, `sd_fd`, `tbl`). yosys no longer emulates
     read-before-write on them (`emulate_transparency` count 0).
     7925 → 7618 (−307). Collision review below.
  3. **One adder in the engine's serial Booth step.** `mu_ph` was
     `ph + {0, ±A, ±2A}`, built as four adders and a 5:1 mux. Now the digit
     selects 0 / A / 2A, a negative digit inverts it, and the +1 enters as a
     carry-in (`(ph & '1') + (x & neg)`). Still 8 steps, same cycle timing.
     Checked in Python for all 8 digits (1.6 M random and edge operands).
     7618 → 7488 (−130).
  4. **Unlit-albedo ROM.** Stage J/K's `clamp10(173 + (dly(col) * ast) >> 4)`
     (the `x8_dly` case, the `x10_py` multiply and the `x11_y` add/clamp) is
     now `C_YROM`, a 256 x 10 block-RAM table addressed by `x9_ca & x9_ast`.
     It is read at stage J and registered into `x11_y` at stage K, so stages
     and latency are unchanged. +1 EBR (27; 28 on the Dual configs).
     7488 → 7364 (−124).
  5. **`f_two6` as plain logic.** "Two or more of the six strict claimants"
     was an integer popcount, which became a carry chain on stage A's critical
     path. Now it is "two in one half, or one in each half" (checked for all
     64 inputs). 7364 → 7349 (−15; HD Dual −30).
  6. **One adder in the engine ALU.** `SUB` was `a + b + 1` next to `a + b`:
     an adder, an incrementer and a mux. Now it is one 33-bit add with `u_cin`
     as the carry-in. This was the HD critical path (`u_wd` → ALU → `u_wd`).
     7349 → 7292 (−57).
  7. **`x6_dlin` without duplicate adder inputs (router1 livelock fix).**
     `x5_c19` always fits 18 bits (a 15-bit constant less two 16-bit squares),
     so yosys merges bits 19..17 into one net. Two sign-extended shifts of it
     then fed that net to both inputs of an adder bit. With S = c(17), the
     18-bit `c >> j` is `c(16..j) - S*2^(17-j)`, and the `-S*2^k` terms fold
     into free high bits of the narrower field:
     `(c >> 7) + (c >> 8) = c(16..7) + (S & '0' & S & c(16..8))`, exact mod
     2^12 (checked in Python for all 2^18 values of c). Still one adder.
     `tools/dupin.py` now reports 0 duplicate CARRY and 0 duplicate-input LUTs
     in all 8 configs.
  8. **Dead `x6_dlin` branch removed.** Outside the sticker zone `x6_dlin`
     was `c>>6 + c>>8 + c>>11`. Stage G reads `x6_dlin` only when
     `x6_stz = '1'`, which is registered on the same edge from the same
     `x5_stz`, so that value was never used. Steps 7 and 8 together:
     7292 → 7270 (−22).
- Tried and dropped: Booth rows with each row's +1 folded into the next row's
  free low bits (`bmul2`). It was exact (exhaustive Python check) but packed
  16 LC larger.
- Packed LC (stock SDK, `cand_try`): SD Standalone 8082 → 7270,
  SD Dual 7609 → 6811, HD Dual 7585 → 6754. EBR 27/32 (28 on the Dual configs).
- Routing (`cand_try.sh`, stock router1, all 8 configs on one seed). Seeds 1–5
  all built 8/8. Own HD (analog / hdmi / dual):

  | Seed | Analog | HDMI | Dual |
  |---|---|---|---|
  | 1 | 79.11 | 81.29 | 83.54 |
  | 2 | 79.41 | 86.74 | 81.26 |
  | 3 | 76.10 | 81.87 | 85.14 |
  | 4 | 78.59 | 77.24 | 82.79 |
  | 5 | 81.93 | 86.74 | 80.03 |

  Before change 7, `cubist_g` livelocked router1 on sd_hdmi at seed 1.

### no_rw_check collision review (simulation can't check this)
- `rf_a` / `rf_b` (register file): every write lands on a DECODE edge. EXEC
  or LDX's state 5 sets `u_we`, the next state is always DECODE, and jumps
  write nothing. The engine's own forwarding (`u_wlast` and source = `u_wa`
  → `u_wd`) replaces the DECODE read that collides. LDX's indexed read
  (states 3/4) never coincides with a write.
- `tbl`: SWR's write lands one clock after its EXEC. A following SRD latches
  its address two clocks later, so a used engine read never collides. Pixel
  reads happen only with `u_done = '1'`, when nothing writes.
- `wram`: SLW writes happen only while the engine runs (vertical blanking).
  The DDA sequencer reads only after `dd_wrap` (active lines) or after
  `dd_kick`, which is issued after the last SLW 0..5.
- `sd_h1/h2/lf/fd`: SLW writes happen only while the engine runs. Pixel-pipe
  reads during blanking only feed pipeline registers, which are flushed long
  before active video.
- Every write enable is a one-clock pulse (each defaults to '0' every clock).
- These cases assume the engine finishes inside vertical blanking. That holds
  on hardware: the dev notes give ~14.8k clocks worst case from engine start,
  against ~16.6k for NTSC and ~99k for HD. In the 403 x 201 sim raster the
  engine overruns into active video, the same way in the reference and the
  candidate.
- Poison check: `tools/poison.py` makes a sim-only copy whose 8 RAMs return
  old data XOR a pattern on every same-edge read/write collision. Run against
  the reference on the wide raster (1003 x 201, 16 frames), it gave 0
  mismatches.

### Proof (GHDL side-by-side against staged v0.5.4, default stimulus)
- Wide raster `-gH_TOTAL=1003 -gV_TOTAL=201`, 128 frames:
  `EQUIV_RESULT compared=25805184 mismatches=0`.
- 403 x 201 raster, 128 frames: `EQUIV_RESULT compared=10368384 mismatches=0`.
- 128 frames walk all 32 switch combinations, including S11.
- Mutation tests (wide raster), each caught:

  | Mutant | Changed logic | Frames | Mismatches |
  |---|---|---|---|
  | jm1 | one `C_YROM` entry +1 | 4 | 15 |
  | jm2 | Booth step: −2A digit decoded as 0 | 4 | 59437 |
  | jm3 | `f_two6` without the cross-half term | 10 | 1253 |
  | jm4 | ALU carry-in forced to 0 | 4 | 37889 |
  | m6 | `x6_dlin` first operand `c >> 8` | 8 | 361 |

- An earlier `x6_dlin` mutant, on the sign bit of the corner-outside region,
  was not caught in 4 frames. That region is under a pixel at the sim raster.
  The exhaustive Python check (`tools/dlin_check2.py`) is the real proof of
  that rewrite.

- **Stock build:** seed 1 builds all 8 configs (seeds 2-5 also). Own HD 79.1 / 81.3 / 83.5 MHz (analog / HDMI / dual).
- **regen_ucode.py** leaves this cubist.vhd unchanged (checked), so the microcode/table regeneration still works.
- **HW check wanted:** whole picture, all modes (no_rw_check on all 8 RAMs relies on the engine finishing inside vertical blanking; new unlit-albedo ROM).
