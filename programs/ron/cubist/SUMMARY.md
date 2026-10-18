# Cubist — dev notes

## Resolution scaling (v0.5.6, 2026-10-18)
- Interlace came from `data_in.field_n` toggling, which core_top ties high:
  on hardware (and in the field_n-high sims) SD was drawn in 243 field
  lines, so the cube was twice as tall as wide. Now `s_ilace` comes from the
  timing ID (`p_tid`: 0 / 8 / 1 / 2 / 10). `s_field` still latches field_n
  (always '1' on hardware, so every field starts on frame line 1: a one-row
  shift, harmless).
- SD pixel aspect: microcode CTL 15 (`s_xa`, Q12 offset from 1.0: 0 on HD,
  +455 = x1.111 on 480 lines, +512 on 480p, -256 = x0.9375 on 576) scales
  the zoom for the projected x axes only (`T11 = Z + Z*xa>>12`, used by the
  box-1 PX multiply; box 0 and all gradients derive from it). 4 words
  (934/1024), inserted after the layer-turn section, so the `CTL rand` draws
  keep their engine timing. Regenerate with `python3 regen_ucode.py`.
- 720p "turn phase differs from 1080p" is not geometry: the turn picker
  reads a free-running LFSR (`s_lfsr`, stepped every clock) at a point in
  the vblank engine, so the drawn face/direction depend on the clocks per
  field of each mode. Orientation glide, auto-spin and turn speed are per
  field. The sims also start the zoom glide from `s_hf`'s power-up 480, so
  after a few frames 1080p is further from its target than 720p/SD (a
  startup transient). A sim-only copy with the LFSR stepped once per engine
  start and the zoom glide bypassed (.scaling_work/cubist_tmp/detzb, sheet
  cubist_dz) renders the same cube at 1080p / 720p / SD (r720 0.99,
  rSD 0.96).
- 1080p: the steady-state picture is bit-identical; the 7-frame sim differs
  from the old render only through a STARTUP glitch the old detector had:
  `s_fpar` powered up '0' against field_n '1', so the first field after load
  was taken as interlaced (hf = 2160), which the zoom glide carried for ~2 s.
  A sim-only copy of the old source with `s_fpar := '1'` renders 0
  differing pixels against v0.5.6 at 1080p (7 frames).
- Build (router1, seed 1 on all 6): HD Analog 77.3 / HD HDMI 79.7 /
  HD Dual 81.6, SD 80.1 / 79.4 / 79.3 MHz; 6533-6566/7680 LC (85 %),
  27/32 EBR. Not flashed. (A first build had `s_xa` 10 bits wide, so
  `to_signed(512, 10)` wrapped the 480p entry to -512; widened to 11.)

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
- **HW-validated 2026-10-15** (Ron: "it looks the same"). Originally flagged for checking: whole picture, all modes (no_rw_check on all 8 RAMs relies on the engine finishing inside vertical blanking; new unlit-albedo ROM).
