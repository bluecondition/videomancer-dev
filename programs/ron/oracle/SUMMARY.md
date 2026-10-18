# Oracle v2 — divinatory eye between two spectra

**STATUS: v2.0.1 FINAL except presets — flashed and signed off by Ron
2026-10-10** ("interesting enough"; "mark it as finalized and completed").
Full rewrite of v1 (the triptych composition kept, every layer redrawn at
1-px resolution). v2.0.1 adds a startup snap: the first three fields after
load set the knobs straight to their positions instead of gliding in from
the power-up value (the snap also exposed two intended 24-bit wraps and an
emulator MRD-model bug, all fixed in the tooling). Remaining: capture real
presets with `./capture_preset.sh` (the five in the TOML are placeholders).
Deliverable: `out/rev_b/ron/oracle.vmprog`.

**v2.0.4 (2026-10-18) — resolution scaling (VHDL only, microcode untouched).**
- Interlace now comes from the timing ID (`p_tid`: 0 / 8 / 1 / 2 / 10 =
  interlaced), not `s_timing.is_interlaced`. The SDK field detector keys on
  a half-line vsync offset; when it reports progressive (the sims, and
  hardware whenever the decoder's vsync has no offset) SD was laid out in
  243 FIELD rows, so the eye came out half as wide (the SD x-aspect term
  `hh + hh/8` was applied to field rows) and the seam teeth reached into the
  window rule ("dashed artifacts along the window frame"). `s_botf` still
  comes from the detector (inert when field_n is tied high: line-doubled).
- Torn-seam teeth (S7) follow the screen: tooth height through `f_tooth`
  (timing-ID class: 1080 x1, 720p x2/3, 480-line x0.45, 576-line x0.53,
  a 16-entry case per class) and one LFSR step per 8 VIRTUAL columns
  (`tx_acc` += 2 / 3 / 4 per pixel, step on the carry; at 1080 this is
  exactly `s_bcol(2:0) = "111"`).
- Tear-burst shifts (S7) in virtual columns: `p_tsc` scales each line's
  shift `d - D0` by 2/xs (x1 1080, x21/32 720p, x1/2 SD) where the output
  stage latches the line delay (`cu_ds`/`cu_dcs` -> `o_dY`/`o_dC`); the
  chroma offset `dc - d` (1..4 px) stays at pixel scale.
- 1080p bit-identical (diff1080). Not changed, because they live in the
  microcode (ROM 511/512 words): tear-burst band rows -- start `rand & 2047`
  and height 2..33 rows (the microcode comment says 2..129, but `rand` is
  the 16-bit LFSR, so `rand >> 11` has 5 bits) are decided inside LINE and
  the VHDL only sees the per-line delay; and the eye's SD x-aspect
  (`wmax = hh + hh/8` when mh < 1000: 1.125 for every SD mode; exact is
  1.111 NTSC, 0.9375 PAL, so PAL eyes are ~20 % too wide) feeds the eye
  width, the rho^2 DDA and the corona. The hh >= 32 floor only matters at
  the smallest K1.
- Build with the tear-shift scaling (router1, seed 1 on all 6): HD Analog
  80.5 / HD HDMI 77.4 / HD Dual 80.0, SD 76.5 / 81.8 / 75.6 MHz;
  6586-6605/7680 LC (86 %; `p_tsc` ~ +100 LC), 26/32 EBR. (The build
  without it needed SEED 2 on HD Dual.) Not flashed.
- Check: a sim-only copy (.scaling_work/oracle_tmp/burst: every re-roll a
  burst, start rows < 256, LFSR stepped per field) puts the same burst at
  SD / 720p / 1080p (sheet oracle_fb); its shift measures 40-44 / 43.5 /
  44 virtual columns (20-22 / 29 / 44 px). The band rows (78..82) stay in
  frame rows, as noted above.

## Build
**Build with router1:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=2400
./build_programs.sh ron oracle`. Router2 missed every HD config on all 6
seeds (best 67 / 48.5 / 56.4 MHz); router1 closes everything on seed 1
(v2.0.0 and v2.0.1 identical):
HD Analog 78.0, HD HDMI 76.6, HD Dual 76.6, SD 78.8 / 81.3 / 78.3 MHz,
7,262-7,319 LC (95%), 26/32 EBR. (Same story as cubist, whose engine this
is.)

## The picture

- **Top — Spectrograph.** Fine vertical bars, each one full spectrum (256-step
  palette from the scope-verified phosphor anchors, HSV-interpolated) with
  four soft dark absorption lines. Chirped: 2.5x denser at the edges than the
  centre. Rows ripple on two sines (+ per-row twinkle). S9 Moire lays a 1/32
  faster line screen over them: sliding colour beats.
- **Middle — the window.** Bevelled inset (bright rule, inner shadow top/left,
  light bottom/right), spectrum leaking through the gutter. Static: 1-px grain
  re-rolled every frame, horizontal streak runs (random length up to K4,
  brightness, faint sepia; even lengths so a run never splits a 4:2:2 pair),
  a drifting two-wave cloud setting the density, prismatic sparks (runs that
  show the bar spectrum behind), Newton's rings expanding from the eye.
- **Eye.** Per-line half-width from two profile ROMs (vesica, circle) mixed by
  Vision, so P12 opens a constant-height hairline slit -> almond -> circle.
  1-px AA edge whose ramp width follows the boundary slope (power of 2), white
  rim, rainbow corona falling off in elliptical rho^2, interior = bands bowed
  through a lens (S8: concentric iris rings moving inward + pupil).
- **Bottom — horizon.** Flat swaying bands, or (S10 Deep) a perspective floor
  receding into a glowing haze under the window, the eye reflected as a
  glinting column of light.
- **Tear (S7).** Random bursts (re-rolled every 4 fields) of rows displaced
  sideways, chroma a few px off the luma, plus jagged-tooth seams.

## Controls
K1 Size · K2 Gaze (slow Lissajous wander) · K3 Aura (corona width + ring
density) · K4 Static (grain -> long streaks) · K5 Flow (one speed for all
drift, square law; 0 = still, static still boils) · K6 Spectrum (bar + band
density) · S7 Tear · S8 Eye Lens/Iris · S9 Moire · S10 Horizon Flat/Deep ·
S11 Palette Spectrum/Opal (cool fold of the spectrum) · **P12 Vision** (the
eye opens; corona, rings, sparks, ripple and sway swell with it).

Presets are hand-typed placeholders — capture real ones with
`./capture_preset.sh` once the look is approved.

## Architecture

- Raster: measured width/height, frame rows on interlace, generated
  parity-locked avid (aurora/cubist recipe), output gated outside it, one
  latency for all modes.
- **Microcoded engine** (port of cubist's, 24-bit): 256-word register file in
  2 x (256x16 + 256x8) EBR, 512-word program ROM, serial 16x12 multiplier and
  25/12 divider. Source: `gen/oracle_ucode.py` (assembler `gen/oasm.py`,
  emulator `gen/oemu.py`), output `oracle_ucode_pkg.vhd`. Per field it smooths
  the six knobs (deadband + glide), derives every per-field constant and
  writes output ports; then, one line ahead of the beam, computes each line's
  parameter block, which XFER copies straight from the register file into
  the line registers AND the stage-0 DDAs during blanking. Run
  `python3 gen/oracle_ucode.py` after any microcode edit: it assembles,
  checks every MUL/DIV operand against the unit widths, flags unintended
  24-bit overflow and prints per-line cycle budgets (NTSC worst ~750 of 858).
- rho^2 = u^2 + v^2: v^2 per line (microcode); u^2 by a second-order DDA at
  stage 5 started n = min(cx, 4 Wa) columns left of the eye (start value,
  step, column from the microcode; exact to 0.006).
- One blend: base colour -> (eye/corona colour or the moire/haze/reflection
  target, whichever alpha is stronger), |difference| x 4-bit alpha.
- Output: composed picture + stage-0 syncs into 256-deep circular EBRs; syncs
  read back D0+17 later, video D0 later (per-line delay on tear rows).
- 26 EBR: palettes 9, cloud sines 2, quarter sine 1, profiles 2, output 4,
  register file 4, program 4.

## Fit history (why it is built this way)
Hardwired first version: 10,358 LC (134%) — the frame/line sequencer, knob
smoothers and per-frame constants alone were ~4,800 LC. Microcode engine ->
8,671; merged blends + narrow units -> 7,837; rho^2 DDA -> 7,665 (99%, 39 MHz);
XFER loading the DDAs directly -> 7,436; engine timing restructure (LATCH only
forwards, EXEC does compares/launches, 24-bit) -> 7,294 (94%, 73.3 MHz);
two-step MIN/MAX takes the select off the adder's carry chain.

## Files
`oracle.vhd`, `oracle.toml`, `oracle_tables_pkg.vhd` (gen/gen_tables.py),
`oracle_ucode_pkg.vhd` (gen/oracle_ucode.py). `oracle.md` is the v1 design
spec (historical). v1.0.0 source is in git (0360e56); its vmprog and sim
PNGs were removed from this folder at finalization.

## Community port — fit for the stock SDK (v2.0.3, 2026-10-15)
- The stock LZX SDK builds 8 configs; its SD Standalone core is ~600 LC larger
  than HD. The staged v2.0.2 packed to 8063/7680 LC there (SD Dual 7553, HD Dual
  7481), 26 EBR.
- Bit-exact changes, all in `oracle.vhd` (the two packages are untouched).
  Packed SD Standalone LC after each step:
  1. **Comparisons as bit tests, one carry chain per add/sub (8063 -> 7790).**
     The zone flags (`p_zone`) come from one 13-bit difference per bound and
     sign/bit tests on it, not six magnitude comparators (the original 12-bit
     `wy0 + 2` / `rc + 2` wraps above 4093 are reproduced by all-ones guards).
     Every compare against a constant in the pixel path is a bit test
     (clamps, `> 2047`, `< 2`, `> 16`, `> 1024`, `> 200`, ...). The ALU's
     `A + B (+1)` is one adder (the carry-in rides in a guard bit) instead of
     an adder plus an incrementer; the chirp DDA's `fr -/+ cA` is one adder
     (`fr + (cA xor lf) + lf`) instead of an adder, a subtractor and a mux.
  2. **Pipeline delays in EBR, palettes in two EBRs a port (-> 7605).**
     Three 16x16 EBR delay lines replace ~230 pure pipeline registers
     (`dl7`: bar hue, run flags and zone, s2 -> s9; `dl4a`/`dl4b`: band hue,
     screen, pupil, coverage, window distances and avid, 4 clocks). Written
     every clock at one pointer, read at a fixed offset, so a read never hits
     the address being written; `no_rw_check` on each (0 transparency
     emulation). The three 256x10 palette ROMs per port (Y, U, V) are re-cut
     into one 16-bit (Y, U high) and one 14-bit (U low, V) ROM, each fitting
     a single EBR: 9 -> 6 EBR, which pays for the delay lines (26 EBR total).
  3. **Add/subtract on one adder, more bit tests (-> 7457).** The blend
     (`p16 = base +/- diff * alpha`), the band sway (`phC +/- sway`) and the
     static mix (`g +/- mix`) store the product one's-complemented when the
     sign says subtract, so the next stage is one adder with the sign as
     carry-in. The divider's compare and subtract share one chain. The rho^2
     DDA's saturation tests, the bevel thresholds and the `> 900` luma clamps
     are bit tests; `(a*(16-a))>>2` (rim) and `(t6*t6)>>6` are tables.
  4. **Eye coverage from the shifted bits (-> 7429).** The 16-bit barrel
     shift + `+8` + clamp(0..16) became a 12-bit shift whose in-range test and
     `+8` are bit operations (checked exhaustively over all 65536 x 8 inputs);
     the edge-distance subtract skips its 4 constant-zero low bits; outside
     the rho^2 run the field always clamps to 16383, so the dead `16383 + v2`
     adder is gone.
  5. **EBR outputs straight into logic (-> 7273).** The palettes are read
     one stage later (delay line 7 -> 8 clocks, the corona colour address one
     register later), so their EBR outputs replace the 90 fabric
     re-registers at s11/s12. The output buffer reads one clock older
     (`wp + not d`, i.e. delay d + 1) and its EBR outputs feed the output
     stage, dropping 34 re-registers. Pure ROM read ports carry no VHDL init
     value, which removes yosys's read-init emulation (and the 199-fanout
     reset net nextpnr promoted to a global for it).
- Every local rewrite was also checked exhaustively (or 2 M random cases for
  the 24/25-bit adders) in Python against the original expression
  (`pb/cand/oracle_tools/chk_all.py`, `chk_ar.py`).
- Packed LC (stock SDK): SD Standalone 8063 -> 7273, SD Dual 7553 -> 6735,
  HD Dual 7481 -> 6658. EBR 26 (27 on the dual configs), unchanged.
- Proven bit-exact against the staged v2.0.2 (GHDL side-by-side, syncs every
  clock, y/u/v while avid is high):
  - 403 x 201, 40 frames, stock stimulus: 0 mismatches in 3.24 M clocks;
  - 403 x 201, 40 frames, switches walking all 32 combinations (one per
    frame): 0 mismatches in 3.24 M clocks;
  - 1003 x 201 (the per-line microcode pass fits the line), 24 frames,
    switch walk: 0 mismatches in 4.84 M clocks.
  Mutation tests (delay-line offset, ALU carry-in, sync read offset, palette
  V source, coverage bit 3) each fail the proof (145 .. 114328 mismatches).
- dupin.py: 0 `SB_CARRY I0==I1` and 0 duplicate-input LUTs in all 8 configs.
- Stock build (router1, all 8 configs) on seed 1 and seed 2; own HD timing
  (core without the Sync Out generator) Analog/HDMI/Dual 76.8/82.3/87.5 MHz
  (seed 1), 76.2/78.8/88.1 MHz (seed 2).

- **HW-validated 2026-10-15** (Ron: "flashed and looks great"), so the 16+14-bit palette ROM split and the EBR delay lines are fine on hardware. Originally flagged for checking: palette colours on every scheme (each palette port's Y/U/V is now split across a 16-bit and a 14-bit ROM, each in its own EBR); the EBR delay lines and the output-buffer read (sims can't see EBR mapping or collisions).
- **Stock build:** seed 1 builds all 8 configs (seed 2 also). Own HD 76.8 / 82.3 / 87.5 MHz (analog / HDMI / dual).
