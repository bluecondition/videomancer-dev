# NAT20 — dev notes

## Phase 1 (v0.1.0, 2026-10-18): geometry

Flat-shaded dice with AA silhouettes; K1 die, K2/K3 yaw/pitch (centre = hero pose), K4 colour, P12 light. Architecture and where it refined the proposal: DESIGN.md §8.

Tooling:
- `python3 nat20_regen.py`: assembles the microcode and writes it, the die table and the engine ROM into nat20.vhd.
- `python3 nat20_rtlmodel.py --sheet`: renders all five dice in three poses from the emulated microcode (720p).
- `python3 nat20_rtlmodel.py --die 4 --yaw 600 --pitch 450 --mode 1080p`: renders one frame.
- Engine clocks per field (emulator): d4 ~8.7k, d6 ~16k, d8 ~12k, d10 ~24k, d20 ~25k. It runs during the field (budget ~225k at NTSC), so this is no constraint.
- Microcode: 857 words (1024-word ROM, 8 EBR), peak 117 of 128 registers.
- Standalone yosys: 3,315 LUT4, 1,160 carry, 2,822 FF, 20 EBR; `emulate_transparency` 0.

Bugs found by the Python model before any VHDL:
- The edge-pass gradient register was named `gy`, the same as the yaw glide's persistent register, so it overwrote the glide. The assembler's `new()` now refuses names already in use.
- The die-table header never wrote the split-face flag, so each half of a d6 square took a different face's normal.

Sim (once, new program): headless GHDL, 1080p dec 4, d20 hero pose.
- The first run showed striped outer faces. Cause: a saturated AA window took its sign from slice bit 17 instead of accumulator bit 25, so far-inside pixels read as far outside every 2^18 units.
- Localised by diffing the sim's engine writes against the emulator (126/126 equal) and probing the DDAs on lines 0/1/100 against the model (bit-exact).
- After the fix the sim matches the model face for face.

Build (router1, seed 1, all 6): HD Analog 74.27, SD Analog 74.31, HD HDMI 75.52, SD HDMI 74.11, HD Dual 77.87, SD Dual 79.65 MHz; 6,390-6,415 LC (83%), 20/32 EBR.
- The critical path was the generated avid's `g_cnt = s_Wl - 1` compare, a 12-deep LUT chain. W - 1 is now a register.
- Rebuilt (router1, seed 1, all 6): HD Analog 87.27, SD Analog 87.57, HD HDMI 86.06, SD HDMI 93.59, HD Dual 83.40, SD Dual 83.73 MHz; 6,391-6,399 LC (83%), 20/32 EBR; all six .bin pass iceunpack. vmprog: out/rev_b/ron/nat20.vmprog. Not flashed yet.

## LC budget (the constraint for phases 2-4)

- SDK core alone (passthru, HD HDMI): 1,137 LC. NAT20's own logic is ~5,250 LC.
- Standalone ablations (yosys, LUT4 / FF):
  - engine ~850 / 420, plus 15 EBR;
  - edge DDAs ~870 / 1,560 (the 780 gx FFs cannot pack with logic);
  - AA chain ~1,100 LUT (three 1/m multiplies, 30 window flags, the 10:1 owner mux);
  - output conditioning ~60 LUT.
- At ~94% there is ~830 LC left. Phases 2-4, done leanly, need ~1,300-1,400.
- Trims with no visible change (~500 LC):
  - 1/m as a 3-bit shift-add instead of a multiply (~200);
  - faces that share an edge paired into slot pairs, so the shared edge is one accumulator. A perfect pairing of the d20's 10 visible faces fails in ~24% of orientations (max matching 4; 4,000 random poses), so the layout is 4 pair slots + 2 single slots: 26 DDAs instead of 30, ~220 LC. It also hosts d10 (4 kites paired + 1 kite as two singles), d8, d6 and d4;
  - knob latches dropped and the descriptor registers thinned (~100).
- Rejected: exact-pixel normalization of the edge functions. It removes the 1/m step, but its rounding breaks vertex concurrency, which shows as specks at vertices.

## v0.1.1 (2026-10-18): the LC trims, no intended visible change

Ron: "proceed with the recommended fixes, the numbers themselves are the most important thing" (the numerals now rank right after geometry and AA).

- **Pair slots.** Slots 2p and 2p+1 (p = 0..3) share their position-0 edge: one accumulator, which the odd slot reads inverted (not E = -E - 1, so it claims where E < 0). That makes 26 accumulators, not 30. Slots 8 and 9 are singles.
  - The engine's pairing pass is a greedy matching over the edge list (fake diagonals first), retried from each start edge until it has max(0, visible - 6) pairs.
  - Leftover triangles fill the singles, then sit alone in free pair slots.
  - A slot's edges are its triangle's, rotated so the shared edge is position 0. SLOT[t] = slot | ke << 4, pos = k - ke mod 3.
  - Study (`.nat20_work/pairing_study.py`, 20,000 random poses per die): one fixed edge order misses on 8% of d20 poses; best-of-start-edges and min-degree greedy never missed.
  - The engine publishes a 10-bit valid-slot mask (port 16) instead of the slot count.
- **AA 1/m as a 3-bit shift-add.** The code is c with 1/m ~ (8 + c)/16, error at most 6.25%, so the 1.5 px inset moves by ≤ 0.09 px. t = w8 * (8 + c) is three adders; the coverage ramp (1 to 2 px inside, t 512..1024) is then pure bit selects. The window is clamped to 0..255 before the shift-add (only 1..2 px inside matters).
- **Registers.**
  - The knob latches are gone: the microcode reads each control once a field, so CTL reads `registers_in` live (K5, K6 and the switches read 0 until they get jobs).
  - The descriptors are no longer carried down the pipe. Each descriptor RAM reads at its own delayed owner address (C codes + silhouettes at stage 2, A luma at stage 4, B chroma at stage 6), so the pipe carries a 4-bit owner instead.
  - The backdrop top is a constant, and its per-line step is single-buffered (static per mode).
- **Front threshold scales with the frame.** A triangle is front when area2 < -32 hf (Q8). The old fixed -64 let rounding count an edge-on d20 face AND its parallel opposite as front: 11 faces in 1 of 10,000 poses. One face was then dropped by triangle order, not size, and every pairing retry ran (80k clocks). Vertex rounding error ~16 hf; twice that drops only faces under ~0.25 px wide, which are always on the outline. The pairing target is also capped at 4.
- Engine sweep (`.nat20_work/engine_sweep.py`, emulator): 40,000 d20 poses over 1080p/720p/480i/576i plus 16,000 of the other dice; no front triangle ever left without a slot.
- Microcode: 949 words, peak 117 registers. Engine clocks: d20 up to ~55k when the first pairing pass falls short (budget ~225k at NTSC).
- Model regression (`nat20_rtlmodel.py`, 60 random poses per die, against v0.1.0):
  - the union of owned pixels is identical in every pose except 29 outline pixels on the d6 (near-edge-on faces now back); no holes;
  - the face partition moves only at exact ties: a crease pixel where the shared function is exactly 0 now goes to the pair's even slot instead of the lower slot;
  - AA-band luma moves by ≤ ~30 codes on 98% of pixels (the coarser 1/m).
- Standalone yosys, program only: LUT4 3,315 → 2,816, carry 1,160 → 1,056, FF 2,822 → 2,440; 20 EBR; `emulate_transparency` 0.
- Build (router1, seed 1, all 6 on the first attempt): HD Analog 90.41, SD Analog 90.24, HD HDMI 87.54, SD HDMI 83.19, HD Dual 90.41, SD Dual 90.49 MHz; 5,541-5,566 LC (72%, was 83%: -840 LC), 20/32 EBR; all six .bin pass iceunpack. vmprog: out/rev_b/ron/nat20.vmprog (409,407 bytes). Not flashed yet.
- Budget now: ~1,670 LC to the ~94% routing ceiling (was ~830), against ~1,300-1,400 for phases 2-4 built leanly.

## v0.2.0 (2026-10-19): the miniature, a scanline vector renderer (proof of concept)

Ron didn't like v0.1.x's creases and outlines. He asked for a proof of concept of a die ~10% of the height, "very detailed". Design: DESIGN.md §10. v0.1.1 is backed up in `.nat20_work/v011/` (sources and vmprog).

- **Engine (758 words, peak 110 registers):**
  - The front pass shades each front triangle into a packed colour.
  - The edge pass writes crossing records (y0 | y1 << 15, x at y0 Q16, slope Q16 per sub-line, colour step | sil | side) through a 26-bit restoring divide.
  - Pairing, slots and the 1/m codes are gone.
  - K5 Size: radius 2.5–11% of the height, glided.
  - Two rounding fixes found by the reference check:
    - x_first is biased by 1/32 px, so the 1/16 px fraction rounds instead of truncating (the error was 25 codes);
    - the slope divide rounds, because its error accumulates down the edge.
- **RTL:**
  - The render sequencer runs after each line's window: 5 clocks for a record that misses the line, 8 + its sub-lines for one that crosses it.
  - Deposit pipe: q1 crossing and read issue, q2/q3 step × fraction, then a run accumulator. It flushes one read-modify-write per pixel run. Within a record the pixels move one way only; records are 8+ clocks apart.
  - The line buffer has one shared read/write port: the display reads and clears behind itself during the window, and the renderer works outside it.
  - The display path is 9 clocks against the 12-clock sync tap, so the window is shifted +3 px.
  - The backdrop is latched at render start, so a render that overlaps the line's end can't mix two backdrop values.
- **Model** (`nat20_rtlmodel.py`) and **reference check** (`.nat20_work/v020/ref_check.py`, 5 dice × 4 modes): error ≤ 11 codes, no leakage, render ≤ 244 clocks a line, engine ≤ 38k clocks.
- **Standalone yosys:** 2,352 LUT4, 864 carry, 1,978 FF, 25 EBR; `emulate_transparency` 0.
- **Sim (once, new datapath):** headless GHDL at 720p60, no decimation, d20 hero pose. Diffed against the model (output conditioning and U/V swap applied): **0 pixels differ** across the frame (`.nat20_work/v020/sim_diff.py`).
  - The captured frame was computed with the power-up frame height (480): the engine reads the measured height one field late, so the first two fields after load draw the die smaller and higher. This is the same as v0.1.
  - `prev_miniature_x3.png` is the model's 1080p sheet at 3× zoom (all five dice, three poses each).
- **Build** (router1, seed 1, all 6 on the first attempt): HD Analog 86.90, SD Analog 87.97, HD HDMI 88.18, SD HDMI 90.02, HD Dual 87.90, SD Dual 88.24 MHz; 5,059–5,075 LC (66%; v0.1.1 was 72%), 25/32 EBR. All six .bin pass iceunpack. vmprog: out/rev_b/ron/nat20.vmprog (390,373 bytes). Not flashed yet.

## v0.3.0 (2026-10-19): numerals

Ron liked the v0.2.0 miniature on hardware: "let's add numbers to each one now". Design: DESIGN.md §11. v0.2.0's vmprog is backed up in `.nat20_work/v020/nat20_v020.vmprog`.

- **Numerals as sprites**, not vector outlines (those would need ~400 crossing records and too many slope divides).
  - Glyphs: Lato Bold distance fields, 12 glyphs (0-9, plus underlined 6 and 9) at 16×20 texels, 4-bit, in four 2×2-interleaved EBR banks (`nat20_glyphs.py`, `glyph_preview.png`).
  - Placement and orientation are the look model's, built from the projected corners with per-shape weights and per-die constants.
  - Ink: white, black on Bone, Frost and Sunflower, lit like its face.
- **Tables:** faces in number order (d4: vertices), the up corner or edge in tri-word bits 12-13.
- **Engine:** 995 of 1024 words, peak 122 registers. New ops: MULSH, MIN, MAX, SRDS, ROMS. GLIDE is a subroutine. The engine needs up to 72k clocks a field (d20; the SD field is ~225k).
- **Renderer:** a sprite phase after the edges, then an 11-stage glyph pipe into the deposit pipe. The 512-word record table holds the edges, their line-start x and the sprites, with one write port shared by the engine and the renderer.
  - Worst line: 494 render clocks at HD (budget ~1,930) and 380 at SD (~714).
- **LC trims** (from standalone ablations):
  - the glyph pipe first measured 957 LUT and the sprite sequencer ~500;
  - chroma ink switches whole at half cover (luma keeps the 17-level ramp), so 2 of the 3 coverage multiplies are gone;
  - glyph base addresses are computed once a span, and texel coordinates are 20-bit;
  - the pixel address is regenerated at the pipe's end, the span is fetched straight into the walk registers, and the AA multiply is 11-bit with saturation flags.
  - Standalone: 3,974 → 3,726 LUT4, 2,842 → 2,779 FF, 29 EBR.
- **Bugs found by the model and sim:**
  - Triangles with the up corner at k = 2 read the wrong 3rd corner: the corner list repeats with period 3 for triangles, not 4. RT came out 0 and the numeral was skipped.
  - Faint boxes round tilted numerals: a small AA scale stretched the ramp past the field's +0.875 texel range. The scale is now floored at 1 px a texel.
  - Horizontal streaks to the window edge: the edge-sample strobe lacked the phase bit, so during numeral spans the deposit pipe took stale edge samples. Fixed with the phase bit (sim).
  - Faint boxes again after the trim: the 11-bit AA clamp put "far outside" one step short of zero. It now saturates (sim).
- **Sims** (720p60, no decimation, d20, two poses), diffed against the model: **0 pixels differ** in both.
- **Build** (router1; all six pass, iceunpack OK):
  - HD Analog 75.03 MHz (**seed 5**), SD Analog 72.96 (seed 1), HD HDMI 74.52 (**seed 2**), SD HDMI 77.64 (seed 1), HD Dual 77.94 (**seed 2**), SD Dual 74.10 (seed 1).
  - 6,970-6,993 LC (91%), 29/32 EBR. vmprog: out/rev_b/ron/nat20.vmprog (510,532 bytes). Not flashed yet.
  - HD margins are thin (0.4-5%), and three HD configs needed later seeds. Read the critical path before adding anything; the next features want LC trims first (the glyph pipe and sprite sequencer are ~1,400 LUT).
- An earlier build at 7,468 LC (97%, before the trims) still closed at 85 MHz on HD Analog seed 1, so the 91% build's thin margins are placement, not depth.
