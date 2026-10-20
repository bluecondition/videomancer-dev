# Dicefall — dev notes

## v0.1.0 (2026-10-20): many numbered dice falling

Ron, 2026-10-19: "write a new one that has as many as possible on screen at once of different colors … all D20s or a random variety of dice based on a switch … rotate randomly and fall like tessera". He chose numbers on the dice (so fewer dice) and HD showing more than SD.

What it is: NAT20 v0.3.0's scanline vector renderer, numerals included, with a just-in-time engine feeding a 4-entry ring, two 128-px slots a scanline, and stateless hashed dice (DESIGN.md).

Capacity (emulated microcode, `df_rtlmodel.py`):
- 1080p and 720p: 20 dice (d20s or mix).
- SD: 4 d20s, or 6 dice with the mix.
- At 10% size, 20 is the two-a-scanline limit; the engine has ~30% to spare at HD.

Tooling:
- `python3 df_regen.py` assembles `df_ucode.py` (1023/1024 words, 128/128 registers) and writes the ROMs into dicefall.vhd.
- `df_rtlmodel.py --mode 1080p --fields 4 --mix` renders a frame from the emulated microcode.
- `.dicefall_work/compose.py` assembled the first dicefall.vhd from NAT20's blocks. dicefall.vhd is the source now (later fixes live only there).

### Bugs found

- **LDX** read the B port's late copy (`u_rda`) instead of the indexed read: no dice drew in the first sim.
- **Ring wait** came after the die's x was written to the header, which could overwrite an entry still waiting to be drawn. It now comes first.
- **Diagonal columns:** a plain LCG put consecutive dice a fixed step apart. The squared mix fixed it (neighbour correlation −0.49 → ≤ 0.05).

### Verification

- **Engine:** a GHDL register-write trace of a whole 720p field (79k writes) is identical to the emulator. The probe copies reported `u_wa`/`u_wd` from `p_rf`, armed on the FALL write after FCTR = 2, with repeats collapsed. The aligner is `.dicefall_work/trace_cmp.py`.
- **Renderer:** the 720p GHDL frame equals the model to within 1 code below the top dice.
- **Image-tester artifact:** every sim shows the field 4 lines low, with the top die's first lines missing. The tester's 8-line vertical blank is shorter than the dry pre-roll, so the first lines' starts arrive mid-pre-roll. Hardware has 4× (720p) to 5× (1080p) the time needed. When diffing a dicefall sim against the model, shift by 4 lines (`df_sim720t.png` vs `render(fields=5)`).

### Build (router1; seeds below)

The first build (pre timing fixes) failed HD HDMI on all 8 swept seeds (66–74 MHz).
- Critical paths were in the render sequencer: the ring-tag mux and compare, and r_line compares feeding the next-state logic directly.
- Glyph stage 6 also ran its saturation compare after its multiply-add.

Fixes, with no change to the picture (the 720p sim is pixel-identical before and after):
- per-entry ready / this-field / next-field flags, registered every clock; `r_fc + 1` as a register;
- RET split into compare (state 2) and apply (state 1);
- the header's `h_top <= r_line` compare and the sprite line test registered one sub-step ahead;
- the glyph saturation compare moved to stage 7, beside the multiply;
- g6's −2048 folded into g5 (the left taps' top bit flipped, read signed).

After the fixes, HD HDMI passed 4 of 8 swept seeds.

| Config | Seed | Fmax (MHz) | LC |
|---|---|---|---|
| HD Analog | 1 | 74.39 | 7,251 |
| SD Analog | 1 | 77.46 | 7,245 |
| HD HDMI | 1 | 78.32 | 7,260 |
| SD HDMI | 1 | 80.39 | 7,254 |
| HD Dual | **2** | 78.26 | 7,254 |
| SD Dual | 1 | 76.64 | 7,248 |

- 94.5% LC, 29/32 EBR. All six `.bin` pass iceunpack.
- `out/rev_b/ron/dicefall.vmprog` (526,348 B). Flashed on HD HDMI: solid gray, no dice (see v0.1.1).
- HD Analog's margin is thin (74.39). If it needs a rebuild, sweep seeds first; router1 seeds 1, 4, 5 and 7 passed HD HDMI.

## v0.1.1 (2026-10-20): the engine waits for the first field

Ron flashed v0.1.0 on HD HDMI (progressive): solid gray, no dice.
- Solid gray means even the backdrop gradient was missing. Its per-line step is the engine's first write each field, so the engine never ran a field.
- Sims at 720p, 1080p and 1080i (field_n tied high, as on hardware) all draw the dice, so the RTL logic is sound in every mode.
- v0.1.0's engine was the first in the library to run free from configuration (`u_done` initial '0'). Every hardware-validated engine (NAT20, CUBIST, Oracle, ATTRACT) is started per field.
- The suspect is the moment after configuration: block RAM (the program ROM) isn't reliably readable yet, and FFs leave the global reset unevenly against the clock. One garbage instruction can reach END or a dead loop, and nothing ever restarted it.

Fix:
- The engine is held (`u_done` initial '1') until the first field start.
- A watchdog (`p_wdog`) restarts it from the top: PC 0, multiplier and divider idle. It fires at a field start once three field starts have passed with no "field begun" write (the backdrop-step SLW).
- The picture is unchanged once running.

Build (router1, seed 1, all 6): HD Analog 77.17, SD Analog 74.16, HD HDMI 74.32, SD HDMI 83.17, HD Dual 81.61, SD Dual 80.26 MHz.
- 7,243–7,261 LC (94.5%), 29/32 EBR; all six `.bin` pass iceunpack.
- vmprog 525,809 B.
- HD HDMI's margin is thin. If HD HDMI misbehaves on hardware while other modes are fine, suspect timing first.
- **HW-confirmed 2026-10-20** (Ron, HD HDMI): "this is working now". The dice draw.

## v0.2.0 (2026-10-20): a numeral on every face; black backdrop

Ron on v0.1.1: "frequently a number will disappear from a face as the dice tumbles down. Also, I'd like the background to be black, or at least on a switch between black and gray textured".

**Vanishing numerals.**
- Cause: a die's ring entry held 5 numeral sprites, given to its front faces in face-index order. A d20 shows about 10 faces.
- So when the die turned, a lower-numbered face coming round took the slot of a face still in plain view, and that face's number vanished.
- Model count over 24 fields: at 1080p every d20 had hit the cap of 5. Uncapped, a d20 shows 5–9 numerals (mostly 7–8), 720p 5–8, the mix up to 8. SD never exceeds 5, since small faces fail the readability test.
- Fix: sprites 5..12 go in a 256 × 32 extension RAM (`xram`, 2 block RAMs).
  - The ring address is now entry | offset (8 bits). Offsets 128..191 are xram; the sprite states read through a 2:1 mux (`w_rdm`) on a registered address bit.
  - The microcode only changes its cap, `MAX_SPR` 5 → 13 (still 1023 words).
  - The header's sprite count widens to 4 bits.
- Cost (model): HD engine 838k → 1,008k clocks a field at 1080p (d20s, of 1.24M); 862k → 979k at 720p. Worst render line 670 of 2,200 (1080p), 635 of 1,650 (720p).

**Backdrop.**
- S8 Backdrop: Black (default) or Gray, latched at the field start. Black is luma 64.
- The gray gradient is unchanged except its chroma: exactly neutral now (128/128, was 130/127), so one constant serves both.
- Watch: obsidian dice (Y ≈ 120) are dark on black. Their faces still shade and their numbers are white.

Build (router1, seed 1, all 6): HD Analog 74.95, SD Analog 73.24, HD HDMI 77.30, SD HDMI 72.30, HD Dual 74.98, SD Dual 73.34 MHz.
- 7,245–7,269 LC (94.6%), 31/32 EBR. The read mux and the wider fields cost almost nothing.
- All six `.bin` pass iceunpack; program_config.bin carries "Backdrop".
- vmprog 530,507 B.
- HD HDMI went from 74.32 to 77.30; HD Analog and HD Dual are now the thin ones (~75).

Verification: 720p GHDL sims, mix on black and d20s on gray (tumble 0).
- d20s show 7–9 numerals.
- The d20 frame equals the model below row 124 (`.dicefall_work/simdiff2.py`: fields 4, row shift 12). The top dice are clipped by the image tester's short vertical blank, as before.
- Worst dry pre-roll (model, `.dicefall_work/prestats.py`): 51k clocks at 1080p (budget ~95k), 33k at 720p (~46k).
- **HW-validated 2026-10-20** (Ron, HD HDMI): "Looks good, no bars and all looks clear."

## v0.3.0 (2026-10-20): S9 video-coloured dice

Ron: "Another switch to implement - instead of the dice being colored, bring in the incoming video to color them, but keep the numbers".

S9 Colour: Dice (default) / Video. In video mode each die is a window onto the input, shaded by its faces' light, with black numerals. S8 still puts black or gray round the dice.

How, with no new microcode (1023/1024 words):
- **Engine, VHDL-side overrides.** In video mode the palette reads (SRD offsets 40..42, which only the palette uses) return Y 484, U = V 512 for every die. Full light is then 420 over black, 13/16 of 512 as the output scales luma. The ink-mask ROM read returns all ones, so every numeral gets black ink.
- **The display backdrop is black in video mode**, so the composed luma is 64 + L: face shade × coverage, near 0 on a numeral. It still passes through the [1 2 1] smoothing, so the HD HDMI bar protection holds.
- **One shift-add product, 64 + a·b/16, serves both modes** (stage 7: two partial sums; stage 8: their sum).
  - Colour mode: a = y − 64, b = 13. This is the old luma scaling toward black, ±1 code.
  - Video mode: a = input luma over black, b = L/32 (4 bits, 16 levels).
  - Off the dice (smoothed L = 0): a = the S8 backdrop over black, b = 13.
- **Video chroma:** the input × 1, ½ or 0 by L (L ≥ 256, ≥ 128, else). Numerals and the outer edge pixels stay neutral.
- The display path is a clock longer in both modes (C_XSHIFT 3 → 2; sync tap still 12).
- The input is taken at the compose stage, so the video sits ~10 px left of its true position, which is invisible inside a die. It's in hardware U/V order, so no swap.

Fitting it (v0.2.0 was 7,245–7,269 LC):
- A first cut with three real multipliers (10×6, 2 × 10×5) placed at 7,867 LC: overflow. yosys builds them ~500 LUT wide.
- Shift-add partial sums brought it to 7,557 (98%): no legal placement.
- Then the shared colour/video product, plus invisible trims, gave 7,303 at HD Analog (95%). The final single-add form gave 7,282–7,293:
  - the die table made a ROM (SWR is never used);
  - the numeral AA scale k narrowed to 6 bits (capped at 63 in the microcode; the model's k is 16–31 in every mode and size), so its multiply loses two rows;
  - the stage-5 chroma clamps dropped (the palette's chroma stays in ~190..830).
- **abc9 "Boxes are not in a topological order"** killed SD Analog's synth twice; seeds and moving TMPDIR don't help. The partial sums were a case whose arms each held an adder (0 / a / 2a / a + 2a). Fix: one add of gated operands per partial sum, with 64 × 16 riding in free as bit 10 of the first operand (a ≤ 959). That was also ~70 LUT smaller.
- **Fit lesson:** at 94%+, a per-pixel multiply in a new display mode only fits by sharing the existing per-pixel arithmetic, here the output luma scaling, which is the same "a × b / 16" shape.

Sims (720p, rainbow input): video mode on gray and colour mode on gray. Video dice take the input's hue at their place, shaded, with black numerals; colour mode is unchanged.

Build (router1): HD Analog 75.99 (seed 1), SD Analog 72.38 (1), HD HDMI 74.80 (1), SD HDMI 78.47 (1), HD Dual **82.15 (seed 2)**, SD Dual 74.96 (1) MHz.
- 7,282–7,293 LC (95%), 31/32 EBR. All six `.bin` pass iceunpack; program_config.bin carries "Colour" / "Video".
- vmprog 528,231 B.
- HD HDMI's margin is thin (+0.55 MHz). If HD HDMI misbehaves while other modes are fine, suspect timing first.
