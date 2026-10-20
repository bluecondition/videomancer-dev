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
