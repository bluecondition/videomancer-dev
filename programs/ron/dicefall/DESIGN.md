# Dicefall — design

Ron, 2026-10-19: "how many can we get on screen at once at 10% … write a new one that has as many as possible on screen at once of different colors. It can be all D20s or a random variety of dice based on a switch. They should rotate randomly and fall like tessera." He chose numbers on the dice (fewer dice), and HD showing more than SD.

## Why NAT20's renderer can't simply draw many

NAT20 stores every die's crossing records twice, so the engine can compute the next field while this one is drawn. Its line buffer is one 256-px window. For many dice:
- the records alone would need 16+ block RAMs;
- dice anywhere across the screen would need a full-width line buffer (48+ block RAMs);
- the engine would need ~66k clocks a d20 with numerals, against 225k a field at SD.

## What Dicefall does

- **Just-in-time engine.** The engine never stops. Field after field, it computes every die in screen order (top first) into the next free entry of a 4-entry ring (512 words: edges at 4e, header at 80, sprites at 8(s + 11)).
  - The renderer takes an entry when the beam reaches its die's top, and frees it once the beam has passed the bottom.
  - Only the dice near the beam are ever stored.
  - Entries carry a field tag. The renderer frees an old field's, waits on the next field's, and the engine stays one field ahead at most (skipping ahead when it falls behind).
- **No per-die state.** All dice fall at one speed (P12) with evenly staggered phases round a cycle of (screen + one die) lines. A die's place in the cycle and its generation (how many times it has wrapped) give a hash: the middle bits of the square of a 13-bit LCG.
  - The hash sets colour, kind (S7), x within the left or right half (alternating, so neighbours never overlap), and two spin angles (phase + an even rate × a tumble clock).
  - A plain LCG put consecutive dice a fixed step apart, in diagonal columns. Squaring broke that (neighbour correlation −0.49 → ≤ 0.05).
- **Two slots.** With N dice evenly staggered, never more than two share a scanline when N ≤ 2 × cycle / window. Each active die gets a 128-px slot in the line buffer.
- **Two physical line-buffer banks** (drawn / shown, swapping each line), four channels (D and F of luma, D of U8 and V8; colour steps a whole pixel at a time, which 4:2:2 hides). The renderer has the whole line.
- **Dry pre-roll.** Dice straddling the top start above line 0. At each field start the renderer runs lines −128..−1 without depositing, so their crossings and numeral texel coordinates arrive at line 0 in step. Engine-side clipping would have needed multiplies the full program ROM couldn't hold.
- **Engine changes from NAT20:**
  - a hardware divider (26 clocks; the slope and numeral-frame divides were a third of NAT20's engine time);
  - ONE register-file copy, with B read a clock after A (2 block RAMs saved, ~15% slower);
  - 1023/1024 words, 128/128 registers.
  - held from configuration until the first field start, with a watchdog that restarts it if three field starts pass without it beginning a field (v0.1.1).

## Capacity (model, `df_rtlmodel.py`)

| Mode | Dice | Engine clocks a field (budget) | Worst line (budget) |
|---|---|---|---|
| 1080p | 20 d20s / 20 mix | 830k / 565k (1.24M) | 604 (2,200) |
| 720p | 20 d20s / 20 mix | 854k / 563k (1.24M) | 573 (1,650) |
| 480i | 4 d20s / 6 mix | 180k / 177k (225k) | 304 (858) |

At 10% size, HD's 20 is the two-a-scanline limit (N ≤ 2 × cycle / window), not the engine's: it has ~30% to spare. More dice at that size would need a third line-buffer slot (8 more block RAMs).

**SD warning:** the same picture shape, but fewer dice. At SD a field has about a fifth of HD's clocks.

## Verification (2026-10-20)

- **Engine:** GHDL register-write trace of a whole 720p field (79k writes) is identical to the microcode emulator (`df_uasm.py`).
- **Renderer:** the 720p GHDL frame equals the model (`df_rtlmodel.py`) to within 1 code below the top dice. The sim shows everything 4 lines low and the top die clipped. That is the image tester's fake 8-line vertical blank: the dry pre-roll (~12k clocks at 720p, ~20k at 1080p, ~4k at SD) overruns it, so the first lines' starts arrive mid-pre-roll. Hardware gives the pre-roll ~47k clocks at 720p, ~95k at 1080p and ~17k at SD (from two line-widths after the last active line to the next field's first).

## Block RAM

29/32: microcode 8, register file 2, engine ROM 1, die table 2, ring 4, line buffers 8, glyphs 4.

## Build

v0.1.1: 6/6 closed at the full clock (router1, seed 1), 74.2–83.2 MHz, 7,243–7,261 LC (94.5%). Details, the timing fixes and the start-up fix in SUMMARY.md.
