# Titler v2.2 — Crawl plan (option 1)

Status: IMPLEMENTED 2026-09-28 (v2.2.0, 6/6 timing-closed, not yet flashed).
Deviations from the plan below: (1) the horizontal negative-edge preload is
`|L| * step` on the shared multiplier instead of the `SM_H_DIV` divide. The
divide drops the remainder, so crawling text would jump a glyph pixel at a
time crossing the left edge. (2) K5 went linear, as the plan's first option.
(3) Ticker preset P12 = 740, about 150 px/s at 60 Hz. (4) The edit-write
cone is now registered one clock (timing fix).
Source: release audit session. Titler v2.1.0 is HW-validated (v2.0), 3321/7680 LC, 3 EBR, HD 73.5–80 MHz.

## Goal

Make P12 a real performance control. P12 becomes a **bipolar Crawl** (news-ticker scroll).
Size moves off the slider onto a knob. Keep every existing feature.

## Control changes

| Control | Now | After |
|---|---|---|
| K1 | H Position | unchanged (still sets the resting position; the crawl offsets from it) |
| K2 | V Position | unchanged |
| K5 | Cursor (only acts in Edit Mode) | **Size/Cursor**: Edit Mode off = Size, Edit Mode on = cursor (same pattern as K4 Color/Erase) |
| P12 | Size | **Crawl**: centre = stopped, above centre = text crawls left (ticker direction), below = crawls right |

### K5 Size pickup (avoid jumps after editing)
- Keep a latched size register `r_size` (10 bit) that replaces `s_size_raw` in `SM_MUL_S`.
- Flag `size_follow`: power-on = '1' (so presets and the knob work straight away).
- Cleared when Edit Mode turns on.
- With Edit Mode off: set `size_follow <= '1'` when K5's top 5 bits change (reuse the existing `s_cursor_5p /= prev_cursor_5p` edge).
- While `size_follow = '1'` and Edit Mode off: `r_size <= unsigned(registers_in(4))`.
- Result: leaving Edit Mode keeps the old size until K5 is actually turned. Moving K5 while not editing also moves the (hidden) cursor, which is harmless.

### P12 Crawl
- `d = |P12 - 512| - 24` (deadband of ±24, clamp at 0). Sign = direction.
- Velocity in 1/16-px per field: `v = (d * d) >> 10` (0..~232, so max ≈ 14.5 px/field). The square gives fine control near the centre.
- Compute `d*d` on the existing shared serial multiplier: add two states (`SM_MUL_V` / `SM_V`) to the vblank sequencer, following the `SM_MUL_CX` / `SM_CX` pattern. One small op per state (sequencer cones are STA-timed at the pixel clock).
- Once per field (serration-guarded, i.e. inside the existing sequencer run started by `saw_active`): `crawl_q += / -= v`.

## Geometry and wrap

The existing sequencer computes `cx = (H * meas_w) >> 10`, `half_w = scale * 64`, and handles a negative left edge (text partly off the left) via the repeated-subtract `SM_H_DIV` preload. Text that runs off the right clips naturally.

- Wrap period `P = meas_w + 2*half_w`, so the text fully leaves one side before it re-enters the other.
- Crawl accumulator `crawl_q` is unsigned, 14 integer + 4 fractional bits, kept in `[0, P)`:
  - after adding: `if crawl >= P then crawl -= P`
  - after subtracting: `if negative then crawl += P`
  - If Size changes and `P` shrinks, the one-subtract-per-field wrap converges over a few fields. That's acceptable, but guard the result.
- Left edge `L = cx - half_w + crawl_int`. If `L >= meas_w` then `L -= P`, giving `L` in `[-2*half_w, meas_w)`.
- Feed `L` into the existing `SM_ORIGIN` logic in place of `cx - half_w`:
  - `L >= 0`: `s_h_origin = L`
  - `L < 0`: residue `= -L`, preload via `SM_H_DIV`
  - Also guard `L >= meas_w` (text invisible): set `s_h_origin` to all-1s and `hpos_init` to sentinel.
- **WIDTHS: HD hero size makes `half_w` up to 32*64 = 2048, so `P` reaches about 1920 + 4096 = 6016. Use ≥14-bit (signed 15-bit for `L`) arithmetic.** The current 12/13-bit `v_cx`/`v_half_w`/`sq_residue` must be widened. Resize BEFORE shifting (CLAUDE.md `shift_left` trap).
- `SM_H_DIV` quotient can now reach 128 (fully off-left). The 8-bit `sq_quotient` holds it, and quotient 128 sets bit 23 = "past text", which is correct. The loop is at most 128 iterations, which is fine in vblank.
- Only the integer part of `crawl_q` moves the text, so slow speeds step 1 px every few fields. That's inherent; accept it.

## TOML changes (`titler.toml`)
- `program_version = "2.2.0"`.
- K5: `name_label = "Size/Cursor"`. The value is continuous now, so drop `steps_32` and use `control_mode = "linear"`, 0–100 %, initial ~400.
  - Check what the Cursor display loses. `steps_32` gave the cursor 32 detents; with linear the cursor still uses the top 5 bits, so it still works, just without detents. If detents matter more, keep `steps_32` and accept size in 32 steps (`r_size` from the top 5 bits `<< 5`). Decide on HW feel.
- P12: `name_label = "Crawl"`, `display_min_value = -100`, `display_max_value = 100`, `initial_value = 512`, suffix `%` (bipolar pattern as in bajaweave/aurora).
- Presets: move each preset's old `linear_potentiometer_12` value to `rotary_potentiometer_5`, and set `linear_potentiometer_12 = 512` (stopped).
  - Consider one new crawling preset, e.g. "Ticker": News Bar look, P12 ≈ 700. That makes 6 of the 8 allowed.
- Update the TOML comments (K4/K5 dual-role note, slider note) and `description` (≤127 bytes; the categories list `["Text"]` is valid).
- Update the VHDL header register map and pipeline comment.

## Build / verify
- `./build_programs.sh ron titler`. Report post-route HD Fmax (last "Max frequency" line per clock) and LC.
- Remove the stale config cache first (TOML labels/presets changed): `rm build/programs/ron/titler/program_config.bin build/programs/ron/titler/rev_b/program_config.bin`. Confirm "Converting TOML..." and `grep -a Crawl` the binary.
- TOML pre-check: `python3 videomancer-sdk/tools/toml-converter/toml_to_config_binary.py programs/ron/titler/titler.toml <scratch>/t.bin`.
- No sim needed by default: build, then Ron checks on hardware.
- Hardware checklist for Ron:
  - Centre deadband holds the text still.
  - Crawl is smooth both ways at slow and fast speeds.
  - Text leaves fully before re-entering.
  - Crawl works at every Size, including hero size.
  - Leaving Edit Mode doesn't change the size.
  - Presets load stopped.
- Then update memory `titler_milestone.md`, and the release-report card if asked.

## Rules that apply (from CLAUDE.md)
- Don't touch `hd_clock_divisor`.
- Blanking gate and single latency already exist; don't add pipeline stages to the pixel path. All the new work is in the vblank sequencer, so latency stays at 7.
- Serration: per-field increments only inside the `saw_active`-guarded sequencer run.
- One op per sequencer state; use the shared serial multiplier, not `*`.
