# Titler — lo-res video character generator (v2.1.0)

## What it does
A 1980s/90s hardware character generator: two lines of 16 characters over
the input video, with the edge and panel treatments the period boxes were
known for. K3 is the "style button" — eight looks from plain ink through
drop shadow, black edge, hollow and emboss to a solid, half-tone or reversed
panel behind the whole block. P12 scales the type from small caption to
full-height hero letters. Text is edited live on the front panel: a cursor
walks the 32 cells (K5), K6 picks the letter under it, K4 doubles as an
erase-and-scroll knob, and T11 centres each line automatically.

## How it works
Processing program, 7-stage pipeline. Physical pixels map to glyph pixels
through Bresenham-style position accumulators that advance by
`step = floor(2^16 / scale)`, so any integer scale is supported with no
multiplier in the pixel path — only a 32-entry step LUT.

**The font ROM returns a whole glyph per read.** 128 entries × 64 bits (8×8
pixels) in EBR, so rows `y-1`, `y` and `y+1` are all in hand at once and the
entire 3×3 neighbourhood costs a couple of OR gates:

```
dil  = row(y-1) or row(y) or row(y+1)
dil3 = dil or (dil sll 1) or (dil srl 1)
outline = dil3(x) and not self          -- black edge / hollow
shadow  = (row(y-1) srl 1)(x) and not self   -- drop shadow, SE
hilite  = (row(y+1) sll 1)(x) and not self   -- emboss, NW
```

Glyphs occupy columns 1..5 of an 8-wide cell, so a one-pixel edge stays
inside the cell horizontally. Vertically it does not — the row above a
glyph's top row belongs to the cell above, holding a *different* character —
so each text line gets its own dedicated top-margin row. The block is 18
glyph rows: row 0 = line 0's margin, 1..8 = line 0, row 9 = line 1's margin,
10..17 = line 1 (row 7 of each cell is the blank inter-line gap). On a margin
row the pipeline fetches that line's character but forces
`row(y-1) = row(y) = 0` and `row(y+1) = glyph row 0`, which is exactly the
"one row above the glyph" the dilation needs. Without it every character
loses the top of its outline. At scale 8 an edge is 8 physical pixels thick —
the chunky look of the real hardware.

S6 collapses everything into a 3-bit pixel class (background / ink / black /
white / dimmed video / matte video) so S7 is a shallow mux plus the blanking
gate.

**The panel styles wrap only the lines that carry text.** The per-field scan
that feeds Justify also reports whether each line is non-empty, and the panel
extent follows: rows 0..18 for both lines, 0..9 for line 0 alone, 9..18 for
line 1 alone, nothing when the buffer is empty. Row 0 is line 0's outline
margin and row 9 doubles as line 1's margin and line 0's bottom pad, so the
padding stays proportionate whichever line is in use. The text's own vertical
centring is unchanged — clearing a line shrinks the mask, it does not move
the remaining line.

**All geometry is screen-relative.** A vblank sequencer derives, from the
runtime-measured active raster:

```
chars  = meas_w >> 7          scale at which 16 chars exactly fill the width
smin   = max(1, chars >> 2)   slider 0%
smax   = min(32, chars * 2)   slider 100% (~8 chars across — the climax)
scale  = smin + (P12 * (smax - smin)) >> 10
cx     = (H Pos * meas_w) >> 10        text centre, active coordinates
cy     = (V Pos * meas_h) >> 10
origin = centre - half extent          (negative = scrolled off left/top)
```

All three products run on **one shared serial shift-add multiplier** (10
cycles each) because per-frame cones are still fully STA-timed at the pixel
clock. The negative-origin preload needs `|origin| / scale`, done by repeated
subtraction — bounded at 64 iterations horizontally and 9 vertically because
`|origin| <= half_extent` (`scale * 64` wide, `scale * 9` tall). The sequencer then walks all 32 cells
recording the last non-space per line, which is what Justify=Centre needs.
Whole run is ~130 cycles against ~38 000 clocks of SD vblank.

## Controls
- **K1 H Position** — text centre across the active width (50% = centred)
- **K2 V Position** — text centre down the active height
- **K3 Style** — Plain / Drop Shadow / Black Edge / Hollow / Emboss / Box /
  Half Box / Reverse
- **K4 Color/Erase** — Edit Off: 1 of 8 inks (latched). Edit On: drag the
  cursor and blank every cell it passes.
- **K5 Cursor** — non-destructive cursor over 32 cells (line, column)
- **K6 Letter** — character code (0..63) written at cursor in Edit Mode
- **T7 Edit Mode** — Off / On (enable cursor + writes)
- **T8 Background** — Video passthrough / Solid black
- **T9 Matte** — Off / On (text glyphs reveal incoming video)
- **T10 Font** — Default lo-fi 5×7 / Sci-Fi Bank-Gothic
- **T11 Justify** — Left / Center (per-line, rescanned every field)
- **P12 Size** — screen-relative scale, caption → hero type

Presets: Lower Third, News Bar, Hero Card, Video Fill, Sci-Fi Emboss.

## v2.1.0
Panel styles (Box / Half Box / Reverse) size their mask to the lines that
actually hold text, instead of always spanning two lines.

## What changed from v1.0.0 (and why)
Four of these were latent hardware faults no simulation would have shown:

1. **Blanking gate added.** v1.0 wrote ink unconditionally, and since the
   position counters ran through blanking a text block near the picture edge
   could paint into the blanking interval — rail-to-rail colour corruption
   over the whole frame.
2. **Palette U/V swapped at compose.** v1.0's palette was authored in
   standard BT.601, which renders with swapped hues on this hardware (red
   reads as cyan). Constants stay standard; the swap happens once, in S7, and
   only on generated colour — passthrough and matte are untouched.
3. **Vsync serration guard.** `frame_counter` and the vblank sequencer fired
   on every vsync falling edge; analog vsync serrates, so the cursor rainbow
   and the underscore flash would have run at several times the intended
   rate. Both are now gated on `saw_active`.
4. **Screen-relative placement.** v1.0 hard-coded a 4092-pixel span for the
   position knobs. On SD (858 clocks/line) that crammed all usable placement
   into the bottom ~20% of the knob; even HD only used the bottom ~27%.
5. **Vertical DDA advances on active lines only**, so interlaced fields stay
   in step.
6. **Drawn luma capped at 768** (was 940, which clips RGB and kills chroma).

## Status
**HW-validated** (v2.0.0 confirmed working on hardware). v2.1.0 adds the
line-aware panel extent and is timing-closed but not yet flashed. 6/6 configs
pass; ~3321/7680 LC total (program alone ~1250 LUT4 / 900 FF / 3 EBR). There
is a lot of room left for a follow-up: roll and crawl motion, a blink
attribute, more than two lines, or filling the 12 spare glyph slots (the
charset uses 52 of 64).
