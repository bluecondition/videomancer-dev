# Titler — lo-fi 16-character video titler with on-the-fly editing

## What it does
Renders two lines of 16 characters each (32 cells total) over the input
video. Knobs control horizontal / vertical position, integer scale (1..32x),
and one of 8 preset text colours. An Edit Mode exposes a single editing
cursor that walks across the 32 cells (K5); K6 picks the letter under the
cursor and K4 doubles as an erase-and-scroll knob. A Matte toggle reveals
the incoming video through the text shapes — combined with a black
background that gives video-filled letterforms on black. A Font toggle
chooses between the default lo-fi 5x7 face and a chunkier Sci-Fi/Bank-Gothic
alternative. The font itself is built by `titler.py` into
`titler_font_pkg.vhd` as a LUT-ROM.

## How it works
Processing program. A 5-stage pipeline (S1..S5) maps each physical pixel to
a glyph-pixel index via Bresenham-style position accumulators: the
accumulator advances by `step = floor(2^16 / scale)` per pixel (and per
line), so any integer scale 1..32x is supported with no multiplier — only a
32-entry step LUT. S2 extracts char/line/glyph-x/y; S3 reads the 32-entry
text buffer and detects the cursor box; S4 reads the 4 kbit font LUT-ROM
(64 glyphs x 8 rows x 8 bits); S5 muxes text vs matte vs background. Glyphs
are stored shifted right by one column inside an 8x8 cell so column 0 is
blank, columns 1..5 are the glyph, columns 6..7 are inter-character gap —
that gives the cursor box clean room to draw around any letter.

## Controls
- **K1 H Position** — horizontal text origin (0..~2046 px)
- **K2 V Position** — vertical text origin (0..~2046 lines)
- **K3 Size** — integer scale 1..32x
- **K4 Color/Erase** — Edit Off: pick 1 of 8 preset colours (latched). Edit On: drag the cursor and blank every cell it passes.
- **K5 Cursor** — non-destructive cursor over 32 cells (line, column)
- **K6 Letter** — character code (0..63) written at cursor in Edit Mode
- **T7 Edit Mode** — Off / On (enable cursor + writes)
- **T8 Background** — Video passthrough / Solid black
- **T9 Matte** — Off / On (text glyphs reveal incoming video)
- **T10 Font** — Default lo-fi 5x7 / Sci-Fi Bank-Gothic

(No T11, no slider, no presets.)

## Presets
None defined in the toml.
