# ATTRACT asset text-grid format

Every graphic is a text grid, one character per pixel:

  .  transparent / colour 0
  1  colour slot 1
  2  colour slot 2
  3  colour slot 3

The actual colours come from whichever 4-colour palette the tile/sprite uses
at run time, so the same art is recoloured freely.  Convention used across the
sheet: 1 = main body, 2 = shade/detail, 3 = highlight/eyes.

Each entry starts with a header line, then its rows:

  @tile NAME        8x8, addressable by the tile layer (first 256 patterns)
  @font HH          8x8 font glyph for ASCII code 0xHH (20..5F); also a tile
  @sprite NAME      16x16 sprite (stored as 4 patterns: TL, TR, BL, BR)
  @sprite8 NAME     8x8 sprite (1 pattern)

Lines starting with # are comments; blank lines are ignored.  NAME is
[a-z0-9_]+.  Rows must be exactly 8 (or 16) characters.
