# Heirloom (started 2026-10-20)

One Videomancer program per still photo of Ron's old modular video synthesizer. Each photo is examined and broken
down (what the patch is doing: its shapes, colours, motion cues, textures, the signal behaviour it implies), and the
program is designed from that breakdown and named after what the picture shows.

Flash from `out/rev_b/ron/heirloom/<name>.vmprog`; rebuild one with `./build_heirloom.sh <name>`.

## Layout

```
programs/ron/heirloom/
  README.md          this file: rules + the photo -> program index
  photos/            drop zone for source photos (JPG/PNG; git-ignored, stays local)
  <name>/            one folder per program
    <name>.vhd
    <name>.toml
    SUMMARY.md       source photo, the breakdown, design mapping, controls, build/HW status
```

Windows path to the drop zone: `\\wsl$\Ubuntu\home\ron\videomancer-dev\programs\ron\heirloom\photos`

## Per-photo workflow

1. **Breakdown**: view the photo and write down what it shows (composition, dominant shapes and how they repeat,
   palette, edge character, implied motion, which classic modular-video process made it, such as a colouriser,
   keyer, ramp/oscillator mixing, feedback or a raster scan). This goes at the top of the program's SUMMARY.md.
2. **Name**: taken from what the picture shows, one word if possible. It must not collide with any existing
   `programs/ron/<name>` or `programs/ron/catalogue-combo/<name>`, because the build links the folder in by name.
3. **Design**: decide what the photo's look becomes as a program, and what each of the 12 controls does.
4. **Build**: `./build_heirloom.sh <name>`, then report post-route Fmax/LC per config. Ron flashes and judges.
5. **Index**: add a row to the table below.

## Rules for every Heirloom program

- **All 12 controls are used**: K1-K6 (`registers_in(0..5)`), S7-S11 (bits 0-4 of `registers_in(6)`),
  P12 (`registers_in(7)`). No unused or placeholder control.
- **P12 slider = the most prominent effect**: the headline performance control. Its full travel must stay
  visibly alive, and 100% must be a designed climax, not math degenerating.
- Every control visibly changes the picture at any setting of the others (none gated behind another's mode);
  switches are instant look-flips.
- Reproduce the photo's LOOK, not its pixels. The program should be able to land on the photographed scene and
  then perform around it. Re-render structurally (shapes, lines, geometry), never a coarse overlay.
- HD-first (1080p @ 74.25 MHz, no clock divisor), the same picture at SD/720p/1080p (virtual 1080-line
  coordinates), and every hardware contract in the repo-root CLAUDE.md (blanking gate, one latency, generated
  avid for drawn colour, luma cap ~768, chroma neutral 512, U/V swap).
- `program_type = "processing"` if `data_in` is read in any mode; `synthesis` only for pure generators.

## Index

| Photo | Program | What it shows | Status |
|-------|---------|---------------|--------|
| WIN_20220629_23_49_20 | [escalator](escalator/) | picture tiled ~7 across, pastel rows with a vivid pink band, green stepped diagonals marching down the grid | v0.2.0 (Source switch) built 2026-10-20, 6/6 (HD Dual seed 2), HD 80–87 MHz, 5.0k LC, 15 EBR; v0.1.0 previewed |
| WIN_20220829_23_30_37 | [cocoon](cocoon/) | stretched hexagon of woven chevrons in a red→blue radial glow, posterized video keyed in and torn at the flanks | v0.2.0 (Source switch) built 2026-10-20, 6/6 (HD Dual seed 2), HD 75–80 MHz, 4.45k LC, 19 EBR; v0.1.0 previewed |
| WIN_20221122_23_37_36 | [starburst](starburst/) | 4-fold mirror kaleidoscope of saddle contour bands (gray X's, CMY chevrons), serrated edges | v0.2.0 (Source switch) built 2026-10-20, 6/6 seed 1, HD 79–80 MHz, 4.6k LC, 17 EBR; v0.1.0 previewed |
