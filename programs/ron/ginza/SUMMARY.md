# Ginza — 1930s Japanese poster reprint

Reference: a Taishō/early-Shōwa department-store poster (five flat inks on cream
paper — grainy olive, navy, sky blue, dusty pink; thin tall pillars with striped
fronts; horizontal beams sticking out with lettering across them; big flat
textured olive rectangles behind; stacked columns of hand-cut kanji; black kanji
on cream bars; "ABCDEFGHI" on a navy beam; a solid blue disc with cream kanji; a
thick pink ring; the pillar motif repeated smaller in the background; slightly
misregistered plates).

## Status

- **v0.4.0 — built 2026-10-16 (see build log).** Ron flashed v0.3: "very
  glitchy and has those horizontal lines; I'd like a nice still image that
  doesn't glitch or move much". v0.4 makes the poster STILL:
  - **Cell memory** (2048×16 EBR, 16 rows × 128 cols): per cell a
    frame-to-frame one-pole luma (1/16 per frame, 10.2 fixed point) and its
    current ink. On each row's sample line (its last line → the row below;
    the frame's first line → row 0) every cell start does a pipelined RMW:
    read, blend the fresh 8-px sample, re-classify with **±24 hysteresis**
    against the stored ink, write back. The row's hold stream is the new
    ink, so a cell flips only when its smoothed luma is clearly past a cut.
  - **Vblank walker**: after the guarded vsync edge, fills row 0 of the hold
    and beam RAMs (one write per column from cell row 0) and rebuilds the
    64×32 echo grid (32 rows × 128 sub-steps × 2 clocks, ≈ 10k clocks) from
    the cell memory — nothing is sampled live any more; row 0 shows the
    previous frame's (stabilised) cells. The walker owns the cell-memory
    read port and the hold/beam write port (single muxed write statement).
  - Output smoothing doubled: two [1 2 1] passes (= [1 4 6 4 1]) on Y/Cb/Cr
    before the luma scale, against the HD HDMI bars; latency 36 → **40**.
  - Grid clamped to b ≥ 17 px (cols ≤ 128); S7 is now **Thin / Wide**
    pillars (b or 2b wide, both 8b tall) — Beams mode would exceed 16 rows.
  - Sim caveat: the walker needs ≈ 10k vblank clocks; the image tester's
    fake 8-line vblank is too short below 1080p/720p decimation 1.
- v0.3.0 (2026-10-16, flashed — glitchy): Ron's second review: olive is
  the GROUND (large flat textured rectangles, never text, never a structure);
  thin tall striped pillars (blue | cream + navy kanji column | pink); navy
  cells become cream text bars that launch horizontal navy beams with cream
  ABCDEFGHI across them. Build numbers: end of this file.
- v0.2.0 (2026-10-16): caps, echo, disc + ring, text bars, two-scale grain.
  6/6 seed 1; HD 100.4 / 93.0 / 91.6 MHz; 4970 LC (65 %), 13 EBR.
- v0.1.0 (2026-10-16): first pass. 6/6 seed 1 except sd_hdmi seed 2 (router2
  stall); HD 86.0 / 92.7 / 103.4 MHz; 3880 LC (50 %), 10 EBR.

## How it works (one clock per pixel, no framebuffer)

1. **Analysis** — luma box-averaged over 8 px, cut into five ink bands
   (cuts 170/470/590/710 + Ink bias ±256). Bands → structures: paper =
   nothing; pink / blue = striped pillar; olive = ground; navy = text bar +
   beam.
2. **Cells** — grid cw × ch (Grid = 9..122 px at HD; Pillars = b × **8b**,
   Beams = 4b × b). A row's inks are sampled from the first 8 px of each cell
   on the **last line of the previous row** (and the frame's first line) into
   a 2048×4 line RAM **and its twin for the beams**; every line of the row
   reads them, at any column offset. Single-bank: the write line reads the
   old row at x while writing the new row at x − 5.
3. **Caps** — first K = b/2 lines of each row: the hold RAM is read at
   x − (K − j)·slant, shearing the cell outline into a parallelogram leaning
   into the depth direction, in the cap ink (pink; blue for pink cells). Caps
   never restart the extrusion age. Olive cells get no cap.
4. **Pillars** — blue/pink cells, front lines: stripes by cell phase —
   left quarter own ink, middle half cream with a 32-px column of navy kanji
   (one per 32-line glyph row, stacked; centred in the stripe, clipped if the
   stripe is narrower), right quarter the cap ink.
5. **Text bars + beams** — navy cells: cream with the same centred navy kanji
   column. On the beam band (lines [ch/2 − b/2, ch/2) of every row) a
   countdown of 3·cw px starts at each navy cell's left edge read from the
   twin RAM → navy beam with cream `ABCDEFGHI2020202` across it (32-px
   letters, vertically centred in the beam, clipped if the beam is thin). The
   beam's own cap (b/4 lines above it) reads the twin RAM sheared by
   (Kb − j)·slant. Beams draw over pillars and caps.
6. **Extrusion** — 2048×16 line RAM {ink(3), age(9)} per column, read at
   x + slant on the previous line. Pillar / bar / beam front → age = Depth,
   ink stored; paper or olive with age > 0 → side ink (navy; blue for navy
   fronts, so beams and bars get blue sides), age − 1. Written 5 clocks after
   the read issue, |slant| ≤ 4 → `no_rw_check`. First active line forces
   age 0. Stripes (S9) = side/own ink alternating on age(2).
7. **Olive ground** — drawn only where nothing else is (after sides and the
   echo), never restarts the age, stippled by the grain.
8. **Echo** — 64×32×3-bit grid RAM (2 EBR) of the fresh per-cell stream
   (x-DDA 64× per line, y-DDA 32× per field phased half a row in), shown at
   half size in the upper right behind the live structures: flat rectangles
   with half-Depth extrusions (echo olive = ground).
9. **Lettering** — three ROM reads of two 256×16 glyph ROMs (`ginza.py`;
   3-px-stroke kanji-like and Latin/digits). Kanji column (pillar stripe and
   bar, one read, two bit selects), Latin on beams, 128-px kanji for the disc.
10. **Grain** — two static shift-add hashes (fine 2 px, coarse 8 px; each
    spot-checked, neighbour correlations < 0.08) averaged: clumpy printed
    stipple; paper holes in olive (full) and on faces (quarter).
11. **Circles** — r² by second differences. Disc at (0.30 W, 0.42 H), R from
    Circles: solid blue with cream kanji. Ring at (0.20 W, 0.80 H), 0.44 R..
    0.63 R, pink. Both over everything, before the plates.
12. **Plates** — 17-deep ink shift register; blue tapped +k, pink −k,
    navy/olive centre; print-order composite with violet overprint. Night
    swaps paper ↔ navy after the composite.
13. **Output** — 6-entry BT.601 palette (poster-sampled; swapped at compose),
    damier's HD-HDMI conditioning, neutral outside the generated avid (aurora
    recipe). Latency 36.

Per-frame products (12, one serial shift-add multiplier, ~170 clocks after the
guarded vsync edge): cell unit, depth, R, R², cx²/cy² ×2, Ro², Ri², cap shear
start K·|slant|, beam cap shear start Kb·|slant|.

## Controls

| | | |
|---|---|---|
| K1 | Ink | band cuts: more paper ↔ more navy |
| K2 | Grid | cell size (default ≈ 47 px pillars at HD) |
| K3 | Slant | depth direction −4..+4 px/line (caps, beam caps, bodies) |
| K4 | Register | plate misregistration 0..8 px |
| K5 | Circles | disc + ring radius, 0..H/2 |
| K6 | Grain | olive / face stipple density |
| S7 | Blocks | Pillars / Beams (cell aspect) |
| S8 | Echo | background copy Off / On |
| S9 | Faces | Flat / Striped |
| S10 | Lettering | Off / On |
| S11 | Paper | Cream / Night |
| P12 | Depth | 12 lines .. half the screen |

## Open questions for hardware

- Cell flicker on noisy sources (one 8-px sample per cell per frame).
- Beam length (3 cells) and height (b/2) are fixed fractions; may want a
  control or a luma-driven length.
- Whether cap and body leaning the same way reads as a prism (one sign in
  `ext_ra` flips the body).
- 1080i sources: field_n tied high on HD inputs → glyph rows / circles
  computed as top-field on both fields.

## Build log

- v0.4.0 (2026-10-16, router2): hd_analog 83.2 MHz · hd_hdmi 80.6 · hd_dual
  80.0 · sd_analog 71.3 · sd_hdmi 72.2 (**seed 2** — seed 1 router stall) ·
  sd_dual 74.8; ~6345 LC (83 %), 23 EBR; `out/rev_b/ron/ginza.vmprog`
  417321 bytes.
- v0.3.0 (2026-10-16, router2): hd_analog 89.2 MHz · hd_hdmi 95.2 · hd_dual
  93.2 · sd_analog 91.8 (**seed 2** — seed 1 router stall) · sd_hdmi 79.2 ·
  sd_dual 91.2; ~5590 LC (73 %), 15 EBR; `out/rev_b/ron/ginza.vmprog`
  369379 bytes.
- v0.2.0: 6/6 seed 1; HD 100.4 / 93.0 / 91.6; 4970 LC, 13 EBR.
