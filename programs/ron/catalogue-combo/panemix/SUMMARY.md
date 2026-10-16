# Panemix — summary

Catalogue combo #105 (seed 20260925): split-screen / quad-mirror panes +
multi-source mixing (blend modes) + thermal-camera false colour. v0.1.0
(2026-09-12) was flashed and liked ("panemix is good"), with no later
hardware notes. v0.2.0 (2026-10-13) is the first finalizing pass, built from
a review of v0.1.0. The v0.1.0 sources and vmprog are in git 3fa438d and
`.panemix_work/v010/`.

## What v0.1 actually did on hardware

- **It opened on Add, not Difference.** The firmware loads a labelled
  default as its label *index*. Blend's default "Difference" (index 3)
  arrived as value 3, which the eighths decode reads as Add. So the look
  you liked at load was Add with the screen saying "Difference". v0.2 keeps
  Add as the default and labels it honestly.
- **The Blend labels lied over most of the knob.** The display splits the
  travel into fifths; v0.1 decoded eighths (Add ¼, Screen ¼, Multiply ⅛,
  Difference ¼, Overlay ⅛). At ~450 the screen said Multiply while you got
  Screen, and so on.
- **K1 Panes was dead with S7 Mirror off.** Unmirrored panes read their own
  screen columns, so the boundaries were invisible. The X-ray Wipe preset
  (Mirror off) showed no panes at all. S7 was also dead at 1 pane.

## Tiered review

### Tier 1: done in v0.2.0 (keeps the look)

Bugs and dead controls:
1. **Blend decode = the labels.** Fifths now, with ±10-count hysteresis so a
   pot sitting on a boundary can't flicker between modes. Default label is
   now Add (what actually ran).
2. **K1 alive at both S7 positions.** S7 is now Style: Repeat / Mirror.
   - Mirror is v0.1 exactly (odd panes mirrored in place).
   - Repeat (was the dead "Mirror off"): every pane repeats the centre 1/N of
     the picture, and Wipe scrolls the strip like film.
   - At 1 pane, Mirror flips the whole picture, so S7 is live at every
     pane count.
3. **K1 labels "1"–"4"** (the linear 1..4 readout rounded differently from
   the quarters decode). The labelled-default trap is guarded: the index
   default is honoured only until the knob first reads ≥ 4.
4. **Isotherms are real black.** They kept the palette's chroma: Y 64 with
   saturated U/V, a dark coloured line, not black.
5. **Isotherms no longer black out whole areas.** At low Range, the
   fixed-width window covered everything, so the screen flashed black as
   Heat cycled past a stop. Flat areas (zero difference, clipped white) also
   went black in Difference mode. Now the window width follows Range
   (constant thickness in luma), and flat or clipped pixels never draw one.
6. **No black seams from the capture's side columns.** Reads are clamped
   12 px inside the line, the edge-fill convention. On analog sources, the
   black blanking columns showed as a stripe at the Wipe wrap point and down
   every mirror seam: at 2 panes, that was a stripe in the middle of the
   screen.

Hardware contracts:
7. **Generated avid** (infrared's HW-verified copy of cubist/aurora): the
   program draws its own colour, so a source avid that wanders a clock
   would swap U/V on that line (thin coloured lines).
8. **HD HDMI bar fix:** [1 2 1] on Y/U/V plus luma scaled to white ~775,
   before the gate. Heat cycling drags the palette's full-swing wrap edge
   across the picture, which is the bar trigger. The kept luma (S8 Keep)
   could also reach 1023. It is now capped at 940, then scaled.
9. **Width latched per field.** The wrap point and the pane boundaries no
   longer jitter with analog line-length wander. All per-frame values
   latch on the guarded vsync edge.
10. One latency (25 → 31 clocks) in all modes. `no_rw_check` is on the two
    luma line buffers.

Proven upgrades:
11. **8 × 256 palettes** (infrared v1.0.0's tables, HW-signed-off): Thermal,
    Ironbow, Plasma, Rainbow, Ice, Night Vision, White Hot, X-Ray. The four
    v0.1 palettes are bit-exact at their old 64 sample points and
    interpolated in between, so the 64-step banding is gone.
12. **P12 Heat squared.** At the bottom of v0.1's linear curve, one step was
    an 18-minute lap, and 100% was 4.3 s. Now it is still at 0, ~1 min a lap
    at 10%, ~2 s at 50%, and ~0.5 s at 100%, infrared's climax curve.
13. **Range 100% = the whole ramp.** v0.1 indexed absolute luma, so black
    sat at entry 4/64 with a faint tint and the ramp's start was never
    reached. The index is now black-relative, ×17/16: black → the ramp
    start, the top of the blend → its end.

Optimisation/cleanup: the chroma line buffers and the mix path were dead
(the palette supplies all chroma; yosys had already pruned them), so they
are deleted. The pane geometry is now per-pane constants built in vblank
(one add per state), so the pixel path is compare → mux → add → clamp. The
blend products and the Mix/Range products are split hi/lo. W/3 is a
shift-add series instead of a ×21845 constant multiply.

Kept on purpose: operand B (the live picture) is one pixel ahead of A (the
line-buffered panes), as in v0.1. At Wipe 0 this makes Difference outline
edges in both directions instead of only horizontal ones.

### Tier 2: look changes (your pick)

- **Warhol panes:** each pane gets its own palette (K5 + pane number), a
  4-up Warhol print. Needs a switch: S10 Invert is the weakest candidate.
- **True quad (2×2):** below mid-screen, the pane parity flips, so mirrored
  and plain panes form a checkerboard. Could be a 5th Panes position
  ("Quad"). It costs a field-latched height and one row flag.
- **Wipe that moves:** K2 bipolar speed, with the centre still, so the panes
  glide continuously. Or a switch that turns K2 from position into speed.
- **P12 climax beyond speed:** Heat could also push Range toward 200% (more
  laps on screen as it speeds up). Or use infrared's Cycle switch, where
  P12 is a glided heat *position* when Cycle is off.
- **Per-pane blend mode:** each pane takes the next blend mode round from
  K3, so 4 panes show 4 modes at once.

### Tier 3: polish / later

- Capture defaults and presets from the device (`./capture_preset.sh`). The
  six presets are hand-written: v0.1's three remapped to the new zones, plus
  Rainbow Film and Night Flip.
- Wipe deadband or glide, if pot noise shows as a 1–2 px horizontal shimmer
  of the panes (one step ≈ 1.9 px at HD).
- Original colour under the palette luma (the inverse of S8 Keep). Not
  possible without a chroma ring (≈2 EBR) or a deep raw-chroma FF pipe.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Panes | 1 / 2 / 3 / 4 vertical panes (quarters; default 2) |
| K2 Wipe | panes, and what they show, slide across the screen (0–100% of the width, wraps) |
| K3 Blend | Add / Screen / Multiply / Difference / Overlay of panes (A) over the picture (B), luma; fifths with hysteresis |
| K4 Mix | 0–100% of the blend mixed onto the picture |
| K5 Palette | Thermal, Ironbow, Plasma, Rainbow, Ice, Night Vision, White Hot, X-Ray (eighths, hysteresis) |
| K6 Range | gain into the palette 0–200%; 100% = black..top of the blend over one ramp; above, the ramp wraps |
| S7 Style | Repeat: centre crop in every pane (filmstrip) / Mirror: odd panes mirrored in place; 1 pane: whole picture flipped |
| S8 Luma | palette luma / the blend's own luma (capped 940) under palette colour |
| S9 Index | palette by the mixed luma / by \|A − B\| |
| S10 Invert | palette reversed |
| S11 Iso | true-black isotherm lines at each eighth of the ramp, width follows Range, none on flat/clipped pixels |
| P12 Heat | palette cycle speed, squared: still at 0, ~0.5 s a lap at 100% |

## Hardware checks for v0.2.0

1. **Default:** does it still look like the v0.1 you liked? It should: Add,
   2 panes, Mirror, Thermal. Whites are dimmer (~775) and edges a touch
   softer ([1 2 1]). Gradients should be smoother (256-entry palette).
2. K3 sweep: the label shown should always be the mode you see.
3. S7 Repeat with K1 2→4: repeated centre crops; K2 should scroll them.
4. K1 = 1 + S7 Mirror: the whole picture flipped against itself.
5. S11 Iso at Range 0–20% with Heat cycling: no whole-screen black flashes.
   Lines should be black, not coloured.
6. Analog source + Wipe / Mirror: no black stripe at the wrap or the mirror
   seam.
7. P12 sweep: speed should ramp gently, then race at the top.
8. HD HDMI: any thick bars? Analog colour source: any thin coloured lines?

## Build

`./build_combo.sh panemix` → `out/rev_b/ron/catalogue-combo/panemix.vmprog`.

v0.2.0 (2026-10-13): all 6 configs pass on seed 1 (router2 default).
HD Analog 93.8, HD HDMI 94.2, HD Dual 88.9 MHz; SD Analog 87.8, SD HDMI
89.2, SD Dual 83.7 MHz. 4,610–4,624 LC (60%), 22/32 EBR (10 luma line
buffers, 12 palette ROM). Standalone yosys: 2,090 LUT4,
`emulate_transparency` 0. Not simulated, per the build-first rule. A copy of
the vmprog is in `.panemix_work/panemix_v020.vmprog`.

v0.1.0 (2026-09-12): HD 74.48 (seed 2) / 75.51 (seed 4) / 80.98 (seed 4) MHz
(analog / HDMI / dual), ~4,000 LC (52%), 12 EBR.
