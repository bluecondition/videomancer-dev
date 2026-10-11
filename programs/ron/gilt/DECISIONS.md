# GILT v2: decisions

Newest last. Each entry: what was decided, why, and who decided.

**D1 (2026-10-10, Ron).** v2 is designed from scratch from `GILT_V2_PROMPT.md` alone. No
references to the original GILT program or its development history. No v1 code,
constants or primitives are reused.

**D2 (2026-10-10, Phase 1).** Texture, band energy and fleck statistics are measured at
"1080-equivalent", meaning the painting's short side shown as 1080 lines. They depend
strongly on resolution: the same Kiss field read 2× the grain at 2048 px. Sizes are
reported in px1080 and as % of the robe width.

**D3 (2026-10-10, Phase 1).** Colour source of truth = sRGB of the measured anchors in
`style_constants.json`. CIELAB is used for analysis; BT.709 10-bit legal Y'CbCr only for
reporting against the prompt's §3.2. Hardware constants get generated from sRGB in Phase 4.

**D4 (2026-10-10, Phase 1).** Pixel-family classifier thresholds as documented in
`analysis/lab.py`. One deviation from a plain hue split: hue 40–58° with C* ≥ 40 counts as
red enamel (vermilion), because the measured hue distribution is bimodal. Enamel peaks at
40–50°, golds at 65–90°.

**D5 (2026-10-10, Phase 1).** Adele I's cells are measured by hand. Three automatic
methods (colour classes, leaf ground, enclosed-by-outline watershed) all produced arbitrary
blobs on her gold-on-gold cloisonné.

**D6 (2026-10-10, Phase 1).** Cell statistics count only cells that contrast with the leaf
around them. Gold-on-gold cells that read only by engraved line are not counted.

**D7 (2026-10-10, Ron).** Zone polygons in `analysis/zones.json` approved as drawn. The
per-zone measurements in STYLE_SPEC / style_constants are final for Phase 1.

**D8 (2026-10-10, Ron).** Skip the Phase 2 model, contact sheets and approvals (Phase 3).
Go straight to Phase 4: build VHDL, preview on hardware, tweak from there. No simulations.

**D9 (2026-10-10, Ron).** All six STYLE_SPEC §10 recommendations accepted:
1. Two golds per bank. KISS: dim brass field, bright saturated figure gold. ADELE:
   saturated orange field, paler higher-contrast ornament.
2. Field texture at the measured strength (grain σ ≈ 27–35 Y10, ~6% gold flecks of
   2–4 px; Kiss finer and sparkier), tuned on hardware.
3. Field stays calm in both banks. No field spirals.
4. Dark outlines 1–3 px. No rim on Kiss bars, black borders on tablets, discs drawn as
   rings. No pale-gold rim per cell; keep the pale aura line around the robe mass.
5. Cell statistics per bank (Kiss small and tall, Stoclet large and wide-spread, Adele
   eyes and triangles).
6. Composition defaults: robe 30–60% of the frame, field 30–45%, jewels ~1%, black 1.5–3%.

**D10 (2026-10-10).** v1 files moved to `legacy/gilt_v1/` (with its work backups) so v2
owns `programs/ron/gilt/`. v2 keeps program_id `com.ron.gilt`.

**D11 (2026-10-10, Phase 4 v2.0.0 fit).** First build was 9,271 LC (120% of the
HX4K). To fit (now ~7,470 LC, 97%), these spec features were cut or simplified:
1. Jewel colour no longer follows the video's hue (§5.6). It is hashed per cell, and
   JEWEL sets the budget only. This removed the chroma columns of the coarse map.
2. Cameo size comes from FIGURE and the frame size (W/8–W/2 × H/6–H/2), not measured
   skin extents (§5.2/§5.3). The centroid is still measured.
3. KD subdivision is 1 level per tile, not up to 4 (§5.4). The cell-size spread comes
   from hashed tile widths and band heights (per-bank LUTs) plus the split.
4. The Adele bank's tesserae-triangle ground is gone (plain cells are plain leaf). The
   Stoclet oval is a disc with engraved rings and an eye slit, on red-orange enamel.
5. Not built: histogram auto-level (§5.2), video-driven jewel hue, stacked triangles,
   rosettes, eye-in-triangle of varied orientation, sub-KD checker blocks (the checker
   is regular 8/16 px squares with occasional gold).
6. Kept: two golds per bank, field grain + flecks at measured strength, flesh keying
   with the cameo gate, robe/field zones from detail + skin proximity, the aura line,
   relief + glint, meadow, the GILD staging, and all 12 controls.

**D12 (2026-10-10).** Timing structure: the beam position is a registered counter with
one-bit flags (pixel 0, inside the line, line end) computed a clock ahead, and the
vblank divider is a shift-register type. Both were critical paths (56.7 → 79.5 MHz on
HD Analog).

**D13 (2026-10-10, Ron, after flashing v2.0.0).** "Neither is good on its own." Combine
v2's colours and iconography with v1 (v0.10)'s gold/brown colour and its smeared
sections. Ron confirmed the smear means the gold-painted picture: the image itself
shown as smooth gold-brown paint, with its own light and shade and no pattern. And the
placement was "weird": it must follow the picture's luma/chroma, not an arbitrary
shape. This supersedes D1 for these elements. v2.1.0:
1. Gold paint: wherever there is no ornament, per-pixel luma goes through v0.10's 32-step
   gold/brown ramp. It is palette entries 0-31 on every page (LEAF tints it; the
   default LEAF reproduces v0.10 exactly; Adele is 6° more orange). Grain is halved
   there so the picture reads.
2. Placement from the picture: the face-centroid proximity (and the centroid, its
   divider and the Cameo box) are gone. The coarse map stores mean chroma again
   (Y5 det3 Cb4 Cr4). Each tile is ornament where texture + colourfulness beat
   ORNAMENT, else gold paint. The motif family comes from the block's colour class
   (warm / cool / green / neutral) and luma band. Kiss: warm → ring-discs, cool →
   tablets and bars, green → engraved gold, neutral → dark bars / bright tablets.
   Adele: warm → red concentric squares and ovals, cool → eyes and spirals, green →
   spirals, neutral → dark bars and checkers / bright eyes and spirals. 1 cell in 4
   borrows another family. Jewel colours follow the class too.
3. Robe leaf is the same ramp, 4 steps brighter than the block's paint; motif gold
   fills are 8 steps brighter.
4. Cameo (S8) now means: flesh only inside blocks whose average colour is skin-like.
5. GILD lays the cells in hashed order, plain areas before busy ones.
Fit improved to ~89% LC.
