# GILT: decisions (v2 and v3)

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

**D14 (2026-10-11, Ron, new brief: "one final try").** Go back to one of the old
versions that looked "very warpy and oily and swirly", and draw the blobs of colour,
symbols, squares, dots, spirals etc. as a texture chroma/luma thing like Sugarcoat: a
procedural texture generator chosen by luma or chroma on a switch, with degrees of
contrast and posterization. v3.0.0, built on that:
1. Base = v0.10.0 (HW-approved 2026-10-09 "I like the organic look now"), the most
   warped version Ron approved: its blurred content field drags the ornament space
   (luma sideways, chroma up/down, S8 Flow swaps), its two parabolic-sine waves and its
   large per-line sway bend it, and its 32-step gold/brown ramp paints the picture. Its
   Worley cells, motif engine, Luma overlay, flesh key, halo, meadow and Gild mixer
   are gone. v0.11's large generated zone wave stays rejected.
2. Drawing = Sugarcoat's mechanism: classify (luma band, or hue octant with grey
   falling back to luma) from the blurred field, then a 16 x 16 theme ROM picks one of
   16 Klimt textures, all evaluated in the warped space. K1 Theme (16 labelled), K2
   Poster (bands 8..2, paint 32..2 tones, hues 8/4/2), K4 Contrast (Sugarcoat's
   proc-amp, also on the paint), S7 Map.
3. P12 = SWIRL (drag up to 2x v0.10, waves and sway up to 3/4 cell, drift rate): the
   oil is the headline. K5 Ink (paint -> ink 4/16..16/16), K6 Density (cells drawn),
   S9 Invert (gold negative), S10 Outline (texture-boundary contour), S11 Jewels.
4. Ground = gold-painted picture everywhere (D13 kept); no raw video reaches the output.
5. Hardware contracts added that v0.10 lacked: blanking gate, generated avid, HD HDMI
   bar conditioning ([1 2 1] + luma scale), serration-safe field event.

**D15 (2026-10-11, Ron, after flashing v3.0.0).** "This is ok but where's the brown and
gold tones?" Cause: the textures covered the gold paint with flat warm blacks, bone and
jewels at 75% (v0.10 looked gold because its ornament was sparse). v3.1.0: dark and light
inks become steps along the same gold/brown ramp, relative to the paint underneath (dark
= paint − Ink − 0..3 per cell, light = paint + Ink + 0..3), so the ornament is
tone-on-tone gold and brown that keeps the picture's shading. The black and bone
palettes are gone. Jewels are the only non-gold colour, mixed over the gold at Ink.


**D16 (2026-10-12, Ron, HW).** "It all still looks blue." v3.0/3.1 output the usual
hardware U/V swap (internal Cr on the u wire), exactly like v0.10, cubist and v2.1, and
the headless sim showed the same blue (the sim decodes standard order). So on Ron's
setup this program needs STANDARD order out. v3.1.1 drops the output swap. The input
swap (used only for the Chroma-map hue classes and the drag axes) is unchanged. If the
Chroma map's hue assignments look wrong (skin should stay plain gold), flip that too.
Open question: why the usual swap is wrong here. Candidates: the generated avid's
parity (on paper it keeps the source's), or a setup change (HDMI vs analog out).
UPDATE: v3.1.1 (standard order out, sim-verified gold) was still blue on hardware, so
both orders gave blue, which a fixed swap cannot do. The builder's HD placements had
HSYNC in the encoder-skew danger zone (v3.1.1 HD HDMI +0.74; v3.1.2 HD HDMI −0.06, HD
Analog +0.36 with data straddling the clock). An HSYNC on the wrong side of the
encoder's sampling edge shifts its Cb/Cr count by one pixel, which swaps the whole frame.
v3.1.2-diag: S11 = live U/V order test (Jewels forced on), HD Analog s33 / HD HDMI s6 /
HD Dual s1 screened clean. Ron's A/B decides the order; then S11 goes back to Jewels.

