# GILT v2 — Klimt Rebuild (Claude Code prompt)

*Save this file in the repo as `GILT_V2_PROMPT.md` and tell Claude Code: "Read GILT_V2_PROMPT.md and execute Phase 0." Or paste it whole.*

---

## 0. What went wrong last time (Ron fills in before sending)

- Version/commit that looked best so far: ___
- What looked wrong (grid? noise/boiling? yellow tint? faces patterned over? muddy edges? too busy? flicker?): ___
- What was reverted, and what the revert broke: ___

If this section is blank, ask me these three questions at the end of Phase 0, before writing any new code.

---

## 1. The ask

Rebuild the Klimt program for the **LZX Videomancer** from a clean slate. Program name stays **GILT** (v2). Incoming live video is re-rendered as a Gustav Klimt Golden Phase painting: faces and hands stay photographic (but painted-pale), everything else becomes gold leaf, cloisonné ornament, black-and-silver inlay, and rare jewel enamel — never a consistent grid.

The v1 implementation is **reference only**. Move it to `legacy/gilt_v1/` (or a branch). Do not port it forward wholesale. You may lift individual primitives (hash function, sync handling, parameter plumbing) only after Phase 0 confirms they work, and you must list each one you reuse in `DECISIONS.md`.

**Platform facts to confirm in the SDK and in my existing programs (PULP, FRINGE, IRIS, etc. in this repo) before designing:** VHDL; raster-scan with no full-frame buffer; 10-bit YCbCr; twelve parameters = 6 knobs, 5 switches, 1 slider (P12). Find the real BRAM/LUT/DSP budget and the pixel clock at the highest supported format. Mirror how my other programs declare parameters, package, and build.

**Format independence:** express every spatial size as a fraction of active width (or of a reference "1080-equivalent" pixel), never as a hard pixel count. Handle interlaced formats so ornament doesn't twitter between fields (derive hashes from frame-line, not field-line, coordinates).

---

## 2. Why a fresh start — likely root causes (verify in Phase 0)

1. **Ornament everywhere.** The analysis below shows Klimt's backgrounds are *texturally* shimmering but *structurally* empty. Dense ornament lives in the garments around the figure. Filling the whole frame with motifs produces noise, not Klimt.
2. **Power-of-two subdivision reads as a grid.** A hashed quadtree on a global lattice still puts cell boundaries on the same lines across the whole screen. The eye finds those lines instantly.
3. **One gold.** Klimt uses (at least) two: a dull, darker *ground* gold and a brighter, more saturated *figure* gold. With one gold everything sits on the same plane.
4. **Accents too common.** Measured jewel-tone coverage is about 1–3% of the canvas. More than that turns into confetti.
5. **Iterating in VHDL with no locked reference.** Without a software model and a saved gallery of approved renders, there's no way to tell better from worse, so changes regress. Phase plan below fixes this.

Mark which of these actually applied after the audit.

---

## 3. What Klimt actually does

### 3.1 Research grounding

- **Origin of the look.** In December 1903 Klimt studied the Byzantine gold mosaics of San Vitale in Ravenna; the Golden Phase (c. 1903–1909) grew directly out of that. His father was a gold engraver; Klimt trained in the applied arts. Think *goldsmith, enamel, mosaic* — not "painting with a gold filter."
- **Materials.** Real gold leaf and silver/platinum leaf over oil, with ornament raised in **gesso relief** (Adele I's bracelets and "AB" monograms are literally sculpted). In *The Kiss* the Belvedere distinguishes genuine gold leaf on the figures from **brass metal leaf over the entire background** — two different metals, two different golds. Fine gold and silver flakes are scattered over the background like a nebula.
- ***The Kiss* (1907–08, Belvedere, square).** Man's robe: tall black and silver/white **rectangles**. Woman's dress: **circles, ring-discs and flowers** in clusters of red, violet and blue. Both robes merge into one gold bell-shaped silhouette with a thin bright aura line. Only faces, hands and feet are flesh. They kneel on a **flower meadow** — a dense pointillist carpet of colored dots on green. No depth anywhere; everything flattens into pattern.
- ***Adele Bloch-Bauer I* (1903–07, Neue Galerie).** Most of the canvas is mottled, glittering gold with *no motifs*. Dress: Egyptian **eyes set in golden triangles**, plus dark vertical slits. Armchair: **spirals**. A halo of cells and spirals around her head. A panel of **red-orange squares with concentric squares** beside her. A few small silver rectangles float in the gold field. Lower left: a green floor with a **black-and-white checker trim**. Face, shoulders, hands: pale, naturalistic, with flushed cheeks, red lips, black hair. The face meets the gold with a **hard edge**.
- **Stoclet Frieze cartoons (1910–11, MAK).** *Tree of Life*: gold **spiral** branches on cream, Eyes of Horus, a black Horus falcon. *Expectation*: stacked **triangles** with striped bands and eyes. *Fulfillment*: robe of large **gold-ringed ovals with eye slits** on red-orange ground, small discs/flowers, and a **block of black and white squares of varied sizes** (not a regular checkerboard). Executed in enamel, mother-of-pearl, mosaic and gold — every element is outlined in gold like **cloisonné**.
- **Landscapes** (not this build — see §9): square format, horizon pushed to the top edge or gone, perspective tipped up flat, mosaic-like pointillist dabs.
- **Late period** (not this build): after ~1912 gold disappears; bright colors and East Asian motifs (Adele II, Friederike Maria Beer).

### 3.2 Measured from images

I ran pixel analysis on three reproductions: *The Kiss* and *Adele I* (small web copies, 224 px — fine for palette and coverage, too small for motif geometry) and the *Stoclet Tree of Life / Expectation / Fulfillment* cartoon (2953 px — used for cell geometry). Web reproductions vary in color grading (this *Kiss* copy runs olive), so **treat these as starting anchors and redo them at high resolution in Phase 1.** All YCbCr values are BT.709, 10-bit legal range (Y 64–940, Cb/Cr centered on 512).

**Palette anchors**

| Role | Source | sRGB | Y | Cb | Cr |
|---|---|---|---|---|---|
| Ground gold (brass leaf) | Kiss background | #927E4A | 499 | 413 | 556 |
| Dark ground | Kiss | #786330 | 408 | 414 | 558 |
| Figure gold | Kiss robe mass | #BDA552 | 628 | 357 | 568 |
| Bright figure gold | Kiss | #C5AA4D | 647 | 338 | 573 |
| Orange gold field | Adele background | #BB7B21 | 511 | 328 | 639 |
| Pale gold highlight | Adele | #DEA44A | 649 | 331 | 628 |
| Dark gold | Adele | #855313 | 371 | 379 | 611 |
| Umber shadow | Kiss / Adele | #392F1D / #623914 | 231 / 283 | 475 / 430 | 533 / 590 |
| Ink (near-black) | Adele hair, dress slits | #1F1012 | 133 | 509 | 538 |
| Silver / neutral | Kiss | #B0A485 | 631 | 451 | 537 |
| Flesh | Kiss woman's face | #AB9E84 | 611 | 460 | 538 |
| Flesh | Adele face | #AD8F85 | 576 | 482 | 566 |
| Meadow green | Kiss | #8D9C56 | 574 | 394 | 497 |
| Floor green (dark) | Adele | #3C430B | 275 | 416 | 509 |
| Cream ground | Stoclet cartoon | #EDDFB5 | 830 | 434 | 544 |
| Red-orange enamel | Adele square panel | #C37728 | 509 | 343 | 658 |

- Every gold lives in one CbCr quadrant: **Cb 328–475, Cr 530–640** (CIELAB hue ≈ 63°–95°, chroma 14–57). Brass (Kiss) sits at Cr ≈ 556–573; Adele's orange gold at Cr ≈ 606–639.
- **Figure gold is ~+130 Y and ~−55 Cb versus ground gold** in *The Kiss*. That brightness/saturation step is the depth cue that puts the figure in front of the ground.
- Flesh is **pale and low-chroma** (Cb 460–482, Cr 538–566) — much closer to neutral than the golds. Klimt's flesh is porcelain against metal.

**Coverage budgets**

| | The Kiss | Adele I | Stoclet cartoon |
|---|---|---|---|
| Gold/brown family | 79% | 92% | 82% |
| Near-black | 2.0% | 4.4% | 2.9% |
| Neutrals (flesh, silver, white) | 4.1% | 1.3% | 2.3% |
| Colored accents | 15.8% (93% of it is meadow green) | 2.8% | 13.7% (87% green) |

Non-green jewel tones (red, violet, blue) are **≈1–3% of the canvas**. Gold dominates; black is a small, sharp accent; color is a spice.

**Luma distribution (Y10 percentiles 2 / 10 / 50 / 90 / 98)**

- The Kiss: 260 / 404 / 509 / 656 / 704
- Adele I: 158 / 329 / 479 / 632 / 717

The Golden Phase lives in the mid-tones. Nothing approaches peak white except tiny silver highlights; true black only in small inlays and hair.

**Where structure lives (mean motif-scale band energy, Y10 codes)**

- The Kiss: background 12 vs man's robe 49 (≈4×)
- Adele I: background 16 vs dress 39 and halo 47 (≈2.5–3×)
- Fine grain in Adele's background (31) is much higher than in the Kiss background (12): Adele's gold *glitters*, the Kiss ground is a smooth dim nebula with sparse flecks.

**Cell geometry (Stoclet *Fulfillment* robe, 375 segmented cells, sizes as % of the robe's width)**

- Percentiles 10 / 25 / 50 / 75 / 90 / 99: **2.2 / 3.0 / 4.8 / 8.4 / 15.8 / 33**, largest ≈ 80%
- Roughly **log-normal**: above the median, count roughly halves each time size doubles. Many small cells, a few giants, everything in between.
- Aspect (h/w) percentiles 10 / 50 / 90: **0.46 / 1.06 / 2.5**
- Every large cell carries a **gold rim**; the ground between ovals is red-orange; the black/white square block contains squares of many sizes.

### 3.3 The rules (the design thesis — build to these)

1. **Zones before pixels.** Decide what each region *is* — FLESH, ROBE, FIELD (plus INK and MEADOW) — then render each zone by its own rules. Klimt composes; he doesn't filter.
2. **Ornament concentrates around the figure; the field stays calm.** Field areas get shimmer and flecks, never motifs. Robe areas get 2.5–4× the structure.
3. **Two golds.** Dull, darker ground gold for the field; brighter, more saturated figure gold for the robe.
4. **Cloisonné.** Ornament is cells separated by thin gold lines, filled with flat leaf, black/silver inlay, or enamel.
5. **Log-normal, non-aligned cells.** Sizes spanning ~2% to ~80% of the robe width, aspects 0.5–2.5, no boundary running straight across the screen.
6. **Color is rare.** Gold family ≥ ~80%, black 2–5%, jewels ≈ 1–3% by default.
7. **Mid-tone luma.** Compress output into Klimt's range; no global tint, no "yellow video."
8. **Hard flesh edge.** Pale photographic flesh meets gold with a crisp edge — no glow, no feather.
9. **Flat and still.** Ornament is screen-anchored and stable; it does not wrap around bodies or boil.

---

## 4. Phase plan (do not skip ahead)

**Phase 0 — Audit (no new code).** Read v1 source, git log, and any saved captures. Write `AUDIT.md`: architecture, what each major commit changed visually, what the revert(s) did, which §2 root causes apply, which primitives are worth reusing. Then ask me §0's questions if unanswered.

**Phase 1 — Reference lab.** Download high-resolution public-domain images (Wikimedia Commons or museum sites) into `refs/`, sources recorded in `refs/SOURCES.md`:
*The Kiss; Adele Bloch-Bauer I; Stoclet Tree of Life, Expectation, Fulfillment; Danaë; Judith I; Portrait of Fritza Riedler; Hope II; Water Serpents I; Beethoven Frieze (details).*
Redo and extend §3.2 at full resolution: CIELAB k-means palettes per painting **and per zone** (propose zone polygons for The Kiss, Adele I and Fulfillment as JSON, show me the overlays, I'll OK them); coverage budgets; luma percentiles; band-energy per zone; cell segmentation stats for the Kiss robes, Adele's dress and Fulfillment; rim width relative to cell size; field grain amplitude and fleck density; rough motif frequency tallies. Output `STYLE_SPEC.md` (tables + short findings) and `style_constants.json`. Keep the analysis scripts — they become the conformance tests in §7.

**Phase 2 — Software model.** Build a raster-faithful Python/NumPy model of the full pipeline in `model/`: per-pixel functions of (x, y, input pixel, line-buffer neighbors, previous-frame coarse map, frame counter, parameters), using the same fixed-point math, LUTs and hash the VHDL will use. Vectorize for speed, but keep the semantics per-pixel and causal. It reads `style_constants.json`. Render the test set (below) and produce contact sheets. **All look iteration happens here,** at reduced resolution, not in VHDL.

Test set in `testmedia/` (I'll supply clips; fill gaps with public-domain footage): frontal portrait; two people close together; full-body standing figure; group of people; greenery/landscape; plain interior wall; low-key dark scene; high-key bright scene; fast-motion clip; a scene with no people at all.

**Phase 3 — Look lock.** I approve contact sheets. Commit and tag `look-v1`, saving the approved renders to `gallery/look-v1/`. From here on every look change re-renders the gallery and shows a before/after sheet; nothing merges without my OK. Reverts go back to tags, never by hand-editing code toward an old look.

**Phase 4 — VHDL port.** Generate a VHDL constants package from `style_constants.json` (single source of truth). Port module by module (analysis/coarse map, cell layout, motif rasterizer, palette mapper, flesh keyer/renderer, field renderer, compositor, control mapping). Testbench runs test frames through simulation and compares output to the model — bit-exact is the target; document every deviation.

**Phase 5 — Hardware and tuning.** Build, run on the unit, tune control curves and defaults, capture new gallery frames from hardware next to the model renders.

---

## 5. The design

### 5.1 Signal flow

```
input ─┬─► per-pixel keys (skin, ink, green) ──────────────┐
       ├─► coarse map accumulate (frame N) ─► double buffer ─► zone decisions (frame N+1)
       └─► line-buffered luma/detail ─────────────────────┐ │
                                                          ▼ ▼
              cell layout (bands → tiles → KD splits) ─► motif rasterizer ─► palette mapper
                                                                                 │
           flesh renderer ◄── FLESH key ─────────────► compositor ◄── field renderer
                                                          │
                                         rims/relief/aura ─► GILD slider gating ─► out
```

### 5.2 Analysis stage — coarse map

Per coarse block (target ≈ 1/40 of active width; choose to fit BRAM), accumulate during frame N: mean Y, mean Cb/Cr, detail (Σ|ΔY| horizontal + vertical), skin fraction, green fraction. Read during frame N+1 with temporal IIR smoothing (α ≈ 1/4–1/8) so zones don't flicker.

Global per-frame accumulators: skin centroid (Σx, Σy, count) and skin extents → drive the cameo and the composition bias. A coarse luma histogram → auto-level the tone maps so dark and bright footage both land in Klimt's luma range.

If BRAM can't hold the map, fall back to line-buffer detail estimates plus a vertical IIR, and tell me what was lost.

### 5.3 Zones (per-pixel priority order)

1. **FLESH.** Skin key in CbCr (tolerance from FIGURE knob) with hysteresis thresholds. In CAMEO mode the key is gated by a soft-edged oval centered on last frame's skin centroid, sized from skin extents × FIGURE — so skin-colored walls far from the person don't stay photographic. Boundary antialiased over ≤ 2 px, never feathered wider.
2. **MEADOW** (switch). Green-dominant coarse blocks, plus the bottom ~1/6 of frame where luma is mid-to-dark, become a flower carpet.
3. **ROBE vs FIELD.** Per-block robe score = detail (textured objects, clothing) + proximity to the skin centroid (ornament is densest next to the figure and thins toward the frame edges) + a small hash jitter, thresholded by ORNAMENT. **Zone membership is decided per cell, not per pixel,** so the robe silhouette is built from cell edges — crisp and mosaic-like, like the Kiss's bell shape. With no skin in frame, proximity drops out and detail alone composes the frame.
4. **INK.** Inside ROBE, the darkest cells become black/umber enamel inlays (bars, tablets, checker blocks). Inside FIELD, dark areas become umber ground, not black. Dark masses touching the top of the flesh region (hair) render as flat warm near-black.

Draw a thin bright **aura line** (≈2–3 px at 1080) along the ROBE/FIELD boundary — the Kiss's halo around the figure mass.

### 5.4 Cell layout — the anti-grid spec

- **Level 0:** horizontal bands of hashed height (≈0.6–1.6 × SCALE). Each band is split into tiles of hashed width (≈0.5–2 × SCALE), generated incrementally in raster order and regenerated identically on every line of the band. Tile boundaries never line up across bands.
- **Levels 1–4:** KD splits. Per tile, a hashed decision to subdivide (probability from INTRICACY, zone — high in robe, near zero in field — and coarse-map detail). Split axis prefers the longer side with some randomness; split position hashed from {1/4, 1/3, 1/2, 2/3, 3/4}. Stop at a minimum size of ≈1.5–2% of robe width, floored so a motif stays readable (≈10–14 px at 1080).
- **Target statistics:** robe-zone cell sizes match the measured log-normal (median ≈ 5% of robe width, p90 ≈ 3× median, rare giants); aspects mostly 0.5–2.5.
- **Field cells** almost never subdivide and get no rims. Exception: a very sparse scatter of small isolated silver or gold rectangles floating in the field (Adele).
- **Region mood:** a coarse hash at ~4× cell scale biases neighborhoods toward one motif family, so motifs cluster (a patch of discs, a column of bars) the way Klimt's do. This is what makes it read as composed rather than randomized.
- **Forbidden:** global power-of-two quadtrees, jittered lattices, any boundary that runs straight across more than ~3 adjacent cells.

### 5.5 Motif library

Each motif is a function of local cell coordinates (u, v ∈ 0..1), cell aspect, a per-cell hash, and the cell's base tone. Use LUTs or CORDIC for angle/radius where needed.

**Bank A — KISS**
- **Tall bar:** black inlay rectangle inset in the cell, often with a smaller silver/white tablet inside or beside it. Strongly preferred in tall cells (aspect > 1.4).
- **Silver tablet:** small pale silver rectangle with a dark border.
- **Ring-disc:** 2–4 concentric rings in jewel or gold tones; neighboring small cells cluster as disc groups via region mood.
- **Rosette:** disc with 5–8 petal modulation and a jewel center.

**Bank B — ADELE / STOCLET**
- **Eye:** almond (intersection of two discs) with a dark iris/pupil disc, in oval or triangular cells.
- **Eye-in-triangle:** up- or down-pointing triangle filling the cell, eye inside.
- **Spiral:** gold spiral line on darker gold or cream (Tree of Life, Adele's chair).
- **Concentric squares:** Chebyshev rings alternating gold / red-orange / black (Adele's panel).
- **Checker block:** black and white squares of *varied* sizes (sub-KD inside the cell) with an occasional gold or colored square. Never a regular checkerboard.
- **Stacked triangles:** rows of triangles with stripe bands (Expectation).
- **Oval with eye slit:** large gold-ringed oval on red-orange ground (Fulfillment) — giant cells.

**Shared:** plain leaf (flat figure gold, rim only — the most common cell); tesserae (cell subdivided into tiny squares with darker grout — Ravenna mosaic texture); fleck scatter.

**Assignment:** weighted choice per cell. Big cells → plain leaf, ovals, eyes. Small cells → discs, tablets, tesserae. Tall → bars. Square-ish → discs, concentric squares, spirals. Starting targets for the robe zone at default settings (tune in Phase 2 against Phase 1 measurements): ~45–55% plain leaf/tesserae, ~25% dark inlay motifs, ~15% gold-on-gold motifs (spirals, eyes, concentric), ≤10% jewel motifs.

### 5.6 Color mapping

- Map directly in YCbCr. **Video chroma never passes through** to gold areas; it only *selects* jewels and meadow.
- **Field ramp (ground gold):** auto-leveled video luma → Y ≈ 230–600, ground hue, contrast ≈ 40–60% of the original, so the room stays dimly legible like a relief under the leaf.
- **Robe ramp (figure gold):** Y ≈ 370–700, brighter and more saturated than the field ramp at every point — the robe always reads in front.
- **Cell base tone:** the cell's mean video luma quantized to 3–5 ramp steps (cells read as flat leaf), plus 15–25% of within-cell video luma for life. This keeps object shapes legible through the gold.
- **LEAF knob** sweeps the gold axis: silver/platinum → olive brass (Kiss ground) → warm gold → orange gold (Adele).
- **Jewel enamel set (6–8):** crimson/vermilion, coral/red-orange, violet, cobalt/lapis blue, malachite/turquoise, mother-of-pearl white, black. Starting values from knowledge; replace with measured values from Phase 1 high-res crops. A jewel's color follows the video's hue in that block when the video is saturated there (a red shirt yields red enamel); otherwise hashed. Budget set by JEWEL: ≈1–3% of frame at default, ~8% max. Jewels appear only in ROBE and MEADOW, never in FIELD.
- **Output luma target:** 2nd–98th percentile ≈ 150–720 on a typical scene. Above ~800 only in tiny silver flecks and glints.
- **Never** produce gold by multiplying or tinting the video. That is the yellow-video failure.

### 5.7 Rims, relief, aura

- Every robe cell gets a pale-gold rim (≈2–3 px at 1080, scaled with format), brighter than the cell interior.
- **RELIEF on:** rims and motif edges get a one-pixel emboss — highlight on top/left, shadow on bottom/right — reading as raised gesso. Plus a subtle **glint**: sparse rim/fleck pixels brighten on a slow cycle (period of several seconds) via a hashed phase per cell. Gentle, never sparkly.
- Aura line on the ROBE/FIELD boundary (§5.3).

### 5.8 Field rendering (FIELD switch)

- **NEBULA (Kiss):** darker brass ground; low-amplitude static grain (≈ ±10–15 Y codes at 1–2 px, with soft blotches at 4–16 px); sparse 1–3 px gold flecks, occasionally silver, at ≈0.5–2% of field pixels, blue-noise-like scatter via hash thresholding.
- **LEAF (Adele):** brighter orange-gold glitter; grain ≈ ±25–35 Y codes; denser bright specks; rare floating small silver/gold rectangles.
- Both keep the low-contrast ghost of the video (§5.6).

### 5.9 Flesh rendering

- Keep the photographic detail; remap toward porcelain: lift shadows slightly, compress contrast ≈15–25%, desaturate ≈40–60% toward the measured flesh points (Cb ≈ 460–480, Cr ≈ 540–565), while partially protecting strong reds (lips, cheeks — Klimt's flush).
- Crisp edge to gold, ≤ 2 px antialias.

### 5.10 Temporal behavior

- Static camera + static subject → output frame-identical except deliberate glint (target < 0.5% pixels changing per frame with RELIEF off).
- Zone and motif decisions change only when the smoothed coarse map crosses hysteresis thresholds; a cell that changes swaps cleanly, with no oscillation.
- Flesh is per-pixel live. Robe/field layout updates at coarse-map rate, so the gold follows a moving figure in small steps. Ornament itself stays screen-anchored.

---

## 6. Controls

| Control | Name | Behavior | Default |
|---|---|---|---|
| K1 | **FIGURE** | Skin-key tolerance and cameo size. Low: only the most certain skin (faces) stays photographic. High: wider tolerance, larger cameo. | 45% |
| K2 | **ORNAMENT** | Robe/field balance. Low: nearly all calm gold field (Adele's background). High: ornament reaches the frame edges (Fulfillment). | 50% |
| K3 | **SCALE** | Base tile size; the whole cell-size distribution scales with it. | 40% |
| K4 | **INTRICACY** | Subdivision depth/probability. Low: big plain leaf blocks. High: swarms of tesserae and small motifs. | 50% |
| K5 | **JEWEL** | Jewel-enamel budget and how strongly video color drives it. 0 = pure gold, black and silver. | 30% |
| K6 | **LEAF** | Metal: silver/platinum → olive brass → warm gold → orange gold. | ~55% (warm gold) |
| S1 | **BANK** | KISS (bars, tablets, ring-discs, rosettes) / ADELE-STOCLET (eyes, triangles, spirals, concentric squares, checker blocks, ovals). | KISS |
| S2 | **FIGURE KEY** | SKIN (key anywhere) / CAMEO (key gated by an oval tracking the skin centroid). | CAMEO |
| S3 | **FIELD** | NEBULA (dim brass with flecks) / LEAF (bright glittering gold). | NEBULA |
| S4 | **MEADOW** | Green areas and the lower frame become a pointillist flower carpet. | Off |
| S5 | **RELIEF** | Gesso emboss on rims and motifs, plus slow glint. | On |
| P12 | **GILD** (slider) | The performance gesture — gold leaf laid on piece by piece (below). | 100% |

**GILD staging:**
- 0%: untouched video.
- 0–15%: luma compresses toward Klimt's range; ground gold creeps into the shadows.
- 15–60%: **cells gild one at a time** in hashed order — field and frame-edge cells first, cells nearest the figure last. Each cell pops from photo to leaf with its rim (laying a leaf is binary — no crossfade).
- 60–90%: subdivision deepens, motifs resolve, jewels appear.
- 90–100%: the flesh window tightens to just face and hands.

Map every knob with a perceptually even curve and a sensible minimum (e.g., SCALE never so small motifs become noise).

---

## 7. Acceptance tests

**Automated** (run on model renders in Phase 2 and on simulation output in Phase 4; reuse the Phase 1 scripts):
- **Palette conformance:** gold family 75–92%; near-black 2–6%; non-green jewel ≤ 3% at default JEWEL; luma 2nd–98th within ≈ [130, 760].
- **Composition:** field-zone motif-scale energy ≤ 1/2.5 of robe-zone energy.
- **Anti-grid:** projection-profile and Hough tests on the rim map show no straight boundary spanning more than ~3 cells, and no dominant periodicity in the rim map's autocorrelation.
- **Cell statistics:** robe cell-size histogram is log-normal-ish; median and p90/median within ±30% of the measured Fulfillment distribution; aspect p10–p90 ≈ 0.5–2.5.
- **Stability:** static input → frame-to-frame difference below target.
- **Flesh:** on test portraits, ≥ 90% of face pixels stay FLESH; boundary width ≤ 2 px.

**Visual** (I approve):
- Contact sheet per test clip: input | GILT | nearest reference painting, at full size and at 25% ("across the room"). It must read as Klimt at both distances.
- The two-second test: at a glance it reads as a Klimt painting, not "a video filter with patterns on it."

---

## 8. Failure modes to design against

Yellow/sepia-tinted video · uniform checkerboard or visible lattice · ornament reading as an overlay floating in front of the subject · faces patterned over · boiling or flickering ornament · muddy glow at the flesh edge · everything equally busy (no calm field) · confetti color · low-contrast mush · motifs so small they read as noise · motifs so large and regular it reads as a pattern generator · the robe silhouette not following the subject at all.

---

## 9. Not now (park these; don't build in v2)

- **ATTERSEE mode:** square-format pointillist landscape tapestry, horizon pushed to the top, no gold.
- **LATE mode:** Adele II / Friederike Maria Beer — bright color fields, East Asian motifs.
- **HOLD switch:** freeze the zone map so the subject moves behind a locked composition.
- Flowers in hair, AB-style monograms in relief, Beethoven Frieze figures.

---

## 10. Working agreements

- **State lives in files** so a fresh session can resume cheaply: `AUDIT.md`, `STYLE_SPEC.md` + `style_constants.json`, `DECISIONS.md`, `PROGRESS.md`, `gallery/`. Start each session by reading only `PROGRESS.md` and `STYLE_SPEC.md`.
- Keep sessions scoped to one phase. Iterate at reduced resolution. Don't re-read large files you've already summarized.
- One look change at a time → re-render gallery → before/after sheet → my approval → commit/tag.
- Ask me before changing the control map, adding scope from §9, or dropping a §5 feature for resources.
- Report every visual-vs-resource tradeoff explicitly: what was cut, what it cost visually, what the alternative would be.

---

## 11. Deliverables

1. `AUDIT.md` (Phase 0).
2. `refs/` + `SOURCES.md`, analysis scripts, `STYLE_SPEC.md`, `style_constants.json` (Phase 1).
3. `model/` with test renders and contact sheets (Phase 2).
4. Tagged `look-v1` gallery (Phase 3).
5. Synthesizable VHDL for GILT v2, constants package generated from `style_constants.json`, testbench comparing against the model, all twelve parameters wired with the curves above (Phase 4).
6. Hardware captures beside model renders, final defaults, and a short user guide for the controls (Phase 5).
