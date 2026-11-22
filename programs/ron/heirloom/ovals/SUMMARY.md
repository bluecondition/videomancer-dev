# Ovals — chains of rainbow-roped vesica ovals on a striped pink ground

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/ovals.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.4 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-03, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Ovals Diag`, `out/rev_b/ron/_diag/ovals_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.2 (`programs/ron/heirloom/ovals/`) is untouched. Phase-2 working files:
`.heirloom_work/rb_ovals/p2/`.

## Phase 2 (v0.6.0)
No notes from Ron for ovals (free hand, 2026-11-03 batch). v0.5.2 sat at 99.6 % LC, so the round first freed about
350 LC with no visible change, then spent it on per-column parameters and two new switch looks:
- **K1 Columns** was a global X zoom of the whole lattice. It is now **Stagger**: each of the four lens columns
  per group (outer left, the double oval's two lenses, outer right) gets its own lens-row phase, like four
  oscillators on one clock with their own phase offsets. The column count is fixed at the photo's.
- **Seams.** v0.5 picked a half's inner or outer lens by quadrant (the fold at T = m). That only works while
  the two lenses are in step; with any Stagger it put a straight full-height seam down every diamond-centre
  line (the phase-1 seam problem, seen in the first v0.6 render). The pixel path now takes the **union of each
  half's inner and outer lens** (three-way max with the other half's inner lens). The blend that hides the side
  flip uses the true crease, |Q_in − Q_out| / 2, in place of the distance to T = m. That also removes the
  v0.5 deviation "the fold picks one lens at the diamond-centre line (a Q step up to ~12 px)".
- Both Stagger patterns keep the two outer columns in one phase, so where neighbouring groups meet (the period
  boundary) nothing steps.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms |
|---|---|---|
| K1 Columns: lattice density ×0.5 .. ×2 (zoom about the centre) | **K1 Stagger** (renamed), bipolar, photo at 512 (all columns in step) | **Right = Braid:** the double oval's two lenses slide apart in phase, up to anti-phase (each ±1/4 row = ±131 lines at 1080p), while the outer columns stand still. The double oval becomes two interleaved chains, one lens bulging where the other pinches. **Left = Rock:** the double oval slides against the outer columns, up to half a row (263 lines = 24 % of the height), so its bellies sit beside the outer columns' X points and the diamonds between them change shape. Glided; rides with K2 (the offsets are in rows) |
| K2 Links: lens-row frequency ×0.5 .. ×2, clamped at both ends (the outer ~12 % of travel did nothing) | **×0.25 .. ×4, exponential over the full travel**, photo at 512 (bit-exact) | 0 = half a link on screen (one giant lens, rows ~2100 lines); 1023 = ~8 rows of 131-line links. The stripes keep their period |
| S8 Glow: On / Off (the bottom ~12 % of the frame) | **S8 Glow: Bottom / Rows** | Bottom = the photo's white glow below line ~945. **Rows:** the belly of every lens row glows across the whole frame: full white over the middle third of each row, fading out over the next ~15 % (it follows the bulge oscillator, so it moves with Speed and K2) |
| S9 Twist: the violet / green ropes swap sides (a colour nudge) | **S9 Wave: Arc / Cusp** (renamed) | Cusp inverts the bulge oscillator (255 − s): every link pinches where it was widest and comes to a sharp point where the links used to touch. The result is cusped hourglass / four-point-star links with concave sides over the whole lattice. The X stars and the Rows glow follow the new pinch / belly rows |

Unchanged: K3 Split, K4 Speed, K5 Ropes, K6 Etch, S7 Stripes, S10 Palette, S11 Source, P12 Swell (all were
already live and bold on the v0.5 sheet).

**Presets** (hand-written placeholders, as everywhere in heirloom):
- Photo (the defaults).
- **Braid** (K1 1023).
- **Rock** (K1 0, Swell 700).
- **Cusp Chain** (S9 Cusp).
- **Glow Beads** (S8 Rows, K2 700).
- **Ice Cusps** (Cusp + Ice, Braid 800, Swell 800).
- **Tall Rock** (Rock 150, K2 200, Swell 900).
- **Dense Lattice** (K2 1000, Split 800, K5 300).

**Engine (new).**
- **Per-column half-widths.** After the v0.5 line computation (states 1..128), the line engine runs four
  more passes (e_lc 129..250, column j = e_lc(6:5)). Each pass reads the bulge ROM at the line's phase ± the
  column's offset, computes M_j = A·s_j on the line multiplier, and stores h_j = (M_j + 1.047·M_j + dl)/2 + p_j
  in place of the shared half-width. The half-width chain (mg16 → t1 → hb16 → hb) now runs every clock off
  e_m16, so the four passes share its adders. The star, the Rows glow and the centre line use the shared phase.
- **Pixel stages 2..8** are the per-half union: s = T − m, a = s + L_in, b = s − L_out, |a|, |b|,
  Q_in = h_in − |a|, Q_out = h_out − |b|. Stage 6 decides on three signs at once (Q_in − qo, Q_out − qo,
  Q_in − Q_out). Stage 7 takes both crease half-distances, and stage 8 the min against the axis distance. The
  stage count is unchanged.
- **Cusp** is one XOR on the bulge ROM output; **Rows** is a per-line mux into the glow weight.
- **K2:** idx = K2 >> 2 into the 0.5 .. 2 table, shifted left or right by idx(7). c_dv is 13 bits; every
  product of it fits its slice (`sweep.py`).

**Fit (nothing visible was cut).** First the cuts, ~−400 LC before any feature:
- The output chroma filter's two 8-deep 10-bit delay lines → a 256 × 20 block-RAM ring (read = write − 6, RAM
  register + one fabric register = 8 clocks, `no_rw_check`, 0 transparency emulation): −160 FF, +2 EBR.
- The video tap's 10-clock FF delay → one register: the camera (and the Osc etch lattice) sits ~8 px left of
  the lattice at 1080p (~1 % of the width at SD). That is invisible for a ring-bending etch, and with black input
  it changes nothing.
- The latency-28 padding (two 30-bit output registers) moved ahead of the beam: the beam starts at
  s_gav_sr(1), every pixel-path avid tap is +2, and data_out = stage 26. The output stream is unchanged.
- The blend distances are saturated at 63 (only e < 32 matters) and Q is carried at 12 bits (|Q| < 1200).
  Both are bit-exact.
- The Osc row step (ys·121/256, a constant multiply) is a 7-entry table, and K1's density slot and multiply are gone.

Then the union and the features took back ~+170 LUT. Net: 7,291–7,3xx LC against v0.5.2's 7,645–7,657.

**Build** (router2, `.heirloom_work/diag_build.sh ovals`, 2026-11-03): post-route Fmax HD Analog 81.18 (seed 1),
SD Analog 77.29 (seed 1), HD HDMI 88.34 (**seed 2**: seed 1 stalled the router under load, not a miss), SD HDMI
84.85 (**seed 2**, same), HD Dual 79.99 (seed 1), SD Dual 88.37 MHz (seed 1). 7,291–7,307 / 7,680 LC (95.1 %,
−354 against v0.5.2), 17 / 32 EBR (+2: the chroma ring). iceunpack OK ×6. The sources' md5 was unchanged across
the build. After the build the presets Staircase / Weave / Tall Weave were renamed Braid / Rock / Tall Rock (they
were named for an earlier pattern), and the vmprog was repacked from the same bitstreams with a regenerated
program_config.bin: 146 bytes differ, all labels. `out/rev_b/ron/_diag/ovals_diag_v0.6.0.vmprog` (482,620 B), copy
in `out/rev_b/ron/heirloom/_diag/`. No pin-skew screen.

**Checks** (working files in `.heirloom_work/rb_ovals/p2/`):
- GHDL analyse / elaborate / synth front end clean (`check.sh`).
- Standalone yosys (`syn.sh`): v0.5.2 4,324 LUT / 3,083 FF / 15 RAM → v0.6.0 4,352 LUT / 2,725 FF / 17 RAM; no
  `emulate_transparency`.
- `sweep.py`: K2's c_dv over the full travel fits every product slice (z_t1, dphi, z_t7, c_fls) in all modes,
  and is monotonic with the default = v0.5's 1030; the Stagger offsets are 0 at 512; the Rows weight is 0..61.
- GHDL 720p60, decimation 1, 2 warm-up + 1 captured field:
  - **Cuts only (before the union): default frame byte-identical to production v0.5.2** (Video, no input, the
    photo look: 0 of 921,600 pixels differ). Osc differs only by the Osc etch lattice's ~10 px shift (visually
    the same, `cmp_d1.png`).
  - **Final: default frame = v0.5.2 except two thin strips on the diamond-centre lines** (1.19 % of pixels
    differ by more than 12; `diff_d2.png`). There the union replaces the old fold's Q step with the true crease.
  - Extremes (sim-only copies with the K1 glide preset): Braid 1023 + Glow Rows (`x2_braid.png`) and Rock 0 +
    Cusp (`x2_rock.png`) render coherently, with no straight seams (`crop_x2.png`). The first v0.6 build's
    staircase render (`x_stag_rows.png`) is the one that showed the fold seams.
- Not checked by sim: SD, motion over time with Stagger, Split / Swell extremes with Stagger, K2 extremes.
  These are reasoned from the code and the sweep.

## Open questions for Ron
1. **Stagger patterns.** Right = Braid (the double oval's lenses apart), left = Rock (the double oval against the
   outer columns). The alternative I drafted first was a staircase rising across each group (outer columns ±1/2
   row). It needs a union across the period boundary too (~+60 LC), because the outer columns of neighbouring
   groups then differ. Want it instead of one side?
2. **Columns.** The column count is now fixed at the photo's (Stagger took K1). Should a count or zoom come
   back, for example on a switch with two or three fixed densities?
3. **S8 Glow Rows vs Off.** Rows replaced the plain Off. Do you miss a glow-free picture? (P12, K5 and Split
   all still show the rings under it.)
4. **Twist.** The violet / green rope swap is gone (S9 is now Wave). Bring it back as a third role somewhere?

## Hardware watch list
- **Renamed controls** (diag vs production labels differ): K1 Columns → **Stagger**; S9 Twist → **Wave**
  (Arc / Cusp); S8 keeps **Glow** but its options are now Bottom / Rows (was On / Off).
- **Default picture:** it should match v0.5 except along the two diamond-centre lines beside the double oval,
  where the union now draws the true crease (thin ~3 px strips; it was a Q step in v0.5).
- **The video etch** now sits ~8 px left of the lattice (the video tap moved ahead). That should be invisible.
  If the etch looks offset, that is why.
- **Seams:** with Stagger off-centre, look along the diamond-centre lines and the period boundary for straight
  vertical seams; the sims showed none at the extremes. At extreme Split / Swell the v0.5 boundary seam (the
  outer lenses straddling the period boundary) is unchanged.
- **Timing:** comfortable (HD 80–88 MHz). HD HDMI and SD HDMI are on seed 2 (router stalls on seed 1 under
  load); if HD HDMI shows lines, re-route that config.
- **Fit:** 95 % LC, so a second round has ~100 LC before the guideline. Nothing was cut for fit.

## Next step
- On approval: `./heirloom_diag.sh merge ovals`, then **repack the previewed diag bitstreams** under the
  production TOML instead of re-routing. Copy `build/programs/ron/ovals_diag/rev_b/bitstreams/*.bin` into a
  scratch dir with `program_config.bin` regenerated from `programs/ron/heirloom/ovals/ovals.toml`, then
  `vmprog_pack.py --no-sign --hardware rev_b --toml-path programs/ron/heirloom/ovals/ovals.toml <dir>
  out/rev_b/ron/heirloom/ovals.vmprog`. Expect the same size as the preview, ~140 bytes different, and no
  `_diag` in the id.
- Then update the production SUMMARY / README / PHASE2 rows and commit only ovals's files and rows
  (`.heirloom_work/p2_batch/stage_rows.py ovals`).
- To rebuild the diag instead: `.heirloom_work/diag_build.sh ovals` (router2).

## v0.5 reference (production)

**v0.5.1 (2026-10-28): built, NOT flashed.** Rebuilt from the photo WIN_20231003_23_01_50 (Heirloom #50).
v0.5.1 is the review iteration on v0.5.0: no straight seams (lens union plus a both-side palette blend at every
side flip) and high-contrast ring interiors.  Presets are hand-written placeholders.

## Reading of the photo (measured)
- **Ground**: hot pink (R 255, G 30..45) with horizontal stripes: B alternates ~78 / ~145 with a half period of
  39.35 lines (period 78.7; edges at y 19, 57, 97, 135 ...).  The left edge (x < ~300) is plain pink: the lattice
  oscillator is hsync-reset (its phase holds at the trough before the first column).
- **Lattice**: columns of vesica lenses ("ovals") stacked into chains that touch at X points (X rows y ~256 and
  ~781; lens rows centred at y ~ -6, 518, 1043).  The strands were tracked row by row (track.py / trackG3.py) and
  fitted to rms 5 px (fitgeo3.py -> geo3.json):
  - lens j = interval [V_j, G_j] per row,
    V_j = xc + c_j + oV_j - M(y), G_j = xc + c_j + oG_j + 1.047 M(y) + dl(y),
    c_j = -(m+L), -(m-L), +(m-L), +(m+L), with m = 255.3 px and L(y) = 185.4 + 59.3 t + 25.2 t^2 (t = (y-520)/520);
  - M(y) = 127.1 |sin(pi (y - 255.6)/525.6)|^0.816 (a rectified sine: the two crossing strand families);
  - xc = 967.6 + 7.6 t; dl = 24.4 - 16.8 t; period 1513.9 px (V4 = V1 + period, seen at the right edge);
  - per-strand offsets oV = (2.0, 7.4, 10.8, 0), oG = (-1.7, 4.6, 4.8, -24.7).
  That is a horizontal triangle folded twice (at m, then at L(y)) and keyed against the rectified vertical sine.
  Four lenses per period: the outer pair and the inner pair, which overlap into the middle **double oval**.
  L grows toward the bottom, so the inner pair converges and the outer pair spreads.
- **Ropes**: every lens has the same orientation, a scan-direction asymmetry and not a symmetric field.
  - Left (V) edge, outside to inside: a green fringe ~30 px outside a violet core (68, 65, 219), then blue,
    brown at +25 px, magenta at +50 px, then red.
  - Right (G) edge, inside to outside: orange, olive, teal, a pure-green core (0, 190, 0), then teal.
- **Channel split**: R is V-shaped dark at the ropes; G is a dark band just inside the V ropes; B carries many rings.
  The B rings follow the contours of Q = the signed distance into the union of lenses (qcont_B.png).
- **Diamonds** between the columns at the X rows: nested contours with red-orange cores at the top and PINK cores
  at the bottom (L is larger there, so the gap is deeper).  Cyan / blue stars sit at the X points.
- **Interiors**: red-orange (22 %), orange (21 %), magenta-pink (14 %), peach (13 %), brown (9 %), cream (8 %)
  rings, cut into 39-line blocks.  The G-side features shift with the stripe parity (G strands +-1.5 px).
- **Glow**: below y ~950 everything except the V ropes and the pink ground whitens (per-row gamma 0 above 940,
  ~0.9 below 990).
- No row tearing (row-to-row shift 0 on every row).  Soft horizontal edges, ~10 px 10-90 %.

## What it does
1. A phase DDA w (half-px units) runs the period-1514 triangle T = |w|.  It is held at the trough left of the
   first column (hsync-reset).
2. The fold picks this half's lens (inner or outer: T vs m), Qf = h_j - ||T - m| - L_j|.  The other half's inner
   lens (the double oval's partner) gives qo = h - |T + m - L|.  **Q = max(Qf, qo)**: the union across the
   period centre, continuous, with a crease where the inner pair overlaps.  The fold levels L_j (L + per-lens
   offset) are clamped at 0, so at Split 0 the pairs merge into one lens.
3. side = the winning lens's half (0 = left / V rope, 1 = right / G rope).  e = the distance to the nearest
   side flip: the lens axis, the diamond-centre line, or half the crease / gap with the other inner lens.
   The weight of the G-side palette is b = 8 +- e/4 (of 16): a +-16 px ramp at axes, +-32 px at creases.
4. Palette index = Q x rope gain + luma x Etch.  One ROM word holds both sides' colours at {bank, index}.
   bank = stripe parity inside a lens and in the deep ground; diamond or ground-fringe elsewhere.
5. colour = A + (B - A) b/16 (A = V-side palette, B = G side; Twist swaps them).  Then a cyan X star near the X
   rows, (64 - s8)(32 - |Q|)/32, and the bottom white glow, gy(y) x kap(side, Q).
6. Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16, then the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Columns | lens columns across | lattice density x0.5 .. x2 (zoom about the centre); 512 = the photo |
| K2 Links | lens rows down each chain | vertical period / ~2^(1.3 (k-512)/512) (x0.4 .. x2.5) |
| K3 Split | spacing of the paired columns (the double oval) | fold level L x k/512: 0 = pairs merge into single columns, 512 = the photo, 1023 = the inner pair crosses |
| K4 Speed | chain flow, stripe roll | bipolar, +-8 deadband: chains flow down / up (default 600 = ~1 line per field, max ~6), stripes roll with them |
| K5 Ropes | rope and ring widths | index gain 2^((512-k)/512): fat .. thin ropes and rings |
| K6 Etch | rings / ropes bent by the picture | luma added to the palette index (default 200 = ~50 px per full luma, max ~250 px) |
| S7 Stripes | ground stripes, interior blocks | On (the photo) / Off |
| S8 Glow | bottom white glow | On (the photo) / Off |
| S9 Twist | rope colours | the violet and green ropes swap sides |
| S10 Palette | whole palette | Candy (the photo) / Ice (U and V exchanged) |
| S11 Source | etch source | Video (the camera) / Osc |
| **P12 Swell** | lens bulge | M amplitude 0 = straight knotted ropes, 512 = the photo, 1023 = huge interlocking ovals |

Presets (placeholders): Photo, Knotted Ropes, Big Links, Chain Mail, Ice Weave.

## Video
The photo is fully synthetic, so the camera is woven into the patch.  In Source = Video, the line-buffered camera
luma is added to the palette index (K6 Etch, default 200): rings, ropes and diamond contours bend around the
picture while the chains, diamonds and ground keep the photo's composition.  Black input gives exactly the
photo look.  Osc substitutes a drifting 2-D triangle lattice through the same path, so K6 stays live.

## Engine
- **Vblank-zone sequencer**: one serial multiplier, a shared glide (1/8 of the gap per field, +-2 deadband) and
  a 128-entry 2^x knob-curve table.  Per field it computes the density, links, rope gain, speed and the stripe
  and flow phases.
- **Per-line engine**: kicked at each line start, it computes the NEXT line (states 1..128, ~130 clocks): the
  sine ROM (|sin|^0.816, 1024 x 8), L(y) on the serial multiplier and its Split scaling, the four clamped fold
  levels, the half-widths, the other-lens offsets m - L_1 / m - L_2, the star, glow and stripe values and the
  line's start phase.
- **Beam**: a phase DDA (Q10), wrapped at +half period and held at -half period.
- **Pixel path**:
  - 1-5: the fold, plus the other inner lens (|do|, qo);
  - 6: the union, side, ground and flip flags;
  - 7-9: the blend weight;
  - 7-10: rope gain and etch (in parallel);
  - 11: ROM address {bank, index} and the glow-table side;
  - 12: palette ROMs, 6 x 1024 x 8;
  - 13: both sides copied to fabric;
  - 14-16: the side blend;
  - 17-19: the star / glow blend and compose;
  - 20-25: output conditioning;
  - 26: the gate;
  - 27-28: output registers.
  Latency 28 in every mode (processing_delay_clks 28).
- **EBR**: palette 12, sine 2, video line buffer 1 = 15 / 32.  The palette and glow table are generated by
  ovals.py into ovals_pkg.vhd, in hardware convention (U = Cr, V = Cb), luma pre-scaled 16/13 with a
  860..1100 -> 860..940 knee.
- **Contracts**: generated avid; one latency in every mode; the [1 2 2 2 2 2 2 2 1]/16 chroma and [1 2 1] luma
  bar fix plus luma 13/16 before the blanking gate; virtual 1080-line coordinates (SD shows the centre 4:3,
  720p the same picture).

## Status
- **v0.5.2 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 80.76, SD Analog s1 81.00, HD HDMI s1 83.12, SD HDMI s1 76.95, HD Dual s2 78.21, SD Dual s1 83.47 MHz; 7645-7657 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.1 (2026-10-28), router2: built, not flashed.**
  - Fmax (MHz): HD Analog 83.35, HD HDMI 84.57, HD Dual 76.87 (**seed 2**, seed 1 missed), SD Analog 81.23,
    SD HDMI 83.27, SD Dual 82.12; every other config on seed 1.
  - 7638-7652 / 7680 LC (99.6 %, over the 92 % guideline: no headroom), 15 / 32 EBR, iceunpack OK x6.
  - out/rev_b/ron/heirloom/ovals.vmprog (494,246 B).
  - GHDL 720p (decimation 1) matches the integer model exactly: 100.00 % of pixels within 12, mean |dRGB| 0.00,
    at defaults and at K3 343.
- **Metrics vs the photo** (local contrast = 32 x 32-block luma std; photo 13.2, lattice 16.5):
  - RTL 720p: v0.5.0 9.9 / 12.5, v0.5.1 13.0 / 16.2 (interior 19.8 vs 17.0).
  - 4 x 4 region means: v0.5.0 max 9.7 / mean 6.8, v0.5.1 max 12.3 / mean 7.3.
- **v0.5.0 (2026-10-27)**: built 6/6 (SD Analog seed 2), 80.4-87.7 MHz, 6731-6756 LC (88 %).  Reviewed: hard
  vertical seams through the double oval and lens axes (fold quadrant switch), flat orange interiors.

## Deviations from the photo, and why
- **Interior rings**: the photo's deep-interior rings change with the lens-row position (a 2-D (Q, width)
  structure).  A 2-D palette did not fit the EBR / LC budget, so the rings are concentric with the lens edges:
  more regular and slightly more pastel (interior luma std 27.5 vs 36.1).  Cream and white peaks are kneed
  under the 777 output cap.
- **Region means** are a little worse than v0.5.0 (the brighter rings push the upper-centre cells up ~+11 in G
  and B); the local contrast now matches.
- **Glow**: a smooth ramp toward white, weighted per palette entry, not the photo's blobby white and orange streaks.
- **Streak blocks** come from the 39-line stripe banks, not the photo's per-band random G-side shifts.
- **Double oval**: the hidden inner ropes meet at a soft crease instead of crossing visibly.  Near the X rows
  the crease and gap blend shows as a soft vertical band (+-32 px).
- Where a half's inner and outer lens overlap (low Split or high Swell), the fold still picks one at the
  diamond-centre line.  With the per-lens offsets this is a Q step of up to ~12 px, inside a blend zone.
  It is not visible at defaults.  The full 3-candidate union was drafted (rb_ovals/syn/v20.vhd) but needs
  ~7750 LC.
- The header comments of ovals.vhd and ovals.py describe the ideal four-lens union; the pixel path implements
  the fold plus the other half's inner lens (the stage comments say so).
- SD shows the centre 4:3: the pink side grounds are mostly cropped.

## Working files
`.heirloom_work/rb_ovals/`:
- `ref.jpg`, `notes.md`;
- strand tracking and geometry: `track.py`, `trackG3.py`, `fitgeo3.py` -> `geo3.json`;
- palette: `mklut*.py`, `mklut17.py` -> `lut17d.npy` / `rom17d.npz` (= `rom.npz`); `mkhook.py` writes `ovals.py`;
- `proto_final.py` (float prototype), `imodel.py` (integer, RTL-exact);
- checks: `match.py`, `regions.py`, `lc2.py`;
- images: `cmp_final.png`, `z_mid_final.png` (photo / v0.5.0 / v0.5.1 middle double oval), `sheet.png`;
- `v051a/` (fallback build without the fold clamp), `syn/v20.vhd` (3-candidate draft).

## K6 top +25 % and default 100 % (2026-11-15)

**Bend type**: K6 Etch adds the camera luma to the palette index. The change is the same as in the main program: c_k6 = K6 / 2 + K6 / 8 and the etch slice p7_et(17:8), +25 % at every position. The K6 default is now 100 % (orchestrator).
- Not rendered; checked by GHDL analysis and the build.
- Build: v0.6.2, router1, all six on seed 1; worst HD 86.64 MHz (HD Analog 89.69, HD HDMI 96.16, HD Dual 86.64); 7,312 / 7,680 LC. `.kinetic_work/k6_builds.txt`: diag ovals OK router1 worst-HD 86.64 LC 7312.

## v0.6.3: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`0 - x` as `+x`), while the GHDL simulator computes it correctly, so every render showed the intended picture and the hardware ran the reversed one. All nine sites were absolute values written as `0 - x`; each is now `-x` (same widths and slices).
- **Fixed (9 sites):**
  - line 506, `v_k := -v_k`: |K1 − 512|, the **K1 Stagger** amount (lens-row phase offset; the sign picks Rock / Braid);
  - line 717, `v_tn := -v_t`: |tq| = |y − 520|, the line engine's distance from the chain's centre row (feeds the lens-pair separation L, centre xc and half-width offset);
  - line 882, `v_n := resize(-v_w, 13)`: T = |w|, the folded lens triangle for the left half of each lens period;
  - line 897, `v_n := -p2_do`: |do|, distance to the other half's inner lens (the crease union);
  - lines 906 / 908, `v_n := -p3_a` / `-p3_b`: |a| and |b|, distances to this half's inner and outer lens centres;
  - lines 948 / 950, `v_dg := -p6_df` / `-p6_dio`: the two crease distances that set the palette blend weight;
  - line 983, `p10_aq <= unsigned(-v_qp(5 downto 0))`: |qp|, the star-glow weight outside each lens edge.
- **On hardware before → now:** the pixel path was essentially broken on hardware. The left half of every lens period read the wrong triangle (882), every inner lens lost its half nearest the period centre (906), every outer lens drew only its outward half (908), so the vesica chains came out as half-ovals with ground stripes; the upper half of every chain (above line 520) had lens pairs popping in and out with a step at line 520 (717); creases had a hard palette edge on one side (948 / 950) and the star glow outside each lens edge was an inverted, over-bright band (983); where L > 511 + T (bottom ~20 % at default Split, much more above ~67 % Split) the double oval's crease was a flat vertical cut (897). The left half of **K1 Stagger** was an instant full Rock with no ramp, and K1 resting at centre flickered between the photo and full Rock on ADC jitter (506). Now the hardware draws the rendered picture: full rainbow-roped double ovals with soft creases, the glow fading away from the edges, and K1 ramping smoothly both ways from the photo at 512.
- No default or preset remaps (group 1, never flashed). No sites left as-is.
- **Build** (router1, `.heirloom_work/skew_tools/build_any.sh ovals`): all six configs seed 1. Post-route Fmax: HD Analog 91.53, SD Analog 84.65, HD HDMI 87.83 (builder seed, replaced, see below), SD HDMI 92.48, HD Dual 91.05, SD Dual 82.58 MHz. 7,461–7,502 / 7,680 LC (97.7 %, ~+170 over v0.6.2's 7,312: the abs logic is real now), 17 / 32 EBR, iceunpack OK ×6.
- **HD HDMI skew:** the builder's HD HDMI router1 s1 was **−0.72 / ≤ −1.54** (negative HSYNC). Screen router1 + router2 s1–40 (77 timing-clean): 6 in the window. Installed **router2 s19 (+1.23 / ≤ −1.60, 83.93 MHz)**. Spare HD HDMI router1 s5 (+1.27 / ≤ −1.10, 83.61 MHz): `.heirloom_work/rb_ovals/skew/ovals_v063_spare_hd_hdmi_router1_s5.vmprog`. HD Analog r1 s1 (−1.52 / ≤ −1.41) and HD Dual r1 s1 (−1.59 / ≤ −0.62) kept. The builder's HD HDMI .bin is `skew/hd_hdmi_replaced_202611212016.bin`.
- `out/rev_b/ron/heirloom/ovals.vmprog` (v0.6.3). Backups of v0.6.2 (vhd, toml, vmprog) in `.heirloom_work/rb_ovals/`. Not flashed.
