# Sinew — ropes and sinews, Heirloom #18

**v0.6.2 (2026-11-18): phase 2 FINAL — Ron approved the diag and it is merged into production** (`out/rev_b/ron/heirloom/sinew.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; NOT flashed at v0.6.2). v0.6.0 = the phase-2 round (see [Phase 2](#phase-2-v060)), v0.6.1/v0.6.2 = the K6 camera pass (K6 = the camera strength, default 100 %, top end +25 %). The replaced production v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.
v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed. Source photo:
`photos/WIN_20230315_00_53_56_Pro.jpg` (1920×1080 capture, 2023-03-15). Replaces v0.1.0 (the picture wrung
into S-curves by a vertical sine), which Ron judged too abstract and not like the photo. Presets are
hand-written placeholders.

## Phase 2 (v0.6.0)
No notes from Ron for sinew (free hand, 2026-11-09 batch). The v0.5 picture is two patched oscillators (ropes θ,
sinews ψ) into a colourizer, and most v0.5 knobs were already module parameters. The weak spots were:
- Melt barely showed in Osc mode (±0.09 rope cycle);
- S7 Tear and S8 Grain were subtle on/off switches;
- Ropes / Sinews spanned only ×0.5..×2;
- the photo's strongest patch cord, the ropes phase-modulating the sinews (the ")" hooks), had no control at all.

The round:
- puts that patch cord on the headline P12 with the pinch, so the slider changes the sinews' *shape*: from straight
  combed lines to knots and hooks;
- widens the two oscillator rates to ×0.25..×4 and makes Twist bipolar (the S-curve reversed);
- makes Melt bend the ropes as well as the sinews;
- re-patches the two subtle switches: S7 Tear becomes Photo / Storm, S8 Grain becomes Mirror (the grain stays on).

Every new effect is exactly zero at the default knob value. The default frame is raw-identical to v0.5.1 (checks
below). Phase-2 working files: `.heirloom_work/rb_sinew/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| P12 Pinch: the vertical modulator's depth (0 = straight evenly spaced sinews, 50 % = the photo, 100 % = vertices pinched to 24 px) | **P12 Pinch**: the same depth, **plus the hooks**: the ropes' phase modulation of the sinews ×0 .. ×1 (50 %, exact) .. ×2.94 | 0 % = dead-straight, evenly spaced sinew lines with no wiggle at all (a combed grid over the ropes). 100 % = the vertices pinched into knots and every sinew swinging ±0.49 cycle (≈ ±137 px at the bulge's 280 px spacing) around each rope: hooks and loops instead of ")" arcs. The photo's wiggle is ±0.166 cycle (≈ ±47 px) |
| K1 Ropes: rope count ×0.5 .. ×2 | **×0.25 .. ×4**, exponential (an octave per quarter turn), photo at 50 % (exact); all three bands | 1.75 ropes across (rope period ≈ 1090 px) .. 28 ropes (≈ 68 px); the photo's 7 at 50 % |
| K2 Sinews: the bulge's base period ×0.5 .. ×2 (the fan's depth fixed in px, so the vertices clamped early) | **×0.25 .. ×4**, exponential, photo at 50 % (exact); scales the **whole** sinew period (bulge and vertices together, so the fan keeps its shape), floor 24 px | sinew spacing at the bulge 1122 px (one or two lines across the frame) .. 70 px (a fine comb; the vertices hit the 24 px floor) |
| K3 Twist: the ropes' S-curve 0 .. ×4 | **×−3 .. 0 .. ×1 .. ×4**: bipolar; below 37.5 % the S-curve reverses (linear 2x − 768 below the default; the v0.5 square law above it, exact) | 0 % = the S swung the other way, ±1.1 cycle (≈ ±300 px top to bottom); 37.5 % = straight leaning ropes; 50 % = the photo (±0.37 cycle ≈ ±100 px); 100 % = ±1.5 cycles (≈ ±400 px) |
| K4 Speed: bipolar, top ±503 | the v0.5 law up to \|v\| 128 (the default 120 is exact), then **×4 steeper**: ends ≈ 3.2× the v0.5 ends | at the ends the pinch rolls ≈ 380 lines/s (v0.5 116), the sinews stream 1.5 cycles/s (0.46), the ropes drift ≈ 76 px/s (23) |
| K6 Melt: video / Osc bends the sinews (to ±0.7 cycle) and, at 1/8, the ropes (±0.09) | the v0.5 curve up to the default (420, exact), then gain 2 .. 32: sinews to **±2 cycles**; above the default the ropes take twice the sinews' share, **to ±0.5 cycle** | max: sinew contours displaced ±2 cycles (≈ ±560 px), ropes ±136 px. In Video the camera's luma pushes the ropes visibly sideways; in Osc the two triangle waves fold the ropes into chevrons. The rope bend steps from ≈ 4 to 8 px as K5 passes its default |
| S7 Tear: Off / On (torn strips at the vertices; subtle) | **S7 Tear: Photo / Storm** (renamed values): Photo = v0.5 On; Storm = every line of every band tears | Storm: phase jumps ×4 (±160/256 cycle ≈ ±170 px) held 1–4 lines on half the lines, slips 16× as often (1 line in 32) and ×4 as far (±17–80 px for 7–38 lines): the whole frame shredded into sliding strips. No Off position any more |
| S8 Grain: Off / On (the ropes' marbling; subtle) | **S8 Mirror: Off / On** (renamed): the right half mirrors the left; the grain is always on | the S-curves and the sinew fans meet at the centre line as a symmetric totem (> < chevrons); the camera melt is not mirrored |

Unchanged: K6 Palette, S9 Wave (Triangle / Saw), S10 Bands, S11 Source (Video / Osc, default Video).

**Presets** (preset 0 = the defaults): **Photo** (renamed from "Default"); **Combed** (P12 0, Twist at the straight
point); **Hooked** (P12 100 %, more sinews); **Totem** (Mirror, Twist ×4, fewer ropes); **Storm** (Storm tear,
fast, P12 68 %); **Molten** (Osc, Melt max: chevron ropes); **Reverse S** (Twist −3×, broad sinews, Saw);
**Fine Weave** (Ropes and Sinews ×4, Bands off). Dropped: Knotted (→ Hooked), Saw Fan, Full Patch.

**Engine (new).**
- **Hooks.** The pixel path reads the wiggle from a 256×16 RAM instead of the cosine ROM. In each vblank zone
  (from zone clock 255, after the field program sets the gain) a serial writer rewrites it as C_COS × hg / 16
  (hg = P12's hook depth, Q4: 0 .. 16 = the photo .. 47), one shift-add per gain bit, 12 clocks per entry
  (≈ 3.1k clocks; the shortest zone, SD, is ≈ 16k). The RAM starts as C_COS and hg = 16 reproduces it exactly.
  No pixel-path change; +1 EBR (the ROM now feeds only the writer). `no_rw_check` (written only in the zone);
  yosys maps it to one SB_RAM40_4K with no transparency emulation.
- **Sinews scale.** In the line program the triangle law's P (24 px floor, as v0.5) is multiplied by
  isc = 4096 · 2^((512 − K2)/256) (exponential by octave, linear within it) and floored at 24 px again before the
  1/P division: two states and one multiply on the shared multiplier, the divider widened by one bit (P up to
  1122 px). v0.5's P0 curve is gone (P0 fixed at the photo's 280.6 px).
- **Ropes curve.** The same exponential (x + 512, octave = bits 10..8) gives the rope scale; f_rsc widened to 16 bits
  (SD at ×4).
- **Twist.** Signed gain (−768 .. 1022, Q8).
- **Melt.** One pixel product (6-bit gain 0..32 × source) at stage 2: ψ += ·2^9 as v0.5; θ += ·2^6 up to the
  default, ·2^7 above it (a per-field flag).
- **Speed.** f_ve = 4v ∓ 384 past ±128 (= v + 3(v − 128)), registered every clock, feeds the three drift phases.
- **Storm.** The tear zone test ORs in the switch, the jump and the slip are ×4 (10-bit offsets), and the slip
  trigger widens from 9 to 5 zero bits.
- **Mirror.** From the half width the three beam DDAs (θ two-rate, ψ, Osc x) step backwards by the one's
  complement of their increment (a 2^-20-cycle error per pixel, invisible); mirror axis at pixel W/2 − 1.
- Latency 24 in every mode, U/V parity and the output conditioning unchanged.

**Build** (ROUTER1: `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh sinew`, 2026-11-09):
HD Analog s1 80.78, SD Analog s1 86.81, HD HDMI s1 81.86, SD HDMI s1 87.94, HD Dual s1 83.69, SD Dual s1
72.67 MHz. 7,187–7,215 / 7,680 LC (93.8 %), 23 / 32 EBR (+1: the wiggle RAM); iceunpack OK ×6; the packed config
carries the new labels and presets. No pin-skew screen.
**Fit history.** The first build reached 7,578 LC (98.7 %; HD Analog 77.37 MHz). Standalone yosys (`p2/abl/`):
production 3,740 LUT, first build 4,370, final 3,993. No feature cut; the saving came from one's-complement mirror
steps (no stored negated increments or hold step), one melt product instead of two, and a leaner writer and
speed law.

**Checks** (working files in `.heirloom_work/rb_sinew/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: every new knob path at RTL widths over all knob extremes (rope increments over all bands and lines
  at SD / 720p / 1080p, the sinew period and 1/P quotient, hook table range, Twist, Melt gains, speed): no
  overflow; the defaults are exact.
- Fitted tables: the ψ offset table (C_DEL2) has no isolated second-difference spike (its largest second
  differences are paired, a rounded corner), and no new control magnifies it.
- Default frame (GHDL 720p60 dec 1, Osc, 2 warm-up + 1 capture, production v0.5.1 simulated the same way): raw
  10-bit output **byte-identical**, on the first source and again on the final fitted source.
- Not verified: no extreme render; the hooks at ×3, Mirror, Storm and the Melt extremes are reasoned from the
  code only.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_sinew/` (`notes.md` has every number).
- **Composition.** Three horizontal bands (row-difference scan):
  - top, rows 2–148: yellow / orange / green vertical bars;
  - middle, rows 150–900: seven orange ropes S-curving down the frame, crossed by thin blue/pink "sinew"
    lines that bulge into six ")" arcs at mid-height (y ≈ 540) and fan into green and black shingles above
    and below, with dense "<" chevrons at the bottom;
  - bottom, rows 902–1077: pink / navy / teal bars with U-shaped teal contours.
- **Mechanism (channel split).** Two horizontal oscillators, each restarting at the line start (hsync-reset),
  into a three-channel colourizer:
  - **Rope oscillator θ (R).** R is a clipped trapezoid of θ in every band. Per row, R's rising edges are
    exactly linear in x (rms 0.01–0.02 cycle):
    - top band: period 297.8 px, duty 0.558;
    - middle band: period 271.8 − 15.8u − 14.2u² px (u = (y − 540)/360, so 280 → 245 px from top to bottom),
      start phase 0.290 + 0.153u + 0.004u² − 0.179u³ (the ropes' S-curve, rms 0.018 cycle),
      duty 0.511 − 0.031u − 0.011u² (rise and fall halves run at different rates);
    - bottom band: period 263 → 251 px, duty 0.49.
  - **Sinew oscillator ψ (B).** B is thin pulses at integer ψ. Found by 2-D ridge detection and
    row-to-row order tracking:
    - each row is ψ = a(y)·x + b(y) + 0.166 cos 2π(θ + 0.22): the sinews are phase-modulated by the ropes;
    - the period follows a vertical triangle: P = 280.55 − 0.5297·s px, s = the soft distance to the nearest
      triangle peak (period 688 lines, centred y = 537, vertex rounded over 64 lines), rms 2.6 px.
      Lines are 280 px apart at the bulge and ~120 px at the vertices (y ≈ 193 and 881);
    - offset b = 408.3/P − 1.443 (the oscillator effectively restarts 408 px before active video), plus a
      ±0.38-cycle correction table along the triangle (a jump at the top seam, y ≈ 220).
  - **Colour.**
    - R is a function of (θ, Ω), R² 0.97–0.99, with Ω = ψ/2 mod 1: alternate sinew cycles differ (a
      divide-by-2). Halving ψ raises G's fit from 0.48 to 0.69 (upper zone) and from 0.63 to 0.76 (bulge).
    - G's mapping drifts along the triangle (upper / bulge / lower zones differ), so it is fitted as three
      slices. About 45 % of G's variance is grain: marbling inside the ropes, x period ~30 px, vertical
      coherence ~13 rows, bending along the sinew lines.
    - B = table(θ, Ω) plus a vertical ramp 25.2u² + 23.9u − 6.9 (u = distance along the triangle / 360 lines):
      the ropes turn pinker toward the bottom.
    - Bottom band: G is a function of (θ, y), the U-shaped contours.
- **Palette (k-means 12, share).** Green (21,152,30) 13.9 %, dark green (18,58,40) 13.5 %, orange
  (247,133,26) 11.8 %, red-orange (247,81,26) 9.8 %, yellow (248,203,8) 9.5 %, teal (12,157,118) 7.9 %,
  salmon (249,115,107) 6.7 %, olive (121,161,27) 6.4 %, navy (18,53,131) 6.0 %, pink (246,39,115) 5.7 %,
  brown (125,51,47) 5.2 %, grey 3.5 %.
- **Texture.** Torn strips at the triangle vertices (y ≈ 150–237 and 852–900: rows displaced up to ±50 px in
  runs of a few lines) and one 5 px slip at y = 744. Soft horizontal edges, no interlace comb.
- **Fit quality (float proto, then RTL).** Middle-band local contrast 0.93 of the photo (9×9 luma σ); 4×4
  region-mean colour error max 8, mean 4.4 (of 255). RTL at 720p: 0.90, max 9, mean 5.4.

## What it does
1. **Rope oscillator.** Per line: the two increments (rise half, fall half) and the start phase from the fitted
   cubics (top/bottom bands: their own constants). Ropes scales the increments; Twist scales the S-curve.
   Per pixel: a 24-bit two-rate DDA.
2. **Sinew oscillator.** Per line:
   - the triangle distance (mod 688 lines; Saw option), the soft triangle s;
   - P = P0·Sinews − Pinch·s and a = 1/P (serial divider);
   - b = 408.3a − 1.443 + the correction table + the stream phase.
   Per pixel: a 22-bit DDA plus the rope-locked wiggle (cosine ROM, 0.166 cycle).
3. **Melt.** Camera luma (or the Osc) × Melt is added to ψ (up to ±0.69 cycle) and, at 1/8, to θ.
4. **Colourizer.**
   - Middle band: R = table(θ6, Ω5); G = three slices along the triangle, blended per line; B = table + the
     blue ramp; plus the grain: a triangle of 9θ + a per-rope random walk + 0.375Ω, ±32 in the ropes,
     ±8 in the gaps.
   - Top band: R, G curves of θ.
   - Bottom band: R, B curves of θ; G = a (θ, row) table interpolated over 16 rows.
5. **Palette routing**, then RGB → 10-bit YUV in the hardware convention (U = Cr, V = Cb). Luma is
   pre-compensated ×16/13 for the output conditioning.
6. **Tear.** Inside 44 lines of each triangle vertex: random phase jumps (both oscillators, up to ±40/256
   cycle), held 1–4 lines, on half the lines. Plus slips: 1 line in 512 starts a ±4..19/256-cycle shift lasting
   7–38 lines.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Pinch** | The sinews' bulge and fan, i.e. the vertical modulator's depth: 0 = straight, evenly spaced sinew lines (no bulge); 50 % = the photo; 100 % = the vertices pinched to 24 px, dense knots and chevrons. Glided. |
| K1 Ropes | Rope count 0.5–2× (the photo: 7 across), all three bands. Glided. |
| K2 Sinews | Sinew line density 0.5–2× (base period 561 → 280 → 140 px). Glided. |
| K3 Twist | The ropes' S-curve 0–4× (centre = the photo; 0 = straight leaning ropes). Glided. |
| K4 Speed | Bipolar, still at the centre ±8. Positive: the pinch rolls down, the sinews stream, the ropes drift. Default 640 ≈ 28 lines/s (one triangle cycle in ~24 s), sinews 0.11 cycle/s. |
| K6 Melt | How far the camera (or Osc) bends the sinews and ropes. Default 420 (gentle bend); max ±0.7 sinew cycle. Glided. |
| K5 Palette | 1 the photo, 2 R/B/G, 3 G/R/B, 4 G/B/R, 5 B/R/G, 6 B/G/R, 7 the photo with G inverted, 8 the negative of 2. |
| S7 Tear | Torn strips at the vertices and slips (default On). |
| S8 Grain | The ropes' marbling (default On). |
| S9 Wave | Triangle (the photo) / Saw: the fan runs one way and resets at the vertex. |
| S10 Bands | The top and bottom bar bands (default On); Off: the sinew patch fills the frame. |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** the camera luma, (Y − 512)/4 tapped straight from `data_in` (raw input stays shallow), × Melt is
  added into the sinew phase and, at 1/8, the rope phase. The sinews and ropes bend around the picture; at
  default knobs the photo's composition still reads (selfie test: the bears bend the lower right).
- **Osc:** two triangle waves (~5 cycles across, ~4 down, drifting per field) drive the same path, so Melt stays
  live: at max they bend the sinews into chevrons.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks; states 1–17 at zone clock 64), on the shared multiplier:
  Ropes scale × the timing step, Sinews base period, Twist, Pinch slope, Melt gain.
- **Glides:** a shared bank (±4 deadband, 1/8 ease per field) on K1, K2, K3, K5 and P12.
- **Per line** (kicked one clock after the source avid rise of line n, computing line n+1; line 0 at zone
  clock 600; ~650 clocks, fits SD's 858-clock line):
  - band select; xorshift tear and grain walks;
  - three Horner cubics (coefficients registered during each multiply); Twist; Ropes scale;
  - triangle distance, soft triangle, P, the 1/P division, b;
  - one 256×16 ROM read: correction offset + blue ramp; slice position and bottom row by shift-adds;
  - tear offsets and rope drift.
  The results are double-buffered (n_* → c_* at the next avid rise).
- **One serial multiplier** (24×18): signed operands loaded into one register pair, magnitudes taken in a
  pre-cycle, the signed product registered in a finish cycle. **One restoring divider** (17-bit quotient).
- **Pixel path, 24 stages:**
  - two DDAs; cosine ROM (+ fabric register); melt (4-bit gain × 8-bit source);
  - ψ, θ; table addresses; ROMs (+ fabric register);
  - slice blend (9×5) and bottom blend (9×4); blue ramp; grain; band mux; palette;
  - RGB → YUV shift-adds;
  - output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums, luma [1 2 1], luma to ~777;
  - blanking gate on the latency-aligned generated avid.
  `C_LAT` = `processing_delay_clks` = 24 in every mode.
- **Memories (22 EBR):** colour tables LR, LG0–2, LB (5 × 2048×6), bottom G 1024×12, cosine 256×16,
  band curves 2 × 256×8, correction/ramp 256×16. All generated by `sinew.py` into `sinew_pkg.vhd`.
- **Shell:** heirloom v0.2.0 (escalator): timing-ID virtual-1080 steps (x spans the width, y = 1080 virtual
  lines), vblank zone, width measurement, generated avid (`p_gavid` / `g_start`).
- **Fit history:**
  - The first RTL was 7807 LC (101 %) and failed placement. Ablations: the line program was 2200 LUT.
  - Cuts: the shared signed multiplier (−360 LUT); tear as direct phase offsets; the correction table and the
    blue ramp in one EBR word (no interpolation, 6 multiplies gone); shift-add slice position and bottom row;
    4 grain walkers; 4-bit melt gain. 6802 LC.
  - Timing (critical paths fixed in turn): engine kicks registered a clock ahead with an explicit idle state;
    one op per state in two shift-add chains; Horner coefficient registered; signed product registered.
    HD Dual router1 went from 74.5 to 88.2 MHz on seed 1 (82–95 MHz over seeds 1–4).
- **Verification** (GHDL 720p60 dec 1 vs the RTL-exact `imodel.py`):
  - Osc defaults: 99.7 % of pixels within ±12/255, mean 1.0 (after the tester's 4-px capture offset).
  - Motion: K4 max, frame 7 = model frame 7 (99.7 %); frame 4 differs (48 %), so it moves as designed.
  - Video = selfie: 94 % within ±12, 99.5 % within ±32 (the camera tap sits a few px off the beam).

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 85.32, SD Analog s1 93.40, HD HDMI s1 82.18, SD HDMI s1 87.67, HD Dual s1 93.76, SD Dual s1 83.89 MHz; 6857-6882 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-27, NOT flashed.** **ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh sinew`), all six configs seed 1.
- Post-route Fmax: HD Analog 82.37, HD HDMI 83.67, HD Dual 88.21, SD Analog 87.18, SD HDMI 85.93,
  SD Dual 86.02 MHz.
- 6871–6900 / 7680 LC (≈ 89.7 %), 22/32 EBR. All bitstreams pass iceunpack. `sinew.vmprog` 487,484 B.
- router2 missed HD (65–74 MHz) on an earlier version of the design; it was not retried on the final one.

## Deviations from the photo
- **The bulge.** The photo's lime flash just right of each ")" arc is weaker; orange runs 20–30 px past the
  arc. The shingle stripes are a little softer than the photo's (64×32 table cells, per-cell median fit).
- **The grain** is synthetic (matched in scale, ~30 px across and ~13-row coherence, and in contrast), not the
  photo's exact marbling.
- **The torn strips** are the same field with random phase jumps: less multicoloured than the photo's top strip.
  The tear realisation is random.
- **The left edge** (x < 40): the photo's line-start smear at the bulge isn't reproduced.
- **SD 4:3:** the same picture (seven ropes across, same rows), compressed horizontally by 3/4; nothing in it is
  round, so nothing distorts visibly.
- **Interlaced modes** draw the same picture in both fields (no half-line offset).
- **Colour tables** are 6 bits per channel; the bottom band's G is 16 rows, interpolated.

## Working files
`.heirloom_work/rb_sinew/`:
- `notes.md`: every measurement, fit, proto round and build.
- Fits: `rcross.py`, `thfit2.py`, `ridge.py`, `track.py`, `psifit2.py`, `trifit2.py`, `ropepoly.py`, `tables.py`.
- `fmodel.py` (the field model), `build_data.py` (the table fit), `proto.py` (float proto),
  `imodel.py` (RTL-exact integer model), `gen_hook.py` (writes `sinew.py`).
- `sheet_final.py` / `sheet.png` (control sheet), `cmp_proto.png`, `cmp_final.png`.
- `abl/` (ablations), `synth_sa.sh` (standalone synth), `check_local.sh` (GHDL check including `sinew_pkg.vhd`).

## Open questions for Ron (2026-11-09 batch)
1. S7 lost its Off position (Photo / Storm). Bring back a clean Tear Off, with Storm elsewhere?
2. The hooks ride P12 with the pinch. Would you rather have them on their own knob (e.g. in place of K6 Palette)?
3. Melt's rope bend above the default is twice the sinews' share (a small step at the default). Give the ropes
   their own gain curve (≈ +60 LC)?
4. S8 is now Mirror and the grain is always on. Do you miss turning the marbling off?

## Hardware watch list
- Renamed: S7 Tear values Off / On → Photo / Storm, S8 Grain → Mirror (Off / On); preset "Default" → "Photo",
  Knotted → Hooked.
- The grain (marbling) is always on.
- Mirror axis at pixel W/2 − 1; the camera melt is not mirrored.
- The hook table is rewritten in each vblank: a P12 move reaches the hooks one field later.
- K2 Sinews near max with P12 high: the vertices sit on the 24 px floor; may moiré on SD.
- No cuts. ROUTER1 as v0.5, all seed 1. No pin-skew screen; if HD HDMI shows lines, re-seed that config with router1.

## Next step
Wait for Ron's preview notes. Rebuild the diag with
`NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh sinew`. On approval: `./heirloom_diag.sh merge sinew`,
then repack the previewed diag bitstreams under the production TOML (copy
`build/programs/ron/sinew_diag/rev_b/bitstreams/*.bin` into the production build dir, regenerate
`program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`) instead of
re-routing; update the production SUMMARY / README / PHASE2 row and commit only sinew's files.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. Here the camera strength was on K5 (Melt), so K5 and K6 swapped: **K6 = Melt** (the camera strength, default now 512 = 50 %), **K5 = Palette** (was K6). Nothing else changed: the VHDL swaps the two knob inputs (the processing is identical), the TOML blocks and preset keys moved with them (the Controls rows above are renumbered). v0.6.1, rebuilt with router1: all six seed 1; HD worst 76.12 MHz (HD Analog 76.12, HD HDMI 79.10, HD Dual 82.14), LC LC. Rebuild: `NEXTPNR_ROUTER="--router router1" ./heirloom_diag.sh build sinew`. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/.

## K6 top +25 % and default 100 % (2026-11-15)

Ron (renders of fern and kiln at K6 = 100 %): bend / element programs get a 25 % stronger camera effect at the top of K6; the K6 default is now 100 % (1023, set with the version bump).

- **Class:** BEND (as production): K6 (Melt) bends the sinews and ropes.
- **Change:** Melt x1.25: below the default `f_m5 = 3.75 K6^2 / 8192` (was 3 K6^2; steps 0 .. 2 as before); above it the gain `f_gp = gpa + gpa / 4` (2 .. 40, was 2 .. 32). Max +-2.5 sinew cycles; the 14-bit melt product and psi (bits 20:0 used) have room.
- **Build:** Built with router1: HD Analog seed 2, the rest seed 1; worst HD 77.91 MHz (HD Analog 77.91, HD HDMI 85.79, HD Dual 77.91); max 7222 / 7680 LC.
