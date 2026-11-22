# Triplicate — the picture photocopied in red, green and blue, cascading over a wall of prints (Heirloom #53)

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/triplicate.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.1 (2026-11-15): K6 camera pass on the diag copy, built, NOT flashed** (see [K6 camera pass](#k6-camera-pass-2026-11-15-v061-diag)).
**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Triplicate Diag`, `out/rev_b/ron/_diag/triplicate_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/triplicate/`) is untouched.
**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Source: `photos/selfie.jpg`.
v0.1.0 tinted and cascaded copies of the video without the collage's structure. This version rebuilds the
collage: the measured bear cut-out, sizes, positions, inks and print wall. The camera picture is printed
through every cut-out. Osc mode substitutes a generated skull-faced bear with a computer.
Working files: `.heirloom_work/rb_triplicate/` (`notes.md`, `proto.py`, `imodel.py`, `sheet_video.png`,
`cmp_final.png`).

## K6 camera pass (2026-11-15, v0.6.1 diag)
The same K6 camera design as production v0.5.2 (see `programs/ron/heirloom/triplicate/SUMMARY.md`, "K6 camera
pass"): K6 = Camera, one Q4 gain g = k / 512 on the store writes (luma level 9 + (q − 9) g, chroma c g):
0 % = flat silhouettes / grey original / blank prints, 50 % = exactly the v0.6.0 store, 100 % = 1.94×.
- **Weak control here:** K5 Wall is still the weakest knob (phase 2 left it unchanged; every other knob now
  restructures the collage, and the switches are all look-flips), and Speed, the only motion control, has to
  keep a knob. So Speed (with its phase-2 orbit law) moved to **K5** and Wall is fixed at 512. Presets: Dance's
  Speed 900 moved to K5; Silhouettes and Negatives lost their darker walls (K5 300 / 200).
- Latency unchanged (21): the gain sits in the store-write path. Print Negative still inverts the (gained)
  store luma.
- **Build** (2026-11-15, `.kinetic_work/tools/k6build.sh diag triplicate`): **router1**, all six configs on seed 1:
  HD Analog 86.12, SD Analog 87.09, HD HDMI 82.56, SD HDMI 86.90, HD Dual 82.75, SD Dual 83.00 MHz (worst HD
  82.56). 7,359–7,378 / 7,680 LC (96 %), 26 / 32 EBR. `out/rev_b/ron/_diag/triplicate_diag_v0.6.1.vmprog`.
  (The first, 8 × 8-product version built at 7,491 LC, worst HD 83.28; replaced.)

## Phase 2 (v0.6.0)
No notes from Ron for triplicate (free hand, 2026-11-09 batch). The collage's natural patch points are the four
cut-outs: their sizes, positions and motion, and how each is printed. v0.5 already had bold Cascade, Recede, Ink
and Tiles. But Size spanned only 60–130 %, K6's only copy motion was a ±10 % breathing of the whole cascade, and
S9 Front changed only the small front copy. The round:
- widens Recede through equal sizes into a **reverse** recession (the front copy biggest), and down to a front
  copy shrunk to 128 lines; widens Size to ×0.33..×1.58;
- gives every copy its **own motion**: past the default speed each copy circles on its own quarter-phase of the
  speed LFO, so the four cut-outs dance in a travelling wave, and the wall races;
- replaces S9 Front with **Print Positive / Negative** (all photocopies printed as negatives), and folds the old
  black front plate into S8, which becomes **RGB / CMYK**.

Every new effect is exactly zero at the default knob value. The default frame is byte-identical to v0.5.1
(checks below). Phase-2 working files: `.heirloom_work/rb_triplicate/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| K1 Recede: R 0 (all the red's size) .. 1 (photo, 50 %) .. 1.5 | **R −0.5 .. 0 (at 17 %) .. 1 (50 %, exact) .. 2**, two-slope; heights floored at 128.5 lines; display −100..100 | front copy 1021 lines (reverse: the colour original is the biggest, the others step down behind it) → 820 (all equal at 17 %) → 417 (photo) → 128 (a tiny figure at the foot of the cascade); green 880 → 699 → 578, blue 913 → 634 → 448 |
| K2 Size: ×0.6 .. ×1.3 | **×0.33 .. ×1 (50 %, exact) .. ×1.58**, two-slope; display −100..100 | red copy 270 → 820 → 1297 lines: from four small figures in the middle of the wall to heads cropped off the top of the frame |
| K6 Speed: bipolar; the wall scrolls (≤ 26 lines/s) and the cascade breathes ±10 % | same law up to the default speed (bit-exact); **past it either way** (knob above 640 or below 384) each copy **circles** on its own quarter-phase of the speed LFO, radius growing to ±526 px across and ±132 lines down at the ends; the wall scroll gains up to 3 lines a field | the four cut-outs dance in a travelling wave, sliding over and under each other (period 17 s at the default .. 4.3 s at the ends); the wall races up to ~200 lines/s. Centre (±16) = still; 384..640 = v0.5's drift, no orbit |
| S8 Inks: RGB / CMY | **RGB / CMYK** (renamed label): CMY copies with the front copy as a black plate (v0.5's S8 CMY + S9 Ink) | the whole cascade flips to the cyan / magenta / yellow / black press set |
| S9 Front: Colour / Ink (the front copy as a black plate; subtle) | **S9 Print: Positive / Negative** (renamed): the photocopies' density from the inverted picture; the colour original stays positive | the three big copies flip from dark ink on tinted paper to solid ink with light features: the camera's darks print as paper |

Unchanged: P12 Cascade, K3 Ink, K4 Tiles, K5 Wall, S7 Order, S10 Flip, S11 Source (Video / Osc, default Video).

**Presets** (preset 0 = the defaults):
- **Photo** (renamed from "Selfie");
- **Nested Dolls**, **Fanned**, **Silhouettes** (as before);
- **CMYK Plates** (now S8 alone);
- **Reverse Stack** (Recede 0, fanned, smaller: the copies grow toward the front);
- **Negatives** (S9 Negative, lighter Ink, darker wall);
- **Dance** (Speed 88 %, wider cascade: the copies circle).

**Engine (new).**
- **Register machine:** the per-field microcode is now 61 steps (was 54); the copy loop is steps 45..60.
  - Size S = min(338 + G2·686/2048, 425 + G2·599/2048) and recession R = min(3·G1/4 − 512, G1/2) (Q.10,
    G = knob × 4). Both are 1024 at the default, as before. R can be negative, so the S·R and SR·DH products now
    take R / SR in the signed operand (same product at the defaults).
  - Per copy: H = max(SH − SR·DH/1024, 2056) (Q.4 lines), so 2^26 / H stays a signed 16-bit step. v0.5 relied
    on its 0.6 / 1.5 knob limits for that.
  - Orbit depth A = 11/8 · max(|speed| − 128, 0) (0 .. 526) per field.
  - Per copy, TOP += A·tri(φ + (ci+1)/4)/4096 and LEFT += A·tri(φ + ci/4)/1024, where φ is the existing
    breathing phase.
  - The copy's phase-shifted triangle comes from a small fabric unit (by quadrant: 16384 − a, −a, a − 16384, a),
    loaded in one step. Loads now write at cycle 2.
- **Wall scroll:** a pre-registered step, speed/64 ± max(|speed| − 128, 0)/8 (Q.4 lines per field).
- **Print Negative:** the density uses the bit-inverted 8-bit luma. The colour original's luma ramp and the
  wall are untouched.
- **CMYK:** the front copy's palette and chroma follow S8 instead of S9.
- **Block-RAM rings (fit):** each rides one 256 × 16 ring off the sync delay's write pointer (`no_rw_check`,
  read 6 / 5 back + RAM register + fabric register = 8 / 7 clocks).
  - The chroma filter's 8-back tap replaces s_csu / s_csv.
  - The pixel path's stage-6 → 13 fields (layer, seam, front chroma, print exposure) replace
    p_sd(10..13) and p_cq(7..13). The exposure field is written from stage 5, so it comes back at stage 12,
    where it is read.
- No pipeline stages added or removed: latency 21 in every mode, U/V parity unchanged.

**Build** (ROUTER1: `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh triplicate`, 2026-11-09):

| Config | Seed | Fmax (MHz) |
|---|---|---|
| HD Analog | 1 | 81.61 |
| SD Analog | 1 | 87.83 |
| HD HDMI | 1 | 92.84 |
| SD HDMI | 1 | 84.76 |
| HD Dual | 1 | 80.26 |
| SD Dual | 1 | 86.79 |

7,072–7,095 / 7,680 LC (92 %), 26 / 32 EBR (+2: the rings). No RAM transparency emulation (standalone yosys);
iceunpack OK ×6; the packed config carries the new labels and presets.
Router2 on the first netlist missed HD Analog on all 6 seeds under batch load (best-effort 71.55 MHz), so stay
on router1. Router1's tightest path was the per-field wall-scroll update, now shortened by the pre-registered
step.

**Checks** (working files in `.heirloom_work/rb_triplicate/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: every machine value at RTL widths over knob extremes, 7 timing modes, speeds and LFO phases.
  No 16-bit, 14-bit, 26-bit or 28-bit overflow; S = R = 1024 at the defaults.
- `spikes.py`: the outline ROM (now magnified up to ×2) has no isolated second-difference spike. The only large
  ones are the real ear-to-dome merge at rows 6–7.
- Default frame (GHDL 720p60 dec 1, Osc, 2 warm-up + 1 capture, production v0.5.1 simulated the same way): raw
  10-bit output **byte-identical**. This proves both rings are aligned.
- Not verified (reasoned from the code and the sweep only): the orbit, Print Negative, reverse Recede, the Size
  extremes and CMYK. SD and interlace were not simulated.

## Reading of the photo (measured)
The photo is a portrait paper collage, 3527 × 4988 (0.707:1). Measurements below are in page heights
(1600-px working copy).
- **Four copies of one cut-out.** The cut-out is a bear with a skull pasted as its face and a TI-99
  computer on its lap.
  - The back three are one-colour photocopies in red, green and blue. The front one is the full-colour
    original.
  - Measured tops / heights: red 0.057 / 0.759, green 0.233 / 0.647, blue 0.379 / 0.587, front
    0.583 / 0.386.
  - Size ratios between neighbours: green/red 0.852, blue/green 0.907, front/blue 0.658.
- **The cascade.** Steps (dx, dy): red→green 0.110, 0.176; green→blue 0.121, 0.147; blue→front
  0.097, 0.204. The slope is about 54° below horizontal, and each copy covers half of the one behind it.
- **The cut.** The three tinted copies share one outline to within 0.01 h. The bounding box is 0.604 wide
  per unit of height.
  - The dome rises between two rounded ears (±0.17 h at the ears).
  - The neck is ±0.14 h.
  - The bell body flares to −0.244 / +0.36 h, wider on the haunch side.
  - The base is flat. The edges are hard scissor curves with no rim or shadow.
- **Inks.** Each copy is ink over tinted paper with a steep photocopy curve: about 40 % of the head region
  is solid ink, and the ramp between ink and paper is linear. Values are hardware 10-bit, u = Cr:

  | Copy | Ink (Y, U, V) | Paper (Y, U, V) |
  |---|---|---|
  | Red | 350, 818, 424 | 824, 585, 475 |
  | Green | 440, 378, 432 | 872, 479, 474 |
  | Blue | 345, 332, 698 | 833, 449, 564 |

  Light toner specks show in the solid areas.
- **The wall.** Square greyscale prints of skull piles on a regular grid with a period of **0.094** both
  ways.
  - The grey is neutral (mean chroma 3.8/255). Luma mean is 105, with the 2 % / 98 % points at 40 / 214.
  - Per-print exposure varies (std 8.9).
  - Some rows are half-height or shifted. Some edges show thin light seams.

## 16:9 mapping
- The frame height equals the page height. Every element keeps its size and vertical position relative
  to the height.
- At Cascade 50 % the page composition sits centred. The wall of prints simply continues to the frame
  edges.
- The P12 slider widens the cascade's horizontal steps beyond the page (up to 2.7×) to use the wide
  frame.
- Geometry is in virtual 1080-line units, so 720p renders the same picture.

## What it does
1. **Store.** The source (camera, or the Osc generator) is written into a 128 × 128 × 4-bit luma store
   and a 32 × 32 U4V4 chroma store.
   - Luma is box-filtered over power-of-2 windows, with an accumulator RAM doing the vertical sums.
   - Luma is held in four parity banks, so the display can read it bilinearly.
2. **Copies.** Four cut-outs use the measured outline: a 128-row ROM of edge intervals, interpolated per
   line, so the curves have no stair steps.
   - Copy heights are S · (0.759 · H − R · ΔH), giving 0.759 / 0.647 / 0.587 / 0.386 H at the defaults.
   - The copies are anchored at their measured bottoms (0.816 / 0.879 / 0.966 / 0.969 H). The cascade
     scales the offsets between them.
   - The front-most copy wins at each pixel. Each copy shows the whole store, magnified bilinearly.
3. **Photocopy.** Density = 2 (HI − L), clamped, with HI = 115..345 set by K3 Ink.
   - Toner grain: 3/256 of pixels lose 3/8 of their density (light specks); half of the pixels lose 1/16.
   - The density is dithered to a 5-bit level that indexes a 32-entry ink→paper ramp fitted to the photo.
   - The front copy takes a luma ramp plus the chroma store, so it shows in colour.
4. **Wall.** Square prints (101 lines at the default) each show a 64 × 64 window of the store in grey.
   - The window is chosen by a hash and biased towards the top of the picture (the head).
   - Some prints are mirrored. Exposure varies per print in four steps.
   - About a quarter of the left and top edges get a light seam. Each row of prints is offset sideways.
   - The wall is 7/8 of the luma ramp, plus the K5 Wall offset.
5. **Output.** Shell output conditioning, then the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Recede | the size step red → green → blue → front | 0 = all copies the red's size; 50 % = the photo's recession; 150 % = steeper |
| K2 Size | the copies' scale | 60–130 % of the photo's sizes (100 % at 50 %) |
| K3 Ink | the photocopy density | faint prints → the photo's ~40 % solid ink → solid silhouettes |
| K4 Tiles | the wall's print size | 43–323 lines (default 101 = the measured 0.094 H) |
| K5 Speed | (motion) wall, cascade and orbits | the old K6 Speed, moved here in v0.6.1 with its phase-2 law unchanged (see the Phase 2 table). The old K5 Wall is fixed at 512 |
| K6 Camera | the picture being copied | camera strength (v0.6.1): store luma contrast about level 9 and chroma; 0 % = flat silhouettes and blank prints, 50 % = the v0.6.0 look, 100 % = 1.94× |
| S7 Order | which ink is at the back | RGB (the photo) / BGR |
| S8 Inks | the ink set | RGB / CMY |
| S9 Front | the full-colour original | Colour / Ink (a fourth, black plate) |
| S10 Flip | the copies' facing | Off / On (the 2nd and 4th copies mirrored) |
| S11 Source | the picture being copied | Video (default) / Osc |
| **P12 Cascade** | the diagonal cascade | 0 = all four nested on one spot (matryoshka); 50 % = the photo's steps; 100 % = horizontal steps 2.7× (fanned across the frame). Vertical steps reach the photo's at 50 % and stay there |

P12, K1, K2 and K4 glide (1/8 of the gap per field). The control sheet is
`.heirloom_work/rb_triplicate/sheet_video.png` (from the proto); every control changes the picture.

## Video
- The camera picture is what gets photocopied. It is printed through all four bear cut-outs:
  - three as red / green / blue ink density;
  - the front one in its own colour.
- The same picture, in grey, fills every print on the wall, each print a different window.
- Osc substitutes a generated figure drawn into the store, so all copies and the wall print it:
  - a shaded skull with eye sockets, nose, cross and rows of writing marks;
  - a cream monitor with a blue screen and a scrolling logo strip;
  - a console with dark keys, on dark fur.
- Every control acts in both modes.

## Engine
- **Per-field register machine** (vblank = "no avid for 1024 clocks").
  - Microcode is in LUT logic. The register file is one EBR (16-bit words).
  - One serial shift-right multiplier with arithmetic result shifts, one restoring divider, and one
    post-op stage (add / clamp / min / max).
  - It runs the knob glides, the cascade / size / recession curves, print size and steps, and per copy:
    height, 1/height, row and column steps, top, left and start phases.
- **Per-line engine in hblank** (8 clocks after the generated avid falls, about 36 clocks).
  - It steps the four copies' rows, reads and interpolates the outline ROM (16 reads, then a lerp product
    stage), loads the copies' column DDAs, and steps the wall rows.
- **Pixel path:** column DDAs and edge compares; front-most winner; bilinear store read; density, grain
  and dither; palette ROM; compose; output conditioning; gate.
- **24 EBR:** luma store 16; chroma store 2; accumulator 1; register file 1; outline ROM 1; palette 2;
  sync delay 1. All collision-free RAMs carry `no_rw_check`.
- **Contracts:**
  - generated avid;
  - virtual 1080-line coordinates from the timing ID;
  - one 21-clock latency in every mode (`processing_delay_clks` = 21);
  - output conditioning (luma [1 2 1] and scaled toward ~777; chroma [1 2 2 2 2 2 2 2 1]/16) before
    the blanking gate;
  - colours in the hardware convention (u = Cr).

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). A 13-FF g_avid chain was added for the gate. Post-route Fmax: HD Analog s2 81.02, SD Analog s1 75.31, HD HDMI s3 75.49, SD HDMI s1 79.98, HD Dual s4 85.41, SD Dual s1 72.71 MHz; 7076-7103 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-21, router2, 6/6 on seed 1, not flashed.**
  - Post-route Fmax: HD Analog 74.28, HD HDMI 76.84, HD Dual 74.67, SD Analog 71.73, SD HDMI 59.39,
    SD Dual 71.55 MHz.
  - Fit: 7058–7080 / 7680 LC (92 %), 24/32 EBR. All six bitstreams pass iceunpack.
- **HD margins are very thin** (HD Analog +0.03 MHz, HD Dual +0.42). If a rebuild misses, try more seeds
  or `NEXTPNR_ROUTER="--router router1"`. The remaining critical paths are routing in the register
  machine's control fan-out (cycle counter / run flag → multiplier enables, register-file write data).
  Pre-decoded phase flags are the restructure if needed.
- **GHDL renders** (720p60, decimation 1, defaults, exact hardware hues via raw2hw), in both Osc and
  Video (fed a crop of selfie.jpg):
  - copy tops 0.058 / 0.235 / 0.381 H (photo 0.057 / 0.233 / 0.379);
  - visible bottoms 0.815 / 0.879 / 0.965 (photo 0.816 / 0.879 / 0.966).
- **Motion:** 4 frames apart at the default Speed, the wall scrolled up 0.35 px (as designed) and the
  cascade breathes.
- SD and interlace modes were not simulated.

## Deviations from the photo
- **Fixed outline.** Each copy prints the camera picture (or the Osc figure) through the fixed, measured
  bear outline. In the photo the copies are photocopies of one paste-up with a large skull. Here the
  cut-out does not follow the camera's subject.
- **Softer copies.** A 128-row, 4-bit store is magnified to up to 820 lines, so fine detail (the skull's
  Greek writing) becomes texture.
- **The wall's content.** The prints are windows of the same live picture (Osc: the generated skull), not
  separate ossuary photographs. The photo's half-height and shifted rows become per-row offsets.
- **Whites and chroma.** Whites are capped at about 777 by the shell's output scale (the HD HDMI bar fix),
  so the papers are slightly dimmer. Chroma is softened by the 9-tap filter (invisible at HD). The front
  copy's colour comes from a coarse 32 × 32 chroma store.
- **Motion.** The photo is still. The default Speed gives a slow drift and breathing instead.
- **SD 4:3.** The picture is the same, with less wall at the sides. At P12 100 % with Size at maximum, the
  red copy's back edge clips at the 4:3 frame edge.

## Open questions for Ron (2026-11-09 batch)
1. S10 Flip (the 2nd and 4th copies mirrored) is now the subtlest switch. Would you rather have a "contact sheet"
   there, with every print on the wall showing the whole camera picture? It's nearly free in LC.
2. Should the copies' orbit start right at the default speed, instead of only past the 384..640 band?
3. Is the reverse recession (the colour original biggest) worth the left part of K1, or should Recede start at
   equal sizes?
4. Should Print Negative also invert the colour original?

## Hardware watch list
- Renamed: S9 Front → Print (Positive / Negative), S8's "CMY" → "CMYK" (the front black plate moved here); K1
  Recede and K2 Size now display −100..100 (0 = the photo); preset "Selfie" → "Photo".
- Orbit (K6 past the default): the copies slide over and under each other. Draw priority is fixed, so the
  colour original always stays in front.
- K2 Size above ~1.06 puts the red copy's head off the top of the frame (by design). At extreme Recede + Size the
  front copy can cover most of the cascade.
- ROUTER1 now (v0.5 was router2). No pin-skew screen.
- Cut for fit: nothing.

## Next step
Wait for Ron's preview notes. Rebuild the diag with
`NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh triplicate`. On approval:
- run `./heirloom_diag.sh merge triplicate`;
- repack the previewed diag bitstreams under the production TOML instead of re-routing: copy
  `build/programs/ron/triplicate_diag/rev_b/bitstreams/*.bin` into the production build dir, regenerate
  `program_config.bin` from the production TOML, then `vmprog_pack.py --no-sign --hardware rev_b`;
- update the production SUMMARY, README and PHASE2 row, and commit only triplicate's files.

## v0.6.3: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`16384 - a` comes out as `a - 16384`). The
simulator is correct, so every render showed the intended picture; only the hardware differed.
- **Site fixed:** line 761, quadrant "00" of each copy's orbit triangle: `r_trc <= 16384 - v_a` ->
  `to_signed(16384, 16) - v_a`. `r_trc` drives the TOP / LEFT orbit offsets (up to +-132 lines / +-526 px).
- **What changes on hardware:** with K5 Speed outside the 384..640 band (orbit depth > 0), each copy used to snap
  across its whole orbit once per cycle on each axis, a quarter-cycle apart: up to ~1050 px sideways or ~264 lines.
  The triangle jumped from +16384 to -16384 and spent its first quarter on the wrong side. Now the copies glide round
  a continuous triangle orbit, as designed. Within 384..640 and at the default nothing changes.
- **Remaps:** none (never flashed at this version).
- **Left as-is:** nothing else listed by the audit.
- **Build:** router1, all six on seed 1: HD Analog 86.30, SD Analog 83.11, HD HDMI 83.38, SD HDMI 75.30, HD Dual
  83.94, SD Dual 83.43 MHz (worst HD 83.38); 7,366-7,394 / 7,680 LC (96 %), 26 / 32 EBR. iceunpack OK x6.
- **HD HDMI skew:** the builder's router1 s1 sat at **-0.31** / data <= -0.89 (negative). Screen hd_hdmi r1 + r2
  s1-40: 74 routed, 40 timing-clean (router2 mostly misses timing here), 2 in the window. Installed **router1 s8
  (+1.76 / <= -1.19, 83.32 MHz)**; spare router1 s12 (+1.34 / <= -0.79, 82.43 MHz) in
  `.heirloom_work/rb_triplicate/skew/triplicate_v063_spare_hd_hdmi_router1_s12.vmprog`. The builder's HD Analog and
  HD Dual seed-1 data margins were too small (<= -0.31 and <= -0.47), so both were re-seeded too: HD Analog ->
  **router1 s9 (-1.58 / <= -0.85, 81.98 MHz)**, the only screened seed with data <= -0.6; HD Dual -> **router1 s8
  (-1.43 / <= -1.10, 82.52 MHz)**. Shipped worst HD 81.98 MHz. Not flashed.
