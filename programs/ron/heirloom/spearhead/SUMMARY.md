# Spearhead — Heirloom #5 (rebuild v0.5.0)

**v0.6.2 (2026-11-18): phase 2 FINAL — Ron approved the diag and it is merged into production** (`out/rev_b/ron/heirloom/spearhead.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; NOT flashed at v0.6.2). v0.6.0 = the phase-2 round (see [Phase 2](#phase-2-v060)), v0.6.1/v0.6.2 = the K6 camera pass (K6 = the camera strength, default 100 %, top end +25 %). The replaced production v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**Source photo**: `photos/WIN_20230103_01_29_30_Pro.jpg`. Rebuilt 2026-10-21 by the REBUILD.md method. Work files:
`.heirloom_work/rb_spearhead/` (`notes.md`, `proto2.py`, `imodel.py`, `gen_tables.py`, `cmp_final.png`, `sheet.png`).

## Phase 2 (v0.6.0)
No notes from Ron for spearhead (free hand, 2026-11-09 batch). The v0.5 knobs were already bold element parameters
(spear count, taper, band width, row height, bend, thrust), so this round turns the **spears into separate voices**
and replaces the two switches that only removed or flipped the whole picture:
- every spear gets its own core offset, so the spears can plunge and swell independently (a depth wave runs along
  the row on K4, past the photo's speed);
- every second spear can point up (S10 Facing), an interleaved zipper of up and down spears;
- the mosaic shadow can be thrown to both sides (S7 Both);
- K1 reaches 8 spears across (was 5).

Every new effect is exactly zero at the default knob value; the default frame is raw-identical to v0.5.1
(checks below). Phase-2 working files: `.heirloom_work/rb_spearhead/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| K1 Spears: 1 .. 5 across (log), 2.04 = photo | **K1 Spears**: 1 .. 2.04 (photo, same position, exact) .. **8** across; the left half of the curve is unchanged, the right half is steeper | spear spacing 1920 px (one centred spear) .. 940 px (photo) .. 240 px. From about 4 across the outer bands of neighbouring spears overlap; at 8 the top of the frame is a lattice of interlocking bands with 8 tips below |
| K4 Speed: bipolar, rolls the row colours, shadow wave and chevrons; still at centre, default slow forward | same motion, **plus a per-spear plunge past the photo's speed** (K4 below 376 or above 648; zero in between, so the default 640 is the photo): each spear's core moves on a triangle depth wave, spear *t* 3/8 of a cycle behind spear *t − 1*, travelling with the speed's sign | amplitude 0 .. ±305 px of core at either end of the knob. At −305 a spear withdraws completely (only its blocky shadow column stays); at +305 it swells into its neighbours and its tip drops off the bottom edge. The wave cycle runs 5.3 s at the threshold .. 1.4 s at the ends |
| S7 Shadow: On / Off | **S7 Shadow: Right / Both** (renamed values): Both also throws the mosaic shadow to the LEFT (the ring sampled D to the right, its wide flank mirrored) | Right = the photo. Both = a second black staircase 80–290 px beyond each spear's left outline, plus mirrored block holes in the right half of the core. The shadow is now always on |
| S10 Point: Down / Up | **S10 Point: Down / Facing** (renamed value): every second spear (odd tiles) points up from the bottom edge, with its own shadow and plunge | Down = the photo. Facing = an interleaved zipper: the up spears' tips sit about 250 lines from the top; at high Thrust the up and down bands interlock |

Unchanged: K2 Taper, K3 Bands, K5 Mosaic, K6 Bend, S8 Rows, S9 Palette, S11 Source (Video / Osc, default Video),
P12 Thrust (still the headline: 0 % short needles .. ~41 % photo .. 100 % interlocking spears; it sets every
spear's core, the plunge rides on top).

**Presets** (preset 0 = the defaults): **Photo**; **Needles** (as before); **Interlock** (as before); **Plunge**
(K4 forward past the photo speed, a little more Thrust); **Zipper** (Facing, ~3.9 spears, more Thrust); **Twin
Shadow** (Both, bigger blocks); **Big Blocks** (as before); **Spear Rain** (8 across, sharper taper, Facing + Both,
K4 reversed into a fast backward plunge). Dropped: Plain Bands (S8 Off on its own; S7 can no longer turn the shadow
off).

**Engine (new).**
- **Spear index.** Each tile DDA (u, the key's uA, the shadow's uB and the new mirrored uC) counts the tiles it
  crosses, mod 8, so every pixel knows which spear it belongs to. Start tiles relative to u: uA one back when
  u0 < 15 px; uB one back or forward when the shadow offset wraps; uC one forward when it wraps.
- **Per-spear core.** At stage 1 each of the four fields takes its spear's line profile (φ, or φ_up for odd spears
  under Facing; the row-held φ_r for the shadow rings) plus that spear's plunge offset δ[t] (an 8 × 12-bit register
  file, synthesized as FFs), registered; stage 2 is unchanged (F = |x| − core − bend). No stage added: latency 16
  in every mode.
- **Plunge offsets.** A 16-bit wave phase advances 1.5 × speed per field. Eight more field jobs on the existing
  serial multiplier (22 jobs, ending at zone clock 471; the line engine's field start moved 330/340 → 490/500)
  compute δ[t] = a × tri(phase + 3t/8) / 512 − a, with a = 1.625 × (|speed| − 136) when positive, else 0, so the
  default gives δ = 0 exactly.
- **Facing.** The line engine runs two more serial products after the down profile (y_up² and × taper,
  y_up = 1079 − y); φ_up is ready at engine clock 75 (HD / SD blanking gives 280 / 138 clocks; the image tester's
  fake 64-clock blanking makes Facing one line late in sims only). The chevron line term no longer flips (it only
  did under v0.5's Point Up).
- **Both.** A fifth tile DDA uC starts at u0 + bw/2 + D (the mirror of uB's bw/2 − D); its ring test swaps the
  flanks (wide on the left), is sampled at the same block starts and ORed into B.
- **K1.** Table entries 29–63 now run 1808 .. 480 Q.1 (geometric); 0–28 unchanged. The tile-origin loop needs at
  most 4 of its 6 wrap steps at 8 across.

**Build** (router2, `.heirloom_work/diag_build.sh spearhead`, 2026-11-09): HD Analog s2 81.36, SD Analog s2 77.33
(s1 stalled the router), HD HDMI s1 78.14, SD HDMI s1 77.53, HD Dual s1 77.33, SD Dual s2 76.18 (s1 stalled) MHz.
6,870–6,904 / 7,680 LC (89.9 %), 4 / 32 EBR; iceunpack OK ×6; the packed config carries the new labels and presets.
No pin-skew screen. Built while ~12 other diag builds and sims loaded the machine (load ~22).

**Checks** (working files in `.heirloom_work/rb_spearhead/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: the new arithmetic at RTL widths over every knob extreme — plunge amplitude 0 at K4 640, max 611 Q.1;
  offsets within 12 bits; core sums −2282 .. 1449 Q.1 (14 bits); F −1955 .. 4708; the mirrored start offset stays
  below the tile period (max 428 < 480); tile-origin wrap ≤ 4 steps.
- Default frame (GHDL 720p60 dec 1, Osc, 2 warm-up + 1 capture, production v0.5.1 rendered the same way): raw
  10-bit output **byte-identical** (`r05_osc` / `r06_osc`).
- One extreme frame (Facing + Both + K4 900, K1 700, Osc: `r06_x.png`): up and down spears alternate correctly,
  each at its own plunge depth, shadows and holes aligned with their spears, no seam slivers.
- Not verified: K1 at 8 across, the Spear Rain preset, the Video-mode looks.

## Reading of the photo (measured)
- **Two spearheads**, centres x = 532 and 1472: one horizontal triangle, period 940 px. Each is a nest of pale pink /
  yellow / olive / hot pink / maroon / blue / navy / black outline bands, wide at the top and converging to tips at
  the bottom.
- **Spear field**: F = |x − xc| − φ(y), φ = 114 − 1.65·10⁻⁴·y².
  - The core edge measures 114/102/92/76/64/50/28/10/−10/−30 px at y = 15…925, within about 5 px of the fit.
  - Band edges sit at F = 0/31/58/87/108/133/157 px.
  - The core and the black outside are where the 3-bit quantiser clips at 0 and 7.
- **XOR colouring**: colour = PAL[i XOR j].
  - j is the band index.
  - i is a 3-bit code per 41.15-line row (first boundary at y = 9.4). It comes from a vertical triangle, period 658
    lines, peak at y = 421, clamp(⌊8.3 − 9.6·|2θ−1|⌋, 0, 7). Rows 0–25:
    `0 0 0 0 1 2 4 5 6 7 7 6 5 4 2 1 0 0 0 0 1 2 4 5 6 7`.
  - Verified: j = code XOR i is a clean contour field over the whole frame, and each row's i scores 0.7–0.96 against
    under 0.2 for every alternative.
- **Mosaic shadow**: key = A XOR B.
  - A = (F taken 15 px earlier) < 147.
  - B = the ring 80 ≤ F < 100 on the left flank, < 220 on the right, sampled at block centres. Blocks are 40.9 × 41,
    23 per tile, locked to the tile. Each row is displaced right by D = 70 + 64·cos (period 16.75 rows, RMS 18 px).
  - 96 % of blocks agree.
- **Background**: a row colour from the 4-bit phase of the row oscillator, plus 0–2 green steps of (−35, +30, −23) RGB
  from a triangle of u = |x − 994| + 0.7y, period 1350. Mean error 44 RGB, against 45 for a free colour per row.
- **Texture**: edges soft over 6–10 px; the fringes are blends of neighbouring bands.

## What it does
- **Per field**: the knob curves, the band thresholds × the Bands scale and the row-oscillator steps, all on one
  serial shift-add unit; the tile origin; the Thrust glide; the Speed drifts.
- **Per line** (the engine computes the next line in about 40 clocks):
  - φ = C0 − y² × taper;
  - the row DDA, with the row-code and shadow-wave phases;
  - i, the background phase, D, the block-sample offset and the chevron line term.
- **Per pixel** (16 stages):
  - four Q.1 tile DDAs plus a block counter restarted every tile;
  - F, then seven comparators give j;
  - A, and B latched per block;
  - the chevron level;
  - PAL[i XOR j], or the row colour plus steps;
  - the key mux;
  - the shell's output conditioning and blanking gate.
- The shell is the heirloom v0.2.0 one: generated avid, timing-ID virtual-1080 steps, vblank zone.

## Controls
| Control | Photo element | Effect |
|---|---|---|
| K1 Spears | the two spearheads (tile period) | 1…5 across (log); 2.04 = photo; a single spear is centred |
| K2 Taper | the outline curve (k·y²) | blunt columns … short arrowheads |
| K3 Bands | the nested outline bands | width ×0.4…×3; the key and shadow thresholds follow |
| K4 Speed | row re-colouring, shadow wave, chevrons | bipolar, still within ±8 of centre; default slow forward |
| K5 Mosaic | 41-line rows and 41-px shadow blocks | 12…160 lines (log) |
| K6 Bend | the source | luma about mid-grey bends the band contours and chevrons; default 64 = ±14 px |
| S7 Shadow | the mosaic shadow and holes | On / Off |
| S8 Rows | the row re-colouring | On / Off (plain nested bands) |
| S9 Palette | band and background palettes | Photo / Alt (channels rotated) |
| S10 Point | spear direction | Down / Up |
| S11 Source | — | Video (default) / Osc |
| P12 Thrust | the spear core C0 | 0 % short needles from the top · ~41 % (default 424) = photo · 100 % neighbouring spears' bands interlock; glided 1/8 per field |

## Video
Source luma is held in a 1-line buffer, centred on mid-grey and scaled by K6. It is added to the spear field and the
chevron field: bright parts swell the bands, dark parts pinch them. Osc writes a moving triangle pattern into the same
buffer, so Bend stays live in Osc mode. At the default K6 the camera only bends and tears the contours (the photo
has no camera content); turn K6 up for a stronger effect.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 83.33, SD Analog s1 83.39, HD HDMI s1 81.35, SD HDMI s1 75.57, HD Dual s1 78.05, SD Dual s1 73.40 MHz; 5812-5829 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
v0.5.0 built 2026-10-21. router2, all six configs on seed 1; all bitstreams pass iceunpack.
- HD Fmax: Analog 82.10, HDMI 81.63, Dual 79.26 MHz.
- SD Fmax: Analog 80.98, HDMI 79.88, Dual 75.81 MHz.
- About 5.83k / 7,680 LC (76 %), 4 / 32 EBR. Latency 16 in every mode.

Not flashed. The presets are hand-written placeholders: Photo, Needles, Interlock, Plain Bands, Big Blocks.

## Deviations from the photo
- **Background V triangles.** The crisp small V triangles between the spears (tips at y ≈ 300 and 910) aren't
  reproduced. The background uses a broad least-squares chevron, and the colour order down the rows is right. Six
  simpler background models fitted worse; they are logged in `notes.md`.
- **Small details dropped.** The shadow wave is a triangle standing in for the fitted cosine. The 1-line glitch
  streaks and the 9-line top strip are not reproduced.
- **Pale pink is dimmer.** It is capped at about 768 output luma by the HD bar fix; the photo has 820.
- **SD crops the sides.** The 4:3 frame crops about 240 virtual px from each side. Both spears stay visible.

## Open questions for Ron (2026-11-09 batch)
1. S7 can no longer switch the shadow off (Right / Both). Do you miss the clean, shadowless spears?
2. The plunge rides on K4 past the photo's speed. Would you rather it had its own control (taking Bend's or
   Mosaic's place), or start from still?
3. S10 Facing replaces v0.5's Up (all spears flipped). Keep Facing, or bring Up back?
4. Is 8 spears across (K1 max) too dense, especially on SD?

## Hardware watch list
- Renamed values: S7 Shadow On / Off → Right / Both, S10 Point Down / Up → Down / Facing. Presets: Plain Bands
  dropped; Plunge, Zipper, Twin Shadow, Spear Rain new.
- K4: the plunge starts just past the default (K4 648 up, 376 down); with the raw knob it begins gently, and pot
  noise at the default can't reach it (±8 deadband + an 8-count margin).
- Thin-ish HD timing: HD Dual 77.33, HD HDMI 78.14 MHz (router2, nondeterministic); HD Analog needed seed 2.
- Facing at high Thrust and many spears gives interlocking up / down bands; check the tile seams for slivers.
- No features cut for fit.

## Next step
Wait for Ron's preview notes. On approval: `./heirloom_diag.sh merge spearhead`, then repack the previewed diag
bitstreams under the production TOML (copy `build/programs/ron/spearhead_diag/rev_b/bitstreams/*.bin` into the
production build dir, regenerate `program_config.bin` from the production TOML,
`vmprog_pack.py --no-sign --hardware rev_b`) instead of re-routing; update the production SUMMARY / README /
PHASE2 row and commit only spearhead's files.


## K6 top +25 % and default 100 % (2026-11-15)

Ron (2026-11-15): the camera effect at K6 = 100 % 25 % stronger; K6 default 100 % (initial 1023, v0.6.2).
- **Classification: bend** (source luma about mid-grey added to the spear and chevron fields), as in production.
- **Change.** `s_bendk = K6/4 + K6/16` (9 bits, 0 .. 318, was 0 .. 255); the bend product is 19 bits; bend
  ±636 Q.1 px (was ±510). The same change as production spearhead (no render for the diag).
- **Build:** router1; HD Analog s1 82.29, SD Analog s1 82.82, HD HDMI s1 77.47, SD HDMI s1 86.71, HD Dual s1 81.73, SD Dual s1 85.34 MHz; worst HD 77.47 MHz; 6895-6903 / 7680 LC; all seed 1. vmprog: out/rev_b/ron/_diag/spearhead_diag_v0.6.2.vmprog.
