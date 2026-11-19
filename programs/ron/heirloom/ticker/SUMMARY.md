# Ticker — camera video through a sample-and-hold mosaic (Heirloom #20)

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/ticker.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.1 (2026-11-15): K6 camera pass on the diag copy, built, NOT flashed** (see [K6 camera pass](#k6-camera-pass-2026-11-15-v061-diag)).
**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Ticker Diag`, `out/rev_b/ron/_diag/ticker_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/ticker/`) is untouched.
**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Source: `photos/WIN_20230319_01_27_56_Pro.jpg`.
v0.1.0 (hashed bar columns scrambled sideways) was judged too abstract. This version runs the camera
through the photo's processing, and Osc mode substitutes a generator with the photo's statistics.
Working files: `.heirloom_work/rb_ticker/` (`notes.md`, `proto2.py`, `imodel.py`, `sheet.png`, `cmp_final.png`).

## K6 camera pass (2026-11-15, v0.6.1 diag)
The same K6 camera design as production v0.5.2 (see `programs/ron/heirloom/ticker/SUMMARY.md`, "K6 camera
pass"): K6 = Camera, the source value becomes 320 + (v − 320) · k / 512 before the cell RAM (Osc wedge exempt):
0 % = flat grey cells, 50 % = exactly the v0.6.0 look, 100 % = 2× contrast.
- **Weak control here:** S9 Edges (Soft / Hard: the smallest change of this copy's controls; S8 is already
  Grid and S10 Mirror, both bold). K6's old Shadow moved to **S9 Shadow**: Photo (old K6 at 512) / Deep (old
  K6 at 800: the Osc wedge 1.56×, and this copy's steeper Video crush, below 0.36 L). Edges is fixed at Soft
  (which also keeps the Hard option's crisp moving edges, an HD HDMI bar risk, out).
- Presets: Kaleido's K6 700 (bigger wedge) became S9 Deep; Slits lost Hard edges.
- **Latency 34 → 36** in every mode (`processing_delay_clks` 36; even, so U/V parity is unchanged): v0.6.0
  had already spent v0.5's two pad stages on its delay rings, so the two K6 stages (product, add / clamp)
  needed two more. The cell-RAM read address, the mirror replay and every later stage index moved two later.
- **Build** (2026-11-15, `.kinetic_work/tools/k6build.sh diag ticker`): **router1**, all six on seed 1: HD Analog
  83.71, SD Analog 87.80, HD HDMI 81.96, SD HDMI 87.37, HD Dual 83.80, SD Dual 87.94 MHz (worst HD 81.96).
  7,139–7,174 / 7,680 LC (93 %), **7 / 32 EBR** (was 8: with Edges fixed at Soft the 9-back luma ring for Hard
  edges is unread and gets pruned). `out/rev_b/ron/_diag/ticker_diag_v0.6.1.vmprog`.
  - The S9 Shadow constants are per-state constants: written as shift-adds on the two-valued setting, its bits
    were copies of one switch flag (SB_CARRY I0 == I1, the router1 self-add hang; the first build's SD Dual
    stalled on seed 1). The rebuild above is the fixed netlist.

## Phase 2 (v0.6.0)
No notes from Ron for ticker (free hand, 2026-11-09 batch). Most v0.5 knobs were already bold (Cells, Rows, Gaps,
Shadow and Hold all restructure the picture), but two switches barely showed: S8 Roll/Lock only stopped the rows
rolling, and S10 Two/One clocks only changed the row slivers. The mosaic was also one rigid grid: every row had
the same cells, on the same phase, standing still. The round:
- makes the **rows modules**: a per-row engine gives every row its own cell clock (S8 Mixed), its own scroll rate
  (the ticker tape, K4 past its default) and its own grid phase, hash-key base and highlight offset;
- re-patches the two weak switches into look-flips: S8 Grid Mixed (rows of different mosaic resolutions) and S10
  Mirror (the right half replays the left half's cells: a kaleidoscope of the camera);
- widens K1 to an exponential ×0.25..×4 and gives K3, K5 and K6 a second, bigger half.

Every new effect is exactly zero at the default knob values; the default frame is byte-identical to v0.5.1
(checks below). Phase-2 working files: `.heirloom_work/rb_ticker/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| K1 Cells: 12 .. 100 cells across, linear (41.8 = the photo at 346) | **K1 Cells**: the H clock, **×0.25 .. ×1 .. ×4, exponential** (an octave per quarter turn); default moved **346 → 512** (same picture); readout 0–100 % | 10.4 cells across (184 px cells, a few huge blocks) → 41.8 (the photo's 46 px) → 167 (11.5 px: fine grain, which Soft blurs into streaks) |
| K3 Cyan: window 0.30 .. 0.85 | **K3 Cyan**: unchanged at and above 50 %; **below it the window drops one for one and widens by half the distance** | 0 % = window 0.13–0.55: nearly every grey A cell turns cyan (about 45 % of the frame; in Video the camera becomes a cyan silhouette). 50 % = the photo (11 % cyan). 100 % = only the brightest cells, no white |
| K4 Speed: bipolar ±512, still in the centre | **K4 Speed**: unchanged within ±128 of the centre (the default 640 = +128 exactly); **beyond, the slope triples** (ends ≈ ±1280, 10× the default) **and every row scrolls sideways at its own rate** 1..4 (row hash; K4 sets the direction), none in 384..640 | the ticker tape: at the ends rows slide at 440 px/s (rate 1) .. 1770 px/s (rate 4, about a screen width a second); rows roll ~4.5 rows/s, highlights hop, held bands drift. Back in 384..640 the rows snap back into alignment |
| K5 Gaps: B-half gain 0 .. 0.87 | **K5 Gaps**: the same gain **plus the A/B duty**: A = 25 % (k5 0) .. 53 % (the photo) .. 81 % (k5 max) of the cell | 0 % = thin 12 px slits on black (a barcode). 100 % = 37 px A blocks with near-full-level 9 px B halves (almost seamless) |
| K6 Shadow: Osc wedge 0 .. 2×; Video crush to 0.23 luma | Osc unchanged; **Video crush steeper above the default** | Video: 50 % = crush below 0.15 (the photo's law); 100 % = **below 0.51**: the camera's darker half goes black (a hard silhouette poster) |
| S8 Rows: Roll / Lock (the rows stop rolling; subtle in stills) | **S8 Grid: Even / Mixed** (renamed) | Mixed: every row gets its own cell size, ×0.5 .. ×2 of K1's (8 sizes from a row hash), centre-anchored: rows of 92 px blocks beside rows of 23 px grain. The sizes travel with their rows as they roll |
| S10 Clocks: Two / One (the row slivers; subtle) | **S10 Mirror: Off / On** (renamed) | the right half replays the left half's cells mirrored about the screen-centre cell, every line including tick lines: a symmetric mosaic. Video = a kaleidoscope of the camera's left half; Osc = a symmetric V wedge; scroll runs in mirror image |

Unchanged: K2 Rows (8..263 rows, square law, already ×0.15..×4.8), S7 Picks, S9 Edges, S11 Source (Video / Osc,
default Video), P12 Hold (0 = no held bars, 50 % = the photo's three bands, 100 % = the lower frame melts into tall
bars). Lock and the single-clock option are gone: K4 at its centre stands the rows still.

**Presets** (preset 0 = the defaults): **Photo** (renamed from "Default"); **Readout** (Osc); **Ticker Tape** (Osc,
K4 960: rows scrolling, light hold); **Mixed Rows** (Grid Mixed, finer, faster roll); **Kaleido** (Mirror, Osc,
bigger wedge); **Meltdown** (as before); **Slits** (K5 9 %: barcode slits, finer, Magenta, Hard); **Cyan Flood**
(K3 6 %, Mixed). Dropped: Fine Grain, Magenta Hard (→ Slits).

**Engine (new).**
- **K1 curve.** The zone's serial-multiplier slot that computed (w/2)·inc now computes 1427·(256 + k1[7:0]),
  shifted by the octave k1[9:8]: 356 .. 1427 (k1 = 512, exactly the v0.5 photo value) .. 5696 per virtual px.
- **One shared serial multiplier**, in the line engine: the vblank zone slots (k2², rows × ysc, yth × m10, the K1
  curve) and the per-row engine load it at disjoint times; it steps every clock until its operand is used up.
- **Row index.** The line engine counts clock-1 wraps (ticked or held, so P12 doesn't re-number rows) from a
  per-field base that follows clock 1's phase wraps, so a row keeps its size and rate as it rolls. A 256 × 8
  row-hash ROM (1 EBR) gives the size factor (bits 2..0) and the scroll rate (bits 4..3).
- **Per-row engine** (tick lines only; ~58 clocks after the avid fall, inside SD's 138-clock hblank and the image
  tester's 64, and after the beam has finished the line):
  1. inc_v × m/8 with m ∈ {4,5,6,8,10,12,14,16} (8 when Even), clamped at 8191;
  2. the mode scale;
  3. A = (w/2 − 17)·inc_row, giving the start phase C_PHC − A (centre-anchored, as v0.5 did per field);
  4. o = rate·T, where T is the scroll time: Q4 virtual px, 24 bits, +5/16 of the K4 excess per field, reset in
     the photo zone. B = o·inc_v / 16 moves the grid phase, and the hash-key base moves by 2·⌊S0 − B⌋ cells, so
     Osc content scrolls rigidly;
  5. G = o·851 / 16 moves the highlight gate with it.

  The row registers feed the beam directly; they change only in hblank. All of them reduce to the v0.5
  per-field values when S8 is Even and T = 0.
- **Cell RAM** 256 → **512 entries** (2 EBR, `no_rw_check`): the entry counter is 9 bits (482 entries at most,
  with the 8191 clamp at 1080p). The hash key is entry[7:0] + the row base, with entry bit 8 in a key bit that is 0
  at the default.
- **A/B duty**: stage 1 compares the beam phase with a per-field register, 34996 + 36·(k5 − 512).
- **Mirror**:
  - stage 2 latches the entry at the first pixel right of the screen centre;
  - from stage 7, pixels right of it read entry 2·e_c − e, clamped at 0;
  - stage 12 takes the RAM value for them, also on tick lines. Their source entries were written at least a
    half cell earlier in the same line, so there is no read/write collision.
- **Fit**:
  - the luma 16-px box (taps 16 and 9) and the chroma 8-sum delay ride three block-RAM rings (one write address,
    `no_rw_check`, read = write − (delay − 3), plus a fabric register; 4 EBR), replacing 330 FF of shift registers;
  - the two output pad stages are gone, so latency is **36 → 34** in every mode (even, so U/V parity is unchanged).

**Build** (router2, `.heirloom_work/diag_build.sh ticker`, 2026-11-09): HD Analog **s2** 81.84 (s1 missed), SD Analog
s1 76.66, HD HDMI s1 76.89, SD HDMI s1 79.11, HD Dual s1 83.32, SD Dual s1 85.57 MHz. 7,098–7,140 / 7,680 LC
(92.6 %), 8 / 32 EBR; no RAM transparency emulation; iceunpack OK ×6; the packed config carries the new labels and
presets. No pin-skew screen.
**Fit history.** The first build reached 7,631 LC (99.4 %; HD Analog 76.52, SD Analog 75.42, HD HDMI 83.97 MHz on
seed 1 before it was stopped). Standalone-yosys ablation (`p2/abl/`) ranked the per-row scroll at ~270 LUT and
Mirror at ~40, the rest spread out. Fitted:
- the multiplier and its loads moved into the line engine (no operand staging registers);
- the row registers feed the beam directly;
- the scroll offset is one adder used twice;
- every row scrolls in K4's direction (no per-row reverse);
- rows snap home instead of easing;
- the delay rings and the pad removal above.

**Checks** (working files in `.heirloom_work/rb_ticker/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`, every new knob path at RTL widths:
  - defaults exact (incv 1427, key window 643 / 817, speed 128 with zero scroll, A/B split 34996, crush 157);
  - at most 482 entries per line in all modes;
  - the row products fit;
  - mirror addresses in range.
- Default frame (GHDL 720p60 dec 1, Osc, 2 warm-up + 1 capture; production v0.5.1 simulated the same way): raw 10-bit
  output **byte-identical** (0 of 921,600 px), on the first version and again on the final source (rings and
  latency 34).
- One extreme frame (Osc, Mixed + Mirror + K5 200 + K4 1000; `v4_ext.png`): symmetric V wedge, rows of mixed cell
  sizes, slit duty.
- Not verified:
  - Video mode with a real camera (Mirror kaleidoscope, crush, flood);
  - scroll motion over time (3 sim fields give only 15–60 px of offset);
  - SD / 720p visuals.

## Reading of the photo (measured)
The frame is camera video through a sample-and-hold, not a generated pattern:
- **Horizontal S&H, locked to H.** Every cyan blob centre sits on one grid, pitch **45.91 px**, phase 25.8
  (concentration 0.95 over 188 blobs; white blobs 0.87). Rising edges cluster at cell phase −12; falling
  edges spread over +3..+22. Each cell is a **24.5 px "A" half**, showing the sample at full level, and a
  **21.4 px "B" half**, holding an independent sample at **black + 0.44 (v − black)**. B never keys, and
  it is uncorrelated with both neighbouring A halves (the regression residual equals its own spread).
- **Line hold on two free-running vertical clocks.** Every line inside a row is identical. The row edges
  are the union of clock 1 (period **19.76 lines**, phase 14) and clock 2 (**23.28 lines**, phase 38).
  Where the two nearly coincide they leave 1–8-line slivers. About half the cells change at each tick.
- **Held bands.** There are no ticks over 587–643, 745–832 and 902–1001, and one 27-line row at 430. A
  triangle wave (period 168 lines, peak at 610) compared with a falling ramp
  (θ = 0.45 − 0.0015 (y − 610)) fits all four (429–458, 588–644, 746–825, 904–1003). The bottom of the
  frame becomes tall vertical bars.
- **Luma window key.** Plateau luma runs continuously from 0.07 to 0.62, is **empty from 0.63 to 0.93**,
  and white sits at 0.94–1.0. The cyan (rgb 24,236,238; hw10 Y 657 U 139 V 641; plateau Y′ 0.714) fills
  exactly that gap, in A halves only.
- **Soft edges and fringe.** Horizontal transitions take 15 px from 10 to 90 % (a 16-px box); row edges
  are hard. Cr = 0.083 (Y[x+4] − Y[x−4]) (correlation 0.82): pink on rising edges, green on falling.
  The greys are warm: Cr +15 → +8 and Cb −13 → −4 (10-bit) from black to 0.55.
- **Content statistics** (A cells outside the dark region): dark 27 %, grey 52 %, cyan 11 %, white 10 %.
  Cyan and white cells are **never horizontally adjacent**; they sit 2, 3 or 5 cells apart (the row
  spectrum peaks at 2.4 cells ≈ 77 px). The 8-cell mean variance is 0.11 (close to iid).
- **The dark region** (the camera's dark object): an inverted triangle, x 370..1560 at the top with its
  apex near (600, 555). Alternate rows read 0.08 / 0.12 luma, and two columns of dim dashes (x ≈ 745,
  1125) run down the lighter rows.

## What it does
1. **Beam.** A 16-bit cell phase (one cell = 65536) steps by `inc = 34.1 · cells` per virtual pixel.
   The grid is anchored at the screen centre, with a 17-px correction for the edge boxes' pipeline lag,
   so it lands on the photo's phase. A is where the phase < 0.534. Every half cell is an entry (A even,
   B odd).
2. **Rows.** Two 20-bit vertical clocks step once per line, and a carry is a tick. A triangle hold wave
   and a Q8 ramp suppress ticks (P12). S10 drops clock 2.
3. **Sampling.** On a tick line, the first pixel of each half cell takes the source value and writes it
   into a 256-entry cell RAM. Every other line reads the RAM back (the line hold).
   - Video: camera luma in photo-luma units (1024 = 1.0), averaged over 4 px. Darks below the Shadow
     threshold are crushed to black.
   - Osc: a 24-bit add/xor hash of (entry, j), where j = (line + boil + bit-reversed entry) / 32, so
     values re-roll on staggered 32-line steps.
     - Greys come from a 256-entry quantile ROM fitted to the photo.
     - Highlights (0.63..0.95) appear where a 77-px triangle oscillator is near its peak AND the hash
       passes.
     - The dark wedge (ragged edge, row stripes, dashes) goes on top.
4. **Display.**
   - B halves: black + gain (K5) × (v − black).
   - A halves: v in [kc, kc + 0.17) → cyan (or magenta), above that → white (0.83), otherwise v.
   - Then a 16-px luma box and an 8-px key box (S9 bypasses both), the Cr fringe, the warm tint and the
     key-colour mix, followed by the shell's output conditioning and blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Cells | the 46-px cell grid | cells across the frame, 12..100 (41.8 = the photo); centre-anchored |
| K2 Rows | the ~20-line rows | vertical clock rate, 8..263 rows per frame (54.6 = the photo); clock 2 runs at 0.85× |
| K3 Cyan | the cyan picks | key window position: low = cyan floods the greys, 50 % = the photo, high = only the brightest cells turn cyan and white disappears |
| K4 Speed | the drift of the patch | bipolar, still in the centre ±8: rows roll, the held bands drift, Osc highlights hop sideways and values boil |
| K5 Gaps | the dark B half of every cell | B gain 0 (black seams) .. 0.87 (seamless); 0.44 = the photo |
| K6 Camera | the camera itself | camera strength (K6 camera pass, v0.6.1): the source's contrast about a mid grey before the cell hold; 0 % = flat grey cells (Osc wedge stays), 50 % = the v0.6.0 look, 100 % = 2× contrast |
| S7 Picks | the key colour | Cyan / Magenta |
| S8 Rows | the rolling rows | Roll / Lock (the clocks stop rolling; Osc still drifts) |
| S9 Shadow | the dark triangle | the old K6 Shadow (v0.6.1): Photo (old K6 512) / Deep (old K6 800: wedge 1.56×, Video crush below 0.36 L). Edges is fixed at Soft |
| S10 Clocks | the row slivers | Two clocks (the photo) / One (regular rows) |
| S11 Source | the camera | Video (default) / Osc |
| **P12 Hold** | the tall held bars | 0 = none (fine rows everywhere); 50 % = the photo's three bands; 100 % = the lower frame melts into tall bars under a few bands of rows |

P12, K1, K2, K3, K5 and K6 glide (1/8 of the gap per field, ±4 deadband). The control sheet is
`.heirloom_work/rb_ticker/sheet.png`; every control changes the picture.

## Video
The camera goes through exactly the processing in the photo:
- the line hold on the two clocks, with the held bands;
- the H-locked half cells, with the B attenuation;
- the cyan/white window key;
- the soft edges, fringe and warm tint.

Its chroma is dropped, because the photo is luma plus the key colour. Osc substitutes the generator
described above. Every control acts in both modes; K6 is the wedge in Osc and the dark-crush in Video.

## Engine
- **Zone sequencer** (vblank = "no avid for 1024 clocks").
  - It runs the glides, the switch latches and the per-field phase motion.
  - One serial shift-add multiplier computes k2², rows × the timing scale, the hold-ramp slope and
    (W/2 − 17) × inc.
  - Every other per-field constant is a shift-add register at most two adders deep.
- **Per-line engine** (5 clocks after the source avid falls): clock carries, the hold triangle against
  the ramp, the tick, row parity, the highlight phase and the wedge edges. Its results are committed at
  the next avid rise.
- **Pixel path** (36 stages), in order:
  - beam → half cell / entry → wedge distances and stagger;
  - 5-stage hash → grey ROM (EBR + fabric register) → source value → cell RAM / hold;
  - key + B gain (the multiply split hi/lo) → luma scale → boxes → fringe and tint → key mix;
  - output conditioning → gate.
- **2 EBR:** the cell RAM (256×16, `no_rw_check`: tick lines only write, other lines only read) and the
  grey ROM.
- **Contracts:**
  - generated avid;
  - virtual 1080-line coordinates (timing-ID steps; centred grid and wedge — 720p renders the same picture);
  - one 36-clock latency in every mode;
  - output conditioning (luma [1 2 1] + 13/16, chroma 9-tap) before the blanking gate;
  - colours in the hardware convention (u = Cr).

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Also gated: the mid-pipe luma 16-box and the key-flag box (up to ~8 px change at the line edges). Post-route Fmax: HD Analog s1 74.25, SD Analog s1 79.68, HD HDMI s1 80.36, SD HDMI s1 86.50, HD Dual s1 75.67, SD Dual s1 82.07 MHz; 6832-6865 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-21, router2, not flashed.**
  - **Build seeds: SD Analog = seed 2** (seed 1 stalled the router) and **HD HDMI = seed 2** (seed 1
    missed timing). The other four are seed 1.
  - Post-route Fmax: HD Analog 77.19, HD HDMI 78.06, HD Dual 87.96, SD Analog 81.51, SD HDMI 81.51,
    SD Dual 80.82 MHz.
  - Fit: 6830–6870 / 7680 LC (89 %), 2/32 EBR. All six bitstreams pass iceunpack.
  - The earlier build of the same pipeline, before the grid-phase constant changed, closed 6/6 on
    seed 1 (HD 76.5–83.7 MHz).
- HD margins are about 4 %. If a later version misses, try `NEXTPNR_ROUTER="--router router1"` before
  restructuring.
- GHDL renders at 1080p decimation 1, defaults, in exact hardware hues (raw2hw):
  - Osc class mix dark/grey/cyan/white 0.41/0.43/0.08/0.08 (photo 0.40/0.44/0.09/0.08).
  - The cyan grid phase is 26.1 px (photo 26.1).
  - The held bands are 429–458, 588–645, 746–825 and 904–995.
  - Raw cyan output: Y 645 U 149 V 636 (photo 657 / 139 / 641).
- Motion, at 720p, 6 fields apart: the rows roll, highlights hop and the held bars resample, with
  about 14 % of pixels changing per 0.1 s at the default Speed.

## Deviations from the photo
- Whites are 0.83 of full scale, not the photo's 0.96: the shell's 13/16 output scale (the HD HDMI bar
  fix) caps them. Greys and cyan are pre-compensated and match.
- Osc content is a statistical substitute: it has the same class mix, bright-cell spacing, row
  behaviour and dark wedge, but a different arrangement of cells. In the photo the wedge edge is a
  cell staircase from the camera; Osc draws a ragged diagonal.
- In the photo the clocks drift slightly (rows of 22–27 lines in the middle third); here both clocks
  run at a fixed rate.
- The grey ROM is an inline constant in `ticker.vhd`, not a build-hook package, because there is only
  one 256-entry table.

## Open questions for Ron (2026-11-09 batch)
1. Every row scrolls in K4's direction at its own speed. The first build also sent half the rows the other way;
   it was cut for fit (~60 LUT). Bring it back?
2. When K4 returns to the photo zone (384..640) the rows snap back into alignment. Would you prefer a glide back
   (a short rewind)?
3. Mixed sizes are a switch. Would a knob amount be better, or sizes that re-roll over time?
4. S8 Lock and S10's single clock are gone (K4's centre still stands the rows still). Do you miss either?

## Hardware watch list
- Renamed: S8 Rows Roll/Lock → **Grid Even / Mixed**, S10 Clocks Two/One → **Mirror Off / On**; preset "Default" →
  "Photo". K1 Cells' default moved 346 → 512 (same picture; readout now 0–100 %).
- Latency changed 36 → 34. There are three new block-RAM delay rings (luma box, chroma box): wrong or posterized
  colours, or a smeared box edge, would point there.
- HD HDMI 76.89 MHz (3.5 % margin); HD Analog needed seed 2.
- In Video the ticker scroll moves the S&H grid over a still camera image (the camera content can't scroll without
  a frame store); in Osc the content scrolls.
- K1 near max with S9 Soft blurs into streaks; at SD, K5 extremes plus K1 max can give 1-px half cells that
  occasionally merge.
- Scroll time wraps after ~19 min at full speed (one jump).
- Cut for fit: per-row reverse scroll direction; the ease-home (rows snap back).

## Next step
Wait for Ron's preview notes. On approval: `./heirloom_diag.sh merge ticker`, then repack the previewed diag
bitstreams under the production TOML (copy `build/programs/ron/ticker_diag/rev_b/bitstreams/*.bin` into the
production build dir, regenerate `program_config.bin` from the production TOML, `vmprog_pack.py --no-sign
--hardware rev_b`) instead of re-routing (router2 is nondeterministic; HD Analog was seed 2); update the production
SUMMARY / README / PHASE2 row and commit only ticker's files.
