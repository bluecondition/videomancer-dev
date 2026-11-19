# Magma — Heirloom #29

**v0.6.3 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/magma.vmprog`, repacked from the diag's own v0.6.3 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.1 (2026-11-03, phase 2): diag built from Ron's notes on v0.6.0, awaiting HW** (`Magma Diag`,
`out/rev_b/ron/_diag/magma_diag_v0.6.1.vmprog`). Production v0.5.1 is untouched. See [v0.6.1](#v061-ron-2026-11-03)
and [Phase 2](#phase-2-v060).
v0.6.0 (2026-11-02): first phase-2 diag, flashed by Ron 2026-11-03.
v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed. Source photo:
`photos/WIN_20230515_00_46_28_Pro.jpg` (1920×1080 capture, 2023-05-15). This replaces v0.1.0 (three
summed triangle waves through a palette), which Ron judged too abstract. Presets are hand-written
placeholders.

## v0.6.1 (Ron, 2026-11-03)
Ron flashed the v0.6.0 diag: "yes, bring the P12 tide back, I don't know what it is but I'm hoping it's something
that grows the existing shapes randomly from the bottom like the tide coming in. Everything else looks pretty good
but double the range of flow in both directions."

**P12 Tide (was Lava).** The uniform lava offset is replaced by a shoreline that rises from the bottom of the lava
field. It is computed per knot (every 36 virtual px) in the per-line spline engine and added to the FR knots, so it
grows and shrinks the fitted shapes themselves:
- **Shoreline:** w = tri(a) + tri(b), 0 .. 254 Q5 cells (±143 lines), from two triangle waves of different
  wavelengths (8.8 and 13.5 knots, phase steps 29 and 19 per knot) lapping in opposite directions. Their sum is an
  irregular, moving front, so shapes in a wave crest grow before their neighbours. The lapping rate is linear in
  |Flow|: about 3.9 s per bob at the default, 2.2 s at the ends, still at the Flow centre.
- **0 .. 50 %:** the tide comes in. Lava ahead of the shoreline recedes by (distance / 2), down to −128 key units
  (gone) 288 lines ahead, so each shape grows back from its bottom edge as the front passes. 0 % = no lava (as v0.6.0).
- **50 %:** the photo, exact for P12 491 .. 527 (a dead zone either side of the default).
- **50 .. 100 %:** the flood comes in behind the shoreline, from the bottom: + (distance / 2) up to +72 (v0.6.0's
  full Lava) 162 lines behind it. 100 % = the whole field flooded, as v0.6.0's 100 %.
- It follows S8 Flip (in Ceiling the tide comes in from the top). The pocket offset and the hot cores still follow
  P12 uniformly, as before.
- Model sheet: `p2/sheet_v061_tide.png` (`p2/imodel3.py`, the v0.6.1 integer model; `p2/sheet3.py`).

**K3 Flow: twice the range at both ends.** Past the default the speed curve's slope goes from 4× to 8.5×
(spd = sq + 7.5 (sq − 153)), so the ends are 2.01× v0.6.0's (3,068 vs 1,525 Q16 cycles per field: one band every
0.36 s). Up to the default (800, and its mirror at 224) nothing changes, so the default frame is unchanged. The pocket
rise and the lava heave depth come from the same speed, so they double too: heave ±80 at the ends (was ±39).
f_spd, f_dq and f_rise widened by one bit (swept: no wraps, monotonic).

Nothing else changed. S9 Cross is kept (the ablation showed it costs only 30 LUT / 71 FF / 1 EBR).
Standalone synth: 4,566 LUT / 1,677 carry (v0.6.0: 4,444 / 1,611).

**Build (router1, 2026-11-03, `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh magma`).**
7,621–7,648 / 7,680 LC (99.2–99.6 %, +~150 over v0.6.0), 25 / 32 EBR. Post-route Fmax:

| Config | Seed | Fmax (MHz) |
|---|---|---|
| HD Analog | 2 (seed 1 missed) | 79.57 |
| SD Analog | 2 (seed 1 stalled router1) | 79.96 |
| HD HDMI | 1 | 75.13 (thin: +0.88) |
| SD HDMI | 1 | 87.57 |
| HD Dual | 1 | 83.22 |
| SD Dual | 1 | 76.91 |

The chip is now full: any further feature needs something removed (S9 Cross is the obvious candidate, ~70–100 LC).
No pin-skew screen (the predictor is blind; see memory `hdmi_tx_placement_skew`). If HD HDMI shows lines, re-seed it.
Not simulated: the default frame is unchanged by construction (tide offset 0 for P12 491–527, Flow curve unchanged
up to the default); the tide was judged on the integer model.

## Phase 2 (v0.6.0)
Ron left no notes on magma (free hand, 2026-11-02). The map follows the phase-2 brief:
- wider ranges, with the photo still at the default;
- element and group parameters (the stripe oscillator, the pocket group, the lava rows) instead of a global zoom;
- the two weak switches (S8 Ripple, S9 Mirror) replaced with bold look-flips;
- looser motion: the lava rows heave out of step, and the pockets and lava follow the Flow knob.

Built as a diag copy (`.heirloom_work/rb_magma/diag/magma_diag/`). Working files are in `.heirloom_work/rb_magma/p2/`:
- `imodel2.py`, the v0.6 integer model;
- `sheet_v060.png`, the model's control sheet;
- `sweep.py`, the width sweep;
- `abl/`, the ablations.

| Control | Was (v0.5) | Now (v0.6.0) |
|---|---|---|
| K1 Stripes | 0.5× .. 2×, photo at the centre | **0.25× .. 4×**, exponential: 4 octaves, piecewise-linear exp2, written as bit concatenation. The photo is still at the centre (512), bit-exact. 0 % gives a few huge bands; 100 % gives fine combed stripes |
| K2 Fan | 0 (straight bands) .. 2× the photo's fan, linear, photo at the centre | **−1× .. 3×**, linear (display −100 .. 300 %): 0 % = the fan reversed (the stripes curve the other way), 25 % = straight, 50 % = the photo (bit-exact), 100 % = 3× (a steep sweep). β carries the fan's sign; the per-field multiplier takes the magnitude |2·K2 − 512| |
| K3 Flow | band drift only; square law, still at the centre; ~7 s per band at the default, 2.2 s at the ends | **All the motion.** Bands: the same speed up to the default (800), then 4× the slope, 0.7 s per band at the ends (3.1×). It now also drives the **pocket rise** (3.5 × depth per field: 448 at the default vs v0.5's 447; they sink when Flow is reversed and stop at the centre) and the **lava heave** (±4 key units at the default, as v0.5's breathe; ±39 at the ends; 0 at the centre) |
| K4 Pockets | 0 = none .. centre = photo .. 100 % = +127 offset on the positive table entries (hard to see) | Unchanged up to the centre. Above it, a square-law boost pk4 = ¾·(K4 − 512)²/512 (0 .. 382) raises the amplitude offset and its ceiling (127 → 509, ~4× the photo's fattest pocket), and a quarter of it lifts the non-positive entries into pockets. 75 % = many more and fatter lens pockets; 100 % = a dense red lattice of lens-shaped pockets over the bands |
| K6 Melt | K5 / 32 (0 .. 31; 12 at the default 400) | Unchanged up to 400, then (K5 − 304) / 8, reaching **89 at the top** (2.9× v0.5's maximum, 7.4× the default), held as a 5-bit mantissa plus a shift. The video bends the bands into tall flames, and 1/8 of it swells the lava, so the camera image starts to print as lava |
| S8 | **Ripple:** Fine (photo) / Coarse (wide rings; looked almost the same) | **Flip:** Floor (photo) / **Ceiling**. The whole spline field (the lava key, the band level and the lava green) is read bottom-up: the yellow-cored top-row blobs sit along the bottom, and the red bottom row and the bottom band hang from the top edge; the stripes and the sky stay. The fine ripple is always on |
| S9 Mirror | Off / On (the right half's stripes reversed into chevrons at the centre) | Off / **Cross**: everywhere, the stripes are crossed with their own mirror image about the screen centre (the maximum of the two saws). This gives an X lattice whose mirrored set drifts the other way. Cross works at every Stripes / Fan setting |

Unchanged: K6 Sky, S7 Cores, S10 Palette, S11 Source, P12 Lava (still the headline: 0 = no lava, 50 % = the photo,
100 % = the flood).

**Lava heave (new, always on).** v0.5's ±4-unit breathe was one LFO for the whole field. It is now a triangle that
travels up the rows, one wave per 1,024 virtual lines (heave = tri(lph + 64·line) × depth / 8192, computed per line).
The top row, the lens row and the bottom row therefore swell and shrink out of step. Its depth comes from Flow (see K3).
At the default it is v0.5's ±4 units, so the photo composition holds: the default frame differs from v0.5 only along
the lava outlines.

**Renamed:** S8 is now **Flip** (was Ripple). S9 keeps the name Mirror, with value labels Off / Cross.

**Considered and dropped to fit:** a P12 "tide", where the lava rows recede top-first and flood bottom-first in
between 0 and 100 %. It cost ~100 LC on top of the rest. The first build placed 7,877 LC (102.6 %).

**Fit.** The first build overflowed placement at 7,877 LC. Fitted with no feature of the final map lost:
- The pocket gain multiply became an amplitude-ceiling boost on the existing multiplier: −154 LUT.
- Melt became a mantissa and shift.
- The K1 / K2 / Flip curves became wiring, with the one's complement on the reversed side.
- The heave triangle became wiring, and the pocket rise a shift-add.
- The kB / kR / lava-green delay chains (152 FF) went into one 256 × 16 block-RAM ring (stage 10 → 16), and the
  luma delay (50 FF) into a second ring (stage 23 → 27).
- The pixel-path depth is unchanged (28), so the U/V parity is unchanged.

**Presets.** Photo (the defaults), Calm Stripes, Lava Flood (K4 640), Crosshatch (S9 Cross, Stripes 70 %, Fan 800),
Ice Night, **Ceiling** (S8 Flip, Lava 63 %), **Boil** (Flow max, Pockets 88 %, Melt 70 %) and **Low Tide** (Lava 29 %,
Stripes 29 %, Fan reversed). All are placeholders, to be captured from hardware.

**Build (router1, 2026-11-02, builder 6/6 seed 1).** Post-route Fmax:

| Config | Fmax (MHz) |
|---|---|
| HD Analog | 91.67 |
| SD Analog | 87.95 |
| HD HDMI | 83.77 |
| SD HDMI | 88.65 |
| HD Dual | 85.34 |
| SD Dual | 89.01 |

7,482–7,492 / 7,680 LC (97.5 %, +~425 over v0.5.1), 25 / 32 EBR (+3: the mirrored wave ROM and the two delay
rings). No pin-skew screen: the builder seeds were kept. Rebuild with
`NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh magma`.

**Checks run.**
- **GHDL** analyse, elaborate and synth front end: clean.
- **Python width sweep** of every new knob curve and product at the RTL's widths (`p2/sweep.py`): no wraps, and fl
  stays positive over the whole K1 × K2 range.
- **Integer model** (`p2/imodel2.py`): the control sheet is `p2/sheet_v060.png`. The K4 and K2 curves were tuned on
  it, and the default differs from v0.5 only along the lava outlines.
- **Two GHDL default-frame renders** (Osc, 720p60 dec 1, 2 warm-up frames), compared with v0.5's `rtl_osc.png`
  (`p2/cmp_rtl_osc_c.png`):
  - Stripes, pockets and lava placement are unchanged; 11 % of pixels differ by more than 12/255, all along the lava
    outlines, from the per-row heave phase.
  - The fitted build renders identically to the pre-fit RTL (0.02 % of pixels, heave rounding at lava edges). Zero
    shift is the best match, so both block-RAM delay rings are aligned.
- **Not simulated:** the new controls away from their defaults (judged on the model), video mode, and NTSC. Hue is
  for the hardware to judge.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_magma/` (`notes.txt` has every number).
- **Composition.**
  - Top half: a cyan field (0, 250, 230) with pale pink streaks sweeping down-right. The fan runs from
    about 70° on the left to about 30° at the top right, with orange/red pockets in the streaks.
  - Bottom half: a blue field with magenta ")" bands, vertical near y 600–700 and leaning left further
    down.
  - Lava blobs, yellow with red rims in the top row and red in the bottom row:
    - top-left x 0–215, y 0–500;
    - top-centre 872–1202 / 81–433, tilted;
    - right 1658–1919 / 121–446;
    - bottom-left 0–250 / 530–1000, with a pointed ">" edge at (250, 770);
    - bottom-centre 837–1184 / 565–966;
    - bottom-right 1700–1919 / 680–850.
  - A row of pointed red lenses at y ≈ 515: x 115–215, 352–666, 816–966 and 1203–1773, with an
    X-pinch at (130, 515).
  - A red band along the bottom edge with a wavy top, and yellow bits along the top edge.
- **Mechanism (from the channel split).**
  - R and B are steep keys of ONE field: blue/cyan below magenta below red. The B-off area lies inside
    the R-on area for 99 % of pixels.
  - G falls with the same field and with height.
  - The lava is a separate key with its own green channel.
  - k-means 12 shares: blue 17.4 %, red 16.6 %, cyan 15.1 %, magenta 10.0 %, violet 9.6 %,
    azure 7.5 %, yellow 6.1 %, lavender 5.9 %, pale cyan 5.8 %, orange 2.9 %.
- **Band oscillator.**
  - The per-row band period fitted over the picture: 133.5 px (y 20) → 159 (260) → 180 (400) →
    198 (640–720) → 191 (900) → 181 (1020).
  - The phase at x = 0 is 247·f(y), with rms 0.02 cycle. The oscillator is hsync-reset with phase 0 at
    x = −247 px.
  - Waveform: a sawtooth that rises sharply over ~0.15 cycle and falls slowly over ~0.6. Blue→magenta
    edges sit at phase 0.8 (median residual 0.03 cycle).
- **Colour laws.**
  - G = clip(Gc − kG·(F − F0)⁺). Gc runs 0.99 → 0.06 from top to bottom, F0 0.35 → −0.4, kG 0.4 → 0.1.
  - B maximum 0.906 (top) → 1.0. Magenta R 0.93–0.98.
- **Lava.** The signed distance of the blob mask is fitted as a quadratic B-spline on a 36-px grid
  (56 × 33 knots), 99.15 % pixel accuracy. The lava green is a second spline, rmse 0.075 inside the
  blobs; a green-vs-depth law managed only 0.20.
- **Pockets.**
  - Share of pixels outside the blobs that are pocket (B off), per 180-line zone: 1.8 / 0.5 / 0.9 /
    2.2 / 1.5 / 2.4 %.
  - Stacked roughly every 33–40 lines.
  - No global vertical oscillator: the best phase coherence over periods and tilts was 0.42.
- **Texture.** Residual correlation length 8–10 px in x and 5–6 lines in y. No global line tear; row
  shifts of 0 / +1 px are just the slant. Fine contour "fingerprint" rings run around the pockets.
- **Fit scores.** The float proto scores 4×4 region means max 18 / mean 6.6 of 255 and local contrast
  0.89 of the photo. The RTL-faithful integer model scores 7.9 / 0.83 (luma scale off).

## What it does
1. **Bands.** One line-locked sawtooth oscillator.
   - Its per-line frequency is fl = α + β·T(v), where T is the fitted curve (512 entries, one every
     2 virtual lines). β = Stripes × Fan and α are worked out once per field.
   - The phase starts at fl·247 at the left edge (fl·487 at SD, which begins 240 virtual px in) and
     steps fl per virtual px.
   - A per-line comb jitter is added to the start phase.
   - With S9 Mirror on, the oscillator runs backwards after the centre pixel.
2. **Band field.**
   - F = wave(phase) + BG/2 + pockets + melt, where BG is the fitted static band level.
   - Pockets ride the bands: a per-band, per-slot amplitude from a 32 × 32 table, shaped by
     min(band-phase trapezoid, vertical slot triangle), ×2.5. Slots rise about 18 virtual lines per
     second.
   - A contour ripple, tri(F·12), is added where F > 0.16 (S8 Coarse: tri(F·3), twice as deep).
3. **Colourizer.**
   - R = clip(121 + 3·(F − off − 0.5)).
   - B = clip((1.9 − F)·8) capped at the row's B maximum.
   - G = clip(Gc − kG·(F − F0)⁺), with {Gc, F0, kG} read for the Sky-shifted line.
4. **Lava.**
   - Key = FR + Lava offset + breathe LFO (±4 px, 9 s) + melt/8.
   - The B and G mix uses the key; the R mix uses the key delayed 6 virtual px, which gives the
     magenta/dark fringes.
   - Lava green = GB plus a hot-core term above 50 % Lava (S7 Flat: dark red).
5. RGB → YUV in the hardware convention, then the output conditioning and the blanking gate.

## Controls (v0.6.1)
| Control | Photo element it drives |
|---|---|
| **P12 Tide** | The lava blobs, the lens row, the bottom band and the pocket density, through a ragged, lapping shoreline rising from the bottom. 0 % = no lava, a calm striped flow; up to 50 % the shapes grow back from the bottom up as the tide comes in; 50 % = the photo; above, the flood comes in behind the shoreline; 100 % = a yellow-hot sea with stripe windows. Glided. |
| K1 Stripes | How many bands: 0.25× .. 4×, exponential, centre = the photo. Glided. |
| K2 Fan | The streaks' fan: −1× (reversed) at 0 %, straight at 25 %, the photo at 50 %, 3× at 100 %. Glided. |
| K3 Flow | All the motion, bipolar, still at the centre ±8: the band drift (positive streams the stripes right; the default 800 ≈ one band every 7 s, 0.36 s at the ends), the pocket rise (they sink when reversed), the lava heave depth (±4 at the default, ±80 at the ends) and the tide's lapping rate. |
| K4 Pockets | The red pockets in the bands: 0 = none, centre = the photo, 100 % = fat lens pockets in every slot (a red lattice). |
| K6 Melt | How far the video (or Osc) bends the bands and swells the lava. Default 400; 7.4× that at the top. |
| K5 Sky | The cyan/blue horizon. Low = night blue/violet with red lava; high = cyan everywhere with yellow lava. |
| S7 Cores | Hot (yellow cores with red rims, the photo) / Flat (red lava). |
| S8 Flip | Floor (the photo) / Ceiling (the lava field upside down, hanging from the top). |
| S9 Mirror | Off / Cross: the stripes crossed with their mirror image (an X lattice). |
| S10 Palette | Magma (the photo) / Ice (green/yellow stripes, blue lava). |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** the camera luma (2-px average, (Y − 512)/2, tapped straight from `data_in`) × Melt is
  added into the band field, so the bands bend around the picture. One eighth of it goes into the lava
  key, so the lava swells on bright areas. At default knobs the photo's composition still reads.
- **Osc:** a triangle of the beam's cell coordinate (288 virtual px period) plus a vertical triangle
  that drifts per field, fed through the same path. Melt stays live.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks):
  - Glides (±4 deadband, 1/8 ease) on K1, K2 and P12. K3–K6 and the switches are latched.
  - One serial shift-add multiplier computes: speed sp²>>9, β = (ks·fan)>>9, qq = (ks·|512 − fan|)>>9,
    and α = ±(f0·qq)>>9.
  - Also updated: the drift, the LFO, the pocket rise, and the comb seed (every 8 fields).
- **Per line** (kicked at the source avid rise of line n; computes line n + 1):
  - One MAC lane blends three control rows into 56 knots × 3 fields (FR, BG, GB) in 504 clocks, so it
    fits SD's line.
  - It writes 55 cell-boundary values B[i] = (D[i] + D[i+1])/2 into a ping-pong RAM.
  - The scalars: the T(v) ROM; fl = α + β·T and ph0 = fl·(247 + u0) on a second serial multiplier;
    dph by shift-add; the colour row {Gc, F0, kG, pocket gate}; the pocket offset; the comb jitter.
  - Everything is handed to the line registers at hsync, which also preloads the cell window.
- **Pixel path, 28 stages:**
  - Beam: cell coordinate (Q16) and band phase (Q19).
  - Cell-crossing window shift, linear-in-x evaluation of FR, BG and GB.
  - Wave ROM.
  - Pocket slot → table ROM (+ fabric copy), min shape, offset, ×shape, ×2.5.
  - F; ripple; colourizer; lava key and green with their delay pipes; lava mix; Ice swap.
  - RGB → YUV: luma 66/129/25; chroma 112/96/16 and 112/40/72.
  - Output: chroma box-8 pair [1 2 2 2 2 2 2 2 1]/16 over a block-RAM ring, luma [1 2 1], luma ×13/16
    (white ≈ 777), blanking gate.
- **Memories (22 EBR):** spline control points 2048 × 20, frequency 512 × 17, colour rows 256 × 26,
  knot RAM 128 × 30 (ping-pong, `no_rw_check`), pocket table 1024 × 8, wave 256 × 8, chroma ring
  256 × 20 (`no_rw_check`), sync ring 256 × 3.
- **Contracts:**
  - Generated avid (parity-locked).
  - One 28-clock latency in every mode, with the syncs in the block-RAM ring.
  - The HD HDMI bar fix before the gate.
  - Virtual 1080-line geometry; SD shows the central 4:3.
  - Colours in the hardware convention (U = Cr, V = Cb).
- **Fit history.**
  - The first RTL (three MAC lanes, quadratic-in-x, per-line op-coded FSM, per-pixel LFSR jitter):
    6.7 k LUT / 3.7 k FF standalone, which would have been about 9 k LC.
  - Ablations and the restructures above brought it to 4.1 k LUT / 2.7 k FF.
  - The last critical path (a clamp feeding the hot-core multiply in one stage) was split.
- **Verification** (GHDL 720p60 decimation 1, against the RTL-exact `imodel.py`):
  - Static frame: 91.4 % of pixels within ±12/255, after the tester's 4-px capture offset.
  - Motion at Flow max: frame 5 matches the model advanced 5 fields (90.1 %) versus 55.6 % un-advanced.
  - Video and NTSC renders checked.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). The once-a-field zone reseed is replaced by the per-line resync. Post-route Fmax: HD Analog s1 90.93, SD Analog s1 85.32, HD HDMI s1 88.04, SD HDMI s1 86.93, HD Dual s1 84.21, SD Dual s1 86.93 MHz; 7052-7081 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-27, NOT flashed.** **ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh magma`); all six configs seed 1.
- Post-route Fmax: HD Analog 88.04, HD HDMI 86.93, HD Dual 86.82, SD Analog 86.76, SD HDMI 85.22,
  SD Dual 84.79 MHz.
- 7063–7084 / 7680 LC (92.0–92.2 %), 22/32 EBR. All bitstreams pass iceunpack.
- `out/rev_b/ron/heirloom/magma.vmprog`, 478,712 B. A rebuild reproduced identical numbers.
- router2 does not close this design family: an earlier netlist gave 72.5 MHz on HD Analog. Rebuild
  with router1.

## Deviations from the photo, and why
- **Pockets and textures** are random realisations at the photo's measured density, not its exact
  pockets. The photo has no coherent oscillator behind them. The fine band-phase wiggles (±0.07 cycle)
  and the whitish wisps in the top half are smoother or lighter.
- **Lava outlines** are linear in x between 36-px knots, so blobs are slightly polygonal. The lenses'
  long pointed tips and the X-pinch at (130, 515) are blunted (spline resolution), and the green fringes
  on the upper-left of the top blobs are weaker. Quadratic-in-x evaluation did not fit the LC budget.
- **Brightness:** drawn whites and yellows are ≈ 777 because the HD HDMI bar fix scales luma by 13/16,
  so the program reads darker than the photo.
- **SD (4:3)** shows the central 1440 virtual px, so the left- and right-edge lava blobs are cropped.
  720p and 1080 are the same picture.
- **Chroma** uses the 8-sample box in every mode (softer chroma at 720p and SD). The chroma conversion
  coefficients are simplified (mean colour change 2/255). Both were area savings.
- **Static fields:** the fitted lava and band level stay in place. Motion is the band flow, the rising
  pockets, the lava breathe, the re-rolling comb and P12.
- **Interlace:** no bottom-field half-line offset (field_n is tied high on hardware).

## Open questions for Ron (2026-11-02 batch, asked in the hand-over report)

1. Bring the P12 tide back in place of Cross? **Answered 2026-11-03: yes (v0.6.1). It fitted with Cross kept.**
2. Does the +-4 heave at the default still read as the photo? **Answered: "everything else looks pretty good".**

**Watch on hardware:**
- Renamed: Ripple -> Flip.
- Cut for fit: the P12 tide.
- 97.5 % LC: little headroom for a second round.
- No pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config.

**Next step:** wait for Ron's preview notes. Rebuild the diag with `.heirloom_work/diag_build.sh magma` (scoped clean + shared build queue). On approval: `./heirloom_diag.sh merge magma`, `./build_heirloom.sh magma`, update the production SUMMARY / README / PHASE2 row, commit that program's files.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. Here the camera strength was on K5 (Melt), so K5 and K6 swapped: **K6 = Melt** (the camera strength, default now 512 = 50 %), **K5 = Sky** (was K6). Nothing else changed: the VHDL swaps the two knob inputs (the processing is identical), the TOML blocks and preset keys moved with them (the Controls rows above are renumbered). v0.6.2, rebuilt with router1: all six seed 1; HD worst 82.19 MHz (HD Analog 82.19, HD HDMI 83.08, HD Dual 84.36), LC LC. Rebuild: `NEXTPNR_ROUTER="--router router1" ./heirloom_diag.sh build magma`. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: a slightly stronger camera effect at the top of K6, default K6 = 100 %. **Bend type**: K6 Melt adds camera luma × gain to the band phase and the lava field, so the bands bend and the lava swells; the picture itself is never shown.
- Production magma got g + g / 4 on its linear Melt gain. This diag's Melt curve is piecewise (K / 32 up to 400, then (K − 304) / 8 as a mantissa << shift), and it sits at 99.6 % LC: the full ×1.25 version (a 1.25 K register, a second mantissa subtractor, a 6-bit mantissa and a wider melt multiplier) came out at 7,716 / 7,680 LC and failed to place on both routers.
- So the diag uses the knob's top segment instead, at zero logic cost. For K6 ≥ 816 the mantissa is 16 + (K − 816) / 16 (one bit finer; it was 16 + (K − 816) / 32). Melt is unchanged up to 816, then ramps to 112 at 1023 (was 89: ×1.27 at the top). The curve stays continuous and monotonic; its slope doubles above 816. |258 × 28| / 16 = 451 still fits the 10-bit melt term. Latency unchanged.
- The TOML default (K6 initial 1023) and version were set by the orchestrator (v0.6.3).
- Build (router1, 2026-11-15): all six seed 1; HD worst 77.49 MHz (HD Analog 80.42, HD HDMI 77.49, HD Dual 87.73); 7,656 / 7,680 LC max (99.7 %). `out/rev_b/ron/_diag/magma_diag_v0.6.3.vmprog`.
