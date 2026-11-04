# Peaks — sunset peaks over a banded sky, reflected in rippling water

**v0.1.0 (2026-11-04): built, NOT flashed.** Heirloom from the video clips, from the still
`photos/vid/WIN_20230210_01_00_21_t3p0.png` (clip WIN_20230210_01_00_21 at 3.0 s). Two yellow peaks rise out of a sky
of diagonal peach/magenta bands and a green/violet weave. Below the horizon the horizon line is replayed with a
sawtooth ripple, and a few H-sync slips smear the sky. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_peaks/` (`notes.md`, `meas*.py`, `proto.py`, `imodel2.py`).
- **Band VCO (sky).** A saw that rises over 7/8 of its cycle, period 946 px. The 2nd and 3rd bands' left edges stay
  946 ± 3 px apart from line 10 to line 370. Its colouriser runs mauve → hot magenta → pink → peach → light peach,
  and the fall is ~96 px wide.
  - The peak leans left 0.7 px per line: 1129 at line 10, 1064 at 190, ~985 at 295.
  - Going down, the band's key threshold rises: 0.58 + 1.05·y/1080 of the saw. The band is 320 px wide at line 20,
    190 at 200, 110 at 300, and gone by ~470.
  - Its palette index drops 1.4 per frame height: peach at the top, coral by line 160, hot magenta by 300.
- **Weave.** A second VCO at ~8.4 cycles per width. The curtains are 205–250 px apart, wider to the right: the
  H ramp is bent ~16 % across the line, like an analog ramp.
  - Its index, 0.24·(1 − cos) + 0.76·y/1080, reads a ring palette: green2 / green / grey / violet-grey / violet /
    blue-violet / mauve / magenta / pink. That gives green and violet-grey at the top, and violet and magenta
    stripes 227 px apart near the horizon.
  - It leans right 0.6 px per line with a ±20 px wobble: the curtain at 1238 on line 10 is at 1401 on line 130 and
    1329 on line 390.
- **Suns.** Two yellow peaks, apex (506, 400) and (1449, 400), 978 px apart. Their centres do not lean.
  - Width 135 px at line 420, 373 at 570, 485 at 600: 0.8 px per line per side, with a rounded apex
    (a hyperbola, r = 60).
  - A soft edge of ~25 px, and a yellow core (255,212,102) fading to peach (255,180,105) at the edge.
- **Slips (static over the clip).** Common H slips of +60 px glided over lines 85–130, −20 at line 316, −20 at
  408, −30 at 468 and +47 at 576. Each has 8–14 smeared rows where the oscillators paused for a stretch of the
  line; the streaks' ends drift per row, which gives the "teeth".
- **Water.** From line 636, every row best-matches the horizon row (it is not a mirror), shifted by −172 to +12 px
  (content moving left) in a sawtooth of ~36–40 rows. The reflected yellow core narrows to a waist near line 780
  and widens again, while the key edge stays.
- **Palette (hw10 Y / U=Cr / V=Cb, k-means share).**
  - violet-grey 359/535/578 (15 %); green 470/443/514 (13 %); yellow 780/624/329 (11 %)
  - yellow-peach 738/659/332; violet 315/659/614; salmon 632/728/413; peach 682/697/362
  - hot magenta 395/883/552
- **Texture.** Edges are soft horizontally (10–90 % over ~10 px). The capture was SD, line-doubled.

## Motion in the clip
- **Static parts.** The bands and the suns do not move over the 3.5 s of the strip; the peach band stays at
  1022–1030 px.
- **Weave.** Sways ±12 px with a ~2 s period.
- **Water.** The sawtooth is clean (40 rows, 172 px, the same phase in m0, m2 and m3). For ~0.5 s around the still
  it breaks into noise: the still is mid-burst, and at t+0.5 s the sky's sections are also slipped by 70 px.
- **Over the clip** (0.5 s → 3 s → 12.5 s) the geometry stays and Ron's knob changed the colouriser: first
  cyan-blue / magenta, then green / violet, then everything pink / orange.
- **What the program does with this.**
  - The bands and suns stay put, as in the clip.
  - The weave sways ±16 px over 128 fields.
  - The ripple's period jitter alternates every 75 fields between full (chaos, the still's burst) and quartered
    (a near-clean sawtooth that shimmers).
  - The reflected core's breath drifts down ~15 lines/s, and the Osc lattice drifts ~75 px/s.
  - In GHDL, two Osc frames two fields apart give an identical sky (mean 0.37 RGB difference) while the water's
    teeth redraw (rows shift 23–54 px).

## What it does
1. **Ramp.** Each line starts the two oscillators at phases the line engine computed. They then step per pixel by
   an increment that shrinks 16 % across the line (the bent analog ramp). The camera's luma (or the Osc lattice)
   is added to both phases (Picture).
2. **Band.** VCO1's saw (rise 7/8) gives v in Q10.
   - The index v − drop1(y) reads the band palette ROM.
   - The key has 5 levels over 72 Q10 units above kt(y) = 0.58 + 1.05·y/1080.
3. **Weave.** VCO2's phase reads a 256-entry 0.24·(1 − cos) table; that plus drop2(y) indexes the weave palette ROM.
4. **Sun.**
   - The band phase is offset 0.175 cycle (the lean cancelled per line) and doubled for Four, then folded to the
     distance from the trough.
   - A 256-entry ROM rounds the apex: sqrt(d² + 60²) − 60.
   - The key is half-width(y) − distance, in 5 levels over 25 px. A 256-entry sun ROM over the key depth gives the
     yellow core.
5. **Blend** (8-bit colours). sky = weave + (band − weave)·k1/4; the tint adds to U or V; out = sky + (sun − sky)·k3/4.
6. **Water.** Below line 636, the engine's sky line holds (Hold) or runs backwards (Mirror).
   - Each line's shift is the slips plus the ripple saw. Its period and amplitude are redrawn per cycle from an
     LFSR, with jitter 0–0.9.
   - The reflected core's half-width breathes ±112 px over 512 lines.
7. **Slips.** Per line: the step slips, the glided slip, and the smear window (hold start and length in output px;
   the oscillators pause there).
8. **Output.**
   - Luma [1 2 1]/4; U and V through one [1 2 2 2 2 2 2 2 1]/16 box. Its delay line is in a block-RAM ring, and the
     running sums resync to neutral after 8 blank samples.
   - Luma 13/16, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bands | the band VCO (and the suns, which ride on it): ×1/4 .. ×4 cycles per width; 2.03 = the clip (512) |
| K2 Curtains | the weave VCO: ×1/4 .. ×4; 8.4 curtains per width at 512 |
| K3 Lean | the vertical LFO into both VCOs: bipolar (±8 deadband). 0 = the lean reversed, 512 = vertical bands and curtains, 700 = the clip (−0.7 / +0.6 px per line), 1023 = 1.6× the clip |
| K4 Picture | how far the camera (or the Osc) displaces the sky: 0 .. 300 px per full luma (square law); 320 = 29 px |
| K5 Peaks | the suns' apex row: at the horizon (0) .. line ~350 (400, the clip) .. the top of the frame (1023: the peaks swallow the sky) |
| K6 Tint | the colouriser's tint: bipolar (±8 deadband). Left adds Cb (towards the clip's cyan/blue look at 0.5 s), right adds Cr (pink/orange, as at 12.5 s); 512 = the still |
| S7 Tears | the H-sync slips and smears: On (the clip) / Off |
| S8 Water | Hold (the clip: the horizon line replayed, rippled) / Mirror (a true rippled reflection of the sky) |
| S9 Suns | Two (the clip) / Four |
| S10 Palette | Sunset (the clip) / Night: blue sky, teal-navy weave, white moons |
| S11 Source | Video / Osc |
| P12 Storm | the water and the slips together. 0 = glassy water, no slips. 400 = the clip (172 px teeth, ~30-line saw, jitter 0.4). 100 % = 400 px teeth ~16 lines apart, jitter 0.9, the slips 3× |

Presets: Clip, Glassy Dusk (Tears off, Storm 0), Mirror Lake, Four Moons (Four + Night), Blue Hour, Gale, Big Peaks.

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer read 4 px ahead, is multiplied
  by the Picture depth and added to both oscillators' phases (the weave's at 4×).
  - The picture bends the bands, curtains and suns sideways, like a camera patched into the VCOs' FM inputs.
  - At the default depth (29 px per full luma) the sunset still reads clearly. At 100 % the picture is embossed
    into the sky and its reflection.
  - With no input, Video mode shows the still.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice, drifting ~75 px/s, feeds the same path, so the sky
  ripples slowly. Picture stays live in both modes.

## Engine
- **Per-field sequencer** (vblank zone): one serial 36 × 16 multiplier, 27 slots of 24 clocks. It computes:
  - the two VCO frequencies (exp2 ROM, ±2 octaves), their per-pixel increments and bend decrements;
  - the lean per virtual line, the displacement depth, the sun half-width at line 0 and the tint;
  - the ripple step, period and jitter, the five slip amounts and the glide step.

  A shared glide (1/8 per field, ±2 deadband) serves K1–K6 and P12.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line with 14 serial products,
  finishing by clock 247, so it fits SD's 858-clock line. It produces:
  - the sky line (hold or mirror), the slips and the glide, and the smear window;
  - the ripple (the reload value and next period, one line ahead);
  - the lean terms, the two start phases and the sun offset;
  - the key ramps (340 / 255 / 46 / 410 × line) and the breath.
- **Pixel path.** 14 stages to the compose and 19 to the output; C_LAT = 18.
  1. DDAs and the displacement.
  2. Displaced phases (Q12).
  3. Saw terms, weave ROM address, sun fold.
  4. Saw value, distance ROM address.
  5. Band index and key.
  6. Key level, weave index.
  7. Palette ROM addresses, sun key.
  8. Sun level, sun ROM address.
  9. Fabric copies.
  10. Differences.
  11. Sky blend.
  12. Tint.
  13. Sun differences.
  14. Compose.
  15–16. Box and [1 2 1]. 17: luma 13/16. 18: copy. 19: gate.
- **ROMs (17 EBR).**
  - Three 512 × 24 palettes (band, weave, sun), each holding two palette families.
  - The 0.24·(1 − cos) table, the apex table and exp2.
  - The 2048 × 8 line buffer and the chroma delay ring (`no_rw_check`).
- **Contracts.**
  - The generated avid, and one latency (18) in every mode.
  - Virtual 1080-line coordinates; SD shows the centre 4:3.
  - Colours in the hardware convention (u = Cr); the running-sum resync.
  - Palette luma pre-scaled 16/13 and capped at 940 before the 13/16 scale.
  - The 8-bit blends wrap mod 256 with no clamp. This is provably safe: every input lies in 6–250, and a model
    sweep over all switch and knob extremes confirmed it.
- **Verified (GHDL, 720p60 real timing ID, decimation 1).** The Osc frame from the final netlist was compared with
  the twin integer model `imodel2.py` after the filters' 4-px delay:
  - Y 75 % exact, 98 % within 4/1023, mean |diff| 0.64; U 72 % exact, mean 0.45; V 74 % exact, mean 0.39.
  - The residual is the model's approximation of the Osc lattice source.
  - The first RTL caught three bugs: a ripple reload that never fired (the water was one long diagonal), a chroma
    ring read one sample late, and a luma tap 2 px off.

## Status
- **v0.1.0 built 2026-11-04 with ROUTER1. NOT flashed.**
  - Command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh peaks`
    (or `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh peaks`).
  - All six configs on seed 1.
  - Post-route Fmax: HD Analog 81.53, HD HDMI 81.12, HD Dual 81.45, SD Analog 82.10, SD HDMI 78.88, SD Dual 83.98 MHz.
  - 7,033–7,048 / 7,680 LC (91.8 %), 17 / 32 EBR. iceunpack OK × 6; the packed config carries the labels and presets.
  - Output: `out/rev_b/ron/heirloom/peaks.vmprog`.
- **Fit history.**
  - v1: three phase DDAs, 10-bit blends, two 9-tap boxes on Y/U/V. Placement failed at 8,972 LC.
  - Moving the sun onto the band VCO, 24-bit phases, 8-bit blends, luma [1 2 1] plus one chroma box (its delay in
    an EBR ring), and the key-ramp shift-adds onto the line multiplier brought it to 7,313 LC (95 %). That version
    passed all six on seed 1 (HD 77.75 / 83.44 / 82.10 MHz).
  - Q12 pixel-path phase adds (invisible: < 0.25 px) and clamp-free blends brought it to the shipped 7,040 LC.
- **Timing history.** router2 missed HD HDMI on seeds 1–4 at 95 % LC (it was stopped there); router1 closed
  everything on seed 1. Stay on router1.
- **Watch on hardware.** No pin-skew screen was run. If HD HDMI shows lines, re-seed that config.

## Deviations from the photo
- **The weave** is one regular oscillator with a triangle wobble. The photo's curtains are more organic: S-curves
  and varied widths, probably a feedback patch.
- **Not drawn:** the thin second magenta band (x 1150 at line 220 → 980 at line 445), the pink rays inside the
  suns, and the capture's dark border.
- **Tears.** Between lines 400 and 636 the photo is a dense stack of small torn rows. The program draws its five
  measured slip events with their smears; P12 scales them.
- **The suns** ride on the band VCO, so they are 946 px apart instead of 978 (less than 2 % of the frame). They
  are placed 0.175 band cycle before the band's trough, with the lean cancelled per line, so they still do not
  lean. The apex sits near line 350 with the soft key, so the visible yellow starts at ~400, as in the photo.
- **Softness.** Edges are softened only horizontally: [1 2 1] luma, 9-tap chroma. The photo is also slightly soft
  vertically (SD line-doubled capture).
- **Motion.** The clip's clean / chaotic water alternation is an LFO (75 fields each), not random bursts.
- **Resolutions.** 720p shows the same picture as 1080p. **SD shows the centre 4:3** (virtual x 240–1680): both
  suns stay on screen, but only ~1.5 band cycles show.
- **Proto.** The proto's lean was fitted with a slightly different curve; the RTL uses a straight lean plus the
  glided slip, as measured.
