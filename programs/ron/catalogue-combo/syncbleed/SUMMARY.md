# Syncbleed — summary

Catalogue combo #131 (seed 20260927): colour bleeding / NTSC dot crawl +
sync loss / horizontal collapse. v0.1.1 (2026-09-13) passed on hardware
("syncbleed is good"), with no later hardware notes. v0.2.0 (2026-10-13) is
the first finalizing pass, built from a review of v0.1.1. It is **not
flashed yet**. The v0.1.1 sources and vmprog are in git 3fa438d and in
`.syncbleed_work/v011/`.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Bleed | chroma smear (IIR) in octave steps: off, 2 .. 128 px (as v0.1); Local and P12 can push it to 256 px |
| K2 Lag | chroma lags luma by 0 .. 63 source px |
| K3 Crawl | dot crawl: subcarrier dots, 188 cycles across the line (~10 px pitch at HD, 3.8 px at SD), phased by hue, strongest on colour, a faint floor on grey |
| K4 Collapse | base squeeze toward the seam, 1x .. 9x (display now shows the factor), glided |
| K5 Seam | collapse centre, bipolar (centre = screen centre) |
| K6 Wave | the squeeze breathes down the picture (1.58 sine cycles a screen in every mode, ~3.6 s a cycle), glided |
| S7 Bleed | Before the collapse (smear squeezed too) / After (screen-space smear) |
| S8 Length | Fixed / Local: two bleed steps per octave of squeeze, so squeezed rows smear longer and stretched rows sharpen |
| S9 Chroma | Same / Fringe (chroma squeezed 1.5x more than luma) |
| S10 Crawl | Locked (still dots) / Drift (the subcarrier slips 270° a field, so the dots crawl as on NTSC) |
| S11 Smear | chroma only / luma too |
| P12 Loss | the squeeze multiplies x1 .. x64, exponentially; bleed +0..3 octaves, crawl climbs. 100% collapses rows to a strip a few pixels wide (about 14 px at HD at the default K4) |

## v0.2.0 changes (from the v0.1.1 review)

Bugs:
- **S8 Length was dead at nearly every setting.** Local added `q / 2048`
  bleed steps, which is zero below 8x squeeze. The defaults never pass 4.5x,
  so the switch changed nothing unless K4 and P12 were both high. It now adds
  two steps per octave of squeeze about 1x (−4 .. +8), so it shows at the
  defaults: the breathing rows smear and sharpen with the wave.
- **The crawl's line flip was reset by the vsync level.** On hardware, vsync
  can read low across active video, which would hold the per-line flip off
  for the whole picture (vertical stripes instead of a checkerboard). The
  line phase is now counted on active lines and reset on the vsync edge.
- **Crawl pushed luma out of range.** Dots were clamped to 0..1023, so they
  went below black and up to full scale (full-scale Y clips RGB and kills
  chroma). Luma is now clamped to 64..940, then a highlight knee brings
  white to ~770.
- **The 1-px crawl did not work at HD.** At 1920 wide, a 1-pixel checkerboard
  is a near-invisible grain. It is also exactly the HD HDMI bar trigger:
  full-swing pixel-to-pixel edges, inverting every frame. The [1 2 1] bar
  fix erases it completely, because [1 2 1] has a zero at Nyquist. The crawl
  is now a real subcarrier: luma += gain × ((U + 64)·sin φ + V·cos φ), with
  188 cycles across the measured width. So the dot phase follows the hue,
  and grey keeps a faint carrier (the +64, the same floor as v0.1). The
  phase flips 180° per line at SD and per line pair at HD (the size of an
  NTSC line on a 1080 screen). At the default Crawl, the dot strength on
  typical saturated colour is about v0.1's.

Mode scaling:
- The dot pitch comes from the measured width (per-field serial divide).
- The wave line step is 103680 / height, so every mode shows 1.58 cycles a
  screen. That is v0.1's HD rate. SD used to show 0.35 cycles, barely a
  wave.

Hardware contracts:
- **HD HDMI bar fix, built in:** [1 2 1] on Y/U/V before the gate. The
  squeeze point-samples the line 2–500 source pixels apart, and the wave
  moves it, which gives sharp moving edges.
- **Per-row U/V parity swap (drape v1.3).** Every row shows the line stored
  one row earlier. On analog sources whose hsync→avid offset wanders, those
  rows had Cb/Cr swapped.
- **Interlace (SD):** the wave phase advanced on every hsync, including
  vblank lines. It now advances on active lines only, and the bottom field
  starts half a step down. HD has no field flag, so HD is unchanged.
- One latency (15, was 14) in all modes. Output is neutral outside avid
  before the smoothing, and gated again at the output.

Controls (bold, always-on):
- **P12 Loss is exponential, x1 .. x64.** In v0.1 it added squeeze linearly
  (+16x), so most of the visible change happened in the first 20% of the
  slider. 100% was a ~110-px strip at HD, not a collapse. Now the change is
  even across the travel: about 1.5x at 10%, 8x at 50% and 64x at 100%, on
  top of K4 and the wave. The squeeze is capped at 512x (the read DDA was
  widened to 31 bits).
- **K4, K6 and P12 run at 10 bits, glided (¼ per field).** v0.1 used 8-bit
  steps: one P12 step moved the strip edges 15–60 px at HD, so slider and
  pot noise made them jump.
- Bleed has one more octave of headroom (256 px) for Local and P12. The K1
  steps themselves are unchanged.

Optimizations:
- `no_rw_check` on all 6 line-buffer RAMs (a bank is only read while the
  other is written). yosys's read-before-write emulation is gone
  (`emulate_transparency` 0).
- Removed the dead line counter and the vsync-level crawl flip.

## Hardware checks for v0.2.0

1. **HD default:** does it still look like the v0.1 you liked? Collapse,
   wave and bleed should match. Whites are dimmer (knee), edges a touch
   softer ([1 2 1]), and the crawl dots are new.
2. **K3 Crawl sweep** on a colourful source. Dots should be visible, about
   5 px wide at HD, with a hue-dependent pattern. S10 Drift should make them
   crawl; Locked should hold them still. Are they too coarse or too strong?
3. **S8 Local at the defaults:** the breathing rows should smear more where
   squeezed and sharpen where stretched.
4. **P12 sweep:** an even collapse over the whole slider, ending in a thin
   strip with streaks. Try the Total Loss preset.
5. HD HDMI: any thick bars? Analog colour source: any thin coloured lines?

## Review backlog (not done)

Tier 2, look changes (your call):
- **Black sides / beam.** Squeezed rows edge-fill with the line's edge
  pixel, so the sides become horizontal streaks. A real collapsed set shows
  the strip on black, brighter as the beam concentrates (as vhold's Beam
  does). This would be an option, or a different P12 climax.
- **Even breathing (log-domain wave).** The wave is linear in q, so the
  stretch half clamps at 4x zoom (q = 64) for ~18% of the cycle at the
  defaults while the squeeze half reaches 4.5x. A multiplicative wave would
  breathe symmetrically.
- **Exponential K4 Collapse.** Most of the visible change happens in the
  first quarter of the knob.
- **Longer, continuous Bleed.** K1 is 8 octave steps up to 128 px (6.7% of
  HD). Half-octave steps would need a third adder in the IIR's one-clock
  loop, which is a timing risk at 74.25 MHz, so it would want care.
- **Sync-loss jitter.** "Sync loss" today is only the collapse. A per-line
  horizontal tear / hook that grows with Loss would sell it more.
- **Comb-filter crawl.** Real comb-filter dot crawl sits only on vertical
  colour transitions (a 1-line chroma difference). Today's crawl is the
  notch-filter kind: dots on all colour.
- **Lag in screen pixels.** Lag is in source pixels, so it shrinks in
  squeezed rows (physically right for a signal fault, but less visible).

Tier 3, optional:
- Capture defaults and presets from hardware (the 4 presets are v0.1's; at
  the new P12 curve, Total Loss is now a near-total collapse, ~300x).
- Bleed steps at P12 25/50/75% pop (one octave each).
- Chroma is stored at half horizontal resolution, 8 bits, truncated (about
  −2 codes of bias).
- The top row shows the previous field's last line (1-line replay).

## Build

`./build_combo.sh syncbleed` → `out/rev_b/ron/catalogue-combo/syncbleed.vmprog`.

v0.2.0 (2026-10-13): all 6 configs pass on seed 1 (router2 default), under
heavy machine load. HD Analog 77.4, HD HDMI 79.2, HD Dual 74.7 MHz; SD
Analog 71.2, SD HDMI 71.3, SD Dual 71.0 MHz. 6,530–6,560 LC (85%), 20/32 EBR.
Standalone yosys: 4,164 LUT4, `emulate_transparency` 0. HD Dual's margin is
thin (+0.5 MHz). If a rebuild ever misses it, measure the seed spread first.
The new LC is mostly the ×64 Loss path (31-bit DDAs, wider seam × q
products, per-line q × 2^frac multiply and barrel), the three crawl
multipliers, the divider and the [1 2 1]. A copy of the vmprog is in
`.syncbleed_work/v020/`.

v0.1.1 (2026-09-13): HD 77.3 / 78 / 84 MHz. Standalone yosys 2,850 LUT4, 19 EBR.
