# Beacon — a candle flame and a hyperbolic bell keyed into blue bars

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Heirloom #26, from
`photos/WIN_20230424_23_53_49_Pro.jpg`. The v0.1.0 version (a hashed blue/black block grid with a
cone and a radial glow) was replaced. The photo is two layers KEYED together, not added: a white
layer (flame, bell, cone, side glows) crossfaded under a blue key (columns × bands, faded around the
light). Presets are hand-written placeholders.

## Reading of the photo (measured)
The work is in `.heirloom_work/rb_beacon/` (notes.md):
- `chan.py` / `rows.py` / `grid.py`: the layer split and the band and column structure.
- `meas_flame.py` / `meas_bell.py`: the flame and bell fits.
- `fittab.py`: the fade tables.

**Two layers.** P = (R+G)/2 is a white layer L, grey-green in the mid tones (G = R + 13..19).
Q = B − P is a blue key k. The colour is a crossfade, out = L·(1−k) + blue(k):
- at (1150, 197) a k ≈ .5 cell is (33,48,133) = half blue + half L (25,49,32);
- at (1850, 197) a black row shows white L (254,255,250) under a pure blue row (50,48,255).

**Flame.**
- Centre x 992–999, an asymmetric tent across x: left slope 2.0–2.8 (mean 2.4) and right 1.95–2.05
  per px; the left flank is steeper in x order.
- The peak profile rises 1.8 per line from the tip (y ≈ 13) to about 365 at y ≈ 250 (core ≥ 245 is
  an almond, half-width 52) and 270 at y 388.
- Neck y 395–440, then a dark rounded cap at (1030, 455–470).

**Bell.**
- The outline is a hyperbola about x = 1030.5: half-width hw(y) = 36.45 + sqrt(150² +
  (0.62(y−395))²) − 150, rms 0.97 px over y 440..792.
- A bright Gaussian line outside (peak 190–245, FWHM about 20 at the top and 12 lower, the right side
  wider) and a dark line about 22 px inside (level 70–85).
- The interior rises from 50 at y 470 to 255 at 590.

**Cone.**
- The same hyperbola continues as the right edge (1215 at y 800 to 1363 at 1060).
- It is shaded D(x) = 28 + 1.75·|x − 1176| (bright to x 1048, a valley at 1176, rising into the
  right line), independent of y.
- Inside the cones, lit cells carry blue: k .38 / .22 on the left wing, and .32 on the right column,
  ramped in over y 640..810. These are the lavender wedges.

**Edge glows.** White ramps at the screen sides, in black rows only: L = 128 + s·(|x − 930| − Wv),
with s = 2.1 on the right and 1.7 on the left. Wv follows the band cycle (843 → 1015), so the ramps
draw ")" arcs.

**Blue key.**
- **Columns.** A horizontal pulse wave of period 391 px rises at 84, 470, 862, 1253 and 1646 and stays
  high for 246 px.
- **Window.** Blue rows (class A) show blue only for x < 880 and x ≥ 1648 (the stage of the beacon
  is two periods wide). Black rows (class B) show blue in the low part of the pulse: a checker on
  the right, bars on the left.
- **Band cycle.** 742 lines: a 110-line blue band, a stripe zone with a 27.2-line period (wider at the
  zone ends), a 40-line blue band, and a 188-line black band. It restarts after a 70-line hold, so
  y 0..180 is all blue.
- **Fade.** The blue dies out around the beacon, more lower down. It is fitted as two 32 × 31 knot
  tables (64 px × 36 lines; rms .044 / .054).
- **x order.** After each cut the blue rises with an RC lag (tau 6–15 px). Before each cut it rings
  with a bright pre-shoot (+.22..+.25 for 3–6 px): the bright blue verticals at x 84, 862/880, 1646.
- **Tear.** The fade edges jitter per line (sd 22 px, lag-1 correlation .72). Sparse 1-line blue
  tracking lines cross the flame and bell.

## What it does
1. **White layer.**
   - The flame is a per-line peak and two slopes, run as a DDA from the flame axis. Below a toe
     level it falls at half slope. A flicker LFO scales the gain and a sway LFO bends the top.
   - The bell is the hyperbola half-width per line from a profile table (scaled by K5). The outline is
     a Gaussian of the distance to it: σ 15 outside on the right, 18 outside on the left, 23 inside.
   - Inside the bell: a ramp × a per-line level × the source (D shading in Osc, camera luma in
     Video). Above y 455 the neck cap (a notched arch) replaces the ramp.
   - The cone's wing key, and the edge-glow ramp in black rows outside the stage.
2. **Blue key.**
   - Raw key: the fade value in its window. Blue rows take max(fade, wing cone); black rows take the
     fade where the column wave is low.
   - Then an x-order RC lag (a one-clock IIR, 1/16 in blue rows and 1/8 in black rows), the pre-shoot
     spike before each cut, and the forced blue in the left column and on the tracking lines.
   - Where the flame is bright (above level 300) it burns through the blue.
3. **Mix.** Colour ROMs for both layers (the photo's ramps: green mid tones for L; blue for k, with
   R and G only above k ≈ .5), then out = L·(1−k) + B(k).
4. **Output.** The shell conditioning and the blanking gate on a generated avid.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Bars | the vertical band cycle | band-cycle frequency ×0.5 → ×2, exponential (the photo's 742 lines at 50 %) |
| K2 Columns | the 391-px blue/black columns | column period 657 → 233 px, exponential (391 at 50 %) |
| K3 Dusk | the fade of the blue around the light | 0 % lifts the blue (+0.6), 100 % deepens the dark (−0.6) |
| K4 Speed | motion | bipolar, still in the centre ±8: the bars roll down (up below centre), 1.4 lines/field at the default; the flicker and sway rates rise with \|speed\| |
| K5 Beam | the bell and cone | hyperbola spread ×0.5 → ×2 |
| K6 Halo | the white ramps at the screen sides | reach −300 → +300 px |
| S7 Bands | the stripe zone | Stripes (photo) / Solid (the zone becomes one blue band) |
| S8 Bell | the bell interior | Solid (photo) / Outline (only the outline, the neck and the flame) |
| S9 Palette | the colours | Blue (photo) / Ember (the blue key turns red, the white layer warms) |
| S10 Tear | ragged fade edges, tracking lines, line noise | On (photo) / Off (clean) |
| S11 Source | the beam interior | Video (default: the camera fills the bell and cone) / Osc (the photo's shading) |
| P12 Flame | the flame (headline) | 0 % = a small ember (gain .15, height .55); 40 % = the photo; 100 % = a towering flame (gain 2.4, height 1.8, wider glow) burning through the bars |

Defaults: K1–K3, K5, K6 at 512; K4 at 635; P12 at 410; all switches at label 0.
Presets: Photo (Source = Osc), Small Flame, Blaze, Wire Beam (Bell Outline, wide beam), Red Night.

## Video
The photo's beam is an oscillator shading (the D valley), so the camera replaces that shading. The
interior of the bell and cone is ramp × level × camera luma (re-mapped 31 + 0.88·Y). The bars,
flame and outlines stay generated, and every other control acts the same in both modes. Osc
substitutes the photo's D(x) shading.

## Engine
- **Micro engine.** It does the per-field and per-line maths.
  - The program is assembled by `.heirloom_work/rb_beacon/masm.py`, whose ISS is the spec.
  - A 24-bit accumulator; a 256 × 32 register file in block RAM holding the six control curves
    (33 points each) and the field state; a 512 × 32 uROM; one instruction per clock (F/D/R/E).
  - A radix-4 serial multiplier and a serial shifter (8 or 1 places per clock); MAX/MIN take one
    stall clock.
  - The assembler inserts the hazard NOPs: an ST is invisible to the next two instructions, and
    OUT acts one clock after E.
- **Field program** (vblank zone):
  - the control glides (snapped to the knobs on the first run);
  - the curve lookups;
  - the column phase, increment and spike windows;
  - the flame gain, height and slopes;
  - the LFO phases and the band roll.
- **Line program.** Kicked at each line start, it computes the NEXT line in at most about 615 clocks
  (inside SD's 720-clock active line). It produces:
  - the band class and edge-glow reach from the band table;
  - the fade knot row and weight (a knot loop lerps 32 knots into a ping-pong knot RAM);
  - the flame peak, slopes and start level;
  - the bell half-widths, interior and outline levels from the Y2 table;
  - the neck cap and notch;
  - the wing-cone ramp;
  - the AR(1) jitter, line noise and tracking line.

  The values go through a staging block RAM, and a copier moves them into the line registers in
  the blanking.
- **Pixel path.** 28 clocks, one latency in every mode:
  - 0: the DDAs (layout x, shape x, column phase, flame level, slope side);
  - 1–2: pixel units and region flags, the flame toe, the burn level, the knot address;
  - 3–7: bell distances, D shading, edge glow, neck cap, the knot lerp, Gaussian outline,
    wing cone, camera luma;
  - 8–9: Dusk, line and interior products, raw key;
  - 10–14: RC lag, pre-shoot, forced blue, burn-through, white-region max, line noise;
  - 15–20: colour ROMs, Ember, the keyed mix;
  - 21–28: the shell's chroma [1 2 2 2 2 2 2 2 1]/16 (two 8-box sums), luma [1 2 1] and 13/16,
    and the blanking gate.
- **Contracts.**
  - damier's generated avid.
  - Virtual 1080-line coordinates from the timing ID: shapes are sized to the height and the column
    layout spans the width.
  - All delay rings and the staging, knot and register RAMs carry `no_rw_check`.
- **Fit history.**
  - Hard-wired sequencers: 13053 → 11003 LC.
  - The micro engine: 9765 → 7916 LC.
  - Glides, LFOs and roll in microcode; staged line values; the serial shifter; flags in the ring's
    spare bits; narrowed products (6-bit line levels, 5-bit knot weight, 5-bit white-layer chroma
    weight): 6806 LC. Tracking lines restored: 6875 LC.
  - Timing restructures (difference-DDA slope side, fabric-registered RF read, registered advance
    flag, delayed OUT, MAX/MIN stall, split neck-cap product): from 62–69 MHz to 77–84 MHz at
    7098 LC.

## Status
- **v0.5.0** built 2026-10-27 with router2, seed 1 on five configs.
- **SD Dual is on SEED 2**: seed 1 stalled router2 into the 900-s make timeout.
- Fmax (MHz):

  | Config | Fmax |
  |---|---|
  | HD Analog | 77.29 |
  | SD Analog | 61.69 |
  | HD HDMI | 83.35 |
  | SD HDMI | 76.63 |
  | HD Dual | **75.34 (thin)** |
  | SD Dual | 79.09 |
- 7073–7098 / 7680 LC (92 %), 25 / 32 EBR, iceunpack OK × 6.
- `out/rev_b/ron/heirloom/beacon.vmprog` (494,442 bytes, unsigned). Not flashed.
- Router1 closes HD Analog at 82.6 / 83.7 MHz (seeds 1 / 2; no CARRY I0==I1). If HD Dual regresses,
  rebuild with `NEXTPNR_ROUTER="--router router1"`.
- **Verified in GHDL:**
  - RTL vs integer model, bit-exact (quarter frame, 720p decimation 2, Tear off, field 4: 0 pixels
    differ), re-checked after every structural change;
  - full 720p renders: Osc defaults (ragged edges, line noise, tracking lines) and Video (the selfie
    fills the beam);
  - motion (model): 24 % of pixels change over 6 fields at the default speed;
  - SD (model): the full picture.

## Deviations from the photo
- **Fade.** A fitted knot table (two classes, 64 px × 36 lines, vertically interpolated), not a
  closed formula. The ragged edges are a per-line AR(1) shift of it.
- **Tracking lines.** 1 line in 32 with Tear on (the photo has about 1 in 70), forced to blue ≥ 140
  over a random 150–660 px span.
- **Edge glow.** One slope (2.1) on both sides; the photo's left ramps are 1.7.
- **Top end.** Whites peak at about 777 output luma (the HD HDMI bar conditioning, 13/16) where the
  photo's flame core is full white. Edges are softer in chroma (the 9-tap box).
- **S8.** Bell Outline replaces the planned Mirror switch, which did not fit.
- **Video.** It can't be "the photo's processing of a camera", because the photo is
  oscillator-made. The camera fills only the beam interior.
- **Resolution.** 720p is the 1080p picture. On SD (4:3) the column, window and fade layout spans
  the width (compressed about 25 %) and the flame and bell keep their shape, sized to the height.
  Interlaced SD fields each draw the full picture.
- **Precision.** The per-line levels (interior, outline, wing cone) are 6-bit and the white layer's
  chroma is mixed with a 5-bit weight; both are invisible after the shell filters.

## Working files
`.heirloom_work/rb_beacon/`:
- `ref.jpg`: the photo.
- `notes.md`: look, measurements, proto log, controls, model, RTL, fit, timing, build.
- `proto.py` / `proto_final.png`: the float proto. `sheet.py` / `sheet.png`: the control sheet.
- `imodel.py`: the integer model.
- `masm.py`: the micro-engine program, assembler and ISS.
- `model_tables.py`, `ftab.npz` / `ramps.npz`: the fitted tables.
- `gen_hook.py` with `hook_head.py` / `hook_tail.py`: writes `beacon.py`.
- `rtl_osc.png`, `rtl_vid.png`: GHDL 720p renders. `rtl10_q.npy` / `imq6_f4.npy`: the bit-exact
  check.
- `cmp_final.png`: photo / proto / RTL Osc / RTL Video.
- `packlc.sh` (LC count), `prof.sh` (4-route timing profile), `check_local.sh` (GHDL check).
- `beacon_v*.vhd`: earlier RTL versions.
