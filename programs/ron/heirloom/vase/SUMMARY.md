# Vase — a glossy lathe vase striped by a rolling colour cycle

**v0.5.0 (2026-10-27): built, NOT flashed.** Rebuilt from the photo WIN_20230424_23_59_47 (the v0.1
"rainbow silhouette" was judged too abstract). The picture is a camera shot of a glossy vase keyed onto
black and colourised by one slow vertical oscillator: a yellow/green/blue body, fast red bars, red and
magenta blocks, and a red key window with a no-red channel. Presets are hand-written placeholders.

## Reading of the photo (measured, .heirloom_work/rb_vase)
- **Silhouette.** One lathe object on black (RGB 1,3,0), with its axis at x ≈ 1018 (0.530 W), wobbling
  between 1000 and 1029.
  - Half-width (threshold-15 edges, geom1.py): neck 65–75 for y 0–90, upper bulb maximum 178 at y ≈ 360,
    waist minimum 106 at y ≈ 555, base 282 at y ≈ 1050. Trend 74.7 + 0.179 y.
  - The right edge is sharp. The left edge is an 18 px grey-green halo with a line comb: edge jitter
    σ = 1.3 px left and 1.5 px right, lag-1 correlation 0.5, left and right independent, 2–3.5 px in the
    upper bulb.
- **The object is camera content.** A glossy vase, unwrapped into lathe coordinates u = (x − c)/hw
  (Su1.png):
  - In the upper bulb, white sheen arms at u ≈ −0.7 and +0.5 form a loop round a dark eye and meet at
    the neck (1000, 105).
  - The lower bulb has sheens at u ≈ −0.75, −0.08 and +0.62, and a dark band at x ≈ 1060.
  - The highlight figure is irregular, so it is a measured table, not a formula. Saturated plateaus were
    extended above 255 (sext.py) so the white sheens stay broad.
- **One slow vertical cycle** drives the colouriser: period 556 lines, origin y = −24 (both cycles in the
  frame agree to ±2 lines).
  - Hue: per cycle bin, (R − G)/2 = c(p)·S + d0(p). In mid-cycle c ≈ +0.14 and d0 ≈ −4 (yellow-brown);
    at the cycle ends c ≈ 0 and d0 ≈ −14 (green).
  - Blue tint: B ≈ S + 0..28 for p 430..75 (slate/teal waist and base). Elsewhere B is a knee,
    2.8·(S − 170), giving white sheens.
  - Zones by phase: 0–17 gap, 17–330 fast bars, 330–358 gap, 358–432 red block, 432–521 off,
    521–556 magenta block.
- **Bars.** Period 26.8 lines, hard-synced to the cycle (bar rows repeat exactly 558 lines apart). They
  widen along the zone from 13.5 to 16.8 lines (duty 0.50 → 0.63).
- **Red key.** A window on d = |x − 1090| in screen x, so the boundaries stay vertical while the shape
  changes:
  - Bar rows are red for 47 ≤ d ≤ 190 on the left and ≤ 183 on the right. This leaves a no-red channel
    around x 1043..1137.
  - Gap rows are red for d ≥ 225 on the left and ≥ 183 on the right: the inverted edge zone (lower left,
    bottom right).
  - The key edges are about 10 px soft.
  - High luma kills the key (smoothly above S ≈ 200–240), so sheens stay white or yellow over the bars.
    The halo also kills it.
  - Thin red lines sit on the bar tops: on that one line, the window opens to d = 0.
- **Red state** (fitted on thin bars, redfit2.py):
  - R = 255·tanh((S − 18)/68), G = 1.8·(S − 168)+, B = 1.7·(S − 125)+.
  - Magenta, where the tint is on (p > 400 or p < 70, fading to red by p ≈ 125): R = Rr(1.25 S) and
    B += 1.3·Rr(1.25 S) + 20.

## What it does
1. **Profile.** Per line, in virtual 1080-line coordinates: a centre and a half-width deviation from
   16-line knots (linearly interpolated), hw = trend + deviation × Bulge, plus per-line edge jitter (Tear).
2. **Lathe coordinate.**
   - A u DDA (step 2^22/hw, one serial divide per line) gives u.
   - u → asin → + spin angle → fold at ±¼ turn → sin → u′.
   - u′ indexes a 128-entry line RAM. The fill engine writes the next line's y-interpolated row of the
     64 × 64 object table into it (64 columns plus their midpoints).
3. **Source.**
   - Osc: the stored glossy vase.
   - Video: the camera luma wrapped round the lathe (one source width per turn, so the front shows the
     middle half of the picture, turning with Spin), combined with max() with the vase's sheens
     (2·(S − 150)) and half the vase. The vase still reads with a dark or missing camera.
4. **Luma.** × Gloss, × edge envelope (smoothstep over the outer quarter of |u|, min'd with a 3-px key
   ramp at the jittered edges).
5. **Cycle (per line).** Phase q (Q.16 of a cycle, + Roll) → zone and bar duty from a 128-entry curve ROM;
   bar phase (q − q0)·bars-per-cycle → bar or gap; overshoot flag on each bar top; colour curves
   A, BO, C, D0, M.
6. **Colouriser (RGB, clipped per channel).**
   - Base: R/G = S ± (C·S·w(S) + D0·w(S)), B = max(A·S + BO, knee(S), halo·S).
   - Red: R = Rr(S) + M·ΔRr, G = 1.8·(S − 168), B = 1.7·(S − 125) + M·Mg(S).
   - Mix by the soft key: min(window on |x − x_key|, luma kill, halo).
   - Then RGB → YUV, a luma curve (greys × 1.25 so the 13/16 conditioning restores them, top compressed),
     and the output conditioning.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Bars | the red bars | bars per colour cycle, 6–54 (photo 20.7, default 314) |
| K2 Bands | the colour cycle (hue bands, blocks, magenta) | cycles down the frame, ≈0.77–4.9 (photo 1.94 = 556 lines, default 497) |
| K3 Roll | bars and colour bands | bipolar roll speed, square law, deadband ±8; positive rolls down (default 640, ≈0.5 line/field) |
| K4 Spin | the vase's sheens | bipolar turn speed, square law, deadband ±8; the sheens slide round the lathe and squash at the rims (default 620, ≈1 turn/min) |
| K5 Key | the red key window / no-red channel | key centre −220..+260 virtual px from the axis (photo +68, default 614) |
| K6 Gloss | the sheens and body brightness | luma gain 0.45–2 (photo 1.0 at 512): dull … blown-out white |
| S7 Swap | red bars vs olive gaps | the key swaps bar and gap rows: red falls between the bars, the blocks go olive, the off zone goes red |
| S8 Tear | edge comb, red overshoot lines | per-line edge jitter + bar-top lines (default On) |
| S9 Blocks | red/magenta blocks and off zones | photo zones (On), or bars all the way down (Off) |
| S10 Palette | the colours | Photo / Rotate (R→G→B: green bars on a violet body) |
| S11 Source | the vase's glossy surface | Video (camera wrapped round the vase, default) / Osc (the stored vase) |
| P12 Bulge | the silhouette (neck, bulbs, waist, base) | profile deviation × 0–2.5: straight cone (0 %), the photo (40 %), an hourglass whose waist pinches to a thread between two swollen bulbs (100 %); glided |

K1, K2, K5 and P12 glide (1/8 of the gap per field, ±4 deadband). K6 is latched per field.

## Video
The photo's vase is camera content, so Video mode puts the camera there through the same processing.
- The camera luma (written into a 2 × 512 line buffer, one source width per turn) is wrapped round the
  lathe and turns with Spin.
- It passes through the same edge envelope, colour cycle, bars, red key and conversion.
- The stored vase's sheens (max with 2·(S − 150)) and half its luma stay on top. A dark or absent camera
  still shows a dim version of the photo's vase, and the silhouette, bars and colours always read.
- Osc mode shows the stored glossy vase: the photo's own camera content.

## Engine
- **Per-line engine** (one serial shift-add multiplier, one serial divider with a constant 2^22 dividend,
  slots of 28 clocks).
  - Line program, 8 slots, ≈224 clocks, kicked at the source avid rise and computing the next line
    (line 0 at zone clock 1024):
    0. profile ROM reads, 16-bit LFSR jitter, fill-row parameters;
    1. centre lerp;
    2. deviation lerp;
    3. trend vy·184;
    4. dev × Bulge, then hw and the jittered edges;
    5. 2^22/hw, then the per-pixel step (× vsx/2);
    6. u0 = −centre·step, curve-ROM read;
    7. bar phase (q − q0)·bars per cycle, zone/duty from the curve word, bar flag, overshoot flag, then
       advance vy, q and the row accumulator.
  - Frame program (zone clock 64): cycle increment qbase·vsy, roll speed m², spin speed m².
  - The curve word stays in the ROM output register and is committed with the line at the next avid rise.
- **Fill engine.** 128 clocks per line: two table reads per column, y-lerp (9×5), columns and midpoints
  into a ping-pong 2 × 128 line RAM.
- **Pixel path, 24 stages + 8 conditioning, latency 32 in every mode.**
  - 0: beam (virtual x in Q.2, u DDA).
  - 1–2: u saturation, edge and key distances, edge factors, halo.
  - 2–4: asin ROM.
  - 5: spin add, key ramps; side signals {on, envelope, key, halo} into a 256 × 16 EBR ring (8 clocks).
  - 6: fold.
  - 7–8: sin ROM and camera buffer.
  - 9–11: line RAM.
  - 12: source select. 13: gloss. 14: envelope.
  - 15–17: colour ROM (256 × 64: kill, w, S·w, knee, red G, red B, Rr, ΔRr, Mg) and blue product.
  - 18–22: hue/blue/red terms, base and red RGB, key mix, edge gate and palette rotation.
  - 23: RGB → YUV. 24: luma curve.
  - Then chroma [1 2 2 2 2 2 2 2 1]/16 (8-bit chroma, delay-8 EBR ring), luma [1 2 1], luma 13/16 and the
    blanking gate on the generated avid.
- **Syncs** are delayed 32 clocks in an EBR ring.
- **ROMs** (generated by vase.py into vase_pkg.vhd, U/V in the hardware convention):
  - object table 4096 × 8 (8 EBR);
  - profile + curves 256 × 32;
  - asin and sin 512 × 10;
  - colour 256 × 64;
  - edge envelope 64 × 5.
- **Contracts:** generated avid (parity-locked start), virtual 1080-line geometry (the vase is sized to
  height and centred at 0.53 W in every mode), engine started per field, all block RAMs `no_rw_check` and
  collision-free.

## Status
- **v0.5.0 built 2026-10-27, not flashed. ROUTER1** (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh vase`).
  - **Seeds:** HD Analog = seed 5 (seeds 1–4 missed); all others seed 1.
  - **Fmax (last post-route line):** HD Analog 75.33, HD HDMI 78.47, HD Dual 75.81, SD Analog 76.46,
    SD HDMI 76.60, SD Dual 76.46 MHz.
  - 6,971–6,994 / 7,680 LC (≈91 %), 24 / 32 EBR. All bitstreams pass iceunpack.
- **router2** missed HD Analog on all 6 seeds (best 72.4 MHz). It passed HD HDMI (77.45, s1) and HD Dual
  (79.37, s2).
- **Fit history:**
  - The first RTL placed at 10,202 LC (132 %). Ablations showed the RGB colouriser at ~2,200 LUT, the
    line + fill engines at ~1,470 LUT and 860 FF, and the frame sequencer at ~340 LUT.
  - Rebuilt lean: no frame sequencer, an 8-slot engine in virtual coordinates, a colour ROM, narrower
    products, EBR rings, S7 Swap instead of Mirror. That reached 7,440 LC with HD HDMI at 63 MHz, whose
    critical path was the camera luma × 75 in one stage.
  - Pipelined that and trimmed further: 6,985 LC. The next critical path is the stage-12 source select.
- **Verification:**
  - GHDL 720p60 (decimation 1), Osc, Roll/Spin still, Tear off, against the integer model (imodel2.py):
    mean abs error 0.018 (10-bit), 0.056 % of pixels > 8 levels.
  - The motion sim matches the model one field of roll/spin later.
  - The Video sim (selfie) shows the picture wrapped round the vase under the bars.
  - Region-mean error against the photo (6 × 4 grid): proto 4.4, RTL 5.8 (shell conditioning and white cap).
  - SD not simulated.

## Deviations from the photo
- **White level.** The shell caps drawn whites at ~777, so the broad white sheens read slightly greyer
  than the photo (greys are pre-boosted so mid-tones match).
- **Dark eye.** The eye in the upper bulb is a little wider and softer (64 × 64 table, 17-line rows).
- **Waist tint.** The slate-blue on the waist's right side is greyer: the tint is one curve per row, while
  the photo's varies across x.
- **Dropped detail.** A small red strip at the far left inside the red/magenta blocks (d ≥ 290 on block
  rows) was dropped for fit.
- **Quantisation.** Zone boundaries are quantised to 1/128 cycle (4.3 lines), bar duty to 8/256 steps, the
  magenta fade to 4 levels and the hue/blue gains to Q.7/Q.4. None of these is visible at normal viewing.
- **Edge jitter.** The jitter is a free-running LFSR, so the comb's pattern differs from the photo's and
  moves every field, as the analog original would.
- **Mirror became Swap.** The originally planned S7 Mirror (symmetric vase) became Swap to fit the chip.
- **Video horizontal resolution.** The camera has 512 samples per turn, so 256 samples across the vase's
  front.

## Working files
`.heirloom_work/rb_vase/`:
- `notes.md`: look, measurements, proto rounds, fit history.
- Measurement scripts: `geom1.py`, `bars2.py`, `phmap.py`, `kbound.py`, `hcurve2.py`, `redfit2.py`, `sext.py`.
- Protos: `proto.py` (the fitting proto) and `pfinal.py` (the proto with controls).
- `imodel2.py` (the integer model of the shipped RTL) with `hwtables2.py`.
- `mkhook2.py`: writes `vase.py`.
- Images: `sheet.png` (control sheet), `cmp_proto.png`, `cmp_final.png`.
- `abl/` (ablations), `prof/` (critical-path profiles).
