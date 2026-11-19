# Vase — a glossy lathe vase striped by a rolling colour cycle

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/vase.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.1 (2026-11-15): K6 camera pass on the diag, built** (K6 Camera, Gloss dropped; see
[K6 camera pass](#k6-camera-pass-v061-2026-11-15); `out/rev_b/ron/_diag/vase_diag_v0.6.1.vmprog`).

**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Vase Diag`, `out/rev_b/ron/_diag/vase_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/vase/`) is untouched.

**v0.5.0 (2026-10-27): built, NOT flashed.** Rebuilt from the photo WIN_20230424_23_59_47 (the v0.1
"rainbow silhouette" was judged too abstract). The picture is a camera shot of a glossy vase keyed onto
black and colourised by one slow vertical oscillator: a yellow/green/blue body, fast red bars, red and
magenta blocks, and a red key window with a no-red channel. Presets are hand-written placeholders.

## K6 camera pass (v0.6.1, 2026-11-15)
The same K6 Camera design as production v0.5.2 (see `programs/ron/heirloom/vase/SUMMARY.md`, "K6 camera pass"):
- **K6 Camera** = surface strength, linear g = K6/512, glided (1/8 per field, ±4 deadband) and decoded per field.
  Video: the camera luma × g on the camera write side (m = K6·19/512 on luma/4, 19 = the shipped ×0.297);
  Osc: the stored vase × g (n = K6/32) in the fill engine, which writes the stage-12 base (max(sheen, half), or
  max(n·S, sheen, half) in Osc) into the line RAM; stage 12 is one max(camera, base).
  0 % = the black-input look, 25 % = ×0.5, **50 % = ×1 = the v0.6.0 look (default)**, 100 % = ×2 (camera ×1.95,
  stored vase ×1.94 = the old Gloss maximum).
- **Old K6 Gloss dropped (fixed ×1), nothing moved.** The diag's other controls were all re-patched into bold
  element parameters and look-flips in phase 2 (Twin, Wrap, the wide Key), so no control is weaker than Gloss,
  and K6 itself now covers Gloss's range: in Osc it runs dull (0 %) .. photo (50 %) .. glazed (100 %), and in
  Video it brightens the surface. The stage-13 gloss multiply is gone. "Swap Glaze" keeps its K6 760 (now
  surface ×1.48: the same glaze in Osc).
- Sheet (production, same law): `.heirloom_work/rb_vase/k6/k6sheet.png`. The diag was not simulated separately.
- **Build (ROUTER1, `.kinetic_work/tools/k6build.sh diag vase`):** HD Analog s1 77.46, **SD Analog s2 80.38** (seed 1
  stalled router1), HD HDMI s1 87.21, SD HDMI s1 78.78, HD Dual s1 87.07, SD Dual s1 86.95 MHz (worst HD 77.46);
  7,390–7,399 / 7,680 LC (96.3 %; v0.6.0 7,262), 24 / 32 EBR. Standalone yosys 4,287 LUT (v0.6.0 4,199). Latency 32
  unchanged. Preview `out/rev_b/ron/_diag/vase_diag_v0.6.1.vmprog`. Not flashed.

## Phase 2 (v0.6.0)
No notes from Ron for vase (free hand, 2026-11-09 batch). The v0.5 knobs were already element parameters
(bar and band oscillators, roll, spin, the key, gloss, the profile), and all of them were live. The weak
spots were:
- K5 Key's ±240 px window shift (a moderate change);
- S8 Tear (the edge comb) and S9 Blocks (bars all the way down), both subtle;
- nothing that leaves the photo's frozen pose: the vase never moves, and the bars are locked to rings.

The round:
- widens K5 Key so the no-red channel slides right across the vase and off it;
- re-patches the two subtle switches into big look-flips: **S8 Twin** (a second, mirrored vase) and
  **S9 Wrap** (the bars wrap round the vase as a helix that turns with Spin);
- gives the headline P12 a second module above the photo: the vase's axis **sways** as a travelling
  S-bend, its phase driven by the spin (the patch cord from the spin oscillator into the centre line).

Every new effect is exactly zero at the default; the default frame is identical to v0.5.1 (checks below).
The edge comb and bar-top lines (v0.5 S8 Tear On) and the photo zones (v0.5 S9 Blocks On) are now always on.
Phase-2 working files: `.heirloom_work/rb_vase/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| K5 Key: key centre −220 .. +260 virtual px from the axis (photo +68 at 614) | **K5 Key**: slope ×1.125 about the photo: −624 .. +67 (614, exact) .. +527 virtual px | the no-red channel (the gap in the red bands right of the axis) slides across the whole vase and off it: towards either end the bar rows go base colour and the red falls in the gaps on the whole vase (the photo's inverted edge zone). 60 % of the width end to end; the last ~13 % (left) / ~5 % (right) of travel is past the vase |
| S8 Tear: edge comb + bar-top lines On / Off | **S8 Twin** (renamed): Off / On | On = two vases, mirror images, at 1/4 and 3/4 of the width (the beam folds at the screen centre; the key, halo, sheens and spin mirror with it; with sway they dance in mirror). The comb and lines are always on |
| S9 Blocks: photo zones On / Off (bars all the way down) | **S9 Wrap** (renamed): Rings / Helix | Helix = the bars inside the bar zone become a spiral, 32 bars per turn round the lathe (about 38° at the front, steeper at the rims), and it turns with K4 Spin like a barber pole; the blocks and the off zone stay rings. Rings = the photo. The zones are always on |
| P12 Bulge: profile deviation ×0 .. ×1 (40 %) .. ×2.5 | **P12 Bulge**: same, **plus above the photo the axis sways**: a travelling S-bend (about one sine down the frame), amplitude 0 at 40 % .. ±306 px at 100 %, its phase 4 × the spin angle | 0 % = straight cone; 40 % = the photo (still); 70 % = a fuller vase bent ±150 px; 100 % = the pinched hourglass, its bulbs swinging ±306 px against each other, undulating as it spins (1 cycle per ~14 s at the default Spin, ~2 Hz at full Spin) |

Unchanged: K1 Bars (6–54 per cycle), K2 Bands (≈0.77–4.9 cycles), K3 Roll, K4 Spin (it now also drives the
helix's turn and the sway's undulation), K6 Gloss (v0.6.1: K6 Camera, Gloss dropped), S7 Swap, S10 Palette, S11 Source.

**Presets** (preset 0 = the defaults): **Photo**; **Dancer** (P12 100 %, faster Spin; was Hourglass);
**Column** (P12 0, more bars; no longer turns Blocks off); **Swap Glaze** (as before); **Twin Vases** (S8 On);
**Barber Pole** (Helix, fast Spin, fewer bars); **Key Slide** (K5 near the left end: red in the gaps, rolling
up); **Snake Pair** (Twin + Helix, P12 80 %, fast Spin).

**Engine (new).**
- **Sway.**
  - Two new line-engine slots (1 and 2) before the centre lerp; the line program grows from 8 to 10 slots
    (280 clocks, inside SD's 858).
  - Phase p = vy/4 + 4·φspin (16 bits, signed).
  - The odd parabola sine p·(32767 − |p|) on the serial multiplier (|p| as p xor sign), then × the amplitude
    straight from the product register.
  - Added (with the edge jitter) into the line centre.
  - The amplitude is (P12 − 410)·2 Q.2 above 410, else 0.
- **Twin.**
  - The beam the vase sees (b_xf) folds at the screen centre: it starts at x' = xcv + Xc/2 and runs down
    to the centre, then up.
  - The u DDA steps with it, so the vase at 3/4 W reflects to 1/4 W. Down-steps use the one's complement
    −step − 1: a drift under 0.001 u, sub-pixel.
  - The u0 product takes (centre − x'(0)), which is 0 in single mode. To fit the 18-bit step, the serial
    multiplier's b operand widened from 16 to 18 bits (18 iterations); every product is numerically
    unchanged.
  - The edges, halo and key distances read b_xf. Single mode: b_xf = b_xv, bit-exact.
- **Helix.**
  - The bar test moved from the line engine to pixel stage 5. The line carries its bar phase (8 bits;
    + 2·φspin with Helix), the zone and the duty.
  - The pixel adds ±2·asin(u) with Helix (one's complement on the minus side: 1/256 bar) and compares with
    the duty. Rings: the same test without the angle, identical to the v0.5 line flag.
  - The bar-top key opening simplifies to the outer ramp (exact: max(min(A,B),B) = B). Bar-top lines are
    off in Helix.
- **Key.** Per-field law (K5 − 614)·1.125 + 67, one add per register.
- No pipeline stages added or removed: latency 32 in every mode, U/V parity unchanged.

**Build** (ROUTER1: `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh vase`, 2026-11-09):
HD Analog s1 79.61, SD Analog s1 77.88, HD HDMI s1 80.80, SD HDMI s1 78.65, **HD Dual s2 76.50** (seed 1 stalled
router1), SD Dual s1 80.69 MHz. 7,258–7,272 / 7,680 LC (94.6 %; v0.5.1 6,963–6,994), 24 / 32 EBR (unchanged); no RAM
transparency emulation; iceunpack OK ×6; the packed config carries the new labels and presets. No pin-skew screen.
**Fit history.** The first diag RTL synthesised to 4,476 LUT (+506 over v0.5.1's 3,970), about 98–100 % LC.
Ablation ranked Twin 251 LUT, a two-slope K5 law 102, sway 46 and Helix 38. Restructured with no feature cut:
- the single-slope key law;
- the u0 operand swap (no result mux or clamp);
- the one's-complement down-step;
- the small signed beam step, with no fold-turn constant;
- the odd parabola fed straight from the product register.
That made 4,199 LUT, then the build's 7,262 LC (`p2/abl/`).

**Checks** (working files in `.heirloom_work/rb_vase/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: the K5 law, the sway parabola × amplitude (all 65,536 phases) and the centre / twin-operand / u0
  product ranges in 1080, 720p and SD at RTL widths: no wrap, K5 monotonic, 67 px at the default.
- Profile table second-difference scan: the only flag is the off-frame last knot (68), which pulls the base's
  bottom 8 lines in by ≤ 4 px (≤ 10 px at P12 100 %). It predates v0.6, and no new control magnifies it.
- Default frame (GHDL 720p60 dec 1, Osc, 2 warm-up + 1 capture, every control set to its default; production
  v0.5.1 simulated the same way): raw 10-bit output **byte-identical**, on the final source.
- One extreme frame (`p2/x_twin_helix.png`: Twin + Helix + P12 100 % + K5 200 + fast Spin, Osc; the glides
  reach about P12 60 % and K5 480 in the 3 simulated fields): two mirror-image vases at 1/4 and 3/4 W, both
  S-bent by the sway, the bar zones wrapped in diagonal spirals, the blocks still rings.
- Not verified: Twin / Helix / sway in Video mode, Helix while spinning (motion), sway at the full ±306 px,
  K5 at its ends, SD and interlaced modes.

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
| K6 Camera | the vase's glossy surface | (v0.6.1) surface strength: the camera (Video) or the stored vase (Osc) × 0–2, max'd with the vase's own sheens and half its luma; 0 % = the black-input look, 50 % = the v0.6.0 look (default), 100 % = ×2; glided. The v0.6.0 K6 Gloss is dropped (×1) |
| S7 Swap | red bars vs olive gaps | the key swaps bar and gap rows: red falls between the bars, the blocks go olive, the off zone goes red |
| S8 Tear | edge comb, red overshoot lines | per-line edge jitter + bar-top lines (default On) |
| S9 Blocks | red/magenta blocks and off zones | photo zones (On), or bars all the way down (Off) |
| S10 Palette | the colours | Photo / Rotate (R→G→B: green bars on a violet body) |
| S11 Source | the vase's glossy surface | Video (camera wrapped round the vase, default) / Osc (the stored vase) |
| P12 Bulge | the silhouette (neck, bulbs, waist, base) | profile deviation × 0–2.5: straight cone (0 %), the photo (40 %), an hourglass whose waist pinches to a thread between two swollen bulbs (100 %); glided |

K1, K2, K5, K6 and P12 glide (1/8 of the gap per field, ±4 deadband).

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
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s2 80.74, SD Analog s1 79.21, HD HDMI s1 76.75, SD HDMI s1 78.13, HD Dual s1 78.21, SD Dual s1 72.98 MHz; 6963-6994 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
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

## Open questions for Ron (2026-11-09 batch)
1. The sway rides on P12 above the photo (one CV into both the bulge and the centre line). Should it have
   its own control instead, e.g. in place of K6 Gloss, so the photo-shaped vase can sway too?
2. Twin puts mirror images at 1/4 and 3/4 of the width. Would a same-handed copy (both spinning the same way)
   or a third copy read better?
3. Helix is fixed at 32 bars per turn (about 38° at the front). Steeper (a barber pole), shallower, or a
   bipolar twist knob?
4. Tear (the edge comb) and Blocks Off (bars all the way down) are gone as switches; the comb and zones are
   always on. Do you miss either?

## Hardware watch list
- Renamed: S8 Tear → Twin (Off / On), S9 Blocks → Wrap (Rings / Helix); preset Hourglass → Dancer. K5 Key and
  P12 Bulge keep their names but go much further (K5 −624 .. +527 px; P12 sways above 40 %).
- P12 above 40 % now moves the vase sideways as it spins; at the default Spin that's slow (a cycle every ~14 s).
- Twin on SD: the copies sit at 1/4 and 3/4 of a 4:3 width, so at P12 100 % with sway the swinging bulbs can
  reach the frame edge and the centre seam (the seam is a mirror, so they meet as reflections).
- Helix turns off the thin bar-top red lines (they're horizontal and don't follow the spiral).
- ROUTER1 (as v0.5). **HD Dual = seed 2** (seed 1 stalled the router); the other five on seed 1. HD Dual is the
  thinnest HD config (76.50 MHz).

## Next step
Wait for Ron's preview notes. Rebuild the diag with
`NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh vase`. On approval: `./heirloom_diag.sh merge vase`,
then repack the previewed diag bitstreams under the production TOML (copy
`build/programs/ron/vase_diag/rev_b/bitstreams/*.bin` into the production build dir, regenerate
`program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`) instead of
re-routing; update the production SUMMARY / README / PHASE2 row and commit only vase's files.
