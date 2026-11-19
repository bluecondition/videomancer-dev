# Tangram — Heirloom #9, rebuilt from the photo

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/tangram.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.2 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Tangram Diag`, `out/rev_b/ron/_diag/tangram_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.0 (`programs/ron/heirloom/tangram/`) is untouched.

**v0.5.0 (2026-10-27): built, NOT flashed.** Rebuilt from `photos/WIN_20230222_00_06_47_Pro.jpg`. It now
redraws the photo itself: an additive RGB patch with pink log-chirped stripe fans (red), a stepped green
zig-zag plateau and lower sawtooth ramps (green), and a bright blue wedge and lower V on navy (blue).
Work files are in `.heirloom_work/rb_tangram/`:
- `notes.txt`, `proto.py` (float proto) and `imodel6.py` (integer model);
- `mc2.py` (engine ISA, assembler, ISS), `mk_hook2.py` and `cmp_final.png`.

## Phase 2 (v0.6.0)
No notes from Ron for tangram (free hand, 2026-11-09 batch). The v0.5 controls were all live, and P12 Fan, K1 Width,
S9 Palette and S10 Mirror were already bold. The weak spots were K2 Wedge (about ×0.4..×2.4), K3 Zigzag (the plateau
only), S7 Tear (the top-left corner only) and a patch that never moved except by streaming. The round:
- widens the shape knobs (Wedge ×0.25..×4, Ramps ×0.25..×4);
- turns K3 into the green layer's **angles**: the plateau zig and the lower ramps' slant together, from inverted
  through flat to ×4;
- makes K4 loosen the patch past the photo's speed: three LFOs swing the blue wedge about its apex, sway the pink
  phase down the frame and slide the plateau;
- turns S7 into a look-flip: **Shred** carries the torn bands and the glitch tears over the whole frame.

Every new effect is exactly zero at the default knob value, and the default frame is byte-identical to v0.5.0
(checks below). Nearly all of it is engine microcode (`p2/mc2.py`); the pixel path gained only a per-line wedge
axis, a per-line pink flow word, the ramps' direction flag and the Shred line logic.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| K2 Wedge: opening ×0.42 .. ×2.4 | **×0.25 .. ×1 .. ×4**, exponential, photo at 50 % (exact) | the wedge's top width from ~200 px (a thin blue spike) through the photo's ~800 px to ~3200 px (the wedge floods the whole top of the frame, its edges off-screen) |
| K3 Zigzag: plateau zig ×0 .. ×1 .. ×2 | **the green layer's angles**: plateau zig **×−1 .. ×0 .. ×1 .. ×4** (two slopes: 0 % = inverted, 25 % = straight, 50 % = photo, exact) **and the lower ramps' slant** with the same gain (falling .. flat .. photo .. ×4) | the photo's zig swings about ±275 px; at 100 % the drops swing −600 .. +820 px and the plateau breaks into wide zig-zag shards; the lower stripes turn from falling-right through horizontal (25 %) to ~45° (100 %) |
| K4 Speed: bipolar square law, top ~4 stripes/s | same curve up to |K4 − 512| = 264 (default **bit-exact**), top **×2** (~8 stripes/s); **plus, past the photo's speed** (K4 below 364 or above 660), three triangle LFOs whose depth grows to the ends: | at either end: **the wedge swings** about its apex (below the frame) by ±0.55 of its height, its axis sweeping ±630 px at the top (~6 s a swing); **the pink phase sways** down the frame (a ±1.5-stripe triangle wave, ~2.6 cycles per height, drifting): the fan edges zig-zag, and at the ends the fans fold into a pink herringbone; **the plateau slides** ±320 px under its fixed right cut. Rates 1 : 3/4 : 11/8, reversed for negative speeds; all three exactly 0 from 364 to 660 |
| K5 Ramps: count ×0.5 .. ×2.9 | **×0.25 .. ×1 .. ×4**, two slopes, photo at 39 % (exact) | lower ramp spacing from ~600 px (two or three broad ramps) to ~40 px (fine hatching) |
| S7 Tear: Off (clean) / On (photo) | **S7 Tear: Shred / Photo** (renamed values; Photo = v0.5 On, the default) | Shred: the two torn top-left bands alternate down the **whole left third** (128-line bands of near-vertical torn stripes) and rows 0..17 of **every 64 lines** are dragged right by 0 .. 136 px across the pink layer. The clean (Off) field is gone |

Unchanged: P12 Fan (pink stripe count ×0.46 .. ×4.2, the headline), K1 Width, K6 Bend, S8 Key, S9 Palette, S10 Mirror,
S11 Source (Video / Osc, default Video).

**Presets:** **Photo** (Osc, as v0.5), **Broad Bands**, **Fine Fans**, **Mirror Temple** (as v0.5); new **Loose Patch**
(K4 100 %: swinging wedge, swaying fans, sliding plateau), **Shredded** (S7 Shred, more fan), **Searchlight** (narrow
wedge, K4 0 % = fast reverse with the swing), **Inverted Zig** (K3 0 %, dense ramps, palette Rotate). Dropped:
Rotated (→ Inverted Zig), Clean Field (no Off any more).

**Engine (new).** Microcode in `.heirloom_work/rb_tangram/p2/mc2.py`; `p2/mk_hook2.py` writes `tangram_diag.py`.
- **Line program** (worst line +69 clocks):
  - pink sway: PB12 = FLR + tri(VY4·10 + SWP)·SWA (one MULL, one MUL), transferred as the line's pink flow word;
    SWA = 0 at the default speed, so it equals the field's flow phase exactly;
  - wedge axis: PB19 = 15939 + LEAN·(1142 − y) >> 8 (Q.4 px; 1142 = the wedge apex line), read by the pixel path
    as `l_xq & "0011"` (= 996.2 px Q.8 exactly at the default);
  - plateau slide: ZG = Z·ZM1 >> 8 + PLX;
  - the slot store is clamped at +2047 px (×4 zig + slide can pass it). v0.5 already wrapped the T2 band's d4
    there at the default (d4 only sets segment 3's ramp rate, giving the fastest ramp), so a clamped d4 reads as
    −32767 in the segment-3 increment and the default stays v0.5's.
- **Field program:** the speed law + 4 × the same law of (a − 264); ex = a − 148; the three LFO phases (rate
  48 + ex/2 a field) and depths (SWA ≤ 96, LEAN ≤ ±0.55 Q.12, PLX ≤ ±320 px); the zig gain two-slope (ZM1),
  |m| and its sign (new OUT 13 → the ramps' direction flag); ISL × |m|; the two-slope Ramps and the ×4 Wedge curves.
- The XFER block grew to 20 words (slot 12 = the pink flow word, slot 19 = the wedge axis; YR moved to RF 93).
  Constants spill into RF 154..159.
- **Pixel path:** the wedge distance (and the axis dots) use the per-line axis; the pink phase adds the per-line
  flow word in place of the field's; the lower ramps' DDA wraps (only its low 14 bits reach the picture) and runs
  down when the slant is negative; Shred re-maps the torn-band rows and the glitch rows in the line logic.
  No pipeline stages added: latency 28 in every mode, U/V parity unchanged.
- **Engine budget** (ISS, every line, 3 knob sets): worst line HD 840 / 2200, 720p 812 / 1650, NTSC 812 / 858,
  PAL 784 / 864 clocks (v0.5: 771 / 743 / 743 / 715); field program + line 0: 1744 clocks.

**Build** (ROUTER1: `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh tangram`, 2026-11-09): HD Analog s1
79.44, SD Analog s1 76.27, HD HDMI **s2** 78.67 (s1 missed), SD HDMI s1 73.56, HD Dual **s2** 80.66 (s1 missed),
SD Dual s1 76.60 MHz. 7,217–7,237 / 7,680 LC (94.2 %; v0.5 7,084–7,113), 24 / 32 EBR (unchanged); iceunpack OK ×6;
the packed config carries the new labels and presets. No pin-skew screen. Standalone yosys put the round at
+59 LUT / +23 carry / +26 FF.

**Checks** (working files in `.heirloom_work/rb_tangram/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- Engine-level default check (`eqchk.py`): the v0.5 and v0.6 microcode run side by side in the ISS over every line
  of 3 fields at HD, 720p and SD: OUT ports, coefficient-engine weights and the XFER block identical, except the
  unused d4 word (slot 7, never read by the pixel path); slot 12 = the flow phase, slot 19 = 15939.
- `sweep.py`: every new knob path at RTL widths (16-bit stores, OUT port widths, ISS operand asserts) over knob
  extremes × LFO phase extremes × 4 modes: 0 violations after the slot clamp (the first run found the breakpoint
  overflow above). Wedge axis stays 374..1618 px; the ramp increment ≤ 1273 (11-bit port).
- Default frame (GHDL 720p60 dec 1, Osc, explicit defaults, 2 warm-up + 1 capture, production v0.5.0 simulated
  the same way): raw 10-bit output **byte-identical**.
- One extreme frame (`p2/r06_ext.png`: K4 100 %, K3 100 % part-glided (about ×2 after 3 fields), S7 Shred, Osc;
  rendered with the first sway period of 216 lines, before it was lengthened to 410): the wedge axis leans hard left
  with its dots, the plateau breaks into shards, the lower ramps slant steeply, Shred's glitch tears sit every 64
  lines, and the swaying fans fold into a pink herringbone. That herringbone is why the sway period was then
  doubled; engine defaults were re-checked after the change.
- Not verified: Video mode with a real camera (unchanged path), SD / interlaced renders, the LFOs' motion over
  time (reasoned from the ISS), Shred with Mirror.

## Reading of the photo (measured)
- **Mechanism:** an ADDITIVE RGB composite. Each colour channel carries one independent oscillator layer,
  summed by an encoder with slight crosstalk.
  - Fitted matrix: R = .924r − .043g − .049b + .084, G = −.027r + 1.031g − .030b + .025,
    B = −.051r − .071g + 1.004b + .018. Median error 5/5/5 per channel.
  - Typical colours: navy (13,9,122), bright blue (8,25,217), pink (250,7,113), cyan-green (3,218,198).
- **Red: pink stripe fans.**
  - Binary stripes with soft edges and a log-chirped phase (k = 3 ln(x − x0(y)) + m, rms 0.08–0.15
    stripe). They are thin and dense on one side, and each next stripe is ~1.4× wider.
  - Two families, "\" from the left and "/" from the top right, meet in "<" elbows at (1140,630),
    (1268,555), (1393,475), (1515,420) and (1650,355). Left stripes 2..5 bow ")".
  - Pink is off inside a top-centre triangle. The key lines are x = 274.5 + 1.078y (through the stripe
    tips) and x = 2108.6 − 0.739y (cutting the right family's lower arms).
  - Two torn 135-line bands of near-vertical stripes sit at the top left (x < 640).
  - Glitch rows 626..643 are shifted 0..34 px right of x 1060.
  - RC smear: rise 0.2, fall 0.125 per pixel. Duty 0.46.
  - The whole phase field is fitted as a quadratic B-spline on 64 px × 64 line knots: 8779 stripe samples,
    rms 0.013 stripe, p95 0.027. The torn bands have their own 5×12 tables.
- **Green: sawtooth ramps.**
  - Lower stripes rise to the right with phase −7.06435 + 12.56432Y − 3.04163Y² + 1.82364X (X, Y in
    1000 px; edges 1.2 px rms). The ramp runs 22 → 252 downward, and the edge spacing grows from 124 to
    200 lines.
  - The plateau has three bands: T1 lines 100..297, T2 298..548, T3 549..610.
    - Each is a per-line sawtooth between breakpoint curves: lit start, drops d1..d4, key edge, and the T3
      lead drop.
    - The drops zig-zag in ">" shapes with a ~240 px period.
    - T1 and T2 ramps are linear (25 → 240). The T3 ramp is convex (f^1.6).
- **Blue: navy base (B 125 at the top → 115 at the bottom).**
  - Wedge: |x − 996.2| < hw(y) = 398.34 − 0.42259y + 6.4545e-5y² (rms 0.5 px).
  - Lower V: y > 1019.44 − 0.36695h − 8.071e-5h², h = |x − 997| (rms 1.3).
  - Both have a 60-px dark fringe outside the edge, a sharp jump to 240 at the edge, and stripes parallel
    to it: wedge period 148 (74 sub), V period 166 (83 sub). Thin dark lines at v ≈ 96 and 432.
  - Small black dots sit on the axis at y = 25, 385 and 810.

## What it does
- **Pink:**
  - Every line, a coefficient engine blends three rows of the fitted spline table (weights × Fan) into
    32 coefficients. It samples the spline every 32 px into a ping-pong RAM of (P, Q) segments.
  - The pixel evaluates P + Q·g/32 and adds the flow and the source bend. It applies the two key lines,
    the torn-band tables and the glitch shift, takes the stripe duty, then applies the RC smear.
- **Green plateau:** per line, the engine evaluates 8 breakpoint curves for the current band (zig curve
  plus 7 slots) and the per-pixel ramp increments. The pixel finds its segment with comparators and runs
  a ramp DDA that restarts at each breakpoint and runs backwards on the mirrored half.
- **Green lower ramps:** a polynomial line offset plus a per-pixel DDA.
- **Blue:** closed form. Wedge distance and V distance address the measured profile ROMs (edge part and
  one stripe period plus flow). The dots are dimmed toward black.
- **Colour:** the three 8-bit layers go through three RGB→YUV ROMs with the fitted matrix folded in.
  Luma is pre-boosted 16/13.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Width | pink stripe width, duty 0.08 .. 0.9 (photo 0.46) |
| K2 Wedge | blue wedge opening ×0.42 .. ×2.4, bipolar, centre = photo (rays keep their 74 px pitch) |
| K3 Zigzag | plateau zig-zag depth ×0 .. ×2 (0 = straight drops) |
| K4 Speed | bipolar flow, still at centre (±8 deadband, square law): pink fans stream, green ramps scroll, blue rays flow; default slow forward |
| K5 Ramps | lower green sawtooth count ×0.5 .. ×2.9 (photo ×1) |
| K6 Bend | how far the source bends the pink stripes (and 0.6× the lower green ramps) |
| S7 Tear | torn top-left bands + glitch rows (On = photo) / clean field |
| S8 Key | centre-triangle key + right cut of the pink fans (On = photo) / pink everywhere |
| S9 Palette | Photo / Rotate (pink → green, green → blue, blue → red) |
| S10 Mirror | pink and green layers mirrored about the wedge axis (symmetric chevrons) |
| S11 Source | Video (default) / Osc |
| P12 Fan | pink stripe count ×0.46 .. ×4.2: 0 % = 3–4 broad chevron bands, 35 % = photo, 100 % = dense fine fans |

## Video
- Camera luma, (Y >> 2) − 128, is scaled by Bend.
  - It is added to the pink stripe phase: ±1/8 stripe at the default, up to ±0.8 at maximum.
  - About 0.6× of it goes into the lower green ramp phase.
- The stripes and ramps bend around the picture while the layout stays the photo's.
- The video tap is luma only, into geometry, never to the output.
- Osc mode substitutes two triangle oscillators that drift diagonally per pixel, per line and per field,
  averaged into a soft moving wave. Bend is live in both modes.

## Engine
- **Microcoded engine** (`mc2.py` → `tangram.py` UC/RFI):
  - **Datapath:** one 20-bit accumulator ALU. The register file (256×16) and the program (337 of 512
    words) live in block RAM.
  - **Pipeline:** F–D–O–E at one instruction per clock. A taken branch costs 4 clocks.
  - **Arithmetic:** a radix-4 Booth multiplier (10 clocks) and a 16/13 serial divider (18 clocks).
  - **Hazards:** the assembler inserts the NOPs; there is no forwarding.
- **Field program (vblank):**
  - glides for K1/K2/K3/K5/P12 (1/8 per field, ±4 deadband);
  - exponential knob curves for Fan, Wedge and Ramps;
  - key thresholds, duty, bend gain, speed and flow;
  - the 24 plateau curve pointers.
- **Line program (one line ahead, at the source's avid rise):**
  - spline weights × Fan, then the pink coefficient engine kick;
  - wedge, ramp and key polynomials and the dot phase;
  - breakpoint curves with zig offset, and ramp increments via the divider.
  - XFER then copies the 19-word block into the line registers in hblank.
  - Worst line: 771 clocks at HD, 743 of 858 at NTSC, 715 of 864 at PAL.
- **Pink coefficient engine:** a 14×4 multiplier over the weights' 4-bit digits (Horner, 12 clocks per
  coefficient), then a 4-stage chord pipeline. 96 taps per line in ~400 clocks.
- **Pixel path and shell:**
  - The pixel path is 25 stages, then the conditioning: luma [1 2 1], 8-bit chroma
    [1 2 2 2 2 2 2 2 1]/16 with its delay in EBR (re-synced every blanking), luma ×13/16.
  - The gate and avid sit at the 4-pixel filter centre (the escalator shell gated 4 px early; fixed here,
    output timing relative to hsync unchanged). Syncs go through an EBR delay of 28.
  - The generated avid is parity-locked (escalator shell), and the timing-ID virtual-1080 steps apply.
  - One latency of 28 in every mode.
- **ROMs:** pink tables 768×16, curves 3×256×16, YE 512×16, BW/BV 512×8, L3 256×8, colour 6×256.

## Status
- v0.5.0 built 2026-10-27 with **ROUTER1**; router2 misses at 61–73 MHz on HD Analog.
- Seeds: all configs seed 1 except **HD Dual (seed 5)**.
- Post-route Fmax: HD Analog 85.85, HD HDMI 77.81, HD Dual 78.82, SD Analog 78.40, SD HDMI 74.83,
  SD Dual 84.57 MHz.
- 7084–7113/7680 LC (92–93 %), 24/32 EBR. iceunpack OK.
- GHDL render at 720p60, decimation 1, matches the integer model exactly on Y, U and V (Osc, default
  knobs; Video and motion checked).
- NOT flashed.

## Deviations from the photo, and why
- The pink phase is evaluated as 32-px chords of the fitted spline (one multiply per pixel, to fit the
  chip). The error is ≤ 0.02 stripe at the photo's Fan and 0.08 at maximum Fan.
- The plateau edges are breakpoint curves; the T2 bottom curls use 4 breakpoints. The zig is one shared
  per-line offset. Ramps are per-pixel DDAs, so at SD/720p a ramp can start up to one virtual pixel late.
- The left-edge dashes (x < 70) are only approximated by flattening the V edge near the line start.
- The glitch rows are a fixed ramp shift, not the photo's exact displacement.
- Osc bend is a smooth synthetic wave, not the photo's (unknown) camera content. In Video mode, raw luma
  frays the stripe edges on textured sources.
- Colour comes from a fitted 3×4 matrix (median error 5 per channel). The dots are dimmed to 1/8, not
  pure black.
- At SD (4:3) the 16:9 composition spans the full width, so shapes are 25 % narrower. Interlaced modes
  draw each field over the full height (line-doubled).
- The picture starts 3–4 px right of the source's active window (the generated-avid recipe). Sims show
  a 4-px blank left column.

## Open questions for Ron (2026-11-09 batch)
1. K4's loosening (wedge swing, pink sway, plateau slide) only starts past the photo's speed. Would you rather have
   it on its own control, for example a switch that toggles the swing at any speed?
2. K3 now also slants the lower ramps. Is coupling the zig and the slant right, or should the slant go back to
   fixed?
3. S7 lost its Off (clean field) position to Shred. Do you miss the clean field?
4. The lower blue V never moves. Should it swing against the wedge, like scissors? That needs one more per-line
   word, about 40 LC.

## Hardware watch list
- Renamed values: S7 Tear is now **Shred / Photo** (Photo = the old On); K3 Zigzag also slants the lower ramps and
  reads −100..100 (0 = the photo).
- K4 past the photo's speed now moves the wedge, the fan stripes and the plateau. Expected, not a fault.
- The engine's NTSC line budget is 812 / 858 clocks (v0.5: 743). If SD shows a stale or torn plateau line, the
  line program is overrunning.
- K3 at 100 % throws the plateau drops up to 820 px: wide shards are expected.
- Router: ROUTER1 (as v0.5). Seeds: HD HDMI and HD Dual on seed 2 (seed 1 missed), the other four on seed 1.

## Next step
Wait for Ron's preview notes. Rebuild the diag with
`NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh tangram`. On approval: `./heirloom_diag.sh merge tangram`,
then repack the previewed diag bitstreams under the production TOML (copy
`build/programs/ron/tangram_diag/rev_b/bitstreams/*.bin` into the production build dir, regenerate
`program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`) instead of re-routing;
update the production SUMMARY / README / PHASE2 row and commit only tangram's files.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 now loads at 100 % (initial_value 1023), and the camera effect at the top of the knob is 25 % stronger.

Bend / element: K6 Bend is how far the source bends the pink stripes (and 0.6x the lower green ramps).

Diag copy: the same change (RFI[246] in tangram_diag.py, 410 -> 512), no render. The microcode's bend gain is BK = k6 x 512 >> 8 = 2 k6, 0..2046 (was k6 x 410 >> 8, 0..1638): x1.25 at every position, inside f_bk's 11 bits. It is a data-only change (the constant is read only by that MUL); no logic changed. The assembler sources .heirloom_work/rb_tangram/mc2.py and p2/mc2.py say const(512) to match.

Build: router1, seeds other than 1: HD HDMI seed 2, HD Dual seed 2, worst HD Fmax 78.67 MHz, 7237 LC (max of 6). Fmax per config: HD Analog 79.44, SD Analog 76.27, HD HDMI 78.67, SD HDMI 73.56, HD Dual 80.66, SD Dual 76.60 MHz.
