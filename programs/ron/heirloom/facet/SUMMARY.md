# Facet — mirrored green eyes, red cone, white-edged wedge and floor
**v0.6.2 (2026-11-02, phase 2): approved by Ron 2026-11-02 after two diag rounds, merged. Each eye pair is its own
module; the wedge sweeps to flat, the floor's slope rides its height, and the wedge/floor overlap is an XOR mix.**
Presets are still placeholders. Heirloom #40, from
`photos/WIN_20230602_01_26_17_Pro.jpg`. v0.1.0 drew three rotating wedge fields and a circle, and Ron judged it too
abstract. v0.5 was rebuilt from a numeric reading of the photo: a float prototype iterated to a near-replica, then an
integer model, then the RTL. Work files are in `.heirloom_work/rb_facet/`; the phase-2 diag copy is in
`.heirloom_work/rb_facet/diag/facet_diag/`, where it was built and previewed as "Facet Diag"
(`./heirloom_diag.sh build|merge facet`).

## Phase 2

### Round 1 — v0.6.0 diag (2026-11-01)
Ron left no notes on v0.5.0, which has never been flashed. He picked a proposed map (all four recommended options),
and asked for it as a diag preview beside the original:
- **K5 Halo -> Spread.** Each eye pair gets its own height, `Hl_i = Hl x (1 + i sigma)`. i counts the pairs out from
  the centre, and the centre eye stays on P12. sigma runs -0.9 per pair at 0 (the pairs shrink outward to slits),
  0 in a ±16 deadband round 50 % (all equal: the photo), and +1.5 per pair at 100 % (x2.5, x4, ...; clamped at
  2047 lines). Under Twin the flanking pair is i = 1.
- **The halo width rides on P12 Open:** x0.15 at 0 %, the photo's width at the default 47 %, x1.95 at 100 %.
- **K4 Speed = rate + depth, square law.** Still below about 4 % (`dep = speed² / 2048` is 0 there). At 50 % (the
  new default, 512) the rate and the depths are v0.5's: breath ±6 %, wedge ±12 %, floor ±25 lines, eye sway ±3 % of a
  period, colour drift ±49. At 100 % the rate is x4 and these depths are x4. Two new motions deepen with speed
  cubed:
  - **the eye-pair ripple:** each pair bobs on a wave running out from the centre, a quarter wave per pair, at twice
    the breath's rate. It is ±4 lines at 50 % and ±253 at 100 %.
  - **the slide:** the wedge, the floor and the gradient slide sideways across the row of eyes, at the breath's rate
    a quarter wave later. It is ±4 px at 50 % and ±253 px at 100 %.
- **S7 Lines -> Floor: V / Peak.** Peak flips the floor into a purple pyramid. Its apex sits 370 lines above the V's
  apex (y 716 at the default), so it rises toward the wedge tip and makes an hourglass. K3 still moves it, and the
  white lines and halo follow its edges. The white edge lines are always on.
- **S8 Glow -> Eyes: Lens / Diamond.** Diamond is a second bank of the shape ROM, with straight flanks to the same
  tips. The pale rims are always on.
- **Default frame.** At Speed 0 with K5 centred, the geometry is v0.5's. At the default P12 the halo width is v0.5's
  256. The output chroma now passes through an 8-bit filter path, which changes it by about ±2 codes. At the new
  default Speed (50 %) the frame carries v0.5's motion plus the ±4-line ripple and the ±4-px slide.

**Engine changes.** The lens-phase DDA is 27 bits. Its top three bits count eye periods, biased by one, so the eye
index never wraps. The index reads an eye table of 8 entries × 32 bits (2 EBR, `no_rw_check`), {Hl_i, yo_i}, at
pixel stage 1. Stage 4 multiplies by Hl_i in place of the field's Hl, and computes `|Y − 518 − yo_i|` (stages 4–5).
A neighbouring eye's values switch at the tip, where both heights are below zero, so there is no seam.
The zone sequencer grew from 28 to 54 jobs over 1296 clocks (the zone counter is 11 bits). The new jobs are:
- speed², the rate, dep², dep³;
- a sine × depth job before each of the 5 LFO jobs;
- the slide and sigma;
- 8 × (Hl_i, yo_i) for the eye table.
The LFO sine index is now set by the sequencer, two clocks before its job.

**Fit and timing.** The first build was 7585 LC (98.8 %) and routed at 41 MHz. The congestion sat on the job decode
feeding the multiplier's operand mux. Getting it to 7387 LC (96.2 %) and 86 MHz on HD Dual (router1, seed 1) took
these changes:
- The operand select codes (`a_sel`, `b_sel`, and a `b_cst` constant register) are registered together with
  `m_jid`, from the job id that is known 24 clocks ahead. The operand fetch is then a shallow mux. Job 0's
  `vw + 48 + xs − 1` became a register, and `K5 − 512` is the inverted MSB.
- The eye-table write port is driven straight from the multiplier.
- One scratch register holds dep², then the LFO sine × depth, then Hl_i.
- The eye index is `floor(phase + ½) mod 8`, with no bias. A small negative sway lands on eye 0.
- The chroma filter's delay lines moved into a 256 × 16 block-RAM ring, read 7 behind (the cutout tip).
- The 28-deep sync delay line moved into a 256 × 4 block-RAM ring, read 25 behind (the gate tap), plus two FFs.
  The latency is unchanged.
- The wedge and floor DDAs are 26 bits wide. Dropping the low bits costs under 0.25 px over the frame.
- Peak's apex offset is folded into the B0 constant (1016 instead of 1386).

Peak changes only the floor's per-field start, the sign of `|X − axA|`, and the side used for the floor edge and its
x-order gap.

### Round 2 — v0.6.2 diag (2026-11-02)
Ron, after flashing v0.6.1:
> I would like Wedge to sweep further. Floor should change angle as it raises and lowers. When floor and wedge
> overlap, there should be another modulation like a x0r or something that mixes floor, wedge and the background in
> an interesting way. make sure to incorporate the incoming video if that's coming in.

- **K2 Wedge sweeps further.** The law is now cubic, `kA = 0.02 + 4.98 (K2/1023)³`. It runs from a needle (0.02) at 0,
  through the photo (0.3045) at 38 % (the new default, 394), to nearly flat (5.0, the edges about 11° off
  horizontal) at 100 %. v0.6 topped out at 1.2. The wedge LFO swings on top of it (±12 % at the default Speed, ±48 %
  at 100 %).
- **K3 Floor changes angle as it rises.** The slope rides the height on a smooth quadratic,
  `2/kB = 9.6 − 15.1x + 7.2x²` with x = K3/1024, two Horner jobs. Sunk off-screen the sides are shallow
  (kB 0.21). At the default 33 % it is exactly the photo's 0.3732 (2/kB = 686 in Q7). Risen to mid-screen it is a
  steep V (kB 1.2). Peak's pyramid sharpens the same way as it climbs.
- **Overlap mix.** Where the wedge and the floor overlap there is an XOR. Outside the eyes it alternates the purple
  and the ground colours, and inside the eyes red and light green.
  - The bands are bit 5 of the gradient index, about 256 D units wide, so they follow the Λ gradient and its drift.
  - Each band is flipped by the parity of the source's top three luma bits, which gives contours every 32 levels.
    The incoming picture shows as purple/ground posterized shapes inside the overlap, and the Osc plaid does the
    same in Osc mode. With no video the overlap is plain diagonal bands.
  - The photo has no overlap (wedge tip y 942, floor apex 1086), so the default frame is unchanged. Raise K3,
    widen K2, or flip S7 to Peak to see it.
- **Paid for, at about the v0.6.1 LC:**
  - The five LFO phase multiplies moved onto the serial multiplier as ratio jobs (master phase × 1 / 11 / 9 / 6 /
    14, and the slide as the breath + 64).
  - X is computed at pixel stage 4 from the free-running beam (b_x − 3 xs), which drops the 3-stage |X| delay chain.
  - The halo inner gaps follow the halo width without the 256 clamp register. `f_iAR` is 7 bits; the default is
    unchanged.
  - The rate is now `speed² / 128` (v0.5's at 50 % within 10 %, x4 at 100 %), one job fewer.
- **Wider fields.** kA and kAf are 15 bits, the wedge DDA is 28 bits, and the floor slope register `f_kf` is Q7. The
  per-line wedge and floor offsets saturate (±4095 / ±8191), which only changes values far from any edge.
- The zone sequencer is 46 + 16 jobs (ids 0–45, the eye table 48–63, line jobs 46/47); line 0 is set up at zc 1560.

## Reading of the photo (measured)
- **Gradient.** `D = y − 1.05|x − 987|`, 45° contours. The background is linear in D (rms 1.5–2.8/255): (0,64,200)
  at the top corners → (0,150,60) at the bottom centre. Purple, red, dark green and light green are each 1-D ramps
  over D. The black cone tip is the dark end of the purple ramp.
- **Eyes.** Period 942–946 px, centre x 984–985. Symmetric about y = 518.5 (top + bottom constant to ±2 px).
  `h(u) = 308·sin(π/2·(429 − u)/470)`, median error 1.25 px; a cos^g shape gave cusped tips and was rejected.
- **Wedge A.** `x = 706.1 + 0.3045y` and `1275.4 − 0.3044y` (0.55–0.70 px rms); axis 990.75; tip y ≈ 942.
- **Floor B.** `y = 0.3759x + 713.1` and `−0.3708x + 1453.6`; apex (991.7, 1085.9).
- **Red** is eye ∩ (A ∪ B).
- **Halo.**
  - Light green beside the A/B edge inside an eye, `Y = 131 + 0.57(D + 275)`, fading to dark green along
    D ≈ −216 in the half-eyes.
  - Before a rising A edge the band is 160 px wide (+0.0012·v²). After a falling edge there is a 38 px dark-green
    gap, then 130 px.
  - B: a 30-row gap in the left half-eye, 6 rows in the right; the band top is at B0 − 176.
- **Edge profile** (vs. horizontal distance hd, polarity-independent):
  - shadow from hd −24 to −4: bg luma × 0.34, Cr × 0.3, Cb + 25;
  - white line peak at hd 0: Y 515–655, ~6 px FWHM;
  - blue after-glow from hd +4 to +20.
- **Rims.**
  - Left-facing edges in the halo: a dip at 0–8 px, then a pale rim peaking at 14–16 px (Y ≈ 785), gone by 40 px.
  - Right-facing edges: white at +4 px.
  - Dark-zone edges: soft over ~15 px, with Cb leading by ~5 px.
- **Texture.**
  - Line jitter 0.6 px rms (lag-1 correlation 0.7).
  - Light-green streaks: Y std 4.6, 6–8 px long.

## What it does
1. **Fields.** Virtual 1080-line coordinates, X in Q.1.
   - Folded eye-phase DDA on |X − 24|; 512-entry shape ROM s(t) plus a perpendicular factor c(t); `h = Hl·s`;
     `m = h − |Y − 518|`; `mp = m·c`.
   - `eA = |X − 31| + kA(Y − 942)` and `eB = (B0 − Y)/kB − |X − 31|`, from per-line DDAs.
   - `D = Y − 1.05|X − 31| + Doff`.
2. **Source.** The source luma (line buffer, half resolution) times K6 adds to D (±520), the eye margin (±27) and
   the edges (±20 px).
3. **Colours.** Class-addressed 256-entry ramp ROMs:
   - CO = background or purple;
   - CI = dark green, light green, red, or the light/dark half-way colour;
   - TG = the white line or the shadow.

   Then `X = lerp(CO, CI, alpha(mp))`, with Cb using an earlier alpha. Then `out = lerp(X, T, w2)`. Outside an eye,
   T is the x-order shadow or white line, with the blue after-glow. Inside, T is the rim white or an outline dark:
   red-cone outlines, left-edge dips, dark-zone dips.
4. **Ember** complements U/V. Then the output conditioning and the gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Eyes | eyes' count / size | 0.75 … 6 across, exponential (2.03 = the photo at 48 %) |
| K2 Wedge | purple wedge, red cone | kA = 0.02 … 5.0 about the fixed tip, cubic (0.3045 = the photo at 38 %) |
| K3 Floor | purple V floor (or peak), red caps | apex from sunk off-screen to mid-frame (peak: low hump .. near the top); the slope rides it, kB 0.21 → 0.373 (photo, 33 %) → 1.2 |
| K4 Speed | motion | rate and depth, square law: still below ~4 %; at 50 % v0.5's motion (breath ±6 %, wedge ±12 %, floor ±25 lines, eye sway ±3 %, colour drift ±49) plus the eye-pair ripple and the slide at ±4; at 100 % rate and depths x4, ripple / slide ±253 lines / px |
| K5 Spread | each eye pair | pair heights 1 + i sigma: sigma −0.9 (slits outward) .. 0 (photo, deadband at 50 %) .. +1.5 (swelling outward) |
| K6 Picture | gradient and edges | source depth |
| S7 Floor | floor | V (the photo) / Peak (pyramid toward the wedge tip, an hourglass) |
| S8 Eyes | eye shape | Lens (the photo) / Diamond (straight flanks) |
| S9 Twin | centre eye | eye phase + P/2 |
| S10 Palette | all colours | Jewel / Ember (chroma complement) |
| S11 Source | — | Video / Osc |
| **P12 Open** | the eyes, the halo | Hl 12 … 640 and halo width x0.15 … x1.95: shut slits → the photo (47 %) → giant eyes filling the frame |

K1, K2, K3, K5 and P12 glide 1/8 of the gap per field and snap on the first fields.

## Video
The camera's luma shades the gradient: the picture shows as blue/green relief in the ground, the purple and the halo.
More gently, it bends the eye, wedge and floor edges, which wrap around the picture. At the default 30 % the
composition reads as the photo. Osc substitutes a slow triangle plaid (±46/255) drifting with the master LFO. Every
video-path control stays live in Osc.

## Engine
- **Serial multiplier.** One shift-accumulator multiplier, 27 × 16 bits, 16 steps.
  - **Per field (zone):** 28 jobs in fixed 24-clock slots: pixels left of the axis; LFO increment; k2² and kA; Hl
    and its LFO; kA and its LFO; B0, plus its LFO; the halo widths; Doff; eye sway / Twin; incp; ph0; r0; phx; the
    wedge and floor DDA starts and steps; Osc y-increment.
  - **Per line:** about 16 clocks after each line, the DDAs step and two jobs compute the halo bow
    `|v|²·hs·0.0011`.
- **Timing support.** Registered zone decodes; glides on one shared datapath, pipelined over 4 clocks per channel.
- **Pixel path (28 stages).**
  - 1–10: fields, keys and distances.
  - 11: halo class and ROM addresses.
  - 12–13: ROMs, then fabric copies.
  - 14–19: the two lerps (weights 0–8, gated-operand multiply), after-glow, streaks.
  - 20: Ember.
  - 21–26: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]).
  - 27: luma × 13/16 and the blanking gate.
- **ROMs (20 EBR):** CO 512 × 32, CI 1024 × 32, TG 512 × 32 (luma stored × 16/13); shape 512 × 16;
  eye-increment 256 × 16; source buffer 1024 × 8 with `no_rw_check`.
- **Shell:** the heirloom v0.2.0 shell — generated avid, one latency in every mode (C_LAT 28), timing-ID virtual
  steps, colours in the hardware convention.

## Status
**v0.6.2, approved 2026-11-02 and merged.** `out/rev_b/ron/heirloom/facet.vmprog` holds the approved diag's six
bitstreams, repacked with the production TOML (id `com.czartroniks.facet`). The merged sources are byte-identical to
the diag's. Ron flashed the v0.6.1 diag (colours correct). To rebuild from source, run
`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 ./build_heirloom.sh facet`. HD Analog stalls router1 on seed 1;
the builder then lands seed 2.

**v0.6.2 diag built 2026-11-02** (round 2: wedge sweep, floor slope, overlap XOR), router1, builder seeds:
- HD Analog 85.31 MHz on seed 2. Seed 1 stalled router1; there are no carry self-adds.
- SD Analog 81.59, HD HDMI 79.95, SD HDMI 85.04, HD Dual 83.46, SD Dual 84.51 MHz, all on seed 1.

7414–7438 / 7680 LC (97 %), 26 EBR, all bitstreams pass iceunpack. The preview was
`out/rev_b/ron/_diag/facet_diag_v0.6.2.vmprog`.

**v0.6.1 diag built 2026-11-01: output-filter resync fix, HW-CONFIRMED (Ron 11-01: colours "look correct")** (`out/rev_b/ron/_diag/facet_diag_v0.6.1.vmprog`). Ron saw wrong colours on v0.6.0. The chroma box is a running 8-sum on a block-RAM ring that never resynced, so one bad sample offset the colours for good. Now p20_u/v go neutral off the generated avid (new 19-FF g_avid delay, s_gav(18)) and the sums snap to 1024 after 8 blank samples. Builder seeds, router1, all seed 1: HD Analog 81.83, SD Analog 85.01, HD HDMI 88.04, SD HDMI 83.52, HD Dual 86.08, SD Dual 84.07 MHz; 7417-7462 LC. The hand-installed HD HDMI s10 is gone (new netlist).

**v0.6.0 diag** built 2026-11-01 with **router1** (`NEXTPNR_ROUTER="--router router1"`; router2 reached only
71 MHz on HD Dual). The preview is `out/rev_b/ron/_diag/facet_diag_v0.6.0.vmprog`, program id
`com.czartroniks.facet_diag`, shown as "Facet Diag". Not merged, NOT flashed.

| Config | Fmax (MHz) | Seed |
|---|---|---|
| HD Analog | 87.19 | 1 |
| HD HDMI | 85.24 | router1 seed 10, hand-installed |
| HD Dual | 86.28 | 1 |
| SD Analog | 83.89 | 1 |
| SD HDMI | 88.84 | 1 |
| SD Dual | 81.48 | 2 (seed 1 stalled router1) |

7372–7397 / 7680 LC (96 %), 26 / 32 EBR. All bitstreams pass iceunpack.

**HD HDMI pin-skew screen** (`tools/hdmi_pin_skew.py`, 23 routes: router1 seeds 1, 3–18 and router2 seeds 1–12):
- The builder's seed 1 sat at hs-clk −0.55, inside the danger zone. No router1 seed from 1 to 9 reached +1.0.
- **Router1 seed 10 gave +2.03, with every data bit at or below −0.55**, at 85.24 MHz. It is hand-installed in the
  preview vmprog. Beacon s42 at +2.31 and cutout s17 at +2.01 were HW-clean.
- The other near-window routes were router2 seed 10 (+2.12, data −0.32, 80.2 MHz) and router2 seed 3 (+1.52, data
  −0.20, 76.1 MHz).
- The netlist, asc and report are in `.heirloom_work/rb_facet/diag/seeds/`.
- A plain `heirloom_diag.sh build` lands the builder's seed 1 again, so re-install seed 10 and repack. The repack is
  `vmprog_pack.py --no-sign --hardware rev_b --toml-path <diag toml> build/programs/ron/facet_diag/rev_b/ <out>`.

v0.5.0 (production) was built 2026-10-21 with router2. Its seeds were HD Dual seed 2 and the others seed 1. It ran
at 77–88 MHz, 7143–7186 LC (93–94 %), 20 EBR. NOT flashed.

## Deviations from the photo
- **Halo boundaries.** The left/right asymmetry of 15–30 px is kept only as the x-order gap and per-side widths, so
  the tip boundaries are within about 10–15 px of the photo.
- **Axes** are +24 / +31 px off centre, as in the photo.
- **Camera blur** is not reproduced.
- **Noise** is a different realisation of the same statistics.
- **Rim highlights** are capped at about 777 by the luma scale.
- **SD** shows the central 1440 virtual px, with the half-eyes cut to their tips.
- **Sim previews.** `sim.sh` previews show the R≈0 background redder than it is; read the raw `output.txt` instead
  (`rb_facet/raw2png.py`).
