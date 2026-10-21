# Facet — mirrored green eyes, red cone, white-edged wedge and floor
**v0.5.0 (rebuilt 2026-10-21): built, NOT flashed.** Heirloom #40, from `photos/WIN_20230602_01_26_17_Pro.jpg`.
v0.1.0 drew three rotating wedge fields and a circle, and Ron judged it too abstract. This version is rebuilt from a
numeric reading of the photo: a float prototype iterated to a near-replica, then an integer model, then the RTL.
Work files are in `.heirloom_work/rb_facet/`.

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
| K2 Wedge | purple wedge, red cone | kA = 0.04 … 1.2 about the fixed tip (0.3045 = the photo) |
| K3 Floor | purple V floor, red caps | apex from sunk off-screen to mid-frame |
| K4 Speed | motion | breath ±6 %, wedge ±12 %, floor ±25 lines, eye sway ±3 % of a period, colour drift ±49 D, Osc drift; one master phase, ratios 16:11:9:14:6; still below 2 % |
| K5 Halo | light band, rims | width × 0.15 … 2.5 |
| K6 Picture | gradient and edges | source depth |
| S7 Lines | white edge lines | on/off (On) |
| S8 Glow | pale rims | on/off (On) |
| S9 Twin | centre eye | eye phase + P/2 |
| S10 Palette | all colours | Jewel / Ember (chroma complement) |
| S11 Source | — | Video / Osc |
| **P12 Open** | the eyes | Hl 12 … 640: shut slits → the photo (47 %) → giant eyes filling the frame |

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
v0.5.0 built 2026-10-21, router2. Seeds: **HD Dual = seed 2** (seed 1 stalled the router); the others seed 1.

| Config | Fmax (MHz) |
|---|---|
| HD Analog | 85.24 |
| HD HDMI | 79.48 |
| HD Dual | 77.14 |
| SD Analog | 87.51 |
| SD HDMI | 81.20 |
| SD Dual | 79.12 |

7143–7186 / 7680 LC (93–94 %), 20 / 32 EBR. All bitstreams pass iceunpack. NOT flashed.

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
