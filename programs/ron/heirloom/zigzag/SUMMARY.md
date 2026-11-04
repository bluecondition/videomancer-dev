# Zigzag — rainbow zigzag ribbons on green, smoked by the camera

**v0.1.0 (2026-11-04): built, NOT flashed.** Heirloom phase 3, from the clip `WIN_20230205_00_16_05` at 0.5 s
(still `photos/vid/WIN_20230205_00_16_05_t0p5.png`). The still shows a bright green ground with two rounded-top red
zigzag bands, and pale blue / white ribbons running down every branch and meeting in V's. A smoky texture (camera or
feedback video) is keyed into the ribbons as yellow blobs and pink / cyan smears. Presets are hand-written placeholders.
Working files: `.heirloom_work/rb_zigzag/` (`notes.md`, `proto1.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **One field.** Every red line folds onto one symmetric field.
  - Apex axis x = 519.5, period 940.5 px. Each contour gives 4 samples per row, all within ±3 px.
  - Separable fit f = u(d) + v(y):
    - u(t) = (1 − t) + 0.3·t(1 − t), with t = d / 470.25. This is a rectified rounded triangle: sharp troughs at
      x = 50 / 990 / 1930, rounded peaks at 520 / 1460.
    - v(y) = 0.6876·y / 1080. It is linear: the quadratic term is −0.016.
    - Contour residual: rms 1.95 px, median 0.56 px.
  - The red lines sit at f = 0.98, 0.74, 0.60 and 0.35 (1 unit = the peak-to-trough swing).
- **Three shapers of f.**
  - **R:** four bumps at 0.35, 0.60, 0.74 and 0.98, peaking at 240–255. R is zero below 0.22, between 0.83 and 0.90,
    and above 1.12. On every row and both branches it crosses 64 at 0.836 and 0.905.
  - **G:** 0.82 at f = 0, a minimum of 0.48 at 0.33, a maximum of 0.92 at 0.62 (the pale ribbon), 0.30 at 0.98 (the
    dark-green Λ interior), rising to 0.97 at 1.68 (the bright bottom).
  - **B:** a window [0.515, 0.865] (50 % points 0.54 / 0.85) with a soft edge of ≈ 0.03, **delayed 36 px**.
    - The B pulses are symmetric about x = 553, against 520 for R and G.
    - This is why the colour order is the same on both branches: yellow, white, light blue, blue, magenta, red,
      orange, left to right.
- **Texture.**
  - B is low on 34 % of the window core (0.31–0.35 in the m-frames).
  - Hole runs have a median of 15 px × 19 lines; the islands between them, 32 × 30.
  - The holes are mostly yellow (R and G high), so B sees the texture with about 8× the gain of R and G.
  - The outer B edges stay clean, so the texture is keyed to the ribbon.
- **Palette** (RGB):
  - ground green (0, 220, 0), grading to (0, 255, 0) at the bottom and (0, 80, 0) inside the Λ;
  - red band (255, 50, 0), with orange / yellow flanks;
  - pale ribbon (240, 230, 255), light blue (120, 180, 255), blue (0, 110, 255), and a magenta fringe.

## Motion in the clip
- **The geometry is static** from t − 0.5 s to t + 1.5 s. The red contours sit at identical positions in every m-frame
  (cross-correlation dx = dy = 0).
- **Only the texture evolves.** The B mask toggles 0.10 of the frame per 0.5 s (0.095–0.106).
- **No knob move is visible** in the 3.5 s strip. The other look from the same session (`WIN_20230205_00_18_15`, two
  minutes later) is a pastel field of diagonal bands, with the fold off and the pitch and palette changed. That look
  gave S7 Slant and S10 Pastel.
- **In the program:**
  - The Osc texture drifts at the clip's rate: three sines moving 0.6–0.8 cycles/s. The model's B mask toggles 0.14
    of the frame per 0.5 s.
  - The geometry holds still, as in the clip.
  - The default Speed streams the colours slowly down the branches (one palette cycle per 27 s). This is the only
    motion the still cannot show; K4 at 512 stops it.

## What it does
1. **Field.**
   - The phase φ runs at Zigzags periods per 1920 virtual px.
   - u = table[fold(φ)], the rectified rounded triangle (1024 × 12).
   - f = u + Pitch·y/H + offset, in Q12. The shapers repeat every 2.0 units.
2. **Texture.**
   - T is either three drifting sines (Osc) or the camera luma's 16-px box minus the same box 40 px earlier (Video),
     scaled to ±1.
   - key(f) is a 7-bit window over [0.49, 0.84] with 1/32-unit edges.
   - kT = key·T·Smoke.
   - R and G read the field at f + ¼·kT. B reads the field delayed 36 virtual px plus 2·kT taken 36 px back: 8× the R / G shift.
3. **Shapers.**
   - R comes from a 1024-entry red table refilled every field: Lines re-indexes the measured R shaper about the ribbon
     centre.
   - G comes from its own table.
   - B = clamp(2·(hw − |f_B − 0.69|)), with hw set by Ribbon.
4. **Colour.**
   - Ground Black dims G to 1/8 outside the blue window.
   - Palette permutes the guns.
   - A fixed shift-add matrix gives Y (16/13 prescale, capped at 876) and clamped Cr / Cb.
   - Pastel halves the chroma and lifts the luma.
5. **Output.**
   - Chroma goes through [1 2 2 2 2 2 2 2 1]/16 and luma through [1 2 1]/4, both centred 4 px back.
   - Then luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Zigzags | the zigzag period: 0.5 .. 8 periods across, exponential (2.04 = the still, at 520) |
| K2 Ribbon | the blue window's width: ×4, a blue flood (0) .. the still (512) .. ×1/4, a thin blue line (1023) |
| K3 Lines | the red shaper's rate about the ribbon: ×0.4, fat bumps (0) .. the still (512) .. ×2.5, many red lines (1023) |
| K4 Speed | bipolar (±8 deadband, square law): the colours stream down (+) or up (−) the branches. 612 = one palette cycle per 27 s, full = 0.84 s |
| K5 Smoke | the texture's bite into the ribbons: 0 = clean ribbons, 285 = the still's hole fraction, 1023 = shredded lace |
| K6 Palette | which shaper drives which gun: Still / GBR / BRG / RBG / GRB / BGR / Negative / Neg GBR (8 equal bands) |
| S7 Fold | Zigzag (the still) / Slant: the table read unfolded, a sawtooth, giving diagonal bands with the shapers repeating every 1.0 |
| S8 Mirror | Off / On: the ramp folded at the centre line, so the zigzags meet in diamonds and X's |
| S9 Ground | Green (the still) / Black: the green shaper dimmed to 1/8 outside the blue ribbon, giving neon on black |
| S10 Tone | Neon (the still) / Pastel: chroma halved, luma lifted (the session's later look) |
| S11 Source | Video / Osc |
| P12 Pitch | the vertical ramp: 0 % = vertical rainbow stripes, 41 % = the still, 100 % = two nested zigzags per branch |

## Video
- **Video mode (the default).**
  - The texture is the camera's luma: a 16-px box minus the same box 40 px earlier (a horizontal band-pass), ×1.25,
    clamped.
  - Inside the ribbons it pushes the shapers' input, so the ribbon shows the picture's mid-scale structure as yellow
    holes and pink / cyan streaks, with B hit 8× harder than R and G.
  - Outside the ribbons nothing changes.
  - With no input the ribbons are clean: the still without its smoke.
- **Osc mode.**
  - Three sines replace the camera: periods 48 / 66 / 84 virtual px at three angles, drifting 0.6–0.8 cycles/s.
  - At the default Smoke they give the still's hole fraction.

## Engine
- **Per-field sequencer (vblank zone).**
  - The shared glide covers K1, K2, K3, K5 and P12.
  - One serial multiplier runs 12 slots: the phase step from an exp2 ROM, the trough anchor, pitch², pq, speed², the
    rate, the ribbon half-width, Smoke gain² and the red-table start.
  - Then the red table is refilled: 1024 ROM reads into a RAM, zone clocks 700 .. 1730.
- **Per line.**
  - v = (|vy − cy| or vy)·pq >> 20, on a serial multiplier.
  - The texture line phases step by constants of the timing ID.
- **Pixel path (24 stages).**
  - 0: the phases (R/G, the B field 36 virtual px back, three texture phases).
  - 1: fold and table addresses. 2: ROMs. 3: fabric copies.
  - 4: the fields and the texture sum. 5: key distances, T × gain. 6: key. 7: key × T, also written to the ring.
  - 8: the R/G field. 9: table addresses. 10: the B field plus the ring's term. 11: fabric copies, |f_B − c|.
  - 12: Slant wrap. 13: B window. 14: Ground / Palette.
  - 15: matrix partials. 16: sums. 17: luma cap, chroma clamp. 18: compose.
  - 19–22: filters and delays. 23: luma 13/16. 24: the gate.
- **Memories (21 EBR):**
  - u table ×2 (6), G ROM (2), R ROM (2), red-table RAM (2), sine ROM ×3 (3), exp2 (1);
  - rings, all `no_rw_check`: kT (1), video (2), chroma (1), syncs (1).
- **Contracts:**
  - the generated avid (parity-locked);
  - one latency of 32 in every mode: the output avid sits 32 clocks after the input avid, with the filter-centred
    picture on it;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr);
  - running-sum resyncs on the video box and the chroma box.
- **Verified (GHDL, 720p60, decimation 1).**
  - The default Osc frame, and the next field, are bit-exact with `imodel.py` on every interior pixel (max
    difference 0).
  - The Video frame (selfie.jpg as the camera) shows the picture's streaks in the ribbons with the composition intact.
  - A Speed-760 capture showed the red lines streaming 6–10 px along the branches.
  - Knob sweep at RTL widths (`sweep.py`): no overflow.

## Status
- **v0.1.0 built 2026-11-04 with ROUTER1, all six configs on seed 1. NOT flashed.**
  - Builder post-route Fmax: HD Analog 90.07, SD Analog 86.01, HD HDMI 91.99, SD HDMI 88.40, HD Dual 89.73,
    SD Dual 92.64 MHz.
  - 5863–5882 / 7680 LC (76–77 %), 21 / 32 EBR. iceunpack OK ×6.
  - Output: `out/rev_b/ron/heirloom/zigzag.vmprog` (413,596 bytes, unsigned).
- **Rebuild:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh zigzag`
  (or `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh zigzag`).
- **router2:** an earlier netlist closed HD at 84–86 MHz, but SD Dual stalled (overused = 2 stuck for 3000+ iterations
  until the timeout). Stay on router1.
- `hd_clock_divisor` untouched. No pin-skew seed screen was run; if HD HDMI shows lines, re-seed that config.
- **Fit history.** The first cut did not fit (8820 LC). It had a hue rotation of 6 per-field multipliers, 4 texture
  waves with a warp, a Lines pixel multiplier and sync shift registers. The cuts:
  - hue rotation → K6 Palette (gun permutations);
  - Lines → a per-field red-table refill;
  - the B texture term → the R/G term delayed through a ring;
  - the warp wave dropped;
  - syncs → a block-RAM ring.

## Deviations from the photo
- **Texture.** It is a generator (Osc) or the live camera (Video); the still's own smoke was camera / feedback content
  and is not reproduced. The Osc lattice is regular (three sines), where the still's blobs are irregular with
  horizontal streaks. The hole fraction (0.33–0.36 against 0.34), hole size (~15–30 px) and drift rate are matched.
- **Ribbon whites.** The R and G shapers are 1024-entry tables of the measured curves; the camera's bloom that widens
  the pale ribbon's whites is not modelled.
- **Slant** reads the table unfolded, so its sawtooth runs at twice the zigzag's rate at the same K1.
- **Line edges.**
  - The first 43 px of a line have no B texture (the ring there holds the previous line's tail).
  - The last pixel column is blank: the generated avid starts 2–3 clocks after the source's.
- **Levels.** Whites come out at ~777 (the shell's 13/16 luma cap). Chroma is 8-bit through the output box.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3: 1440 virtual px, one and a half periods at the default, with the period and pitch
    unchanged.
  - The 16-px / 40-px video box is in real pixels, so the Video texture is slightly coarser at SD.
