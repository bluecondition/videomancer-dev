# Plumage — a fan of curved violet/magenta feathers over a warm ground with a cool wipe

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom phase 3 (video clips), from the clip
`WIN_20230710_00_15_14` at 1.5 s (`photos/vid/WIN_20230710_00_15_14_t1p5.png`). A fan of curved feathers sweeps
up and right from a root at the lower left: near-vertical at the left, about 30° at the right, each feather with a
crisp dark leading edge and banded violet / purple / olive / magenta / pink shading that fades into the ground. A row
of short spikes runs along the bottom. The ground is warm (yellow → orange → crimson, left to right) inside a band
that scrolls down through a cool one (green → teal → navy). A green → black root strip runs down the far left, and
torn streaks cover the top left. Presets are hand-written; three of them are the clip's other looks.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_plumage/` (`notes.md` has every number).
- **Feather edges.** The edge map gave 18 traced edge curves, 11 crisp leading edges and 7 trailing edges
  (`epts.py`, `curves.py`). Several field families fit the edge tangents about equally well (0.155 rad rms):
  power curves, log spirals, and parallel log curves φ = y + 930 ln(x + 140). The parallel log curves were chosen
  (0.157 rad): one log per pixel, and the curvature is a single constant K = 930.
- **Crowding.** The lanes crowd toward the lower right: the leading edges sit at φ = 6351, 6900, 7325, 7570 and 7814.
  The second log u = −ln(8545 − φ) / 0.28 spaces them evenly, at 0.288, 0.298, 0.225 and 0.288 of a lane (sd 0.03).
  PI = 8545 = 368 + 1080 + 930 ln 2060, which puts the crowding point just past the bottom-right corner.
  - Crisp edges sit at a lane fraction of 0.5 ± 0.15.
  - Blades fill 0.33–0.6 of a lane, about 9 long feathers in the frame.
- **Spikes.** Their crisp edges run at 33° at x 370 and 24° at x 730. That is a flatter family of the same curves,
  y + 0.375 K ln(x + 140). The tips sit in about 3 rows, 170 lines apart, with a tooth about every 650 px along a row.
- **Feather palette** (across a blade, from the crisp edge t = 0 to the trailing edge t = 1):
  (55,0,60) → (62,0,95) → (78,0,172) → (95,0,182) violet → (118,5,140) → (118,14,95) → (92,54,64) olive →
  (108,48,80) → (150,24,150) → (190,2,200) magenta → (214,10,186) → (240,60,160) pink fringe. A soft blend into the
  ground follows.
- **Ground.** It varies with x only (`bgfit.py`).
  - Warm: Y 777 → 358, U(Cr) 599 → 926, V(Cb) 218 → 479.
  - Cool: Y 530 → 144, U 300 → 466, V 371 → 700.
  - The warm band spans rows top(x)..950. The top is lumpy: green at x 470–740 above y 230, teal at x 1150–1300
    above y 150, navy at x > 1450 above y 260.
- **Root strip.** Bright green from x 15 to 90, fading to black at x 100–180.
- **Texture.**
  - Torn streaks: per-line horizontal offsets in the top left (x < 800, y < 750).
  - The warm/cool boundary is streaked by a per-line vertical jitter: yellow/green streaks at the top left, thin cyan
    lines across the orange at the top middle, and red/navy speckle at the top right.
  - The central and right blades are clean.

## Motion in the clip
The whole 10.5 s clip was re-extracted at 10 fps (`seq3/`, `warm.py`, `track.py`).
- **The warm band is a vertical wipe scrolling down.** The warm fraction peaks at 1.9 s and 7.1 s and is zero from
  3.8 to 5.4 s, a period of 5.3 s. The band's lower edge moves about 420 px/s and its upper edge about 200 px/s:
  about 300 px/s over a 1600-line period.
- **Hooked tips stream left** (inward, toward the root) at about 180 px/s. The leftmost dark hook moved from x 1236
  at 1.3 s to x 876 at 3.5 s.
- **The long fan on the left stays put** for the whole clip.
- **In the cool phase** (3–5.5 s), rows of hooked teeth fill the upper right over green, teal and navy.
- **What the program does with it.** At the default Speed (772) and 60 fields/s:
  - the band scrolls down 4.9 lines per field, about 295 lines/s, a 5.4 s cycle;
  - the feather segments and the teeth stream toward the root at about 189 and 184 px/s.

  The sky therefore cycles warm → cool → warm the way the clip does, and tips and teeth slide in from the right.
  Rates scale with the field rate (5/6 at 50 Hz).
- **Ron's other looks of this patch:**
  - `00_03_01`: a rainbow colouriser, many fine concentric arcs, and a hexagon in the loop. This is S7 Rainbow with
    more feathers; K6 at full warps the fan into hexagonal cells.
  - `00_13_38`: everything torn into liquid horizontal streaks. This is P12 at full.
  - `00_15_14` at 6.5 s: the still's look with a strong row of teeth. This is K5.

## What it does
1. **KL = 930 ln(x + 140)**, in half-pixel units, with the tear offset applied first. dx is normalized; KL is
   KE[p] + KLN[m] from two tables already multiplied by K, so there is no pixel multiplier.
2. **Z = PI − s·y − KL + bend·luma.** Sweep scales y, which is equivalent to K / s. PI = 368 + 1080 s + 930 ln 2060,
   so the crowding point stays past the bottom-right corner.
3. **u = lane0 − G ln Z.** Z is normalized, and G ln(1 + m/256) and G p ln 2 come from two per-field block-RAM tables.
   frac(u) is the position across the lane (crisp edge at 0), and the lane index picks a segment offset.
4. **Feather.** t = frac(u) · QA'(tap) / 16, where QA' = (1 / width) / taper^0.7, a per-field table.
   - The segment coordinate a = x + stream + OFFA(lane), mod 32768 h.
   - A blade exists on a < 30000, tapered over 1024 h at both ends, so tips slide in from the right and stream to
     the root.
5. **Teeth.** Rows of y·s + 0.375·KL, 170.7 lines apart.
   - Tooth phase = 63·x + stream + OFFB(row): one tooth per 520 px, a rounded nose and a pointed tip (ROM).
   - t = frac · QB(phase) / 16.
   - Only the lowest K5 rows exist; the topmost partial row is narrowed by 1–3 octaves.
6. **Compose.** Solid teeth go over feathers, feathers over the teeth's fringe, and that over the ground. The 128-entry
   palette runs t 0..2; above t = 1 is the soft trailing fringe, which alternates palette and ground by column.
   The luma [1 2 1] and the chroma boxes average it to a half blend.
7. **Ground.** Warm or cool by x (ROM, 32-px steps) in a 1600-line band (2048 band units at 1.28 per line).
   - The band phase per line = y·1.28 − scroll + jitter.
   - The top boundary is lumpy (ROM by x / 8), and both edges are Bayer-dithered over 35 lines.
   - Root strip at x < 190.
8. **Tear.** Per-line offsets from two LFSRs, drawn every second line: ±12 h, plus ±80 h on a fraction of the
   lines. They apply in full left of the region limit, at half over its last 512 h, and fade with y. The vertical
   band jitter is ±13 or ±77 band units.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Feathers | the number of feathers, G ×1/3 .. ×3 (exponential): 0 = three or four broad plumes; 512 = the still's ~9; 1023 = a dense fan of ~27 fine quills |
| K2 Sweep | the feathers' curvature (y scale s = 2^(−0.75 (k − 512)/256), equivalent to K 330 .. 2630): 0 = long flat arcs sweeping right; 512 = the still; 1023 = tall near-vertical plumes |
| K3 Width | the feather width, 0.09 .. 0.99 of a lane (two-slope exponential): 0 = thin quills on open ground; 512 = the still (0.52); 1023 = overlapping plumes that hide the ground |
| K4 Speed | bipolar, still in the centre ±8: the sky wipe, the feather tips and the teeth streaming toward the root (or away when negative). 772 = the clip's rates, 1023 = 4× |
| K5 Teeth | the rows of teeth: 0 = none; 512 = the still's spikes along the bottom right; 1023 = rows of hooked teeth over the whole frame (the clip's cool-phase look) |
| K6 Bend | how hard the camera (or the Osc) bends the feathers and keys the sky: (k/1023)²; 400 = default |
| S7 Colours | Plumage (the still's violet/olive/magenta) / Rainbow (the `00_03_01` look: navy → blue → cyan → green → yellow → red → magenta) |
| S8 Mirror | One Fan / Two Fans: the field folded at the centre, roots at both edges, the plumes meeting in a V |
| S9 Sky | Day / Night: warm and cool swap. The frame goes mostly green/teal/navy with a warm band passing through |
| S10 Root | Below / Above: the fan hangs from the top left and sweeps down |
| S11 Source | Video / Osc |
| P12 Tear | 0 = a pristine fan, clean boundaries; 380 = the still (torn top left, streaked sky boundary); 1023 = the whole picture shredded into horizontal streaks (the `00_13_38` liquid look). Since v0.2.0 the feathers stay intact; the ground, root strip, sky boundary and teeth tear |

**Presets:**
- Still (the defaults);
- Rainbow Arcs (K1 860, K3 760, Rainbow, P12 200);
- Liquid (K4 950, P12 1023);
- Shark Teeth (K5 900, Night);
- Twin Plumes (K1 380, Mirror);
- Quills (K1 820, K3 80);
- Flat Sweep (K2 60, K5 700);
- Hanging Plumes (K2 960, Root Above).

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, times K6² (8×8 multiply,
  nibble partials), enters twice.
  - Added to Z, it bends the feather lanes around bright areas: up to about 146 lines at white at the default.
  - Subtracted from the band phase, it keys the sky: about 230 lines at white, so the picture appears as warm/cool
    shapes in the ground.
  - The fan, the teeth and the root keep the still's composition. With no input, Video mode is the still.
- **Osc mode.** The shell's drifting triangle lattice (640 × 540 virtual px) feeds the same path at twice the gain.
  - The sky breaks into warm/cool lattice cells and the plumes kink along the lattice.
  - It is a clear look-flip at the default Bend.
  - K6 at full gives the hexagonal-cell warp, an echo of the hexagon in `00_03_01`.

## Engine
- **Per-field sequencer** (vblank zone, 1024 clocks without avid):
  - One serial shift-add multiplier, 7 slots, with operands pre-muxed. The line engine shares it outside the slots'
    window.
  - A shared glide (1/8 per field, ±2 deadband) on K1, K2, K3, K5, K6 and P12.
  - A 256 × 16 exp2 ROM.
  - It computes G, 1/width, rows, the sweep scale s (straight from the exp ROM), PI and B0 (one product of
    17280·s), lane0 and speed².
  - The stream, wipe and tooth steps, the tear gain and the tear region limits are shift-adds.
- **Table fill.** From zc 330, on its own serial multiplier, 16 clocks per entry, 403 entries:
  - G ln(1 + m/256) (256 entries);
  - G p ln 2 mod 16 lanes (19 entries);
  - the feather taper × 1/width (128 entries).

  The RAMs power up holding the default tables, and their write enables are one-clock pulses.
- **Line engine.** Kicked at each line start, it computes the NEXT line in about 150 clocks (it fits SD's 858):
  - s·y and the Z and teeth line terms;
  - the tear draw × gain × y-envelope and the band jitter × gain × envelope;
  - the band phase y·1.28 − scroll;
  - two 32-bit Galois LFSRs, stepped 32 times every second line.
- **Pixel path (27 stages):**
  - mirror and tear;
  - dx;
  - normalize, then the KLN ROM;
  - KL;
  - Z;
  - normalize, then the GLN / GE tables;
  - u;
  - lane and row tables;
  - the segment coordinate and the taper RAM;
  - the tooth ROM;
  - two 8 × 12 multiplies (as nibble partials);
  - compose;
  - the palette and ground ROMs;
  - the colour.
- **Rings.** Five block-RAM delay rings, each with a fabric copy after the read:
  - the segment base, stages 4 → 16;
  - the tooth phase base, 4 → 13;
  - the ground mode/index, 8 → 23;
  - the luma, 8 px back so it lines up with the chroma boxes;
  - the syncs and the generated avid, for the first 24 clocks of the latency.

  All carry `no_rw_check`.
- **Output.** U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum resync (both snap to neutral
  after 8 blank samples). Luma goes through [1 2 1], then 13/16, then the blanking gate. C_LAT is 32 in every mode.
- **Contracts:**
  - the generated avid;
  - one latency;
  - virtual 1080-line coordinates (h units, 2 / 3 / 4 h per pixel at 1080 / 720 / SD);
  - colours in the hardware convention (U = Cr), generated by `plumage.py`;
  - luma pre-scaled 16/13 with a soft knee, capped at 940 (drawn whites about 777).
- **Verified.** GHDL at 720p60, decimation 1, black input, against the integer model `imodel.py` at the same field:
  - mean |diff| 2.55, with 90 % of pixels within 8 levels after the tester's 4-px capture offset;
  - the remainder is the random tear and jitter draws and the teeth phase (`rtl_diff.png`).

  `cmp_final.png` = still | proto_final | RTL Video black | RTL Osc | RTL Video camera.
- **Fit history.**
  - The first cut had K × L1 and G × L2 pixel multipliers, s × 1/w × (1 + j/8) × taper, luma boxes and a 40-stage
    pipe: 9104 LC (118 %).
  - K was folded into the log ROM, the sweep became a y scale, and G and the width moved into per-field tables. The
    per-lane width code was dropped, luma went to [1 2 1], the syncs went into a ring, and five sequencer slots
    became shift-adds. Then came the dithered fringe and C_LAT 32, and the line engine moved onto the sequencer's
    multiplier. The result is 7140–7173 LC.

## Status
- **v0.1.0, 2026-11-04, router2** (`MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh plumage`).
- **Build seeds: HD HDMI = seed 2** (seed 1 missed). The other five are seed 1.
- Post-route Fmax: HD Analog 75.05 (thin), SD Analog 79.59, HD HDMI 80.86, SD HDMI 74.53, HD Dual 89.37,
  SD Dual 85.15 MHz.
- 7140–7173 / 7680 LC (93 %), 23 / 32 EBR. iceunpack OK × 6. The packed config carries the labels and presets.
  `dupin.py` reports 0 `CARRY I0==I1`.
- `out/rev_b/ron/heirloom/plumage.vmprog`. **Not flashed.**
- HD Analog's margin is thin. If a rebuild misses, try `NEXTPNR_ROUTER="--router router1"` first, then seeds. A
  test route of a slightly larger netlist gave 78.8 MHz on HD Analog seed 1.

## Deviations from the photo
- **Feathers.** They are regular log-curve lanes with uniform widths. The still's blades vary in width (0.33–0.6 of a
  lane) and in curvature from blade to blade; the per-lane width code was cut for budget. Lane positions match the
  measured crisp edges to about a tenth of a lane in the middle and the right. The leftmost narrow blades
  (x 180–350) are fewer and wider than in the still.
- **Hook.** The central feather's curled tip at (1200–1340, 70–170) is not drawn. Tips are straight tapers.
- **Spikes.** They are drawn as regular rows of teeth with rounded noses (the clip's cool-phase teeth), so there are
  more of them, and more regular, across the lower right than the still's few loose wedges.
- **Ground.** It is two x-ramps with a lumpy top. The still's salmon glow at (900–1250, 550–750) and its blue
  bottom-right corner detail are not drawn.
- **Edges.** Luma edges are crisper than the camera-soft still (luma [1 2 1]; chroma is blurred about 9 px).
- **Brightness.** The brightest yellow comes out at about 777 output luma, the shell's white cap.
- **Tear.** The tear and jitter realizations are random, regenerated every second line each field. The still's
  top-right speckle shows only where the band boundary sits.
- **Left edge.** It fades in from black over about 8 px (filter warm-up). The capture border is not drawn.
- **Resolutions.** 720p shows the same geometry as 1080p. **SD shows the centre 4:3** (virtual x 240..1680), so the
  root strip and the steepest left blades are off-screen at SD, and Mirror's two roots are cut off.
- Nothing visible was cut for budget beyond the per-lane width variation.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. K6 here already was the camera strength, so only its default changed: initial value 400 -> 512 (v0.1.1). The vmprog was repacked from the same bitstreams (verified: only the 150-byte settings block differs), so timing, seeds and LC are as above. Presets keep their own K6 values. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/heirloom/.

## K6 top +25 % and default 100 % (2026-11-15)

Ron (renders of fern and kiln at K6 = 100 %): bend / element programs get a 25 % stronger camera effect at the top of K6; the K6 default is now 100 % (1023, set with the version bump).

- **Class:** BEND: K6 (Bend) adds K6^2 x luma to the feather field Z and keys the sky band.
- **Change:** Gain x1.25 at every position: `c_vg` now holds `1.25 K6^2 / 4096` in half units (Q-1, 0 .. 158, the same 8 bits); the pixel path takes one shift less (`q6_vz = vp >> 1`, widened to 15 bits; `q6_vd = vp(14:4)`, mod 2048 like q7_dd). The Osc keeps twice the camera gain under its old ceiling (127 = the old 255), so the Osc at 100 % is unchanged. 1023 -> 1.24x.
- **Build:** Built with router2: HD Analog seed 2, the rest seed 1; worst HD 78.15 MHz (HD Analog 78.41, HD HDMI 78.15, HD Dual 78.83); max 7203 / 7680 LC.
- **Check:** GHDL 480p render, bear, K6 = 100 %, old vs new (glides snapped in scratch copies): `.heirloom_work/rb_plumage/k6b/k6b_cmp.png`, the same picture with a stronger bend.
- **Router note:** router1 hung on SD Analog (seeds 1 and 2 froze at 6021 remaining arcs, the dup-pin ping-pong; the
  netlist has 6 LUTs with two equal inputs in the knob-exponent / glide logic, no CARRY I0 == I1). Router2, the
  default, routed all six. Rebuild with router2.

## v0.2.0: Ron's preview notes (2026-11-21)

Ron (from hardware, v0.1.2): "is it possible to make the tear knob/effect not apply to the feathers"

**P12 Tear: the feathers now ignore it.** The tear is still one per-line x offset added at pixel stage 3 (`q3_xt`), with
the same draws, gain, region and envelopes as before. The feather path now reads an untorn copy of x instead (`q3_xu`,
the mirrored x registered alongside; one 12-bit register). The untorn x feeds:
- dx, so KL = 930 ln(x + 140), Z and the lane coordinate u (the crisp leading edges and the banded blade shading);
- the feather segment coordinate a (ring 1), so the tips and tapers stream whole.

The feathers therefore draw intact at every Tear position, 1023 included, inside a torn picture. The torn x still
drives everything else, unchanged:
- the ground's warm / cool x ramp and its lumpy top boundary;
- the root strip at the left edge;
- the tooth phase (ring 2).

The vertical band jitter never used x and is unchanged. dx's clamp is gone: x is now never negative (0..3839 h), so dx is
280..4119.

**Judgement calls**
- **Teeth (K5) still tear.** Ron named only the feathers, and the teeth are a separate element on their own knob. Their
  phase along each row is torn as before. Their row coordinate comes from the same KL as the feathers, which is now
  untorn, so torn teeth slide sideways inside fixed rows; before, the row curves also wobbled a little. To make the
  teeth intact too, feed `q4_vb` from `q3_xu` instead of `q3_xt` (one line).
- **P12 is still bold everywhere on its travel, just not on the feathers.**
  - The main visible tear is now the warm/cool boundary shredding into streaks. That is the vertical jitter: up to about
    ±170 lines on the big-draw line pairs at the still (380) and about ±460 at 1023. It is at full strength over the
    top ~290 lines, fades to 1/8 by line ~740 and stays at 1/8 below.
  - The root strip's edge goes jagged and the boundary lumps tear.
  - Above 380 the tear region grows to the whole frame and tears the teeth.
  - The ground itself is a smooth x ramp in 32-px steps, so its horizontal tear reads only as faint streaks in flat
    ground: about ±1 colour step at the still and ±3 at 1023 (sd ±15 / ±100 px at 1080).
- **The "Liquid" preset** (P12 1023) is now intact feathers over a shredded sky and ground, no longer everything
  liquid. The name and values are kept.
- The HW-captured defaults and every other control are unchanged.

**Resolutions.** Unchanged: the tear and the untorn x are both in virtual h units, so 720p shows the same picture as
1080p. SD still shows the centre 4:3, as before.

**Build.** Router2, the program's router (router1 hangs on SD Analog), `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh plumage`:
- **6/6 on seed 1.** HD Analog no longer needs seed 2.
- Post-route Fmax: HD Analog 82.40, SD Analog 75.59, HD HDMI 80.40, SD HDMI 79.22, HD Dual 86.06, SD Dual 72.08 MHz.
- 7158–7198 / 7680 LC (94 %), 23 / 32 EBR. iceunpack OK × 6. The packed config reads v0.2.0.
- Backups of v0.1.2 (vhd, toml, vmprog): `.heirloom_work/rb_plumage/plumage_v012.*`.

**HD HDMI re-seed (the HSYNC placement skew, after the v0.2.0 build).** `.heirloom_work/skew_tools/screen.sh plumage`:
hd_hdmi router1 + router2 s1-60, 102 routed, 85 timing-clean.
- **The builder's HD HDMI router2 s1 had a negative HSYNC** (−0.52 / data ≤ −1.48), the known-bad side.
- **Shipped HD HDMI router2 s5: HSYNC +1.61 / data ≤ −1.57, 80.47 MHz.** This is the most positive seed in the
  +1.0..+2.3 window.
- **Spare: router2 s19** (+1.11 / ≤ −1.68, 85.34 MHz), at
  `.heirloom_work/rb_plumage/skew/plumage_v020_spare_hd_hdmi_router2_s19.vmprog`.
- **HD Analog and HD Dual keep the builder's router2 s1** (re-routed byte-identical). HD Analog is at +1.52 / ≤ −1.20
  and HD Dual at −1.59 / ≤ −1.26; both have data ≤ −0.6.
- **A full builder rebuild re-routes HD HDMI on seed 1**, the negative HSYNC. After any rebuild, re-run the screen
  and `.heirloom_work/skew_tools/install.sh plumage router2 5`, or re-install the equivalent seed.
- Record which of s5 and the s19 spare Ron finds clean.

## v0.2.1: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`512 - k` became `k - 512`); the simulator
is correct, so every render showed the intended picture while the hardware did not.

- **Fixed: line 626** (vblank sequencer, step 25), `v_d := 512 - signed(resize(s_g3(12 downto 3), 12))` ->
  `to_signed(512, 12) - ...`: the K3 Width exponent (two slopes for 1 / width).
- **Fixed: line 643** (step 33), the same form for K2 Sweep: the y-scale exponent `2^(-0.75 (k2 - 512) / 256)`.
- **On hardware before / after.**
  - K3 ran backwards with its two slopes swapped: 0 = overlapping plumes (the gentle 0.94-octave half), 1023 = thin
    quills (the steep 2.5-octave half). Now K3 runs as labelled: 0 = thin quills, 1023 = overlapping plumes.
  - K2 ran backwards: 0 = tall plumes, 1023 = long flat arcs. Now 0 = long flat arcs sweeping right, 1023 = tall
    near-vertical plumes.
  - The defaults (K2 = K3 = 512) are the same picture as before.
- **Preset remaps (1024 - v, exact).** The fixed decode at 1024 - v gives exactly the old hardware `v_d`, so every
  preset looks as it did on hardware: Rainbow Arcs K3 760 -> 264, Quills K3 80 -> 944, Flat Sweep K2 60 -> 964,
  Hanging Plumes K2 960 -> 64. The preset names still describe the sims, not what they show (Quills shows fat plumes,
  Flat Sweep tall plumes, as on hardware before); rename or flip them if Ron wants the names to match.
- **Left as is:** no other audited site in this program.
- **Build** v0.2.1, router2 (the program's router):
  - HD Analog 83.27, SD Analog 76.44, HD HDMI 82.37, SD HDMI 74.48, HD Dual 78.61, SD Dual 90.33 MHz (worst HD 78.61).
  - **HD Dual seed 2** (seed 1 missed timing); the other five on seed 1. HD Dual was then re-seeded anyway (below).
  - 7196-7235 / 7680 LC (94 %), 23 / 32 EBR. iceunpack OK x6.
  - Backups of v0.2.0 (vhd, toml, vmprog): `.heirloom_work/rb_plumage/plumage_v020.*`.
- **Skew** (`QUICK=1 .heirloom_work/skew_tools/screen.sh plumage` -> r1 + r2 s1-40, 68 HD HDMI timing-clean, 3 in the
  window; plus re-routes of the builder's router2 HD Analog s1 / HD Dual s2, byte-identical to the build):
  - Builder HD HDMI router2 s1 was **−0.16 / ≤ −1.48** (negative HSYNC, unsafe).
  - Installed HD HDMI **router2 s3 (+1.18 / ≤ −1.20, 85.75 MHz)**.
  - Builder HD Dual router2 s2 had data at **+0.07**, so it was replaced by **router1 s7 (−1.53 / ≤ −1.25, 92.97 MHz)**.
  - HD Analog router2 s1 (+0.70 / ≤ −1.30) kept.
  - Spare: HD HDMI router2 s4 (+1.07 / ≤ −0.94, 82.49 MHz), at
    `.heirloom_work/rb_plumage/skew/plumage_v021_spare_hd_hdmi_router2_s4.vmprog`.
  - The v0.2.0 screen and its spare moved to `.heirloom_work/rb_plumage/skew_v020/`. The replaced builder bins are in
    `.heirloom_work/rb_plumage/skew/*_replaced_*.bin`.
  - **A full rebuild re-routes seed 1.** After any rebuild re-screen (router2 is not deterministic, so the same
    seeds may not reproduce).
- Not flashed.
