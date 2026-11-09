# Streamers — bold rainbow ribbons swinging down the frame

**v0.1.1 built 2026-11-09, NOT flashed** (v0.1.0 built earlier the same day, superseded). Heirloom phase 3 (video
clips), from the still `photos/vid/WIN_20231104_16_39_25_t0p5.png`. That is clip WIN_20231104_16_39_25 at 0.5 s;
the clip is 5.5 s long.

The still shows about seven warm ribbons (magenta / orange / yellow / red / cream) and six cool ones (blue /
green / navy / teal / cyan) running top to bottom:
- straight at the left edge, swinging wider and wider toward the right;
- crossing in hard X junctions where their swings disagree;
- glossy lavender and white highlights behind the red ribbons, and thin pale rims on their leading edges.

Presets are hand-written placeholders.

## v0.1.1 (send-back fix, 2026-11-09)
- **S9 Sync no longer draws a block at the left edge.** In v0.1.0, VCO Q restarted only at red leading edges
  inside the picture. Rows whose first edge fell just before x = 0 ran Q free across the whole first P cycle,
  which drew a hard-edged magenta / green rectangle about 270 virtual px wide. Under Mirror a similar block
  appeared at the centre.
  - Under Sync, Q's phase is now kept at θQ + d × FQ / FP everywhere, where d is the P phase since the red
    leading edge.
  - Each line starts at that value. The line engine computes FQ / FP with a restoring serial divider, plus one
    more product.
  - The first pixel never counts as an edge.
  - Running right, Q restarts at the red leading edge.
  - Under Mirror, the left half runs the other way, so Q restarts there at the ribbon's other (trailing) edge,
    at θQ + FQ / FP. That keeps the halves mirror images.
  - Both restarts carry the detector's 4-pixel lag (± 4 increments).
  - The left edge (and the centre under Mirror) is clean on every row at any Swing, Red and Blue setting.
  - RTL Sync and Sync + Mirror frames are bit-exact with `imodel.py`.
- **K6 Bend default 400 → 280.** A heavily textured camera picture no longer shreds the ribbons. The Osc lattice
  is now patched in at twice the camera's level, so Source = Osc still bends the default picture boldly.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_streamers/`; `notes.md` has the steps.
- **Two oscillators, not one colouriser.** The channel planes show it:
  - R is a clean, soft square wave.
  - G is a sawtooth with two teeth per cycle. Alternate teeth start at about 10 and 55 of 255.
  - B is a clipped triangle of the same cycle as G.
  - Autocorrelation periods, for rows 20-120 / 300-400 / 900-1000:

    | Channel | Period (px) |
    |---|---|
    | R | 298 / 268 / 284 |
    | G | 578 / 617 / 672 |
    | B | 578 / 605 / 642 |

  - So VCO P drives R, and VCO Q drives G and B. Their frequencies differ and are modulated differently, which
    is why warm and cool ribbons cross.
- **hsync-reset FM.** Every edge position scales linearly in x about a fixed point. So each VCO is reset by
  hsync, and its frequency is set per line.
  - The reset points, where the phase is constant across lines, are x0 = -148 px for P and -196 px for Q. They
    come from a grid search on label purity.
  - The per-row periods fit `F(y) = f (1 + ramp yc + A sin(2 pi y / 560 + a) + B sin(2 pi y / 760 + b))`, with
    0.37 % rms error for P and 0.25 % for Q:

    | VCO | f | Ramp | A (phase) | B (phase) |
    |---|---|---|---|---|
    | P | 1 / 284.2 px | -1.3 % | 2.93 % (-117 deg) | 3.20 % (-116 deg) |
    | Q | 1 / 643.7 px | -21.4 % | 3.59 % (+120 deg) | 3.23 % (-12 deg) |

- **Colour is a function of (phase P, phase Q).** A joint 32 × 128 table explains 96 % of the variance in R,
  94 % in G and 87 % in B. The program uses a lighter form of the same table, each line giving R / G / B
  variance explained:
  - Two palettes indexed by phase Q (256 bins): T_HI where the P square is high, T_LO where it is low.
    93 / 91 / 78 %.
  - Plus a gloss table. It adds the lavender highlight (first tooth) and the pale-white highlight (second
    tooth) over the first quarter cycle after each red ribbon's trailing edge. 94.5 / 92 / 80.5 %.
- **Square.** R is high from P phase 0.535 for 0.49 of a cycle.
- **Texture.**
  - Luma edges go 10-90 % in about 6 px, and chroma edges in about 16 px. The camera's chroma trails its luma
    by about 3 px.
  - Thin pale rims, 4-6 px wide, sit on the warm ribbons' leading edges.
  - There is no tearing and no comb. A faint scan-line texture comes from the webcam.
- **Default frame vs the still** (integer model, 1080p):
  - Mean saturation 0.792 vs 0.793.
  - The 4 × 4 region means agree within about 5-25 levels.
  - Mean absolute pixel difference is 26.6, from edge misregistration of the swings.

## Motion in the clip
- **Measurement.** 42 frames (every 4th) were measured the same way (`curves_t.pkl`, `kymo_P.png`,
  `kymo_Q.png`). Both period curves travel DOWN the frame.
- **Joint fit** over all frames, with LFOs shared by both VCOs:
  - LFO A: 560 lines, one cycle per 1.08 s (518 lines/s, about 17 lines per frame).
  - LFO B: 740 lines, one cycle per 4.35 s.
  - At t = 0.5 s, the time fit's P phases agree with the still's fit to about 20 deg.
- **What moves.** The left edge never moves (hsync reset). The right half swings, and the crossings slide
  continuously; consecutive frames differ by about 7.6 RGB on average. Nothing tears, and no knob is turned in
  this 5.5 s clip.
- **The program.** At the default Speed (768), LFO A advances 0.0154 cycles per field, one cycle per 1.08 s at
  60 Hz. LFO B moves at 1.08 / 4.35 of that. Both travel downward. The RTL was checked moving the right way: a
  capture 12 fields later shows the swings shifted down.
- **Ron's knobs on the same patch** (other clips). These set the control ranges: K1 / K2 density, K3 wavelength,
  P12 depth.
  - WIN_20231104_16_34_08 at 0.5 s: both VCOs about twice as dense, with deeper FM.
  - The same clip at 28.0 s: about four times as dense, one long S per frame height, many small eyes.
  - WIN_20231104_16_42_14 at 6.5 s: about seven ribbons, with shifted colours.
  - WIN_20230219_01_18_20 is a different patch (horizontal waves with tearing) and was not used.

## What it does
1. **Per field:**
   - the VCO base frequencies (K1, K2, exponential);
   - the four LFO amplitudes (P12);
   - the LFO steps per line (K3);
   - the LFO times (K4);
   - the red share (K5);
   - the bend gain (K6, doubled for Osc).
2. **Per line**, computed for the next line:
   - four LFO sines (or Zigzag triangles);
   - m = ramp + A sin + B sin, for each VCO;
   - F = G (1 + m);
   - the increment F × xs, and the start phase F × D, where D is the distance from the reset point to the first
     pixel;
   - for Sync, FQ / FP, Q's start value and its restart values.
3. **Per pixel:**
   - Two phase DDAs run. Under Mirror they count down to the centre pixel and up after it.
   - The camera (or Osc) luma × Bend is added to both phases; Q gets half.
   - The P square picks T_HI, or T_LO + gloss, by phase Q.
   - A pale rim covers the first 6 virtual px of each red ribbon.
4. **Output:**
   - U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes, Y through one. This gives the still's soft chroma and
     about 6 px luma edges, and is also the HD HDMI bar fix.
   - Luma is scaled 13/16, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Red | VCO P frequency: the warm ribbons' count, ×1/3 .. ×3 (512 = the still's 6.8 across) |
| K2 Blue | VCO Q frequency: the cool ribbons' count and the colour order along the warm ones, ×1/3 .. ×3 (512 = the still) |
| K3 Wave | the swings' wavelength: 0.5 .. 7.7 swings per frame height (512 = the still's 1.9); LFO B follows at 760/560 |
| K4 Speed | bipolar (±8 deadband, square law): the swings travel down (right) or up (left). 768 = the clip, one wave per 1.08 s; full = 3.8 waves/s |
| K5 Width | the red ribbons' share of each P cycle, 9 % .. 89 % (512 = the still's 49 %): thin red threads .. red dominating |
| K6 Bend | how far the camera bends both VCO phases: 0 .. 3 cycles of P per full luma, square law (280 = 0.22). The Osc lattice enters at twice that level |
| S7 Mirror | both VCOs reset at the screen centre instead of the left edge: a symmetric pattern, straight in the middle and swinging at both sides |
| S8 Cables | RGB / GBR: the colour cables rotated (R ← G, G ← B, B ← R), giving green, violet and yellow ribbons |
| S9 Sync | Free / Sync: VCO Q hard-synced to every red ribbon's leading edge, so each ribbon carries one regular colour sequence (K2 sets how far it runs); continued from the last off-screen edge at each line start |
| S10 Shape | Sine / Zigzag: triangle LFOs, giving zigzag ribbons with sharp corners |
| S11 Source | Video / Osc |
| P12 Swing | FM depth, D = 0.25 + 7.75 (p/1023)²: gently swaying straight ribbons (0 %) .. the still (318, D = 1) .. ribbons whipped into horizontal braids (100 %, D = 8) |

## Video
- **Video (the default).** Camera luma, taken as (Y >> 2) - 16 from a one-line buffer, is added to both VCO
  phases; Q gets half.
  - In patch terms, the camera is plugged into both oscillators' phase inputs. The ribbons bend and fray around
    the picture's shapes, and the colours shift with them.
  - At the default Bend (280) a textured picture still reads as ribbons.
  - With no input (black), Video mode gives the still exactly.
- **Osc.** A drifting triangle lattice, 640 × 540 virtual px, feeds the same path at twice the camera's level.
  At the default Bend the ribbons zigzag across the lattice and drift with it. K6 scales both.

## Engine
- **Per-field sequencer** (vblank zone: no avid for 1024 clocks):
  - One serial shift-add multiplier with 19 slots, including the bend gain. Each slot is decoded two clocks
    ahead with pre-muxed operands, as in lozenge's shell.
  - The shared glide for K1, K2, K3, K5, K6 and P12.
  - A 256-entry exp2 ROM for the exponential controls.
  - The LFO times advance at the speed products' read-out.
- **Per-line engine.** It is kicked at each line start, and once in the zone for line 0. It computes the NEXT
  line in about 160 clocks on its own serial multiplier, which fits SD's 858-clock line:
  - four sine lookups (1024 × 12 ROM), taken just before each product, or the Zigzag triangles;
  - two sign-magnitude amplitude products per VCO;
  - G × |m| and F × D;
  - the Sync ratio FQ / FP: a 24-step restoring division with a registered shift-in and one trial subtraction
    per step;
  - the Sync start product d × FQ / FP, and the up / down restart values.

  The LFO phases and ramps are per-line DDAs, reloaded each field.
- **Pixel path (21 stages):**
  1. The two phase DDAs (one 25-bit adder each does ±increment), with the Mirror flag. The Sync restart detector
     runs on the unbent P phase, sees edges in scan order, and never fires on the first pixel.
  2. Stages 2-4: luma from the line buffer.
  3. Stage 5: the bend nibble products.
  4. Stage 6: their sum.
  5. Stage 7: the bent phases, P bits 23..8 and Q bits 23..14.
  6. Stage 8: offsets from the red edges, and the palette address.
  7. Stage 9: the red bit, the gloss envelope, the rim bit. The palette EBRs are read when their data is needed:
     T_GL at 9, T_LO at 10, T_HI at 11.
  8. Stage 10: the gloss fabric copy.
  9. Stage 11: the gloss products.
  10. Stage 12: T_LO + gloss / 8.
  11. Stage 13: select and clamp.
  12. Stage 14: the rim blend.
  13. Stage 15: black off the generated avid.
  14. Stages 16-19: the output boxes. Their x(n - 8) taps come from one 256 × 64 block-RAM ring, read 6
      addresses behind the write, with `no_rw_check`.
  15. Stage 20: luma 13/16.
  16. Stage 21: the gate.
- **Block RAM (24 EBR):**
  - palettes T_LO / T_HI / T_GL, 3 × 512 × 32 (page 1 = Cables);
  - sine 1024 × 12;
  - exp2 256 × 16;
  - the 2048 × 8 line buffer;
  - the filter ring.
- **Contracts:**
  - damier's generated avid;
  - one latency, C_LAT 20, in every mode;
  - the output boxes' running sums resync to black off the generated avid;
  - virtual 1080-line coordinates;
  - colours in the hardware convention (U = Cr).
- **Verified.** GHDL renders at 720p60, decimation 1, default knobs, were compared with `imodel.py`, counting
  differing pixels in the interior after the tester's 4-px raster offset:
  - Video with black input: 0 differing pixels.
  - Sync: 0.
  - Sync + Mirror: 0.

  The Osc and camera renders look right; the model's Osc lattice is an approximation, so it isn't bit-exact. In
  v0.1.0, the NTSC render and a 1080p capture taken 12 fields later (moving downward) were also checked.

## Status
- **v0.1.1 built 2026-11-09 with ROUTER1, NOT flashed.**
  - **Build seeds: HD HDMI = seed 2** (seed 1 missed). The other five are seed 1.
  - Post-route Fmax:

    | Config | Fmax (MHz) |
    |---|---|
    | HD Analog | 78.25 |
    | HD HDMI | 79.32 (seed 2) |
    | HD Dual | 76.64 |
    | SD Analog | 76.64 |
    | SD HDMI | 74.44 |
    | SD Dual | 69.37 |

  - 6,946-6,959 / 7,680 LC (91 %), 24 / 32 EBR.
  - Every bitstream passed iceunpack, and the packed config carries the labels and presets.
  - Output: `out/rev_b/ron/heirloom/streamers.vmprog` (493,263 bytes).
- **Rebuild:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh streamers`, or
  `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh streamers`. dupin reported 0 `CARRY I0==I1` on the
  standalone synth of the near-final netlist (v011c); router1 completed without stalls on the final build.
- **v0.1.0**, superseded: router1, all six on seed 1, HD Analog 80.89 / HD HDMI 80.80 / HD Dual 79.41 MHz, 6,685
  LC.
- **Fit and timing history.**
  - The first v0.1.0 build came to 7,495 LC. It was cut to 6,685 LC by three changes:
    - moving the output boxes' delay lines into the block-RAM ring;
    - narrowing the phase pipeline;
    - reading the sines just in time and using one add/subtract DDA.
  - The v0.1.1 Sync fix costs about 270 LC.
  - The first version of the divider had a multiplexed shift-in bit. It became the critical path and failed HD
    Analog on seeds 1-2 at about 67 MHz. Registering the shift-in fixed it.
  - The T_LO / T_HI reads were retimed to drop their fabric delay chains.
- **Watch on hardware:**
  - HD HDMI bars (crisp moving edges; the conditioning is in place);
  - the HD HDMI margin, which needed seed 2.

## Deviations from the photo
- **Glossy eyes.** They are modelled as a phase-Q gloss table that fades behind each red ribbon's trailing edge.
  The still's eyes are more distinct lens shapes with darker dark-green cores. The program gives softer, more
  streak-like lavender and white highlights in places.
- **Whites.** They come out at about 777 output luma, the shell's 13/16 white cap, so the cream ribbons read
  light beige-grey. The 95th-percentile luma is 207 against the still's 222.
- **Luma vs chroma.** Luma takes one box and chroma two, so luma leads chroma by 4 px. The still's camera chroma
  trails by about 3 px, so this matches it.
- **Left edge and capture texture.** The left edge fades in over a few pixels (filter warm-up). Not drawn: the
  capture borders (the blue strip at x < 15, the black bottom lines), the webcam scan-line texture, and the faint
  dark arcs near the bottom.
- **LFO B on Q.** LFO B travels for Q as well as P, which keeps the patch model simple. The clip's time fit puts
  most of Q's motion in LFO A.
- **Speed in 50 Hz modes.** The motion steps are per field, so 50 Hz modes move about 17 % slower.
- **Resolutions.**
  - 720p shows the same picture as 1080p.
  - SD shows the centre 4:3, so the straightest left 240 virtual px fall outside the screen.
  - Each SD field draws the whole picture (field_n is tied high on hardware).
- **Sync details.** Under Sync, VCO Q's restarts land 4 pixels after each edge, with those 4 increments added
  back. Mirror folds can show a sliver a few pixels wide at the centre, where P's phase dips across the red edge;
  this is the real shape of the fold.
- **Nothing visible was cut for budget.**
