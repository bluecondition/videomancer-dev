# Seaweed — wavy green curtain bands swaying in a travelling wave, with brown cores, violet lines and black pebble blobs

**v0.1.0 built 2026-11-10, not flashed.** Heirloom phase 3 (video clips), from the still
`photos/vid/WIN_20230402_23_03_06_t0p5.png`: clip WIN_20230402_23_03_06 at 0.5 s (the clip is 9.1 s long). The
alternative look is the same patch in WIN_20230402_23_06_55 (16.5 s). Presets are hand-written placeholders.

The still shows about nine and a half wavy vertical bands of green, mint and lavender:
- they are nearly straight at the left edge and swing harder toward the right, folding into "<" chevrons at the
  right edge;
- each band is split by a dark core: brown with a blue edge in the lower two thirds, or a thin violet line;
- magenta streaks run just below the blobs, and hard black pebble blobs line the top third.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_seaweed/`; `notes.md` has the steps.
- **One hsync-reset VCO, FM'd per line.** Nine dark-core contours (G minima, tracked down the frame) fit
  x = x0 + (X − x0) · b(y) with an rms error of 2.8 px.
  - The reset point is x0 = −155 px. The band period per row is 204.0 · (1.0097 + 0.0971 sin(2π y / 562 − 0.642)) px,
    fitted with rms 0.0044. That is 186 .. 226 px, or 9.4 bands across.
  - So VCO P is reset by hsync and its frequency is set per line by one sinusoidal vertical LFO: 562 lines, 9.6 %
    depth.
- **Unwrapping.** Resampling each row by u = (x − x0) / period(y) straightens every band (`unwrap.png`,
  `cycles.png`). The colour then depends on:
  - the band phase fu = frac(u);
  - the band's parity (a sub-octave: green → yellow-green on odd bands, mint → lavender on even bands);
  - a slow key level that differs per band.
- **Odd bands** carry a brown core, 0 .. 0.46 cycle wide. It is black at its sharp left edge and ramps to brown and
  orange toward the right, with a blue line and a pink fringe on its right edge. A teardrop top ends in a cyan fleck.
  Near the top of the frame the core is absent, leaving only a thin orange line.
- **Even bands** carry a violet line (fu 0.31 – 0.46) with a bright blue edge. Where the key rises, the line turns
  magenta, and at the top it becomes a hard black pebble, 0.15 – 0.5 cycle wide, 200 – 500 lines tall, with a thin
  red-orange rim.
- **The key flips between parities down the frame.** That is a half-rate oscillator Q, q = u / 2 + ψ, with ψ a
  vertical ramp. A 2-D (P, Q) colour table explains 66.5 % of the variance; smoothed at 64 × 32 bins, 75.4 %. The
  states also repeat every four bands (bands 1 and 5 alike, 3 and 7, 2 and 6, 4 and 8), so a quarter-rate term
  is added.
- **Palette** (k-means, 12 clusters), with pixel shares:

  | Colour | RGB | Share |
  |---|---|---|
  | black-brown | (31, 23, 13) | 13.8 % |
  | orange | (234, 151, 72) | 13.3 % |
  | green | (14, 255, 152) | 11.5 % |
  | green | (120, 223, 113) | 10.5 % |
  | yellow-green | (179, 192, 93) | 10.0 % |
  | green | (65, 246, 151) | 10.0 % |
  | brown | (141, 82, 35) | 9.6 % |
  | mint | (124, 221, 188) | 8.2 % |
  | lavender | (194, 187, 204) | 4.6 % |
  | violet | (116, 66, 127) | 3.1 % |
  | blue | (77, 39, 239) | 3.0 % |
  | magenta | (216, 71, 172) | 2.4 % |

- **Texture.** Edges are soft (a webcam filming a monitor): about 6 – 10 px for luma, more for chroma. There is no
  tearing and no comb.
- **Default frame vs the still** (integer model, 1080p):

  | Measure | Still | Model |
  |---|---|---|
  | Mean RGB | (117, 158, 108) | (114, 160, 110) |
  | Luma p5 / p50 / p95 | 8 / 173 / 191 | 6 / 174 / 190 |
  | Black share (luma < 20) | 6.2 % | 6.1 % |
  | Mean saturation | 0.62 | 0.60 |

  The 4 × 4 region means agree within a median of 14 levels (41 at worst).

## Motion in the clip
- **Measurement.** Every second frame of the clip was unwrapped with a free LFO phase, keeping the phase that made
  the bands straightest (`motion.py`).
  - The LFO phase falls linearly at −2.825 rad/s, so the swings travel **down** at 562 lines / 2.224 s = 253
    lines/s (about 4.2 lines per 60 Hz field).
  - The depth (9.7 %) and the mean period hold over the whole 9 s. No knob is turned in either clip
    (`contact_a.png`, `contact_b.png`).
  - At t = 0.5 s the time fit's phase (−0.504) agrees with the still's (−0.642) to within 8°.
- **In phase space** (`cyc_t.png`), the core and blob pattern is nearly static on the left and morphs on the right,
  where blobs split and merge at bands 7 – 8. Q follows the LFO with a slightly different depth.
- **The program.**
  - At the default Speed (768), the LFO advances 0.0075 cycle per field, one wave per 2.22 s at 60 Hz, travelling
    down.
  - Q's rate carries 1/6 of P's FM (es = m/8 + m/32), so the right-hand blobs and cores morph as the wave passes.
    The left side stays steady, as in the clip.
  - The blobs stay along the top because the blob key adds a vertical ramp.
  - GHDL frames at field 3 and field 15 are both bit-exact with the integer model, and the swings move down
    between them (`cmp_motion_rtl.png`).

## What it does
1. **Per field:**
   - the band frequency F0 (K1, exponential) and the LFO step per line (K2, exponential);
   - the LFO time (K3) and the FM depth D (P12, square law);
   - the blob threshold (K4) and the core offset and width (K5);
   - the bend gain (K6, ×4 for Osc).
2. **Per line**, computed for the next line:
   - the LFO sine, interpolated between 1024-entry ROM samples;
   - m = D sin, F = F0 (1 − m), the increment F · xs, and the start phase F · dist (dist = reset point to the first
     pixel);
   - the vertical phase step |dF| · (dist + n · xs), with dF smoothed 1/8 per line;
   - Q's start q0 = 0.165 + ψ(y) + 0.33 es and rate 1/2 + es;
   - the four quarter-rate levels R4[j] = 0.35 cos(2π((j + 0.33) / 4 + ψR(y)));
   - the blob and magenta key offsets, vr(y) − t2 and vr(y) − t2 + 1.
3. **Per pixel:**
   - Band phase u (7.24 fixed point) plus the camera or Osc luma × Bend. Its integer part is the band number k
     (parity = k(0)); its fraction is fu.
   - z = cos(2π(q0 + k / 2 + k es)) + R4[k mod 4]. The cosine is a parabolic 1024 × 16 ROM, interpolated over 6 more
     phase bits.
   - Odd bands: a core of width wo · sqrt(r (2 − r)), r = clamp(z + zoff). Its colour comes from the distance to
     its right edge (black → brown → orange), with a 0.03-cycle orange lead-in, a cyan tip while it is narrower
     than 0.05 cycle, and a blue / lavender / pink edge after it.
   - Even bands: a blob of half-width 0.27 · sqrt(r (2 − r)), r = clamp(2 (z + vr − t2)), around fu = 0.33, with a
     0.012-cycle rim. Below the blob threshold the violet line turns magenta in 8 steps, and the band always has
     its blue / violet-magenta edge.
   - Everywhere else, the 2-band ground palette: page (Colours) | parity | fu, 128 entries per band.
   - **No single-row lines.** 1 – 2 px features (rims, the cyan tip) are dropped where the band phase moves more
     than 0.006 cycle per line, and the 6 – 8 px edge lines where it moves more than 0.016. In those places a
     feature that thin would be a single-row horizontal line.
4. **Output:**
   - U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes, Y through one. This gives the still's soft edges and is
     the HD HDMI bar fix.
   - Luma is scaled 13/16; the palette luma is pre-scaled 16/13 and capped at 940.
   - Then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bands | VCO P's frequency, the band count: ×1/3 .. ×3 (512 = the still's 9.4 across). 0 = three fat bands; 1023 = about 28 fine ones |
| K2 Wave | the LFO wavelength, the swings down the frame: 0.48 .. 7.7 per frame height (512 = the still's 1.92) |
| K3 Speed | bipolar (±8 deadband, square law): the swings travel down (right) or up (left). 768 = the clip, one wave per 2.22 s; full = 4.1× |
| K4 Blobs | the black blobs' key threshold: none (0) .. the still (512) .. black pools down most of the even bands (1023) |
| K5 Cores | the brown cores: thin threads, width 0.06 cycle (0) .. the still (512) .. fat dark cores 0.7 cycle wide in every odd band (1023) |
| K6 Bend | how far the camera (or the Osc, at four times the level) bends the band phase: 0 .. 3 cycles per full luma, square law (150 = 0.15) |
| S7 Mirror | VCO P reset at the screen centre instead of the left edge: a symmetric curtain, straight in the middle and swinging at both sides |
| S8 Pools | Ink (the still's black blobs) / Glow (lit yellow pools with an orange rim) |
| S9 Flip | the vertical ramps inverted: blobs along the bottom, cores reaching down from the top ("hanging" kelp) |
| S10 Colours | Kelp (the still) / Coral (chroma rotated 160°: coral pink and lavender bands, navy cores, amber lines; the blobs stay black) |
| S11 Source | Video / Osc |
| P12 Sway | FM depth D = 0.02 + 0.38 (p / 1023)²: a straight, calm curtain (0 %) .. the still (458) .. whipped braids folding into horizontal chevrons (100 %, D = 0.40) |

## Video
- **Video (the default).** Camera luma, taken as (Y >> 2) − 16 from a line buffer, is added to VCO P's phase. In
  patch terms, the camera is plugged into the band oscillator's phase input.
  - The bands bend and fray around the picture. Blobs and cores follow, because they are keyed on the bent band
    number.
  - At the default Bend (150, 0.15 cycle per full luma) the curtain still reads clearly with `photos/selfie.jpg`,
    and at full Bend it shreds into the picture.
  - With no input (black), Video mode gives the still exactly.
- **Osc.** A drifting triangle lattice of 160 × 135 virtual px (about 20 px/s) feeds the same path at four times
  the camera's level. At the default Bend every band and blob zigzags. K6 scales it.

## Engine
- **Per-field sequencer** (vblank zone: no avid for 1024 clocks):
  - one serial shift-add multiplier, 16 slots, each decoded two clocks ahead with pre-muxed operands (lozenge's
    shell);
  - the shared glide for K1, K2, K4, K5, K6 and P12;
  - a 256-entry exp2 ROM for the exponential controls.
- **Per-line engine.** It is kicked at each line start, and twice in the zone: a pre-run for line −1, whose F seeds
  line 0's vertical step, then line 0. It computes the NEXT line in about 110 clocks on its own serial multiplier,
  which fits SD's 858-clock line:
  - the interpolated sine;
  - D · |s|, F0 · |m| and F · dist;
  - |dF| · dist;
  - four R4 lookups.

  The LFO phase, ψ, ψR and the blob ramp are per-line DDAs, reloaded each field.
- **Pixel path (33 stages, C_LAT 32):**
  1. The phase DDA and its vertical-step twin, both reversing at the centre for Mirror.
  2. Stage 2: the steep flags.
  3. Stages 4 – 6: luma from the line buffer and the bend nibble products.
  4. Stage 7: the bent phase (bits 30..14).
  5. Stages 8 – 10: k (es >> 5) and q.
  6. Stages 11 – 15: the key ROM, its interpolation, + R4.
  7. Stages 16 – 17: the key offsets and clamps, and the sqrt ROM address.
  8. Stages 18 – 20: the sqrt ROM and the width product.
  9. Stages 21 – 22: the edge distances and the feature index.
  10. Stages 23 – 26: the feature and ground ROMs and the select.
  11. Stage 27: black off the generated avid.
  12. Stages 28 – 31: the output boxes (their x(n − 8) taps from one 256 × 64 block-RAM ring with `no_rw_check`;
      its 14 spare bits carry fu and the steep flags from stage 10 to stage 18).
  13. Stage 32: luma 13/16.
  14. Stage 33: the gate.
- **Block RAM (23 EBR):**
  - sine 1024 × 12;
  - exp2 256 × 16;
  - key cosine 1024 × 16;
  - sqrt 512 × 8;
  - ground palette 512 × 32;
  - features 256 × 32;
  - the line buffer 2048 × 8 (`no_rw_check`: the beam column trails the write column by 4 – 5 px);
  - the output ring.
- **Contracts:**
  - damier's generated avid;
  - one latency, C_LAT 32, in every mode;
  - the output boxes' running sums resync to black off the generated avid;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr), from `seaweed.py`.
- **Verified** (GHDL, 720p60, decimation 1, counting differing pixels in the interior after the tester's 4-px raster
  offset):
  - Video with black input, Speed still: 0 differing pixels against `imodel.py`;
  - default Speed at field 3: 0 differing pixels;
  - a capture 12 fields later (field 15): 0 differing pixels.

  The Osc render matches the model's approximate lattice to within a mean of 6 codes. The camera, Osc and NTSC
  renders look right.
- **Single-row lines.** `thincheck.py` counts 1-row peaks and dips of 24 px or more. It finds 0 at the defaults,
  P12 0 and 100 %, Wave max, and at P12 100 % with Flip, Mirror, Blobs max or Cores max.

## Status
- **v0.1.0 built 2026-11-10 with ROUTER1, NOT flashed.**
  - **Build seeds: HD Dual = seed 2** (seed 1 stalled the router). The other five are seed 1.
  - Post-route Fmax:

    | Config | Seed | Fmax (MHz) |
    |---|---|---|
    | HD Analog | 1 | 85.59 |
    | HD HDMI | 1 | 84.53 |
    | HD Dual | 2 | 84.03 |
    | SD Analog | 1 | 87.70 |
    | SD HDMI | 1 | 86.25 |
    | SD Dual | 1 | 87.70 |

  - 7,067 – 7,091 / 7,680 LC (92 %), 23 / 32 EBR.
  - All six bitstreams pass iceunpack, and the packed config carries the labels and presets.
  - Output: `out/rev_b/ron/heirloom/seaweed.vmprog` (495,277 bytes).
- **Rebuild:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh seaweed`.
  dupin found 0 `CARRY I0==I1` on the standalone synth.
- **router2 also closes this netlist, with thinner margins.** On a machine loaded with five GHDL sims it needed
  HD Analog seed 2 (82.21 MHz), HD HDMI seed 2 (75.53) and HD Dual seed 3 (78.19).
- **Fit history.**
  - The first build came to 7,762 LC (101 %).
  - Cut to 7,005 by:
    - adding the bend on phase bits 30..14 only;
    - carrying fu and the steep flags in the output ring's spare bits instead of FFs;
    - replacing the parabola multiplier by a ROM;
    - an unsigned vertical-step DDA and narrower field steps.
  - That 7,005-LC version closed all six configs on seed 1 with router2 (HD Analog 87.97, HD HDMI 80.27, HD Dual
    75.39 MHz), but its 512-step key quantisation stepped the blob edges into 1-2-row slivers.
  - The final version adds the interpolated 1024 × 16 key ROM and the smoothed vertical step, and reclaims LC with
    a reduced k × es product and `no_rw_check` on the line buffer: 7,067 LC.
- **Watch on hardware:**
  - HD HDMI bars (crisp moving edges; the conditioning is in place);
  - if HD Dual stalls on a rebuild, it closed on seed 2.

## Deviations
- **Blobs.** They are crescents that follow the band curvature. The still's are fatter, rounder pebbles, and its
  S-shaped double blob at the top right is not reproduced.
- **Per-band variety.** The core and blob pattern comes from a half-rate key oscillator plus a quarter-rate term,
  so it is more regular than the still's: cores start at similar heights in alternate bands.
- **Palette shares.** Orange is 9 % against the still's 13.5 %, and saturated green (65, 246, 151) is 18 %
  against 10 %. Mint (124, 221, 188) is under-represented.
- **Fine detail.**
  - The still's thin cyan flecks appear only as a cyan tip on narrow core ends.
  - The webcam softness is approximated by the shell's output filter: luma 9-tap, chroma 16-tap.
  - Whites come out at about 777, the 13/16 cap.
- **Extreme settings.** With Wave and Sway both at maximum, the braids alias into scan-line streaks (the
  vertical-step guard only drops thin features). Bands max with Sway max leaves 11 – 21 short 1-row runs of up to
  29 px.
- **Default bend.** The default camera bend (150) is mild: with the selfie the camera mostly frays the band edges.
  Full Bend shreds into the picture.
- **Speed in 50 Hz modes.** The motion steps are per field, so 50 Hz modes move about 17 % slower.
- **Resolutions.**
  - 720p shows the same picture as 1080p.
  - SD shows the centre 4:3, so the straightest left 240 virtual px fall outside the screen.
  - SD's 4.4 virtual lines per row put small stair steps on near-horizontal band edges.
- **Not drawn:** the capture border (1 – 3 px). The left edge fades in over a few pixels (filter warm-up).
- **Nothing visible was cut for budget.**
