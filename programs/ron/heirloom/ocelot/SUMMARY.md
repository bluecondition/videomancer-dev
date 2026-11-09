# Ocelot — orange zebra chevrons with a black ocelot body of glowing dashes and blue crescents rolling up

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20231227_00_20_47` at 6.0 s
(`photos/vid/WIN_20231227_00_20_47_t6p0.png`). The lower part of the frame is a zebra: orange vertical bands with a
dark left ramp and a yellow right edge over soft salmon / grey-teal horizontal stripes, the bands dipped into "<"
chevrons on two rows and crowding together toward the right. The top part is a black "ocelot" body with scalloped
edges: rows of glowing tan dash triplets with salmon cores, blue / cyan crescents, and red flames where the orange
bands push up into it. In the clip the body rolls up through the zebra, one body every 9.4 s. Presets are
hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_ocelot/` (`notes.md`, `model.py`, `fitphi.py`, `fitstretch.py`,
`fitstretch2.py`, `fitblue.py`, `profile.py`, `proto.py`, `luts.py`, `imodel.py`, `sheet.py`, `thincheck2.py`). A
clean zebra was composited from the clip's frames (`zebra_clean.png`).
- **Palette** (k = 14, whole still):
  - black (8,8,1) 29.9 %, orange (247,123,1) 20.0 %, grey-teal (77,88,70) 10.9 %, salmon (250,81,58) 10.0 %;
  - tan (169,113,65) 6.0 %, dark reds (50,10,1) / (103,23,1) / (140,78,16) 12.2 %, red (224,12,0) 2.4 %;
  - blues (1,12,208) / (1,81,249) / (3,15,143) 5.4 %, cyan (17,175,236) 1.8 %.
- **Stripes.** Only R changes between salmon and grey (G 80-88, B 60-75 stay). Period ~195 lines (pink ~94, grey
  ~101), 10-90 % transitions over ~24 lines. In the still the pink halves start on rows 640, 834 and 1030.
- **Zebra bands.** One chirped H oscillator.
  - Band left edges at x -17, 171, 377, 555, 725, 868, 997, 1122, 1245, 1367, 1505, 1666, 1833.
  - Period ~200 px at x 300, ~122 px at x 1100-1300, ~170 px at the right edge.
  - The band occupies ~45 % of a cycle: a dark-red soft left ramp (~20 px), orange (252,120,0), a thin yellow edge
    (242,158,0), then a quick fall.
- **Chevrons.** Frame-locked "<" dips on rows 252 and 780 (period 528; they stay at 252 / 780 +-2 for the whole
  clip).
  - The dip shape is (1 - d/160)^1.35, d = lines from the cusp row.
  - Its depth in phase rises from 0 at x 0 to 0.89 cycle at x ~580, then falls to ~0.3 at the right.
  - A fit with A (x/x0) e^(1 - x/x0) gives x0 550, A 0.88. The band edges fit with rms ~6 px per row.
- **Body (the key).** A triangle LFO of 1165 lines, plus |sin|^0.6 scallops 455 px across, against a threshold.
  - The black-only cusps sit at x 300, 810, 1230 and 1710; the bowls reach row ~630 and the cusps rise to ~445.
  - The top edge is the mirror image: its cusps poke down to row ~110.
  - Body coverage ~50 %.
- **Stretch.** The body's LFO frequency-modulates the band oscillator: the bands fan out to the right near the body.
  Measured per row from the band and dash phases, eps = 0.235 max(0, Rv - 0.32): 0.16 at the body centre (rows
  262-296), 0.022 on row 640, 0 by ~200 lines outside the body.
- **Dashes.** Three glowing bars per pink half of the stripe oscillator.
  - They sit +12, +46 and +80 lines from the pink start, 34 apart.
  - Each bar glows ~36 lines tall: tan (200,145,90) on the left, a salmon (246,120,85) core on the right, a dark-red
    halo.
  - Horizontally a dash is the band-phase window 0.30 .. 0.80, so the dash columns share the zebra's chirp and
    chevrons.
- **Crescents.** 77 % of the blue pixels lie in the band-phase window 0.55 .. 0.95.
  - Which columns light shifts from the body edge (cycles 0, 2, 5, 8) to its centre (1, 3, 7, 10). A gate on the W
    phase + 1.1 Rv fits this.
  - Their colour runs blue (1,12,208) -> (1,81,249) -> cyan (17,175,236) with the dash brightness underneath.
- **Flames.** Red spikes 60-100 px tall rise into the black where the orange bands meet the edge: the band's
  brightness pushes the key back.

## Motion in the clip
Measured over the whole 11.5 s clip at 30 fps (`kymo.py`, `breath.py`, `cusptime.py`, `stripetime.py`).
- **Body.** It rolls UP ~124 lines/s, i.e. 2.07 lines per 60 Hz field. Period 1165 lines = 9.4 s.
- **Stripes and dash rows.** They creep up ~10 lines/s, i.e. 0.17 line per field.
- **Chevron rows.** They never move.
- **Bands.** They breathe +-30 px at x 600 (+-75 px at x 1800) as the body passes: the stretch rides the LFO. Each
  row reaches its floor ~1.4 s after the body has gone by.
- **Jumps.** At 3.1 s and 10.5 s the stripes, dashes and body jump down ~45 lines together (a V-slip of the rolling
  oscillators). Not modelled.
- No knob is turned. The other candidates of the clip (0.5, 4.0, 8.0 s) are the same patch at other roll phases.
  At 4.0 s the body is centred and the crescents read as full ")" arcs.
- **The program:**
  - At the default Roll (700) the body moves 2.05 lines per field (123 lines/s at 60 Hz) and the stripes creep 0.17
    line per field.
  - The stretch, the crescent gate and the flames all follow the rolling LFO.
  - At power-up the body sits at the still's phase.
  - `cmp_motion.png` shows the integer model at -0.5 / +0.5 / +1.0 / +1.5 s next to the m-frames.
  - The RTL is bit-exact with the model across fields: the 720p frame matched after 2 rolls and 3 stripe creeps.

## What it does
1. **Key LFO.** Rv(y) is a triangle over a 1165-line period that rolls with Roll. The stretch is
   eps = Stretch max(0, Rv - 0.32), and the beam runs xs = x (1 - eps) (a per-line DDA step).
2. **Zebra.** phi = Bands PHIP(xs) + DPHI(xs) Chevron T(y). The colour comes from ZEBRA(w, phi): a 64-step band
   profile per stripe pink weight w (8 levels).
3. **Body.** The key is s = Rv + 0.31 W(theta) - 0.5 - kc, with theta = (xs + 165) / 455 and kc set by Body.
   - Black where s - beta F(phi) > 0 (F is a triangle over the band phase that peaks on the band body).
   - Red flames where s > 0 but the pixel is not black.
   - Invert negates s.
4. **Dashes.** brightness = GV(distance to the nearest bar) x GH(phi).
   - The colour is a heat ramp (dark red -> tan) with a salmon core over the dash's right part.
   - There is no glow within 40 key units (~23 lines) of the body edge.
5. **Crescents.** Inside the body, where phi is in 0.53 .. 0.97 and frac(theta + 1.1 Rv + 0.15) < gw.
   - The gate width gw ramps 0.25 .. 0.62 with the stripe's pink weight.
   - The colour is the dash brightness on a deep-blue -> cyan ramp.
   - Fur = Flood opens the gate everywhere.
6. **Output.** Ice exchanges U and V. Then chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16, and the
   blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Stripes | the salmon / grey stripes and the dash rows (they share the oscillator): x0.4, two giant stripes (0) .. the clip (512) .. x2.5, fine pinstripes and tight triplets (1023) |
| K2 Bands | the zebra band frequency, and with it the dash columns, flames and crescent windows: x0.4, 5 fat bands (0) .. the clip's 13 (512) .. x2.5, 33 thin bands (1023) |
| K3 Chevron | the chevron dips: reversed ">" (0) .. straight bands (256) .. the clip (512) .. 3x deep zigzags (1023) |
| K4 Roll | bipolar (+-8 deadband, square law): the body rolls up (+) or down (-); 700 = the clip's 2 lines/field; full = 16 lines/field |
| K5 Stretch | how far the body fans the bands: squeezed toward the body (0) .. none (256) .. the clip (512) .. 4x, the bands and dashes stream out to the right (1023) |
| K6 Etch | how far the camera (or the Osc) bends the bands and the body's edge: 0 .. ~0.9 band cycle and ~540 lines per full luma (square law); 400 = 0.13 cycle / 38 lines |
| S7 Twin | one body (the clip) / two bodies per frame, each half as tall |
| S8 Invert | Off (the clip) / On: the key inverted, the zebra inside the body's outline and the dash fur outside |
| S9 Fur | Crescents (the clip) / Flood: blue fur over every crescent window of the body |
| S10 Colours | Ocelot (the clip) / Ice: U and V exchanged (pale cyan zebra, orange crescents, cyan flames) |
| S11 Source | Video / Osc |
| P12 Body | the ocelot body's size and the flames: a thin string of scalloped beads, 27 % coverage (0) .. the clip, 48 % (512) .. the body fills 90 % of the frame, the zebra seen only through scalloped windows, flames 50 % taller (1023) |

Glides (1/8 of the gap per field) on K1, K2, K3, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) - 16 from a one-line buffer, is box-filtered over 4 px and
  added to the band phase and to the key.
  - The zebra bands bend around the picture, and the body's edge (with its flames) swells over bright areas.
  - At the default Etch the still's structure stays readable on selfie.jpg (`cmp_final.png`, bottom); fine camera
    texture frizzes the band edges.
  - With no input, Video mode is the still.
- **Osc mode.** A drifting 640 x 540 virtual-px triangle lattice feeds the same path, centred on its mean.
  - Its gain is 16x the camera's on the phase and 8x on the key: about +-1 band cycle and +-160 lines at the
    default Etch.
  - The bands swing into big waves and the body's outline follows the lattice.

## Engine
- **Per-field sequencer** (vblank zone):
  - one serial shift-add multiplier, 4 slots: stripe step = mode base x 2^(K1), Roll^2, Roll^2 x 233, Etch^2;
  - the shared glide.
  - Bands is a 1/32-octave exponent from a 32-entry exp2 table (no multiply).
  - Chevron, Stretch, Body and the flame push are piecewise-linear shift-adds.
  - The roll and the stripes' creep are applied at the start of the zone.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in ~40 clocks (it fits SD's 858).
  It shares the multiplier:
  - Rv from the key phase, and eps = Stretch max(0, Rv - 0.32) (a product);
  - the x step xs (1 - eps), the W phase step (x18) and the SD start;
  - the stripe pink weight and the distance to the nearest bar (minima split over three states), then the bar glow
    from a 64-entry table;
  - T' = T(cusp) x Chevron (a product);
  - the crescent gate term and width, and the key line term.
- **Beam DDAs:** xs (Q10 h) and the W phase (Q24).
- **Pixel path (21 stages):**
  1. beam;
  2. table addresses (PHI 1024 x 20, 2 virtual px per entry; W 256 x 8), crescent gate phase;
  3. EBR read, luma box;
  4. fabric copies;
  5. products: Bands x PHIP (12x8), DPHI x T' (9x11), the 5/4 W, luma x Etch;
  6. band phase (Q8) and key s;
  7. flame profile, dash window GH, salmon step, crescent window and gate, Invert;
  8. flame push product, dash brightness (6x6);
  9. key decisions and the palette address;
  10. palette ROM (1024 x 24);
  11. fabric copy;
  12. flame red, Ice, 10 bits;
  13. pipeline;
  14. pipeline;
  15. filter input;
  16. chroma running sums, luma taps;
  17. chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4;
  18. luma 13/16;
  19. pipeline;
  20. pipeline;
  21. blanking gate.
- **Stores** (19 EBR):

  | Store | Shape | EBR |
  |---|---|---|
  | band phase table PHIP / DPHI | 1024 x 20 | 5 |
  | palette (zebra 512, dash 256, crescents 256) | 1024 x 24 | 6 |
  | scallop shape W | 256 x 8 | 1 |
  | line buffer | 2048 x 8 | 4 |
  | chroma 8-back ring (`no_rw_check`) | 256 x 20 | 2 |
  | C_LAT sync ring (`no_rw_check`) | 256 x 3 | 1 |

  The chevron dip (129 x 8), the bar glow and the dash window (64 x 6 each) are fabric tables.
- **Contracts:**
  - the generated avid;
  - one latency, C_LAT 20, in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr, V = Cb);
  - the chroma running sums snap back to black after 8 blank samples (resync);
  - luma pre-scaled 16/13 and capped at 940 in the ROM.
- **Verified:**
  - GHDL 720p60, decimation 1, black input: the default frame is **bit-exact with `imodel.py`** (100 % of pixels,
    at the RTL's field count).
  - Osc and camera frames rendered (`rtl_osc.png`, `rtl_cam.png`).
  - Four RTL bugs were found this way and fixed:
    - a signed-slice compare (deep-pink stripes read grey);
    - a 10-bit overflow of 5 W (the scallops wrapped);
    - a signed resize folding the cusp-ROM address 128 to 0 (two garbled rows every 352 lines);
    - the line engine's three-way minimum in one clock (the HD critical path at 74.99 MHz, now split over three
      states).
  - `thincheck2.py` (1080p; local extrema > 120 in Y or chroma; runs >= 24 px) finds no single-row slivers at the
    default, P12 0 / 100, Twin, Invert, Flood, Stripes 1023 or Stretch 1023, at three roll phases.
  - `sheet.png` (control sheet) and `cmp_motion.png` are rendered from the bit-exact model.

## Status
- **v0.1.0 built 2026-11-09 with router2 (the builder's default); all six configs on seed 1. Not flashed.**
- Post-route Fmax (MHz):

  | Config | Fmax |
  |---|---|
  | HD Analog | 87.21 |
  | SD Analog | 88.93 |
  | HD HDMI | 88.49 |
  | SD HDMI | 90.60 |
  | HD Dual | 91.79 |
  | SD Dual | 88.89 |

- 5,677-5,710 / 7,680 LC (74 %), 19 / 32 EBR.
- iceunpack OK x6; the packed config carries the labels and presets; dupin.py reports 0 duplicate-input carries.
- Output: `out/rev_b/ron/heirloom/ocelot.vmprog`.
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh ocelot`.
- Timing history: the first build of an earlier netlist reached HD Analog 74.99 MHz. The critical path was the line
  engine's distance minimum, which is now split over three states.
- Presets are hand-written. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **Body edge.** The scallops are a regular |sin|^0.6 bowl train. The clip's edge is more irregular.
  - Its fish lenses (red bodies with dark eyes along the edge, clearest at 4.0 / 8.0 s) are not drawn.
  - The thin tan rim on the steep edge segments is not drawn.
- **Flames.** They are triangles of the band phase pushing the key, in solid red. The still's flames are softer and
  more varied.
- **Crescents.** They are band-phase windows gated by a W-phase / Rv window. They have the right placement
  statistics and become ")" arcs as the body passes, but they are harder-edged and more regular than the still's
  rounded blobs.
- **Dashes.** The vertical glow is smooth (0.38-line table steps). The salmon core switches in 4 steps across the
  dash's right half.
- **Stretch.** It is proportional to x (a DDA step). At the body centre the still's rightmost dash columns are ~0.2
  cycle off from it.
- **Not modelled:** the clip's two V-slip jumps (3.1 / 10.5 s).
- **Stripes 1023.** The gaps between dash bars shrink to ~1 line of dark red: low contrast, no hard slivers.
- **Resolutions.**
  - 720p matches 1080p.
  - SD shows the centre 4:3 (virtual x 240 .. 1680): the composition, chevrons and body hold, and the outer bands are
    cropped.
- **Shell.** The RTL shell comes from velvet v0.1.1, the phase-2 fern / lozenge lineage with the latest fixes.
