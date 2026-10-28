# Ovals — chains of rainbow-roped vesica ovals on a striped pink ground

**v0.5.1 (2026-10-28): built, NOT flashed.** Rebuilt from the photo WIN_20231003_23_01_50 (Heirloom #50).
v0.5.1 is the review iteration on v0.5.0: no straight seams (lens union plus a both-side palette blend at every
side flip) and high-contrast ring interiors.  Presets are hand-written placeholders.

## Reading of the photo (measured)
- **Ground**: hot pink (R 255, G 30..45) with horizontal stripes: B alternates ~78 / ~145 with a half period of
  39.35 lines (period 78.7; edges at y 19, 57, 97, 135 ...).  The left edge (x < ~300) is plain pink: the lattice
  oscillator is hsync-reset (its phase holds at the trough before the first column).
- **Lattice**: columns of vesica lenses ("ovals") stacked into chains that touch at X points (X rows y ~256 and
  ~781; lens rows centred at y ~ -6, 518, 1043).  The strands were tracked row by row (track.py / trackG3.py) and
  fitted to rms 5 px (fitgeo3.py -> geo3.json):
  - lens j = interval [V_j, G_j] per row,
    V_j = xc + c_j + oV_j - M(y), G_j = xc + c_j + oG_j + 1.047 M(y) + dl(y),
    c_j = -(m+L), -(m-L), +(m-L), +(m+L), with m = 255.3 px and L(y) = 185.4 + 59.3 t + 25.2 t^2 (t = (y-520)/520);
  - M(y) = 127.1 |sin(pi (y - 255.6)/525.6)|^0.816 (a rectified sine: the two crossing strand families);
  - xc = 967.6 + 7.6 t; dl = 24.4 - 16.8 t; period 1513.9 px (V4 = V1 + period, seen at the right edge);
  - per-strand offsets oV = (2.0, 7.4, 10.8, 0), oG = (-1.7, 4.6, 4.8, -24.7).
  That is a horizontal triangle folded twice (at m, then at L(y)) and keyed against the rectified vertical sine.
  Four lenses per period: the outer pair and the inner pair, which overlap into the middle **double oval**.
  L grows toward the bottom, so the inner pair converges and the outer pair spreads.
- **Ropes**: every lens has the same orientation, a scan-direction asymmetry and not a symmetric field.
  - Left (V) edge, outside to inside: a green fringe ~30 px outside a violet core (68, 65, 219), then blue,
    brown at +25 px, magenta at +50 px, then red.
  - Right (G) edge, inside to outside: orange, olive, teal, a pure-green core (0, 190, 0), then teal.
- **Channel split**: R is V-shaped dark at the ropes; G is a dark band just inside the V ropes; B carries many rings.
  The B rings follow the contours of Q = the signed distance into the union of lenses (qcont_B.png).
- **Diamonds** between the columns at the X rows: nested contours with red-orange cores at the top and PINK cores
  at the bottom (L is larger there, so the gap is deeper).  Cyan / blue stars sit at the X points.
- **Interiors**: red-orange (22 %), orange (21 %), magenta-pink (14 %), peach (13 %), brown (9 %), cream (8 %)
  rings, cut into 39-line blocks.  The G-side features shift with the stripe parity (G strands +-1.5 px).
- **Glow**: below y ~950 everything except the V ropes and the pink ground whitens (per-row gamma 0 above 940,
  ~0.9 below 990).
- No row tearing (row-to-row shift 0 on every row).  Soft horizontal edges, ~10 px 10-90 %.

## What it does
1. A phase DDA w (half-px units) runs the period-1514 triangle T = |w|.  It is held at the trough left of the
   first column (hsync-reset).
2. The fold picks this half's lens (inner or outer: T vs m), Qf = h_j - ||T - m| - L_j|.  The other half's inner
   lens (the double oval's partner) gives qo = h - |T + m - L|.  **Q = max(Qf, qo)**: the union across the
   period centre, continuous, with a crease where the inner pair overlaps.  The fold levels L_j (L + per-lens
   offset) are clamped at 0, so at Split 0 the pairs merge into one lens.
3. side = the winning lens's half (0 = left / V rope, 1 = right / G rope).  e = the distance to the nearest
   side flip: the lens axis, the diamond-centre line, or half the crease / gap with the other inner lens.
   The weight of the G-side palette is b = 8 +- e/4 (of 16): a +-16 px ramp at axes, +-32 px at creases.
4. Palette index = Q x rope gain + luma x Etch.  One ROM word holds both sides' colours at {bank, index}.
   bank = stripe parity inside a lens and in the deep ground; diamond or ground-fringe elsewhere.
5. colour = A + (B - A) b/16 (A = V-side palette, B = G side; Twist swaps them).  Then a cyan X star near the X
   rows, (64 - s8)(32 - |Q|)/32, and the bottom white glow, gy(y) x kap(side, Q).
6. Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16, then the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Columns | lens columns across | lattice density x0.5 .. x2 (zoom about the centre); 512 = the photo |
| K2 Links | lens rows down each chain | vertical period / ~2^(1.3 (k-512)/512) (x0.4 .. x2.5) |
| K3 Split | spacing of the paired columns (the double oval) | fold level L x k/512: 0 = pairs merge into single columns, 512 = the photo, 1023 = the inner pair crosses |
| K4 Speed | chain flow, stripe roll | bipolar, +-8 deadband: chains flow down / up (default 600 = ~1 line per field, max ~6), stripes roll with them |
| K5 Ropes | rope and ring widths | index gain 2^((512-k)/512): fat .. thin ropes and rings |
| K6 Etch | rings / ropes bent by the picture | luma added to the palette index (default 200 = ~50 px per full luma, max ~250 px) |
| S7 Stripes | ground stripes, interior blocks | On (the photo) / Off |
| S8 Glow | bottom white glow | On (the photo) / Off |
| S9 Twist | rope colours | the violet and green ropes swap sides |
| S10 Palette | whole palette | Candy (the photo) / Ice (U and V exchanged) |
| S11 Source | etch source | Video (the camera) / Osc |
| **P12 Swell** | lens bulge | M amplitude 0 = straight knotted ropes, 512 = the photo, 1023 = huge interlocking ovals |

Presets (placeholders): Photo, Knotted Ropes, Big Links, Chain Mail, Ice Weave.

## Video
The photo is fully synthetic, so the camera is woven into the patch.  In Source = Video, the line-buffered camera
luma is added to the palette index (K6 Etch, default 200): rings, ropes and diamond contours bend around the
picture while the chains, diamonds and ground keep the photo's composition.  Black input gives exactly the
photo look.  Osc substitutes a drifting 2-D triangle lattice through the same path, so K6 stays live.

## Engine
- **Vblank-zone sequencer**: one serial multiplier, a shared glide (1/8 of the gap per field, +-2 deadband) and
  a 128-entry 2^x knob-curve table.  Per field it computes the density, links, rope gain, speed and the stripe
  and flow phases.
- **Per-line engine**: kicked at each line start, it computes the NEXT line (states 1..128, ~130 clocks): the
  sine ROM (|sin|^0.816, 1024 x 8), L(y) on the serial multiplier and its Split scaling, the four clamped fold
  levels, the half-widths, the other-lens offsets m - L_1 / m - L_2, the star, glow and stripe values and the
  line's start phase.
- **Beam**: a phase DDA (Q10), wrapped at +half period and held at -half period.
- **Pixel path**:
  - 1-5: the fold, plus the other inner lens (|do|, qo);
  - 6: the union, side, ground and flip flags;
  - 7-9: the blend weight;
  - 7-10: rope gain and etch (in parallel);
  - 11: ROM address {bank, index} and the glow-table side;
  - 12: palette ROMs, 6 x 1024 x 8;
  - 13: both sides copied to fabric;
  - 14-16: the side blend;
  - 17-19: the star / glow blend and compose;
  - 20-25: output conditioning;
  - 26: the gate;
  - 27-28: output registers.
  Latency 28 in every mode (processing_delay_clks 28).
- **EBR**: palette 12, sine 2, video line buffer 1 = 15 / 32.  The palette and glow table are generated by
  ovals.py into ovals_pkg.vhd, in hardware convention (U = Cr, V = Cb), luma pre-scaled 16/13 with a
  860..1100 -> 860..940 knee.
- **Contracts**: generated avid; one latency in every mode; the [1 2 2 2 2 2 2 2 1]/16 chroma and [1 2 1] luma
  bar fix plus luma 13/16 before the blanking gate; virtual 1080-line coordinates (SD shows the centre 4:3,
  720p the same picture).

## Status
- **v0.5.1 (2026-10-28), router2: built, not flashed.**
  - Fmax (MHz): HD Analog 83.35, HD HDMI 84.57, HD Dual 76.87 (**seed 2**, seed 1 missed), SD Analog 81.23,
    SD HDMI 83.27, SD Dual 82.12; every other config on seed 1.
  - 7638-7652 / 7680 LC (99.6 %, over the 92 % guideline: no headroom), 15 / 32 EBR, iceunpack OK x6.
  - out/rev_b/ron/heirloom/ovals.vmprog (494,246 B).
  - GHDL 720p (decimation 1) matches the integer model exactly: 100.00 % of pixels within 12, mean |dRGB| 0.00,
    at defaults and at K3 343.
- **Metrics vs the photo** (local contrast = 32 x 32-block luma std; photo 13.2, lattice 16.5):
  - RTL 720p: v0.5.0 9.9 / 12.5, v0.5.1 13.0 / 16.2 (interior 19.8 vs 17.0).
  - 4 x 4 region means: v0.5.0 max 9.7 / mean 6.8, v0.5.1 max 12.3 / mean 7.3.
- **v0.5.0 (2026-10-27)**: built 6/6 (SD Analog seed 2), 80.4-87.7 MHz, 6731-6756 LC (88 %).  Reviewed: hard
  vertical seams through the double oval and lens axes (fold quadrant switch), flat orange interiors.

## Deviations from the photo, and why
- **Interior rings**: the photo's deep-interior rings change with the lens-row position (a 2-D (Q, width)
  structure).  A 2-D palette did not fit the EBR / LC budget, so the rings are concentric with the lens edges:
  more regular and slightly more pastel (interior luma std 27.5 vs 36.1).  Cream and white peaks are kneed
  under the 777 output cap.
- **Region means** are a little worse than v0.5.0 (the brighter rings push the upper-centre cells up ~+11 in G
  and B); the local contrast now matches.
- **Glow**: a smooth ramp toward white, weighted per palette entry, not the photo's blobby white and orange streaks.
- **Streak blocks** come from the 39-line stripe banks, not the photo's per-band random G-side shifts.
- **Double oval**: the hidden inner ropes meet at a soft crease instead of crossing visibly.  Near the X rows
  the crease and gap blend shows as a soft vertical band (+-32 px).
- Where a half's inner and outer lens overlap (low Split or high Swell), the fold still picks one at the
  diamond-centre line.  With the per-lens offsets this is a Q step of up to ~12 px, inside a blend zone.
  It is not visible at defaults.  The full 3-candidate union was drafted (rb_ovals/syn/v20.vhd) but needs
  ~7750 LC.
- The header comments of ovals.vhd and ovals.py describe the ideal four-lens union; the pixel path implements
  the fold plus the other half's inner lens (the stage comments say so).
- SD shows the centre 4:3: the pink side grounds are mostly cropped.

## Working files
`.heirloom_work/rb_ovals/`:
- `ref.jpg`, `notes.md`;
- strand tracking and geometry: `track.py`, `trackG3.py`, `fitgeo3.py` -> `geo3.json`;
- palette: `mklut*.py`, `mklut17.py` -> `lut17d.npy` / `rom17d.npz` (= `rom.npz`); `mkhook.py` writes `ovals.py`;
- `proto_final.py` (float prototype), `imodel.py` (integer, RTL-exact);
- checks: `match.py`, `regions.py`, `lc2.py`;
- images: `cmp_final.png`, `z_mid_final.png` (photo / v0.5.0 / v0.5.1 middle double oval), `sheet.png`;
- `v051a/` (fallback build without the fold clamp), `syn/v20.vhd` (3-candidate draft).
