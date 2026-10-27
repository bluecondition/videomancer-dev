# Magma — Heirloom #29

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230515_00_46_28_Pro.jpg` (1920×1080 capture, 2023-05-15). This replaces v0.1.0 (three
summed triangle waves through a palette), which Ron judged too abstract. Presets are hand-written
placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_magma/` (`notes.txt` has every number).
- **Composition.**
  - Top half: a cyan field (0, 250, 230) with pale pink streaks sweeping down-right. The fan runs from
    about 70° on the left to about 30° at the top right, with orange/red pockets in the streaks.
  - Bottom half: a blue field with magenta ")" bands, vertical near y 600–700 and leaning left further
    down.
  - Lava blobs, yellow with red rims in the top row and red in the bottom row:
    - top-left x 0–215, y 0–500;
    - top-centre 872–1202 / 81–433, tilted;
    - right 1658–1919 / 121–446;
    - bottom-left 0–250 / 530–1000, with a pointed ">" edge at (250, 770);
    - bottom-centre 837–1184 / 565–966;
    - bottom-right 1700–1919 / 680–850.
  - A row of pointed red lenses at y ≈ 515: x 115–215, 352–666, 816–966 and 1203–1773, with an
    X-pinch at (130, 515).
  - A red band along the bottom edge with a wavy top, and yellow bits along the top edge.
- **Mechanism (from the channel split).**
  - R and B are steep keys of ONE field: blue/cyan below magenta below red. The B-off area lies inside
    the R-on area for 99 % of pixels.
  - G falls with the same field and with height.
  - The lava is a separate key with its own green channel.
  - k-means 12 shares: blue 17.4 %, red 16.6 %, cyan 15.1 %, magenta 10.0 %, violet 9.6 %,
    azure 7.5 %, yellow 6.1 %, lavender 5.9 %, pale cyan 5.8 %, orange 2.9 %.
- **Band oscillator.**
  - The per-row band period fitted over the picture: 133.5 px (y 20) → 159 (260) → 180 (400) →
    198 (640–720) → 191 (900) → 181 (1020).
  - The phase at x = 0 is 247·f(y), with rms 0.02 cycle. The oscillator is hsync-reset with phase 0 at
    x = −247 px.
  - Waveform: a sawtooth that rises sharply over ~0.15 cycle and falls slowly over ~0.6. Blue→magenta
    edges sit at phase 0.8 (median residual 0.03 cycle).
- **Colour laws.**
  - G = clip(Gc − kG·(F − F0)⁺). Gc runs 0.99 → 0.06 from top to bottom, F0 0.35 → −0.4, kG 0.4 → 0.1.
  - B maximum 0.906 (top) → 1.0. Magenta R 0.93–0.98.
- **Lava.** The signed distance of the blob mask is fitted as a quadratic B-spline on a 36-px grid
  (56 × 33 knots), 99.15 % pixel accuracy. The lava green is a second spline, rmse 0.075 inside the
  blobs; a green-vs-depth law managed only 0.20.
- **Pockets.**
  - Share of pixels outside the blobs that are pocket (B off), per 180-line zone: 1.8 / 0.5 / 0.9 /
    2.2 / 1.5 / 2.4 %.
  - Stacked roughly every 33–40 lines.
  - No global vertical oscillator: the best phase coherence over periods and tilts was 0.42.
- **Texture.** Residual correlation length 8–10 px in x and 5–6 lines in y. No global line tear; row
  shifts of 0 / +1 px are just the slant. Fine contour "fingerprint" rings run around the pockets.
- **Fit scores.** The float proto scores 4×4 region means max 18 / mean 6.6 of 255 and local contrast
  0.89 of the photo. The RTL-faithful integer model scores 7.9 / 0.83 (luma scale off).

## What it does
1. **Bands.** One line-locked sawtooth oscillator.
   - Its per-line frequency is fl = α + β·T(v), where T is the fitted curve (512 entries, one every
     2 virtual lines). β = Stripes × Fan and α are worked out once per field.
   - The phase starts at fl·247 at the left edge (fl·487 at SD, which begins 240 virtual px in) and
     steps fl per virtual px.
   - A per-line comb jitter is added to the start phase.
   - With S9 Mirror on, the oscillator runs backwards after the centre pixel.
2. **Band field.**
   - F = wave(phase) + BG/2 + pockets + melt, where BG is the fitted static band level.
   - Pockets ride the bands: a per-band, per-slot amplitude from a 32 × 32 table, shaped by
     min(band-phase trapezoid, vertical slot triangle), ×2.5. Slots rise about 18 virtual lines per
     second.
   - A contour ripple, tri(F·12), is added where F > 0.16 (S8 Coarse: tri(F·3), twice as deep).
3. **Colourizer.**
   - R = clip(121 + 3·(F − off − 0.5)).
   - B = clip((1.9 − F)·8) capped at the row's B maximum.
   - G = clip(Gc − kG·(F − F0)⁺), with {Gc, F0, kG} read for the Sky-shifted line.
4. **Lava.**
   - Key = FR + Lava offset + breathe LFO (±4 px, 9 s) + melt/8.
   - The B and G mix uses the key; the R mix uses the key delayed 6 virtual px, which gives the
     magenta/dark fringes.
   - Lava green = GB plus a hot-core term above 50 % Lava (S7 Flat: dark red).
5. RGB → YUV in the hardware convention, then the output conditioning and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Lava** | The lava blobs, the lens row, the bottom band and the pocket density. 0 % = no lava, a calm striped flow; 50 % = the photo; 100 % = the lava floods into a yellow-hot sea with stripe windows. Glided. |
| K1 Stripes | How many bands: 0.5× .. 2×, centre = the photo. Glided. |
| K2 Fan | How far the streaks fan out from the left edge: 0 = straight vertical bands, centre = the photo, max = 2×. Glided. |
| K3 Flow | Band drift, bipolar, still at the centre ±8. Positive streams the stripes right; square law. Default 800 ≈ one band every 7 s. |
| K4 Pockets | The red pockets in the bands: 0 = none, centre = the photo, max = a pocket in every slot. |
| K5 Melt | How far the video (or Osc) bends the bands and swells the lava. Default 400. |
| K6 Sky | The cyan/blue horizon. Low = night blue/violet with red lava; high = cyan everywhere with yellow lava. |
| S7 Cores | Hot (yellow cores with red rims, the photo) / Flat (red lava). |
| S8 Ripple | Fine (the photo's fingerprint contours) / Coarse (wide topographic rings). |
| S9 Mirror | Off / On: the stripe flow mirrored about the centre into chevrons. |
| S10 Palette | Magma (the photo) / Ice (green/yellow stripes, blue lava). |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** the camera luma (2-px average, (Y − 512)/2, tapped straight from `data_in`) × Melt is
  added into the band field, so the bands bend around the picture. One eighth of it goes into the lava
  key, so the lava swells on bright areas. At default knobs the photo's composition still reads.
- **Osc:** a triangle of the beam's cell coordinate (288 virtual px period) plus a vertical triangle
  that drifts per field, fed through the same path. Melt stays live.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks):
  - Glides (±4 deadband, 1/8 ease) on K1, K2 and P12. K3–K6 and the switches are latched.
  - One serial shift-add multiplier computes: speed sp²>>9, β = (ks·fan)>>9, qq = (ks·|512 − fan|)>>9,
    and α = ±(f0·qq)>>9.
  - Also updated: the drift, the LFO, the pocket rise, and the comb seed (every 8 fields).
- **Per line** (kicked at the source avid rise of line n; computes line n + 1):
  - One MAC lane blends three control rows into 56 knots × 3 fields (FR, BG, GB) in 504 clocks, so it
    fits SD's line.
  - It writes 55 cell-boundary values B[i] = (D[i] + D[i+1])/2 into a ping-pong RAM.
  - The scalars: the T(v) ROM; fl = α + β·T and ph0 = fl·(247 + u0) on a second serial multiplier;
    dph by shift-add; the colour row {Gc, F0, kG, pocket gate}; the pocket offset; the comb jitter.
  - Everything is handed to the line registers at hsync, which also preloads the cell window.
- **Pixel path, 28 stages:**
  - Beam: cell coordinate (Q16) and band phase (Q19).
  - Cell-crossing window shift, linear-in-x evaluation of FR, BG and GB.
  - Wave ROM.
  - Pocket slot → table ROM (+ fabric copy), min shape, offset, ×shape, ×2.5.
  - F; ripple; colourizer; lava key and green with their delay pipes; lava mix; Ice swap.
  - RGB → YUV: luma 66/129/25; chroma 112/96/16 and 112/40/72.
  - Output: chroma box-8 pair [1 2 2 2 2 2 2 2 1]/16 over a block-RAM ring, luma [1 2 1], luma ×13/16
    (white ≈ 777), blanking gate.
- **Memories (22 EBR):** spline control points 2048 × 20, frequency 512 × 17, colour rows 256 × 26,
  knot RAM 128 × 30 (ping-pong, `no_rw_check`), pocket table 1024 × 8, wave 256 × 8, chroma ring
  256 × 20 (`no_rw_check`), sync ring 256 × 3.
- **Contracts:**
  - Generated avid (parity-locked).
  - One 28-clock latency in every mode, with the syncs in the block-RAM ring.
  - The HD HDMI bar fix before the gate.
  - Virtual 1080-line geometry; SD shows the central 4:3.
  - Colours in the hardware convention (U = Cr, V = Cb).
- **Fit history.**
  - The first RTL (three MAC lanes, quadratic-in-x, per-line op-coded FSM, per-pixel LFSR jitter):
    6.7 k LUT / 3.7 k FF standalone, which would have been about 9 k LC.
  - Ablations and the restructures above brought it to 4.1 k LUT / 2.7 k FF.
  - The last critical path (a clamp feeding the hot-core multiply in one stage) was split.
- **Verification** (GHDL 720p60 decimation 1, against the RTL-exact `imodel.py`):
  - Static frame: 91.4 % of pixels within ±12/255, after the tester's 4-px capture offset.
  - Motion at Flow max: frame 5 matches the model advanced 5 fields (90.1 %) versus 55.6 % un-advanced.
  - Video and NTSC renders checked.

## Status
- **v0.5.0 built 2026-10-27, NOT flashed.** **ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh magma`); all six configs seed 1.
- Post-route Fmax: HD Analog 88.04, HD HDMI 86.93, HD Dual 86.82, SD Analog 86.76, SD HDMI 85.22,
  SD Dual 84.79 MHz.
- 7063–7084 / 7680 LC (92.0–92.2 %), 22/32 EBR. All bitstreams pass iceunpack.
- `out/rev_b/ron/heirloom/magma.vmprog`, 478,712 B. A rebuild reproduced identical numbers.
- router2 does not close this design family: an earlier netlist gave 72.5 MHz on HD Analog. Rebuild
  with router1.

## Deviations from the photo, and why
- **Pockets and textures** are random realisations at the photo's measured density, not its exact
  pockets. The photo has no coherent oscillator behind them. The fine band-phase wiggles (±0.07 cycle)
  and the whitish wisps in the top half are smoother or lighter.
- **Lava outlines** are linear in x between 36-px knots, so blobs are slightly polygonal. The lenses'
  long pointed tips and the X-pinch at (130, 515) are blunted (spline resolution), and the green fringes
  on the upper-left of the top blobs are weaker. Quadratic-in-x evaluation did not fit the LC budget.
- **Brightness:** drawn whites and yellows are ≈ 777 because the HD HDMI bar fix scales luma by 13/16,
  so the program reads darker than the photo.
- **SD (4:3)** shows the central 1440 virtual px, so the left- and right-edge lava blobs are cropped.
  720p and 1080 are the same picture.
- **Chroma** uses the 8-sample box in every mode (softer chroma at 720p and SD). The chroma conversion
  coefficients are simplified (mean colour change 2/255). Both were area savings.
- **Static fields:** the fitted lava and band level stay in place. Motion is the band flow, the rising
  pockets, the lava breathe, the re-rolling comb and P12.
- **Interlace:** no bottom-field half-line offset (field_n is tied high on hardware).
