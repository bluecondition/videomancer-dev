# RUTT ETRA XY — Rutt Etra displaced on both axes

ruttetra (horizontal scan lines pushed down by luma, hidden-line terrain)
with the displacement opened up to both axes. K5 **Mode** picks how:

| Mode | Picture |
|---|---|
| Classic | the ruttetra picture: lines pushed straight down |
| Slant < > / << >> | every point also moves sideways: 1 or 2 px across per row down, left or right |
| Mesh < > | slant 1 plus rungs from every grid node down to the next line (wire mesh) |
| Weave | Classic plus a second family of VERTICAL lines pushed sideways (the ruttetra_v picture), woven over the first |
| Zoom (v0.2.0) | luma zoom: every point pushed out from the picture centre by (distance from centre) x (brightness), on both axes |

A NEW program (Ron, 2026-11-14: "a version of RuttEtra that incorporates
both horizontal and vertical displacement", then "all 3 of these modes on a
knob"); ruttetra and ruttetra_v are untouched. Zoom added 2026-11-14 at Ron's
request (a friend's definition: "X ramp x luma, V ramp x luma ... most zoomed
in at its brightest point and not zoomed at all where it's black").

## Engine

The base is ruttetra v0.2.4's rotating-bank displaced-scanline gather (8
banks x 1024 x 12, 24 EBR; see that SUMMARY). What is new:

- **Slant = a per-bank sideways read offset.** A point displaced by d rows
  also moves L*d pixels sideways, and a hit needs d = age, so bank k is
  simply read L*age_k pixels to the side. Each bank has an offset
  accumulator (10.4 signed columns) stepped once per active row beside its
  age, by a per-timing-class step (1/16 columns per field row at slant 1:
  1080p 8, 1080i 16, 720p 8, 480i 18, 480p 9, 576i 15, 576p 8) so the angle
  is the same on every raster. It stops stepping at age 256 (pure skirt).
- **Centring.** Mid-grey is displaced by `scale_eff` rows (at most 128); the
  read column is moved by that many rows' worth of slant, so mid-grey
  material stays on its source column and bright / dark lean opposite ways
  (without it the whole picture slid sideways with the Relief slider and
  left a black wedge). The node grid is un-shifted by the same amount so the
  rungs stay on their source columns.
- **Reading outside the stored row.** A bank read off its row must neither
  hit nor cover. Displacement is never 0 in real data, so 0 is the "no data"
  word: a negative (or wrapped) address forces it, and every sampled row
  ends in one zero word written just after its last column. Per bank and
  scanline: `seen` (valid data has begun) and `past` (a zero after valid
  data: dead until the next scanline). The first valid column restarts the
  connected stroke. The read-ahead clamp holds 2 columns inside the measured
  line end, so a line a pixel short never reads its own marker.
- **Mesh rungs.** A node at source column c0 of a line displaced by d shows
  at x = c0 + L*d, so a pixel under a line is on a rung when (x - L*d) lands
  on the node grid, d being the covering line's displacement at the pixel
  (drawn straight down from the node; the line itself carries the sideways
  step to the next node). The grid pitch is the power of two nearest the
  line spacing (16 / 32 / 64 virtual px), so the test is a 6-bit add per
  bank against a phase counter (it counts down for a right slant). Rungs
  take the covering line's brightness and draw through its glow but not its
  stroke.
- **Weave = borrowed candidate slots.** A separate vertical-line engine
  (first cut) cost ~970 LC and did not fit. Instead, in Weave the scan lines
  keep banks 0..3 and the vertical lines take candidate slots 4..7: their
  words {previous row's displacement, this row's, brightness} are loaded
  straight into those slots' bank-output registers (new at slot 7, shifting
  down on every sampled column), the slots' age registers count display
  pixels instead of rows (one saturating adder per slot serves both uses),
  and the connected-stroke, classify and resolve stages are shared
  unchanged. Cost ~330 LC. The displacement comes from the SAME stream the
  banks are written from (W4), sampled every Nth virtual pixel; a 256 x 16
  per-column RAM keeps the previous row's displacement. A candidate enters
  3 pixels after its source column (the write-path depth), invisible.
- **One-hot resolve.** "First candidate that hits or covers, in recency
  order" as a one-hot vector: highest set bit in each half of the newer /
  older run of the bank rotation, then a 4-way rank (Weave ranks the two
  families separately). A line is visible when its family's winner hits;
  payloads are AND-OR-ed (dim shift as a thermometer, so with Hidden Line
  off the OR keeps the brighter of two crossing lines). This replaced the
  rotate + priority-index + 8:1 payload-mux stages and one pipeline stage.
- **Classifier without comparators.** The stroke thickness is folded into
  the far-end offset (e = age - hi - T), and the glow band is 4 or 8 units,
  so every test is a sign or a few-bit pattern. -375 LC against the
  comparator version.
- **Smoothing IIR as one carry chain**: `acc - (acc >> s) + (lum >> s)`
  behind a 3:2 compressor. The old sub -> barrel -> add loop was the
  critical path on every failing seed at this fill; then the contrast
  multiply, which got the stage to itself (black-level subtract moved into
  the input register).
- **Displacement hold** stops 16 units short of the retrievable window
  (8 x spacing, 4 x in Weave), so the whole stroke of a line at full
  displacement lands inside it. With the hold at window - 1 (as ruttetra)
  a held line showed only its upper glow at SD / 1080i, where a field row
  spans several units.
- Input chroma delay (Video colour) in block RAM: 8 + 8 bits in a 256 x 16
  circular buffer. From v0.2.0 the candidates' brightness nibbles ride the
  same kind of buffer (256 x 32, 2 EBR, same write pointer) from the bank
  EBR outputs to the resolve stage instead of five register stages per bank.
- **Zoom (v0.2.0), still on the row-bank engine, at full resolution.** Ron
  expected to need a coarse frame store (as Feedback); it is not needed. A
  point with zoom amount e moves from (c, r) to x = xc + (c - xc)(1 + e),
  y = r + S + e (r - yc):
  - *Vertical part.* The displacement is S + e (r - yc): the write path
    scales the relief by the row's distance from the centre row and adds or
    subtracts it about a baseline S (the lines can only be drawn below their
    source row, so everything is pushed down by S and bright material moves
    up or down from there). The hit test is the unchanged d = age.
  - *Horizontal part.* On a display row at Y' = y - S - yc, a point of bank
    k that lands there came from column c = x - (x - xc) q with
    q = (age_k - S) / Y': one slope per bank per row. The bank's read offset
    starts at -xc q and grows by q per column. q is held to 0 .. 1/4 (outside
    that no point of the line can be on the row, and any valid column gives
    the right skirt answer).
  - *Hardware.* The offset registers are the slant's (whole columns, one
    shared operand). Each bank's slope and offset fraction sit in a 256 x 16
    RAM that is read-modify-written round-robin, one bank per pixel: every 8
    pixels a bank adds 4 q to its fraction and the carry moves its offset one
    column. A sequencer runs once per line in horizontal blanking for the
    next row (~105 clocks; SD has 133): the row's relief scale from a
    Z x row accumulator (centre row = 512, so no multiply), Y' and each
    bank's (age - S) kept as sign + magnitude and stepped by the row advance
    (the per-bank ones in the same RAM; a freshly sampled bank restarts at
    -S), then an 8-step restoring division per bank whose quotient bits also
    build -(xc x q) by Horner. A quotient bit in the first two steps means
    q >= 1/4 and the bits are forced to 0100 0000.
  - *Fit.* First cut (wide per-bank slope / offset registers, a sequencer
    with absolute-value units, an 8:1 age mux and two multipliers) was
    7579 LC = 98%; the sequencer alone was ~530 LC. The RAM round-robin, the
    sign + magnitude datapath, the brightness delay in block RAM and one
    serial multiplier for the two per-field products (relief scale x window
    normaliser, roll phase x spacing; they were two always-on array
    multipliers and a critical path) brought it to 7042 = 91.7%.
- [1 2 1]/4 output conditioning on Y/U/V in every mode (rungs and vertical
  lines are pixel-rate full-swing edges on every scanline, the HD HDMI
  thick-bar trigger); 1 / 2 pixel strokes from those two sources are drawn
  x1.5 / x1.25 ahead of it. Drawn luma held at 768.
- 16-clock latency, identical in every mode; blanking-gated.

## Controls

P12 **Relief** (headline, glided). K1 Scan Lines, K2 Thickness/Glow, K3 Line
Color, K4 Contrast, K6 Smoothing, S7 Peaks, S8 Hidden Line, S9 Shading, S11
Input: as ruttetra. **K5 Mode** (was Roll): Mesh < / Slant << / Slant < /
Classic / Slant > / Slant >> / Mesh > / Weave / Zoom (nine labels: the
index is value x 9 / 1024). **S10 Roll** Run / Freeze: the
roll is now a fixed slow drift (ruttetra's default speed), since its knob
went to Mode.

## Differences and compromises (told to Ron)

- **Weave reach**: each family lives 4 samples, not 8, so its displacement
  holds at 4 x spacing - 16 and relief is normalised to that (unity at
  spacing 64). Dense line settings give shallow relief in Weave.
- **Weave vertical lines lag the picture** by the smoothing filter's delay
  (2^s - 1 px to the right): the scan lines cancel it by reading stored rows
  ahead, the vertical lines are drawn from the live stream.
- **Mesh grid pitch** is 16 / 32 / 64 px, not the line spacing, so cells are
  not exactly square. Rungs are drawn straight down from each node (a
  stepped mesh), not as slanted edges.
- **Against ruttetra's Classic**: glow band 4 units (T <= 5) or 8 (T >= 6)
  instead of T; beam-speed dimming thresholds are powers of two (2 / 8,
  4 / 16, 8 / 32 by thickness); the output filter softens steep strokes a
  little; full displacement holds 15 rows earlier; Video colour uses 8-bit
  chroma; roll speed is fixed.
- **Slant shows black wedges** at the sides where the lines have leaned off
  the stored row (up to 2 x the displacement at Slant << / >>).
- SD: thickness floors as in the siblings (a stroke is at least one field
  row / 2 virtual px); 720p's centring uses 3/8 column per row instead of 1/3.
- Roll speed and direction are no longer adjustable.
- **Zoom**: the picture sits S rows lower (S = 1 + half the relief scale, at
  most ~120 rows at full Relief); the zoom centre is virtual row 512 + S,
  not 540; the zoom range at the top / bottom edge is S rows (about 23% at
  full Relief with sparse lines, less with dense lines because the window is
  8 x spacing); offsets move in whole columns every 4 columns; rungs and the
  second line family are not available in Zoom.

## Status

**v0.2.1 (2026-11-14): Banded shading draws nothing over black; built, not
flashed.** Ron: "is it possible for an image on a black background, to not
have the dark gray grid lines in the background around the image, or to
maybe have that on a switch"; of four ways to bring them back he chose
"tie it to S9 Shading": Banded (default) hides, Luma shows them as before.
- A stored word with brightness 0 (source below 1/16 after contrast) is not
  a candidate at all in Banded: no line and no skirt (an invisible but solid
  floor would clip the image's own displaced lines), for the scan lines, the
  rungs and the vertical lines alike. Its flag is taken at the bank output
  register; a stroke never connects to a black column (it restarts flat),
  and for the vertical lines the per-column RAM carries "previous row was
  black".
- Outline spikes: an outline is a few pixels soft and the smoothing filter
  ramps out of black, and the line climbed that ramp as a tall stroke at
  every edge. In Banded the write path trims the edge: the first three
  samples after black are not drawn and reload the filter, and the two
  samples before black are not drawn (one-sample look-ahead at W2, one more
  by the raw level at W0).
- With no floor below, a rung would hang from the image's lowest line until
  its bank aged out, so in Banded a rung also ends two grid pitches below
  its line.
- Luma shading is unchanged.

6/6 with the default build (router2): HD Analog 74.85 (**seed 2**; seed 1
stalled router2), HD HDMI 75.70 (**seed 5**; seeds 1-4 missed), HD Dual
74.93 (seed 1), SD Analog 76.75 / SD HDMI 62.85 / SD Dual 73.61 (seed 1,
Fmin 27); 7091-7111 LC (92.5%), 29/32 EBR. HD margins are thin (0.6-1.4
MHz): the ~50 LC this added to v0.2.0 took the HD HDMI seed sweep from 6 of
6 passing to 2 of 6 (seeds 5 and 6), always on the smoothing filter's
feedback loop or the write path, routing-bound. If more goes in, take
something out first. Sims: an elliptical cut-out of the test picture on
black (`~/ruttetra_xy_work/src_onblack.png`) in Weave / Mesh / Classic /
Zoom with Banded, Classic and Weave with Luma, and the ordinary full-frame
picture in Banded (unchanged). Stills: `~/ruttetra_xy_work/sims/` (n_sheet,
m_sheet, o_sheet).

**v0.2.0 (2026-11-14): Zoom mode added; built, not flashed.** 6/6 with the
default build (`./build_programs.sh ron ruttetra_xy`, router2), every config
on seed 1: HD Analog 76.72, HD HDMI 75.84, HD Dual 78.09, SD Analog 81.14,
SD HDMI 80.74, SD Dual 76.65 MHz; 7030-7073 LC (92%), 29/32 EBR. HD HDMI
seed sweep on the same netlist: 6 of 6 seeds pass (75.8-85.9 MHz). (The
same design with the brightness delay left in registers, 27 EBR and +100
LC, passed 3 of 6.) Version before the zoom is kept as
`~/ruttetra_xy_work/ruttetra_xy_v010_prezoom.vhd` / `.toml` and
`ruttetra_xy_v010.vmprog`.

Sims (GHDL): Zoom at 720p dec 1 with the real-hblank tester copy
(`HB=1 ~/ruttetra_xy_work/sim.sh`; the stock tester's 64-clock hblank
starves the per-row sequencer): default relief, full relief with Luma
shading, and Peaks flipped with Hidden Line off; Classic / Weave / Mesh
re-checked at 1080p dec 2 after the brightness-delay and multiplier
changes. Stills: `~/ruttetra_xy_work/sims/` (y_zoom_a/b/c, f_zoom,
f_sheet). Not simmed: Zoom at SD (the real-hblank tester has HD blanking
only; SD's 138-clock hblank is covered by budget, not by a sim).

First-flash things to watch in Zoom: the picture sitting low (by design);
any horizontal tearing or a wrong first row at the top of the field (the
sequencer's field-start trigger is the hsync of blank lines); behaviour on
an SD source (tightest blanking budget); line-end glitches on analog
sources.

**v0.1.0 (2026-11-14): built, not flashed.** 6/6 with the default build
(`./build_programs.sh ron ruttetra_xy`, router2): HD Analog 85.95 (seed 1),
HD HDMI 78.52 (seed 1), HD Dual 86.00 (seed 1), SD Analog 87.68 / SD HDMI
74.38 (seed 1, Fmin 27), SD Dual 83.04 (**seed 2**; seed 1 stalled router2);
6738-6764 LC (88%), 26/32 EBR. HD HDMI seed sweep on the same netlist: 5 of
6 seeds pass at 76-81 MHz, one stalls the router.

Sims (GHDL, structure only): every mode at 1080p dec 2; Mesh < and Weave at
full 1080p (dec 1); Weave and Mesh at 720p dec 1, NTSC dec 1 and 1080i
dec 2; Hidden Line off; the five presets (720p dec 2). All draw as intended
after the displacement-hold fix. Hue not judged. Stills in
`~/ruttetra_xy_work/sims/` (e_hd_weave, e_hd_meshl, c_720_mesh, c_720_weave,
d_sd_mesh, presets_sheet).

First-flash things to watch: HD HDMI bars in Weave / Mesh (next lever = a
wider chroma filter); thin vertical line brightness against the scan lines;
Video-colour registration; the slant wedge on analog sources (line-length
wander moves the end marker a pixel).

Fit history: first cut (separate vertical-line engine, index resolve)
7790 LC = 101%; borrowed slots + one-hot resolve 7084; merged stroke stage
6994; comparator-free classifier 6719; final 6750. Work files and the
ablation / pack / seed-sweep scripts are in `~/ruttetra_xy_work/`.
