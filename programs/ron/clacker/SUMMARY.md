# Clacker — video as a mechanical flip-disc wall (v0.2.1)

**v0.2.1 (HW bug fix):** "sometimes green half circles on the right side or
bottom of screen" — the commit burst copied ALL 64 row-buffer words into the
grid, including columns whose cell centre is off-screen and are therefore never
sampled; their all-zero init words rendered as permanent green half-discs
(chroma code 0 = saturated green), appearing/disappearing with Disc Size
(`width mod W < W/2`). Reproduced in sim at W=110 on the 480-wide sim raster.
Fix: row buffer inits to C_CELL_RST, and the burst writes C_CELL_RST beyond the
last-sampled column (rb_hi/bst_hi) so unsampled cells stay invisible black —
robust across Disc Size changes. Also: Luma Mod now stores NEUTRAL chroma
(code 4) instead of 0, killing a green-wall flash for one refresh period when
S8 toggles back to Colour.

**v0.2 additions:** 1080i keeps discs round (field_n-toggle interlace detect →
row pitch halved in field lines, dy steps 2 per line via adds-only
`(d+2)² = d² + 4d_new − 4`, odd field starts one screen line down so fields
interleave); **Clack top band = PERPETUAL** (a settled disc re-flips to the same
value instead of resting — the wall tumbles forever, even on a still; the ~47
frame re-flip period staggers across the refresh wave); refresh wave sweeps
top-down like a real sign; square cards get their left/right bevel
(`dx² ≥ 7r²/8` band). Progressive output verified pixel-identical to v0.1
above the (previously green-bugged) bottom partial row.

A grid of discs (or square cards) mounted on a dark backing panel. Unlike a
per-pixel shader, every disc **holds its state between frames**: when the
video's value for its cell changes, that disc physically **swings** to its new
position over the following frames — a damped, overshooting settle that briefly
shows the disc's black back at the peak of the bounce. Rows re-evaluate on a
travelling refresh wave, so the wall updates in a ripple, not a jump.

## Controls

K1 Disc Size (smooth 34..162 px) · K2 Levels (2..16 quantization steps) ·
K3 Fill (gapped → touching → overfilled) · K4 Refresh (travelling 16-frame
update wave → every row every frame) · K5 Tilt Shade (flat unlit → full edge-on
blackout) · K6 Chroma (0.75–2.7×) · S7 Shape (Disc / Square card) ·
**S8 Mode** (Colour / Luma Mod) · S9 Backing (Black / dimmed live video) ·
S10 Grid (bezel lines) · S11 Bypass · **P12 Clack** = instant snap → long lazy
overshooting tumbles (the wall never stops moving).

**Colour** — every disc rests flat; its face carries the quantized video colour.
**Luma Mod** — every face is the same fixed dot colour and the video's luma sets
each disc's *resting tilt angle*: bright = flat and catching the light, dark =
edge-on and nearly invisible. Greyscale rendered by mechanical angle alone.
K2 Levels means "number of steps" in both modes, so no control goes dead.

## Architecture

Streaming pipeline c1..c14 (LATENCY 14). No divider anywhere in the program.

- **The trick that makes it fit**: a disc is EDGE-ON at the instant its state
  changes, so the face it was previously showing is never needed on screen —
  no per-cell history. The grid stores only `val(10) & stamp(6)` per cell
  (`y4 & u3 & v3`), 64×32 cells addressed as a plain `{row(5), col(6)}`
  concatenation, so there is no `row*COLS` multiply. 8 EBRs, canonical 1W1R.
- **Swing**: `age = frame_cnt - stamp` indexes a 32-entry damped-ring curve
  `CV = 256*(1 - e^(-t/5)*cos(2*pi*t/13))`. Values >256 are an overshoot past
  flat: the squash folds back (`512 - cv`) and, past `C_FOLD_HYST`, the black
  back of the disc comes into view. The hysteresis stops the curve's ~2% tail
  ripple (t=17..22) from flashing the back face for an invisible squash change.
- **age → curve table** rebuilt every frame in vblank by a plain accumulator
  (`acc += spd`) — 64 adds, no multiplier, and the Clack rate is folded in. The
  pixel path is left with one lookup.
- **Shape**: `dy^2 < s^2*(r^2 - dx^2)` (disc) / `dy^2 < s^2*r^2 and dx^2 < r^2`
  (square card) — both share one multiply. `dy^2` advances incrementally at each
  h edge (adds only), so no line-rate multiplier exists for STA to time.
- **Update path**: row R is sampled at its centre line, snooping the pixel
  path's OWN grid read for the old state (the address is already exactly
  `{R, c}` at the cell centre), then burst-committed into grid row R during the
  h-blank of row R+1's first line — the raster has left row R by then. A Refresh
  mask (`(row + fcnt) and mask = 0`) gates which rows re-evaluate per frame.
- **Stamp pinning**: on a refresh with no change, a settled disc (age ≥ 47) is
  re-stamped to hold age at 47, so age can never run past the 6-bit wrap before
  the next refresh (≤ 16 frames) and fake a spurious re-flip.

## Timing

Routed, verified by re-running nextpnr at each accepted seed and reading the
LAST "Max frequency" line — **not** the build script's displayed figure, which is
the pre-route estimate and overstates this design by up to 17 MHz.

v0.2.1 (all seed 1, zero retries — every config passed the post-route gate
first try; HD routed figures re-verified by standalone nextpnr on identical
netlists):

| Config | Fmax (routed) | Fmin |
|---|---|---|
| HD Analog | 77.60 | 74.25 |
| HD HDMI | 76.62 | 74.25 |
| HD Dual | 80.25 | 74.25 |
| SD × 3 | pass (gate 27) | 27 |

v0.1's HD Dual margin was 0.29 MHz; the v0.2 seed sweep (18 runs, 3 HD configs
× seeds 1–6) showed seed 1 gives ≥ 2.9 MHz margin on all three, so no custom
base SEED is needed. Same-netlist routed spread across seeds remains huge
(hd_dual: 66.3–83.6), reconfirming the sweep-before-optimise rule.

~5,580 LCs (73%), 10 BRAMs (8 grid + 1 rowbuf + 1 curve table). No divider.

## Lessons

- **Seed variance dwarfed every optimisation.** HD Analog routed 72–86 MHz
  across seeds 1–7 on the *same* netlist (seeds 1–2 fail, 3/5/6/7 pass). Three
  successive "fixes" moved the routed number 73.9 → 71.7 → 73.6 → 72.3 — all
  noise. **Measure the seed spread before optimising**; the placement estimate
  (76 → 87 MHz across those same edits) is a poor predictor of the routed result.
- **The displayed Fmax is the pre-route estimate** (`parse_build_stats` takes
  `head -n1` of "Max frequency"). The routed number is the later `Warning:` line.
  The retry loop greps `"Max frequency.*FAIL"`, which *does* catch the post-route
  failure — but it accepts the last seed best-effort if none close.
- **Real wins came from removing logic, not restructuring it**: deleting the
  per-pixel `age*spd` multiply (folded into a vblank-built table) and splitting a
  serial multiply→add→clamp so the add sits *parallel* to a multiply. Splitting
  the `s2*t` multiply into half-width products raised the estimate but not the
  routed result.
- **Chroma code 0 is not neutral.** Expanding 3-bit chroma by bit-replication
  (correct for unipolar luma) yields 0/146/292/438/585/… — no code is 512, so
  every grey came out tinted; chroma is signed about 512 and needs a plain shift
  (`u3 * 128`) plus rounding. The same trap made the grid's all-zero reset word
  render as a **saturated green wall** — caught in sim, fixed with an explicit
  neutral-chroma reset constant.

## Status

v0.2.1 built + packaged 2026-07-22 (`out/rev_b/ron/clacker.vmprog`).
**Sim-verified**: Colour mode reproduces the input's colours with neutral greys;
Luma Mod renders the image purely as tilt; power-on frames 2/3 show discs
squashed into ellipses and flipped onto their black backs mid-swing; frame 50 on
a STATIC image shows one row mid-tumble under perpetual Clack (v0.1 would be
frozen flat); square cards + bezel grid render clean. The 1080i path can't be
exercised by the (progressive) image sim — verified by construction against the
videosky/fireworks patterns; **check disc roundness on a 1080i output first**
when HW testing. **Not HW-tested.**

Sims: `sim_000_input.png`, `sim_001_colour.png`, `sim_002_lumamod.png`,
`sim_003_midswing_backface.png`, `sim_004_perpetual_wave.png`,
`sim_005_squares_bezel.png`, `sim_006_green_edge_bug.png` (v0.2 repro),
`sim_007_green_edge_fixed.png`.
