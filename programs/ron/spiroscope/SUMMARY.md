# Spiroscope (v5.1 — vector renderer)

**v5.1** (HW feedback: "not enough linework, lines not clean, want thickness
knob"): slots 7→8 per line (+14% crossings kept; 10 slots packed to 30/32 EBR
— congestion cliff — so 8 = ~21 EBR was the ceiling); **K6 = Thickness 1..8 px**
(display-only: horizontal mask widening + a 3-row interval cache that stamps
each row's intervals onto its neighbours — vertical dilation with zero extra
store reads; live-tweakable, no redraw; draw speed is now a fixed rate);
Trail default up to 600; fixed an SD bug (fixed 120-word stamp-bank clear
overran SD's hblank — now clears to raster width).

A pure spirograph **vector renderer** at full frame resolution — crisp 2-px
strokes at up to **1920×1080**, raster-adaptive (720p/1080i/1080p/SD via the
firmware timing ID in `registers_in(8)`). The etch-a-sketch mode of v1–v4 was
dropped per user direction; v1–v4's framebuffer approach was abandoned because
BRAM caps any framebuffer at 320×180 — never crisp.

## Architecture

The curve lives as horizontal **intervals bucketed per scanline**:

- **Store** — 1080 rows × 7 slots × 14 bits (x:11 + run-width:3) = 27 EBRs,
  single 1W1R, read data registered (wide-EBR-cascade lesson). Empty = x 2047.
  Address = row×7 + slot (shift-sub).
- **Pen** — walks the epicycle continuously (shared sin/cos LUT, 8-bit angle
  + 2-cycle reads; one shared 11×10 multiplier, operand@k → product@k+2).
  The walker merges consecutive same-row pixels into **runs (≤8 px)** — this
  is what keeps near-horizontal curve sections (lobe tops) from flooding a
  line's slots — and inserts each run (first empty slot).
- **Eraser** — re-runs the exact same curve W substeps behind the pen and
  removes its runs. Determinism is exact (same phases, LUT, walker, merge
  rules → identical intervals), which REQUIRES frozen geometry: K3/K4 are
  32-zone quantized (like discrete pen holes), and any geometry change
  (gears/arm/phase/harmonic/kaleido) triggers a store **clear** + fresh
  redraw. W = trail window (slider) — the geometry animation period.
- **Paint (S7)** — eraser off; ink accumulates (7 runs/line capacity, nearby
  ink deduplicated). Gear changes while painting LAYER figures (no clear).
  Slider slammed full = erase (any mode).
- **Display** — per hsync (all within hblank): clear the stamp bank (120
  words × 16 bit; bank = target-row parity) while reading the row's 7 slots,
  then stamp each interval as a (w+2)-px mark via 1–2 word RMWs. The beam
  reads the other bank through a 3-stage pipeline (C_LATENCY=3).
- Read-port sharing: display scan has priority; the plotter's scan aborts and
  retries. Interlaced modes share the store between fields (y computed in
  field coordinates, vertical amplitude halved via `>>10` instead of `>>9`).

## Gears / controls

inc = (zone+1)·128 + 3 — near-integer ratios lock recognizable m:n rosettes;
the +3 detune precesses them slowly. An adaptive speed shift (by gear-zone
sum) keeps chord lengths uniform. 18-bit phases.

| Ctrl | Function |
|------|----------|
| K1/K2 | Gear A / B (16 zones, labelled 1–16) |
| K3 | Arm R2:R1 (32 zones) |
| K4 | Phase / twist (32 zones) |
| K5 | Colour (8 palette) |
| K6 | **Line thickness** 1..8 px (display-only, live) |
| Slider | **Trail window** (hero); slam = erase |
| S7 | Paint mode |
| S8 | Harmonic (R3 = base/4) |
| S9 | Kaleidoscope (4-fold, mirrored intervals) |
| S10 | Rainbow (16-hue pinwheel: octant + ring + rotation) |
| S11 | Background black / live video |

## Status

v5.1 built + packaged; all HD configs **verified post-route at the packaged
seeds**: HD Analog 86.0 (seed 1), HD HDMI 78.8 (seed 1), HD Dual 86.6 MHz
(seed 4); SD configs pass trivially. 23/32 EBRs, ~5690 LCs. **v5.0 core not
hardware-validated; v5.1 additions not hardware-validated.**

v5.1 timing splits (each found from the actual nextpnr path report):
mask generation split into ones-barrel + position-barrel states; slot-select
split from x-validity check; the arm radii (zone×base/32 multiply + R1
subtract) moved out of the vsync latch into free-running register stages
(vsync-gated cones are still STA-timed — ziffern lesson).

Note: the store was originally 1080 rows (27 EBR, 30/32 total) — that sat on
the congestion cliff and made every config a seed lottery; dropping to 720
rows with 1080p row-pairing moved everything to comfortable margins.

## Timing lessons (v5 bring-up: 47 → 68.7 → 75 → 82 MHz)

1. `(cnt_h − cnt_t) > window` as an FSM idle condition = two chained 20-bit
   carry chains gating everything → 47 MHz. Fix: maintain `lag` as an
   up/down counter and register the comparison into a `tail_due` flag.
2. The teleport cone (abs-subtract → >48 compare → walker-init muxes) in one
   state → 68.7 MHz. Fix: split into a distance-register state + decision
   state.
3. The walker's extend-or-emit cone (role mux → add/compare → run-update
   muxes) → ~75 MHz marginal. Fix: register `f_extr`/`f_extl` flags in a
   dedicated state.
4. The display stamper's mask barrel-shifter fed straight from the slot mux
   → 71 MHz on hd_analog. Fix: register x/w in the select state, compute
   masks in their own state (the RMW read is in flight anyway).
5. As always: register wide EBR-cascade read data (`s_st_rdata_r`) before
   comparing, and give the sine LUT only 8 angle bits (chord deviation from
   the true arc at 256 steps/rev is < 0.05 px — invisible).
6. build_programs.sh now RETRIES on post-route timing misses ("seed N missed
   timing") — but acceptance on the LAST retry seed can still be a
   best-effort miss, and nextpnr is nondeterministic run-to-run on the same
   seed. Verify the accepted seeds manually; if a config needs >2 seeds,
   treat it as a margin problem and shave the critical path instead.
