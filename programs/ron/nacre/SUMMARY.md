# NACRE — concentric circles, each carrying its OWN gradient (v0.2)

A dedicated fork of **Hypnos Cascade** whose entire subject is the shading.
The cascade motion is unchanged — move the centre and the innermost circle
leads, each outer ring holding until the one inside reaches it, then being
shoved outward, never overlapping. What is new is the **fill**:

> every ring's gradient belongs to **that ring**, measured between its own
> circle and the circle nested inside it — so it **squeezes** where the inner
> circle crowds it and **swells** where the gap opens, with smooth round
> contours morphing between the two circles as the cascade moves.

Formally the field is `f = d_in / (d_in + d_out)`, the true Euclidean
distances to the two bounding circles, triangle-folded to luma (dark at both
circle edges, bright mid-ring).

## Why a fork
The per-circle field needs a knot engine that measured ~4350 LC. Cascade v0.9
came out at **11454 / 7680 LC (149 %)** with it — it cannot fit alongside
cascade's mark renderer, outer-field tracker and stepped modes even at 16
rings. Nacre deletes all three and makes the seed stream the whole renderer.

## How it works
- **E_SPAN** (unchanged from cascade) computes each ring's two scanline edges
  and writes them to a span store.
- **E_HERM** walks the line's runs in ascending x — left edges outside-in,
  the innermost present ring's central run, then the right edges inside-out —
  and evaluates `f` at **knots**: every run end, a one-ring-spacing grid, and
  **each run's inner circle-centre column** (that is where `r(x)` kinks;
  without it the central run interpolates straight across the innermost disc
  and it renders as a flat blob).
- Between knots the field is **linear**, so the pixel path is `f += d` and a
  triangle fold — one add per pixel, no multiply, no divide, no tracker.
- **C_NC** chained discs (8 HD / 6 SD) plus **rigid outer rings** to C_NT
  (10 / 8) that share the outermost chained offset: fills more screen for
  free — no storage, just engine slots.
- The engine computes line N+1 during line N — a plain double buffer. Running
  further ahead buys **latency, not throughput**, and never fixed the budget.

## The contracts that make it correct

Seven of these were violated in v0.1 and each one on its own wrecked the
field. An RTL-faithful cycle model of the engine (`nacre_model.py`) is what
found them: it reproduces every process with exact VHDL signal semantics —
each one reads the *pre-edge* value of every signal and writes a *post-edge*
value — renders the same seed stream the die does, and diffs it against the
analytic field in about two seconds. Five of the seven are one-cycle read
skews, which is precisely the class of bug a picture cannot localise.

**1. `rrom_q` trails `hn` by one cycle.** `p_norm` reads the ROM at the
*pre-edge* `rrom_a`, so `rrom_q` is always one cycle behind `hn` no matter how
many stages the unit has. Reading both in the same cycle silently used the
*previous* knot's reciprocal on every divide — distances, the gap, the slope.

**2. The numerator must not saturate.** `hn` is 15 bits and `p_norm` scales it
by the *same* exponent as the divisor, which for a short piece is a shift
**up**. Feeding it `df >> 3` saturated for every piece under 128 px, so the
slope came out ~16× too small and the field barely moved. It is `df >> 8`,
with the guard cut from Q12 to **Q6** so `d0` fits the constant `>> 3`.

**3. Every piece must span ≥ 2 px.** The pixel path's seed prefetch takes two
cycles, so two knots one pixel apart make the second reseed read a **stale**
word and the rest of the line desynchronises. Runs shorter than 2 px are
skipped and the knot chooser folds the bound into its candidate tests.

**4. Word 0 is prefetched during blanking.** Issuing its address at the avid
rise left the line-start load reading the previous line's leftover word.

**5. Write strobes are one-shot.** Without an explicit `'0'` default,
`spst_we` / `sd_we` latch high and every later cycle rewrites the same slot.

**6. Run ends clamp at BOTH raster edges.** A negative left edge propagates
into `x_next` as a huge unsigned value.

**7. The line init reads its two dy² products three cycles after issue.**
Reading them one cycle early left `hm_ody2` holding E_SPAN's last `w²`, so
every line's *first* run had a bogus outer circle. Concentric rings hid it
completely — the two circles share a centre, so the wrong value equals the
right one; it only appears once the nest is dragged, which is the whole point
of the program.

**Latency contract:** shared multiply — operands at T, `m_p` at T+3; radix-2
sqrt — loaded at T, readable at T+15; `p_norm` (4 stages) — `hn` at T+4,
`rrom_q` at T+5.

## Throughput is a hard per-line budget

Running N lines ahead does **not** buy throughput. The pass is triggered by
the active-line rise, so if it has not finished it misses the next trigger and
that line's bank is never written. The measured backlog for an over-budget
engine reaches 40–70 lines, far past any bank count — so the frame has to fit,
full stop.

The knot frame is **30 cycles**. Getting there took dropping the sub-pixel
distance correction (measured worth 1.4/255 of mean error for 8 cycles a
knot), pipelining the previous piece's slope divide into the next knot's sqrt
shadow, and folding `d_out` and the gap into one cycle. The ring count then
follows from the budget, since the floor is 2 knots per run and 2·C_NT runs.

| | worst pass | line period | margin |
|---|---|---|---|
| HD (C_NT=10) | 1945 clk | 2200 clk | 12 % |
| SD (C_NT=8) | 1467 clk | 1716 clk | 15 % |

## Timing shaped the architecture

Every one of these was measured on `hd_analog`, fixed, and re-measured; each
fix simply exposed the next path.

| critical path | measured | fix |
|---|---|---|
| `p_norm` encoder + barrel + clamp, one cycle | 22.4 ns (44.7 MHz) | split into stages |
| radix-4 sqrt, two chained 27-bit subtracts | 18.2 ns (55.0 MHz) | **radix-2** |
| next-knot chooser compare/mux/add chain | 18.0 ns (55.6 MHz) | spread over 3 states |
| centre-glide accumulator (inherited) | ~15 ns (67.3 MHz) | split across sequencer slots |
| span-store EBR read + edge clamp | 13.1 ns (75.8 MHz) | clamp registered (`sp_cl`) |
| `p_norm` barrel + clamp | 14.0 ns (71.3 MHz) | coarse/fine barrel split |

The radix-2 sqrt is what pays for the 30-cycle frame and therefore the ring
count — it costs 7 cycles a knot. The chooser moved into the sqrt's idle
window because the next knot's x does not depend on this knot's `f`. The
final `p_norm` split pre-shifts left by a constant 9 so the whole normalise
becomes a *right* shift of `nrm_sh+1` (1..18), then splits that into a
multiple of 4 and a remainder — halving the mux depth.

### Where the six configs actually stand (v0.3, post-route)

| config | Fmax | required | verdict |
|---|---|---|---|
| HD Analog | 77.95 MHz (seed 3) | 74.25 | pass, 5.0 % |
| SD Analog | 69.34 MHz (seed 1) | 27 | pass |
| HD HDMI | 77.28 MHz (seed 1) | 74.25 | pass, 4.1 % |
| SD HDMI | 77.15 MHz (seed 2) | 27 | pass |
| **HD Dual** | **61.02 MHz (best of 6 seeds)** | 74.25 | **FAILS** |
| SD Dual | 66.95 MHz (seed 1) | 27 | pass |

**HD Dual does not close.** It closed at 75.60 MHz before the round-contour
work, so the finer grid cost it — and 61 MHz is an 18 % miss, far outside
seed noise, so it needs a path fix rather than reseeding. **The packaged
`out/rev_b/ron/nacre.vmprog` therefore contains a failing hd_dual bitstream
and must not be treated as shippable** (the builder accepts a best-effort
bitstream after its seed retries; see [[build_timing_verification]]).

**Single Fmax readings prove nothing here: router2 is run-nondeterministic.**
An 8-seed sweep of the pre-split netlist returned 63.4 / 71.0 / 63.4 / 66.0 /
64.3 / 67.5 / 67.4 / 65.6 MHz — all failing — even though one earlier one-off
run of that same seed had reported 76.4 MHz PASS. Sweep, then judge from the
last post-route report.

## Round contours cost knots, and knots cost rings

The knot grid is **one ring spacing**, and the central run gets **half** that
again. Two spacings — the obvious choice, and what v0.2 shipped with — leaves
a run's only knots at its ends, so the field is linear in x across a whole
ring width and the iso-contours come out **polygonal**: the innermost disc
renders as a visible diamond, the rings as hexagons. A small test raster
hides this completely (at spacing 14 the facets are 14 px and look round),
which is exactly why it survived into a build I had called correct.

The central run needs its own step because its field is a **cone** — the
inner circle degenerates towards a point — so edge plus centre-column knots
interpolate a pyramid no matter how fine the outer grid is.

| grid | mean | p99 | clk/line |
|---|---|---|---|
| 2 spacings | 8.11 | 58 | 1777 |
| 1 spacing | 4.98 | 23 | 2017 |
| 1 spacing, central run at ½ | **3.92** | **13** | **1749** |

A ¼ step on the central run scored marginally better still but put SD over
the line budget (1723 vs 1716) and left HD at 4 %. **The ring count pays for
the grid**: at 12/10 rings the finer grid left only 4 % throughput margin,
and a missed line trigger means a whole line's seed bank is never written.

## Model vs exact field
`nacre_model.py --image` renders the engine's own seed stream and diffs it
against the analytic field:

**mean 3.9 / 255, p99 13** on a dragged nest at HD-like scale — better than
the design-study prototype (`../cascade/percircle_int2.py`, mean 8.2). The
residual is edge quantisation: the engine's circle edges are integer `isqrt`.

**Test at the scale the program runs at.** The model is only as good as the
geometry you feed it — a 480×270 raster at spacing 14 rated the polygonal
build at mean 7.6 and looked round, and a concentric nest hid the stale-dy²
bug entirely. `diag_centre.png` / `diag_fixed.png` are the before/after
crops of the innermost disc.

## Controls
| Phys | Control |
|---|---|
| K1 / K2 | Centre X / Y |
| K3 | Period (ring spacing) |
| K4 | Steps — quantise the gradient to 2..64 levels (S9 = Steps only) |
| S7 | Glide |
| S8 | Cascade (Uniform / Cascade) |
| S9 | Ramp — Steps / **Smooth** (default) |
| S10 | Catch-Up (Stay / Home) |
| K5, K6, P12, S11 | spare |

## Simulator notes
- **`--decimation 1` is mandatory.** The engine's work is in *pixels* and does
  not shrink with the raster, but the sim's line period does (fixed 64-clk
  hblank), so a decimated sim starves the pass and renders garbage. At
  decimation 1 the line is 1984 clk against the engine's 1945.
- TOML defaults apply in sim: S7/S8/S10 default ON, K4 defaults non-zero.
