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
  (12 / 10) that share the outermost chained offset: fills more screen for
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
*pre-edge* `rrom_a`, so a normalise issued at T gives `hn` at T+3 and
`rrom_q` at T+4. Reading both in the same cycle silently used the *previous*
knot's reciprocal on every divide — distances, the gap, and the slope.

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
sqrt — loaded at T, readable at T+15; `p_norm` — `hn` at T+3, `rrom_q` at T+4.

## Throughput is a hard per-line budget

Running N lines ahead does **not** buy throughput. The pass is triggered by
the active-line rise, so if it has not finished it misses the next trigger and
that line's bank is never written. The measured backlog for an over-budget
engine reaches 40–70 lines, far past any bank count — so the frame has to fit,
full stop.

The knot frame is **29 cycles**. Getting there took dropping the sub-pixel
distance correction (measured worth 1.4/255 of mean error for 8 cycles a
knot), pipelining the previous piece's slope divide into the next knot's sqrt
shadow, and folding `d_out` and the gap into one cycle. The ring count then
follows from the budget, since the floor is 2 knots per run and 2·C_NT runs.

| | worst pass | line period | margin |
|---|---|---|---|
| HD (C_NT=12) | 1727 clk | 2200 clk | 21 % |
| SD (C_NT=10) | 1437 clk | 1716 clk | 16 % |

**Timing shaped the architecture twice.** The radix-4 sqrt (two chained 27-bit
subtracts per cycle) measured **18.2 ns / 55 MHz** — it was the design's
critical path, and radix-2 is what pays for the 29-cycle frame and the ring
count. The next-knot chooser then took over as the critical path at 18 ns, so
its compare/mux/add chain now runs across three states inside the sqrt's idle
window (the next knot's x does not depend on this knot's `f`, so it can).
Earlier, `p_norm`'s priority encoder + barrel + clamp in one cycle measured
22.4 ns; it is now three pipeline stages.

## Model vs exact field
`nacre_model.py --image` renders the engine's own seed stream and diffs it
against the analytic field:

| scenario | mean | p99 | max |
|---|---|---|---|
| concentric bullseye | 7.6 / 255 | 42 | 99 |
| dragged nest | 9.7 / 255 | 54 | 141 |

That is the same band the design-study prototype reached
(`../cascade/percircle_int2.py`, mean 8.2). The residual is edge quantisation
— the engine's circle edges are integer `isqrt` — plus linear interpolation
between knots. `model_check.png` and `model_drag.png` are the sheets
(engine / analytic / error ×4).

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
  decimation 1 the line is 1984 clk against the engine's 1727.
- TOML defaults apply in sim: S7/S8/S10 default ON, K4 defaults non-zero.
