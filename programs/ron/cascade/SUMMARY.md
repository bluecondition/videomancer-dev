# HYPNOS CASCADE — coupled concentric circles (v1.0: 96/48 discs, adaptive ring count, Catch-Up actually centres, S11 = video saturation mod)

Concentric black/white circles that **push each other**. Move the centre (K1/K2)
and the innermost circle leads; each outer ring holds still until the one inside
closes its slack, then is shoved rigidly outward — a wave of motion travelling
outward ring-by-ring **through a 96-ring chain at HD (48 at SD)** (the outer
field follows as one body once the last modelled ring is pushed). The rings are
hard-coupled and **never overlap** — a band of the opposite colour always
survives — and every edge is an **exact integer circle**. When you stop, they
either glide home concentric (**Catch-Up = Home**) or freeze in the dragged nest
(**Stay**).

A dedicated fork of **Hypnos** (v0.3 of the main program is untouched).

## v0.9 — the whole screen cascades, and Catch-Up lands

Two user reports, plus one defect found while fixing them.

### 1. Catch-Up now actually centres

The settle pull was `d + floor(-d/8)`, which is `d - ceil(d/8)`: exact for
`d > 0`, but for `-7 <= d <= -1` `ceil(d/8)` is **0**, so it was a **no-op**.
And the sign is not symmetric in practice — dragging the centre right/down
makes every ring lag *left/up*, i.e. `d` **negative** — so the common case
froze each ring up to 7 px off its parent and stopped. That is cumulative down
the chain: ring 23 settled **161 px** off centre while ring 0 (pinned to the
glided centre) sat exactly right, which is what "the centre circle isn't lined
up with the other rings" looks like.

Fixed by pulling the step's *magnitude* down by `ceil(|d|/8)`, rounding away
from zero on both sides, so every ring lands on exactly 0 from any drag
direction. Verified against a bit-exact model of the chain (`chain_settle.py`
method: seed the per-ring steps, iterate the real octagon/rotate arithmetic,
read the fixed point) — old: `(-7,-5)` per ring from a left/up seed; new:
`(0,0)` from all four quadrants.

**Catch-Up is a WAVE, so its duration scales with the ring count.** Ring *k*
can only close once the one inside it has, which costs ~7 frames per ring at
`C_SET = 3`. Measured in the chain model at spacing 30: 168 frames (2.8 s) at
24 rings, but **672 frames (11.2 s) at 96**. Quadrupling the rings quadruples
the glide home, which would read as broken. `C_SET` dropped 3 → 2 (pull
`ceil(|d|/4)` per frame) to put 96 rings back at 294 frames (4.9 s), the same
few-seconds character v0.8 had. `C_SET = 1` would be 105 frames (1.8 s) —
snappier than v0.8 — if that reads better on hardware.

**RTL-verified**: a 160-frame NTSC ÷1 run (SD config, 48 rings, `C_SET = 2`,
Glide off so the centre jumps once and the chain lags fully) settles to an
*exact* bullseye — centre-row edge gaps all exactly 30 px (the spacing), 59
across the centre disc, edges at 0/30/…/330 | 389/419/…/689 → centre 359.5
against `cx` = 360. v0.8 would have frozen every ring 7 px off its parent.

**126 is the verifiable ceiling** (raised from 96, 2026-08-02, "all the way
to the edges at high period"): hardware allows 142 (15·N+56 ≤ 2200) but the
÷1 sim line is only 1984 clk → N ≤ 128, and 127 is the widest count the
7-bit ring-index signals can hold with their C_NR sentinel — so 126 (even,
for mark parity). Corners (1102 px) covered at spacing ≥ 9 px, was ≥ 12.
Catch-Up cost: the settle wave is ~3 frames/ring at C_SET = 2, so home from
a full drag is now ~384 frames (6.4 s), was 294 (4.9 s); C_SET = 1 would
roughly halve it if that reads too slow.

### 2. Ring count 24 → 126 (HD) / 48 (SD)

**Ring count is a throughput question, never a fit one.** Measured LC across a
sweep at fixed everything else:

| C_NR | 24 | 48 | 64 | 96 | 110 |
|---|---|---|---|---|---|
| LC | 7080 | 7108 | 7091 | 7091 | 7056 |
| EBR | 4 | 4 | 4 | 4 | 4 |

Flat, because the chain scan, the span pass and the pixel path are all
serialized or constant-cost and the offsets live in EBR. The real bound is the
per-line pass — E_SPAN (15 slots/ring, launched at the active rise, spilling
into hblank via `l_pend`) + E_LINE (46) — which must be back in `E_IDLE`
before the **next** active rise or that line's pass never launches:

> `15*C_NR + 56 <= clocks_per_line`

| | prog_clk | clk/line | max rings |
|---|---|---|---|
| HD | 74.25 MHz | 2200 | 142 |
| SD | 13.5 MHz | 858 | 53 |

**SD is 858 clocks, not 1716** — `prog_clk` is `i_vid_dec_clk`, the 13.5 MHz
decoder pixel clock (`GEN_DIRECT_PROG_CLK`); the 27 MHz PLL feeds the *encoder*
only and the 27 MHz build constraint is pure margin. (An earlier note claiming
1716 was wrong and cost a wasted pass at 96 rings on SD.)

Chose **96 / 48** for margin (HD 1496/2200, SD 776/858). What it buys: the nest
reaches the HD screen corners for any spacing ≥ 12 px instead of ≥ 46 px, so
the cascade wave travels the whole picture rather than a disc in the middle.

Chain words stay 15-bit — `|e[k]|` is bounded by the **centre's own travel**
(~10.2k worst case at W = 4095), never by `k*A`, so the 640 px slack cap does
not have to scale with the ring count. Index words are 7-bit (`t_kl` is mod
128), so `C_NR <= 126` is the hard ceiling.

### 3. The radius clamp was inverting the screen (pre-existing)

Ring *k*'s radius is `(k+1)*spacing`, clamped at `C_RCL` = 4095. At coarse
spacings every ring past `C_RCL/spacing` **piles up at the same radius** and
they all write their edge mark to the **same line-buffer cell**. The cell holds
one mark, so the pile collapses to a single toggle and `pile-1` marks vanish —
and when that count is odd the mark parity **inverts for the rest of the line**:
a hard vertical seam with everything beyond it colour-flipped. It hits **54 %**
of the Period range (whenever the pile edge lands on screen, i.e. HD with the
centre knob in its outer ~18 %). Present in v0.8 at 24 rings over 40 % of the
Period range; going to 96 would have widened that to 62 %.

Fixed by making the modelled count **adaptive**: a running radius in the chain
scan finds the largest **even** count whose outermost ring still fits under
`C_RCL`, and the outer field seams onto *that* ring instead of ring `C_NR-1`
(`s_nreff`, `s_racc`, `s_ocap`). No ring is ever clamped, so the parity is
exact — and the per-line span cost now scales with the spacing rather than with
`C_NR` (default Period spends 26 rings, not 96). Cost: ~63 LC.

## v4.1 — 24 discs, ~87 % slack (octagon clamp)  *(historical — see v0.9 above for the current ring count)*

Per user request: 24 modelled discs, and rings should nearly TOUCH before
yielding — the minimum edge-to-edge gap at the push point is now **~12 % of
the ring spacing** (v3: 41 %; the first v4 misread the request and shrank the
slack instead — lesson recorded).  That much slack overlaps rings on diagonal
drags with a per-axis box clamp, so the chain clamp is an **OCTAGON**
(per-axis ±A plus 45°-rotated ±A·√2, A ≈ 0.875·spacing, computed per frame as
pure shift-adds; the coupling scan is fully serialized through one shared
step/clamp unit, 14 vblank phases per ring, with the clamp bound
pre-registered — single-cycle versions of both the slack decode and the clamp
cone were the critical path).  A 640-px slack cap keeps offsets inside the
15-bit chain words.  24 discs did not fit the v3 architecture (per-ring
comparators cost ~60 LC each; 24 rings × 8 engine slots = 208 clks > every
hblank), so:

- **Render — 1bpp mark line buffer.** The span engine writes each disc's two
  scanline edges as single-bit **marks** (one EBR bank per line, double-banked);
  the pixel path is ONE running-parity XOR over the marks. Strict nesting
  (guaranteed by the chain clamp) makes edge-count parity equal the innermost
  disc's parity (C_NR even). Discs overhanging the left screen edge toggle a
  line-start init bit; off-right edges are simply dropped; the displayed bank
  is cleared 2 px behind the read; a vblank pass wipes both banks. Only disc
  23 keeps equality toggles — its exact in/out is the handoff to the outer
  tracker. Disc count is now nearly free: pixel-path cost is constant.
- **Engine runs a line early.** The span pass for line N runs during line
  N−1's ACTIVE video (chain RAM, shared multiply, sqrt units are all idle
  then — only the adds-only tracker runs per pixel). ~210 clks fit even SD's
  720-clk active line. The tracker line-seed (E_LINE, now ~50 clks) stays in
  the hblank. Frame startup: vblank chain E_CLEAR → E_SPAN → E_LINE.
- **Outer field** (rings past the nest): unchanged v3 incremental r² band
  tracker, seamed exactly onto the last modelled disc.
- **Timing fixes for 74.25 MHz**: E_LINE bracket corrections retimed to 2
  slots each (compare signs registered before the update — kills a 29-bit
  carry → enable-cone path), and the per-pixel tracker no longer maintains
  the 12-bit ring index: crossings toggle a 1-bit **parity flop** (t_p); the
  full index lives only in the hblank seed. Without these, no seed passed.


## v1.4 — saturation mod removed; Video mode = pure refraction

Once the refraction landed the user dropped the saturation mod ("we don't
need it anymore"). S11 Video now passes the picture through **bit-exactly**
at Wave = 0 (verified: zero differing pixels outside the ≤19 px left-edge
wrap) and P12 bends it. The multiplies, gain derivation and clamps are gone
(~140 LC back). The right-edge tail freeze found during this pass is fixed:
writes continue C_VB+2 cycles past line end repeating the held last pixel,
which also turns the left-edge wrap into a clean edge-clamp. The whole
sat-mod saga (proc-amp %, blanking gate, prominence ladder 25→50→full,
quarter-x steps) is retained below for the record.

## v1.3 — P12 Wave: ring refraction of the incoming video

In S11 Video mode, P12 bends the picture: the ring field horizontally
displaces WHERE the video is sampled, so each ring is a glass ripple and the
cascade drags the ripples through the image. Vertical detail snakes along the
rings; the saturation mod rides on top. `d = (key−128) >> zone`, clamped ±9,
via a 32-tap BRAM delay line (2 EBR, active-pixel-gated writes so line starts
wrap onto the previous line's tail — a soft ≤19 px left-edge smear — instead
of processing blanking garbage into a coloured stripe). Depth 0 realigns
**pixel-exactly** (sync tap C_VB+5 = 15, found by measurement, ±1-cycle
theory was wrong twice). Hard rings shear in slices; S9 Smooth gives true
waves.

**Paying for it (was 102 % LC):** sat gain quantised to ¼× steps
(`A = (key+16)/32`, 0..8, endpoints/grey still exact — 9 levels is under the
saturation JND), chroma recentre `u−512` rewired as a free MSB invert,
wave depth as shift zones instead of a multiply, 32-tap line instead of 256.
Result 7575/7680 = 98.6 %.

## v1.0 — S11 Video: the rings saturate the incoming picture

S11 no longer picks Stepped/Smooth shading (that was the multi-ring fade, and
the user asked for the switch back — K5/K6 are still free if it is wanted
again). It now selects **Video: Saturate**: the rendered ring field, whatever
K4/S9 have made of it, scales the input's **chroma about neutral** —

| field | saturation |
|---|---|
| black | **0x — fully decoloured** |
| mid-grey | **unchanged** (1.0x) |
| white | **2x** |

(±25 % was HW-approved but subtle; ±50 % still read as weak. Multiplicative
saturation only shows on already-coloured pixels — near-grey source pixels
stay near-grey at any gain, and vivid ones clip — so the endpoints must be
EXTREME to be prominent. 0..2x is the full physical range.)

The user's "25 / 50 / 75 %" is the **proc-amp reading, where 50 % is normal** —
NOT a fraction of the source. Reading it as 0.25x..0.75x caps the picture at
three-quarters saturation, so the whole image looks washed out and can never
be boosted; that shipped once and was rejected on hardware. Grey must be a
true no-op.

Luma is passed through untouched, so the circles are never drawn — you only
see the picture breathing between washed-out and vivid as the cascade moves
through it. Every ring look still applies: hard rings give a two-level
25/75 split, K4 steps give banded saturation, S9 Smooth gives a continuous
sweep.

Costs **one 11×8 multiply per chroma component**: `m = (key+2)/4`
(0..64, exact at black/grey/white) and `c' = (c·m)/32`. Folding the whole
gain into `m` means no separate `c/2` term and no carried copy of `c`, which
paid for the clamp that boost needs (1.25 × 512 = 640 overflows). The input is
realigned to the field through a **BRAM delay line** (2 EBR, RAM 4 → 6)
rather than 4 ranks of flops — at 93 % LC the flops had nowhere to go.
Latency 6 → 7.

**Sim-verified**: 74.8 % and 125.1 % of source at the field's black/white
endpoints, neutral grey in gives ONE colour out (no invented chroma), luma
passthrough within 1 LSB and hue shift under 0.4 deg mean — a pure saturation
change. Measure boost with a source that has headroom: a fully-saturated test
image clips at the gamut and reads 112 % instead of 125 %, which looks like an
RTL error and is not one.

**Fit is now 98.9 % LC** (7582–7601 / 7680) — all six still close on the
builder's own seeds, but there is no room left for another feature without
reclaiming base LC first.

| Config | Fmax | margin | seed |
|---|---|---|---|
| HD Analog | 75.85 MHz | +2.2 % | 1 |
| HD HDMI | 76.92 MHz | +3.6 % | 1 |
| HD Dual | 75.83 MHz | +2.1 % | 3 |
| SD Analog | 77.15 MHz | — | 1 |
| SD HDMI | 77.68 MHz | — | 1 |
| SD Dual | 67.47 MHz | — | 1 |

**HD Analog and HD Dual are at ~2.2 %, under this program's 2.7 % rule.** Far
better than the 0.2 % build that showed rough ring edges on hardware, but if
edge roughness or noise lines reappear, seed-sweep those two first.

**Trap paid for here:** `resize(SIGNED, 10)` keeps the sign bit and drops bit
9, so every chroma result above 511 folded back down (896 → 384) and the
screen came out a saturated GREEN. Slice `v(9 downto 0)` when the value is
provably in range; do not `resize` a signed down to its exact unsigned width.

## v0.2–v0.4 — K4 GRADIENT width + S11 Shading (Stepped/Smooth)

The marks now carry an enter/leave **direction bit** (the EBR banks' free
second bit), so the pixel path counts the TRUE ring index k (5-bit, mod 32;
line-start index seeded by the engine's overhang count; the outer tracker
keeps a matching mod-32 counter).  In **Stepped** mode EVERY
RING carries its own ramp-in/ramp-out (dark at both edges, bright
mid-ring): a second 16-bit phase runs at ring period (Q1 = 2^16/spacing
per radial pixel) and is **RE-ANCHORED TO ZERO AT EVERY TRUE DISC EDGE**
via the mark buffer, so each ring's shading is welded to its own moving
edges and cascades with them.  K4 sets the FINENESS: zone 0 = the classic
hard black/white, zones 1–6 = the ramp quantized to **2 / 4 / 8 / 16 / 32
/ 64 gray steps within each ring** (top bits of the ramp, bit-replicated
to exact 0..255).  Timing subtleties: the mark data trails the crossing
detect by one cycle, so the crossing direction is REGISTERED and applied
together with the mark gate (one fewer output delay reg keeps latency 6);
a mark landing exactly at x=0 is invisible to the pixel path, so the
engine flags it (m0_done) and the line seed snaps to 0.  Known mid-drag
artifact: between edges the ramp advances by single-centre radius, so
scanlines near a dragged ring's tangent hold-then-snap (horizontal
streaks); exact when concentric/settled.  **S11 Smooth** = a continuous ramp fading evenly
black↔white over the K4 width: a 16-bit PHASE accumulates ±Q =
2^16/(2w·spacing) at every integer-radius crossing and its top bits fold
straight into luma — a per-pixel-smooth radial gradient with zero per-pixel
multiplies, anchored at the outer centre (exact in Uniform and settled
states; during an active drag it glides with the chain rather than
squishing per ring — per-disc smooth distance is not computable on this
chip).  S9 Invert applies to both.

The machinery that made Smooth affordable also REPLACED the outer-field
renderer: an integer-RADIUS tracker (the r² bracket tracker at spacing 1 —
its taps degenerate to +4/0/+2 and 17-bit arithmetic) detects every radial
pixel crossing; `r mod spacing` and ring-index counters ride the same
crossings.  The 28-bit r²-bracket tracker, its candidate arms and the
squared-bracket line seed are GONE (the line seed is now a linear ±s
ladder), and the disc-23 hit comparators fell out too (the crossing counter
itself knows "outside all discs").  Leanest cascade build yet: **88 % LC**.
Funded by dropping E_SPAN to ONE sqrt unit (15-slot ring cadence, ~370 clks
during the previous line's active video — the line-early scheme had the
budget).

**Verified pixel-exact** against an integer-exact Python model — 0
mismatching pixels in nine scenarios (uniform bullseye, concentric nest,
dragged nest, per-ring stepped ramps at 16/64 steps concentric + 64 steps
dragged, 2/32-ring smooth fades, smooth fade on a dragged nest) at 480×270
sim raster.

## Per-circle gradient — moved to `nacre`

The natural next step for this program ("each ring's gradient should squeeze
and swell with the ring it sits in, measured against the circle nested
inside it") does **not fit here**: the knot engine it needs measures ~4350
LC, and a v0.9 cascade carrying it synthesised to **11454 / 7680 LC (149 %)**
even after dropping to 16 rings. It now lives in `programs/ron/nacre/`
(work in progress). This file is unchanged v0.8 behaviour.

The design study for it is in this directory and is worth keeping:
`percircle_int2.py` (the validated integer model), `shading_compare.png`,
`shading_settled.png`, `shading_percircle.png`, `percircle_lin.png`.

### The blanking-interval bug (three hardware rounds to find)

S11 Video showed rail-to-rail green/magenta garbage + noisy sheared lines on
hardware while ring mode was pixel-perfect — and every verifiable layer
passed: GHDL progressive AND 1080i5994 interlaced sims bit-clean, the exact
multiply/clamp expressions synthesised through ghdl+yosys and exhaustively
equivalence-tested on the netlist (34,816 vectors, 0 errors), icetime
sign-off PASSED on all three HD bins. The fault was in the one place no sim
checks: **the blanking interval**. The video pipeline free-runs, so outside
avid it emitted PROCESSED blanking data — input blanking chroma 0 scaled by
0.75 → 128 sitting where the encoder's per-line colour reference lives.
Corrupt that reference and every ACTIVE pixel decodes wrong (whole-screen
colour garbage, unstable lines) even though each one leaves the FPGA
bit-correct. Ring mode never hit it because its chroma is a constant 512;
the gain-1.0 diagnostic didn't either because 0 in → 0 out reproduced the
input's own blanking. Fix: in Video mode, gate y/u/v to 64/512/512 whenever
delayed avid = '0' — make blanking look exactly like the proven ring mode.

**Rule for every future video-processing program: what you emit during
blanking is part of the interface.** Sims only score the active region;
neutral-gate your outputs outside avid.

### Two config traps this cost a hardware round

**`program_type` must be `processing`, not `synthesis`.** The docs define
synthesis as "generates output without input", and the host uses the byte at
program_config.bin offset 7916 (0 = processing, 1 = synthesis). Cascade was
born a pure generator and still declared `synthesis`, so no input video was
routed to it — chroma sat at **code 0, which is not neutral but saturated
GREEN**, and the ring field scaled it into green/tan rings instead of
modulating a picture. Any program that reads `data_in.y/u/v` in ANY mode must
declare `processing`.

**The packer silently reuses a stale `program_config.bin`.** It is generated
from the TOML by `tools/toml-converter/toml_to_config_binary.py`, NOT by the
packer, and the packer will happily package an old one — so new labels never
reach the hardware while everything reports "Validation PASSED". Symptom: the
switch on the unit still shows its previous name. Fix: delete
`build/programs/<v>/<p>/{,rev_b/}program_config.bin`, regenerate, repack, and
grep the binary for the new strings before believing it.

## Control map (unchanged)

| Phys | Control | Range / detent |
|---|---|---|
| K1 | **Center X** | bipolar ±2 W, detent = frame centre |
| K2 | **Center Y** | bipolar ±2 H, detent = frame centre |
| K3 | **Period** | ring density (exp), spacing 4..2048 px |
| S7 | **Glide** | one-pole slew on the centre (drives the cascade). Default ON |
| S8 | **Cascade** | Uniform (classic bullseye) / Cascade (sequential push). Default ON |
| S9 | **Ramp** | Steps (K4 fineness, classic hard at 0) / Smooth (full-res per-ring ramp, K4 ignored — the soft gradient cascades with the rings). Invert is gone (v0.8) |
| S10 | **Catch-Up** | Stay (freeze dragged) / Home (glide back concentric). Default ON |
| K4 | **Gradient** | fineness: classic / 2..64 steps within each ring. Default classic |
| S11 | **Output** | Rings (generator, as always) / **Video**: the incoming picture, colour untouched, REFRACTED by the rings (P12 Wave). Default Rings |
| P12 | **Wave** | refraction depth: the ring field horizontally displaces the video like glass ripples (S11 Video only). Off at bottom; 3 zones up to ±9 px. Slider per the primary-control rule |
| K5, K6 | **spare** | |

## Simulator notes (cost hours — read before simming)

- **THE SIM PICKS AN HD CORE CONFIG FOR HD VIDEO MODES.** `--video-mode`
  selects the config, so the default `1080p2997` elaborates an **HD** package
  → `C_ENABLE_HD` true → **96 rings**, not 48. Combine that with
  `--decimation 2` (line = 960 + 64 = 1024) and the span pass needs 1496 —
  **starved**, and it renders as blank alternate lines plus horizontal seams
  that look exactly like an RTL bug. It is not one. Cost me six bisect sims.
  Budget check before every run — `15*C_NR + 56 <= W/decimation + 64`:

  | raster | line | 26 rings | 48 | 68 | 96 |
  |---|---|---|---|---|---|
  | 1080p ÷1 | 1984 | ok | ok | ok | ok |
  | 1080p ÷2 | 1024 | ok | ok | **starved** | **starved** |
  | 1080p ÷4 | 544 | ok | **starved** | **starved** | **starved** |
  | ntsc ÷1 | 784 | ok | ok (8 clk spare) | **starved** | **starved** |

  So: **HD ring counts can only be simmed at `--decimation 1`.** `--video-mode
  ntsc --decimation 1` is the honest SD test (720 active, 784 line — tighter
  than real hardware's 858) and it also exercises the hblank spill.
  The adaptive count helps here: at coarse Periods `s_nreff` is small, so
  `--decimation 4` is fine at the default Period and starves at fine ones.
- **Catch-Up needs hundreds of frames.** ~7 frames per ring at `C_SET = 3`,
  ~3.5 at `C_SET = 2` — 294 frames to reach concentric at 96 rings. A
  30-frame warmup captures a still-dragged nest; that is the model working,
  not a settle bug. Verify the settle on the **SD** config (48 rings, 148
  frames) where a full run is affordable.

- **TOML defaults apply in sim.** S7/S8/S10 default ON; a "clean" scenario
  must explicitly `--set toggle_switch_7=0` etc., or the glide/settle
  dynamics contaminate the capture (settle's floor-shift walks each ring
  −1 px/frame once the centre is still).
- The sim raster is `video-mode / decimation` (default 1080p/4 = **480×270**),
  NOT the input image size; frame 1 publishes with unmeasured W/H defaults
  (720/480) — a bit-exact model must replay that transient.
- v4+ needs **no tool hblank patch** at decimation ≤ 4: E_LINE (50 clks) fits
  the fixed 64-clk sim hblank and E_SPAN (~370 clks) runs during active.
  Deeper decimation starves E_SPAN.
- The tool renders **BT.601 studio range**: png = round((Y10−64)·255/876),
  clipped.  0/255 endpoints survive, mid-grays do NOT (85 → 81, 170 → 180):
  exact-compare models must apply this mapping.
- K4 also defaults non-zero via the TOML — set `rotary_potentiometer_4=0`
  for hard-mode scenarios.

## Fit / timing

**~7.14k LC (93 %), 4 EBR** (chain x/y + 2 mark banks). All six configs pass
**icetime sign-off STA** (never trust the build's pre-route estimate).  An
early HW test showed rough ring edges + occasional full-width noise lines;
that build's hd_analog margin was only 0.2 % — since then seeds are chosen
for ≥ 2.7 % margin on every HD config as the suspected fix:

**v0.9 routed results** (7117–7148 LC = 93 %, 4 EBR, all six PASS):

| Config | Fmax (routed) | margin | seed |
|---|---|---|---|
| HD Analog | 79.25 MHz | +6.7 % | 4 (swept) |
| HD HDMI | 78.10 MHz | +5.2 % | 5 (swept) |
| HD Dual | 78.49 MHz | +5.7 % | 5 |
| SD Analog | 78.64 MHz (Fmin 27) | — | 1 |
| SD HDMI | 73.07 MHz (Fmin 27) | — | 1 |
| SD Dual | 79.05 MHz (Fmin 27) | — | 1 |

**HD Analog is seed-fragile: only 1 of 10 seeds passes** (seed 4 at 79.25;
the other nine land 62.9–73.8). The builder reported its last retry — seed 6
at **64.31 MHz** — as "✓ Completed", which is the best-effort trap: always
read the Fmax, never the tick. hd_hdmi's own run passed at 74.62 (+0.5 %,
under the 2.7 % rule) and was swept to seed 5. Both icepacked in and
repacked (`casc_sweep.sh`).

v0.8 numbers, for comparison: HD 78.6/78.0/80.2, SD 59.4/76.1/74.6.

Shipped `out/rev_b/ron/cascade.vmprog` (program v0.9; presets: Gradient
Waves = S11 Smooth, Velvet Rings = S9 Smooth) repacked from these six
bitstreams, validation PASSED. (v0.8 HD seeds were swept manually — the
builder's own run best-efforted hd_analog at 70.6 — then icepacked into
the build tree and repacked.)  (Packer caveat: it will happily pack with a MISSING
config bitstream — count 6 .bins before shipping.)  Timing lessons this round: the vblank SEQUENCER never
executes slot 0 (schedule nothing there); the single-slot spacing decode
(negate+ROM+barrel) and an un-preregistered one-hot multiplier operand were
the hd_hdmi critical paths — both now pipelined across free slots. **HW-iterating** (margin fix + octagon slack + gradient awaiting a
hardware pass; if edge roughness survives these margins, report the output
video mode). A plain rebuild may land failing HD seeds; reseed with the
scratchpad `sweep_cfg.sh`.
