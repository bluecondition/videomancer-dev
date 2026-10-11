# Videomancer technique catalog

Proven rendering/engine patterns from the `programs/ron/` library (plus cicero), distilled from build logs and hardware sessions (compiled 2026-08-16). Companion to [CLAUDE.md](CLAUDE.md), which holds the always-relevant rules; this file is the design-time menu. Deep per-program history lives in the Claude memory dir.

**Status tags**: *HW-validated* (looked right on the device) · *HW-iterating* (on the device, still being tuned) · *timing-closed* (all 6 configs routed, never flashed) · *sim-verified* (image sim only) · *WIP* · *DEAD-END* (tried, failed, do not retry).
**Cost figures** are LC% of the HX4K's 7680, EBR out of 32, and routed MHz vs the 74.25 target.

When a new program proves a new technique (or kills one), add or amend an entry here.

## 3D, geometry & zoom

### Vblank BRAM-prefetch of per-frame geometry (active mesh)
A vblank copy engine resolves all per-frame variant/morph/knob selection ONCE into "active" EBRs; the per-pixel render reads single BRAM words with no muxes on the critical path. — **bishop**, HW-validated; took HD HDMI from a ~69 MHz ceiling to 86. BRAM has 2-cycle effective latency — pipeline control `_pre → _pre2` to match, or everything renders stacked at center. DEAD-END: the per-edge case-mux alternative maxed ~69 MHz no matter how pipelined.

### Active-edge-list rasterizer
During line clear, append each active edge's index to a small list RAM; the stamp phase iterates only that list — stamp cost O(active-this-row), replacing the N:1 active-vector mux that was the HD critical path (59.9 → 66+ MHz on a 347-edge mesh). — **bishop** dense-wireframe, HW-validated. At ~90% BRAM the design goes congestion-bound; the real fix was shrinking data (edges <256, BRAMs 512→256). Morph deltas belong to VERTICES, not edge endpoints — sparse per-vertex morph ROMs + an edge→slot map cut morph BRAM 12→1.

### Scanline EOR (even-odd) fill with boundary/detail decoupling
Split edges into *boundary* (writes exactly one 1 px centre crossing per row to a dedicated line buffer read by a parity walker; half-open [ymin,ymax) rule) and *detail* (visible only; thick stamps). — **bishop v2 + dense**, HW-validated. Open polylines MUST be detail (chords create phantom closures); sharp tips need a tiny flat top (split the vertex) — three flag hacks all regressed. DEAD-END: XOR/parity buffer (RMW — becomes ~4000 FFs). DEAD-END: runtime arithmetic that changes crossing counts; only selection among build-validated variants is safe.

### Q-format sized to the BRAM word
Pick edge-DDA fixed point so state fits one EBR word (Q9.7 = 16-bit; ~0.2 px roundoff invisible). — **bishop v2**, HW-validated. Q12.8 spilled to 2 EBRs and cost 7–10 MHz on every HD config from routing pressure alone — precision upgrades can cost timing via BRAM splits, not logic.

### Constant half-vector shading under an orthographic camera
View and light are world constants, so the half-vector is constant; each cube face's H·n is ±one of three whole-cube dot products — 3 dot products per FRAME. One shared H serves diffuse and specular (3 per-pixel multiplies, not 6). — **cubist**, HW-VALIDATED (v0.5.3, 2026-10-01). Keep ambient OUT of the tilt subtraction or averted faces crush black; shade in linear, gamma via a 256-entry ROM at the end.

### Microcoded geometry engine replacing a hardwired frame FSM
One 32-bit ALU + BRAM register file (2 copies for dual read) + microcode ROM, 4 cycles/instruction, replaces ~110 hardwired FSM states. Serializing the multiplier alone is NOT enough — operand muxes only die when operands come from a register file. — **cubist**, HW-VALIDATED (v0.5.3, 2026-10-01). A microcode ROM is a constant — a dummy program lets yosys const-fold decode logic and lie about area (tell: EBR count doesn't move). BRAM register file reads a cycle late — explicit states. Build the ISA emulator FIRST (seconds vs 20-minute renders). **Keep the ALU tiny**: no barrel shifters (serial shifter; MRD/DIV select fixed shift sets the assembler enforces), MIN/MAX/CLP/ABS/BIT/NEG as assembler macros, divider takes magnitudes by microcode contract (emulator asserts), right-shifting serial multiplier with the Booth sign on the last step — no negators anywhere. **Timing at 74.25 is all routing**: latch operands into fabric first; compute branch conditions in the latch into FFs; launch every unit (MUL/DIV/ROM/LDX/slot-write) in the latch straight from the RF outputs so operand A feeds only the ALU; pre-decode the result source; make every result source a fabric register; never share a ROM port with the pixel pipe.

### Microcoded PER-LINE engine feeding the pixel path (cubist's engine, extended)
Cubist's ALU + dual-copy BRAM register file + program ROM, run twice: a field program in vblank (knob deadband+glide via one subroutine, every per-field constant, OUT ports) and a line program one line ahead of the beam whose parameter block an XFER instruction copies (one word a clock, in hblank) into the pixel path's line registers AND straight into the stage-0 DDAs, so DDA start values need no holding registers (per-field port registers fed from the engine never pack with a LUT: ~490 of them cost ~490 LC). Replaced a hardwired frame+line sequencer, knob smoothers and constant logic of ~4,800 LC with ~1,200; the whole program went 134% -> 95%. 24-bit datapath; unsigned 16x12 serial multiplier and 25/12 divider (operands are magnitudes by microcode contract; the emulator asserts every MUL/DIV operand against the unit widths and flags unintended 24-bit overflow, with phase registers whitelisted to wrap). **At 94% the latch-time branch conditions were the critical path** (forwarded operand -> 32-bit OR/compare, 21 ns of routing, 43 MHz): make LATCH only forward into two operand registers and do compares, branches, MIN/MAX and every unit launch in EXEC from those registers (73 MHz), then give MIN/MAX a second EXEC clock so the select isn't behind the adder's carry chain. Budget the line program per mode (the emulator prints worst clocks/line: NTSC ~750 of 858) and sim at 720p60 dec 1 (default 480x270 gives only 544 clk/line). router2 missed every HD config on 6 seeds; router1 closed all six on seed 1. — **oracle v2.0.1**, HW-validated (signed off 2026-10-10), 6/6 router1 seed 1.

### Parallel affine DDAs for convex-polygon ownership (no line engine)
Under an orthographic camera each face's (u,v) is affine in screen x/y, so run one accumulator per visible-face coordinate (+gx per active pixel, +wrap = gy·ystep − W·gx once per line, seeded per field) and let the pixel's owner be whichever face's (u,v) is inside the square — faces of a convex solid never overlap. Replaced a span/edge-table line engine + display list + prefetcher (−1,400 LC). The adder only ever adds gx: park wrap/seed in the gx register for one step from a small BRAM (no accumulator input mux). Block-float steep faces (|g| ≥ 2^16 → count whole units) so 30-bit accumulators never wrap into phantom faces. **Needs a strict + padded (64-unit) fallback owner**: independently-rounded DDAs leave crease pixels no face claims (bright backdrop dots); a single padded test needed >32 units of steal. — **cubist**, HW-VALIDATED (v0.5.3, 2026-10-01). **Outline corners**: rounding each face's corner IN ITS OWN PLANE (arc or per-axis stretched arc) disagrees between faces on steep views and leaves spikes; seams look perfect because they are shading inside one face, never an outline cut. What shipped: crisp corners (straight silhouette distance) + a tiny 45° chamfer (du+dv, ~3 px) + drop PADDING-ONLY owner pixels in silhouette zones (the 64-unit overhang past a crease otherwise sticks out as a sliver where the neighbour's corner is cut — p_cell mirrors the distance, so the overhang reads as inside). A truly round outline needs screen-space circle tests per vertex (hundreds of LC). Check ownership against exact point-in-parallelogram with an integer model before building (.cubist_work/check_dda.py pattern).

### Normalized cell units for per-pixel marches
Run the march in units where one cell = exactly 1024 at every resolution (steps 512/a, 512/b from two vblank restoring divides): all constants become compile-time, words shrink to 15-bit, resolution-independent. — **penrose**, timing-closed (81–92 MHz); took the design from 139% LC (unplaceable) to 75%. Reach for it whenever per-pixel word widths scale with resolution.

### Isometric heightfield march on remainder accumulators
Iso coords U=b·x−a·y, V=b·x+a·y as accumulators; cell indices as remainder accumulators (no divide); parallel front→back cover test picks the nearest hit — fills the frame with no sprite-count limit. — **ziggurat** (sim-verified, marginal at full clock) reformulated in **pinboard** (timing-closed, 86% LC): with i=iv+k, cover tests collapse to `ivrem+e >= 0`, face classify to pure sign bits. March depth KF must derive exactly from max height (a "safe" KF+2 was 104% LC vs 86%). Keep everything shift/add/compare — a literal /3 or mod 3 costs ~18 ns. Per-cell gates apply to the finished value, never per-operand. The radial gate/hash must be evaluated PER marched k — sharing one across cells makes a cell's height depend on which pixel views it.

### Near-free shaded sphere
`sh = (R²−r²)>>ks − (dx+dy)>>kd` — level sets are exact circles offset up-left: dome, light direction, and (thresholded) specular from one expression; r² maintained incrementally (`sqx += 2dx+1`). — **pinboard**, timing-closed.

### Polar-silhouette ROM for arbitrary rotating solids
Model the solid offline, store the projected silhouette per elevation as a polar radius table R(θ) in BRAM: membership = one read + one compare; **rotation = a subtract on the angle index**. A combined 1024×12 table returns radius + octant sub-angle in one read — no sqrt, no divider. — **morsel**, HW-tested.

### Octagonal distance metric
`max(a,b) + min(a,b)/2` replaces squared euclidean for round shapes — adds/shifts only, and you store the radius, not its square. Removed 6 multiplies at once, the biggest single win in a 54→80 MHz recovery. — **rollthebones**, HW-validated.

### Cosine-free exact rotation: tan shear + per-object sec-scaled radius
For square/round sprites, rotate with `u = x + y·t, v = y − x·t` (t = tan θ, |t| ≤ 1): that is a true rotation scaled by sec θ, so pre-scale the object's radii by sec θ once per object (sample pipe; two selected shifts of R, ≤2.5% error) instead of a per-pixel cos multiply. Objects with 90° symmetry spin through 360° with a 10-bit angle (9 bits per quarter + a rot90 bit that transposes asymmetric patterns). Keep shading in screen space so the light doesn't spin with the object. — **rollthebones v1.0.1**, HW-validated (per-die tilt to ±45°; v0.4 used it for rolling dice, HW-rejected as a look, not a technique failure).

### One shared, operand-registered multiplier for a sparse per-object pipe
When a sample/sprite pipe carries one object at a time (strobes ≥ N clocks apart), its K multiplies can share ONE multiplier: the stage holding the strobe picks the operands (priority mux on the stage strobes), the operands are REGISTERED, the product registers next cycle, and each consumer reads it two stages after the pick. Five per-die products (tilt, jitter x/y, U/V saturation gain) went from five multipliers to one: 102% → 92% LC. Unregistered operands put strobe→mux→multiply in one cycle and became the HD critical path (74.9 MHz on the last seed); registering them gave 79-83 MHz on seed 1. — **rollthebones v1.0.1**, HW-validated.

### Find the LUT multipliers first; merge time-disjoint ones, serialize the rest
On the HX4K every `*` is a LUT array (14×14 ≈ 490 LUT, 15×15 ≈ 570). Stub each one in an ablation (`a*b` → `resize(a) xor resize(b)`, which keeps downstream logic live) before hunting anywhere else: in cascade the two multipliers were **~1060 LUT, 14 % of the chip**, and nothing else came close.
1. **Merge users that never overlap.** The vblank sequencer copies its operands into the engine's multiplier while the engine is idle (product +1 clock; the sequencer stalls if the engine isn't idle). −495 LUT, bit-exact.
2. **Serialize.** A shift-add unit (signed multiplicand × UNSIGNED 14-bit multiplier, one 17-bit add per clock, 14 steps) fits a 15-clock per-ring cadence when you pipeline three rings per group: stage k−2's edges, root k−1's product, multiply k. Load it from a next-operand register (request at T, product at T+16); arithmetic in front of the load mux was the critical path. Every user must arrange a non-negative multiplier operand (squares use |x|). −360 LUT more, bit-exact in sim.
3. **Expect control-fanout timing instead.** Fix it by registering the sequencer step-enable (`seq_go`), splitting compare→enable cones (nearest-edge tracker), registering per-group counter flags at phase 0, and giving per-frame registers one `keep` copy per consumer region.
4. **Route with router1.** router2 passed 1 of 17 HD HDMI seeds on this netlist; router1 passed every seed at 78–88 MHz (same pattern as cubist).
— **cascade v2.1.2**, HW-validated 2026-10-10 (98.5 % → 94.6 % LC with Width + left-edge fix + Moiré added).

### Moiré by XOR with a half-phase copy of the rings
For concentric bands k = floor(r/s), XOR the band parity with `k_o ⊕ (r mod s ≥ s/2)` = round(r/s) mod 2, computed by an independent radius tracker around a reference centre. At rest that DOUBLES the bands; when the drawn rings are offset from the reference (a cascade drag), you get true per-pixel moiré arcs. Two traps:
- XOR with `(r mod s ≥ s/2)` ALONE only shifts the rings by s/2. Measure run lengths along a row; it "looked doubled" by eye.
- The bare half-band bit flips at full radii too, so it collides with the drawn edges at Pythagorean points (exact integer distances), giving ~70 one-px dots/frame. The `k_o ⊕ h` reference has no full-radius transitions, so it never touches the drawn edges.
Don't "fix" the dots with a d²−1 tracker convention: every centre-row pixel is at an integer distance, so it drew a 1-px line along the centre row. A scanline-state variant (forced at marks, toggled at half crossings) killed the dots but made hold-then-snap streaks. — **cascade v2.1.2 S9**, HW-validated.

### Light on a curved surface = light·radial projection
Offset the specular highlight by the projection of the light vector onto the local radial direction (`(ex·lx + ey·ly)>>8`) — the highlight slides around bends tracking the light, no CORDIC; straights use the same projection evaluated at the port so families unify; clamp to make elbow highlights concentric. — **pipedream v2.3**, timing-closed (~97% LC, 0 BRAM). Feed the clamp a quarter-px projection or crossovers staircase. The FSM shared multiplier is 2-cycle — reading at N+1 gets the previous product.

### Edge-hash connectivity (stateless network topology)
Hash the shared grid EDGE coordinate: a cell opens edge P iff `hash(edge_coord, seed) < threshold` — both neighbours hash the same coord so they always agree; no stored grid, no double-buffering. 4 edge bits/cell give every join type. — **pipedream v2**, timing-closed; deleting the v1 grid machinery freed ~2000 LC.

### Exact miter-stroke polylines as shared half-plane tests
A stroked polyline with miter joins = per-segment slabs clipped by miter lines → half-plane tests; quantize slopes to shift-add, share a handful of x-products, every plane becomes a constant compare. Derive the plane set numerically in Python first. — **overlook**, timing-closed (40% LC, 0 BRAM, no multiplies). Never sum individually-truncated shifts (one wide sum, ONE final shift); carry 4 fractional coordinate bits or zoom shows texel stairs; octave-mantissa zoom must be `(128+t[6:0])<<oct` for boundary continuity.

### Per-line projective floor (perspective for any per-pixel u/v pattern)
Any pattern driven by a u DDA + per-line v can be put in true ground-plane perspective by a per-line engine that runs during the PREVIOUS line on the vblank shared multiplier and swaps in at avid rise: `D = 64·H/2 + K·s'` (s' = frame row from centre), `r = (H/2<<18)/D` (16-step serial restoring divider, Q4.12 — no per-frame 1/H needed), `du = du0·r`, `u0 = ucen − du·W/2`, `v = vcen + s'·du` (mod-ladder). K = 0 is bit-identical flat; it is a valid homography with square texels at the centre row. Line 0 is recomputed on every blanking hsync before the field (idempotent; the last run before avid wins), so field_n is read late like ribbon. Must-haves: 6 fractional bits on du and the u accumulator (integer per-line du jitters screen-side edges ~5 px); fog linear in 1/r = screen rows (depth-linear fog is only ~10 rows deep = a hard horizon); a density fade on du hides horizon aliasing; pre-fade the palette per line through the same multiplier so the pixel path only selects. `|s'|` instead of s' = a mirrored carpet valley for free. Cost ~1150 LC, 0 BRAM. — **overlook v1.3**, timing-closed 6/6 seed 1 (router1), not HW-tested.

### Flatten the control if/elsif chain that gates a sequencer
`if vsync_event elsif seq_running elsif line_start elsif lseq/=0 then case lseq` makes every engine register's clock-enable the whole priority decode, plus the raw sync pins: 10–12 ns of routing into ~200 CENs (overlook HD Analog: 74.4 MHz on seed 2, router2 stalling). Register the sync edges into 1-bit event flags, then write each block as its OWN `if` (engines that are disjoint in time by construction; restarts placed last so their counter write wins). The CEN becomes `lseq = k`. 74 → 85–91 MHz on all HD configs with router1. — **overlook v1.3**.
Same trap, per pixel: a registered cell-boundary strobe (`pr_cell`) used as the LAST arm of `if x_loc = 1 … elsif x_loc = 6 … elsif pr_cell` still made its ~200-register fan-out wait on the x_loc compares (x_loc → wreach_q/bot_q/surf_q at 22–28 ns). Put the high-fanout strobe FIRST (`if pr_cell then … elsif avid then <x_loc arms>`); the arms never overlap, so behaviour is identical. Also −90 LC at 97% — **pixelbeach v3.24**, HW-validated.

### Fract-ring fractal as parallel bit-slices
`uv=fract(uv*2); ring test` iterated: with divisions locked to 2, each level is a fixed bit-slice of one 17-bit zoomed coordinate — all 6 levels evaluate in parallel, shallowest hit wins. Squares via `(16h+l)² = concat(h²,l²)+32·h·l` (two LUT tables + one 4×4 mult). — **orbifold**, timing-closed (0 BRAM, 82–103 MHz). Chebyshev mode needs the radius band halved or it degenerates to grid lines. v0.3: all shapes share the squared ring test by feeding a pre-scaled 8-bit LINEAR distance (max/sum/min combos of |x|,|y|) into the x squarer with y=0, per-shape scale on the threshold; seamless infinite zoom = exp2-table mantissa (constant rate) + ring identity keyed on the continuous size class c = level + E + coarsest/finest level thinned out over the last quarter octave before the drift wrap; per-level orbit/pulse from a vblank serial-multiplier sequencer (95% LC, 83–99 MHz seed 1).

### Cheap end-over-end tumble: y·cosθ
Rotation of a flat shape about a horizontal axis under orthographic projection = only Y foreshortens by cosθ — one LUT cos + a few vblank multiplies per sprite. — **tessera**, HW-validated. Drive foreshortening with a TRIANGLE wave, not linear angle (cos dwells face-on and rushes edge-on = fake speed change); x-bound the test or flat edges fire across the whole scanline.

### Multi-plane perspective zoom (planes ARE the depth quantization)
N full inverse-mapped grids, each scaled about the SCREEN CENTRE with per-plane speed gains, composited front-to-back; disjoint per-cell ownership splits content across depths. — **ziffern** NP=3/4, HW-validated. DEAD-END: per-cell corner-scale "zoom" (glyph grows in its fixed slot) is not perspective. Per-plane glyph ROMs must be per-plane generate-block EBR reads (one replicated LUT mux = router wedge); NP=5 is toolchain-blocked (abc9 crash).

### Zoom shells with exp2-LUT interpolated scale
N shells sample a cell-hash starfield in scaled space, per-shell scale swept one octave per cycle from a 256×13 exp2 EBR and LERPed via the shared multiplier (raw 256-step scale = ~2 px radial judder at slow warp); the fade envelope crosses zero exactly at the wrap; wrap-epoch bits re-seed the hash. — **warpfield**, v1.1.0 HW-VALIDATED, FINAL except presets (signed off 2026-10-07; 6/6 seed 1 router1, HD 78–80 MHz, 99% LC, 24 EBR). **Cell-edge kernel clip**: a per-cell star whose kernel reaches R units must have its offset ≥ R from every cell edge, or the neighbouring cell (which draws only its own star) slices it flat — Ron saw 'flat cutoffs on the bottom/right'. Offsets 4..11 in a 16-unit cell + kernels windowed to zero at 4 units fixed it (starfield v3.1.0 ported the same fix). **Alpha-over video composite without multipliers**: alpha band (opaque / 3/4 / 1/2 / 1/4 / 0) from the brightest star's kernel weight, out = under + (star − under)·a by shift-adds; both layers legal-range so the convex mix needs no clamps; T8 off = under-layer black with alpha forced 1, so one path renders both modes. The first ADDITIVE version was HW-rejected ('blends with the video too much'). **Delay-pipe pruning**: yosys prunes unread fields of the FF video pipe; tapping pipe(14).u/v for the overlay un-pruned ~450 FFs and overflowed the HX4K (7722 LC) — a 16-deep EBR circular buffer (2 EBR, read ptr = write ptr − 13 + one fabric re-register) carries the video sample instead. 6 shells = 95% LC; DEAD-END: 8 shells = 115%. **Constant multiplies in canonical signed-digit form**: yosys lowers `x * 0x9E37` to one adder per SET BIT (10 → 9 adders of 16 bits per shell); the CSD form `2^15+2^13−2^9+2^6−2^3−1` is 5 add/subs and bit-identical — −384 LUT4 standalone across 6 shells (−289 net after the finalization fixes). Check any `* constant` with ≥ 6 set bits; mod-2^N results want same-width `shift_left` (top bits are discarded on purpose).

## Polar, circles & CORDIC

### HX4K CORDIC playbook
Vectoring-mode CORDIC for euclidean distance (no mult/sqrt): **8 iterations** (not 14) is pixel-perfect and the LC drop is what closes routing at high util; pack 2 iterations per pipeline stage; G=5 guard bits or small-magnitude error explodes; unscale with a constant-multiply, never a shift-sum (floors small radii). The angle comes almost free → spiral/polar. A polar twist must use an INTEGER arm count (`phase += ang·arms`, whole cycles per turn) or it leaves a hard seam ray where the angle wraps (hypnos ≤v0.3, iris v1.1). — **hypnos**, timing-closed (88% LC, 0 BRAM); **chantilly** needed 10 iterations because the ANGLE rasterises dot edges there. Prototype the exact integer CORDIC in Python — float protos hide fixed-point bugs.

### Log-domain polar engine (CORDIC replacement)
`hi,lo = max/min(|dx|,|dy|)` → two cheap integer log2 units (priority encode + normalise + 256-segment PWL in EBR, 7×6 interp mult) → `u = log2(hi/lo)` indexes two interpolated EBR tables: `log2 r = log2 hi + ½·log2(1+4^-u)` and `θ_oct = atan(2^-u)` (1/16-octave steps, ~1e-5 accurate). Shapes become `+ log2 S(θ)` tables (square = `log2 hi` exactly); warp is `p·(log2 d − ref)`; exp2-mantissa × freq-mantissa + ONE shift gives the phase. ~2400 LUT cheaper than a 12-iteration CORDIC (whose add/sub costs ~3 LUT/bit) and a better angle; costs ~22 EBR. Integer coords (Q4 sub-pixel cost +275 LC) + a centre glide that never moves slower than 1 px/frame, so glide tails slide instead of ticking. — **hypnos v1.0**, sim-verified, timing-closed at 96.5% LC with router1 (79–90 MHz seed 1). Trap hit: `to_unsigned(16*16384, 18)` wraps to 0 — a "u too large" clamp fired on every pixel and rendered pure squares.

### Analytic-gradient box-filter AA (1-px edges at any scale)
Coverage of a band square-wave over one pixel = `(P(φ+w/2) − P(φ−w/2))/w`, `P(x) = ⌊x⌋·D + min(frac x, D)` — handles any duty and fades to the duty grey on its own as `w → 1 cycle`. Get `w = |∇φ|` analytically in log2 (radial `Lf + log2 p + (p−1)(log2 d − ref)`, angular `log2(arms/2πr)`, combine `max + ½log2(1+2^-2|a−b|)`), round log2 w UP to a half octave so the divide is a shift (× 1/√2 by shift-adds; edges 1–1.4 px), widen w above ¼ cycle/px so it hits a full cycle at Nyquist, and force exact full coverage when the window is wholly inside the band (else mantissa rounding leaves 252–254 streaks in flat areas). — **hypnos v1.0**, sim-verified (exactly 1.0 AA px per edge). The old adaptive power-of-2 hardness (v0.x) gave 6-px blur at wide periods. Slice the `(N<<Ri)>>k` result instead of saturating it — a saturate compare kept a 33-bit barrel alive (~130 LUT).

### Pre-CORDIC coordinate shift (shift the input, not the output)
For radius-dependent centre effects, nudge (dx,dy) toward the target by weight(r)·error BEFORE the distance pipeline. — **hypnos**, timing-closed. DEAD-END: the "correct" post-CORDIC projection needs a sine ROM + 3 mults = 96% LC and marginal timing. 128 weight levels, not 32, or slow glides terrace.

### 1bpp mark buffer + parity pixel path (nested rings/discs)
A line-early engine writes each disc's two edge x's as blind '1' marks into a 1bpp banked line buffer; the pixel path is a running parity XOR (strict nesting ⇒ parity = innermost disc). Ring count becomes engine THROUGHPUT (`15·N + 56 ≤ clocks/line`), never fit — LC flat from 24 to 110 rings. — **cascade**, HW-validated (126 rings HD). Needs an even ring count; left-overhang discs toggle a line-start INIT bit; clear both banks in vblank; latch the display bank at active rise, never derive it combinationally; adaptive ring count at coarse spacing or piled-up marks invert parity.

### Run the span engine a line (or N lines) early
Trigger next-line span computation at the ACTIVE-line rising edge (BRAM/mult are idle during active video) instead of cramming into hblank; N-lines-early generalizes with N+1 banks — but buys latency only, never throughput (see CLAUDE per-line budgets). — **cascade**, HW-validated. Vblank sequencers counting 20..1: a `when 0` slot never executes.

### rho^2 by second differences (exact quadratic field, two adders)
u^2 = s(x-cx)^2 along a scanline is Q += D; D += 2s, with start value, step and start column computed per field (microcode): start n = min(cx, 4Wa) columns left of the eye (beyond that rho^2 >= 16 anyway), Q(xs) = s n^2, D(xs) = s(1-2n); hold 16.0 before the start and once Q passes 16 on the far side. Replaced an 11x10 multiply + barrel + 11-bit square (~350 LUT) with a 29+28-bit DDA, exact to 0.006 in rho^2. TRAP hit: saturating the 29-bit register to all-ones made Q>>14 = 32767, and +v^2 wrapped the 15-bit rho^2 sum to ~0 -> a full-strength corona over the right fifth of the screen; saturate to exactly 16.0 (2^28-1). — **oracle v2**, HW-validated (the flashed build has the fix).

### Per-line half-width profile ROMs for an opening AA shape
An eye that opens at constant height: per line v = |y-cy|/hh, read two 256x12 profiles (vesica, circle) with half-step interpolation, mix by the opening (vesica -> circle), x the open width -> half-width w(y) in Q4 px. Per pixel it is one compare plus a coverage ramp 0.5 + (w - dx)/2^k with k = ceil(log2 |w(y) - w(y-1)|), so steep tips get a ramp as wide as the boundary moves per row and flanks stay 1 px. — **oracle v2**, HW-validated.

### Squared-radial-distance circle test
`dx² + dy² ≤ R²` with an annulus for outline — perfectly round, no height LUT; share one unit across scaled layers by shifting slot bits. — **candyhills** bushes (HW-iterating), **chantilly** cells. Pipeline as operands / squares+sum / compare (fused = 63.7 MHz, split = 84.3); clamp wide unsigned BEFORE abs at slot edges (`unsigned(-v_dx)` at −64 wraps to 0).

### Screen-true circles via aspect-corrected vertical Q4 DDA
Work in "horizontal pixel" units everywhere: the per-line vertical advance is a Q4 step (16 on square-pixel HD/720p, 18 NTSC, 15 PAL, doubled per field under detected interlace, bottom field seeded one frame-line later) and the row pitch is expressed in the same units — then `dx²+dy² ≤ R²` is a circle ON THE DISPLAY in every video mode, and grid squareness is automatic. Interlace detect = field_n toggling at the frame anchor (self-clears on mode change, clacker-style). — **polka**, timing-closed.

### Dual-lattice lane B: adjacent-row dots OR negative-space holes
Run gumball's two half-offset X lattices, but give lane B a per-frame ROLE: staggered grid below full coverage → nearest adjacent-row centre (exact hex Voronoi union, offset rows merge correctly); otherwise → the DUAL lattice (cell corners) drawing background-coloured "hole" discs, composed as `min(dotα, max_α − holeα)`. With a hole-radius ramp `r = ⅞·octc − ¾·kiss − R/4` (square; rising ramp past the coverage point for stagger), holes stay strictly inside the uncovered corner gap until well past the kiss point — dots stay perfect circles through the kiss — then take over as the union closes, so one fader sweeps grow → kiss → merge → invert-to-round-holes continuously. Prove the purity invariant numerically (R + r vs true corner distance) before building. — **polka**, timing-closed. Companion: alpha AA per lane via per-frame case-shift + gain (hard = 3R edge band, soft = R² parabolic dome — same pixel path, different per-frame divisor).

### Tintable palette colours through one shared sine path
Store palette entries as (luma, saturation, hue-angle); dots resolve entry -> palette LUT -> sine EBR -> sat multiply, and a global tint knob is ONE 10-bit add — placed in the EBR ADDRESS stage, which has slack (stacked on the entry-mux+LUT stage it was a hard 74.0 MHz ceiling).  The frame-constant BACKGROUND colour needs no second ROM or port: a pal_hij flag overrides the entry mux during the vsync cone so the FSM rides the whole pixel colour path (LUT, tint, sine EBR), then finishes with the serial multiplier.  Rotating hue with sat/luma pinned keeps a curated combo harmonious at every knob position. — **polka v2**, timing-closed.

### One per-line lattice-shift port = slant, wave, and scatter layouts
Add a single wrapped per-line offset to the lattice seeds at each h edge and mux its source per layout: accumulated shear (slant), a small sine LUT scaled per frame (wave; animate the phase for motion), zero (grids).  Per-dot scatter rides the same lattice: hashed x-jitter with a power-of-2 budget capped at Wh−R (dies exactly at the kiss point, so a swell/merge arc stays exact) plus 2-level y-jitter as two per-line dy² variants muxed by a hash bit — no new pixel multipliers anywhere. — **polka v2**, timing-closed.

### FSM discipline at scale (polka's timing-closure ledger)
A live-LFSR-stubbed ablation (stub must DRIVE the pixel path or yosys const-folds everything and lies) showed the frame+line FSM was 2524 of 4820 program LUTs. What closed 46 → 74+ MHz, in order: drop per-row pulse ripple (spec allowed lockstep; per-row R/tri/hole products were ~400 LUT of line-FSM), one-op-per-state splits of every fat state, sequencing the once-per-field drift accumulator through dedicated states, registering the raw sync/video input pins before ANY cone (a VSYNC pin was a 12 ns routing hop), latching SPI knobs in state 0 instead of at the vsync trigger (fanout), and rewriting multi-state min/clamp chains as LINEAR single-writer registers (a register written from 5 states = a wide scattered D-mux; fr_hv's was the hd_dual critical path). Remaining passes are seed-tail events at ~94% LC — hunt seeds solo; parallel load reliably degrades router2's post-route Fmax, and a "PASS" line from a stalled run is the placement estimate, not a result. — **polka**.

### Exact hex Voronoi = nearest-of-2
Two half-offset X lattice counters (even/odd rows); per pixel compare dx²+dy² for own-row vs adjacent-row nearest centre — 2 small squarers + 1 compare = true hex cells. — **gumball** v1.0.1, HW-validated (FINAL 2026-10-03). Use incremental per-line squares ((d+1)² = d²+2d+1), not a shared line-rate squarer (its mux+mult cone was the critical path).

### One-field fake-Phong sphere shading
A single field e² = (dx−ox)²+(dy−oy)² drives BOTH diffuse dome and specular hotspot; per-frame normalization by priority shift + serial divide keeps the pixel path at a 2-stage shift + 8×8 mult. — **gumball**.

### Honeycomb / tiling lattices without divides
Hex Voronoi of a brick lattice reduces both edge families to linear bisector tests (`|u|=512`, `|u|+2|v|=1280`) — within 8% of regular hex. Generally: every tiling texture = one 10-bit wrapping phase + one comparison. — **fringe**, timing-closed + pixel-exact vs model (2-layer 10-texture engine at 94.7% LC, 0 EBR). For 50%-duty gratings, invert ≡ half-period phase shift — an invert switch duplicates the phase knob.

### Grid-free polar halftone: dots-per-ring = 6·ring
Concentric arcs with dot count ∝ ring index keep cells ≈square at every radius and never align radially; `frac_θ = θ·ring·6 mod 4096` makes the wrap seamless by construction. A spin is ONE angle add before `θ·ring` (mod 4096 stays seamless; flip its sign on alternate bands for counter-rotating gears). — **chantilly** v1.0.0, FINAL except presets (signed off 2026-10-09; 82% LC, 8 EBR).

### Rounder halftone dots: running box + vertical IIR in one line buffer
A per-pixel luma → dot-radius halftone traces the video's contours inside each cell. Blur the luma first: an exact 8-px running box (`sum += new − 8th`, no reset needed) then a vertical one-pole IIR `new = above + (box − above)/4` stored in ONE 4096×8 memory (8 EBR, bank = line parity: read the other bank, write your own). The same buffer gives a free vertical gradient (`box − above`) for horizontal-edge ink. Saturate the column counter at 2047 so the blanking tail can't wrap onto the start of this line's bank; seed the frame's first line with the box. Compare gradients against `+th`/`−th` in parallel, never `abs` then compare (EBR → sub → abs → cmp in one stage pinned HD HDMI at 67.4 MHz on every seed). — **chantilly**, FINAL except presets.

## Sprites, rasterizers & line-buffer engines

### Dual-bank line buffer with parity carried per pixel
Inline two bank arrays; read uses live line-parity, write uses the parity CAPTURED at read-issue and carried down the pipe — end-of-line trailing writes land in the correct old bank with zero special-casing, sidestepping the write-addr-delay drift bug entirely. — **shearline** (timing-closed), reused by sidewinder, taffy. A clear-sweep write plus a blend write to one bank = two write ports → FF explosion (169k LC); mux into ONE (addr,data,we).

### Rolling wipe for a bbox-cleared line buffer (per-bank pointers)
A sprite/line-buffer engine that clears only its object's x-range each line leaves anything stamped elsewhere — power-up EBR content, stamps from the first frames before mesh/geometry settled, a pan's trail — sitting there for ever: a stale bit in one bank shows on every other row as a thin vertical line. Add a WIPE state between the clear and the stamps that zeroes N (32) entries per line at a rolling pointer, so the whole buffer is cleaned within a few hundred lines and the stamps can never be erased. TRAP: with ONE pointer and banks alternating per line, an EVEN chunk count (2048/32 = 64) means bank A only ever sees the even chunks and bank B the odd ones — half of each bank is never wiped, and the lines appear on some loads and not others. Use one pointer per bank (or an odd chunk count). 32 cycles/line. — **bishop v0.4.3**, HW-validated 2026-10-09.

### SINGLE-bank line buffer (when the stamp trails the beam)
If the renderer stamps column c at `avid_start + c + D` (D = its pipeline depth) while the beam reads column c at `avid_start + c`, every read precedes its own overwrite by D clocks and ONE bank serves both. Write EVERY column (alpha 0 outside the shape) and clear-behind-beam deletes with it — leaving the canonical 1W1R shape with no two-source contention on the single EBR port. — **aurora v1.7–v3.0.3**, HW-validated (v3.0.2 on HD HDMI, 2026-10-08): 24→18 EBR, and the bank mux + display-bank latch + clear FSM all go. Pad the word to a power-of-2 width with **LIVE** bits (tie-off zeros get pruned): aurora went 14 b (hue9+alpha5 → a 7-block 2048×2 mapping with two writers) to 16 b (hue10+alpha6 — the extra hue bit stays live through solid mode's `hue·7` segment math, the extra alpha bit doubles feather resolution at the same feather width). Check the vblank restamp of row 0 obeys the same D>0 rule.

### Time-multiplexed line-buffer sprite engine
One shared rasterizer walks each sprite's span per scanline into a 1–2-bit ping-pong line buffer; scanout reads/clears the other bank. Fits 10–64 sprites where independent per-pixel rasterizers wall at ~8. — **tessera**, HW-validated. Fill budget: N·(2R+~5) ≤ line clocks is the all-on-one-line worst case — with golden y-spacing only ~4–5 sprites ever share a scanline, so tessera v15 went 20 → 64 sprites at ~2× the size (height 0.75/√N; per-line cost grows ~size², so size the law from a Monte Carlo of the busiest line with random orientations/phases, not the all-face-on bound). Overrun guard = two flags: avid_start while the pass is busy → end the walk, skip the remaining walks but still step every edge (nothing desyncs), start the next pass the moment this one ends (a line-period budget guard cost ~195 cells for the same result); sweep both banks to 0 during vsync (stale frame-wrap content); per-edge thresholds (edge length × width) for even outline thickness. DEAD-END: 10 independent per-pixel rasterizers.

### Sprite records in a FIFO or BRAM — never runtime-indexed register arrays
Fill logic reads only the FIFO head (pop, walk, push-back), or sequential BRAM slots with a 1-cycle-latency state; geometry rewrites in vblank. — **tessera** v7/v11 (20 sprites at 89% LC vs 11 in registers at 96%). DEAD-END: register arrays indexed by a runtime signal (v6: 125% LC of mux trees). Latch the write address WITH the registered pulse when the counter increments the same cycle, or slot 0 is never written.

### Scaled bitmap sprites in logo space (one shared accumulator, per-sprite offsets)
To scale N sprites that sample a shared ROM, don't compute `(h − x_i)·step` per pixel: keep ONE accumulator per axis holding `h_count·step` (same reset/advance as pixel_counter, so it's exact and aligned), and give each sprite a per-frame offset `C_i = −x_i·step` from a serial multiplier. Logo-space coordinate = accumulator + C_i, which is the same one add per sprite per axis as the unscaled delta. A streaming prefetch that needs a lead gets a second accumulator preloaded with `lead·step`. `step = 1/scale` comes from a serial divider; latch (scale, step) as a PAIR once per field, glide + deadband the knob, and wait ~1k clocks after the field latch before wall physics so collisions use this field's size. — **dvdlogo v1.0**, HW-validated (0.5–3x, 4 logos, +1050 LC including divider/glide/wider multiplier). A 2-entry parity prefetch buffer stays correct while a ROM word spans ≥ 8 screen px (0.5x for 16-px words with an 8-px lead). Timing: the knob deadband (add + compare + register enable fused) and a wall compare that fans out to every predicted-position bit both failed HD. Split the first one op per stage, and precompute the flags free-running during the wait.
**invaders** v0.4 (HW-validated) uses it for ONE sprite with a growing zoom and no divider. Parametrise the view by its CENTRE: `u_start = 64·256 − xc·k`, `v_start = 64·256 − yc·k`, with k = sprite px per screen px ×256 (256 = 1×, which reproduces `h − x` exactly, so the normal case uses the same path). One 9-step serial multiplier per frame does u then v; load v_acc a few clocks after vsync, before the first active line. The win flight is integer physics on (xc, yc) in 12.4 plus k −3/frame. Its velocities were bounded so the positions never need saturation logic, which saved ~50 LC.

### Sharp-head overlay on a coarse persistence canvas
When a soft low-res canvas (glow, trails, embers) can't get finer — pyro's 320×180×2-bit canvas already held 29 of 32 EBR, and 4×4-px cells at 1080p would need 2× the chip — draw only the HEADS at native resolution on top. A memoryless engine already computes every spark position each frame: keep quarter-cell precision (radius fraction × cos/sin as shift-adds, droop in quarter cells), convert ×1.5 to 1080p pixels, and write one head-table entry per bright head (256×32, 2 EBR). A per-line renderer stamps line y+1's heads (3×3 plus / 5×5 diamond) into the non-displayed bank of a two-bank 1-bit line buffer (1 EBR each; the displayed bank is cleared one pixel behind the beam, so no clear pass) — ~1.3k of the 2200-clock 1080p line for 250 heads. Read the line buffer at the canvas pipeline's own stage-2 x_count and a head lands on its glow by construction (with 6-px cells the bilinear canvas peaks cell S at x_count = 6S). The table goes final when the engine pass ends (inside vblank at 1080p); stamp line 0 during the blanking before the frame. Head pixel = luma 768 in the shell's palette colour (a whitish hot core). The overlay alone cost ~+650 LC spread thin over many small pieces; it fit only via the attribution time-mux (see Voronoi color attribution). — **pyro**, FINAL (signed off 2026-10-10; 1080p only).

### Interval-store vector rendering (framebuffer-free crisp curves)
Store a curve as per-scanline horizontal intervals (rows × slots × {x, runwidth}); a pen walker inserts runs, a trailing eraser re-walks the identical deterministic path W steps behind; display stamps intervals into a small line buffer each hblank. — **spiroscope v5**, timing-closed/HW-iterated. Eraser determinism requires frozen geometry — quantize geometry knobs to zones, clear+redraw on change. DEAD-END: BRAM framebuffers for line art (cap at ~320×180, never crisp).

### Edge-on-at-change flip element (cross-frame state without history)
A flipping element is edge-on (invisible) exactly when its state changes, so the previous face never needs display → store only {value, timestamp}; age = frame − stamp indexes a damped-ring curve ROM rebuilt per frame in vblank (near-zero LC, kills a per-pixel multiply). — **clacker**, timing-closed. Pin settled cells' stamps or age wrap fakes a re-flip; hysteresis on the back-face reveal.

### Snoop-RMW grid sampling
Sample a display grid by snooping the pixel path's OWN BRAM read at the cell-centre line into a small row buffer; burst-commit during the next hblank. Avoids a true RMW port entirely. — **clacker**. Clamp the burst to last-sampled column (unsampled columns hold init data — and all-zero chroma is GREEN).

### Ping-pong by parity with unconditional dual read
Write bank(par), read bank(!par) — collision impossible; read BOTH arrays every clock and mux registered outputs (a conditional read maps to logic, not BRAM). — **c64**, HW-confirmed. DEAD-END: per-column read-modify-write accumulator BRAM — simmed perfectly, black-screened on HW (GHDL defines read-during-write; silicon doesn't).

### Interlace-correct circular geometry
Detect interlace by field_n toggling (recompute every vsync so it self-clears on mode change); halve row pitch, step dy by 2/line with incremental squares, start the odd field +1 line. — **clacker v0.2**. Most of the library renders 2:1 tall on 1080i without this; not sim-testable in the progressive image sim.

### N sprite ghosts for the price of one row mux
Space echo copies ≥ sprite width apart so at most one covers any pixel → pick the covering ghost early, do ONE wide ROM-row mux regardless of count; carry a small selector index down the pipe. — **invaders**, timing-closed. DEAD-END: overlapping ghosts (each needs its own 128:1 mux; 3 of them = 66 MHz). Companion: prefix-OR trailing-edge shadow, coarsened to 4-px groups (full 128-bit prefix-OR ×2 = ~1500 LC, DEAD-END).

### Render-time collision latches (arcade sprite-collision registers)
Detect collisions where the pixels are already being drawn. A few latches fed from the final per-pixel flags (shot∧alien, shot∧bunker, bomb∧bunker, bomb∧ship) each take the FIRST overlap of the frame in raster order, along with the object indices that ride down the pipe beside the flags. Snapshot them at vsync and let the FSM commit. One latch per type is enough, because an overlap left unresolved is still there next frame. This is pixel-accurate (bombs no longer kill on the transparent corners of a box) and it replaced a serial one-projectile-per-frame bbox scan that missed thin targets. — **invaders** v0.2+, HW-validated. Keep projectile speed below (target height + projectile height) per frame, or a shot can jump a target between frames. Snapshot on a PRIVATE registered copy of vsync (keep): the shared vsync_start net fanning out to ~40 snapshot registers was a critical path.

### 16-pixel-word sprite ROMs (narrow select instead of a whole-row mux)
Store every sprite as 16-pixel words with address {row, col(hi)} and pick one bit with a 16:1 select of `not col(3..0)`, instead of registering a 128-bit row and doing a 128:1 mux. Mirroring is a bitwise NOT of the column bits, on the address and the index. Anything that needs whole-row facts gets its own precomputed ROM: a 128×32 group-OR ROM feeds the edge-shadow prefix-OR. A per-pixel colour region is a second 1-bit plane on the same address. The invaders eyeball mask is a flood fill of the region's lit pixels, bounded by a disc that the art's outlines close (stable as the radius grows), so the mask is a strict subset of the silhouette. — **invaders** v0.3, HW-validated: with an early video tap, a serial ship multiply and a shared band LUT, 6520 → 5780 LC and 22 → 20 EBR at an identical look. A ROM relaid out per chunk ({chunk, row, word}) gives destructible-terrain debris sprites with no address arithmetic.

### Draw a second sprite in another sprite's DDA view space
A big prop that must stay registered to a zoomed sprite (the invaders death TV around the 3.2× wizard) can be drawn in the sprite's OWN DDA coordinates rather than given its own scaler. Pick the prop scale as a power-of-two multiple of the sprite's, so src = (u + c_x) >> 2, (v + c_y) >> 2: two constant adds, a bit slice, and in-range = high bits zero. It then grows, eases and rumbles with the sprite for free. If its ROM path is idle at that moment (the formation was faded out), time-share it: no EBR. GOTCHA: the prop exists only as long as that view does. Snapping the view back while the prop was still fading drew a 1×-scaled copy at the sprite's home position, so hold the view until the prop is gone. — **invaders** v0.7, signed off.

### Per-pixel crossfade through ONE shared sprite path (complementary Bayer)
Two layers that share a single ROM/render path (TV vs alien formation) can still crossfade. Each pixel's 4×4 Bayer value b picks layer A if b < level_A, and layer B shows if (15 − b) < level_B. With level_A + level_B = 16 the tests are exact complements, so every pixel is exactly one layer and the path serves both. Outside the crossfade the same tests give plain dither dissolves, with no multiplies. Make the decisions one stage early (registered) so the source mux select is a flop. Start the ramps on multiples of 16 so dt(3..0) is the step. — **invaders** v0.7.1, signed off.

### Spare ROM bits for a second mask (pocket fill for edge trails)
A 1-bit mask that only matters on LIT pixels (eyeball colour) leaves its bits on UNLIT pixels free for a second, disjoint mask. Invaders stores a trail-gap mask there at no EBR cost: outside pockets between body parts, i.e. unlit pixels outside a 5×5-closed silhouette that lie between two body parts in their row. The closing keeps thin interior linework out; a plain "connected to the outside" flood fill coloured the art's lines, because they leak to the border. A gap pixel is shadow when a body group lies inside the band window (base..tap). Model the mask in Python on the real sprite before building. — **invaders** v0.7.3, signed off.

### Per-line sprite-slot selection
Sprites in disjoint vertical bands: pick the active slot PER LINE (2 compares → a RAM address); the per-pixel path never changes. Slot state must live in EBR, not registers (3 register slots = ~125 LC of decode+mux and a blown chip; ~free in BRAM). — **pulp**, timing-closed. Baking geometry into sprite ROMs replaced ~1,400 LC of SDF math with 6 EBR + ~300 LC.

### Glow-kernel ROMs for soft sprites
Per-layer EBR ROM addressed {glow, shape, |dy|, |dx|} → weight; shapes, depth fade, brightness ramps baked into contents; composite in the weight domain before one shared brightness multiply. — **starfield v3.1.0**, timing-closed (6/6 seed 1 router1, HD 93–98 MHz, 99% LC, 20 EBR; not flashed); reused by warpfield (which found the cell-edge kernel clip; starfield 3.1 ported the fix). **Cell-safe kernels shrink the ROM**: once every kernel is zero from 4 units, |d| ≤ 3 holds every non-zero weight — 2-bit |dx|,|dy|, a 256-entry ROM, one EBR per layer instead of two (16 → 8 EBR; assert the dropped entries are zero in the generator). **Sub-pixel scroll**: whole-pixel layer speeds with a 1 px/frame floor made every layer move together at slow speeds (no parallax exactly where a slider push starts) — 18.6 fixed-point scroll and a floor relative to the front speed (1/16) fix it. Hash-bit bias: when presence = hash < threshold, top bits of survivors are ~always 0 — derive per-item properties from LOW bits, xor-folded. A wrapping background injected ~8 px late is invisible — skip 7 stages of delay-matching.

### Glyph ROM from Unifont .hex
Vendor a GNU Unifont subset, parse with a build hook into a VHDL pkg, flatten into ONE single-port BRAM (addr = glyph·16+row) — combinational multi-row reads triplicate as LUT-ROM. In-cell glyph scaling is cheap on top: offset from the cell centre is the MSB-flip of the in-cell position, × 16/s (small LUT) >> (4+zoom) gives the glyph pixel, and an offset outside −8..7 (sign-extension test) is off-glyph; grow only on x (half-width glyphs have room), shrink on both axes. — **matrix_rain**, HW-validated (v1.0.0 FINAL 2026-10-05).

### Hashed per-scanline filament runs (edge-launched, bundle-coherent)
Each scanline launches ONE run at a fired edge; its length, fire decision and pattern seed come from a 7-stage single-op spatial hash of (frame_line >> K, column/16, seed), so lines are independent at K=0 and bundle into coherent sheaves as K grows — patch-level randomness with zero storage (no per-column line buffer). A 16-bit phase accumulator inside the run drives all pattern engines; output is a lerp `ground + (base-ground)*int/256` so fades land on whatever the ground is. — **fray**, HW-validated (v0.4.2 FINAL except presets 2026-10-03; 6/6 seed 1, HD 85–87 MHz, 65% LC, 10 EBR). v0.2 adds a per-run tail taper `min(pattern, sat(rem << (8-k)))` with k = msb(len)−1 fixed at launch — a divide-free fade over the last quarter-to-half of every run. Lessons: the first shift-add hash had 0.9999 neighbour correlation on its top bits — search shift constants numerically before building; start the hash one column ahead (a cx+1 counter) to buy a stage for splitting the length multiply; never let an accumulator's carry-out drive a wide register's enable — register the wrap and act a pixel late.

## Displacement & feedback

### Rotating-bank displaced-scanline gather (Rutt-Etra)
Vertical luma-displacement without a frame buffer: make displacement one-sided (down from the source row), so every line crossing the current pixel comes from a row ABOVE — already streamed. Every Nth row writes `{disp8, bright4}` per column (cols ×2 decimated) into one of K=8 rotating banks (1024×12 = 3× 1024×4 EBR each, 24 EBR); per pixel, 8 parallel `age_k − disp_k` window tests + a recency-priority resolve (rotate hit vector by head, priority-encode) pick the nearest line — recency IS the hidden-line rule, and `age ≥ disp` coverage gives free opaque terrain skirts. Max disp = K×spacing−1 (denser lines ⇒ shallower relief, inherent). Mask the head bank on its write row (read-during-write undefined); per-bank kept read-address registers beat the 8-way EBR fanout net. v0.2 refinements: **connected strokes** from x2-decimated columns by midpoint subdivision (phase 0 draws prev->mid, phase 1 mid->this; test `age-lo >= 0 and age-hi < T`, cover on `age-lo`), dimmed by segment span (beam-speed look); a causal IIR's group delay (2^s-1 px) cancelled for free by READING the completed stored rows 2^s px ahead (clamp at the measured last column) - no latency; line-grid roll = spacing-counter preload of phase x spacing at vsync (16-bit phase in spacing units wraps for free), with the field's top line faded by the phase so lines never pop in. — **ruttetra**, v0.1 HW-validated (77+ MHz HD, 53% LC); v0.2 final (75-81 MHz HD, 75% LC), signed off 2026-09-28.

### Coarse frame store in EBR (frame-recursive pictures without a frame buffer)
A whole frame fits on-chip once it is coarse and/or shallow: 256x128 x 1 bit = 8 EBR, 64x32 x 16 bit = 8 EBR (power-of-2 grid on both axes so the address is a plain `{row, col}` concat). Two banks ping-ponged by frame parity (write bank(par), read bank(!par), both read every clock) give a true recurrence: this frame's cell = f(this frame's sample, LAST frame's cell read from a *displaced* address) — decay trails, zoom/drift/spin feedback tunnels, random-walk diffusion, temporal derivatives. Mechanics that made it clean: (1) cell geometry from two DDAs (`xacc += cols; if xacc >= W` per pixel, same per line) on the runtime-measured width/height, sampling the row's MIDDLE line (flag when the row accumulator passes H/2); (2) one-pole IIR along the line, latched at the cell end = box-ish average with no divide; (3) the display read (cell of the current pixel) owns every cell-START slot of the shared read port, the feedback read is a *request register* issued on the next non-start slot (1-clock deferral), and a display hold register covers the stolen slot — so any cell width >= 5 px works; (4) all per-cell feedback math is a **pulse-tagged pixel-rate pipeline** (a valid bit travels with the data, one op per stage, no per-cell hold registers, so pipeline depth is unconstrained by cell width); (5) a free-running per-cell chain (hash, centre offsets, Bayer) computed for the CURRENT cell and captured at the end-of-cell pulse — its depth must be <= min cell width - 1. With 1-bit cells, persistence is *stochastic* (cell survives if hash < P) and sub-cell zoom velocities come from stochastic rounding of the fractional offset (floor + carry-if-frac > hash) — both give continuous control at 1 bit. Storing the previous SAMPLE beside the previous OUTPUT (mosaic's 16-bit cell) is what makes |sample - old sample| a real motion signal; any law that feeds its own output back through XOR/subtract oscillates on static input. — **afterimage** (1-bit 256x128, 16 EBR, ~3975 LC = 52%, HD 84.8/91.2/88.6 MHz all seed 1; v1.0.0 FINAL, HW-validated 2026-10-07), **mosaic** (colour 64x32, 16 EBR, ~4700 LC = 61%, 4 laws: trails / stochastic-IIR blur / motion / hold; HD 78.0 (seed 3; seed spread 69.6-78 with the critical path in the CORE's SPI peripheral, not the program) / 82.0 / 83.4 MHz), afterimage HW-validated and signed off 2026-10-07 (v1.0.0: Persist grain compare fixed to the labelled 25..100%, output luma 1-2-1 low-pass against the 1-px full-swing-step HDMI bars, defaults captured from HW); mosaic timing-closed, not HW-tested. Genre caution: the coarse store should BE the picture (blocks as primitive) or drive a structural re-render — a smooth coarse field modulating the live raster is the rejected overlay genre.

### Single-bank frame-recursive spread field (halo / corona / trails)
A halo wider than a line-buffer window can see needs lines the raster has not delivered yet, so build it ACROSS frames: a 128x64 x 4-bit cell field (one 8192x4 store + a 128x8 aux word per column carrying {above-row old value, running injection}), per cell per frame `F = max(inject, spread*max(4 neighbours), persist*F)`, bilinear-upsampled for display. One port serves two prefetch reads, the recursion write and the aux read/write at cell rate; the four corners are prefetched a full cell AHEAD (reading mid-cell stripes every cell boundary), and the row's field write lands on its last line, when injection is complete. One field = neon halo + Kirlian corona + ink bleed + cel shade + trails. PERSIST as a third max() term is a NO-OP inside a wide halo: it must change what the field IS (cut neighbour coupling x0.55, self-retention 0.93), so the glow tightens and a wake appears. Trails are soft and cell-blocky, never sharp ghosts. Two wrap traps, both invisible in stills and sims: (1) the **recursion and the display need DIFFERENT edge conditions**: a missing neighbour contributes 0 to the recursion, but the display holds the edge value (else the bottom row fades out); split them free by reading different registers of the prefetch chain, and clamp `row+1` at the last row, zero the aux "above" on row 0. (2) **Grid dims rounded UP from the raster guarantee one partial row** (`rows = H/64 + 1`) that never reaches its "last line", the step that writes the field and clears its aux inject accumulator; the accumulator survived vblank and the next frame's row 0 read it as its own (bottom haze reappearing at the top). Close the partial row on the frame's last active line too, and force the row's first-line inject to 0. -- **livewire** v1.1.1 (89% LC, 23 EBR, 78-86 MHz HD), HW-VALIDATED, finalized 2026-10-09.

### One-bank grid simulation swept just behind the raster (advected fire)
A whole-grid recurrence (semi-Lagrangian advection: each cell GATHERS from where its parcel was last field) normally needs two banks. One bank is enough when parcels only come from below (v >= 0): job(d) runs as the raster enters grid row d and does three things in order — prefetch grid row d+2 into a 3-slot row cache (slot = d mod 3; 66 jobs/field divides by 3), simulate row d-1 into a one-row EBR buffer, write back row d-2 from that buffer. The sim reads only rows >= its own, all still last field's; the raster reads only the cache, filled before those rows are overwritten; the one-row-delayed write-back lets v = 0 read the row itself unmodified. Jobs 64/65 after s_vsedge finish the last row, flush, and prefetch rows 0/1. The display side is a free-running alternating cache read two cells ahead into cur/next/next2 register pairs with a 6-read preload per hsync (min cell width ~5 px); horizontal/vertical fractions come from DDAs whose increments a serial restoring divider derives per field from the measured width / active-line count (cells*2^16/W, rows*2^16/H), so any mode fills the screen with the grid. Per-cell work runs on a 20-clock two-phase sequencer (phase A: velocity for cell n on one multiplier + one 2-stage hash unit + a smoothstep S/D ROM; phase B: gather/cool/burn for cell n-1 on a second multiplier, with the burner's own bilinear in its idle slots) — 131 periods/row = 2.75k clocks, sized to 480i's 3.2k-clock row budget (a 40-step serial version would not fit interlaced SD). Curl-noise velocity is ANALYTIC: the derivative of smoothstep-bilinear value noise comes from the same 4 corner hashes (A = e10<<8 + dA*S(fy), dpsi/dx = A8*D(fx) with D(f) = 6f(1-f)), no finite differences. Buoyancy (v = vbase + T*kb) stretches hot tongues, ENTRAINMENT (u += (Tr-Tl)*ke from a 3-cell shift of the row's own reads) makes them self-sharpen and separate — that term is the difference between a lava lamp and fire. Genre note: the smooth field IS the picture; adding a fine ridged display-noise layer on top re-created the "weave grain" look that got Inferno called fake. HW lessons (v0.1→v1.0): 128 cells across 1080p read as too soft/wide → 256x64 at 4 bits (same 16 EBR) with hash-dithered stochastic rounding (nearest rounding would freeze sub-level cooling; a slowly-phased per-cell hash avoids 60 Hz sparkle); keep the burner rows OFF-SCREEN (on screen their per-cell random stretches into vertical lines) and run the unreached row jobs as a post-vsync chain; any constant lattice drift reads as the fire marching that way → random eased gusts (centre-weighted target every 0.4..1.5 s) driving both a whole-field wind and the lattice drifts; the headline slider as Blaze (fuel blend toward a full-screen climax, speed untouched) beat a 1..5x knob multiplier. Post-finalization HW bug (v1.0.2): the three-period cell pipeline gathered cell p-3 but wrote its result to cell p-1 — a uniform 2-cell-per-field translation of the whole field that read as "the fire always leans right" (the burner re-anchors each field, so translation looks like shear); a second slip read the write-back cell after the late phase had overwritten it (grid crept up a row per field). After re-spacing a multi-period pipeline, re-derive the cell index at EVERY write, not just the data hazards; a per-row cross-correlation "lean metric" on renders plus knob-zeroed / forced-term source variants localized it in two render rounds. — **pyre** v1.0.2 (256x64 x 4-bit, 26 EBR, ~6165 LC = 80%, HD 78.2/77.8 (s2)/79.4 MHz), HW-VALIDATED, finalized 2026-10-07.

### Sub-banked bilinear frame store + lookahead affine DDA (smooth frame feedback)
Coarse frame-store feedback that reads back SMOOTH at full resolution: split each grid into 4 sub-banks by (col parity, row parity), addr {bank, r>>1, c>>1}, so any 2x2 neighbourhood is exactly one cell per sub-bank -- one registered read per sub-bank per clock gives all four bilinear taps (even sub-bank col = (iu>>1) + iu(0), odd = iu>>1; reorder taps by iu(0)/iv(0)). The read address is a per-pixel affine DDA (u, v in Q18 cells: zoom/rotate/drift about the centre, aspect-correct for square px: dudx = a kx, dudy = -b kx, dvdx = b ky, dvdy = a ky) that runs C_LOOK px AHEAD of the raster (holds pixel C_LOOK's value while avid is low), so a long read pipe lands on the mixer with live video only 3 FFs deep. Every sub-pixel bias inside the loop is a drift per generation (60/s): sample at pixel AND line centres, cancel the write-side IIR group delay in the read offset ((2^k - 0.5) kx - n kx/2 = 0 at 1920 wide), +1/32 cell for the weight floor, and ROUND every shift in the mixer and per-generation chain. 4-bit store + LFSR-dithered stochastic rounding keeps slow decays alive. Fit lessons: 17 yosys array multipliers were ~2700 LUT (4-bit-weight shift-add `smul4` instead); a case-per-state vblank sequencer with one add per state was ~1700 LUT (one shared accumulator with a REGISTERED operand, steps every other clock); at 93% LC a third of LCs were DFF-only (chain products straight from macc, carry less). A one-pole IIR that fails timing splits into e <= x/2^k - acc/2^k, acc <= acc + e: same DC gain and DC group delay, real stable poles for k = 2..4. -- **feedback** v0.1.0, FINAL except presets (signed off by Ron 2026-10-11; 92% LC, 25 EBR, HD 75.5-78.9 MHz, HD seeds 3).

### Output-buffer tear: displace the finished picture, not the generators
Write the composed pixel into 256-deep circular EBRs (Y, U, V) and the stage-0 syncs/avid into a parallel one at the same pointer; read syncs a fixed D0+N later and video D0 later, so a per-line read delay (latched mid-line for the line about to leave) shifts EVERY layer of that row at once — glitch bands, chroma a few px off the luma via its own read address, black (blank-gated) fill at the opened edge. 4 EBR + ~60 LC; constant latency for all modes; delays 2..254 never touch the write address (no_rw_check). — **oracle v2**, HW-validated.

### Recursive line feedback (diagonal smear)
Each scanline reads the previous OUTPUT line at (x − shift), lerps with live video, writes back; accumulating shift = diagonal comet trails. — **shearline**, timing-closed. The blend must be the 1-multiply form `live + decay·(prev−live)>>10` — the 2-mult form stuck HD at 65 MHz.

### Per-scanline wave displacement with blanking-time step sequencers
Write live line to bank(par), read bank(!par) at a wave-shifted address; ALL wave math in registered step-FSM stages during hblank/vblank — zero per-pixel math beyond counters. — **sidewinder**, timing-closed (v1.1 not flashed); forked into taffy. Wrap without per-pixel modulo: init a wrapped read pointer once per line (single mod), then one compare per pixel. v1.1 generalises it to a **bouncing run-along pointer** that IS the RAM read address (no compare in front of RADDR): per pixel it steps ±1 and at the inset bound R either wraps to L (Wrap) or holds R one pixel and turns (Mirror, period 2S, edge column doubled); per line one shared reducer folds the start column `(off − L) mod S` (Wrap) or `mod 2S` then `u ≥ S → C1 − u, down` (Mirror) in ~6 hblank steps per channel, so per-channel offsets (chroma split = three pointers Y/U/V) cost one pointer each. Kick the line sequencer at the active-video fall and advance phase/freq there (active lines only); a per-line `freq += twist` accumulator pivoted by a per-field `H/2·twist` product changes the wavelength down the screen about the centre row.

### Per-line envelope BRAM + DDA stretch
A 2048×10 BRAM of per-line envelopes {state, level} advanced once per line at separate attack/release rates; the effect is a DDA read address. Effect-agnostic — new modes just remap `env`. — **taffy**, HW-validated, v0.4 FINAL (48% LC). A passing slider can't show a complete gesture — add a one-shot armed→attack→decay→spent machine. Register the RAM read address (a combinational accumulator→compare→RADDR cone was 15.2 ns).

### Bounded per-row pull (tape flag), NOT a cumulative shear
A per-line DDA that accumulates a shear is unbounded by construction: even 1 px/line is a full screen width over 1080 lines, so almost the whole slider travel shreds the picture into diagonal slabs. Make the row offset a bounded POSITION function instead — an exponentially decaying pull seeded at the frame anchor (top-of-frame flag) plus a second one re-seeded at the head-switch band — amplitude = a fraction of the measured width, so the extremes bend the image instead of destroying it. Decay by `bend - (bend>>k)`: one subtract, no compares, and it is the shape real flagging has. — **deadchannel**, HW-validated (v1.5, 2026-10-09). HW-REJECTED in the cumulative form ("just diagonal streaks"). The linear "subtract a rate, clamp at zero" decay chains two compares and an add into one pixel-clock cone: 6 MHz. Fold per-line jogs (dropout tears) into the same offset so Y and chroma stay locked, and wrap with one conditional add/sub (offset < width).

### Line-average edge fill for displacement overshoot
Latch a 64-px line average (window starting past column 8 — capture sources carry black blanking-edge columns) at hsync; paint it for reads within ~6 px of either border. Trigger on reads of columns < C_EDGE, not just off-screen reads — that's what kills the seam; gate on displacement ≠ 0 or dry lines paint their own edges. — **lagoon** (HW-validated), redshift (taffy used it in v0.3, moved to the address clamp below in v0.4). Per-side variant (2026-09-13, the nine catalogue-combo warp programs mirage/halation/tremor/etchwarp/impasto/parallax/flutter/hueflutter/trapquant): sum columns 8..71 and width-72..width-9 separately, latch both averages at the avid fall, and re-latch them into the fetch stage on the delayed avid (`s_avid_sr(K-1)='1' and s_avid_sr(K)='0'`, K just past the fetch stage) so the colour swaps between lines, not mid-line; a side flag (`addr >= width`) pipelined with the bg flag picks left or right. ~60 FFs + six 16-bit adders, off the pixel critical path. SUPERSEDED the same day after a hardware look: the black blanking columns at the line ends still showed as a seam between picture and fill (and filter windows, pixel cells and keystone scaling widen it). What worked: an address CLAMP, `f_edge_addr` -- any read < C_EDGE (12) or > width-1-C_EDGE is redirected to that inset column, so the fill is the line's own edge pixel (no accumulators, no fill mux) and the blanking columns never reach the screen; windowed filters (impasto box sums, halation blur cascade) additionally hold the last active luma through blanking and re-sync their running sums to hold*N so the edge pixel itself is not darkened. Timing trap in the clamp: never compare the mirrored / wrapped address (it sits behind a negate or ±width adder) -- flutter fell 80 -> 67 MHz. Compare the SOURCE value against per-line registered bounds (-w_edge, 2w-1-C_EDGE, w+C_EDGE, C_EDGE-w, w+w_edge) in parallel with the adder, and let the final mux be the only serial step: back to 78-79 MHz.

## Accumulator & DDA engines

### Accumulator-pair projection (rotation without multiplies)
Never compute `x·cosθ + y·sinθ` from coordinates — advance the projection by `cos` per pixel and `sin` per line. Continuous angle free; scale = a per-frame multiply on the step; more effects = more pairs; carry 8 fractional bits. — **ribbon** (HW-iterating), **videosky**, **overlook**, **intaglio** stripe fields. Seed accumulators at 0 per frame (a periodic field's origin is invisible phase; centring costs 2 vblank mults). Per-line DDAs advance on ACTIVE lines only (CLAUDE: interlace). MOTION is free: seed with a per-frame OFFSET instead of 0 and step it in vblank — scale the step by the pitch (`<< octave`) so it moves in stops/frame at any zoom (ribbon v0.3 Flow). Exponential zoom = barrel-shift octave + 2^-x mantissa table on the step; any displacement added in accumulator units (a warp) must be multiplied by the same mantissa per frame, or it jumps 2:1 at every octave seam.

### Continuous-pitch/angle screens from a phase pair
Phase P (24-bit, wraps phase-continuously since 2^24 mod 2^16 = 0) with lines as level sets; Q = P rotated 90° gives crosshatch and a modulation argument free. Adding luma to the phase BEFORE the thickness compare = relief displacement — lines bow around the image, far richer than thickness modulation. — **videosky**, v1.0.0 FINAL, HW-validated (signed off 2026-10-06; 41% LC, 0 BRAM).

### One accumulator = cell id + sub-cell coordinate (+ free octaves)
A 24-bit DDA with 8 fractional bits yields sub-cell coords (15:8) AND cell id (23:16) — a rotatable, scalable lattice for the cost of a level-set. An octave is a left-shift of the warped coordinate; octaves share accumulator + domain warp. — **daedalus** (v1.0.0 FINAL except presets, 2026-09-30). Domain warp must land on the full coordinate BEFORE the cell/sub-cell split; never vary pitch per pixel — warp the domain instead. Stroke widths in sub-cell units shrink with the cell — at daedalus v0.4's default they were 0.67 px and dropped between scan lines; convert a pixel weight to cell units per frame (w_px × step) so Pitch changes spacing, not thickness (v0.5).

### Fractional phase-accumulator grid engine (sub-pixel zoom)
Per-pixel `phase += step` (Q10.18): cell = high bits, glyph column = mid bits; step from a reciprocal LUT with endpoint LERP; centring = −centre·step via a vblank shift-add multiply. — **ziffern**, HW-validated. Integer pitch + modulo centring = jitter; FRAC ≥ 18; pot ADC noise needs a glide filter, not more precision. Compute LUT differences from REGISTERED reads. Sim only at `--decimation 1`, warmup ≥ 12.

### Handshake-sequenced vblank multiply chains
Per-frame sequential multiplies that feed each other chain by done-handshakes (producer registers → consumer starts 2 cycles later), never all launched at vsync — a stale read renders fine but MOVES wrong, and no still sim catches it. — **ziffern**, **warpfield**, **pinboard** (all 11 per-frame ops on ONE 16×16 shift-add sequential multiplier, 18 clk/op — a parallel multiplier would be STA-timed at pixel clock for no benefit).

### Golden-ratio additive sequence for even-but-random placement
`pos(i) = base + i·0.618·range (mod range)` — low-discrepancy, no clumps, no visible grid (shift form: 1/2+1/8−1/128+1/1024 = 633/1024; the old 39/64 is a LATTICE at 64 sprites — exactly one per 1/64 slot). Interleaved subsets (depth layer = slot mod 4): step (φ+2)/4 = 0.6545 makes every layer its own golden sequence from ONE accumulator (4·step = 2.618 ≡ 0.618); the plain step gives each layer 4φ ≡ 0.47 and clumps (tessera v16: 576p busiest line 128% → 98%). — **tessera v12**, HW-validated. Random hash offsets CLUMP (Poisson) — that's the failure this replaces. Split accumulate-with-wrap across two stages.

### Multiply-free wave terrain via phase accumulators
Per-pixel phase accumulators reset to a per-frame scroll offset at line start; crest = sine + triangle at distinct wavelengths; per-layer fractional scroll = parallax; per-layer hazed palettes precomputed per frame (v1.0: one shared 3-stage lerp run by a vblank sequencer instead of 24 parallel chains — only −42 LC, yosys had already folded most of it). — **candyhills**, HW-iterating; v1.0 needs router1 (router2 69–71 MHz, router1 76–82 on the same netlist). **Continuous scroll speed without a multiply**: run K1 = 1..64 SUB-STEPS per frame in vblank (a down-counter loaded at vsync), each adding an exact 1/128-px constant to world-x; the crest phase adds ±STEP only when world-x crosses a WHOLE pixel (its integer LSB flips), so the phase is always floor(world-x)·STEP. That puts the terrain on the SAME integer grid as the stripes and sprites; a sub-pixel crest under integer-gridded bushes made every bush on a slope shiver ~1 px against the crest line (HW, candyhills). Reverse = add the two's-complement constant (one adder). Frame-lock LFOs — a free-running per-line LFO aliases against frame rate (bajaweave's phantom "sweet spots").

### World-anchored pattern coordinates
Any screen-anchored frequency/fan term slips against scrolling terrain (apparent speed = phase-rate/gradient); anchor the modulation to WORLD coordinates — e.g. fan = cos of the terrain's own crest sine at world-x. — **candyhills**, HW-iterating; ~10 HW rounds proved every screen-anchored variant slips. Keep world terms mod the pattern period (wrap shear). Each sin/cos LUT output is its own ~3-EBR table — carry the angle down the pipe and mux at the winner stage instead of duplicating.

### Stateless per-column sprite evaluation for clean edge scroll
Objects attached to scrolling terrain must derive per-pixel from current-line values only — any held per-slot anchor mis-evaluates the leading partial slot at screen-left (sprites pop). If a held anchor is unavoidable, seed it from column 0 at line start and keep it line-local. — **candyhills** bushes, HW-iterating. Sprite distances from a known centre can be CARRY-FREE: |dx| about a power-of-2 slot centre = the low bits xor'd with the inverted centre bit (centre between pixels), |dy| = ones'-complement fold (d xor sign) + OR-reduce clamp. Replacing subtract+negate+clamp this way was candyhills v1.1's top critical-path fix. **Better (candyhills v1.0):** when the attached terrain is a function of world-x only, sample it at every sprite CENTRE into a per-layer EBR keyed by world slot and read it back on the NEXT line (parity-banked, so a read never collides with a write). The anchor is then exact, with no extrapolation. Run the terrain pipeline 64 px past both screen edges in hblank (post-roll after avid falls, pre-roll from x = −64) so off-screen centres are sampled too; the old slope extrapolation lifted bushes near screen-left. Group placement by a 4-bit S-box PERMUTATION of the region index (density d ⇒ exactly d/16 regions lit) — a hash + threshold left a 16-region layer with NO groups at low density.

### Woven PM line oscillators: equal PIXEL speed per wave, parity weave, interpolated LFO
Two per-pixel sine carriers phase-modulated by per-line LFO sines. The LFO angle keeps 6 fraction bits and its value is sin + cos·frac (tangent interpolation: 0.55 vs 3.6 table-unit error), so a slow vertical sway moves by its exact rate every field instead of in whole-table-step judder. Layering: the stripe parity (bit 10 of an 11-bit PM phase, sampled at the trough) picks over/under per crossing — a checkerboard in (stripe1, stripe2) is a plain weave — with a continuous out = max(T, B·m/32) gain fade instead of a hard key, so nothing crawls as the pattern moves. The motion rule learned the hard way: advancing both sways by the same ANGLE per field (one shared drift) gives pixel speeds ∝ wavelength; when both flow the same way, the woven crossings' speed then cycles between the two wave speeds once per sway period (23 → 16 → 23 lines/field every 0.54 s) and reads as a STUTTER although each wave alone is perfectly uniform — diagnosed with an exact-RTL Python model of the per-line offsets plus a crossing tracker (`.bajaweave_work/motion_model.py`, `crossings.py`). Give each wave its own per-field rate L·step_k (equal lines/field, one serial multiply per field) so the weave translates rigidly, and add life as a small slow RELATIVE drift (≤ 3 lines/field, swinging over ~16 s) rather than a speed ratio. — **bajaweave**, HW-validated v1.0.

### Travelling-wave phase modulation for cloth sway (no per-column seams)
Swaying a fold pattern (fold-phase DDA, triangle → velvet LUT) by giving each 32-px column its own hashed sway accumulator tears the cloth: neighbouring columns can be up to ¼ fold out of phase, so every column boundary is a hard luma step.

Instead, add one continuous offset to the fold phase: two per-pixel triangle waves (periods ~640 px and ~1075 px, each with its own per-frame drift, travelling in opposite directions), summed, × a per-line amplitude, /2.
- The amplitude is set at avid fall from the active-line index: 16 at the top, ~49 at the bottom, i.e. pinned at the rail and billowing below.
- Its slope stays far below the fold frequency, so folds bunch and spread but never fold back on themselves.
- Index the hem/rounded-bottom LUT with the SWAYED phase, or the hem bulges stay put while their folds move.
- "Sway off" is best done by freezing the drift: the drape keeps its irregular shape instead of snapping to perfectly regular folds.

Cost ~60 LC: one 11×8 multiply in its own stage, plus two 16-bit DDAs. — **blacklodge v1.0.0**, HW-validated.

### Rotated-coordinate field for slanted banding
Register `y_rot = y − x/2` once; use its bit-slices for band selection, border tests, and per-band seeds — every derived feature automatically shares the slope. — **solitude**, user-approved (52% LC).

## Noise, texture & hashing

### Hash discipline
At least one integer ADD is load-bearing (xor/rotate is linear over GF(2) → visible lattice structure: whole rows merging, diagonal weave); use exactly as many adds as needed (each lengthens the critical path — split multi-add hashes across stages). Prefer shift-and-ADD bijections (odd multiplies ×9/×33/×5, xor-with-own-high-bits) so distinct cells never collide; threshold the TOP bits (best-mixed). Never feed a hash `x` and `x xor const`. Spot-check every new hash numerically (CLAUDE). — **mondrian**, **inferno**, **daedalus**.

### Step game RNGs per frame, never per clock
An 8-bit LFSR free-running at the pixel clock and sampled once per frame aliases against the frame length. 1080p is 2,475,000 clocks = 225 (mod 255), so the samples walk a 17-state cycle. Bomb columns 3/4 took ~11 of 17 picks, and at high drop rates all the bombs seemed to come from one alien. Step a 16-bit Galois LFSR k times at a fixed FSM phase (k = bits consumed per frame), and map to 0..5 by "first 3-bit field < 6". A model gives 14–19 % per column. — **invaders** v0.3, HW-validated.

### Dither/decay randomness rules (three HW-observed failure modes)
(1) `hash(addr)+framecount` TRANSLATES — fading content appears to scroll; use `hash(addr) xor lfsr_snapshot`. (2) x-LSB xor line-LSB dither reads as a CHECKERBOARD near thresholds; use x bits ≥1 xor a per-line-stepped LFSR. (3) An avid-gated LFSR FREEZES in vblank — a vblank FSM sampling it gets a periodic value (global pulsing); sample during active video and latch. — **fireworks/lagoon/inferno**, all HW-caught, all sim-invisible.

### Aperiodic masks: no bit-hash, no global time bits
Never XOR a global time bit into a per-pixel presence mask (the whole row flashes as one line); any XOR/AND of coordinate bits is periodic (micro-motif tiles visibly). A hand-placed aperiodic 64-entry constant mask ROM (maps to LUTs), indexed by `x xor small-y-shift`, LC-neutral vs the hash. — **pixelbeach** foam saga, HW-confirmed failures.

### Patch-level randomness, never pixel-level
Organic "hand-made chaos" = groups of parallel regular lines whose POSITION/SIZE are random (latched per run, carried via a per-column line buffer) with zero per-pixel noise — per-pixel grain reads as static on luma, full stop. — **etchplate v3.1**, sim-verified vs reference. DEAD-END: independent per-line runs + per-pixel dropout (rejected on HW as "static applied to luma").

### Servo-locked per-line gradient
`g += const` per line; at vsync `gstart += target − g_measured` — converges to an exact screen-height fit in any mode, zero division. The servo is NOT idempotent: run it on the first vsync edge after active video only (serrated vsync re-servos against a k-line "frame" → whole screen at full heat). — **inferno**, HW-validated (guard added v1.1); variant in lagoon.

### Anti-terracing on tall noise cells
Tall lerp cells hold the smoothstep fraction constant for many rows; the terrace boundaries align across columns and read as contour lines. Finer fraction table (64-entry) on tall-axis lerps + LFSR dither below the step size; ridged folds DOUBLE step contrast — dither those too. — **inferno v3.5**, HW-iterated.

### Anti-lattice: per-column vertical phase
A cell grid always reads as rows of dots and per-cell jitter can NEVER fix it (a feature only wanders within its own cell). Slide each cell COLUMN vertically by its own random phase — boundaries move with the content. — **morsel**, HW-tested. Per-row horizontal + per-column vertical together is circular — pick one.

### Binary mezzotint comparator
Luma vs dithered-threshold compare re-renders the image as grain density. For density to TRACK the picture the dither must span the whole luma range (±~440): grist v0.1 used knob-set grain (±92 at defaults) and its "mezzotint" climax was a hard two-tone posterize with a grainy contour. Fix: renormalise the octave gains per frame (vblank serial divide `A = 7040/span`, ramped by the develop slider) so the climax span is fixed and the amount knobs become the octave MIX (fine spray vs aquatint); the pixel path keeps small multiplies. Corollary for coordinate hashes: a 16-bit pack of x(9:0)/y(9:0) repeats at +1024 on 1080p — fold the high bits in with an ADD (an xor only xor-translates a GF(2)-linear pack). Soft variant without a line buffer: hash rows y..y+3 in parallel (frame rows, so it stays field-coherent) + 3 horizontal delay taps = a 4×4 box. That turns pixel stipple into organic ink blobs, and its Nyquist zero removes the row-alternation peak of xor-rotate-add hashes (254× the noise floor in the fine nibble). Smooth the block octave bilinearly, and draw pits from a separate box-filtered nibble so they become blobs too. — **grist v0.3**, HW-validated (FINAL 2026-10-03, Soft is the chosen default), 6/6 seed 1 at 69% LC.

## Edges & filters

### Shared multi-tap luma window with uniform mode latency
One 5×5 window (4 chained line buffers) feeds ALL operators; simpler modes pass through the deepest stage so every mode shares one latency and downstream controls behave identically. — **edgelab**, timing-closed (v1.1: 92% LC, 29 EBR). Shift chains crossing hblank leak left-edge garbage — blank at THREE offsets (window fill, accept guard, output guard).

### Kirsch compass without 8 wide ALUs
`max_i |8·Si − 3T| = max(8·maxSi − 3T, 3T − 8·minSi)` — one min/max tree over the ring triple-sums replaces 8 directional convolutions. — **edgelab**.

### Direction hue without a direction line buffer
When the displayed edge row sits lines behind the detector (NMS + dilate windows each add a line), don't line-buffer the direction: run a second small Sobel on the LATER rows of the same luma window (edgelab: rows 2..4, one line below the displayed row — invisible for a hue), fold to the first octant (`a=max, b=min, swap flag`), bin with shift-add ratio tests (`5b<a` ≈ tan 11.25°, `3b>2a` ≈ tan 33.75°) + the gx/gy signs → 16 sectors on a 360° wheel, then a valid-gated 4-bit shift chain lines it up with the output column. ~220 LUT, 0 EBR. Complementary hue = flip the sector MSB. — **edgelab v1.1**, timing-closed, not HW-tested.

### Canny NMS diagonal-pair sign check
With gx = right−left and gy = lower−upper (screen y down), SAME-sign gradients point down-right → compare DR/UL; opposite signs → UR/DL. edgelab v1.0 had them swapped (thins along the edge, not across → speckled diagonals); nothing but a careful sign trace catches it. — **edgelab v1.1**.

### Scale-by-level without multipliers
Three 10×8 soft multiplies (colour × gradient level, one per Y/U/V) cost 573 LUTs on HX4K — more than the hue + morph features combined. A 3-term shift-add on the level's top bits (`m7·p/2 + m6·p/4 + m5·p/8`, 8 steps) costs ~70 and looks the same on an edge glow. Ablate before assuming a "small" multiply is small. — **edgelab v1.1**.

### Quarter-res streaming Canny for edge EXISTENCE
Detect at 4-px column granularity: 4-px average → [1,2,1] blur → Sobel → |gx|+|gy| → double threshold → line+left hysteresis; skip NMS when only presence matters. MUST be a 4-stage pipeline (monolithic = 46 MHz). Suppress accepts on the first 2 active lines (buffers hold last frame's bottom; pixel_y counts from vsync so `y<2` never fires). — **inferno v3**, HW-iterated.

### Block-float gradient normalization
One shared priority-encoded shift (from |gx|+|gy|) scales both gradient components: direction exact (hue/env lookup, no atan2), magnitude log-compressed. Split shift-select and barrel+clamp into two stages; the height field must be 10-bit + h-blurred (8-bit contours ring once normalized). — **mercurial**, timing-closed. (The field-overlay GENRE is rejected — reuse the trick, not the genre.)

### Gradient-direction bucketing with hysteresis
Bin straight to 4 symmetric directions with two compares, |gy| vs |gx|·tan22.5 (≈>>2+>>3+>>5) and |gx|·tan67.5 (≈2·+>>2+>>3+>>5); 8 bins rounded to 4 skews every boundary ~11°. Direct 22.5° selection shreds soft shading (DEAD-END). Hold-last-strong-direction (~63 px run) so low-|∇| zones inherit direction instead of striping. Direction floor ~24, not ~96. **Mirror trap:** a Sobel gy taken as (row above − row) points UP, while stripe/accumulator families step +sin per line DOWN. Assign the diagonal bins in screen space (signs differ = 45°), or diagonal strokes run ACROSS the form while H/V look right. Opposite-slope sim checks can't catch this, so model it numerically (intaglio v0.1 shipped mirrored). — **intaglio**, timing-closed. Companion: octant-fold + 3 parallel shift-add tangent compares = 5-bit hue sector, no atan2 (**turpentine**).

### Thick class-boundary outline via a {class, age} line buffer
Tracing the border between two per-pixel classes in a streaming pipeline: horizontal strokes from an N-bit class shift-history (differ-from-any tap = N px stroke), vertical strokes from a ping-pong line buffer storing {class, age3} — each line: edge fires → age = thickness; else age = max(age−1, 0); outline = edge or age ≠ 0, so the stroke extends `thickness` lines downward with no multi-line window. The age update is a legal cross-line RMW because ping-pong reads last line's bank while writing this line's. 4-bit word × 2048 × 2 banks = 4 EBR; read issued 2 stages early + fabric re-register; write trails read by the pipe offset so addresses never collide. Gate h-edge with `px ≥ 8` (blanking garbage in the history) — **cleave**, timing-closed. Stroke is asymmetric (below/one side of the edge); reads as intentional jitter on organic class fields.

### Edge fringes want a FRINGE, not a slab
An "edge throws a colour sideways" program reads as blocks, not edges, the moment the throw gets wide: neighbouring throws swallow each other and whole regions go flat (additively magenta for an opposed pair), so the eye sees the fill, not the contour. A thin band -- 0.4%..2.0% of screen width, ~8..38 px at HD -- reads as the edge treatment the idea promises. Diagnose before restructuring: a bit-faithful Python model of the detector showed marks firing on 99-100% of the lines a contour crosses, so "the colour doesn't run down the whole edge" was a width problem, not a detector problem. A flood-to-next-edge mode then needs its own much larger cap (16x here) or it can never cross a region. — **parallax**.

### Narrow the CONCEPT before optimising the implementation
Parallax threw a colour band off every detected edge, which forced a reverse-scan pass, a mark buffer, a paint-map buffer and two countdown engines just to make the leftward throw causal. Restricting it to the LEFTMOST and RIGHTMOST edge of each scanline -- one glow off the subject's left outline, one off its right -- deleted all of that: both extents fall out of the single forward analysis pass (first mark wins the left, every mark overwrites the right), so a line's whole geometry is two column numbers latched at the turn-around and resolved into four bounds, and the pixel path is compares against registered values. 14 -> 10 EBR, a pipeline pass and a scanline of lag gone, and it is the look the brief was actually describing. Watch for: with leftmost-wins, ONE spurious mark captures the entire left side -- the blur/gradient registers still hold the previous line's tail at line start, so the inter-line step reads as an edge at column ~0 and pins the glow to the frame edge. Reject marks inside a margin of the window half-width (18 samples here). — **parallax**.

### Reverse-scan pass for non-causal sideways dilation (and flood-to-next-edge)
Extending a mark to the RIGHT on a raster is a countdown; extending it LEFT needs lookahead, and "flood until the NEXT edge" is worse — it depends on the *nearest* edge ahead, which is not a countdown in forward order at all (a reload-on-mark counter tracks the FARTHEST-ahead edge in the window, not the nearest). Rather than delay the display through a FIFO as deep as the maximum reach (~650 clocks for a third-of-screen slab — past half an SD line, and it moves the picture inside its own sync), store the line's marks in a 1-bit buffer and re-scan it RIGHT-TO-LEFT during the following line: reversed, both the leftward dilation and the flood rule are ordinary causal countdowns. Write the finished paint map to a second buffer, walked forward by the display pass one line later. Cost: 2 ping-pong bit-buffer pairs (4 EBR at 1024×4), slab map lags the picture by 2 scanlines (invisible), display path undelayed so the source stays bit-sharp and program latency stays flat across modes. — **parallax**, kept here because it is the right answer whenever every edge must participate; parallax itself no longer needs it (see the entry above).

### Half-resolution analysis path beside a full-rate display path
When a control path only decides *where* something goes, run it on pixel-pair averages: every buffer halves (1024-deep covers HD), every shift register halves for the same spatial reach, and marks quantize to 2 px — invisible on a slab boundary. Lets a 16-tap box blur cover 32 px for 17 taps of FFs. — **parallax**.

### Centre the mark on the gradient run, don't hang it off the corner
A gradient run straddles the edge it belongs to, so its leading corner sits half a run-width OUTSIDE the object; anything hung off that corner leaves a visible dark halo between the fill and the subject, and the halo grows with the blur. Take BOTH marks (rising and falling edges) at their run's leading corner and push them through a shift register tapped at a blur-tracking delay — written at the pipeline's own column, the delay lands each mark mid-run, i.e. on the edge. One 8-deep 2-bit SR + an 8:1 mux. — **parallax**.

### Additive-light collision for opposed colour overlays
Where two overlays collide, summing them as light (Y and both chroma offsets added about their neutrals, then clamped) beats last-writer-wins: it is what a real anaglyph overlap does (blue over red → magenta) and it generalises — green + magenta sums to white. Compute the sum once per field into a third palette register, so the overlap is one flat colour and the pixel path stays a 4:1 mux. — **parallax**.

### Separable gradient still needs SYMMETRIC vertical support
A horizontal difference on one row plus a vertical IIR on the RESULT is not a substitute for a Sobel's `[1 2 1]` column sum: the IIR is causal, so it only reaches 41% of a gradient on a contour four lines tall, and a curved contour moves column every line or two -- so faces and organic shapes are attenuated hardest, exactly where the program needs to be boldest. Two line buffers and a real column sum fix it. Where vertical persistence is still wanted, make the IIR **rise instantly and decay slowly** (`gs = gh` when |gh|>|prev|, else `prev + (gh-prev)>>k`): full sensitivity on an edge's first line, smoothing only on the way out. Resolve the rise/decay decision and the pre-shifted step ONE STAGE EARLY -- the compare feeding a 3-way mux feeding the add was the routed critical path (71.4 MHz). — **parallax**, after livewire.

### Size an edge threshold to the steps real pictures contain
A linear threshold sweep across the plausible gradient range wastes most of the knob: a 0..200-luma sweep demands a 104-luma step at its midpoint, which almost nothing in a real picture has, so two thirds of the travel is dead. Square the complement instead (`thr = 3 + ((1023-k)^2 >> 14)`): 132 luma at the floor, 36 at the centre, 6 at the top. Verify against a printed contrast wedge (bars stepping 6/12/20/30/45/64/90/128 luma off mid-grey) and report which bars fire per knob position -- eyeballing a photo cannot tell you this. — **parallax**.

### Threshold hysteresis + edge qualification for real video
±24-luma hysteresis on any 1-bit threshold, plus qualification (a transition counts only if the run it ends was ≥6 px) — kills micro-edge chatter that chains into full-width bars. — **etchplate v3.2**. Clean synthetic test images hide this class entirely.

### Edge cracks folded into the emboss stream
A gradient emboss already computes `gx`, `gy` and carries a relief value down a delay line into the dough adder. Thresholding `|gx|+|gy|` and *replacing* that delay-line value with a fixed negative depth (and adding a lip value on the next pixel) draws the picture's hard edges as cracks in the surface: no new delay line, no extra add at the consumer, ~90 LC. Tie the threshold to the relief knob so one control sets "how many and how deep". — **morsel** v0.2, timing-closed only.

## Video processing

### U/V swap once at each boundary
Author internals in standard BT.601; swap chroma ONCE at the input latch and ONCE at output compose; passthrough stays identity. — **gilt** (HW-confirmed: standard-authored gold rendered AQUA), prism, cga. Pre-swapped stored palettes also work but preview swapped in sim. Turpentine has NO swap and is likely wrong on HW — don't copy it.

### Pair-averaged chroma for 1-px coloured strokes (422 packer parity)
The core's 444→422 packer takes U from the EVEN avid-relative pixel of each pair and V from the ODD one (`yuv444_30b_to_yuv422_20b.vhd`). An isolated 1-px stroke pixel on a neutral ground therefore reaches the encoder with ONE chroma axis and its hue swings to that axis — blue strokes read red/yellow in places, hidden once lines are ≥3 px because interior pairs are complete. Cure: give BOTH pixels of a 422 pair the pair-averaged chroma (alpha = hits in the pair, 0/1/2 → ground / (ground+line)/2 / line), luma stays per pixel so edges remain sharp. The even pixel needs one pixel of lookahead: keep show undelayed (= next), delayed-1 (= this pixel) and delayed-2 (= previous), mirror the packer's phase on the delayed avid (0 at the first active pixel, toggle per active pixel, 0 through blanking), and grow the sync/video pipe by 2 (lookahead + registered output) for ALL modes. A 1-2-1 chroma LPF is NOT a substitute — it still weights the two axes 1:2. ~150 LC. — **bishop v0.4.2**, HW-CONFIRMED 2026-10-08 ("colors are fixed"). Applies to any program drawing thin generated colour (titler, matrix_rain, fray, cga, spiroscope).

### Replaying a stored line on other rows: per-row U/V swap by hsync parity
A program that shows a line CAPTURED on one row on DIFFERENT rows (drape, slit-scan, line-hold, frame stores) must correct for chroma phase. Two facts combine: the encoder decides Cb vs Cr by counting from hsync, and the program's U/V labels follow avid. The hsync→avid offset parity varies between rows on analog sources. Record it per line: toggle a bit every clock, reset it at hsync, and latch it at the avid rise. Latch the capture row's parity too, then SWAP U↔V at output on rows where the two parities differ. Keep each pixel's own U/V pair together (3 RAMs). — **drape v1.3**, HW-VALIDATED 2026-09-30, ~2 LC.
DEAD-ENDS:
- No swap (v1.1): an overall green/magenta cast. Each colour sits next to its own swap, which averages to U=V.
- One raw-4:2:2 chroma RAM with a parity-forced read LSB (v1.2): it saves 5 EBR, but it mixes Cb/Cr from pixels up to 4 apart, giving thin dark lines at every colour edge.

The image sim feeds 4:4:4 and cannot show any of this. Model the core chain in Python instead: hsync-locked 4:2:2 → `yuv422_20b_to_yuv444_30b` (u = c[odd], v = c[odd+1]) → program mapping → packer (U on even, V on odd) → encoder by hsync parity. Count wrong-chroma pixels at bar edges; plain passthrough already smears ~2 px.

### Whole-glyph ROM fetch: a free 3x3 neighbourhood for text/sprite edges
Store the font/sprite as ONE word per glyph (`128 entries x 64 bits` = an 8x8 glyph, 3 EBR) instead of `char x row x 8 bits`. One read hands you rows y-1, y and y+1 at once, so the full 3x3 neighbourhood is a couple of OR gates: `dil = r(y-1) or r(y) or r(y+1)`; `dil3 = dil or (dil sll 1) or (dil srl 1)`; `outline = dil3(7-x) and not self`; `shadow = (r(y-1) srl 1)(7-x) and not self`; `hilite = (r(y+1) sll 1)(7-x) and not self`. That buys drop shadow, black edge, hollow and emboss — the whole 1980s character-generator style set — for ~400 LUT4. `srl` moves toward higher column index, i.e. RIGHT. — **titler v2**, HW-VALIDATED, 6/6 at 73.5-87.5 MHz. Extract rows with an explicit 8-way `case` function; a computed slice (`g(63-8*y downto 56-8*y)`) is a synthesis hazard. THE TRAP: a 1-px edge stays inside its cell horizontally (glyphs in cols 1..5 of 8) but NOT vertically — the row above a glyph's top row belongs to the cell above, holding a different character, so every glyph silently loses the top of its outline. Fix by giving each text line its OWN top-margin row (titler's block is 18 rows: 0 = line 0 margin, 1..8 = line 0, 9 = line 1 margin, 10..17 = line 1) and on a margin row fetching that line's character with `r(y-1) = r(y) = 0, r(y+1) = glyph row 0`. Half-extent then becomes `scale*9`, not a pure shift — resize BEFORE shifting.

### Active-region counters make position knobs mode-portable AND the DDA interlace-correct
Reset the horizontal counter at `avid_start` and advance it only while `avid`; advance the vertical counter only at `avid_start`. Every geometry term is then in active-picture coordinates, the vertical DDA automatically obeys "advance on ACTIVE lines only" (no 30 Hz interlace twitter), the text/shape band can never land in blanking, and the height counter does NOT zero on blanking hsyncs — so it is safe to sample at vsync, unlike a raw line counter. Compare the vertical counter BEFORE its increment: that value is the index of the line just starting, which is what makes origin 0 work. Width = `h_act + 1` at `avid_end` (h_act stops on the LAST active index). Feed knobs through `(knob * meas_w) >> 10` on a shared serial shift-add multiplier in the vblank sequencer — a plain `*` is still STA-timed at the pixel clock and will not close. — **titler v2**, HW-validated. Symptom this fixes: v1.0 hard-coded a 4092-px span and crammed ALL of SD's placement into the bottom ~20% of the knob (~27% on HD).

### Half-rate 8-bit chroma line buffers (dodging the 30/32 EBR cliff)
Three full 10b×2048 dual-bank line buffers cost 30/32 EBR — right on the router congestion cliff. Store Y full-rate/10-bit but U and V at half horizontal resolution and 8 bits (write addr `px(10 downto 1)`, read `addr(10 downto 1)`, expand with `shift_left(resize(u8,10),2)`): 18/32 EBR, and neutral 128 expands to exactly 512, so there is no missing-neutral-chroma trap. Chroma displacement then quantizes to 2 px, invisible on smear/misconvergence effects. — **deadchannel**, HW-validated. The buffer has no write enable, so a half-rate write address simply point-samples the odd pixel.

### Decode every control free-running, never in one frame-event branch
Latching all parameters in a single `if <frame anchor>` branch makes every knob and switch depend on that one event firing. When it stopped re-firing on real video timing, deadchannel rendered perfectly but was completely deaf: picture fine, all twelve controls dead. Read the SPI registers CONTINUOUSLY instead and pipeline the derivation one op per stage; only genuinely per-frame state (line count, band position, seeds) belongs in the anchor, backed by a line-count ceiling so housekeeping survives a stuck anchor. Costs nothing and it ran FASTER (77 → 83 MHz) because the anchor cone shrank. — **deadchannel v1.2**. Diagnostic: flip an unmissable switch (Invert). If it does nothing, suspect the latch branch, not the pixel path.

### Frame anchor on the vsync EDGE, never the vsync level
The probable root cause of deadchannel's dead anchor is `if vsync_n='0' then clear; elsif avid='1' and not armed then FIRE`, where the level clear has priority. If vsync reads low across active video, the anchor NEVER fires. Symptoms: per-frame state frozen and the line counter free-running to its wrap (a band flashing and scrolling every ~3.8 frames). A line-count ceiling placed inside that `elsif` cannot rescue it. Use the house pattern every HW-validated program uses: fire on the first vsync falling EDGE after active video (saw-active flag set on `avid`, cleared on fire), which fires once per frame whichever way the pulse sits. Put any line-count ceiling OUTSIDE every vsync condition, and sample `field_n` during active video rather than at the edge. — **deadchannel v1.3**, HW-validated: the band rolled correctly for the first time.

### Random glitch-band events with exponential knob curves
A dropout band reads as tape damage when it is an EVENT, not a smooth roll:
- On a random frame (hit chance from a table), it jumps in at a random height (`rand8 × measured height >> 8`) and a random size.
- It holds 1–8 frames while scrolling in a random direction, then vanishes.
- Every draw comes from a second, independent LFSR, formed free-running; the anchor only samples it.
- Inside the band, each line draws its own recipe at hsync, and every field is an identity value outside the band, so the pixel path applies them unconditionally:
  - read-address AND mask (block pixelation; chroma follows for free);
  - luma AND mask (posterize);
  - U/V swap;
  - negative;
  - static.
- On top of the recipe: per-line ±slip, plus white dropout comets (a per-pixel 1/1024 trigger loads a 7-bit countdown; luma = 640 + 4·count fades along the tail).

Map the knob EXPONENTIALLY: hit chance 2^(12k−12) and size 0.12·10^(2k−2) of the height, in 32-/16-entry tables. That keeps the lower half subtle (1 event per 30 s at 10%, ~1/s at 50%) and makes 100% every frame and ~190 lines. Ron asked for exactly this after a linear K1 was busy everywhere. — **deadchannel v1.4/v1.5**, HW-validated. Origin: Ron PREFERRED the accidental flashing bar of the broken v1.1 anchor. A bug look the user likes is a design brief: rebuild its character deliberately.

### Per-line geometry belongs OUTSIDE the per-line decision cone
Anything evaluated at hsync or active-video-fall is timed at the pixel clock, so a per-line test like `|aline - band_pos| < half` (subtract → abs → two compares, all chained) becomes the design's critical path — 19 ns / 51 MHz in deadchannel. Register the four band EDGES once per frame and per line do a plain register-to-register compare; likewise run knob→amplitude→rate derivations as a free-running one-op-per-cycle chain and let the frame anchor merely SAMPLE the result. Same design, 51 → 76 MHz, no feature cut. — **deadchannel**, HW-validated. Corollary: reading an SPI register inside the anchor cone drags a deadband, a variable shift and an abs onto that path.

### Quadtree by parallel independent hashes
Cell size resolved from 3 INDEPENDENT coarse-block hashes (64/32/16 px) evaluated in parallel — timing-shallow, neighbours subdivide independently. Normalize in-cell coords to 0..63 so motifs are size-independent. — **gilt**, timing-closed (77% LC). Quadtree squares can't make tall shapes (tall bars keyed to absolute sheared x across stacked cells). A second overlay closed easily BECAUSE it mirrored already-closed stage shapes.

### Grid-aligned cell merge (cheap "not-square")
Cells stay grid-aligned; each hashes grow-right/grow-down bits; a pixel resolves against the 2×2 up-left neighbourhood — coverage is pure hash-bit logic, no coordinate compares. — **mondrian**, HW-validated (v1.0.0). DEAD-END: porting quilt's jittered off-grid overlap (+1300 LC, past the congestion cliff). Mondrian's continuous-grid HD Dual hit a routing wall at only 65% LC (v0.1 shipped a hybrid vmprog with a stepped hd_dual slot). The likely cause (inferred, not ablated) was Video Fill reading `pipe(16).y/u/v`, which kept ~500 FF-only delay cells alive: it was the only large netlist change in v0.2, which moved that delay into a 16-deep EBR ring, and the wall was gone (61% LC, router2 completes). router1 then closed all 6 configs on seed 1 (HD Dual 76.95), so one source builds everything. When a config walls well under 90% LC, look for a deep FF video pipe before blaming the netlist.

### Per-cell-column luma hold for stable region decisions
A decision that must be identical across a patch can't read per-pixel luma: sample one luma per coarse cell into a small RAM and hold. For decisions used within the same cell row, ping-pong by cell-row parity (one row of lag). — **morsel**, HW-tested (quilt v0.1–v0.4 used it too, but as a synthesis program it never saw input on HW).
Upgrade when patches can span cells (overlapping rectangles) or the sample must be clean: a **per-frame block-average map** — 32×32 box/IIR luma averages written into one bank of a 2-bank map, the picture reading the previous frame's bank, so every reader of a block sees one value all frame and a cell can read its CENTRE block (no row lag, no half-cell offset). Each overlap candidate carries its OWN cell's decision (prefetch own + up row per column; the left pair is the previous column's), so a rectangle spanning grid cells stays whole. An IIR down the block's lines (est −= est/16, += line avg) stays a true mean for a partial bottom block row where a box sum would need a divide. 8 EBR for 1080p at row·60+col. — **quilt** v0.5, timing-closed only.

### Case-selected fixed bit-slices instead of variable shifts
Scale logic rarely needs a barrel shifter — extract the 2-bit ramp field `coord(s:s−1)` + a few bits via ONE case over the scale setting. — **quilt**, HW-tested (−120 LC, 8 barrels removed, +3–8 MHz).

### Separable axis tests for 2D regions
Factor region membership into independent X-only and Y-only bits in one stage, AND/OR-combine in the next — a fused 2D comparator tree was a ~16 ns wall (55 → 101 MHz). — **revenant**, timing-closed. Sibling: build all N clamped candidates in parallel + shallow-tree argmax (**quilt**; serial argmax = 35 MHz). Better still, take the carry chains out of the argmax stage: when the priority is `cov & rest`, register the 6 pairwise `rest(i) >= rest(j)` compares in the stage that computes cov (they don't depend on it), then `vv(i) >= vv(j) = not cov(j) or (cov(i) and p_ij)` is one LUT and the winner two more — quilt v0.5 HD HDMI 74.9 → 81.6 MHz, all configs seed 1 (timing-closed).

### Phase-field streaks + per-column drip state (pixel sort)
Fake the streaks, store only the drip. The streaks are iso-bars of a PHASE field: a per-pixel DDA (pitch continuous via `xacc += step`) plus luma bend, ripple and row wander, all in phase so their amplitude is in pitches. Bits 24..16 are the bar index, whose hash locks the bar's hue down the column; the 3/4-duty bar is `not(F15 and F14)`; brightness rides live luma. Mix two diagonal triangle waves + a leaky random walk for organic bends (one triangle wave = hard zig-zags).

The real vertical sort does NOT need full-colour columns (the old "blows EBR" claim was wrong): store a compact per-column STATE of hue idx3 + bar bit + energy4 = 8 bits, 2048x8 = 4 EBR. Below each pixel the state from above wins while (energy minus a stochastic decay of 1, odds k/128) beats this pixel's luma energy, so bars pour down over darker image and sky and end raggedly; k sets the drip length. The new state depends on the read, so use PING-PONG banks by row parity (8 EBR): yosys can't infer read-modify-write on one RAM. Substitute the drip before the palette and saturation stages so it shares them. — **glitchscape v1.0.0**, signed off 2026-10-09 (HW-validated).

Related, same program: luma-band CONTOUR filaments (band = lc*k>>10; white where band != left or != above; 2048x4 single-bank line buffer) replace noise-gated iso-lines that boiled; JPEG MACROBLOCKS = one colour per 16x16 block, point-sampled on the block's first row into a 256-entry EBR (u8,v8 + y8: power-of-2 widths, all bits live; chroma u8&"00" keeps 512 exactly neutral) and held across the block.

### Move the threshold, not the gain
Knob-controlled intensity of a thresholded feature: move the THRESHOLD, not a per-pixel multiply on the field. Replace K² knob response with a piecewise-linear shift knee (~120 LC per multiply saved, feels identical). — **lagoon**, HW-validated.

### Per-frame constant → shifted-bit-term decomposition
A frame-constant narrow multiplier operand (mix 0..8): replace the multiply with per-bit shifted terms summed across two stages. — **doom v2**, timing-closed; **ziffern** gains. Companion: inline 3-stage lerp `a + (t8·(b−a))>>8` replaces SDK `interpolator_u` (t=0 exactly dry, ≤4 LSB full-wet error; freed ~400 LC and took HD Dual 69→87 — **turpentine**).

### EBR lookup-table quantizer with a free-running serial builder
Per-pixel nearest-of-N colour (N parallel |dy|+|du|+|dv| units + argmin tree) is ~3000 LC for N=16. Instead index a 1024x4 EBR by luma(4b)&u(3b)&v(3b) and let a 5-state serial builder recompute every cell forever (~1.3 ms/pass) against a LUT-ROM palette; the pixel path is LUT read -> index -> palette ROM read. A Bayer dither ahead of the lookup hides the cell edges. Palette switches rebuild over the next pass, no vblank budget. ~250 LC + 1 EBR. — **croupier**, timing-closed only (89% LC, 25 EBR).

### Memoryless 2D area averaging as anti-flicker
Quantization-boundary flicker is best fought with memoryless spatial averaging (cannot oscillate on a still image). — **c64**, HW-confirmed. True 2D averaging has 1-cell-row latency (blank the top row via a stale flag). DEAD-END: c64's cross-frame per-cell hysteresis grid made flicker worse. For palette-match flicker specifically, CGA's margin hysteresis + two-field debounce IS HW-validated (M=1 → 10–50×, M=2 → ~400× fewer flips; re-seed the grid on contrast/mode change or decision controls go dead under Freeze) — the mechanism difference matters: margin-on-challenger, not keep-last-unless-beaten.

### Value-snap hysteresis for DITHERED quantizers (modular levels)
Decision hysteresis (keep the stored palette index unless beaten by a margin) breaks under ordered dither when several cells share a tile: each cell is judged against another cell's dither phase, patterns scramble and incumbent flips cascade into flicker. Store the tile's undithered LUMA instead and snap each cell to it before adding the dither. Keep 16-code resolution in 4 bits by storing bits 7:4 only (value mod 256): `d8 = y(7:0) - (S & "1000")` as a signed 8-bit wrap resolves the ambiguity from the live pixel (noise is a few codes, not 128); snap if |d8| <= 24 (the stored centre is within 8 of its source, leaving 16 codes of noise margin). DEAD-END: storing bits 9:6 (64-code levels) posterised gradients into thick flat bands, seen on hardware as bars. Switch the grid between the two meanings with the dither setting, re-seed on the switch, and hold both off while the change is pending. — **cga** v2.0.6, HW-validated. Related, unexplained: 1-pixel cells + dither gave noisy colour bars on HDMI that ran partway -> right edge (not Stability, not rail chroma, not avid dropout); cga removed the 1-px size.

### Cell-jitter primitive clipping fix
A jittered primitive drawn only in its own cell clips into half-moons: either test neighbour cells or size the jitter budget so r + |j|max + 1 ≤ cell half (shrinking as it's thrown reads as intentional). Pipe each pixel's own cell context down the render lanes; guard unsigned subtractions in size math (a wrap becomes a giant primitive). — **turpentine v3**.

### Content field: drive ornament STRUCTURE from a blurred copy of the input
Build a low-res blurred copy of the incoming video (one-pole IIR along the line for horizontal, a decimated single-bank line buffer feeding a second IIR for vertical — luma and both chroma axes, 3 EBR at 512×24) and drive every *structural* decision from it instead of from screen coordinates: warp the pattern lattice by it (luma displaces x, chroma displaces y, up to ±128 px), XOR a quantized *material* id (tonal band + chroma sector) into the cell-identity hash so pattern boundaries land on image contours rather than cell edges, and let brightness/saturation set cell size, ornament density, shade and accent hue. The field is never drawn — it only gives the generator something to follow that is not a coordinate. — **gilt v0.7**, timing-closed (88% LC, 9 EBR, HD 75.3–77.9 MHz). Cures the "looks mathematical / follows a grid" failure that a quadtree over (x,y) could not: irregularity computed from position is still arithmetic laid over the picture. Keep the field pipeline FREE-RUNNING beside the pixel path — its values arrive 3–6 px stale, invisible under an 8 px blur, and that is what keeps the pixel stages depth-neutral (per-frame constants become registered per-pixel signals with no added depth). EBR trap: read addr = x>>2 and write addr = the same group delayed 2 cycles, enabled only when x mod 4 = 3, so read and write never touch one word. Open risk: no hysteresis on the band index, so watch for shimmer where the smoothed value hovers on a material boundary. **v0.8 follow-up (HW feedback: "still too much grid"):** the field alone was not enough — the *rendered* lattice lines (cell seams) and any motif keyed to a fixed pitch (16 px bars) re-introduce the grid no matter how the lattice is warped. Delete seams outright; give bars a per-cell axis/pitch/phase; turn discs into blobs with an 8 px-coherent wobble hash on the warped lattice (±64 on r²) folded with the grain into one adder operand; add per-cell size/squash (shift-muxes on |dx|,|dy|), ring pitch and dark↔light inversion. DEAD-END for timing: a per-cell shape term (±(|dx|+|dy|)≪k) *inside* the r² stage — adder+mux in front of the sq-LUT sum = 69.7 MHz and a router-stalling 91% LC. Deform the coordinates with shifts, never the sum.

### Worley (nearest-jittered-point) cells for an ornament partition
When a cell-based generator must not read as a grid, the partition itself has to go: a quadtree of axis-aligned squares stays a grid no matter how the coordinates are warped or how much per-cell randomisation sits on top (gilt v0.7/v0.8 both failed on hardware for exactly this). Worley cells fix it structurally: a lattice of points (pitch 2^k, k from a knob), each hashed anywhere inside its cell; per pixel test FOUR candidates — own cell plus the neighbour on the pixel's side along each axis — and take the nearest. Use the **octagonal metric** max(|dx|,|dy|) + (|dx|+|dy|)/2 to choose (no squares, 8-bit), then run the existing r²/max/bar geometry on the winner's offset only. Pipeline: cell+fraction (normalised to a 0..63 space so downstream is pitch-independent, with the fraction pre-offset ±64 for the neighbour candidates) → 4 parallel hash_a → 4× (subtract + abs + sign) → 4× octagonal d → **six parallel compares** → one-level argmin + winner mux + the winning cell's identity hash_a; six stages, +4 latency. Cost ≈ 4 hash_a + 8 sub/abs + 4 metric + 6 compares + muxes ≈ what a 3-level quadtree with its own normalisation cost, so it was LC-neutral once the Luma overlay was made to SHARE the cells (own hash salt only) instead of running a duplicate engine. XOR the picture's material id into the point hash and the tessellation itself re-rolls at image contours. Whole-cell lookups (tile RAM by cell origin) must address a row that is already sampled: use half a pitch above the winner's top edge (the lower neighbour is only chosen from a cell's lower half) and size the ring for two pitches. — **gilt v0.9**, standalone 3808 LUT4 / 10 EBR; timing status recorded in the gilt memory.

### Smooth domain-warp waves (organic, not geometric)
Once the partition is irregular, a generator still reads *geometric* if anything in it is a straight line — Worley bisectors, square motifs, ruled bars. Per-cell randomisation does not fix that; a smooth warp of the coordinates before the cell engine does, because every edge, ring and stripe downstream inherits the curve. Cheap smooth wave = **parabolic sine**: a 32-entry hump ROM `i·(32−i)` for one half-period, negated for the other (C1-continuous, no multiply). Two waves, phase `2y + x/2` and `2x − y/2`, pick the 6 phase bits by pitch so the wavelength is two pitches along the main axis and eight along the other (two waves at the same frequency would form a lattice), amplitude ±¼ pitch: one bends x as you go down, the other y as you go across. Time-share ONE ROM between the two waves on alternate pixels (address mux, single `f(...)` call — two calls infer two ROMs). Run all of it free-running beside the pixel path, like the content field, so the pixel stages keep their depth. Reuse the waves as the blob/ring wobble (replaces a blocky 8 px hash) and as marbling in an isophote band (`blurred_luma·4 + wave/2`, bit-sliced) to get contour lines that wrap the figure and stay continuous across cells. A large sway (period 512 lines) at per-line resolution: a triangle of the drifting phase smoothed down the frame by a one-pole filter (~32 lines), stepped once per line on hsync, rounds the corners without a ROM. **Dead-end:** the exact parabola via an 8-cycle serial multiply per line cost ~100 LUT4 standalone, as much as both ROM waves together. Related cheap geometry: a near-round radius is `max(a + b/2, b + a/2)` (both sums in parallel with the compare, one mux, same depth as max); a 16-sector angle with no arctangent comes from the two sign bits plus `|dy| ≤ |dx|/2`, `|dx| > |dy|`, `|dx| > |dy|/2`, with odd quadrants reversed so the order is continuous around the circle; `radius + sector` gives a round spiral. Bars keyed to the ABSOLUTE warped position (re-struck late from the free-running counters; a constant few-px offset is invisible), with axis, pitch and phase per cell group, run on across neighbouring cells like cloth folds. **Keep the warp gentle and the geometry clean when motifs carry fine lines** (gilt v0.13, after HW "chaotic blobs, no spirals or circles visible"): a ±¼-pitch warp at a 2-pitch wavelength stretches a circle by up to ~80 %, and ANY per-pixel noise added to r²/radius (grain dither for "hand-cut" edges, ±2–6 units) dissolves ring and spiral lines that are only ~4 units wide. Use ±⅛ pitch, wobble ≤ ±2 units on the radius, no grain in geometry; let grain live in the ground's luma only. — **gilt v0.10**, HW-validated (Ron: "I like the organic look now"; 6/6, HD 77–79 MHz post-route, 97% LC).

### Panels (worlds) on the isophotes of a heavy luma blur
To compose a frame like a painting — distinct pattern regions framed by sweeping lines — rather than one texture everywhere: build a second, much heavier luma blur (a slower one-pole on top of the existing horizontal one, plus a vertical one-pole kept at ×4 precision in a spare lane of the content-field line buffer; shift-4 IIRs on 8-bit values stall short of their target, hence the extra bits), add a slow drifting wave so flat video still splits into panels, and quantize: each 32-level band is a WORLD with its own motif rules and ground tint, chosen by a small band→world table (a second table gives a second "bank"). Draw a deep line where the zone value's low bits are ~0: panel edges are isophotes, so the line is fine across a picture edge and a broad stroke on flat ground. Confine colour to chosen worlds, so it reads as focal points rather than confetti. A flowing tile grid is just the absolute warped position at one pitch per tile: fold each axis about the tile centre, take the max (Chebyshev) for grout / hollow-square ring / centre dot, and index a 16-entry 4×4 pattern (one LUT per output bit) with the next two bits for tile type, colour and a P5 rank. — **gilt v0.11** (6/6 seed 1, HD 79–80 MHz, 96% LC). **HW verdict: the slow drifting wave that split flat video into panels was REJECTED** ("big arbitrary waves that aren't in the original pic"). v0.12 keeps the idea but drives it from the picture alone: heavy-luma band × a heavy warm/cool class (Cr − Cb through the same one-pole pair), a small table to the world, and frame lines from a world-CHANGE detector (previous pixel, held 3 px; previous line via world + 2-bit hold count stored in the line buffer, held 3 lines) instead of iso-bands (an iso-band line turns a flat area sitting on a boundary into one big blob). Lesson: never let generated large-scale structure (waves, drift) decide composition; generated texture is fine only at cell scale. Not flashed.

### Image → structure via a per-cell luma map + per-frame param RAM
To let input video reshape a generator (not overlay it): sample luma once per cell at its centre (4-px average) into a 4-bit map EBR, double-buffered by frame parity (write this frame's bank, read last frame's — no read/write collision and no mid-glyph change). Keep the map RAW (luma only); a tiny vblank sequencer (5 single-op states × 16 levels) turns slider + knobs into a 16-entry param EBR indexed by luma (glyph scale, stretched tail length, matching fade scale), so the pixel path is two lookups and the slider needs no per-pixel multiply. — **matrix_rain** P12 Image, HW-validated: bright cells get bold 2×-wide glyphs and tails stretched up to ~4× (sparse rain fills and lingers in bright areas — this is what keeps the image readable at LOW density), dark cells shrink to 0.4×. DEAD-ENDS on the way: lighting every glyph cell at video luma (static glyph field = "vertical colour blocks that never go away") and dimming rain in dark areas (masks the rain into a picture-shaped block) — Ron wants the background black or the input video only; shrink-only scaling applied to lit rain alone was too subtle to see.

### Chroma proportional to level (tails that really fade to black)
Render every lit pixel as level L above black: Y = 64 + L·k, U/V = 512 ± L·k·|c|/512 with (c, k) per colour from a generated palette (k a scale8 luma gain keeping the dim hues' body peak unclipped). A fixed full-strength chroma kept a "black" tail end ~40% green (green gets +388 from chroma alone). Saturated green hits G-clip near L≈520 — perceived brightness above that comes from WHITENING (¾ chroma on the leading rows, ½ on head/glow), not more Y. Multiply split: L × |c| operand hi/lo across two stages, one keep-duplicated L register per consumer (a shared one cost 5 ns of routing). — **matrix_rain**, HW-validated.

### Run-length class speck filter with look-ahead (clean posterised / classified video)
Per-pixel classification (hue octant, luma band, palette index) speckles on video noise. Accept a class only after it has held for N consecutive pixels (N from a knob, 1 = raw), OR immediately when it equals the clean class on the line above (a 2048×4 class line RAM, written behind the output counter, read at the live x — never the same word); otherwise the last accepted class holds. The accepted stream is read back through a VARIABLE tap of a shift register (tap = Nmax+1−N) so the total class delay is constant and genuine edges do not creep right by N. Screen-space pattern stages need not be delayed to match: sample them ~26 px AHEAD of the class stream (a constant texture offset, invisible) and let the pixel/dx counters run on through blanking so the overrun never sees the next line. Only source-derived side paths (gloss, outline, shade) take the full delay. — **sugarcoat v1.1**, timing-closed (HD 76–86 MHz, +~300 LC, 2 EBR). Related design lesson from the same program: arithmetic permutations of a class→texture map ((c·a+r) mod 16) read as random on hardware; curate THEMES whose luma ramps are sorted by each texture's mean brightness and whose hue maps follow colour, and keep K1's low bits for order-preserving variants (adjacent-pair swap, one-step tone shift) — **sugarcoat v1.2**.

## Physics, particles & animation

### Memoryless particles from an age ROM
Recompute positions each frame from age via a small ease/droop ROM; evaluate pos(age) and pos(age−1) and line-walk between. No per-particle position RAM. — **fireworks**, HW-iterating. Product widths: rise `ease×height` hit 138 — a 7-bit slice wrapped tall rockets to the ground. Decorrelate angle/speed with a bit-reversed particle index (sequential indices step 6.3° and draw a visible spiral).

### Memoryless replot = scrubbable time
When particles are pure f(seed, age), age becomes a control: freeze + slider scrubs the timeline both directions (bursts collapse back into rockets). Latch the slider at the freeze edge and scrub relative to it, so engaging freeze holds the exact frame wherever the slider rests. The age curves don't even need a ROM: ease = 126 − ⌊(127−i)²/128⌋ and droop = ⌊i²/128⌋ come off the shared multiplier in 6 FSM states (127−i is just NOT i on 7 bits), which freed pyro's EBR for the head overlay. — **pyro**, FINAL (signed off 2026-10-10). Never read SPI registers inside the vsync decision cone (~5 MHz) — decode to free-running 1-bit flags.

### Voronoi color attribution at readout
Color a monochrome persistence canvas by nearest-centre argmin over live emitter centres at READOUT — zero color BRAM; subtract 2× each shell's live radius so overlapping shells own their own ball. — **fireworks**. Clamp weighted distances to 8 bits — a 5-way argmin of 10-bit values was itself the critical path. Pyro lessons (v0.9–v1.0): (1) a shell's region must HOLD from its burst until its slot bursts again — re-centring on the relaunching rocket let other shells re-claim the old burst's fading embers (the main cause of colours 'mixing'); draw rockets white with a separate cheap per-slot launch-column test instead. (2) Weighted distance = dist − radius + 128, clamped to 8 bits, so inside overlapping balls the DEEPER one wins (clamping at 0 tied them and the lowest slot took the whole overlap). (3) The inputs change once per cell, so time-multiplex: ONE distance pipeline walks the slots on the cell's sub-phases for the NEXT cell and commits a result bank as the cell's first pixel reaches the argmin stage — lossless, and it bought back ~400 LUT (4 parallel copies were ~480 LUT / 230 carry). — **pyro**, FINAL.

### Symmetric shifts in feedback loops
`shift_right` (floor) on a signed feedback term is an energy pump (positives never damped to 0, negatives always get −1) — the system charges and collapses cyclically. Any feedback term uses symmetric shifts; floor is safe only feed-forward. Every signed floor-shift feeding an unsigned slice needs a `>= 0` clamp. Stress-sim physics at max knobs for 60+ frames before building. — **pixelbeach**, HW-confirmed then fixed.

### Conservative flux form for grid simulations
Quantize each interface flux ONCE, add left / subtract right — shifting the flux difference per column leaks mass (shoreline creeps). Never clamp height on dry cells (adds mass); pairwise diffusion in flux form kills the odd-even checkerboard. — **pixelbeach** shallow-water, physics HW-validated.

### Display snapshot banks (physics rate ≠ display rate)
Copy live physics BRAMs into display-only snapshot banks right after vsync; display reads only snapshots, physics owns the live RAMs all frame — tick budget becomes the whole frame, no port mux, no tearing. Gate decay effects on the field's last tick so visual rates are tick-count-independent. — **pixelbeach v3.5**, HW-validated (+2 EBR).

### Double-banked slot RAMs (frame-parity banks)
Any per-column/per-cell map built during the frame but read by the raster mid-frame MUST be double-banked (addr MSB = frame parity; swap at vsync; vblank-sweep the incoming bank). — **inferno** luma-mod, HW-iterated. DEAD-END: single-banked + slot-policy tweaks (only rarer). DEAD-END: frame-age tags (stale slots resurrect on wrap).

### Per-lane lifecycle engine with a sliding 3-lane window
N vertical lanes hold small state records in EBRs; a vblank FSM steps lifecycles; render prefetches prev/cur/next lane records through the shared read port and tests 3 candidates per pixel — motion crosses lane borders, neighbours merge. — **sanguine**, v0.1 HW-approved; v1.0.0 (2026-10-06) awaiting HW. Resolve per-lane decisions against LAST frame's BRAM outputs (1-frame lag invisible) — v1.0 moved the bottom-edge test and the paint-width add there too and the walk fell off the critical path; (lane,line)-constant values go in the window record; OR-flag priority beats encoded-code compares. Per-lane "paint record" {deepest row, widest radius} in one 16-bit word renders a permanent solid path with a rounded end (same squares test as the head blob, on rows past the end). Register |dx|/|dy| and every squares-LUT fetch in their own stage before the add+compare tests — sharing the LUTs between two tests put the fetch on the critical path (71 → 83 MHz seed 1).

### Per-column lighthouse register
One EBR of per-column {strongest value, its row}; influence = val − dist·2^k — 2-D vertical propagation in a streaming pipeline, zero extra ports, one-field lag invisible. — **kudzu**, timing-closed (96% LC).

### Cell/phase counters rasterize any cell size
Per-pixel {phase, cell-index} counters (wrap → increment) rasterize ANY cell size, zero division; measure usable cells FROM the counters at line/frame end. — **fireworks** (HW), **mondrian** continuous grid. Counters saturate at the canvas edge AND coords clamp before address formation; store cell size/mask as registered per-frame signals.

### Stabilise the compare's INPUT, not the decision
Flickering presence compares: deadband the control, average the sample over the whole cell, refresh state on ONE field under interlace. — **morsel**. DEAD-END: softening the decision (grow features from a speck) — size wobble reads as wrong as flashing; quantising the input AMPLIFIES dither near step edges.

### Base+delta reference storage
Several thresholds of form `base + small_delta`: store the 8-bit deltas, derive every compare from ONE shared `row − base` subtract (~65 LC back at a 99.7% LC wall). — **pixelbeach**.

### Per-cell decay memory ("wet where the water was, drying since it left")
When something should mark the cells it touched and fade back by TIME, give every cell its own small counter in EBR instead of deriving the mark from the moving shape. Pixel Beach: 4096×4 (column × 16 rows under the beach line, 4 EBR). The display reads it per cell in the next-cell prefetch (address issued mid-cell, latched at the cell boundary). It writes back once per field on the cell row's first line: 15 while the water covers the cell, else −1 every 12 fields. Shade = counter thresholds. yosys inferred the read-modify-write as one EBR fine, because the write data comes from a FABRIC register latched off RDATA. The collision read on the write clock is never used, so `no_rw_check` is safe. Showing the value being written on that first line avoids a 1-line lag. Replaced a "band between the water edge and a shrinking peak edge": that shaded by position not time, vanished when the next wash began, and slid with the edge's sine. Cost: +6 LUT, −19 carry, +25 FF net. — **pixelbeach v3.18**, HW-validated ("looks perfect").

## Engine architecture & timing structure

### Vblank serial shift-add engine for per-frame products
One small FSM computes ALL per-frame knob products serially right after vsync instead of N combinational multiply cones (still STA-timed at pixel clock). Fringe: 8 products in ~110 cycles (~350 LC saved); pinboard: 11 ops on one 16×16 sequential multiplier. — **fringe**, **inferno**, **hypnos**, **pinboard**. Pre-fold algebra into the constants so pixel stages are slices/zero-tests; chain dependent products by handshake.

### Shallow write enables for state the render path reads everywhere
FFs that are read all over the pixel path (64 bunker-chunk bits feeding the chunk-alive muxes) get placed all over the die, so their write-enable cone becomes a routing-bound critical path. Committing "clear bit (bnk, chk) in FSM phase N" decoded the hit indices inside the phase logic: FSM phase → decode → 64 CENs, ~11 ns of routing. Fix: a separate process builds a registered one-hot clear MASK every clock from the snapshot, and the FSM only raises two registered strobes (reset, clear). Each bit is then one LUT: `rst or (bit and not (stb and mask))`. — **invaders** v0.3, HW-validated: HD Analog went from 2/4 seeds to 6/6. The same lesson applies to the reset: never let the raw vsync_start edge reach wide state enables. Phase 0 only latches flags, and the reset runs a phase later.

### Vblank-built LUT for narrow-input per-pixel functions
A per-pixel function of a narrow input (6-bit age, hue knob): rebuild a small table each frame in vblank with a plain accumulator — kills a per-pixel multiply AND infers as 1 EBR. — **clacker**, **bajaweave**.

### EBR ring buffer for the dry-video delay + address-side delay
Replace an N-deep 30-bit FF video-delay with a circular EBR buffer + sync-bit shift regs (~450–700 LC). Corollary: delay the ADDRESS (one narrow subtract upstream), not the wide result downstream. — **edgelab**, **inferno** (HW), **mercurial**. Pad the word to power-of-2 width (CLAUDE: BRAM rules).

### Scattered register files are ROUTING structures
Per-frame register-file writes must be bare flop→flop: form the value AND a pre-decoded one-hot write enable one cycle early (add+clamp+decode in the write cycle = 10.6 ns of routing). Inverse: deliver a per-run parameter as ONE registered value chosen when the run opens, not three muxes at use-sites. — **ribbon** (71.5 → 83.1 MHz), **nacre**.

### Half-rate FSM clock-enable
A big control FSM on a clock-enable every other cycle gets 2× the period while display stays full-rate; replace 1-clock pulses with toggle handshakes (a CE'd FSM misses pulses). — **pixelbeach**, HW-validated.

### Serial multiplier behind a half-rate FSM: mismatch stall, aligned release, registered enable
Replacing an FSM's LUT-array `mp <= ma*mb` (12×10 = **294 LUT** by ablation) with a Robertson shift-add (13-bit add/shift per clock, 10 clocks; ~50 LC) needs no edits at the multiply sites if the FSM simply stalls until the product is ready:
- `m_stall = (ma,mb) /= (op_a,op_b)`. The multiplier starts on a mismatch, runs on the live ma/mb (the FSM is stalled, so they hold), and copies them into op_a/op_b only on an edge right after an FSM CE edge (`if done and fsm_ce='1'`).
- The FSM runs on `fsm_en <= not fsm_ce and not m_stall` (registered) and uses `if fsm_en = '1'`.
- Every state still sees the product of the operands the previous state wrote, the same schedule as the old 1-clock array.

Two wrong versions first:
- (a) A `busy` flag in the CE condition: it is set one clock after the operand write, so it is a 1-clock path into every FSM CEN (20–23 ns).
- (b) The comparator straight into the CE: 26–29 ns through the fan-out.

Prove the handshake with a cycle model (random ops, assert `mp = ma·mb` whenever the FSM advances) before building.

Remaining 1-clock paths: `fsm_en` → CEN through the state decode (it got no global buffer, 8/8 used, 24 ns on one seed) and `ma/mb` → comparator → `fsm_en`. These are bounded only by the reported worst path. — **pixelbeach v3.22**, HW-validated (the waves, sky lerps and sun position were unchanged on the device).

### Sprite/asset ROMs over arithmetic
Per-pixel geometry that can be precomputed becomes a ROM: pulp's bubble SDF ~1,400 LC → three 64×64 2-bit ROMs (6 EBR, ~300 LC); morsel's silhouettes; starfield's kernels. The generate-side lives in a Python build hook.

### Three rotating Y banks = post-warp vertical gradient
A cel/edge outline that must follow a geometric warp needs gy AFTER the warp, i.e. two SOURCE lines read at the same warped address. Rotate three Y banks (write w, read w-1 and w-2 at one address each; U/V stay dual-bank) and compute |gx|+|gy| on the fetched stream — no second pass, no edge buffer. 15 EBR for Y. — **billboard** (cel + keystone, 84 MHz HD), timing-closed only.

### Delayed write port carries parity AND enable from the input time
When a pre-buffer stage chain (solar_fold, bleed IIR, box average, Kuwahara windows) delays the write by N clocks, delay the bank parity and the avid enable along with the data (`s_wx/s_wp/s_wc` shift chains, or a counter started from the delayed avid). The last N pixels of a line then land after hsync in the CORRECT (old) bank — SD's 16-clock front porch is shorter than most chains. — **mirage**, **smudge**, **impasto** (41-clock chain), **phosphene**.

### Box-sum ring reads N-1 back
A running N-sum from an EBR ring: at cycle c write s(c) at w and read w-(N-1); the accumulator then covers exactly N samples ending at the current pixel. Reading N back gives N+1 samples (1.5x brightness at N=2) and at N=64 a same-address read-during-write (undefined on EBR). Hold the sum at 64*N through blanking while the ring refills with neutral. — **smudge**, **stencil** (v0.1.1 fix), **impasto**, **vitrail**.

### Sine/curve ROM output needs a second fabric register before a multiply
A `s_q <= ROM(addr)` register is absorbed into the EBR output latch; feeding `s_q` straight into a sequencer multiply puts EBR clk-to-q + column routing + the multiplier in one period (reflector/outliner: 66-70 MHz). Add `s_q2 <= s_q` and one wait state. — **reflector**, **outliner** (v2: 79-87 MHz), **tremor**, **collapse**, **tonecurve**.

### Make a wrapping coordinate continuous BEFORE a DDA interpolates it (iris v1.1)

A line-rate engine samples a periodic coordinate (an angle, an arm index) every N pixels and a
DDA interpolates between samples. Wherever the stored coordinate wraps -- at the angle's cut, or
where a modular field overflows -- the two samples straddling it differ by the wrap constant,
and the DDA sweeps linearly through it: a whole turn's worth of dots planted in N pixels along
the cut. If the wrap is an integer number of cells (iris: A mod 16 arms), the dot test at the
samples is perfectly correct and only the interpolated pixels are wrong, which makes it look
like a drawing bug that "moves with the pupil and counts with Arms" rather than a sampling one.
The fix is in the engine, not the pixel path: count wrap crossings along the line (compare the
previous sample's quadrant: 3->0 forward, 0->3 back), keep a small per-line accumulator of
(crossings x wrap-constant) and add it to the stored coordinate, so consecutive samples stay
continuous; modulo one cell nothing changes. Export the wrap constant per frame if it depends on
a control. Two traps: reset the accumulator at the line's FIRST REAL sample, not at run start
(pipeline-fill periods carry garbage), and be aware that a coarse-raster sim may never cross
the cut at all -- iris's NTSC renders showed zero wraps at settings where 1080p hardware drew
the line, and the sim's knob values can land on the one arm count (a multiple of 16) that is
immune. Diagnose from behaviour under each control. -- **iris** (HW-confirmed).

### Time-shared layer machine: half-step smoothing beats lerp and line-alternation (iris v1.0)

When one evaluator serves N layers on a fixed slot schedule, the slow layers are HELD between
evaluations and every edge becomes a staircase with treads as wide as the slot period (4 px
for red/blue on iris's G,R,G,B schedule). Three fixes, rendered and compared:

- **Half-step** (chosen): keep the previous evaluation; for the first half of the period show
  the midpoint of old and new, then the new value. Treads halve and gain an intermediate tone
  -- the same quality as a layer evaluated twice as often -- with no look-behind, no comb, and
  ~25 cells per layer (one 9-bit adder, a mux, one register). The fast layer is aligned by
  showing ITS previous evaluation; the sync tap moves by the period/2. Seed the "previous"
  value with the fresh one at the first evaluation of each line, and gate the hold registers
  on an active flag carried down the pipeline, or blanking-time evaluations pollute the
  history at line ends.
- **Linear ramp** between evaluations: smoothest, but each layer needs a slope register, a
  guarded subtract (truncate toward zero or a fade-to-black wraps to a bright pixel) and an
  accumulator -- ~100 cells -- and on a 3 px feature a 4 px ramp is a smear, not an edge.
- **Line-alternation** (layer A at double rate on even lines, B on odd, a line buffer carrying
  the other across): sharpest horizontally and cheap (2 EBR, ~45 cells), but the fresh/stale
  alternation is a 1-line comb along every boundary that is not exactly vertical -- on small
  dots, all of them. Rejected on sight.

The budget for the winner came from narrowing accumulators to the bits that are actually
read: a DDA whose top bits feed only their own carry chain looks live to synthesis. Count the
bits that reach a consumer, slice the reload and increment to that width (modular arithmetic
makes it exact), and take the cells back. -- **iris** (HW status: timing-closed, awaiting HW).

### Fade an undersampled lattice out at its Nyquist limit, from the CONFORMAL scale (iris v0.9)

A line-rate engine that samples procedural coordinates every N pixels and DDAs between samples
cannot represent a lattice finer than about one cell per N pixels. Past that the DDA interpolates
across structure it never sampled: the cells land in the wrong places and the field turns to
coloured hash along whichever direction is locally crowded. In iris this read as a thin, broken,
many-coloured seam running out of the pupil. The diagnostic tell is that it responds to NO
control -- it is a property of the sampler, not of the picture. (Measure it with a local
4-neighbour deviation on rays out of the feature, not with a row/column statistic: the feature is
usually off-centre and a row through the wrong place finds nothing.)

Cure: attenuate the cells by the local density -- full strength at one cell per N px, gone by one
per N/2 -- so the lattice ends rather than degenerating. How you MEASURE the density is the whole
technique:

- A conformal map (any Möbius/analytic warp) crowds both axes by the same factor, so ONE scalar
  covers both. Anything that already anti-aliases has it: an AA shift that converts cell-space
  distance into pixels IS `log2(cell units per pixel)`, so the two thresholds are integer
  constants one octave apart, and the ramp between them is a plain slice of a subtraction the
  engine already performs.
- NEVER derive it from the difference of a WRAPPING coordinate (an angle, an arm index). Across
  the branch cut the difference reads as a full-scale change and blacks out a radial spoke -- a
  new seam in place of the old one. Reducing modulo the period does not reliably cure it.
- A single-axis difference (the coordinate's change along the scan) misses crowding in the
  perpendicular direction entirely. In a coaxal pencil the rings crowd along the line of centres
  while the arms converge in every direction, so a ring-only test cleans the horizontal streaks
  and leaves a plume above and below.

Compute it in the per-sample engine and carry it in spare bits of an existing sample word, not in
the pixel path: the two per-axis formulations needed two 21-bit absolute values plus a compare per
pixel and cost 165 cells and ~15 MHz against the conformal one. A cheaper formulation beat every
placement-level remedy. Calibration warning: a float model can confirm the ALGEBRA of such a
constant while being wrong about UNITS (normalised vs pixel coordinates) -- iris shipped an inert
build that way. Prefer a constant pinned by an internal calibration the hardware already relies
on. — **iris** (HW status: timing-closed, not yet HW-tested).

### Fading a dot: subtract AFTER the alpha clamp, never offset the ramp (iris v0.7)

An anti-aliased dot's alpha is `clamp(bias + b, 0, cap)` where `b` is the (barrel-scaled)
signed distance from the dot's edge. `b` SATURATES across the dot's whole interior, so biasing
the ramp -- the obvious way to fade -- only walks the edge inward by a pixel or two and can
never dim the core. iris shipped a per-frame ring fade built that way and it looked like the
outer ring being clipped off rather than fading. Subtracting an attenuation from the alpha
AFTER the clamp carries a dot smoothly to nothing, shrinking and dimming together, for one
8-bit subtract. Put the attenuation before the ink/light composite and it also flips sign for
free: coverage 0 is black on an additive ground and white on a subtractive one, so the same
term gives a black pupil in Light and a white one in Ink. — **iris**.

### Glide a ZOOM in log space, not in the radius (iris v0.7)

A first-order ease on a radius (`R += (target-R)>>3`) spends most of its motion in the first
two frames and then creeps, because the apparent zoom rate is dR/R. Over a 2.2x change that
reads as a lunge followed by a crawl -- and anything that only enters the frame in the last few
percent (iris's rim ring) appears to pop in after the move looked finished. Ramp the exponent
instead: `R = R0 * 2^(t * log2(R1/R0))` with t stepped linearly, which is equal ratios per
frame. Costs one log2 and one exp2 per frame in a blanking-time sequencer. — **iris**.

### The build script's green tick is not timing closure (iris v0.7)

`build_programs.sh` retries six seeds and, when it runs out, ACCEPTS the last attempt and prints
"✓ Completed (seed 6)". On iris that shipped HD Dual at 74.13 MHz against a 74.25 MHz
requirement -- a miss, packaged and labelled as a pass -- and HD HDMI at 74.28, a margin of
0.03 MHz on a router whose run-to-run spread on the same netlist is over 10 MHz. Read the routed
Fmax for every config and treat the tick as decoration. Two adjacent traps: nextpnr prints a
placement ESTIMATE before the routed line (on iris the two were uncorrelated -- five seeds
estimating 76-81 MHz all routed at 68-72), so take the last line and only after the run ends; and
`rm <cfg>.bin` does not force a rebuild, because make will reuse a stale `.asc` and just re-run
icepack, yielding a fresh-looking bitstream from the previous seed -- delete the `.asc` too.
Landing a good seed is cheap: build once for the `.json`, sweep placements over it in parallel
(nohup, not bare `&`), then rebuild that one config with `SEED=<best>` and repack. — **iris**.

### Size a CORDIC for the screen CORNERS, not for the drawn shape (iris)

A vectoring CORDIC grows its x register by the gain 1.6467 during the rotations, so the
datapath must hold (worst-case input magnitude) x 1.6467 x 2^guard_bits. Size that at the
furthest point the raster SCANS, not the furthest point the picture DRAWS: in a framed 16:9
raster the corner sits 2.17 "shape radii" out, well outside the visible artwork, but it is
still sampled every line. On iris an 18-bit datapath overflowed there (168k counts against
131k), the magnitude came back as garbage, the ring coordinate landed back inside the drawn
range and painted thin arcs of stray dots in all four corners -- invisible in every render of
the shape itself and only reported from hardware. One extra bit fixed it for ~40 LC.

### Size ROMs to the next multiple of 256 words, not the next power of two (iris)

A 1112-word microcode ROM declared `array(0 to 2047)` costs 8 EBRs; declared `array(0 to 1279)`
it costs 5, and yosys still infers block RAM (the usual "declare a full power-of-2 depth" rule
is about small arrays that must not become logic -- it does not mean round up to 2^n). On iris
that single declaration took 23 EBRs to 20 and the routed Fmax from 62.45 to 74.47 MHz at the
same seed, for +47 LC. At 90%+ logic utilisation the EBR columns fragment the fabric, so spare
block RAMs are worth real megahertz: measure EBR count as carefully as LC.

### A fixed, centred outer circle makes the second focus free (iris v0.5)

If the outer circle of the pencil is FIXED and CENTRED, normalise the screen so it is the
unit circle: the two limiting points are then inverse in it, and the map is the disc
automorphism w = (z-a)/(1 - conj(a) z). The far focus never appears, so the denominator is
bounded and BOTH CORDIC lanes are fed by plain affine accumulators (55 LC total) instead of
a 32-bit lane with a per-frame normalisation shift. Every reference value becomes closed
form -- no probing of the pixel engine: |a| = 2D/(Q + sqrt(Q^2-4D^2)) with Q = 1+D^2-P^2
(use that RATIONALISED form; the textbook (Q-sqrt)/2D cancels catastrophically as the inner
circle approaches the centre), the outer circle maps to |w| = 1 exactly, and |w| anywhere on
the axis is |(De-a)/(1-a De)|. Worth ~100 LC and a much simpler per-frame solver; it does not
by itself solve a utilisation problem, because the removed probe hardware roughly pays for the
new accumulators.

### Coaxal limiting points turn two nested circles into one log-polar lattice (iris, timing-blocked)

Any two nested circles (a pupil inside an iris) are a coaxal pencil with two limiting
points a and b. The Moebius map w = (z-a)/(z-b) sends BOTH circles to circles about the
origin, so a dot field that is a plain log-polar lattice in w draws as a perspective cone
on screen, and because Moebius maps preserve circles every dot stays a TRUE circle at any
pupil offset. Per pixel you never form w: a vectoring CORDIC about a and about b gives the
two magnitudes and angles, u = log|z-a| - log|z-b| is the ring coordinate and
theta = arg(z-a) - arg(z-b) the arm coordinate. The dot test S(dv) <= B(du) with two
per-frame tables is then exact in w-space. The sequencer solves a and b once per frame
(one square root) and PROBES the live pixel engine for its own reference log values, so
CORDIC gain and table rounding cancel by construction instead of being modelled.

Make the per-frame log tables carry the ring SCALE (rings per octave) so the engine needs
no multiplier at all: the lane output is log2(value) x invn directly. Keep a separate raw
exponent+mantissa sum for anything that must stay in true log2 units (the anti-alias shift).

Cost on an HX4K: one three-stage serial CORDIC, one log lane, a 4-EBR line sample buffer
at 8 px spacing, one dot machine time-shared by the three colour layers, and a microcoded
sequencer = ~7200 LC (94%) and 23 EBR, which routes at 64-67 MHz -- NOT enough for the
74.25 MHz HD clock. Status: renders correctly, closes SD, HD needs ~500 more cells freed.

### A second parallel lattice costs flip-flops, not logic: merge before blending
A second cell lattice running in parallel with the first (morsel's drop layer: 19 stages, own hashes, own cell-luma buffer) first cost ~1,680 LC, 71% → 98%. It wasn't a ROM in logic, as first suspected. FF count rose by 1,100, and on iCE40 every FF without a LUT partner costs a whole LC. Measure LUT and FF separately (count `SB_DFF*` in the json) before guessing. What got it back to 89%: (1) merge both layers into ONE top layer *before* blending (pick layer A where its coverage ≥ ½, else B, with alpha = max). Same-material overlaps need no AA, and this saved a whole 3-channel blend and its carries. (2) Carry constant colour pairs as a 1-bit flag, not 20 bits. (3) Blend chroma in 3 steps (under / average / top). (4) Read ping-pong cell buffers at the stage where the cell row is known, not early plus a delay chain. (5) Round features need no radius ROM: a 16-gon `max(4mx, 3.75mx+1.5mn, 2.875(mx+mn))` gives ~3% error with adds only. — **morsel** v0.2.0; the drop layer itself was HW-REJECTED as a look ("I don't like the circle chips") and removed in v0.2.2. The fit lessons stand.

### Knob smoother: deadband + 10.4 glide, one tiny entity per geometry knob
Pot ADC flicker of ±1–2 counts reached the picture every field (one count of a stripe-frequency knob moved the right-edge bars 18 px — "stutter at times"). A small entity per knob: the hold h follows raw only when |raw − h| ≥ 5 (3 still hopped under ±2 noise in a model), and a 10.4 output eases toward h by diff>>3 per field (snapping inside half a count), pipelined so the field-start update is a plain mux. Feed the SMOOTHED 10.4 value into the per-frame maths (carrier increment, LFO step carried ×16 into a wider accumulator). ~180 LC for five knobs. Corollary at 90% LC: every fused per-field operation in the drift/gain sequencer (abs → compare → min; a 4-term constant multiply; L+d then abs) became the HD critical path in turn (seed spread 75–88 MHz) — one operation per register restored 89–94 MHz. — **bajaweave** v0.14/v0.17, HW-validated.

Rail snap + pipelining (**revenant** v0.5–v0.6, signed off 2026-10-10): a deadband capture needs `raw < DB → 0` and `raw > 1023−DB → 1023`. Without them a slow sweep parks 1..DB codes short of the end, so "0 = frozen" crept instead of freezing. Compute the capture free-running into a `_n` register (one signed subtract covers both directions) and only latch it at hsync. Fed straight into the hsync-gated enable, the compare chain became the HD HDMI critical path (74.8 MHz, 9.9 ns routing); latched, it closed at 87.5.

### Parity-free 4:2:2 pair averaging from a chroma delay line
A cheaper form of bishop's pair-averaged chroma (see that entry). The core's packer point-samples U on EVEN and V on ODD avid pixels, so emit U = mean(U[n], U[n+1]) and V = mean(V[n−1], V[n]) on EVERY pixel. Whichever pixel the packer samples then carries its pair's mean, with no phase tracking. If the colour stage already feeds a delay line (chroma separation), write the averages at its input: `uf(0) <= (u_in + u_prev)/2` lines up with `yd(k+1)` and `vb(0) <= (v_in + v_prev)/2` with `yd(k)`. The lookahead is then free and latency does not change. Per-channel separation taps read `uf(i−1)` and `vb(j)`. About 25 LC. — **revenant v1.0.0**, signed off 2026-10-10.

### `no_rw_check`: stop yosys emulating read-before-write on EBR
Any RAM written and read on one clock carries hidden emulation: VHDL gives read-old-data, iCE40 EBR collisions are undefined, so yosys delays the write port and adds a bypass mux (~74 FF + 43 LUT per 32-bit RAM; the FFs show up named `r_x.0.0_WDATA/WADDR`). Collision-free RAMs (bank-split tables, rings with a fixed read offset, register files that forward) declare `attribute no_rw_check of r_x : signal is true`. This works only if the read-data register has an init value, so that GHDL emits an async read + FF and yosys' memory_dff merges it. Check that `emulate_transparency` count is 0 in the log. 9 RAMs: −231 LUT, −225 FF (~380 LC). Rule in CLAUDE.md. — **damier** v0.2.0.

### Block-rate complex lens on one 4-product multiplier
A radial complex warp U' = U + Re((M(r)−1)(U+iV)), with M = (1+g)e^{iτ} giving bulge → twist → vortex → ripple rings. It is evaluated once per 4-pixel block and interpolated across the block. Index: r² (exact, from the accumulators) >> sh, with sh even per field so the lens always spans 128–511 of 512 entries {c, d Q10}. Entries i and i+1 are read on successive clocks of the block (one table, no even/odd split), with 16 interpolation steps. Interpolating was what killed the 45-px twist jumps. The four real products (Uc, Vd, Ud, Vc) fill the block's 4 clocks exactly on ONE 13×12 multiplier (operands registered, product 2 clocks later). Gauss's 3-product trick was WORSE here: its pre-adds widen the multiplier to 14×13 and add registers. U, V go in as block-float mantissas (>> (12−n), n = zoom octave), and the result goes back >> n. Each consumer gets its own `keep` copy of n, because a shared decode crossing the die was the critical path. The coordinate waits in a 32-bit EBR ring meanwhile. The main path interpolates incrementally (D = 4qp, then D += qc−qp per pixel; bit-exact with j·Δ>>2, one stage and ~100 FF cheaper). Hand-written radix-4 Booth was NOT smaller than yosys `*` (482 vs 471 LUT for 14×13). **Invisible rim** (v0.2.1, after Ron saw "a jagged thick circle around the edge"):
- Profile the warp with E = (1−u)², so the value AND slope are 0 at the rim.
- Never add shading: a 4u(1−u) limb-darkening ring outlined the lens before the warp showed, and jagged it with 4-px block steps.
- Round c = m·cos τ − 1: cos 0 is 32767/32768, and truncation left c = −1 inside the lens, a ~1 px step at the rim.
- Interpolate the sine. A bare 512-entry quarter-sine steps 1/2048 turn, which is a 2–3 px jump at 1000 px from a rotation centre. It staircased the whole vortex and also jittered the lattice spin and the lens orbit.

v0.2.1 still showed the ring, and so did v0.2.2. **The reported ring was a pipeline-phase bug, not the profile.** The v0.2.0 fit pass made the lens read entries i and i+1 on successive clocks of the 4-clock block. The B5/B6 stages then took the outside flag and the interpolation fraction "straight from the sample". By then the sample register already held the NEXT block. At the table's end, entry 511's wrapped i+1 (entry 0, the lens centre) leaked through un-zeroed: a thin jagged ring at index 511. That is r = sqrt(511·2^sh), so it stayed put while the lens grew and doubled when sh stepped. All interior blocks were also interpolated with their neighbour's fraction. Fix (v0.2.3): capture the fraction and flag with entry i, at the edge where the sample moves on.

How it escaped:
- The decimated (480×270) sims never put the table end on screen.
- The numpy "bit-accurate" model encoded the intended pipeline, not the RTL's, so it agreed with my intent.
- Only a full-1080p RTL sim at the user's settings showed it. Use the sim-only copy with glides bypassed (3 warmup frames, ~14 min).

The profile change (v0.2.2) stays as a real improvement: (1−u)² crushed a 1.5× band at 0.7R. The new profile is (1−u)²/(1+10u), on a table 2.2× wider, with Q12 values, and its squeeze peaks at 1.18×.

Lessons:
- In a phase-read block pipeline, every side value travels with its data. "The sample holds for 4 clocks" is true only for stages inside that window.
- When HW shows an artifact the sims don't, reproduce at the HW raster before redesigning anything. — **damier** v0.2.3 (HW-validated: ring gone, "looks really nice"). The whole lens was removed in v0.3.0 when Ron preferred a different slider (see Slide); the techniques stand.

### Resonance: concentric rings of the layout's own shape metric, period fixed in pixels, tightening into moiré
Every cell's shape metric m (the one the shape engine already computes: 2·max for squares, the near-round radius for circles, |r − 128| for Truchet arcs …) feeds a ring generator: ring = bit 7 of (m·F16/16 − Q), XORed into the ink, so shape outlines survive as phase flips.

- **Frequency.** F16 = cycles per half cell × 16 = cell_px·(2^(s/128) − 1)/32. That puts the period in PIXELS, so it is the same at every zoom: ~85 px at 25%, ~17 px at 50%, ~4 px at 75%, ~1 px at 100%, where the rings alias into large moiré figures. cell_px = 65536/zoom step comes from Newton on the zoom mantissa (the old lens rcp code). It is floored by a per-cell ramp so 8-px squares respond from ~6%.
- **Pixel cost.** One 9×11 multiply split 9×6 + 9×5 (only product bits 11:4 matter), plus one add and one subtract: ~150 LUT and 2 stages.
- **Phase trap.** A ring phase Q is a whole-cell colour flip when F is near 0 (r = −Q everywhere). Let Q flow only once a cell holds ~2 rings, and settle it to 0 (shortest way mod 256) below that.
- **Flow.** Q advanced at a steady pixel speed, (Speed + 16)·Y/8192 per field. Ron asked for stationary rings that change only with the slider, so v0.4.2 fixes Q at 0. The ring phase port remains if motion is ever wanted again.
- **Knob noise is magnified.** Any control that feeds F jitters the rings by m·ΔF. One pot count moved them 1/8–1/4 ring at the cell edge, which read on hardware as "glitchy, as if the slider is jumping" (v0.4.0). Fix: a 5-count deadband hold before the glide on every knob in the chain, including Scale through cell_px, and compute F from the glide's fractional value.

— **damier** v0.4.3, HW-iterating (Ron: "works well now"). He then asked for stationary rings (v0.4.2) and a slider range that stops at the old 50% (~17 px period, v0.4.3): the 1-px moiré climax read as "lines too small" on hardware.

### Slide: per-row / per-column offsets from a few per-field phases (café wall to sliding-tile loom)
P12 slides a lattice apart. After the waves, each lattice row j shifts along U by one of 8 per-field registers. A 2-bit speed class (xor/add hash of j's bit pairs, ~2 LUT levels) and j's parity select it: even rows +P_k, odd rows h − P_k. h is the static café-wall shift; the P_k slide in alternate directions at 0.5/0.75/1/1.5× one rate. Then each column (after the row shift) shifts along V by ±C_k, by its own class and parity. That is 4 pixel stages: select, add, select, add, and the layout sees the shifted coordinate, so cells carry their content.

The microcode keeps the phases persistent, with 1/16-unit fractions. Below its threshold each phase eases to the nearest 2-SQUARE alignment, because an odd square swaps the checker's colours.

Grey mortar is a band of width mw on row boundaries (fv + mw/2 mod 256 < mw), overriding the palette at the output. When the columns move, a column band fades in too. Without it the climax was dashed grout and merged same-colour blobs; with it, a wall of separately sliding tiles.

Cost: ~200 FF of port registers plus 2 small muxes and 2 adders. The café-wall tilt illusion needs mid-grey mortar and a row shift around 1/4–1/3 of a tile; a half-tile shift is the symmetric brick pattern, with no tilt. — **damier** v0.3.0, RTL-sim-verified, built but never flashed. Ron asked for Resonance instead before trying it; the code is in `.damier_work/v030_shipped/`.

### Fold-coordinate lattice accumulators with exact serial line seeds
Mirror / flip / kaleido symmetry by folding screen coords (P,Q) = (|X|, |Y|), then (max, min). One pixel step changes exactly ONE of P, Q by ±1, so U = a·dP − b·dQ and V = b·dP + a·dQ each add one of {a, −a, b, −b}. These come from registered negatives, chosen one pixel ahead. r² adds 2(±d)+1, registered one clock ahead, so acc_r runs one pixel behind (invisible for a smooth field), and only dP, dQ are tracked (4 trackers → 2 removed the 10-ns `sel` fanout path). Line seeds (U, V, r² at the left edge) are computed EXACTLY per line by a serial radix-2 signed multiplier: 6 products × 14 clocks ≈ 90 of the line's 858+ clocks, while the previous line draws. That replaced a unit-step walker at equal LC with no 2,300-step vblank walk. Timing: keep the line-start strobe out of the step-decision cone. The "next pixel" registers preload the left edge (a field constant) during blanking, and the operand registers load every clock. — **damier** v0.2.0, timing-closed, not flashed.

### Output conditioning against the HD HDMI thick bars ([1 2 1] on Y/U/V + luma scale)
**The symptom.** HD HDMI shows thick bright bars: several lines tall, starting at a random x, running to the right edge and flashing. They appeared across many programs (CGA, Fray, Ribbon, DAMIER), always on pictures with sharp, high-contrast edges in motion.

**The cure.** A final output stage before the blanking gate:
1. Keep a 2-pixel window per channel and output the centre pixel smoothed horizontally, `(prev + 2·centre + next) >> 2`, on Y, U and V.
2. Scale luma toward black: `d = y − 64; y' = y − (d>>3) − (d>>4)` (= 64 + 13/16·d, white 940 → ~775). That brings any palette under the ~768 drawn-luma cap and halves the sharpest steps.

Cost: +2 pipeline stages (widen the sync ring; same latency in every mode), ~150 LC. At HD the softening is barely visible.

**How it was proven.** A diagnostic build put the guard on a switch (S10 standing in for Zigzag), with the guard-off path delayed to the same latency on ONE fixed placement. Ron flipped it while bars were showing: "The bars stop with guard." That A/B pattern is the right way to test any program-side bar cure, because placement and content stay identical.

**History.** Ribbon's luma-only 1-2-1 had "cleared them then did not hold". DAMIER adds chroma smoothing and the luma scale. Not yet bisected (smoothing vs scale); ship both.

— **damier** v0.4.4-diag A/B (HW-confirmed), made permanent in v0.4.5.

## Verification & debugging methods

- **RTL-faithful Python cycle model**: translate engine processes mechanically (read pre-edge, write post-edge) so it emits the identical stream as the die; diff against analytic math in ~2 s. Reach for it immediately on any streaming-engine bug — 5 of nacre's 7 defects were one-cycle read skews a picture can't localise. Taxonomy: sync-RAM prefetch is 2 cycles; ROM reads trail consumers by one; write strobes need explicit '0' defaults; clamp run ends to W not W−1. Always test dragged/asymmetric geometry — symmetry hides wrong-operand bugs. — **nacre** (program dropped; method transfers).
- **GHDL-vs-model pixel diff**: target 0 pixels different; check white fraction first (black-on-black passes trivially); vary only un-glided controls or sweep glide steps 0..8. — **fringe**, **morsel** (memory: sim_model_pixel_diff).
- **Knob sweep harness**: re-implement each sequencer slot in Python at the RTL's REAL widths; assert every knob monotonic and full-range (morsel `check_frame_math.py`). Stills cannot reveal wrap faults.
- **Bypass control frame**: a pixel-clean BYPASS mode isolates full-field noise to one path before diagnosing sub-bugs. — **mercurial**.
- **Continuity proof**: sweep knob values inside a single step of the old quantized design — old renders identical, new must differ monotonically. — **ribbon**.
- **Chroma-gain rubric**: measure sat changes on a source with gamut headroom (a saturated test image clips and reads 112% instead of 125% — looks like an RTL bug, isn't). Acceptance: endpoint percentages, luma within 1 LSB, mean hue shift < 0.4°. — **cascade**.
- **ISA emulator first** for any microcoded engine (seconds vs 20-minute renders) — but it can't see hardware sequencing bugs. — **cubist**.
- **On-screen diagnostics**: when only hardware shows the bug, render blink cells comparing two rates by eye. — **pixelbeach**. Dump raw pixel grids rather than trusting a clever detector; derive test oracles from the source being replaced, not from memory of it. — **bishop**, **cubist**.
- **HDMI TX pin-skew screening (`tools/hdmi_pin_skew.py`)**: thin full-width lines in one colour (blue → brown) that appear in one BUILD only are per-line U/V swaps. The encoder counts Cb/Cr from HSYNC, and the core forwards its clock through fabric, so HSYNC landing near the clock edge is a placement lottery that nextpnr never checks. Route 8–24 seeds with `--report r.json --detailed-timing-report` and keep those with HD HDMI/Analog hs-clk ≈ +1.0..+2.0 and all data ≤ −0.3. HD Dual always lands at −1.5..−3. Anchors: +0.48 showed lines and +1.44 was clean; every screened build since has been clean. The report lists only ~40% of nets (here only the final LUT→pin hop), so it is a relative screen, not an arrival time, and you cannot build an all-paths STA on it. — **pixelbeach 3.17.2**, HW-validated; reused by wake, damier.
- **Netlist before hand-CSE**: yosys may have already merged the shared term — pyro's line-rate pre-composites gained nothing.
- **Hardware bisection by palette slots**: when only hardware shows a content-triggered glitch, build ONE config with the other palette/table slots replaced by single-property variants of the culprit (exact copy, chroma halved, mono, black lifted, luma squeezed...) — one knob sweep answers six questions. A switch repurposed as an A/B toggle for a candidate fix (here a luma low-pass) is the decisive follow-up. — **ribbon** v0.3.3.
- **The recurring ~10 px HDMI bars (CGA, Fray, Ribbon): what is now KNOWN NOT to be the cause.** (a) Picture content per se: a luma 1-2-1 low-pass seemed to clear them on one build and did not hold on the next. (b) Ground bounce from the 20 same-bank data pins onto the fabric-forwarded TX clock: measured (clock edge 1-2 ns after the data burst) but DISPROVED — an invisible 511/512 strip flipping 18 pins the same way every clock, in the worst-margin placement, never barred. (c) Placement alone: v0.3.3's placement barred only with Flow on. What holds: static pictures never bar; bars need the pattern to crawl (Flow), more at faster Flow; the rate drifts over days. Best remaining lead = a marginal downstream link (cable / sink / ADV7513) reacting to particular pixel-value sequences. Untested: cable and display swap. — **ribbon** v1.0.0; method entry above (palette-slot bisection + a switch as fix A/B) is still the way to attack it.

## Cross-library notes (2026-07 analysis)

- The HW-validation backlog is the biggest structural gap — many programs are timing-closed but never flashed; keep the status tags honest.
- Standing rework candidates then: tessera (10-triangle rasteriser — since built), laser_cathedral (dead K4, Phase-2 beams unbuilt), pixelbeach (no "done" definition), tiler (flickers at N=1/2), oubliette (loot stubbed), etchplate (intended filament look still unmet — see memory `etchplate_intended_look`).
- Cheap cross-library passes: dead-knob cleanup; continuous-control pass (many knobs decode only top 2–3 bits); back-port fixes between forks (revenant↔glitchscape, mondrian↔quilt↔triangulator).

## Dead-end index

Quick lookup — details in the entries above.

| Dead-end | Program |
|---|---|
| Shared-angle drift for two sways of different wavelength flowing the same way (crossing-speed wobble = stutter) | bajaweave |
| Per-edge case-mux geometry fetch (vs active-mesh BRAM) | bishop |
| XOR/parity boundary buffer (RMW); runtime crossing-count arithmetic | bishop |
| Per-column RMW accumulator BRAM (sim-clean, HW black screen) | c64 |
| Cross-frame per-cell keep-last hysteresis grid | c64 |
| Independently edge-hashed port networks ("network, not labyrinth") | daedalus |
| Post-CORDIC centre projection (sine ROM + 3 mults) | hypnos |
| Runtime-indexed sprite register arrays; 10 independent rasterizers | tessera |
| BRAM framebuffers for line art | spiroscope |
| Per-cell corner-scale "zoom" (not perspective); NP=5 (abc9) | ziffern |
| 8 zoom shells (115% LC) | warpfield |
| Jittered off-grid cell overlap port | mondrian |
| 22.5° direct direction selection | intaglio |
| Pure xor/shift spatial hashes | mondrian, inferno |
| Per-pixel grain / per-line random runs for organic texture | etchplate |
| Single-banked slot maps; frame-age tags | inferno |
| Overlapping full-mux sprite ghosts; full-res 128-bit prefix-OR | invaders |
| Per-clock LFSR sampled once per frame (frame length aliases it to a 17-state cycle) | invaders |
| Decoding hit indices inside FSM phase logic into widely-read state-bit enables | invaders |
| if/elsif variable-accumulating loop over 32 groups (synthesised as a ~10-LUT ripple, HD 50–55 MHz) — use explicit term vectors + one balanced `/= 0` | invaders |
| A sprite drawn in another sprite's view space outliving that view (it snapped back with the view) | invaders |
| Serial one-projectile-per-frame bbox hit scan (misses thin rows, bbox kills) | invaders |
| Softening a flickering presence decision (grow-from-speck) | morsel |
| Runtime descriptor math breaking EOR parity | bishop expressions |
| Field-overlay genre (as a genre) | mercurial/redshift/kudzu |
| nacre program as a whole (line budget + routing topology) | nacre |

### Slope-scaled stroke width (uniform line weight in a scanline stamper)
**Exemplar:** bishop (`dense-wireframe-model`, v0.3). **HW status:** timing-closed only.

A per-scanline stamper draws each edge as a horizontal run of N px centred on the
DDA's crossing. The perpendicular thickness that produces is `N / sqrt(1+a^2)`
for integer slope `a` — so a FIXED pad draws near-vertical edges at full weight
and near-horizontal ones at ~1 px, and any control that nudges an edge's slope
(a morph, noise) makes its weight visibly jump. Scale the per-side extension
with the slope instead:

    ext = pad                      when a = 0, or the edge is exactly horizontal
    ext = pad*a + pad/2            otherwise
    sweep = a + 2*ext + 1, start_rel = (slope<0 ? s_int : 0) - ext

That holds perpendicular thickness within ~16% of `2*pad+1` at every
orientation, with one branch instead of three. Exactly-horizontal edges (where
`slope` holds the WIDTH and cur_x is parked) get their thickness from widening
the activation window by ±pad rows instead — and widening EVERY edge's window
by ±pad turns those extra rows into a square line cap, provided the sweep is
clamped in x (below). Pre-back-up the cur_x load value by `pad*slope` so the cap
rows extrapolate the line correctly. Cost: `ext`'s shift-add wants its own
pipeline stage — hanging it off the mesh BRAM read in the same cycle as the
abs() was a critical path.

**Companion — the x-extent clamp must be DYNAMIC.** A static ROM of the neutral
geometry has to be disabled the moment a morph or noise moves an edge, and with
it off the thickness sweep of a near-horizontal edge runs clean off its
endpoints (stray spurs). Have the vblank copy engine write each edge's actual
post-transform span, padded by `pad` (+1 px of DDA slack) and pre-clipped to the
head's bbox. Folding the bbox in means the per-stamp path needs two compares
instead of four, and it is impossible to write outside the region the clear
phase wipes.
