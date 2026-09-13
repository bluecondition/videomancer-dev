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
View and light are world constants, so the half-vector is constant; each cube face's H·n is ±one of three whole-cube dot products — 3 dot products per FRAME. One shared H serves diffuse and specular (3 per-pixel multiplies, not 6). — **cubist**, WIP (look verified vs model). Keep ambient OUT of the tilt subtraction or averted faces crush black; shade in linear, gamma via a 256-entry ROM at the end.

### Microcoded geometry engine replacing a hardwired frame FSM
One 32-bit ALU + BRAM register file (2 copies for dual read) + microcode ROM, 3+ cycles/instruction, replaces ~30 hardwired expressions. Serializing the multiplier alone is NOT enough — operand muxes only die when operands come from a register file. — **cubist**, WIP; measured −2241 LUT/−937 carry vs the FSM. A microcode ROM is a constant — a dummy program lets yosys const-fold decode logic and lie about area (tell: EBR count doesn't move). BRAM register file reads a cycle late — explicit RF-READ states. Build the ISA emulator FIRST (seconds vs 20-minute renders).

### Normalized cell units for per-pixel marches
Run the march in units where one cell = exactly 1024 at every resolution (steps 512/a, 512/b from two vblank restoring divides): all constants become compile-time, words shrink to 15-bit, resolution-independent. — **penrose**, timing-closed (81–92 MHz); took the design from 139% LC (unplaceable) to 75%. Reach for it whenever per-pixel word widths scale with resolution.

### Isometric heightfield march on remainder accumulators
Iso coords U=b·x−a·y, V=b·x+a·y as accumulators; cell indices as remainder accumulators (no divide); parallel front→back cover test picks the nearest hit — fills the frame with no sprite-count limit. — **ziggurat** (sim-verified, marginal at full clock) reformulated in **pinboard** (timing-closed, 86% LC): with i=iv+k, cover tests collapse to `ivrem+e >= 0`, face classify to pure sign bits. March depth KF must derive exactly from max height (a "safe" KF+2 was 104% LC vs 86%). Keep everything shift/add/compare — a literal /3 or mod 3 costs ~18 ns. Per-cell gates apply to the finished value, never per-operand. The radial gate/hash must be evaluated PER marched k — sharing one across cells makes a cell's height depend on which pixel views it.

### Near-free shaded sphere
`sh = (R²−r²)>>ks − (dx+dy)>>kd` — level sets are exact circles offset up-left: dome, light direction, and (thresholded) specular from one expression; r² maintained incrementally (`sqx += 2dx+1`). — **pinboard**, timing-closed.

### Polar-silhouette ROM for arbitrary rotating solids
Model the solid offline, store the projected silhouette per elevation as a polar radius table R(θ) in BRAM: membership = one read + one compare; **rotation = a subtract on the angle index**. A combined 1024×12 table returns radius + octant sub-angle in one read — no sqrt, no divider. — **morsel**, HW-tested.

### Octagonal distance metric
`max(a,b) + min(a,b)/2` replaces squared euclidean for round shapes — adds/shifts only, and you store the radius, not its square. Removed 6 multiplies at once, the biggest single win in a 54→80 MHz recovery. — **rollthebones**, timing-closed.

### Light on a curved surface = light·radial projection
Offset the specular highlight by the projection of the light vector onto the local radial direction (`(ex·lx + ey·ly)>>8`) — the highlight slides around bends tracking the light, no CORDIC; straights use the same projection evaluated at the port so families unify; clamp to make elbow highlights concentric. — **pipedream v2.3**, timing-closed (~97% LC, 0 BRAM). Feed the clamp a quarter-px projection or crossovers staircase. The FSM shared multiplier is 2-cycle — reading at N+1 gets the previous product.

### Edge-hash connectivity (stateless network topology)
Hash the shared grid EDGE coordinate: a cell opens edge P iff `hash(edge_coord, seed) < threshold` — both neighbours hash the same coord so they always agree; no stored grid, no double-buffering. 4 edge bits/cell give every join type. — **pipedream v2**, timing-closed; deleting the v1 grid machinery freed ~2000 LC.

### Exact miter-stroke polylines as shared half-plane tests
A stroked polyline with miter joins = per-segment slabs clipped by miter lines → half-plane tests; quantize slopes to shift-add, share a handful of x-products, every plane becomes a constant compare. Derive the plane set numerically in Python first. — **overlook**, timing-closed (40% LC, 0 BRAM, no multiplies). Never sum individually-truncated shifts (one wide sum, ONE final shift); carry 4 fractional coordinate bits or zoom shows texel stairs; octave-mantissa zoom must be `(128+t[6:0])<<oct` for boundary continuity.

### Fract-ring fractal as parallel bit-slices
`uv=fract(uv*2); ring test` iterated: with divisions locked to 2, each level is a fixed bit-slice of one 17-bit zoomed coordinate — all 6 levels evaluate in parallel, shallowest hit wins. Squares via `(16h+l)² = concat(h²,l²)+32·h·l` (two LUT tables + one 4×4 mult). — **orbifold**, timing-closed (0 BRAM, 82–103 MHz). Chebyshev mode needs the radius band halved or it degenerates to grid lines.

### Cheap end-over-end tumble: y·cosθ
Rotation of a flat shape about a horizontal axis under orthographic projection = only Y foreshortens by cosθ — one LUT cos + a few vblank multiplies per sprite. — **tessera**, HW-validated. Drive foreshortening with a TRIANGLE wave, not linear angle (cos dwells face-on and rushes edge-on = fake speed change); x-bound the test or flat edges fire across the whole scanline.

### Multi-plane perspective zoom (planes ARE the depth quantization)
N full inverse-mapped grids, each scaled about the SCREEN CENTRE with per-plane speed gains, composited front-to-back; disjoint per-cell ownership splits content across depths. — **ziffern** NP=3/4, HW-validated. DEAD-END: per-cell corner-scale "zoom" (glyph grows in its fixed slot) is not perspective. Per-plane glyph ROMs must be per-plane generate-block EBR reads (one replicated LUT mux = router wedge); NP=5 is toolchain-blocked (abc9 crash).

### Zoom shells with exp2-LUT interpolated scale
N shells sample a cell-hash starfield in scaled space, per-shell scale swept one octave per cycle from a 256×13 exp2 EBR and LERPed via the shared multiplier (raw 256-step scale = ~2 px radial judder at slow warp); the fade envelope crosses zero exactly at the wrap; wrap-epoch bits re-seed the hash. — **warpfield**, timing-closed. 6 shells = 95% LC; DEAD-END: 8 shells = 115%.

## Polar, circles & CORDIC

### HX4K CORDIC playbook
Vectoring-mode CORDIC for euclidean distance (no mult/sqrt): **8 iterations** (not 14) is pixel-perfect and the LC drop is what closes routing at high util; pack 2 iterations per pipeline stage; G=5 guard bits or small-magnitude error explodes; unscale with a constant-multiply, never a shift-sum (floors small radii). The angle comes almost free → spiral/polar. — **hypnos**, timing-closed (88% LC, 0 BRAM); **chantilly** needed 10 iterations because the ANGLE rasterises dot edges there. Prototype the exact integer CORDIC in Python — float protos hide fixed-point bugs.

### Pre-CORDIC coordinate shift (shift the input, not the output)
For radius-dependent centre effects, nudge (dx,dy) toward the target by weight(r)·error BEFORE the distance pipeline. — **hypnos**, timing-closed. DEAD-END: the "correct" post-CORDIC projection needs a sine ROM + 3 mults = 96% LC and marginal timing. 128 weight levels, not 32, or slow glides terrace.

### 1bpp mark buffer + parity pixel path (nested rings/discs)
A line-early engine writes each disc's two edge x's as blind '1' marks into a 1bpp banked line buffer; the pixel path is a running parity XOR (strict nesting ⇒ parity = innermost disc). Ring count becomes engine THROUGHPUT (`15·N + 56 ≤ clocks/line`), never fit — LC flat from 24 to 110 rings. — **cascade**, HW-validated (126 rings HD). Needs an even ring count; left-overhang discs toggle a line-start INIT bit; clear both banks in vblank; latch the display bank at active rise, never derive it combinationally; adaptive ring count at coarse spacing or piled-up marks invert parity.

### Run the span engine a line (or N lines) early
Trigger next-line span computation at the ACTIVE-line rising edge (BRAM/mult are idle during active video) instead of cramming into hblank; N-lines-early generalizes with N+1 banks — but buys latency only, never throughput (see CLAUDE per-line budgets). — **cascade**, HW-validated. Vblank sequencers counting 20..1: a `when 0` slot never executes.

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
Two half-offset X lattice counters (even/odd rows); per pixel compare dx²+dy² for own-row vs adjacent-row nearest centre — 2 small squarers + 1 compare = true hex cells. — **gumball**, timing-closed + sim-verified (92% LC). Use incremental per-line squares ((d+1)² = d²+2d+1), not a shared line-rate squarer (its mux+mult cone was the critical path).

### One-field fake-Phong sphere shading
A single field e² = (dx−ox)²+(dy−oy)² drives BOTH diffuse dome and specular hotspot; per-frame normalization by priority shift + serial divide keeps the pixel path at a 2-stage shift + 8×8 mult. — **gumball**.

### Honeycomb / tiling lattices without divides
Hex Voronoi of a brick lattice reduces both edge families to linear bisector tests (`|u|=512`, `|u|+2|v|=1280`) — within 8% of regular hex. Generally: every tiling texture = one 10-bit wrapping phase + one comparison. — **fringe**, timing-closed + pixel-exact vs model (2-layer 10-texture engine at 94.7% LC, 0 EBR). For 50%-duty gratings, invert ≡ half-period phase shift — an invert switch duplicates the phase knob.

### Grid-free polar halftone: dots-per-ring = 6·ring
Concentric arcs with dot count ∝ ring index keep cells ≈square at every radius and never align radially; `frac_θ = θ·ring·6 mod 4096` makes the wrap seamless by construction. — **chantilly**, timing-closed (69% LC, 0 BRAM).

## Sprites, rasterizers & line-buffer engines

### Dual-bank line buffer with parity carried per pixel
Inline two bank arrays; read uses live line-parity, write uses the parity CAPTURED at read-issue and carried down the pipe — end-of-line trailing writes land in the correct old bank with zero special-casing, sidestepping the write-addr-delay drift bug entirely. — **shearline** (timing-closed), reused by sidewinder, taffy. A clear-sweep write plus a blend write to one bank = two write ports → FF explosion (169k LC); mux into ONE (addr,data,we).

### SINGLE-bank line buffer (when the stamp trails the beam)
If the renderer stamps column c at `avid_start + c + D` (D = its pipeline depth) while the beam reads column c at `avid_start + c`, every read precedes its own overwrite by D clocks and ONE bank serves both. Write EVERY column (alpha 0 outside the shape) and clear-behind-beam deletes with it — leaving the canonical 1W1R shape with no two-source contention on the single EBR port. — **aurora v1.7**, HW-iterating: 24→18 EBR, and the bank mux + display-bank latch + clear FSM all go. Pad the word to a power-of-2 width with **LIVE** bits (tie-off zeros get pruned): aurora went 14 b (hue9+alpha5 → a 7-block 2048×2 mapping with two writers) to 16 b (hue10+alpha6 — the extra hue bit stays live through solid mode's `hue·7` segment math, the extra alpha bit doubles feather resolution at the same feather width). Check the vblank restamp of row 0 obeys the same D>0 rule.

### Time-multiplexed line-buffer sprite engine
One shared rasterizer walks each sprite's span per scanline into a 1–2-bit ping-pong line buffer; scanout reads/clears the other bank. Fits 10–20 sprites where independent per-pixel rasterizers wall at ~8. — **tessera**, HW-validated. Fill budget = N·(2R+~5) ≤ line clocks — cap size so full count fits; sweep both banks to 0 during vsync (stale frame-wrap content); per-edge thresholds (edge length × width) for even outline thickness. DEAD-END: 10 independent per-pixel rasterizers.

### Sprite records in a FIFO or BRAM — never runtime-indexed register arrays
Fill logic reads only the FIFO head (pop, walk, push-back), or sequential BRAM slots with a 1-cycle-latency state; geometry rewrites in vblank. — **tessera** v7/v11 (20 sprites at 89% LC vs 11 in registers at 96%). DEAD-END: register arrays indexed by a runtime signal (v6: 125% LC of mux trees). Latch the write address WITH the registered pulse when the counter increments the same cycle, or slot 0 is never written.

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

### Per-line sprite-slot selection
Sprites in disjoint vertical bands: pick the active slot PER LINE (2 compares → a RAM address); the per-pixel path never changes. Slot state must live in EBR, not registers (3 register slots = ~125 LC of decode+mux and a blown chip; ~free in BRAM). — **pulp**, timing-closed. Baking geometry into sprite ROMs replaced ~1,400 LC of SDF math with 6 EBR + ~300 LC.

### Glow-kernel ROMs for soft sprites
Per-layer EBR ROM addressed {glow, shape, |dy|, |dx|} → weight; shapes, depth fade, brightness ramps baked into contents; composite in the weight domain before one shared brightness multiply. — **starfield 3.0**, timing-closed; reused by warpfield. Hash-bit bias: when presence = hash < threshold, top bits of survivors are ~always 0 — derive per-item properties from LOW bits, xor-folded. A wrapping background injected ~8 px late is invisible — skip 7 stages of delay-matching.

### Glyph ROM from Unifont .hex
Vendor a GNU Unifont subset, parse with a build hook into a VHDL pkg, flatten into ONE single-port BRAM (addr = glyph·16+row) — combinational multi-row reads triplicate as LUT-ROM. — **matrix_rain**, timing-closed (31% LC).

### Hashed per-scanline filament runs (edge-launched, bundle-coherent)
Each scanline launches ONE run at a fired edge; its length, fire decision and pattern seed come from a 7-stage single-op spatial hash of (frame_line >> K, column/16, seed), so lines are independent at K=0 and bundle into coherent sheaves as K grows — patch-level randomness with zero storage (no per-column line buffer). A 16-bit phase accumulator inside the run drives all pattern engines; output is a lerp `ground + (base-ground)*int/256` so fades land on whatever the ground is. — **fray**, timing-closed (6/6 seed 1, HD 79–89 MHz, 57% LC, 10 EBR). Lessons: the first shift-add hash had 0.9999 neighbour correlation on its top bits — search shift constants numerically before building; start the hash one column ahead (a cx+1 counter) to buy a stage for splitting the length multiply; never let an accumulator's carry-out drive a wide register's enable — register the wrap and act a pixel late.

## Displacement & feedback

### Rotating-bank displaced-scanline gather (Rutt-Etra)
Vertical luma-displacement without a frame buffer: make displacement one-sided (down from the source row), so every line crossing the current pixel comes from a row ABOVE — already streamed. Every Nth row writes `{disp8, bright4}` per column (cols ×2 decimated) into one of K=8 rotating banks (1024×12 = 3× 1024×4 EBR each, 24 EBR); per pixel, 8 parallel `age_k − disp_k` window tests + a recency-priority resolve (rotate hit vector by head, priority-encode) pick the nearest line — recency IS the hidden-line rule, and `age ≥ disp` coverage gives free opaque terrain skirts. Max disp = K×spacing−1 (denser lines ⇒ shallower relief, inherent). Mask the head bank on its write row (read-during-write undefined); per-bank kept read-address registers beat the 8-way EBR fanout net. — **ruttetra**, HW-validated (77+ MHz HD, 53% LC).

### Coarse frame store in EBR (frame-recursive pictures without a frame buffer)
A whole frame fits on-chip once it is coarse and/or shallow: 256x128 x 1 bit = 8 EBR, 64x32 x 16 bit = 8 EBR (power-of-2 grid on both axes so the address is a plain `{row, col}` concat). Two banks ping-ponged by frame parity (write bank(par), read bank(!par), both read every clock) give a true recurrence: this frame's cell = f(this frame's sample, LAST frame's cell read from a *displaced* address) — decay trails, zoom/drift/spin feedback tunnels, random-walk diffusion, temporal derivatives. Mechanics that made it clean: (1) cell geometry from two DDAs (`xacc += cols; if xacc >= W` per pixel, same per line) on the runtime-measured width/height, sampling the row's MIDDLE line (flag when the row accumulator passes H/2); (2) one-pole IIR along the line, latched at the cell end = box-ish average with no divide; (3) the display read (cell of the current pixel) owns every cell-START slot of the shared read port, the feedback read is a *request register* issued on the next non-start slot (1-clock deferral), and a display hold register covers the stolen slot — so any cell width >= 5 px works; (4) all per-cell feedback math is a **pulse-tagged pixel-rate pipeline** (a valid bit travels with the data, one op per stage, no per-cell hold registers, so pipeline depth is unconstrained by cell width); (5) a free-running per-cell chain (hash, centre offsets, Bayer) computed for the CURRENT cell and captured at the end-of-cell pulse — its depth must be <= min cell width - 1. With 1-bit cells, persistence is *stochastic* (cell survives if hash < P) and sub-cell zoom velocities come from stochastic rounding of the fractional offset (floor + carry-if-frac > hash) — both give continuous control at 1 bit. Storing the previous SAMPLE beside the previous OUTPUT (mosaic's 16-bit cell) is what makes |sample - old sample| a real motion signal; any law that feeds its own output back through XOR/subtract oscillates on static input. — **afterimage** (1-bit 256x128, 16 EBR, 3882 LC = 51%, HD 84.9/91.1/92.4 MHz all seed 1), **mosaic** (colour 64x32, 16 EBR, ~4700 LC = 61%, 4 laws: trails / stochastic-IIR blur / motion / hold; HD 78.0 (seed 3; seed spread 69.6-78 with the critical path in the CORE's SPI peripheral, not the program) / 82.0 / 83.4 MHz), both timing-closed, not HW-tested. Genre caution: the coarse store should BE the picture (blocks as primitive) or drive a structural re-render — a smooth coarse field modulating the live raster is the rejected overlay genre.

### Recursive line feedback (diagonal smear)
Each scanline reads the previous OUTPUT line at (x − shift), lerps with live video, writes back; accumulating shift = diagonal comet trails. — **shearline**, timing-closed. The blend must be the 1-multiply form `live + decay·(prev−live)>>10` — the 2-mult form stuck HD at 65 MHz.

### Per-scanline wave displacement with blanking-time step sequencers
Write live line to bank(par), read bank(!par) at a wave-shifted address; ALL wave math in registered step-FSM stages during hblank/vblank — zero per-pixel math beyond counters. — **sidewinder**, timing-closed; forked into taffy. Wrap without per-pixel modulo: init a wrapped read pointer once per line (single mod), then one compare per pixel; track a parallel unwrapped counter for the out-of-range test.

### Per-line envelope BRAM + DDA stretch
A 2048×10 BRAM of per-line envelopes {state, level} advanced once per line at separate attack/release rates; the effect is a DDA read address. Effect-agnostic — new modes just remap `env`. — **taffy**, HW-seen and liked (50% LC). A passing slider can't show a complete gesture — add a one-shot armed→attack→decay→spent machine. Register the RAM read address (a combinational accumulator→compare→RADDR cone was 15.2 ns).

### Bounded per-row pull (tape flag), NOT a cumulative shear
A per-line DDA that accumulates a shear is unbounded by construction: even 1 px/line is a full screen width over 1080 lines, so almost the whole slider travel shreds the picture into diagonal slabs. Make the row offset a bounded POSITION function instead — an exponentially decaying pull seeded at the frame anchor (top-of-frame flag) plus a second one re-seeded at the head-switch band — amplitude = a fraction of the measured width, so the extremes bend the image instead of destroying it. Decay by `bend - (bend>>k)`: one subtract, no compares, and it is the shape real flagging has. — **deadchannel**, timing-closed (all 6, 74.8–80.6 MHz, 4.4k LC). HW-REJECTED in the cumulative form ("just diagonal streaks"). The linear "subtract a rate, clamp at zero" decay chains two compares and an add into one pixel-clock cone: 6 MHz. Fold per-line jogs (dropout tears) into the same offset so Y and chroma stay locked, and wrap with one conditional add/sub (offset < width).

### Line-average edge fill for displacement overshoot
Latch a 64-px line average (window starting past column 8 — capture sources carry black blanking-edge columns) at hsync; paint it for reads within ~6 px of either border. Trigger on reads of columns < C_EDGE, not just off-screen reads — that's what kills the seam; gate on displacement ≠ 0 or dry lines paint their own edges. — **lagoon** (HW-validated), taffy, redshift. Taffy invariant: in Stretch the sample always lands between x and centre, so clamp logic only ever fires in Squeeze. Per-side variant (2026-09-13, the nine catalogue-combo warp programs mirage/halation/tremor/etchwarp/impasto/parallax/flutter/hueflutter/trapquant): sum columns 8..71 and width-72..width-9 separately, latch both averages at the avid fall, and re-latch them into the fetch stage on the delayed avid (`s_avid_sr(K-1)='1' and s_avid_sr(K)='0'`, K just past the fetch stage) so the colour swaps between lines, not mid-line; a side flag (`addr >= width`) pipelined with the bg flag picks left or right. ~60 FFs + six 16-bit adders, off the pixel critical path. SUPERSEDED the same day after a hardware look: the black blanking columns at the line ends still showed as a seam between picture and fill (and filter windows, pixel cells and keystone scaling widen it). What worked: an address CLAMP, `f_edge_addr` -- any read < C_EDGE (12) or > width-1-C_EDGE is redirected to that inset column, so the fill is the line's own edge pixel (no accumulators, no fill mux) and the blanking columns never reach the screen; windowed filters (impasto box sums, halation blur cascade) additionally hold the last active luma through blanking and re-sync their running sums to hold*N so the edge pixel itself is not darkened. Timing trap in the clamp: never compare the mirrored / wrapped address (it sits behind a negate or ±width adder) -- flutter fell 80 -> 67 MHz. Compare the SOURCE value against per-line registered bounds (-w_edge, 2w-1-C_EDGE, w+C_EDGE, C_EDGE-w, w+w_edge) in parallel with the adder, and let the final mux be the only serial step: back to 78-79 MHz.

### Scan-order per-column state carry (waterfall/sand/lava)
Per-column physics state carried down the frame in scan order at HALF horizontal res (even pixels update, odd render); state in narrow BRAMs; a frozen output-luma ring gives cheap reflections. — **spillway v0.2**, sim+routing-verified (89% LC). Label BRAM reads with the column the DATA belongs to (dout lags addr by 2) — ±1 = diagonal drift, instantly visible in image sim. Binary thresholds look "unfinished" — graded margin opacity + pixel-rate LFSR dither. A registered shared-multiplier SUM adds a consumer latency cycle — every consumer silently read stale products.

### 2nd-order spring for weighty slider response
Asymmetric stiffness (attack err>>2 / release err>>5), SHARED damping (vel>>5+vel>>6, integrate >>3): one ~17% backwash lobe, ~2 s settle. — **spillway** floodgate (constants reusable). Branching damping by error sign kills the ring; needs anti-windup (clamp zeroes velocity); stage the 4-term accumulate across ≥3 sequencer steps.

## Accumulator & DDA engines

### Accumulator-pair projection (rotation without multiplies)
Never compute `x·cosθ + y·sinθ` from coordinates — advance the projection by `cos` per pixel and `sin` per line. Continuous angle free; scale = a per-frame multiply on the step; more effects = more pairs; carry 8 fractional bits. — **ribbon** (HW-iterating), **videosky**, **overlook**, **intaglio** stripe fields. Seed accumulators at 0 per frame (a periodic field's origin is invisible phase; centring costs 2 vblank mults). Per-line DDAs advance on ACTIVE lines only (CLAUDE: interlace).

### Continuous-pitch/angle screens from a phase pair
Phase P (24-bit, wraps phase-continuously since 2^24 mod 2^16 = 0) with lines as level sets; Q = P rotated 90° gives crosshatch and a modulation argument free. Adding luma to the phase BEFORE the thickness compare = relief displacement — lines bow around the image, far richer than thickness modulation. — **videosky**, timing-closed (41% LC, 0 BRAM).

### One accumulator = cell id + sub-cell coordinate (+ free octaves)
A 24-bit DDA with 8 fractional bits yields sub-cell coords (15:8) AND cell id (23:16) — a rotatable, scalable lattice for the cost of a level-set. An octave is a left-shift of the warped coordinate; octaves share accumulator + domain warp. — **daedalus** (HW-viewed, 0 BRAM). Domain warp must land on the full coordinate BEFORE the cell/sub-cell split; never vary pitch per pixel — warp the domain instead.

### Fractional phase-accumulator grid engine (sub-pixel zoom)
Per-pixel `phase += step` (Q10.18): cell = high bits, glyph column = mid bits; step from a reciprocal LUT with endpoint LERP; centring = −centre·step via a vblank shift-add multiply. — **ziffern**, HW-validated. Integer pitch + modulo centring = jitter; FRAC ≥ 18; pot ADC noise needs a glide filter, not more precision. Compute LUT differences from REGISTERED reads. Sim only at `--decimation 1`, warmup ≥ 12.

### Handshake-sequenced vblank multiply chains
Per-frame sequential multiplies that feed each other chain by done-handshakes (producer registers → consumer starts 2 cycles later), never all launched at vsync — a stale read renders fine but MOVES wrong, and no still sim catches it. — **ziffern**, **warpfield**, **pinboard** (all 11 per-frame ops on ONE 16×16 shift-add sequential multiplier, 18 clk/op — a parallel multiplier would be STA-timed at pixel clock for no benefit).

### Golden-ratio additive sequence for even-but-random placement
`pos(i) = base + i·0.618·range (mod range)` — low-discrepancy, no clumps, no visible grid (shift form: 39/64). — **tessera v12**, HW-validated. Random hash offsets CLUMP (Poisson) — that's the failure this replaces. Split accumulate-with-wrap across two stages.

### Multiply-free wave terrain via phase accumulators
Per-pixel phase accumulators reset to a per-frame scroll offset at line start; crest = sine + triangle at distinct wavelengths; per-layer fractional scroll = parallax; per-layer hazed palettes precomputed per frame. — **candyhills**, HW-iterating. Frame-lock LFOs — a free-running per-line LFO aliases against frame rate (bajaweave's phantom "sweet spots").

### World-anchored pattern coordinates
Any screen-anchored frequency/fan term slips against scrolling terrain (apparent speed = phase-rate/gradient); anchor the modulation to WORLD coordinates — e.g. fan = cos of the terrain's own crest sine at world-x. — **candyhills**, HW-iterating; ~10 HW rounds proved every screen-anchored variant slips. Keep world terms mod the pattern period (wrap shear). Each sin/cos LUT output is its own ~3-EBR table — carry the angle down the pipe and mux at the winner stage instead of duplicating.

### Stateless per-column sprite evaluation for clean edge scroll
Objects attached to scrolling terrain must derive per-pixel from current-line values only — any held per-slot anchor mis-evaluates the leading partial slot at screen-left (sprites pop). If a held anchor is unavoidable, seed it from column 0 at line start and keep it line-local. — **candyhills** bushes, HW-iterating.

### Rotated-coordinate field for slanted banding
Register `y_rot = y − x/2` once; use its bit-slices for band selection, border tests, and per-band seeds — every derived feature automatically shares the slope. — **solitude**, user-approved (52% LC).

## Noise, texture & hashing

### Hash discipline
At least one integer ADD is load-bearing (xor/rotate is linear over GF(2) → visible lattice structure: whole rows merging, diagonal weave); use exactly as many adds as needed (each lengthens the critical path — split multi-add hashes across stages). Prefer shift-and-ADD bijections (odd multiplies ×9/×33/×5, xor-with-own-high-bits) so distinct cells never collide; threshold the TOP bits (best-mixed). Never feed a hash `x` and `x xor const`. Spot-check every new hash numerically (CLAUDE). — **mondrian**, **inferno**, **daedalus**, **spillway** (mult-free xorshift on counter seeds has no avalanche — 3 staged add-shift rounds).

### Dither/decay randomness rules (three HW-observed failure modes)
(1) `hash(addr)+framecount` TRANSLATES — fading content appears to scroll; use `hash(addr) xor lfsr_snapshot`. (2) x-LSB xor line-LSB dither reads as a CHECKERBOARD near thresholds; use x bits ≥1 xor a per-line-stepped LFSR. (3) An avid-gated LFSR FREEZES in vblank — a vblank FSM sampling it gets a periodic value (global pulsing); sample during active video and latch. — **fireworks/lagoon/inferno**, all HW-caught, all sim-invisible.

### Aperiodic masks: no bit-hash, no global time bits
Never XOR a global time bit into a per-pixel presence mask (the whole row flashes as one line); any XOR/AND of coordinate bits is periodic (micro-motif tiles visibly). A hand-placed aperiodic 64-entry constant mask ROM (maps to LUTs), indexed by `x xor small-y-shift`, LC-neutral vs the hash. — **pixelbeach** foam saga, HW-confirmed failures.

### Patch-level randomness, never pixel-level
Organic "hand-made chaos" = groups of parallel regular lines whose POSITION/SIZE are random (latched per run, carried via a per-column line buffer) with zero per-pixel noise — per-pixel grain reads as static on luma, full stop. — **etchplate v3.1**, sim-verified vs reference. DEAD-END: independent per-line runs + per-pixel dropout (rejected on HW as "static applied to luma").

### Servo-locked per-line gradient
`g += const` per line; at vsync `gstart += target − g_measured` — converges to an exact screen-height fit in any mode, zero division. — **inferno**, HW-iterated; variant in lagoon.

### Anti-terracing on tall noise cells
Tall lerp cells hold the smoothstep fraction constant for many rows; the terrace boundaries align across columns and read as contour lines. Finer fraction table (64-entry) on tall-axis lerps + LFSR dither below the step size; ridged folds DOUBLE step contrast — dither those too. — **inferno v3.5**, HW-iterated.

### Anti-lattice: per-column vertical phase
A cell grid always reads as rows of dots and per-cell jitter can NEVER fix it (a feature only wanders within its own cell). Slide each cell COLUMN vertically by its own random phase — boundaries move with the content. — **morsel**, HW-tested. Per-row horizontal + per-column vertical together is circular — pick one.

### Binary mezzotint comparator
Luma vs dithered-threshold compare re-renders the image as grain density, zero multipliers. — **grist** exploration, Python-preview only (the standout of 9 candidates; amplitude-max and 64-px-block variants were weak).

## Edges & filters

### Shared multi-tap luma window with uniform mode latency
One 5×5 window (4 chained line buffers) feeds ALL operators; simpler modes pass through the deepest stage so every mode shares one latency and downstream controls behave identically. — **edgelab**, timing-closed (83% LC, 29 EBR). Shift chains crossing hblank leak left-edge garbage — blank at THREE offsets (window fill, accept guard, output guard).

### Kirsch compass without 8 wide ALUs
`max_i |8·Si − 3T| = max(8·maxSi − 3T, 3T − 8·minSi)` — one min/max tree over the ring triple-sums replaces 8 directional convolutions. — **edgelab**.

### Quarter-res streaming Canny for edge EXISTENCE
Detect at 4-px column granularity: 4-px average → [1,2,1] blur → Sobel → |gx|+|gy| → double threshold → line+left hysteresis; skip NMS when only presence matters. MUST be a 4-stage pipeline (monolithic = 46 MHz). Suppress accepts on the first 2 active lines (buffers hold last frame's bottom; pixel_y counts from vsync so `y<2` never fires). — **inferno v3**, HW-iterated.

### Block-float gradient normalization
One shared priority-encoded shift (from |gx|+|gy|) scales both gradient components: direction exact (hue/env lookup, no atan2), magnitude log-compressed. Split shift-select and barrel+clamp into two stages; the height field must be 10-bit + h-blurred (8-bit contours ring once normalized). — **mercurial**, timing-closed. (The field-overlay GENRE is rejected — reuse the trick, not the genre.)

### Gradient-direction bucketing with hysteresis
Bin to 8 directions via shift-approximated tan compares, then ROUND to 4 (45°) — direct 22.5° selection shreds soft shading (DEAD-END); hold-last-strong-direction (~63 px run) so low-|∇| zones inherit direction instead of striping. Direction floor ~24, not ~96. — **intaglio**, timing-closed. Companion: octant-fold + 3 parallel shift-add tangent compares = 5-bit hue sector, no atan2 (**turpentine**).

### Thick class-boundary outline via a {class, age} line buffer
Tracing the border between two per-pixel classes in a streaming pipeline: horizontal strokes from an N-bit class shift-history (differ-from-any tap = N px stroke), vertical strokes from a ping-pong line buffer storing {class, age3} — each line: edge fires → age = thickness; else age = max(age−1, 0); outline = edge or age ≠ 0, so the stroke extends `thickness` lines downward with no multi-line window. The age update is a legal cross-line RMW because ping-pong reads last line's bank while writing this line's. 4-bit word × 2048 × 2 banks = 4 EBR; read issued 2 stages early + fabric re-register; write trails read by the pipe offset so addresses never collide. Gate h-edge with `px ≥ 8` (blanking garbage in the history) — **cleave**, timing-closed. Stroke is asymmetric (below/one side of the edge); reads as intentional jitter on organic class fields.

### Threshold hysteresis + edge qualification for real video
±24-luma hysteresis on any 1-bit threshold, plus qualification (a transition counts only if the run it ends was ≥6 px) — kills micro-edge chatter that chains into full-width bars. — **etchplate v3.2**. Clean synthetic test images hide this class entirely.

## Video processing

### U/V swap once at each boundary
Author internals in standard BT.601; swap chroma ONCE at the input latch and ONCE at output compose; passthrough stays identity. — **gilt** (HW-confirmed: standard-authored gold rendered AQUA), prism, cga. Pre-swapped stored palettes also work but preview swapped in sim. Turpentine has NO swap and is likely wrong on HW — don't copy it.

### Whole-glyph ROM fetch: a free 3x3 neighbourhood for text/sprite edges
Store the font/sprite as ONE word per glyph (`128 entries x 64 bits` = an 8x8 glyph, 3 EBR) instead of `char x row x 8 bits`. One read hands you rows y-1, y and y+1 at once, so the full 3x3 neighbourhood is a couple of OR gates: `dil = r(y-1) or r(y) or r(y+1)`; `dil3 = dil or (dil sll 1) or (dil srl 1)`; `outline = dil3(7-x) and not self`; `shadow = (r(y-1) srl 1)(7-x) and not self`; `hilite = (r(y+1) sll 1)(7-x) and not self`. That buys drop shadow, black edge, hollow and emboss — the whole 1980s character-generator style set — for ~400 LUT4. `srl` moves toward higher column index, i.e. RIGHT. — **titler v2**, HW-VALIDATED, 6/6 at 73.5-87.5 MHz. Extract rows with an explicit 8-way `case` function; a computed slice (`g(63-8*y downto 56-8*y)`) is a synthesis hazard. THE TRAP: a 1-px edge stays inside its cell horizontally (glyphs in cols 1..5 of 8) but NOT vertically — the row above a glyph's top row belongs to the cell above, holding a different character, so every glyph silently loses the top of its outline. Fix by giving each text line its OWN top-margin row (titler's block is 18 rows: 0 = line 0 margin, 1..8 = line 0, 9 = line 1 margin, 10..17 = line 1) and on a margin row fetching that line's character with `r(y-1) = r(y) = 0, r(y+1) = glyph row 0`. Half-extent then becomes `scale*9`, not a pure shift — resize BEFORE shifting.

### Active-region counters make position knobs mode-portable AND the DDA interlace-correct
Reset the horizontal counter at `avid_start` and advance it only while `avid`; advance the vertical counter only at `avid_start`. Every geometry term is then in active-picture coordinates, the vertical DDA automatically obeys "advance on ACTIVE lines only" (no 30 Hz interlace twitter), the text/shape band can never land in blanking, and the height counter does NOT zero on blanking hsyncs — so it is safe to sample at vsync, unlike a raw line counter. Compare the vertical counter BEFORE its increment: that value is the index of the line just starting, which is what makes origin 0 work. Width = `h_act + 1` at `avid_end` (h_act stops on the LAST active index). Feed knobs through `(knob * meas_w) >> 10` on a shared serial shift-add multiplier in the vblank sequencer — a plain `*` is still STA-timed at the pixel clock and will not close. — **titler v2**, HW-validated. Symptom this fixes: v1.0 hard-coded a 4092-px span and crammed ALL of SD's placement into the bottom ~20% of the knob (~27% on HD).

### Half-rate 8-bit chroma line buffers (dodging the 30/32 EBR cliff)
Three full 10b×2048 dual-bank line buffers cost 30/32 EBR — right on the router congestion cliff. Store Y full-rate/10-bit but U and V at half horizontal resolution and 8 bits (write addr `px(10 downto 1)`, read `addr(10 downto 1)`, expand with `shift_left(resize(u8,10),2)`): 18/32 EBR, and neutral 128 expands to exactly 512, so there is no missing-neutral-chroma trap. Chroma displacement then quantizes to 2 px, invisible on smear/misconvergence effects. — **deadchannel**, timing-closed. The buffer has no write enable, so a half-rate write address simply point-samples the odd pixel.

### Decode every control free-running, never in one frame-event branch
Latching all parameters in a single `if <frame anchor>` branch makes every knob and switch depend on that one event firing. When it stopped re-firing on real video timing, deadchannel rendered perfectly but was completely deaf: picture fine, all twelve controls dead. Read the SPI registers CONTINUOUSLY instead and pipeline the derivation one op per stage; only genuinely per-frame state (line count, band position, seeds) belongs in the anchor, backed by a line-count ceiling so housekeeping survives a stuck anchor. Costs nothing and it ran FASTER (77 → 83 MHz) because the anchor cone shrank. — **deadchannel v1.2**. Diagnostic: flip an unmissable switch (Invert). If it does nothing, suspect the latch branch, not the pixel path.

### Per-line geometry belongs OUTSIDE the per-line decision cone
Anything evaluated at hsync or active-video-fall is timed at the pixel clock, so a per-line test like `|aline - band_pos| < half` (subtract → abs → two compares, all chained) becomes the design's critical path — 19 ns / 51 MHz in deadchannel. Register the four band EDGES once per frame and per line do a plain register-to-register compare; likewise run knob→amplitude→rate derivations as a free-running one-op-per-cycle chain and let the frame anchor merely SAMPLE the result. Same design, 51 → 76 MHz, no feature cut. — **deadchannel**, timing-closed. Corollary: reading an SPI register inside the anchor cone drags a deadband, a variable shift and an abs onto that path.

### Quadtree by parallel independent hashes
Cell size resolved from 3 INDEPENDENT coarse-block hashes (64/32/16 px) evaluated in parallel — timing-shallow, neighbours subdivide independently. Normalize in-cell coords to 0..63 so motifs are size-independent. — **gilt**, timing-closed (77% LC). Quadtree squares can't make tall shapes (tall bars keyed to absolute sheared x across stacked cells). A second overlay closed easily BECAUSE it mirrored already-closed stage shapes.

### Grid-aligned cell merge (cheap "not-square")
Cells stay grid-aligned; each hashes grow-right/grow-down bits; a pixel resolves against the 2×2 up-left neighbourhood — coverage is pure hash-bit logic, no coordinate compares. — **mondrian**, timing-closed. DEAD-END: porting quilt's jittered off-grid overlap (+1300 LC, past the congestion cliff). Mondrian's continuous-grid HD Dual was a pure routing wall — shipped as a hybrid vmprog with a stepped hd_dual slot.

### Per-cell-column luma hold for stable region decisions
A decision that must be identical across a patch can't read per-pixel luma: sample one luma per coarse cell into a small RAM and hold. For decisions used within the same cell row, ping-pong by cell-row parity (one row of lag). — **quilt**, **morsel**, both HW-tested.

### Case-selected fixed bit-slices instead of variable shifts
Scale logic rarely needs a barrel shifter — extract the 2-bit ramp field `coord(s:s−1)` + a few bits via ONE case over the scale setting. — **quilt**, HW-tested (−120 LC, 8 barrels removed, +3–8 MHz).

### Separable axis tests for 2D regions
Factor region membership into independent X-only and Y-only bits in one stage, AND/OR-combine in the next — a fused 2D comparator tree was a ~16 ns wall (55 → 101 MHz). — **revenant**, timing-closed. Sibling: build all N clamped candidates in parallel + shallow-tree argmax (**quilt**; serial argmax = 35 MHz).

### Iso-bar height field as fake pixel-sort
True vertical pixel-sort needs per-column color storage (blows EBR at 1280 wide). Iso-bars of `px + (luma>>shift) + ripple`, bar hue locked to a column hash, brightness riding live luma — reads as vertical streaks, 0 BRAM. — **glitchscape**, timing-closed. Mix two diagonal triangle waves + a leaky random walk for organic bends (one triangle wave = hard zig-zags).

### Move the threshold, not the gain
Knob-controlled intensity of a thresholded feature: move the THRESHOLD, not a per-pixel multiply on the field. Replace K² knob response with a piecewise-linear shift knee (~120 LC per multiply saved, feels identical). — **lagoon**, HW-validated.

### Per-frame constant → shifted-bit-term decomposition
A frame-constant narrow multiplier operand (mix 0..8): replace the multiply with per-bit shifted terms summed across two stages. — **doom v2**, timing-closed; **ziffern** gains. Companion: inline 3-stage lerp `a + (t8·(b−a))>>8` replaces SDK `interpolator_u` (t=0 exactly dry, ≤4 LSB full-wet error; freed ~400 LC and took HD Dual 69→87 — **turpentine**).

### EBR lookup-table quantizer with a free-running serial builder
Per-pixel nearest-of-N colour (N parallel |dy|+|du|+|dv| units + argmin tree) is ~3000 LC for N=16. Instead index a 1024x4 EBR by luma(4b)&u(3b)&v(3b) and let a 5-state serial builder recompute every cell forever (~1.3 ms/pass) against a LUT-ROM palette; the pixel path is LUT read -> index -> palette ROM read. A Bayer dither ahead of the lookup hides the cell edges. Palette switches rebuild over the next pass, no vblank budget. ~250 LC + 1 EBR. — **croupier**, timing-closed only (89% LC, 25 EBR).

### Memoryless 2D area averaging as anti-flicker
Quantization-boundary flicker is best fought with memoryless spatial averaging (cannot oscillate on a still image). — **c64**, HW-confirmed. True 2D averaging has 1-cell-row latency (blank the top row via a stale flag). DEAD-END: c64's cross-frame per-cell hysteresis grid made flicker worse. For palette-match flicker specifically, CGA's margin hysteresis + two-field debounce IS HW-validated (M=1 → 10–50×, M=2 → ~400× fewer flips; re-seed the grid on contrast/mode change or decision controls go dead under Freeze) — the mechanism difference matters: margin-on-challenger, not keep-last-unless-beaten.

### Cell-jitter primitive clipping fix
A jittered primitive drawn only in its own cell clips into half-moons: either test neighbour cells or size the jitter budget so r + |j|max + 1 ≤ cell half (shrinking as it's thrown reads as intentional). Pipe each pixel's own cell context down the render lanes; guard unsigned subtractions in size math (a wrap becomes a giant primitive). — **turpentine v3**.

### Content field: drive ornament STRUCTURE from a blurred copy of the input
Build a low-res blurred copy of the incoming video (one-pole IIR along the line for horizontal, a decimated single-bank line buffer feeding a second IIR for vertical — luma and both chroma axes, 3 EBR at 512×24) and drive every *structural* decision from it instead of from screen coordinates: warp the pattern lattice by it (luma displaces x, chroma displaces y, up to ±128 px), XOR a quantized *material* id (tonal band + chroma sector) into the cell-identity hash so pattern boundaries land on image contours rather than cell edges, and let brightness/saturation set cell size, ornament density, shade and accent hue. The field is never drawn — it only gives the generator something to follow that is not a coordinate. — **gilt v0.7**, timing-closed (88% LC, 9 EBR, HD 75.3–77.9 MHz). Cures the "looks mathematical / follows a grid" failure that a quadtree over (x,y) could not: irregularity computed from position is still arithmetic laid over the picture. Keep the field pipeline FREE-RUNNING beside the pixel path — its values arrive 3–6 px stale, invisible under an 8 px blur, and that is what keeps the pixel stages depth-neutral (per-frame constants become registered per-pixel signals with no added depth). EBR trap: read addr = x>>2 and write addr = the same group delayed 2 cycles, enabled only when x mod 4 = 3, so read and write never touch one word. Open risk: no hysteresis on the band index, so watch for shimmer where the smoothed value hovers on a material boundary. **v0.8 follow-up (HW feedback: "still too much grid"):** the field alone was not enough — the *rendered* lattice lines (cell seams) and any motif keyed to a fixed pitch (16 px bars) re-introduce the grid no matter how the lattice is warped. Delete seams outright; give bars a per-cell axis/pitch/phase; turn discs into blobs with an 8 px-coherent wobble hash on the warped lattice (±64 on r²) folded with the grain into one adder operand; add per-cell size/squash (shift-muxes on |dx|,|dy|), ring pitch and dark↔light inversion. DEAD-END for timing: a per-cell shape term (±(|dx|+|dy|)≪k) *inside* the r² stage — adder+mux in front of the sq-LUT sum = 69.7 MHz and a router-stalling 91% LC. Deform the coordinates with shifts, never the sum.

### Worley (nearest-jittered-point) cells for an ornament partition
When a cell-based generator must not read as a grid, the partition itself has to go: a quadtree of axis-aligned squares stays a grid no matter how the coordinates are warped or how much per-cell randomisation sits on top (gilt v0.7/v0.8 both failed on hardware for exactly this). Worley cells fix it structurally: a lattice of points (pitch 2^k, k from a knob), each hashed anywhere inside its cell; per pixel test FOUR candidates — own cell plus the neighbour on the pixel's side along each axis — and take the nearest. Use the **octagonal metric** max(|dx|,|dy|) + (|dx|+|dy|)/2 to choose (no squares, 8-bit), then run the existing r²/max/bar geometry on the winner's offset only. Pipeline: cell+fraction (normalised to a 0..63 space so downstream is pitch-independent, with the fraction pre-offset ±64 for the neighbour candidates) → 4 parallel hash_a → 4× (subtract + abs + sign) → 4× octagonal d → **six parallel compares** → one-level argmin + winner mux + the winning cell's identity hash_a; six stages, +4 latency. Cost ≈ 4 hash_a + 8 sub/abs + 4 metric + 6 compares + muxes ≈ what a 3-level quadtree with its own normalisation cost, so it was LC-neutral once the Luma overlay was made to SHARE the cells (own hash salt only) instead of running a duplicate engine. XOR the picture's material id into the point hash and the tessellation itself re-rolls at image contours. Whole-cell lookups (tile RAM by cell origin) must address a row that is already sampled: use half a pitch above the winner's top edge (the lower neighbour is only chosen from a cell's lower half) and size the ring for two pitches. — **gilt v0.9**, standalone 3808 LUT4 / 10 EBR; timing status recorded in the gilt memory.

## Physics, particles & animation

### Memoryless particles from an age ROM
Recompute positions each frame from age via a small ease/droop ROM; evaluate pos(age) and pos(age−1) and line-walk between. No per-particle position RAM. — **fireworks**, HW-iterating. Product widths: rise `ease×height` hit 138 — a 7-bit slice wrapped tall rockets to the ground. Decorrelate angle/speed with a bit-reversed particle index (sequential indices step 6.3° and draw a visible spiral).

### Memoryless replot = scrubbable time
When particles are pure f(seed, age), age becomes a control: freeze + slider scrubs the timeline both directions (bursts collapse back into rockets); hand-fire is an age override. — **pyro**, HW-iterating (89% LC, 32/32 EBR). Never read SPI registers inside the vsync decision cone (~5 MHz) — decode to free-running 1-bit flags.

### Voronoi color attribution at readout
Color a monochrome persistence canvas by nearest-centre argmin over live emitter centres at READOUT — zero color BRAM; subtract 2× each shell's live radius so overlapping shells own their own ball. — **fireworks**. Clamp weighted distances to 8 bits — a 5-way argmin of 10-bit values was itself the critical path.

### Symmetric shifts in feedback loops
`shift_right` (floor) on a signed feedback term is an energy pump (positives never damped to 0, negatives always get −1) — the system charges and collapses cyclically. Any feedback term uses symmetric shifts; floor is safe only feed-forward. Every signed floor-shift feeding an unsigned slice needs a `>= 0` clamp. Stress-sim physics at max knobs for 60+ frames before building. — **pixelbeach**, HW-confirmed then fixed.

### Conservative flux form for grid simulations
Quantize each interface flux ONCE, add left / subtract right — shifting the flux difference per column leaks mass (shoreline creeps). Never clamp height on dry cells (adds mass); pairwise diffusion in flux form kills the odd-even checkerboard. — **pixelbeach** shallow-water, physics HW-validated.

### Display snapshot banks (physics rate ≠ display rate)
Copy live physics BRAMs into display-only snapshot banks right after vsync; display reads only snapshots, physics owns the live RAMs all frame — tick budget becomes the whole frame, no port mux, no tearing. Gate decay effects on the field's last tick so visual rates are tick-count-independent. — **pixelbeach v3.5**, HW-validated (+2 EBR).

### Double-banked slot RAMs (frame-parity banks)
Any per-column/per-cell map built during the frame but read by the raster mid-frame MUST be double-banked (addr MSB = frame parity; swap at vsync; vblank-sweep the incoming bank). — **inferno** luma-mod, HW-iterated. DEAD-END: single-banked + slot-policy tweaks (only rarer). DEAD-END: frame-age tags (stale slots resurrect on wrap).

### Per-lane lifecycle engine with a sliding 3-lane window
N vertical lanes hold small state records in EBRs; a vblank FSM steps lifecycles; render prefetches prev/cur/next lane records through the shared read port and tests 3 candidates per pixel — motion crosses lane borders, neighbours merge. — **sanguine**, v0.1 HW-approved. Resolve per-lane decisions against LAST frame's BRAM outputs (1-frame lag invisible); (lane,line)-constant values go in the window record; OR-flag priority beats encoded-code compares.

### Per-column lighthouse register
One EBR of per-column {strongest value, its row}; influence = val − dist·2^k — 2-D vertical propagation in a streaming pipeline, zero extra ports, one-field lag invisible. — **kudzu**, timing-closed (96% LC).

### Cell/phase counters rasterize any cell size
Per-pixel {phase, cell-index} counters (wrap → increment) rasterize ANY cell size, zero division; measure usable cells FROM the counters at line/frame end. — **fireworks** (HW), **mondrian** continuous grid. Counters saturate at the canvas edge AND coords clamp before address formation; store cell size/mask as registered per-frame signals.

### Stabilise the compare's INPUT, not the decision
Flickering presence compares: deadband the control, average the sample over the whole cell, refresh state on ONE field under interlace. — **morsel**. DEAD-END: softening the decision (grow features from a speck) — size wobble reads as wrong as flashing; quantising the input AMPLIFIES dither near step edges.

### Base+delta reference storage
Several thresholds of form `base + small_delta`: store the 8-bit deltas, derive every compare from ONE shared `row − base` subtract (~65 LC back at a 99.7% LC wall). — **pixelbeach**.

### Spillway waterline recipe (composited fluid look)
Water composites as a quarter-step CROSSFADE (alpha 0..3), never additively (speckle mush). Filament presence from long-period cells (4× the brightness cell) that scroll with the fall and never re-roll (re-roll = rain/static); only fine foam re-rolls. Coverage thins with velocity (`cthr = min(240,w) − v/2`); streak decay slow (6–9). Rock impacts fire one-shot at bright→dark ledge lips or every dark region eats the fall. Foam scroll 7–30 px/frame (100+ = temporal aliasing). — **spillway** prototype findings.

## Engine architecture & timing structure

### Vblank serial shift-add engine for per-frame products
One small FSM computes ALL per-frame knob products serially right after vsync instead of N combinational multiply cones (still STA-timed at pixel clock). Fringe: 8 products in ~110 cycles (~350 LC saved); pinboard: 11 ops on one 16×16 sequential multiplier. — **fringe**, **inferno**, **hypnos**, **pinboard**. Pre-fold algebra into the constants so pixel stages are slices/zero-tests; chain dependent products by handshake.

### Vblank-built LUT for narrow-input per-pixel functions
A per-pixel function of a narrow input (6-bit age, hue knob): rebuild a small table each frame in vblank with a plain accumulator — kills a per-pixel multiply AND infers as 1 EBR. — **clacker**, **bajaweave**.

### EBR ring buffer for the dry-video delay + address-side delay
Replace an N-deep 30-bit FF video-delay with a circular EBR buffer + sync-bit shift regs (~450–700 LC). Corollary: delay the ADDRESS (one narrow subtract upstream), not the wide result downstream. — **edgelab**, **inferno** (HW), **mercurial**. Pad the word to power-of-2 width (CLAUDE: BRAM rules).

### Scattered register files are ROUTING structures
Per-frame register-file writes must be bare flop→flop: form the value AND a pre-decoded one-hot write enable one cycle early (add+clamp+decode in the write cycle = 10.6 ns of routing). Inverse: deliver a per-run parameter as ONE registered value chosen when the run opens, not three muxes at use-sites. — **ribbon** (71.5 → 83.1 MHz), **nacre**.

### Half-rate FSM clock-enable
A big control FSM on a clock-enable every other cycle gets 2× the period while display stays full-rate; replace 1-clock pulses with toggle handshakes (a CE'd FSM misses pulses). — **pixelbeach**, HW-validated.

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

## Verification & debugging methods

- **RTL-faithful Python cycle model**: translate engine processes mechanically (read pre-edge, write post-edge) so it emits the identical stream as the die; diff against analytic math in ~2 s. Reach for it immediately on any streaming-engine bug — 5 of nacre's 7 defects were one-cycle read skews a picture can't localise. Taxonomy: sync-RAM prefetch is 2 cycles; ROM reads trail consumers by one; write strobes need explicit '0' defaults; clamp run ends to W not W−1. Always test dragged/asymmetric geometry — symmetry hides wrong-operand bugs. — **nacre** (program dropped; method transfers).
- **GHDL-vs-model pixel diff**: target 0 pixels different; check white fraction first (black-on-black passes trivially); vary only un-glided controls or sweep glide steps 0..8. — **fringe**, **morsel** (memory: sim_model_pixel_diff).
- **Knob sweep harness**: re-implement each sequencer slot in Python at the RTL's REAL widths; assert every knob monotonic and full-range (morsel `check_frame_math.py`). Stills cannot reveal wrap faults.
- **Bypass control frame**: a pixel-clean BYPASS mode isolates full-field noise to one path before diagnosing sub-bugs. — **mercurial**.
- **Continuity proof**: sweep knob values inside a single step of the old quantized design — old renders identical, new must differ monotonically. — **ribbon**.
- **Chroma-gain rubric**: measure sat changes on a source with gamut headroom (a saturated test image clips and reads 112% instead of 125% — looks like an RTL bug, isn't). Acceptance: endpoint percentages, luma within 1 LSB, mean hue shift < 0.4°. — **cascade**.
- **ISA emulator first** for any microcoded engine (seconds vs 20-minute renders) — but it can't see hardware sequencing bugs. — **cubist**.
- **On-screen diagnostics**: when only hardware shows the bug, render blink cells comparing two rates by eye. — **pixelbeach**. Dump raw pixel grids rather than trusting a clever detector; derive test oracles from the source being replaced, not from memory of it. — **bishop**, **cubist**.
- **Netlist before hand-CSE**: yosys may have already merged the shared term — pyro's line-rate pre-composites gained nothing.

## Cross-library notes (2026-07 analysis)

- The HW-validation backlog is the biggest structural gap — many programs are timing-closed but never flashed; keep the status tags honest.
- Standing rework candidates then: tessera (10-triangle rasteriser — since built), laser_cathedral (dead K4, Phase-2 beams unbuilt), pixelbeach (no "done" definition), tiler (flickers at N=1/2), oubliette (loot stubbed), etchplate (intended filament look still unmet — see memory `etchplate_intended_look`).
- Cheap cross-library passes: dead-knob cleanup; continuous-control pass (many knobs decode only top 2–3 bits); back-port fixes between forks (revenant↔glitchscape, mondrian↔quilt↔triangulator).

## Dead-end index

Quick lookup — details in the entries above.

| Dead-end | Program |
|---|---|
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
