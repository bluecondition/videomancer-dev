# Mercurial

**Luma as material.** The incoming picture's luma is treated as the height
field of a deformable metal surface; the image is re-rendered as light
striking that surface. Three subsystems, built as three milestones:

1. **Chrome Engine** (v0.1) -- stateless: Sobel surface normals -> procedural
   sky/ground environment + specular + thin-film iridescence.
2. **Strata Replacer** (v0.2) -- 4 soft-keyed luma bands, each replaced by a
   synthesized material (interference / brushed metal / chrome / dry video),
   thresholds drifting slowly.
3. **Rubber Engine** (v0.3) -- coarse per-region damped springs chase the
   luma; the ringing *residual* is added back to full-res luma before the
   gradient window, so motion sends a physical shudder through the surface.

## Status

- **v0.3 COMPLETE (all three engines) -- timing CLOSED (routed) on all 6
  configs, every config exiting the seed sweep early (= genuine routed
  pass, not a last-seed accept): HD Analog seed 5, HD HDMI seed 2, HD Dual
  seed 3, SD all seed 1. ~7140 LC (93%), 29/32 EBR.** Built + packaged
  (out/rev_b/ron/mercurial.vmprog); NOT yet hardware-tested.
- v0.1 sim checks passed: iridescence renders a full direction-hue wheel on
  a radial ramp; env vertical sense correct (sky-lit top, dark ground);
  shading continuous (no posterization); flat fields neutral; specular
  blowout tracks the Light knob (az 0 -> left equator, 180 -> right).
- v0.2 sim checks passed: staircase renders interference / brushed /
  chrome / dry per band with soft melts; drift moves the band boundaries
  across warmup counts; portrait composes all four materials.
- v0.3 sim checks passed: springs converge (testcard rings at w10, settles
  by w40 at moderate Elasticity, retains the dither breathing); rubber
  bypass frame is pixel-clean; 1080i weaves correctly (field-agnostic
  grid, no combing); sync ALIGNMENT PASS (tests/vhdl harness).

## Architecture notes (v0.1)

- Single streaming pipeline, `C_LATENCY = 18`, fixed across all milestones
  (S1..S3 are reserved stub delays for the v0.3 rubber residual).
- **Height field = FULL 10-BIT luma**, horizontally [1 2 1]-blurred on the
  line-buffer write path (2 history regs, no extra stage). 8-bit height
  rendered quantization contours as dotted rings once normalized -- 10-bit
  + h-blur turns them into organic "beaten metal" ripple. 3x3 Sobel via 2
  chained 2048x10 line buffers (phosphor structure; 5 EBR each).
- **Block-float gradient normalization** (the load-bearing trick): one
  shared shift (x1..x16 from a priority encode of |gx|+|gy|) scales BOTH
  components -- direction exact (iridescence hue), magnitude log-compressed
  so gentle slopes shade like edges. Floor at mag 24 (10-bit units) keeps
  noise-level gradients flat. Without this the image is just a luma remap:
  real slopes (~1-3 luma/px) never reach shading thresholds.
- Environment: CONTINUOUS Luster-steepened ramp on the vertical tilt
  (`128 - gys >>> lus`, normal up = bright sky / down = dark ground) + a
  horizon glint band gated to NON-FLAT sideways-tilting surfaces (an
  ungated glint makes every flat field glow white) + drifting sine sheen.
  The original 5-zone ladder posterized; the ramp never does.
- Specular: `align8 = clamp(K - |gx-lx| - |gy-ly|)`, `spec = align8^2 >> 8`;
  Polish opens K, top zone switches to a hard-knee sharpened highlight
  (shift-only). Light vector at FULL qsin length (+/-511) to match the
  normalized gradient magnitudes (0.625x scaling left specular unreachable).
  Highlights desaturate by shift-select on the spec top bits.
- Light vector: shared 64-entry quarter-wave sine (fireworks `f_qsin`, zero
  EBR -- the SDK full LUT infers as 6 EBR); the vblank light sequencer
  borrows the per-pixel sheen tap's angle port (never both active at once).
- Iridescence: U/V offsets ARE the (rotated, film-scaled) normalized
  gradient, gated by magnitude>>1 -- the UV plane is an angle space, so hue
  = slope direction with no atan2 and no hue LUT. Film knob = thickness
  (shift 2..5) + 45-degree hue rotation (shift-add octant rotate).
- Mercury (P12): master dry->material morph; dry video via a 32x30 EBR
  delay ring (edgelab pattern), taps at delay 14 (S14 diff) and 16 (S16).
- EBR: 2x 2048x10 line buffers (10) + dry ring (2) = **12 / 32**.

## Timing battle log (HD @ 74.25 -- each step found via routed critical path)

1. v0.1 first build ~57 MHz: (mat - dry) * p12 in one stage -- subtract
   feeding an 11x11 multiply. Split diff from multiply -> 72.8-82.7.
2. v0.1 routed check: displayed "Completed" Fmax on the LAST retry seed is
   the PRE-ROUTE estimate; the routed number (second "Max frequency" line)
   was 69-72 = miss. The single-stage block-float normalize (mag priority
   encode -> barrel shift -> clamp) was the path; split into S9 (registered
   shift select) + S10 (barrel shift) in the v0.2 restage.
3. v0.2 first build 50 MHz: sa11 -> shared qsin ROM -> vsync-latched s_lx /
   threshold captures. Those paths are STA-timed at full clock even though
   captures only fire in vblank (ziffern lesson, new clothing). Registered
   the ROM output -> 80+ pre-route, but routed still 69.9: the path had
   just moved to seq-mux -> ROM -> s_qs_r. Registered the ANGLE as well
   (mux and ROM in separate cycles, sequencer captures two-behind, sheen
   2px late -- invisible) -> all 6 configs pass ROUTED mid-seed-sweep.
4. S17 flatten: strata sum needs NO clamp (f8 <= 248 keeps A+(B-A)*f8>>8
   inside [A,B]), so Mercury diff = (A - dry) + bd with (A - dry)
   pre-registered in parallel with the blend multiplies.
5. Standing structure: every multiply alone in its stage; material lanes
   fully assembled one stage before the A/B mux; highlight knee / base mix
   / desat one stage ahead of assembly; rotation split from film
   shift+clamp; sync/dry-ring taps re-derived per restage.

## Controls

| Ctrl | Name | Function |
|---|---|---|
| K1 | Light | light azimuth (orbit speed when S11 on) |
| K2 | Polish | specular window; top zone = hard-knee tight blowout |
| K3 | Luster | environment contrast |
| K4 | Film | iridescence thickness (bits 6:5) + hue rotation (bits 9:7) |
| K5 | Drift | (v0.2) strata threshold animation |
| K6 | Elasticity | (v0.3) spring macro + wobble depth |
| P12 | Mercury | master dry <-> material morph (headline) |
| S7 | Stiffness | (v0.3) bright-stiff / dark-stiff |
| S8 | Iridesce | thin-film chroma on/off (off = cool-steel tint) |
| S9 | Strata | (v0.2) bands on / all-chrome |
| S10 | Rubber | (v0.3) live / bypass |
| S11 | Orbit | auto light rotation (keeps static input alive) |

## Rubber Engine (v0.3)

- Coarse spring grid stores wobble dynamics; the RESIDUAL (pos - target)
  is bilinearly upsampled and added to the height field BEFORE the
  gradient window, so full-res detail rides the wobbling sheet and the
  chrome normals shudder with motion.
- Geometry from the MEASURED line width (spiroscope pattern -- also keeps
  decimated GHDL sims correct): 1080i 32x16-field-line cells (60x34),
  720p 32x32 (40x23), SD 16x8 (45x31/36); stride 60/60/48, row_base
  incremental, max addr 2039 < 2048. Field-agnostic: both interlaced
  fields drive the same cells at field rate.
- Memories: state 2048x20 {pos u10 8.2, vel s10} = 10 EBR (updater-only
  R/W); residual 2048x8 = 4 EBR (W updater / R display, port-exclusive by
  construction -- no arbiter); per-column target IIR 256x16 = 1 EBR
  (`vacc <- vacc + ((avg<<4) - vacc)>>2`, immune to partial bottom rows).
- Updater: one cell row per hblank (armed at the hsync closing a row's
  last active line; final partial row fires at vblank start), 7-deep
  pipeline, 1 cell/clk (worst 67 clk vs 138 min hblank). All shifts:
  vel' = clamp(vel + (T-pos)>>>ksh - vel>>>dsh +/- LFSR dither),
  ksh = 6 - T-bucket (bright stiff, S7 inverts), dsh = 2 + Elasticity
  zone, pos' = clamp(pos + vel'>>1). Runs even in Bypass (instant
  re-engage).
- Display: rolling 2-corner prefetch mid-span + hblank prefetch of cols
  0..1; bilinear via 3 small mults in the reserved S1..S3 stages;
  Elasticity depth = shift (floored at >>1: full-depth rails the field).

### v0.3 debug log (sim-diagnosed)

1. Full-field structured noise: (a) the span-start corner shift also fired
   on column 0, clobbering the hblank-prefetched corners with stale data
   every line; (b) the updater's vacc address used live u_cx while the
   state address was registered -- springs chased their right neighbor's
   target; (c) S2 consumed c00/c01 directly while the corner registers
   shift at span boundaries -- the lerp base and slope disagreed for 2 px
   at every boundary. All three fixed; a rubber-BYPASS control frame
   (pixel-clean) isolated the fault to the residual path.
2. The remaining "noise" at max Elasticity was CORRECT behavior: full-depth
   wobble of a cold-started, lightly-damped spring field rails every cell;
   converges cleanly at moderate settings. Depth now floors at >>1.
3. VERTICAL-NEAREST residual upsampling is OFF the fallback menu
   permanently: the residual feeds a gradient engine with block-float
   normalization, so any vertical step renders as a hard horizontal line
   every cell row. The 5-bit vertical lerp (steps ~1 luma/line, below the
   mag-24 floor) is load-bearing.

### v0.3 LC squeeze (7746/7680 placement fail -> ~7140, 93%)

Cut, in order applied: 45-degree iridescence half-rotation (quadrant hue
rotation stays); Mercury multiplies to p12 top 8 bits; blend multiplies to
f8 top 7 bits; Polish sharp-knee; highlight desat; sheen (orbit + drift +
spring dither still keep static input breathing); brushed x-lerp
(unfiltered 32x1 hash streaks still read as brushed); dry-tap delay
registers traded for a second 32x30 EBR ring copy (+2 EBR, -90 FF).
EBR headroom remaining: 3.

## Verification

- Sim note: lzx-vhdl-cli renders standard BT.601 (U=Cb, V=Cr); the steel
  tint / iridescent hue family may swap on hardware (mirror of the hue
  wheel, aesthetically equivalent) -- HW check item.
- HW check items: iridescent hue family (above); spring feel across the
  Elasticity range on live camera motion; brushed streak texture on
  native 10-bit sources (sims were 8-bit-quantized).
