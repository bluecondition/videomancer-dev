# Taffy — slider-scanned horizontal stretch bands

**v0.4.0 (2026-09-28): FINAL — signed off by Ron on hardware ("I like the way this looks").**

## What it does
P12 scans a scanline down the picture (100% = top, 0% = bottom). The lines it
touches stretch horizontally outward from the centre column, and snap back
behind it as the slider moves on. Drag the slider slowly and you drag a bulge
through the image; drag it fast and you leave a trail of half-relaxed lines
melting back to normal. The relaxation is what makes it a performance control
rather than a static warp — the picture keeps moving after you stop.

A dead zone of ±Centre px either side of the centre column is left untouched:
widen it and the stretch starts further out, so the middle of the frame stays
readable while the flanks smear.

**S11 picks how a line answers the band.** In *Sustain* the envelope chases the
band: full while the slider is on the line, receding once it leaves. That means
a quick pass either snaps the line to full (fast Pull) or never gets it there
(slow Pull) — the stretch is rarely something you watch happen. In *One Shot*
the band triggers the line and lets go: however briefly the slider touched it,
the line grows to full and recedes on its own, so a sweep leaves a wake of
complete stretches behind it. It re-arms only once the band has left, so parking
the slider fires each line once rather than throbbing.

## How it works
Processing program, forked from **sidewinder** (dual-bank line buffer, run-along
read pointer, blanking-time step sequencers). Incoming video is written to its
own parity bank and read back one line later at a remapped address.

**Per-line envelope.** A 2048×10 BRAM holds `{state[9:8], level[7:0]}` per
scanline. Once per line during horizontal blanking the sequencer reads it,
advances it, and writes it back. Pull and Relax are the attack/release rates, so
the trail behind the slider is just lines still releasing.

In Sustain the level chases a target (255 inside the band, 0 outside, feathered
by Soft). In One Shot the 2-bit state runs armed → attack → decay → spent, with
the band's target only *triggering* the transition out of armed; spent re-arms
when the target returns to 0. Sustain still maintains the state field (armed
when the band is off the line, spent when it is on it), so flipping S11
mid-performance neither fires every line at once nor strands one mid-pulse, and
armed walks a stretched line down gracefully rather than snapping it to 0.

**The stretch is a DDA, not a per-pixel multiply.** Output pixel x samples
source distance `Centre + (|x−c| − Centre)·inv/256`, where `inv < 256`
magnifies. Rather than evaluate that per pixel, the accumulator is seeded once
per line to `src(0)` and then adds either 256 (inside the dead zone, 1:1) or
`inv` (outside it) per pixel. Zero multiplies on the pixel path; the dead-zone
test is registered one pixel ahead so the per-pixel path is mux → add, never
compare → mux → add.

In Stretch the source always lands between x and the centre, so reads can never
leave the line — only Squeeze runs off the ends, into the edge clamp below.

## Controls
- **K1 Pull** — how fast a line grows to full stretch (quadratic: 1–255 env
  steps/frame, so the low end crawls for seconds).
- **K2 Relax** — how fast it recedes again (same curve).
- **K3 Height** — band thickness, 1–255 lines.
- **K4 Centre** — half-width of the untouched centre dead zone, 0–511 px.
- **K5 Depth** — magnification at full envelope, up to 64×.
- **K6 Soft** — feathers the band edges over 1–128 lines. In Sustain that's a
  lens-like bulge instead of a rectangular slab; in One Shot the feather is the
  trigger's reach, so lines fire before the band proper arrives and the wake
  runs ahead of the slider.
- **S7 Polarity** — Stretch (outward) / Squeeze (inward).
- **S8 Hold** — freeze every envelope: paint a shape with the slider, latch it,
  let go. Gates the BRAM write only, so it also freezes a One Shot mid-pulse.
- **S9 Steps** — quantize the envelope to 4 levels (top 2 bits replicated →
  0/85/170/255) → terraced bands.
- **S10 Bypass**.
- **S11 Mode** — Sustain / One Shot (see above).
- **P12 Position** — the scanned line.

The Black-fill option that lived on S10 in v0.1 was dropped to free the switch
for Mode; the edge clamp below replaced it outright.

## Edge clamp (v0.4)
Squeeze pulls the picture in from the borders, and reads that run off the line
used to land on the source's edge column — a **black blanking column** on
capture sources, so the reveal came in as a black bar.

v0.3 painted a 64-px line-average matte there (lagoon/redshift pattern, 3×16-bit
accumulators + a fill mux). v0.4 replaces it with the catalogue-combo **address
clamp** the user preferred on hardware (2026-09-13: the average left a black
seam between picture and fill; wanted "one colour = the colour leading up to
it"): every read `< C_EDGE` or `> width−1−C_EDGE` (C_EDGE = 12) is redirected to
that inset column, so the reveal is the line's own edge colour drawn outward and
the blanking columns never reach the screen. The clamp is ungated (as in the
catalogue programs), so every line — dry or displaced — shows the same edges
and a band never exposes a ragged black border against its neighbours.

No dry/wet mix: env = 0 reproduces the input exactly (inv = 256, pixel x reads
x) apart from those 12 clamp columns each side.

## v0.4 review fixes (2026-09-28)
- **Blanking gate.** The line buffer is read every clock, so the wet output
  carried picture (edge pixels) through h/v blanking where the encoder takes its
  colour reference — the blanking-interval contract that no sim checks. `p_wet`
  now forces Y/U/V to 64/512/512 on the latency-aligned avid (tap
  `C_TOTAL_LATENCY−2`, one stage before `data_out.avid`).
- **Pixel alignment.** Pixel k was written at address k+1 (the write used the
  already-incremented counter), and the identity read happened to cancel it for
  Y — but the 2:1 chroma address `(k+1)>>1` split every chroma pair, giving odd
  pixels their right neighbour's chroma. Writes now use `s_wr_x` (the pixel's
  own index) and the DDA seed carries a −1 (pre-computed as `s_lo_m1` in the
  frame sequencer so the seed stage stays one subtract).
- **One Shot flip saturation.** A Sustain line flipped to One Shot while still
  part-stretched sits ARMED with env > 0; when the band returned, ARMED→ATTACK
  took `env + Pull` unsaturated and could wrap (200 + 100 → 44). Now clamps at
  255.

## Multiply ledger (5) — all in blanking-time sequencers, none per-pixel
Per frame: `p12d × active_h` (selected line), `K1²` and `K2²` (rate curves).
Per line: `env × Depth` (effect amount), `lo × inv` (DDA seed).

## Timing
v0.4: all 6 configs close at **seed 1, no retries** — HD Analog 78.7, HD HDMI
78.3, HD Dual 76.5 MHz (HD needs 74.25); SD 75.5–82.4. 3672–3691/7680 LCs (48%,
−130 vs v0.3's average fill), 27/32 EBR (22 line buffer + 5 envelope), full
clock everywhere — no divisor. (v0.3 was 3812 LCs, 78.1–93.4 MHz.)

## Timing lessons
1. **The read-address range test was the whole problem.** Leaving `p_rdaddr`
   combinational ran one cone from `s_acc` through a carry-chain comparator
   straight into `RAM.RADDR`: 15.2 ns, **12 of it routing**, pinning hd_hdmi at
   65–70 MHz. It passed on exactly one lucky seed out of six. Registering the
   address split it into `acc → compare → reg` and `reg → RAM`, and every seed
   now passes at 80+. One clock of latency, absorbed by the dry shift register.
2. **Narrow the accumulator.** Every bit of `s_acc` is a carry cell in that
   comparator. Sized to the true worst case (Squeeze at full Depth: −1.05M to
   +1.57M → signed 22, int part 14 bits) rather than a round 24.
3. **Re-register SPI bits that land in a pixel-rate cone.** `s_clamp` came off
   long global routing directly into the comparator LUTs; a local copy keeps it
   out of the critical cone.
4. Adding a pipeline stage *helped* here because util is 50% — the opposite of
   the 90%+ regime where extra registers make routing worse.
5. The v0.3 edge-fill bounds went into `p_rdaddr` — the very cone that was the
   original critical path — without costing anything, because both compares are
   against **pre-registered** values (`C_EDGE` is a constant, `s_w_edge` is
   computed in the vblank sequencer) and sit parallel to the existing test
   rather than extending it. `width − C_EDGE` inline would have re-broken it.

## Verification
Built and timing-verified on all 6 configs (routed Fmax read from nextpnr's
Warning/Info line, not the pre-route estimate — the build script's
"✓ Completed" on a last-seed retry is a best-effort **miss**, which is exactly
how the original hd_hdmi failure hid). v0.4 **HW-validated** 2026-09-28 and signed off as final (not simulated).

## Presets
- Scanner, Long Trail, Single Line, Soft Lens, Terraces, Wake (One Shot), Pinch

## Planned
Further horizontal-line effects driven by the same slider + envelope: the
envelope, the band shaping and the DDA are all effect-agnostic, so a new mode
only has to turn `env` into a different per-line remap. Would arrive as an
"Effect" rotary with `value_labels` (see slitscan's K3 Slices for the pattern).
