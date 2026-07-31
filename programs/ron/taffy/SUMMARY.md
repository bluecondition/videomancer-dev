# Taffy — slider-scanned horizontal stretch bands

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
leave the line — the edge logic only ever fires in Squeeze.

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
for Mode; the edge fill below replaced it outright.

## Edge fill (v0.3)
Squeeze pulls the picture in from the borders, and the reads that reveal used to
clamp onto the source's edge column — which on a capture source is a **black
blanking column**, so the reveal came in as a black bar. (Left only, in
practice: the right border clamped onto real picture. Lagoon documents the same
asymmetry.)

Lagoon's left-fill pattern, applied to both edges as redshift v2.1 does: each
line sums 64 pixels starting past pixel 8 (past the blanking columns, so the
black never enters the average either), latches `sum >> 6` at hsync, and reads
landing within `C_EDGE = 6` columns of either border paint that colour instead.
The latch timing is free — the line that just finished is exactly the line read
back during the next one, so the fill colour always belongs to the line it
fills. The mux sits on the BRAM outputs at `p_wet` (sidewinder's stable-select
precedent). The picture now shrinks into a soft matte of its own colour.

Two Taffy-specific details:
- The fill triggers on reads of columns `< C_EDGE`, **not** merely off-screen
  reads — that's lagoon's actual insight, and it's what kills the black seam
  between the fill and the picture.
- It's gated on `s_fill_en` (per line, `v_eff /= 0`). Without that gate, a line
  at env = 0 would paint its own columns 0–5, breaking the exact-dry null.
  Redshift gates the same way on `dx /= 0`.

No dry/wet mix: env = 0 already reproduces the input exactly (inv = 256,
acc = 0), so the effect has a real null.

## Multiply ledger (5) — all in blanking-time sequencers, none per-pixel
Per frame: `p12d × active_h` (selected line), `K1²` and `K2²` (rate curves).
Per line: `env × Depth` (effect amount), `lo × inv` (DDA seed).

## Timing
All 6 configs close at **seed 1, no retries**: 78.1–93.4 MHz (HD needs 74.25).
3812/7680 LCs (50%), 27/32 EBR (22 line buffer + 5 envelope), full clock
everywhere — no divisor. The edge fill cost ~174 LCs (three 16-bit accumulators)
and no BRAM.

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
how the original hd_hdmi failure hid). **Not yet simulated or HW-tested.**

## Presets
- Scanner, Long Trail, Single Line, Soft Lens, Terraces, Wake (One Shot), Pinch

## Planned
Further horizontal-line effects driven by the same slider + envelope: the
envelope, the band shaping and the DDA are all effect-agnostic, so a new mode
only has to turn `env` into a different per-line remap. Would arrive as an
"Effect" rotary with `value_labels` (see slitscan's K3 Slices for the pattern).
