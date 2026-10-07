# MORSEL

**The whole screen is one chocolate-chip cookie, and the video is baked into
it.**

Forked from the single "choc-chip cookie" swatch inside **Sugarcoat** — which
was one of eight candy materials there, drawn as a 4-way-flipped triangle in
an 8/16/32/64 px cell. Here that one material becomes the entire program, and
the chip is rebuilt from an actual 3-D model.

---

## The chip

`genchips.py` builds a real morsel — flat circular base, sides bulging up from
it, tip curling over to one side — as a solid of revolution with a curled
axis, then **photographs it from 8 different 3-D orientations** and stores each
projected **silhouette** as a polar radius table `R(theta)`, 128 steps.

| variant | elevation | reads as |
|---|---|---|
| 0 | 0° (side-on) | classic morsel: flat bottom, rounded shoulders, hooked tip |
| 1-3 | 9-32° | tipped teardrops, base foreshortening into an ellipse |
| 4-5 | 45-58° | three-quarter view, strong hook |
| 6-7 | 72-87° (top-down) | round chip with a small point off one side |

A solid of revolution projects to the **union of its stacked disks**: a
horizontal disk of radius `r` at height `z` seen from elevation `phi` becomes
an ellipse centred `(cx, z·cos phi − cy·sin phi)` with semi-axes
`(r, r·sin phi)`. `shapes.png` is the contact sheet of all 8.

Why polar rather than a rotation matrix: the pixel path already has to work
out its angle about the chip centre (the rim light needs it), so a
radius-vs-angle table buys an **arbitrary** silhouette — flat bottom, curl,
point — for one BRAM read and one compare. **Rotating a chip is then a
subtract on the angle index**: free, and continuous at 2.8° steps rather than
the 4 flips it was forked from.

Per chip, from the cell hash: free 128-step rotation, ±19% size jitter,
position jitter scaled to whatever slack the cell has left, dark/milk
chocolate, and (S11) white chocolate.

**Lighting** — one fixed upper-left rim light per chip: a dome gradient whose
strength rises toward the rim (where a convex body's normal swings outward),
a tight specular glint on the lit shoulder, a dark contact edge opposite, and
a soft shadow thrown onto the dough beside it. Chips and dough both carry
hash grain, so nothing is a flat fill.

## Baking the video in

Two mechanisms, deliberately at different frequencies:

* **Dough tone** follows the video luma pixel-for-pixel through the Bake ramp
  — fine detail, and always stays a cookie colour at both ends.
* **Chip density and size** follow a **per-cell** luma sample (S7) — the
  coarse halftone that makes a face read from across the room. Dark cells get
  more, and bigger, chips.
* S10 **Emboss** adds relief from the video's own gradient, so the cookie
  surface is sculpted by the picture rather than merely tinted by it.
* S8 **Tint** lets the dough take the video's chroma as well.

## Anti-lattice (the one non-obvious structural decision)

A plain cell grid reads as **rows of dots**, and per-cell jitter can never fix
it: a chip may only wander as far as the slack left inside its own cell, which
is nearly nothing once the chips are large. So every cell **column** is slid
vertically by its own pseudo-random phase. The cell boundaries move with the
chip, so nothing is ever clipped, and the rows disappear entirely.

## Controls

| | |
|---|---|
| K1 | **Scale** — cell 8 / 16 / 32 / 64 / 128 px |
| K2 | **Shape** — which slice of the 8 3-D orientations: side-on teardrops ↔ round chips with a point |
| K3 | **Bake** — pale golden dough → dark baked (v0.2.2 reaches ~25% darker) |
| K4 | **Relief** — shading depth + specular glint |
| K5 | **Grain** — dough surface texture |
| K6 | **Chip Size** |
| P12 | **Chips** — headline: chip density over the whole fader, plain dough (0) → every white cell (100%) |
| S7 | **Luma Drives** — Density: chips where the picture is white, none on black (chance = whiteness × P12) · Size: P12 alone sets which cells get chips, and the picture sets each chip's size (none on black, full on white) |
| S8 | **Tint** — dough takes the video's chroma |
| S9 | **Invert** — flip the luma polarity |
| S10 | **Cracks** — the crust splits along the picture's hard edges (dark crack, lit lip); Relief sets how many and how deep |
| S11 | **White Chips** — white chocolate in the mix |

The emboss (relief from the video gradient) is always on. S10 was labelled
"Emboss" in v0.1 but nothing read it, so it was a dead switch.

## Implementation

Streaming, `C_LATENCY = 22`, no CORDIC and no per-pixel divides.

```
1-5    cell lattice: column -> column hash -> vertical phase -> cell row
6-9    cell hash -> rotation/variant/size/jitter/presence + per-cell luma
10-12  polar: |cx|,|cy| -> C_RTAB -> exact radius + octant fraction -> angle
13-16  C_SHAPE silhouette radius, C_COS rim light, register slice, multiply
17-18  signed distance inside the silhouette; coverage + shading DECISIONS
19-20  one shading add, one dough add, chip and dough colours
21-22  alpha scale, composite
```

`C_RTAB` (1024×12) returns **both** `radius*4` and the octant sub-angle from
one read — no square root and no divider anywhere in the pixel path. Only
three multiplies exist at all (chip size, bake ramp, noise gain); the
composite is a shift-add because alpha only spans 0..4.

Everything with a ladder or a multiply in it — the size table, the jitter
table, the shape window, the bake constants — is built one item per cycle by
the **vblank sequencer**, so the pixel path only ever sees a table read or a
mux.

`preview_morsel.py` is a numpy transcription of the exact integer pipeline; it
renders a full frame in a second and is what the design was tuned against.

### Timing close (HD needs 74.25 MHz)

First build: **57 MHz**. Every fix came from profiling the nextpnr critical
path (`--timing-allow-fail -l log`, read "Critical path report"), never from
guessing — the classic symptom was there, an Fmax that barely moved with big
area changes.

1. **The dough stage** did two grain multiplies and five adds in one cycle.
   The grain now finishes 14 stages earlier (free-running noise needs no
   alignment), the emboss is pre-shifted at insertion rather than where it
   meets the dough tone, and the cast shadow is pre-applied into a second
   offset so the final stage picks one and adds once. **57 → 66 MHz.**
2. **The vblank sequencer** then became the critical path — a reminder that
   per-frame logic is timed at the pixel clock like everything else (same trap
   sugarcoat hit). Its chains were split across slots, the jitter-room
   calculation `(126 − 1.73·szb)/4` became the `C_JMAX` table, every
   non-constant multiply was removed, and the slot ranges were **aligned to
   power-of-two boundaries** so a table index is just the low bits of `s_seq`
   — no subtract, and the range decode is bit-matching. **66 → 73 → 93 MHz**
   pre-route.
3. Barrel-shifting the grain coordinates straight into the hash put
   `s_scl → shift → hash` in one cone; the shift is now registered.

All 6 configs close (71% LC, 11/32 BRAM, no clock divisor):

| config | Fmax | required | seed |
|---|---|---|---|
| HD Analog | 86.46 MHz | 74.25 | 1 |
| SD Analog | 74.23 MHz | 27 | 2 |
| HD HDMI | 87.37 MHz | 74.25 | 1 |
| SD HDMI | 79.67 MHz | 27 | 1 |
| HD Dual | 82.10 MHz | 74.25 | 1 |
| SD Dual | 85.27 MHz | 27 | 2 |

Which seed stalls router2 moves around between edits (it was HD Analog before,
the two SD configs here) — always ~360 s on the retry. Nothing to chase; the
build script sweeps seeds automatically.

`C_SHAPE`'s BRAM clk-to-q is 2.15 ns; feeding it straight into the size
multiply left only 0.8 MHz of margin, so the ROM output has its own register
slice (this is why `C_LATENCY` is 22).

### Why chips were flashing, and why they must not resize either

Chip presence is a compare: `cell_hash_byte < threshold`. The hash byte is
fixed per cell forever, but the threshold moved by an LSB every field from
three independent sources, so any cell sitting on the line toggled between
"full chip" and "nothing" at frame rate:

1. **The P12 fader's ADC dithers**, and `s_dens` is re-latched every field.
   With 256 possible hash values, ~1 cell in 256 sits on the boundary — a
   handful of chips on screen, flickering.
2. **The per-cell luma was a single pixel** (cell-centre column, cell-row top
   line), carrying the source's full noise straight into the compare.
3. **Interlace.** `s_lcnt` resets every field, so both fields build the same
   cell grid — but they sample *different physical lines* for that luma. The
   threshold then alternated at 30 Hz, the loudest version of the fault.

The first attempt made presence a *margin* and let marginal chips grow in from
a speck. That removed the flashing but replaced it with something just as
wrong: **chips changing size on their own.** A chip's size must be a fixed
property of its cell — nothing that moves may touch it.

So the compare is binary again, chip size comes from the cell hash and K6 and
nothing else (the old luma-to-size term is gone too), and every source of
threshold wobble is dealt with **at the source**:

* **`s_dens` has a deadband** — fader dither alone cannot move the threshold.
* **The luma sample averages the whole cell width** (8-128 px depending on
  Scale) instead of one pixel, dropping sample noise 3-11x. Note that
  *quantising* it instead would have made things worse, not better: coarse
  steps amplify the dither for any cell near a step boundary.
* **The cell buffer refreshes on one field only** when interlace is detected.
  The other field then reads exactly the values the first one stored, so both
  fields of a frame render identically and the 30 Hz alternation is gone. This
  costs one AND gate — the ping-pong banking already makes it work out.

What remains is that chips still appear and disappear when the *picture*
genuinely changes, which is the point of Luma Mod. **S7 off freezes the chip
layout completely** and leaves the video showing through the dough tone and
emboss only.

The general lesson: a hard binary decision on a live analog-derived quantity
has to be stabilised on its INPUT. Softening the decision instead just moves
the artifact somewhere the eye still catches it.

### shift_left width trap (three dead/cycling knobs)

Optimising the vblank sequencer for timing, three constant multiplies were
rewritten as shift-adds:

```vhdl
v_m20 := resize(shift_left(s_k6, 3) - s_k6, 20);   -- WRONG: k6 * 7
```

`shift_left` on an `unsigned` returns **the operand's own width** — the bits
shifted off the top are gone — and `"+"`/`"-"` return `max(L,R)`, so the whole
expression silently became mod 1024. The `resize` at the end was far too late.
The original `s_k6 * 56` was correct only because `"*"(unsigned, natural)`
widens to `2*L`.

Symptoms, all three from the same line pattern:

| control | was | should be |
|---|---|---|
| K6 Chip Size | ramps 20→27 and **wraps 4×** across the knob | 20→75 monotonic |
| K2 Shape | pinned at 0 — **a completely dead control** | 0→4 |
| K4 specular | cycles 0→56 repeatedly | 0→319 |

Fix: `resize` to the target width **before** shifting.

The reason this shipped is worth noting: `preview_morsel.py` is bit-exact for
the *pixel* path but computes per-frame constants in clean Python ints, and a
still render cannot reveal the fault anyway — every individual frame looks
correct, only *sweeping the knob* shows it. `check_frame_math.py` now
re-implements each sequencer slot at the VHDL's real widths and asserts each
knob is monotonic and spans its full range. Run it after touching `p_frame`.

### Hash trap (worth remembering)

The first `f_pack` was `(x & y_low) xor (y & x_low)` — which, worked through,
**depends only on `x xor y`**. Every cell on a diagonal shared a hash
(measured diagonal correlation 0.29, visible as cross-hatching in the dough),
and the column phase — whose second argument was `x xor const` — collapsed to
a **single constant for every column**, silently restoring the exact row
lattice the phase was there to destroy. Rotating one half before the xor
fixes both (diagonal correlation −0.03).

## v0.2 — finalization pass (2026-10-04)

**Hardware contract fixes**, all things v0.1 predates:

* **Blanking gate.** The output was never gated, so the blanking interval
  carried dough colour (U/V ≠ 512). It's now forced to Y 64 / U V 512 whenever
  the latency-aligned avid is low.
* **Interlace detection vs serrated vsync.** `s_ilace` was judged at every
  vsync falling edge. Analog vsync has several per field, so the second edge
  compared the field with itself and cleared the flag. That quietly disabled
  the one-field cell refresh (the 30 Hz flashing fix) on interlaced analog SD.
  It's now judged at the first active pixel of each field.
* **Emboss line buffer read/write collision.** Same-address read and write in
  one cycle is undefined on iCE40 EBR. The write now trails the read by one
  pixel.
* **P12 deadband wrap.** `s_dens + 1` was 8 bits and wrapped at the top of the
  fader, so 254/255 dither re-triggered the threshold every field. The
  deadband now runs on the 10-bit fader, compared one bit wider.

**Look changes** (v0.2.2 state, 2026-10-06):

* **P12 = chip density over the whole fader**: plain dough at 0, every white
  cell at 100%.
* **S7 Density: chips where the picture is WHITE, none on black** (Ron,
  2026-10-06). v0.1 and v0.2.0 put chips on the dark cells. A cell's chance of a
  chip is now `whiteness × P12`. Whiteness is the held cell luma measured from
  video black, `(L − 16) × (1 + 1/8 + 1/32)` clamped to 0..255, so black never
  gets a chip at any P12. The threshold is one 8×8 multiply on stage 10, with
  the compare `hash < thr` on stage 11. S9 flips the polarity.
* **S7 Size** (v0.2.1, Ron's spec): P12 alone picks the cells, and the same
  whiteness scales each chip's SIZE: no chip on black, a full-size (K6) chip on
  white, a classic size halftone. A 7×8 multiply on its own stage, then a mode
  mux; a zero-size chip is absent. This is NOT the size wobble rejected in
  August: the input is the held whole-cell average (one-field refresh), so a
  still picture never moves a chip. Chip colour is the hash dark/milk in both
  modes.
* **S10 Cracks.** `|gx| + |gy|` above a threshold (52 → 21 across K4) replaces
  the emboss value with a dark crack (−140/−210/−280 by Relief). The pixel
  after a crack gets a lit lip. This is folded into the emboss stream, so it
  costs no delay line and no extra dough add.
* **K3 Bake darker** (v0.2.2): the dough base is `480 − (k3/4 + k3/16)`, so
  it ends at 162 instead of 225.

**Tried and removed:**

* **v0.2.0 drop-lattice climax** (HW-rejected 2026-10-06, "I don't like the
  circle chips"). A second half-cell-offset lattice of round chocolate drops
  behind the morsels filled in over the top quarter of P12. Fit lesson worth
  keeping (TECHNIQUES): its first cut cost +1,680 LC, almost all FLIP-FLOPS
  from a 19-stage parallel pipeline. A single chocolate layer before one blend,
  a chroma flag, and late cell-buffer reads got it to 89%; removing it
  entirely puts v0.2.2 at 74%.
* **v0.2.0 S7 Tone** (chip shade from luma), replaced by Size in v0.2.1.

Kept from that work: cell-luma buffers read at the stage where the cell row is
known (no 4-deep delay chains; collision-free because a row reads the bank it
doesn't write), chocolate chroma as one white/dark flag, and a 3-step chroma
blend. Latency is 23 (stage 20 is an alignment slot), the same in every mode.

v0.2.2 build: see Status. Presets: P12 values were reset for the new curve
(Fresh Batch 820, Fine Halftone 900, Loaded 1023, Giant Morsels 820,
Dark & White 880, Cracked Crust 700, Size Halftone 1000). Capture real ones on
hardware. `preview_morsel.py` does NOT model the v0.2 changes;
`check_frame_math.py` covers P12, Bake and the crack slots.

## Colour convention

Authored in **standard BT.601** (U = Cb, V = Cr) and swapped at both ends,
because the Videomancer carries chroma U↔V swapped — same as sugarcoat and
gilt. Headless BT.601 sims therefore show the cookie blue; the Python preview
renders the intended colours.

## Status

v0.2.2 (2026-10-06): all 6 configs closed on **seed 1**, 74% LC, 11/32 EBR,
no clock divisor, `.vmprog` packaged. HD Analog 81.50, HD HDMI 78.87,
HD Dual 85.31 MHz (74.25 required); SD Analog 79.78, SD HDMI 81.08,
SD Dual 75.04 MHz (27 required). **Not yet flashed** (v0.2.1 was seen on
HW, which is where the drops were rejected). Still to check on hardware: no
flashing on stills and moving video at several Scales, with an interlaced SD
source, and at P12 100%; Density now chips the WHITE areas; S7 Size; S10
Cracks (threshold and depth may want tuning); the darker Bake; a clean picture
edge (blanking gate). Then capture presets from the hardware.
