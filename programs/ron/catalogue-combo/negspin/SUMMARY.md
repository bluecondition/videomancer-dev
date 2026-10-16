# Negspin — summary

Catalogue combo #76 (seed 20260922): hue rotation / saturation + invert /
negative / bit-plane isolation. v0.1.x was flashed and liked ("negspin is
good", 2026-09-12). There were no later hardware notes. v0.2.0 (2026-10-12)
is the first finalizing pass, built from a review of v0.1.1 while Ron was
away. It is **not flashed yet**.
v0.1.1 sources: git 3fa438d, and `.negspin_work/v011/`. Its vmprog is
`.negspin_work/negspin_v011.vmprog`.

Build: `./build_combo.sh negspin` → `out/rev_b/ron/catalogue-combo/negspin.vmprog`.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Hue | base hue rotation, 0–360°. 0 = unrotated (it was −90° in v0.1) |
| K2 Sat | saturation 0–400%, 100% = unity |
| K3 Spread | extra hue rotation per luma band (8 bands), up to ~315° on the brightest band |
| K4 Plane | Negative / Bit 8 … Bit 2 (labelled). The plane picture: the negative, or one bit-plane as black/white |
| K5 Mix | plane picture mixed over the luma, 0–100% (100% is now the plane alone) |
| K6 Chroma | chroma bit-planes: Off / 3 / 5 / 9 / 17 / 33 / 65 / 129 levels per channel (labelled, default 9) |
| S7 Luma | Normal / Negative (64 ↔ 940) |
| S8 Chroma | Normal / Negative (hue + 180°). S7 + S8 together = a full colour negative |
| S9 Flip | the plane bit flips the hue 180°. At Plane = Negative it uses the source's top bit (the bright half, or strongly saturated areas) |
| S10 Source | what the plane bits and the Negative picture come from: Luma / Sat |
| S11 Order | chroma bit-planes Before the rotation (the colour lattice spins with the hue) / After (a fixed palette the spinning colours snap between) |
| P12 Spin | hue spin speed, bipolar, squared. Centre (±8 codes) = still; the ends ≈ 0.55 s a turn |

## v0.2.0 changes (batch 1 of the review)

Bugs:
- **Hue 0° was a −90° rotation.** The sine ROM holds −cos, and v0.1 read
  sin at the angle and cos a quarter turn on. That is one quarter turn
  early, so at Hue 0 every colour was turned −90°: (U, V) → (V, −U). The
  reads are now sin = ROM(a + 64) and cos = ROM(a + 128), checked
  numerically: Hue 0 is now the identity, and v0.1's Hue-0 colours are now
  at Hue 270°. Gouache and hueflutter use the same ROM read (see Notes).
- **Luma Negative sent blacks superwhite.** It was 1023 − y, so black (64)
  became 959, above legal white. Full-scale Y clips RGB and kills chroma on
  hardware. The negative is now a mirror about the video range, 1004 − y on
  luma clamped to 64..940, so 64 ↔ 940. K4's Negative picture uses the same
  mirror.
- **K6 chroma bit-planes made greys flicker into full colour.** Masking low
  bits off signed chroma floors toward −∞, so a grey pixel with −1 code of
  noise jumped a whole step negative. At 1 plane, noisy greys flashed
  between neutral and U or V = −512 (fully saturated), and there was no +512
  level. The planes now round to the nearest level (+½ step, then mask), in
  both Order positions. Greys stay grey, and the levels are symmetric
  (2^k + 1 per channel, hence the "3 … 129 Levels" labels).
- **Sat-source planes were mostly empty.** |U| + |V| of real video rarely
  passes ~250, so Bit 8 from Sat was nearly all black. The saturation source
  is now ×2 (clamped).
- **Mix topped out at 255/256**, so a faint copy of the original stayed at
  100%. It now spans 0..256.
- **Spin crept at centre.** There was no deadband: one code of pot noise
  turned the hue one step every ~8 s.

Hardware contracts:
- HD HDMI bar fix, built in from the start. Bit-planes at high Mix are
  1-px, full-swing 64↔940 edges that move with the video, which is the
  known trigger. The fix is a highlight knee, 600 + (y − 600)/2 (white
  ~770), then [1 2 1] on Y/U/V just before the blanking gate. The knee is
  dropout's variant, not damier's linear scale: it leaves everything below
  600 untouched, so a plain colour-processed picture isn't dimmed, and it
  gives the same white (~770) for the hard plane edges.
- Latency 18 → 22, the same in all modes.

Controls:
- **K4 and K6 are labelled** (the shelf audit's to-do). K4 uses outliner's
  labels. Their defaults (Bit 7, 9 Levels) are not label 0, so the control
  latch reads raw values 0..7 as the label index. That is the edgelab fix
  for the firmware writing label indexes at program load.
- **S9 Flip and S10 Source work at Plane = Negative.** In v0.1 both did
  nothing there. Now Flip keys on the source's top bit (bit 9, the one
  plane K4 doesn't list). Source picks the Negative picture: the luma
  negative, or an inverted saturation map as luma (greys white, colours
  dark).
- **K6 defaults to 9 Levels**, so S11 Order shows at power-on (the shelf
  audit's to-do). The presets pin K6 at Off, as in v0.1.
- **P12 Spin is squared**, with a centre deadband. It uses the same curve as
  gouache v0.2.0. The ends are ~8× faster than v0.1's top (one turn in
  ~0.55 s instead of ~4.3 s), and the old top speed sits at ~37% from centre.
  Preset P12 values were re-solved for the same speeds: Plane Flip
  640 → 611, Crushed Chroma 400 → 418.
- Categories → Color / Glitch. "Colorizer" and "Stylize" are not in the
  schema's list.

Timing / fit:
- The four 11×10 rotation products are split hi/lo on the sine operand
  (eight 11×5 / 11×6 products, then a recombine stage), as in gouache.
  v0.1.1's HD HDMI had stalled seed 1 and missed seeds 2–3 before passing
  on seed 4.
- Hue + phase is pre-summed per clock in the control process, so the pixel
  angle stage is one add plus a bit flip for Flip.

## Review backlog

### Tier 2: needs Ron's call (these change the look)
- **A. S11 Order still does nothing at K6 = Off.** Off has no planes to put
  in order. Options:
  - leave it (the default now shows it);
  - at Off, Order picks whether the Spread bands key on the plane-mixed
    luma (Before) or on the original (After);
  - give S11 a new job, e.g. Spread Steps / Smooth (see C).
- **B. S10 Source also keys the Spread bands.** Hue bands by saturation
  instead of luma: greys stay put and colours fan out. This makes S10 do
  something even at Mix 0 with Flip off.
- **C. Smooth Spread.** The 8 hard luma bands give contoured hue steps. A
  continuous option would give a hue gradient over brightness.
- **D. S8 Chroma Negative is exactly Hue + 180°.** It's useful as an instant
  flip, and it pairs with S7 for a true negative. An alternative is Mirror
  (U ↔ V), which reverses the hue wheel; K1 can't reach that.
- **E. Sat knob.** Unity sits at 25% of travel. A proc-amp curve with unity
  at centre would be symmetric. The presets would need translating.
- **F. Possible thin hue lines on HD HDMI.** If the source avid's
  hsync parity wanders, the encoders mis-pair U/V on that line. For a hue
  rotation the result is the opposite rotation on that line
  (swap · R(θ) · swap = R(−θ)): thin lines of the wrong hue over coloured
  areas, at any Hue other than 0° / 180°. Nobody has reported this on v0.1.
  If it shows, the cure is the cubist/aurora generated avid (~25 LC).
- **G. Presets.** Capture them from hardware once looks are dialled in
  (`./capture_preset.sh`). 4 slots are free.

### Tier 3: polish and optimizations (small)
- **a.** The 8-bit hue angle steps 1.4° at a time. Very slow spins tick
  rather than glide. A 10-bit angle would need a 1024-entry ROM (still
  2 EBR) and wider address math.
- **b.** Luma band 0 (64..127) is half as wide as the others. Normalizing
  luma before banding would even them out.
- **c.** At K4 Negative with Mix 50%, the luma is flat ~502. That is the
  maths of a half negative, and it's the same as v0.1. Fine, but it's worth
  knowing when dialling Mix.
- **d. LC.** 4,884 LC (64%) for a per-pixel chain. The rotation and gain
  products dominate. There's headroom; nothing to cut unless Tier 2 needs
  space.

## Hardware check list (v0.2.0)
1. Power-on default: a Bit-7 zebra at ~50% over the picture, chroma
   posterized (9 levels), colours unrotated. Each switch S7–S11 should
   visibly change it.
2. Hue: 0° = the source's own colours. v0.1's look is at Hue 270°.
3. S7 Negative: blacks become white (~770 after the knee), not a clipped
   superwhite. Chroma should survive in the former blacks.
4. K6 at 3–5 Levels on a grey or mono source: greys stay grey (no colour
   sparkle). S11 Before/After with P12 spinning: the colour lattice turns
   (Before), or colours snap between fixed palette steps (After).
5. K4 Negative: S9 Flip turns the bright half's hue. S10 Sat swaps the
   negative for an inverted saturation map.
6. P12: still near centre, slow creep just off it, fast colour cycling at
   both ends.
7. HD HDMI: any thick bars at Mix 100% on a Bit plane with motion. Any thin
   wrong-hue lines over colour at Hue ~90° (Tier 2 F).

## Notes
- **Gouache (v0.2.0, another tab) and hueflutter read the same ROM the same
  way**, so their Hue 0 is −90° too. The fix is free: sin address = a + 64,
  cos address = a xor 128. Not touched here.

## Build results
v0.2.0 (2026-10-12), router2, every config on seed 1:

| Config | Fmax (MHz) | v0.1.1 |
|---|---|---|
| HD Analog | 83.49 | 77.69 |
| HD HDMI | 83.84 | 78.87 (seed 4; seed 1 stalled, 2–3 missed) |
| HD Dual | 78.00 | 77.77 |
| SD Analog / HDMI / Dual | 85.19 / 84.13 / 79.76 | 71.49 / 74.82 / 74.48 |

LC is 4,876–4,897 of 7,680 (~64%, was ~4,440), with 2 EBR (the two sine
ROMs). No RAM is written, so no_rw_check doesn't apply.
NOT FLASHED, NOT COMMITTED.
