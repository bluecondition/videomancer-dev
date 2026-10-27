# Wrinkle Lab — progress

Fork of `homegrownvhs/wrinkle` v2.1.0 (Adam Pflanzer, GPL-3.0), taken from the
community repo copy (it carries the May HD-timing fixes and the SDK 1.1
latency padding; `programs/homegrownvhs/wrinkle` in this repo is the older
April original). Own program_id `com.czartroniks.wrinkle`, name "Wrinkle Lab",
so it sits beside the original on the device for A/B.

## Why it barred (diagnosis 2026-10-19)

1. **Input LPF rang undamped at the defaults.** The v2.1.0 timing fix split the
   IIR into `E <= X - I` then `I <= I + E>>k`, so each update used the error of
   the pixel before. At Smoothing 0 (k = 0, the default and every preset) that is
   `I(n) = I(n-1) - I(n-2) + X(n-1)`: poles on the unit circle, an undamped
   6-pixel ring from every edge, running to the end of the line. A model of it
   rang +-759 codes after a 100->800 step and overflowed 1023 on 574 of 1920
   pixels; noise alone grew to +-80..150 codes by the right edge. The fold then
   multiplies that into full-swing pixel-rate garbage. The April original used
   the stable one-stage form.
2. **No output guard.** The fold drives Y to 1023 and U/V to the 0/1023 rails in
   sharp pixel steps, the known HD HDMI thick-bar trigger.
3. **No blanking gate.** The interpolators hold their last input while `enable`
   is low, so each line's last pixel was smeared across the blanking interval.

## v2.2.0 (2026-10-19)

- IIR restored to a true first-order filter in a timing-friendly form:
  stage A pre-shifts the input (`X>>k`), the loop is `(I + X>>k) - (I>>k)`;
  first pixel of each line uses k = 0 (exact snap). Tracks the original
  filter within 1 code for k <= 7.
- Stage 4b clamps to legal range: Y 64..1023, U/V 64..960.
- Wet-path guard (not on dry, so Mix 0% is still the untouched source):
  horizontal [1 2 1]/4 on Y/U/V, then a luma knee halving the slope above 512
  (1023 -> 767). This is a variant of damier's linear 13/16 scale: mids and
  darks keep the original look, only highlights compress.
- Blanking gate at the output register (bypass still passes raw input).
- One latency in every mode: 20 clocks (`processing_delay_clks = 20`).
- Dry/wet alignment fixed (wet was 1 px behind dry in the mix; video was 1 px
  behind the syncs).

Build 2026-10-20 (`./build_programs.sh ron wrinkle`, router2): 6/6 timing-closed,
~5850/7680 LC (76%), 9/32 EBR (the three sine ROMs). Post-route Fmax: HD Analog
76.6 (seed 1), HD HDMI 74.38 (**seed 2**, thin), HD Dual 76.2 (seed 1),
SD Analog 78.0 (**seed 2**, seed 1 stalled), SD HDMI 73.1, SD Dual 77.2.
Not flashed yet.

HW 2026-10-20: **bars persist on v2.2.0** (settings not yet reported).

## Diag build (2026-10-20): `.wrinkle_work/wrinkle_diag.vmprog`

Source `.wrinkle_work/diag/`, built by `.wrinkle_work/build_diag.sh`. "Wrinkle Diag"
(`com.czartroniks.wrinklediag`), one placement, latency 28 at every level:
- K6 = Guard: Off (v2.1.0 output range, no smoothing/knee) / Lab (v2.2.0) /
  Damier ([1 2 1] + linear 13/16) / Heavy (5-tap Y, 9-tap U/V, linear 5/8).
  Brightness fixed at 0.
- S11 = true 1-cycle passthru (rollthebones style).
- 6/6 closed: HD HDMI 79.5 (seed 2), HD Analog 75.5 (s1), HD Dual 76.4 (s2;
  s1 stalled), SD 75.1 / 71.7 / 79.5 (s1). 88% LC.

HW 2026-10-20 on the diag: **"Heavy setting seems to get rid of bars."**
(Other levels / Mix 0 / Passthru not reported.)

## v2.3.0 (2026-10-20)

Heavy guard moved into Lab (wet path only): Y [1 4 6 4 1]/16, U/V
[1 2 2 2 2 2 2 2 1]/16, luma 64 + 5/8 (y - 64), white ~662. Latency 24.
Build: 6/6 closed, ~6400/7680 LC (83%). HD Analog 75.5 (s1), HD HDMI 75.9
(**seed 2**), HD Dual 75.9 (s1); SD 75.9 / 67.1 / 70.4 (s1). Diag 2: HD HDMI
78.9 (s2), HD Analog 76.1, HD Dual 74.5 (s1, thin).

## Diag 2: `.wrinkle_work/wrinkle_diag2.vmprog`

Bisects Heavy on one placement. K6: Heavy / Bright (Heavy filters, Lab knee
luma, white 767) / UV121 (chroma back to [1 2 1]) / Y121 (luma back to
[1 2 1]). A level that stays bar-free means the part it weakened isn't needed.

HW 2026-10-20 on diag 2: **bars only at UV121.** Heavy, Bright and Y121 clean.
So the wide chroma smoothing is the lever; luma needs neither the wider filter
nor the 5/8 dimming.

## v2.4.0 (2026-10-20)

Guard = U/V [1 2 2 2 2 2 2 2 1]/16 + Y [1 2 1]/4 + luma knee (white ~767);
latency 24. Brighter and sharper than v2.3.0. This exact combination was not a
diag position (Bright and Y121 each weakened one luma part), so confirm on HW.
Build: 6/6 closed, ~6330/7680 LC (82%). HD Analog 75.2 (**seed 2**), HD HDMI
77.2 (s1), HD Dual 76.6 (s1); SD 78.6 / 75.1 / 74.9 (s1).

## Open questions / next experiments

- Which of the three fixes mattered for the bars? Only the IIR fix changes the
  default picture's structure; a HW A/B on one placement would need a spare
  switch (all five are used; S11 Bypass is the candidate).
- Raw video is now carried 20 FFs deep for bypass/dry (was 16). If bypass or low
  Mix greens out on HD HDMI, use the rollthebones 1-cycle passthru bypass.
- If thin coloured lines show on HD HDMI, try a generated avid (aurora/cubist).
