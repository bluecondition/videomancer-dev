# Slipframe — Datamosh-style processor

## What it does
A multi-stage glitch processor that combines five datamosh-flavoured effects in one pass: horizontal smear (per-pixel variable delay), chroma lag (extra Y/U/V offset), scanline tear (per-line horizontal shift), line-to-line persistence (blend with the previous scanline), and bit-crush (mask the LSBs). A Glitch toggle momentarily forces all knobs to max for a punch-in artifact, and a dry/wet mix and bypass round out the controls.

## How it works
Processing program. The smear uses three `variable_delay_u` BRAM lines (10-bit × 2048 samples each) addressed by `base_smear + luma-or-noise modulation + per-line tear offset + chroma-lag offset`. A dual-bank `video_line_buffer` holds the previous scanline's Y for persistence blend. An LFSR drives the random tear pattern and a per-frame "smear drift" random walk so the image slips around over time. BRAM budget is ~100 Kbit of the iCE40-HX4K's 128 Kbit. 13-clock pipeline; multiplies are pre-registered to keep Fmax at 74.25 MHz.

## Controls
- **K1 Smear** — base horizontal delay in samples (0–1023).
- **K2 Smear Mod** — strength of luma- or noise-driven modulation of the smear.
- **K3 Chroma Lag** — additional 0–511 samples of delay on U/V.
- **K4 Tear** — magnitude of the per-line horizontal offset.
- **K5 Persist** — blend amount with the previous scanline's luma.
- **K6 Crush** — number of low bits to mask (Off, 1–7).
- **T7 Smear Src** — modulation source: Luma vs Noise.
- **T8 Tear Mode** — Lines (alternating) vs Random (LFSR-picked).
- **T9 Chroma Dir** — Same direction or Split (U/V in opposite directions).
- **T10 Glitch** — momentary boost: forces smear/mod/tear/lag knobs to max.
- **T11 Bypass** — pass input video straight through.
- **S12 Mix** — dry/wet crossfade (0 = dry, 1023 = full wet).

## Presets
- Clean
- P-Frame Drift
- Torn Transport
- Chroma Wreck
- Full Mosh
