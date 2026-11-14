# VENEER / VENEER INK — summary (v0.1.0)

The finished-program follow-up to the FLATLINE lab, built from the preview cards Ron picked
on 2026-11-13 (mattes only; XDoG with few strokes; Canny at 50% and at high gain; adaptive
thresholds) and his brief: an ink mode with the edge options, and a texture mode that replaces
the flat colours with sets of generative organic textures.

**Status: both builds timing-closed and packed (2026-11-14), not flashed.**
Design, costs and deviations: [DESIGN.md](DESIGN.md). Control cards: [CONTROLS.md](CONTROLS.md).

## The two builds

| | **Veneer** `veneer.vmprog` | **Veneer Ink** `veneer_ink.vmprog` |
|---|---|---|
| Fill | S7: Ink (region colour, optional wash) / Texture (eight sets) | flat ink |
| Lines | none | Outline (K6), Sketch (K3), Contour (K5): independent layers |
| Shared | K1 set / palette, K2 Hue, K4 Colour mode, S8 Fixed / Adaptive, P12 Levels 2..8 | |

One source (`veneer.vhd`); `variants.py` stamps `programs/ron/veneer_ink/` and generates both
TOMLs; `tables.py` holds the recipes, sets, palettes and the random table. They are two
bitstreams because the matte pipeline is ~62% of the HX4K, the texture machinery ~32% and the
fine-line layers ~17% (measured table in DESIGN.md section 1).

## Build record (dev flow, router2 default, judge the LAST post-route Fmax)

Configs in order HD Analog / SD Analog / HD HDMI / SD HDMI / HD Dual / SD Dual; seeds other
than 1 noted (reuse them for a repack). SD configs are judged against the 27 MHz SD clock.

| Build | LC | EBR | HD Analog | SD Analog | HD HDMI | SD HDMI | HD Dual | SD Dual |
|---|---|---|---|---|---|---|---|---|
| veneer | 6513-6552 (85%) | 23 | 81.85 | 81.59 s2 | 79.70 s3 | 75.95 | 76.89 s3 | 77.78 s3 |
| veneer_ink | 5997-6011 (78%) | 27 | 80.84 | 78.65 | 81.61 | 78.95 s2 | 78.43 s4 | 80.17 |

All six bitstreams of each build unpack. router2 stalled on about half the first seeds (the
machine was also running the render matrix); the builder's timeout and retry handled it. Every
HD config has at least 2.6 MHz of margin.

## Previews

https://claude.ai/artifact/H7w4NKEpu8z3JHUm1wfTWv — GHDL renders of both builds (76 cards at
960x540, six at full 1080p) on a photo, a portrait, synth gradients and a test card, in hardware
colour order. Tooling: `~/veneer_work/previews/run_previews.py` (matrix, sim copies with
`C_SIM_FAST`), `gen_page.py`; page files in `.veneer_work/previews/`.

`model.py` renders the fill from an integer model at 1080p in about half a minute (approximate
mattes, exact texture math): `python3 model.py IMAGE OUT.png --set 4 --colour 2`.

## What to look at first on hardware

1. **Veneer**, preset Timber: regions filled with wood grain and burl rings, embossed; S7 to Ink
   gives the flat mattes with a black ground. K1 through the eight sets. K5 up: the textures
   should flow smoothly with no tearing at the left edge.
2. **Region edges on live video.** The temporal vote is compiled out and the hysteresis band is
   one code (DESIGN.md section 5): if region edges boil on a noisy source, that is the constant
   to raise (`C_HYST`), at the price of flat-bottomed shapes on smooth gradients.
3. **Veneer Ink**, preset Contour, then Sketch, Outline, All Lines; S11 Paper for the line
   drawing.
4. **HD HDMI bars.** Both builds carry the [1 2 1] + luma conditioning; the texture chroma ramp
   follows a smoothed copy of the texture so chroma never swings at pixel rate. If bars show on
   fast Drift, widen the chroma filter first (CLAUDE.md).
5. SD / 720p: mattes and textures are larger relative to the screen (DESIGN.md section 6).

## Sim notes

- The image tester is plain BT.601 both ways; hardware order = pre-swap the stimulus Cb/Cr and
  post-swap the render (`~/veneer_work/uvswap.py`).
- Always `--set` the stepped knobs and P12 to zone centres (K1: 64 + 128 i; K4, K6: 128 + 256 i;
  P12 for L levels: 73, 219, 365, 512, 658, 804, 950).
- The texture line start needs about 50 clocks of blanking; the tester's fixed 64-clock hblank
  is enough at every decimation.
- A portrait and one full-resolution render are part of the check: the blocky temporal-vote
  artifact only showed there.
