# GILT: progress

**v3.0.0 (2026-10-11): built from Ron's "one final try" brief (DECISIONS D14). Not
flashed yet.** Controls: `GUIDE.md`. Summary and build results: `SUMMARY.md`.

v3 = v0.10.0's warp field (the warpy, oily, swirly look Ron approved) + a
Sugarcoat-style texture generator (classify by luma or chroma, theme ROM, 16 Klimt
textures in the warped space) over the gold-painted picture.

## Next
Ron flashes `out/rev_b/ron/gilt.vmprog` and reports. Likely tuning points, all
constants in `gilt.vhd`:
- texture shapes and sizes (E8 rules: ring radii, square frames, eye size, blob sizes)
- the theme table `C_TMAP` (which texture goes where)
- paint/texture alignment `C_PDLY` (texture regions come from the blurred field)
- chroma-map saturation floor `C_SATF`
- outline thickness (2 px / 2 lines now)
- Swirl ranges: drag levels (`f_wscale`), wave levels (sequencer state 7), drift rate
- presets are hand-written placeholders: capture real ones with `./capture_preset.sh`

## History
- v0.1-v0.15 (v1 line): `legacy/gilt_v1/` (v0.10.0 source: `work/gilt_v0100.vhd`)
- v2.0.0 / v2.1.0: `versions/` (vhd, toml, vmprog, GUIDE, SUMMARY); v2 design inputs
  (`GILT_V2_PROMPT.md`, `STYLE_SPEC.md`, `analysis/`) stay for reference
- v3.0.0: this folder
