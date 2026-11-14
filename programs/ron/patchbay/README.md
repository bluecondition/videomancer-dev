# Patchbay (started 2026-11-14)

The patches Ron never built: each program is a patch that could have been made on his old modular video
synthesizer, designed from the signal flow outward, with the analog character and the playable controls of a
phase-2 Heirloom program. Heirloom reproduces patches that existed; Patchbay invents new ones in the same
language. The brief and the acceptance bar are in [PATCHBAY.md](PATCHBAY.md). The set is open-ended.

Flash from `out/rev_b/ron/patchbay/<name>.vmprog`; rebuild one with `./build_set.sh patchbay <name>`.

## Index

| # | Program | The patch | P12 headline | Status |
|---|---------|-----------|--------------|--------|
| 1 | [munch](munch/) | Digital logic: ramp ADCs through XOR / AND / ADD into a DAC colourizer (munching squares, Sierpinski) | Munch | v0.1.0 built 2026-11-14 (ROUTER1, all six seed 1; HD 95.4-100.2 MHz; 6,478-6,503 LC, 85 %, 18 EBR); not flashed |
| 2 | [chladni](chladni/) | Four sine VCOs through two multipliers: standing-wave sand figures on a plate | Resonance | v0.1.0 built 2026-11-14 (router2, all six seed 1; HD 75.3-84.5 MHz, HD HDMI thin at 75.27; 6,939-6,948 LC, 90 %, 24 EBR); not flashed |
| 3 | [pulsar](pulsar/) | Stacked scope traces with hidden lines (the pulsar ridgeline plot) | Burst | v0.1.0 built 2026-11-14 (ROUTER1, all six seed 1; HD 86.8-93.0 MHz; 6,891-6,911 LC, 90 %, 22 EBR); not flashed |
| 4 | [skyline](skyline/) | Sample-and-hold bar graph with peak caps and a reflection | Drive | v0.1.0 built 2026-11-14 (router2; SD Dual seed 2, others seed 1; HD 83.9-88.4 MHz; 6,674-6,712 LC, 87 %, 12 EBR); not flashed |
| 5 | [lissajous](lissajous/) | An X-Y scope figure rescanned as a raster through arcsine shapers | Detune | v0.1.0 built 2026-11-14 (router2, all six seed 1; HD 81.9-84.2 MHz; 7,046-7,095 LC, 92 %, 19 EBR); not flashed |
| 6 | [crosscolor](crosscolor/) | Fine luma patterns through a composite decoder: rainbow cross-colour and dot crawl | Bleed | v0.1.0 built 2026-11-14 (router2; SD Dual seed 2, others seed 1; HD 79.5-80.8 MHz; 6,911-6,937 LC, 90 %, 21 EBR); not flashed |
| 7 | [euclid](euclid/) | A Euclidean step sequencer drawn as a piano roll | Fill | v0.1.0 built 2026-11-14 (router2, all six seed 1; HD 85.7-91.4 MHz; 5,533-5,560 LC, 18 EBR); not flashed |
| 8 | [agate](agate/) | Cascaded cross-modulated ramps: marbling and cut-stone bands | Stir | v0.1.0 built 2026-11-14 (ROUTER1, all six seed 1; HD 82.0-94.4 MHz; 7,150-7,180 LC, 93 %, 24 EBR); not flashed |
| 9 | [limelight](limelight/) | Soft-keyed rotating ramps: swinging beams of gelled light | Blaze | v0.1.0 built 2026-11-14 (ROUTER1; HD HDMI seed 3, others seed 1; HD 78.5-80.5 MHz; 6,857-6,895 LC, 90 %, 16 EBR); not flashed |
| 10 | [colorfield](colorfield/) | Soft window keys as paint: breathing colour fields | Dissolve | v0.1.0 built 2026-11-14 (ROUTER1, all six seed 1; HD 84.8-87.8 MHz; 6,704-6,717 LC, 87 %, 24 EBR); not flashed |
