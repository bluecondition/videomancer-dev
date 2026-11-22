# Patchbay (started 2026-11-14)

The patches Ron never built: each program is a patch that could have been made on his old modular video
synthesizer, designed from the signal flow outward, with the analog character and the playable controls of a
phase-2 Heirloom program. Heirloom reproduces patches that existed; Patchbay invents new ones in the same
language. The brief and the acceptance bar are in [PATCHBAY.md](PATCHBAY.md). The set is open-ended.

Flash from `out/rev_b/ron/patchbay/<name>.vmprog`; rebuild one with `./build_set.sh patchbay <name>`.

## Index

| # | Program | The patch | P12 headline | Status |
|---|---------|-----------|--------------|--------|
| 1 | [munch](munch/) | Digital logic: ramp ADCs through XOR / AND / ADD into a DAC colourizer (munching squares, Sierpinski) | Munch | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 93.9-100.1 MHz; 6,693 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 2 | [chladni](chladni/) | Four sine VCOs through two multipliers: standing-wave sand figures on a plate | Resonance | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 93.9-99.4 MHz; 7,014 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 3 | [pulsar](pulsar/) | Stacked scope traces with hidden lines (the pulsar ridgeline plot) | Burst | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 88.4-91.3 MHz; 7,035 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 4 | [skyline](skyline/) | Sample-and-hold bar graph with peak caps and a reflection | Drive | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 93.1-100.2 MHz; 6,725 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 5 | [lissajous](lissajous/) | An X-Y scope figure rescanned as a raster through arcsine shapers | Detune | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 76.4-86.3 MHz; 7,062 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 6 | [crosscolor](crosscolor/) | Fine luma patterns through a composite decoder: rainbow cross-colour and dot crawl | Bleed | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 84.5-92.4 MHz; 7,139 LC); K6 camera default 100 %; not flashed |
| 7 | [euclid](euclid/) | A Euclidean step sequencer drawn as a piano roll | Fill | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 87.8-91.8 MHz; 5,776 LC); K6 camera default 100 %; not flashed |
| 8 | [agate](agate/) | Cascaded cross-modulated ramps: marbling and cut-stone bands | Stir | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 80.6-86.7 MHz; 6,922 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 9 | [limelight](limelight/) | Soft-keyed rotating ramps: swinging beams of gelled light | Blaze | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 96.4-99.8 MHz; 6,970 LC); K6 camera default 100 %; not flashed |
| 10 | [colorfield](colorfield/) | Soft window keys as paint: breathing colour fields | Dissolve | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 78.9-82.2 MHz; 7,132 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 11 | [monochord](monochord/) | H-synced harmonic VCOs through rectifiers and window comparators: a stack of vibrating strings, row n in n lobes | Pluck | v0.1.0 built 2026-11-22 (wave 2), router2 seed 1 all configs, HD 83.4-84.9 MHz, 7,023 LC (91 %), 21 EBR; model = RTL (0 px: 720p, 480i, 1080i; camera frames in structure only); cut to two strings per pixel (P12 top 1.6x row height, Fan = two harmonics, Strobe = string + outline); thin features keep 720p line counts, so thicker at SD; not flashed |
| 12 | doppler | A ring generator whose centre runs down a delay line: wavefronts from a moving source, up to the Mach cone | Mach | planned 2026-11-22 (wave 2), build running |
| 13 | ringdown | A square wave through eight resonant filters, comparators summed: step responses drawn as solid count-bands | Resonance | planned 2026-11-22 (wave 2), build running |
| 14 | [argon](argon/) | Shape generators through outline extraction and glow: a neon sign of a ring, a triangle and a hexagon | Voltage | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs, HD 75.7-77.9 MHz (HD Analog +1.5 MHz: thin), 91 % LC, 24 EBR; model = RTL (0 px: 720p, NTSC, 1080i; camera frames in structure only); RGB glow tables with a shoulder; only P12 glides; TOML processing_delay_clks corrected after the build (config binary verified identical); not flashed |
| 15 | [separatrix](separatrix/) | V ramp squared plus an H cosine into a quantiser: the phase portrait of a pendulum | Gravity | v0.1.0 built 2026-11-22 (wave 2), router2 seed 1 all configs, HD 76.9-86.2 MHz (HD Analog 2.6 MHz margin), 7,011 LC, 23 EBR; model = RTL (0 px: 720p, 480i, 1080i; camera frames match in structure only); K5 = eight colourings (no continuous hue rotation); SD shows three eyes 25 % narrower than HD; not flashed |
