# Onevolt (started 2026-11-25)

Ten small racks of real analog video modules, each cabled into one patch and simulated as a Videomancer program.
The knobs are the modules' panel knobs; each switch swaps one module for another on the same cables; K6 is the
camera strength, S11 the input module (Video / Osc), P12 the control a performer would ride. No reference
images: the racks were chosen from researched module documentation (LZX Industries, Syntonie, BSO, Visible
Signals and others). The brief, the research pointers and the acceptance bar are in [ONEVOLT.md](ONEVOLT.md).

Flash from `out/rev_b/ron/onevolt/<name>.vmprog`; rebuild one with `./build_set.sh onevolt <name>`.

## Index

| # | Program | The rack | P12 headline | Status |
|---|---------|----------|--------------|--------|
| 1 | stairwell | LZX Gen3: Scrolls -> DSG3 (two shapes in series) -> Stairs or Ribbons -> ESG3. Folded shapes in three colour taps, each band subdividing the last | Steps | spec written, not built |
| 2 | gyre | LZX Expedition: ramps -> Navigator with a video-rate rotation CV -> Arch -> Staircase or Doorway -> Color Chords. A lattice wound into a vortex | Twist | spec written, not built |
| 3 | [ondine](ondine/) | Syntonie: Ondes -> Formes -> Combines -> Solaire -> Matrice. A two-oscillator lattice with FM, AM and a rectifier chain | Fold | v0.1.0 built 2026-11-25, router2 seed 1 all configs, HD 78.5-79.4 MHz, 6,959 LC (90.6 %, about 200 over the set budget), 22 EBR; model = RTL (0 px: 720p, 480i, 1080i, switches, P12 ends); S10 is ADD / SUB; the camera also feeds the Solaire injection; third modulator cut; not flashed; NO in-window HD HDMI placement found (lines possible on HD HDMI) |
| 4 | [flyback](flyback/) | LZX Expedition: a shape gradient through Staircase into Prismatic Ray's sync input -> Color Chords. Bars that restart on every contour | Shape | v0.1.0 built 2026-11-25, ROUTER1 seed 1 all configs (router2 missed HD Analog), HD 89.4-90.4 MHz, 6,742 LC (88 %), 21 EBR; model = RTL (0 px at 720p; NTSC, 1080i and the switches matched before a last one-constant change); S8 defaults to Gate; SD crops the diamond's side tips; camera busy on detailed scenes; not flashed; HD HDMI re-placed router1 seed 4 (HSYNC +1.36 ns) |
| 5 | [overshoot](overshoot/) | LZX Contour and Curtain, Syntonie Detail, fed by DSG3 + Keychain. Horizontal-only filters: lit edges with comet tails, bas-relief, ringing | Width | v0.1.0 built 2026-11-25, ROUTER1 seed 1 all configs (router2 stalled on HD HDMI), HD 88.6-101.7 MHz, 6,882 LC (89.6 %, about 130 over the set budget), 12 EBR; model = RTL (0 px: 720p, 480i, 1080i, switches, P12 ends); Detail resonance cut (S9 = DSG3 Average / Maximum); syncs delayed through a block-RAM ring (unproven on hardware); not flashed; HD HDMI placement in the pin-skew window (HSYNC +1.14 ns) |
| 6 | [portcullis](portcullis/) | LZX Castle: ADC, Clock VCO, D flip-flops, shift register, gates, counter, DAC. Three-bit video in clocked columns with colour echoes | Clock | v0.1.0 built 2026-11-25, router2 seed 1 all configs, HD 78.0-85.5 MHz (HD Dual thinnest), 6,582 LC (86 %), 19 EBR; model = RTL (0 px: 720p, SD and 1080i fields, switches, P12 ends); S8 is DSG3 Average / Maximum (the free-running clock was dropped: it tore band edges); the camera reads as a silhouette; not flashed; HD HDMI re-placed router2 seed 29 (HSYNC +2.30 ns) |
| 7 | [subcarrier](subcarrier/) | LZX Swatch loop with a Syntonie quadrature oscillator, Factors, Proc and Keychain. The screen as the colour plane; the camera as the luma | Spectrum | v0.1.0 built 2026-11-25, router2 seed 1 all configs, HD 85.4-88.0 MHz, 6,591 LC (86 %), 16 EBR; model = RTL (0 px: 720p, 480i, 1080i, switches, P12 ends); default S10 = DSG3 Abs; a per-field camera-presence fade keeps the patch's own luma on a black input; not flashed; HD HDMI re-placed router1 seed 7 (HSYNC +1.25 ns) |
| 8 | [marquetry](marquetry/) | Syntonie: Rampes -> Courbes -> Seuils -> Isohelie -> Matrice. Three shape gradients shifted, cut by logic and posterised with bit swaps | Curve | v0.1.1 2026-11-25 = v0.1.0 RTL with HD HDMI re-placed (router1 seed 6, HSYNC +1.45 ns; v0.1.0's router2 seed 1 showed thin yellow / cyan lines on hardware); other configs router2 seed 1; HD 76.9-96.7 MHz, 6,500 LC (85 %), 20 EBR; model = RTL; awaiting hardware |
| 9 | hachure | BSO Scanner or Visible Signals Quarterizer choosing among four Angles + Stairs textures by a shape gradient. A figure drawn in four hatchings | Blend | spec written, not built |
| 10 | casement | LZX Gen3: Scrolls -> Stacker -> Switcher over Matte, DWO3 and the camera. Three sliding, tiling windows with priority; the camera shows through | Tiles | spec written, not built |
