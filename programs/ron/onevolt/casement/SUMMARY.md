# Casement: three striped windows per mirrored cell, the camera seen through the largest

**v0.1.0 built 2026-11-25, not flashed.** Onevolt 10. A Stacker patch: Scrolls' mirrored ramps, three Procs that
slide each window on its own path, the Stacker's three priority keys driving a Switcher, Matte colours striped
by Stairs folds of each layer's own ramp, an Angles colour field as the ground, and the camera keyed into the
largest window. Revised after review (the first build's black-input default was flat rectangles on navy): S9 is
now Stairs bars (the default) / flat Matte, and the default composition was redesigned around the bars. The
presets are hand-written placeholders.

## The rack

- **LZX Scrolls**: the two ramps (H, V) with Waveform = Mirror, scaled by Tiles (P12), so every cell is the mirror
  image of its neighbour. Its LFO outputs drive the Procs (Corner path 11 s, Ping Pong 17 s / 23 s, a 41 s
  drift).
- **LZX Proc** (three channels): a bias on each mirrored ramp sets one window's position per layer and axis.
- **LZX Stairs**: (S9 Stairs, the default) a tap on each layer's own Proc'd ramp, folded into bars. The bars are
  added to that layer's Matte luma; the hue is kept.
- **LZX Angles**: (S7) its 45 and 135 degree mixes of the ramps before the fold, so the tiling turns diagonal.
- **LZX Stacker**: three window keys, |H| and |V| each windowed and ANDed. Size per channel (K1-K3), Ratio on all
  three (K4). Priority by subtraction (layer 1 over 2 over 3).
- **LZX DSG3 + Keychain**: (S8) the DSG3 parabolas (u^2 per axis, Average) keyed by three Keychain thresholds give
  discs and rings, with the same priority.
- **LZX Switcher**: four RGB inputs, with the keys as select logic: a four-layer hard-edge compositor.
- **LZX SMX3**: (S10) the three raw keys summed instead of switched, so overlaps add to lighter colours (gels).
- **LZX Matte**: flat colours A (layer 1), B (layer 2), C (layer 3) and the ground's base, in eight designed sets
  (K5).
- **LZX FKG3**: the camera keyed over layer 3's striped Matte C by its own luma (soft key, K6 = the key gain).
- **LZX TBC2**: the camera (decoder) as Y, U, V.
- **LZX Angles** (a second one): the ground, three angled ramps as R, G, B, dark.

## The patch

    Scrolls ramp 1 (H) / ramp 2 (V), Waveform Mirror, scale = P12 Tiles            (S7: the ramps through Angles 45 / 135)
      -> Proc ch 1, 2, 3: + offset (Scrolls LFOs: Corner 11 s; Ping Pong 17 / 23 s; drift 41 s)
      -> Stacker ch 1, 2, 3 H in / V in (Size K1, K2, K3; Ratio K4)                (S8: DSG3 parabolas -> Keychain x3, thresholds K1-K3)
      -> out 1, out 2 = key 2 - out 1, out 3 = key 3 - (out 1 + out 2)
      -> Switcher select: in 1 = Matte A, in 2 = Matte B, in 3 = FKG3 (TBC2 camera over Matte C, key gain K6),
                          in 4 = Angles colour field                                 (S10: SMX3 sums the raw keys x fills + ground)
    Stairs (S9 Stairs): Proc 1 H -> bars into Matte A's luma; Proc 2 V -> Matte B's luma; Proc 3 H + V -> Matte C's
      luma (the FKG3 background)                                                    (S9 Matte: no Stairs, flat fills)
      -> encoder (soft knee above 860, clip) -> the shell's output conditioning
    S11 Osc: a drifting triangle lattice (colour from its phases) into the FKG3 / TBC2 jack, at 2x the camera's level

## What it draws

At the defaults (black input, Bauhaus set, Tiles 66 %), a woven casement six cells across, mirrored in pairs:

- **Layer 1:** large yellow panes with vertical Stairs bars (1/8 cell).
- **Layer 2:** red panes with fine horizontal bars (1/16 cell). The yellow panes cover their corners: four red
  panes pinwheel round each yellow one.
- **Layer 3:** a deep-blue field of nested diamonds (1/8 cell), under both.
- **Ground:** thin navy-to-purple leaded mullions where the panes stop short of the folds.

Every pane is striped and the three textures run in three directions at three pitches. The bars belong to their
layer: they tile, mirror and slide with it.

**Motion:**
- the yellow pane circles a small rectangle (11 s);
- the red panes bounce (17 s / 23 s);
- the diamond field drifts there and back (41 s), with a slight size breath.

As the layers slide, their edges cross and the occlusion changes, and the mullions narrow, close and reopen.

**With a camera,** layer 3 becomes the camera picture wherever it is bright. The diamonds stay in the shadows, so
the picture is etched with them.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Size 1 | Stacker channel 1 Size | the yellow pane's half-width, 0.02 .. 0.52 cell (default 55 %): a dot .. joined with its mirror neighbours into bands |
| K2 Size 2 | Stacker channel 2 Size | the red panes, same range (default 51 %) |
| K3 Size 3 | Stacker channel 3 Size | the camera's diamond field, same range (default 86 %: a field with thin mullions) |
| K4 Ratio | Stacker Ratio (all three) | wide slots .. squares (centre) .. tall slots: widened axis x 2^(6x^2), narrowed x 2^(-x^2). The ends give full-width bars / full-height columns. Under S8 it clips the discs into lenses and slices |
| K5 Colour | Matte | eight designed sets (Mono, Ember, Bauhaus, Acid, Sea, Neon, Sunset, Pastel), smoothstep-blended between neighbours; default Bauhaus |
| K6 Camera | FKG3 key gain | 0 = layer 3 is Matte C with its diamonds (exactly the black-input picture) .. 100 % (default): the camera at full level wherever its luma is above ~58 %, blending into the diamonds in the shadows (8 levels) |
| S7 Ramps | straight ramps / **Angles** | rectangles on a square lattice / rhombi on a diagonal lattice (discs on a quincunx under S8). The bars turn diagonal and the diamonds turn into nested squares |
| S8 Keys | **Stacker** / **DSG3 + Keychain** | rectangles / discs and rings (crescents, since the layers are offset), still striped |
| S9 Fills | **Stairs** / **Matte** | each layer striped by its own folded ramp (default) / flat Matte colours (the first build's look) |
| S10 Mixer | **Switcher** / **SMX3** | priority (upper layers occlude) / the raw keys added at half level over the ground: striped gels, nothing hidden |
| S11 Source | TBC2 camera / **Osc** | the camera / a drifting coloured triangle lattice at 2x the camera's level |
| P12 Tiles | Scrolls ramp scale | cell 1440 .. 480 .. 160 virtual px (exponential). 0 % = one great set of windows with broad bars; default 66 % = six cells across; 100 % = 12 across, a dense mosaic (the fine vertical bars blur away in the output filter there) |

## Video

The camera enters at Switcher input 3 through an FKG3 soft luma key.

- **Key:** over layer 3's striped Matte C, alpha = min(1.7 x K6 x luma, 1) in eighths, fill = C' + alpha (camera -
  C'), where C' is Matte C with the diamond bars in its luma.
- **Chroma:** the camera's U and V pass untouched (the dry path, no swap).
- **Sampling:** the key luma is sampled 5 source pixels ahead of the beam and the colour 8 pixels ahead (three
  registers from the input). The picture sits ~8 px left of the source, which is invisible, and the sample is held
  through blanking at the right edge.
- **Black input:** the window shows the striped Matte C, which is exactly the K6 = 0 picture.
- **Osc:** the shell's drifting 640 x 540 virtual-px triangle lattice, luma at twice the camera's full scale and
  its two phases as chroma, into the same key.

## Engine

- **Per field** (vblank zone: no avid for 1024 clocks):
  - the shared glide (K1-K6, P12; 1/8 per field);
  - 37 slots on ONE serial 30-bit multiplier, with registered tick and window decodes:
    - Tiles (exp2 ROM);
    - the six Proc offsets (one triangle unit, Scrolls LFO values times their depths);
    - Ratio (e = 6x^2, exp2 reads);
    - the beam steps and starts (S7 adds the 45-degree terms);
    - six window half-widths and three disc thresholds;
    - the Matte blend (smoothstep, twelve lerps from the set ROM);
  - then a serial gap rule that closes any mullion under two field rows at a fold.
- **Per line**: the two Scrolls phase accumulators advance on active lines only.
- **Per pixel**:
  - two 20-bit folded phases (the Mirror), 12 bits per cell;
  - per layer and axis, u = m - c and its ones'-complement |u|;
  - the Stacker windows (|u| < half-width) and the DSG3 parabola ROMs (six 256 x 12 EBR, then a fabric copy),
    summed and thresholded;
  - three Stairs folds, (fold - 64) x 4 added to the fill's luma (see below);
  - priority gates, then one adder tree for both compositors (SMX3 halves it);
  - the knee and clip;
  - the camera key chain in parallel (stages 3-9).
- **The Stairs folds**: each is an 8-bit slice of a layer's own |u|, folded to a triangle. The slices:
  - layer 1: |uA|, 1/8 cell;
  - layer 2: |uB|, 1/16 cell;
  - layer 3: |uA| + |uB|, 1/8 cell.

  The vertical bars stay soft (at least 2 field rows per half-cycle):
  - layer 2 switches to 1/8 cell at half amplitude once 1/16 would be 4 field rows or fewer per half-cycle, and
    stops once 1/8 would be;
  - layer 3 stops under S7 at 2 rows (the sum then steps twice per row).

  These are decided per field from the gap-rule threshold, which is two rows in cell units.
- **Output**: the lozenge shell's conditioning:
  - Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync;
  - luma 13/16;
  - the blanking gate;
  - one latency (C_LAT 20) in every mode and a generated avid;
  - virtual 1080-line coordinates (SD shows the centre 4:3, cells square) and colours in the hardware convention.
- **Block RAM (12)**: parabola ROMs 6, exp2 1, Matte sets 1, box delay rings 4.

## Status

- **Pin-skew placement (orchestrator, after the build).** HD HDMI re-placed for the encoder pin skew: router2 seed 38 (HSYNC +1.43 ns, data <= -0.96 ns, 77.33 MHz); the builder's router2 seed 1 was outside the +1.0..+2.3 ns window. A plain rebuild gives the builder's placement again: re-run .heirloom_work/skew_tools/install.sh casement router2 38 hd_hdmi. Screen data: .heirloom_work/rb_casement/skew/.

- **Built 2026-11-25 with router2 (the build default), not flashed. HD HDMI closed on seed 2; the other configs
  on seed 1.** Rebuild: `./build_set.sh onevolt casement`, giving `out/rev_b/ron/onevolt/casement.vmprog` (458,736
  bytes).

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 78.63 | 6,837 |
  | SD Analog | 1 | 83.62 | 6,828 |
  | HD HDMI | 2 | 81.02 | 6,830 |
  | SD HDMI | 1 | 75.09 | 6,834 |
  | HD Dual | 1 | 77.51 | 6,845 |
  | SD Dual | 1 | 67.54 | 6,845 |

  - **Fit:** 6,828 - 6,845 / 7,680 LC (89 %), up about 105 LC on the first build (6,726 - 6,740). Under the
    raised 6,950 allowance, but over the original 6,750 budget. 12 / 32 EBR.
  - **Bitstreams:** iceunpack OK x6. The packed config carries the new S9 labels, defaults and presets.
  - **Not done:** no pin-skew screening.
  - **scan.py:** 0 integer-minus-signed sites.
- **Model vs RTL** (working files in `.onevolt_work/rb_casement/`; `check.sh` clean). GHDL 720p60, decimation 1.
  Every frame is **bit-exact with the integer model `imodel.py`** (0 differing pixels, rows 1 .. 719):
  - the default black-input frame (`rtl_default.png`);
  - S9 Matte (flat);
  - P12 at 0 and at 1023;
  - all switches S7-S11 on;
  - S7 + S8 + S10 with bars, Ratio 300, Colour 700, Tiles 900;
  - Osc;
  - the camera with selfie.jpg and with bear.png;
  - a motion frame 5 fields later;
  - both 480i fields (720 x 243) and both 1080i fields (1920 x 540).
- **Thin lines** (`thincheck_v9.py` on the model; 720p and 480i, 14 settings x 4 times, each with bars and flat):
  - **Flat (S9 Matte), the original test:** no 1-row hard lines >= 24 px except one 480i time each for S11 and for
    S7 + S8.
  - **Bars:** with bars a soft bar's peak row would trip the original test, so bars use a sliver test instead: an
    isolated extremum row that no ramp continues. It passes everywhere at 720p, and at 480i everywhere except S11
    (one time).
  - **Over time** (one field in 7 over a minute, `slivrate.py`):
    - default with bars: 0 / 515 fields at 480i and at 720p;
    - 480i with S11 or with S7 + S8: 4.1 % of fields show a transient 1-row sliver. It is a lower layer's edge
      landing within one row of a higher layer's moving edge, the same class as the first build, whose geometry
      gave 8.7 % with S11.
- **Sheets:**
  - `sheet.png`, `strip.png` and `camera.png` come from the integer model;
  - `cmp_final.png` = proto | model | RTL black | RTL Osc | RTL selfie.

## Watch on hardware

- Crisp moving striped edges on HD HDMI. The shell's conditioning is in.
- The zone sequencer needs ~1,100 clocks after the idle test; every real timing has 14,000 or more.
- **SD and 720p differ from 1080p:**
  - layer 2's fine horizontal bars turn into 1/8-cell bars at half amplitude from P12 42 % at 480i (so at the
    default) and disappear from 74 %;
  - the same coarsening starts at 79 % at 1080i and 92 % at 720p, and never happens at 1080p;
  - SD also shows only the centre 4:3 (fewer cells across).

## Cut / deviations

- **DWO3 cut to fit** (the first build was 7,330 LC at 51 MHz). Its place on S9 is now Stairs bars (and the
  default), made from each layer's own ramp: no oscillator, no multiplier. Negative (Proc at gain -1) is gone.
- **The discs come from fixed parabola ROMs.** K4 Ratio clips them (lenses, slices) instead of stretching them into
  ellipses, so S8 keeps K4 live without a per-field table writer.
- **The Angles ground keeps only its Cr ramp across the screen** (Y and Cb flat): a navy-to-purple gradient.
- **Motion**:
  - the Orbital path is two quadrature triangles (Scrolls' Corner shape), not sines;
  - the breath is a slice of the 41 s drift's quadrature;
  - speeds are per field, so 50 Hz runs 17 % slower and 24p 60 % slower.
- **Tiles steps by 0.27 % per exp2 entry** (no interpolation); with the glide this is under 3 px at the edges.
- **The camera key has eight levels,** and its colour is sampled 3 px after its luma (soft key; invisible after
  the 9-tap boxes). The camera sits ~8 px left of the source; the last 8 px of each line repeat its last column.
- **The camera covers less of the screen than in the first build** (at the default, layer 3 shows on 32 - 35 %
  of the frame; layer 1 25 - 29 %, layer 2 21 - 26 %, the ground 13 - 18 %). It is fully there with K3 up or K1 / K2 down.
- **Bars folded from the window edge** (each pane starting on a dark bar) were tried and dropped: on SD they put
  a 1-row spike at each window centre in 22 % of fields.
- **The vertical [1 1]/2 line mix of the separatrix shell is not used** (lozenge's conditioning).
- No 1-px coverage antialiasing: vertical edges are softened by the shell's boxes, horizontal edges are hard rows.
