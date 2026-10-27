# Mood HD

A circuit-level rebuild of the 1976 music visualizer developed as "Project Mood" (US patent 4,081,829). Each stereo channel draws a stair-stepped diamond: its ENERGY (rectified, integrated audio) sets the size against summed V-shaped ramps, and its zero-crossing rate picks the colour from a 16-tap phase-delay colour modulator through the patent's Table I. SOLID, HOLE and RING object logic, the "last channel to turn on wins" colour flip-flop, H and V arrays and AUTO mode all follow the circuit. A built-in phantom stereo record plays with nothing patched.

Mood HD is the CLEAN render of **Mood** (same circuit model, same controls). The tiles are true |x|+|y| ramps that land exactly on the screen. ENERGY is interpolated between virtual lines, so contour fringes become smooth curves rather than blocky steps. Edges are anti-aliased, and RING draws uniform-width outlines along the HOLE band's edges instead of the original's lopsided one-shots.

**P12 (Volume):** The volume fader of the stereo the unit sat beside. It scales both channels ahead of the detectors, so the objects collapse to nothing at the bottom and flood their tiles at the top. CONTOUR shapes how they respond: with a fast contour they snap to the fader, with a slow one they swell. At zero the colours freeze and AUTO stops changing, because silence has no zero crossings.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | L Gain | 0–100% | Left (channel 1) level into its detector: the size of its diamond. On EXTERNAL, the left audio level arrives here (see below) |
| K2 | R Gain | 0–100% | Right (channel 2) level, likewise |
| K3 | L Contour | 0–100% | Left integrator time constant, 0.3 ms to 100 ms (log). Low settings leave audio ripple on ENERGY, so the edges move line by line in combed, fringed outlines. High settings give a clean diamond that breathes |
| K4 | R Contour | 0–100% | Right integrator time constant |
| K5 | Color | 6–30 deg | Phase step per delay stage (the chip's VGG). It fans the 16 hues out from the treble tap: bass colours swing furthest. Above ~22 deg the 16 taps wrap a full turn and hues repeat |
| K6 | Shape | Solid / Hole / Ring / Auto | Solid = either channel; Hole = the band between the two diamonds (equal levels show nothing); Ring = uniform-width outlines (one reference unit) along the band's edges; Auto = random arrays and shapes driven by the audio |
| S7 + S8 | Horiz A, Horiz B | Gray-coded pair | H array ×1 / ×2 / ×3 / ×5. In Auto: update rate frantic / lively / relaxed / rare |
| S9 + S10 | Vert A, Vert B | Gray-coded pair | V array ×1 / ×2 / ×4 / ×8. In Auto: lock Solid / lock Hole / lock Ring / Auto All |
| S11 | Source | Internal / External | Internal = the phantom stereo record; External = audio from the Audio/CV inputs via K1 and K2 |
| P12 | Volume | 0–100% | Master level into both channels, ahead of the gain stage |

**Switch pairs are Gray-coded**, so one flip always moves to the adjacent setting: (Off, Off) first, (On, Off) second, (On, On) third, (Off, On) fourth.

| Horiz A | Horiz B | Array | Auto rate |
|---|---|---|---|
| Off | Off | ×1 | Frantic (several per second) |
| On | Off | ×2 | Lively (~1 per second) |
| On | On | ×3 | Relaxed (~every 4 s) |
| Off | On | ×5 | Rare (~every 15 s) |

| Vert A | Vert B | Array | Auto shape |
|---|---|---|---|
| Off | Off | ×1 | Solid lock |
| On | Off | ×2 | Hole lock |
| On | On | ×4 | Ring lock |
| Off | On | ×8 | Auto All |

AUTO redraws the H and V arrays (and the shape, unless it is locked) from a 6-bit random register. That register is made from the two low bits of each channel's colour register and of the live zero-crossing counter. AUTO never touches gain, contour or colour, and it does not change with a silent source.

## External audio (Source = External)

The program sees audio only as parameter values, so on EXTERNAL it rebuilds a waveform per channel: level × carrier. The level is the K1 (left) or K2 (right) register as the modulation operators drive it. The carrier's pitch follows how busy that level is: it counts level reversals over ~0.5 s, so busier music gives higher counts and moves the colours along Table I.

Recommended patch (confirm the operator names on the unit):
- Left audio → modulation operator in envelope-follower mode → K1 (L Gain); right audio → K2 (R Gain).
- Fast attack (~5 ms) and medium release (~50–100 ms), so drum hits pump the level and produce reversals.
- Knob positions around 10–20% as the floor, with the modulation depth reaching the top of the range on loud passages.
- Volume (P12) still scales both channels. With nothing patched, K1 and K2 behave as plain size controls on a steady carrier.

## Presets

Breathing (power-on look), Ring Grid, Fringe, Auto All.

## Technical notes

- Program type: synthesis; core `yuv444_30b`. The source lives in **Mood** (`programs/ron/mood`); this folder's build hook copies the shared core and package.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Tiles span the active picture exactly (N across, M down at every format). Each virtual line is one 15.734 kHz audio sample, in real time.
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
