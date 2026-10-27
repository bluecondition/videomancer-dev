# MOOD (v0.1.0 — circuit-level 1976 music visualizer)

**STATUS: built, not flashed (2026-10-27).** First build of MOOD (VINTAGE)
and its sister MOOD HD (CLEAN, `programs/ron/mood_hd`). The microcode and
display were checked against the spec's validation list in the Python model.
The RTL was not simulated: build, then hardware.

Brief: Ron's "MOOD: Claude Code Build Prompt" (2026-10-27). Rebuild the
Video Music block by block from US 4,081,829, add a CLEAN HD render and a
built-in phantom stereo source. Atari's name and logo stay out of the UI.

## Files

| File | What |
|---|---|
| `mood_core.vhd` | the whole design, `G_CLEAN` generic (shared with mood_hd) |
| `mood.vhd` | `program_top` wrapper, `G_CLEAN => false` |
| `mood_ucode.py` | the sample program (microcode) |
| `mood_uasm.py`, `mood_uemu.py` | assembler, emulator (DAMIER's engine + banked raw ROM reads, 1024 words) |
| `mood_tables.py` | ROM tables: quarter sine, contour coefficients, Table I, popcount, legal chroma radius, notes, patterns, format constants |
| `mood.py` | build hook: writes `mood_pkg.vhd` here and in `../mood_hd` |
| `mood_model.py` | emulator + display model: `budget`, `validate`, `render` |
| `tools/synth_standalone.sh` | standalone yosys size check (`clean` arg for MOOD HD) |
| `../mood_hd/mood_hd.py` | MOOD HD hook: reruns mood.py, copies mood_core.vhd |

## Architecture

1. **Virtual NTSC raster.** 228 clocks × 262 lines; 208 × 200 (lines 31–230)
   are mapped onto the active picture at every format (DDA steps from the
   timing ID). VINTAGE ramps span the full period, so tiles crop slightly at
   the edges; CLEAN tiles land exactly on the screen.
2. **Engine** (DAMIER's, HW-validated design): one sample per tick, ticks
   every 858 / 1716 / 4719 pixel clocks = 15.734 kHz real time in every
   format. Each tick restarts the program at PC 0, which doubles as the
   watchdog. Ticks start after the first field anchor. Worst case is 615
   clocks (emulator), against an SD budget of 858.
3. **ENERGY ring.** Every sample writes {E1, E2} into a 1024-entry EBR ring.
   Each output line reads its virtual line from the latest complete virtual
   frame (base = write pointer − 262, latched per field), so ENERGY changes
   line by line: the CONTOUR fringe. CLEAN reads two entries and interpolates.
4. **Frame tasks** (one per sample after each 262-sample virtual frame):
   latch the alternating counter, SHAPE zone / switches / AUTO → MODE,
   gains and contours, both colours (Table I → hue → legal U/V), Path B
   carrier pitch and mid pan, CLEAN AA constants. AUTO's check runs
   mid-frame.
5. **Pixel paths.** VINTAGE: tick DDA → H step DDA → tri → ref → comparators
   → object logic + RS flip-flop → one luma level / selected colour → one-pole
   smear (luma τ ≈ 0.5 virtual clock, chroma ≈ 2). CLEAN: Q24 tile phase →
   |φ−½| → distance → × pixels-per-unit → coverage → shape coverage → colour
   × coverage → [1 2 1].
6. Generated avid (parity vote), blanking gate, U/V swapped at the output
   only.

## Decisions and departures from the patent

- **EXTERNAL is Path B.** The program only receives `registers_in`; raw
  audio never reaches the fabric. Audio = K1/K2 level × a carrier whose pitch
  follows level reversals (deadband 8 LSB, τ ≈ 0.5 s, ~8 counts per
  reversal/s, plus 1 so the carrier never stops).
- **RENDER = two programs** (MOOD / MOOD HD): all 12 controls are taken, and
  the SDK has no settings outside them.
- **Composition is identical at every format** (spec rule), so 4:3 SD shows
  the same tiles squeezed to 0.75 width. The visible window is 208 clocks,
  not 160, so 16:9 HD keeps the diamond proportions of a 4:3 TV showing 160
  clocks.
- **Real-time virtual clock.** At 24/25/30p the display shows the latest
  complete virtual frame and skips the rest. Tempo, contour and colour
  calibration stay correct, and the display stays tear-free.
- **Count calibration:** rising crossings per 1/60 s, 60 Hz per count. This
  matches the patent's own frequency column: 0–120 Hz = counts 0–1.
- **Colour-select toggle** fires on the rising edge of (ch1 AND ch2). That is
  the patent's inverter/NOR/XOR toggle with D1 > D2, and it gives "last to
  turn on wins". Equal energies therefore alternate colour tile to tile. At
  each line start the flip-flop resets to ch1 with "both previously off".
- **AUTO:** checks once per virtual frame. Rates: ≥5 of 6 ones every frame;
  ≥6 every frame; ≥6 every 4th frame; ≥6 every 16th frame. The new arrays
  and shape come from the register one frame AFTER it fires (drawn at the
  firing instant they would always be all ones). The 4th shape code maps to
  Solid.
- **Hue:** hue(n) = φ0 − n·θ (longer tap = more lag), θ = 6–30°.
  φ0 puts DL16 (bass) at red with COLOR centred, so frequency runs
  red → yellow → green → cyan → blue → magenta.
- **Palette:** one luma (Y 458) and the per-hue legal chroma radius, capped
  at 0.40 for flat, near-equal hues.
- **Interlace:** no input carries field_n, so both fields draw the same
  picture.
- **VINTAGE ring:** an edge at the first visible pixel does not fire; on
  the original those pulses ran in blanking.
- **CLEAN ring** = the HOLE band minus its interior shrunk by one reference
  unit, so equal energies still show nothing (validation 1). The AA width
  uses 0.8 / the larger axis gradient rather than the exact norm.

## Phantom stereo (mood_ucode.py)

96–132 BPM, wandering per phrase. 8- or 16-bar phrases cycle full →
breakdown → build → drop (1 in 8 skips the breakdown). Voices:
- Shared kick: 150 → 48 Hz, ~65 ms.
- Per-channel bass random walks: minor pentatonic, 41–247 Hz, unison 1 in 4.
- A mid partial (294–1976 Hz) with a slow envelope, auto-panned.
- Hats: per channel, ~16 ms.
- Snare noise: both channels, ~65 ms. The build's last bar rolls kick and
  snare, which is where the counter rolls over.

Model at defaults: ENERGY median ~8.6 units, peaks ~19. Every Table I tap
occurs; rollover about once per 12 s.

## Model validation (python3 mood_model.py validate) — all PASS

1. Equal energies: SOLID shows the diamond; HOLE and RING show nothing.
2. ch1 > ch2: HOLE band; SOLID outer band in ch1's colour, core in ch2's.
3. Table I 0..63 (formula and ROM); 66 crossings latch as 2.
4. VINTAGE ring inside on left edges, outside on right, thin at the tips.
5. H×5 V×8: SD and 1080p agree on 98.6% of a 120×60 grid.
6. Contour 0: 0.35 units of line-to-line change; contour 100%: 0.002.
7. AUTO silent: one mode; with the phantom: 10 modes in 10 s.
8. COLOR sweep moves DL16 16× further than DL1.

## Build (2026-10-27, ROUTER1, all 6 configs seed 1)

`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron mood` (and `ron mood_hd`).

| | HD Analog | SD Analog | HD HDMI | SD HDMI | HD Dual | SD Dual | LC | EBR |
|---|---|---|---|---|---|---|---|---|
| MOOD (VINTAGE) | 82.50 | 79.65 | 83.37 | 80.28 | 88.32 | 84.75 | 3315–3344 (43%) | 18 |
| MOOD HD (CLEAN) | 91.46 | 97.45 | 93.25 | 97.91 | 99.69 | 95.05 | 5312–5345 (70%) | 21 |

EBR breakdown: program 8, register file 2, ROM 4, plus the ENERGY ring
(VINTAGE 4 at 16 bits, CLEAN 7 at 26 used bits). The first CLEAN cut
missed 74.25 on every seed (65–70 MHz). Its critical path was the coverage
stage (compare/mux → ±W → clamp in one clock). Splitting it, pre-muxing the
colour and splitting the colour multiply by m's bits brought it to 91–96 MHz.
Sync latency: MOOD 12, MOOD HD 16 (`processing_delay_clks`).
