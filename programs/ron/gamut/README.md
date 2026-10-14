# GAMUT — palette explorer

Phase 1 delivered the curated library as data; Phase 2 adds the VHDL library view and the DIVE/LIFT transition.

| File | Role |
|---|---|
| `palettes.txt` | The human-readable library: 14 shelves in K3 order, one palette per line in K1 order. Edit this. |
| `gamut.py` | Build hook (runs automatically from `build_programs.sh`). Validates, compiles the ROM package, writes `contact/manifest.json`. `python3 gamut.py --sheets` also renders the contact sheets. |
| `gamut_palettes_pkg.vhd` | Generated ROM constants. Do not edit. |
| `gamut_ref.py` | NumPy reference renderer of the Phase 2 geometry (same integer math as the VHDL). `python3 gamut_ref.py [--gif]` → `contact/ref/` stills of the library at every density, SHAPE extremes, a scroll mid-glide and LIFT at 8 progress values, plus `phase2_sheet.png`. |
| `gamut.vhd` / `gamut.toml` | The program (Phase 2 controls: K1 Browse, K3 Shelf, K4 View, K6 Shape, S7 Glide, S10 Theme, P12 Dive). |
| `contact/` | Review material: `NN_<shelf>_dark/light.png` per shelf, `00_library_*.png` overview, `index.html` (hover for hex / YUV / OKLCH, theme toggle, check report). |

## Source format

```
shelf WHEEL
COMPLEMENT BLUE  | #0E3A8C #3F6FD8 #F2F0EA #F2A33A #B5651D | roles=DSNSA
PLUGE            | yuv(29,512,512) yuv(64,512,512) ...      | hushed src=-4IRE,black,+4IRE
CRT PHOSPHORS    | xy(0.218,0.712) xy(0.523,0.469) ...      | src=P1,P3,...
```

* Names ≤ 16 chars. 2–16 colours per palette. Colour order is the palette's intentional order.
* `#rrggbb` sRGB; `yuv(Y,U,V)` exact 10-bit codes (sub-black allowed); `xy(x,y)` CIE chromaticity shown at its brightest in-gamut luminance (phosphors).
* `roles=` one letter per colour for the WEIGHTED view (D dominant, S secondary, A accent, N neutral); derived from OKLCH chroma when absent.
* `hushed` marks a deliberately low-contrast palette and skips the contrast check.

## Checks (errors fail the build)

* Near-duplicate palettes anywhere in the library: same colour count and every colour within OKLab ΔE 0.045 of a match → **error**.
* Lightness contrast: OKLab L span < 0.22 without `hushed` → warning. Two colours within ΔE 0.02 inside one palette → warning.

## ROM layout (BT.601 10-bit, UNSWAPPED — swap U/V at output compose, per the sim colour convention)

* Colour word, 32 bits: `Y[31:22] U[21:12] V[11:2] role[1:0]`. 985 colours → depth 1024 → 8 EBR.
* Palette word, 16 bits: `[15] hushed, [14:4] first colour offset, [3:0] count−1`. Depth 256 → 1 EBR.
* Shelf word, 16 bits: `[15:8] first palette index, [7:0] palette count`. 14 shelves (fabric constants).

Palette names are in `contact/manifest.json` for a future text layer; they are not in the ROM.

## Phases

1. **Library data** (this) — curate via the contact sheets, then rerun `gamut.py --sheets`.
2. Library view + DIVE (LIFT) with reference renders at several progress values.
3. FOCUS, swatch magnification, CHIP + RAMP studies.
4. GLIDE springs, K1 morph, VIDEO mapping, remaining transitions/views/studies, GENERATE, text.

## Phase 2 rendering architecture

The first VHDL cut (one state per operation, each with its own adders) synthesised to 152 % of the HX4K. The shipped structure is a microcoded one:

* **`gamut_engine.vhd`** — one 20-bit accumulator engine: LD/ADD/SUB/MIN/MAX (+immediate forms), ST, MUL (serial 16×16, ~17 cycles), jumps, ABS, LOG2. Four-stage pipeline (fetch, file address, operand, execute). Operands come from a 128-entry **register file in EBR** (no mux) or a ~40-way external-source mux (controls, measured W/H, ROM data, multiplier result). Program ROM: 1024 × 24 in EBR, frame program at 0, line program at 768.
* **`gamut_asm.py`** — the assembler: symbolic sources/dests, labels, automatic NOP padding for store→read hazards (distance 3) and MUL→MRES reads. Emits `gamut_prog_pkg.vhd` (program, source/dest index constants, register-file init, copy list). Run by `gamut.py` at build time.
* **Frame program** (runs at vsync, ~1000 cycles): K1 glide + 10 % hysteresis cursor, density constants (k/256 of W,H), scroll spring (rows×256, `v += (t-p)>>5 - v>>2`), dive spring with dwell zones, smoothstep, hero rectangle lerped toward the screen, per-column evaluator constants, squared radii / AA shift amounts / shadow spreads (theme strength folded into the shift), dominant-colour tint, carousel slide-and-fade, row-walker start, backdrop centre squares. Stores land in the file and directly in the named FFs the pixel path reads.
* **Line program** (runs during each line for the NEXT line, ~300 cycles; also at hsync during vblank): row walker, recede inset / dim, card y edges with sub-pixel fraction, four EY² terms via the square ROM, palette lookups per column. Results stay in the file; a **copy sequencer** moves 20 entries into the named FFs at each hsync, so the budget is a whole line, not the hblank (SD-safe).
* **Pixel path** (14 stages): list card from per-frame column constants + per-row inset, hero from per-frame constants; `EX = 2cx-1`, `EX²` (square ROM for the list, multiplier for the hero), `D² = EX²+EY²`, differences against `R²`/ring/shadow radii, shift-based saturating ramps (AA 0..16, shadow 0..64), one shared colour-ROM port (hero owns it where it covers; address delayed so the colour lands at the blend), **one blend**: (over, cov) = hero | ring | list×dim over (list if fully covered | shadowed backdrop). Receding/fading cards blend toward the backdrop.
* **Backdrop**: radial lift from running `dx²` (per pixel) + `dy²` (per line walker), 10-bit luma with a 2-bit LFSR dither, chroma tinted toward the cursor palette's dominant colour.
* Latency: 14-stage pipe, s_io at 15, 1 pad → `processing_delay_clks = 16`. EBR: colour 8, palette 1, squares 6, program 6, register file 2 = 23.

## Build record

v0.2.0 (2026-10-14), `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron gamut`, ~6930 LC (90 %), 23 EBR:

| Config | Seed | Fmax |
|---|---|---|
| HD Analog | 2 | 76.95 MHz |
| SD Analog | 1 | 72.08 MHz |
| HD HDMI | 2 | 74.58 MHz |
| SD HDMI | 1 | 74.76 MHz |
| HD Dual | 1 | 75.57 MHz |
| SD Dual | 1 | 73.83 MHz |

Timing history: the first cut (one state per op) was 152 % of the chip; the engine rewrite reached 90 % but HD sat at ~64 MHz with the critical path in the engine's execute stage, then in its external-source mux feeding the shifter; registering the mux output (the D1/D2 split) closed all six. Router2 silently stalls on this netlist (overused=2 for 400k+ iterations): use router1. Not yet flashed.

Not in Phase 2 yet: scroll track / pagination dots (reference renderer only), K2/K5/S8/S9/S11 controls, bar gutters at high SHAPE, generative shelves, hero shadow on neighbouring cards (shadows fall on the backdrop only).
