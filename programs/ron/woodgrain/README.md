# Woodgrain — an Atari 2600 inside the Videomancer

Woodgrain is a cycle-exact Atari 2600 built in the FPGA. It has a 6507 CPU, the TIA video chip,
the RIOT (RAM, timer and I/O) and a 4K cartridge ROM. The console runs one game frame per output
field. Its 160 × 216 picture is scaled to a TV-shaped window on every raster.

## Cartridges

The build hook (`woodgrain.py`) picks the cartridge:

1. `cartridge.bin` in this folder, if it exists. Any 2K or 4K 2600 ROM works (2K images are
   mirrored). Use only a ROM you have the right to use, for example a dump of a cartridge you
   own. `cartridge.bin` and the generated `woodgrain_cart_pkg.vhd` are gitignored, so a
   personal dump never gets committed. The flashed `.vmprog` contains the ROM, so don't share
   it.
2. Otherwise `roms/rooms.bin`, the bundled cartridge. It is **ROOMS**, the procedurally generated
   room explorer `procgen1.a` from Steven Hugg's
   [8bitworkshop](https://github.com/sehugg/8bitworkshop) VCS examples, licensed CC0. You walk
   off any edge of a walled room into the next random room. Each room has its own colours and a
   Green Man. `roms/rooms.a` is that source with its include files replaced by inline equates.
   `python3 ../tools/asm6502.py rooms.a rooms.bin` rebuilds it byte-identically.

After changing the cartridge, rebuild with `./build_programs.sh ron woodgrain`.

Not supported yet: bank-switched cartridges (8K and larger: F8, F6, ...), paddles and sound.
The Videomancer has no audio output.

## Controls

| Control | Function |
|---|---|
| K1 Stick X | Joystick left/right: the outer thirds push the stick (with hysteresis) |
| K2 Stick Y | Joystick down/up (clockwise = up) |
| S7 Fire | Joystick button, held while the switch is on |
| S8 Reset | Each flip, either way, presses **Game Reset** for 8 fields |
| S9 Select | Each flip presses **Game Select** |
| S10 Difficulty | Left difficulty switch, B / A |
| S11 Auto | While K1/K2 rest in their middle thirds, a wandering joystick plays (a new heading every ~0.6–1 s) |

## Picture on each raster

| Raster | Atari pixel | Picture |
|---|---|---|
| 1080p (24–30 Hz) | 9 clocks × 5 rows | 1440 × 1080. Each output frame runs one shown and one hidden game frame, so games keep their speed |
| 1080i | 9 clocks × 2.5 field rows (3, 2, 3, 2 …) | 1440 × 1080 |
| 720p | 5 × 3 | 800 × 648 (90% of the height) |
| 480p / 576p | 4 × 2 | 640 × 432 |
| 480i / 576i | 4 × 1 field row | 640 × 216 field rows |

The window shows game lines 25–240 after the VSYNC-off line. That is the usual 192-line picture
with a 12-line margin. On PAL the picture is 75% of the height, as an NTSC cartridge on a PAL TV
would be.

**1080p limitation.** 1080p only shows every second game frame. A game that multiplexes sprites
on alternate frames (flicker) shows only one of the two sets there. Every other raster shows
every frame.

## Architecture

- **`woodgrain_cpu.vhd`** — a bus-cycle-exact 6502. One CPU cycle is one 6-clock **slot**:
  - ph0–ph2: the access happens;
  - ph3: the state machine steps;
  - ph4–ph5: a 2-stage nibble ALU (decimal mode included).

  It handles every documented opcode and the stable illegal ones (LAX SAX DCP ISB SLO RLA SRE
  RRA ANC ALR ARR SBX LXA, multi-byte NOPs). Decoding uses a 256 × 32 EBR ROM generated from
  `tools/isa6502.py`.
- **`woodgrain_vcs.vhd`** — the console:
  - **Timing:** a slot's three color clocks belong to CPU cycle n+1. Reads therefore see the TIA
    as Stella does, and writes land before clock 3n+3+d, with d = 0/1/2/6 per register as in
    Stella's delay table. TIA registers change only on even phases, and color clocks tick only
    on odd ones.
  - **Objects:** Stella's counter model. Object counters run 0..159 and tick on visible clocks
    and HMOVE pulses. Draws start at decodes 156/12/28/60, with first pixels 5 clocks later for
    players and 4 for missiles and the ball.
  - **Not modelled:** starfield and late-HMOVE quirks, NUSIZ changes mid-draw.
- **Pacing.** Unpaced, the console runs ~1.9× real time at SD and far more at HD. It stops at
  the game's VSYNC until the next output field starts. It also stops before a line that would
  overwrite an 8-line ring-buffer slot the display hasn't read yet.
- **`woodgrain.vhd`**:
  - mode table from the timing ID;
  - window and column counters;
  - ring buffer and palette ROM (Stella's NTSC palette as 10-bit YUV, hardware U/V convention);
  - generated avid (cubist recipe);
  - HD HDMI bar fix: [1 2 1] on Y/U/V, luma ×13/16;
  - blanking gate.

  Latency is 8 clocks in every mode.

EBR: cartridge 8, decode 2, RAM 1, ring 4, palette 2 (17 of 32).

## Verification

| Test | Result |
|---|---|
| CPU fuzz (`.woodgrain_work/cputest`): 12 seeds × 30,000 instructions of random code in random 64K memory vs py65 | Identical PC, A, X, Y, S, P and cycle counts at every instruction, including 13k decimal-mode ADC/SBC (py65's `DEC abs` cycle count, 3, is wrong; patched to 6) |
| Console vs reference model (`tools/vcs_model.py`, py65 + Python TIA/RIOT), joystick held right | ROOMS frames match pixel for pixel, frame by frame |
| 720p image-tester sim of the whole program | Centred 800 × 648 room |

Hardware status: **timing-closed only, not flashed**.

## Build

Build with **router1**: `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron woodgrain`.
router2 missed or stalled on all six HD HDMI seeds.

v0.1.0 (2026-10-27), 4,412–4,440 LC (58%), 17/32 EBR:

| Config | Seed | Fmax |
|---|---|---|
| HD Analog | 1 | 83.8 MHz |
| HD HDMI | **2** | 82.6 MHz (seed 1: 74.23) |
| HD Dual | 1 | 83.6 MHz |
| SD Analog | 1 | 75.9 MHz |
| SD HDMI | 1 | 83.1 MHz |
| SD Dual | 1 | 79.0 MHz |

The critical path runs from the slot-phase decode through the HMOVE apply / pulse logic to the
TIA object register enables. It is about 80% routing. If timing ever needs margin:

- register the slot-phase flags one-hot;
- precompute each object's HMOVE stop compare on the even phase before each color clock.

## Tools

| Tool | Purpose |
|---|---|
| `tools/isa6502.py` | Opcode table |
| `tools/asm6502.py` | Small dasm-compatible assembler |
| `tools/vcs_model.py` | Reference 2600 model, renders PNGs: `python3 tools/vcs_model.py rom.bin out.png --frames 60 --joy R` (needs `py65` and `pillow`) |
| `tools/gen_tables.py` | Decode ROM and palette generator (run by the hook) |
