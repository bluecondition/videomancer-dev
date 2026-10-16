# ATTRACT — plan (v2, simplified)

Revised 2026-10-16 after Ron's review of v1:
- reuse the same sprites, tiles and characters, arranged differently for each game style;
- keep the games simple;
- pixelate everything;
- push the limits of what can be stored.

v1's "one vmprog per cabinet" is dropped. **Everything is one program.**

## The idea

ATTRACT works like one 1981 arcade board with one graphics ROM, plus a small CPU running a handful of simple games.
- Every game draws from the **same sheet** of 8×8 patterns, recoloured by palette and flipped.
- A brick, a girder, a maze wall, a cube face, a dirt block and a road stripe are all pieces of the same sheet.
- BLIP and the moths appear in every game.
- K1 picks the game; with auto-cycle on, the games rotate after each GAME OVER behind a burst of channel-change static.

## Canvas: one size, pixelated everywhere

- Every game draws on the same **256×240 canvas** of square pixels, 32×30 tiles of 8×8.
- It is scaled by a whole number per mode and centred, with black at the sides. The canvas never changes with the mode, so there is no per-mode layout work.

| Mode | Pixel size | Picture |
|---|---|---|
| 1080p / 1080i | 4×4 | 1024×960, centred |
| 720p | 3×3 | 768×720 |
| NTSC / PAL / 480p / 576p | 2× wide, 1 line per field (2 lines in 480p/576p) | 512×480, centred |

Three platform facts stay in force:
- **1080p is 30 Hz at most.** The games run 2 logic ticks per frame there, so they keep arcade speed.
- **There is no 240p mode.** SD draws the same line on both fields: a solid 240-line picture with no flicker.
- **HD HDMI bars.** The bar fix is built in. At 4×4 pixels it only rounds the outermost output pixel of each block.

## Memory: spend it on art

The chip has 32 block RAMs of 4 kbit each, 16 KB in all, and nothing else. The board's own needs are kept to the minimum, and **everything left over becomes the graphics ROM**.

| Block | EBR |
|---|---|
| Tilemap: 32×30 cells, tile number + 2-bit palette + H/V flip | 3 |
| Sprite line buffer: two lines of 4-bit pixels | 1 |
| Sprite table: 64 sprites, double-buffered | 1 |
| Palette (32 colours from the 3-3-2 space) + per-row scroll table | 1 |
| Game CPU: program ROM, 2,048 words = 4 KB | 8 |
| Game CPU: work RAM | 1 |
| BLIP's face video thumbnail | 1 |
| **Board total** | **16** |
| **Graphics ROM: 8×8 2bpp patterns** | **12–14** |

The graphics ROM holds **384–448 patterns (6–7 KB)**, about what a 1980 maze-game board had. Tiles use the first 256 patterns; sprites can use any of them.
- A 16×16 sprite is 4 patterns. An 8×8 sprite (shots, POP, popup digits) is 1.
- Bosses and the MOTHER are 2–4 sprites side by side.
- Up to 16 sprites per line. More than that drops the lowest priority, which gives the authentic flicker.

The build reports how full the graphics ROM and program ROM are. Space trades between them: one EBR is 32 more patterns or 256 more words of game code. The aim is to fill both.

Rough split of the sheet:

| Group | Patterns |
|---|---|
| Font (0–9, A–Z, © ! ? - . : and a few symbols), also used for HUD, cards and score popups | ~48 |
| Maze walls (16 joins; style comes from palette and flips) | ~20 |
| Bricks, girders, ladders, conduits, docks | ~30 |
| Road, lanes, water, logs, car bodies | ~30 |
| Dirt, rock, terrain slopes, buildings and windows, cube faces, field blocks | ~50 |
| Dots, power dot, stars, misc | ~20 |
| **Tiles subtotal** | **~200** |
| BLIP: front/back/side walk, jump, climb, power-off death, ship, turret | ~15 sprites |
| Moths: mothling, LUNA, MOTHER, GRUB | ~8 |
| Recoloured enemies (beetle, spark, spool/ball) | ~6 |
| Effects (POP, BOOM, SPARKLE) | ~6 |
| The 8 bonus items | 8 |
| **Sprites subtotal** | **~45 sprites ≈ 180 patterns** |

## Hardware tricks shared by every game

Each is small, and each gives several games their look:
- **Per-row horizontal scroll.** Each of the 30 tile rows slides independently, which makes traffic and river lanes, conveyors and the ripple of a bumped floor.
- **Whole-screen scroll** (X and Y): terrain scrolling and screen shake.
- **2-bit palette per tile, plus flips**: the same brick in four colours, and cube tops that change colour when hopped.
- **Palette cycling.** The CPU rewrites palette entries for glow, shimmer, blinking power dots and the white flash.
- **Starfield**: a procedural pattern behind everything, no memory needed.

## Game logic: one small CPU, ten small programs

- Ten games hardwired in logic would need several chips. Instead, a small CPU runs them from the 4 KB program ROM.
  - It is the same engine family as GAMUT and cubist, already proven on this chip at full HD clock.
  - Estimated ~1,000 LC.
- A shared library does the common work: movement, collisions, score, lives, death and respawn, text cards and the attract cycle.
- Each game is a short program on top of that library: its layout generator plus a few rules.
- The whole board should land around **55–65 % of the logic cells**, leaving headroom.

## The games: same pieces, different arrangement

Each game is deliberately simple: one screen, one rule set, the demo pilot playing until three lives are gone. Same seed (K4) + same stage = same level.

| Game | Arranged from | Rules (simple) |
|---|---|---|
| DOT MATRIX | maze-wall tiles (generated, mirrored), dots | BLIP eats dots; 4 moths chase; power dot turns them pale and edible |
| SKYLINE SIEGE | building + window tiles, ground line | moth formation marches and drops; BLIP turret shoots; buildings chip away tile by tile |
| SWARMATRON | starfield only | moths in a breathing grid peel off and dive on short path tables; BLIP fighter |
| THUMP TOWN | brick floors, conduits | enemies walk the floors; BLIP bumps from below, then kicks the flipped ones off |
| HIGH STEEL | girders, ladders | spools roll down; BLIP climbs and jumps them to reach the top |
| CROSSTALK | road and river rows (per-row scroll) | BLIP hops lanes and rides logs to the docks |
| TANGLE | scattered field blocks | a grub-segment chain snakes down; BLIP shoots from the bottom |
| LOW PASS | terrain columns (whole-screen scroll) | BLIP ship flies the scrolling terrain, bombing tanks and dodging rockets |
| PRISM PEAK | cube faces stacked into a pyramid | BLIP hops diagonally; cube tops change colour; bouncers fall |
| STRATA | dirt tiles | BLIP digs tunnels; grubs follow them; rocks fall |

**Attract cycle per game** (kept short):
1. Title card: name + © 1981 DODECA.
2. READY!
3. Demo play.
4. GAME OVER.
5. Channel-change static to the next game.

HUD: 1UP, HIGH SCORE and lives on every game. Initials entry and high-score tables only if the program ROM has room at the end.

## Order of work

| Step | Delivers | You look at |
|---|---|---|
| 1 — Sheet | the shared sheet as text-grid files + build script → contact sheet PNG (1× and 4×), and a few mock screens (static PNGs composed from the sheet) showing the same pieces arranged as maze / invaders / platform / lanes | look, pixel size and originality, before anything is wired |
| 2 — Board | tiles, sprites, palette, CPU; a flashable test screen with BLIP and the moths walking over a tile layout | flash it |
| 3 — First game + cycle | DOT MATRIX plus the title / GAME OVER / channel-change cycle | flash it |
| 4+ | add games one at a time until the program ROM is full, cheapest reuse first | flash each |

Builds use router1 from the start. `hd_clock_divisor` is not touched.

## Still open (non-blocking)

- **Control map**: the spec's control section never arrived. Only K1 (game), K4 (seed) and S1 (auto-cycle) are fixed so far. I'll propose the rest at step 2 unless you have one.
- **Video section**: BLIP's face showing the input video is budgeted (1 EBR). Any other video or CRT/bezel feature waits for that section.
