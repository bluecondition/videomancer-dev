# ATTRACT

An arcade that plays itself. A 1981-style board runs fictional **DODECA** coin-ops in attract mode forever. BLIP, a little walking TV, plays every game, and the moths hunt him in every one.

Every game is drawn from **one shared sheet** of 8×8 pixel art. A brick, a girder, a maze wall and a road stripe are all pieces of the same sheet, recoloured by palette and flipped. The board has:
- a tile layer with per-row scroll;
- a 64-sprite engine (16 per line, overflow drops the rest);
- a palette of 3-3-2 colours;
- a little game CPU running the games from its own ROM.

The picture is a 256×240 canvas of big square pixels, centred on any video mode: 4×4 at 1080, 3×3 at 720p, line-doubled at SD.

## Games (v0.3.0)

| Game | Genre | The same pieces, arranged as… |
|---|---|---|
| DOT MATRIX | maze chase | a mirrored maze built by a random walk; four moths with their own chase personalities; power dots dazzle them |
| SKYLINE SIEGE | marching invaders | a moth formation over a tile city that erodes, colour bands like cellophane overlays, a MOTHER flyover |
| CROSSTALK | lane crosser | road and river lanes slid by the per-row scroll hardware; BLIP hops, rides logs and fills the docks |

## Controls

| Control | Function |
|---|---|
| K1 Game | **Cycle** (a new game after each GAME OVER), or pin one game |
| K4 Seed | level seed: the same knob position and stage always give the same level |
| S11 Attract | **Attract**: channel-change static, RAM OK / ROM OK, title card, READY!, demo, GAME OVER. **Endless**: gameplay only |

© 1981 DODECA.
