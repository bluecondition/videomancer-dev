# Wizard Space Invaders — Sprite Package

1-bit sprite ROMs for a Space Invaders-style game on the **LZX Videomancer**
(FPGA video processor, programmed in VHDL).

## What's in the package

| File | Sprite | Size | Frames |
|------|--------|------|--------|
| `wizard_invaders_sprites.vhd` | **Combined package — use this one** | — | all |
| `wizard_player.vhd` | Wizard (player ship) | 128×128 | 1 |
| `alien_ghost.vhd` | Scanline ghost | 48×48 | 2 |
| `alien_mite.vhd` | Static mite | 48×48 | 2 |
| `alien_worm.vhd` | Tracking worm | 48×48 | 2 |
| `alien_crawler.vhd` | Glitch crawler | 48×48 | 2 |
| `alien_gremlin.vhd` | Color-bar gremlin | 48×48 | 2 |
| `wand_shot.vhd` | Wand-shot projectile | 8×16 | 1 |
| `bunker.vhd` | Videomancer bunker (destructible) | 128×112 | 1 |
| `explosion.vhd` | Explosion burst | 48×48 | 4 |

`wizard_invaders_sprites.vhd` contains every sprite in a single VHDL package
with shared array types. The individual files are the same data on their own,
in case you'd rather include them separately.

## Sprite format

Every sprite is strictly **1-bit**: a bit value of `1` is a lit pixel, `0`
is transparent (let the background show through).

- Row 0 is the **top** scanline.
- In each row vector the **MSB is the leftmost** pixel, **bit 0 the
  rightmost**.
- Sizes use shared types: `sprite128x128_t`, `sprite48x48_t`,
  `sprite8x16_t`, `sprite128x112_t`.

## Drawing a sprite

For a sprite of width `W`, height `H`, drawn with its top-left corner at
screen position `(ox, oy)`, test the current scan position `(sx, sy)`:

```vhdl
if  sx >= ox and sx < ox + W
and sy >= oy and sy < oy + H then
    bit := SPRITE( sy - oy )( (W-1) - (sx - ox) );
    if bit = '1' then
        -- draw the sprite colour here
    else
        -- transparent: pass the background through
    end if;
end if;
```

This is the same approach as the `pong` example in the Videomancer SDK.

## Animation

Sprites with multiple frames expose them as `_F0`, `_F1`, … constants
(e.g. `GHOST_F0`, `GHOST_F1`). Drive the frame index from a slow
free-running counter:

- **Aliens** (2 frames): toggle F0/F1 on the fleet step for the classic
  marching wobble.
- **Explosion** (4 frames): play F0→F1→F2→F3 once when something is hit,
  then stop drawing it.

## Destructible bunkers

`VIDEOMANCER_BUNKER` is a ROM, so it can't be modified in place. To make the
bunkers erode:

1. At level start, copy `VIDEOMANCER_BUNKER` into a RAM-backed array
   (block RAM) — one copy per bunker on screen.
2. On a shot collision, write `'0'` into the RAM bits around the hit
   point. The hole appears and the bunker visibly degrades.
3. Draw the bunkers from RAM instead of the ROM constant.

The ROM constant is the pristine starting state to initialise that RAM from.

## Notes

- The wizard sprite already has the white sticker border from the source
  art removed — it keys off the wizard's own black outline.
- The wand-shot should spawn from a fixed offset near the wizard's raised
  wand hand (upper-right of the 128×128 cell), not from the sprite centre.
- All sprites are white-on-transparent; invert the bits if your keying
  setup needs a black matte instead.
