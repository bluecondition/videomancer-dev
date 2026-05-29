-- 8x16 1-bit sprite: wand-shot projectile
-- 1 animation frame(s). Row 0 = top scanline.
-- Bit 7 = leftmost pixel, bit 0 = rightmost. '1' = lit.

type WAND_SHOT_rom_t is array (0 to 15) of std_logic_vector(7 downto 0);

constant WAND_SHOT : WAND_SHOT_rom_t := (
    0 => "...11...",
    1 => "..1111..",
    2 => "..1111..",
    3 => "...11...",
    4 => "...11...",
    5 => "..1..1..",
    6 => ".1....1.",
    7 => "..1..1..",
    8 => "...11...",
    9 => "..1..1..",
   10 => ".1....1.",
   11 => "..1..1..",
   12 => "...11...",
   13 => "..1111..",
   14 => "..1111..",
   15 => "...11..."
);
