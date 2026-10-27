-- gilt.vhd
--
-- GILT v3 -- live video re-rendered as Klimt gold paint, swimming in oil.
--
-- Two halves, each taken from a version Ron approved:
--
--   THE OIL (from v0.10, "I like the organic look now"). A blurred content
--   field of the incoming picture (~8 px x ~4 lines, luma and both chroma
--   axes) DRAGS the ornament space: luma pulls it sideways, chroma pulls it
--   up and down, so the patterns pile up and stretch around the picture's
--   own shapes. Two smooth parabolic-sine waves and a large per-line sway
--   bend it further and drift with time. P12 SWIRL sets all of it, from
--   flat (0) to fully melted (100%).
--
--   THE TEXTURES (sugarcoat's mechanism, Klimt's vocabulary). Every pixel is
--   CLASSIFIED -- by its luma band (Luma map) or its hue octant (Chroma map,
--   with neutral pixels falling back to luma bands) -- and a THEME table
--   turns the class into one of 16 procedural ornament textures: dots,
--   circles-in-circles, spirals, hollow squares, checker, eyes, colour blobs,
--   tall bars, mosaic, oil marble, chevrons, rosettes, triangles, gold
--   flakes, waves, plain leaf. All are evaluated in the WARPED space, so the
--   ornament swims while its boundaries follow the picture.
--
--   The ground under every texture is the GOLD-PAINTED PICTURE: per-pixel
--   luma through v0.10's 32-step gold/brown ramp, with leaf grain. The
--   textures' inks stay on that ramp (v3.1, "where's the brown and gold
--   tones?"): a dark ink is the paint moved INK steps down toward umber,
--   a light ink the same distance up toward bright gold. Only the jewel
--   accents leave the gold/brown family.
--
-- Controls: K1 Theme (16) | K2 Poster | K3 Scale | K4 Contrast | K5 Ink |
--   K6 Density | S7 Map | S8 Flow | S9 Invert | S10 Outline | S11 Jewels |
--   P12 Swirl.
--
-- Pipeline (C_LATENCY = 16, one latency in every mode):
--   field (free-running): IIR blur -> drives, contrast -> class -> theme ROM
--     -> texture index s_tex, outline flag s_edge; waves -> s_warpx/s_warpy
--   E1 latch | E2 warp + paint contrast | E3 row stagger, paint clamp, grain
--   hash | E4 cell slices, paint posterize | E5 cell hashes, folded offsets |
--   E6 squares, max/min, sector | E7 metrics | E8 16 texture rules | E9
--   texture mux + paint ROM | E10 ramp read + jewel | E11 grain, jewel-gold |
--   E12 x Ink | E13 add | E14 outline | E15 [1 2 1] | E16 luma scale + blanking gate
--
-- Hardware contracts: U/V swapped at the field input only (the output goes
-- out in standard order since v3.1.1 -- see E16); the
-- output avid is GENERATED (parity-locked start, field-latched width) and
-- the blanking gate follows it; HD HDMI bar conditioning ([1 2 1] on Y/U/V,
-- luma scaled to white ~775) before the gate; serration-safe field event;
-- frame-line texture coordinates under interlace. No raw video reaches the
-- output.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture gilt of program_top is

    constant C_LATENCY : integer := 16;
    -- extra paint delay so the sharp paint lines up with the texture
    -- regions, which come out of the blurred (lagging) field
    constant C_PDLY    : integer := 6;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_cu10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    -- |signed| clamped into 0..255 (u8)
    function f_abs8(v : signed) return unsigned is
        variable t : signed(v'length downto 0);
    begin
        t := resize(v, v'length + 1);
        if t < 0 then
            t := -t;
        end if;
        if t > 255 then
            t := to_signed(255, t'length);
        end if;
        return resize(unsigned(t), 8);
    end function;

    -- one-pole IIR on an 8-bit content-field value
    function f_iir(cur, tgt : unsigned(7 downto 0); sh : integer)
        return unsigned is
        variable d, r : signed(9 downto 0);
    begin
        d := resize(signed('0' & tgt), 10) - resize(signed('0' & cur), 10);
        r := resize(signed('0' & cur), 10) + shift_right(d, sh);
        return unsigned(r(7 downto 0));     -- always in 0..255: slice, don't resize
    end function;

    -- SWIRL drag scaler: 0, 1/8, 1/4, 3/8, 1/2, 3/4, 1, 2 (shift-adds)
    function f_wscale(d : signed(10 downto 0); lvl : unsigned(2 downto 0))
        return signed is
        variable r : signed(10 downto 0);
    begin
        case to_integer(lvl) is
            when 0      => r := (others => '0');
            when 1      => r := shift_right(d, 3);
            when 2      => r := shift_right(d, 2);
            when 3      => r := shift_right(d, 2) + shift_right(d, 3);
            when 4      => r := shift_right(d, 1);
            when 5      => r := shift_right(d, 1) + shift_right(d, 2);
            when 6      => r := d;
            when others => r := shift_left(d, 1);
        end case;
        return r;
    end function;

    -- per-frame shift (15 = off)
    function f_shr(v : signed; s : integer) return signed is
        variable r : signed(v'length - 1 downto 0);
    begin
        if s >= 15 then
            r := (others => '0');
        else
            r := shift_right(v, s);
        end if;
        return r;
    end function;

    -- parabolic SINE: 32-entry hump, period 64 in t (v0.10)
    type t_hump is array (0 to 31) of unsigned(8 downto 0);
    constant C_HUMP : t_hump := (
        to_unsigned(  0,9), to_unsigned( 31,9), to_unsigned( 60,9), to_unsigned( 87,9),
        to_unsigned(112,9), to_unsigned(135,9), to_unsigned(156,9), to_unsigned(175,9),
        to_unsigned(192,9), to_unsigned(207,9), to_unsigned(220,9), to_unsigned(231,9),
        to_unsigned(240,9), to_unsigned(247,9), to_unsigned(252,9), to_unsigned(255,9),
        to_unsigned(256,9), to_unsigned(255,9), to_unsigned(252,9), to_unsigned(247,9),
        to_unsigned(240,9), to_unsigned(231,9), to_unsigned(220,9), to_unsigned(207,9),
        to_unsigned(192,9), to_unsigned(175,9), to_unsigned(156,9), to_unsigned(135,9),
        to_unsigned(112,9), to_unsigned( 87,9), to_unsigned( 60,9), to_unsigned( 31,9));

    function f_psin(t : unsigned(5 downto 0)) return signed is
        variable h : signed(9 downto 0);
    begin
        h := signed(resize(C_HUMP(to_integer(t(4 downto 0))), 10));
        if t(5) = '1' then
            return -h;
        else
            return h;
        end if;
    end function;

    -- value-noise hashes (v0.10, both with an integer add for mixing)
    function f_hash_a(cx, cy : unsigned(7 downto 0);
                      seed   : unsigned(15 downto 0)) return unsigned is
        variable h16 : unsigned(15 downto 0);
        variable h   : unsigned(11 downto 0);
    begin
        h16 := (cx & cy) xor seed;
        h16 := h16 xor (x"0" & h16(15 downto 4));
        h   := h16(11 downto 0);
        return h + rotate_left(h, 5);
    end function;

    function f_hash_b(hin : unsigned(11 downto 0)) return unsigned is
        variable h : unsigned(11 downto 0);
    begin
        h := hin xor rotate_left(hin, 9);
        h := h + rotate_left(h, 3);
        h := h xor ("00000" & h(11 downto 5));
        return h;
    end function;

    -- fold a 6-bit in-cell coordinate to its distance from the centre (0..31)
    function f_fold(l : unsigned(5 downto 0)) return unsigned is
    begin
        if l(5) = '1' then
            return l(4 downto 0);
        else
            return not l(4 downto 0);
        end if;
    end function;

    -- squared-distance LUT, 32 entries (logic ROM)
    type t_sq is array (0 to 31) of unsigned(9 downto 0);
    constant C_SQ : t_sq := (
        to_unsigned(  0, 10), to_unsigned(  1, 10), to_unsigned(  4, 10), to_unsigned(  9, 10),
        to_unsigned( 16, 10), to_unsigned( 25, 10), to_unsigned( 36, 10), to_unsigned( 49, 10),
        to_unsigned( 64, 10), to_unsigned( 81, 10), to_unsigned(100, 10), to_unsigned(121, 10),
        to_unsigned(144, 10), to_unsigned(169, 10), to_unsigned(196, 10), to_unsigned(225, 10),
        to_unsigned(256, 10), to_unsigned(289, 10), to_unsigned(324, 10), to_unsigned(361, 10),
        to_unsigned(400, 10), to_unsigned(441, 10), to_unsigned(484, 10), to_unsigned(529, 10),
        to_unsigned(576, 10), to_unsigned(625, 10), to_unsigned(676, 10), to_unsigned(729, 10),
        to_unsigned(784, 10), to_unsigned(841, 10), to_unsigned(900, 10), to_unsigned(961, 10));

    ----------------------------------------------------------------------
    -- palettes (standard BT.601 10-bit; U/V swapped only at the edges)
    ----------------------------------------------------------------------
    -- the gold paint: v0.10's 32-entry gold/brown ramp, dark -> light
    type t_ramp is array (0 to 31) of unsigned(9 downto 0);
    constant C_RAMP_Y : t_ramp := (
        to_unsigned(  40,10), to_unsigned(  94,10), to_unsigned( 135,10),
        to_unsigned( 184,10), to_unsigned( 228,10), to_unsigned( 273,10),
        to_unsigned( 318,10), to_unsigned( 361,10), to_unsigned( 402,10),
        to_unsigned( 440,10), to_unsigned( 476,10), to_unsigned( 511,10),
        to_unsigned( 546,10), to_unsigned( 579,10), to_unsigned( 612,10),
        to_unsigned( 644,10), to_unsigned( 676,10), to_unsigned( 703,10),
        to_unsigned( 729,10), to_unsigned( 755,10), to_unsigned( 780,10),
        to_unsigned( 804,10), to_unsigned( 825,10), to_unsigned( 845,10),
        to_unsigned( 865,10), to_unsigned( 885,10), to_unsigned( 903,10),
        to_unsigned( 913,10), to_unsigned( 924,10), to_unsigned( 934,10),
        to_unsigned( 945,10), to_unsigned( 955,10));
    constant C_RAMP_U : t_ramp := (
        to_unsigned( 505,10), to_unsigned( 471,10), to_unsigned( 448,10),
        to_unsigned( 426,10), to_unsigned( 405,10), to_unsigned( 385,10),
        to_unsigned( 366,10), to_unsigned( 347,10), to_unsigned( 329,10),
        to_unsigned( 317,10), to_unsigned( 306,10), to_unsigned( 294,10),
        to_unsigned( 284,10), to_unsigned( 281,10), to_unsigned( 277,10),
        to_unsigned( 274,10), to_unsigned( 270,10), to_unsigned( 276,10),
        to_unsigned( 282,10), to_unsigned( 289,10), to_unsigned( 295,10),
        to_unsigned( 304,10), to_unsigned( 320,10), to_unsigned( 336,10),
        to_unsigned( 352,10), to_unsigned( 368,10), to_unsigned( 383,10),
        to_unsigned( 396,10), to_unsigned( 408,10), to_unsigned( 421,10),
        to_unsigned( 433,10), to_unsigned( 445,10));
    constant C_RAMP_V : t_ramp := (
        to_unsigned( 520,10), to_unsigned( 550,10), to_unsigned( 570,10),
        to_unsigned( 587,10), to_unsigned( 602,10), to_unsigned( 615,10),
        to_unsigned( 626,10), to_unsigned( 636,10), to_unsigned( 645,10),
        to_unsigned( 649,10), to_unsigned( 653,10), to_unsigned( 657,10),
        to_unsigned( 660,10), to_unsigned( 659,10), to_unsigned( 657,10),
        to_unsigned( 656,10), to_unsigned( 655,10), to_unsigned( 650,10),
        to_unsigned( 644,10), to_unsigned( 638,10), to_unsigned( 632,10),
        to_unsigned( 626,10), to_unsigned( 617,10), to_unsigned( 609,10),
        to_unsigned( 600,10), to_unsigned( 591,10), to_unsigned( 583,10),
        to_unsigned( 577,10), to_unsigned( 571,10), to_unsigned( 564,10),
        to_unsigned( 558,10), to_unsigned( 552,10));

    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    -- JEWEL inks: crimson, vermilion, orange, amber | viridian, lapis,
    -- silver-blue, rose (0-3 = the warm set, 4-7 = the cool set)
    constant C_JW_Y : t_pal := (
        to_unsigned(312,10), to_unsigned(428,10), to_unsigned(564,10), to_unsigned(635,10),
        to_unsigned(416,10), to_unsigned(311,10), to_unsigned(600,10), to_unsigned(547,10));
    constant C_JW_U : t_pal := (
        to_unsigned(445,10), to_unsigned(324,10), to_unsigned(226,10), to_unsigned(211,10),
        to_unsigned(469,10), to_unsigned(743,10), to_unsigned(596,10), to_unsigned(485,10));
    constant C_JW_V : t_pal := (
        to_unsigned(834,10), to_unsigned(854,10), to_unsigned(777,10), to_unsigned(665,10),
        to_unsigned(332,10), to_unsigned(412,10), to_unsigned(446,10), to_unsigned(745,10));
    -- outline ink (warm near-black)
    constant C_INK_Y : unsigned(9 downto 0) := to_unsigned( 30, 10);
    constant C_INK_U : unsigned(9 downto 0) := to_unsigned(506, 10);
    constant C_INK_V : unsigned(9 downto 0) := to_unsigned(520, 10);

    ----------------------------------------------------------------------
    -- LUMA BAND ROM: {N code, luma(7:2)} -> class 0..7. N bands (2..8)
    -- spread over the whole class range, so the darkest and brightest
    -- textures of a theme are always in use.
    ----------------------------------------------------------------------
    type t_lrom is array (0 to 511) of unsigned(2 downto 0);
    function f_mk_lrom return t_lrom is
        variable r : t_lrom;
        variable n, y8, b, c : integer;
    begin
        for code in 0 to 7 loop
            n := code + 2;
            if n > 8 then
                n := 8;
            end if;
            for y6 in 0 to 63 loop
                y8 := y6 * 4 + 2;
                if y8 < 16 then
                    b := 0;
                else
                    b := ((y8 - 16) * n) / 224;
                end if;
                if b > n - 1 then
                    b := n - 1;
                end if;
                c := (b * 14 + (n - 1)) / (2 * (n - 1));   -- round(b * 7 / (n-1))
                r(code * 64 + y6) := to_unsigned(c, 3);
            end loop;
        end loop;
        return r;
    end function;
    constant C_LROM : t_lrom := f_mk_lrom;

    ----------------------------------------------------------------------
    -- THEME ROM: 16 themes x 16 classes -> texture. Per theme: entries 0-7
    -- = luma bands dark -> bright; 8-15 = hue octants green, olive, cyan,
    -- blue, red, amber, pink, violet (quadrant mode uses 8/10/12/14, half
    -- mode 8/12). Amber (skin) stays plain gold paint in every theme.
    -- Textures: 0 leaf 1 dots 2 rings 3 spiral 4 squares 5 checker 6 eyes
    -- 7 blobs 8 bars 9 mosaic A marble B chevron C rosette D triangle
    -- E flakes F waves.
    ----------------------------------------------------------------------
    type t_tmap is array (0 to 255) of std_logic_vector(3 downto 0);
    constant C_TMAP : t_tmap := (
        x"9", x"5", x"4", x"6", x"3", x"2", x"1", x"0", x"3", x"1", x"2", x"6", x"4", x"0", x"7", x"6",  -- Adele
        x"5", x"8", x"9", x"7", x"2", x"C", x"3", x"E", x"C", x"7", x"2", x"8", x"5", x"0", x"C", x"8",  -- Kiss
        x"A", x"3", x"D", x"6", x"3", x"1", x"3", x"0", x"3", x"3", x"F", x"6", x"D", x"0", x"C", x"3",  -- Tree of Life
        x"A", x"F", x"7", x"A", x"2", x"F", x"E", x"0", x"F", x"A", x"F", x"A", x"7", x"0", x"7", x"2",  -- Danae
        x"B", x"D", x"5", x"4", x"6", x"C", x"1", x"E", x"C", x"1", x"B", x"6", x"4", x"0", x"C", x"D",  -- Stoclet
        x"8", x"5", x"D", x"B", x"4", x"9", x"1", x"0", x"D", x"B", x"8", x"5", x"4", x"0", x"9", x"8",  -- Frieze
        x"9", x"4", x"9", x"7", x"9", x"2", x"E", x"0", x"9", x"9", x"9", x"9", x"4", x"0", x"7", x"9",  -- Mosaic
        x"A", x"F", x"3", x"A", x"7", x"F", x"A", x"0", x"A", x"F", x"A", x"F", x"7", x"0", x"7", x"A",  -- Oil
        x"3", x"A", x"3", x"2", x"3", x"1", x"3", x"E", x"3", x"3", x"3", x"3", x"3", x"0", x"3", x"3",  -- Spirals
        x"6", x"5", x"6", x"1", x"6", x"2", x"6", x"0", x"6", x"1", x"6", x"6", x"2", x"0", x"6", x"6",  -- Eyes
        x"1", x"2", x"1", x"E", x"7", x"1", x"E", x"0", x"1", x"E", x"2", x"1", x"7", x"0", x"7", x"1",  -- Dots
        x"4", x"5", x"4", x"9", x"8", x"4", x"1", x"0", x"4", x"9", x"4", x"8", x"4", x"0", x"5", x"4",  -- Squares
        x"C", x"7", x"C", x"1", x"C", x"E", x"C", x"0", x"C", x"1", x"C", x"7", x"C", x"0", x"C", x"C",  -- Garden
        x"9", x"B", x"D", x"9", x"4", x"E", x"9", x"E", x"D", x"B", x"9", x"9", x"4", x"0", x"D", x"B",  -- Byzantine
        x"7", x"9", x"7", x"4", x"7", x"C", x"7", x"E", x"7", x"9", x"7", x"7", x"4", x"0", x"7", x"7",  -- Jewels
        x"5", x"9", x"4", x"3", x"6", x"2", x"1", x"E", x"C", x"F", x"A", x"8", x"D", x"0", x"7", x"B"   -- All
    );

    -- mosaic tile kinds (0 gold, 1 pale, 2 jewel, 3 dark)
    type t_mosk is array (0 to 15) of integer range 0 to 3;
    constant C_MOSK : t_mosk := (0, 1, 0, 2, 1, 0, 0, 1, 0, 3, 1, 0, 2, 0, 1, 0);

    -- ink selects
    constant C_SG : std_logic_vector(1 downto 0) := "00";   -- ground (gold paint)
    constant C_SD : std_logic_vector(1 downto 0) := "01";   -- dark ink
    constant C_SL : std_logic_vector(1 downto 0) := "10";   -- light ink
    constant C_SC : std_logic_vector(1 downto 0) := "11";   -- jewel

    -- chroma-map saturation floor (|du|+|dv| in 8-bit units, after contrast)
    constant C_SATF : unsigned(7 downto 0) := to_unsigned(20, 8);

    ----------------------------------------------------------------------
    -- controls (field-latched) + per-frame terms
    ----------------------------------------------------------------------
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6, s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_map   : std_logic := '0';     -- S7: 0 Luma, 1 Chroma
    signal s_flow  : std_logic := '0';     -- S8: swap the drag axes
    signal s_inv   : std_logic := '0';     -- S9: gold negative
    signal s_outl  : std_logic := '1';     -- S10: outline
    signal s_jewel : std_logic := '1';     -- S11: jewel colours
    signal s_uvsw  : std_logic := '0';     -- v3.1.2 diag (S11): 1 = swap U/V out

    signal s_seq    : unsigned(3 downto 0) := (others => '0');
    signal s_theme  : unsigned(3 downto 0) := (others => '0');
    signal s_gain   : integer range 0 to 7 := 4;
    signal s_pc     : unsigned(2 downto 0) := (others => '0');
    signal s_lpn    : unsigned(2 downto 0) := to_unsigned(4, 3);    -- N code (N - 2)
    signal s_pqb    : integer range 0 to 4 := 1;                    -- paint quant bits
    signal s_hmask  : std_logic_vector(1 downto 0) := "11";
    signal s_k      : integer range 4 to 7 := 6;                    -- pitch 2^k
    signal s_ink_t  : unsigned(4 downto 0) := to_unsigned(12, 5);   -- /16
    signal s_dens   : unsigned(7 downto 0) := to_unsigned(160, 8);
    signal s_lvl    : unsigned(2 downto 0) := "101";                -- swirl level
    signal s_wsa, s_wsb : integer range 0 to 15 := 15;              -- wave shifts
    signal s_ssa, s_ssb : integer range 0 to 15 := 15;              -- sway shifts
    signal s_drift  : unsigned(10 downto 0) := (others => '0');
    signal s_dph    : unsigned(5 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- position / line / field bookkeeping
    ----------------------------------------------------------------------
    signal s_prev_hsync_n, s_prev_vsync_n : std_logic := '1';
    signal s_hc     : unsigned(11 downto 0) := (others => '0');    -- clocks from hsync
    signal s_aline  : unsigned(10 downto 0) := (others => '0');    -- active line
    signal s_yf     : unsigned(10 downto 0) := (others => '0');    -- frame line
    signal s_seen, s_saw, s_fev : std_logic := '0';
    signal s_fbit, s_fbit_p, s_ilace : std_logic := '0';
    signal s_avcnt, s_wmeas, s_wl : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- generated avid (parity-locked start, fixed width)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := (others => '0');
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt, g_wm1 : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- CONTENT FIELD (v0.10): blurred picture -> drag, class, marble
    ----------------------------------------------------------------------
    type t_vram is array (0 to 511) of std_logic_vector(23 downto 0);
    signal vram    : t_vram := (others => (others => '0'));
    signal vram_q  : std_logic_vector(23 downto 0) := (others => '0');
    signal vram_ra : unsigned(8 downto 0) := (others => '0');
    signal vram_wa : unsigned(8 downto 0) := (others => '0');
    signal vram_wd : std_logic_vector(23 downto 0) := (others => '0');
    signal vram_we : std_logic := '0';
    signal vw_a1, vw_a2 : unsigned(8 downto 0) := (others => '0');
    signal vw_e1, vw_e2 : std_logic := '0';

    signal s_ylp, s_ulp, s_vlp : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_ysm, s_usm, s_vsm : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal fa_du, fa_dv : signed(8 downto 0)   := (others => '0');
    signal fa_ysm       : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal fb_lumd      : signed(8 downto 0)   := (others => '0');
    signal fb_wdy       : signed(8 downto 0)   := (others => '0');
    signal s_cwx, s_cwy : signed(10 downto 0)  := (others => '0');
    -- contrast (class path)
    signal ga_y, gb_y, ga_u, gb_u, ga_v, gb_v : signed(10 downto 0) := (others => '0');
    signal cy5 : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal cdu5, cdv5 : signed(7 downto 0) := (others => '0');
    -- class
    signal lr_q, lr_r : unsigned(2 downto 0) := (others => '0');
    signal f6_sat : unsigned(7 downto 0) := (others => '0');
    signal f6_dus, f6_dvs, f6_cmp : std_logic := '0';
    signal f7_hcls : unsigned(3 downto 0) := (others => '0');
    signal f7_satok : std_logic := '0';
    signal f8_cls : unsigned(3 downto 0) := (others => '0');
    signal tm_q   : std_logic_vector(3 downto 0) := (others => '0');
    signal s_tex  : unsigned(3 downto 0) := (others => '0');
    -- waves
    signal s_t1, s_t2   : unsigned(5 downto 0) := (others => '0');
    signal s_wm1, s_wm2 : signed(9 downto 0)   := (others => '0');   -- +/-256
    signal s_wtog       : std_logic := '0';
    signal s_wa1, s_wb1, s_wa2, s_wb2 : signed(9 downto 0) := (others => '0');
    signal s_wxs, s_wys : signed(10 downto 0)  := (others => '0');
    signal s_warpx, s_warpy : signed(10 downto 0) := (others => '0');
    signal s_cb         : unsigned(2 downto 0) := (others => '0');   -- marble band
    signal lw_f         : signed(12 downto 0)  := (others => '0');
    signal s_lw         : signed(9 downto 0)   := (others => '0');

    -- OUTLINE: texture-boundary detector on the field (line RAM of tex + edge)
    type t_cram is array (0 to 2047) of std_logic_vector(4 downto 0);
    signal cram    : t_cram := (others => (others => '0'));
    signal cram_q  : std_logic_vector(4 downto 0) := (others => '0');
    signal cw_a    : unsigned(10 downto 0) := (others => '0');
    signal cw_d    : std_logic_vector(4 downto 0) := (others => '0');
    signal tex_d1, tex_d2, tex_d3 : unsigned(3 downto 0) := (others => '0');
    signal hc_d1   : unsigned(10 downto 0) := (others => '0');
    signal s_edge  : std_logic := '0';

    -- both line RAMs never read the word being written (vram: read runs a
    -- group ahead; cram: write trails the read by two clocks), so drop
    -- yosys' read-during-write emulation (damier lesson)
    attribute no_rw_check : boolean;
    attribute no_rw_check of vram : signal is true;
    attribute no_rw_check of cram : signal is true;

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    signal iy1 : unsigned(9 downto 0) := (others => '0');
    signal x1  : unsigned(11 downto 0) := (others => '0');
    signal y1  : unsigned(10 downto 0) := (others => '0');
    signal wx2, wy2, wx3, wxb3, wy3 : unsigned(11 downto 0) := (others => '0');
    signal pa2, pb2 : signed(7 downto 0) := (others => '0');
    signal pi3 : unsigned(4 downto 0) := (others => '0');
    signal gha3, ghb4 : unsigned(11 downto 0) := (others => '0');
    type t_pq is array (0 to 4 + C_PDLY) of unsigned(4 downto 0);
    signal pq_sr : t_pq := (others => (others => '0'));

    signal cx4, cy4, cxb4 : unsigned(7 downto 0) := (others => '0');
    signal lx4, ly4, lxb4 : unsigned(5 downto 0) := (others => '0');

    signal hA5, hC5 : unsigned(11 downto 0) := (others => '0');
    signal ax5, ay5, axb5 : unsigned(4 downto 0) := (others => '0');
    signal ngx5, ngy5, cpar5, cyo5 : std_logic := '0';
    signal nx5, ny5 : unsigned(7 downto 0) := (others => '0');
    signal yy5 : unsigned(6 downto 0) := (others => '0');
    signal lx5, ly5 : unsigned(5 downto 0) := (others => '0');
    signal gr5, gr6, gr7, gr8, gr9 : signed(6 downto 0) := (others => '0');

    signal hA6, hB6, hC6 : unsigned(11 downto 0) := (others => '0');
    signal sqx6, sqy6 : unsigned(9 downto 0) := (others => '0');
    signal mx6, mn6, ax6, ay6, axb6 : unsigned(4 downto 0) := (others => '0');
    signal sec6 : unsigned(3 downto 0) := (others => '0');
    signal nx6, ny6 : unsigned(7 downto 0) := (others => '0');
    signal yy6 : unsigned(6 downto 0) := (others => '0');
    signal lx6, ly6, lyt6 : unsigned(5 downto 0) := (others => '0');
    signal cpar6 : std_logic := '0';
    signal wob6 : signed(9 downto 0) := (others => '0');

    signal hA7, hB7, hC7 : unsigned(11 downto 0) := (others => '0');
    signal r2_7  : unsigned(10 downto 0) := (others => '0');
    signal rb2_7 : unsigned(11 downto 0) := (others => '0');
    signal oct7, e7 : unsigned(5 downto 0) := (others => '0');
    signal cheb7 : unsigned(4 downto 0) := (others => '0');
    signal sec7  : unsigned(3 downto 0) := (others => '0');
    signal tri7  : signed(7 downto 0) := (others => '0');
    signal lyt7  : unsigned(5 downto 0) := (others => '0');
    signal chv7, wav7 : unsigned(7 downto 0) := (others => '0');
    signal mcode7 : unsigned(3 downto 0) := (others => '0');
    signal sc7    : unsigned(5 downto 0) := (others => '0');
    signal mgr7, infl7, rect7, inner7, pres7, cpar7 : std_logic := '0';

    type t_ts is array (0 to 15) of std_logic_vector(1 downto 0);
    type t_tp is array (0 to 15) of unsigned(2 downto 0);
    signal ts8 : t_ts := (others => (others => '0'));
    signal tp8 : t_tp := (others => (others => '0'));

    signal pqk8, dk8, lt8 : unsigned(4 downto 0) := (others => '0');
    signal sel9  : std_logic_vector(1 downto 0) := (others => '0');
    signal pidx9 : unsigned(2 downto 0) := (others => '0');
    signal ix9   : unsigned(4 downto 0) := (others => '0');
    signal jw9, jw10 : std_logic := '0';
    signal edge_sr : std_logic_vector(9 to 13) := (others => '0');
    signal gr10 : signed(6 downto 0) := (others => '0');

    signal by10, bu10, bv10 : unsigned(9 downto 0) := (others => '0');   -- ramp
    signal jy10, ju10, jv10 : unsigned(9 downto 0) := (others => '0');   -- jewel

    signal d11_y, d11_u, d11_v : signed(10 downto 0) := (others => '0');
    signal py11, pu11, pv11 : unsigned(9 downto 0) := (others => '0');
    -- the Ink coefficient feeds three multipliers: one copy per multiplier
    -- so the placer can sit each beside its own (v0.7 lesson)
    signal t11_y, t11_u, t11_v : unsigned(4 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of t11_y : signal is true;
    attribute keep of t11_u : signal is true;
    attribute keep of t11_v : signal is true;

    signal m12_y, m12_u, m12_v : signed(16 downto 0) := (others => '0');
    signal py12, pu12, pv12 : unsigned(9 downto 0) := (others => '0');
    signal o13_y, o13_u, o13_v : unsigned(9 downto 0) := (others => '0');
    signal o14_y, o14_u, o14_v : unsigned(9 downto 0) := (others => '0');
    -- output conditioning (HD HDMI bar fix, damier v0.4.4)
    signal c1_y, c1_u, c1_v, c2_y, c2_u, c2_v : unsigned(9 downto 0) := (others => '0');
    signal o15_y, o15_u, o15_v : unsigned(9 downto 0) := (others => '0');
    signal o_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal o_u, o_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal sr_ga        : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- position, lines, field event (serration-safe), width measurement
    ------------------------------------------------------------------------
    p_pos : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_fev <= '0';

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hc <= (others => '0');
                if s_seen = '1' then
                    s_seen  <= '0';
                    s_wmeas <= s_avcnt;
                    if s_aline /= 2047 then
                        s_aline <= s_aline + 1;
                    end if;
                else
                    s_aline <= (others => '0');
                end if;
                s_avcnt <= (others => '0');
            else
                if s_hc /= 4095 then
                    s_hc <= s_hc + 1;
                end if;
                if data_in.avid = '1' then
                    s_avcnt <= s_avcnt + 1;
                end if;
            end if;

            if data_in.avid = '1' then
                s_seen <= '1';
                s_saw  <= '1';
                s_fbit <= not data_in.field_n;     -- bottom field = field_n '0'
            end if;

            -- vsync fall that follows active video: one event per field
            -- (analog vsync serrates into several edges)
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_fev   <= '1';
                s_wl    <= s_wmeas;
                s_fbit_p <= s_fbit;
                if s_fbit /= s_fbit_p then
                    s_ilace <= '1';
                else
                    s_ilace <= '0';
                end if;
            end if;

            -- frame line (interlace: the two fields interleave)
            if s_ilace = '1' then
                s_yf <= s_aline(9 downto 0) & s_fbit;
            else
                s_yf <= s_aline;
            end if;
        end if;
    end process p_pos;

    ------------------------------------------------------------------------
    -- control latch (once per field) + per-frame sequencer (one op per state)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_ja, v_jb : integer range 0 to 4;
        variable v_t : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            if s_fev = '1' then
                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_p12 <= unsigned(registers_in(7));
                s_map   <= registers_in(6)(0);
                s_flow  <= registers_in(6)(1);
                s_inv   <= registers_in(6)(2);
                s_outl  <= registers_in(6)(3);
                -- v3.1.2 DIAG: S11 is a live U/V output-order test; jewels on
                s_jewel <= '1';
                s_uvsw  <= registers_in(6)(4);
                -- the swirl drifts at its own level, once per field
                s_drift <= s_drift + resize(s_lvl, 11);
                s_seq <= to_unsigned(15, 4);
            elsif s_seq /= 0 then
                case to_integer(s_seq) is
                    when 15 =>
                        -- THEME (labelled, 16): a value below the label count
                        -- is the raw label INDEX (firmware loads defaults so)
                        if s_k1 < 16 then
                            s_theme <= s_k1(3 downto 0);
                        else
                            s_theme <= s_k1(9 downto 6);
                        end if;
                    when 14 =>
                        -- CONTRAST: 0.375 .. 3, step 4 (centre) = unity
                        s_gain <= to_integer(s_k4(9 downto 7));
                    when 13 =>
                        s_pc <= s_k2(9 downto 7);
                    when 12 =>
                        -- POSTER: bands 8,7,6,5,4,3,2,2 ; paint levels
                        -- 32,16,16,8,8,4,4,2 ; hue sectors 8,8,8,4,4,2,2,2
                        case to_integer(s_pc) is
                            when 0 => s_lpn <= "110"; s_pqb <= 0; s_hmask <= "11";
                            when 1 => s_lpn <= "101"; s_pqb <= 1; s_hmask <= "11";
                            when 2 => s_lpn <= "100"; s_pqb <= 1; s_hmask <= "11";
                            when 3 => s_lpn <= "011"; s_pqb <= 2; s_hmask <= "10";
                            when 4 => s_lpn <= "010"; s_pqb <= 2; s_hmask <= "10";
                            when 5 => s_lpn <= "001"; s_pqb <= 3; s_hmask <= "00";
                            when 6 => s_lpn <= "000"; s_pqb <= 3; s_hmask <= "00";
                            when others => s_lpn <= "000"; s_pqb <= 4; s_hmask <= "00";
                        end case;
                    when 11 =>
                        -- SCALE (labelled, 4): pitch 16 / 32 / 64 / 128 px
                        if s_k3 < 4 then
                            s_k <= 4 + to_integer(s_k3(1 downto 0));
                        else
                            s_k <= 4 + to_integer(s_k3(9 downto 8));
                        end if;
                    when 10 =>
                        -- INK: 4/16 .. 16/16 of the way from paint to ink
                        v_t := ('0' & s_k5(9 downto 6)) - ("000" & s_k5(9 downto 8));
                        s_ink_t <= v_t + 4;
                    when 9 =>
                        -- DENSITY: fraction of cells carrying their motif
                        if s_k6(9 downto 2) < 24 then
                            s_dens <= to_unsigned(24, 8);
                        else
                            s_dens <= s_k6(9 downto 2);
                        end if;
                    when 8 =>
                        s_lvl <= s_p12(9 downto 7);
                    when 7 =>
                        -- SWIRL waves: amplitude = pitch x {0, 1/16, 1/8, 3/16,
                        -- 1/4, 3/8, 1/2, 3/4} as two shifted terms of the
                        -- +/-256 sine: shift = 8 - k + j
                        case to_integer(s_lvl) is
                            when 0 => v_ja := 0; v_jb := 0;
                            when 1 => v_ja := 4; v_jb := 0;
                            when 2 => v_ja := 3; v_jb := 0;
                            when 3 => v_ja := 3; v_jb := 4;
                            when 4 => v_ja := 2; v_jb := 0;
                            when 5 => v_ja := 2; v_jb := 3;
                            when 6 => v_ja := 1; v_jb := 0;
                            when others => v_ja := 1; v_jb := 2;
                        end case;
                        if v_ja = 0 then
                            s_wsa <= 15; s_ssa <= 15;
                        else
                            s_wsa <= 8 - s_k + v_ja; s_ssa <= 7 - s_k + v_ja;
                        end if;
                        if v_jb = 0 then
                            s_wsb <= 15; s_ssb <= 15;
                        else
                            s_wsb <= 8 - s_k + v_jb; s_ssb <= 7 - s_k + v_jb;
                        end if;
                    when 6 =>
                        s_dph <= s_drift(8 downto 3);
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- generated avid: starts 2 or 3 clocks after the source's avid rise,
    -- whichever keeps the hsync->start parity at its voted usual value;
    -- ends after the field-latched width (aurora / cubist)
    ------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;
            g_wm1 <= s_wl - 1;
            if g_hsr = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if g_avr = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;

    ------------------------------------------------------------------------
    -- LARGE SWAY (v0.10): per-line triangle of a drifting phase, smoothed
    -- down the frame into a sine-like sway; amplitude follows SWIRL
    ------------------------------------------------------------------------
    p_lwave : process(clk)
        variable v_ph  : unsigned(8 downto 0);
        variable v_tri : signed(12 downto 0);
        variable v_h   : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                v_ph := s_yf(8 downto 0) + s_drift(10 downto 2);
                if v_ph(8) = '1' then
                    v_tri := signed(resize(not v_ph(7 downto 0), 13)) - 128;
                else
                    v_tri := signed(resize(v_ph(7 downto 0), 13)) - 128;
                end if;
                lw_f <= lw_f + shift_right(shift_left(v_tri, 4) - lw_f, 5);
            end if;
            v_h := resize(shift_right(lw_f, 4), 10);              -- +/-128
            s_lw <= f_shr(v_h, s_ssa) + f_shr(v_h, s_ssb);
        end if;
    end process p_lwave;

    ------------------------------------------------------------------------
    -- CONTENT FIELD + CLASSIFIER + WAVES (free-running beside the pixel
    -- path; consumed a few px stale, invisible under the blur)
    ------------------------------------------------------------------------
    p_field : process(clk)
        variable v_y, v_u, v_v    : unsigned(7 downto 0);
        variable v_py, v_pu, v_pv : unsigned(7 downto 0);
        variable v_wy             : signed(11 downto 0);
        variable v_d              : signed(10 downto 0);
        variable v_s              : signed(11 downto 0);
        variable v_adu, v_adv     : unsigned(7 downto 0);
        variable v_p1, v_p2       : unsigned(8 downto 0);
        variable v_t1, v_t2       : unsigned(5 downto 0);
        variable v_tw             : unsigned(5 downto 0);
        variable v_hw             : signed(9 downto 0);
        variable v_fx, v_fy       : signed(10 downto 0);
        variable v_cb             : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            ------------------------------------------------------------
            -- F1: horizontal IIR (~8 px), reloaded through blanking
            ------------------------------------------------------------
            v_y := unsigned(data_in.y(9 downto 2));
            v_u := unsigned(data_in.v(9 downto 2));   -- HW U<->V swap
            v_v := unsigned(data_in.u(9 downto 2));
            if data_in.avid = '0' then
                s_ylp <= v_y;  s_ulp <= v_u;  s_vlp <= v_v;
            else
                s_ylp <= f_iir(s_ylp, v_y, 3);
                s_ulp <= f_iir(s_ulp, v_u, 3);
                s_vlp <= f_iir(s_vlp, v_v, 3);
            end if;
            -- read the group we are about to update; its write lands two
            -- cycles later, when the read address has moved on
            vram_ra <= s_hc(10 downto 2);
            vw_a1   <= s_hc(10 downto 2);
            if data_in.avid = '1' and s_hc(1 downto 0) = "11" then
                vw_e1 <= '1';
            else
                vw_e1 <= '0';
            end if;
            vw_a2 <= vw_a1;
            vw_e2 <= vw_e1;

            ------------------------------------------------------------
            -- F2: vertical IIR (~4 lines) through the line buffer
            ------------------------------------------------------------
            v_py := unsigned(vram_q(23 downto 16));
            v_pu := unsigned(vram_q(15 downto  8));
            v_pv := unsigned(vram_q( 7 downto  0));
            if s_aline < 2 then
                s_ysm <= s_ylp;  s_usm <= s_ulp;  s_vsm <= s_vlp;
            else
                s_ysm <= f_iir(v_py, s_ylp, 2);
                s_usm <= f_iir(v_pu, s_ulp, 2);
                s_vsm <= f_iir(v_pv, s_vlp, 2);
            end if;
            vram_wd <= std_logic_vector(s_ysm) & std_logic_vector(s_usm)
                       & std_logic_vector(s_vsm);
            vram_wa <= vw_a2;
            vram_we <= vw_e2 and data_in.avid;

            ------------------------------------------------------------
            -- F3: chroma offsets about neutral
            ------------------------------------------------------------
            fa_du  <= signed('0' & s_usm) - to_signed(128, 9);
            fa_dv  <= signed('0' & s_vsm) - to_signed(128, 9);
            fa_ysm <= s_ysm;

            ------------------------------------------------------------
            -- F4: drag drives | CONTRAST terms (sugarcoat proc-amp)
            ------------------------------------------------------------
            fb_lumd <= signed('0' & fa_ysm) - to_signed(128, 9);
            -- vertical drive: warm/cool chroma axis plus a luma term, so
            -- a grey picture still bends vertically
            v_wy := shift_left(resize(fa_dv - fa_du, 12), 1)
                    + shift_right(resize(signed('0' & fa_ysm) - to_signed(128, 9), 12), 1);
            if v_wy > 127 then
                fb_wdy <= to_signed(127, 9);
            elsif v_wy < -127 then
                fb_wdy <= to_signed(-127, 9);
            else
                fb_wdy <= resize(v_wy, 9);
            end if;

            v_d := resize(signed('0' & fa_ysm) - to_signed(128, 10), 11);
            case s_gain is
                when 0 => ga_y <= shift_right(v_d, 2); gb_y <= shift_right(v_d, 3);
                when 1 => ga_y <= shift_right(v_d, 1); gb_y <= (others => '0');
                when 2 => ga_y <= shift_right(v_d, 1); gb_y <= shift_right(v_d, 3);
                when 3 => ga_y <= shift_right(v_d, 1); gb_y <= shift_right(v_d, 2);
                when 4 => ga_y <= v_d;                 gb_y <= (others => '0');
                when 5 => ga_y <= v_d;                 gb_y <= shift_right(v_d, 1);
                when 6 => ga_y <= shift_left(v_d, 1);  gb_y <= (others => '0');
                when others => ga_y <= shift_left(v_d, 1); gb_y <= v_d;
            end case;
            v_d := resize(fa_du, 11);
            case s_gain is
                when 0 => ga_u <= shift_right(v_d, 2); gb_u <= shift_right(v_d, 3);
                when 1 => ga_u <= shift_right(v_d, 1); gb_u <= (others => '0');
                when 2 => ga_u <= shift_right(v_d, 1); gb_u <= shift_right(v_d, 3);
                when 3 => ga_u <= shift_right(v_d, 1); gb_u <= shift_right(v_d, 2);
                when 4 => ga_u <= v_d;                 gb_u <= (others => '0');
                when 5 => ga_u <= v_d;                 gb_u <= shift_right(v_d, 1);
                when 6 => ga_u <= shift_left(v_d, 1);  gb_u <= (others => '0');
                when others => ga_u <= shift_left(v_d, 1); gb_u <= v_d;
            end case;
            v_d := resize(fa_dv, 11);
            case s_gain is
                when 0 => ga_v <= shift_right(v_d, 2); gb_v <= shift_right(v_d, 3);
                when 1 => ga_v <= shift_right(v_d, 1); gb_v <= (others => '0');
                when 2 => ga_v <= shift_right(v_d, 1); gb_v <= shift_right(v_d, 3);
                when 3 => ga_v <= shift_right(v_d, 1); gb_v <= shift_right(v_d, 2);
                when 4 => ga_v <= v_d;                 gb_v <= (others => '0');
                when 5 => ga_v <= v_d;                 gb_v <= shift_right(v_d, 1);
                when 6 => ga_v <= shift_left(v_d, 1);  gb_v <= (others => '0');
                when others => ga_v <= shift_left(v_d, 1); gb_v <= v_d;
            end case;

            ------------------------------------------------------------
            -- F5: SWIRL-scaled drag | contrast sums, re-centre, clamp
            ------------------------------------------------------------
            s_cwx <= f_wscale(resize(fb_lumd, 11), s_lvl);
            s_cwy <= f_wscale(resize(fb_wdy,  11), s_lvl);

            v_s := resize(ga_y, 12) + resize(gb_y, 12) + 128;
            if v_s < 0 then
                cy5 <= (others => '0');
            elsif v_s > 255 then
                cy5 <= (others => '1');
            else
                cy5 <= unsigned(v_s(7 downto 0));
            end if;
            v_s := resize(ga_u, 12) + resize(gb_u, 12);
            if v_s > 127 then
                cdu5 <= to_signed(127, 8);
            elsif v_s < -127 then
                cdu5 <= to_signed(-127, 8);
            else
                cdu5 <= v_s(7 downto 0);
            end if;
            v_s := resize(ga_v, 12) + resize(gb_v, 12);
            if v_s > 127 then
                cdv5 <= to_signed(127, 8);
            elsif v_s < -127 then
                cdv5 <= to_signed(-127, 8);
            else
                cdv5 <= v_s(7 downto 0);
            end if;

            ------------------------------------------------------------
            -- F6: luma band ROM read | hue terms
            ------------------------------------------------------------
            lr_q <= C_LROM(to_integer(s_lpn & cy5(7 downto 2)));
            v_adu := f_abs8(cdu5);
            v_adv := f_abs8(cdv5);
            f6_sat <= v_adu + v_adv;                  -- both <= 127: no wrap
            f6_dus <= not cdu5(7);                    -- Cb >= 0
            f6_dvs <= not cdv5(7);                    -- Cr >= 0
            if v_adu > v_adv then
                f6_cmp <= '1';
            else
                f6_cmp <= '0';
            end if;

            ------------------------------------------------------------
            -- F7: register the ROM | hue octant (POSTER masks it to 4 / 2)
            ------------------------------------------------------------
            lr_r <= lr_q;
            f7_hcls <= '1' & f6_dvs & (f6_dus and s_hmask(1)) & (f6_cmp and s_hmask(0));
            if f6_sat >= C_SATF then
                f7_satok <= '1';
            else
                f7_satok <= '0';
            end if;

            ------------------------------------------------------------
            -- F8: CLASS -- Chroma map: hue octant where the picture has a
            -- colour, luma band where it is grey; Luma map: luma band
            ------------------------------------------------------------
            if s_map = '1' and f7_satok = '1' then
                f8_cls <= f7_hcls;
            else
                f8_cls <= '0' & lr_r;
            end if;

            ------------------------------------------------------------
            -- F9/F10: theme ROM -> texture index (fabric re-register)
            ------------------------------------------------------------
            tm_q  <= C_TMAP(to_integer(s_theme & f8_cls));
            s_tex <= unsigned(tm_q);

            ------------------------------------------------------------
            -- W1..W5: ORGANIC WAVES (v0.10) + drift. Two medium sines,
            -- wavelength 2 pitches, one time-shared ROM; amplitude, sway
            -- and drift rate all follow SWIRL.
            ------------------------------------------------------------
            v_p1 := (s_yf(7 downto 0) & '0') + s_hc(9 downto 1);
            v_p2 := (s_hc(7 downto 0) & '0') - s_yf(9 downto 1);
            case s_k is
                when 4 =>      v_t1 := v_p1(5 downto 0); v_t2 := v_p2(5 downto 0);
                when 5 =>      v_t1 := v_p1(6 downto 1); v_t2 := v_p2(6 downto 1);
                when 6 =>      v_t1 := v_p1(7 downto 2); v_t2 := v_p2(7 downto 2);
                when others => v_t1 := v_p1(8 downto 3); v_t2 := v_p2(8 downto 3);
            end case;
            s_t1 <= v_t1 + s_dph;
            s_t2 <= v_t2 - s_dph;
            s_wtog <= not s_wtog;
            if s_wtog = '0' then v_tw := s_t1; else v_tw := s_t2; end if;
            v_hw := f_psin(v_tw);
            if s_wtog = '0' then
                s_wm1 <= v_hw;
            else
                s_wm2 <= v_hw;
            end if;
            s_wa1 <= f_shr(s_wm1, s_wsa);
            s_wb1 <= f_shr(s_wm1, s_wsb);
            s_wa2 <= f_shr(s_wm2, s_wsa);
            s_wb2 <= f_shr(s_wm2, s_wsb);
            s_wxs <= resize(s_wa1, 11) + resize(s_wb1, 11) + resize(s_lw, 11);
            s_wys <= resize(s_wa2, 11) + resize(s_wb2, 11);
            -- S8 FLOW swaps which axis luma and chroma drag
            if s_flow = '0' then
                v_fx := s_cwx;  v_fy := s_cwy;
            else
                v_fx := s_cwy;  v_fy := s_cwx;
            end if;
            s_warpx <= v_fx + s_wxs;
            s_warpy <= v_fy + s_wys;
            -- MARBLE band: isophotes of the blurred luma, marbled by a wave
            v_cb := signed(shift_left(resize(s_ysm, 12), 2))
                    + resize(shift_right(s_wm2, 1), 12);
            s_cb <= unsigned(v_cb(5 downto 3));
        end if;
    end process p_field;

    p_vram : process(clk)
    begin
        if rising_edge(clk) then
            if vram_we = '1' then
                vram(to_integer(vram_wa)) <= vram_wd;
            end if;
            vram_q <= vram(to_integer(vram_ra));
        end if;
    end process p_vram;

    ------------------------------------------------------------------------
    -- OUTLINE: a texture boundary against the 2 previous pixels or the
    -- line above. The flag is stored with the texture so the line below
    -- inherits it (2 lines thick). Read at hc, write at hc-2: never the
    -- same word.
    ------------------------------------------------------------------------
    p_edge : process(clk)
        variable v_e : std_logic;
    begin
        if rising_edge(clk) then
            tex_d1 <= s_tex;
            tex_d2 <= tex_d1;
            tex_d3 <= tex_d2;
            hc_d1  <= s_hc(10 downto 0);
            cram_q <= cram(to_integer(s_hc(10 downto 0)));   -- aligned with tex_d1

            v_e := '0';
            if s_avid_sr(12) = '1' and s_avid_sr(14) = '1' then
                if tex_d1 /= tex_d2 or tex_d1 /= tex_d3 then
                    v_e := '1';
                end if;
                if s_aline /= 0 and tex_d1 /= unsigned(cram_q(3 downto 0)) then
                    v_e := '1';
                end if;
            end if;
            s_edge <= v_e or (cram_q(4) and s_avid_sr(12));
            cw_a <= hc_d1;
            cw_d <= v_e & std_logic_vector(tex_d1);
            cram(to_integer(cw_a)) <= cw_d;
        end if;
    end process p_edge;

    ------------------------------------------------------------------------
    -- main pixel pipeline
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d     : signed(6 downto 0);
        variable v_s     : signed(8 downto 0);
        variable v_half, v_halfb : unsigned(11 downto 0);
        variable v_q     : unsigned(4 downto 0);
        variable v_h     : unsigned(11 downto 0);
        variable v_grain : signed(6 downto 0);
        variable v_qq    : unsigned(1 downto 0);
        variable v_qd    : unsigned(1 downto 0);
        variable v_sec   : unsigned(3 downto 0);
        variable v_r13   : signed(12 downto 0);
        variable v_t8    : signed(7 downto 0);
        variable v_fn    : unsigned(5 downto 0);
        variable v_t4    : unsigned(3 downto 0);
        variable v_t6    : unsigned(5 downto 0);
        variable v_ts    : t_ts;
        variable v_tp    : t_tp;
        variable v_b     : boolean;
        variable v_sp    : unsigned(5 downto 0);
        variable v_bin, v_brim : boolean;
        variable v_pi    : integer range 0 to 7;
        variable v_py    : signed(11 downto 0);
        variable v_o     : signed(12 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_dl, v_yo : unsigned(9 downto 0);
        variable v_ik, v_ia : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            --------------------------------------------------------------
            -- E1: input latch (luma only: the paint), beam position
            --------------------------------------------------------------
            iy1 <= unsigned(data_in.y);
            x1  <= s_hc;
            y1  <= s_yf;

            --------------------------------------------------------------
            -- E2: WARP -- the picture and the waves drag ornament space |
            -- paint CONTRAST terms on the 32-step ramp index
            --------------------------------------------------------------
            wx2 <= x1 + unsigned(resize(s_warpx, 12));
            wy2 <= ('0' & y1) + unsigned(resize(s_warpy, 12));
            v_d := signed(resize(iy1(9 downto 5), 7)) - 16;
            case s_gain is
                when 0 => pa2 <= resize(shift_right(v_d, 2), 8); pb2 <= resize(shift_right(v_d, 3), 8);
                when 1 => pa2 <= resize(shift_right(v_d, 1), 8); pb2 <= (others => '0');
                when 2 => pa2 <= resize(shift_right(v_d, 1), 8); pb2 <= resize(shift_right(v_d, 3), 8);
                when 3 => pa2 <= resize(shift_right(v_d, 1), 8); pb2 <= resize(shift_right(v_d, 2), 8);
                when 4 => pa2 <= resize(v_d, 8);                 pb2 <= (others => '0');
                when 5 => pa2 <= resize(v_d, 8);                 pb2 <= resize(shift_right(v_d, 1), 8);
                when 6 => pa2 <= shift_left(resize(v_d, 8), 1);  pb2 <= (others => '0');
                when others => pa2 <= shift_left(resize(v_d, 8), 1); pb2 <= resize(v_d, 8);
            end case;

            --------------------------------------------------------------
            -- E3: half-pitch row stagger (dots sit like bricks; bars
            -- stagger per row PAIR) | paint index clamp | leaf grain hash
            --------------------------------------------------------------
            v_half  := (others => '0');
            v_halfb := (others => '0');
            case s_k is
                when 4 =>      v_half(3) := wy2(4); v_halfb(3) := wy2(5);
                when 5 =>      v_half(4) := wy2(5); v_halfb(4) := wy2(6);
                when 6 =>      v_half(5) := wy2(6); v_halfb(5) := wy2(7);
                when others => v_half(6) := wy2(7); v_halfb(6) := wy2(8);
            end case;
            wx3  <= wx2 + v_half;
            wxb3 <= wx2 + v_halfb;
            wy3  <= wy2;
            v_s := resize(pa2, 9) + resize(pb2, 9) + 16;
            if v_s < 0 then
                pi3 <= (others => '0');
            elsif v_s > 31 then
                pi3 <= (others => '1');
            else
                pi3 <= unsigned(v_s(4 downto 0));
            end if;
            gha3 <= f_hash_a(wx2(9 downto 2), wy2(9 downto 2), x"9E37");

            --------------------------------------------------------------
            -- E4: cell slices at pitch 2^k, in-cell position normalised
            -- to 0..63 | paint POSTER + INVERT | grain hash B
            --------------------------------------------------------------
            case s_k is
                when 4 =>
                    cx4 <= wx3(11 downto 4);  lx4 <= wx3(3 downto 0) & "00";
                    cy4 <= wy3(11 downto 4);  ly4 <= wy3(3 downto 0) & "00";
                    cxb4 <= wxb3(11 downto 4); lxb4 <= wxb3(3 downto 0) & "00";
                when 5 =>
                    cx4 <= '0' & wx3(11 downto 5);  lx4 <= wx3(4 downto 0) & '0';
                    cy4 <= '0' & wy3(11 downto 5);  ly4 <= wy3(4 downto 0) & '0';
                    cxb4 <= '0' & wxb3(11 downto 5); lxb4 <= wxb3(4 downto 0) & '0';
                when 6 =>
                    cx4 <= "00" & wx3(11 downto 6);  lx4 <= wx3(5 downto 0);
                    cy4 <= "00" & wy3(11 downto 6);  ly4 <= wy3(5 downto 0);
                    cxb4 <= "00" & wxb3(11 downto 6); lxb4 <= wxb3(5 downto 0);
                when others =>
                    cx4 <= "000" & wx3(11 downto 7);  lx4 <= wx3(6 downto 1);
                    cy4 <= "000" & wy3(11 downto 7);  ly4 <= wy3(6 downto 1);
                    cxb4 <= "000" & wxb3(11 downto 7); lxb4 <= wxb3(6 downto 1);
            end case;
            case s_pqb is
                when 0 =>      v_q := pi3;
                when 1 =>      v_q := pi3(4 downto 1) & '1';
                when 2 =>      v_q := pi3(4 downto 2) & "10";
                when 3 =>      v_q := pi3(4 downto 3) & "100";
                when others => v_q := pi3(4) & "1000";
            end case;
            if s_inv = '1' then
                v_q := not v_q;
            end if;
            pq_sr(0) <= v_q;
            for i in 1 to 4 + C_PDLY loop
                pq_sr(i) <= pq_sr(i - 1);
            end loop;
            ghb4 <= f_hash_b(gha3);

            --------------------------------------------------------------
            -- E5: cell hashes | offsets from the cell centre (folded) |
            -- continuous coordinates for the cross-cell textures | grain
            --------------------------------------------------------------
            hA5 <= f_hash_a(cx4, cy4, x"B7C3");
            hC5 <= f_hash_a(cxb4, '0' & cy4(7 downto 1), x"5D2F");
            ax5  <= f_fold(lx4);
            ay5  <= f_fold(ly4);
            axb5 <= f_fold(lxb4);
            ngx5 <= not lx4(5);
            ngy5 <= not ly4(5);
            nx5 <= cx4(1 downto 0) & lx4;
            ny5 <= cy4(1 downto 0) & ly4;
            yy5 <= cy4(0) & ly4;
            cpar5 <= cx4(0) xor cy4(0);
            cyo5  <= cy4(0);
            lx5 <= lx4;
            ly5 <= ly4;
            -- gold-leaf grain: fine sparkle, bright glints, dark flecks
            v_h := ghb4;
            v_grain := resize(signed('0' & v_h(4 downto 0)) - 16, 7);
            if v_h(11 downto 8) = "1111" then
                v_grain := to_signed(52, 7);
            elsif v_h(11 downto 8) = "0000" then
                v_grain := to_signed(-26, 7);
            end if;
            gr5 <= v_grain;

            --------------------------------------------------------------
            -- E6: hash B | squares | max / min | 16-sector angle
            --------------------------------------------------------------
            hB6 <= f_hash_b(hA5);
            hA6 <= hA5;
            hC6 <= hC5;
            sqx6 <= C_SQ(to_integer(ax5));
            sqy6 <= C_SQ(to_integer(ay5));
            if ax5 > ay5 then
                mx6 <= ax5;  mn6 <= ay5;
            else
                mx6 <= ay5;  mn6 <= ax5;
            end if;
            -- angle without an arctangent: two ratio compares + the signs,
            -- odd quadrants reversed so the order runs round the circle
            if ay5 <= ('0' & ax5(4 downto 1)) then
                v_qq := "00";
            elsif ax5 > ay5 then
                v_qq := "01";
            elsif ax5 > ('0' & ay5(4 downto 1)) then
                v_qq := "10";
            else
                v_qq := "11";
            end if;
            v_qd := ngy5 & (ngx5 xor ngy5);
            if v_qd(0) = '1' then
                v_qq := not v_qq;
            end if;
            sec6 <= v_qd & v_qq;
            ax6 <= ax5;  ay6 <= ay5;  axb6 <= axb5;
            nx6 <= nx5;  ny6 <= ny5;  yy6 <= yy5;
            lx6 <= lx5;  ly6 <= ly5;
            if cyo5 = '1' then
                lyt6 <= not ly5;                       -- alternate rows point down
            else
                lyt6 <= ly5;
            end if;
            cpar6 <= cpar5;
            wob6  <= shift_right(s_wm1, 1);            -- +/-128 on r^2: blobs, not discs
            gr6   <= gr5;

            --------------------------------------------------------------
            -- E7: metrics -- r^2, wobbled r^2, ~round radius, Chebyshev,
            -- spiral angle, almond, triangle, zigzag, wave, tile codes
            --------------------------------------------------------------
            r2_7 <= ('0' & sqx6) + ('0' & sqy6);
            v_r13 := signed(resize(sqx6, 13)) + signed(resize(sqy6, 13)) + resize(wob6, 13);
            if v_r13 < 0 then
                rb2_7 <= (others => '0');
            else
                rb2_7 <= unsigned(v_r13(11 downto 0));
            end if;
            oct7  <= resize(mx6, 6) + resize(mn6(4 downto 1), 6);
            cheb7 <= mx6;
            v_sec := sec6;
            if hB6(0) = '1' then
                v_sec := not v_sec;                    -- per-cell chirality
            end if;
            sec7 <= v_sec + hB6(6 downto 3);           -- per-cell rotation
            e7   <= resize(ay6, 6) + resize(sqx6(9 downto 5), 6);
            v_t8 := signed(resize(lyt6, 8)) - 10 - signed(resize(ax6 & '0', 8));
            tri7 <= v_t8;
            lyt7 <= lyt6;
            chv7 <= ny6 + resize(ax6, 8);
            if nx6(6) = '1' then
                v_fn := nx6(5 downto 0);
            else
                v_fn := not nx6(5 downto 0);
            end if;
            wav7 <= ny6 + resize(v_fn(5 downto 1), 8);
            v_t4 := lx6(5 downto 4) & ly6(5 downto 4);
            mcode7 <= hB6(3 downto 0) xor v_t4;
            if lx6(3 downto 1) = "000" or ly6(3 downto 1) = "000" then
                mgr7 <= '1';
            else
                mgr7 <= '0';
            end if;
            v_t6 := lx6(5 downto 3) & ly6(5 downto 3);
            sc7 <= hB6(11 downto 6) xor v_t6;
            if lx6(2 downto 0) >= 2 and lx6(2 downto 0) <= 5
               and ly6(2 downto 0) >= 2 and ly6(2 downto 0) <= 5 then
                infl7 <= '1';
            else
                infl7 <= '0';
            end if;
            if axb6 < 22 and yy6 >= 6 and yy6 < 122 then
                rect7 <= '1';
            else
                rect7 <= '0';
            end if;
            if axb6 < 11 and yy6 >= 22 and yy6 < 106 then
                inner7 <= '1';
            else
                inner7 <= '0';
            end if;
            if hB6(7 downto 0) < s_dens then
                pres7 <= '1';
            else
                pres7 <= '0';
            end if;
            hA7 <= hA6;  hB7 <= hB6;  hC7 <= hC6;
            cpar7 <= cpar6;
            gr7 <= gr6;

            --------------------------------------------------------------
            -- E8: the 16 TEXTURE RULES, all in parallel. Each gives an ink
            -- select (ground / dark / light / jewel) and a palette index.
            --------------------------------------------------------------
            v_ts := (others => C_SG);
            v_tp := (others => "000");

            -- 0 LEAF: the gold paint alone

            -- 1 DOTS: a dark dot per cell, some jewelled, bone highlight
            if hA7(0) = '1' then v_b := r2_7 < 144; else v_b := r2_7 < 100; end if;
            if pres7 = '1' and v_b then
                if r2_7 < 9 then
                    v_ts(1) := C_SL;  v_tp(1) := hB7(10 downto 8);
                elsif hB7(11 downto 10) = "11" then
                    v_ts(1) := C_SC;  v_tp(1) := hA7(5 downto 3);
                else
                    v_ts(1) := C_SD;  v_tp(1) := '0' & hB7(9 downto 8);
                end if;
            end if;

            -- 2 RINGS: circles in circles -- dot, jewel/bone ring, dark
            -- ring, pale ring
            if pres7 = '1' then
                if r2_7 < 25 then
                    v_ts(2) := C_SD;  v_tp(2) := '0' & hB7(9 downto 8);
                elsif r2_7 < 64 then
                    null;
                elsif r2_7 < 144 then
                    if hA7(1) = '1' then v_ts(2) := C_SC; else v_ts(2) := C_SL; end if;
                    v_tp(2) := hB7(10 downto 8);
                elsif r2_7 < 256 then
                    null;
                elsif r2_7 < 400 then
                    v_ts(2) := C_SD;  v_tp(2) := '0' & hB7(9 downto 8);
                elsif r2_7 < 576 then
                    null;
                elsif r2_7 < 784 then
                    v_ts(2) := C_SL;  v_tp(2) := hA7(8 downto 6);
                end if;
            end if;

            -- 3 SPIRAL: two arms (ink + pale), ~2 turns, per-cell
            -- chirality and rotation
            v_sp := oct7 + resize(sec7, 6);
            if pres7 = '1' and oct7 < 30 then
                if oct7 < 3 then
                    v_ts(3) := C_SD;  v_tp(3) := "000";
                elsif v_sp(3 downto 2) = "00" then
                    v_ts(3) := C_SD;  v_tp(3) := '0' & hB7(9 downto 8);
                elsif v_sp(3 downto 2) = "10" then
                    v_ts(3) := C_SL;  v_tp(3) := hB7(10 downto 8);
                end if;
            end if;

            -- 4 SQUARES: Adele's hollow squares -- red/orange frame, black
            -- inner frame, bone centre
            if pres7 = '1' then
                if cheb7 >= 23 and cheb7 < 28 then
                    v_ts(4) := C_SC;  v_tp(4) := '0' & hB7(9 downto 8);
                elsif cheb7 >= 12 and cheb7 < 16 then
                    v_ts(4) := C_SD;  v_tp(4) := "000";
                elsif cheb7 < 5 then
                    v_ts(4) := C_SL;  v_tp(4) := hB7(10 downto 8);
                end if;
            end if;

            -- 5 CHECKER: the Kiss robe -- black and silver tablets on gold
            if cheb7 < 27 then
                if cpar7 = '1' then
                    v_ts(5) := C_SD;  v_tp(5) := '0' & hB7(9 downto 8);
                else
                    v_ts(5) := C_SL;  v_tp(5) := hB7(10 downto 8);
                end if;
            end if;

            -- 6 EYES: almond lid (parabolic arcs), white, jewel iris, pupil
            if pres7 = '1' and e7 < 13 then
                if r2_7 < 12 then
                    v_ts(6) := C_SD;  v_tp(6) := "000";
                elsif r2_7 < 56 then
                    v_ts(6) := C_SC;  v_tp(6) := '1' & hB7(9 downto 8);
                elsif e7 >= 10 then
                    v_ts(6) := C_SD;  v_tp(6) := "001";
                else
                    v_ts(6) := C_SL;  v_tp(6) := hB7(10 downto 8);
                end if;
            end if;

            -- 7 BLOBS: wobbling jewel blobs with a dark rim, per-cell size
            case hA7(1 downto 0) is
                when "00"   => v_bin := rb2_7 < 256; v_brim := rb2_7 < 416;
                when "01"   => v_bin := rb2_7 < 400; v_brim := rb2_7 < 560;
                when "10"   => v_bin := rb2_7 < 576; v_brim := rb2_7 < 736;
                when others => v_bin := rb2_7 < 784; v_brim := rb2_7 < 944;
            end case;
            if pres7 = '1' then
                if v_bin then
                    v_ts(7) := C_SC;  v_tp(7) := hB7(10 downto 8);
                elsif v_brim then
                    v_ts(7) := C_SD;  v_tp(7) := "000";
                end if;
            end if;

            -- 8 BARS: tall Kiss rectangles, two cells high, staggered per
            -- row pair; framed, solid or jewelled
            if rect7 = '1' then
                case hC7(2 downto 0) is
                    when "000" | "001" | "010" =>
                        if inner7 = '1' then
                            v_ts(8) := C_SL;  v_tp(8) := hC7(7 downto 5);
                        else
                            v_ts(8) := C_SD;  v_tp(8) := '0' & hC7(4 downto 3);
                        end if;
                    when "011" | "100" =>
                        if inner7 = '1' then
                            v_ts(8) := C_SD;  v_tp(8) := '0' & hC7(4 downto 3);
                        else
                            v_ts(8) := C_SL;  v_tp(8) := hC7(7 downto 5);
                        end if;
                    when "101" =>
                        if inner7 = '1' then
                            v_ts(8) := C_SD;  v_tp(8) := "000";
                        else
                            v_ts(8) := C_SC;  v_tp(8) := "110";
                        end if;
                    when others =>
                        v_ts(8) := C_SD;  v_tp(8) := '0' & hC7(4 downto 3);
                end case;
            end if;

            -- 9 MOSAIC: 4x4 tesserae per cell, dark grout, gold / pale /
            -- jewel / dark tiles
            if mgr7 = '1' then
                v_ts(9) := C_SD;  v_tp(9) := "001";
            else
                case C_MOSK(to_integer(mcode7)) is
                    when 1 =>      v_ts(9) := C_SL;  v_tp(9) := hA7(2 downto 0);
                    when 2 =>      v_ts(9) := C_SC;  v_tp(9) := hA7(5 downto 3) xor mcode7(2 downto 0);
                    when 3 =>      v_ts(9) := C_SD;  v_tp(9) := '1' & hA7(1 downto 0);
                    when others => null;
                end case;
            end if;

            -- 10 MARBLE: isophote bands of the blurred picture, swirled by
            -- the wave -- the oil itself
            case s_cb is
                when "000" | "001" => v_ts(10) := C_SD;  v_tp(10) := "100";
                when "011" | "100" => v_ts(10) := C_SL;  v_tp(10) := "001";
                when "110"         => v_ts(10) := C_SC;  v_tp(10) := "011";
                when others        => null;
            end case;

            -- 11 CHEVRON: zigzag stripes running across the cells
            case chv7(4 downto 3) is
                when "00"   => v_ts(11) := C_SD;  v_tp(11) := "000";
                when "10"   => v_ts(11) := C_SL;  v_tp(11) := "001";
                when others => null;
            end case;

            -- 12 ROSETTE: eight petals round a jewel centre
            if pres7 = '1' and oct7 < 28 then
                if oct7 < 6 then
                    v_ts(12) := C_SC;  v_tp(12) := "01" & hA7(0);
                elsif oct7 < 9 then
                    v_ts(12) := C_SD;  v_tp(12) := "000";
                elsif sec7(0) = '0' then
                    v_ts(12) := C_SL;  v_tp(12) := hB7(10 downto 8);
                end if;
            end if;

            -- 13 TRIANGLE: up / down rows, dark with a pale or jewel core
            if pres7 = '1' and tri7 >= 0 and lyt7 < 54 then
                if tri7 >= 12 then
                    if hB7(9) = '1' then v_ts(13) := C_SC; else v_ts(13) := C_SL; end if;
                    v_tp(13) := hB7(10 downto 8);
                else
                    v_ts(13) := C_SD;  v_tp(13) := "000";
                end if;
            end if;

            -- 14 FLAKES: scattered squares of leaf (density-scaled)
            if infl7 = '1' and sc7 < s_dens(7 downto 2) then
                if sc7(1 downto 0) = "11" then
                    v_ts(14) := C_SC;  v_tp(14) := hA7(2 downto 0);
                else
                    v_ts(14) := C_SL;  v_tp(14) := hB7(2 downto 0);
                end if;
            end if;

            -- 15 WAVES: wavy lines, thick ink + thin pale
            case wav7(4 downto 2) is
                when "000" | "001" => v_ts(15) := C_SD;  v_tp(15) := "000";
                when "100"         => v_ts(15) := C_SL;  v_tp(15) := "001";
                when others        => null;
            end case;

            ts8 <= v_ts;
            tp8 <= v_tp;
            gr8 <= gr7;
            -- INKS ARE GOLD AND BROWN: a dark ink is the paint moved INK
            -- steps (plus a per-cell 0..3) down the gold/brown ramp toward
            -- umber, a light ink the same distance up toward bright gold,
            -- so the ornament stays in v0.10's tones and keeps the
            -- picture's light and shade
            v_ik := signed(resize(pq_sr(3 + C_PDLY), 7));
            pqk8 <= pq_sr(3 + C_PDLY);
            v_ia := v_ik - signed(resize(s_ink_t, 7)) - signed(resize(hA7(11 downto 10), 7));
            if v_ia < 0 then
                dk8 <= (others => '0');
            else
                dk8 <= unsigned(v_ia(4 downto 0));
            end if;
            v_ia := v_ik + signed(resize(s_ink_t, 7)) + signed(resize(hA7(10 downto 9), 7));
            if v_ia > 31 then
                lt8 <= (others => '1');
            else
                lt8 <= unsigned(v_ia(4 downto 0));
            end if;

            --------------------------------------------------------------
            -- E9: pick the texture the picture asked for, and the ramp
            -- step its ink sits on (INVERT swaps dark and light; JEWELS
            -- off turns a jewel into a light ink)
            --------------------------------------------------------------
            sel9  <= ts8(to_integer(s_tex));
            pidx9 <= tp8(to_integer(s_tex));
            edge_sr(9) <= s_edge;
            jw9 <= '0';
            case ts8(to_integer(s_tex)) is
                when C_SD =>
                    if s_inv = '0' then ix9 <= dk8; else ix9 <= lt8; end if;
                when C_SL =>
                    if s_inv = '0' then ix9 <= lt8; else ix9 <= dk8; end if;
                when C_SC =>
                    if s_jewel = '1' then
                        ix9 <= pqk8;
                        jw9 <= '1';
                    else
                        ix9 <= lt8;
                    end if;
                when others =>
                    ix9 <= pqk8;
            end case;
            gr9 <= gr8;

            --------------------------------------------------------------
            -- E10: gold/brown ramp read (paint, or the ink's ramp step) |
            -- jewel colour
            --------------------------------------------------------------
            by10 <= C_RAMP_Y(to_integer(ix9));
            bu10 <= C_RAMP_U(to_integer(ix9));
            bv10 <= C_RAMP_V(to_integer(ix9));
            v_pi := to_integer(pidx9);
            jy10 <= C_JW_Y(v_pi);  ju10 <= C_JW_U(v_pi);  jv10 <= C_JW_V(v_pi);
            jw10 <= jw9;
            gr10 <= gr9;
            edge_sr(10) <= edge_sr(9);

            --------------------------------------------------------------
            -- E11: leaf grain | jewel - gold | E12: x INK | E13: add
            --------------------------------------------------------------
            v_py := signed(resize(by10, 12)) + resize(gr10, 12);
            py11 <= f_cu10(v_py);
            pu11 <= bu10;
            pv11 <= bv10;
            if jw10 = '1' then
                d11_y <= signed(resize(jy10, 11)) - signed(resize(by10, 11));
                d11_u <= signed(resize(ju10, 11)) - signed(resize(bu10, 11));
                d11_v <= signed(resize(jv10, 11)) - signed(resize(bv10, 11));
            else
                d11_y <= (others => '0'); d11_u <= (others => '0'); d11_v <= (others => '0');
            end if;
            t11_y <= s_ink_t;  t11_u <= s_ink_t;  t11_v <= s_ink_t;
            edge_sr(11) <= edge_sr(10);

            m12_y <= d11_y * signed('0' & t11_y);
            m12_u <= d11_u * signed('0' & t11_u);
            m12_v <= d11_v * signed('0' & t11_v);
            py12 <= py11;  pu12 <= pu11;  pv12 <= pv11;
            edge_sr(12) <= edge_sr(11);

            v_o := signed(resize(py12, 13)) + resize(m12_y(16 downto 4), 13);
            o13_y <= f_cu10(v_o);
            v_o := signed(resize(pu12, 13)) + resize(m12_u(16 downto 4), 13);
            o13_u <= f_cu10(v_o);
            v_o := signed(resize(pv12, 13)) + resize(m12_v(16 downto 4), 13);
            o13_v <= f_cu10(v_o);
            edge_sr(13) <= edge_sr(12);

            --------------------------------------------------------------
            -- E14: OUTLINE -- dark contour where the texture changes
            --------------------------------------------------------------
            if s_outl = '1' and edge_sr(13) = '1' then
                o14_y <= C_INK_Y;  o14_u <= C_INK_U;  o14_v <= C_INK_V;
            else
                o14_y <= o13_y;  o14_u <= o13_u;  o14_v <= o13_v;
            end if;

            --------------------------------------------------------------
            -- E15: [1 2 1]/4 horizontal smoothing on Y, U and V (the HD
            -- HDMI thick-bar fix; c1 is the centre)
            --------------------------------------------------------------
            c1_y <= o14_y;  c2_y <= c1_y;
            c1_u <= o14_u;  c2_u <= c1_u;
            c1_v <= o14_v;  c2_v <= c1_v;
            v_sy := resize(c2_y, 12) + shift_left(resize(c1_y, 12), 1) + resize(o14_y, 12);
            v_su := resize(c2_u, 12) + shift_left(resize(c1_u, 12), 1) + resize(o14_u, 12);
            v_sv := resize(c2_v, 12) + shift_left(resize(c1_v, 12), 1) + resize(o14_v, 12);
            o15_y <= v_sy(11 downto 2);
            o15_u <= v_su(11 downto 2);
            o15_v <= v_sv(11 downto 2);

            --------------------------------------------------------------
            -- E16: luma scaled toward black (64 + 13/16 (y - 64), white
            -- ~775) | blanking gate on the generated avid | U/V swap out
            --------------------------------------------------------------
            if o15_y > 64 then
                v_dl := o15_y - 64;
            else
                v_dl := (others => '0');
            end if;
            v_yo := o15_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- v3.1.1: STANDARD order out (u = Cb, v = Cr). With the
            -- generated avid, the usual hardware U/V swap made v3.0/3.1
            -- show blue where the gold should be (Ron, HW 2026-10-12; the
            -- sim showed the same blue).
            if sr_ga(C_LATENCY - 6) = '0' then
                o_y <= to_unsigned(64, 10);
                o_u <= to_unsigned(512, 10);
                o_v <= to_unsigned(512, 10);
            elsif s_uvsw = '0' then
                o_y <= v_yo;
                o_u <= o15_u;
                o_v <= o15_v;
            else
                o_y <= v_yo;
                o_u <= o15_v;
                o_v <= o15_u;
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
            sr_ga        <= g_avid          & sr_ga       (0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_u);
    data_out.v       <= std_logic_vector(o_v);
    data_out.avid    <= sr_ga(C_LATENCY - 5);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture gilt;
