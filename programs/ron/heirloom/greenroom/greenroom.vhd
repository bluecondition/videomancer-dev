-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Greenroom v0.5.0 -- Heirloom #54, rebuilt from television.jpg (a painted paper collage).
--
-- The collage: green and teal acrylic washed and scraped over magazine cut-outs.  A portable TV
-- (dark cabinet, pale-green control panel with two dials, a grille) shows a woman's face -- the one
-- clean, bright picture.  Around it: an orange paint blob and a teal clock (top left), a brown onion
-- vase under a dark funnel, pink hands over the set, a blue scraped mountain with orange flecks, a
-- child's face with yellow spirals (bottom left), orange splatters, a man's suit (right), dark
-- scraped teal with cream flecks (bottom right) and a caption strip "...cture yourself on this pa...".
--
-- The program: a 96 x 120-cell layout map of the collage (12 EBR, 4 x 3 tiles of 16 fitted colours)
-- painted through a triangle-wave + streak warp (organic, scraped edges) and a multiplicative
-- grain / streak / blotch texture matched to the photo's local contrast; the 16:9 sides continue as
-- a quiet dark wash (the darkest four colours of the adjacent tile, picked by the stroke hash).  Drawn
-- over it: the clock numerals, the TV (screen, rim, panel with dials, grille, cabinet lines) and the
-- caption (a 1-bit glyph strip).  The camera (or the Osc portrait) is written into a 128 x 64 store
-- and shown, clean and bright, on the TV screen and teal-tinted in the clock centre.  P12 Wash scrapes
-- the paint off (the live camera picture under the collage) or floods it (deep green; only the TV and
-- the caption float).
--
--   K1  Orange  0 = the orange paint dissolves into the wash, 50 % = the photo, 100 % = orange drips
--   K2  Brush   warp of the paint edges (0 = mosaic cells .. 2x the photo)
--   K3  Grain   texture contrast (0 = flat .. 2x the photo)
--   K4  Tuning  the TV picture rolls and snows (0 = locked, the photo)
--   K5  Speed   bipolar paint flow (centre = still)
--   K6  Tint    wash hue: yellow-green .. the photo .. teal
--   S7  Crawl   the caption crawls
--   S8  Screen  Colour / Mono
--   S9  Clock   Camera / Paint (the clock centre)
--   S10 Lines   scanlines on the TV picture
--   S11 Source  Video / Osc
--   P12 WASH    0 = scraped (the camera under the collage), 50 % = the photo, 100 % = flooded
--
-- Virtual 1080-line coordinates from the timing ID (page centred, page height = frame height); the
-- DDA starts and steps are per-mode constant tables.  Element shapes come from small square tables
-- (per-line terms sequenced through one table).  Constant compares are built from gates (f_ltu) so
-- they map to LUTs, not carry chains.  Delay rings in EBR carry the syncs and the chroma filter's
-- 8-back sample.  Generated avid; output conditioning (luma [1 2 1] and toward ~777, chroma
-- [1 2 2 2 2 2 2 2 1]/16) before the blanking gate.  Latency 28 in every mode.  Colours in the
-- hardware convention (u = Cr, v = Cb).  EBR: map 12, palette 1, glyphs 1, store 8 + 1, rings 2.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.greenroom_pkg.all;
use work.all;

architecture greenroom of program_top is
    constant C_LAT : integer := 28;

    ---------------------------------------------------------------------------
    -- mode tables.  Class: 0 1080p, 1 1080i, 2 720p, 3 480i, 4 480p, 5 576i, 6 576p.
    ---------------------------------------------------------------------------
    function f_cls(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(3, 3);
            when 4          => return to_unsigned(4, 3);
            when 8          => return to_unsigned(5, 3);
            when 12         => return to_unsigned(6, 3);
            when 5 | 6 | 14 => return to_unsigned(2, 3);
            when 1 | 2 | 10 => return to_unsigned(1, 3);
            when others     => return to_unsigned(0, 3);
        end case;
    end function;
    subtype t_i16 is integer range -32768 to 32767;
    subtype t_i22 is integer range -2097152 to 2097151;
    type t_ytab is record
        ys   : t_i16;                       -- virtual lines per line, Q.8
        dl0, dl1, dl2, dl3 : t_i16;         -- wave phases per line, Q.4 (12-bit phase)
        dv   : t_i16;                       -- map rows per line, Q.12
        dsv, dsv2, dgy : t_i16;             -- screen / clock store rows, glyph rows per line, Q.12
    end record;
    function f_ytab(c : unsigned(2 downto 0)) return t_ytab is
    begin
        case to_integer(c) is
            when 1 => return (512, -1824, 2592, 3008, -1152, 910, 3495, 7825, 7282);
            when 2 => return (384, -1368, 1944, 2256, -864, 683, 2621, 5869, 5461);
            when 3 => return (1138, -4055, 5761, 6685, -2561, 2023, 7769, 17393, 16185);
            when 4 => return (576, -2052, 2916, 3384, -1296, 1024, 3932, 8803, 8192);
            when 5 => return (960, -3420, 4860, 5640, -2160, 1707, 6554, 14672, 13653);
            when 6 => return (480, -1710, 2430, 2820, -1080, 853, 3277, 7336, 6827);
            when others => return (256, -912, 1296, 1504, -576, 455, 1748, 3913, 3641);
        end case;
    end function;
    -- page x (half units) at pixel 0 and the values that start there; sd = SD (720 wide), fl = Flip
    type t_ptab is record
        p0  : t_i16;
        u0  : t_i22;                        -- map column, Q.12 cells (pxh * 228)
        f0, f1, f2, f3 : t_i16;             -- wave phases (pxh * kx/2, 12 bit)
        su0, su0c, gx0 : t_i22;             -- screen / clock store column, glyph column (Q.8)
        a1, a2 : integer range 0 to 65535;  -- hash column Weyl starts
        ocx : t_i16;                        -- Osc centre, source half units
    end record;
    function f_ptab(sd : std_logic) return t_ptab is
    begin
        if sd = '1' then return (-576, -131328, 3968, 960, 320, 640, -91322, -138200, -143252, 32904, 6516, 1440);
        else return (-1056, -240768, 448, 3808, 4000, 3904, -128282, -220280, -199892, 27556, 11946, 1920); end if;
    end function;
    -- per-pixel steps: xs half-px per pixel, signed by Flip
    type t_xtab is record
        xs  : t_i16;
        xsa : t_i16;                        -- |xs|
        du  : t_i16;                        -- 228 xs
        ax0, ax1, ax2, ax3 : t_i16;         -- kx/2 * xs
        dsu, dsuc, dgx : t_i16;             -- 77 xs, 171 xs, 118 xs
    end record;
    function f_xtab(xc : unsigned(1 downto 0)) return t_xtab is
        variable s : integer;
    begin
        case to_integer(xc) is
            when 1 => s := 3;
            when 2 => s := 4;
            when others => s := 2;
        end case;
        return (s, s, 228 * s, 50 * s, -23 * s, 35 * s, -58 * s, 77 * s, 171 * s, 118 * s);
    end function;
    -- store write: capture the camera's central 68 % x 85 % into 128 x 64
    type t_wcfg is record
        x0 : unsigned(8 downto 0);    -- first pixel (0.16 W)
        kc : unsigned(14 downto 0);   -- store columns per pixel, Q.16
        n  : unsigned(1 downto 0);    -- log2 pixels averaged per column
        y0 : unsigned(6 downto 0);    -- first line (0.075 field lines)
        kr : unsigned(14 downto 0);   -- store rows per line, Q.16
        m  : unsigned(1 downto 0);    -- log2 lines averaged per row
    end record;
    function f_wcfg(c : unsigned(2 downto 0)) return t_wcfg is
    begin
        case to_integer(c) is
            when 3 => return (to_unsigned(115, 9), to_unsigned(17134, 15), "01", to_unsigned(18, 7), to_unsigned(20560, 15), "01");
            when 4 => return (to_unsigned(115, 9), to_unsigned(17134, 15), "01", to_unsigned(36, 7), to_unsigned(10280, 15), "10");
            when 5 => return (to_unsigned(115, 9), to_unsigned(17134, 15), "01", to_unsigned(22, 7), to_unsigned(17134, 15), "01");
            when 6 => return (to_unsigned(115, 9), to_unsigned(17134, 15), "01", to_unsigned(43, 7), to_unsigned(8567, 15), "10");
            when 2 => return (to_unsigned(205, 9), to_unsigned(9638, 15), "10", to_unsigned(54, 7), to_unsigned(6853, 15), "11");
            when 1 => return (to_unsigned(307, 9), to_unsigned(6425, 15), "11", to_unsigned(41, 7), to_unsigned(9137, 15), "10");
            when others => return (to_unsigned(307, 9), to_unsigned(6425, 15), "11", to_unsigned(81, 7), to_unsigned(4569, 15), "11");
        end case;
    end function;

    -- hash constants
    constant K1H : unsigned(15 downto 0) := x"9E37";   -- column steps
    constant K2H : unsigned(15 downto 0) := x"7F4B";
    constant K1V : unsigned(15 downto 0) := x"6A09";   -- row steps
    constant K2V : unsigned(15 downto 0) := x"58F1";
    constant K3X : unsigned(15 downto 0) := x"9E37";   -- grain
    constant K3Y : unsigned(15 downto 0) := x"6A09";
    constant SEED1 : unsigned(15 downto 0) := x"5555";
    constant SEED2 : unsigned(15 downto 0) := x"3C3C";
    constant SEED3 : unsigned(15 downto 0) := x"1234";

    function f_tri12(p : unsigned(11 downto 0)) return unsigned is     -- 0..255 triangle of a 12-bit phase
    begin
        if p(11) = '1' then return not p(10 downto 3); else return p(10 downto 3); end if;
    end function;
    -- x < c for a constant c, as gates (yosys maps "<" to a carry chain; these become LUTs)
    function f_ltu(x : unsigned; c : integer) return boolean is
        constant n : integer := x'length;
        variable xv : unsigned(n - 1 downto 0);
        variable cv : unsigned(n - 1 downto 0);
        variable lt : boolean := false;
        variable eq : boolean := true;
    begin
        if c <= 0 then return false; end if;
        if n < 31 then
            if c >= 2 ** n then return true; end if;
        end if;
        xv := x; cv := to_unsigned(c, n);
        for i in n - 1 downto 0 loop
            if cv(i) = '1' then lt := lt or (eq and xv(i) = '0'); end if;
            eq := eq and (xv(i) = cv(i));
        end loop;
        return lt;
    end function;
    function f_lts(x : signed; c : integer) return boolean is
        constant n : integer := x'length;
        variable u : unsigned(n - 1 downto 0);
    begin
        u := unsigned(x);
        u(n - 1) := not u(n - 1);
        return f_ltu(u, c + 2 ** (n - 1));
    end function;
    function f_in(x : unsigned; a, b : integer) return boolean is      -- a <= x < b
    begin
        return (not f_ltu(x, a)) and f_ltu(x, b);
    end function;
    function f_ins(x : signed; a, b : integer) return boolean is
    begin
        return (not f_lts(x, a)) and f_lts(x, b);
    end function;
    function f_abs13(v : signed(12 downto 0)) return unsigned is      -- |v|, 12 bits
        variable a : signed(12 downto 0);
    begin
        if v < 0 then a := -v; else a := v; end if;
        return unsigned(a(11 downto 0));
    end function;
    type t_sq5 is array (0 to 31) of unsigned(9 downto 0);
    function f_sq5i return t_sq5 is
        variable r : t_sq5;
    begin
        for i in 0 to 31 loop r(i) := to_unsigned(i * i, 10); end loop;
        return r;
    end function;
    constant C_SQ5 : t_sq5 := f_sq5i;
    type t_sq6 is array (0 to 63) of unsigned(11 downto 0);
    function f_sq6i return t_sq6 is
        variable r : t_sq6;
    begin
        for i in 0 to 63 loop r(i) := to_unsigned(i * i, 12); end loop;
        return r;
    end function;
    constant C_SQ6 : t_sq6 := f_sq6i;

    ---------------------------------------------------------------------------
    -- colours {Y, U = Cr, V = Cb}
    ---------------------------------------------------------------------------
    type t_c3 is array (0 to 2) of unsigned(9 downto 0);
    function c3(y, u, v : integer) return t_c3 is
    begin
        return (to_unsigned(y, 10), to_unsigned(u, 10), to_unsigned(v, 10));
    end function;
    constant C_WASH   : t_c3 := c3(258, 432, 516);
    constant C_OBG    : t_c3 := c3(392, 373, 529);
    constant C_OHAIR  : t_c3 := c3(634, 588, 374);
    constant C_OHAIR2 : t_c3 := c3(482, 583, 402);
    constant C_OFACE  : t_c3 := c3(714, 619, 426);
    constant C_OEYE   : t_c3 := c3(206, 509, 533);
    constant C_OMOUTH : t_c3 := c3(414, 748, 449);
    constant C_OCOLL  : t_c3 := c3(844, 510, 486);
    -- element colours by index (luma pre-boosted 16/13 with a knee; 0 = paint, 4 = the tinted wash)
    function f_col(i : integer range 0 to 18) return t_c3 is
    begin
        case i is
            when 1  => return c3(649, 757, 353);    -- orange drip
            when 3  => return c3(208, 482, 512);    -- clock numerals
            when 5  => return c3(876, 548, 433);    -- cream scraped edge
            when 6  => return c3(455, 455, 451);    -- cabinet lines
            when 7  => return c3(273, 483, 485);    -- grille
            when 8  => return c3(523, 471, 440);    -- grille lines
            when 9  => return c3(350, 503, 525);    -- grille, lavender part
            when 10 => return c3(485, 503, 525);
            when 11 => return c3(697, 478, 404);    -- control panel
            when 12 => return c3(875, 505, 455);    -- dial rim
            when 13 => return c3(181, 503, 505);    -- dial ring
            when 14 => return c3(734, 491, 436);    -- dial centre
            when 15 => return c3(885, 509, 480);    -- dial bar
            when 16 => return c3(620, 483, 485);    -- screen rim
            when 17 => return c3(460, 452, 409);    -- caption strip
            when 18 => return c3(244, 496, 487);    -- caption letters
            when others => return c3(258, 432, 516); -- wash
        end case;
    end function;

    ---------------------------------------------------------------------------
    -- memories
    ---------------------------------------------------------------------------
    signal m_ra : unsigned(13 downto 0) := (others => '0');
    signal m_q : std_logic_vector(3 downto 0) := (others => '0');
    signal pal_ra : unsigned(7 downto 0) := (others => '0');
    signal pal_q, pal_q2 : std_logic_vector(15 downto 0) := (others => '0');
    signal gl_ra : unsigned(7 downto 0) := (others => '0');
    signal gl_q, gl_q2 : std_logic_vector(15 downto 0) := (others => '0');
    type t_bank is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal bk0, bk1, bk2, bk3 : t_bank := (others => (others => '0'));    -- bank = row(0) & col(0)
    attribute no_rw_check : boolean;
    attribute no_rw_check of bk0 : signal is true;
    attribute no_rw_check of bk1 : signal is true;
    attribute no_rw_check of bk2 : signal is true;
    attribute no_rw_check of bk3 : signal is true;
    type t_cst is array (0 to 511) of std_logic_vector(7 downto 0);
    signal cst : t_cst := (others => (others => '0'));                     -- chroma U4 & V4 (signed)
    attribute no_rw_check of cst : signal is true;
    signal b_we : std_logic_vector(3 downto 0) := (others => '0');
    signal b_wa : unsigned(10 downto 0) := (others => '0');
    signal b_wd : std_logic_vector(3 downto 0) := (others => '0');
    signal b_ra0, b_ra1, b_ra2, b_ra3 : unsigned(10 downto 0) := (others => '0');
    signal b_q0, b_q1, b_q2, b_q3 : std_logic_vector(3 downto 0) := (others => '0');
    signal c_we : std_logic := '0';
    signal c_wa, c_ra : unsigned(8 downto 0) := (others => '0');
    signal c_wd, c_q : std_logic_vector(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- timing, controls
    ---------------------------------------------------------------------------
    signal s_tid : std_logic_vector(3 downto 0) := (others => '1');
    signal s_cls : unsigned(2 downto 0) := (others => '0');
    signal s_xc : unsigned(1 downto 0) := (others => '0');
    signal s_sd : std_logic := '0';
    signal s_yt : t_ytab := f_ytab("000");
    signal s_pt : t_ptab := f_ptab('0');
    signal s_xt : t_xtab := f_xtab("00");
    signal s_wc : t_wcfg := f_wcfg("000");
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(10 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(3 downto 0) := (others => '1');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_lines, s_crawl, s_mono, s_clkp, s_osc : std_logic := '0';
    signal r_lpar : std_logic := '0';
    type t_g is array (0 to 2) of unsigned(12 downto 0);                   -- glides Q.3: P12 K2 K3
    signal s_g : t_g := (to_unsigned(4096, 13), to_unsigned(4096, 13), to_unsigned(4096, 13));
    signal s_k1, s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_kn : unsigned(12 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := (others => '0');
    signal s_spr, s_sp : signed(9 downto 0) := (others => '0');
    signal s_gs3 : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal s_gh : unsigned(1 downto 0) := "10";
    signal s_gt4 : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_tho : signed(8 downto 0) := to_signed(64, 9);
    signal s_dens : unsigned(4 downto 0) := (others => '0');
    signal s_tu, s_tv : signed(6 downto 0) := (others => '0');
    signal s_wu, s_wv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ths : signed(9 downto 0) := to_signed(-60, 10);
    signal s_scr, s_fld : std_logic := '0';
    signal s_pa : unsigned(9 downto 0) := (others => '0');
    signal s_snow : unsigned(3 downto 0) := (others => '0');
    signal s_roll : unsigned(15 downto 0) := (others => '0');               -- screen rows, Q.8 (wraps at 64 rows)
    signal s_cr8 : unsigned(15 downto 0) := (others => '0');                -- caption crawl, Q.8 glyph columns
    signal s_dr : unsigned(11 downto 0) := (others => '0');                 -- paint flow (wave drift)
    signal s_t : unsigned(9 downto 0) := (others => '0');
    signal s_sway : signed(9 downto 0) := (others => '0');                  -- Osc head sway, half units
    signal s_blink : std_logic := '0';
    signal s_ocx : unsigned(12 downto 0) := to_unsigned(1920, 13);          -- Osc centre, source half units

    ---------------------------------------------------------------------------
    -- per-line state (reset at the field start, advanced at the generated avid fall)
    ---------------------------------------------------------------------------
    signal l_vy : unsigned(18 downto 0) := (others => '0');                 -- vy, Q.8
    signal l_v : unsigned(18 downto 0) := (others => '0');                  -- map row, Q.12
    signal l_r1, l_r2 : unsigned(15 downto 0) := (others => '0');
    signal l_qh0, l_qhn : signed(12 downto 0) := (others => '0');
    signal l_vyp : unsigned(18 downto 0) := (others => '0');
    signal l_f1, l_f2 : std_logic := '0';
    signal l_a1s, l_a2s, l_a3s : unsigned(15 downto 0) := (others => '0');
    type t_u16x4 is array (0 to 3) of unsigned(15 downto 0);
    signal l_ly : t_u16x4 := (others => (others => '0'));                   -- wave phases, Q.4
    signal l_sv, l_sv2, l_gy : signed(22 downto 0) := (others => '0');      -- Q.12
    -- row values of the current line
    signal r_vi : unsigned(10 downto 0) := (others => '0');
    signal r_pan, r_gri, r_glo, r_cab, r_band, r_gly, r_dial, r_clk, r_cwin : std_logic := '0';
    signal r_gy : unsigned(3 downto 0) := (others => '0');
    signal r_ey2 : unsigned(9 downto 0) := (others => '1');                -- screen corner ey^2 (px)
    signal r_dd2 : unsigned(9 downto 0) := (others => '1');                -- dial dy^2
    signal r_dyd : signed(6 downto 0) := (others => '0');                   -- dial dy, half units
    signal r_cd2 : unsigned(11 downto 0) := (others => '1');               -- clock dy^2
    signal r_cdy, r_cdy866 : signed(8 downto 0) := (others => '0');       -- clock dy (px), *0.866
    signal r_eyv : unsigned(10 downto 0) := (others => '1');
    signal r_ddv, r_cdv : unsigned(5 downto 0) := (others => '0');
    signal r_sqc : unsigned(1 downto 0) := (others => '0');
    signal r_sv : unsigned(13 downto 0) := (others => '0');                 -- store row, Q.8 (0..64)
    signal r_svok : std_logic := '0';
    signal r_g3 : unsigned(1 downto 0) := (others => '0');                  -- line mod 3 (grille)
    -- Osc rows
    signal o_hum, o_eyer, o_mthr : std_logic := '0';
    signal o_hhw, o_fhw, o_chw, o_fx : unsigned(9 downto 0) := (others => '0');   -- hair / face / collar half-widths, px
    signal o_d470, o_d540, o_d478, o_d644, o_d860 : signed(11 downto 0) := (others => '0');
    signal o_a470, o_a540 : unsigned(10 downto 0) := (others => '0');
    signal o_f0, o_f1 : std_logic := '0';
    signal r_e1, r_e2, r_ddA, r_ddB, r_dc : signed(11 downto 0) := (others => '0');
    signal r_dsel : std_logic := '0';

    ---------------------------------------------------------------------------
    -- generated avid
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start, g_fall : std_logic := '0';
    signal g_ssr : std_logic_vector(14 downto 0) := (others => '0');        -- g_start delays
    signal g_asr : std_logic_vector(14 downto 0) := (others => '0');        -- g_avid delays

    ---------------------------------------------------------------------------
    -- main beam
    ---------------------------------------------------------------------------
    signal b_qh : signed(12 downto 0) := (others => '0');
    signal b_u : signed(21 downto 0) := (others => '0');                   -- map column, Q.12 (stage 5)
    type t_u12x4 is array (0 to 3) of unsigned(11 downto 0);
    signal b_f : t_u12x4 := (others => (others => '0'));
    signal b_a1, b_a2, b_a3 : unsigned(15 downto 0) := (others => '0');
    signal b_cy1, b_cy2 : std_logic := '0';
    signal b_gcnt : unsigned(1 downto 0) := (others => '0');                -- pixel mod 3

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    type t_u8x4 is array (0 to 3) of unsigned(7 downto 0);
    signal p1_t : t_u8x4 := (others => (others => '0'));
    signal p1_t5, p1_t6 : unsigned(7 downto 0) := (others => '0');
    signal p1_w1, p1_w2, p1_w3 : unsigned(15 downto 0) := (others => '0');
    signal p2_sx, p2_sy : signed(10 downto 0) := (others => '0');
    signal p2_h1, p2_h2, p2_h3 : unsigned(15 downto 0) := (others => '0');
    signal p3_jxa, p3_jxb, p3_jya, p3_jyb : signed(14 downto 0) := (others => '0');
    signal p3_sxy : signed(11 downto 0) := (others => '0');
    signal p3_h1, p3_h2, p3_h3 : unsigned(15 downto 0) := (others => '0');
    signal p4_jxs, p4_jys : signed(14 downto 0) := (others => '0');
    signal p4_sxy : signed(11 downto 0) := (others => '0');
    signal p4_h1, p4_h2, p4_h3 : unsigned(15 downto 0) := (others => '0');
    signal p5_jx, p5_jy : signed(12 downto 0) := (others => '0');
    signal p5_ta, p5_tb : signed(10 downto 0) := (others => '0');
    signal p6_colf : signed(16 downto 0) := (others => '0');
    signal p6_rowf : signed(15 downto 0) := (others => '0');
    signal p6_t : signed(11 downto 0) := (others => '0');
    signal p5_sl, p6_sl : unsigned(1 downto 0) := (others => '0');
    signal p5_sj, p6_sj : unsigned(3 downto 0) := (others => '0');
    signal p7_col : unsigned(6 downto 0) := (others => '0');
    signal p7_row : unsigned(6 downto 0) := (others => '0');
    signal p7_side : std_logic := '0';
    signal p7_br, p7_bc : unsigned(1 downto 0) := (others => '0');
    signal p7_tma, p7_tmb : signed(15 downto 0) := (others => '0');
    signal p7_sl : unsigned(1 downto 0) := (others => '0');
    signal p8_bank, p9_bank : unsigned(3 downto 0) := (others => '0');
    signal p8_side, p9_side : std_logic := '0';
    signal p8_sl, p9_sl : unsigned(1 downto 0) := (others => '0');
    type t_s6a is array (natural range <>) of signed(5 downto 0);
    signal p_tm : t_s6a(8 to 15) := (others => (others => '0'));
    type t_s7a is array (natural range <>) of signed(6 downto 0);
    signal p_n : t_s7a(5 to 12) := (others => (others => '0'));
    signal p_drip : std_logic_vector(5 to 14) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := (others => '0');
    signal p13_dep : unsigned(7 downto 0) := (others => '0');
    signal p13_or, p13_og : std_logic := '0';
    signal p13_ns : signed(10 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := (others => '0');
    signal p14_rev, p14_edge, p14_fld, p14_og : std_logic := '0';
    signal p15_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p15_u, p15_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p15_tx : unsigned(1 downto 0) := (others => '0');                 -- texture 0 none / 1 half / 2 full
    signal p15_cam : unsigned(1 downto 0) := (others => '0');                -- 0 none 1 screen 2 reveal 3 clock
    signal p16_py : signed(15 downto 0) := (others => '0');
    signal p16_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p16_cam : unsigned(1 downto 0) := (others => '0');
    signal p17_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p17_u, p17_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p17_cam : unsigned(1 downto 0) := (others => '0');
    signal p18_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p18_u, p18_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ---------------------------------------------------------------------------
    -- element beam (stage 10) and the store reader
    ---------------------------------------------------------------------------
    signal e2_pxh : signed(12 downto 0) := (others => '0');
    signal e2_su, e2_gx : signed(19 downto 0) := (others => '0');
    signal e2_dsu : signed(11 downto 0) := (others => '0');
    signal x14_ds, x14_dd : unsigned(11 downto 0) := (others => '0');
    signal x14_dc, x14_ddx : signed(12 downto 0) := (others => '0');
    signal x14_pan, x14_gri, x14_glo, x14_cab, x14_band, x14_gok, x14_gline : std_logic := '0';
    signal x14_gbit : unsigned(3 downto 0) := (others => '0');
    signal x15_ein, x15_eout, x15_dok, x15_cok, x15_bar : std_logic := '0';
    signal x15_ex2 : unsigned(9 downto 0) := (others => '0');
    signal x15_dx2 : unsigned(9 downto 0) := (others => '0');
    signal x15_cx2 : unsigned(11 downto 0) := (others => '0');
    signal x15_cx, x15_c8 : signed(8 downto 0) := (others => '0');
    signal x15_pan, x15_gri, x15_glo, x15_cab, x15_band, x15_gok, x15_gline : std_logic := '0';
    signal x15_gbit : unsigned(3 downto 0) := (others => '0');
    signal x16_gok : std_logic := '0';
    signal x16_gbit : unsigned(3 downto 0) := (others => '0');
    type t_fl is record
        scr, rim, d1, d2, d3, bar, ring, cin, pan, gri, glo, gline, cab, band, gly, mark : std_logic;
    end record;
    constant C_FL0 : t_fl := (others => '0');
    signal x16_fl, x17_fl : t_fl := C_FL0;
    signal s14_u0, s14_v0 : unsigned(6 downto 0) := (others => '0');
    signal s14_fu, s14_fv : unsigned(3 downto 0) := (others => '0');
    signal s14_ok : std_logic := '0';
    type t_u4a is array (natural range <>) of unsigned(3 downto 0);
    signal s_fu, s_fv : t_u4a(15 to 18) := (others => (others => '0'));
    signal s_upar, s_vpar : std_logic_vector(15 to 17) := (others => '0');
    signal s_ok : std_logic_vector(15 to 19) := (others => '0');
    signal s17_q0, s17_q1, s17_q2, s17_q3 : unsigned(3 downto 0) := (others => '0');
    signal s18_a, s18_b : unsigned(8 downto 0) := (others => '0');       -- Q.4 luma, rows v0 / v0+1
    signal s19_l : unsigned(8 downto 0) := (others => '0');
    signal s20_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s20_u, s20_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s17_cq, s18_cq, s19_cq : std_logic_vector(7 downto 0) := (others => '0');
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal s_cwin18, s_cwin19 : std_logic := '0';
    -- camera tap (reveal)
    signal t1_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal t1_u, t1_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ---------------------------------------------------------------------------
    -- output conditioning
    ---------------------------------------------------------------------------
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_cring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal s_cring : t_cring := (others => x"8080");
    attribute no_rw_check of s_cring : signal is true;
    signal s_cwp, s_crp : unsigned(7 downto 0) := (others => '0');
    signal s_cq, s_cq8 : std_logic_vector(15 downto 0) := x"8080";
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal s_sring : t_sring := (others => "1110");
    attribute no_rw_check of s_sring : signal is true;
    signal s_swp, s_srp : unsigned(7 downto 0) := (others => '0');
    signal s_sq, s_do : std_logic_vector(3 downto 0) := "1110";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y19, s_y20, s_y21, s_y22, s_y23, s_y24x, s_yf : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uf, s_vf : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yo : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uo, s_vo : unsigned(9 downto 0) := to_unsigned(512, 10);

    ---------------------------------------------------------------------------
    -- source side: store writer and the Osc generator
    ---------------------------------------------------------------------------
    signal w_si : unsigned(11 downto 0) := (others => '0');
    signal w_cacc : unsigned(15 downto 0) := (others => '0');
    signal w_col : unsigned(7 downto 0) := (others => '0');
    signal w_cnt : unsigned(3 downto 0) := (others => '0');
    signal w_sy : unsigned(13 downto 0) := (others => '0');
    signal w_emit : std_logic := '0';
    signal w_ey : unsigned(13 downto 0) := (others => '0');
    signal w_eu, w_ev : unsigned(9 downto 0) := (others => '0');
    signal w_ecol : unsigned(6 downto 0) := (others => '0');
    signal w_skip : unsigned(6 downto 0) := (others => '0');
    signal w_racc : unsigned(23 downto 0) := (others => '0');
    signal w_row : unsigned(6 downto 0) := (others => '0');
    signal w_lcnt : unsigned(3 downto 0) := (others => '0');
    signal w_rok, w_first : std_logic := '0';
    -- Osc
    signal o_sxh : signed(12 downto 0) := (others => '0');
    signal o_hair, o_face, o_eye, o_mouth, o_coll, o_str : std_logic := '0';
    signal o_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal o_u, o_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal x_y, x_u, x_v : unsigned(9 downto 0) := to_unsigned(64, 10);    -- source pixel (camera or Osc)
    signal x_av : std_logic := '0';

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_map : process(clk)
    begin
        if rising_edge(clk) then
            m_q <= C_GRMAP(to_integer(m_ra));
        end if;
    end process p_map;
    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            pal_q <= C_GRPAL(to_integer(pal_ra));
        end if;
    end process p_pal;
    p_gly : process(clk)
    begin
        if rising_edge(clk) then
            gl_q <= C_GRGLY(to_integer(gl_ra));
        end if;
    end process p_gly;
    p_bank0 : process(clk)
    begin
        if rising_edge(clk) then
            b_q0 <= bk0(to_integer(b_ra0));
            if b_we(0) = '1' then bk0(to_integer(b_wa)) <= b_wd; end if;
        end if;
    end process;
    p_bank1 : process(clk)
    begin
        if rising_edge(clk) then
            b_q1 <= bk1(to_integer(b_ra1));
            if b_we(1) = '1' then bk1(to_integer(b_wa)) <= b_wd; end if;
        end if;
    end process;
    p_bank2 : process(clk)
    begin
        if rising_edge(clk) then
            b_q2 <= bk2(to_integer(b_ra2));
            if b_we(2) = '1' then bk2(to_integer(b_wa)) <= b_wd; end if;
        end if;
    end process;
    p_bank3 : process(clk)
    begin
        if rising_edge(clk) then
            b_q3 <= bk3(to_integer(b_ra3));
            if b_we(3) = '1' then bk3(to_integer(b_wa)) <= b_wd; end if;
        end if;
    end process;
    p_cst : process(clk)
    begin
        if rising_edge(clk) then
            c_q <= cst(to_integer(c_ra));
            if c_we = '1' then cst(to_integer(c_wa)) <= c_wd; end if;
        end if;
    end process;

    -- delay rings: chroma 8 samples back (conditioning), syncs C_LAT - 1 (written every clock)
    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            s_cq <= s_cring(to_integer(s_crp));
            s_cring(to_integer(s_cwp)) <= std_logic_vector(p18_u(9 downto 2)) & std_logic_vector(p18_v(9 downto 2));
        end if;
    end process p_cring;
    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            s_sq <= s_sring(to_integer(s_srp));
            s_sring(to_integer(s_swp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & g_avid;
        end if;
    end process p_sring;

    -- mode tables (combinational from the registered mode class)
    s_yt <= f_ytab(s_cls);
    s_pt <= f_ptab(s_sd);
    s_xt <= f_xtab(s_xc);
    s_ys <= to_unsigned(s_yt.ys, 12);

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, width, per-field values, sync delays
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_p : signed(13 downto 0);
        variable v_tri : unsigned(9 downto 0);
        variable v_kk : unsigned(9 downto 0);
        variable v_kn : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_cls <= f_cls(s_tid);
            s_wc <= f_wcfg(s_cls);
            case to_integer(s_cls) is
                when 0 | 1 => s_xc <= "00"; s_sd <= '0';
                when 2     => s_xc <= "01"; s_sd <= '0';
                when others => s_xc <= "10"; s_sd <= '1';
            end case;

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle(10) = '0' then
                s_idle <= s_idle + 1;
            end if;
            s_zone <= s_idle(10);
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zgo = '1' then s_zc <= (others => '0'); elsif s_zc /= 15 then s_zc <= s_zc + 1; end if;

            -- width: source avid pixels per line, max over the field (generated avid length)
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;          -- K5 Speed, deadband
            if f_ins(v_s, -15, 16) then s_spr <= (others => '0'); else s_spr <= v_s(9 downto 0); end if;
            s_k4 <= unsigned(registers_in(3));

            if s_zgo = '1' then
                if s_wmax /= 0 then s_w <= s_wmax; end if;
                s_wmax <= (others => '0');
                s_sp <= s_spr;
                s_crawl <= registers_in(6)(0);
                s_mono  <= registers_in(6)(1);
                s_clkp  <= registers_in(6)(2);
                s_lines <= registers_in(6)(3);
                s_k1 <= unsigned(registers_in(0));
                s_k6 <= unsigned(registers_in(5));
                s_osc   <= registers_in(6)(4);
                s_t <= s_t + 1;
                -- wave drift (12-bit phases): rates +1, -2, -1, +2 of Speed / 16
                s_dr <= s_dr + unsigned(resize(shift_right(s_sp, 4), 12));
                if s_crawl = '1' then s_cr8 <= s_cr8 + 256; end if;
                -- K4 Tuning: roll (Q.8 rows per field) ~ k4^2
                v_kk := s_k4(9 downto 5) * s_k4(9 downto 5);
                s_roll <= s_roll + resize(v_kk, 16);
            end if;
            -- glides, one channel per clock after the zone start: g += (knob*8 - g) / 8
            case to_integer(s_zc) is
                when 2 => v_kn := unsigned(registers_in(7));
                when 3 => v_kn := unsigned(registers_in(1));
                when others => v_kn := unsigned(registers_in(2));
            end case;
            s_kn <= v_kn & "000";
            if s_zc >= 3 and s_zc <= 5 then
                for i in 0 to 2 loop
                    if to_integer(s_zc) - 3 = i then
                        v_d := signed(resize(s_kn, 14)) - signed(resize(s_g(i), 14));
                        v_d := shift_right(v_d, 3);
                        if v_d = 0 and s_kn /= s_g(i) then
                            if s_kn > s_g(i) then v_d := to_signed(1, 14); else v_d := to_signed(-1, 14); end if;
                        end if;
                        v_d := signed(resize(s_g(i), 14)) + v_d;
                        s_g(i) <= unsigned(v_d(12 downto 0));
                    end if;
                end loop;
            end if;

            -- derived per-field values (settle during vblank)
            g_wm1 <= s_w - 1;
            s_gs3 <= s_g(1)(12 downto 10);                                         -- K2 Brush: wave warp 0..7/4
            s_gh  <= s_g(1)(12 downto 11);                                         -- streak warp 0, 1/4, 1/2, 1
            s_gt4 <= s_g(2)(12 downto 9);                                          -- K3 Grain
            if s_k1(9) = '0' then                                                  -- K1 Orange
                s_tho <= signed(resize(s_k1(8 downto 2), 9)) - 64;                 -- dissolve into the wash below 50 %
                s_dens <= (others => '0');
            else
                s_tho <= to_signed(64, 9);
                s_dens <= s_k1(8 downto 4);                                        -- drips 0..62 / 256 of the stroke cells
            end if;
            v_d := signed(resize(s_k6, 14)) - 512;                                 -- K6 Tint
            if f_ins(v_d, -7, 8) then v_d := (others => '0'); end if;
            s_tu <= resize(-shift_right(v_d, 4), 7);
            s_tv <= resize(shift_right(v_d, 4) + shift_right(v_d, 6), 7);
            s_wu <= unsigned(to_signed(432, 10) + resize(s_tu, 10));
            s_wv <= unsigned(to_signed(516 - 1024, 10) + resize(s_tv, 10));
            -- P12 Wash: scrape below 50 %, flood above; threshold 0.75 |p - 512| - 60
            v_p := signed(resize(s_g(0)(12 downto 3), 14)) - 512;
            if v_p < 0 then
                s_scr <= '1'; s_fld <= '0'; s_pa <= unsigned(resize(-v_p, 10));
            else
                s_scr <= '0'; s_fld <= '1'; s_pa <= unsigned(resize(v_p, 10));
            end if;
            if v_p = 0 then s_fld <= '0'; end if;
            v_d := signed(resize(s_pa, 14)) - signed(resize(shift_right(s_pa, 2), 14)) - 60;
            s_ths <= resize(v_d, 10);
            s_snow <= s_k4(9 downto 6);
            -- Osc: head sway (triangle, ~8 s), blink, centre
            if s_t(9) = '1' then v_tri := not s_t; else v_tri := s_t; end if;
            s_sway <= signed(resize(v_tri(8 downto 0), 10)) - 256 + signed(resize(v_tri(8 downto 2), 10));
            if f_ltu(s_t(7 downto 0), 6) then s_blink <= '1'; else s_blink <= '0'; end if;
            s_ocx <= unsigned(to_signed(s_pt.ocx, 13) + resize(s_sway, 13));

            -- sync delays: hsync / vsync / field and the generated avid through the ring (read 25 back,
            -- + EBR + 1 = C_LAT - 1 = 27 registers, the same as the pixel path)
            s_swp <= s_swp + 1; s_srp <= s_swp - 25; s_do <= s_sq;
            s_cwp <= s_cwp + 1; s_crp <= s_cwp - 5; s_cq8 <= s_cq;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 2/3 clocks after the source avid rise)
    ---------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;
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
            if g_start = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
            g_avd <= g_avid;
            g_ssr <= g_ssr(13 downto 0) & g_start;
            g_asr <= g_asr(13 downto 0) & g_avid;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';
    g_fall <= g_avd and not g_avid;

    ---------------------------------------------------------------------------
    -- Per-line state: line 0 at the field start (zone + 12), advanced at the generated avid fall.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_vy : unsigned(18 downto 0);
        variable v_sk : unsigned(9 downto 0);
        variable v_q : signed(12 downto 0);
        variable v_vi : unsigned(10 downto 0);
        variable v_d : signed(11 downto 0);
        variable v_ad : unsigned(10 downto 0);
        variable v_sqi : unsigned(5 downto 0);
        variable v_sqo : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_zc = 12 then
                l_vy <= (others => '0');
                l_v <= (others => '0');
                l_r1 <= SEED1; l_r2 <= SEED2;
                l_qh0 <= to_signed(s_pt.p0, 13);
                l_a1s <= to_unsigned(s_pt.a1, 16); l_a2s <= to_unsigned(s_pt.a2, 16);
                l_a3s <= SEED3;
                l_ly(0) <= shift_left(resize(to_unsigned((s_pt.f0 + 256) mod 4096, 12) + s_dr, 16), 4);
                l_ly(1) <= shift_left(resize(to_unsigned((s_pt.f1 + 1344) mod 4096, 12) - (s_dr(10 downto 0) & '0'), 16), 4);
                l_ly(2) <= shift_left(resize(to_unsigned((s_pt.f2 + 816) mod 4096, 12) - s_dr, 16), 4);
                l_ly(3) <= shift_left(resize(to_unsigned((s_pt.f3 + 1904) mod 4096, 12) + (s_dr(10 downto 0) & '0'), 16), 4);
                l_sv <= to_signed(-739246, 23);                     -- (0 - 423) * 64/150, Q.12
                l_sv2 <= to_signed(-1058357, 23);                   -- 32 + (0 - 304) * 64/67
                l_gy <= to_signed(-2428473, 23);                    -- (0 - 667) * 16/18
            elsif g_fall = '1' then
                l_vyp <= l_vy;
                l_vy <= l_vy + s_ys;
                l_v <= l_v + to_unsigned(s_yt.dv, 19);
                l_a3s <= l_a3s + K3Y;
                r_lpar <= not r_lpar;
                l_ly(0) <= l_ly(0) + unsigned(to_signed(s_yt.dl0, 16));
                l_ly(1) <= l_ly(1) + unsigned(to_signed(s_yt.dl1, 16));
                l_ly(2) <= l_ly(2) + unsigned(to_signed(s_yt.dl2, 16));
                l_ly(3) <= l_ly(3) + unsigned(to_signed(s_yt.dl3, 16));
                l_sv <= l_sv + to_signed(s_yt.dsv, 23);
                l_sv2 <= l_sv2 + to_signed(s_yt.dsv2, 23);
                l_gy <= l_gy + to_signed(s_yt.dgy, 23);
            end if;
            -- the row terms of the hashes, one and two clocks after the line advance
            l_f1 <= g_fall; l_f2 <= l_f1;
            if l_f1 = '1' then
                if l_vy(18 downto 12) /= l_vyp(18 downto 12) then l_r1 <= l_r1 + K1V; end if;   -- (vy >> 4) * K1V
                if l_vy(18 downto 13) /= l_vyp(18 downto 13) then l_r2 <= l_r2 + K2V; end if;   -- (vy >> 5) * K2V
                v_vi := l_vy(18 downto 8);
                v_sk := resize(shift_right(resize(v_vi, 13) + shift_left(resize(v_vi, 13), 1), 4), 10);   -- 3 vy / 16
                l_qhn <= to_signed(s_pt.p0, 13) - signed(resize(v_sk & '0', 13));
            end if;
            if l_f2 = '1' then
                l_qh0 <= l_qhn;
                if l_qhn(12 downto 3) /= l_qh0(12 downto 3) then l_a1s <= l_a1s - K1H; end if;
                if l_qhn(12 downto 4) /= l_qh0(12 downto 4) then l_a2s <= l_a2s - K2H; end if;
            end if;

            -- row values of the current line (vi in virtual lines), pipelined over four registers
            -- (they change once per line, during the horizontal blank)
            r_vi <= l_vy(18 downto 8);
            -- B: flags and differences
            if f_in(r_vi, 405, 594) then r_pan <= '1'; else r_pan <= '0'; end if;
            if f_in(r_vi, 403, 597) then r_gri <= '1'; else r_gri <= '0'; end if;
            if not f_ltu(r_vi, 470) then r_glo <= '1'; else r_glo <= '0'; end if;
            if f_in(r_vi, 596, 598) or f_in(r_vi, 609, 611) then r_cab <= '1'; else r_cab <= '0'; end if;
            if f_in(r_vi, 661, 692) then r_band <= '1'; else r_band <= '0'; end if;
            if f_in(r_vi, 271, 338) then r_cwin <= '1'; else r_cwin <= '0'; end if;
            if l_gy(22 downto 16) = 0 then r_gly <= '1'; else r_gly <= '0'; end if;
            r_gy <= unsigned(l_gy(15 downto 12));
            r_e1 <= 442 - signed(resize(r_vi, 12)); r_e2 <= signed(resize(r_vi, 12)) - 554;
            r_ddA <= signed(resize(r_vi, 12)) - 433; r_ddB <= signed(resize(r_vi, 12)) - 558;
            if f_ltu(r_vi, 495) then r_dsel <= '1'; else r_dsel <= '0'; end if;
            r_dc <= signed(resize(r_vi, 12)) - 304;
            o_d470 <= signed(resize(r_vi, 12)) - 470; o_d540 <= signed(resize(r_vi, 12)) - 540;
            o_d478 <= signed(resize(r_vi, 12)) - 478; o_d644 <= signed(resize(r_vi, 12)) - 644;
            o_d860 <= signed(resize(r_vi, 12)) - 860;
            if ((r_vi + (s_t(7 downto 0) & "00") + (s_t(7 downto 0) & '0')) and to_unsigned(255, 11)) < to_unsigned(40, 11) then o_hum <= '1'; else o_hum <= '0'; end if;
            -- C: screen corner ey = max(442 - vi, vi - 554, 0); dials at 433 / 558; clock at 304; Osc rows
            if r_e1(11) = '0' and r_e1 /= 0 then r_eyv <= unsigned(r_e1(10 downto 0));
            elsif r_e2(11) = '0' then r_eyv <= unsigned(r_e2(10 downto 0));
            else r_eyv <= (others => '0'); end if;
            if r_dsel = '1' then v_d := r_ddA; else v_d := r_ddB; end if;
            r_dyd <= resize(shift_left(v_d, 1), 7);
            if f_ins(v_d, -17, 18) then r_dial <= '1'; else r_dial <= '0'; end if;
            if v_d < 0 then v_ad := unsigned(resize(-v_d, 11)); else v_ad := unsigned(resize(v_d, 11)); end if;
            r_ddv <= v_ad(5 downto 0);
            if f_ins(r_dc, -60, 61) then r_clk <= '1'; else r_clk <= '0'; end if;
            if r_dc < 0 then v_ad := unsigned(resize(-r_dc, 11)); else v_ad := unsigned(resize(r_dc, 11)); end if;
            r_cdv <= v_ad(5 downto 0);
            r_cdy <= resize(r_dc, 9);
            r_cdy866 <= resize(r_dc - shift_right(r_dc, 3) - shift_right(r_dc, 7), 9);
            if o_d470 < 0 then o_a470 <= unsigned(resize(-o_d470, 11)); else o_a470 <= unsigned(resize(o_d470, 11)); end if;
            if o_d540 < 0 then o_a540 <= unsigned(resize(-o_d540, 11)); else o_a540 <= unsigned(resize(o_d540, 11)); end if;
            if f_ins(o_d478, -2, 3) or (s_blink = '0' and f_ins(o_d478, -14, 15)) then o_eyer <= '1'; else o_eyer <= '0'; end if;
            if f_ins(o_d644, -11, 12) then o_mthr <= '1'; else o_mthr <= '0'; end if;
            if o_d860 < 0 then o_chw <= (others => '0');
            else o_chw <= to_unsigned(430, 10) - resize(shift_right(unsigned(o_d860(10 downto 0)), 2), 10); end if;
            -- D: hair / face half-widths (octagons)
            if not f_ltu(o_a470, 335) then o_hhw <= (others => '0');
            elsif f_ltu(o_a470, 138) then o_hhw <= to_unsigned(335, 10);
            else o_hhw <= resize(473 - o_a470, 10); end if;
            if not f_ltu(o_a540, 245) then o_f0 <= '1'; else o_f0 <= '0'; end if;
            if f_ltu(o_a540, 101) then o_f1 <= '1'; else o_f1 <= '0'; end if;
            o_fx <= resize(346 - o_a540, 10);
            if o_f0 = '1' then o_fhw <= (others => '0');
            elsif o_f1 = '1' then o_fhw <= to_unsigned(185, 10);
            else o_fhw <= o_fx - shift_right(o_fx, 2); end if;
            -- one square table, sequenced: ey^2, dial dy^2, clock dy^2
            r_sqc <= r_sqc + 1;
            case to_integer(r_sqc) is
                when 0 => v_sqi := r_eyv(5 downto 0);
                when 1 => v_sqi := r_ddv;
                when others => v_sqi := r_cdv;
            end case;
            v_sqo := C_SQ6(to_integer(v_sqi));
            case to_integer(r_sqc) is
                when 0 =>
                    if r_eyv(10 downto 5) = 0 then r_ey2 <= v_sqo(9 downto 0); else r_ey2 <= (others => '1'); end if;
                when 1 => r_dd2 <= v_sqo(9 downto 0);
                when others => r_cd2 <= v_sqo;
            end case;
            if r_sqc = 2 then r_sqc <= (others => '0'); end if;
            -- store window: the clock centre rows read the clock window, else the screen
            if r_cwin = '1' then
                r_sv <= unsigned(l_sv2(17 downto 4));
                if l_sv2(22 downto 18) = 0 then r_svok <= '1'; else r_svok <= '0'; end if;
            else
                r_sv <= unsigned(l_sv(17 downto 4)) + s_roll(13 downto 0);                -- rolled (K4), wraps 64 rows
                if l_sv(22 downto 18) = 0 then r_svok <= '1'; else r_svok <= '0'; end if;
            end if;
            if s_zc = 12 then r_g3 <= (others => '0');
            elsif g_fall = '1' then
                if r_g3 = 2 then r_g3 <= (others => '0'); else r_g3 <= r_g3 + 1; end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Main beam: wave phases, hash columns (stage 0); map column (stage 5)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_qh : signed(12 downto 0);
        variable v_c5 : unsigned(4 downto 0);
        variable v_n4 : unsigned(3 downto 0);
        variable v_n5 : unsigned(4 downto 0);
        variable v_c6 : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            if g_ssr(2) = '1' then
                for k in 0 to 3 loop b_f(k) <= l_ly(k)(15 downto 4); end loop;
                v_qh := l_qh0;
                b_a1 <= l_a1s + l_r1; b_a2 <= l_a2s + l_r2; b_a3 <= l_a3s;
                b_gcnt <= (others => '0');
            else
                v_qh := b_qh;
                if g_asr(2) = '1' then
                    b_f(0) <= b_f(0) + unsigned(to_signed(s_xt.ax0, 12));
                    b_f(1) <= b_f(1) + unsigned(to_signed(s_xt.ax1, 12));
                    b_f(2) <= b_f(2) + unsigned(to_signed(s_xt.ax2, 12));
                    b_f(3) <= b_f(3) + unsigned(to_signed(s_xt.ax3, 12));
                    v_qh := b_qh + to_signed(s_xt.xs, 13);
                    if v_qh(12 downto 3) /= b_qh(12 downto 3) then
                        b_a1 <= b_a1 + K1H;
                    end if;
                    if v_qh(12 downto 4) /= b_qh(12 downto 4) then
                        b_a2 <= b_a2 + K2H;
                    end if;
                    b_a3 <= b_a3 + K3X;
                    if b_gcnt = 2 then b_gcnt <= (others => '0'); else b_gcnt <= b_gcnt + 1; end if;
                end if;
            end if;
            b_qh <= v_qh;
            -- streak-row carries of the column: r1 = (vy + (-c1 mod 16)) >> 4, r2 = (vy + (15 c2 mod 32)) >> 5
            v_n4 := (not unsigned(v_qh(6 downto 3))) + 1;
            v_c5 := ('0' & l_vy(11 downto 8)) + ('0' & v_n4);
            b_cy1 <= v_c5(4);
            v_n5 := (unsigned(v_qh(4 downto 4)) & "0000") - unsigned(v_qh(8 downto 4));
            v_c6 := ('0' & l_vy(12 downto 8)) + ('0' & v_n5);
            b_cy2 <= v_c6(5);
            -- map column (stage 5)
            if g_ssr(7) = '1' then
                b_u <= to_signed(s_pt.u0, 22);
            elsif g_asr(7) = '1' then
                b_u <= b_u + to_signed(s_xt.du, 22);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (paint)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a : unsigned(15 downto 0);
        variable v_f : unsigned(11 downto 0);
        variable v_s : signed(11 downto 0);
        variable v_j : signed(14 downto 0);
        variable v_t : signed(11 downto 0);
        variable v_row : unsigned(6 downto 0);
        variable v_rj : unsigned(7 downto 0);
        variable v_ad : unsigned(13 downto 0);
        variable v_y : unsigned(9 downto 0);
        variable v_k : signed(10 downto 0);
        variable v_tm : signed(15 downto 0);
        variable v_tx : unsigned(1 downto 0);
        variable v_cam : unsigned(1 downto 0);
        variable v_idx : integer range 0 to 18;
        variable v_c : t_c3;
        variable v_o : signed(11 downto 0);
        variable v_d : unsigned(5 downto 0);
        variable v_dp : unsigned(8 downto 0);
        variable v_dy : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: triangles (12-bit phases), derived phases f1 + 2 f2, f3 + 2 f4; hash sums
            for k in 0 to 3 loop p1_t(k) <= f_tri12(b_f(k)); end loop;
            v_f := b_f(0) + (b_f(1)(10 downto 0) & '0'); p1_t5 <= f_tri12(v_f);
            v_f := b_f(2) + (b_f(3)(10 downto 0) & '0'); p1_t6 <= f_tri12(v_f);
            if b_cy1 = '1' then p1_w1 <= b_a1 + K1V; else p1_w1 <= b_a1; end if;
            if b_cy2 = '1' then p1_w2 <= b_a2 + K2V; else p1_w2 <= b_a2; end if;
            p1_w3 <= b_a3;

            -- 2: warp sums (SX, SY in -336..336); hash round 1
            p2_sx <= signed(resize(p1_t(0), 11)) + signed(resize(p1_t(1), 11)) + signed(resize(p1_t5(7 downto 1), 11))
                     + signed(resize(p1_t5(7 downto 3), 11)) - 336;
            p2_sy <= signed(resize(p1_t(2), 11)) + signed(resize(p1_t(3), 11)) + signed(resize(p1_t6(7 downto 1), 11))
                     + signed(resize(p1_t6(7 downto 3), 11)) - 336;
            v_a := p1_w1 xor shift_right(p1_w1, 7); p2_h1 <= v_a + shift_left(v_a, 3);
            v_a := p1_w2 xor shift_right(p1_w2, 7); p2_h2 <= v_a + shift_left(v_a, 3);
            v_a := p1_w3 xor shift_right(p1_w3, 7); p2_h3 <= v_a + shift_left(v_a, 3);

            -- 3: warp products (SX * GS3 in two parts); hash round 2
            v_j := (others => '0');
            if s_gs3(0) = '1' then v_j := v_j + resize(p2_sx, 15); end if;
            if s_gs3(1) = '1' then v_j := v_j + shift_left(resize(p2_sx, 15), 1); end if;
            p3_jxa <= v_j;
            if s_gs3(2) = '1' then p3_jxb <= shift_left(resize(p2_sx, 15), 2); else p3_jxb <= (others => '0'); end if;
            v_j := (others => '0');
            if s_gs3(0) = '1' then v_j := v_j + resize(p2_sy, 15); end if;
            if s_gs3(1) = '1' then v_j := v_j + shift_left(resize(p2_sy, 15), 1); end if;
            p3_jya <= v_j;
            if s_gs3(2) = '1' then p3_jyb <= shift_left(resize(p2_sy, 15), 2); else p3_jyb <= (others => '0'); end if;
            p3_sxy <= resize(p2_sx, 12) + resize(p2_sy, 12);
            v_a := p2_h1 xor shift_right(p2_h1, 5); p3_h1 <= v_a + shift_left(v_a, 4);
            v_a := p2_h2 xor shift_right(p2_h2, 5); p3_h2 <= v_a + shift_left(v_a, 4);
            v_a := p2_h3 xor shift_right(p2_h3, 5); p3_h3 <= v_a + shift_left(v_a, 4);

            -- 4: warp sums; hash round 3 (streaks)
            p4_jxs <= p3_jxa + p3_jxb;
            p4_jys <= p3_jya + p3_jyb;
            p4_sxy <= p3_sxy;
            p4_h1 <= p3_h1 xor shift_right(p3_h1, 8);
            p4_h2 <= p3_h2 xor shift_right(p3_h2, 8);
            p4_h3 <= p3_h3;

            -- 5: jx, jy (Q.8 cells) = (SX*GS3 + streak*GH) / 4; texture terms; wash noise; drips
            v_s := signed(resize(p4_h2(15 downto 8), 12)) - 128;
            case to_integer(s_gh) is
                when 0 => v_j := (others => '0');
                when 1 => v_j := resize(v_s, 15);
                when 2 => v_j := resize(shift_left(v_s, 1), 15);
                when others => v_j := resize(shift_left(v_s, 2), 15);
            end case;
            v_j := p4_jxs + v_j;
            p5_jx <= resize(shift_right(v_j, 2), 13);
            v_s := signed(resize(p4_h1(15 downto 8), 12)) - 128;
            case to_integer(s_gh) is
                when 0 => v_j := (others => '0');
                when 1 => v_j := resize(v_s, 15);
                when 2 => v_j := resize(shift_left(v_s, 1), 15);
                when others => v_j := resize(shift_left(v_s, 2), 15);
            end case;
            v_j := p4_jys + v_j;
            p5_jy <= resize(shift_right(v_j, 2), 13);
            -- T = G + 7/8 S1 + 7/8 S2 + (SX + SY) / 4  (the grain is strong: the output [1 2 1] softens it)
            v_t := signed(resize(p4_h3(7 downto 0), 12)) - 128;
            v_s := signed(resize(p4_h1(7 downto 0), 12)) - 128;
            p5_ta <= resize(v_t + v_s - shift_right(v_s, 3), 11);
            v_s := signed(resize(p4_h2(7 downto 0), 12)) - 128;
            p5_tb <= resize(v_s - shift_right(v_s, 3) + shift_right(p4_sxy, 2), 11);
            p_n(5) <= resize(shift_right(p4_sxy, 4) + shift_right(v_s, 3), 7);
            if p4_h2(15 downto 8) < ("00" & s_dens & '0') then p_drip(5) <= '1'; else p_drip(5) <= '0'; end if;   -- drips: up to 24 %
            p5_sl <= p4_h2(9 downto 8);                                    -- side wash: dark streaks
            p5_sj <= p4_h2(13 downto 10);

            -- 6: warped coordinates (Q.8 cells); texture sum; side wash level
            p6_colf <= resize(shift_right(b_u, 4), 17) + resize(p5_jx, 17);
            p6_rowf <= signed(resize(l_v(18 downto 4), 16)) + resize(p5_jy, 16);
            p6_t <= resize(p5_ta, 12) + resize(p5_tb, 12);
            p6_sl <= p5_sl; p6_sj <= p5_sj;

            -- 7: cells, bank; texture gain partials (T * GT4)
            if p6_colf(16) = '1' or p6_colf(15) = '1' or p6_colf(14 downto 13) = "11" then p7_side <= '1'; else p7_side <= '0'; end if;
            p7_col <= unsigned(p6_colf(14 downto 8));
            if p6_rowf(15) = '1' then v_row := (others => '0');
            elsif not f_ltu(unsigned(p6_rowf(14 downto 8)), 119) then v_row := to_unsigned(119, 7);
            else v_row := unsigned(p6_rowf(14 downto 8)); end if;
            p7_row <= v_row;
            -- bank row; on the sides the boundary is jittered by the stroke hash (a ragged seam)
            if p6_colf(16) = '1' or p6_colf(15) = '1' or p6_colf(14 downto 13) = "11" then
                v_rj := resize(v_row, 8) + resize(p6_sj, 8);
            else
                v_rj := resize(v_row, 8) + 8;
            end if;
            if not f_ltu(v_rj, 98) then p7_br <= "11"; elsif not f_ltu(v_rj, 68) then p7_br <= "10";
            elsif not f_ltu(v_rj, 38) then p7_br <= "01"; else p7_br <= "00"; end if;
            if p6_colf(16) = '1' then p7_bc <= "00"; elsif p6_colf(15) = '1' or p6_colf(14 downto 13) = "11" then p7_bc <= "10";
            else p7_bc <= unsigned(p6_colf(14 downto 13)); end if;
            p7_sl <= p6_sl;
            v_tm := (others => '0');
            if s_gt4(0) = '1' then v_tm := v_tm + resize(p6_t, 16); end if;
            if s_gt4(1) = '1' then v_tm := v_tm + shift_left(resize(p6_t, 16), 1); end if;
            p7_tma <= v_tm;
            v_tm := (others => '0');
            if s_gt4(2) = '1' then v_tm := v_tm + shift_left(resize(p6_t, 16), 2); end if;
            if s_gt4(3) = '1' then v_tm := v_tm + shift_left(resize(p6_t, 16), 3); end if;
            p7_tmb <= v_tm;

            -- 8: map address (EBR read 8 -> data 9), bank; texture factor tm6 = clamp(T * GT4 / 64, +-31)
            m_ra <= shift_left(resize(p7_row, 14), 6) + shift_left(resize(p7_row, 14), 5) + resize(p7_col, 14);
            p8_bank <= resize(p7_br & '0', 4) + resize(p7_br, 4) + resize(p7_bc, 4);
            p8_side <= p7_side; p8_sl <= p7_sl;
            v_tm := shift_right(p7_tma + p7_tmb, 6);
            if v_tm(15) = '0' and v_tm(14 downto 5) /= 0 then p_tm(8) <= to_signed(31, 6);
            elsif v_tm(15) = '1' and (v_tm(14 downto 5) /= "1111111111" or v_tm(4 downto 0) = 0) then p_tm(8) <= to_signed(-31, 6);
            else p_tm(8) <= v_tm(5 downto 0); end if;
            -- 9: map data (EBR); 10: palette address (sides: the darkest four of the bank);
            -- 11: palette data (EBR); 12: fabric copy
            p9_bank <= p8_bank; p9_side <= p8_side; p9_sl <= p8_sl;
            if p9_side = '1' then pal_ra <= p9_bank & "00" & p9_sl; else pal_ra <= p9_bank & unsigned(m_q); end if;
            pal_q2 <= pal_q;
            for i in 9 to 15 loop p_tm(i) <= p_tm(i - 1); end loop;
            for i in 6 to 12 loop p_n(i) <= p_n(i - 1); end loop;
            for i in 6 to 14 loop p_drip(i) <= p_drip(i - 1); end loop;

            -- 13: expand the palette word {Y6, U5, V5}: Y10 = 128 + 13 Y6; depth = 229 - 5.25 Y6; key partials
            v_y := resize(unsigned(pal_q2(15 downto 10)), 10);
            p13_y <= shift_left(v_y, 3) + shift_left(v_y, 2) + v_y + 128;
            v_d := unsigned(pal_q2(15 downto 10));
            v_dp := shift_left(resize(v_d, 9), 2) + resize(v_d, 9) + resize(v_d(5 downto 2), 9);
            if not f_ltu(v_dp, 229) then p13_dep <= (others => '0'); else p13_dep <= resize(229 - v_dp, 8); end if;
            p13_u <= resize(shift_left(resize(unsigned(pal_q2(9 downto 5)), 10), 4), 10) + 256;
            p13_v <= resize(shift_left(resize(unsigned(pal_q2(4 downto 0)), 10), 4), 10) + 256;
            if not f_ltu(unsigned(pal_q2(9 downto 5)), 26) then p13_or <= '1'; else p13_or <= '0'; end if;
            p13_ns <= resize(p_n(12), 11) - resize(s_ths, 11);
            if resize(p_n(12), 9) + s_tho < 0 then p13_og <= '1'; else p13_og <= '0'; end if;

            -- 14: keys (scrape: depth + noise; flood: 255 - depth + noise); tint (not on the orange)
            v_k := signed(resize(p13_dep, 11)) + p13_ns;
            if s_scr = '1' and v_k < 0 then p14_rev <= '1'; else p14_rev <= '0'; end if;
            if s_scr = '1' and v_k(10 downto 4) = 0 then p14_edge <= '1'; else p14_edge <= '0'; end if;
            v_k := to_signed(255, 11) - signed(resize(p13_dep, 11)) + p13_ns;
            if s_fld = '1' and v_k < 0 then p14_fld <= '1'; else p14_fld <= '0'; end if;
            p14_og <= p13_or and p13_og;
            p14_y <= p13_y;
            if p13_or = '1' then
                p14_u <= p13_u; p14_v <= p13_v;
            else
                p14_u <= unsigned(signed(p13_u) + resize(s_tu, 10)); p14_v <= unsigned(signed(p13_v) + resize(s_tv, 10));
            end if;

            -- 15: colour index (paint, elements, TV, caption), texture amount, camera select
            v_idx := 0; v_cam := "00";
            if p_drip(14) = '1' then v_idx := 1; end if;
            if p14_og = '1' then v_idx := 4; end if;
            if x17_fl.mark = '1' then v_idx := 3; end if;
            if x17_fl.cin = '1' and s_clkp = '0' then v_cam := "11"; end if;
            if p14_fld = '1' then v_idx := 4; v_cam := "00"; end if;
            if p14_edge = '1' then v_idx := 5; v_cam := "00"; end if;
            if p14_rev = '1' then v_cam := "10"; end if;
            if x17_fl.cab = '1' then v_idx := 6; v_cam := "00"; end if;
            if x17_fl.gri = '1' then
                v_cam := "00";
                if x17_fl.glo = '1' then
                    if x17_fl.gline = '1' then v_idx := 10; else v_idx := 9; end if;
                else
                    if x17_fl.gline = '1' then v_idx := 8; else v_idx := 7; end if;
                end if;
            end if;
            if x17_fl.pan = '1' then
                v_cam := "00"; v_idx := 11;
                if x17_fl.d1 = '1' then v_idx := 12; end if;
                if x17_fl.d2 = '1' then v_idx := 13; end if;
                if x17_fl.d3 = '1' then v_idx := 14; end if;
                if x17_fl.bar = '1' then v_idx := 15; end if;
            end if;
            if x17_fl.rim = '1' then v_idx := 16; v_cam := "00"; end if;
            if x17_fl.scr = '1' then v_cam := "01"; end if;
            if x17_fl.band = '1' then
                v_cam := "00";
                if x17_fl.gly = '1' then v_idx := 18; else v_idx := 17; end if;
            end if;
            case v_idx is
                when 0 =>
                    p15_y <= p14_y; p15_u <= p14_u; p15_v <= p14_v;
                when 4 =>
                    p15_y <= C_WASH(0); p15_u <= s_wu; p15_v <= s_wv;
                when others =>
                    v_c := f_col(v_idx);
                    p15_y <= v_c(0); p15_u <= v_c(1); p15_v <= v_c(2);
            end case;
            case v_idx is
                when 6 to 11 | 13 | 14 | 17 | 18 => v_tx := "01";
                when 12 | 15 | 16 => v_tx := "00";
                when others => v_tx := "10";
            end case;
            p15_tx <= v_tx; p15_cam <= v_cam;

            -- 16: texture product (Y - 64) * tm (halved for the TV parts)
            v_o := signed(resize(p15_y, 12)) - 64;
            v_tm := resize(v_o * p_tm(15), 16);
            if p15_tx = "01" then v_tm := shift_right(v_tm, 1); elsif p15_tx = "00" then v_tm := (others => '0'); end if;
            p16_py <= v_tm;
            p16_y <= p15_y; p16_u <= p15_u; p16_v <= p15_v; p16_cam <= p15_cam;

            -- 17: textured colour (clamped); chroma follows the luma texture (Cr + dY/8, Cb - 3 dY/16)
            v_dy := shift_right(p16_py, 6);
            v_tm := signed(resize(p16_y, 16)) + v_dy;
            if v_tm(15) = '1' or v_tm(14 downto 6) = 0 then p17_y <= to_unsigned(64, 10); elsif v_tm(14 downto 10) /= 0 then p17_y <= to_unsigned(1023, 10);
            else p17_y <= unsigned(v_tm(9 downto 0)); end if;
            v_tm := signed(resize(p16_u, 16)) + shift_right(v_dy, 3);
            if v_tm(15) = '1' then p17_u <= to_unsigned(0, 10); elsif v_tm(14 downto 10) /= 0 then p17_u <= to_unsigned(1023, 10);
            else p17_u <= unsigned(v_tm(9 downto 0)); end if;
            v_tm := signed(resize(p16_v, 16)) - shift_right(v_dy, 3) - shift_right(v_dy, 4);
            if v_tm(15) = '1' then p17_v <= to_unsigned(0, 10); elsif v_tm(14 downto 10) /= 0 then p17_v <= to_unsigned(1023, 10);
            else p17_v <= unsigned(v_tm(9 downto 0)); end if;
            p17_cam <= p16_cam;

            -- 18: the camera: TV screen (store), scraped-through reveal (live tap), clock centre (store, teal)
            case to_integer(p17_cam) is
                when 1 =>
                    if s_lines = '1' and r_lpar = '1' then
                        p18_y <= s20_y - shift_right(s20_y, 2) + 16;                      -- S10 Lines: scanlines
                    else
                        p18_y <= s20_y;
                    end if;
                    p18_u <= s20_u; p18_v <= s20_v;
                when 2 => p18_y <= t1_y; p18_u <= t1_u; p18_v <= t1_v;
                when 3 =>
                    p18_y <= resize(shift_right(s20_y, 1) + shift_right(s20_y, 3) + 140, 10);
                    p18_u <= to_unsigned(418, 10); p18_v <= to_unsigned(512, 10);
                when others => p18_y <= p17_y; p18_u <= p17_u; p18_v <= p17_v;
            end case;

            -- 22..27: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums, the 8-back
            -- sample from an EBR ring; luma [1 2 1]); 28: luma toward black (white 940 -> 777), blanking gate
            s_boxu <= s_boxu + p18_u(9 downto 2) - unsigned(s_cq8(15 downto 8));
            s_boxv <= s_boxv + p18_v(9 downto 2) - unsigned(s_cq8(7 downto 0));
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            s_uf <= resize(shift_right(resize(s_boxu, 12) + s_boxud, 2), 10);
            s_vf <= resize(shift_right(resize(s_boxv, 12) + s_boxvd, 2), 10);
            s_y19 <= p18_y; s_y20 <= s_y19; s_y21 <= s_y20; s_y22 <= s_y21;
            s_y23 <= s_y22; s_y24x <= s_y23;
            s_yf <= resize(shift_right(resize(s_y22, 12) + shift_left(resize(s_y23, 12), 1) + resize(s_y24x, 12), 2), 10);
            if s_sq(0) = '1' then
                if s_yf(9 downto 6) /= 0 then
                    s_yo <= s_yf - shift_right(s_yf - 64, 3) - shift_right(s_yf - 64, 4);
                else
                    s_yo <= to_unsigned(64, 10);
                end if;
                s_uo <= s_uf; s_vo <= s_vf;
            else
                s_yo <= to_unsigned(64, 10); s_uo <= to_unsigned(512, 10); s_vo <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_yo);
    data_out.u       <= std_logic_vector(s_uo);
    data_out.v       <= std_logic_vector(s_vo);
    data_out.hsync_n <= s_do(3);
    data_out.vsync_n <= s_do(2);
    data_out.field_n <= s_do(1);
    data_out.avid    <= s_do(0);

    ---------------------------------------------------------------------------
    -- Element beam (stage 10) and the store reader.  Shapes from small square tables:
    -- screen corners r 19 (rim 21), dials r 17 / 15.5 / 9.5 and a bar, clock ring 39..60 with
    -- twelve numeral marks and a centre r 33.
    ---------------------------------------------------------------------------
    p_elem : process(clk)
        variable v_e : signed(12 downto 0);
        variable v_x : unsigned(11 downto 0);
        variable v_a : signed(12 downto 0);
        variable v_n : std_logic;
        variable v_ua, v_ub : unsigned(6 downto 0);
        variable v_va, v_vb : unsigned(6 downto 0);
        variable v_m : signed(9 downto 0);
        variable v_q0, v_q1, v_q2, v_q3 : unsigned(3 downto 0);
        variable v_cu, v_cv : signed(4 downto 0);
        variable v_ly : unsigned(9 downto 0);
        variable v_sn : signed(11 downto 0);
        variable v_r2 : unsigned(12 downto 0);
        variable v_cx, v_c8 : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            -- 10: element beam
            if g_ssr(12) = '1' then
                e2_pxh <= to_signed(s_pt.p0, 13);
                if r_cwin = '1' then e2_su <= to_signed(s_pt.su0c, 20); e2_dsu <= to_signed(s_xt.dsuc, 12);
                else e2_su <= to_signed(s_pt.su0, 20); e2_dsu <= to_signed(s_xt.dsu, 12); end if;
                e2_gx <= to_signed(s_pt.gx0, 20) + signed(resize(s_cr8, 20));
            elsif g_asr(12) = '1' then
                e2_pxh <= e2_pxh + to_signed(s_xt.xs, 13);
                e2_su <= e2_su + resize(e2_dsu, 20);
                e2_gx <= e2_gx + to_signed(s_xt.dgx, 20);
            end if;

            -- 11: distances (half units) and rectangles; glyph word read; store coordinates
            x14_ds <= f_abs13(e2_pxh - 823);
            x14_dd <= f_abs13(e2_pxh - 1118);
            x14_dc <= e2_pxh - 328;
            x14_ddx <= e2_pxh - 1118;
            if f_ins(e2_pxh, 1076, 1164) and r_pan = '1' then x14_pan <= '1'; else x14_pan <= '0'; end if;
            if f_ins(e2_pxh, 1164, 1276) and r_gri = '1' then x14_gri <= '1'; else x14_gri <= '0'; end if;
            if not f_lts(e2_pxh, 1188) and r_glo = '1' then x14_glo <= '1'; else x14_glo <= '0'; end if;
            if f_ins(e2_pxh, 574, 1280) and r_cab = '1' then x14_cab <= '1'; else x14_cab <= '0'; end if;
            if f_ins(e2_pxh, 638, 1188) and r_band = '1' then x14_band <= '1'; else x14_band <= '0'; end if;
            if f_ins(e2_pxh, 638, 1194) and r_gly = '1' then x14_gok <= '1'; else x14_gok <= '0'; end if;
            if b_gcnt = 0 or r_g3 = 0 then x14_gline <= '1'; else x14_gline <= '0'; end if;
            gl_ra <= r_gy & unsigned(e2_gx(15 downto 12));
            x14_gbit <= unsigned(e2_gx(11 downto 8));
            if e2_su(19 downto 15) = 0 and r_svok = '1' then s14_ok <= '1'; else s14_ok <= '0'; end if;
            s14_u0 <= unsigned(e2_su(14 downto 8)); s14_fu <= unsigned(e2_su(7 downto 4));
            s14_v0 <= resize(r_sv(13 downto 8), 7);
            s14_fv <= r_sv(7 downto 4);

            -- 12: squares (px): screen corner ex, dial dx, clock dx; clock numeral line terms
            v_e := signed(resize(x14_ds, 13)) - 175;
            x15_ein <= v_e(12);
            if v_e(12) = '0' and v_e(11 downto 6) /= 0 then x15_eout <= '1'; else x15_eout <= '0'; end if;
            x15_ex2 <= C_SQ5(to_integer(unsigned(v_e(5 downto 1))));
            if x14_dd(11 downto 6) = 0 then x15_dok <= '1'; else x15_dok <= '0'; end if;
            x15_dx2 <= C_SQ5(to_integer(x14_dd(5 downto 1)));
            v_x := f_abs13(x14_dc);
            if v_x(11 downto 7) = 0 then x15_cok <= '1'; else x15_cok <= '0'; end if;
            x15_cx2 <= C_SQ6(to_integer(v_x(6 downto 1)));
            v_e := x14_ddx + resize(r_dyd, 13);
            if f_ins(v_e, -4, 5) then x15_bar <= '1'; else x15_bar <= '0'; end if;
            -- clock numeral lines (px): |dy|, |dx|, |dx/2 - .866dy|, |.866dx - dy/2|, |.866dx + dy/2|, |dx/2 + .866dy| < 4
            v_cx := resize(shift_right(x14_dc, 1), 9);
            x15_cx <= v_cx;
            x15_c8 <= v_cx - shift_right(v_cx, 3) - shift_right(v_cx, 7);
            x15_pan <= x14_pan; x15_gri <= x14_gri; x15_glo <= x14_glo; x15_cab <= x14_cab;
            x15_band <= x14_band; x15_gok <= x14_gok; x15_gbit <= x14_gbit; x15_gline <= x14_gline;
            gl_q2 <= gl_q;

            -- 13: radius compares
            v_r2 := resize(x15_ex2, 13) + resize(r_ey2, 13);
            if x15_eout = '0' and f_ltu(r_ey2, 361) and (x15_ein = '1' or f_ltu(v_r2, 361)) then
                x16_fl.scr <= '1'; x16_fl.rim <= '0';
            elsif x15_eout = '0' and f_ltu(r_ey2, 441) and (x15_ein = '1' or f_ltu(v_r2, 441)) then
                x16_fl.scr <= '0'; x16_fl.rim <= '1';
            else
                x16_fl.scr <= '0'; x16_fl.rim <= '0';
            end if;
            v_r2 := resize(x15_dx2, 13) + resize(r_dd2, 13);
            if r_dial = '1' and x15_dok = '1' and f_ltu(v_r2, 289) then x16_fl.d1 <= '1'; else x16_fl.d1 <= '0'; end if;
            if r_dial = '1' and x15_dok = '1' and f_ltu(v_r2, 240) then x16_fl.d2 <= '1'; else x16_fl.d2 <= '0'; end if;
            if r_dial = '1' and x15_dok = '1' and f_ltu(v_r2, 90) then x16_fl.d3 <= '1'; else x16_fl.d3 <= '0'; end if;
            if r_dial = '1' and x15_dok = '1' and f_ltu(v_r2, 240) and x15_bar = '1' then x16_fl.bar <= '1'; else x16_fl.bar <= '0'; end if;
            v_r2 := resize(x15_cx2, 13) + resize(r_cd2, 13);
            if r_clk = '1' and x15_cok = '1' and f_in(v_r2, 1521, 3600) then x16_fl.ring <= '1'; else x16_fl.ring <= '0'; end if;
            if r_clk = '1' and x15_cok = '1' and f_ltu(v_r2, 1089) then x16_fl.cin <= '1'; else x16_fl.cin <= '0'; end if;
            v_n := '0';
            if f_ins(r_cdy, -3, 4) or f_ins(x15_cx, -3, 4) then v_n := '1'; end if;
            if f_ins(shift_right(x15_cx, 1) - r_cdy866, -3, 4) or f_ins(x15_c8 - shift_right(r_cdy, 1), -3, 4) then v_n := '1'; end if;
            if f_ins(x15_c8 + shift_right(r_cdy, 1), -3, 4) or f_ins(shift_right(x15_cx, 1) + r_cdy866, -3, 4) then v_n := '1'; end if;
            x16_fl.mark <= v_n;
            x16_fl.pan <= x15_pan; x16_fl.gri <= x15_gri; x16_fl.glo <= x15_glo; x16_fl.gline <= x15_gline;
            x16_fl.cab <= x15_cab; x16_fl.band <= x15_band;
            x16_gok <= x15_gok; x16_gbit <= x15_gbit;
            -- 14
            x17_fl <= x16_fl;
            x17_fl.mark <= x16_fl.mark and x16_fl.ring;
            x17_fl.gly <= x16_gok and gl_q2(to_integer(x16_gbit));

            -- store reader: 12 address; 13 data; 14 fabric; 15 horizontal lerps; 16 vertical lerp; 17 colour
            -- bank (pv, pu) holds rows with v % 2 = pv, columns u % 2 = pu
            v_ua := s14_u0; v_ub := s14_u0 + 1;
            v_va := s14_v0; v_vb := s14_v0 + 1;
            if v_ub = 0 then v_ub := s14_u0; end if;
            if v_vb(6) = '1' then v_vb := s14_v0; end if;
            if s14_u0(0) = '0' then
                b_ra0(5 downto 0) <= v_ua(6 downto 1); b_ra1(5 downto 0) <= v_ub(6 downto 1);
                b_ra2(5 downto 0) <= v_ua(6 downto 1); b_ra3(5 downto 0) <= v_ub(6 downto 1);
            else
                b_ra0(5 downto 0) <= v_ub(6 downto 1); b_ra1(5 downto 0) <= v_ua(6 downto 1);
                b_ra2(5 downto 0) <= v_ub(6 downto 1); b_ra3(5 downto 0) <= v_ua(6 downto 1);
            end if;
            if s14_v0(0) = '0' then
                b_ra0(10 downto 6) <= v_va(5 downto 1); b_ra1(10 downto 6) <= v_va(5 downto 1);
                b_ra2(10 downto 6) <= v_vb(5 downto 1); b_ra3(10 downto 6) <= v_vb(5 downto 1);
            else
                b_ra0(10 downto 6) <= v_vb(5 downto 1); b_ra1(10 downto 6) <= v_vb(5 downto 1);
                b_ra2(10 downto 6) <= v_va(5 downto 1); b_ra3(10 downto 6) <= v_va(5 downto 1);
            end if;
            c_ra <= s14_v0(5 downto 2) & s14_u0(6 downto 2);
            s_fu(15) <= s14_fu; s_fv(15) <= s14_fv; s_upar(15) <= s14_u0(0); s_vpar(15) <= s14_v0(0);
            for i in 16 to 18 loop s_fu(i) <= s_fu(i - 1); s_fv(i) <= s_fv(i - 1); end loop;
            for i in 16 to 17 loop s_upar(i) <= s_upar(i - 1); s_vpar(i) <= s_vpar(i - 1); end loop;
            s_ok(15) <= s14_ok;
            for i in 16 to 19 loop s_ok(i) <= s_ok(i - 1); end loop;
            s_cwin18 <= r_cwin; s_cwin19 <= s_cwin18;
            -- fabric copies, re-ordered to (u0,v0) (u1,v0) (u0,v1) (u1,v1)
            v_q0 := unsigned(b_q0); v_q1 := unsigned(b_q1); v_q2 := unsigned(b_q2); v_q3 := unsigned(b_q3);
            if s_upar(16) = '1' then v_q0 := unsigned(b_q1); v_q1 := unsigned(b_q0); v_q2 := unsigned(b_q3); v_q3 := unsigned(b_q2); end if;
            if s_vpar(16) = '1' then
                s17_q0 <= v_q2; s17_q1 <= v_q3; s17_q2 <= v_q0; s17_q3 <= v_q1;
            else
                s17_q0 <= v_q0; s17_q1 <= v_q1; s17_q2 <= v_q2; s17_q3 <= v_q3;
            end if;
            s17_cq <= c_q; s18_cq <= s17_cq; s19_cq <= s18_cq;
            -- horizontal lerps, Q.4: a + (b - a) * fu / 16
            v_m := signed(resize(s17_q1, 10)) - signed(resize(s17_q0, 10));
            v_m := resize(v_m * signed(resize(s_fu(17), 6)), 10);
            v_m := signed(resize(s17_q0 & "0000", 10)) + v_m;
            s18_a <= unsigned(v_m(8 downto 0));
            v_m := signed(resize(s17_q3, 10)) - signed(resize(s17_q2, 10));
            v_m := resize(v_m * signed(resize(s_fu(17), 6)), 10);
            v_m := signed(resize(s17_q2 & "0000", 10)) + v_m;
            s18_b <= unsigned(v_m(8 downto 0));
            -- vertical lerp
            v_m := resize((signed(resize(s18_b, 10)) - signed(resize(s18_a, 10))) * signed(resize(s_fv(18), 6)), 14)(13 downto 4);
            v_m := signed(resize(s18_a, 10)) + v_m;
            s19_l <= unsigned(v_m(8 downto 0));
            -- luma 64 + 4 L (Q.4 -> 10 bit), snow (K4), chroma (S8 Mono)
            if not f_ltu(s19_l, 219) then v_ly := to_unsigned(940, 10); else v_ly := resize(s19_l & "00", 10) + 64; end if;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            if s_cwin19 = '0' and s_lfsr(11 downto 8) < s_snow then
                v_ly := resize(s_lfsr(7 downto 0) & "0", 10) + 200;              -- snow: noise replaces the picture
            end if;
            if s_ok(19) = '0' then v_ly := to_unsigned(64, 10); end if;
            s20_y <= v_ly;
            v_cu := signed(s19_cq(7 downto 4)) & '0'; v_cv := signed(s19_cq(3 downto 0)) & '0';
            if s_mono = '1' or s_ok(19) = '0' then
                s20_u <= to_unsigned(512, 10); s20_v <= to_unsigned(512, 10);
            else
                v_sn := to_signed(512, 12) + shift_left(resize(v_cu, 12), 4);
                s20_u <= unsigned(v_sn(9 downto 0));
                v_sn := to_signed(512, 12) + shift_left(resize(v_cv, 12), 4);
                s20_v <= unsigned(v_sn(9 downto 0));
            end if;
        end if;
    end process p_elem;

    ---------------------------------------------------------------------------
    -- Source side: camera / Osc pixel, reveal tap, store writer
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_dx : signed(12 downto 0);
        variable v_ax : unsigned(11 downto 0);
        variable v_af : unsigned(11 downto 0);
        variable v_e : signed(12 downto 0);
        variable v_ea : unsigned(11 downto 0);
        variable v_c : t_c3;
        variable v_lo : unsigned(16 downto 0);
        variable v_ra : unsigned(23 downto 0);
        variable v_s : unsigned(15 downto 0);
        variable v_q : signed(11 downto 0);
        variable v_u4, v_v4 : signed(11 downto 0);
        variable v_dith : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- Osc portrait (source virtual coordinates): teal screen, hair, face, eyes, mouth, collar
            if data_in.avid = '1' and s_avq = '0' then
                o_sxh <= -signed(resize(s_ocx, 13));
            elsif data_in.avid = '1' then
                o_sxh <= o_sxh + to_signed(s_xt.xsa, 13);
            end if;
            v_ax := f_abs13(o_sxh);
            v_af := shift_right(v_ax, 1);                                   -- |dx| px
            if v_af < o_hhw then o_hair <= '1'; else o_hair <= '0'; end if;
            if v_af < o_fhw then o_face <= '1'; else o_face <= '0'; end if;
            if o_eyer = '1' and f_in(v_af, 44, 112) then o_eye <= '1'; else o_eye <= '0'; end if;
            if o_mthr = '1' and f_ltu(v_af, 70) then o_mouth <= '1'; else o_mouth <= '0'; end if;
            if v_af < o_chw then o_coll <= '1'; else o_coll <= '0'; end if;
            v_dx := o_sxh + signed(resize(r_vi & '0', 13));
            o_str <= v_dx(5);
            v_c := C_OBG;
            if o_hair = '1' then if o_str = '1' then v_c := C_OHAIR2; else v_c := C_OHAIR; end if; end if;
            if o_coll = '1' then v_c := C_OCOLL; end if;
            if o_face = '1' then v_c := C_OFACE; end if;
            if o_face = '1' and o_eye = '1' then v_c := C_OEYE; end if;
            if o_face = '1' and o_mouth = '1' then v_c := C_OMOUTH; end if;
            if o_hum = '1' then v_c(0) := v_c(0) + 36; end if;
            o_y <= v_c(0); o_u <= v_c(1); o_v <= v_c(2);
            -- source pixel: camera or Osc
            if s_osc = '1' then
                x_y <= o_y; x_u <= o_u; x_v <= o_v;
            else
                x_y <= unsigned(data_in.y); x_u <= unsigned(data_in.u); x_v <= unsigned(data_in.v);
            end if;
            x_av <= data_in.avid;
            -- reveal tap (held through the end of the line)
            if x_av = '1' then t1_y <= x_y; t1_u <= x_u; t1_v <= x_v; end if;

            -- store writer: columns (2^n pixels averaged), one line per store row
            w_emit <= '0'; b_we <= (others => '0'); c_we <= '0';
            if x_av = '1' then
                w_si <= w_si + 1;
                if w_si >= s_wc.x0 then
                    -- column DDA from the first captured pixel (16-bit fraction; its carry starts a column);
                    -- a new column restarts the window
                    v_lo := ('0' & w_cacc) + resize(s_wc.kc, 17);
                    if w_si = s_wc.x0 then
                        w_cacc <= (others => '0'); w_col <= (others => '0');
                    else
                        w_cacc <= v_lo(15 downto 0);
                        if v_lo(16) = '1' then w_col <= w_col + 1; end if;
                    end if;
                    if w_si = s_wc.x0 or v_lo(16) = '1' then
                        w_cnt <= to_unsigned(1, 4);
                        w_sy <= resize(x_y, 14);
                    elsif w_cnt < shift_left(to_unsigned(1, 4), to_integer(s_wc.n)) then
                        w_cnt <= w_cnt + 1;
                        w_sy <= w_sy + x_y;
                        -- the window's last pixel: emit the column sample
                        if w_cnt = shift_left(to_unsigned(1, 4), to_integer(s_wc.n)) - 1 and w_col(7) = '0' then
                            w_emit <= '1'; w_ecol <= w_col(6 downto 0);
                            w_ey <= w_sy + x_y; w_eu <= x_u; w_ev <= x_v;
                        end if;
                    end if;
                end if;
            else
                w_si <= (others => '0');
            end if;
            -- rows: line DDA at the source avid fall; the first line of each store row is written
            if s_zgo = '1' then
                w_skip <= s_wc.y0; w_racc <= (others => '0'); w_row <= (others => '0'); w_lcnt <= (others => '0');
                w_rok <= '0';
            elsif data_in.avid = '0' and s_avq = '1' then
                if w_skip /= 0 then w_skip <= w_skip - 1; end if;
                if w_skip = 1 then
                    w_racc <= (others => '0'); w_row <= (others => '0'); w_lcnt <= (others => '0'); w_rok <= '1';
                elsif w_skip = 0 then
                    v_ra := w_racc + resize(s_wc.kr, 24);
                    w_racc <= v_ra;
                    if v_ra(23 downto 16) /= w_racc(23 downto 16) then
                        w_lcnt <= (others => '0');
                    elsif w_lcnt /= 15 then
                        w_lcnt <= w_lcnt + 1;
                    end if;
                    w_row <= v_ra(22 downto 16);
                    if v_ra(23 downto 22) /= "00" then w_rok <= '0'; end if;
                end if;
            end if;
            if w_lcnt = 0 then w_first <= '1'; else w_first <= '0'; end if;
            if w_emit = '1' and w_rok = '1' and w_first = '1' then
                -- 4-bit level: (mean - 64 + dither) / 64
                case std_logic_vector'(w_row(0) & w_ecol(0)) is
                    when "00" => v_dith := to_unsigned(0, 6);
                    when "01" => v_dith := to_unsigned(32, 6);
                    when "10" => v_dith := to_unsigned(48, 6);
                    when others => v_dith := to_unsigned(16, 6);
                end case;
                v_q := signed(resize(shift_right(w_ey, to_integer(s_wc.n)), 12)) - 64 + signed(resize(v_dith, 12));
                v_q := shift_right(v_q, 6);
                if v_q(11) = '1' then v_q := (others => '0'); elsif v_q(10 downto 4) /= 0 then v_q := to_signed(15, 12); end if;
                b_wa <= w_row(5 downto 1) & w_ecol(6 downto 1);
                b_wd <= std_logic_vector(v_q(3 downto 0));
                case std_logic_vector'(w_row(0) & w_ecol(0)) is
                    when "00" => b_we <= "0001";
                    when "01" => b_we <= "0010";
                    when "10" => b_we <= "0100";
                    when others => b_we <= "1000";
                end case;
                if w_ecol(1 downto 0) = "00" and w_row(1 downto 0) = "00" then
                    -- chroma: one pixel per 4 x 4 cells
                    v_u4 := signed(resize(w_eu, 12)) - 512 + 16;
                    v_v4 := signed(resize(w_ev, 12)) - 512 + 16;
                    v_u4 := shift_right(v_u4, 5); v_v4 := shift_right(v_v4, 5);
                    if v_u4(11) = '0' and v_u4(10 downto 3) /= 0 then v_u4 := to_signed(7, 12); elsif v_u4(11) = '1' and v_u4(10 downto 3) /= "11111111" then v_u4 := to_signed(-8, 12); end if;
                    if v_v4(11) = '0' and v_v4(10 downto 3) /= 0 then v_v4 := to_signed(7, 12); elsif v_v4(11) = '1' and v_v4(10 downto 3) /= "11111111" then v_v4 := to_signed(-8, 12); end if;
                    c_we <= '1';
                    c_wa <= w_row(5 downto 2) & w_ecol(6 downto 2);
                    c_wd <= std_logic_vector(v_u4(3 downto 0)) & std_logic_vector(v_v4(3 downto 0));
                end if;
            end if;
        end if;
    end process p_src;

end architecture greenroom;
