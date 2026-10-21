-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Mandala v0.1.0 -- Heirloom #6 (2026-10-20), from WIN_20230128_01_06_08.
--
-- The photo: smooth psychedelic contour rings -- nested rounded lobes in a
-- continuous rainbow (magenta / pink / blue / green / red / yellow) growing
-- out of several centres and meeting in soft seams, mirrored left / right.
--
-- The program: three elliptical distance fields from three orbiting centres;
-- the field at each pixel is the NEAREST centre's r^2, so every centre owns
-- a lobe of nested rings and the lobes meet in Voronoi seams.  The rings are
-- a continuous palette cycling outward; the video luma etches into the field.
--
--   K1  Rings    ring density
--   K2  Hue      palette rotation
--   K3  Orbit    centre orbit speed, bipolar (centre = still)
--   K4  Aspect   vertical stretch of the rings (1/2 .. 4x)
--   K5  Etch     video depth into the field
--   K6  Weight   the third centre's lobe shrinks as its field is weighted up
--   S7  Mirror   centre 2 is the left/right mirror of centre 1 / orbits on its own
--   S8  Centres  Three / Two
--   S9  Palette  Rainbow / Magenta-blue
--   S10 Rings    Smooth / Hard (8 bands)
--   S11 Source   Video / Osc
--   P12 CENTRES  spread: all centres together (one onion) .. far apart (lobes)
--
-- Geometry in virtual 1080-line units.  dx^2 per centre by second differences
-- per pixel, dy^2 per centre per line and the per-frame products on one
-- serial multiplier.  Generated avid; output conditioning before the
-- blanking gate.  Latency 20 clocks.  EBR: 16 line buffer + 4 palette.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture mandala of program_top is
    constant C_LAT : integer := 20;

    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);
            when 4          => return to_unsigned(576, 12);
            when 8          => return to_unsigned(960, 12);
            when 12         => return to_unsigned(480, 12);
            when 5 | 6 | 14 => return to_unsigned(384, 12);
            when 1 | 2 | 10 => return to_unsigned(512, 12);
            when others     => return to_unsigned(256, 12);
        end case;
    end function;

    -- palettes (cyclic): A rainbow, B magenta / white / blue / pink
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_KA : t_keys := ((0, 328, 960, 360), (43, 840, 584, 64), (85, 580, 136, 216), (128, 678, 64, 663),
                               (171, 164, 440, 960), (213, 426, 887, 808), (255, 328, 960, 360));
    constant C_KB : t_keys := ((0, 426, 887, 808), (64, 940, 512, 512), (128, 164, 440, 960),
                               (192, 752, 612, 512), (255, 426, 887, 808));
    type t_pal is array (0 to 511) of std_logic_vector(31 downto 0);
    function f_lerp(a, b, t, n : integer) return integer is
    begin
        return a + ((b - a) * t) / n;
    end function;
    function f_pal return t_pal is
        variable r : t_pal;
        variable y, u, v : integer;
    begin
        for i in 0 to 255 loop
            for k in 0 to C_KA'length - 2 loop
                if i >= C_KA(k).i and i <= C_KA(k + 1).i then
                    y := f_lerp(C_KA(k).y, C_KA(k + 1).y, i - C_KA(k).i, C_KA(k + 1).i - C_KA(k).i);
                    u := f_lerp(C_KA(k).u, C_KA(k + 1).u, i - C_KA(k).i, C_KA(k + 1).i - C_KA(k).i);
                    v := f_lerp(C_KA(k).v, C_KA(k + 1).v, i - C_KA(k).i, C_KA(k + 1).i - C_KA(k).i);
                end if;
            end loop;
            r(i) := std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
                    & std_logic_vector(to_unsigned(v, 10)) & "00";
            for k in 0 to C_KB'length - 2 loop
                if i >= C_KB(k).i and i <= C_KB(k + 1).i then
                    y := f_lerp(C_KB(k).y, C_KB(k + 1).y, i - C_KB(k).i, C_KB(k + 1).i - C_KB(k).i);
                    u := f_lerp(C_KB(k).u, C_KB(k + 1).u, i - C_KB(k).i, C_KB(k + 1).i - C_KB(k).i);
                    v := f_lerp(C_KB(k).v, C_KB(k + 1).v, i - C_KB(k).i, C_KB(k + 1).i - C_KB(k).i);
                end if;
            end loop;
            r(256 + i) := std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
                          & std_logic_vector(to_unsigned(v, 10)) & "00";
        end loop;
        return r;
    end function;
    signal pal : t_pal := f_pal;
    signal s_pal_ra : unsigned(8 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- signed triangle: tri8 - 64, as magnitude + sign
    function f_stmag(v : unsigned(7 downto 0)) return unsigned is
        variable t : unsigned(6 downto 0);
    begin
        t := f_tri8(v);
        if t >= 64 then return resize(t - 64, 7); else return resize(to_unsigned(64, 7) - t, 7); end if;
    end function;
    function f_stneg(v : unsigned(7 downto 0)) return std_logic is
    begin
        if f_tri8(v) >= 64 then return '0'; else return '1'; end if;
    end function;

    -- line buffer {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- oscillator source
    signal s_src : std_logic := '0';
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- timing / controls
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(7 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_cx : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_zk : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_hue : unsigned(7 downto 0) := (others => '0');
    signal s_spd : signed(5 downto 0) := (others => '0');
    signal s_aspk : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_etch : unsigned(9 downto 0) := (others => '0');
    signal s_wt : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    signal s_mirror, s_three, s_palb, s_hard : std_logic := '0';
    -- orbit
    signal s_t : unsigned(7 downto 0) := (others => '0');
    signal s_d : unsigned(9 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_u12 is array (natural range <>) of unsigned(11 downto 0);
    type t_u21 is array (natural range <>) of unsigned(20 downto 0);
    type t_u23 is array (natural range <>) of unsigned(22 downto 0);
    type t_u24 is array (natural range <>) of unsigned(23 downto 0);
    type t_s16 is array (natural range <>) of signed(15 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_om : t_u10(0 to 5) := (others => (others => '0'));        -- offset magnitudes
    signal s_on : t_sl(0 to 5) := (others => '0');                     -- offset signs
    signal s_cxc : t_u11(1 to 3) := (others => to_unsigned(960, 11));  -- centre x (virtual)
    signal s_cyc : t_u11(1 to 3) := (others => to_unsigned(540, 11));  -- centre y
    signal s_c2 : t_u12(1 to 3) := (others => to_unsigned(1920, 12));  -- 2 cx
    signal s_q0 : t_u24(1 to 3) := (others => (others => '0'));
    signal s_inc0 : t_s16(1 to 3) := (others => (others => '0'));
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    -- serial multiplier
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(3 downto 0) := (others => '0');
    signal s_jst : std_logic := '0';
    signal s_jid : unsigned(3 downto 0) := (others => '0');
    type t_u7 is array (natural range <>) of unsigned(6 downto 0);
    signal s_stm : t_u7(0 to 5) := (others => (others => '0'));      -- |st(t + k)| registered

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1, s_fall2 : std_logic := '0';
    signal s_lc : unsigned(6 downto 0) := (others => '1');
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_ady : t_u11(1 to 3) := (others => to_unsigned(540, 11));
    signal s_dy2_n : t_u21(1 to 3) := (others => (others => '0'));
    signal s_dy2a : t_u23(1 to 3) := (others => (others => '0'));

    -- beam state
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_q : t_u24(1 to 3) := (others => (others => '0'));
    signal s_inc : t_s16(1 to 3) := (others => (others => '0'));

    -- pixel path
    signal s_qa1 : t_u24(1 to 3) := (others => (others => '0'));
    signal s_r2 : t_u24(1 to 3) := (others => (others => '0'));
    signal s_m3 : unsigned(23 downto 0) := (others => '0');
    signal s_r3_3 : unsigned(23 downto 0) := (others => '0');
    signal s_p3 : unsigned(23 downto 0) := (others => '0');
    signal s_y3, s_y4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_m4, s_r3c4 : unsigned(24 downto 0) := (others => '0');
    signal s_me4 : unsigned(19 downto 0) := (others => '0');
    signal s_f5 : unsigned(23 downto 0) := (others => '0');
    signal s_me5, s_me6 : unsigned(9 downto 0) := (others => '0');
    signal s_p6 : unsigned(23 downto 0) := (others => '0');
    signal s_idx7 : unsigned(7 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y12, s_y13, s_y14, s_y15, s_y16, s_y17x, s_yf17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            s_pal_q <= pal(to_integer(s_pal_ra));
        end if;
    end process p_pal;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, orbit, serial products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_c : signed(12 downto 0);
        variable v_i : signed(16 downto 0);
        variable v_ph : unsigned(7 downto 0);
        variable v_st : std_logic;
        variable v_jb : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 255 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- knobs
            s_zk <= unsigned(registers_in(0)(9 downto 2)) + 1;                 -- K1 Rings 1 .. 256 (wraps 256 -> 0: floor)
            if registers_in(0)(9 downto 2) = "11111111" then s_zk <= (others => '1'); end if;
            s_hue <= unsigned(registers_in(1)(9 downto 2));                    -- K2 Hue
            v_s := signed(resize(unsigned(registers_in(2)), 11)) - 512;        -- K3 Orbit, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(5 downto 0); end if;
            s_aspk <= unsigned(registers_in(3)(9 downto 7));                   -- K4 Aspect (8 zones)
            s_etch <= unsigned(registers_in(4));                               -- K5 Etch
            s_wt <= unsigned(registers_in(5)(9 downto 2));                     -- K6 Weight
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mirror <= registers_in(6)(0);
                s_three  <= not registers_in(6)(1);
                s_palb   <= registers_in(6)(2);
                s_hard   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_t <= s_t + unsigned(resize(s_spd, 8));
            end if;

            -- derived
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1); s_inc2 <= to_unsigned(32, 6);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14); s_inc2 <= to_unsigned(18, 6);
                when others => v_vw := resize(s_w, 14); s_inc2 <= to_unsigned(8, 6);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cx <= s_vw(11 downto 1);
            -- spread d = 0.6875 p12 (max 703)
            s_d <= resize(s_g(11 downto 3), 10) + resize(s_g(11 downto 5), 10) + resize(s_g(11 downto 6), 10);

            -- centres from the offsets (settle a few clocks after the jobs)
            for i in 1 to 3 loop
                if i = 2 and s_mirror = '1' then
                    if s_on(0) = '1' then v_c := signed(resize(s_cx, 13)) + signed(resize(s_om(0), 13));
                    else v_c := signed(resize(s_cx, 13)) - signed(resize(s_om(0), 13)); end if;
                else
                    if s_on(2 * i - 2) = '1' then v_c := signed(resize(s_cx, 13)) - signed(resize(s_om(2 * i - 2), 13));
                    else v_c := signed(resize(s_cx, 13)) + signed(resize(s_om(2 * i - 2), 13)); end if;
                end if;
                if v_c < 0 then v_c := (others => '0'); elsif v_c >= signed(resize(s_vw, 13)) then v_c := signed(resize(s_wm1, 13)); end if;
                s_cxc(i) <= unsigned(v_c(10 downto 0));
                if i = 2 and s_mirror = '1' then
                    if s_on(1) = '1' then v_c := to_signed(540, 13) - signed(resize(s_om(1), 13));
                    else v_c := to_signed(540, 13) + signed(resize(s_om(1), 13)); end if;
                else
                    if s_on(2 * i - 1) = '1' then v_c := to_signed(540, 13) - signed(resize(s_om(2 * i - 1), 13));
                    else v_c := to_signed(540, 13) + signed(resize(s_om(2 * i - 1), 13)); end if;
                end if;
                if v_c < 0 then v_c := (others => '0'); elsif v_c > 1079 then v_c := to_signed(1079, 13); end if;
                s_cyc(i) <= unsigned(v_c(10 downto 0));
                s_c2(i) <= s_cxc(i) & '0';
                case to_integer(s_xs) is
                    when 4      => v_i := to_signed(16, 17) - signed(shift_left(resize(s_c2(i), 17), 3));
                    when 3      => v_i := to_signed(9, 17) - signed(shift_left(resize(s_c2(i), 17), 2) + shift_left(resize(s_c2(i), 17), 1));
                    when others => v_i := to_signed(4, 17) - signed(shift_left(resize(s_c2(i), 17), 2));
                end case;
                s_inc0(i) <= v_i(15 downto 0);
            end loop;

            -- serial unit.  Zone: jobs 0..5 d x |st(t + k)| (offsets), 6..8
            -- (2 cx_i)^2, 9..11 the first line's dy_i^2; per line: 9..11.
            v_st := '0'; v_jb := (others => '0');
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 4   => v_st := '1'; v_jb := x"0";
                    when 20  => v_st := '1'; v_jb := x"1";
                    when 36  => v_st := '1'; v_jb := x"2";
                    when 52  => v_st := '1'; v_jb := x"3";
                    when 68  => v_st := '1'; v_jb := x"4";
                    when 84  => v_st := '1'; v_jb := x"5";
                    when 104 => v_st := '1'; v_jb := x"6";
                    when 120 => v_st := '1'; v_jb := x"7";
                    when 136 => v_st := '1'; v_jb := x"8";
                    when 160 => v_st := '1'; v_jb := x"9";
                    when 176 => v_st := '1'; v_jb := x"A";
                    when 192 => v_st := '1'; v_jb := x"B";
                    when others => null;
                end case;
            else
                case to_integer(s_lc) is
                    when 2  => v_st := '1'; v_jb := x"9";
                    when 18 => v_st := '1'; v_jb := x"A";
                    when 34 => v_st := '1'; v_jb := x"B";
                    when others => null;
                end case;
            end if;
            s_jst <= v_st; s_jid <= v_jb;
            s_stm(0) <= f_stmag(s_t); s_stm(1) <= f_stmag(s_t + 64); s_stm(2) <= f_stmag(s_t + 85);
            s_stm(3) <= f_stmag(s_t + 149); s_stm(4) <= f_stmag(s_t + 170); s_stm(5) <= f_stmag(s_t + 234);
            if s_jst = '1' then
                s_mjob <= s_jid; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
                case to_integer(s_jid) is
                    when 0 => s_ma <= resize(s_d, 24); s_mb <= resize(s_stm(0), 12);
                    when 1 => s_ma <= resize(s_d, 24); s_mb <= resize(s_stm(1), 12);
                    when 2 => s_ma <= resize(s_d, 24); s_mb <= resize(s_stm(2), 12);
                    when 3 => s_ma <= resize(s_d, 24); s_mb <= resize(s_stm(3), 12);
                    when 4 => s_ma <= resize(s_d, 24); s_mb <= resize(s_stm(4), 12);
                    when 5 => s_ma <= resize(s_d, 24); s_mb <= resize(s_stm(5), 12);
                    when 6 => s_ma <= resize(s_c2(1), 24); s_mb <= s_c2(1);
                    when 7 => s_ma <= resize(s_c2(2), 24); s_mb <= s_c2(2);
                    when 8 => s_ma <= resize(s_c2(3), 24); s_mb <= s_c2(3);
                    when 9 => s_ma <= resize(s_ady(1), 24); s_mb <= resize(s_ady(1), 12);
                    when 10 => s_ma <= resize(s_ady(2), 24); s_mb <= resize(s_ady(2), 12);
                    when others => s_ma <= resize(s_ady(3), 24); s_mb <= resize(s_ady(3), 12);
                end case;
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case to_integer(s_mjob) is
                    when 0 | 1 | 2 | 3 | 4 | 5 =>
                        s_om(to_integer(s_mjob)) <= s_macc(15 downto 6);
                    when 6 => s_q0(1) <= s_macc;
                    when 7 => s_q0(2) <= s_macc;
                    when 8 => s_q0(3) <= s_macc;
                    when 9 => s_dy2_n(1) <= s_macc(20 downto 0);
                    when 10 => s_dy2_n(2) <= s_macc(20 downto 0);
                    when others => s_dy2_n(3) <= s_macc(20 downto 0);
                end case;
            end if;
            -- offset signs (free-running from the phase)
            s_on(0) <= f_stneg(s_t); s_on(1) <= f_stneg(s_t + 64);
            s_on(2) <= f_stneg(s_t + 85); s_on(3) <= f_stneg(s_t + 149);
            s_on(4) <= f_stneg(s_t + 170); s_on(5) <= f_stneg(s_t + 234);

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid
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
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Source side: line-buffer write (video or oscillator); per-line |dy_i|
    -- (at the fall) and dy_i^2 with aspect (committed at the rise)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_dy : signed(12 downto 0);
        variable v_a : unsigned(22 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_otu <= to_unsigned(320, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot) & '0', 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot), 10);
            s_otv <= to_unsigned(320, 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot), 10);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_oty) & std_logic_vector(s_otu) & std_logic_vector(s_otv) & "00";
            else
                s_lb_wd <= data_in.y & data_in.u & data_in.v & "00";
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;

            s_fall1 <= '0';
            if data_in.avid = '0' and s_avq = '1' then s_fall1 <= '1'; end if;
            s_fall2 <= s_fall1;
            if s_fall1 = '1' then s_lc <= (others => '0'); elsif s_lc /= 127 then s_lc <= s_lc + 1; end if;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
            end if;
            -- |dy_i| for the coming line (free-running; the per-line jobs
            -- start 2 clocks after the fall, the zone jobs long after)
            for i in 1 to 3 loop
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - signed(resize(s_cyc(i), 13));
                if v_dy < 0 then s_ady(i) <= resize(unsigned(-v_dy), 11);
                else s_ady(i) <= unsigned(v_dy(10 downto 0)); end if;
            end loop;

            if data_in.avid = '1' and s_avq = '0' then
                for i in 1 to 3 loop
                    case to_integer(s_aspk) is
                        when 0 => v_a := resize(s_dy2_n(i)(20 downto 1), 23);
                        when 1 => v_a := resize(s_dy2_n(i)(20 downto 1), 23) + resize(s_dy2_n(i)(20 downto 2), 23);
                        when 2 => v_a := resize(s_dy2_n(i), 23);
                        when 3 => v_a := resize(s_dy2_n(i), 23) + resize(s_dy2_n(i)(20 downto 2), 23);
                        when 4 => v_a := resize(s_dy2_n(i), 23) + resize(s_dy2_n(i)(20 downto 1), 23);
                        when 5 => v_a := shift_left(resize(s_dy2_n(i), 23), 1);
                        when 6 => v_a := shift_left(resize(s_dy2_n(i), 23), 1) + resize(s_dy2_n(i), 23);
                        when others => v_a := shift_left(resize(s_dy2_n(i), 23), 2);
                    end case;
                    s_dy2a(i) <= v_a;
                end loop;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state: column, dx_i^2 by second differences
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_col <= (others => '0');
                for i in 1 to 3 loop
                    s_q(i) <= s_q0(i);
                    s_inc(i) <= s_inc0(i);
                end loop;
            elsif g_avid = '1' then
                s_col <= s_col + 1;
                for i in 1 to 3 loop
                    s_q(i) <= unsigned(signed(s_q(i)) + resize(s_inc(i), 24));
                    s_inc(i) <= s_inc(i) + signed(resize(s_inc2, 16));
                end loop;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_i : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: dx^2 copies; video read
            for i in 1 to 3 loop s_qa1(i) <= resize(s_q(i)(23 downto 2), 24); end loop;
            s_lb_ra <= s_col;

            -- 2: r_i^2
            for i in 1 to 3 loop s_r2(i) <= s_qa1(i) + resize(s_dy2a(i), 24); end loop;

            -- 3: min(r1, r2); r3 weight product; video (fabric copy)
            if s_r2(1) < s_r2(2) then s_m3 <= s_r2(1); else s_m3 <= s_r2(2); end if;
            s_r3_3 <= s_r2(3);
            s_p3 <= s_r2(3)(23 downto 8) * s_wt;
            s_y3 <= unsigned(s_lb_q(31 downto 22));

            -- 4: weighted r3 (or out of play with two centres); etch product
            if s_three = '1' then
                s_r3c4 <= resize(s_r3_3, 25) + resize(s_p3, 25);
            else
                s_r3c4 <= (others => '1');
            end if;
            s_m4 <= resize(s_m3, 25);
            if s_y3 > 64 then s_me4 <= (s_y3 - 64) * s_etch; else s_me4 <= (others => '0'); end if;

            -- 5: field = nearest centre
            if s_r3c4 < s_m4 then s_f5 <= s_r3c4(23 downto 0); else s_f5 <= s_m4(23 downto 0); end if;
            s_me5 <= s_me4(19 downto 10);

            -- 6: ring zoom
            s_p6 <= s_f5(23 downto 8) * s_zk;
            s_me6 <= s_me5;

            -- 7: index = rings + hue + etch
            s_idx7 <= s_p6(15 downto 8) + s_hue + s_me6(7 downto 0) + resize(s_me6(9 downto 8), 8);

            -- 8: palette address (hard: 8 bands)
            if s_hard = '1' then v_i := s_idx7(7 downto 5) & "00000"; else v_i := s_idx7; end if;
            s_pal_ra <= s_palb & v_i;

            -- 9: ROM register; 10: fabric copy; 11: colour
            s_y11 <= unsigned(s_pal_q(31 downto 22));
            s_u11 <= unsigned(s_pal_q(21 downto 12));
            s_v11 <= unsigned(s_pal_q(11 downto 2));

            -- 12..17: output conditioning (centre at stage 16 -> 17)
            s_csu(0) <= s_u11; s_csv(0) <= s_v11;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u11 - s_csu(7);
            s_boxv <= s_boxv + s_v11 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u17 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v17 <= v_sc(13 downto 4);
            s_y12 <= s_y11; s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14;
            s_y16 <= s_y15; s_y17x <= s_y16;
            v_sy := resize(s_y15, 12) + shift_left(resize(s_y16, 12), 1) + resize(s_y17x, 12);
            s_yf17 <= v_sy(11 downto 2);
            -- 18: luma toward black; blanking gate
            if s_yf17 > 64 then v_dl := s_yf17 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(16) = '1' then
                s_y18 <= s_yf17 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
            -- 19..20
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture mandala;
