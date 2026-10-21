-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cocoon v0.2.0 -- Heirloom #2 (2026-10-20), from WIN_20220829_23_30_37.
--
-- The photo: a horizontally stretched hexagonal window in the middle of the
-- screen filled with a dense woven chevron (zigzag) texture, lit by a radial
-- glow -- red core fading to deep blue -- with light gray outer columns, and
-- the source video posterized into colour blobs keyed in at the flanks,
-- torn sideways in horizontal bands.
--
-- The program: a radial glow field (red -> blue -> gray, from the squared
-- distance to the screen centre) with a hexagonal / elliptical cocoon in
-- which a chevron texture picks the colour instead; the video, posterized,
-- shows wherever its luma passes the key, torn sideways in random bands.
--
--   K1  Key     luma threshold for the video to show (raised inside the cocoon)
--   K2  Weave   chevron zigzag frequency across
--   K3  Rows    chevron row height
--   K4  Glow    glow radius (small red core .. the whole screen warm)
--   K5  Tear    sideways tearing of the video in 8-line bands
--   K6  Speed   texture scroll, bipolar (centre = still)
--   S7  Shape   Hexagon / Ellipse
--   S8  Palette Fire (red, yellow, blue) / Ice (the complement hues)
--   S9  Poster  video posterized (8 luma x 4 x 4 chroma) / full colour
--   S10 Source  Video / Osc (an internal triangle-wave oscillator replaces the video)
--   S11 Weave   Chevron rows / Diagonal stripes
--   P12 COCOON  size: a small core .. fills the screen with the weave
--
-- Geometry in virtual 1080-line units (round at every mode); the video read
-- pointer is real pixels.  dx^2 by second differences per pixel, dy^2 and
-- the per-frame squares on one serial multiplier.  Generated avid; output
-- conditioning before the blanking gate.  Latency 20 clocks.  EBR: 16 line
-- buffer + 4 palette.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cocoon of program_top is
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

    -- palettes: A (outside) = warm core -> red -> blue -> gray; B (inside the
    -- cocoon) = red -> orange -> yellow -> green -> yellow, over the chevron
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_KA : t_keys := ((0, 520, 830, 260), (40, 328, 960, 360), (150, 164, 440, 960), (255, 700, 512, 512));
    constant C_KB : t_keys := ((0, 328, 960, 360), (64, 584, 772, 212), (128, 840, 584, 64),
                               (192, 580, 136, 216), (255, 840, 584, 64));
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

    -- triangle of an 8-bit phase -> 7 bits
    function f_tri(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- line buffer {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- internal oscillator source (Source = Osc): triangle waves of x and y
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
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);     -- virtual width = 2 cx
    signal s_thr, s_thrin : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_fx : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_fy : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_gk : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal s_tk : unsigned(7 downto 0) := (others => '0');
    signal s_spd : signed(6 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_r : unsigned(10 downto 0) := to_unsigned(600, 11);      -- cocoon reach
    signal s_hh : unsigned(10 downto 0) := to_unsigned(337, 11);     -- half height
    signal s_hh2 : unsigned(21 downto 0) := (others => '0');
    signal s_cx2sq : unsigned(23 downto 0) := (others => '0');       -- (2 cx)^2
    signal s_inc0 : signed(14 downto 0) := (others => '0');          -- xs^2 - 2 xs (2 cx)
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);       -- 2 xs^2
    signal s_fxs : unsigned(9 downto 0) := to_unsigned(16, 10);      -- fx * xs
    signal s_fys : unsigned(9 downto 0) := to_unsigned(16, 10);      -- fy * ys / 256
    signal s_ph : unsigned(7 downto 0) := (others => '0');
    signal s_shape, s_ice, s_poster, s_diag : std_logic := '0';
    -- serial multiplier
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(1 downto 0) := (others => '0');
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";

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
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_ady_n, s_ady : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_dy2_n, s_dy2 : unsigned(19 downto 0) := to_unsigned(291600, 20);
    signal s_adyok : std_logic := '0';
    signal s_py : unsigned(11 downto 0) := (others => '0');
    signal s_tp_n, s_tp : unsigned(7 downto 0) := (others => '0');
    signal s_tear_n : unsigned(10 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');          -- read start pointer
    signal s_l8 : unsigned(2 downto 0) := (others => '0');            -- line mod 8

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_dxh : signed(12 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');
    signal s_px : unsigned(12 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_u7 is array (natural range <>) of unsigned(6 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_adx1 : unsigned(10 downto 0) := (others => '0');
    signal s_q1 : unsigned(23 downto 0) := (others => '0');
    signal s_a1 : unsigned(7 downto 0) := (others => '0');
    signal s_qi2 : unsigned(21 downto 0) := (others => '0');
    signal s_sum2 : unsigned(11 downto 0) := (others => '0');
    signal s_t : t_u7(2 to 7) := (others => (others => '0'));
    signal s_hx3 : unsigned(21 downto 0) := (others => '0');
    signal s_hex : t_sl(3 to 5) := (others => '0');
    signal s_r2_4 : unsigned(22 downto 0) := (others => '0');
    signal s_l10_5 : unsigned(9 downto 0) := (others => '0');
    signal s_ell5 : std_logic := '0';
    signal s_lp6 : unsigned(18 downto 0) := (others => '0');
    signal s_in : t_sl(6 to 11) := (others => '0');
    signal s_l7 : unsigned(7 downto 0) := (others => '0');
    signal s_rdd : t_u11(1 to 6) := (others => (others => '0'));
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y10, s_u10, s_v10, s_yp10, s_up10, s_vp10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py10, s_pu10, s_pv10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_key11, s_vid11 : std_logic := '0';
    signal s_vy11, s_vu11, s_vv11 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py11, s_pu11, s_pv11 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y13, s_y14, s_y15, s_y16, s_y17, s_y18x, s_yf18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- RAMs
    ---------------------------------------------------------------------------
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
    -- Controls, vblank zone, per-field latches, serial products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_t : unsigned(10 downto 0);
        variable v_p : unsigned(11 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_i : signed(15 downto 0);
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
            elsif s_zc /= 127 then
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
            v_k := unsigned(registers_in(0));                                  -- K1 Key: 96 .. 991
            s_thr <= to_unsigned(96, 10) + v_k - resize(v_k(9 downto 3), 10);
            v_t := resize(s_thr, 11) + 192;
            if v_t > 1000 then s_thrin <= to_unsigned(1000, 10); else s_thrin <= v_t(9 downto 0); end if;
            s_fx <= resize(unsigned(registers_in(1)(9 downto 4)), 7) + 1;      -- K2 Weave 1 .. 64
            s_fy <= resize(unsigned(registers_in(2)(9 downto 3)), 8) + 4;      -- K3 Rows 4 .. 131
            s_gk <= resize(unsigned(registers_in(3)(9 downto 2)), 9) + 8;      -- K4 Glow 8 .. 263
            s_tk <= unsigned(registers_in(4)(9 downto 2));                     -- K5 Tear
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;        -- K6 Speed, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(6 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_shape  <= registers_in(6)(0);
                s_ice    <= registers_in(6)(1);
                s_poster <= registers_in(6)(2);
                s_src    <= registers_in(6)(3);
                s_diag   <= registers_in(6)(4);
                s_ph <= s_ph + unsigned(resize(s_spd, 8));
            end if;

            -- derived
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            -- reach 96 + 1.40625 p12 (max 1535); half height 9/16 of it
            v_p := to_unsigned(96, 12) + resize(s_g(11 downto 2), 12) + resize(s_g(11 downto 4), 12)
                   + resize(s_g(11 downto 5), 12) + resize(s_g(11 downto 7), 12);
            s_r <= v_p(10 downto 0);
            s_hh <= resize(s_r(10 downto 1), 11) + resize(s_r(10 downto 4), 11);
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_vw, 16), 3));
                               s_inc2 <= to_unsigned(32, 6);
                               s_fxs <= shift_left(resize(s_fx, 10), 2);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_vw, 16), 2) + shift_left(resize(s_vw, 16), 1));
                               s_inc2 <= to_unsigned(18, 6);
                               s_fxs <= shift_left(resize(s_fx, 10), 1) + resize(s_fx, 10);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_vw, 16), 2));
                               s_inc2 <= to_unsigned(8, 6);
                               s_fxs <= shift_left(resize(s_fx, 10), 1);
            end case;
            s_inc0 <= v_i(14 downto 0);

            -- serial multiplier: jobs 0 (2cx)^2, 1 hh^2, 2 fy x ys, 3 ady^2
            if s_zc = 4 then
                s_ma <= resize(s_vw, 24); s_mb <= s_vw; s_mjob <= "00"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 24 then
                s_ma <= resize(s_hh, 24); s_mb <= resize(s_hh, 12); s_mjob <= "01"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 44 then
                s_ma <= resize(s_fy, 24); s_mb <= s_ys; s_mjob <= "10"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 64 or s_fall2 = '1' then
                s_ma <= resize(s_ady_n, 24); s_mb <= resize(s_ady_n, 12); s_mjob <= "11"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "00"   => s_cx2sq <= s_macc;
                    when "01"   => s_hh2 <= s_macc(21 downto 0);
                    when "10"   => s_fys <= s_macc(17 downto 8);
                    when others => s_dy2_n <= s_macc(19 downto 0);
                end case;
            end if;

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
    -- Source side: line-buffer write; per-line values (next at the source
    -- avid fall, committed at its rise)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_dy : signed(12 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_tr : unsigned(10 downto 0);
        variable v_a : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator source: x ramp per source pixel, y ramp per line, phase per field
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

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_ady_n <= to_unsigned(540, 11);
                s_py <= (others => '0');
                s_l8 <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_tear_n <= (others => '0');
            elsif s_fall1 = '1' then
                -- line end (one clock after the fall): next line's values
                s_vy_n <= s_vy_n + s_ys;
                s_py <= s_py + s_fys;
                s_l8 <= s_l8 + 1;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                if s_l8 = 7 then
                    v_t := s_lfsr(7 downto 0) * s_tk;
                    v_tr := resize(v_t(15 downto 6), 11);
                    if v_tr >= s_wm1(10 downto 0) then s_tear_n <= s_wm1(10 downto 0);
                    else s_tear_n <= v_tr; end if;
                end if;
            elsif s_fall2 = '1' then
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - 540;
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11);
                else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
            end if;
            -- texture row phase for the coming line (tri of the row ramp, or the
            -- ramp itself for diagonal stripes) plus the scroll phase
            if s_diag = '1' then v_a := s_py(10 downto 3); else v_a := '0' & f_tri(s_py(10 downto 3)); end if;
            s_tp_n <= v_a + s_ph;

            if data_in.avid = '1' and s_avq = '0' then
                s_ady <= s_ady_n;
                s_dy2 <= s_dy2_n;
                if s_ady_n <= s_hh then s_adyok <= '1'; else s_adyok <= '0'; end if;
                s_tp <= s_tp_n;
                if s_tear_n = 0 then s_rst <= (others => '0'); else s_rst <= s_wm1(10 downto 0) + 1 - s_tear_n; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state: read pointer, dx (Q.1), dx^2 by second differences, texture x
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
                s_dxh <= -signed(resize(s_vw, 13));
                s_q <= s_cx2sq;
                s_inc <= s_inc0;
                s_px <= (others => '0');
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_dxh <= s_dxh + signed(resize(s_xs, 13));
                s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                s_inc <= s_inc + signed(resize(s_inc2, 15));
                s_px <= s_px + s_fxs;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ad : signed(12 downto 0);
        variable v_l : unsigned(12 downto 0);
        variable v_i : unsigned(8 downto 0);
        variable v_c : unsigned(10 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: |dx|, dx^2 copy, texture phase
            if s_dxh < 0 then v_ad := -s_dxh; else v_ad := s_dxh; end if;
            s_adx1 <= unsigned(v_ad(11 downto 1));
            s_q1 <= s_q;
            s_a1 <= s_px(11 downto 4) + s_tp;
            s_rdd(1) <= s_rdp;

            -- 2: dx^2 (integer), |dx| + |dy|, triangle
            s_qi2 <= s_q1(23 downto 2);
            s_sum2 <= resize(s_adx1, 12) + resize(s_ady, 12);
            s_t(2) <= f_tri(s_a1);

            -- 3: 3/8 dx^2, hexagon test
            s_hx3 <= resize(s_qi2(21 downto 2), 22) + resize(s_qi2(21 downto 3), 22);
            if s_sum2 <= s_r and s_adyok = '1' then s_hex(3) <= '1'; else s_hex(3) <= '0'; end if;

            -- 4: r^2
            s_r2_4 <= resize(s_hx3, 23) + resize(s_dy2, 23);

            -- 5: glow level, ellipse test
            v_l := s_r2_4(22 downto 10);
            if v_l > 1023 then s_l10_5 <= (others => '1'); else s_l10_5 <= v_l(9 downto 0); end if;
            if s_r2_4 <= resize(s_hh2, 23) then s_ell5 <= '1'; else s_ell5 <= '0'; end if;

            -- 6: glow product; inside flag
            s_lp6 <= s_l10_5 * s_gk;
            if s_shape = '1' then s_in(6) <= s_ell5; else s_in(6) <= s_hex(5); end if;

            -- 7: glow index; video read
            if s_lp6(18 downto 14) /= 0 then s_l7 <= (others => '1'); else s_l7 <= s_lp6(13 downto 6); end if;
            s_lb_ra <= s_rdd(6);

            -- 8: palette address (B inside: 2t + L/4, saturated)
            v_i := resize(s_t(7) & '0', 9) + resize(s_l7(7 downto 2), 9);
            if s_in(7) = '1' then
                if v_i(8) = '1' then s_pal_ra <= '1' & x"FF"; else s_pal_ra <= '1' & v_i(7 downto 0); end if;
            else
                s_pal_ra <= '0' & s_l7;
            end if;

            -- 9: video (fabric copy of the buffer output register)
            s_y9 <= unsigned(s_lb_q(31 downto 22));
            s_u9 <= unsigned(s_lb_q(21 downto 12));
            s_v9 <= unsigned(s_lb_q(11 downto 2));

            -- 10: palette fabric copy; posterize
            s_py10 <= unsigned(s_pal_q(31 downto 22));
            s_pu10 <= unsigned(s_pal_q(21 downto 12));
            s_pv10 <= unsigned(s_pal_q(11 downto 2));
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            if s_y9 > 64 then v_c := resize(s_y9 - 64, 11); else v_c := (others => '0'); end if;
            s_yp10 <= resize(v_c(9 downto 7) & "0000000", 10) + 64;
            v_c := resize(s_u9, 11) + 64;
            if v_c(10) = '1' then s_up10 <= to_unsigned(960, 10); else s_up10 <= v_c(9 downto 7) & "0000000"; end if;
            v_c := resize(s_v9, 11) + 64;
            if v_c(10) = '1' then s_vp10 <= to_unsigned(960, 10); else s_vp10 <= v_c(9 downto 7) & "0000000"; end if;

            -- 11: key; palette hue complement (Ice); posterized or live video
            if s_in(10) = '1' then
                if s_y10 >= s_thrin then s_key11 <= '1'; else s_key11 <= '0'; end if;
            else
                if s_y10 >= s_thr then s_key11 <= '1'; else s_key11 <= '0'; end if;
            end if;
            s_py11 <= s_py10;
            if s_ice = '1' then
                s_pu11 <= resize(to_unsigned(1024, 11) - resize(s_pu10, 11), 10);
                s_pv11 <= resize(to_unsigned(1024, 11) - resize(s_pv10, 11), 10);
            else
                s_pu11 <= s_pu10; s_pv11 <= s_pv10;
            end if;
            if s_poster = '1' then
                s_vy11 <= s_yp10; s_vu11 <= s_up10; s_vv11 <= s_vp10;
            else
                s_vy11 <= s_y10; s_vu11 <= s_u10; s_vv11 <= s_v10;
            end if;

            -- 12: compose
            if s_key11 = '1' then
                s_y12 <= s_vy11; s_u12 <= s_vu11; s_v12 <= s_vv11;
            else
                s_y12 <= s_py11; s_u12 <= s_pu11; s_v12 <= s_pv11;
            end if;

            -- 13..18: output conditioning (centre at stage 17 -> 18)
            s_csu(0) <= s_u12; s_csv(0) <= s_v12;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u12 - s_csu(7);
            s_boxv <= s_boxv + s_v12 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v18 <= v_sc(13 downto 4);
            s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14; s_y16 <= s_y15;
            s_y17 <= s_y16; s_y18x <= s_y17;
            v_sy := resize(s_y16, 12) + shift_left(resize(s_y17, 12), 1) + resize(s_y18x, 12);
            s_yf18 <= v_sy(11 downto 2);
            -- 19: luma toward black; blanking gate
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(17) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            -- 20
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            -- delays
            for i in 3 to 7 loop s_t(i) <= s_t(i - 1); end loop;
            for i in 4 to 5 loop s_hex(i) <= s_hex(i - 1); end loop;
            for i in 7 to 11 loop s_in(i) <= s_in(i - 1); end loop;
            for i in 2 to 6 loop s_rdd(i) <= s_rdd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cocoon;
