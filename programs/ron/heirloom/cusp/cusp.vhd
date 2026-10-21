-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cusp v0.1.0 -- Heirloom #17 (2026-10-20), from WIN_20230315_00_30_57.
--
-- The photo: horizontal rainbow stripes (blue / cyan / green / magenta /
-- red / yellow) bent into right-pointing cusps by a triangular displacement
-- that repeats across the screen, dark navy inside the large cusps, two
-- pattern rows top and bottom.
--
-- The program: stripe phase = y * pitch + bend(x), where the bend is a
-- triangle wave of x (or the picture's luma: the picture drawn as stripe
-- displacement, an oscilloscope look) scaled by P12 and by a slow vertical
-- envelope (K3 Rows).  The phase picks a hue from a rainbow ROM; phases
-- under the Dark width paint navy, and the picture's luma always widens
-- the navy where it is bright, so the source carves the pattern even in
-- Tri mode.
--
--   K1  Pitch    stripe pitch
--   K2  Cusps    cusps across the screen (bend frequency)
--   K3  Rows     vertical envelope of the bend (pattern rows)
--   K4  Hue      hue cycling speed (0 = still)
--   K5  Dark     width of the navy band
--   K6  Speed    stripe scroll, bipolar (centre = still)
--   S7  Cusps    point Right / Left
--   S8  Palette  Rainbow / Duo (two hues)
--   S9  Bend     Tri (triangle of x) / Picture (the source luma)
--   S10 Comb     bend offset by half a cusp on alternate lines
--   S11 Source   Video / Osc
--   P12 BEND     bend depth: flat stripes .. cusps four stripes deep
--
-- Generated avid; output conditioning before the blanking gate; virtual
-- 1080-line geometry.  Latency 20 clocks.  EBR: 16 line buffer + 2 ROM.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cusp of program_top is
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
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_RB : t_keys := ((0, 328, 960, 360), (43, 840, 584, 64), (85, 580, 136, 216), (128, 678, 64, 663),
                               (171, 164, 440, 960), (213, 426, 887, 808), (256, 328, 960, 360));
    type t_pal is array (0 to 255) of std_logic_vector(31 downto 0);
    function f_lerp(a, b, t, n : integer) return integer is
    begin
        return a + ((b - a) * t) / n;
    end function;
    function f_rainbow return t_pal is
        variable r : t_pal;
        variable y, u, v : integer;
    begin
        for i in 0 to 255 loop
            for k in 0 to C_RB'length - 2 loop
                if i >= C_RB(k).i and i < C_RB(k + 1).i then
                    y := f_lerp(C_RB(k).y, C_RB(k + 1).y, i - C_RB(k).i, C_RB(k + 1).i - C_RB(k).i);
                    u := f_lerp(C_RB(k).u, C_RB(k + 1).u, i - C_RB(k).i, C_RB(k + 1).i - C_RB(k).i);
                    v := f_lerp(C_RB(k).v, C_RB(k + 1).v, i - C_RB(k).i, C_RB(k + 1).i - C_RB(k).i);
                end if;
            end loop;
            r(i) := std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
                    & std_logic_vector(to_unsigned(v, 10)) & "00";
        end loop;
        return r;
    end function;
    signal pal : t_pal := f_rainbow;
    signal s_pal_ra : unsigned(7 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');
    constant C_NAVY : std_logic_vector(31 downto 0) := "0001100100" & "0111100000" & "1010000000" & "00";

    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

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
    signal s_k1, s_k2, s_k3, s_k4, s_k5 : unsigned(7 downto 0) := (others => '0');
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_f, s_r : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_fys, s_rys : unsigned(11 downto 0) := to_unsigned(64, 12);
    signal s_gk : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_gxs : unsigned(8 downto 0) := to_unsigned(16, 9);
    signal s_hue : unsigned(7 downto 0) := (others => '0');
    signal s_darkw : unsigned(7 downto 0) := (others => '0');
    signal s_amp : unsigned(9 downto 0) := (others => '0');
    signal s_scr : unsigned(15 downto 0) := (others => '0');
    signal s_left, s_duo, s_pic, s_comb : std_logic := '0';
    signal s_ca, s_cb : std_logic_vector(31 downto 0) := (others => '0');
    signal s_zra : unsigned(7 downto 0) := (others => '0');
    -- serial multiplier
    signal s_ma : unsigned(19 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(19 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun, s_mjob : std_logic := '0';

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
    signal s_ly_n, s_ry_n : unsigned(15 downto 0) := (others => '0');
    signal s_env : unsigned(6 downto 0) := (others => '0');
    signal s_al_n, s_al : unsigned(9 downto 0) := (others => '0');       -- bend amplitude this line
    signal s_ly : unsigned(15 downto 0) := (others => '0');
    signal s_lpar : std_logic := '0';
    signal s_gx0 : unsigned(15 downto 0) := (others => '0');

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_gx : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u7 is array (natural range <>) of unsigned(6 downto 0);
    signal s_bt : t_u7(1 to 3) := (others => (others => '0'));
    signal s_y3, s_y4, s_y5, s_y6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_b4 : unsigned(6 downto 0) := (others => '0');
    signal s_pr5 : unsigned(16 downto 0) := (others => '0');
    signal s_ph6 : unsigned(15 downto 0) := (others => '0');
    signal s_dk7, s_dk8, s_dk9 : std_logic := '0';
    signal s_du7, s_du8, s_du9 : std_logic := '0';
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_rc9 : std_logic_vector(31 downto 0) := (others => '0');
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y11, s_y12, s_y13, s_y14, s_y15, s_y16x, s_yf16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17, s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17, s_u18, s_v18, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    -- Controls, vblank zone, per-field latches, serial products, duo colours
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_g : signed(12 downto 0);
        variable v_s : signed(10 downto 0);
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

            s_k1 <= unsigned(registers_in(0)(9 downto 2));
            s_k2 <= unsigned(registers_in(1)(9 downto 2));
            s_k3 <= unsigned(registers_in(2)(9 downto 2));
            s_k4 <= unsigned(registers_in(3)(9 downto 2));
            s_k5 <= unsigned(registers_in(4)(9 downto 2));
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;
            v_s := shift_right(v_s, 2);
            if v_s > -2 and v_s < 2 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_f <= resize(s_k1(7 downto 1), 8) + 16;          -- 16 .. 143 phase / virtual line
                s_r <= resize(s_k3(7 downto 2), 8) + 8;           -- 8 .. 71 phase / virtual line
                s_gk <= resize(s_k2(7 downto 2), 7) + 1;          -- 1 .. 64 phase / virtual unit
                s_hue <= s_hue + resize(s_k4(7 downto 3), 8);
                s_darkw <= s_k5;
                s_amp <= s_g(11 downto 2);
                s_left <= registers_in(6)(0);
                s_duo  <= registers_in(6)(1);
                s_pic  <= registers_in(6)(2);
                s_comb <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_scr <= s_scr + unsigned(resize(shift_left(s_k6r, 3), 16));
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => s_gxs <= shift_left(resize(s_gk, 9), 2);
                when 3      => s_gxs <= shift_left(resize(s_gk, 9), 1) + resize(s_gk, 9);
                when others => s_gxs <= shift_left(resize(s_gk, 9), 1);
            end case;

            -- serial products: f x ys (zone 4..), r x ys (zone 20..), >> 8
            if s_zc = 4 then
                s_ma <= resize(s_f, 20); s_mb <= s_ys; s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0'); s_mjob <= '0';
            elsif s_zc = 20 then
                s_ma <= resize(s_r, 20); s_mb <= s_ys; s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0'); s_mjob <= '1';
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(18 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                if s_mjob = '0' then s_fys <= s_macc(19 downto 8); else s_rys <= s_macc(19 downto 8); end if;
            end if;

            -- duo colours: two rainbow reads in the zone
            case to_integer(s_zc) is
                when 40 => s_zra <= s_hue;
                when 41 => s_zra <= s_hue + 128;
                when others => null;
            end case;
            case to_integer(s_zc) is
                when 43 => s_ca <= s_pal_q;
                when 44 => s_cb <= s_pal_q;
                when others => null;
            end case;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

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
    -- Source side: line-buffer write; per-line stripe phase, bend envelope
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_p : unsigned(16 downto 0);
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

            if s_zone = '1' then
                s_ly_n <= s_scr;
                s_ry_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_ly_n <= s_ly_n + resize(s_fys, 16);
                s_ry_n <= s_ry_n + resize(s_rys, 16);
            end if;
            -- bend envelope: 64 .. 127 of the amplitude
            s_env <= "1" & f_tri8(s_ry_n(15 downto 8))(6 downto 1);
            v_p := s_amp * s_env;
            s_al_n <= v_p(16 downto 7);

            if data_in.avid = '1' and s_avq = '0' then
                s_ly <= s_ly_n;
                s_al <= s_al_n;
                s_lpar <= not s_lpar;
                if s_comb = '1' and s_lpar = '1' then s_gx0 <= x"8000"; else s_gx0 <= (others => '0'); end if;
            end if;
        end if;
    end process p_src;

    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= (others => '0');
                s_gx <= s_gx0;
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_gx <= s_gx + resize(s_gxs, 16);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_b : unsigned(16 downto 0);
        variable v_d : unsigned(8 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: triangle of x; video read
            s_bt(1) <= f_tri8(s_gx(15 downto 8));
            s_lb_ra <= s_rdp;
            -- 3: fabric copy of the source luma
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            -- 4: bend source
            if s_pic = '1' then s_b4 <= s_y3(9 downto 3); else s_b4 <= s_bt(3); end if;
            s_y4 <= s_y3;
            -- 5: bend x amplitude
            s_pr5 <= s_b4 * s_al;
            s_y5 <= s_y4;
            -- 6: stripe phase
            v_b := s_pr5(15 downto 0) & '0';
            if s_left = '1' then s_ph6 <= s_ly - v_b(15 downto 0); else s_ph6 <= s_ly + v_b(15 downto 0); end if;
            s_y6 <= s_y5;
            -- 7: rainbow address; navy test (the luma widens the navy)
            if s_zone = '1' then s_pal_ra <= s_zra; else s_pal_ra <= s_ph6(15 downto 8) + s_hue; end if;
            v_d := resize(s_ph6(15 downto 8), 9) + resize(s_y6(9 downto 4), 9);
            if v_d < s_darkw then s_dk7 <= '1'; else s_dk7 <= '0'; end if;
            s_du7 <= s_ph6(15);
            -- 8: ROM output register; 9: fabric
            s_dk8 <= s_dk7; s_du8 <= s_du7;
            s_rc9 <= s_pal_q; s_dk9 <= s_dk8; s_du9 <= s_du8;
            -- 10: compose
            if s_dk9 = '1' then
                s_y10 <= unsigned(C_NAVY(31 downto 22)); s_u10 <= unsigned(C_NAVY(21 downto 12)); s_v10 <= unsigned(C_NAVY(11 downto 2));
            elsif s_duo = '1' then
                if s_du9 = '1' then
                    s_y10 <= unsigned(s_ca(31 downto 22)); s_u10 <= unsigned(s_ca(21 downto 12)); s_v10 <= unsigned(s_ca(11 downto 2));
                else
                    s_y10 <= unsigned(s_cb(31 downto 22)); s_u10 <= unsigned(s_cb(21 downto 12)); s_v10 <= unsigned(s_cb(11 downto 2));
                end if;
            else
                s_y10 <= unsigned(s_rc9(31 downto 22)); s_u10 <= unsigned(s_rc9(21 downto 12)); s_v10 <= unsigned(s_rc9(11 downto 2));
            end if;

            -- 11..16: output conditioning (centre at stage 15 -> 16)
            s_csu(0) <= s_u10; s_csv(0) <= s_v10;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u10 - s_csu(7);
            s_boxv <= s_boxv + s_v10 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u16 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v16 <= v_sc(13 downto 4);
            s_y11 <= s_y10; s_y12 <= s_y11; s_y13 <= s_y12; s_y14 <= s_y13;
            s_y15 <= s_y14; s_y16x <= s_y15;
            v_sy := resize(s_y14, 12) + shift_left(resize(s_y15, 12), 1) + resize(s_y16x, 12);
            s_yf16 <= v_sy(11 downto 2);
            -- 17: luma toward black; blanking gate
            if s_yf16 > 64 then v_dl := s_yf16 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(15) = '1' then
                s_y17 <= s_yf16 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            s_bt(2) <= s_bt(1); s_bt(3) <= s_bt(2);
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cusp;
