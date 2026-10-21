-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Warp v0.1.0 -- Heirloom #48 (2026-10-20), from WIN_20230909_23_29_18.
--
-- The photo: an orange / green / violet stripe field warped into a large
-- arc and sheared sideways in horizontal blocks, like a perspective fan,
-- fading to black at the left edge.
--
-- The program: stripe phase = a*x + b*y + c*r^2 (+ the source luma), where
-- the r^2 term (P12) bends the stripes into arcs about the screen centre;
-- horizontal blocks (K3) each shear the phase and the source sideways by a
-- random amount (K4); three colours or a rainbow paint the phase, with a
-- hue rotation (K5); the left edge fades to black.
--
--   K1  Pitch    stripe pitch
--   K2  Angle    diagonal angle
--   K3  Block    block height
--   K4  Shear    per-block sideways shear
--   K5  Hue      hue rotation
--   K6  Drift    stripes flow, bipolar (centre = still)
--   S7  Shear    Off / On
--   S8  Palette  Trio (orange / green / violet) / Rainbow
--   S9  Arcs     Convex / Concave
--   S10 Mirror   the x term mirrored about the centre
--   S11 Source   Video / Osc
--   P12 CURVE    the r^2 weight: straight stripes .. tight arcs
--
-- Generated avid; output conditioning before the blanking gate; virtual
-- 1080-line geometry.  Latency 20 clocks.  EBR: 6 line buffer + 2 ROM.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture warp of program_top is
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
    type t_cs is array (0 to 15) of integer range -128 to 127;
    constant C_COS : t_cs := (127, 117, 90, 49, 0, -49, -90, -117, -127, -117, -90, -49, 0, 49, 90, 117);
    constant C_SINR : t_cs := (0, 49, 90, 117, 127, 117, 90, 49, 0, -49, -90, -117, -127, -117, -90, -49);
    type t_col is array (0 to 2) of integer;
    type t_trio is array (0 to 3) of t_col;
    constant C_TRIO : t_trio := ((584, 772, 212), (580, 136, 216), (349, 756, 852), (580, 136, 216));

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
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_k1, s_k2, s_k3, s_k4, s_k5 : unsigned(7 downto 0) := (others => '0');
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_a, s_b : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_ax : unsigned(9 downto 0) := to_unsigned(32, 10);
    signal s_bys : unsigned(11 downto 0) := (others => '0');
    signal s_bh : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_bhq : unsigned(17 downto 0) := (others => '0');
    signal s_shk : unsigned(7 downto 0) := (others => '0');
    signal s_cos, s_sin : signed(7 downto 0) := (others => '0');
    signal s_cw : unsigned(7 downto 0) := (others => '0');            -- curve weight
    signal s_scr : unsigned(15 downto 0) := (others => '0');
    signal s_cx2sq : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_fx0 : unsigned(19 downto 0) := (others => '0');          -- a * cx (Mirror start)
    signal s_t1, s_t2 : signed(12 downto 0) := (others => '0');       -- left-fade thresholds (Q.1 dx)
    signal s_shr, s_rb, s_conc, s_mir : std_logic := '0';
    -- serial multiplier
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(1 downto 0) := (others => '0');
    signal s_lfsr : unsigned(15 downto 0) := x"91D3";

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
    signal s_cy_n : unsigned(17 downto 0) := (others => '0');
    signal s_ly_n : unsigned(15 downto 0) := (others => '0');
    signal s_dph_n : unsigned(15 downto 0) := (others => '0');
    signal s_dpx_n : unsigned(8 downto 0) := (others => '0');
    signal s_ady_n : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_dy2_n, s_dy2 : unsigned(19 downto 0) := to_unsigned(291600, 20);
    signal s_p0 : unsigned(15 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_dxh : signed(12 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');
    signal s_fx : unsigned(19 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u16 is array (natural range <>) of unsigned(15 downto 0);
    type t_u2 is array (natural range <>) of unsigned(1 downto 0);
    signal s_fxd : t_u16(1 to 4) := (others => (others => '0'));
    signal s_fd : t_u2(1 to 8) := (others => (others => '0'));
    signal s_q1 : unsigned(23 downto 0) := (others => '0');
    signal s_qi2 : unsigned(21 downto 0) := (others => '0');
    signal s_r3 : unsigned(21 downto 0) := (others => '0');
    signal s_y3, s_y4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_pr4 : unsigned(19 downto 0) := (others => '0');
    signal s_ph5 : unsigned(15 downto 0) := (others => '0');
    signal s_ti6 : unsigned(1 downto 0) := (others => '0');
    signal s_ti7, s_ti8 : unsigned(1 downto 0) := (others => '0');
    signal s_rc8 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_du10, s_dv10 : signed(10 downto 0) := (others => '0');
    signal s_y10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_puc11, s_pus11, s_pvc11, s_pvs11 : signed(18 downto 0) := (others => '0');
    signal s_y11 : unsigned(9 downto 0) := to_unsigned(64, 10);
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

    function f_clamp13(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 960 then return to_unsigned(960, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

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
    -- Controls, vblank zone, per-field latches, serial products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_g : signed(12 downto 0);
        variable v_s : signed(10 downto 0);
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
                s_a <= resize(s_k1(7 downto 1), 8) + 4;               -- 4 .. 131 phase / virtual unit
                s_b <= resize(s_k2(7 downto 1), 8);                   -- 0 .. 127 phase / virtual line
                s_bh <= resize(s_k3, 10) + resize(s_k3, 10) + 24;     -- 24 .. 534 virtual lines
                s_shk <= s_k4;
                s_cos <= to_signed(C_COS(to_integer(s_k5(7 downto 4))), 8);
                s_sin <= to_signed(C_SINR(to_integer(s_k5(7 downto 4))), 8);
                s_cw <= s_g(11 downto 4);
                s_shr  <= registers_in(6)(0);
                s_rb   <= registers_in(6)(1);
                s_conc <= registers_in(6)(2);
                s_mir  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_scr <= s_scr + unsigned(resize(shift_left(s_k6r, 3), 16));
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            s_bhq <= s_bh & x"00";
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                               s_ax <= shift_left(resize(s_a, 10), 2);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                               s_ax <= shift_left(resize(s_a, 10), 1) + resize(s_a, 10);
                when others => v_vw := resize(s_w, 14);
                               s_ax <= shift_left(resize(s_a, 10), 1);
            end case;
            s_vw <= v_vw(11 downto 0);
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_vw, 16), 3)); s_inc2 <= to_unsigned(32, 6);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_vw, 16), 2) + shift_left(resize(s_vw, 16), 1)); s_inc2 <= to_unsigned(18, 6);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_vw, 16), 2)); s_inc2 <= to_unsigned(8, 6);
            end case;
            s_inc0 <= v_i(14 downto 0);
            -- left fade: dx (Q.1, = 2x - vw) below -3/4 vw -> black, below -5/8 vw -> half
            s_t1 <= -signed(resize(s_vw(11 downto 1), 13) + resize(s_vw(11 downto 2), 13));
            s_t2 <= -signed(resize(s_vw(11 downto 1), 13) + resize(s_vw(11 downto 3), 13));

            -- serial: job 0 b x ys (>> 8), job 1 vw^2, job 2 a x cx, job 3 ady^2 (per line)
            if s_zc = 4 then
                s_ma <= resize(s_b, 24); s_mb <= s_ys; s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0'); s_mjob <= "00";
            elsif s_zc = 20 then
                s_ma <= resize(s_vw, 24); s_mb <= s_vw; s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0'); s_mjob <= "01";
            elsif s_zc = 36 then
                s_ma <= resize(s_a, 24); s_mb <= '0' & s_vw(11 downto 1); s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0'); s_mjob <= "10";
            elsif s_zc = 52 or s_fall2 = '1' then
                s_ma <= resize(s_ady_n, 24); s_mb <= resize(s_ady_n, 12); s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0'); s_mjob <= "11";
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "00"   => s_bys <= s_macc(19 downto 8);
                    when "01"   => s_cx2sq <= s_macc;
                    when "10"   => s_fx0 <= s_macc(19 downto 0);
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
    -- Source side: line-buffer write; per-line phase, blocks, |dy|
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(18 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_dy : signed(12 downto 0);
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
                s_vy_n <= (others => '0');
                s_cy_n <= (others => '0');
                s_ly_n <= s_scr;
                s_dph_n <= (others => '0');
                s_dpx_n <= (others => '0');
                s_ady_n <= to_unsigned(540, 11);
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_ly_n <= s_ly_n + resize(s_bys, 16);
                v_c := resize(s_cy_n, 19) + s_ys;
                if v_c >= s_bhq then
                    s_cy_n <= resize(v_c - s_bhq, 18);
                    s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                    v_t := s_lfsr(7 downto 0) * s_shk;
                    s_dph_n <= v_t;
                    s_dpx_n <= v_t(15 downto 7);
                else
                    s_cy_n <= v_c(17 downto 0);
                end if;
            elsif s_fall2 = '1' then
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - 540;
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11); else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_dy2 <= s_dy2_n;
                if s_shr = '1' then
                    s_p0 <= s_ly_n + s_dph_n;
                    if s_dpx_n = 0 then s_rst <= (others => '0');
                    else s_rst <= s_wm1(10 downto 0) + 1 - resize(s_dpx_n, 11); end if;
                else
                    s_p0 <= s_ly_n;
                    s_rst <= (others => '0');
                end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state: dx (Q.1), dx^2 by second differences, a * x (or a * |x|)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
                s_dxh <= -signed(resize(s_vw, 13));
                s_q <= s_cx2sq;
                s_inc <= s_inc0;
                if s_mir = '1' then s_fx <= s_fx0; else s_fx <= (others => '0'); end if;
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_dxh <= s_dxh + signed(resize(s_xs, 13));
                s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                s_inc <= s_inc + signed(resize(s_inc2, 15));
                if s_mir = '1' and s_dxh < 0 then s_fx <= s_fx - resize(s_ax, 20); else s_fx <= s_fx + resize(s_ax, 20); end if;
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
        variable v_ph : unsigned(15 downto 0);
        variable v_yl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: copies; left fade class; read
            s_q1 <= s_q;
            s_fxd(1) <= s_fx(15 downto 0);
            if s_dxh < s_t1 then s_fd(1) <= "10"; elsif s_dxh < s_t2 then s_fd(1) <= "01"; else s_fd(1) <= "00"; end if;
            s_lb_ra <= s_rdp;
            -- 2: dx^2
            s_qi2 <= s_q1(23 downto 2);
            -- 3: r^2; source luma (fabric)
            s_r3 <= s_qi2 + resize(s_dy2, 22);
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            -- 4: curve product
            s_pr4 <= s_r3(20 downto 9) * s_cw;
            s_y4 <= s_y3;
            -- 5: phase = line start + a x + luma (+-) curve
            v_ph := s_p0 + s_fxd(4) + resize(s_y4(9 downto 2) & "0000000", 16);
            if s_conc = '1' then s_ph5 <= v_ph - (s_pr4(14 downto 0) & '0'); else s_ph5 <= v_ph + (s_pr4(14 downto 0) & '0'); end if;
            -- 6: trio index / rainbow address
            s_ti6 <= s_ph5(15 downto 14);
            s_pal_ra <= s_ph5(15 downto 8);
            -- 7: ROM output register; 8: fabric
            s_ti7 <= s_ti6; s_ti8 <= s_ti7;
            s_rc8 <= s_pal_q;
            -- 9: colour; left fade
            if s_rb = '1' then
                v_yl := unsigned(s_rc8(31 downto 22));
                s_u9 <= unsigned(s_rc8(21 downto 12)); s_v9 <= unsigned(s_rc8(11 downto 2));
            else
                v_yl := to_unsigned(C_TRIO(to_integer(s_ti8))(0), 10);
                s_u9 <= to_unsigned(C_TRIO(to_integer(s_ti8))(1), 10);
                s_v9 <= to_unsigned(C_TRIO(to_integer(s_ti8))(2), 10);
            end if;
            case s_fd(8) is
                when "10"   => s_y9 <= to_unsigned(64, 10);
                when "01"   => s_y9 <= to_unsigned(32, 10) + resize(v_yl(9 downto 1), 10);
                when others => s_y9 <= v_yl;
            end case;
            -- 10..12: hue rotation
            s_du10 <= signed(resize(s_u9, 11)) - 512;
            s_dv10 <= signed(resize(s_v9, 11)) - 512;
            s_y10 <= s_y9;
            s_puc11 <= s_du10 * s_cos; s_pus11 <= s_du10 * s_sin;
            s_pvc11 <= s_dv10 * s_cos; s_pvs11 <= s_dv10 * s_sin;
            s_y11 <= s_y10;
            s_u12 <= f_clamp13(resize(shift_right(s_puc11 - s_pvs11, 7), 13) + 512);
            s_v12 <= f_clamp13(resize(shift_right(s_pus11 + s_pvc11, 7), 13) + 512);
            s_y12 <= s_y11;

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
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            -- delays
            for i in 2 to 4 loop s_fxd(i) <= s_fxd(i - 1); end loop;
            for i in 2 to 8 loop s_fd(i) <= s_fd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture warp;
