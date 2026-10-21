-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cathode v0.1.0 -- Heirloom #23 (2026-10-20), from WIN_20230407_23_48_05.
--
-- The photo: a glitchy magenta / green / yellow pixel mosaic -- coarse
-- blocks, rows torn sideways -- with two large purple parabolic "tube"
-- shapes sweeping down through it.
--
-- The program: the video (or the oscillator) as a coarse mosaic (K1, sampled
-- and held per cell, rows held through the line buffer), posterized to four
-- colours by luma (K5 palette), with rows torn sideways in 8-line bands
-- (K3).  Over it two purple parabolas (centres K4 apart, curvature K2) whose
-- vertices sweep down the screen (K6); P12 is their width, from hairlines
-- to a purple flood.
--
--   K1  Blocks   mosaic cell size, 8 .. 263 px (square)
--   K2  Curve    parabola opening, tight .. wide
--   K3  Tear     sideways tearing in 8-line bands
--   K4  Spacing  distance between the two tubes
--   K5  Palette  Cathode / Phosphor / Amber / Ice
--   K6  Sweep    vertex sweep speed, bipolar (centre = still)
--   S7  Tubes    On / Off
--   S8  Mosaic   On / Off (plain picture)
--   S9  Tube     Purple / Cyan
--   S10 Poster   On / Off (full-colour mosaic)
--   S11 Source   Video / Osc
--   P12 TUBES    tube width: hairlines .. a purple flood
--
-- Virtual 1080-line geometry; dx^2 by second differences; generated avid;
-- output conditioning before the blanking gate.  Latency 16 clocks.
-- 16 EBR.  Colours: u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cathode of program_top is
    constant C_LAT : integer := 16;

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

    -- 4 palettes x 4 luma bands {y, u = Cr, v = Cb}
    type t_col is array (0 to 2) of integer;
    type t_pal is array (0 to 15) of t_col;
    constant C_PAL : t_pal := (
        (426, 887, 808), (580, 136, 216), (840, 584, 64), (584, 772, 212),     -- Cathode: magenta, green, yellow, orange
        (120, 300, 380), (380, 200, 300), (580, 136, 216), (820, 300, 200),    -- Phosphor: dark green .. bright
        (200, 600, 400), (450, 760, 260), (640, 740, 180), (840, 584, 64),     -- Amber: brown .. yellow
        (164, 440, 960), (400, 380, 760), (678, 64, 663), (940, 512, 512));    -- Ice: blue, steel, cyan, white
    constant C_PUR_Y : unsigned(9 downto 0) := to_unsigned(260, 10);
    constant C_PUR_U : unsigned(9 downto 0) := to_unsigned(700, 10);
    constant C_PUR_V : unsigned(9 downto 0) := to_unsigned(860, 10);

    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    signal s_src : std_logic := '0';
    signal s_ox, s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_k1c : unsigned(8 downto 0) := to_unsigned(32, 9);
    signal s_k2s : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_k3t : unsigned(7 downto 0) := (others => '0');
    signal s_k4s : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal s_j5 : unsigned(1 downto 0) := "00";
    signal s_live5 : std_logic := '0';
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_mcw : unsigned(8 downto 0) := to_unsigned(32, 9);
    signal s_mcwm1 : unsigned(8 downto 0) := to_unsigned(31, 9);
    signal s_mch : unsigned(9 downto 0) := to_unsigned(32, 10);
    signal s_mchq : unsigned(17 downto 0) := (others => '0');
    signal s_sh : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_tk : unsigned(7 downto 0) := (others => '0');
    signal s_sp : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal s_pal : unsigned(1 downto 0) := "00";
    signal s_tubes, s_mosaic, s_cyan, s_poster : std_logic := '0';
    signal s_yv : unsigned(11 downto 0) := to_unsigned(400, 12);
    signal s_wid : unsigned(9 downto 0) := to_unsigned(60, 10);
    signal s_d1, s_d2 : unsigned(12 downto 0) := (others => '0');   -- 2|dx| at line start (Q.1 units)
    signal s_sq1, s_sq2 : unsigned(23 downto 0) := (others => '0');
    signal s_inc01, s_inc02 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    -- serial unit
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(1 downto 0) := (others => '0');
    signal s_lfsr : unsigned(15 downto 0) := x"C0DE";

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_cy_n : unsigned(17 downto 0) := (others => '0');
    signal s_rowf_n, s_rowf : std_logic := '1';
    signal s_l8 : unsigned(2 downto 0) := (others => '0');
    signal s_tear_n : unsigned(10 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');
    signal s_yoff_n, s_yoff : signed(12 downto 0) := (others => '0');

    -- beam
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_mu, s_ms : unsigned(10 downto 0) := (others => '0');
    signal s_q1, s_q2 : unsigned(23 downto 0) := (others => '0');
    signal s_inc1, s_inc2b : signed(14 downto 0) := (others => '0');

    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_qa1, s_qb1 : unsigned(19 downto 0) := (others => '0');
    signal s_fa2, s_fb2 : unsigned(14 downto 0) := (others => '0');
    signal s_ta3, s_tb3 : signed(16 downto 0) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_tube : t_sl(4 to 6) := (others => '0');
    signal s_y4, s_u4, s_v4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_idx4 : unsigned(3 downto 0) := (others => '0');
    signal s_y5, s_u5, s_v5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py5, s_pu5, s_pv5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y8, s_y9, s_y10, s_y11, s_y12, s_y13x, s_yf13 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y14, s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14, s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

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

    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_ch : unsigned(10 downto 0);
        variable v_d : unsigned(13 downto 0);
        variable v_i : signed(15 downto 0);
        variable v_yv : signed(12 downto 0);
        variable v_wd : unsigned(10 downto 0);
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

            s_k1c <= resize(unsigned(registers_in(0)(9 downto 2)), 9) + 8;        -- K1 Blocks 8 .. 263
            s_k2s <= to_unsigned(7, 4) + resize(unsigned(registers_in(1)(9 downto 7)), 4);   -- K2 Curve shift 7 .. 14
            s_k3t <= unsigned(registers_in(2)(9 downto 2));                       -- K3 Tear
            s_k4s <= unsigned(registers_in(3));                                   -- K4 Spacing 0 .. 1023 (virtual px, half each side)
            v_k := unsigned(registers_in(4));                                    -- K5 Palette (4 labels)
            if v_k >= 4 then s_live5 <= '1'; end if;
            if s_live5 = '0' and v_k < 4 then
                s_j5 <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_j5 <= v_k(9 downto 8);
            end if;
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;          -- K6 Sweep, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mcw <= s_k1c;
                s_sh <= s_k2s;
                s_tk <= s_k3t;
                s_sp <= s_k4s;
                s_pal <= s_j5;
                s_tubes  <= not registers_in(6)(0);
                s_mosaic <= not registers_in(6)(1);
                s_cyan   <= registers_in(6)(2);
                s_poster <= not registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                -- vertex sweep: wraps over 0 .. 1535
                v_yv := signed(resize(s_yv, 13)) + resize(s_k6r, 13);
                if v_yv < 0 then v_yv := v_yv + 1536; elsif v_yv >= 1536 then v_yv := v_yv - 1536; end if;
                s_yv <= unsigned(v_yv(11 downto 0));
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_mcwm1 <= s_mcw - 1;
            case to_integer(s_xs) is
                when 4      => v_ch := shift_left(resize(s_mcw, 11), 1);
                when 3      => v_ch := resize(s_mcw, 11) + resize(s_mcw(8 downto 1), 11);
                when others => v_ch := resize(s_mcw, 11);
            end case;
            s_mch <= v_ch(9 downto 0);
            s_mchq <= s_mch & x"00";
            -- tube width 4 + p12 * 0.6 (virtual lines)
            v_wd := to_unsigned(4, 11) + resize(s_g(11 downto 3), 11) + resize(s_g(11 downto 5), 11);
            s_wid <= v_wd(9 downto 0);
            -- line-start 2|dx| for the two tubes: vw -+ 2 sp (clamped at 0)
            v_d := resize(s_vw, 14) - resize(s_sp(9 downto 1) & '0', 14);
            if v_d(13) = '1' then s_d1 <= (others => '0'); else s_d1 <= v_d(12 downto 0); end if;
            v_d := resize(s_vw, 14) + resize(s_sp(9 downto 1) & '0', 14);
            s_d2 <= v_d(12 downto 0);
            case to_integer(s_xs) is
                when 4      => s_inc01 <= resize(to_signed(16, 16) - signed(shift_left(resize(s_d1, 16), 3)), 15);
                               s_inc02 <= resize(to_signed(16, 16) - signed(shift_left(resize(s_d2, 16), 3)), 15);
                               s_inc2 <= to_unsigned(32, 6);
                when 3      => s_inc01 <= resize(to_signed(9, 16) - signed(shift_left(resize(s_d1, 16), 2) + shift_left(resize(s_d1, 16), 1)), 15);
                               s_inc02 <= resize(to_signed(9, 16) - signed(shift_left(resize(s_d2, 16), 2) + shift_left(resize(s_d2, 16), 1)), 15);
                               s_inc2 <= to_unsigned(18, 6);
                when others => s_inc01 <= resize(to_signed(4, 16) - signed(shift_left(resize(s_d1, 16), 2)), 15);
                               s_inc02 <= resize(to_signed(4, 16) - signed(shift_left(resize(s_d2, 16), 2)), 15);
                               s_inc2 <= to_unsigned(8, 6);
            end case;

            -- serial unit: jobs 0 / 1 = d1^2, d2^2
            if s_zc = 4 then
                s_ma <= resize(s_d1, 24); s_mb <= resize(s_d1, 12); s_mjob <= "00"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 24 then
                s_ma <= resize(s_d2, 24); s_mb <= resize(s_d2, 12); s_mjob <= "01"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                if s_mjob = "00" then s_sq1 <= s_macc; else s_sq2 <= s_macc; end if;
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

    p_src : process(clk)
        variable v_c : unsigned(18 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_tr : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_otu <= to_unsigned(320, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot) & '0', 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot), 10);
            s_otv <= to_unsigned(320, 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot), 10);

            s_lb_we <= data_in.avid and (s_rowf or not s_mosaic);
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_oty) & std_logic_vector(s_otu) & std_logic_vector(s_otv) & "00";
            else
                s_lb_wd <= data_in.y & data_in.u & data_in.v & "00";
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;

            s_fall1 <= '0';
            if data_in.avid = '0' and s_avq = '1' then s_fall1 <= '1'; end if;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_cy_n <= (others => '0');
                s_rowf_n <= '1';
                s_l8 <= (others => '0');
                s_tear_n <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                v_c := resize(s_cy_n, 19) + s_ys;
                if v_c >= s_mchq then
                    s_cy_n <= resize(v_c - s_mchq, 18);
                    s_rowf_n <= '1';
                else
                    s_cy_n <= v_c(17 downto 0);
                    s_rowf_n <= '0';
                end if;
                s_l8 <= s_l8 + 1;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                if s_l8 = 7 then
                    v_t := s_lfsr(7 downto 0) * s_tk;
                    v_tr := resize(v_t(15 downto 6), 11);
                    if v_tr >= s_wm1(10 downto 0) then s_tear_n <= s_wm1(10 downto 0); else s_tear_n <= v_tr; end if;
                end if;
            end if;
            s_yoff_n <= signed(resize(s_vy_n(19 downto 8), 13)) - signed(resize(s_yv, 13));
            if data_in.avid = '1' and s_avq = '0' then
                s_rowf <= s_rowf_n;
                s_yoff <= s_yoff_n;
                if s_tear_n = 0 then s_rst <= (others => '0'); else s_rst <= s_wm1(10 downto 0) + 1 - s_tear_n; end if;
            end if;
        end if;
    end process p_src;

    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
                s_mu <= (others => '0');
                s_ms <= s_rst;
                s_q1 <= s_sq1; s_inc1 <= s_inc01;
                s_q2 <= s_sq2; s_inc2b <= s_inc02;
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                if s_mu = resize(s_mcwm1, 11) then
                    s_mu <= (others => '0');
                    if s_rdp = s_wm1(10 downto 0) then s_ms <= (others => '0'); else s_ms <= s_rdp + 1; end if;
                else
                    s_mu <= s_mu + 1;
                end if;
                s_q1 <= unsigned(signed(s_q1) + resize(s_inc1, 24));
                s_inc1 <= s_inc1 + signed(resize(s_inc2, 15));
                s_q2 <= unsigned(signed(s_q2) + resize(s_inc2b, 24));
                s_inc2b <= s_inc2b + signed(resize(s_inc2, 15));
            end if;
        end if;
    end process p_beam;

    p_pix : process(clk)
        variable v_q : unsigned(19 downto 0);
        variable v_a, v_b : signed(16 downto 0);
        variable v_pi : integer range 0 to 15;
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: read (mosaic cell start, or every pixel); dx^2 copies
            if s_mosaic = '1' then s_lb_ra <= s_ms; else s_lb_ra <= s_rdp; end if;
            s_qa1 <= s_q1(23 downto 4);
            s_qb1 <= s_q2(23 downto 4);

            -- 2: curvature shift 7 .. 14
            v_q := s_qa1;
            case to_integer(s_sh) is
                when 7      => s_fa2 <= v_q(19 downto 5);
                when 8      => s_fa2 <= '0' & v_q(19 downto 6);
                when 9      => s_fa2 <= "00" & v_q(19 downto 7);
                when 10     => s_fa2 <= "000" & v_q(19 downto 8);
                when 11     => s_fa2 <= "0000" & v_q(19 downto 9);
                when 12     => s_fa2 <= "00000" & v_q(19 downto 10);
                when 13     => s_fa2 <= "000000" & v_q(19 downto 11);
                when others => s_fa2 <= "0000000" & v_q(19 downto 12);
            end case;
            v_q := s_qb1;
            case to_integer(s_sh) is
                when 7      => s_fb2 <= v_q(19 downto 5);
                when 8      => s_fb2 <= '0' & v_q(19 downto 6);
                when 9      => s_fb2 <= "00" & v_q(19 downto 7);
                when 10     => s_fb2 <= "000" & v_q(19 downto 8);
                when 11     => s_fb2 <= "0000" & v_q(19 downto 9);
                when 12     => s_fb2 <= "00000" & v_q(19 downto 10);
                when 13     => s_fb2 <= "000000" & v_q(19 downto 11);
                when others => s_fb2 <= "0000000" & v_q(19 downto 12);
            end case;

            -- 3: t = (y - yv) - dx^2 / 2^sh; video fabric copy
            s_ta3 <= resize(s_yoff, 17) - signed(resize(s_fa2, 17));
            s_tb3 <= resize(s_yoff, 17) - signed(resize(s_fb2, 17));
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 4: tube flags (|t| < width); posterize index
            v_a := s_ta3; if v_a < 0 then v_a := -v_a; end if;
            v_b := s_tb3; if v_b < 0 then v_b := -v_b; end if;
            if s_tubes = '1' and (v_a < signed(resize(s_wid, 17)) or v_b < signed(resize(s_wid, 17))) then s_tube(4) <= '1'; else s_tube(4) <= '0'; end if;
            s_idx4 <= s_pal & s_y3(9 downto 8);
            s_y4 <= s_y3; s_u4 <= s_u3; s_v4 <= s_v3;

            -- 5: palette
            v_pi := to_integer(s_idx4);
            s_py5 <= to_unsigned(C_PAL(v_pi)(0), 10);
            s_pu5 <= to_unsigned(C_PAL(v_pi)(1), 10);
            s_pv5 <= to_unsigned(C_PAL(v_pi)(2), 10);
            s_y5 <= s_y4; s_u5 <= s_u4; s_v5 <= s_v4;

            -- 6: posterized or full colour
            if s_poster = '1' then
                s_y6 <= s_py5; s_u6 <= s_pu5; s_v6 <= s_pv5;
            else
                s_y6 <= s_y5; s_u6 <= s_u5; s_v6 <= s_v5;
            end if;

            -- 7: tubes over
            if s_tube(6) = '1' then
                if s_cyan = '1' then
                    s_y7 <= to_unsigned(678, 10); s_u7 <= to_unsigned(64, 10); s_v7 <= to_unsigned(663, 10);
                else
                    s_y7 <= C_PUR_Y; s_u7 <= C_PUR_U; s_v7 <= C_PUR_V;
                end if;
            else
                s_y7 <= s_y6; s_u7 <= s_u6; s_v7 <= s_v6;
            end if;

            -- 8..13: output conditioning (centre at stage 12 -> 13)
            s_csu(0) <= s_u7; s_csv(0) <= s_v7;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u7 - s_csu(7);
            s_boxv <= s_boxv + s_v7 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u13 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v13 <= v_sc(13 downto 4);
            s_y8 <= s_y7; s_y9 <= s_y8; s_y10 <= s_y9; s_y11 <= s_y10;
            s_y12 <= s_y11; s_y13x <= s_y12;
            v_sy := resize(s_y11, 12) + shift_left(resize(s_y12, 12), 1) + resize(s_y13x, 12);
            s_yf13 <= v_sy(11 downto 2);
            if s_yf13 > 64 then v_dl := s_yf13 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(12) = '1' then
                s_y14 <= s_yf13 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;

            for i in 5 to 6 loop s_tube(i) <= s_tube(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cathode;
