-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Terrace v0.1.0 -- Heirloom #11 (2026-10-20), from WIN_20230305_23_38_39.
--
-- The photo: three panels of a cell grid in which every column drops the
-- row pattern by one cell, so an 8-colour sequence (yellow / magenta / cyan /
-- white / light gray / gray / dark gray / black) runs as diagonal staircases.
--
-- The program: a grid of cells (K1 wide, K2 tall); each cell's colour is
-- palette[(row + column x slope + scroll) mod 8], with P12 the slope -- 0
-- gives horizontal bands, higher values steeper staircases (continuous,
-- a Q.8 phase per column).  The screen is 1 / 2 / 4 panels (K5), each
-- panel dropped one cell.  The video (or the oscillator), sampled as a
-- mosaic on the same cells, is mixed into the cell colour by K6.
--
--   K1  Width    cell width, 16 .. 271 px
--   K2  Height   cell height, 8 .. 263 lines (1080-line canvas)
--   K3  Palette  CMYK / Warm / Cool / Neon
--   K4  Scroll   pattern scroll, bipolar (centre = still)
--   K5  Panels   1 / 2 / 4
--   K6  Mix      the picture's cell colour mixed into the pattern, 0 .. 100 %
--   S7  Swap     palette order swapped (halves exchanged)
--   S8  Grid     dark grid lines
--   S9  Direction staircases descend right / left
--   S10 Tint     Colour (the picture's full colour) / Luma (its luma on the pattern's hue)
--   S11 Source   Video / Osc
--   P12 SLOPE    row step per column: bands .. steep staircases
--
-- Virtual 1080-line geometry (cell height), real-pixel cell widths (counts);
-- generated avid; output conditioning before the blanking gate.  Latency 16
-- clocks.  16 EBR.  Colours: u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture terrace of program_top is
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
    function f_clamp(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 960 then return to_unsigned(960, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- 4 palettes x 8 cells {y, u = Cr, v = Cb}
    type t_col is array (0 to 2) of integer;
    type t_pal is array (0 to 31) of t_col;
    constant C_PAL : t_pal := (
        (840, 584, 64), (426, 887, 808), (678, 64, 663), (940, 512, 512), (700, 512, 512), (480, 512, 512), (260, 512, 512), (64, 512, 512),      -- CMYK
        (840, 584, 64), (584, 772, 212), (328, 960, 360), (752, 612, 512), (940, 512, 512), (426, 887, 808), (260, 512, 512), (64, 512, 512),     -- Warm
        (678, 64, 663), (164, 440, 960), (349, 756, 852), (192, 608, 696), (747, 516, 594), (940, 512, 512), (580, 136, 216), (64, 512, 512),     -- Cool
        (64, 512, 512), (426, 887, 808), (64, 512, 512), (678, 64, 663), (64, 512, 512), (840, 584, 64), (64, 512, 512), (580, 136, 216));        -- Neon

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
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_k1c : unsigned(8 downto 0) := to_unsigned(80, 9);
    signal s_k2h : unsigned(8 downto 0) := to_unsigned(40, 9);
    signal s_j3 : unsigned(1 downto 0) := "00";
    signal s_live3 : std_logic := '0';
    signal s_k4r : signed(9 downto 0) := (others => '0');
    signal s_j5 : unsigned(1 downto 0) := "01";
    signal s_live5 : std_logic := '0';
    signal s_k6m : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_cw : unsigned(8 downto 0) := to_unsigned(80, 9);
    signal s_cwm1 : unsigned(8 downto 0) := to_unsigned(79, 9);
    signal s_chq : unsigned(16 downto 0) := (others => '0');
    signal s_pal : unsigned(1 downto 0) := "00";
    signal s_psh : unsigned(1 downto 0) := "01";
    signal s_pw : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_pwm1 : unsigned(10 downto 0) := to_unsigned(959, 11);
    signal s_mix : unsigned(7 downto 0) := (others => '0');
    signal s_slope : unsigned(8 downto 0) := (others => '0');
    signal s_swap, s_grid, s_dir, s_luma : std_logic := '0';
    signal s_t : unsigned(10 downto 0) := (others => '0');           -- scroll phase (Q.8 rows)

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_cy_n : unsigned(16 downto 0) := (others => '0');
    signal s_row_n : unsigned(7 downto 0) := (others => '0');
    signal s_rowf_n, s_rowf : std_logic := '1';
    signal s_rp : unsigned(10 downto 0) := (others => '0');         -- row phase (row << 8 + scroll)

    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_cu, s_cs : unsigned(10 downto 0) := (others => '0');
    signal s_pu : unsigned(10 downto 0) := (others => '0');
    signal s_pn : unsigned(1 downto 0) := (others => '0');
    signal s_cacc : unsigned(10 downto 0) := (others => '0');

    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_idx1 : unsigned(4 downto 0) := (others => '0');
    signal s_gr : t_sl(1 to 6) := (others => '0');
    signal s_py2, s_pu2, s_pv2 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py3, s_pu3, s_pv3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py4, s_pu4, s_pv4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_dy4, s_du4, s_dv4 : signed(11 downto 0) := (others => '0');
    signal s_py5, s_pu5, s_pv5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_my5, s_mu5, s_mv5 : signed(20 downto 0) := (others => '0');
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
        variable v_ch : unsigned(10 downto 0);
        variable v_t : signed(11 downto 0);
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
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            s_k1c <= resize(unsigned(registers_in(0)(9 downto 2)), 9) + 16;     -- K1 Width 16 .. 271
            s_k2h <= resize(unsigned(registers_in(1)(9 downto 2)), 9) + 8;      -- K2 Height 8 .. 263
            v_k := unsigned(registers_in(2));                                  -- K3 Palette (4 labels)
            if v_k >= 4 then s_live3 <= '1'; end if;
            if s_live3 = '0' and v_k < 4 then
                s_j3 <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_j3 <= v_k(9 downto 8);
            end if;
            v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;        -- K4 Scroll, deadband
            v_s := shift_right(v_s, 3);
            if v_s > -1 and v_s < 1 then s_k4r <= (others => '0'); else s_k4r <= v_s(9 downto 0); end if;
            v_k := unsigned(registers_in(4));                                  -- K5 Panels (3 labels)
            if v_k >= 3 then s_live5 <= '1'; end if;
            if s_live5 = '0' and v_k < 3 then
                s_j5 <= v_k(1 downto 0);
            elsif v_k < 341 then s_j5 <= "00";
            elsif v_k < 682 then s_j5 <= "01";
            else s_j5 <= "10"; end if;
            s_k6m <= unsigned(registers_in(5)(9 downto 2));                     -- K6 Mix
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_cw <= s_k1c;
                s_pal <= s_j3;
                s_psh <= s_j5;
                s_mix <= s_k6m;
                s_swap <= registers_in(6)(0);
                s_grid <= registers_in(6)(1);
                s_dir  <= registers_in(6)(2);
                s_luma <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                v_t := signed(resize(s_t, 12)) + resize(s_k4r, 12);
                s_t <= unsigned(v_t(10 downto 0));
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            s_cwm1 <= s_cw - 1;
            s_chq <= s_k2h & x"00";
            case to_integer(s_psh) is
                when 0      => s_pw <= s_w(10 downto 0);
                when 1      => s_pw <= '0' & s_w(10 downto 1);
                when others => s_pw <= "00" & s_w(10 downto 2);
            end case;
            s_pwm1 <= s_pw - 1;
            s_slope <= s_g(11 downto 3);

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
        variable v_c : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_otu <= to_unsigned(320, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot) & '0', 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot), 10);
            s_otv <= to_unsigned(320, 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot), 10);

            -- the mosaic row is written on its first line only
            s_lb_we <= data_in.avid and s_rowf;
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_oty) & std_logic_vector(s_otu) & std_logic_vector(s_otv) & "00";
            else
                s_lb_wd <= data_in.y & data_in.u & data_in.v & "00";
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;

            if s_zone = '1' then
                s_cy_n <= (others => '0');
                s_row_n <= (others => '0');
                s_rowf_n <= '1';
            elsif data_in.avid = '0' and s_avq = '1' then
                v_c := resize(s_cy_n, 18) + s_ys;
                if v_c >= s_chq then
                    s_cy_n <= resize(v_c - s_chq, 17);
                    s_row_n <= s_row_n + 1;
                    s_rowf_n <= '1';
                else
                    s_cy_n <= v_c(16 downto 0);
                    s_rowf_n <= '0';
                end if;
            end if;
            if data_in.avid = '1' and s_avq = '0' then
                s_rowf <= s_rowf_n;
                s_rp <= shift_left(resize(s_row_n(2 downto 0), 11), 8) + s_t;
            end if;
        end if;
    end process p_src;

    -- beam: column, cell counters (S&H start), panel index, slope accumulator
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_col <= (others => '0');
                s_cu <= (others => '0');
                s_cs <= (others => '0');
                s_pu <= (others => '0');
                s_pn <= (others => '0');
                s_cacc <= (others => '0');
            elsif g_avid = '1' then
                s_col <= s_col + 1;
                if s_cu = resize(s_cwm1, 11) then
                    s_cu <= (others => '0');
                    s_cs <= s_col + 1;
                    if s_dir = '1' then s_cacc <= s_cacc - resize(s_slope, 11); else s_cacc <= s_cacc + resize(s_slope, 11); end if;
                else
                    s_cu <= s_cu + 1;
                end if;
                if s_pu = s_pwm1 then
                    s_pu <= (others => '0');
                    s_pn <= s_pn + 1;
                else
                    s_pu <= s_pu + 1;
                end if;
            end if;
        end if;
    end process p_beam;

    p_pix : process(clk)
        variable v_ph : unsigned(10 downto 0);
        variable v_pi : integer range 0 to 31;
        variable v_y, v_u, v_v : signed(12 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: mosaic read; cell colour index; grid flag
            s_lb_ra <= s_cs;
            v_ph := s_rp + s_cacc + shift_left(resize(s_pn, 11), 8);
            s_idx1 <= s_pal & (v_ph(10 downto 8) xor (s_swap & "00"));
            if s_grid = '1' and (s_cu < 2 or s_rowf = '1') then s_gr(1) <= '1'; else s_gr(1) <= '0'; end if;

            -- 2: palette (constant table)
            v_pi := to_integer(s_idx1);
            s_py2 <= to_unsigned(C_PAL(v_pi)(0), 10);
            s_pu2 <= to_unsigned(C_PAL(v_pi)(1), 10);
            s_pv2 <= to_unsigned(C_PAL(v_pi)(2), 10);

            -- 3: video fabric copy; palette copy
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));
            s_py3 <= s_py2; s_pu3 <= s_pu2; s_pv3 <= s_pv2;

            -- 4: differences picture - pattern
            s_dy4 <= signed(resize(s_y3, 12)) - signed(resize(s_py3, 12));
            if s_luma = '1' then
                s_du4 <= (others => '0'); s_dv4 <= (others => '0');
            else
                s_du4 <= signed(resize(s_u3, 12)) - signed(resize(s_pu3, 12));
                s_dv4 <= signed(resize(s_v3, 12)) - signed(resize(s_pv3, 12));
            end if;
            s_py4 <= s_py3; s_pu4 <= s_pu3; s_pv4 <= s_pv3;

            -- 5: mix products
            s_my5 <= s_dy4 * signed(resize(s_mix, 9));
            s_mu5 <= s_du4 * signed(resize(s_mix, 9));
            s_mv5 <= s_dv4 * signed(resize(s_mix, 9));
            s_py5 <= s_py4; s_pu5 <= s_pu4; s_pv5 <= s_pv4;

            -- 6: mixed cell colour
            v_y := signed(resize(s_py5, 13)) + resize(shift_right(s_my5, 8), 13);
            v_u := signed(resize(s_pu5, 13)) + resize(shift_right(s_mu5, 8), 13);
            v_v := signed(resize(s_pv5, 13)) + resize(shift_right(s_mv5, 8), 13);
            s_y6 <= f_clamp(v_y); s_u6 <= f_clamp(v_u); s_v6 <= f_clamp(v_v);

            -- 7: grid lines
            if s_gr(6) = '1' then
                s_y7 <= to_unsigned(64, 10); s_u7 <= to_unsigned(512, 10); s_v7 <= to_unsigned(512, 10);
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
            -- 14: luma toward black; blanking gate
            if s_yf13 > 64 then v_dl := s_yf13 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(12) = '1' then
                s_y14 <= s_yf13 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;

            for i in 2 to 6 loop s_gr(i) <= s_gr(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture terrace;
