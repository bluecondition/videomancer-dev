-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Spearhead v0.1.0 -- Heirloom #5 (2026-10-20), from WIN_20230103_01_29_30.
--
-- The photo: two large down-pointing spearheads, each a stack of nested
-- outlines (black / magenta / yellow / blue / white), hanging over a coarse
-- pastel mosaic of diagonal colour bands; two copies across the screen.
--
-- The program: the screen is split into 1 / 2 / 4 cells; in each, a
-- parabolic spearhead f = dx^2 / 2^K2 + (y - y0) < 0 is cut into nested
-- rings (K3) painted from an 8-colour outline palette.  Behind it the video
-- (or the oscillator) as a coarse mosaic -- sampled and held per cell
-- across, rows held through the line buffer -- washed toward pastel (K5)
-- with optional diagonal pastel bands (S10).  P12 plunges the spearheads
-- down the screen until the rings fill it; K6 makes them bob.
--
--   K1  Cells    spearheads across: 1 / 2 / 4
--   K2  Width    spearhead width (parabola opening), narrow .. wide
--   K3  Rings    outline ring density
--   K4  Mosaic   mosaic cell size, 8 .. 263 px (square)
--   K5  Wash     pastel wash of the backdrop, 0 .. 100 %
--   K6  Bob      bobbing amplitude of the tips (triangle LFO)
--   S7  Point    Down / Up
--   S8  Mosaic   On / Off (plain picture behind)
--   S9  Palette  outline palette A (photo) / B
--   S10 Bands    diagonal pastel bands tint the backdrop
--   S11 Source   Video / Osc
--   P12 PLUNGE   tip position: small tips at the top .. giant rings past the bottom
--
-- Virtual 1080-line geometry; generated avid; output conditioning before the
-- blanking gate.  Latency 16 clocks.  16 EBR.  Colours: u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture spearhead of program_top is
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

    -- outline palettes (8 rings each): A = photo (black, magenta, yellow,
    -- blue, white, pink, black, yellow); B = cool (navy, cyan, white, violet,
    -- green, black, lavender, blue)
    type t_col is array (0 to 2) of integer;
    type t_pal is array (0 to 15) of t_col;
    constant C_PAL : t_pal := (
        (64, 512, 512), (426, 887, 808), (840, 584, 64), (164, 440, 960), (940, 512, 512), (752, 612, 512), (64, 512, 512), (840, 584, 64),
        (120, 560, 700), (678, 64, 663), (940, 512, 512), (349, 756, 852), (580, 136, 216), (64, 512, 512), (747, 516, 594), (164, 440, 960));

    -- line buffer {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- oscillator source
    signal s_src : std_logic := '0';
    signal s_ox, s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- timing / controls
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(5 downto 0) := (others => '0');
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_j1 : unsigned(1 downto 0) := "01";
    signal s_live1 : std_logic := '0';
    signal s_k2s : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_k3r : unsigned(7 downto 0) := to_unsigned(40, 8);
    signal s_k4c : unsigned(8 downto 0) := to_unsigned(32, 9);
    signal s_k5w : unsigned(7 downto 0) := (others => '0');
    signal s_k6b : unsigned(8 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_ksh : unsigned(1 downto 0) := "01";
    signal s_sh : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_rk : unsigned(7 downto 0) := to_unsigned(40, 8);
    signal s_cwpx : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_cwpxm1 : unsigned(10 downto 0) := to_unsigned(959, 11);
    signal s_cwv : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_mcw : unsigned(8 downto 0) := to_unsigned(32, 9);
    signal s_mcwm1 : unsigned(8 downto 0) := to_unsigned(31, 9);
    signal s_mch : unsigned(9 downto 0) := to_unsigned(32, 10);
    signal s_mchq : unsigned(17 downto 0) := (others => '0');
    signal s_wash : unsigned(7 downto 0) := (others => '0');
    signal s_bob : unsigned(8 downto 0) := (others => '0');
    signal s_up, s_mosaic, s_palb, s_bands : std_logic := '0';
    signal s_y0 : signed(11 downto 0) := to_signed(400, 12);
    signal s_lfo : unsigned(7 downto 0) := (others => '0');
    signal s_bobv : signed(11 downto 0) := (others => '0');
    signal s_cx2sq : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    -- serial unit
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(1 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_cy_n : unsigned(17 downto 0) := (others => '0');
    signal s_rowf_n, s_rowf : std_logic := '1';
    signal s_yoff_n, s_yoff : signed(12 downto 0) := (others => '0');
    signal s_vyl : unsigned(7 downto 0) := (others => '0');

    -- beam
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_cu, s_cs : unsigned(10 downto 0) := (others => '0');
    signal s_mu, s_ms : unsigned(10 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');
    signal s_vxq : unsigned(12 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_q1 : unsigned(19 downto 0) := (others => '0');
    signal s_dg1 : unsigned(7 downto 0) := (others => '0');
    signal s_fq2 : unsigned(14 downto 0) := (others => '0');
    signal s_dg : t_u10(2 to 7) := (others => (others => '0'));
    signal s_f3 : signed(16 downto 0) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_nf4 : unsigned(11 downto 0) := (others => '0');
    signal s_in : t_sl(4 to 7) := (others => '0');
    signal s_y4, s_u4, s_v4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_dy4, s_du4, s_dv4 : signed(11 downto 0) := (others => '0');
    signal s_ring5 : unsigned(19 downto 0) := (others => '0');
    signal s_y5, s_u5, s_v5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py5, s_pu5, s_pv5 : signed(20 downto 0) := (others => '0');
    signal s_idx6 : unsigned(3 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py7, s_pu7, s_pv7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y9, s_y10, s_y11, s_y12, s_y13, s_y14x, s_yf14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

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

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_i : signed(15 downto 0);
        variable v_ch : unsigned(10 downto 0);
        variable v_y0 : signed(12 downto 0);
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
            elsif s_zc /= 63 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- K1 Cells (3 labels: 1 / 2 / 4; the firmware loads the default as the index)
            v_k := unsigned(registers_in(0));
            if v_k >= 3 then s_live1 <= '1'; end if;
            if s_live1 = '0' and v_k < 3 then
                s_j1 <= v_k(1 downto 0);
            elsif v_k < 341 then s_j1 <= "00";
            elsif v_k < 682 then s_j1 <= "01";
            else s_j1 <= "10"; end if;
            s_k2s <= to_unsigned(5, 4) + resize(unsigned(registers_in(1)(9 downto 7)), 4);   -- K2 Width: shift 5 .. 12
            s_k3r <= resize(unsigned(registers_in(2)(9 downto 3)), 8) + 16;                  -- K3 Rings 16 .. 143
            s_k4c <= resize(unsigned(registers_in(3)(9 downto 2)), 9) + 8;                   -- K4 Mosaic 8 .. 263 px
            s_k5w <= unsigned(registers_in(4)(9 downto 2));                                  -- K5 Wash
            s_k6b <= unsigned(registers_in(5)(9 downto 1));                                  -- K6 Bob 0 .. 511
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_ksh <= s_j1;
                s_sh <= s_k2s;
                s_rk <= s_k3r;
                s_mcw <= s_k4c;
                s_wash <= s_k5w;
                s_bob <= s_k6b;
                s_up     <= registers_in(6)(0);
                s_mosaic <= not registers_in(6)(1);
                s_palb   <= registers_in(6)(2);
                s_bands  <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_lfo <= s_lfo + 2;
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
            case to_integer(s_ksh) is
                when 0      => s_cwpx <= s_w(10 downto 0); s_cwv <= s_vw;
                when 1      => s_cwpx <= '0' & s_w(10 downto 1); s_cwv <= '0' & s_vw(11 downto 1);
                when others => s_cwpx <= "00" & s_w(10 downto 2); s_cwv <= "00" & s_vw(11 downto 2);
            end case;
            s_cwpxm1 <= s_cwpx - 1;
            s_mcwm1 <= s_mcw - 1;
            case to_integer(s_xs) is
                when 4      => v_ch := shift_left(resize(s_mcw, 11), 1);
                when 3      => v_ch := resize(s_mcw, 11) + resize(s_mcw(8 downto 1), 11);
                when others => v_ch := resize(s_mcw, 11);
            end case;
            s_mch <= v_ch(9 downto 0);
            s_mchq <= s_mch & x"00";
            -- dx^2 second differences (dx in Q.1: start at -cwv)
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_cwv, 16), 3)); s_inc2 <= to_unsigned(32, 6);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_cwv, 16), 2) + shift_left(resize(s_cwv, 16), 1)); s_inc2 <= to_unsigned(18, 6);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_cwv, 16), 2)); s_inc2 <= to_unsigned(8, 6);
            end case;
            s_inc0 <= v_i(14 downto 0);
            -- tip: 150 + 1.2 p12 + bob (triangle LFO scaled by K6)
            s_bobv <= signed(resize(s_lfo, 12)) - 128;
            v_y0 := to_signed(150, 13) + signed(resize(s_g(11 downto 2), 13)) + signed(resize(s_g(11 downto 5), 13))
                    + signed(resize(s_g(11 downto 6), 13)) + resize(s_bobv, 13);
            s_y0 <= v_y0(11 downto 0);

            -- serial unit: job 0 (cwv)^2, job 1 bob x lfo (unused slot kept simple)
            if s_zc = 4 then
                s_ma <= resize(s_cwv, 24); s_mb <= s_cwv; s_mjob <= "00"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                s_cx2sq <= s_macc;
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
    -- Source side: line-buffer write (only on the first line of a mosaic row
    -- when the mosaic is on); per-line values
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(18 downto 0);
        variable v_yl : signed(12 downto 0);
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

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_cy_n <= (others => '0');
                s_rowf_n <= '1';
            elsif data_in.avid = '0' and s_avq = '1' then
                s_vy_n <= s_vy_n + s_ys;
                v_c := resize(s_cy_n, 19) + s_ys;
                if v_c >= s_mchq then
                    s_cy_n <= resize(v_c - s_mchq, 18);
                    s_rowf_n <= '1';
                else
                    s_cy_n <= v_c(17 downto 0);
                    s_rowf_n <= '0';
                end if;
            end if;
            -- f's per-line term: y - y0 (Down) or (1080 - y) - y0 (Up)
            if s_up = '1' then
                v_yl := to_signed(1080, 13) - signed(resize(s_vy_n(19 downto 8), 13)) - resize(s_y0, 13);
            else
                v_yl := signed(resize(s_vy_n(19 downto 8), 13)) - resize(s_y0, 13);
            end if;
            s_yoff_n <= v_yl;
            if data_in.avid = '1' and s_avq = '0' then
                s_rowf <= s_rowf_n;
                s_yoff <= s_yoff_n;
                s_vyl <= s_vy_n(11 downto 4);
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: column, spearhead cell (dx^2 by second differences), mosaic cell
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_col <= (others => '0');
                s_cu <= (others => '0');
                s_cs <= (others => '0');
                s_mu <= (others => '0');
                s_ms <= (others => '0');
                s_q <= s_cx2sq;
                s_inc <= s_inc0;
                s_vxq <= (others => '0');
            elsif g_avid = '1' then
                s_col <= s_col + 1;
                s_vxq <= s_vxq + s_xs;
                if s_cu = s_cwpxm1 then
                    s_cu <= (others => '0');
                    s_q <= s_cx2sq;
                    s_inc <= s_inc0;
                else
                    s_cu <= s_cu + 1;
                    s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                    s_inc <= s_inc + signed(resize(s_inc2, 15));
                end if;
                if s_mu = resize(s_mcwm1, 11) then
                    s_mu <= (others => '0');
                    s_ms <= s_col + 1;
                else
                    s_mu <= s_mu + 1;
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_q : unsigned(19 downto 0);
        variable v_f : signed(16 downto 0);
        variable v_n : signed(16 downto 0);
        variable v_y, v_u, v_v : signed(12 downto 0);
        variable v_t : signed(8 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_pi : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            -- 1: mosaic read (held at the cell start) or plain; dx^2 copy; diagonal phase
            if s_mosaic = '1' then s_lb_ra <= s_ms; else s_lb_ra <= s_col; end if;
            s_q1 <= s_q(23 downto 4);                               -- dx^2 (integer part / 4)
            s_dg1 <= s_vxq(11 downto 4) + s_vyl;

            -- 2: width shift (5 .. 12 on dx^2); buffer output register
            v_q := s_q1;
            case to_integer(s_sh) is
                when 5      => s_fq2 <= v_q(17 downto 3);
                when 6      => s_fq2 <= v_q(18 downto 4);
                when 7      => s_fq2 <= v_q(19 downto 5);
                when 8      => s_fq2 <= '0' & v_q(19 downto 6);
                when 9      => s_fq2 <= "00" & v_q(19 downto 7);
                when 10     => s_fq2 <= "000" & v_q(19 downto 8);
                when 11     => s_fq2 <= "0000" & v_q(19 downto 9);
                when others => s_fq2 <= "00000" & v_q(19 downto 10);
            end case;
            s_dg(2) <= resize(f_tri8(s_dg1), 10);

            -- 3: f = dx^2 / 2^sh + (y - y0); video fabric copy
            s_f3 <= signed(resize(s_fq2, 17)) + resize(s_yoff, 17);
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 4: inside / -f clamped; wash differences
            v_f := s_f3;
            if v_f < 0 then
                s_in(4) <= '1';
                v_n := -v_f;
                if v_n > 4095 then s_nf4 <= (others => '1'); else s_nf4 <= unsigned(v_n(11 downto 0)); end if;
            else
                s_in(4) <= '0';
                s_nf4 <= (others => '0');
            end if;
            s_y4 <= s_y3; s_u4 <= s_u3; s_v4 <= s_v3;
            s_dy4 <= to_signed(820, 12) - signed(resize(s_y3, 12));
            s_du4 <= to_signed(512, 12) - signed(resize(s_u3, 12));
            s_dv4 <= to_signed(512, 12) - signed(resize(s_v3, 12));

            -- 5: ring product; wash products
            s_ring5 <= s_nf4 * s_rk;
            s_y5 <= s_y4; s_u5 <= s_u4; s_v5 <= s_v4;
            s_py5 <= s_dy4 * signed(resize(s_wash, 9));
            s_pu5 <= s_du4 * signed(resize(s_wash, 9));
            s_pv5 <= s_dv4 * signed(resize(s_wash, 9));

            -- 6: ring index; washed backdrop
            s_idx6 <= s_palb & s_ring5(13 downto 11);
            v_y := signed(resize(s_y5, 13)) + resize(shift_right(s_py5, 8), 13);
            v_u := signed(resize(s_u5, 13)) + resize(shift_right(s_pu5, 8), 13);
            v_v := signed(resize(s_v5, 13)) + resize(shift_right(s_pv5, 8), 13);
            s_y6 <= f_clamp(v_y); s_u6 <= f_clamp(v_u); s_v6 <= f_clamp(v_v);

            -- 7: palette; diagonal band tint on the backdrop
            v_pi := to_integer(s_idx6);
            s_py7 <= to_unsigned(C_PAL(v_pi)(0), 10);
            s_pu7 <= to_unsigned(C_PAL(v_pi)(1), 10);
            s_pv7 <= to_unsigned(C_PAL(v_pi)(2), 10);
            v_t := signed(resize(s_dg(6), 9)) - 64;
            s_y7 <= s_y6;
            if s_bands = '1' then
                s_u7 <= f_clamp(signed(resize(s_u6, 13)) + resize(v_t, 13));
                s_v7 <= f_clamp(signed(resize(s_v6, 13)) - resize(v_t, 13));
            else
                s_u7 <= s_u6; s_v7 <= s_v6;
            end if;

            -- 8: compose
            if s_in(7) = '1' then
                s_y8 <= s_py7; s_u8 <= s_pu7; s_v8 <= s_pv7;
            else
                s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
            end if;

            -- 9..14: output conditioning (centre at stage 13 -> 14)
            s_csu(0) <= s_u8; s_csv(0) <= s_v8;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u8 - s_csu(7);
            s_boxv <= s_boxv + s_v8 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u14 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v14 <= v_sc(13 downto 4);
            s_y9 <= s_y8; s_y10 <= s_y9; s_y11 <= s_y10; s_y12 <= s_y11;
            s_y13 <= s_y12; s_y14x <= s_y13;
            v_sy := resize(s_y12, 12) + shift_left(resize(s_y13, 12), 1) + resize(s_y14x, 12);
            s_yf14 <= v_sy(11 downto 2);
            -- 15: luma toward black; blanking gate
            if s_yf14 > 64 then v_dl := s_yf14 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(13) = '1' then
                s_y15 <= s_yf14 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u15 <= s_u14; s_v15 <= s_v14;
            else
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            end if;
            -- 16
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;

            -- delays
            for i in 3 to 7 loop s_dg(i) <= s_dg(i - 1); end loop;
            for i in 5 to 7 loop s_in(i) <= s_in(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture spearhead;
