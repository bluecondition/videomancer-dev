-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Beacon v0.1.0 -- Heirloom #26 (2026-10-20), from WIN_20230424_23_53_49.
--
-- The photo: black and ultramarine blocks, a white flame in the centre -- a
-- soft glowing head above a bright cone -- and rows of horizontal dashes
-- tearing the blocks left and right.
--
-- The program: a vertical cone (|x| <= k (y - tip)) with a soft glow blob
-- above its tip (the squared distance to the glow centre), white or warm,
-- over a field of blue / black blocks (a hash of the block grid, the
-- source's luma lighting the blue) with rows of dashes where the block grid
-- is shifted sideways.  P12 raises the flame from a stub at the bottom to a
-- spire reaching the top.
--
--   K1  Blue     level of the blue blocks
--   K2  Blocks   block size 16 / 32 / 64 / 128
--   K3  Dashes   how many rows tear into dashes
--   K4  Cone     cone width
--   K5  Glow     glow radius
--   K6  Flicker  random per-field glow flicker
--   S7  Blocks   On / Off (plain black field)
--   S8  Dashes   On / Off
--   S9  Flame    White / Warm
--   S10 Cone     Solid / Filled with the source
--   S11 Source   Video / Osc
--   P12 FLAME    flame height: a stub .. a spire to the top of the screen
--
-- Virtual 1080-line geometry; dx^2 by second differences, dy^2 and the
-- per-frame squares on the serial unit.  Generated avid; output conditioning
-- before the gate.  Latency 20 clocks.  EBR: 16 line buffer.  u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture beacon of program_top is
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
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_bluk : unsigned(9 downto 0) := (others => '0');
    signal s_bsk : unsigned(1 downto 0) := "01";
    signal s_dshk : unsigned(7 downto 0) := (others => '0');
    signal s_conk : unsigned(7 downto 0) := (others => '0');
    signal s_rgk : unsigned(8 downto 0) := (others => '0');
    signal s_flk : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_blu : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal s_bs : unsigned(1 downto 0) := "01";
    signal s_dsh : unsigned(7 downto 0) := (others => '0');
    signal s_con : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_rg : unsigned(8 downto 0) := to_unsigned(120, 9);
    signal s_fl : unsigned(7 downto 0) := (others => '0');
    signal s_flk9 : unsigned(8 downto 0) := to_unsigned(256, 9);     -- this field's flicker gain
    signal s_y0 : unsigned(10 downto 0) := to_unsigned(700, 11);     -- cone tip row
    signal s_yg : signed(11 downto 0) := to_signed(580, 12);         -- glow centre row
    signal s_rg2 : unsigned(23 downto 0) := (others => '0');
    signal s_gsh : unsigned(1 downto 0) := "10";
    signal s_cx2sq : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_blkon, s_dshon, s_warm, s_fill : std_logic := '0';
    signal s_seed : unsigned(7 downto 0) := (others => '0');
    -- serial multiplier
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(1 downto 0) := (others => '0');
    signal s_lfsr : unsigned(15 downto 0) := x"1D5B";

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1, s_fall2, s_fall3 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_dyc : unsigned(9 downto 0) := (others => '0');           -- rows below the tip, clamped
    signal s_hwp : unsigned(17 downto 0) := (others => '0');
    signal s_hwc_n, s_hwc : unsigned(11 downto 0) := (others => '0'); -- cone half-width
    signal s_adg_n : unsigned(10 downto 0) := (others => '0');        -- |y - yg|
    signal s_dyg2_n, s_dyg2 : unsigned(19 downto 0) := (others => '0');
    signal s_by_n, s_by : unsigned(6 downto 0) := (others => '0');    -- block row
    signal s_roff_n, s_roff : unsigned(6 downto 0) := (others => '0');-- dash row offset (0 = no dash)
    signal s_bhash : unsigned(7 downto 0) := (others => '0');

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_dxh : signed(12 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');
    signal s_vxa : unsigned(12 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_adx1 : unsigned(10 downto 0) := (others => '0');
    signal s_q1 : unsigned(23 downto 0) := (others => '0');
    signal s_bx1 : unsigned(6 downto 0) := (others => '0');
    signal s_rdd : t_u11(1 to 3) := (others => (others => '0'));
    signal s_qi2 : unsigned(21 downto 0) := (others => '0');
    signal s_cone : t_sl(2 to 7) := (others => '0');
    signal s_bxd2 : unsigned(6 downto 0) := (others => '0');
    signal s_r2_3 : unsigned(22 downto 0) := (others => '0');
    signal s_blue : t_sl(3 to 7) := (others => '0');
    signal s_gd4 : signed(24 downto 0) := (others => '0');
    signal s_glow5 : unsigned(7 downto 0) := (others => '0');
    signal s_glf6 : unsigned(16 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g7 : unsigned(7 downto 0) := (others => '0');
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_du8, s_dv8 : signed(11 downto 0) := (others => '0');
    signal s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_g8 : unsigned(7 downto 0) := (others => '0');
    signal s_y9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_pu9, s_pv9 : signed(20 downto 0) := (others => '0');
    signal s_u9, s_v9 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_i : signed(15 downto 0);
        variable v_f : unsigned(15 downto 0);
        variable v_h : unsigned(11 downto 0);
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

            s_bluk <= unsigned(registers_in(0));                               -- K1 Blue level
            s_bsk <= unsigned(registers_in(1)(9 downto 8));                    -- K2 Blocks 16 .. 128
            s_dshk <= unsigned(registers_in(2)(9 downto 2));                   -- K3 Dashes
            s_conk <= resize(unsigned(registers_in(3)(9 downto 3)), 8) + 8;    -- K4 Cone width 8 .. 135
            s_rgk <= resize(unsigned(registers_in(4)(9 downto 2)), 9) + 40;    -- K5 Glow radius 40 .. 295
            s_flk <= unsigned(registers_in(5)(9 downto 2));                    -- K6 Flicker
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_blu <= s_bluk; s_bs <= s_bsk; s_dsh <= s_dshk; s_con <= s_conk; s_rg <= s_rgk; s_fl <= s_flk;
                s_blkon <= not registers_in(6)(0);
                s_dshon <= not registers_in(6)(1);
                s_warm  <= registers_in(6)(2);
                s_fill  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_seed <= s_seed + 1;
                -- flicker gain 256 - rnd * k / 256
                v_f := s_lfsr(7 downto 0) * s_fl;
                s_flk9 <= to_unsigned(256, 9) - resize(v_f(15 downto 8), 9);
                -- tip row: 900 (stub) .. 120 (spire)
                v_h := to_unsigned(900, 12) - resize(s_g(11 downto 2), 12) + resize(s_g(11 downto 4), 12) + resize(s_g(11 downto 5), 12);
                s_y0 <= v_h(10 downto 0);
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
            s_yg <= signed(resize(s_y0, 12)) - signed(resize(s_rg, 12));
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_vw, 16), 3));
                               s_inc2 <= to_unsigned(32, 6);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_vw, 16), 2) + shift_left(resize(s_vw, 16), 1));
                               s_inc2 <= to_unsigned(18, 6);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_vw, 16), 2));
                               s_inc2 <= to_unsigned(8, 6);
            end case;
            s_inc0 <= v_i(14 downto 0);
            -- glow shift from the radius class (keeps the core bright at every radius)
            if s_rg < 64 then s_gsh <= "00"; elsif s_rg < 128 then s_gsh <= "01"; elsif s_rg < 224 then s_gsh <= "10"; else s_gsh <= "11"; end if;

            -- serial multiplier: job 0 (2cx)^2, job 1 rg^2, job 3 |y - yg|^2 per line
            if s_zc = 4 then
                s_ma <= resize(s_vw, 24); s_mb <= s_vw; s_mjob <= "00"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 24 then
                s_ma <= resize(s_rg, 24); s_mb <= resize(s_rg, 12); s_mjob <= "01"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 44 or s_fall3 = '1' then
                s_ma <= resize(s_adg_n, 24); s_mb <= resize(s_adg_n, 12); s_mjob <= "11"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "00"   => s_cx2sq <= s_macc;
                    when "01"   => s_rg2 <= s_macc;
                    when others => s_dyg2_n <= s_macc(19 downto 0);
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
    -- Source side: line-buffer write; per-line cone width, glow dy, block row,
    -- dash offset
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_d : signed(12 downto 0);
        variable v_r : unsigned(13 downto 0);
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
            s_fall3 <= s_fall2;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
            end if;
            -- cone: rows below the tip -> half-width = rows x width / 64
            v_d := signed(resize(s_vy_n(19 downto 8), 13)) - signed(resize(s_y0, 13));
            if v_d < 0 then s_dyc <= (others => '0'); elsif v_d > 1023 then s_dyc <= (others => '1'); else s_dyc <= unsigned(v_d(9 downto 0)); end if;
            s_hwp <= s_dyc * s_con;
            if s_fall3 = '1' or s_zone = '1' then
                s_hwc_n <= s_hwp(17 downto 6);
            end if;
            -- glow: |y - yg|
            v_d := signed(resize(s_vy_n(19 downto 8), 13)) - resize(s_yg, 13);
            if v_d < 0 then s_adg_n <= resize(unsigned(-v_d), 11); else s_adg_n <= unsigned(v_d(10 downto 0)); end if;
            -- block row and dash offset of the coming line
            case s_bs is
                when "00"   => s_by_n <= s_vy_n(14 downto 8) + s_seed(6 downto 0);
                when "01"   => s_by_n <= s_vy_n(15 downto 9) + s_seed(6 downto 0);
                when "10"   => s_by_n <= s_vy_n(16 downto 10) + s_seed(6 downto 0);
                when others => s_by_n <= s_vy_n(17 downto 11) + s_seed(6 downto 0);
            end case;
            -- an 8-line row tears into dashes when its hash is under the Dashes level
            v_r := s_vy_n(18 downto 11) * to_unsigned(37, 6);
            s_bhash <= v_r(7 downto 0) xor s_seed;
            if s_dshon = '1' and s_bhash < s_dsh then
                s_roff_n <= resize(s_bhash(6 downto 2), 7) + 3;
            else
                s_roff_n <= (others => '0');
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_hwc <= s_hwc_n;
                s_dyg2 <= s_dyg2_n;
                s_by <= s_by_n;
                s_roff <= s_roff_n;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= (others => '0');
                s_vxa <= (others => '0');
                s_dxh <= -signed(resize(s_vw, 13));
                s_q <= s_cx2sq;
                s_inc <= s_inc0;
            elsif g_avid = '1' then
                s_rdp <= s_rdp + 1;
                s_vxa <= s_vxa + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 13));
                s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                s_inc <= s_inc + signed(resize(s_inc2, 15));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ad : signed(12 downto 0);
        variable v_h : unsigned(11 downto 0);
        variable v_gp : unsigned(24 downto 0);
        variable v_g8 : unsigned(16 downto 0);
        variable v_y : unsigned(11 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_fu, v_fv : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: |dx|, dx^2, block column
            if s_dxh < 0 then v_ad := -s_dxh; else v_ad := s_dxh; end if;
            s_adx1 <= unsigned(v_ad(11 downto 1));
            s_q1 <= s_q;
            case s_bs is
                when "00"   => s_bx1 <= s_vxa(11 downto 5);
                when "01"   => s_bx1 <= s_vxa(12 downto 6);
                when "10"   => s_bx1 <= '0' & s_vxa(12 downto 7);
                when others => s_bx1 <= "00" & s_vxa(12 downto 8);
            end case;
            s_rdd(1) <= s_rdp;

            -- 2: dx^2 (integer); cone test; dashed block column
            s_qi2 <= s_q1(23 downto 2);
            if resize(s_adx1, 12) < s_hwc then s_cone(2) <= '1'; else s_cone(2) <= '0'; end if;
            s_bxd2 <= s_bx1 + s_roff;

            -- 3: r^2 to the glow centre; block hash -> blue
            s_r2_3 <= resize(s_qi2, 23) + resize(s_dyg2, 23);
            v_h := resize(resize(s_bxd2, 12) * to_unsigned(23, 5), 12) xor resize(resize(s_by, 12) * to_unsigned(41, 6), 12);
            if s_blkon = '1' and (v_h(3) xor v_h(5) xor v_h(7)) = '1' then s_blue(3) <= '1'; else s_blue(3) <= '0'; end if;

            -- 4: glow distance
            s_gd4 <= signed(resize(s_rg2, 25)) - signed(resize(s_r2_3, 25));
            s_lb_ra <= s_rdd(3);

            -- 5: glow level 0 .. 255 (positive part, shifted by the radius class)
            if s_gd4 <= 0 then
                s_glow5 <= (others => '0');
            else
                v_gp := unsigned(s_gd4);
                case s_gsh is
                    when "00"   => v_gp := shift_right(v_gp, 3);
                    when "01"   => v_gp := shift_right(v_gp, 5);
                    when "10"   => v_gp := shift_right(v_gp, 7);
                    when others => v_gp := shift_right(v_gp, 8);
                end case;
                if v_gp > 255 then s_glow5 <= (others => '1'); else s_glow5 <= v_gp(7 downto 0); end if;
            end if;

            -- 6: flicker; source (fabric copy)
            s_glf6 <= s_glow5 * s_flk9;
            s_y6 <= unsigned(s_lb_q(31 downto 22)); s_u6 <= unsigned(s_lb_q(21 downto 12)); s_v6 <= unsigned(s_lb_q(11 downto 2));

            -- 7: base colour: cone / blue block / black
            if s_glf6(16) = '1' then s_g7 <= (others => '1'); else s_g7 <= s_glf6(15 downto 8); end if;
            if s_cone(6) = '1' then
                if s_fill = '1' then
                    s_y7 <= s_y6; s_u7 <= s_u6; s_v7 <= s_v6;
                else
                    s_y7 <= to_unsigned(900, 10); s_u7 <= to_unsigned(512, 10); s_v7 <= to_unsigned(512, 10);
                end if;
            elsif s_blue(6) = '1' then
                s_y7 <= resize(shift_right(resize(s_blu, 11) + resize(s_y6, 11), 1), 10);
                s_u7 <= to_unsigned(440, 10); s_v7 <= to_unsigned(960, 10);
            else
                s_y7 <= to_unsigned(64, 10); s_u7 <= to_unsigned(512, 10); s_v7 <= to_unsigned(512, 10);
            end if;

            -- 8: add the glow to luma; chroma pull toward the flame colour
            v_y := resize(s_y7, 12) + resize(s_g7, 12) + shift_left(resize(s_g7, 12), 1);
            if v_y > 940 then s_y8 <= to_unsigned(940, 10); else s_y8 <= v_y(9 downto 0); end if;
            if s_warm = '1' then v_fu := to_signed(700, 12); v_fv := to_signed(360, 12);
            else v_fu := to_signed(512, 12); v_fv := to_signed(512, 12); end if;
            s_du8 <= v_fu - signed(resize(s_u7, 12));
            s_dv8 <= v_fv - signed(resize(s_v7, 12));
            s_u8 <= s_u7; s_v8 <= s_v7; s_g8 <= s_g7;

            -- 9: products
            s_pu9 <= s_du8 * signed(resize(s_g8, 9));
            s_pv9 <= s_dv8 * signed(resize(s_g8, 9));
            s_y9 <= s_y8; s_u9 <= s_u8; s_v9 <= s_v8;

            -- 10..12: chroma mix (stage 12 register)
            s_y12 <= s_y9;
            s_u12 <= unsigned(resize(signed(resize(s_u9, 12)) + resize(shift_right(s_pu9, 8), 12), 10));
            s_v12 <= unsigned(resize(signed(resize(s_v9, 12)) + resize(shift_right(s_pv9, 8), 12), 10));

            -- 13..18: output conditioning
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
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(17) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            for i in 3 to 7 loop s_cone(i) <= s_cone(i - 1); end loop;
            for i in 4 to 7 loop s_blue(i) <= s_blue(i - 1); end loop;
            for i in 2 to 3 loop s_rdd(i) <= s_rdd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture beacon;
