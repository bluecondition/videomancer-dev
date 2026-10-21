-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Shield v0.1.0 -- Heirloom #38 (2026-10-20), from WIN_20230529_22_56_41.
--
-- The photo: two heraldic shields side by side -- a rounded arch filled with
-- orange / red horizontal stripes above nested blue / red / black V
-- chevrons, dark at the cell edges, each shield mirror-symmetric.
--
-- The program: N cells across; in each, the region under a parabola from the
-- cell's top centre (P12 opens it from a spike to the whole cell) is filled
-- with horizontal stripes (orange / red, the source's luma showing through,
-- or the picture itself), the rest with V chevrons |x| + k y in blue / red /
-- black bands; the cell edges darken.  dx^2 by second differences restarting
-- at every cell.
--
--   K1  Cells     shields across, 1 .. 4
--   K2  Stripes   stripe pitch
--   K3  Chevrons  chevron band pitch
--   K4  Slope     chevron slope (flat bands .. steep V's)
--   K5  Drift     chevrons flow, bipolar (centre = still)
--   K6  Edge      darkening at the cell edges
--   S7  Mirror    the picture mirrored in every cell
--   S8  Palette   Heraldic (orange / red, blue / red / black) / Cool
--   S9  Fill      Stripes / Picture
--   S10 Chevrons  pointing down / up
--   S11 Source    Video / Osc
--   P12 ARCH      arch opening: chevrons only .. tall arches filling the cells
--
-- Virtual 1080-line geometry (cell widths from a per-frame serial divide);
-- generated avid; output conditioning before the gate.  Latency 20 clocks.
-- EBR: 16 line buffer.  u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture shield of program_top is
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

    -- colours {y, u = Cr, v = Cb}: 0/1 stripes A (orange / red), 2..5 chevrons A
    -- (blue red black red), 6/7 stripes B (cyan / blue), 8..11 chevrons B
    -- (magenta green black green); a case, not an array-of-arrays constant
    function f_col(i : integer) return std_logic_vector is
        variable y, u, v : integer;
    begin
        case i is
            when 0  => y := 584; u := 772; v := 212;
            when 1  => y := 328; u := 960; v := 360;
            when 2  => y := 164; u := 440; v := 960;
            when 3  => y := 328; u := 960; v := 360;
            when 4  => y := 64;  u := 512; v := 512;
            when 5  => y := 328; u := 960; v := 360;
            when 6  => y := 678; u := 64;  v := 663;
            when 7  => y := 164; u := 440; v := 960;
            when 8  => y := 426; u := 887; v := 808;
            when 9  => y := 580; u := 136; v := 216;
            when 10 => y := 64;  u := 512; v := 512;
            when others => y := 580; u := 136; v := 216;
        end case;
        return std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10)) & std_logic_vector(to_unsigned(v, 10));
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
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_j1 : unsigned(1 downto 0) := to_unsigned(1, 2);
    signal s_spk : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_cpk : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_slk : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_drk : signed(5 downto 0) := (others => '0');
    signal s_edk : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_n : unsigned(1 downto 0) := to_unsigned(1, 2);
    signal s_sp : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_cp : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_sl : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_ed : unsigned(7 downto 0) := (others => '0');
    signal s_ga : unsigned(9 downto 0) := to_unsigned(256, 10);      -- arch gain
    signal s_cph : unsigned(11 downto 0) := (others => '0');          -- chevron phase
    signal s_mir, s_cool, s_pic, s_up : std_logic := '0';
    signal s_cwpx, s_cwv : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_cwpxm1 : unsigned(10 downto 0) := to_unsigned(959, 11);
    signal s_cwv2 : unsigned(12 downto 0) := to_unsigned(1920, 13);
    signal s_cw2 : unsigned(23 downto 0) := (others => '0');          -- cw^2 (Q.2 start of dx^2)
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_edge0 : unsigned(10 downto 0) := to_unsigned(360, 11);   -- edge zone start (3/8 cw)
    -- serial unit
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun, s_mdiv : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');
    signal s_dnum : unsigned(11 downto 0) := (others => '0');
    signal s_drem : unsigned(12 downto 0) := (others => '0');
    signal s_dquo : unsigned(11 downto 0) := (others => '0');

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
    signal s_dya_n : unsigned(10 downto 0) := (others => '0');        -- vy - 60 (arch), clamped
    signal s_thr_n, s_thr : unsigned(23 downto 0) := (others => '0'); -- arch threshold on dx^2
    signal s_vyk_n, s_vyk : unsigned(11 downto 0) := (others => '0'); -- chevron y term
    signal s_vyr : unsigned(11 downto 0) := (others => '0');
    signal s_vyc : unsigned(19 downto 0) := (others => '0');
    signal s_stp_n, s_stp : std_logic := '0';                         -- stripe parity
    signal s_vsp : unsigned(17 downto 0) := (others => '0');

    -- beam state
    signal s_cu : unsigned(12 downto 0) := (others => '0');
    signal s_cupx, s_cdpx, s_cs : unsigned(10 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_dxh1 : signed(13 downto 0) := (others => '0');
    signal s_q1 : unsigned(23 downto 0) := (others => '0');
    signal s_m1, s_cs1 : unsigned(10 downto 0) := (others => '0');
    signal s_adx2 : unsigned(11 downto 0) := (others => '0');
    signal s_qi2 : unsigned(21 downto 0) := (others => '0');
    signal s_rd2, s_rd3 : unsigned(10 downto 0) := (others => '0');
    signal s_arch : t_sl(3 to 7) := (others => '0');
    signal s_f3 : unsigned(12 downto 0) := (others => '0');
    signal s_d3 : signed(12 downto 0) := (others => '0');
    signal s_fp4 : unsigned(20 downto 0) := (others => '0');
    signal s_dp4 : unsigned(19 downto 0) := (others => '0');
    signal s_band5 : unsigned(1 downto 0) := (others => '0');
    signal s_dark : t_u10(5 to 8) := (others => (others => '0'));
    signal s_band6 : unsigned(1 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_d8 : unsigned(9 downto 0) := (others => '0');
    signal s_k8 : unsigned(8 downto 0) := (others => '0');
    signal s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_m9 : unsigned(18 downto 0) := (others => '0');
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
    -- Controls, vblank zone, per-field latches, serial unit
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_r : unsigned(12 downto 0);
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

            -- K1 Cells (4 zones, deadband after warm-up)
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_j1 <= v_k(9 downto 8);
            end if;
            s_spk <= resize(unsigned(registers_in(1)(9 downto 5)), 6) + 2;       -- K2 Stripes 2 .. 33
            s_cpk <= unsigned(registers_in(2)(9 downto 2));                      -- K3 Chevrons
            s_slk <= unsigned(registers_in(3)(9 downto 2));                      -- K4 Slope
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;          -- K5 Drift, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_drk <= (others => '0'); else s_drk <= v_s(5 downto 0); end if;
            s_edk <= unsigned(registers_in(5)(9 downto 2));                      -- K6 Edge
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_n <= s_j1;
                s_sp <= s_spk; s_cp <= s_cpk; s_sl <= s_slk; s_ed <= s_edk;
                s_cph <= s_cph + unsigned(resize(s_drk, 12));
                s_mir  <= registers_in(6)(0);
                s_cool <= registers_in(6)(1);
                s_pic  <= registers_in(6)(2);
                s_up   <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                -- arch gain 1 .. 1024 (squared feel: p12^2 / 1024 + 1)
                s_ga <= resize(shift_right(resize(s_g(11 downto 2), 20) * resize(s_g(11 downto 2), 20), 10), 10) + 1;
            end if;

            -- derived
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cwpxm1 <= s_cwpx(10 downto 0) - 1;
            s_cwv2 <= s_cwv & '0';
            s_edge0 <= resize(s_cwv(11 downto 2), 11) + resize(s_cwv(11 downto 3), 11);   -- 3/8 cw
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_cwv, 16), 3));
                               s_inc2 <= to_unsigned(32, 6);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_cwv, 16), 2) + shift_left(resize(s_cwv, 16), 1));
                               s_inc2 <= to_unsigned(18, 6);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_cwv, 16), 2));
                               s_inc2 <= to_unsigned(8, 6);
            end case;
            s_inc0 <= v_i(14 downto 0);

            -- serial unit: job 0 w / N, job 1 vw / N (divides), job 2 cw x cw, job 3 (vy - 60) x ga per line
            if s_zc = 4 then
                s_dnum <= s_w; s_drem <= (others => '0'); s_mb <= resize(s_n, 12) + 1;
                s_mjob <= "000"; s_mdiv <= '1'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 24 then
                s_dnum <= s_vw; s_drem <= (others => '0'); s_mb <= resize(s_n, 12) + 1;
                s_mjob <= "001"; s_mdiv <= '1'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 44 then
                s_ma <= resize(s_cwv, 24); s_mb <= s_cwv; s_macc <= (others => '0');
                s_mjob <= "010"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 64 or s_fall2 = '1' then
                s_ma <= resize(s_dya_n, 24); s_mb <= resize(s_ga, 12); s_macc <= (others => '0');
                s_mjob <= "011"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mdiv = '1' then
                    v_r := s_drem(11 downto 0) & s_dnum(11);
                    s_dnum <= s_dnum(10 downto 0) & '0';
                    if v_r >= resize(s_mb, 13) then
                        s_drem <= v_r - resize(s_mb, 13);
                        s_dquo <= s_dquo(10 downto 0) & '1';
                    else
                        s_drem <= v_r;
                        s_dquo <= s_dquo(10 downto 0) & '0';
                    end if;
                else
                    if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                    s_ma <= s_ma(22 downto 0) & '0';
                    s_mb <= '0' & s_mb(11 downto 1);
                end if;
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_cwpx <= s_dquo;
                    when "001"  => s_cwv <= s_dquo;
                    when "010"  => s_cw2 <= s_macc;
                    when others => s_thr_n <= s_macc;
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
    -- Source side: line-buffer write; per-line arch threshold, chevron y term,
    -- stripe parity
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_d : signed(12 downto 0);
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
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
            end if;
            -- arch: dy above the top margin (row 60)
            v_d := signed(resize(s_vy_n(19 downto 8), 13)) - 60;
            if v_d < 0 then s_dya_n <= (others => '0'); else s_dya_n <= unsigned(v_d(10 downto 0)); end if;
            -- chevron y term: slope x y (or x (1080 - y) when pointing up), plus the drift phase
            if s_up = '1' then s_vyr <= to_unsigned(1080, 12) - s_vy_n(19 downto 8); else s_vyr <= s_vy_n(19 downto 8); end if;
            s_vyc <= s_vyr * s_sl;
            s_vyk_n <= resize(s_vyc(19 downto 8), 12) + s_cph;
            -- stripe parity: bit of (y x pitch) / 16
            s_vsp <= s_vy_n(19 downto 8) * s_sp;
            s_stp_n <= s_vsp(4);

            if data_in.avid = '1' and s_avq = '0' then
                s_thr <= s_thr_n;
                s_vyk <= s_vyk_n;
                s_stp <= s_stp_n;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state: cell position, dx^2 by second differences restarting at
    -- every cell (dx = cu - cw in Q.1)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_cu <= (others => '0');
                s_cupx <= (others => '0');
                s_cdpx <= s_cwpxm1;
                s_cs <= (others => '0');
                s_q <= s_cw2;
                s_inc <= s_inc0;
            elsif g_avid = '1' then
                v_n := resize(s_cu, 14) + s_xs;
                if v_n >= s_cwv2 then
                    s_cu <= resize(v_n - s_cwv2, 13);
                    s_q <= s_cw2;
                    s_inc <= s_inc0;
                else
                    s_cu <= v_n(12 downto 0);
                    s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                    s_inc <= s_inc + signed(resize(s_inc2, 15));
                end if;
                if s_cupx = s_cwpxm1 then
                    s_cupx <= (others => '0');
                    s_cdpx <= s_cwpxm1;
                    s_cs <= s_cs + s_cwpx(10 downto 0);
                else
                    s_cupx <= s_cupx + 1;
                    s_cdpx <= s_cdpx - 1;
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_dx : signed(13 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ci : integer range 0 to 11;
        variable v_cc : std_logic_vector(29 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: dx (Q.1) and the mirrored read offset
            s_dxh1 <= signed(resize(s_cu, 14)) - signed(resize(s_cwv, 14));
            s_q1 <= s_q;
            if s_mir = '1' and s_cupx >= s_cdpx then s_m1 <= s_cdpx; else s_m1 <= s_cupx; end if;
            s_cs1 <= s_cs;

            -- 2: |dx|, dx^2
            if s_dxh1 < 0 then v_dx := -s_dxh1; else v_dx := s_dxh1; end if;
            s_adx2 <= unsigned(v_dx(12 downto 1));
            s_qi2 <= s_q1(23 downto 2);
            s_rd2 <= s_cs1 + s_m1;

            -- 3: arch test; chevron field; edge distance
            if resize(s_qi2, 24) <= s_thr then s_arch(3) <= '1'; else s_arch(3) <= '0'; end if;
            s_f3 <= resize(s_adx2, 13) + resize(s_vyk, 13);
            s_d3 <= signed(resize(s_adx2, 13)) - signed(resize(s_edge0, 13));
            s_rd3 <= s_rd2;

            -- 4: chevron band product; edge product; read
            s_fp4 <= s_f3 * s_cp;
            if s_d3 < 0 then s_dp4 <= (others => '0'); else s_dp4 <= unsigned(s_d3(11 downto 0)) * s_ed; end if;
            s_lb_ra <= s_rd3;

            -- 5: band index (2 bits), darkening amount
            s_band5 <= s_fp4(14 downto 13);
            if s_dp4(19 downto 12) /= 0 then s_dark(5) <= to_unsigned(255, 10); else s_dark(5) <= resize(s_dp4(11 downto 4), 10); end if;

            -- 6: source (fabric copy)
            s_y6 <= unsigned(s_lb_q(31 downto 22)); s_u6 <= unsigned(s_lb_q(21 downto 12)); s_v6 <= unsigned(s_lb_q(11 downto 2));
            s_band6 <= s_band5;

            -- 7: colour: arch stripes (or the picture) / chevron palette
            if s_arch(6) = '1' then
                if s_pic = '1' then
                    s_y7 <= s_y6; s_u7 <= s_u6; s_v7 <= s_v6;
                else
                    if s_cool = '1' then v_ci := 6; else v_ci := 0; end if;
                    if s_stp = '1' then v_ci := v_ci + 1; end if;
                    v_cc := f_col(v_ci);
                    s_y7 <= resize(shift_right(resize(s_y6, 11) + resize(unsigned(v_cc(29 downto 20)), 11), 1), 10);
                    s_u7 <= unsigned(v_cc(19 downto 10)); s_v7 <= unsigned(v_cc(9 downto 0));
                end if;
            else
                if s_cool = '1' then v_ci := 8 + to_integer(s_band6); else v_ci := 2 + to_integer(s_band6); end if;
                v_cc := f_col(v_ci);
                s_y7 <= unsigned(v_cc(29 downto 20)); s_u7 <= unsigned(v_cc(19 downto 10)); s_v7 <= unsigned(v_cc(9 downto 0));
            end if;

            -- 8: edge darkening operands
            s_d8 <= s_y7 - 64;
            s_k8 <= to_unsigned(256, 9) - resize(s_dark(7)(7 downto 0), 9);
            s_u8 <= s_u7; s_v8 <= s_v7;

            -- 9: product
            s_m9 <= s_d8 * s_k8;
            s_u9 <= s_u8; s_v9 <= s_v8;

            -- 10..12: luma back (stage 12 register)
            s_y12 <= resize(s_m9(18 downto 8), 10) + 64;
            s_u12 <= s_u9; s_v12 <= s_v9;

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

            for i in 4 to 7 loop s_arch(i) <= s_arch(i - 1); end loop;
            for i in 6 to 8 loop s_dark(i) <= s_dark(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture shield;
