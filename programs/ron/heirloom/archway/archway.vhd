-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Archway v0.1.0 -- Heirloom #36 (2026-10-20), from WIN_20230523_01_26_43.
--
-- The photo: black ground, a tall arch of thin cyan / blue lines -- nested
-- parabolas rising from the bottom centre -- stepped pink / yellow / navy
-- blocks along its outer flanks, a pink chevron inside.
--
-- The program: the field F = y + a dx^2 (parabolas from the top centre; P12
-- sets the curvature) is cut into thin ray lines inside the arch zone and
-- into coarse colour blocks (bands of F stepped by column) outside it; a
-- pink chevron sits at the bottom centre.  The picture shows dark between the
-- rays and lights the flank blocks (or replaces them).
--
--   K1  Rays     ray pitch (powers of two)
--   K2  Width    ray line width
--   K3  Blocks   flank block width
--   K4  Hue      ray colour pair: cyan/blue, yellow/orange, magenta/violet, green/white
--   K5  Drift    rays flow, bipolar (centre = still)
--   K6  Reach    how far the rays extend before the flanks take over
--   S7  Rays     On / Off (the arch zone shows the darkened picture)
--   S8  Flanks   Blocks / Picture
--   S9  Chevron  On / Off
--   S10 Mirror   the picture mirrored about the centre
--   S11 Source   Video / Osc
--   P12 ARCH     curvature: a gentle arch .. a needle
--
-- Virtual 1080-line geometry; dx^2 by second differences; the per-frame
-- square on the serial unit.  Generated avid; output conditioning before the
-- gate.  Latency 20 clocks.  EBR: 16 line buffer.  u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture archway of program_top is
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
    -- colours {y, u = Cr, v = Cb}: 0..7 ray pairs, 8..11 flank blocks
    function f_col(i : integer) return std_logic_vector is
        variable y, u, v : integer;
    begin
        case i is
            when 0  => y := 678; u := 64;  v := 663;   -- cyan
            when 1  => y := 164; u := 440; v := 960;   -- blue
            when 2  => y := 840; u := 584; v := 64;    -- yellow
            when 3  => y := 584; u := 772; v := 212;   -- orange
            when 4  => y := 426; u := 887; v := 808;   -- magenta
            when 5  => y := 349; u := 756; v := 852;   -- violet
            when 6  => y := 580; u := 136; v := 216;   -- green
            when 7  => y := 940; u := 512; v := 512;   -- white
            when 8  => y := 752; u := 612; v := 512;   -- pink
            when 9  => y := 840; u := 584; v := 64;    -- yellow
            when 10 => y := 120; u := 470; v := 700;   -- navy
            when others => y := 752; u := 612; v := 512;
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
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_cxp : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_cxp2 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_rsk : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_wdk : unsigned(7 downto 0) := to_unsigned(32, 8);
    signal s_bsk : unsigned(1 downto 0) := "01";
    signal s_huk : unsigned(1 downto 0) := "00";
    signal s_drk : signed(5 downto 0) := (others => '0');
    signal s_rek : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_rs : unsigned(3 downto 0) := to_unsigned(8, 4);          -- ray pitch shift 6 .. 13
    signal s_wd : unsigned(7 downto 0) := to_unsigned(32, 8);
    signal s_duty : unsigned(13 downto 0) := to_unsigned(32, 14);
    signal s_bs : unsigned(1 downto 0) := "01";
    signal s_hu : unsigned(1 downto 0) := "00";
    signal s_fz : unsigned(20 downto 0) := (others => '0');
    signal s_fph : unsigned(15 downto 0) := (others => '0');
    signal s_a : unsigned(8 downto 0) := to_unsigned(16, 9);
    signal s_cx2sq : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_rays, s_fpic, s_chev, s_mir : std_logic := '0';
    -- serial multiplier
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';

    -- generated avid
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
    signal s_vyp_n, s_vyp : unsigned(20 downto 0) := (others => '0');  -- y * 16 + drift phase
    signal s_vyb_n, s_vyb : unsigned(10 downto 0) := (others => '0');  -- rows below the chevron apex (0 above)

    -- beam state
    signal s_bcol : unsigned(10 downto 0) := (others => '0');
    signal s_vxa : unsigned(12 downto 0) := (others => '0');
    signal s_dxh : signed(12 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_adx1 : unsigned(10 downto 0) := (others => '0');
    signal s_q1 : unsigned(23 downto 0) := (others => '0');
    signal s_cb1 : unsigned(6 downto 0) := (others => '0');
    signal s_rd1 : unsigned(10 downto 0) := (others => '0');
    signal s_rdd : t_u11(2 to 4) := (others => (others => '0'));
    signal s_qi2 : unsigned(19 downto 0) := (others => '0');
    signal s_cbd : t_u10(2 to 6) := (others => (others => '0'));
    signal s_chv : t_sl(2 to 7) := (others => '0');
    signal s_hi3, s_lo3 : unsigned(18 downto 0) := (others => '0');
    signal s_pa4 : unsigned(19 downto 0) := (others => '0');
    signal s_f5 : unsigned(20 downto 0) := (others => '0');
    signal s_frac6 : unsigned(13 downto 0) := (others => '0');
    signal s_par6, s_zn6 : std_logic := '0';
    signal s_idx6 : unsigned(1 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
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
    -- Controls, vblank zone, per-field latches, serial square
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_i : signed(15 downto 0);
        variable v_d : unsigned(21 downto 0);
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

            s_rsk <= unsigned(registers_in(0)(9 downto 7));                    -- K1 Rays: pitch 4 .. 512 lines
            s_wdk <= resize(unsigned(registers_in(1)(9 downto 3)), 8) + 4;     -- K2 Width 4 .. 131 / 256 of the pitch
            s_bsk <= unsigned(registers_in(2)(9 downto 8));                    -- K3 Blocks 32 .. 256
            s_huk <= unsigned(registers_in(3)(9 downto 8));                    -- K4 Hue pair
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;        -- K5 Drift, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_drk <= (others => '0'); else s_drk <= v_s(5 downto 0); end if;
            s_rek <= unsigned(registers_in(5));                                -- K6 Reach
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_rs <= resize(s_rsk, 4) + 6; s_wd <= s_wdk; s_bs <= s_bsk; s_hu <= s_huk;
                s_fph <= s_fph + unsigned(resize(s_drk, 16)) + unsigned(resize(s_drk, 16));
                s_fz <= shift_left(resize(s_rek, 21), 7);
                s_rays <= not registers_in(6)(0);
                s_fpic <= registers_in(6)(1);
                s_chev <= not registers_in(6)(2);
                s_mir  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_a <= resize(s_g(11 downto 4), 9) + 1;
            end if;

            -- derived
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cxp <= s_w(11 downto 1);
            s_cxp2 <= s_w;
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_vw, 16), 3));
                               s_inc2 <= to_unsigned(32, 6);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_vw, 16), 2) + shift_left(resize(s_vw, 16), 1));
                               s_inc2 <= to_unsigned(18, 6);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_vw, 16), 2));
                               s_inc2 <= to_unsigned(8, 6);
            end case;
            s_inc0 <= v_i(14 downto 0);
            -- duty = width x pitch / 256
            v_d := shift_left(resize(s_wd, 22), to_integer(s_rs));
            s_duty <= v_d(21 downto 8);

            -- serial: (2cx)^2
            if s_zc = 4 then
                s_ma <= resize(s_vw, 24); s_mb <= s_vw; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
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
    -- Source side: line-buffer write; per-line y terms
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

            if s_zone = '1' then
                s_vy_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
            end if;
            s_vyp_n <= shift_left(resize(s_vy_n(19 downto 8), 21), 4) + resize(s_fph, 21);
            v_d := signed(resize(s_vy_n(19 downto 8), 13)) - 780;
            if v_d <= 0 then s_vyb_n <= (others => '0'); else s_vyb_n <= unsigned(v_d(10 downto 0)); end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_vyp <= s_vyp_n;
                s_vyb <= s_vyb_n;
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
                s_bcol <= (others => '0');
                s_vxa <= (others => '0');
                s_dxh <= -signed(resize(s_vw, 13));
                s_q <= s_cx2sq;
                s_inc <= s_inc0;
            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
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
        variable v_fr : unsigned(20 downto 0);
        variable v_ci : integer range 0 to 11;
        variable v_cc : std_logic_vector(29 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: |dx|, dx^2, block column, (mirrored) read column
            if s_dxh < 0 then v_ad := -s_dxh; else v_ad := s_dxh; end if;
            s_adx1 <= unsigned(v_ad(11 downto 1));
            s_q1 <= s_q;
            case s_bs is
                when "00"   => s_cb1 <= s_vxa(12 downto 6);
                when "01"   => s_cb1 <= '0' & s_vxa(12 downto 7);
                when "10"   => s_cb1 <= "00" & s_vxa(12 downto 8);
                when others => s_cb1 <= "000" & s_vxa(12 downto 9);
            end case;
            if s_mir = '1' and s_bcol >= s_cxp then s_rd1 <= resize(s_cxp2 - resize(s_bcol, 12), 11); else s_rd1 <= s_bcol; end if;

            -- 2: dx^2 (integer); chevron test
            s_qi2 <= s_q1(21 downto 2);
            if s_vyb /= 0 and resize(s_adx1, 12) < resize(s_vyb, 12) then s_chv(2) <= '1'; else s_chv(2) <= '0'; end if;
            s_cbd(2) <= resize(s_cb1, 10);
            s_rdd(2) <= s_rd1;

            -- 3: a dx^2 in two partial products
            s_hi3 <= s_qi2(19 downto 10) * s_a;
            s_lo3 <= s_qi2(9 downto 0) * s_a;

            -- 4: a dx^2 (1/16-line units); read
            s_pa4 <= resize(s_hi3, 20) + resize(s_lo3(18 downto 10), 20);
            s_lb_ra <= s_rdd(4);

            -- 5: field F = 16 y + a dx^2 + phase
            s_f5 <= resize(s_pa4, 21) + s_vyp;

            -- 6: fractional part at the ray pitch, parity, zone, flank index; source
            v_fr := s_f5;
            case to_integer(s_rs) is
                when 6  => s_frac6 <= resize(v_fr(5 downto 0), 14); s_par6 <= v_fr(6);
                when 7  => s_frac6 <= resize(v_fr(6 downto 0), 14); s_par6 <= v_fr(7);
                when 8  => s_frac6 <= resize(v_fr(7 downto 0), 14); s_par6 <= v_fr(8);
                when 9  => s_frac6 <= resize(v_fr(8 downto 0), 14); s_par6 <= v_fr(9);
                when 10 => s_frac6 <= resize(v_fr(9 downto 0), 14); s_par6 <= v_fr(10);
                when 11 => s_frac6 <= resize(v_fr(10 downto 0), 14); s_par6 <= v_fr(11);
                when 12 => s_frac6 <= resize(v_fr(11 downto 0), 14); s_par6 <= v_fr(12);
                when others => s_frac6 <= resize(v_fr(12 downto 0), 14); s_par6 <= v_fr(13);
            end case;
            if s_f5 < s_fz then s_zn6 <= '1'; else s_zn6 <= '0'; end if;
            s_idx6 <= s_f5(12 downto 11) + s_cbd(5)(1 downto 0);
            s_y6 <= unsigned(s_lb_q(31 downto 22)); s_u6 <= unsigned(s_lb_q(21 downto 12)); s_v6 <= unsigned(s_lb_q(11 downto 2));

            -- 7: colour
            if s_chv(6) = '1' and s_chev = '1' then
                v_cc := f_col(8);
                s_y7 <= unsigned(v_cc(29 downto 20)); s_u7 <= unsigned(v_cc(19 downto 10)); s_v7 <= unsigned(v_cc(9 downto 0));
            elsif s_zn6 = '1' then
                if s_rays = '1' and s_frac6 < s_duty then
                    v_ci := to_integer(s_hu) * 2;
                    if s_par6 = '1' then v_ci := v_ci + 1; end if;
                    v_cc := f_col(v_ci);
                    s_y7 <= unsigned(v_cc(29 downto 20)); s_u7 <= unsigned(v_cc(19 downto 10)); s_v7 <= unsigned(v_cc(9 downto 0));
                else
                    s_y7 <= to_unsigned(64, 10) + resize(s_y6(9 downto 2), 10);
                    s_u7 <= to_unsigned(384, 10) + resize(s_u6(9 downto 2), 10);
                    s_v7 <= to_unsigned(384, 10) + resize(s_v6(9 downto 2), 10);
                end if;
            else
                v_cc := f_col(8 + to_integer(s_idx6));
                if s_fpic = '1' then
                    s_y7 <= s_y6;
                else
                    s_y7 <= resize(shift_right(resize(s_y6, 11) + resize(unsigned(v_cc(29 downto 20)), 11), 1), 10);
                end if;
                s_u7 <= unsigned(v_cc(19 downto 10)); s_v7 <= unsigned(v_cc(9 downto 0));
            end if;

            -- 8..12: pad (stage 12 register)
            s_y12 <= s_y7; s_u12 <= s_u7; s_v12 <= s_v7;

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

            for i in 3 to 7 loop s_chv(i) <= s_chv(i - 1); end loop;
            for i in 3 to 6 loop s_cbd(i) <= s_cbd(i - 1); end loop;
            for i in 3 to 4 loop s_rdd(i) <= s_rdd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture archway;
