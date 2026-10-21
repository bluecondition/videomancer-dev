-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Vesica v0.1.0 -- Heirloom #13 (2026-10-20), from WIN_20230307_01_04_17.
--
-- The photo: a pale green ground with a pastel diamond-outline lattice
-- (blue / pink / yellow lines), an orange vesica -- an eye -- in the centre
-- filled with nested chevron rings, crescents above and below it, and
-- horizontal tearing.
--
-- The program: two diagonal stripe families draw the lattice; two circles
-- (one above, one below the centre) overlap into a vesica whose inside is
-- painted with nested rings (iso-lines of the lens shape) in orange tones,
-- the circles' remaining parts are the crescents; the video (or the
-- oscillator) etches the rings and can tint the lattice.
--
--   K1  Pitch    lattice pitch
--   K2  Width    lattice line width
--   K3  Rings    ring density inside the eye
--   K4  Aspect   eye aspect: the circles' offset (round eye .. a thin lens)
--   K5  Drift    lattice drift, bipolar (centre = still)
--   K6  Tear     sideways tearing of the video in 8-line bands
--   S7  Lattice  on / off
--   S8  Rings    Rings / Solid eye
--   S9  Crescent crescents on / off
--   S10 Tint     lattice lines take the video's chroma
--   S11 Source   Video / Osc
--   P12 EYE      opening of the vesica: a slit .. the whole screen
--
-- Virtual 1080-line geometry; dx^2 by second differences (shared by both
-- circles), dy^2 for each circle per line on the serial multiplier.
-- Generated avid; output conditioning before the blanking gate.  Latency
-- 20 clocks.  16 EBR.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture vesica of program_top is
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

    -- colours (u = Cr, v = Cb)
    constant C_GRD_Y : unsigned(9 downto 0) := to_unsigned(824, 10);   -- pale green
    constant C_GRD_U : unsigned(9 downto 0) := to_unsigned(459, 10);
    constant C_GRD_V : unsigned(9 downto 0) := to_unsigned(430, 10);
    constant C_CRS_Y : unsigned(9 downto 0) := to_unsigned(428, 10);   -- red-orange
    constant C_CRS_U : unsigned(9 downto 0) := to_unsigned(822, 10);
    constant C_CRS_V : unsigned(9 downto 0) := to_unsigned(340, 10);

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

    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_fx : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_lw : unsigned(6 downto 0) := to_unsigned(24, 7);
    signal s_k3 : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_ek : unsigned(8 downto 0) := to_unsigned(200, 9);
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_tk : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- latched / derived
    signal s_lat, s_solid, s_cres, s_tint : std_logic := '0';
    signal s_e : unsigned(9 downto 0) := to_unsigned(200, 10);
    signal s_r : unsigned(10 downto 0) := to_unsigned(500, 11);
    signal s_r2 : unsigned(21 downto 0) := (others => '0');
    signal s_cx2sq : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_fxs : unsigned(8 downto 0) := to_unsigned(16, 9);
    signal s_fys : unsigned(9 downto 0) := to_unsigned(16, 10);
    signal s_lph : unsigned(11 downto 0) := (others => '0');
    signal s_lw2, s_lw4 : unsigned(6 downto 0) := (others => '0');
    -- serial multiplier
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');
    signal s_lfsr : unsigned(15 downto 0) := x"7A3C";

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
    signal s_ad1_n, s_ad2_n : unsigned(10 downto 0) := to_unsigned(340, 11);
    signal s_dy1_n, s_dy1, s_dy2_n, s_dy2 : unsigned(19 downto 0) := (others => '0');
    signal s_py : unsigned(11 downto 0) := (others => '0');
    signal s_pyl : unsigned(11 downto 0) := (others => '0');
    signal s_tear_n : unsigned(10 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');
    signal s_l8 : unsigned(2 downto 0) := (others => '0');

    -- beam
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_px : unsigned(11 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_u2 is array (natural range <>) of unsigned(1 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_g1, s_g2 : unsigned(7 downto 0) := (others => '0');
    signal s_q1 : unsigned(23 downto 0) := (others => '0');
    signal s_rdd : t_u11(1 to 2) := (others => (others => '0'));
    signal s_lf2 : std_logic := '0';
    signal s_lb2 : unsigned(1 downto 0) := (others => '0');
    signal s_qi2 : unsigned(21 downto 0) := (others => '0');
    signal s_lf : t_sl(3 to 7) := (others => '0');
    signal s_lbd : t_u2(3 to 7) := (others => (others => '0'));
    signal s_r1_3, s_r2_3 : unsigned(22 downto 0) := (others => '0');
    signal s_in1_4, s_in2_4 : std_logic := '0';
    signal s_mx4, s_mn4 : unsigned(22 downto 0) := (others => '0');
    signal s_eye5, s_crs5 : std_logic := '0';
    signal s_d5 : unsigned(21 downto 0) := (others => '0');
    signal s_cl5 : unsigned(9 downto 0) := (others => '0');
    signal s_y5, s_u5, s_v5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_eye6, s_crs6 : std_logic := '0';
    signal s_pr6 : unsigned(15 downto 0) := (others => '0');
    signal s_cl6 : unsigned(9 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_eye7, s_crs7 : std_logic := '0';
    signal s_ri7 : unsigned(7 downto 0) := (others => '0');
    signal s_cl7 : unsigned(9 downto 0) := (others => '0');
    signal s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y9, s_y10, s_y11, s_y12, s_y13, s_y14x, s_yf14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yp : t_u10(15 to 20) := (others => to_unsigned(64, 10));
    signal s_up, s_vp : t_u10(15 to 20) := (others => to_unsigned(512, 10));

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
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_i : signed(15 downto 0);
        variable v_p : unsigned(12 downto 0);
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

            s_fx <= resize(unsigned(registers_in(0)(9 downto 5)), 6) + 1;       -- K1 Pitch 1 .. 32
            s_lw <= resize(unsigned(registers_in(1)(9 downto 4)), 7) + 8;       -- K2 Width 8 .. 71
            s_k3 <= resize(unsigned(registers_in(2)(9 downto 3)), 8) + 8;       -- K3 Rings 8 .. 135
            s_ek <= resize(unsigned(registers_in(3)(9 downto 2)), 9) + 16;      -- K4 Aspect 16 .. 271
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;         -- K5 Drift
            if v_s > -8 and v_s < 8 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            s_tk <= unsigned(registers_in(5)(9 downto 2));                      -- K6 Tear
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_lat   <= not registers_in(6)(0);
                s_solid <= registers_in(6)(1);
                s_cres  <= not registers_in(6)(2);
                s_tint  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_lph <= s_lph + unsigned(resize(shift_right(s_k5r, 3), 12));
                s_e <= resize(s_ek, 10) + resize(s_ek(8 downto 1), 10);        -- 24 .. 406
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            -- radius = e + 32 + 0.78 p12 (the eye never closes)
            v_p := resize(s_e, 13) + 32 + resize(s_g(11 downto 2), 13) - resize(s_g(11 downto 4), 13) - resize(s_g(11 downto 5), 13) - resize(s_g(11 downto 6), 13);
            s_r <= v_p(10 downto 0);
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_vw, 16), 3));
                               s_inc2 <= to_unsigned(32, 6);
                               s_fxs <= shift_left(resize(s_fx, 9), 2);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_vw, 16), 2) + shift_left(resize(s_vw, 16), 1));
                               s_inc2 <= to_unsigned(18, 6);
                               s_fxs <= shift_left(resize(s_fx, 9), 1) + resize(s_fx, 9);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_vw, 16), 2));
                               s_inc2 <= to_unsigned(8, 6);
                               s_fxs <= shift_left(resize(s_fx, 9), 1);
            end case;
            s_inc0 <= v_i(14 downto 0);
            s_lw2 <= '0' & s_lw(6 downto 1);
            s_lw4 <= "00" & s_lw(6 downto 2);

            -- serial unit: 0 (2cx)^2, 1 R^2, 2 fx x ys (per-line lattice step), 3 dy1^2, 4 dy2^2
            if s_zc = 4 then
                s_ma <= resize(s_vw, 24); s_mb <= s_vw; s_mjob <= "000"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 24 then
                s_ma <= resize(s_r, 24); s_mb <= resize(s_r, 12); s_mjob <= "001"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 44 then
                s_ma <= resize(s_fx, 24); s_mb <= s_ys; s_mjob <= "010"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 64 or s_fall2 = '1' then
                s_ma <= resize(s_ad1_n, 24); s_mb <= resize(s_ad1_n, 12); s_mjob <= "011"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 84 or s_fall3 = '1' then
                s_ma <= resize(s_ad2_n, 24); s_mb <= resize(s_ad2_n, 12); s_mjob <= "100"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_cx2sq <= s_macc;
                    when "001"  => s_r2 <= s_macc(21 downto 0);
                    when "010"  => s_fys <= s_macc(17 downto 8);
                    when "011"  => s_dy1_n <= s_macc(19 downto 0);
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
    -- Source side: oscillator, line-buffer write, per-line values (two |dy|
    -- for the two circle centres at 540 -+ e, lattice row phase, tear)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_dy : signed(12 downto 0);
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
                s_ad1_n <= to_unsigned(540, 11) - resize(s_e, 11);
                s_ad2_n <= to_unsigned(540, 11) + resize(s_e, 11);
                s_py <= s_lph;
                s_l8 <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_tear_n <= (others => '0');
            elsif s_fall1 = '1' then
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
                -- |y - (540 - e)| and |y - (540 + e)|
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - 540 + signed(resize(s_e, 13));
                if v_dy < 0 then s_ad1_n <= resize(unsigned(-v_dy), 11); else s_ad1_n <= unsigned(v_dy(10 downto 0)); end if;
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - 540 - signed(resize(s_e, 13));
                if v_dy < 0 then s_ad2_n <= resize(unsigned(-v_dy), 11); else s_ad2_n <= unsigned(v_dy(10 downto 0)); end if;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_dy1 <= s_dy1_n;
                s_dy2 <= s_dy2_n;
                s_pyl <= s_py;
                if s_tear_n = 0 then s_rst <= (others => '0'); else s_rst <= s_wm1(10 downto 0) + 1 - s_tear_n; end if;
            end if;
        end if;
    end process p_src;

    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
                s_px <= (others => '0');
                s_q <= s_cx2sq;
                s_inc <= s_inc0;
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_px <= s_px + s_fxs;
                s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                s_inc <= s_inc + signed(resize(s_inc2, 15));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a, v_b : unsigned(11 downto 0);
        variable v_d : unsigned(22 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ri : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: lattice phases (two diagonals), dx^2 copy
            v_a := s_px + s_pyl; v_b := s_px - s_pyl;
            s_g1 <= v_a(11 downto 4); s_g2 <= v_b(11 downto 4);
            s_q1 <= s_q;
            s_rdd(1) <= s_rdp;

            -- 2: lattice line flag + pastel band (by position inside the line)
            s_lf2 <= '0'; s_lb2 <= "00";
            if resize(s_g1, 8) < resize(s_lw, 8) then
                s_lf2 <= '1';
                if resize(s_g1, 8) < resize(s_lw4, 8) then s_lb2 <= "00";
                elsif resize(s_g1, 8) < resize(s_lw2, 8) then s_lb2 <= "01";
                else s_lb2 <= "10"; end if;
            elsif resize(s_g2, 8) < resize(s_lw, 8) then
                s_lf2 <= '1';
                if resize(s_g2, 8) < resize(s_lw4, 8) then s_lb2 <= "00";
                elsif resize(s_g2, 8) < resize(s_lw2, 8) then s_lb2 <= "01";
                else s_lb2 <= "10"; end if;
            end if;
            s_qi2 <= s_q1(23 downto 2);
            s_rdd(2) <= s_rdd(1);

            -- 3: the two circles' r^2; video read issued
            s_r1_3 <= resize(s_qi2, 23) + resize(s_dy1, 23);
            s_r2_3 <= resize(s_qi2, 23) + resize(s_dy2, 23);
            s_lf(3) <= s_lf2; s_lbd(3) <= s_lb2;
            s_lb_ra <= s_rdd(2);

            -- 4: inside tests, max / min
            if s_r1_3 <= resize(s_r2, 23) then s_in1_4 <= '1'; else s_in1_4 <= '0'; end if;
            if s_r2_3 <= resize(s_r2, 23) then s_in2_4 <= '1'; else s_in2_4 <= '0'; end if;
            if s_r1_3 >= s_r2_3 then s_mx4 <= s_r1_3; s_mn4 <= s_r2_3; else s_mx4 <= s_r2_3; s_mn4 <= s_r1_3; end if;

            -- 5: eye / crescent, lens depth, crescent luma; video fabric copy
            s_eye5 <= s_in1_4 and s_in2_4;
            s_crs5 <= s_in1_4 xor s_in2_4;
            v_d := resize(s_r2, 23) - s_mx4;
            s_d5 <= v_d(21 downto 0);
            if s_mn4(22 downto 13) > 400 then s_cl5 <= to_unsigned(200, 10); else s_cl5 <= to_unsigned(600, 10) - s_mn4(22 downto 13); end if;
            s_y5 <= unsigned(s_lb_q(31 downto 22));
            s_u5 <= unsigned(s_lb_q(21 downto 12));
            s_v5 <= unsigned(s_lb_q(11 downto 2));

            -- 6: ring product
            s_pr6 <= s_d5(17 downto 10) * s_k3;
            s_eye6 <= s_eye5; s_crs6 <= s_crs5; s_cl6 <= s_cl5;
            s_y6 <= s_y5; s_u6 <= s_u5; s_v6 <= s_v5;

            -- 7: ring index etched by the luma
            v_ri := resize(s_pr6(15 downto 8), 9) + resize(s_y6(9 downto 5), 9);
            s_ri7 <= v_ri(7 downto 0);
            s_eye7 <= s_eye6; s_crs7 <= s_crs6; s_cl7 <= s_cl6;
            s_u7 <= s_u6; s_v7 <= s_v6;

            -- 8: compose
            if s_eye7 = '1' then
                if s_solid = '1' then
                    s_y8 <= to_unsigned(584, 10); s_u8 <= to_unsigned(772, 10); s_v8 <= to_unsigned(212, 10);
                else
                    case s_ri7(1 downto 0) is
                        when "00"   => s_y8 <= to_unsigned(528, 10); s_u8 <= to_unsigned(812, 10); s_v8 <= to_unsigned(244, 10);
                        when "01"   => s_y8 <= to_unsigned(584, 10); s_u8 <= to_unsigned(772, 10); s_v8 <= to_unsigned(212, 10);
                        when "10"   => s_y8 <= to_unsigned(728, 10); s_u8 <= to_unsigned(664, 10); s_v8 <= to_unsigned(128, 10);
                        when others => s_y8 <= to_unsigned(584, 10); s_u8 <= to_unsigned(772, 10); s_v8 <= to_unsigned(212, 10);
                    end case;
                end if;
            elsif s_crs7 = '1' and s_cres = '1' then
                s_y8 <= s_cl7; s_u8 <= C_CRS_U; s_v8 <= C_CRS_V;
            elsif s_lf(7) = '1' and s_lat = '1' then
                if s_tint = '1' then
                    s_y8 <= to_unsigned(760, 10); s_u8 <= s_u7; s_v8 <= s_v7;
                else
                    case s_lbd(7) is
                        when "00"   => s_y8 <= to_unsigned(696, 10); s_u8 <= to_unsigned(478, 10); s_v8 <= to_unsigned(623, 10);
                        when "01"   => s_y8 <= to_unsigned(736, 10); s_u8 <= to_unsigned(624, 10); s_v8 <= to_unsigned(540, 10);
                        when others => s_y8 <= to_unsigned(862, 10); s_u8 <= to_unsigned(544, 10); s_v8 <= to_unsigned(368, 10);
                    end case;
                end if;
            else
                s_y8 <= C_GRD_Y; s_u8 <= C_GRD_U; s_v8 <= C_GRD_V;
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
                s_yp(15) <= s_yf14 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_up(15) <= s_u14; s_vp(15) <= s_v14;
            else
                s_yp(15) <= to_unsigned(64, 10); s_up(15) <= to_unsigned(512, 10); s_vp(15) <= to_unsigned(512, 10);
            end if;
            for i in 16 to 20 loop s_yp(i) <= s_yp(i - 1); s_up(i) <= s_up(i - 1); s_vp(i) <= s_vp(i - 1); end loop;

            for i in 4 to 7 loop s_lf(i) <= s_lf(i - 1); s_lbd(i) <= s_lbd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_yp(20));
    data_out.u       <= std_logic_vector(s_up(20));
    data_out.v       <= std_logic_vector(s_vp(20));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture vesica;
