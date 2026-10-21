-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lotus v0.1.0 -- Heirloom #33 (2026-10-20), from WIN_20230515_23_23_36.
--
-- The photo: above a horizontal seam, orange / green nested rounded
-- diamonds; below it, pink-white lobes on acid green; two of each across.
--
-- The program: the screen is split at a seam (P12).  Above it every cell
-- holds nested diamonds (a|x| + b|y| bands) in an orange / green palette;
-- below it every cell holds a soft lobe (r^2 from the cell centre: white
-- core, pink body, green ground).  The video (or the oscillator) etches the
-- diamonds and swells the lobes.
--
--   K1  Tiles    cells across, 1 .. 4
--   K2  Rings    diamond ring density
--   K3  Lobes    lobe size
--   K4  Hue      Orange/Green, Blue/Pink, Red/Cyan, Mono
--   K5  Drift    diamond rings flow outward / inward, bipolar
--   K6  Tear     sideways tearing of the video in 8-line bands
--   S7  Zones    Diamonds above / Lobes above
--   S8  Invert   palettes inverted
--   S9  Rings    Hard / Soft (shaded) rings
--   S10 Grid     dark lines on the cell edges and the seam
--   S11 Source   Video / Osc
--   P12 SEAM     the seam: all lobes .. all diamonds
--
-- Virtual 1080-line geometry; cell widths from serial divides, dx^2 per cell
-- by second differences restarted at every cell, per-line squares and
-- products on the serial unit.  Generated avid; output conditioning before
-- the gate.  Latency 20 clocks.  16 EBR.  Colours: u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture lotus of program_top is
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

    -- 4 hues x (diamond bands 0..3, lobe core, lobe body, ground) {y, u = Cr, v = Cb}
    type t_col is array (0 to 2) of integer;
    type t_pdef is array (0 to 31) of t_col;
    constant C_PDEF : t_pdef := (
        (528, 812, 244), (580, 136, 216), (728, 664, 128), (300, 300, 300), (940, 512, 512), (752, 612, 512), (716, 336, 214), (64, 512, 512),   -- orange/green, white, pink, acid green
        (164, 440, 960), (752, 612, 512), (678, 64, 663),  (349, 756, 852), (940, 512, 512), (426, 887, 808), (164, 440, 960), (64, 512, 512),   -- blue/pink, white, magenta, blue
        (328, 960, 360), (678, 64, 663),  (840, 584, 64),  (192, 608, 696), (940, 512, 512), (584, 772, 212), (120, 470, 700), (64, 512, 512),   -- red/cyan, white, orange, navy
        (260, 512, 512), (700, 512, 512), (480, 512, 512), (940, 512, 512), (940, 512, 512), (600, 512, 512), (200, 512, 512), (64, 512, 512));  -- mono
    type t_pal is array (0 to 31) of std_logic_vector(29 downto 0);
    function f_mkpal return t_pal is
        variable r : t_pal;
    begin
        for i in 0 to 31 loop
            r(i) := std_logic_vector(to_unsigned(C_PDEF(i)(0), 10)) & std_logic_vector(to_unsigned(C_PDEF(i)(1), 10))
                    & std_logic_vector(to_unsigned(C_PDEF(i)(2), 10));
        end loop;
        return r;
    end function;
    constant C_PAL : t_pal := f_mkpal;

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
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_j1 : unsigned(1 downto 0) := "01";
    signal s_zk : unsigned(7 downto 0) := to_unsigned(32, 8);
    signal s_k3 : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_j4 : unsigned(1 downto 0) := (others => '0');
    signal s_live4 : std_logic := '0';
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_tk : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- latched / derived
    signal s_n : unsigned(1 downto 0) := "01";
    signal s_hue : unsigned(1 downto 0) := (others => '0');
    signal s_swapz, s_inv, s_soft, s_grid : std_logic := '0';
    signal s_cwpx, s_cwv : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_cwpxm1 : unsigned(10 downto 0) := to_unsigned(959, 11);
    signal s_cwv2 : unsigned(12 downto 0) := to_unsigned(1920, 13);
    signal s_a : unsigned(11 downto 0) := to_unsigned(128, 12);
    signal s_cw2sq : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_seam : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_ctop, s_cbot : unsigned(10 downto 0) := (others => '0');
    signal s_l2 : unsigned(9 downto 0) := (others => '0');
    signal s_l1 : unsigned(9 downto 0) := (others => '0');
    signal s_ph : unsigned(23 downto 0) := (others => '0');
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
    signal s_lfsr : unsigned(15 downto 0) := x"5EED";

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1, s_fall2, s_fall3 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_ft_n, s_fb_n : unsigned(10 downto 0) := (others => '0');
    signal s_fvb_n, s_fvb : unsigned(23 downto 0) := (others => '0');
    signal s_dy2_n, s_dy2 : unsigned(21 downto 0) := (others => '0');
    signal s_above_n, s_above, s_seaml_n, s_seaml : std_logic := '0';
    signal s_l8 : unsigned(2 downto 0) := (others => '0');
    signal s_tear_n, s_tear : unsigned(10 downto 0) := (others => '0');

    -- beam
    signal s_cu : unsigned(12 downto 0) := (others => '0');
    signal s_cupx, s_cdpx, s_cs : unsigned(10 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');
    signal s_cnew : std_logic := '0';

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_fu1 : unsigned(12 downto 0) := (others => '0');
    signal s_m1, s_cs1 : unsigned(10 downto 0) := (others => '0');
    signal s_q1 : unsigned(23 downto 0) := (others => '0');
    signal s_edge : t_sl(1 to 9) := (others => '0');
    signal s_plo2 : unsigned(20 downto 0) := (others => '0');
    signal s_phi2 : unsigned(16 downto 0) := (others => '0');
    signal s_rd2 : unsigned(10 downto 0) := (others => '0');
    signal s_qi2 : unsigned(21 downto 0) := (others => '0');
    signal s_pa3 : unsigned(24 downto 0) := (others => '0');
    signal s_rd3 : unsigned(11 downto 0) := (others => '0');
    signal s_r2_3 : unsigned(22 downto 0) := (others => '0');
    signal s_s4 : unsigned(25 downto 0) := (others => '0');
    signal s_r2s : t_u11(4 to 7) := (others => (others => '0'));
    signal s_s5, s_s6 : unsigned(25 downto 0) := (others => '0');
    signal s_y6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_m7 : unsigned(15 downto 0) := (others => '0');
    signal s_s7 : unsigned(25 downto 0) := (others => '0');
    signal s_y7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_s8 : unsigned(25 downto 0) := (others => '0');
    signal s_r2e8 : unsigned(10 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_sh9 : unsigned(6 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y11, s_y12, s_y13, s_y14, s_y15, s_y16x, s_yf16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yp : t_u10(17 to 20) := (others => to_unsigned(64, 10));
    signal s_up, s_vp : t_u10(17 to 20) := (others => to_unsigned(512, 10));

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
        variable v_r : unsigned(12 downto 0);
        variable v_i : signed(15 downto 0);
        variable v_ph : signed(24 downto 0);
        variable v_p : unsigned(11 downto 0);
        variable v_k3 : unsigned(8 downto 0);
        variable v_cb : unsigned(11 downto 0);
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

            -- K1 Tiles (4 zones, +-12 count deadband after warm-up)
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_j1 <= v_k(9 downto 8);
            end if;
            s_zk <= resize(unsigned(registers_in(1)(9 downto 3)), 8) + 4;      -- K2 Rings 4 .. 131
            v_k3 := resize(unsigned(registers_in(2)(9 downto 2)), 9) + 16;      -- K3 Lobes 16 .. 255
            if v_k3 > 255 then s_k3 <= (others => '1'); else s_k3 <= v_k3(7 downto 0); end if;
            v_k := unsigned(registers_in(3));                                  -- K4 Hue (4 labels)
            if v_k >= 4 then s_live4 <= '1'; end if;
            if s_live4 = '0' and v_k < 4 then s_j4 <= v_k(1 downto 0); else s_j4 <= v_k(9 downto 8); end if;
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;        -- K5 Drift
            if v_s > -8 and v_s < 8 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            s_tk <= unsigned(registers_in(5)(9 downto 2));                     -- K6 Tear
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_n <= s_j1;
                s_hue <= s_j4;
                s_swapz <= registers_in(6)(0);
                s_inv   <= registers_in(6)(1);
                s_soft  <= registers_in(6)(2);
                s_grid  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                v_ph := signed(resize(s_ph, 25)) + shift_left(resize(s_k5r, 25), 6);
                s_ph <= unsigned(v_ph(23 downto 0));
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            -- seam 60 + 0.9 p12 (virtual lines); zone centres
            v_p := to_unsigned(60, 12) + resize(s_g(11 downto 2), 12) - resize(s_g(11 downto 6), 12) - resize(s_g(11 downto 7), 12);
            s_seam <= v_p(10 downto 0);
            s_ctop <= '0' & s_seam(10 downto 1);
            v_cb := resize(s_seam, 12) + 1080;
            s_cbot <= v_cb(11 downto 1);
            s_a <= shift_left(resize(s_zk, 12), 2);
            s_cwpxm1 <= s_cwpx(10 downto 0) - 1;
            s_cwv2 <= s_cwv & '0';
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_cwv, 16), 3)); s_inc2 <= to_unsigned(32, 6);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_cwv, 16), 2) + shift_left(resize(s_cwv, 16), 1)); s_inc2 <= to_unsigned(18, 6);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_cwv, 16), 2)); s_inc2 <= to_unsigned(8, 6);
            end case;
            s_inc0 <= v_i(14 downto 0);
            s_l1 <= '0' & s_l2(9 downto 1);

            -- serial unit: divides 0 w/N, 1 vw/N; products 2 cwv^2, 3 k3^2, 4 ft x a (line), 5 fb^2 (line)
            if s_zc = 4 then
                s_dnum <= s_w; s_drem <= (others => '0'); s_mb <= resize(s_n, 12) + 1;
                s_mjob <= "000"; s_mdiv <= '1'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 24 then
                s_dnum <= s_vw; s_drem <= (others => '0'); s_mb <= resize(s_n, 12) + 1;
                s_mjob <= "001"; s_mdiv <= '1'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 44 then
                s_ma <= resize(s_cwv, 24); s_mb <= s_cwv; s_macc <= (others => '0');
                s_mjob <= "010"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 64 then
                s_ma <= resize(s_k3, 24); s_mb <= resize(s_k3, 12); s_macc <= (others => '0');
                s_mjob <= "011"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 84 or s_fall2 = '1' then
                s_ma <= resize(s_ft_n, 24); s_mb <= s_a; s_macc <= (others => '0');
                s_mjob <= "100"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 104 or s_fall3 = '1' then
                s_ma <= resize(s_fb_n, 24); s_mb <= resize(s_fb_n, 12); s_macc <= (others => '0');
                s_mjob <= "101"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
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
                    when "010"  => s_cw2sq <= s_macc;                 -- (cwv)^2 = (2 * cwv/2)^2, Q.2 of the half width
                    when "011"  => s_l2 <= s_macc(15 downto 6);
                    when "100"  => s_fvb_n <= s_macc;
                    when others => s_dy2_n <= s_macc(21 downto 0);
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
    -- Source side: oscillator, line-buffer write, per-line zone / distances
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_d : signed(12 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_tr : unsigned(10 downto 0);
        variable v_vy : unsigned(11 downto 0);
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
                s_ft_n <= s_ctop;
                s_fb_n <= s_cbot;
                s_above_n <= '1';
                s_seaml_n <= '0';
                s_l8 <= (others => '0');
                s_tear_n <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_l8 <= s_l8 + 1;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                if s_l8 = 7 then
                    v_t := s_lfsr(7 downto 0) * s_tk;
                    v_tr := resize(v_t(15 downto 6), 11);
                    if v_tr >= s_wm1(10 downto 0) then s_tear_n <= s_wm1(10 downto 0); else s_tear_n <= v_tr; end if;
                end if;
            elsif s_fall2 = '1' then
                v_vy := s_vy_n(19 downto 8);
                if v_vy < resize(s_seam, 12) then s_above_n <= not s_swapz; else s_above_n <= s_swapz; end if;
                if v_vy >= resize(s_seam, 12) and v_vy < resize(s_seam, 12) + 4 then s_seaml_n <= '1'; else s_seaml_n <= '0'; end if;
                v_d := signed(resize(v_vy, 13)) - signed(resize(s_ctop, 13));
                if v_d < 0 then s_ft_n <= resize(unsigned(-v_d), 11); else s_ft_n <= unsigned(v_d(10 downto 0)); end if;
                v_d := signed(resize(v_vy, 13)) - signed(resize(s_cbot, 13));
                if v_d < 0 then s_fb_n <= resize(unsigned(-v_d), 11); else s_fb_n <= unsigned(v_d(10 downto 0)); end if;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_fvb <= s_fvb_n;
                s_dy2 <= s_dy2_n;
                s_above <= s_above_n;
                s_seaml <= s_seaml_n;
                s_tear <= s_tear_n;
            end if;
        end if;
    end process p_src;

    p_beam : process(clk)
        variable v_n : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            s_cnew <= '0';
            if g_start = '1' then
                s_cu <= (others => '0');
                s_cupx <= (others => '0');
                s_cdpx <= s_cwpxm1;
                s_cs <= (others => '0');
                s_q <= s_cw2sq;
                s_inc <= s_inc0;
                s_cnew <= '1';
            elsif g_avid = '1' then
                v_n := resize(s_cu, 14) + s_xs;
                if v_n >= s_cwv2 then
                    s_cu <= resize(v_n - s_cwv2, 13);
                    s_q <= s_cw2sq;
                    s_inc <= s_inc0;
                    s_cnew <= '1';
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
        variable v_f : signed(13 downto 0);
        variable v_r : unsigned(11 downto 0);
        variable v_e : unsigned(11 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_pi : integer range 0 to 31;
        variable v_ci : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: |x| about the cell centre, mirrored read offset, dx^2 copy, cell edge
            v_f := signed(resize(s_cu, 14)) - signed(resize(s_cwv, 14));
            if v_f < 0 then s_fu1 <= resize(unsigned(-v_f), 13); else s_fu1 <= unsigned(v_f(12 downto 0)); end if;
            if s_cupx < s_cdpx then s_m1 <= s_cupx; else s_m1 <= s_cdpx; end if;
            s_cs1 <= s_cs;
            s_q1 <= s_q;
            if s_cupx < 4 then s_edge(1) <= s_grid; else s_edge(1) <= '0'; end if;

            -- 2: a|x| partials; read address; dx^2 integer
            s_plo2 <= s_fu1 * s_a(7 downto 0);
            s_phi2 <= s_fu1 * s_a(11 downto 8);
            s_rd2 <= s_cs1 + s_m1;
            s_qi2 <= s_q1(23 downto 2);

            -- 3: a|x|; torn read address; r^2
            s_pa3 <= resize(s_plo2, 25) + shift_left(resize(s_phi2, 25), 8);
            s_rd3 <= resize(s_rd2, 12) + resize(s_tear, 12);
            s_r2_3 <= resize(s_qi2, 23) + resize(s_dy2, 23);

            -- 4: field; lobe radius scale; read (wrapped)
            s_s4 <= resize(s_pa3, 26) + resize(s_fvb, 26);
            v_r := s_r2_3(22 downto 11);
            if v_r > 2047 then s_r2s(4) <= (others => '1'); else s_r2s(4) <= v_r(10 downto 0); end if;
            if s_rd3 >= s_w then s_lb_ra <= resize(s_rd3 - s_w, 11); else s_lb_ra <= s_rd3(10 downto 0); end if;

            -- 5: drift
            s_s5 <= s_s4 + resize(s_ph, 26);

            -- 6: video luma (fabric copy)
            s_y6 <= unsigned(s_lb_q(31 downto 22));
            s_s6 <= s_s5;

            -- 7: etch product
            if s_y6 > 64 then s_m7 <= (s_y6 - 64) * to_unsigned(48, 6); else s_m7 <= (others => '0'); end if;
            s_s7 <= s_s6; s_y7 <= s_y6;

            -- 8: etched field (above) / swollen lobe radius (below)
            s_s8 <= s_s7 + resize(s_m7, 26);
            v_e := resize(s_r2s(7), 12);
            if v_e > resize(s_y7(9 downto 3), 12) then s_r2e8 <= resize(v_e - resize(s_y7(9 downto 3), 12), 11); else s_r2e8 <= (others => '0'); end if;

            -- 9: colour index
            if s_above = '1' then
                v_ci := '0' & s_s8(15 downto 14);
                s_sh9 <= s_s8(13 downto 7);
            else
                if s_r2e8 < s_l1 then v_ci := "100";
                elsif s_r2e8 < s_l2 then v_ci := "101";
                else v_ci := "110"; end if;
                s_sh9 <= (others => '0');
            end if;
            if s_inv = '1' then v_ci(1 downto 0) := not v_ci(1 downto 0); end if;
            if s_edge(8) = '1' or (s_seaml = '1' and s_grid = '1') then v_ci := "111"; end if;
            v_pi := to_integer(s_hue & v_ci);
            s_y9 <= unsigned(C_PAL(v_pi)(29 downto 20));
            s_u9 <= unsigned(C_PAL(v_pi)(19 downto 10));
            s_v9 <= unsigned(C_PAL(v_pi)(9 downto 0));

            -- 10: soft shading of the rings
            if s_soft = '1' and s_y9 > 300 then s_y10 <= s_y9 - resize(s_sh9 & '0', 10); else s_y10 <= s_y9; end if;
            s_u10 <= s_u9; s_v10 <= s_v9;

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
                s_yp(17) <= s_yf16 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_up(17) <= s_u16; s_vp(17) <= s_v16;
            else
                s_yp(17) <= to_unsigned(64, 10); s_up(17) <= to_unsigned(512, 10); s_vp(17) <= to_unsigned(512, 10);
            end if;
            for i in 18 to 20 loop s_yp(i) <= s_yp(i - 1); s_up(i) <= s_up(i - 1); s_vp(i) <= s_vp(i - 1); end loop;

            for i in 5 to 7 loop s_r2s(i) <= s_r2s(i - 1); end loop;
            for i in 2 to 9 loop s_edge(i) <= s_edge(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_yp(20));
    data_out.u       <= std_logic_vector(s_up(20));
    data_out.v       <= std_logic_vector(s_vp(20));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lotus;
