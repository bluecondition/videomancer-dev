-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Sinew v0.1.0 -- Heirloom #18 (2026-10-20), from WIN_20230315_00_53_56.
--
-- The photo: vertical rainbow bars (yellow / green / orange) at the top and
-- bottom, and across the middle band the same bars wrung into S-curves by a
-- vertical sine, with a fine row ripple on top.
--
-- The program: every line of the source (video, or the internal warm-bar
-- oscillator) is read back shifted sideways by A * sin(y), inside a centre
-- band (K2) or everywhere (S7), plus a per-line random ripple (K3); the
-- shift folds sign every 8 lines with S10 for an interlace-comb look.  The
-- result gets a hue rotation (K4), a contrast gain (K6) and an optional warm
-- tint (S9).  P12 is the wring amplitude.
--
--   K1  Period   vertical period of the sine
--   K2  Band     height of the wrung band
--   K3  Ripple   per-line random ripple
--   K4  Hue      hue rotation of the result
--   K5  Speed    sine travel, bipolar (centre = still)
--   K6  Contrast luma gain 0.25 .. 2x
--   S7  Band     Band only / All (the whole screen is wrung)
--   S8  Ripple   Off / On
--   S9  Tint     warm cast on / off
--   S10 Fold     the shift folds sign every 8 lines (combs)
--   S11 Source   Video / Osc (warm vertical bars)
--   P12 WRING    sine amplitude: flat .. +-500 virtual px
--
-- Generated avid; output conditioning before the blanking gate; virtual
-- 1080-line geometry (the shift is real pixels).  Latency 16 clocks.  16 EBR.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture sinew of program_top is
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

    -- quarter-wave sine, 64 entries over the period (signed, +-127)
    type t_sin is array (0 to 63) of integer range -128 to 127;
    constant C_SIN : t_sin := (
        0, 12, 25, 37, 49, 60, 71, 81, 90, 98, 106, 112, 117, 122, 125, 127,
        127, 127, 125, 122, 117, 112, 106, 98, 90, 81, 71, 60, 49, 37, 25, 12,
        0, -12, -25, -37, -49, -60, -71, -81, -90, -98, -106, -112, -117, -122, -125, -127,
        -127, -127, -125, -122, -117, -112, -106, -98, -90, -81, -71, -60, -49, -37, -25, -12);
    -- hue rotation: 16 steps of cos / sin
    type t_cs is array (0 to 15) of integer range -128 to 127;
    constant C_COS : t_cs := (127, 117, 90, 49, 0, -49, -90, -117, -127, -117, -90, -49, 0, 49, 90, 117);
    constant C_SINR : t_cs := (0, 49, 90, 117, 127, 117, 90, 49, 0, -49, -90, -117, -127, -117, -90, -49);
    -- warm bars for the oscillator: yellow / green / orange / red
    type t_col is array (0 to 2) of integer;
    type t_cols is array (0 to 3) of t_col;
    constant C_WARM : t_cols := ((840, 584, 64), (580, 136, 216), (584, 772, 212), (328, 960, 360));

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
    signal s_k1, s_k2, s_k3, s_k4, s_k6 : unsigned(7 downto 0) := (others => '0');
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_per : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_pys : unsigned(11 downto 0) := to_unsigned(64, 12);
    signal s_bh : unsigned(9 downto 0) := to_unsigned(200, 10);
    signal s_rk : unsigned(7 downto 0) := (others => '0');
    signal s_cos, s_sin : signed(7 downto 0) := (others => '0');
    signal s_gn : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_amp : unsigned(9 downto 0) := (others => '0');
    signal s_tph : unsigned(15 downto 0) := (others => '0');
    signal s_all, s_rip, s_tint, s_fold : std_logic := '0';
    -- serial multiplier
    signal s_ma : unsigned(19 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(19 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_lfsr : unsigned(15 downto 0) := x"5EED";

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1, s_fall2, s_fall3, s_fall4 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_sph_n : unsigned(15 downto 0) := (others => '0');
    signal s_sinv : signed(7 downto 0) := (others => '0');
    signal s_inb : std_logic := '0';
    signal s_dp : signed(18 downto 0) := (others => '0');
    signal s_rpl : signed(17 downto 0) := (others => '0');
    signal s_off : signed(12 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');

    -- beam
    signal s_rdp : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_du4, s_dv4, s_dy4 : signed(10 downto 0) := (others => '0');
    signal s_puc5, s_pus5, s_pvc5, s_pvs5 : signed(18 downto 0) := (others => '0');
    signal s_py5 : signed(19 downto 0) := (others => '0');
    signal s_u6, s_v6, s_y6 : signed(12 downto 0) := (others => '0');
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
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

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial product
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
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;
            v_s := shift_right(v_s, 2);
            if v_s > -2 and v_s < 2 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            s_k6 <= unsigned(registers_in(5)(9 downto 2));
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_per <= resize(s_k1(7 downto 1), 8) + 8;                -- 8 .. 135 phase / virtual line
                s_bh <= resize(s_k2, 10) + resize(s_k2, 10) + 40;        -- 40 .. 550 virtual lines
                s_rk <= s_k3;
                s_cos <= to_signed(C_COS(to_integer(s_k4(7 downto 4))), 8);
                s_sin <= to_signed(C_SINR(to_integer(s_k4(7 downto 4))), 8);
                s_gn <= resize(s_k6(7 downto 1), 8) + resize(s_k6, 8) + 32;   -- 32 .. 414 / 128
                s_amp <= s_g(11 downto 2);
                s_all  <= registers_in(6)(0);
                s_rip  <= registers_in(6)(1);
                s_tint <= registers_in(6)(2);
                s_fold <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_tph <= s_tph + unsigned(resize(shift_left(s_k5r, 4), 16));
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;

            if s_zc = 4 then
                s_ma <= resize(s_per, 20); s_mb <= s_ys; s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(18 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                s_pys <= s_macc(19 downto 8);
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
    -- Source side: line-buffer write (video / warm bars); per-line shift
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_dy : signed(12 downto 0);
        variable v_ad : unsigned(11 downto 0);
        variable v_ci : integer range 0 to 3;
        variable v_lum : unsigned(9 downto 0);
        variable v_rp : signed(8 downto 0);
        variable v_o : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- warm vertical bars: colour by bar index, luma shaded by a triangle, slow drift
            if data_in.avid = '1' then s_ox <= s_ox + 2; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 1; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_ci := to_integer(s_ox(8 downto 7) + s_ot(5 downto 4) + s_oy(9 downto 8));
            v_lum := to_unsigned(C_WARM(v_ci)(0), 10);
            s_oty <= v_lum - resize(f_tri8(s_ox(6 downto 0) & '0'), 10);
            s_otu <= to_unsigned(C_WARM(v_ci)(1), 10);
            s_otv <= to_unsigned(C_WARM(v_ci)(2), 10);
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
            s_fall2 <= s_fall1; s_fall3 <= s_fall2; s_fall4 <= s_fall3;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_sph_n <= s_tph;
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_sph_n <= s_sph_n + resize(s_pys, 16);
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            elsif s_fall2 = '1' then
                s_sinv <= to_signed(C_SIN(to_integer(s_sph_n(15 downto 10))), 8);
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - 540;
                if v_dy < 0 then v_ad := unsigned(resize(-v_dy, 12)); else v_ad := unsigned(v_dy(11 downto 0)); end if;
                if s_all = '1' or v_ad < resize(s_bh, 12) then s_inb <= '1'; else s_inb <= '0'; end if;
                v_rp := signed(resize(s_lfsr(7 downto 0), 9)) - 128;
                s_rpl <= v_rp * signed(resize(s_rk, 9));
            elsif s_fall3 = '1' then
                if s_inb = '1' then s_dp <= s_sinv * signed(resize(s_amp, 11)); else s_dp <= (others => '0'); end if;
            elsif s_fall4 = '1' then
                v_o := resize(shift_right(s_dp, 7), 13);
                if s_fold = '1' and s_vy_n(11) = '1' then v_o := -v_o; end if;
                if s_rip = '1' then v_o := v_o + resize(shift_right(s_rpl, 9), 13); end if;
                s_off <= v_o;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                -- read start = (-offset) mod width
                if s_off < 0 then
                    s_rst <= unsigned(resize(-s_off, 11));
                elsif resize(s_off, 13) >= signed(resize(s_wm1, 13)) then
                    s_rst <= (others => '0');
                elsif s_off = 0 then
                    s_rst <= (others => '0');
                else
                    s_rst <= s_wm1(10 downto 0) + 1 - unsigned(resize(s_off, 11));
                end if;
            end if;
        end if;
    end process p_src;

    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path: read, hue rotation, contrast, tint
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_u, v_v : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: read
            s_lb_ra <= s_rdp;
            -- 3: fabric copy
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));
            -- 4: differences
            s_du4 <= signed(resize(s_u3, 11)) - 512;
            s_dv4 <= signed(resize(s_v3, 11)) - 512;
            s_dy4 <= signed(resize(s_y3, 11)) - 512;
            -- 5: products
            s_puc5 <= s_du4 * s_cos;
            s_pus5 <= s_du4 * s_sin;
            s_pvc5 <= s_dv4 * s_cos;
            s_pvs5 <= s_dv4 * s_sin;
            s_py5 <= s_dy4 * signed(resize(s_gn, 9));
            -- 6: sums
            s_u6 <= resize(shift_right(s_puc5 - s_pvs5, 7), 13) + 512;
            s_v6 <= resize(shift_right(s_pus5 + s_pvc5, 7), 13) + 512;
            s_y6 <= resize(shift_right(s_py5, 7), 13) + 512;
            -- 7: tint, clamp
            v_u := s_u6; v_v := s_v6;
            if s_tint = '1' then v_u := v_u + 48; v_v := v_v - 32; end if;
            s_y7 <= f_clamp13(s_y6);
            s_u7 <= f_clamp13(v_u);
            s_v7 <= f_clamp13(v_v);

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
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture sinew;
