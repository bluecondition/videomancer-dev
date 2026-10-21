-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Greenroom v0.1.0 -- Heirloom #54 (2026-10-20), from television.jpg (a
-- painted paper collage).
--
-- The collage: green and teal washes scraped over magazine cuttings, orange
-- splatters, a clock, and in the middle a television set showing a face.
--
-- The program: a painterly field (three triangle-wave octaves through a
-- green / teal palette, orange where the field peaks) with a rounded
-- television window in the middle showing the video (or the oscillator)
-- behind a dark bezel.  P12 tunes the set from a small screen to the whole
-- picture.
--
--   K1  Splatter  how much of the field turns orange (0 = none)
--   K2  Scale     brush scale of the field
--   K3  Contrast  field contrast
--   K4  Bezel     bezel width
--   K5  Drift     the field drifts, bipolar (centre = still)
--   K6  Position  window position, left .. right
--   S7  Window    TV (video inside the set) / Full (video everywhere, paint over it)
--   S8  Splatter  Orange / Palette
--   S9  Palette   Green / Blue
--   S10 Wash      video washed green / clean
--   S11 Source    Video / Osc
--   P12 TUNE      window size
--
-- Virtual 1080-line geometry.  Window = rectangle with radius-48 corners
-- (a 64-entry quarter-circle ROM).  Generated avid; output conditioning
-- before the gate.  Latency 20.  EBR: 16 line buffer + 4 palette.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture greenroom of program_top is
    constant C_LAT : integer := 20;
    constant C_CR : integer := 48;    -- corner radius, virtual px

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

    -- palettes A (green) and B (blue)
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_KA : t_keys := ((0, 150, 430, 440), (90, 193, 418, 437), (160, 372, 288, 588), (220, 580, 136, 216), (255, 700, 200, 250));
    constant C_KB : t_keys := ((0, 120, 470, 560), (90, 164, 440, 960), (160, 372, 288, 588), (220, 349, 756, 852), (255, 678, 64, 663));
    type t_pal is array (0 to 511) of std_logic_vector(31 downto 0);
    function f_lerp(a, b, t, n : integer) return integer is
    begin
        return a + ((b - a) * t) / n;
    end function;
    function f_pal return t_pal is
        variable r : t_pal;
        variable y, u, v : integer;
    begin
        for i in 0 to 255 loop
            for k in 0 to C_KA'length - 2 loop
                if i >= C_KA(k).i and i <= C_KA(k + 1).i then
                    y := f_lerp(C_KA(k).y, C_KA(k + 1).y, i - C_KA(k).i, C_KA(k + 1).i - C_KA(k).i);
                    u := f_lerp(C_KA(k).u, C_KA(k + 1).u, i - C_KA(k).i, C_KA(k + 1).i - C_KA(k).i);
                    v := f_lerp(C_KA(k).v, C_KA(k + 1).v, i - C_KA(k).i, C_KA(k + 1).i - C_KA(k).i);
                end if;
            end loop;
            r(i) := std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
                    & std_logic_vector(to_unsigned(v, 10)) & "00";
            for k in 0 to C_KB'length - 2 loop
                if i >= C_KB(k).i and i <= C_KB(k + 1).i then
                    y := f_lerp(C_KB(k).y, C_KB(k + 1).y, i - C_KB(k).i, C_KB(k + 1).i - C_KB(k).i);
                    u := f_lerp(C_KB(k).u, C_KB(k + 1).u, i - C_KB(k).i, C_KB(k + 1).i - C_KB(k).i);
                    v := f_lerp(C_KB(k).v, C_KB(k + 1).v, i - C_KB(k).i, C_KB(k + 1).i - C_KB(k).i);
                end if;
            end loop;
            r(256 + i) := std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
                          & std_logic_vector(to_unsigned(v, 10)) & "00";
        end loop;
        return r;
    end function;
    signal pal : t_pal := f_pal;
    signal s_pal_ra : unsigned(8 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    -- quarter circle: half height of a radius-48 corner at offset ax
    type t_corner is array (0 to 63) of unsigned(5 downto 0);
    function f_corner return t_corner is
        variable r : t_corner;
        variable v : integer;
    begin
        for i in 0 to 63 loop
            if i >= C_CR then
                v := 0;
            else
                v := integer(sqrt(real(C_CR * C_CR - i * i)));
            end if;
            r(i) := to_unsigned(v, 6);
        end loop;
        return r;
    end function;
    constant C_CORNER : t_corner := f_corner;

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
    signal s_k1 : unsigned(7 downto 0) := (others => '0');
    signal s_k2 : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal s_k3 : unsigned(7 downto 0) := (others => '0');
    signal s_k4 : unsigned(6 downto 0) := (others => '0');
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_thr : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal s_fx1, s_fx2, s_fx3 : unsigned(9 downto 0) := (others => '0');   -- per-pixel phase steps
    signal s_fy1, s_fy2, s_fy3 : unsigned(9 downto 0) := (others => '0');   -- per-line phase steps
    signal s_fa, s_fb, s_fc : unsigned(6 downto 0) := to_unsigned(4, 7);
    signal s_con : unsigned(7 downto 0) := to_unsigned(160, 8);
    signal s_bz : unsigned(6 downto 0) := (others => '0');
    signal s_t : unsigned(7 downto 0) := (others => '0');
    signal s_ww, s_hh : unsigned(10 downto 0) := to_unsigned(400, 11);     -- half width / height
    signal s_wwb, s_hhb : unsigned(10 downto 0) := to_unsigned(420, 11);   -- with the bezel
    signal s_wc, s_hc, s_wcb, s_hcb : signed(11 downto 0) := (others => '0');   -- minus the corner radius
    signal s_cx : signed(12 downto 0) := to_signed(960, 13);
    signal s_full, s_sorange, s_blue, s_wash : std_logic := '0';
    -- serial multiplier
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
    signal s_fall1 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_py1, s_py2, s_py3 : unsigned(11 downto 0) := (others => '0');
    signal s_tp1_n, s_tp2_n, s_tp3_n, s_tp1, s_tp2, s_tp3 : unsigned(7 downto 0) := (others => '0');
    signal s_ady_n, s_ady : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_ay_n, s_ayb_n, s_ay, s_ayb : signed(11 downto 0) := (others => '0');
    signal s_iny_n, s_inyb_n, s_iny, s_inyb : std_logic := '0';

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_dxh : signed(12 downto 0) := (others => '0');
    signal s_px1, s_px2, s_px3 : unsigned(11 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_adx1 : unsigned(10 downto 0) := (others => '0');
    signal s_a1, s_a2 : unsigned(7 downto 0) := (others => '0');
    signal s_b3 : unsigned(6 downto 0) := (others => '0');
    signal s_rdd : t_u11(1 to 6) := (others => (others => '0'));
    signal s_ax2, s_axb2 : signed(11 downto 0) := (others => '0');
    signal s_inx2, s_inxb2 : std_logic := '0';
    signal s_o1, s_o2 : unsigned(6 downto 0) := (others => '0');
    signal s_a3 : unsigned(7 downto 0) := (others => '0');
    signal s_ci3, s_cib3 : unsigned(5 downto 0) := (others => '0');
    signal s_cz3, s_czb3, s_inx3, s_inxb3 : std_logic := '0';
    signal s_o12 : unsigned(7 downto 0) := (others => '0');
    signal s_o3 : unsigned(6 downto 0) := (others => '0');
    signal s_ch4, s_chb4 : unsigned(5 downto 0) := (others => '0');
    signal s_cz4, s_czb4, s_inx4, s_inxb4 : std_logic := '0';
    signal s_n4 : unsigned(8 downto 0) := (others => '0');
    signal s_in5, s_inb5 : std_logic := '0';
    signal s_np5 : unsigned(16 downto 0) := (others => '0');
    signal s_in, s_inb : t_sl(6 to 10) := (others => '0');
    signal s_idx6 : unsigned(7 downto 0) := (others => '0');
    signal s_spl : t_sl(7 to 10) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py9, s_pu9, s_pv9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y11, s_y12, s_y13, s_y14, s_y15x, s_yf15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y16, s_y17, s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16, s_u17, s_v17, s_u18, s_v18, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- RAMs
    ---------------------------------------------------------------------------
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
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_p : unsigned(12 downto 0);
        variable v_c : signed(13 downto 0);
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

            s_k1 <= unsigned(registers_in(0)(9 downto 2));                     -- K1 Splatter
            s_k2 <= unsigned(registers_in(1)(9 downto 5));                     -- K2 Scale 0..31
            s_k3 <= unsigned(registers_in(2)(9 downto 2));                     -- K3 Contrast
            s_k4 <= unsigned(registers_in(3)(9 downto 3));                     -- K4 Bezel 0..127
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;        -- K5 Drift, deadband
            v_s := shift_right(v_s, 5);
            if v_s > -1 and v_s < 1 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            s_k6 <= unsigned(registers_in(5));                                 -- K6 Position
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_thr <= not s_k1;                                            -- 255 = never .. 0 = all
                s_fa <= resize(s_k2, 7) + 2;                                  -- 2 .. 33
                s_fb <= resize(s_k2 & '0', 7) + 3;                            -- 3 .. 65
                s_fc <= resize(s_k2(4 downto 1), 7) + 1;                      -- 1 .. 16
                s_con <= to_unsigned(64, 8) + resize(s_k3(7 downto 1), 8) + resize(s_k3(7 downto 2), 8);
                s_bz <= s_k4;
                s_t <= s_t + unsigned(resize(s_k5r, 8));
                s_full    <= registers_in(6)(0);
                s_sorange <= not registers_in(6)(1);
                s_blue    <= registers_in(6)(2);
                s_wash    <= not registers_in(6)(3);
                s_src     <= registers_in(6)(4);
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
            -- phase steps per pixel (fa * xs etc.) and per line
            case to_integer(s_xs) is
                when 4      => s_fx1 <= shift_left(resize(s_fa, 10), 2); s_fx2 <= shift_left(resize(s_fb, 10), 2); s_fx3 <= shift_left(resize(s_fc, 10), 2);
                when 3      => s_fx1 <= shift_left(resize(s_fa, 10), 1) + resize(s_fa, 10); s_fx2 <= shift_left(resize(s_fb, 10), 1) + resize(s_fb, 10); s_fx3 <= shift_left(resize(s_fc, 10), 1) + resize(s_fc, 10);
                when others => s_fx1 <= shift_left(resize(s_fa, 10), 1); s_fx2 <= shift_left(resize(s_fb, 10), 1); s_fx3 <= shift_left(resize(s_fc, 10), 1);
            end case;
            -- window: half width 100 + 7/8 p12 (100 .. 995), half height 9/16 of it
            v_p := to_unsigned(100, 13) + resize(s_g(11 downto 2), 13) - resize(s_g(11 downto 5), 13);
            s_ww <= v_p(10 downto 0);
            s_hh <= resize(s_ww(10 downto 1), 11) + resize(s_ww(10 downto 4), 11);
            s_wwb <= s_ww + resize(s_bz, 11);
            s_hhb <= s_hh + resize(s_bz, 11);
            s_wc <= signed(resize(s_ww, 12)) - C_CR;
            s_hc <= signed(resize(s_hh, 12)) - C_CR;
            s_wcb <= signed(resize(s_wwb, 12)) - C_CR;
            s_hcb <= signed(resize(s_hhb, 12)) - C_CR;
            -- centre x: vw/2 + (k6 - 512) * 3/4
            v_c := signed(resize(s_vw(11 downto 1), 14)) + (signed(resize(s_k6, 14)) - 512) - shift_right(signed(resize(s_k6, 14)) - 512, 2);
            s_cx <= v_c(12 downto 0);

            -- serial products: per-line steps fa*ys etc. (>> 8)
            if s_zc = 4 then
                s_ma <= resize(s_fa, 24); s_mb <= s_ys; s_macc <= (others => '0'); s_mjob <= "00"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 20 then
                s_ma <= resize(s_fb, 24); s_mb <= s_ys; s_macc <= (others => '0'); s_mjob <= "01"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 36 then
                s_ma <= resize(s_fc, 24); s_mb <= s_ys; s_macc <= (others => '0'); s_mjob <= "10"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "00"   => s_fy1 <= s_macc(17 downto 8);
                    when "01"   => s_fy2 <= s_macc(17 downto 8);
                    when others => s_fy3 <= s_macc(17 downto 8);
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
    -- Source side: line-buffer write; per-line values
    ---------------------------------------------------------------------------
    p_src : process(clk)
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

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_py1 <= (others => '0'); s_py2 <= (others => '0'); s_py3 <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_py1 <= s_py1 + s_fy1; s_py2 <= s_py2 + s_fy2; s_py3 <= s_py3 + s_fy3;
            end if;
            -- next line's values (settle during blanking)
            v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - 540;
            if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11); else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
            s_tp1_n <= resize(f_tri8(s_py1(10 downto 3)), 8);
            s_tp2_n <= s_t - s_py2(10 downto 3);
            s_tp3_n <= s_py3(10 downto 3);
            s_ay_n <= signed(resize(s_ady_n, 12)) - s_hc;
            s_ayb_n <= signed(resize(s_ady_n, 12)) - s_hcb;
            if s_ady_n <= s_hh then s_iny_n <= '1'; else s_iny_n <= '0'; end if;
            if s_ady_n <= s_hhb then s_inyb_n <= '1'; else s_inyb_n <= '0'; end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_ady <= s_ady_n;
                s_tp1 <= s_tp1_n; s_tp2 <= s_tp2_n; s_tp3 <= s_tp3_n;
                s_ay <= s_ay_n; s_ayb <= s_ayb_n;
                s_iny <= s_iny_n; s_inyb <= s_inyb_n;
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
                s_dxh <= -s_cx;
                s_px1 <= (others => '0'); s_px2 <= (others => '0'); s_px3 <= (others => '0');
            elsif g_avid = '1' then
                s_rdp <= s_rdp + 1;
                s_dxh <= s_dxh + signed(resize(s_xs, 13));
                s_px1 <= s_px1 + s_fx1; s_px2 <= s_px2 + s_fx2; s_px3 <= s_px3 + s_fx3;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ad : signed(12 downto 0);
        variable v_i : unsigned(8 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_in, v_inb : std_logic;
        variable v_wu, v_wv : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: |dx| (half units -> units); phases
            if s_dxh < 0 then v_ad := -s_dxh; else v_ad := s_dxh; end if;
            s_adx1 <= unsigned(v_ad(11 downto 1));
            s_a1 <= s_px1(10 downto 3) + s_tp1;
            s_a2 <= s_px2(10 downto 3) + s_tp2;
            s_b3 <= f_tri8(s_px3(10 downto 3) + s_t);
            s_rdd(1) <= s_rdp;

            -- 2: corner offsets, x-in-rectangle; octaves 1, 2
            s_ax2 <= signed(resize(s_adx1, 12)) - s_wc;
            s_axb2 <= signed(resize(s_adx1, 12)) - s_wcb;
            if s_adx1 <= s_ww then s_inx2 <= '1'; else s_inx2 <= '0'; end if;
            if s_adx1 <= s_wwb then s_inxb2 <= '1'; else s_inxb2 <= '0'; end if;
            s_o1 <= f_tri8(s_a1);
            s_o2 <= f_tri8(s_a2);
            s_a3 <= s_tp3 + resize(s_b3, 8);

            -- 3: corner ROM index (0..63), corner-zone flags; octave sums
            if s_ax2 <= 0 then s_ci3 <= (others => '0'); s_cz3 <= '0';
            elsif s_ax2 >= 63 then s_ci3 <= (others => '1'); s_cz3 <= '1';
            else s_ci3 <= unsigned(s_ax2(5 downto 0)); s_cz3 <= '1'; end if;
            if s_axb2 <= 0 then s_cib3 <= (others => '0'); s_czb3 <= '0';
            elsif s_axb2 >= 63 then s_cib3 <= (others => '1'); s_czb3 <= '1';
            else s_cib3 <= unsigned(s_axb2(5 downto 0)); s_czb3 <= '1'; end if;
            s_inx3 <= s_inx2; s_inxb3 <= s_inxb2;
            s_o12 <= resize(s_o1, 8) + resize(s_o2, 8);
            s_o3 <= f_tri8(s_a3);

            -- 4: corner half heights; field sum
            s_ch4 <= C_CORNER(to_integer(s_ci3));
            s_chb4 <= C_CORNER(to_integer(s_cib3));
            s_cz4 <= s_cz3; s_czb4 <= s_czb3; s_inx4 <= s_inx3; s_inxb4 <= s_inxb3;
            s_n4 <= resize(s_o12, 9) + resize(s_o3, 9);

            -- 5: inside tests (rectangle, and in the corner zone ay <= corner height); contrast product
            v_in := s_inx4 and s_iny;
            if s_cz4 = '1' and s_ay > 0 and s_ay > signed(resize(s_ch4, 12)) then v_in := '0'; end if;
            v_inb := s_inxb4 and s_inyb;
            if s_czb4 = '1' and s_ayb > 0 and s_ayb > signed(resize(s_chb4, 12)) then v_inb := '0'; end if;
            s_in5 <= v_in; s_inb5 <= v_inb;
            s_np5 <= s_n4 * s_con;

            -- 6: field index (saturated)
            if s_np5(16) = '1' then s_idx6 <= (others => '1'); else s_idx6 <= s_np5(15 downto 8); end if;
            s_in(6) <= s_in5; s_inb(6) <= s_inb5;

            -- 7: palette address, splatter flag, video read
            s_pal_ra <= s_blue & s_idx6;
            if s_idx6 >= s_thr then s_spl(7) <= '1'; else s_spl(7) <= '0'; end if;
            s_lb_ra <= s_rdd(6);

            -- 8: ROM / RAM output registers; 9: fabric copies
            s_py9 <= unsigned(s_pal_q(31 downto 22)); s_pu9 <= unsigned(s_pal_q(21 downto 12)); s_pv9 <= unsigned(s_pal_q(11 downto 2));
            s_y9 <= unsigned(s_lb_q(31 downto 22)); s_u9 <= unsigned(s_lb_q(21 downto 12)); s_v9 <= unsigned(s_lb_q(11 downto 2));

            -- 10: compose
            v_wu := resize(s_u9, 11) + 288; v_wv := resize(s_v9, 11) + 588;      -- washed toward teal
            if (s_full = '1' or s_in(9) = '1') and not (s_full = '1' and s_spl(9) = '1' and s_in(9) = '0') then
                if s_wash = '1' then
                    s_y10 <= s_y9; s_u10 <= v_wu(10 downto 1); s_v10 <= v_wv(10 downto 1);
                else
                    s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
                end if;
            elsif s_inb(9) = '1' and s_full = '0' then
                s_y10 <= to_unsigned(96, 10); s_u10 <= to_unsigned(512, 10); s_v10 <= to_unsigned(512, 10);
            elsif s_spl(9) = '1' and s_sorange = '1' then
                s_y10 <= to_unsigned(584, 10); s_u10 <= to_unsigned(772, 10); s_v10 <= to_unsigned(212, 10);
            else
                s_y10 <= s_py9; s_u10 <= s_pu9; s_v10 <= s_pv9;
            end if;

            -- 11..15: output conditioning (centre at stage 14 -> 15)
            s_csu(0) <= s_u10; s_csv(0) <= s_v10;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u10 - s_csu(7);
            s_boxv <= s_boxv + s_v10 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u15 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v15 <= v_sc(13 downto 4);
            s_y11 <= s_y10; s_y12 <= s_y11; s_y13 <= s_y12; s_y14 <= s_y13; s_y15x <= s_y14;
            v_sy := resize(s_y13, 12) + shift_left(resize(s_y14, 12), 1) + resize(s_y15x, 12);
            s_yf15 <= v_sy(11 downto 2);
            -- 16: luma toward black; blanking gate
            if s_yf15 > 64 then v_dl := s_yf15 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(14) = '1' then
                s_y16 <= s_yf15 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u16 <= s_u15; s_v16 <= s_v15;
            else
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            end if;
            -- 17..20: pad
            s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            -- delays
            for i in 2 to 6 loop s_rdd(i) <= s_rdd(i - 1); end loop;
            for i in 7 to 10 loop s_in(i) <= s_in(i - 1); s_inb(i) <= s_inb(i - 1); end loop;
            for i in 8 to 10 loop s_spl(i) <= s_spl(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture greenroom;
