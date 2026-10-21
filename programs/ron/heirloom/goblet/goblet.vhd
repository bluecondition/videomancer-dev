-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Goblet v0.1.0 -- Heirloom #49 (2026-10-20), from WIN_20230916_01_08_00.
--
-- The photo: vertical rainbow stripes (green / purple / red / cyan / yellow)
-- with a cyan glass goblet drawn over them as a smooth outline band -- bowl,
-- stem and foot -- mirrored left and right, with chrome highlights along the
-- glass.
--
-- The program: a goblet profile (64-entry half-width table down the screen,
-- interpolated, wobbled by a triangle wave and scaled by P12) drawn as a band
-- |x - w(y)| < width in cyan with a bright highlight at the band centre, over
-- vertical rainbow stripes (or the picture).  Filled mode tints the whole
-- goblet.
--
--   K1  Stripes   stripe pitch
--   K2  Width     band width
--   K3  Hue       stripe palette offset
--   K4  Wobble    profile wobble
--   K5  Drift     stripes scroll, bipolar (centre = still)
--   K6  Highlight highlight brightness along the glass
--   S7  Mirror    stripes mirrored about the goblet axis
--   S8  Fill      Stripes (the source's luma shows through) / Picture
--   S9  Goblet    Outline / Filled
--   S10 Palette   Rainbow / Warm
--   S11 Source    Video / Osc
--   P12 GOBLET    profile scale: a tiny glass .. wider than the screen
--
-- Virtual 1080-line geometry; generated avid; output conditioning before the
-- gate.  Latency 20 clocks.  EBR: 16 line buffer + 4 palette.  u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture goblet of program_top is
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

    -- palettes: A rainbow (R Y G C B M R), B warm (red -> orange -> yellow -> white)
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_KA : t_keys := ((0, 328, 960, 360), (43, 840, 584, 64), (85, 580, 136, 216), (128, 678, 64, 663),
                               (170, 164, 440, 960), (213, 426, 887, 808), (255, 328, 960, 360));
    constant C_KB : t_keys := ((0, 220, 820, 400), (80, 328, 960, 360), (160, 584, 772, 212), (220, 840, 584, 64), (255, 940, 512, 512));
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

    -- goblet half-width down the screen (64 rows, virtual px at scale 1.0)
    type t_prof is array (0 to 63) of integer range 0 to 511;
    constant C_PROF : t_prof := (
        0, 0, 0, 295, 289, 284, 277, 271, 263, 256, 249, 241, 232, 224, 215, 206, 197, 187, 177, 167, 156, 145, 134, 122,
        110, 96, 82, 65, 44, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 34,
        55, 76, 97, 118, 138, 159, 180, 201, 222, 0, 0, 0);

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
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_cx : unsigned(11 downto 0) := to_unsigned(960, 12);      -- virtual centre
    signal s_cxp : unsigned(10 downto 0) := to_unsigned(960, 11);     -- real centre
    signal s_offv : unsigned(10 downto 0) := (others => '0');          -- second vase offset (virtual)
    signal s_fx : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_fxs : unsigned(9 downto 0) := to_unsigned(16, 10);
    signal s_bwk : unsigned(7 downto 0) := to_unsigned(24, 8);        -- band width (virtual px)
    signal s_huek : unsigned(7 downto 0) := (others => '0');
    signal s_wobk : unsigned(7 downto 0) := (others => '0');
    signal s_drk : signed(5 downto 0) := (others => '0');
    signal s_hlk : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched
    signal s_scale : unsigned(9 downto 0) := to_unsigned(256, 10);    -- profile scale, 1.0 = 256
    signal s_bw : unsigned(7 downto 0) := to_unsigned(24, 8);
    signal s_hue : unsigned(7 downto 0) := (others => '0');
    signal s_wob : unsigned(7 downto 0) := (others => '0');
    signal s_hl : unsigned(7 downto 0) := (others => '0');
    signal s_sph : unsigned(7 downto 0) := (others => '0');           -- stripe scroll phase
    signal s_mir, s_pic, s_fill, s_warm : std_logic := '0';
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";

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
    signal s_pi : unsigned(5 downto 0) := (others => '0');
    signal s_p0, s_p1 : unsigned(8 downto 0) := (others => '0');
    signal s_pfrac : unsigned(3 downto 0) := (others => '0');
    signal s_pint : unsigned(12 downto 0) := (others => '0');          -- interpolated profile, Q.4
    signal s_wobv : unsigned(7 downto 0) := (others => '0');
    signal s_pd : signed(10 downto 0) := (others => '0');
    signal s_pm : signed(15 downto 0) := (others => '0');
    signal s_pf : unsigned(3 downto 0) := (others => '0');
    signal s_p0d, s_p0dd : unsigned(8 downto 0) := (others => '0');
    signal s_vp : unsigned(22 downto 0) := (others => '0');
    signal s_wp : unsigned(15 downto 0) := (others => '0');
    signal s_pz : std_logic := '0';
    signal s_hw_n, s_hw : unsigned(10 downto 0) := (others => '0');
    signal s_cxl : signed(12 downto 0) := to_signed(960, 13);

    -- beam state
    signal s_vxa : unsigned(12 downto 0) := (others => '0');          -- Q.1 virtual x
    signal s_px : unsigned(12 downto 0) := (others => '0');           -- stripe phase

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_dxa1, s_dxb1 : signed(13 downto 0) := (others => '0');
    signal s_px1 : unsigned(7 downto 0) := (others => '0');
    signal s_adxa2, s_adxb2 : unsigned(12 downto 0) := (others => '0');
    signal s_sga2, s_sgb2 : std_logic := '0';
    signal s_px2 : unsigned(7 downto 0) := (others => '0');
    signal s_adx3 : unsigned(12 downto 0) := (others => '0');
    signal s_sg : t_sl(3 to 6) := (others => '0');
    signal s_px3 : unsigned(7 downto 0) := (others => '0');
    signal s_in : t_sl(4 to 11) := (others => '0');
    signal s_px4, s_px5 : unsigned(7 downto 0) := (others => '0');
    signal s_e4 : signed(13 downto 0) := (others => '0');
    signal s_ae5 : unsigned(12 downto 0) := (others => '0');
    signal s_band : t_sl(6 to 11) := (others => '0');
    signal s_hlp6 : unsigned(15 downto 0) := (others => '0');
    signal s_hld : t_u10(7 to 11) := (others => (others => '0'));
    signal s_bcol : unsigned(10 downto 0) := (others => '0');
    signal s_rdd : t_u11(1 to 6) := (others => (others => '0'));
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py9, s_pu9, s_pv9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(64, 10);
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

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            s_pal_q <= pal(to_integer(s_pal_ra));
        end if;
    end process p_pal;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
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

            s_fx <= resize(unsigned(registers_in(0)(9 downto 4)), 7) + 1;        -- K1 Stripes 1 .. 64
            s_bwk <= resize(unsigned(registers_in(1)(9 downto 4)), 8) + 4;       -- K2 Width 4 .. 67 virtual px
            s_huek <= unsigned(registers_in(2)(9 downto 2));                     -- K3 Hue
            s_wobk <= unsigned(registers_in(3)(9 downto 2));                     -- K4 Wobble
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;          -- K5 Drift, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_drk <= (others => '0'); else s_drk <= v_s(5 downto 0); end if;
            s_hlk <= unsigned(registers_in(5)(9 downto 2));                      -- K6 Highlight
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_bw <= s_bwk; s_hue <= s_huek; s_wob <= s_wobk; s_hl <= s_hlk;
                s_sph <= s_sph + unsigned(resize(s_drk, 8));
                s_mir  <= registers_in(6)(0);
                s_pic  <= registers_in(6)(1);
                s_fill <= registers_in(6)(2);
                s_warm <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
            end if;

            -- derived
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cx <= '0' & s_vw(11 downto 1);
            s_cxp <= s_w(11 downto 1);
            s_offv <= (others => '0');
            -- scale 32 .. 1055 / 256 (a tiny glass .. four times the screen)
            s_scale <= to_unsigned(32, 10) + s_g(11 downto 2);
            case to_integer(s_xs) is
                when 4      => s_fxs <= shift_left(resize(s_fx, 10), 2);
                when 3      => s_fxs <= shift_left(resize(s_fx, 10), 1) + resize(s_fx, 10);
                when others => s_fxs <= shift_left(resize(s_fx, 10), 1);
            end case;

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
    -- Source side: line-buffer write (video or oscillator); per-line profile,
    -- half-width, tear; the 65536 / half-width divide; commit at the rise
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_w : unsigned(17 downto 0);
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
            s_fall4 <= s_fall3;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
            end if;
            -- profile of the coming line: table row = vy / 16.875 ~ (vy * 61) >> 10, interpolated
            v_w := s_vy_n(19 downto 8) * to_unsigned(61, 6);
            s_pi <= v_w(15 downto 10);
            s_pfrac <= v_w(9 downto 6);
            s_p0 <= to_unsigned(C_PROF(to_integer(s_pi)), 9);
            if s_pi = 63 then s_p1 <= (others => '0'); else s_p1 <= to_unsigned(C_PROF(to_integer(s_pi + 1)), 9); end if;
            s_pd <= signed(resize(s_p1, 11)) - signed(resize(s_p0, 11));
            s_pf <= s_pfrac; s_p0d <= s_p0;
            s_pm <= s_pd * signed(resize(s_pf, 5));
            s_p0dd <= s_p0d;
            s_pint <= unsigned(resize(signed(shift_left(resize(s_p0dd, 13), 4)) + resize(s_pm, 13), 13));
            -- wobble: a triangle of y scaled by K4, added where the glass exists
            s_wobv <= resize(f_tri8(s_vy_n(16 downto 9) + s_sph), 8);
            s_vp <= s_pint * s_scale;                    -- Q.4 x Q.8 = Q.12
            s_wp <= s_wobv * s_wob;
            s_pz <= '0'; if s_pint = 0 then s_pz <= '1'; end if;
            if s_fall4 = '1' or s_zone = '1' then
                if s_pz = '1' then
                    s_hw_n <= (others => '0');
                else
                    s_hw_n <= resize(s_vp(22 downto 12), 11) + resize(s_wp(15 downto 10), 11);
                end if;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_hw <= s_hw_n;
                s_cxl <= signed(resize(s_cx, 13));
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
                s_vxa <= (others => '0');
                s_px <= (others => '0');
                s_bcol <= (others => '0');
            elsif g_avid = '1' then
                s_vxa <= s_vxa + s_xs;
                s_px <= s_px + s_fxs;
                s_bcol <= s_bcol + 1;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: dx to both centres
            s_dxa1 <= signed(resize(s_vxa(12 downto 1), 14)) - resize(s_cxl, 14) + signed(resize(s_offv, 14));
            s_dxb1 <= signed(resize(s_vxa(12 downto 1), 14)) - resize(s_cxl, 14) - signed(resize(s_offv, 14));
            if s_mir = '1' and s_vxa(12 downto 1) >= s_cx then
                s_px1 <= s_hue - s_px(12 downto 5) + s_sph;
            else
                s_px1 <= s_px(12 downto 5) + s_hue + s_sph;
            end if;
            s_rdd(1) <= s_bcol;

            -- 2: |dx|, signs
            if s_dxa1 < 0 then s_adxa2 <= resize(unsigned(-s_dxa1), 13); s_sga2 <= '1';
            else s_adxa2 <= unsigned(s_dxa1(12 downto 0)); s_sga2 <= '0'; end if;
            if s_dxb1 < 0 then s_adxb2 <= resize(unsigned(-s_dxb1), 13); s_sgb2 <= '1';
            else s_adxb2 <= unsigned(s_dxb1(12 downto 0)); s_sgb2 <= '0'; end if;
            s_px2 <= s_px1;

            -- 3: nearest centre
            if s_adxa2 <= s_adxb2 then s_adx3 <= s_adxa2; s_sg(3) <= s_sga2;
            else s_adx3 <= s_adxb2; s_sg(3) <= s_sgb2; end if;
            s_px3 <= s_px2;

            -- 4: inside; distance from the glass edge
            if s_adx3 < resize(s_hw, 13) then s_in(4) <= '1'; else s_in(4) <= '0'; end if;
            s_e4 <= signed(resize(s_adx3, 14)) - signed(resize(s_hw, 14));
            s_px4 <= s_px3;

            -- 5: |e|
            if s_e4 < 0 then s_ae5 <= resize(unsigned(-s_e4), 13); else s_ae5 <= unsigned(s_e4(12 downto 0)); end if;
            s_px5 <= s_px4;

            -- 6: band test, highlight product (bright at the band centre)
            if s_ae5 < resize(s_bw, 13) and s_hw /= 0 then s_band(6) <= '1'; else s_band(6) <= '0'; end if;
            s_hlp6 <= resize(s_ae5(7 downto 0), 8) * s_hl;
            s_pal_ra <= s_warm & s_px5;
            s_lb_ra <= s_rdd(5);

            -- 7: highlight luma: 940 - |e| * hl / 8, floored at the cyan luma
            if s_hlp6(15 downto 3) >= 262 then s_hld(7) <= to_unsigned(678, 10);
            else s_hld(7) <= to_unsigned(940, 10) - resize(s_hlp6(15 downto 3), 10); end if;

            -- 8: EBR output registers
            -- 9: fabric copies
            s_y9 <= unsigned(s_lb_q(31 downto 22)); s_u9 <= unsigned(s_lb_q(21 downto 12)); s_v9 <= unsigned(s_lb_q(11 downto 2));
            s_py9 <= unsigned(s_pal_q(31 downto 22)); s_pu9 <= unsigned(s_pal_q(21 downto 12)); s_pv9 <= unsigned(s_pal_q(11 downto 2));

            -- 10: fill: picture / stripes (palette chroma, luma = average of both)
            if s_pic = '1' then
                s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            else
                s_y10 <= resize(shift_right(resize(s_y9, 11) + resize(s_py9, 11), 1), 10);
                s_u10 <= s_pu9; s_v10 <= s_pv9;
            end if;

            -- 11: filled goblet tints the inside cyan (chroma toward cyan)
            if s_fill = '1' and s_in(10) = '1' then
                s_y11 <= s_y10;
                s_u11 <= to_unsigned(64, 10) + resize(s_u10(9 downto 2), 10) + resize(s_u10(9 downto 3), 10);
                s_v11 <= to_unsigned(400, 10) + resize(s_v10(9 downto 2), 10) + resize(s_v10(9 downto 3), 10);
            else
                s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            end if;

            -- 12: compose: the glass band in cyan with the highlight
            if s_band(11) = '1' then
                s_y12 <= s_hld(11); s_u12 <= to_unsigned(64, 10); s_v12 <= to_unsigned(663, 10);
            else
                s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;
            end if;

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

            -- delays
            for i in 4 to 6 loop s_sg(i) <= s_sg(i - 1); end loop;
            for i in 5 to 11 loop s_in(i) <= s_in(i - 1); end loop;
            for i in 7 to 11 loop s_band(i) <= s_band(i - 1); end loop;
            for i in 8 to 11 loop s_hld(i) <= s_hld(i - 1); end loop;
            for i in 2 to 6 loop s_rdd(i) <= s_rdd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture goblet;
