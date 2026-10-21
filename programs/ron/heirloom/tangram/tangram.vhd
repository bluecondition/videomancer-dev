-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tangram v0.1.0 -- Heirloom #9 (2026-10-20), from WIN_20230222_00_06_47.
--
-- The photo: three stripe fields keyed into each other -- red / magenta
-- diagonals on dark blue, blue verticals, green / cyan diagonals the other
-- way and rainbow stripes -- with the seams between them torn sideways in
-- horizontal bands.
--
-- The program: three generated stripe oscillators (one diagonal, one
-- vertical, one counter-diagonal) and a key that picks which one shows at
-- every pixel: the key is the luma of the source (video, or the internal
-- oscillator) scaled by P12, so at 0 only field 1 shows and at 100 % the
-- three interleave finely along the picture's tones.  Horizontal bands
-- (48 virtual lines) each shear every field and the key sideways by a
-- random amount (K4), tearing the seams like the photo.
--
--   K1  Pitch 1  stripe pitch of field 1 (diagonals)
--   K2  Pitch 2  stripe pitch of fields 2 and 3
--   K3  Angle    diagonal slope of fields 1 and 3
--   K4  Shear    per-band sideways tear of the seams
--   K5  Rate     key drift: the key bands slide through the tones
--   K6  Hue      colour scheme rotation (all three fields)
--   S7  Field 3  Solid (green / cyan) / Rainbow stripes
--   S8  Shear    tears to the right / to the left
--   S9  Key      Hard / Soft (a fourth, blended state between fields 1 and 2)
--   S10 Move     stripes scroll
--   S11 Source   Video / Osc
--   P12 KEY      key gain: field 1 only .. all three finely interleaved
--
-- Generated avid; output conditioning before the blanking gate; virtual
-- 1080-line geometry.  Latency 16 clocks.  EBR: 16 line buffer + 2 ROM.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture tangram of program_top is
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

    -- rainbow ROM: 256 hues, piecewise linear through the verified primaries
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_RB : t_keys := ((0, 328, 960, 360), (43, 840, 584, 64), (85, 580, 136, 216), (128, 678, 64, 663),
                               (171, 164, 440, 960), (213, 426, 887, 808), (256, 328, 960, 360));
    type t_pal is array (0 to 255) of std_logic_vector(31 downto 0);
    function f_lerp(a, b, t, n : integer) return integer is
    begin
        return a + ((b - a) * t) / n;
    end function;
    function f_rainbow return t_pal is
        variable r : t_pal;
        variable y, u, v : integer;
    begin
        for i in 0 to 255 loop
            for k in 0 to C_RB'length - 2 loop
                if i >= C_RB(k).i and i < C_RB(k + 1).i then
                    y := f_lerp(C_RB(k).y, C_RB(k + 1).y, i - C_RB(k).i, C_RB(k + 1).i - C_RB(k).i);
                    u := f_lerp(C_RB(k).u, C_RB(k + 1).u, i - C_RB(k).i, C_RB(k + 1).i - C_RB(k).i);
                    v := f_lerp(C_RB(k).v, C_RB(k + 1).v, i - C_RB(k).i, C_RB(k + 1).i - C_RB(k).i);
                end if;
            end loop;
            r(i) := std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
                    & std_logic_vector(to_unsigned(v, 10)) & "00";
        end loop;
        return r;
    end function;
    signal pal : t_pal := f_rainbow;
    signal s_pal_ra : unsigned(7 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_k1, s_k2, s_k3, s_k4, s_k5 : unsigned(7 downto 0) := (others => '0');
    signal s_hue : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_a1, s_a2 : unsigned(7 downto 0) := to_unsigned(16, 8);       -- phase per virtual unit
    signal s_a1x, s_a2x : unsigned(9 downto 0) := to_unsigned(32, 10);    -- per pixel
    signal s_b : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_bys : unsigned(11 downto 0) := to_unsigned(16, 12);          -- per field line
    signal s_shk : unsigned(7 downto 0) := (others => '0');
    signal s_bias : unsigned(7 downto 0) := (others => '0');
    signal s_gain : unsigned(7 downto 0) := (others => '0');
    signal s_scr : unsigned(15 downto 0) := (others => '0');
    signal s_rb, s_shl, s_soft, s_move : std_logic := '0';
    signal s_c1a, s_c1b, s_c2a, s_c2b, s_c3a, s_c3b : std_logic_vector(31 downto 0) := (others => '0');
    signal s_zra : unsigned(7 downto 0) := (others => '0');
    -- serial multiplier
    signal s_ma : unsigned(19 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(19 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_lfsr : unsigned(15 downto 0) := x"C3A5";

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
    signal s_bnd_n : unsigned(5 downto 0) := (others => '0');          -- band counter (48 virtual lines)
    signal s_ly1_n, s_ly3_n : unsigned(15 downto 0) := (others => '0');
    signal s_dph_n, s_dph : unsigned(15 downto 0) := (others => '0');  -- band shear, phase units
    signal s_dpx_n : unsigned(8 downto 0) := (others => '0');          -- band shear, real px
    signal s_p1s, s_p2s, s_p3s : unsigned(15 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_px1, s_px2, s_px3 : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u3 is array (natural range <>) of unsigned(2 downto 0);
    type t_c is array (natural range <>) of std_logic_vector(31 downto 0);
    signal s_sq : t_u3(1 to 6) := (others => (others => '0'));
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_kf4 : unsigned(7 downto 0) := (others => '0');
    signal s_rc : t_c(4 to 6) := (others => (others => '0'));
    signal s_pr5 : unsigned(15 downto 0) := (others => '0');
    signal s_sel6 : unsigned(1 downto 0) := (others => '0');
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
    -- Controls, vblank zone, per-field latches, per-frame products, scheme
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_g : signed(12 downto 0);
        variable v_ax : unsigned(9 downto 0);
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
            s_k5 <= unsigned(registers_in(4)(9 downto 2));
            s_hue <= unsigned(registers_in(5)(9 downto 2));
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_a1 <= resize(s_k1(7 downto 1), 8) + 2;      -- 2 .. 129 phase / virtual unit
                s_a2 <= resize(s_k2(7 downto 1), 8) + 2;
                s_b <= resize(s_k3(7 downto 1), 8);            -- 0 .. 127 phase / virtual line
                s_shk <= s_k4;
                s_bias <= s_bias + resize(s_k5(7 downto 3), 8);
                s_rb   <= registers_in(6)(0);
                s_shl  <= registers_in(6)(1);
                s_soft <= registers_in(6)(2);
                s_move <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                if s_move = '1' then s_scr <= s_scr + 384; end if;
            end if;

            -- derived
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            s_gain <= s_g(11 downto 4);
            case to_integer(s_xs) is
                when 4      => s_a1x <= shift_left(resize(s_a1, 10), 2);
                               s_a2x <= shift_left(resize(s_a2, 10), 2);
                when 3      => s_a1x <= shift_left(resize(s_a1, 10), 1) + resize(s_a1, 10);
                               s_a2x <= shift_left(resize(s_a2, 10), 1) + resize(s_a2, 10);
                when others => s_a1x <= shift_left(resize(s_a1, 10), 1);
                               s_a2x <= shift_left(resize(s_a2, 10), 1);
            end case;

            -- serial product: b x ys >> 8 (zone clocks 4 .. 16)
            if s_zc = 4 then
                s_ma <= resize(s_b, 20); s_mb <= s_ys; s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(18 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                s_bys <= s_macc(19 downto 8);
            end if;

            -- colour scheme: six rainbow reads in the zone (address at 20 .. 25,
            -- through the pixel path's address register, data latched at 23 .. 28)
            case to_integer(s_zc) is
                when 20 => s_zra <= s_hue;
                when 21 => s_zra <= s_hue + 32;
                when 22 => s_zra <= s_hue + 128;
                when 23 => s_zra <= s_hue + 152;
                when 24 => s_zra <= s_hue + 80;
                when 25 => s_zra <= s_hue + 112;
                when others => null;
            end case;
            case to_integer(s_zc) is
                when 23 => s_c1a <= s_pal_q;
                when 24 => s_c1b <= s_pal_q;
                when 25 => s_c2a <= s_pal_q;
                when 26 => s_c2b <= s_pal_q;
                when 27 => s_c3a <= s_pal_q;
                when 28 => s_c3b <= s_pal_q;
                when others => null;
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
    -- Source side: line-buffer write (video / oscillator); per-line values
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(19 downto 0);
        variable v_t : unsigned(15 downto 0);
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
                s_bnd_n <= (others => '0');
                s_ly1_n <= s_scr;
                s_ly3_n <= s_scr;
                s_dph_n <= (others => '0');
                s_dpx_n <= (others => '0');
            elsif s_fall1 = '1' then
                -- next line: diagonal phases, band counter (48 virtual lines), band shear
                s_ly1_n <= s_ly1_n + resize(s_bys, 16);
                s_ly3_n <= s_ly3_n - resize(s_bys, 16);
                v_c := resize(s_vy_n, 20) + s_ys;
                if v_c(19 downto 8) >= 48 then
                    s_vy_n <= resize(v_c - to_unsigned(48 * 256, 20), 20);
                    s_bnd_n <= s_bnd_n + 1;
                    s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                    v_t := s_lfsr(7 downto 0) * s_shk;
                    s_dph_n <= v_t;
                    s_dpx_n <= v_t(15 downto 7);
                else
                    s_vy_n <= v_c;
                end if;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                -- commit: line-start phases include the band shear; read start pointer
                s_p1s <= s_ly1_n + s_dph_n;
                s_p2s <= s_scr + s_dph_n;
                s_p3s <= s_ly3_n + s_dph_n;
                if s_shl = '1' then
                    s_rst <= resize(s_dpx_n, 11);
                else
                    if s_dpx_n = 0 then s_rst <= (others => '0');
                    else s_rst <= s_wm1(10 downto 0) + 1 - resize(s_dpx_n, 11); end if;
                end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state: three phase accumulators, read pointer
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
                s_px1 <= s_p1s;
                s_px2 <= s_p2s;
                s_px3 <= s_p3s;
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_px1 <= s_px1 + resize(s_a1x, 16);
                s_px2 <= s_px2 + resize(s_a2x, 16);
                s_px3 <= s_px3 + resize(s_a2x, 16);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ca, v_cb : std_logic_vector(31 downto 0);
        variable v_f1, v_f2 : std_logic_vector(31 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_m : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: stripe square waves; video read; rainbow address for field 3
            s_sq(1) <= s_px3(15) & s_px2(15) & s_px1(15);
            s_lb_ra <= s_rdp;
            if s_zone = '1' then s_pal_ra <= s_zra; else s_pal_ra <= s_px3(15 downto 8) + s_hue; end if;
            -- 2: (buffer / ROM output registers)
            -- 3: fabric copies
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            -- 4: key value with drift; field-3 rainbow colour (fabric)
            s_kf4 <= s_y3(9 downto 2) + s_bias;
            s_rc(4) <= s_pal_q;
            -- 5: key x gain
            s_pr5 <= s_kf4 * s_gain;
            -- 6: field select
            s_sel6 <= s_pr5(15 downto 14);
            -- 7: compose
            if s_sq(6)(0) = '1' then v_f1 := s_c1a; else v_f1 := s_c1b; end if;
            if s_sq(6)(1) = '1' then v_f2 := s_c2a; else v_f2 := s_c2b; end if;
            case s_sel6 is
                when "00" => v_ca := v_f1; v_cb := v_f1;
                when "01" => v_ca := v_f2; v_cb := v_f2;
                when "10" =>
                    if s_rb = '1' then v_ca := s_rc(6);
                    elsif s_sq(6)(2) = '1' then v_ca := s_c3a; else v_ca := s_c3b; end if;
                    v_cb := v_ca;
                when others =>
                    if s_soft = '1' then v_ca := v_f1; v_cb := v_f2; else v_ca := v_f2; v_cb := v_f2; end if;
            end case;
            v_m := resize(unsigned(v_ca(31 downto 22)), 11) + resize(unsigned(v_cb(31 downto 22)), 11); s_y7 <= v_m(10 downto 1);
            v_m := resize(unsigned(v_ca(21 downto 12)), 11) + resize(unsigned(v_cb(21 downto 12)), 11); s_u7 <= v_m(10 downto 1);
            v_m := resize(unsigned(v_ca(11 downto 2)), 11) + resize(unsigned(v_cb(11 downto 2)), 11); s_v7 <= v_m(10 downto 1);

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

            -- delays
            for i in 2 to 6 loop s_sq(i) <= s_sq(i - 1); end loop;
            s_rc(5) <= s_rc(4); s_rc(6) <= s_rc(5);
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture tangram;
