-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Jukebox v0.1.0 -- Heirloom #16 (2026-10-20), from WIN_20230315_00_28_42.
--
-- The photo: ten tall vertical panels, each lit by a rainbow gradient of
-- rounded-square rings -- red, yellow, magenta, blue, cyan -- centred in
-- the panel and mirrored top to bottom about the middle of the screen.
--
-- The program: N panels across; in each, a rounded-square distance from the
-- panel centre (a blend of max(|x|,|y|) and |x|+|y|, |y| half-weighted so
-- the rings run tall) is cut into a continuous cyclic palette.  The rows
-- mirror about a movable line; the picture's luma etches into the rings.
--
--   K1  Panels   panels across, 1 .. 16
--   K2  Round    ring shape: square .. diamond
--   K3  Hue      palette rotation speed, bipolar
--   K4  Palette  Rainbow / Warm / Cool / Neon
--   K5  Centre   mirror line within the row
--   K6  Etch     video depth into the rings
--   S7  Mirror   rings mirror about the centre line / repeat from the top
--   S8  Rings    Smooth / Hard (8 bands)
--   S9  Rows     One / Two
--   S10 Invert   palette hues complemented
--   S11 Source   Video / Osc
--   P12 ZOOM     ring density: one huge ring .. dozens of fine ones
--
-- Geometry in virtual 1080-line units.  Panel width from one serial divide
-- per frame.  Generated avid; output conditioning before the blanking gate.
-- Latency 20 clocks.  EBR: 16 line buffer + 8 palette.  Colours in the
-- hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture jukebox of program_top is
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

    -- four cyclic palettes: Rainbow / Warm / Cool / Neon
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_K0 : t_keys := ((0, 328, 960, 360), (43, 840, 584, 64), (85, 580, 136, 216), (128, 678, 64, 663),
                               (171, 164, 440, 960), (213, 426, 887, 808), (255, 328, 960, 360));
    constant C_K1 : t_keys := ((0, 328, 960, 360), (64, 584, 772, 212), (128, 840, 584, 64), (176, 940, 512, 512),
                               (216, 584, 772, 212), (255, 328, 960, 360));
    constant C_K2 : t_keys := ((0, 164, 440, 960), (64, 678, 64, 663), (128, 940, 512, 512), (176, 349, 756, 852),
                               (216, 192, 608, 696), (255, 164, 440, 960));
    constant C_K3 : t_keys := ((0, 426, 887, 808), (40, 64, 512, 512), (85, 678, 64, 663), (128, 64, 512, 512),
                               (171, 840, 584, 64), (213, 64, 512, 512), (255, 426, 887, 808));
    type t_pal is array (0 to 1023) of std_logic_vector(31 downto 0);
    function f_lerp(a, b, t, n : integer) return integer is
    begin
        return a + ((b - a) * t) / n;
    end function;
    function f_seg(k : t_keys; i : integer) return std_logic_vector is
        variable y, u, v : integer;
    begin
        y := 64; u := 512; v := 512;
        for j in 0 to k'length - 2 loop
            if i >= k(j).i and i <= k(j + 1).i then
                y := f_lerp(k(j).y, k(j + 1).y, i - k(j).i, k(j + 1).i - k(j).i);
                u := f_lerp(k(j).u, k(j + 1).u, i - k(j).i, k(j + 1).i - k(j).i);
                v := f_lerp(k(j).v, k(j + 1).v, i - k(j).i, k(j + 1).i - k(j).i);
            end if;
        end loop;
        return std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
               & std_logic_vector(to_unsigned(v, 10)) & "00";
    end function;
    function f_pal return t_pal is
        variable r : t_pal;
    begin
        for i in 0 to 255 loop
            r(i) := f_seg(C_K0, i);
            r(256 + i) := f_seg(C_K1, i);
            r(512 + i) := f_seg(C_K2, i);
            r(768 + i) := f_seg(C_K3, i);
        end loop;
        return r;
    end function;
    signal pal : t_pal := f_pal;
    signal s_pal_ra : unsigned(9 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_j1 : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal s_rk : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_spd : signed(5 downto 0) := (others => '0');
    signal s_j4, s_pali : unsigned(1 downto 0) := (others => '0');
    signal s_live4 : std_logic := '0';
    signal s_ck : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_etch : unsigned(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    signal s_mirror, s_hard, s_rows2, s_inv : std_logic := '0';
    -- field-latched / derived
    signal s_n : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal s_cwv : unsigned(11 downto 0) := to_unsigned(240, 12);     -- cell width (virtual)
    signal s_cwv2 : unsigned(12 downto 0) := to_unsigned(480, 13);    -- Q.1 cell width
    signal s_chq : unsigned(19 downto 0) := (others => '0');          -- row height << 8
    signal s_cc : unsigned(11 downto 0) := to_unsigned(540, 12);      -- mirror line within the row
    signal s_zk : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal s_ph : unsigned(7 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    -- serial divider
    signal s_dnum : unsigned(11 downto 0) := (others => '0');
    signal s_drem : unsigned(12 downto 0) := (others => '0');
    signal s_dquo : unsigned(11 downto 0) := (others => '0');
    signal s_dden : unsigned(4 downto 0) := to_unsigned(10, 5);
    signal s_di : unsigned(3 downto 0) := (others => '0');
    signal s_drun : std_logic := '0';

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
    signal s_cv_n : unsigned(19 downto 0) := (others => '0');
    signal s_ay_n, s_ay : unsigned(11 downto 0) := (others => '0');   -- |dy| / 2

    -- beam state
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_cu : unsigned(12 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ax1 : unsigned(11 downto 0) := (others => '0');
    signal s_mx2, s_mn2 : unsigned(11 downto 0) := (others => '0');
    signal s_p2 : unsigned(19 downto 0) := (others => '0');
    signal s_mx3 : unsigned(11 downto 0) := (others => '0');
    signal s_f3 : unsigned(12 downto 0) := (others => '0');
    signal s_y3, s_y4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_me4 : unsigned(19 downto 0) := (others => '0');
    signal s_p4 : unsigned(21 downto 0) := (others => '0');
    signal s_me5 : unsigned(9 downto 0) := (others => '0');
    signal s_idx5 : unsigned(7 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y12, s_y13, s_y14, s_y15, s_y16, s_y17x, s_yf17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    -- Controls, vblank zone, per-field latches, cell width divide
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_r : unsigned(12 downto 0);
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

            -- K1 Panels (16 zones, deadband after warm-up)
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 3 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 60 or v_k(9 downto 6) = 15)) then
                s_j1 <= v_k(9 downto 6);
            end if;
            s_rk <= unsigned(registers_in(1)(9 downto 2));                     -- K2 Round
            v_s := signed(resize(unsigned(registers_in(2)), 11)) - 512;        -- K3 Hue rate, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(5 downto 0); end if;
            -- K4 Palette (4 labels; the firmware loads the default as the index)
            v_k := unsigned(registers_in(3));
            if v_k >= 4 then s_live4 <= '1'; end if;
            if s_live4 = '0' and v_k < 4 then
                s_j4 <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_j4 <= v_k(9 downto 8);
            end if;
            s_ck <= unsigned(registers_in(4)(9 downto 2));                     -- K5 Centre
            s_etch <= unsigned(registers_in(5));                               -- K6 Etch
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_n <= s_j1;
                s_pali <= s_j4;
                s_mirror <= not registers_in(6)(0);
                s_hard   <= registers_in(6)(1);
                s_rows2  <= registers_in(6)(2);
                s_inv    <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_ph <= s_ph + unsigned(resize(s_spd, 8));
            end if;

            -- derived
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1); s_inc2 <= to_unsigned(32, 6);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14); s_inc2 <= to_unsigned(18, 6);
                when others => v_vw := resize(s_w, 14); s_inc2 <= to_unsigned(8, 6);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cwv2 <= s_cwv & '0';
            -- row height 1080 / 540; mirror line = K5 of it
            if s_rows2 = '1' then
                s_chq <= to_unsigned(540 * 256, 20);
                s_cc <= resize(s_ck & '0', 12) + resize(s_ck(7 downto 3), 12);      -- 2.125 ck (0 .. 540)
            else
                s_chq <= to_unsigned(1080 * 256, 20);
                s_cc <= resize(s_ck & "00", 12) + resize(s_ck(7 downto 2), 12);     -- 4.25 ck (0 .. 1080)
            end if;
            s_zk <= to_unsigned(2, 9) + resize(s_g(11 downto 4), 9);          -- 2 .. 257

            -- serial divide: cwv = vw / N
            if s_zc = 4 then
                s_dnum <= s_vw; s_drem <= (others => '0'); s_dden <= resize(s_n, 5) + 1;
                s_drun <= '1'; s_di <= (others => '0');
            elsif s_drun = '1' then
                v_r := s_drem(11 downto 0) & s_dnum(11);
                s_dnum <= s_dnum(10 downto 0) & '0';
                if v_r >= resize(s_dden, 13) then
                    s_drem <= v_r - resize(s_dden, 13);
                    s_dquo <= s_dquo(10 downto 0) & '1';
                else
                    s_drem <= v_r;
                    s_dquo <= s_dquo(10 downto 0) & '0';
                end if;
                if s_di = 11 then s_drun <= '0'; else s_di <= s_di + 1; end if;
            elsif s_di = 11 then
                s_di <= (others => '0');
                s_cwv <= s_dquo;
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
    -- Source side: line-buffer write; cell-row DDA and b|dy| per line
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(20 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_ad : unsigned(11 downto 0);
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
                s_cv_n <= (others => '0');
            elsif s_fall1 = '1' then
                v_c := resize(s_cv_n, 21) + s_ys;
                if v_c >= s_chq then
                    s_cv_n <= resize(v_c - s_chq, 20);
                else
                    s_cv_n <= v_c(19 downto 0);
                end if;
            end if;
            -- |dy| / 2 about the mirror line, or the half ramp from the row top
            v_d := signed(resize(s_cv_n(19 downto 8), 13)) - signed(resize(s_cc, 13));
            if v_d < 0 then v_ad := resize(unsigned(-v_d), 12); else v_ad := unsigned(v_d(11 downto 0)); end if;
            if s_mirror = '0' then v_ad := s_cv_n(19 downto 8); end if;
            s_ay_n <= '0' & v_ad(11 downto 1);

            if data_in.avid = '1' and s_avq = '0' then
                s_ay <= s_ay_n;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state: column, cell x (Q.1)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_col <= (others => '0');
                s_cu <= (others => '0');
            elsif g_avid = '1' then
                s_col <= s_col + 1;
                v_n := resize(s_cu, 14) + s_xs;
                if v_n >= s_cwv2 then s_cu <= resize(v_n - s_cwv2, 13); else s_cu <= v_n(12 downto 0); end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_f : signed(13 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_i : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: |dx| (virtual); video read
            v_f := signed(resize(s_cu, 14)) - signed(resize(s_cwv, 14));
            if v_f < 0 then v_f := -v_f; end if;
            s_ax1 <= unsigned(v_f(12 downto 1));
            s_lb_ra <= s_col;

            -- 2: max / min of |dx|, |dy|/2
            if s_ax1 >= s_ay then s_mx2 <= s_ax1; s_mn2 <= s_ay; else s_mx2 <= s_ay; s_mn2 <= s_ax1; end if;

            -- 3: roundness product; video (fabric copy)
            s_p2 <= s_mn2 * s_rk;
            s_mx3 <= s_mx2;
            s_y3 <= unsigned(s_lb_q(31 downto 22));

            -- 4: rounded-square distance; etch product
            s_f3 <= resize(s_mx3, 13) + resize(s_p2(19 downto 8), 13);
            if s_y3 > 64 then s_me4 <= (s_y3 - 64) * s_etch; else s_me4 <= (others => '0'); end if;
            s_y4 <= s_y3;

            -- 5: ring zoom
            s_p4 <= s_f3 * s_zk;
            s_me5 <= s_me4(19 downto 10);

            -- 6: index = rings + hue phase + etch
            s_idx5 <= s_p4(11 downto 4) + s_ph + s_me5(7 downto 0) + resize(s_me5(9 downto 8), 8);

            -- 7: palette address (hard: 8 bands)
            if s_hard = '1' then v_i := s_idx5(7 downto 5) & "00000"; else v_i := s_idx5; end if;
            s_pal_ra <= s_pali & v_i;

            -- 8: ROM register; 9: fabric copy; 10: invert
            s_y9 <= unsigned(s_pal_q(31 downto 22));
            s_u9 <= unsigned(s_pal_q(21 downto 12));
            s_v9 <= unsigned(s_pal_q(11 downto 2));
            s_y10 <= s_y9;
            if s_inv = '1' then
                s_u10 <= resize(to_unsigned(1024, 11) - resize(s_u9, 11), 10);
                s_v10 <= resize(to_unsigned(1024, 11) - resize(s_v9, 11), 10);
            else
                s_u10 <= s_u9; s_v10 <= s_v9;
            end if;
            -- 11: colour
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;

            -- 12..17: output conditioning (centre at stage 16 -> 17)
            s_csu(0) <= s_u11; s_csv(0) <= s_v11;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u11 - s_csu(7);
            s_boxv <= s_boxv + s_v11 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u17 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v17 <= v_sc(13 downto 4);
            s_y12 <= s_y11; s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14;
            s_y16 <= s_y15; s_y17x <= s_y16;
            v_sy := resize(s_y15, 12) + shift_left(resize(s_y16, 12), 1) + resize(s_y17x, 12);
            s_yf17 <= v_sy(11 downto 2);
            -- 18: luma toward black; blanking gate
            if s_yf17 > 64 then v_dl := s_yf17 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(16) = '1' then
                s_y18 <= s_yf17 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture jukebox;
