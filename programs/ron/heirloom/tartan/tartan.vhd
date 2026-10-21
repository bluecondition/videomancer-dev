-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tartan v0.1.0 -- Heirloom #25 (2026-10-20), from WIN_20230410_01_13_24.
--
-- The photo: a diagonal stripe field (cyan / green / orange / magenta /
-- black) sliced into horizontal bands, every band slid sideways and
-- recoloured, with black bars cut across.
--
-- The program: a diagonal stripe phase (x + angle * y) cut into 8 bands
-- painted from an 8-entry palette; horizontal bands (K3 height) each add a
-- random (or stepped) phase offset scaled by P12 and rotate the palette;
-- the source luma keys the stripes against black (K5), and a slow vertical
-- oscillator cuts black bars (S9).
--
--   K1  Pitch    stripe pitch
--   K2  Angle    diagonal angle (centre = vertical stripes)
--   K3  Band     band height
--   K4  Palette  Tartan / Neon / Earth / Mono
--   K5  Black    key level: how much of the source goes black
--   K6  Speed    stripe scroll, bipolar (centre = still)
--   S7  Offsets  Random / Stepped (each band one step further)
--   S8  Rotate   palette rotates per band
--   S9  Bars     black bars from a slow oscillator
--   S10 Flip     angle flipped
--   S11 Source   Video / Osc
--   P12 SLICE    band offset amount: an unbroken field .. shredded
--
-- Generated avid; output conditioning before the blanking gate; virtual
-- 1080-line geometry.  Latency 16 clocks.  EBR: 6 (the source luma).
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture tartan of program_top is
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

    type t_col is array (0 to 2) of integer;
    constant BLK : t_col := (64, 512, 512);
    constant WHT : t_col := (940, 512, 512);
    constant GRY : t_col := (480, 512, 512);
    constant LGR : t_col := (700, 512, 512);
    constant DGR : t_col := (260, 512, 512);
    constant CYN : t_col := (678, 64, 663);
    constant GRN : t_col := (580, 136, 216);
    constant ORG : t_col := (584, 772, 212);
    constant MAG : t_col := (426, 887, 808);
    constant YEL : t_col := (840, 584, 64);
    constant RED : t_col := (328, 960, 360);
    constant BLU : t_col := (164, 440, 960);
    constant VIO : t_col := (349, 756, 852);
    constant PNK : t_col := (752, 612, 512);
    constant CRM : t_col := (860, 540, 470);
    constant MNT : t_col := (760, 420, 440);
    type t_pdef is array (0 to 31) of t_col;
    constant C_PDEF : t_pdef := (
        CYN, GRN, ORG, MAG, BLK, CYN, GRN, ORG,     -- Tartan
        MAG, BLK, YEL, BLK, CYN, BLK, GRN, PNK,     -- Neon
        ORG, CRM, RED, DGR, YEL, MNT, ORG, BLK,     -- Earth
        WHT, LGR, GRY, DGR, BLK, DGR, GRY, LGR);    -- Mono
    type t_pal is array (0 to 31) of std_logic_vector(31 downto 0);
    function f_pal return t_pal is
        variable r : t_pal;
    begin
        for i in 0 to 31 loop
            r(i) := std_logic_vector(to_unsigned(C_PDEF(i)(0), 10)) & std_logic_vector(to_unsigned(C_PDEF(i)(1), 10))
                    & std_logic_vector(to_unsigned(C_PDEF(i)(2), 10)) & "00";
        end loop;
        return r;
    end function;
    constant C_PAL : t_pal := f_pal;

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
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_k1, s_k3, s_k5 : unsigned(7 downto 0) := (others => '0');
    signal s_k2r, s_k6r : signed(9 downto 0) := (others => '0');
    signal s_j4 : unsigned(1 downto 0) := (others => '0');
    signal s_live4 : std_logic := '0';
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_a : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_ax : unsigned(9 downto 0) := to_unsigned(32, 10);
    signal s_b : unsigned(7 downto 0) := (others => '0');
    signal s_bneg : std_logic := '0';
    signal s_bys : unsigned(11 downto 0) := (others => '0');
    signal s_bh : unsigned(9 downto 0) := to_unsigned(64, 10);          -- band height, virtual lines
    signal s_bhq : unsigned(17 downto 0) := (others => '0');
    signal s_thr : unsigned(9 downto 0) := (others => '0');
    signal s_sl : unsigned(7 downto 0) := (others => '0');
    signal s_pali : unsigned(1 downto 0) := (others => '0');
    signal s_scr : unsigned(15 downto 0) := (others => '0');
    signal s_bph : unsigned(15 downto 0) := (others => '0');
    signal s_step, s_rot, s_bars, s_flip : std_logic := '0';
    -- serial multiplier
    signal s_ma : unsigned(19 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(19 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_lfsr : unsigned(15 downto 0) := x"7A21";

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
    signal s_cy_n : unsigned(17 downto 0) := (others => '0');
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_ly_n : unsigned(15 downto 0) := (others => '0');
    signal s_bi_n : unsigned(5 downto 0) := (others => '0');
    signal s_dph_n : unsigned(15 downto 0) := (others => '0');
    signal s_rot_n : unsigned(2 downto 0) := (others => '0');
    signal s_bar_n : std_logic := '0';
    signal s_p0 : unsigned(15 downto 0) := (others => '0');
    signal s_rotl : unsigned(2 downto 0) := (others => '0');
    signal s_barl : std_logic := '0';

    -- beam
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_px : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u3 is array (natural range <>) of unsigned(2 downto 0);
    signal s_bi : t_u3(1 to 3) := (others => (others => '0'));
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_c4 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_k4, s_k5x : std_logic := '0';
    signal s_c5 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
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

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial product
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_g : signed(12 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_k : unsigned(9 downto 0);
        variable v_t : unsigned(10 downto 0);
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
            v_s := signed(resize(unsigned(registers_in(1)), 11)) - 512;
            if v_s > -8 and v_s < 8 then s_k2r <= (others => '0'); else s_k2r <= v_s(9 downto 0); end if;
            s_k3 <= unsigned(registers_in(2)(9 downto 2));
            -- K4 Palette (4 labels; the firmware loads the default as the index)
            v_k := unsigned(registers_in(3));
            if v_k >= 4 then s_live4 <= '1'; end if;
            if s_live4 = '0' and v_k < 4 then
                s_j4 <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_j4 <= v_k(9 downto 8);
            end if;
            s_k5 <= unsigned(registers_in(4)(9 downto 2));
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;
            v_s := shift_right(v_s, 2);
            if v_s > -2 and v_s < 2 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_a <= resize(s_k1(7 downto 1), 8) + 4;                    -- 4 .. 131 phase / virtual unit
                if s_k2r < 0 then s_b <= unsigned(resize(-s_k2r, 10)(9 downto 2)); s_bneg <= not registers_in(6)(3);
                else s_b <= unsigned(s_k2r(9 downto 2)); s_bneg <= registers_in(6)(3); end if;
                s_bh <= resize(s_k3, 10) + resize(s_k3, 10) + 24;         -- 24 .. 534 virtual lines
                s_pali <= s_j4;
                v_t := resize(s_k5, 11) + resize(s_k5, 11) + resize(s_k5, 11) + 64;   -- 64 .. 829
                s_thr <= v_t(9 downto 0);
                s_sl <= s_g(11 downto 4);
                s_step <= registers_in(6)(0);
                s_rot  <= registers_in(6)(1);
                s_bars <= registers_in(6)(2);
                s_flip <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_scr <= s_scr + unsigned(resize(shift_left(s_k6r, 3), 16));
                s_bph <= s_bph + 700;
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            s_bhq <= s_bh & x"00";
            case to_integer(s_xs) is
                when 4      => s_ax <= shift_left(resize(s_a, 10), 2);
                when 3      => s_ax <= shift_left(resize(s_a, 10), 1) + resize(s_a, 10);
                when others => s_ax <= shift_left(resize(s_a, 10), 1);
            end case;

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
    -- Source side: line-buffer write; per-line stripe phase, bands, bars
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(18 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_bar : unsigned(6 downto 0);
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
                s_cy_n <= (others => '0');
                s_ly_n <= s_scr;
                s_bi_n <= (others => '0');
                s_dph_n <= (others => '0');
                s_rot_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                if s_bneg = '1' then s_ly_n <= s_ly_n - resize(s_bys, 16); else s_ly_n <= s_ly_n + resize(s_bys, 16); end if;
                v_c := resize(s_cy_n, 19) + s_ys;
                if v_c >= s_bhq then
                    -- new band: random or stepped offset, palette rotation
                    s_cy_n <= resize(v_c - s_bhq, 18);
                    s_bi_n <= s_bi_n + 1;
                    s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                    if s_step = '1' then
                        v_t := resize(s_bi_n, 8) * s_sl;
                        s_dph_n <= v_t(13 downto 0) & "00";
                    else
                        v_t := s_lfsr(7 downto 0) * s_sl;
                        s_dph_n <= v_t;
                    end if;
                    s_rot_n <= s_lfsr(10 downto 8);
                else
                    s_cy_n <= v_c(17 downto 0);
                end if;
            end if;
            -- black bars: a slow triangle of y with a 1/4 duty
            v_bar := f_tri8(s_vy_n(17 downto 10) + s_bph(15 downto 8));
            if v_bar < 24 then s_bar_n <= '1'; else s_bar_n <= '0'; end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_p0 <= s_ly_n + s_dph_n;
                if s_rot = '1' then s_rotl <= s_rot_n; else s_rotl <= (others => '0'); end if;
                s_barl <= s_bar_n and s_bars;
            end if;
        end if;
    end process p_src;

    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= (others => '0');
                s_px <= s_p0;
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_px <= s_px + resize(s_ax, 16);
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
            -- 1: band index (3 bits) with the band's palette rotation; read
            s_bi(1) <= s_px(15 downto 13) + s_rotl;
            s_lb_ra <= s_rdp;
            -- 3: fabric copy of the source luma
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            -- 4: palette colour (LUT); key
            s_c4 <= C_PAL(to_integer(s_pali & s_bi(3)));
            if s_y3 < s_thr then s_k4 <= '1'; else s_k4 <= '0'; end if;
            -- 5: register
            s_c5 <= s_c4; s_k5x <= s_k4;
            -- 6: compose: black where keyed or barred
            if s_k5x = '1' or s_barl = '1' then
                s_y6 <= to_unsigned(64, 10); s_u6 <= to_unsigned(512, 10); s_v6 <= to_unsigned(512, 10);
            else
                s_y6 <= unsigned(s_c5(31 downto 22)); s_u6 <= unsigned(s_c5(21 downto 12)); s_v6 <= unsigned(s_c5(11 downto 2));
            end if;
            -- 7
            s_y7 <= s_y6; s_u7 <= s_u6; s_v7 <= s_v6;

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

            s_bi(2) <= s_bi(1); s_bi(3) <= s_bi(2);
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture tartan;
