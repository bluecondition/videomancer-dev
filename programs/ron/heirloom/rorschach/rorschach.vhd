-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Rorschach v0.1.0 -- Heirloom #4 (2026-10-20), from WIN_20221124_22_54_05.
--
-- The photo: the picture posterized into a pastel palette (cream, pink,
-- blue, red, magenta), vertical wavy cream / pink stripes behind it, heavy
-- row tearing, the whole thing mirrored about the centre line.
--
-- The program: the video (or the oscillator) posterized to a 16-colour
-- pastel palette by luma band and chroma quadrant, keyed by luma over a
-- backdrop of vertical wavy stripes, mirrored LEFT / RIGHT about an axis
-- (a vertical mirror needs a frame store), and torn sideways in 8-line
-- bands by P12.
--
--   K1  Key     luma threshold for the posterized picture to show
--   K2  Stripes backdrop stripe pitch
--   K3  Wave    stripe wobble depth
--   K4  Rate    stripe wobble vertical rate
--   K5  Tint    palette saturation 0 .. 200 %
--   K6  Axis    mirror axis position
--   S7  Mirror  Off / On
--   S8  Stripes Off (flat cream) / On
--   S9  Palette Pastel / Bold
--   S10 Key     Bright parts show / Dark parts show
--   S11 Source  Video / Osc
--   P12 TEAR    0 = clean mirror .. shredded in 8-line bands
--
-- Generated avid; output conditioning before the blanking gate.  Latency 20
-- clocks.  16 EBR.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture rorschach of program_top is
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
    -- triangle of an 8-bit phase -> 7 bits
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- clamp a 13-bit signed value into the 64 .. 960 video range
    function f_clamp(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 960 then return to_unsigned(960, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;
    -- saturate a 9-bit unsigned to 8 bits
    function f_sat8(v : unsigned(8 downto 0)) return unsigned is
    begin
        if v(8) = '1' then return x"FF"; else return v(7 downto 0); end if;
    end function;

    -- line buffer {y, u, v, 00} (read/write can collide: read-before-write kept)
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- internal oscillator source (Source = Osc)
    signal s_src : std_logic := '0';
    signal s_ox : unsigned(11 downto 0) := (others => '0');
    signal s_oy : unsigned(11 downto 0) := (others => '0');
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
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- serial multiplier (12 steps)
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');
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
    signal s_fall1, s_fall2 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_bcol : unsigned(10 downto 0) := (others => '0');
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

    type t_col is array (0 to 2) of integer;
    type t_ptab is array (0 to 31) of t_col;
    -- index = palette & luma-high & luma-mid & Cr-high & Cb-high
    constant C_PT : t_ptab := (
        (300, 400, 300), (300, 440, 900), (328, 960, 360), (426, 887, 808),
        (760, 420, 440), (700, 450, 700), (752, 612, 512), (747, 516, 594),
        (900, 530, 470), (900, 490, 560), (900, 560, 500), (940, 512, 512),
        (900, 530, 470), (900, 490, 560), (900, 560, 500), (940, 512, 512),
        (580, 136, 216), (164, 440, 960), (328, 960, 360), (426, 887, 808),
        (678, 64, 663), (300, 500, 800), (584, 772, 212), (349, 756, 852),
        (840, 584, 64), (940, 512, 512), (752, 612, 512), (900, 530, 470),
        (840, 584, 64), (940, 512, 512), (752, 612, 512), (900, 530, 470));
    signal s_thr : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_fx : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_fxs : unsigned(9 downto 0) := to_unsigned(16, 10);
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal s_fy : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_ts : unsigned(8 downto 0) := to_unsigned(128, 9);
    signal s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ax : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_ax2 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_tk : unsigned(7 downto 0) := (others => '0');
    signal s_mir, s_strp, s_pal2, s_pol : std_logic := '0';
    -- per line
    signal s_py : unsigned(11 downto 0) := (others => '0');
    signal s_wo_n, s_wo : unsigned(7 downto 0) := (others => '0');
    signal s_tear_n : unsigned(10 downto 0) := (others => '0');
    signal s_l8 : unsigned(2 downto 0) := (others => '0');
    signal s_sum : unsigned(12 downto 0) := (others => '0');
    signal s_rsd_n, s_rsu, s_rsd : unsigned(10 downto 0) := (others => '0');
    -- beam
    signal s_ru, s_rd : unsigned(10 downto 0) := (others => '0');
    signal s_px : unsigned(12 downto 0) := (others => '0');
    -- pixel stages
    signal s_sa1 : unsigned(7 downto 0) := (others => '0');
    signal s_st : t_sl(2 to 6) := (others => '0');
    signal s_key : t_sl(4 to 6) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_idx4 : unsigned(4 downto 0) := (others => '0');
    signal s_cy5, s_cy6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_dcu5, s_dcv5 : signed(10 downto 0) := (others => '0');
    signal s_pu6, s_pv6 : signed(20 downto 0) := (others => '0');
    signal s_yd, s_ud, s_vd : t_u10(7 to 11) := (others => to_unsigned(64, 10));

begin

    ---------------------------------------------------------------------------
    -- Line buffer
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;


    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);

    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            -- vblank zone: no avid for 1024 clocks
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

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- knobs (free-running decode)
            v_k := unsigned(registers_in(0));
            s_thr <= to_unsigned(96, 10) + v_k - resize(v_k(9 downto 3), 10);      -- K1 Key 96 .. 992
            s_fx <= resize(unsigned(registers_in(1)(9 downto 4)), 7) + 2;          -- K2 Stripes 2 .. 65
            s_wa <= unsigned(registers_in(2)(9 downto 2));                         -- K3 Wave
            s_fy <= resize(unsigned(registers_in(3)(9 downto 4)), 7) + 1;          -- K4 Rate 1 .. 64
            s_ts <= resize(unsigned(registers_in(4)(9 downto 2)), 9);              -- K5 Tint /128
            s_k6 <= unsigned(registers_in(5));                                     -- K6 Axis

            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            -- per field
            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mir  <= registers_in(6)(0);
                s_strp <= registers_in(6)(1);
                s_pal2 <= registers_in(6)(2);
                s_pol  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_tk <= s_g(11 downto 4);

            end if;

            -- derived (free-running from the latched values)
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            case to_integer(s_xs) is
                when 4      => s_fxs <= shift_left(resize(s_fx, 10), 2);
                when 3      => s_fxs <= shift_left(resize(s_fx, 10), 1) + resize(s_fx, 10);
                when others => s_fxs <= shift_left(resize(s_fx, 10), 1);
            end case;
            s_ax2 <= shift_left(resize(s_ax, 12), 1);

            -- serial unit (12 shift-add steps per job)
            if s_zc = 4 then
                s_ma <= resize(s_w(11 downto 1), 24); s_mb <= resize(s_k6, 12); s_mjob <= "000";
                s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when others => s_ax <= resize(s_w(11 downto 2), 11) + resize(s_macc(21 downto 10), 11);
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
    -- Generated avid (parity-locked start, field-latched width)
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
    -- Source side: oscillator, line-buffer write (Video / Osc), per-line
    -- values (computed after the source avid fall, committed at its rise)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_p : unsigned(14 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_tr : unsigned(10 downto 0);
        variable v_m : unsigned(12 downto 0);

    begin
        if rising_edge(clk) then
            -- oscillator: x ramp per source pixel, y ramp per line, phase per field
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
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_py <= (others => '0');
                s_l8 <= (others => '0');
                s_tear_n <= (others => '0');

            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_py <= s_py + s_fy;
                s_l8 <= s_l8 + 1;
                if s_l8 = 7 then
                    v_t := s_lfsr(7 downto 0) * s_tk;
                    v_tr := resize(v_t(15 downto 6), 11);
                    if v_tr >= s_wm1(10 downto 0) then s_tear_n <= s_wm1(10 downto 0);
                    else s_tear_n <= v_tr; end if;
                end if;

            elsif s_fall2 = '1' then
                null;

            end if;
            -- stripe wobble for the coming line; mirrored read start modulo W
            v_p := s_wa * f_tri8(s_py(10 downto 3));
            s_wo_n <= v_p(14 downto 7);
            s_sum <= resize(s_ax2, 13) + resize(s_tear_n, 13);
            v_m := s_sum;
            if v_m >= shift_left(resize(s_w, 13), 1) then v_m := v_m - shift_left(resize(s_w, 13), 1);
            elsif v_m >= resize(s_w, 13) then v_m := v_m - resize(s_w, 13); end if;
            s_rsd_n <= v_m(10 downto 0);

            if data_in.avid = '1' and s_avq = '0' then
                s_wo <= s_wo_n;
                s_rsu <= s_tear_n;
                s_rsd <= s_rsd_n;

            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state (initialised on the g_start edge so stage 1 sees pixel 0)
    ---------------------------------------------------------------------------
    p_beam : process(clk)

    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_bcol <= (others => '0');
                s_ru <= s_rsu;
                s_rd <= s_rsd;
                s_px <= (others => '0');

            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
                if s_ru = s_wm1(10 downto 0) then s_ru <= (others => '0'); else s_ru <= s_ru + 1; end if;
                if s_rd = 0 then s_rd <= s_wm1(10 downto 0); else s_rd <= s_rd - 1; end if;
                s_px <= s_px + s_fxs;

            end if;

        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = read address; compose at stage 12)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_c : t_col;
        variable v_l2, v_l1, v_cu, v_cv : std_logic;

    begin
        if rising_edge(clk) then
            -- 1: mirrored read address, stripe phase
            if s_mir = '1' and s_bcol >= s_ax then s_lb_ra <= s_rd; else s_lb_ra <= s_ru; end if;
            s_sa1 <= s_px(11 downto 4) + s_wo;
            -- 2: stripe parity
            s_st(2) <= f_tri8(s_sa1)(6);
            -- 3: video (fabric copy)
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));
            -- 4: classify into a palette index; key
            if s_y3 > 600 then v_l2 := '1'; else v_l2 := '0'; end if;
            if s_y3 > 300 then v_l1 := '1'; else v_l1 := '0'; end if;
            if s_u3 > 512 then v_cu := '1'; else v_cu := '0'; end if;
            if s_v3 > 512 then v_cv := '1'; else v_cv := '0'; end if;
            s_idx4 <= s_pal2 & v_l2 & v_l1 & v_cu & v_cv;
            if s_y3 >= s_thr then s_key(4) <= not s_pol; else s_key(4) <= s_pol; end if;
            -- 5: palette entry
            v_c := C_PT(to_integer(s_idx4));
            s_cy5 <= to_unsigned(v_c(0), 10);
            s_dcu5 <= signed(to_unsigned(v_c(1), 11)) - 512;
            s_dcv5 <= signed(to_unsigned(v_c(2), 11)) - 512;
            -- 6: tint products
            s_pu6 <= s_dcu5 * signed(resize(s_ts, 10));
            s_pv6 <= s_dcv5 * signed(resize(s_ts, 10));
            s_cy6 <= s_cy5;
            -- 7: compose: posterized picture / stripes / cream
            if s_key(6) = '1' then
                s_yd(7) <= s_cy6;
                s_ud(7) <= f_clamp(to_signed(512, 13) + resize(shift_right(s_pu6, 7), 13));
                s_vd(7) <= f_clamp(to_signed(512, 13) + resize(shift_right(s_pv6, 7), 13));
            elsif s_strp = '1' and s_st(6) = '1' then
                s_yd(7) <= to_unsigned(752, 10); s_ud(7) <= to_unsigned(612, 10); s_vd(7) <= to_unsigned(512, 10);
            else
                s_yd(7) <= to_unsigned(900, 10); s_ud(7) <= to_unsigned(530, 10); s_vd(7) <= to_unsigned(470, 10);
            end if;
            -- 8..12: delays
            for i in 8 to 11 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            s_y12 <= s_yd(11); s_u12 <= s_ud(11); s_v12 <= s_vd(11);

            -- 13..18: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]; centre at stage 17 -> 18)
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
            -- 19: luma toward black (white 940 -> 777); blanking gate
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(17) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            -- 20
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
            for i in 3 to 6 loop s_st(i) <= s_st(i - 1); end loop;
            for i in 5 to 6 loop s_key(i) <= s_key(i - 1); end loop;

        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture rorschach;
