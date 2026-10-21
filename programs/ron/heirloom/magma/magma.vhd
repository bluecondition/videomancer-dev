-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Magma v0.1.0 -- Heirloom #29 (2026-10-20), from WIN_20230515_00_46_28.
--
-- The photo: a cyan / magenta / blue field streaked sideways, with red and
-- yellow blobs, flat red regions where the field saturates, mirrored about a
-- line.
--
-- The program: a smooth field (three triangle waves: x, y and x+y; or the
-- video luma on Source), mirrored left / right (S7, axis K4), smeared
-- horizontally (K1), mapped through a cyan / magenta / blue / yellow palette
-- (K2 hue, K5 cycling); wherever the field rises above the lava threshold
-- (P12) the colour is flat red.
--
--   K1  Streak   horizontal smear of the field
--   K2  Hue      palette offset
--   K3  Blobs    field scale (big blobs .. fine ripples)
--   K4  Axis     mirror axis position
--   K5  Speed    field drift / palette cycling, bipolar
--   K6  Contrast field contrast 0.25 .. 2.25 x
--   S7  Mirror   Off / On
--   S8  Palette  Cyan-magenta / Green-orange-violet
--   S9  Red      Flats / Outlines (red only at the threshold edge)
--   S10 Poster   palette posterized Off / On
--   S11 Source   Video / Osc (the three-wave field)
--   P12 LAVA     red threshold: a few islands .. a red sea
--
-- Generated avid; output conditioning before the blanking gate.  Latency 20
-- clocks.  16 EBR line buffer + 4 EBR palette.  Colours in the hardware
-- convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture magma of program_top is
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

    -- horizontal IIR smear state (the smeared pixel)
    signal s_z : unsigned(2 downto 0) := (others => '0');
    signal s_lumonly : std_logic := '0';
    signal s_sy : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_su, s_sv : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_KA : t_keys := ((0, 678, 64, 663), (64, 164, 440, 960), (128, 426, 887, 808), (192, 840, 584, 64), (255, 678, 64, 663));
    constant C_KB : t_keys := ((0, 580, 136, 216), (85, 584, 772, 212), (170, 349, 756, 852), (255, 580, 136, 216));
    type t_pal is array (0 to 511) of std_logic_vector(31 downto 0);
    function f_lerp(a, b, t, n : integer) return integer is
    begin
        return a + ((b - a) * t) / n;
    end function;
    function f_seg(k : t_keys; i : integer; c : integer) return integer is
        variable r : integer := 0;
        variable a, b : integer;
    begin
        for s in 0 to k'length - 2 loop
            if i >= k(s).i and i <= k(s + 1).i then
                if c = 0 then a := k(s).y; b := k(s + 1).y;
                elsif c = 1 then a := k(s).u; b := k(s + 1).u;
                else a := k(s).v; b := k(s + 1).v; end if;
                r := f_lerp(a, b, i - k(s).i, k(s + 1).i - k(s).i);
            end if;
        end loop;
        return r;
    end function;
    function f_pal return t_pal is
        variable r : t_pal;
    begin
        for i in 0 to 255 loop
            r(i) := std_logic_vector(to_unsigned(f_seg(C_KA, i, 0), 10)) & std_logic_vector(to_unsigned(f_seg(C_KA, i, 1), 10))
                    & std_logic_vector(to_unsigned(f_seg(C_KA, i, 2), 10)) & "00";
            r(256 + i) := std_logic_vector(to_unsigned(f_seg(C_KB, i, 0), 10)) & std_logic_vector(to_unsigned(f_seg(C_KB, i, 1), 10))
                    & std_logic_vector(to_unsigned(f_seg(C_KB, i, 2), 10)) & "00";
        end loop;
        return r;
    end function;
    signal pal : t_pal := f_pal;
    signal s_pal_ra : unsigned(8 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_hue : unsigned(7 downto 0) := (others => '0');
    signal s_fo : unsigned(5 downto 0) := to_unsigned(4, 6);
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ax : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_ax2 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_spd : signed(6 downto 0) := (others => '0');
    signal s_cg : unsigned(8 downto 0) := to_unsigned(128, 9);
    signal s_thr : unsigned(7 downto 0) := to_unsigned(200, 8);
    signal s_ph : unsigned(7 downto 0) := (others => '0');
    signal s_mir, s_pal, s_outl, s_post : std_logic := '0';
    signal s_foy : unsigned(9 downto 0) := to_unsigned(4, 10);
    -- per line
    signal s_rsd_n, s_rsd : unsigned(10 downto 0) := (others => '0');
    -- beam
    signal s_ru, s_rd : unsigned(10 downto 0) := (others => '0');
    -- pixel stages
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_lvl5 : unsigned(9 downto 0) := (others => '0');
    signal s_p6 : unsigned(18 downto 0) := (others => '0');
    signal s_l8 : t_u8(5 to 7) := (others => (others => '0'));
    signal s_idx7 : unsigned(7 downto 0) := (others => '0');
    signal s_red : t_sl(7 to 10) := (others => '0');
    signal s_py10, s_pu10, s_pv10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(64, 10);

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
            s_z <= unsigned(registers_in(0)(9 downto 7));                                -- K1 Streak
            s_hue <= unsigned(registers_in(1)(9 downto 2));                              -- K2 Hue
            s_fo <= resize(unsigned(registers_in(2)(9 downto 5)), 6) + 1;                -- K3 Blobs 1 .. 32
            s_k4 <= unsigned(registers_in(3));                                           -- K4 Axis
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;                  -- K5 Speed
            v_s := shift_right(v_s, 3);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(6 downto 0); end if;
            s_cg <= to_unsigned(32, 9) + resize(unsigned(registers_in(5)(9 downto 2)), 9);   -- K6 Contrast /128

            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            -- per field
            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mir  <= registers_in(6)(0);
                s_pal  <= registers_in(6)(1);
                s_outl <= registers_in(6)(2);
                s_post <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_lumonly <= '1';
                s_thr <= not s_g(11 downto 4);
                s_ph <= s_ph + unsigned(resize(s_spd, 8));

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
            s_ax2 <= shift_left(resize(s_ax, 12), 1);

            -- serial unit (12 shift-add steps per job)
            if s_zc = 4 then
                s_ma <= resize(s_w(11 downto 1), 24); s_mb <= resize(s_k4, 12); s_mjob <= "000";
                s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 24 then
                s_ma <= resize(s_fo, 24); s_mb <= s_ys; s_mjob <= "001";
                s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_ax <= resize(s_w(11 downto 2), 11) + resize(s_macc(21 downto 10), 11);
                    when others => s_foy <= s_macc(17 downto 8);
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
        variable v_m : unsigned(12 downto 0);

    begin
        if rising_edge(clk) then
            -- oscillator: x ramp per source pixel, y ramp per line, phase per field
            if data_in.avid = '1' then s_ox <= s_ox + s_fo; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + s_foy; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + unsigned(resize(s_spd, 8)); end if;
            -- the three-wave field as the oscillator source (luma only)
            s_oty <= to_unsigned(64, 10) + shift_left(resize(f_tri8(s_ox(11 downto 4) + s_ot), 10), 1)
                     + shift_left(resize(f_tri8(s_oy(11 downto 4) - s_ot), 10), 1)
                     + shift_left(resize(f_tri8(s_ox(11 downto 4) + s_oy(11 downto 4) + s_ot), 10), 1);
            s_otu <= to_unsigned(512, 10);
            s_otv <= to_unsigned(512, 10);

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

            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

            elsif s_fall2 = '1' then

            end if;
            v_m := resize(s_ax2, 13);
            if v_m >= resize(s_w, 13) then v_m := v_m - resize(s_w, 13); end if;
            s_rsd_n <= v_m(10 downto 0);

            if data_in.avid = '1' and s_avq = '0' then
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
                s_ru <= (others => '0');
                s_rd <= s_rsd;

            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
                if s_ru = s_wm1(10 downto 0) then s_ru <= (others => '0'); else s_ru <= s_ru + 1; end if;
                if s_rd = 0 then s_rd <= s_wm1(10 downto 0); else s_rd <= s_rd - 1; end if;

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
        variable v_d, v_e, v_n : signed(11 downto 0);
        variable v_zc : integer range 0 to 7;
        variable v_i : unsigned(11 downto 0);
        variable v_ed : signed(9 downto 0);
        variable v_a : unsigned(7 downto 0);

    begin
        if rising_edge(clk) then
            -- 1: mirrored read address
            if s_mir = '1' and s_bcol >= s_ax then s_lb_ra <= s_rd; else s_lb_ra <= s_ru; end if;
            -- 3: field (fabric copy)
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));
            -- 4: IIR smear (luma only)
            v_d := signed(resize(s_y3, 12)) - signed(resize(s_sy, 12));
            case to_integer(s_z) is
                when 0 => v_d := v_d;
                when 1 => v_d := shift_right(v_d, 1);
                when 2 => v_d := shift_right(v_d, 2);
                when 3 => v_d := shift_right(v_d, 3);
                when 4 => v_d := shift_right(v_d, 4);
                when 5 => v_d := shift_right(v_d, 5);
                when 6 => v_d := shift_right(v_d, 6);
                when others => v_d := shift_right(v_d, 7);
            end case;
            v_n := signed(resize(s_sy, 12)) + v_d;
            s_sy <= unsigned(v_n(9 downto 0));
            v_d := signed(resize(s_u3, 12)) - signed(resize(s_su, 12));
            v_e := signed(resize(s_v3, 12)) - signed(resize(s_sv, 12));
            if s_lumonly = '1' then
                v_zc := 0;
            else
                v_zc := to_integer(s_z);
            end if;
            case v_zc is
                when 0 => v_d := v_d; v_e := v_e;
                when 1 => v_d := shift_right(v_d, 1); v_e := shift_right(v_e, 1);
                when 2 => v_d := shift_right(v_d, 2); v_e := shift_right(v_e, 2);
                when 3 => v_d := shift_right(v_d, 3); v_e := shift_right(v_e, 3);
                when 4 => v_d := shift_right(v_d, 4); v_e := shift_right(v_e, 4);
                when 5 => v_d := shift_right(v_d, 5); v_e := shift_right(v_e, 5);
                when 6 => v_d := shift_right(v_d, 6); v_e := shift_right(v_e, 6);
                when others => v_d := shift_right(v_d, 7); v_e := shift_right(v_e, 7);
            end case;
            v_n := signed(resize(s_su, 12)) + v_d;
            s_su <= unsigned(v_n(9 downto 0));
            v_n := signed(resize(s_sv, 12)) + v_e;
            s_sv <= unsigned(v_n(9 downto 0));
            -- 5: level above black
            if s_sy > 64 then s_lvl5 <= s_sy - 64; else s_lvl5 <= (others => '0'); end if;
            s_l8(5) <= s_sy(9 downto 2);
            -- 6: contrast product
            s_p6 <= s_lvl5 * s_cg;
            -- 7: palette index; red flags
            v_i := s_p6(18 downto 7);
            if v_i > 255 then s_idx7 <= x"FF"; else s_idx7 <= v_i(7 downto 0); end if;
            v_ed := signed(resize(s_l8(6), 10)) - signed(resize(s_thr, 10));
            if s_outl = '1' then
                if v_ed > -12 and v_ed < 12 then s_red(7) <= '1'; else s_red(7) <= '0'; end if;
            else
                if v_ed >= 0 then s_red(7) <= '1'; else s_red(7) <= '0'; end if;
            end if;
            -- 8: palette address
            v_a := s_idx7 + s_hue + s_ph;
            if s_post = '1' then v_a(4 downto 0) := (others => '0'); end if;
            s_pal_ra <= s_pal & v_a;
            -- 10: palette fabric copy
            s_py10 <= unsigned(s_pal_q(31 downto 22));
            s_pu10 <= unsigned(s_pal_q(21 downto 12));
            s_pv10 <= unsigned(s_pal_q(11 downto 2));
            -- 11: compose
            if s_red(10) = '1' then
                s_y11 <= to_unsigned(328, 10); s_u11 <= to_unsigned(960, 10); s_v11 <= to_unsigned(360, 10);
            else
                s_y11 <= s_py10; s_u11 <= s_pu10; s_v11 <= s_pv10;
            end if;
            -- 12
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;

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
            for i in 6 to 7 loop s_l8(i) <= s_l8(i - 1); end loop;
            for i in 8 to 10 loop s_red(i) <= s_red(i - 1); end loop;

        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture magma;
