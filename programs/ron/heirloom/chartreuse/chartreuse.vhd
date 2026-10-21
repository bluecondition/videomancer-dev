-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Chartreuse v0.1.0 -- Heirloom #51 (2026-10-20), from WIN_20231211_00_38_58.
--
-- The photo: monochrome lime / green / white: smooth glossy folds, a wavy
-- ribbon field with thin chrome highlight lines along the folds, and a soft
-- orb at the left.
--
-- The program: a fold field (a triangle wave of x bent by a triangle wave
-- of y, P12 = the bend, plus a slower second fold; or the video luma on
-- Source) shaded as smooth green with thin white highlight lines wherever
-- the field crosses a narrow window (K2), a soft orb (K4) added at the left,
-- the hue selectable (K3) and alternating in Duo.
--
--   K1  Folds    fold pitch
--   K2  Shine    highlight line width
--   K3  Hue      green .. any of 8 hues
--   K4  Orb      orb size
--   K5  Drift    fold drift, bipolar
--   K6  Contrast field contrast 0.25 .. 2.25 x
--   S7  Shine    highlight lines Off / On
--   S8  Orb      Off / On
--   S9  Hue      Mono / Duo (the hue alternates with the field)
--   S10 Mirror   Off / On (left / right about the centre)
--   S11 Source   Video / Osc (the fold field)
--   P12 FOLD     bend depth: straight ribbons .. deep S-folds
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

architecture chartreuse of program_top is
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

    type t_cos is array (0 to 31) of integer range -128 to 127;
    constant C_COS : t_cos := (127, 125, 117, 106, 90, 71, 49, 25, 0, -25, -49, -71, -90, -106, -117, -125, -127, -125, -117, -106, -90, -71, -49, -25, 0, 25, 49, 71, 90, 106, 117, 125);
    signal s_fo : unsigned(5 downto 0) := to_unsigned(4, 6);
    signal s_hw : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal s_hue : unsigned(2 downto 0) := (others => '0');
    signal s_orbk : unsigned(7 downto 0) := (others => '0');
    signal s_spd : signed(5 downto 0) := (others => '0');
    signal s_cg : unsigned(8 downto 0) := to_unsigned(128, 9);
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal s_ax : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_shine, s_orb, s_duo, s_mir : std_logic := '0';
    signal s_oyi : unsigned(9 downto 0) := to_unsigned(5, 10);
    signal s_cx2sq : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    -- per line
    signal s_wob : unsigned(14 downto 0) := (others => '0');
    signal s_ady_n : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_dy2_n, s_dy2 : unsigned(19 downto 0) := to_unsigned(291600, 20);
    signal s_rsd_n, s_rsd : unsigned(10 downto 0) := (others => '0');
    -- beam
    signal s_ru, s_rd : unsigned(10 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');
    -- pixel stages
    signal s_q1 : unsigned(23 downto 0) := (others => '0');
    signal s_qi2 : unsigned(21 downto 0) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_r2_3 : unsigned(22 downto 0) := (others => '0');
    signal s_od4 : unsigned(7 downto 0) := (others => '0');
    signal s_y4 : unsigned(7 downto 0) := (others => '0');
    signal s_l5 : unsigned(7 downto 0) := (others => '0');
    signal s_p6 : signed(17 downto 0) := (others => '0');
    signal s_l7 : unsigned(7 downto 0) := (others => '0');
    signal s_hl7 : std_logic := '0';
    signal s_cu7, s_cv7 : signed(7 downto 0) := (others => '0');
    signal s_y8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_pu8, s_pv8 : signed(16 downto 0) := (others => '0');
    signal s_yd, s_ud, s_vd : t_u10(9 to 11) := (others => to_unsigned(64, 10));

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
        variable v_i : signed(15 downto 0);

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
            s_fo <= resize(unsigned(registers_in(0)(9 downto 5)), 6) + 1;                -- K1 Folds 1 .. 32
            s_hw <= resize(unsigned(registers_in(1)(9 downto 6)), 5) + 1;                -- K2 Shine 1 .. 16
            s_hue <= unsigned(registers_in(2)(9 downto 7));                              -- K3 Hue (8)
            s_orbk <= unsigned(registers_in(3)(9 downto 2));                             -- K4 Orb
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;                  -- K5 Drift
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(5 downto 0); end if;
            s_cg <= to_unsigned(32, 9) + resize(unsigned(registers_in(5)(9 downto 2)), 9);   -- K6 Contrast /128

            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            -- per field
            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_shine <= registers_in(6)(0);
                s_orb   <= registers_in(6)(1);
                s_duo   <= registers_in(6)(2);
                s_mir   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_wa <= s_g(11 downto 4);

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
            s_ax <= s_w(11 downto 1);
            -- orb centre at vw/4: dx (Q.1) starts at -vw/2; second differences
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_vw, 16), 2));
                               s_inc2 <= to_unsigned(32, 6);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_vw, 16), 1) + resize(s_vw, 16));
                               s_inc2 <= to_unsigned(18, 6);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_vw, 16), 1));
                               s_inc2 <= to_unsigned(8, 6);
            end case;
            s_inc0 <= v_i(14 downto 0);

            -- serial unit (12 shift-add steps per job)
            if s_zc = 4 then
                s_ma <= resize(s_vw(11 downto 1), 24); s_mb <= resize(s_vw(11 downto 1), 12); s_mjob <= "000";
                s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 24 then
                s_ma <= to_unsigned(5, 24); s_mb <= s_ys; s_mjob <= "010";
                s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 44 or s_fall2 = '1' then
                s_ma <= resize(s_ady_n, 24); s_mb <= resize(s_ady_n, 12); s_mjob <= "001";
                s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_cx2sq <= s_macc;
                    when "010"  => s_oyi <= s_macc(17 downto 8);
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
        variable v_dy : signed(12 downto 0);

    begin
        if rising_edge(clk) then
            -- oscillator: x ramp per source pixel, y ramp per line, phase per field
            if data_in.avid = '1' then s_ox <= s_ox + s_fo; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + s_oyi; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + unsigned(resize(s_spd, 8)); end if;
            -- the fold field as the oscillator source (luma only)
            s_wob <= s_wa * f_tri8(s_oy(9 downto 2));
            s_oty <= to_unsigned(64, 10)
                     + shift_left(resize(f_tri8(s_ox(11 downto 4) + s_wob(14 downto 7) + s_ot), 10), 2)
                     + shift_left(resize(f_tri8(s_ox(11 downto 6) + s_oy(9 downto 3) - s_ot), 10), 1);
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
                s_ady_n <= to_unsigned(540, 11);

            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                null;

            elsif s_fall2 = '1' then
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - 540;
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11);
                else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;

            end if;
            s_rsd_n <= s_wm1(10 downto 0);

            if data_in.avid = '1' and s_avq = '0' then
                s_rsd <= s_rsd_n;
                s_dy2 <= s_dy2_n;

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
                s_q <= s_cx2sq;
                s_inc <= s_inc0;

            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
                if s_ru = s_wm1(10 downto 0) then s_ru <= (others => '0'); else s_ru <= s_ru + 1; end if;
                if s_rd = 0 then s_rd <= s_wm1(10 downto 0); else s_rd <= s_rd - 1; end if;
                s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                s_inc <= s_inc + signed(resize(s_inc2, 15));

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
        variable v_o : unsigned(10 downto 0);
        variable v_a : signed(9 downto 0);
        variable v_l : signed(12 downto 0);
        variable v_s9 : unsigned(8 downto 0);
        variable v_hi : unsigned(4 downto 0);

    begin
        if rising_edge(clk) then
            -- 1: mirrored read address; dx^2 copy
            if s_mir = '1' and s_bcol >= s_ax then s_lb_ra <= s_rd; else s_lb_ra <= s_ru; end if;
            s_q1 <= s_q;
            -- 2: dx^2 integer
            s_qi2 <= s_q1(23 downto 2);
            -- 3: field (fabric copy); r^2
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));
            s_r2_3 <= resize(s_qi2, 23) + resize(s_dy2, 23);
            -- 4: orb falloff index; field 8-bit
            v_o := s_r2_3(22 downto 12);
            if v_o > 255 then s_od4 <= x"FF"; else s_od4 <= v_o(7 downto 0); end if;
            s_y4 <= s_y3(9 downto 2);
            -- 5: orb added to the field
            if s_orb = '1' and s_orbk > s_od4 then
                v_s9 := resize(s_y4, 9) + resize(s_orbk - s_od4, 9);
            else
                v_s9 := resize(s_y4, 9);
            end if;
            s_l5 <= f_sat8(v_s9);
            -- 6: contrast product about mid
            s_p6 <= (signed(resize(s_l5, 9)) - 128) * signed(resize(s_cg, 9));
            -- 7: level; highlight; hue
            v_l := resize(shift_right(s_p6, 7), 13) + 128;
            if v_l < 0 then s_l7 <= x"00"; elsif v_l > 255 then s_l7 <= x"FF"; else s_l7 <= unsigned(v_l(7 downto 0)); end if;
            if s_shine = '1' and unsigned(v_l(4 downto 0)) < s_hw and v_l >= 0 and v_l <= 255 then s_hl7 <= '1'; else s_hl7 <= '0'; end if;
            v_hi := s_hue & "00";
            if s_duo = '1' and v_l(5) = '1' then v_hi := v_hi + 12; end if;
            s_cu7 <= to_signed(C_COS(to_integer(v_hi)), 8);
            s_cv7 <= to_signed(C_COS(to_integer(v_hi + 24)), 8);
            -- 8: luma; chroma products scaled by the level
            if s_hl7 = '1' then
                s_y8 <= to_unsigned(940, 10);
            else
                s_y8 <= to_unsigned(64, 10) + shift_left(resize(s_l7, 10), 1) + resize(s_l7, 10);
            end if;
            s_pu8 <= s_cu7 * signed(resize(s_l7, 9));
            s_pv8 <= s_cv7 * signed(resize(s_l7, 9));
            -- 9: compose
            s_yd(9) <= s_y8;
            s_ud(9) <= f_clamp(to_signed(512, 13) + resize(shift_right(s_pu8, 6), 13));
            s_vd(9) <= f_clamp(to_signed(512, 13) + resize(shift_right(s_pv8, 6), 13));
            -- 10..12
            for i in 10 to 11 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
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

        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture chartreuse;
